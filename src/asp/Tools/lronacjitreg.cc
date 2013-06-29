// __BEGIN_LICENSE__
//  Copyright (c) 2009-2013, United States Government as represented by the
//  Administrator of the National Aeronautics and Space Administration. All
//  rights reserved.
//
//  The NGT platform is licensed under the Apache License, Version 2.0 (the
//  "License"); you may not use this file except in compliance with the
//  License. You may obtain a copy of the License at
//  http://www.apache.org/licenses/LICENSE-2.0
//
//  Unless required by applicable law or agreed to in writing, software
//  distributed under the License is distributed on an "AS IS" BASIS,
//  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
//  See the License for the specific language governing permissions and
//  limitations under the License.
// __END_LICENSE__


/// \file lrojitreg.cc
///

#include <asp/Tools/stereo.h>
#include <vw/InterestPoint.h>
#include <boost/accumulators/accumulators.hpp>
#include <boost/accumulators/statistics.hpp>
#include <vw/FileIO/DiskImageResource.h>
#include <vw/Stereo/PreFilter.h>
#include <vw/Stereo/CorrelationView.h>
#include <vw/Stereo/CostFunctions.h>
#include <vw/Stereo/DisparityMap.h>

#include <vw/Stereo/Correlate.h>

#include <asp/Core/DemDisparity.h>
#include <asp/Core/LocalHomography.h>

using namespace vw;
using namespace vw::stereo;
using namespace asp;


/* LROJITREG utility

- Use the correlate tool to determine the mean X and Y offset
  between the two cubes (which should be almost parallel with 
  some overlap)
- Output those two values!
*/


// TODO: Move this class to a library file!
/// Class to compute a running standard deviation
/// - Code from http://www.johndcook.com/standard_deviation.html
class RunningStandardDeviation
{
public:
  RunningStandardDeviation() : m_n(0) {}

  void Clear()
  {
    m_n = 0;
  }

  void Push(double x)
  {
    m_n++;

    // See Knuth TAOCP vol 2, 3rd edition, page 232
    if (m_n == 1)
    {
      m_oldM = m_newM = x;
      m_oldS = 0.0;
    }
    else
    {
      m_newM = m_oldM + (x - m_oldM)/m_n;
      m_newS = m_oldS + (x - m_oldM)*(x - m_newM);

      // set up for next iteration
      m_oldM = m_newM; 
      m_oldS = m_newS;
    }
  }

  int NumDataValues() const
  {
    return m_n;
  }

  double Mean() const
  {
    return (m_n > 0) ? m_newM : 0.0;
  }

  double Variance() const
  {
    return ( (m_n > 1) ? m_newS/(m_n - 1) : 0.0 );
  }

  double StandardDeviation() const
  {
    return sqrt( Variance() );
  }

private:
  int m_n;
  double m_oldM, m_newM, m_oldS, m_newS;

};


struct Parameters : asp::BaseOptions 
{
  // Input paths
  std::string leftFilePath;
  std::string rightFilePath;
  std::string rowLogFilePath;

  // Settings
  float log;
  int   h_corr_min, h_corr_max;
  int   v_corr_min, v_corr_max;
  int   xkernel, ykernel;
  int   lrthresh;
  int   correlator_type;
  int   cropWidth;  
  bool  usePyramid;
};



bool handle_arguments(int argc, char* argv[],
                     Parameters &opt) 
{ 
  po::options_description general_options("Options");
  general_options.add_options()
    ("help,h", "Display this help message")
    //("left",            po::value(&opt.leftFilePath  ), "Explicitly specify the \"left\" input file")
    //("right",           po::value(&opt.rightFilePath ), "Explicitly specify the \"right\" input file")
    ("rowLog",          po::value(&opt.rowLogFilePath)->default_value(""), "Explicitly specify the per row output text file")
    ("log",             po::value(&opt.log)->default_value(1.4), "Apply LOG filter with the given sigma, or 0 to disable")
    ("cropWidth",       po::value(&opt.cropWidth )->default_value(300), "Crop images to this width before disparity search")    
    ("h-corr-min",      po::value(&opt.h_corr_min)->default_value(-30), "Minimum horizontal disparity")
    ("h-corr-max",      po::value(&opt.h_corr_max)->default_value( 30), "Maximum horizontal disparity")
    ("v-corr-min",      po::value(&opt.v_corr_min)->default_value(-5), "Minimum vertical disparity")
    ("v-corr-max",      po::value(&opt.v_corr_max)->default_value(5), "Maximum vertical disparity")
    ("xkernel",         po::value(&opt.xkernel   )->default_value(15), "Horizontal correlation kernel size")
    ("ykernel",         po::value(&opt.ykernel   )->default_value(15), "Vertical correlation kernel size")
    ("lrthresh",        po::value(&opt.lrthresh  )->default_value(2), "Left/right correspondence threshold")
    ("correlator-type", po::value(&opt.correlator_type)->default_value(0), "0 - Abs difference; 1 - Sq Difference; 2 - NormXCorr")
    ("affine-subpix", "Enable affine adaptive sub-pixel correlation (slower, but more accurate)")
    ("pyramid", "Use the pyramid based correlator");
  general_options.add( asp::BaseOptionsDescription(opt) );
    
  po::options_description positional("");
  positional.add_options()
    ("left",  po::value(&opt.leftFilePath))
    ("right", po::value(&opt.rightFilePath));  
    
  po::positional_options_description positional_desc;
  positional_desc.add("left",  1);
  positional_desc.add("right", 1);
  

  std::string usage("[options] <left> <right>");
  po::variables_map vm =
    asp::check_command_line( argc, argv, opt, general_options, general_options,
                             positional, positional_desc, usage );

  opt.usePyramid = vm.count("pyramid");

  if ( !vm.count("left") || !vm.count("right") )
    vw_throw( ArgumentErr() << "Requires <left> and <right> input in order to proceed.\n\n"
              << usage << general_options );

  //TODO: Should output file be mandatory?
  //if ( opt.output_file.empty() )
  //  opt.output_file = fs::path( opt.image_file ).replace_extension( "_rpcmapped.tif" ).string();
  return true;
}

bool determineShifts(Parameters & params, 
                     double &dX, double &dY)
{
  
  // TODO: Verify that the images have already been initialized with the proper ISIS calls?
  //       --> lronac2isis, spiceinit, noproj, etc.


  // Verify images are present
  boost::filesystem::path leftBoostPath (params.leftFilePath );
  boost::filesystem::path rightBoostPath(params.rightFilePath);
  
  if (!boost::filesystem::exists(boost::filesystem::path(params.leftFilePath)))
  {
    printf("Error: input file %s is missing!\n", params.leftFilePath.c_str());
    return false;
  }
  if (!boost::filesystem::exists(boost::filesystem::path(params.rightFilePath)))
  {
    printf("Error: input file %s is missing!\n", params.rightFilePath.c_str());
    return false;
  }
  
  
  // Load both images  
  printf("Loading images left=%s and right=%s...\n",
         params.leftFilePath.c_str(),
         params.rightFilePath.c_str());
  DiskImageView<PixelGray<float> > left_disk_image (params.leftFilePath );
  DiskImageView<PixelGray<float> > right_disk_image(params.rightFilePath);
  
  
  // Pad out both images to be the same size
  int cols = std::min(left_disk_image.cols(), right_disk_image.cols());
  int rows = std::min(left_disk_image.rows(), right_disk_image.rows());

  ImageViewRef<PixelGray<float> > leftExt  = edge_extend(left_disk_image, 0,0,cols,rows);
  ImageViewRef<PixelGray<float> > rightExt = edge_extend(right_disk_image,0,0,cols,rows);
  
  printf("Input image size = %d by %d\n", cols, rows);
  
  
  
  // Restrict processing to the border of the images
  // - Since both images were nproj'd the overlap areas should be in about the same spots.
  const int imageMidPointX  = cols / 2;
  const int leftCropStartX  = imageMidPointX - (params.cropWidth/2); 
  const int rightCropStartX = imageMidPointX - (params.cropWidth/2); 
  const int imageHeight     = rows;
  const int imageTopRow     = 0;
//  vw::CropView<ImageViewRef<PixelGray<float> > > left (leftExt,
//                                                       leftCropStartX,   imageTopRow, 
//                                                       params.cropWidth, imageHeight);
//  vw::CropView<ImageViewRef<PixelGray<float> > > right(rightExt,
//                                                       rightCropStartX,  imageTopRow, 
//                                                       params.cropWidth, imageHeight);

  ImageViewRef<PixelGray<float> > left  = vw::CropView<DiskImageView<PixelGray<float> > > (left_disk_image,
                                                       leftCropStartX,   imageTopRow, 
                                                       params.cropWidth, imageHeight);
  ImageViewRef<PixelGray<float> > right = vw::CropView<DiskImageView<PixelGray<float> > > (right_disk_image,
                                                       rightCropStartX,  imageTopRow, 
                                                       params.cropWidth, imageHeight);

  printf("Disparity search image size = %d by %d\n", params.cropWidth, rows);

//  // Dump the cropped image to disk
//  ImageView<PixelRGB<uint8> > rgbLeft  = left;
//  ImageView<PixelRGB<uint8> > rgbRight = right;
//  vw::write_image("/root/data/cropdumpLeft.tif",  rgbLeft);
//  vw::write_image("/root/data/cropdumpRight.tif", rgbRight);  


  // Use correlation function to compute image disparity

  stereo::CostFunctionType corr_type = ABSOLUTE_DIFFERENCE;
  if (params.correlator_type == 1)
    corr_type = SQUARED_DIFFERENCE;
  else if (params.correlator_type == 2)
    corr_type = CROSS_CORRELATION;

 
  printf("Running stereo correlation...\n");
  printf("Search bounds:\n");
  printf("h_corr_min = %d\n", params.h_corr_min);  
  printf("h_corr_max = %d\n", params.h_corr_max);  
  printf("v_corr_min = %d\n", params.v_corr_min);  
  printf("v_corr_max = %d\n", params.v_corr_max);        
  
  
//  ImageViewRef<PixelMask<Vector2i> > disparity_map;
  ImageView<PixelMask<Vector2i> > disparity_map;
  int    corr_timeout   = 0;
  double seconds_per_op = 0.0;
  if (params.usePyramid) 
  {
    printf("Using pyramid search.\n");
    disparity_map =
      stereo::pyramid_correlate( left, right,
                                 constant_view( uint8(255), left ),
                                 constant_view( uint8(255), right ),
                                 stereo::LaplacianOfGaussian(params.log),
                                 BBox2i(Vector2i(params.h_corr_min, params.v_corr_min),
                                        Vector2i(params.h_corr_max, params.v_corr_max)),
                                 Vector2i(params.xkernel, params.ykernel),
                                 corr_type, corr_timeout, seconds_per_op,
                                 params.lrthresh, 5 );
  } 
  else 
  {
    printf("Using non-pyramid search.\n");  
    disparity_map =
      stereo::correlate( left, right,
                         stereo::LaplacianOfGaussian(params.log),
                         BBox2i(Vector2i(params.h_corr_min, params.v_corr_min),
                                Vector2i(params.h_corr_max, params.v_corr_max)),
                         Vector2i(params.xkernel, params.ykernel),
                         corr_type, params.lrthresh);
  }  
  
  
  // Compute the mean horizontal and vertical shifts
  // - Currently disparity_map contains the per-pixel shifts
  
  printf("Accumulating offsets...\n");  

  //TODO: Pick exact output format
  std::ofstream logFile;
  const bool writeLogFile = !params.rowLogFilePath.empty();
  if (writeLogFile)
  {
    logFile.open(params.rowLogFilePath.c_str());
    if (logFile.fail())
    {
      printf("Failed to create output log file %s!\n", params.rowLogFilePath.c_str());
      return false;
    }
    else
      printf("Created output log file %s\n", params.rowLogFilePath.c_str());
  }
  
  const int X_INDEX = 0;
  const int Y_INDEX = 1;  
  
  double meanVertOffset      = 0.0;
  double meanHorizOffset     = 0.0;
  int    numValidRows        = 0;
  int    totalNumValidPixels = 0;
  
  RunningStandardDeviation stdCalcX, stdCalcY;
  
  std::vector<double> rowOffsets(disparity_map.rows());
  std::vector<double> colOffsets(disparity_map.rows());  
  for (int row=0; row<disparity_map.rows(); ++row)
  {
  
    // Accumulate the shifts for this row
    double rowSum        = 0.0;
    double colSum        = 0.0;
    int    numValidInRow = 0;
    for (int col=0; col<disparity_map.cols(); ++col)
    {
    
      if (is_valid(disparity_map(col,row)))
      {
//        printf("Location %d, %d has values x=%d, y=%d\n", 
//                row, col, disparity_map(col,row)[X_INDEX], 
//                disparity_map(col,row)[Y_INDEX]);
        rowSum += disparity_map(col,row)[Y_INDEX];
        colSum += disparity_map(col,row)[X_INDEX];
        ++numValidInRow;
                
      }
    }  
    //printf("%d valid in row %d\n", numValidInRow, row);
    
    // Compute mean shift for this row
    if (numValidInRow == 0)
    {
      rowOffsets[row] = 0;
      colOffsets[row] = 0;
    }
    else 
    {
      rowOffsets[row] = rowSum / static_cast<double>(numValidInRow);
      colOffsets[row] = colSum / static_cast<double>(numValidInRow);
      totalNumValidPixels += numValidInRow;
      ++numValidRows;
      
//      printf("%d valid in row %d\n", numValidInRow, row);    
//      printf("Row offset = %lf, col offset = %lf\n\n", 
//             rowOffsets[row], colOffsets[row]);      

      stdCalcX.Push(colOffsets[row]);
      stdCalcY.Push(rowOffsets[row]);
    }
    
    
    if (writeLogFile)
    {
      logFile << row << ", " << rowOffsets[row]
                     << ", " << colOffsets[row] << std::endl;    
    }
    

    //TODO: Should we accumulate this per-pixel instead of per-row?      
    // Accumulate global shift
    meanVertOffset  += rowOffsets[row];
    meanHorizOffset += colOffsets[row];
  }
  
  if (writeLogFile)
  {
    logFile << std::endl << "Average Sample Offset: " << stdCalcX.Mean() << " StdDev: " << stdCalcX.StandardDeviation()
            << std::endl << "Average Line Offset: "   << stdCalcY.Mean() << " StdDev: " << stdCalcX.StandardDeviation() << std::endl;
    logFile.close();                   
  }  
  
  if (numValidRows == 0)
  {
    printf("Error: No valid pixel matches found!\n");
    return false;
  }
  
  // Compute overall mean shift
  meanVertOffset  = meanVertOffset  / static_cast<double>(numValidRows);
  meanHorizOffset = meanHorizOffset / static_cast<double>(numValidRows);
  
  dX = meanHorizOffset;
  dY = meanVertOffset;
  
  printf("%d valid pixels in %d rows\n", totalNumValidPixels, numValidRows);


  return true;
}

int main(int argc, char* argv[]) 
{
  try 
  {
    // Parse the input parameters
    Parameters params;
    if (!handle_arguments(argc, argv, params))
    {
      printf("Failed to parse input parameters!\n");
      return false;
    }
    
    double dX, dY;
    if (determineShifts(params, dX, dY))
    {
      // Success, print the results
      printf("Mean sample offset = %lf\n", dX);
      printf("Mean line   offset = %lf\n", dY);
    }

  } ASP_STANDARD_CATCHES;

  return 0;
}






