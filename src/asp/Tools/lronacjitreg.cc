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
#include <vw/FileIO/DiskImageView.h>
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
using std::endl;
using std::setprecision;
using std::setw;

/* LROJITREG utility

- Use the correlate tool to determine the mean X and Y offset
  between the two cubes (which should be almost parallel with 
  some overlap)
- Output those two values!
*/


// TODO: Move this class to a library file!
/// Class to compute a running standard deviation
/// - Code from http://www.johndcook.com/standard_deviation.html
class RunningStatistics
{
public:
  RunningStatistics() : m_n(0) {}

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
  Vector2i kernel;
  int   lrthresh;
  int   correlator_type;
  int   cropWidth;  
};



bool handle_arguments(int argc, char* argv[],
                     Parameters &opt) 
{ 
  po::options_description general_options("Options");
  general_options.add_options()
    ("output-log",          po::value(&opt.rowLogFilePath)->default_value(""), "Explicitly specify the per row output text file")
    ("log",             po::value(&opt.log)->default_value(1.4), "Apply LOG filter with the given sigma, or 0 to disable")
    ("crop-width",       po::value(&opt.cropWidth )->default_value(200), "Crop images to this width before disparity search")    
    ("h-corr-min",      po::value(&opt.h_corr_min)->default_value( 0), "Minimum horizontal disparity - computed automatically if not set.")
    ("h-corr-max",      po::value(&opt.h_corr_max)->default_value(-1), "Maximum horizontal disparity - computed automatically if not set.")
    ("v-corr-min",      po::value(&opt.v_corr_min)->default_value( 0), "Minimum vertical disparity - computed automatically if not set.")
    ("v-corr-max",      po::value(&opt.v_corr_max)->default_value(-1), "Maximum vertical disparity - computed automatically if not set.")
    ("kernel",          po::value(&opt.kernel    )->default_value(Vector2i(15,15)), "Correlation kernel size")
    ("lrthresh",        po::value(&opt.lrthresh  )->default_value(2), "Left/right correspondence threshold")
    ("correlator-type", po::value(&opt.correlator_type)->default_value(0), "0 - Abs difference; 1 - Sq Difference; 2 - NormXCorr")
    ("affine-subpix", "Enable affine adaptive sub-pixel correlation (slower, but more accurate)");
  
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

  if ( !vm.count("left") || !vm.count("right") )
    vw_throw( ArgumentErr() << "Requires <left> and <right> input in order to proceed.\n\n"
              << usage << general_options );

  return true;
}

bool determineShifts(Parameters & params, 
                     double &dX, double &dY)
{
  
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
  
  
  const int imageWidth  = std::min(left_disk_image.cols(), right_disk_image.cols());
  const int imageHeight = std::min(left_disk_image.rows(), right_disk_image.rows());
  const int imageTopRow = 0;
  const int imageMidPointX  = imageWidth / 2;
  const int cropStartX  = imageMidPointX - (params.cropWidth/2);

  // Restrict processing to the border of the images
  // - Since both images were nproj'd the overlap areas should be in about the same spots.
  const BBox2i crop_roi( cropStartX, imageTopRow,
			 params.cropWidth, imageHeight );


  const int SEARCH_RANGE_EXPANSION = 10;
  int ipFindXOffset = 0;
  int ipFindYOffset = 0;

  // If the search box was not fully populated use ipfind to estimate a search region
  if ( (params.h_corr_min > params.h_corr_max) || (params.v_corr_min > params.v_corr_max) )
  {

    // Now use interest point finding/matching functions to estimate the search offset between the images

    // Gather interest points
    asp::IntegralAutoGainDetector detector( 500 );
    ip::InterestPointList ip1 = ip::detect_interest_points( crop(left_disk_image,crop_roi ), detector );
    ip::InterestPointList ip2 = ip::detect_interest_points( crop(right_disk_image,crop_roi), detector );

    ip::SGradDescriptorGenerator descriptor;
    describe_interest_points( crop(left_disk_image,crop_roi ), descriptor, ip1 );
    describe_interest_points( crop(right_disk_image,crop_roi), descriptor, ip2 );

    // Match interest points
    ip::DefaultMatcher matcher(0.5);
    std::vector<ip::InterestPoint> matched_ip1, matched_ip2;
    matcher(ip1, ip2, matched_ip1, matched_ip2 );
    ip::remove_duplicates( matched_ip1, matched_ip2 );


    // Filter interest point matches
    math::RandomSampleConsensus<math::SimilarityFittingFunctor, math::InterestPointErrorMetric> ransac( math::SimilarityFittingFunctor(),
												      math::InterestPointErrorMetric(),
												      100, 5, 100, true );
    std::vector<Vector3> ransac_ip1 = ip::iplist_to_vectorlist(matched_ip1);
    std::vector<Vector3> ransac_ip2 = ip::iplist_to_vectorlist(matched_ip2);

    Matrix<double> H(ransac(ransac_ip1, ransac_ip2));
    std::cout << "ipfind based similarity: " << H << std::endl;

    // Use the estimated transform between the images to determine a search offset range
    ipFindXOffset = -1*static_cast<int>(H[0][2]);
    ipFindYOffset = -1*static_cast<int>(H[1][2]);

  } // End ipfind case

  BBox2i searchRegion(Vector2i(ipFindXOffset-SEARCH_RANGE_EXPANSION, ipFindYOffset-SEARCH_RANGE_EXPANSION),
    		      Vector2i(ipFindXOffset+SEARCH_RANGE_EXPANSION, ipFindYOffset+SEARCH_RANGE_EXPANSION));  

  // Factor in user bounds overrides
  if (params.h_corr_min < params.h_corr_max)
  {
    searchRegion.min()[0] = params.h_corr_min;
    searchRegion.max()[0] = params.h_corr_max;
  }
  if (params.v_corr_min < params.v_corr_max)
  {
    searchRegion.min()[1] = params.v_corr_min;
    searchRegion.max()[1] = params.v_corr_max;    
  }

  printf("Disparity search image size = %d by %d\n", params.cropWidth, imageHeight);
  std::cout << "Offset search region = " << searchRegion << endl;


  // Use correlation function to compute image disparity
  stereo::CostFunctionType corr_type = ABSOLUTE_DIFFERENCE;
  if (params.correlator_type == 1)
    corr_type = SQUARED_DIFFERENCE;
  else if (params.correlator_type == 2)
    corr_type = CROSS_CORRELATION;
 
  printf("Running stereo correlation...\n");
  
  // Pyramid Correlation works best rasterizing in 1024^2 chunks
  vw_settings().set_default_tile_size(1024);

  int    corr_timeout   = 0;
  double seconds_per_op = 0.0;
  DiskCacheImageView<PixelMask<Vector2i> >
    disparity_map
    ( stereo::pyramid_correlate( crop( left_disk_image, crop_roi ),
				 crop( right_disk_image, crop_roi ),
				 constant_view( uint8(255), left_disk_image ),
				 constant_view( uint8(255), right_disk_image ),
				 stereo::LaplacianOfGaussian(params.log),
				 searchRegion,
				 params.kernel,
				 corr_type, corr_timeout, seconds_per_op,
				 params.lrthresh, 5 ) );

  // DEBUG!!!  
  //write_image("/home/smcmich1/data/dispMap.tif", disparity_map);




  // Compute the mean horizontal and vertical shifts
  // - Currently disparity_map contains the per-pixel shifts
  
  printf("Accumulating offsets...\n");  

  std::ofstream out;
  const bool writeLogFile = !params.rowLogFilePath.empty();
  if (writeLogFile)
  {
    out.open(params.rowLogFilePath.c_str());
    if (out.fail())
    {
      printf("Failed to create output log file %s!\n", params.rowLogFilePath.c_str());
      return false;
    }

    printf("Created output log file %s\n", params.rowLogFilePath.c_str());

    out << "#       Lronacjitreg ISIS Application Results" << endl;
    out << "#    Coordinates are (Sample, Line) unless indicated" << endl;
    out << "#\n#    ****  Image Input Information ****\n";
    out << "#  FROM:  " << params.leftFilePath << endl;
    out << "#    Lines:       " << setprecision(0) << imageHeight << endl;
    out << "#    Samples:     " << setprecision(0) << imageWidth << endl;
    out << "#    SampOffset:  " << cropStartX << endl;
    out << "#    TopLeft:     " << setw(7) << setprecision(0)
        << cropStartX << " "
        << setw(7) << setprecision(0)
        << 0 << endl;
    out << "#    LowerRight:  " << setw(7) << setprecision(0)
        << cropStartX + params.cropWidth << " "
        << setw(7) << setprecision(0)
        << imageHeight << endl;
    out << "\n";
    out << "#  MATCH: " << params.rightFilePath << endl;
    out << "#    Lines:       " << setprecision(0) << imageHeight << endl;
    out << "#    Samples:     " << setprecision(0) << imageWidth << endl;
    out << "#    SampOffset:  " << cropStartX << endl;
    out << "#    TopLeft:     " << setw(7) << setprecision(0)
        << cropStartX << " "
        << setw(7) << setprecision(0)
        << 0 << endl;
    out << "#    LowerRight:  " << setw(7) << setprecision(0)
        << cropStartX+params.cropWidth  << " "
        << setw(7) << setprecision(0)
        << imageHeight << endl;
    out << "\n";

  }
  
  double meanVertOffset      = 0.0;
  double meanHorizOffset     = 0.0;
  int    numValidRows        = 0;
  int    totalNumValidPixels = 0;
  
  RunningStatistics stdCalcX, stdCalcY;
  
//  std::vector<double> rowOffsets(disparity_map.rows());
//  std::vector<double> colOffsets(disparity_map.rows());  
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
        float dY = disparity_map(col,row)[1]; // Y
        float dX = disparity_map(col,row)[0]; // X
        rowSum += dY;
        colSum += dX;
        ++numValidInRow;
      
        stdCalcX.Push(dX);
        stdCalcY.Push(dY);
      }
    }  
/*    
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
    }
 
    
    if (writeLogFile)
    {
      out << row << ", " << rowOffsets[row]
                 << ", " << colOffsets[row] << std::endl;    
    }
*/    
  } // End loop through rows

  printf("Initial X mean = %f\n", stdCalcX.Mean());
  printf("Initial Y mean = %f\n", stdCalcY.Mean());

  float minDispX = stdCalcX.Mean() - 3.0f*stdCalcX.StandardDeviation();
  float maxDispX = stdCalcX.Mean() + 3.0f*stdCalcX.StandardDeviation();

  float minDispY = stdCalcY.Mean() - 3.0f*stdCalcY.StandardDeviation();
  float maxDispY = stdCalcY.Mean() + 3.0f*stdCalcY.StandardDeviation();

  printf("Y limits: %f to %f\n", minDispY, maxDispY);
  printf("X limits: %f to %f\n", minDispX, maxDispX);


  // Loop through the rows again, removing outliers
  RunningStatistics stdCalcFinalX, stdCalcFinalY;
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
        float dY = disparity_map(col,row)[1]; // Y
        float dX = disparity_map(col,row)[0]; // X

        // Skip this value if it is outside the valid range
        if ( (dY < minDispY) || (dY > maxDispY) || 
             (dX < minDispX) || (dX > maxDispX)   )
          break;

        rowSum += dY;
        colSum += dX;
        ++numValidInRow;
      
        stdCalcFinalX.Push(dX);
        stdCalcFinalY.Push(dY);
      }
    }  
    
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
    }
    
    
    if (writeLogFile)
    {
      out << row << ", " << rowOffsets[row]
                 << ", " << colOffsets[row] << std::endl;    
    }
    
  } // End loop through rows




  
  if (writeLogFile)
  {

    out << "\n#  **** Registration Data ****\n";
    out << "#   RegFile: " << "" << endl;
    out << "#   OverlapSize:      " << setw(7) << params.cropWidth << " "
        << setw(7) << imageHeight << "\n";
    out << "#   Sample Spacing:   " << setprecision(1) << 1 << endl;
    out << "#   Line Spacing:     " << setprecision(1) << 1 << endl;
    out << "#   Columns, Rows:    " << params.kernel[0] << " " << params.kernel[1] << endl;
    out << "#   Corr. Algorithm:  ";
    switch(params.correlator_type)
    {
      case 1:  out << "SQUARED_DIFFERENCE"  << endl; break;
      case 2:  out << "CROSS_CORRELATION"   << endl; break;
      default: out << "ABSOLUTE_DIFFERENCE" << endl; break;
    };
    if(numValidRows > 0) 
    {
      out << "#   Average Sample Offset: " << setprecision(4)
          << stdCalcFinalX.Mean()
          << "  StdDev: " << setprecision(4) << stdCalcFinalX.StandardDeviation()
          << endl;
      out << "#   Average Line Offset:   " << setprecision(4)
          << stdCalcFinalY.Mean()
          << " StdDev: " << setprecision(4) << stdCalcFinalY.StandardDeviation()
          << endl;
     }
     else  // No valid rows
     {
       out << "#   Average Sample Offset: " << "NULL\n";
       out << "#   Average Line Offset:   " << "NULL\n";
     }

    out.close();                   
  }  
  
  if (numValidRows == 0)
  {
    printf("Error: No valid pixel matches found!\n");
    return false;
  }
  
  // Compute overall mean shift
  meanVertOffset  = stdCalcFinalY.Mean();
  meanHorizOffset = stdCalcFinalX.Mean();
  
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






