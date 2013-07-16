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

// Copyright (c) 2010--2012,
// François Pomerleau and Stephane Magnenat, ASL, ETHZ, Switzerland
// You can contact the authors at <f dot pomerleau at gmail dot com> and
// <stephane at magnenat dot net>

// All rights reserved.

// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//     * Redistributions of source code must retain the above copyright
//       notice, this list of conditions and the following disclaimer.
//     * Redistributions in binary form must reproduce the above copyright
//       notice, this list of conditions and the following disclaimer in the
//       documentation and/or other materials provided with the distribution.
//     * Neither the name of the <organization> nor the
//       names of its contributors may be used to endorse or promote products
//       derived from this software without specific prior written permission.

// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
// ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
// WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL ETH-ASL BE LIABLE FOR ANY
// DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
// (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
// ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

#include <vw/FileIO.h>
#include <vw/Image.h>
#include <vw/Cartography.h>
#include <vw/InterestPoint.h>
#include <vw/Math.h>
#include <vw/Math/RANSAC.h>
#include <vw/Mosaic/ImageComposite.h>
#include <asp/Core/Common.h>
#include <asp/Core/Macros.h>
#include <pointmatcher/PointMatcher.h>
#include <limits>

#include <boost/filesystem.hpp>
namespace fs = boost::filesystem;
namespace po = boost::program_options;

using namespace vw;
using namespace std;
using namespace vw::cartography;

// Allows FileIO to correctly read/write these pixel types
namespace vw {
  template<> struct PixelFormatID<Vector3>   { static const PixelFormatEnum value = VW_PIXEL_GENERIC_3_CHANNEL; };
}

#if 0
template <int dim>
class HomogeneousTransformFunctor : public UnaryReturnSameType {
  Matrix<double,dim+1,dim+1> m_trans;

 public:
   HomogeneousTransformFunctor(Matrix<double,dim+1,dim+1> trans) : m_trans(trans) {}

  inline Vector<double,dim> operator()(Vector<double,dim> pt) const {
    if (pt == Vector<double,dim>())
      return pt;

    Vector<double,dim+1> pt_h;
    subvector(pt_h, 0, dim) = pt;
    pt_h[dim] = 1;

    Vector<double,dim+1> result = m_trans * pt_h;
    if (result[dim] != 1)
      result /= result[dim];
    return subvector(result,0,dim);
  }
};

void match_orthoimages(string const& out_prefix,
                       string const& left_image_name,
                       string const& right_image_name,
                       std::vector<InterestPoint> & matched_ip1,
                       std::vector<InterestPoint> & matched_ip2,
                       size_t const& max_points )
{

  vw_out() << "\t--> Finding Interest Points for the orthoimages\n";

  string left_ip_file, right_ip_file;
  ip::ip_filenames(out_prefix, left_image_name, right_image_name,
                   left_ip_file, right_ip_file);
  string match_file = ip::match_filename(out_prefix, left_image_name, right_image_name);

  // Building / Loading Interest point data
  if ( fs::exists(match_file) ) {

    vw_out() << "\t    * Using cached match file.\n";
    read_binary_match_file(match_file, matched_ip1, matched_ip2);
    vw_out() << "\t    * " << matched_ip1.size() << " matches\n";

  } else {
    std::vector<InterestPoint> ip1_copy, ip2_copy;

    if ( !fs::exists(left_ip_file) ||
         !fs::exists(right_ip_file) ) {

      // Worst case, no interest point operations have been performed before
      vw_out() << "\t    * Locating Interest Points\n";
      DiskImageView<PixelGray<float32> > left_disk_image(left_image_name);
      DiskImageView<PixelGray<float32> > right_disk_image(right_image_name);

      // Interest Point module detector code.
      OBALoGInterestOperator obalog_detector(0.03);
      IntegralInterestPointDetector<OBALoGInterestOperator> detector( obalog_detector, 200 );
      std::list<InterestPoint> ip1, ip2;
      vw_out() << "\t    * Processing " << left_image_name << "...\n" << std::flush;
      ip1 = detect_interest_points( left_disk_image, detector );
      vw_out() << "Located " << ip1.size() << " points.\n";
      vw_out() << "\t    * Processing " << right_image_name << "...\n" << std::flush;
      ip2 = detect_interest_points( right_disk_image, detector );
      vw_out() << "Located " << ip2.size() << " points.\n";

      vw_out() << "\t    * Generating descriptors..." << std::flush;
      SGradDescriptorGenerator descriptor;
      descriptor( left_disk_image, ip1 );
      descriptor( right_disk_image, ip2 );
      vw_out() << "done.\n";

      // Writing out the results
      vw_out() << "\t    * Caching interest points: "
               << left_ip_file << " & " << right_ip_file << std::endl;
      write_binary_ip_file( left_ip_file, ip1 );
      write_binary_ip_file( right_ip_file, ip2 );

    }

    vw_out() << "\t    * Using cached IPs.\n";
    ip1_copy = read_binary_ip_file(left_ip_file);
    ip2_copy = read_binary_ip_file(right_ip_file);

    vw_out() << "\t    * Matching interest points\n";
    ip::DefaultMatcher matcher(0.6);

    matcher(ip1_copy, ip2_copy, matched_ip1, matched_ip2,
            TerminalProgressCallback( "asp", "\t    Matching: "));
    ip::remove_duplicates(matched_ip1, matched_ip2);
    vw_out(InfoMessage) << "\t    " << matched_ip1.size() << " putative matches.\n";
    asp::cnettk::equalization( matched_ip1, matched_ip2, max_points );
    vw_out(InfoMessage) << "\t    " << matched_ip1.size() << " thinned matches.\n";

    vw_out() << "\t    * Caching matches: " << match_file << "\n";
    write_binary_match_file( match_file, matched_ip1, matched_ip2);
  }

}
struct Options : public asp::BaseOptions {
  Options() : dem1_nodata(std::numeric_limits<double>::quiet_NaN()), dem2_nodata(std::numeric_limits<double>::quiet_NaN()) {}
  // Input
  string dem1_name, dem2_name, ortho1_name, ortho2_name;
  double dem1_nodata, dem2_nodata;
  size_t max_points;

  // Output
  string output_prefix;
};

void handle_arguments( int argc, char *argv[], Options& opt ) {
  po::options_description general_options("");
  general_options.add_options()
    ("max-match-points", po::value(&opt.max_points)->default_value(800), "The max number of points that will be enforced after matching.")
    ("default-value", po::value(&opt.dem1_nodata), "The value of missing pixels in the first dem")
    ("output-prefix,o", po::value(&opt.output_prefix), "Specify the output prefix.");
  general_options.add( asp::BaseOptionsDescription(opt) );

  po::options_description positional("");
  positional.add_options()
    ("dem1", po::value(&opt.dem1_name), "Explicitly specify the first dem")
    ("dem2", po::value(&opt.dem2_name), "Explicitly specify the second dem")
    ("ortho1", po::value(&opt.ortho1_name), "Explicitly specify the first orthoimage")
    ("ortho2", po::value(&opt.ortho2_name), "Explicitly specify the second orthoimage");

  po::positional_options_description positional_desc;
  positional_desc.add("dem1", 1);
  positional_desc.add("ortho1", 1);
  positional_desc.add("dem2", 1);
  positional_desc.add("ortho2", 1);

  std::string usage("<dem1> <ortho1> <dem2> <ortho2>");
  po::variables_map vm =
    asp::check_command_line( argc, argv, opt, general_options, general_options,
                             positional, positional_desc, usage );

  if ( opt.dem1_name.empty() || opt.dem2_name.empty() ||
       opt.ortho1_name.empty() || opt.ortho2_name.empty() )
    vw_throw( ArgumentErr() << "Missing input files.\n"
              << usage << general_options );
  if ( opt.output_prefix.empty() )
    opt.output_prefix = change_extension(fs::path(opt.dem1_name), "").string();
}
#endif

typedef PointMatcher<double> PM;
typedef PM::DataPoints DP;
typedef PM::Parameters Parameters;
//typedef PointMatcherSupport::CurrentBibliography;
using namespace PointMatcherSupport;

// Load a point cloud
template<typename T>
typename PointMatcher<T>::DataPoints loadPC(const std::string& fileName, Vector3 shift)
{
  validateFile(fileName);

  typedef typename PointMatcher<T>::DataPoints::Label Label;
  typedef typename PointMatcher<T>::DataPoints::Labels Labels;
  typedef typename PointMatcher<T>::Vector Vector;
  typedef typename PointMatcher<T>::Matrix Matrix;
  typedef typename PointMatcher<T>::TransformationParameters TransformationParameters;
  typedef typename PointMatcher<T>::Matrix Parameters;
  typedef typename PointMatcher<T>::DataPoints DataPoints;

  vector<T> xData;
  vector<T> yData;
  vector<T> zData;
  vector<T> padData;
  vector<string> header;
  int dim = 3;
  Labels labels;

  for (int i=0; i < dim; i++){
    string text;
    text += char('x' + i);
    labels.push_back(Label(text, 1));
  }
  labels.push_back(Label("pad", 1));

  cartography::GeoReference dem_georef;
  cartography::read_georeference( dem_georef, fileName );
  DiskImageView<float> dem( fileName );
  double nodata = std::numeric_limits<double>::quiet_NaN();
  boost::shared_ptr<DiskImageResource> dem_rsrc( new DiskImageResourceGDAL(fileName) );
  if (dem_rsrc->has_nodata_read()){
    nodata = dem_rsrc->nodata_read();
    cout<<"nodata =" << nodata << std::endl;
  }

  Vector3 mean_center;
  int count = 0;
  for (int j = 0; j < dem.rows(); j++ ) {
    int local_count = 0;
    Vector3 local_mean;
    for ( int i = 0; i < dem.cols(); i++ ) {
      if (dem(i, j) == nodata) continue;
      Vector2 lonlat = dem_georef.pixel_to_lonlat( Vector2(i,j) );
      Vector3 lonlatrad( lonlat.x(), lonlat.y(), dem(i,j) );
      Vector3 xyz = dem_georef.datum().geodetic_to_cartesian( lonlatrad );
      if ( xyz != Vector3() && xyz == xyz ){
        xData.push_back(xyz[0]);
        yData.push_back(xyz[1]);
        zData.push_back(xyz[2]);
        padData.push_back(1);
        local_mean += xyz;
        local_count++;
      }
    }
    if ( local_count > 0 ) {
      local_mean /= double(local_count);
      double afraction = double(count) / double(count + local_count);
      double bfraction = double(local_count) / double(count + local_count);
      mean_center = afraction*mean_center + bfraction*local_mean;
      count += local_count;
    }
  }
  std::cout << "mean is " << mean_center << std::endl;

  std::cout << "Loaded points: " << count << std::endl;
  assert(xData.size() == yData.size());
  int nbPoints = xData.size();

  // Transfer loaded points in specific structure (eigen matrix)
  Matrix features(dim+1, nbPoints);
  for(int i=0; i < nbPoints; i++){
    features(0,i) = xData[i] - mean_center[0] + shift[0];
    features(1,i) = yData[i] - mean_center[1] + shift[1];
    features(2,i) = zData[i] - mean_center[2] + shift[2];
    features(3,i) = 1;
  }

  DataPoints dataPoints(features, labels);
  //cout << "Loaded " << dataPoints.features.cols() << " points." << endl;
  //cout << "Find " << dataPoints.features.rows() << " dimensions." << endl;
  //cout << features << endl;

  return dataPoints;
}

// Load a point cloud
template<typename T>
typename PointMatcher<T>::DataPoints loadFile(const std::string& fileName, Vector3 shift){

  const boost::filesystem::path path(fileName);
  const string& ext(boost::filesystem::extension(path));
  if (boost::iequals(ext, ".tif"))
    return loadPC<T>(fileName, shift);

  return DP::load(fileName);
}

// Dump command-line help
void usage(char *argv[])
{
  cerr << endl << endl;
  cerr << "* To list modules:" << endl;
  cerr << "  " << argv[0] << " -l" << endl;
  cerr << endl;
  cerr << "* To run ICP:" << endl;
  cerr << "  " << argv[0] << " [OPTIONS] reference.csv reading.csv" << endl;
  cerr << endl;
  cerr << "OPTIONS can be a combination of:" << endl;
  cerr << "--config YAML_CONFIG_FILE  Load the config from a YAML file (default: default parameters)" << endl;
  cerr << "--output FILENAME          Name of output files (default: test)" << endl;
  cerr << endl;
  cerr << "Running this program with a VTKFileInspector as Inspector will create three" << endl;
  cerr << "vtk ouptput files: ./test_ref.vtk, ./test_data_in.vtk and ./test_data_out.vtk" << endl;
  cerr << endl << "2D Example:" << endl;
  cerr << "  " << argv[0] << " examples/data/2D_twoBoxes.csv examples/data/2D_oneBox.csv" << endl;
  cerr << endl << "3D Example:" << endl;
  cerr << "  " << argv[0] << " examples/data/car_cloud400.csv examples/data/car_cloud401.csv" << endl;
  cerr << endl;

}

// Make sure that the command arguments make sense
int validateArgs(const int argc, char *argv[], bool& isCSV, string& configFile, string& outputBaseFile)
{
  if (argc == 1)
    {
      cerr << "Not enough arguments, usage:";
      usage(argv);
      return 1;
    }
  else if (argc == 2)
    {
      if (string(argv[1]) == "-l")
        {
          return -1; // we use -1 to say that we wish to quit but in a normal way
        }
      else
        {
          cerr << "Wrong option, usage:";
          usage(argv);
          return 2;
        }
    }

  const int endOpt(argc - 2);
  for (int i = 1; i < endOpt; i += 2)
    {
      const string opt(argv[i]);
      if (i + 1 > endOpt)
        {
          cerr << "Missing value for option " << opt << ", usage:"; usage(argv); exit(1);
        }
      if (opt == "--config")
        configFile = argv[i+1];
      else if (opt == "--output")
        outputBaseFile = argv[i+1];
      else
        {
          cerr << "Unknown option " << opt << ", usage:"; usage(argv); exit(1);
        }
    }
  return 0;
}


// To do: Rm unneeded dependencies on top.
// To do: Put ASP note on top.
// To do: Investigate if we can get by using floats instead of double.
// To do: Better ways of using memory are needed.
// To do: Must subtract same mean from both datasets!!! This is a bug!!!

int main( int argc, char *argv[] ) {

  std::cout << "6now 2in main!!!" << std::endl;
  string configFile;
  string outputBaseFile("test");
  bool isCSV = false;
  const int ret = validateArgs(argc, argv, isCSV, configFile, outputBaseFile);
  std::cout << "Config file is " << configFile << std::endl;

  if (ret != 0)
    return ret;
  string refFile  = argv[argc-2];
  string dataFile = argv[argc-1];

  std::cout << "xxx ---loaded files: " << refFile << ' ' << dataFile << std::endl;

  Vector3 shift1, shift2(20, -30, 100);
  std::cout << "shift is " << shift2 << std::endl;

  // Load point clouds
  const DP ref  = loadFile<double>(refFile,  shift1);
  const DP data = loadFile<double>(dataFile, shift2);

  // Create the default ICP algorithm
  PM::ICP icp;

  if (configFile.empty())
    {
      // See the implementation of setDefault() to create a custom ICP algorithm
      icp.setDefault();
    }
  else
    {
      // load YAML config
      ifstream ifs(configFile.c_str());
      std::cout << "---- Will load file: " << configFile << std::endl;
      if (!ifs.good())
        {
          cerr << "Cannot open config file " << configFile << ", usage:"; usage(argv); exit(1);
        }
      std::cout << "now before" << std::endl;
      icp.loadFromYaml(ifs);
      std::cout << "now after" << std::endl;
    }

  // Compute the transformation to express data in ref
  PM::TransformationParameters T = icp(data, ref);
  cout << "match ratio: " << icp.errorMinimizer->getWeightedPointUsedRatio() << endl;

  // Transform data to express it in ref
  DP data_out(data);
  icp.transformations.apply(data_out, T);

  // Safe files to see the results
  ref.save(outputBaseFile + "_ref.vtk");
  data.save(outputBaseFile + "_data_in.vtk");
  data_out.save(outputBaseFile + "_data_out.vtk");
  cout << "Final transformation:" << endl << T << endl;

  return 0;
#if 0

   Options opt;
  try {
    handle_arguments( argc, argv, opt );
    DiskImageResourceGDAL ortho1_rsrc(opt.ortho1_name), ortho2_rsrc(opt.ortho2_name),
      dem1_rsrc(opt.dem1_name), dem2_rsrc(opt.dem2_name);

    // Pull out dem nodatas
    if ( dem1_rsrc.has_nodata_read() ) {
      opt.dem1_nodata = dem1_rsrc.nodata_read();
      vw_out() << "\tFound DEM1 input nodata value: "
               << opt.dem1_nodata << endl;
    }
    if ( dem2_rsrc.has_nodata_read() ) {
      opt.dem2_nodata = dem2_rsrc.nodata_read();
      vw_out() << "\tFound DEM2 input nodata value: "
               << opt.dem2_nodata << endl;
    } else {
      vw_out() << "\tMissing nodata value for DEM2. Using DEM1 nodata value.\n";
      opt.dem2_nodata = opt.dem1_nodata;
    }

    typedef DiskImageView<float> dem_type;
    dem_type dem1(opt.dem1_name), dem2(opt.dem2_name);

    GeoReference ortho1_georef, ortho2_georef, dem1_georef, dem2_georef;
    read_georeference(ortho1_georef, ortho1_rsrc);
    read_georeference(ortho2_georef, ortho2_rsrc);
    read_georeference(dem1_georef, dem1_rsrc);
    read_georeference(dem2_georef, dem2_rsrc);

    ImageViewRef<Vector3> pointcloud1 =
      geodetic_to_cartesian( dem_to_geodetic( apply_mask(dem1, opt.dem1_nodata), dem1_georef ), dem1_georef.datum() );
    ImageViewRef<Vector3> pointcloud2 =
      geodetic_to_cartesian( dem_to_geodetic( apply_mask(dem2, opt.dem2_nodata), dem2_georef ), dem2_georef.datum() );

    std::vector<InterestPoint> matched_ip1, matched_ip2;

    match_orthoimages("",
                      opt.ortho1_name, opt.ortho2_name,
                      matched_ip1, matched_ip2, opt.max_points);

    vw_out() << "\t--> Rejecting outliers using RANSAC.\n";
    std::vector<Vector4> ransac_ip1, ransac_ip2;
    for (size_t i = 0; i < matched_ip1.size(); i++) {

      Vector3 xyz1 = pointcloud1( matched_ip1[i].x, matched_ip1[i].y );
      Vector3 xyz2 = pointcloud2( matched_ip2[i].x, matched_ip2[i].y );

      if ( xyz1 != Vector3() &&
           xyz2 != Vector3() ) {
        ransac_ip1.push_back(Vector4(xyz1.x(), xyz1.y(), xyz1.z(), 1));
        ransac_ip2.push_back(Vector4(xyz2.x(), xyz2.y(), xyz2.z(), 1));
      } else {
        vw_out() << "Actually dropped something.\n";
      }
    }

    std::vector<size_t> indices;
    Matrix<double> trans;
    math::RandomSampleConsensus<math::SimilarityFittingFunctorN<3>,math::L2NormErrorMetric>
      ransac( math::SimilarityFittingFunctorN<3>(), math::L2NormErrorMetric(), 2000, 200, 2*ransac_ip1.size()/3, true);
    trans = ransac(ransac_ip2, ransac_ip1);
    indices = ransac.inlier_indices(trans, ransac_ip2, ransac_ip1);

    vw_out() << "\t    * Ransac Result: " << trans << "\n";
    vw_out() << "\t                     # inliers: " << indices.size() << "\n";

    if (1) {
      for ( size_t i = 0; i < ransac_ip1.size(); i++ ) {
        std::cout << ransac_ip1[i] << " " << ransac_ip2[i] << " " << trans * ransac_ip2[i] << " "
                  << norm_2(ransac_ip1[i]-ransac_ip2[i]) << " "
                  << norm_2(ransac_ip1[i]-trans*ransac_ip2[i]) << std::endl;
      }
    }

    { // Saving transform to human readable text
      std::string filename = (fs::path(opt.dem1_name).branch_path() / (fs::basename(fs::path(opt.dem1_name)) + "__" + fs::basename(fs::path(opt.dem2_name)) + "-Matrix.txt")).string();
      std::ofstream ofile( filename.c_str() );
      ofile << std::setprecision(15) << std::flush;
      ofile << "# inliers: " << indices.size() << endl;
      ofile << trans << endl;
      ofile.close();
    }

    // ImageViewRef<Vector3> point_cloud =
    //   geodetic_to_cartesian( dem_to_geodetic( create_mask(dem1_dmg, opt.dem1_nodata), dem1_georef ), dem1_georef.datum() );

    // ImageViewRef<Vector3> point_cloud_trans =
    //   per_pixel_filter(point_cloud, HomogeneousTransformFunctor<3>(trans));

    // DiskImageResourceGDAL point_cloud_rsrc(opt.output_prefix + "-PC.tif",
    //                                        point_cloud_trans.format(),
    //                                        opt.raster_tile_size,
    //                                        opt.gdal_options);
    // block_write_image(point_cloud_rsrc, point_cloud_trans,
    //                   TerminalProgressCallback("asp", "\t--> Transforming: "));
  } ASP_STANDARD_CATCHES;

#endif
  return 0;
}
