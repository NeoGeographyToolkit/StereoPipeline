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
// Francois Pomerleau and Stephane Magnenat, ASL, ETHZ, Switzerland
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
#include <asp/Tools/point2dem.h>
#include <pointmatcher/PointMatcher.h>
#include <limits>
#include <cstring>
#include <boost/filesystem.hpp>
namespace fs = boost::filesystem;
namespace po = boost::program_options;

using namespace vw;
using namespace std;
using namespace vw::cartography;

// Allows FileIO to correctly read/write these pixel types
namespace vw {
  typedef Vector<float64,6> Vector6;
  template<> struct PixelFormatID<Vector3>   { static const PixelFormatEnum value = VW_PIXEL_GENERIC_3_CHANNEL; };
  template<> struct PixelFormatID<Vector3f>  { static const PixelFormatEnum value = VW_PIXEL_GENERIC_3_CHANNEL; };
  template<> struct PixelFormatID<Vector4>   { static const PixelFormatEnum value = VW_PIXEL_GENERIC_4_CHANNEL; };
  template<> struct PixelFormatID<Vector6>   { static const PixelFormatEnum value = VW_PIXEL_GENERIC_6_CHANNEL; };
}

struct Options : public asp::BaseOptions {
  // Input
  string reference, source, config_file, csv_format;
  // Output
  //string output_prefix;
};

void handle_arguments( int argc, char *argv[], Options& opt ) {
  po::options_description general_options("");
  general_options.add_options()
    //("output-prefix,o", po::value(&opt.output_prefix), "Specify the output prefix.")
    ("config-file,c", po::value(&opt.config_file), "Specify the configuration file.")
    ("csv-format", po::value(&opt.csv_format)->default_value("earth_lat_lon_height"), "Specify the input csv file format [earth_lat_lon_height lola_rdr_point_per_row].");
  general_options.add( asp::BaseOptionsDescription(opt) );

  po::options_description positional("");
  positional.add_options()
    ("reference", po::value(&opt.reference),
     "The reference (fixed) point cloud/DEM")
    ("source", po::value(&opt.source),
     "The source (movable) point cloud/DEM");

  po::positional_options_description positional_desc;
  positional_desc.add("reference", 1);
  positional_desc.add("source", 1);

  std::string usage("<reference cloud> <source cloud>");
  po::variables_map vm =
    asp::check_command_line( argc, argv, opt, general_options, general_options,
                             positional, positional_desc, usage );

  if ( opt.reference.empty() || opt.source.empty() )
    vw_throw( ArgumentErr() << "Missing input files.\n"
              << usage << general_options );
//   if ( opt.output_prefix.empty() )
//     opt.output_prefix = change_extension(fs::path(opt.reference), "").string();
}

typedef PointMatcher<double> PM;
typedef PM::DataPoints DP;
typedef PM::Parameters Parameters;
using namespace PointMatcherSupport;

vw::Vector3 cartesian_to_geodetic_adj(const vw::cartography::GeoReference& georef,
                                      vw::Vector3 xyz
                                      ){

  // cartesian_to_geodetic() returns longitude between -180 and 180.
  // Sometimes this is 360 degrees less than what desired,
  // so here we do an adjustment.
  // To do: This may not be perfectly fool-proof.
  vw::Vector3 G = georef.datum().cartesian_to_geodetic(xyz);
  Vector2 ll = georef.pixel_to_lonlat(Vector2(0, 0));
  G(0) += 360*round(ll[0] - G(0))/360.0;
  return G;
}

double dem_height_diff(Vector3 const& xyz, GeoReference const& dem_georef,
                       ImageView<float> const& dem, double nodata){

  Vector3 llh = cartesian_to_geodetic_adj(dem_georef, xyz);
  Vector2 pix = dem_georef.lonlat_to_pixel(subvector(llh, 0, 2));
  int c = (int)round(pix[0]), r = (int)round(pix[1]);
  if (c < 0 || c >= dem.cols() || r < 0 || r >= dem.rows() || dem(c, r) == nodata){
    //std::cout << "nval: " << llh[0] << ' ' << llh[1] << std::endl;
    return std::numeric_limits<double>::quiet_NaN();
  }
  //std::cout << "yval: " << llh[0] << ' ' << llh[1] << std::endl;
  return std::abs(llh[2] - dem(c, r));
}

void calc_errors(ImageView<Vector3> const& point_cloud,
                 GeoReference const& dem_georef,
                 ImageView<float> const& dem, double nodata,
                 ImageView<double> & errors
                 ){

  double nan = std::numeric_limits<double>::quiet_NaN();

  int count = 0;
  double mean = 0;

  errors.set_size(point_cloud.cols(), point_cloud.rows());
  for (int col = 0; col < point_cloud.cols(); col++){
    for (int row = 0; row < point_cloud.rows(); row++){

      Vector3 xyz = point_cloud(col, row);
      double e;
      if ( xyz == Vector3() || !(xyz == xyz) ) // invalid and NaN check
        e = nan;
      else
        e = dem_height_diff(xyz, dem_georef, dem, nodata);
      errors(col, row) = e;

      if (e != e) continue; // NaN
      count++;
      mean += e;
    }
  }

  mean /= count;
  std::cout << "good points and mean: " << count << ' ' << mean << std::endl;

  return;
}

void null_check(const char* token, string const& line){
  if (token == NULL)
    vw_throw( vw::IOErr() << "Failed to read line: " << line << "\n" );
}

// Load a RDR_*PointPerRow_csv_table.csv file used for LOLA. Code
// copied from Ara Nefian's lidar2dem tool.
// We will ignore lines which do not start with year (or a value that
// cannot be converted into an integer greater than zero, specifically).
template<typename T>
typename PointMatcher<T>::DataPoints loadRDR(std::string demFile,
                                             const std::string& fileName,
                                             bool calc_shift,
                                             Vector3 & shift,
                                             ImageView<Vector3> & point_cloud
                                             ){

  cartography::GeoReference dem_georef;
  cartography::read_georeference( dem_georef, demFile );
  ImageView<float> dem = copy(DiskImageView<float>(demFile));
  double nodata = std::numeric_limits<double>::quiet_NaN();
  boost::shared_ptr<DiskImageResource> dem_rsrc( new DiskImageResourceGDAL(demFile) );
  if (dem_rsrc->has_nodata_read()){
    nodata = dem_rsrc->nodata_read();
    cout<<"nodata =" << nodata << std::endl;
  }

  validateFile(fileName);
  const int bufSize = 1024;
  char temp[bufSize];
  ifstream file( fileName.c_str() );
  if( !file ) {
    vw_throw( vw::IOErr() << "Unable to open file \"" << fileName << "\"" );
  }

  typedef typename PointMatcher<T>::DataPoints::Label Label;
  typedef typename PointMatcher<T>::DataPoints::Labels Labels;
  typedef typename PointMatcher<T>::Vector Vector;
  typedef typename PointMatcher<T>::Matrix Matrix;
  typedef typename PointMatcher<T>::TransformationParameters TransformationParameters;
  typedef typename PointMatcher<T>::Matrix Parameters;
  typedef typename PointMatcher<T>::DataPoints DataPoints;

  int dim = 3;
  Labels labels;

  //cartography::Datum datum;
  //datum.set_well_known_datum("D_MOON");

  for (int i=0; i < dim; i++){
    string text;
    text += char('x' + i);
    labels.push_back(Label(text, 1));
  }
  labels.push_back(Label("pad", 1));

  vector<Vector3> points;
  Vector3 mean_center;
  string line;
  int year, month, day, hour, min;
  double lon, lat, rad, sec, is_invalid;
  bool is_first_line = true;
  char sep[] = ", \t";
  while( getline(file, line, '\n') ) {

    // We went with C-style file reading instead of C++ in this instance
    // because we found it to be significantly faster on large files.

    strncpy(temp, line.c_str(), bufSize);
    const char* token = strtok(temp, sep); null_check(token, line);

    int ret = sscanf(token, "%d-%d-%dT%d:%d:%lg", &year, &month, &day, &hour,
                     &min, &sec);
    if( year <= 0 ) { continue; }

    token = strtok(NULL, sep); null_check(token, line);
    ret += sscanf(token, "%lg", &lon);

    token = strtok(NULL, sep); null_check(token, line);
    ret += sscanf(token, "%lg", &lat);
    token = strtok(NULL, sep); null_check(token, line);
    ret += sscanf(token, "%lg", &rad);
    rad *= 1000; // km to m

    // Scan 7 more fields, until we get to the is_invalid flag.
    for (int i = 0; i < 7; i++)
      token = strtok(NULL, sep); null_check(token, line);
    ret += sscanf(token, "%lg", &is_invalid);

    // Be prepared for the fact that the first line may be the header.
    if (ret != 10){
      if (!is_first_line){
        vw_throw( vw::IOErr() << "Failed to read line: " << line << "\n" );
      }else{
        is_first_line = false;
        continue;
      }
    }
    is_first_line = false;

    if (is_invalid) continue;

    Vector3 lonlatrad( lon, lat, 0 );
    //Vector3 xyz = datum.geodetic_to_cartesian( lonlatrad );
    Vector3 xyz = dem_georef.datum().geodetic_to_cartesian( lonlatrad );
    if ( xyz == Vector3() || !(xyz == xyz) ) continue; // invalid and NaN check

    // Adjust the point so that it is at the right distance from
    // planet center. This will work even if the planet is Earth,
    // which is an ellipsoid rather than a sphere.
    xyz = rad*(xyz/norm_2(xyz));

    double herr = dem_height_diff(xyz, dem_georef, dem, nodata);
    if (herr != herr) continue; // NaN

    points.push_back(xyz);
    mean_center += xyz;
  }

  mean_center = mean_center/points.size();
  std::cout << "mean is " << mean_center << std::endl;

  if (calc_shift) shift = mean_center;
  std::cout << "calc shift is " << calc_shift << std::endl;

  point_cloud.set_size(points.size(), 1);
  for (int i = 0; i < (int)points.size(); i++) {
    Vector3 xyz = points[i];
    point_cloud(i, 0) = xyz - shift;
  }

  std::cout << "Loaded points: " << points.size() << std::endl;
  int nbPoints = points.size();

  // Transfer loaded points in specific structure (eigen matrix)
  Matrix features(dim+1, nbPoints);
  for(int i=0; i < nbPoints; i++){
    features(0,i) = points[i][0] - shift[0];
    features(1,i) = points[i][1] - shift[1];
    features(2,i) = points[i][2] - shift[2];
    features(3,i) = 1;
  }

  DataPoints dataPoints(features, labels);
  return dataPoints;
}

// Load a CSV file having lat, lon, and height values (in meters from datum).
// We assume the Earth datum.
template<typename T>
typename PointMatcher<T>::DataPoints loadLLH(std::string demFile,
                                             const std::string& fileName,
                                             bool calc_shift,
                                             Vector3 & shift,
                                             ImageView<Vector3> & point_cloud
                                             ){

  cartography::GeoReference dem_georef;
  cartography::read_georeference( dem_georef, demFile );
  ImageView<float> dem = copy(DiskImageView<float>(demFile));
  double nodata = std::numeric_limits<double>::quiet_NaN();
  boost::shared_ptr<DiskImageResource> dem_rsrc( new DiskImageResourceGDAL(demFile) );
  if (dem_rsrc->has_nodata_read()){
    nodata = dem_rsrc->nodata_read();
    cout<<"nodata =" << nodata << std::endl;
  }

  validateFile(fileName);
  const int bufSize = 1024;
  char temp[bufSize];
  ifstream file( fileName.c_str() );
  if( !file ) {
    vw_throw( vw::IOErr() << "Unable to open file \"" << fileName << "\"" );
  }

  typedef typename PointMatcher<T>::DataPoints::Label Label;
  typedef typename PointMatcher<T>::DataPoints::Labels Labels;
  typedef typename PointMatcher<T>::Vector Vector;
  typedef typename PointMatcher<T>::Matrix Matrix;
  typedef typename PointMatcher<T>::TransformationParameters TransformationParameters;
  typedef typename PointMatcher<T>::Matrix Parameters;
  typedef typename PointMatcher<T>::DataPoints DataPoints;

  int dim = 3;
  Labels labels;

  //cartography::Datum datum;
  //datum.set_well_known_datum("");

  for (int i=0; i < dim; i++){
    string text;
    text += char('x' + i);
    labels.push_back(Label(text, 1));
  }
  labels.push_back(Label("pad", 1));

  vector<Vector3> points;
  Vector3 mean_center;
  string line;
  double lon, lat, height;
  bool is_first_line = true;
  char sep[] = ", \t";
  while( getline(file, line, '\n') ) {

    // We went with C-style file reading instead of C++ in this instance
    // because we found it to be significantly faster on large files.

    strncpy(temp, line.c_str(), bufSize);
    const char* token;
    int ret = 0;
    token = strtok(temp, sep); null_check(token, line);
    ret += sscanf(token, "%lg", &lat);

    token = strtok(NULL, sep); null_check(token, line);
    ret += sscanf(token, "%lg", &lon);

    token = strtok(NULL, sep); null_check(token, line);
    ret += sscanf(token, "%lg", &height);

    // Be prepared for the fact that the first line may be the header.
    if (ret != 3){
      if (!is_first_line){
        vw_throw( vw::IOErr() << "Failed to read line: " << line << "\n" );
      }else{
        is_first_line = false;
        continue;
      }
    }
    is_first_line = false;

    Vector3 llh( lon, lat, height );
    //Vector3 xyz = datum.geodetic_to_cartesian( llh );
    Vector3 xyz = dem_georef.datum().geodetic_to_cartesian( llh );
    if ( xyz == Vector3() || !(xyz == xyz) ) continue; // invalid and NaN check

    double herr = dem_height_diff(xyz, dem_georef, dem, nodata);
    if (herr != herr) continue; // NaN

    points.push_back(xyz);
    mean_center += xyz;
  }

  mean_center = mean_center/points.size();
  std::cout << "mean is " << mean_center << std::endl;

  if (calc_shift) shift = mean_center;
  std::cout << "calc shift is " << calc_shift << std::endl;

  point_cloud.set_size(points.size(), 1);
  for (int i = 0; i < (int)points.size(); i++) {
    Vector3 xyz = points[i];
    point_cloud(i, 0) = xyz - shift;
  }

  std::cout << "Loaded points: " << points.size() << std::endl;
  int nbPoints = points.size();

  // Transfer loaded points in specific structure (eigen matrix)
  Matrix features(dim+1, nbPoints);
  for(int i=0; i < nbPoints; i++){
    features(0,i) = points[i][0] - shift[0];
    features(1,i) = points[i][1] - shift[1];
    features(2,i) = points[i][2] - shift[2];
    features(3,i) = 1;
  }

  DataPoints dataPoints(features, labels);
  return dataPoints;

}

// Load a DEM
template<typename T>
typename PointMatcher<T>::DataPoints loadDEM(const std::string& fileName,
                                             bool calc_shift,
                                             Vector3 & shift,
                                             ImageView<Vector3> & point_cloud
                                             ){
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
  ImageView<float> dem = copy(DiskImageView<float>(fileName));
  double nodata = std::numeric_limits<double>::quiet_NaN();
  boost::shared_ptr<DiskImageResource> dem_rsrc( new DiskImageResourceGDAL(fileName) );
  if (dem_rsrc->has_nodata_read()){
    nodata = dem_rsrc->nodata_read();
    cout<<"nodata =" << nodata << std::endl;
  }

  point_cloud.set_size(dem.cols(), dem.rows());
  Vector3 mean_center;
  int count = 0;
  for (int j = 0; j < dem.rows(); j++ ) {
    int local_count = 0;
    Vector3 local_mean;
    for ( int i = 0; i < dem.cols(); i++ ) {
      if (dem(i, j) == nodata) continue;
      Vector2 lonlat = dem_georef.pixel_to_lonlat( Vector2(i,j) );
      Vector3 llh( lonlat.x(), lonlat.y(), dem(i,j) );
      Vector3 xyz = dem_georef.datum().geodetic_to_cartesian( llh );
      if ( xyz == Vector3() || !(xyz == xyz) ) continue; // invalid and NaN check
      point_cloud(i, j) = xyz;
      xData.push_back(xyz[0]);
      yData.push_back(xyz[1]);
      zData.push_back(xyz[2]);
      local_mean += xyz;
      local_count++;
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

  if (calc_shift) shift = mean_center;
  std::cout << "calc shift is " << calc_shift << std::endl;
  //std::cout << "setting the shift to 0!!!" << std::endl;
  //shift = Vector3(0, 0, 0);
  for (int j = 0; j < point_cloud.rows(); j++ ) {
    for ( int i = 0; i < point_cloud.cols(); i++ ) {
      Vector3 xyz = point_cloud(i, j);
      if ( xyz == Vector3() || !(xyz == xyz) ) continue; // invalid and NaN check
      point_cloud(i, j) -= shift;
    }
  }

  std::cout << "Loaded points: " << count << std::endl;
  assert(xData.size() == yData.size());
  int nbPoints = xData.size();

  // Transfer loaded points in specific structure (eigen matrix)
  Matrix features(dim+1, nbPoints);
  for(int i=0; i < nbPoints; i++){
    features(0,i) = xData[i] - shift[0];
    features(1,i) = yData[i] - shift[1];
    features(2,i) = zData[i] - shift[2];
    features(3,i) = 1;
  }

  DataPoints dataPoints(features, labels);
  return dataPoints;
}

// Load a point cloud
template<typename T>
typename PointMatcher<T>::DataPoints loadPC(const std::string& fileName,
                                            bool calc_shift,
                                            Vector3 & shift,
                                            ImageView<Vector3> & point_cloud
                                            ){

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
  int dim = 3;
  Labels labels;

  for (int i=0; i < dim; i++){
    string text;
    text += char('x' + i);
    labels.push_back(Label(text, 1));
  }
  labels.push_back(Label("pad", 1));

  point_cloud = read_n_channels<3>(fileName);
  Vector3 mean_center;
  int count = 0;
  for (int j = 0; j < point_cloud.rows(); j++ ) {
    int local_count = 0;
    Vector3 local_mean;
    for ( int i = 0; i < point_cloud.cols(); i++ ) {
      Vector3 xyz = point_cloud(i, j);
      if ( xyz == Vector3() || !(xyz == xyz) ) continue; // invalid and NaN check
      xData.push_back(xyz[0]);
      yData.push_back(xyz[1]);
      zData.push_back(xyz[2]);
      local_mean += xyz;
      local_count++;
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

  if (calc_shift) shift = mean_center;
  std::cout << "calc shift is " << calc_shift << std::endl;
  //std::cout << "setting the shift to 0!!!" << std::endl;
  //shift = Vector3(0, 0, 0);
  for (int j = 0; j < point_cloud.rows(); j++ ) {
    for ( int i = 0; i < point_cloud.cols(); i++ ) {
      Vector3 xyz = point_cloud(i, j);
      if ( xyz == Vector3() || !(xyz == xyz) ) continue; // invalid and NaN check
      point_cloud(i, j) -= shift;
    }
  }

  std::cout << "Loaded points: " << count << std::endl;
  assert(xData.size() == yData.size());
  int nbPoints = xData.size();

  // Transfer loaded points in specific structure (eigen matrix)
  Matrix features(dim+1, nbPoints);
  for(int i=0; i < nbPoints; i++){
    features(0,i) = xData[i] - shift[0];
    features(1,i) = yData[i] - shift[1];
    features(2,i) = zData[i] - shift[2];
    features(3,i) = 1;
  }

  DataPoints dataPoints(features, labels);
  return dataPoints;
}

// Load file from disk and convert to libpointmatcher's format
template<typename T>
typename PointMatcher<T>::DataPoints loadFile(Options const& opt,
                                              const std::string& demFile, //tmp!!!
                                              const std::string& fileName,
                                              bool calc_shift,
                                              Vector3 & shift,
                                              ImageView<Vector3> & point_cloud
                                              ){

  const boost::filesystem::path path(fileName);
  const string& ext(boost::filesystem::extension(path));
  if (boost::iequals(ext, ".tif")){
    // Load tif files, PC or DEM
    int nc = get_num_channels(fileName);
    if (nc == 1)
      return loadDEM<T>(fileName, calc_shift, shift, point_cloud);
    if (nc >= 3)
      return loadPC<T>(fileName, calc_shift, shift, point_cloud);
    vw_throw(ArgumentErr() << "File: " << fileName
             << " is neither a point cloud or DEM.\n");
  }

  if (opt.csv_format == "lola_rdr_point_per_row")
    return loadRDR<T>(demFile, fileName, calc_shift, shift, point_cloud);

  return loadLLH<T>(demFile, fileName, calc_shift, shift, point_cloud);

  // Load CSV and VTK files
  //return DP::load(fileName);
}

// To do: Put all licenses in the ASP license file.
// To do: Add documentation.
// To do: Deal with output prefix.
// To do: We now assume first argument is DEM and second is CSV!!!
// To do: Put the DEM loading in just one place.
// To do: Integrate loadPC and loadDEM into one function to rm duplication.
// To do: Format for CSV files requires convertion from lonlat to xyz.
// To do: Move stuff from point2dem.h and from here to Core in pc_utils.h!
// See also point2mesh and point2las, which use this stuff.
// To do: Rm unneeded dependencies on top.
// To do: Investigate if we can get by using floats instead of double.
// To do: Better ways of using memory are needed.

int main( int argc, char *argv[] ) {

  Options opt;
  try {
    handle_arguments( argc, argv, opt );

    // Create the default ICP algorithm
    PM::ICP icp;

    if (opt.config_file.empty()){
      // See the implementation of setDefault() to create a custom ICP algorithm
      icp.setDefault();
    }else{
      // load YAML config file
      ifstream ifs(opt.config_file.c_str());
      std::cout << "---- Will load file: " << opt.config_file << std::endl;
      if (!ifs.good()) vw_throw( ArgumentErr()
                                 << "Cannot open configuration file: "
                                 << opt.config_file << "\n" );
      icp.loadFromYaml(ifs);
    }

    std::cout << "Loading files: " << opt.reference << ' '
              << opt.source << std::endl;

    // Load the point clouds. We will shift both point clouds by the
    // centroid of the first one to bring them closer to origin.
    Vector3 shift;
    bool calc_shift = true;
    ImageView<Vector3> ref_point_cloud, data_point_cloud;
    const DP ref  = loadFile<double>(opt, opt.reference, opt.reference,
                                     calc_shift, shift, ref_point_cloud);
    calc_shift = false;
    const DP data = loadFile<double>(opt, opt.reference, opt.source,
                                     calc_shift, shift, data_point_cloud);

    // Compute the transformation to express data in ref
    PM::TransformationParameters T = icp(data, ref);
    cout << "match ratio: " << icp.errorMinimizer->getWeightedPointUsedRatio()
         << endl;

    // Transform data to express it in ref
    DP data_out(data);
    icp.transformations.apply(data_out, T);

    // Safe files to see the results
    //   ref.save(outputBaseFile + "_ref.vtk");
    //   data.save(outputBaseFile + "_data_in.vtk");
    //   data_out.save(outputBaseFile + "_data_out.vtk");
    cout << "Final transformation:" << endl << T << endl;
    //   for (int row = 0; row < 4; row++){
    //     for (int col = 0; col < 4; col++){
    //       std::cout << T(row, col) << " ";
    //     }
    //     std::cout << std::endl;
    //   }

    ImageView<Vector3> trans_data_point_cloud = copy(data_point_cloud);

    // Put the shifts back and apply the transform
    for (int col = 0; col < ref_point_cloud.cols(); col++){
      for (int row = 0; row < ref_point_cloud.rows(); row++){
        Vector3 P = ref_point_cloud(col, row);
        if ( P == Vector3() || !(P == P) ) continue;
        ref_point_cloud(col, row) += shift;
      }
    }
    for (int col = 0; col < data_point_cloud.cols(); col++){
      for (int row = 0; row < data_point_cloud.rows(); row++){
        Vector3 P = data_point_cloud(col, row);
        if ( P == Vector3() || !(P == P) ) continue;
        data_point_cloud(col, row) += shift;
      }
    }
    for (int col = 0; col < trans_data_point_cloud.cols(); col++){
      for (int row = 0; row < trans_data_point_cloud.rows(); row++){
        Vector3 P = trans_data_point_cloud(col, row);
        if ( P == Vector3() || !(P == P) ) continue;
        Eigen::VectorXd V(4);
        V[0] = P[0]; V[1] = P[1]; V[2] = P[2]; V[3] = 1;
        V = T*V;
        P[0] = V[0]; P[1] = V[1]; P[2] = V[2];
        trans_data_point_cloud(col, row) = P + shift;
      }
    }

    cartography::GeoReference dem_georef;
    cartography::read_georeference( dem_georef, opt.reference );
    ImageView<float> dem = copy(DiskImageView<float>(opt.reference));
    double nodata = std::numeric_limits<double>::quiet_NaN();
    boost::shared_ptr<DiskImageResource> dem_rsrc( new DiskImageResourceGDAL(opt.reference) );
    if (dem_rsrc->has_nodata_read()){
      nodata = dem_rsrc->nodata_read();
      cout<<"nodata =" << nodata << std::endl;
    }

    ImageView<double> beg_errors, end_errors;
    calc_errors(data_point_cloud, dem_georef, dem, nodata, beg_errors);
    calc_errors(trans_data_point_cloud, dem_georef, dem, nodata, end_errors);

    std::string outRefFile = "out_ref.tif";
    std::string outDataFile = "out_data.tif";
    std::string transDataFile = "out_trans_data.tif";
    std::cout << "Writing: " << outRefFile << std::endl;
    asp::block_write_gdal_image(outRefFile, ref_point_cloud, opt,
                                TerminalProgressCallback("asp", "\t-->: "));
    std::cout << "Writing: " << outDataFile << std::endl;
    asp::block_write_gdal_image(outDataFile, data_point_cloud, opt,
                                TerminalProgressCallback("asp", "\t-->: "));
    std::cout << "Writing: " << transDataFile << std::endl;
    asp::block_write_gdal_image(transDataFile, trans_data_point_cloud, opt,
                                TerminalProgressCallback("asp", "\t-->: "));

  } ASP_STANDARD_CATCHES;

  return 0;
}
