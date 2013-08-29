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
#include <vw/Math.h>
#include <asp/Core/Common.h>
#include <asp/Core/Macros.h>
#include <asp/Tools/point2dem.h>
#include <pointmatcher/PointMatcher.h>
#include <limits>
#include <cstring>
#include <boost/filesystem.hpp>
#include <boost/algorithm/string.hpp>
namespace fs = boost::filesystem;
namespace po = boost::program_options;

using namespace vw;
using namespace std;
using namespace vw::cartography;

typedef double RealT; // We will use doubles in libpointmatcher.
typedef PointMatcher<RealT> PM;
typedef PM::DataPoints DP;
using namespace PointMatcherSupport;

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
  string reference, source, init_transform_file, alignment_method, config_file, datum;
  PointMatcher<RealT>::Matrix init_transform;
  int num_iter, max_num_reference_points, max_num_source_points;
  double diff_translation_err, diff_rotation_err, max_disp, outlier_ratio;
  bool compute_translation_only, save_trans_source, save_trans_ref, verbose;
  // Output
  string output_prefix;
  Options():max_disp(0.0){}
};

void handle_arguments( int argc, char *argv[], Options& opt ) {
  po::options_description general_options("");
  general_options.add_options()
    ("initial-transform",
    po::value(&opt.init_transform_file)->default_value(""), "The file containing the rotation + translation transform to be used as an initial guess. It can come from a previous run of the tool.")
    ("num-iterations", po::value(&opt.num_iter)->default_value(400), "Maximum number of iterations.")
    ("diff-rotation-error", po::value(&opt.diff_rotation_err)->default_value(1e-4), "Change in rotation amount below which the algorithm will stop.")
    ("diff-translation-error", po::value(&opt.diff_translation_err)->default_value(1e-3), "Change in translation amount below which the algorithm will stop.")
    ("max-displacement",
     po::value(&opt.max_disp), "Maximum expected displacement of source points as result of alignment, in meters (after the initial guess transform is applied to the source points). Used for removing gross outliers in the source point cloud.") // no default on purpose, set to 0 in constructor
    ("outlier-ratio", po::value(&opt.outlier_ratio)->default_value(0.75), "Fraction of source (movable) points considered inliers (after gross outliers further than max-displacement from reference points are removed).")
    ("max-num-reference-points", po::value(&opt.max_num_reference_points)->default_value(100000000), "Maximum number of (randomly picked) reference points to use.")
    ("max-num-source-points", po::value(&opt.max_num_source_points)->default_value(100000), "Maximum number of (randomly picked) source points to use (after discarding gross outliers).")
    ("alignment-method", po::value(&opt.alignment_method)->default_value("point-to-plane"), "The type of iterative closest point method to use. [point-to-plane, point-to-point]")
    ("datum", po::value(&opt.datum)->default_value(""), "Use this datum for CSV files instead of auto-detecting it. [WGS_1984, D_MOON, D_MARS, etc.]")
    ("config-file", po::value(&opt.config_file)->default_value(""),
    "This is an advanced option. Read the alignment parameters from a configuration file, in the format expected by libpointmatcher, over-riding the command-line options.")
    ("output-prefix,o", po::value(&opt.output_prefix)->default_value("run/run"), "Specify the output prefix.")
    ("compute-translation-only", po::bool_switch(&opt.compute_translation_only)->default_value(false)->implicit_value(true),
     "Compute the transform from source to reference point cloud as a translation only (no rotation).")
    ("save-transformed-source-points", po::bool_switch(&opt.save_trans_source)->default_value(false)->implicit_value(true),
     "Apply the obtained transform to the source points so they match the reference points and save them.")
    ("save-inv-transformed-reference-points", po::bool_switch(&opt.save_trans_ref)->default_value(false)->implicit_value(true),
     "Apply the inverse of the obtained transform to the reference points so they match the source points and save them.")
    ("verbose", po::bool_switch(&opt.verbose)->default_value(false)->implicit_value(true),
     "Print debug information");

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

  string usage("[options] <reference cloud> <source cloud>");
  po::variables_map vm =
    asp::check_command_line( argc, argv, opt, general_options, general_options,
                             positional, positional_desc, usage );

  if ( opt.reference.empty() || opt.source.empty() )
    vw_throw( ArgumentErr() << "Missing input files.\n"
              << usage << general_options );

  if ( opt.output_prefix.empty() )
    vw_throw( ArgumentErr() << "Missing output prefix.\n"
              << usage << general_options );

  asp::create_out_dir(opt.output_prefix);

  // Read the initial transform
  int dim = 3;
  opt.init_transform = PointMatcher<RealT>::Matrix::Identity(dim + 1, dim + 1);
  if (opt.init_transform_file != ""){
    validateFile(opt.init_transform_file);
    PointMatcher<RealT>::Matrix T;
    ifstream is(opt.init_transform_file.c_str());
    for (int row = 0; row < dim + 1; row++){
      for (int col = 0; col < dim + 1; col++){
        double a;
        if (! (is >> a) )
          vw_throw( vw::IOErr() << "Failed to read initial transform from: "
                    << opt.init_transform_file << "\n" );
        opt.init_transform(row, col) = a;
      }
    }
    cout.precision(16);
    cout << "Initial guess transform:\n" << opt.init_transform << endl;
  }

}

vw::Vector3 cartesian_to_geodetic_adj(vw::cartography::GeoReference const& georef,
                                      vw::Vector3 xyz
                                      ){

  // cartesian_to_geodetic() returns longitude between -180 and 180.
  // Sometimes this is 360 degrees less than what desired,
  // so here we do an adjustment.
  // To do: This may not be perfectly fool-proof.
  vw::Vector3 G = georef.datum().cartesian_to_geodetic(xyz);
  Vector2 ll = georef.pixel_to_lonlat(Vector2(0, 0));
  G(0) += 360*round((ll[0] - G(0))/360.0);
  return G;
}

double dem_height_diff(Vector3 const& xyz, GeoReference const& dem_georef,
                       ImageView<float> const& dem, double nodata){

  Vector3 llh = cartesian_to_geodetic_adj(dem_georef, xyz);
  Vector2 pix = dem_georef.lonlat_to_pixel(subvector(llh, 0, 2));
  int c = (int)round(pix[0]), r = (int)round(pix[1]);
  if (c < 0 || c >= dem.cols() || r < 0 || r >= dem.rows() || dem(c, r) == nodata){
    //vw_out() << "nval: " << llh[0] << ' ' << llh[1] << endl;
    return numeric_limits<double>::quiet_NaN();
  }
  //vw_out() << "yval: " << llh[0] << ' ' << llh[1] << endl;
  return std::abs(llh[2] - dem(c, r));
}

void null_check(const char* token, string const& line){
  if (token == NULL)
    vw_throw( vw::IOErr() << "Failed to read line: " << line << "\n" );
}

template<typename T>
typename PointMatcher<T>::DataPoints::Labels form_labels(int dim){

  typedef typename PointMatcher<T>::DataPoints::Label Label;
  typedef typename PointMatcher<T>::DataPoints::Labels Labels;

  Labels labels;
  for (int i=0; i < dim; i++){
    string text;
    text += char('x' + i);
    labels.push_back(Label(text, 1));
  }
  labels.push_back(Label("pad", 1));

  return labels;
}

template<typename T>
typename PointMatcher<T>::DataPoints
form_data_points(int dim, Vector3 const& shift, vector<Vector3> const& points){

  int numPoints = points.size();

  typename PointMatcher<T>::Matrix features(dim+1, numPoints);
  for(int col = 0; col < numPoints; col++){
    for (int row = 0; row < dim; row++)
      features(row, col) = points[col][row] - shift[row];
    features(dim, col) = 1;
  }

  return typename PointMatcher<T>::DataPoints(features, form_labels<T>(dim));
}

void pick_at_most_m_unique_elems_from_n_elems(int m, int n, vector<int>& elems){

  // Out of the elements 0, 1,..., n - 1, pick m unique
  // random elements and sort them in increasing order.

  elems.clear();

  if (m < 1 || n < 1) return;
  if (m > n) m = n;

  vector<int> all(n);
  for (int i = 0; i < n; i++) all[i] = i;

  // Swap a randomly selected element from index 0 to j with the one
  // at index j. Then decrement j. Done after m elements are
  // processed.
  for (int j = n-1; j >= n-m; j--){
    int r = rand()%(j+1); // 0 <= r <= j
    swap(all[r], all[j]);
  }

  elems.resize(m);
  for (int i = 0; i < m; i++) elems[i] = all[n-m+i];
  sort(elems.begin(), elems.end());

}

template<typename T>
typename PointMatcher<T>::DataPoints
random_pc_subsample(int m, typename PointMatcher<T>::DataPoints const& in_points){

  // Return at most m random points out of the input point cloud.

  int n = in_points.features.cols();
  vector<int> elems;
  pick_at_most_m_unique_elems_from_n_elems(m, n, elems);
  m = elems.size();

  int dim = 3;
  typename PointMatcher<T>::Matrix features(dim+1, m);
  for(int col = 0; col < m; col++){
    for (int row = 0; row < dim; row++)
      features(row, col) = in_points.features(row, elems[col]);
    features(dim, col) = 1;
  }

  return typename PointMatcher<T>::DataPoints(features, form_labels<T>(dim));
}

template<typename T>
typename PointMatcher<T>::DataPoints load_csv(string const& file_name,
                                              int num_points_to_load,
                                              string const& input_datum_str,
                                              bool verbose,
                                              bool calc_shift,
                                              Vector3 & shift,
                                              string & actual_datum_str,
                                              bool & is_lola_rdr_format,
                                              double & mean_longitude
                                              ){

  is_lola_rdr_format = false;

  validateFile(file_name);

  const int bufSize = 1024;
  char temp[bufSize];
  ifstream file( file_name.c_str() );
  if( !file ) {
    vw_throw( vw::IOErr() << "Unable to open file \"" << file_name << "\"" );
  }

  // Find how many lines are in the file
  int num_points = 0;
  string line;
  while ( getline(file, line, '\n') ){
    num_points++;
  }
  file.clear(); file.seekg(0, ios_base::beg); // go back to start of file

  // We will randomly pick or not a point with probability load_ratio
  double load_ratio = (double)num_points_to_load/std::max(1.0, (double)num_points);

  int dim = 3;
  vector<Vector3> points;
  Vector3 mean_center;
  bool is_first_line = true;
  char sep[] = ", \t";

  // Peek at first line and see how many elements it has
  getline(file, line);
  file.clear(); file.seekg(0, ios_base::beg); // go back to start of file
  strncpy(temp, line.c_str(), bufSize);
  const char* token = strtok (temp, sep);
  int numTokens = 0;
  while (token != NULL){
    numTokens++;
    token = strtok (NULL, sep);
  }
  if (numTokens < 3){
    vw_throw( vw::IOErr() << "Expecting at least three fields on each "
              << "line of file: " << file_name << "\n" );
  }

  cartography::Datum datum;
  if (numTokens > 3){
    is_lola_rdr_format = true;
    if (verbose)
      vw_out() << "Guessing file " << file_name
               << " to be in LOLA RDR PointPerRow format.\n";
    datum.set_well_known_datum("D_MOON");
    if (input_datum_str != "" && input_datum_str != "D_MOON")
      vw_throw( vw::IOErr() << "The datum for LOLA RDR PointPerRow format "
                << "is expected to be D_MOON. Got instead: '"
                << input_datum_str << "'\n" );

  }else{
    is_lola_rdr_format = false;
    if (verbose)
      vw_out() << "Guessing file " << file_name
               << " to be in latitude,longitude,height above datum (meters) "
               << "format.\n";
    datum.set_well_known_datum("WGS_1984");
  }

  if (input_datum_str != "") datum.set_well_known_datum(input_datum_str);
  actual_datum_str = datum.name();
  if (verbose) vw_out() << "Using the datum: " << actual_datum_str << endl;

  mean_longitude = 0.0;

  while ( getline(file, line, '\n') ){

    int year, month, day, hour, min;
    double lon, lat, rad, height, sec, is_invalid;

    double r = (double)std::rand()/(double)RAND_MAX;
    if (r > load_ratio) continue;

    // We went with C-style file reading instead of C++ in this instance
    // because we found it to be significantly faster on large files.

    Vector3 xyz;
    if (!is_lola_rdr_format){

      // lat,lon,height format

      strncpy(temp, line.c_str(), bufSize);
      const char* token = strtok(temp, sep); null_check(token, line);
      int ret = sscanf(token, "%lg", &lat);

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
      xyz = datum.geodetic_to_cartesian( llh );
      if ( xyz == Vector3() || !(xyz == xyz) ) continue; // invalid and NaN check

    }else{

      // Load a RDR_*PointPerRow_csv_table.csv file used for LOLA. Code
      // copied from Ara Nefian's lidar2dem tool.
      // We will ignore lines which do not start with year (or a value that
      // cannot be converted into an integer greater than zero, specifically).

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
      xyz = datum.geodetic_to_cartesian( lonlatrad );
      if ( xyz == Vector3() || !(xyz == xyz) ) continue; // invalid and NaN check

      // Adjust the point so that it is at the right distance from
      // planet center.
      xyz = rad*(xyz/norm_2(xyz));
    }

    points.push_back(xyz);
    mean_center += xyz;
    mean_longitude += lon;
  }

  mean_center    /= points.size();
  mean_longitude /= points.size();

  if (calc_shift) shift = mean_center;

  if (verbose) vw_out() << "Loaded points: " << points.size() << endl;
  return form_data_points<T>(dim, shift, points);
}

// Load a DEM
template<typename T>
typename PointMatcher<T>::DataPoints load_dem(string const& file_name,
                                              int num_points_to_load,
                                              bool calc_shift,
                                              Vector3 & shift
                                              ){
  validateFile(file_name);

  cartography::GeoReference dem_georef;
  bool is_good = cartography::read_georeference( dem_georef, file_name );
  if (!is_good) vw_throw(ArgumentErr() << "DEM: " << file_name
                         << " does not have a georeference.\n");

  DiskImageView<float> dem(file_name);
  double nodata = numeric_limits<double>::quiet_NaN();
  boost::shared_ptr<DiskImageResource> dem_rsrc
    ( new DiskImageResourceGDAL(file_name) );
  if (dem_rsrc->has_nodata_read()) nodata = dem_rsrc->nodata_read();

  // We will randomly pick or not a point with probability load_ratio
  int num_points = dem.cols()*dem.rows();
  double load_ratio = (double)num_points_to_load/std::max(1.0, (double)num_points);

  vector<Vector3> points;
  int dim = 3;
  Vector3 mean_center;
  int count = 0;
  for (int j = 0; j < dem.rows(); j++ ) {
    int local_count = 0;
    Vector3 local_mean;

    for ( int i = 0; i < dem.cols(); i++ ) {

      double r = (double)std::rand()/(double)RAND_MAX;
      if (r > load_ratio) continue;

      if (dem(i, j) == nodata) continue;
      Vector2 lonlat = dem_georef.pixel_to_lonlat( Vector2(i,j) );
      Vector3 llh( lonlat.x(), lonlat.y(), dem(i,j) );
      Vector3 xyz = dem_georef.datum().geodetic_to_cartesian( llh );
      if ( xyz == Vector3() || !(xyz == xyz) ) continue; // invalid and NaN check
      points.push_back(xyz);
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
  //vw_out() << "mean is " << mean_center << endl;

  if (calc_shift) shift = mean_center;

  vw_out() << "Loaded points: " << points.size() << endl;
  return form_data_points<T>(dim, shift, points);
}

// Load a point cloud
template<typename T>
typename PointMatcher<T>::DataPoints load_pc(string const& file_name,
                                             int num_points_to_load,
                                             bool calc_shift,
                                             Vector3 & shift
                                             ){

  validateFile(file_name);

  vector<Vector3> points;
  const int dim = 3;

  // To do: Is it faster to to do for_each?
  ImageViewRef<Vector3> point_cloud = read_n_channels<dim>(file_name);

  // We will randomly pick or not a point with probability load_ratio
  int num_points = point_cloud.cols()*point_cloud.rows();
  double load_ratio = (double)num_points_to_load/std::max(1.0, (double)num_points);

  Vector3 mean_center;
  int count = 0;
  for (int j = 0; j < point_cloud.rows(); j++ ) {
    int local_count = 0;
    Vector3 local_mean;
    for ( int i = 0; i < point_cloud.cols(); i++ ) {

      double r = (double)std::rand()/(double)RAND_MAX;
      if (r > load_ratio) continue;

      Vector3 xyz = point_cloud(i, j);
      if ( xyz == Vector3() || !(xyz == xyz) ) continue; // invalid and NaN check
      points.push_back(xyz);
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
  //vw_out() << "mean is " << mean_center << endl;

  if (calc_shift) shift = mean_center;

  vw_out() << "Loaded points: " << points.size() << endl;
  return form_data_points<T>(dim, shift, points);
}

string get_file_type(string const& file_name){

  boost::filesystem::path path(file_name);
  string ext = boost::filesystem::extension(path);
  boost::algorithm::to_lower(ext);
  if (boost::iequals(ext, ".tif")){
    int nc = get_num_channels(file_name);
    if (nc == 1)
      return "DEM";
    if (nc >= 3)
      return "PC";
    vw_throw(ArgumentErr() << "File: " << file_name
             << " is neither a point cloud or DEM.\n");
  }else if (boost::iequals(ext, ".csv") || boost::iequals(ext, ".txt")){
    return "CSV";
  }
  vw_throw( ArgumentErr() << "Unknown file type: " << file_name << "\n" );
}

// Load file from disk and convert to libpointmatcher's format
template<typename T>
typename PointMatcher<T>::DataPoints load_file(Options const& opt,
                                               string const& file_name,
                                               int num_points_to_load,
                                               bool calc_shift,
                                               Vector3 & shift
                                               ){

  vw_out() << "Reading: " << file_name << endl;

  string file_type = get_file_type(file_name);
  if (file_type == "DEM")
    return load_dem<T>(file_name, num_points_to_load, calc_shift, shift);
  if (file_type == "PC")
    return load_pc<T>(file_name, num_points_to_load, calc_shift, shift);
  if (file_type == "CSV"){
    string actual_datum_str;
    bool verbose = true;
    bool is_lola_rdr_format;
    double mean_longitude;
    return load_csv<T>(file_name, num_points_to_load, opt.datum, verbose,
                       calc_shift, shift, actual_datum_str, is_lola_rdr_format,
                       mean_longitude
                       );
  }

  vw_throw( ArgumentErr() << "Unknown file type: " << file_name << "\n" );
}

double calc_mean(vector<double> const& errs, int len){
  double mean = 0;
  for (int i = 0; i < len; i++){
    mean += errs[i];
  }
  if (len == 0) return 0;
  return mean/len;
}

PointMatcher<RealT>::Matrix apply_shift(PointMatcher<RealT>::Matrix const& T,
                                        Vector3 const& shift){

  // Consider a 4x4 matrix T which implements a rotation + translation
  // y = A*x + b. Consider a point s in space close to the points
  // x. We want to make that the new origin, so the points x get
  // closer to origin. In the coordinates (x2 = x - s, y2 = y - s) the
  // transform becomes y2 + s = A*(x2 + s) + b, or
  // y2 = A*x2 + b + A*s - s. Encode the obtained transform into another
  // 4x4 matrix T2.

  VW_ASSERT(T.cols() == 4 && T.rows() == 4,
            ArgumentErr() << "Expected square matrix of size 4.");

  Eigen::MatrixXd A = T.block(0, 0, 3, 3);
  Eigen::MatrixXd b = T.block(0, 3, 3, 1);

  Eigen::MatrixXd s = b;
  for (int i = 0; i < 3; i++) s(i, 0) = shift[i];

  Eigen::MatrixXd b2 = b + A*s - s;
  PointMatcher<RealT>::Matrix T2 = T;
  T2.block(0, 3, 3, 1) = b2;

  return T2;
}

void calc_stats(string label, PointMatcher<RealT>::Matrix const& dists){

  vector<double> errs(dists.cols()*dists.rows());
  int count = 0;
  for (int col = 0; col < dists.cols(); col++){
    for (int row = 0; row < dists.rows(); row++){
      errs[count] = dists(row, col);
      count++;
    }
  }
  sort(errs.begin(), errs.end());

  int len = errs.size();
  double a25 = calc_mean(errs, len/4),   a50  = calc_mean(errs, len/2);
  double a75 = calc_mean(errs, 3*len/4), a100 = calc_mean(errs, len);

  vw_out() << "Number of errors: " << len << endl;
  vw_out() << label << ": mean of smallest errors:"
           << " 25%: " << a25 << " 50%: " << a50
           << " 75%: " << a75 << " 100%: " << a100 << endl;
}

void dump_llh(DP const & data, Vector3 const& shift){

  // We assume here the planet is Earth
  cartography::Datum datum;
  datum.set_well_known_datum("WGS_1984");

  vw_out() << "size of data: " << data.features.rows() << ' '
           << data.features.cols() << endl;

  for (int c = 0; c < data.features.cols(); c++){
    Vector3 xyz;
    for (int r = 0; r < data.features.rows() - 1; r++)
      xyz[r] = data.features(r, c) + shift[r];
    Vector3 llh = datum.cartesian_to_geodetic(xyz);
    vw_out() << "llh " << llh[0] << ' ' << llh[1] << ' ' << llh[2] << endl;
  }

}

void save_errors(string errFile, PointMatcher<RealT>::Matrix const& errors){
  vw_out() << "Writing: " << errFile << endl;
  ofstream ef(errFile.c_str());
  for (int row = 0; row < (int)errors.rows(); row++){
    for (int col = 0; col < (int)errors.cols(); col++){
      ef << errors(row, col) << endl;
    }
  }
}

void save_transforms(Options const& opt,
                     PointMatcher<RealT>::Matrix const& T){

  // Save the transform and its inverse.

  string transFile = opt.output_prefix + "-transform.txt";
  cout.precision(16);
  cout << "Alignment transform:" << endl << T << endl;
  vw_out() << "Writing: " << transFile  << endl;
  ofstream tf(transFile.c_str());
  tf.precision(16);
  tf << T << endl;
  tf.close();

  string iTransFile = opt.output_prefix + "-inverse-transform.txt";
  PointMatcher<RealT>::Matrix invT = T.inverse();
  vw_out() << "Writing: " << iTransFile  << endl;
  ofstream itf(iTransFile.c_str());
  itf.precision(16);
  itf << invT << endl;
  itf.close();

}

// Apply a transform to each point in the given point cloud
struct TransformPC : public ReturnFixedType<Vector3> {
  PointMatcher<RealT>::Matrix m_T;
  TransformPC(PointMatcher<RealT>::Matrix const& T):m_T(T){}

  Vector3 operator() (Vector3 const& P) const {
    if (P == Vector3()) return Vector3();
    Eigen::VectorXd V(4);
    V[0] = P[0]; V[1] = P[1]; V[2] = P[2]; V[3] = 1;
    V = m_T*V;
    Vector3 Q;
    Q[0] = V[0]; Q[1] = V[1]; Q[2] = V[2];
    return Q;
  }
};
template <class ImageT>
UnaryPerPixelView<ImageT, TransformPC>
inline transform_pc( ImageViewBase<ImageT> const& image,
                     PointMatcher<RealT>::Matrix const& T) {
  return UnaryPerPixelView<ImageT, TransformPC>( image.impl(), TransformPC(T) );
}

void calc_max_displacment(DP const& source, DP const& trans_source){

  double max_obtained_disp = 0.0;
  int numPts = source.features.cols();
  const int dim = 3;
  for(int col = 0; col < numPts; col++){
    Vector3 s, t;
    for (int row = 0; row < dim; row++){
      s[row] = source.features(row, col);
      t[row] = trans_source.features(row, col);
    }
    max_obtained_disp = max(max_obtained_disp, norm_2(s - t));
  }
  vw_out() << "Maximum displacement of source points: "
           << max_obtained_disp << " m" << endl;
}

void save_trans_point_cloud(Options const& opt,
                            string input_file,
                            string output_prefix,
                            PointMatcher<RealT>::Matrix const& T
                            ){

  // Apply a given transform to the point cloud in input file,
  // and save it.

  string file_type = get_file_type(input_file);

  string output_file;
  if (file_type == "CSV")
    output_file = output_prefix + ".csv";
  else
    output_file = output_prefix + ".tif";
  vw_out() << "Writing: " << output_file << endl;

  if (file_type == "DEM"){
    cartography::GeoReference dem_georef;
    bool is_good = cartography::read_georeference( dem_georef, input_file );
    if (!is_good) vw_throw(ArgumentErr() << "DEM: " << input_file
                           << " does not have a georeference.\n");

    DiskImageView<float> dem(input_file);
    double nodata = numeric_limits<double>::quiet_NaN();
    boost::shared_ptr<DiskImageResource> dem_rsrc
      ( new DiskImageResourceGDAL(input_file) );
    if (dem_rsrc->has_nodata_read()) nodata = dem_rsrc->nodata_read();

    ImageViewRef<Vector3> point_cloud =
      geodetic_to_cartesian( dem_to_geodetic( apply_mask(dem, nodata),
                                              dem_georef ),
                             dem_georef.datum() );
    asp::block_write_gdal_image(output_file, transform_pc(point_cloud, T),
                                opt,
                                TerminalProgressCallback("asp", "\t--> "));
  }else if (file_type == "PC"){
    const int dim = 3;
    ImageViewRef<Vector3> point_cloud = read_n_channels<dim>(input_file);
    asp::block_write_gdal_image(output_file, transform_pc(point_cloud, T),
                                opt,
                                TerminalProgressCallback("asp", "\t--> "));
  }else if (file_type == "CSV"){

    bool verbose = false;
    bool calc_shift = true;
    Vector3 shift;
    string actual_datum_str;
    bool is_lola_rdr_format;
    double mean_longitude;
    DP point_cloud = load_csv<RealT>(input_file, numeric_limits<int>::max(),
                                     opt.datum, verbose, calc_shift, shift,
                                     actual_datum_str, is_lola_rdr_format,
                                     mean_longitude
                                     );

    cartography::Datum datum;
    datum.set_well_known_datum(actual_datum_str);

    ofstream outfile( output_file.c_str() );
    outfile.precision(16);

    if (is_lola_rdr_format)
      outfile << "# longitude,latitude,radius (km)" << endl;
    else
      outfile << "# latitude,longitude,height above datum (meters)" << endl;

    int numPts = point_cloud.features.cols();
    int dim = 3;
    for(int col = 0; col < numPts; col++){

      Eigen::VectorXd V(dim + 1);
      for (int row = 0; row < dim; row++)
        V[row] = point_cloud.features(row, col) + shift[row];
      V[dim] = 1;

      // Apply the transform
      V = T*V;

      Vector3 P;
      for (int row = 0; row < dim; row++) P[row] = V[row];
      Vector3 llh = datum.cartesian_to_geodetic(P); // lon-lat-height
      llh[0] += 360.0*round((mean_longitude - llh[0])/360.0); // 360 deg adjustment

      if (is_lola_rdr_format)
        outfile << llh[0] << ',' << llh[1] << ',' << norm_2(P)/1000.0 << endl;
      else
        outfile << llh[1] << ',' << llh[0] << ',' << llh[2] << endl;
    }
    outfile.close();

  }else{
    vw_throw( ArgumentErr() << "Unknown file type: " << input_file << "\n" );
  }

}

void debug_save_point_cloud(DP const& point_cloud, Vector3 const& shift,
                            string const& output_file){

  int numPts = point_cloud.features.cols();
  int dim = 3;

  cartography::Datum datum;
  datum.set_well_known_datum("WGS_1984");

  vw_out() << "Writing: " << output_file << endl;
  ofstream outfile( output_file.c_str() );
  outfile.precision(16);

  for(int col = 0; col < numPts; col++){

    Vector3 P;
    for (int row = 0; row < dim; row++)
      P[row] = point_cloud.features(row, col) + shift[row];

    Vector3 llh = datum.cartesian_to_geodetic(P); // lon-lat-height
    outfile << llh[1] << ',' << llh[0] << ',' << llh[2] << endl;
  }
}

int main( int argc, char *argv[] ) {

  // Mandatory line for Eigen
  Eigen::initParallel();

  Options opt;
  try {
    handle_arguments( argc, argv, opt );

    // Set the number of threads for OpenMP
    if ( opt.num_threads != 0 ) {
      vw::vw_out() << "\t--> Setting number of processing threads to: "
                   << opt.num_threads << endl;
      omp_set_num_threads(opt.num_threads);
    }

    // Create the default ICP algorithm
    PM::ICP icp;
    if (opt.config_file == ""){
      // Read the options from the command line
      icp.setParams(opt.num_iter, opt.outlier_ratio, opt.diff_rotation_err,
                    opt.diff_translation_err, opt.alignment_method);

    }else{
      vw_out() << "Will read the options from: " << opt.config_file << endl;
      ifstream ifs(opt.config_file.c_str());
      if (!ifs.good())
        vw_throw( ArgumentErr() << "Cannot open configuration file: "
                  << opt.config_file << "\n" );
      icp.loadFromYaml(ifs);
    }

    // Load the point clouds. We will shift both point clouds by the
    // centroid of the first one to bring them closer to origin.

    // Load the subsampled reference point cloud.
    Vector3 shift;
    bool calc_shift = true;
    Stopwatch sw1;
    sw1.start();
    DP ref  = load_file<RealT>(opt, opt.reference, opt.max_num_reference_points,
                               calc_shift, shift);
    sw1.stop();
    if (opt.verbose) vw_out() << "Loading the reference point cloud took "
                              << sw1.elapsed_seconds() << " [s]" << endl;
    //ref.save(outputBaseFile + "_ref.vtk");

    // Load the subsampled source point cloud. If the user wants
    // to filter gross outliers in the source points based on
    // max_disp, load a lot more points than asked, filter based on
    // max_disp, then resample to the number desired by the user.
    int num_source_pts = opt.max_num_source_points;
    if (opt.max_disp > 0.0) num_source_pts = max(num_source_pts, 50000000);
    calc_shift = false;
    Stopwatch sw2;
    sw2.start();
    DP source = load_file<RealT>(opt, opt.source, num_source_pts,
                                 calc_shift, shift);
    sw2.stop();
    if (opt.verbose) vw_out() << "Loading the source point cloud took "
                              << sw2.elapsed_seconds() << " [s]" << endl;

    // The point clouds are shifted, so shift the initial transform as well.
    PointMatcher<RealT>::Matrix initT = apply_shift(opt.init_transform, shift);

    // Apply the initial guess transform to the source point cloud.
    icp.transformations.apply(source, initT);

    // Create a separate ICP object for filtering gross outliers and
    // statistics computations.  Need this as the ICP object used to
    // compute the transform internally modifies the points before
    // placing them in the tree.
    PM::ICP icp2;
    PointMatcher<RealT>::Matrix beg_errors;
    Stopwatch sw3;
    sw3.start();
    icp2.initICP(ref);
    sw3.stop();
    if (opt.verbose) vw_out() << "Point cloud initialization took "
                              << sw3.elapsed_seconds() << " [s]" << endl;

    if (opt.max_disp > 0.0){
      // Filter gross outliers
      Stopwatch sw4;
      sw4.start();
      icp2.filterGrossOutliersAndCalcErrors(ref, opt.max_disp*opt.max_disp,
                                            source, beg_errors); //in-out
      sw4.stop();
      if (opt.verbose) vw_out() << "Filter gross outliers took "
                                << sw4.elapsed_seconds() << " [s]" << endl;
      source = random_pc_subsample<RealT>(opt.max_num_source_points, source);
      vw_out() << "Reducing number of source points to " << source.features.cols() << endl;
    }

    // Calculate the errors before doing ICP
    Stopwatch sw5;
    sw5.start();
    double big = 1e+300;
    icp2.filterGrossOutliersAndCalcErrors(ref, big,
                                          source, beg_errors); //in-out
    sw5.stop();
    if (opt.verbose) vw_out() << "Initial error computation took "
                              << sw5.elapsed_seconds() << " [s]" << endl;


    calc_stats("Input", beg_errors);
    //dump_llh(source, shift);

    // Compute the transformation to align the source to reference.
    int dim = 3;
    PointMatcher<RealT>::Matrix Id
      = PointMatcher<RealT>::Matrix::Identity(dim + 1, dim + 1);
    Stopwatch sw6;
    sw6.start();
    PointMatcher<RealT>::Matrix T = icp(source, ref, Id,
                                        opt.compute_translation_only);
    vw_out() << "Match ratio: " << icp.errorMinimizer->getWeightedPointUsedRatio()
             << endl;
    sw6.stop();
    if (opt.verbose) vw_out() << "ICP took "
                              << sw6.elapsed_seconds() << " [s]" << endl;

    // Transform the source to make it close to reference.
    DP trans_source(source);
    icp.transformations.apply(trans_source, T);

    // Calculate by how much points move as result of T
    calc_max_displacment(source, trans_source);

    // Calculate the errors after doing ICP
    PointMatcher<RealT>::Matrix end_errors;
    Stopwatch sw7;
    sw7.start();
    icp2.filterGrossOutliersAndCalcErrors(ref, big,
                                          trans_source, end_errors); // in-out
    sw7.stop();
    if (opt.verbose) vw_out() << "Final error computation took "
                              << sw7.elapsed_seconds() << " [s]" << endl;

    calc_stats("Output", end_errors);

    // We must apply to T the initial guess transform
    PointMatcher<RealT>::Matrix combinedT = T*initT;

    // Go back to the original coordinate system, undoing the shift
    PointMatcher<RealT>::Matrix globalT = apply_shift(combinedT, -shift);

    save_transforms(opt, globalT);

    if (opt.save_trans_ref){
       string trans_ref_prefix = opt.output_prefix + "-trans_reference";
       save_trans_point_cloud(opt, opt.reference, trans_ref_prefix,
                              globalT.inverse());
    }

    if (opt.save_trans_source){
       string trans_source_prefix = opt.output_prefix + "-trans_source";
       save_trans_point_cloud(opt, opt.source, trans_source_prefix, globalT);
    }

    save_errors(opt.output_prefix + "-beg_errors.txt", beg_errors);
    save_errors(opt.output_prefix + "-end_errors.txt", end_errors);

  } ASP_STANDARD_CATCHES;

  return 0;
}
