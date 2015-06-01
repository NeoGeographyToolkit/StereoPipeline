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

// This tool uses libpointmatcher for alignment,
// https://github.com/ethz-asl/libpointmatcher
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

#include <vw/Core/Stopwatch.h>
#include <vw/FileIO/DiskImageView.h>
#include <vw/Math.h>
#include <vw/Image.h>
#include <vw/Cartography/Datum.h>
#include <vw/Cartography/GeoReference.h>
#include <vw/Cartography/PointImageManipulation.h>
#include <vw/FileIO/DiskImageUtils.h>
#include <asp/Core/Common.h>
#include <asp/Core/Macros.h>
#include <asp/Core/PointUtils.h>
#include <liblas/liblas.hpp>

#include <limits>
#include <cstring>

#include <pointmatcher/PointMatcher.h>

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

// Note: Just changing 3 to 2 below won't be enough to make the code
// work with 2D point clouds. There are some Vector3's all over the place.
const int DIM = 3;

string UNSPECIFIED_DATUM = "unspecified_datum";

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
  string reference, source, init_transform_file, alignment_method, config_file, datum, csv_format_str, csv_proj4_str;
  PointMatcher<RealT>::Matrix init_transform;
  int num_iter, max_num_reference_points, max_num_source_points;
  double diff_translation_err, diff_rotation_err, max_disp, outlier_ratio;
  double semi_major, semi_minor;
  bool compute_translation_only, save_trans_source, save_trans_ref, highest_accuracy, verbose;
  // Output
  string out_prefix;
  Options():max_disp(-1.0), verbose(true){}
};

void handle_arguments( int argc, char *argv[], Options& opt ) {
  po::options_description general_options("");
  general_options.add_options()
    ("initial-transform",
    po::value(&opt.init_transform_file)->default_value(""), "The file containing the rotation + translation transform to be used as an initial guess. It can come from a previous run of the tool.")
    ("num-iterations", po::value(&opt.num_iter)->default_value(1000), "Maximum number of iterations.")
    ("diff-rotation-error", po::value(&opt.diff_rotation_err)->default_value(1e-8), "Change in rotation amount below which the algorithm will stop (if translation error is also below bound), in degrees.")
    ("diff-translation-error", po::value(&opt.diff_translation_err)->default_value(1e-3), "Change in translation amount below which the algorithm will stop (if rotation error is also below bound), in meters.")
    ("max-displacement",
     po::value(&opt.max_disp)->default_value(0.0), "Maximum expected displacement of source points as result of alignment, in meters (after the initial guess transform is applied to the source points). Used for removing gross outliers in the source point cloud.")
    ("outlier-ratio", po::value(&opt.outlier_ratio)->default_value(0.75), "Fraction of source (movable) points considered inliers (after gross outliers further than max-displacement from reference points are removed).")
    ("max-num-reference-points", po::value(&opt.max_num_reference_points)->default_value(100000000), "Maximum number of (randomly picked) reference points to use.")
    ("max-num-source-points", po::value(&opt.max_num_source_points)->default_value(100000), "Maximum number of (randomly picked) source points to use (after discarding gross outliers).")
    ("alignment-method", po::value(&opt.alignment_method)->default_value("point-to-plane"), "The type of iterative closest point method to use. [point-to-plane, point-to-point]")
    ("highest-accuracy", po::bool_switch(&opt.highest_accuracy)->default_value(false)->implicit_value(true),
     "Compute with highest accuracy for point-to-plane (can be much slower).")
    ("csv-format", po::value(&opt.csv_format_str)->default_value(""), asp::csv_opt_caption().c_str())
    ("csv-proj4", po::value(&opt.csv_proj4_str)->default_value(""),
     "The PROJ.4 string to use to interpret the entries in input CSV files.")
    ("datum", po::value(&opt.datum)->default_value(""), "Use this datum for CSV files instead of auto-detecting it. [WGS_1984, D_MOON (radius is assumed to be 1,737,400 meters), D_MARS (radius is assumed to be 3,396,190 meters), etc.]")
    ("semi-major-axis", po::value(&opt.semi_major)->default_value(0), "Explicitly set the datum semi-major axis in meters.")
    ("semi-minor-axis", po::value(&opt.semi_minor)->default_value(0), "Explicitly set the datum semi-minor axis in meters.")
    ("config-file", po::value(&opt.config_file)->default_value(""),
     "This is an advanced option. Read the alignment parameters from a configuration file, in the format expected by libpointmatcher, over-riding the command-line options.")
    ("output-prefix,o", po::value(&opt.out_prefix)->default_value("run/run"), "Specify the output prefix.")
    ("compute-translation-only", po::bool_switch(&opt.compute_translation_only)->default_value(false)->implicit_value(true),
     "Compute the transform from source to reference point cloud as a translation only (no rotation).")
    ("save-transformed-source-points", po::bool_switch(&opt.save_trans_source)->default_value(false)->implicit_value(true),
     "Apply the obtained transform to the source points so they match the reference points and save them.")
    ("save-inv-transformed-reference-points", po::bool_switch(&opt.save_trans_ref)->default_value(false)->implicit_value(true),
     "Apply the inverse of the obtained transform to the reference points so they match the source points and save them.");
  //("verbose", po::bool_switch(&opt.verbose)->default_value(false)->implicit_value(true),
  // "Print debug information");

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

  string usage("--max-displacement arg [other options] <reference cloud> <source cloud> -o <output prefix>");
  bool allow_unregistered = false;
  std::vector<std::string> unregistered;
  po::variables_map vm =
    asp::check_command_line( argc, argv, opt, general_options, general_options,
                             positional, positional_desc, usage,
                             allow_unregistered, unregistered );

  if ( opt.reference.empty() || opt.source.empty() )
    vw_throw( ArgumentErr() << "Missing input files.\n"
              << usage << general_options );

  if ( opt.out_prefix.empty() )
    vw_throw( ArgumentErr() << "Missing output prefix.\n"
              << usage << general_options );

  if ( opt.max_disp == 0.0 )
    vw_throw( ArgumentErr() << "The max-displacement option was not set. Use -1 if it is desired not to use it.\n"
              << usage << general_options );

  if ( opt.num_iter < 0 )
    vw_throw( ArgumentErr() << "The number of iterations must be non-negative.\n"
              << usage << general_options );

  if ( (opt.semi_major != 0 && opt.semi_minor == 0)
       ||
       (opt.semi_minor != 0 && opt.semi_major == 0)
       ){

    vw_throw( ArgumentErr() << "One of the semi-major or semi-minor axes"
              << " was specified, but not the other one.\n"
              << usage << general_options );
  }

  if (opt.semi_major < 0 || opt.semi_minor < 0){
    vw_throw( ArgumentErr() << "The semi-major and semi-minor axes cannot "
              << "be negative.\n" << usage << general_options );
  }

  if (opt.datum != "" && opt.semi_major != 0 && opt.semi_minor != 0 ){
    vw_throw( ArgumentErr() << "Both the datum string and datum semi-axes were "
              << "specified. At most one needs to be set.\n"
              << usage << general_options );
  }

  // Create the output directory
  asp::create_out_dir(opt.out_prefix);

  // Turn on logging to file
  asp::log_to_file(argc, argv, "", opt.out_prefix);

  // Read the initial transform
  opt.init_transform = PointMatcher<RealT>::Matrix::Identity(DIM + 1, DIM + 1);
  if (opt.init_transform_file != ""){
    validateFile(opt.init_transform_file);
    PointMatcher<RealT>::Matrix T;
    ifstream is(opt.init_transform_file.c_str());
    for (int row = 0; row < DIM + 1; row++){
      for (int col = 0; col < DIM + 1; col++){
        double a;
        if (! (is >> a) )
          vw_throw( vw::IOErr() << "Failed to read initial transform from: "
                    << opt.init_transform_file << "\n" );
        opt.init_transform(row, col) = a;
      }
    }
    vw_out() << "Initial guess transform:\n" << opt.init_transform << endl;
  }

}

string get_file_type(string const& file_name){

  if (asp::is_csv(file_name))
    return "CSV";

  if (asp::is_las(file_name))
    return "LAS";

  boost::filesystem::path path(file_name);
  string ext = boost::filesystem::extension(path);
  boost::algorithm::to_lower(ext);
  int nc = get_num_channels(file_name);
  if (nc == 1)
    return "DEM";
  if (nc >= 3)
    return "PC";
  vw_throw(ArgumentErr() << "File: " << file_name
           << " is neither a point cloud nor a DEM.\n");
}

void read_datum(Options& opt, asp::CsvConv& csv_conv, Datum& datum){

  // Set up the datum, need it to read CSV files.

  // First, get the datum from the DEM if available.
  string dem_file = "";
  if ( get_file_type(opt.reference) == "DEM" )
    dem_file = opt.reference;
  else if ( get_file_type(opt.source) == "DEM" )
    dem_file = opt.source;
  if (dem_file != ""){
    GeoReference geo;
    bool is_good = cartography::read_georeference( geo, dem_file );
    if (!is_good) vw_throw(ArgumentErr() << "DEM: " << dem_file
                           << " does not have a georeference.\n");
    datum = geo.datum();
    vw_out() << "Detected datum from " << dem_file << ":\n" << datum << std::endl;
  }

  // Then, try to set it from the las file if available.
  string las_file = "";
  if ( get_file_type(opt.reference) == "LAS" ){
    GeoReference geo;
    if (asp::georef_from_las(opt.reference, geo)){
      las_file = opt.reference;
      datum = geo.datum();
      vw_out() << "Detected datum from " << las_file << ":\n" << datum << std::endl;
    }
  }
  if ( get_file_type(opt.source) == "LAS" ){
    GeoReference geo;
    if (asp::georef_from_las(opt.source, geo)){
      las_file = opt.source;
      datum = geo.datum();
      vw_out() << "Detected datum from " << las_file << ":\n" << datum << std::endl;
    }
  }

  // A lot of care is needed below.
  if (opt.datum != ""){
    // If the user set the datum, use it.
    datum.set_well_known_datum(opt.datum);
    vw_out() << "Will use datum (for CSV files): "
             << datum << std::endl;
  }else if (opt.semi_major > 0 && opt.semi_minor > 0){
    // Otherwise, if the user set the semi-axes, use that.
    datum = Datum("User Specified Datum", "User Specified Spheroid",
                  "Reference Meridian",
                  opt.semi_major, opt.semi_minor, 0.0);
    vw_out() << "Will use datum (for CSV files): " << datum << std::endl;
  }else if (dem_file == "" && las_file == "" &&
            (opt.csv_format_str == "" || csv_conv.format != asp::XYZ) ){
    // There is no DEM/LAS to read the datum from, and the user either
    // did not specify the CSV format (then we set it to lat, lon,
    // height), or it is specified as containing lat, lon, rather
    // than xyz.
    bool has_csv = ( get_file_type(opt.reference) == "CSV" ) ||
      ( get_file_type(opt.source) == "CSV" );
    if (has_csv){
      // We are in trouble, will not be able to convert input lat,
      // lon, to xyz.
      vw_throw( ArgumentErr() << "Cannot detect the datum. "
                << "Please specify it via --datum or "
                << "--semi-major-axis and --semi-minor-axis.\n" );
    }else{
      // The inputs are ASP point clouds. Need CSV only on output.
      vw_out() << "No datum specified. Will write output CSV files "
               << "in the x,y,z format." << std::endl;
      opt.csv_format_str = "1:x 2:y 3:z";
      parse_csv_format(opt.csv_format_str, opt.csv_proj4_str, csv_conv);
    }
  }

  return;
}

vw::Vector3 cartesian_to_geodetic_adj(vw::cartography::GeoReference const&
                                      geo, vw::Vector3 xyz){

  // cartesian_to_geodetic() returns longitude between -180 and 180.
  // Sometimes this is 360 degrees less than what desired,
  // so here we do an adjustment.
  // To do: This may not be perfectly fool-proof.
  vw::Vector3 G = geo.datum().cartesian_to_geodetic(xyz);
  Vector2 ll = geo.pixel_to_lonlat(Vector2(0, 0));
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
void random_pc_subsample(int m, typename PointMatcher<T>::DataPoints& points){

  // Return at most m random points out of the input point cloud.

  int n = points.features.cols();
  vector<int> elems;
  pick_at_most_m_unique_elems_from_n_elems(m, n, elems);
  m = elems.size();

  for (int col = 0; col < m; col++){
    for (int row = 0; row < DIM; row++)
      points.features(row, col) = points.features(row, elems[col]);
  }
  points.features.conservativeResize(Eigen::NoChange, m);
}

template<typename T>
int load_csv_aux(string const& file_name, int num_points_to_load,
                 BBox2 const& lonlat_box, bool verbose,
                 bool calc_shift, Vector3 & shift,
                 GeoReference const& geo, asp::CsvConv const& C,
                 bool & is_lola_rdr_format, double & mean_longitude,
                 typename PointMatcher<T>::DataPoints & data){

  validateFile(file_name);

  is_lola_rdr_format = false;

  int num_total_points = asp::csv_file_size(file_name);

  std::string sep_str = asp::csv_separator();
  const char* sep = sep_str.c_str();

  const int bufSize = 1024;
  char temp[bufSize];
  ifstream file( file_name.c_str() );
  if( !file ) {
    vw_throw( vw::IOErr() << "Unable to open file \"" << file_name << "\"" );
  }

  // We will randomly pick or not a point with probability load_ratio
  double load_ratio
    = (double)num_points_to_load/std::max(1.0, (double)num_total_points);

  data.features.conservativeResize(DIM+1, std::min(num_points_to_load,
                                                   num_total_points));
  data.featureLabels = form_labels<T>(DIM);

  // Peek at the first valid line and see how many elements it has
  string line;
  while ( getline(file, line, '\n') )
    if (asp::is_valid_csv_line(line)) break;

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

  if (C.csv_format_str == ""){
    if (numTokens > 20){
      is_lola_rdr_format = true;
      if (verbose)
        vw_out() << "Guessing file " << file_name
                 << " to be in LOLA RDR PointPerRow format.\n";
    }else{
      is_lola_rdr_format = false;
      if (verbose)
        vw_out() << "Guessing file " << file_name
                 << " to be in latitude,longitude,height above datum (meters) "
                 << "format.\n";
    }
  }

  if (is_lola_rdr_format &&
      geo.datum().semi_major_axis() != geo.datum().semi_minor_axis() ){
    vw_throw( ArgumentErr() << "The CSV file was detected to be in the"
              << " LOLA RDR format, yet the datum semi-axes are not equal "
              << "as expected for the Moon.\n" );
  }

  bool shift_was_calc = false;
  bool is_first_line = true;
  int points_count = 0;
  mean_longitude = 0.0;
  line = "";
  while ( getline(file, line, '\n') ){

    if (points_count >= num_points_to_load) break;

    if (!asp::is_valid_csv_line(line)) continue;

    double r = (double)std::rand()/(double)RAND_MAX;
    if (r > load_ratio) continue;

    // We went with C-style file reading instead of C++ in this instance
    // because we found it to be significantly faster on large files.

    Vector3 xyz;
    double lon = 0.0, lat = 0.0;

    if (C.csv_format_str != ""){

      // Parse custom CSV file with given format string
      bool success;
      Vector3 vals = asp::parse_csv_line(is_first_line, success, line, C);
      if (!success) continue;

      bool return_point_height = false; // will return xyz
      xyz = asp::csv_to_cartesian_or_point_height(vals, geo, C, return_point_height);

      // Decide if the point is in the box. Also save for the future
      // the longitude of the point, we'll use it to compute the mean
      // longitude.
      if (C.lon_index >= 0 && C.lon_index < (int)vals.size() &&
          C.lat_index >= 0 && C.lat_index < (int)vals.size() ){
        lon = vals[C.lon_index];
        lat = vals[C.lat_index];
      }else{
        Vector3 llh = geo.datum().cartesian_to_geodetic(xyz);
        lon = llh[0];
        lat = llh[1];
      }
      // Skip points outside the given box
      if (!lonlat_box.empty() && !lonlat_box.contains(Vector2(lon, lat)))
        continue;

    }else if (!is_lola_rdr_format){

      // lat,lon,height format
      double lat, height;

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

      // Skip points outside the given box
      if (!lonlat_box.empty() && !lonlat_box.contains(Vector2(lon, lat)))
        continue;

      Vector3 llh( lon, lat, height );
      xyz = geo.datum().geodetic_to_cartesian( llh );
      if ( xyz == Vector3() || !(xyz == xyz) ) continue; // invalid and NaN check

    }else{

      // Load a RDR_*PointPerRow_csv_table.csv file used for LOLA. Code
      // copied from Ara Nefian's lidar2dem tool.
      // We will ignore lines which do not start with year (or a value that
      // cannot be converted into an integer greater than zero, specifically).

      int year, month, day, hour, min;
      double lat, rad, sec, is_invalid;

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

      // Skip points outside the given box
      if (!lonlat_box.empty() && !lonlat_box.contains(Vector2(lon, lat)))
        continue;

      Vector3 lonlatrad( lon, lat, 0 );

      xyz = geo.datum().geodetic_to_cartesian( lonlatrad );
      if ( xyz == Vector3() || !(xyz == xyz) ) continue; // invalid and NaN check

      // Adjust the point so that it is at the right distance from
      // planet center.
      xyz = rad*(xyz/norm_2(xyz));
    }

    if (calc_shift && !shift_was_calc){
      shift = xyz;
      shift_was_calc = true;
    }

    for (int row = 0; row < DIM; row++)
      data.features(row, points_count) = xyz[row] - shift[row];
    data.features(DIM, points_count) = 1;

    points_count++;
    mean_longitude += lon;

    // Throw an error if the lon and lat are not within bounds.
    // Note that we allow some slack for lon, perhaps the point
    // cloud is say from 350 to 370 degrees.
    if (std::abs(lat) > 90.0)
      vw_throw(ArgumentErr() << "Invalid latitude value: "
               << lat << " in " << file_name << "\n");
    if (lon < -360.0 || lon > 2*360.0)
      vw_throw(ArgumentErr() << "Invalid longitude value: "
               << lon << " in " << file_name << "\n");
  }
  data.features.conservativeResize(Eigen::NoChange, points_count);

  mean_longitude /= points_count;

  return num_total_points;
}

// Load a csv file
template<typename T>
void load_csv(string const& file_name,
                 int num_points_to_load,
                 BBox2 const& lonlat_box,
                 bool verbose,
                 bool calc_shift,
                 Vector3 & shift,
                 GeoReference const& geo,
                 asp::CsvConv const& C,
                 bool & is_lola_rdr_format,
                 double & mean_longitude,
                 typename PointMatcher<T>::DataPoints & data){

  int num_total_points = load_csv_aux<T>(file_name, num_points_to_load,
                                         lonlat_box, verbose,
                                         calc_shift, shift,
                                         geo, C, is_lola_rdr_format,
                                         mean_longitude, data
                                         );

  int num_loaded_points = data.features.cols();
  if (!lonlat_box.empty()                    &&
      num_loaded_points < num_points_to_load &&
      num_loaded_points < num_total_points){
    // We loaded too few points. Just load them all, as CSV files are
    // not too large, we will drop extraneous points later.
    load_csv_aux<T>(file_name, num_total_points, lonlat_box,
                    false, // Skip repeating same messages
                    calc_shift, shift,
                    geo, C, is_lola_rdr_format,
                    mean_longitude, data
                    );
  }

  return;
}

// Load a DEM
template<typename T>
void load_dem(bool verbose, string const& file_name,
              int num_points_to_load, BBox2 const& lonlat_box,
              bool calc_shift, Vector3 & shift,
              typename PointMatcher<T>::DataPoints & data){

  validateFile(file_name);

  data.features.conservativeResize(DIM+1, num_points_to_load);
  data.featureLabels = form_labels<T>(DIM);

  cartography::GeoReference dem_geo;
  bool is_good = cartography::read_georeference( dem_geo, file_name );
  if (!is_good) vw_throw(ArgumentErr() << "DEM: " << file_name
                         << " does not have a georeference.\n");

  DiskImageView<float> dem(file_name);
  double nodata = numeric_limits<double>::quiet_NaN();
  boost::shared_ptr<DiskImageResource> dem_rsrc
    ( new DiskImageResourceGDAL(file_name) );
  if (dem_rsrc->has_nodata_read()) nodata = dem_rsrc->nodata_read();

  // Load only points within lonlat_box
  BBox2i pix_box;
  if (!lonlat_box.empty()){
    pix_box.grow(dem_geo.lonlat_to_pixel(lonlat_box.min()));
    pix_box.grow(dem_geo.lonlat_to_pixel(lonlat_box.max()));
    pix_box.grow(dem_geo.lonlat_to_pixel(Vector2(lonlat_box.min().x(),
                                                 lonlat_box.max().y())));
    pix_box.grow(dem_geo.lonlat_to_pixel(Vector2(lonlat_box.max().x(),
                                                 lonlat_box.min().y())));
    pix_box.expand(1); // to counteract casting to int
    pix_box.crop(bounding_box(dem));
  }
  if (pix_box.empty())
    pix_box = bounding_box(dem);

  // We will randomly pick or not a point with probability load_ratio
  int num_points = pix_box.width()*pix_box.height();
  double load_ratio = (double)num_points_to_load/std::max(1.0, (double)num_points);

  bool shift_was_calc = false;
  int points_count = 0;

  TerminalProgressCallback tpc("asp", "\t--> ");
  double inc_amount = 1.0 / double(pix_box.width() );
  if (verbose) tpc.report_progress(0);

  for (int i = pix_box.min().x(); i < pix_box.max().x(); i++ ) {
    if (points_count >= num_points_to_load) break;

    for (int j = pix_box.min().y(); j < pix_box.max().y(); j++ ) {
      if (points_count >= num_points_to_load) break;

      double r = (double)std::rand()/(double)RAND_MAX;
      if (r > load_ratio) continue;

      if (dem(i, j) == nodata) continue;

      Vector2 lonlat = dem_geo.pixel_to_lonlat( Vector2(i,j) );

      // Skip points outside the given box
      if (!lonlat_box.empty() && !lonlat_box.contains(lonlat)) continue;

      Vector3 llh( lonlat.x(), lonlat.y(), dem(i,j) );
      Vector3 xyz = dem_geo.datum().geodetic_to_cartesian( llh );
      if ( xyz == Vector3() || !(xyz == xyz) ) continue; // invalid and NaN check

      if (calc_shift && !shift_was_calc){
        shift = xyz;
        shift_was_calc = true;
      }

      for (int row = 0; row < DIM; row++)
        data.features(row, points_count) = xyz[row] - shift[row];
      data.features(DIM, points_count) = 1;

      points_count++;
    }

    if (verbose) tpc.report_incremental_progress( inc_amount );
  }
  if (verbose) tpc.report_finished();

  data.features.conservativeResize(Eigen::NoChange, points_count);

}

template<typename T>
int64 load_pc_aux(bool verbose,
                  string const& file_name,
                  int num_points_to_load,
                  BBox2 const& lonlat_box,
                  bool calc_shift,
                  Vector3 & shift,
                  GeoReference const& geo,
                  typename PointMatcher<T>::DataPoints & data){

  validateFile(file_name);

  data.features.conservativeResize(DIM+1, num_points_to_load);
  data.featureLabels = form_labels<T>(DIM);

  // To do: Is it faster to to do for_each?
  ImageViewRef<Vector3> point_cloud = asp::read_cloud<DIM>(file_name);

  // We will randomly pick or not a point with probability load_ratio
  int64 num_total_points = point_cloud.cols()*point_cloud.rows();
  double load_ratio
    = (double)num_points_to_load/std::max(1.0, (double)num_total_points);

  bool shift_was_calc = false;
  int64 points_count = 0;

  TerminalProgressCallback tpc("asp", "\t--> ");
  double inc_amount = 1.0 / double(point_cloud.rows() );
  if (verbose) tpc.report_progress(0);

  for (int j = 0; j < point_cloud.rows(); j++ ) {

    if (points_count >= num_points_to_load) break;

    for ( int i = 0; i < point_cloud.cols(); i++ ) {

      if (points_count >= num_points_to_load) break;

      double r = (double)std::rand()/(double)RAND_MAX;
      if (r > load_ratio) continue;

      Vector3 xyz = point_cloud(i, j);
      if ( xyz == Vector3() || !(xyz == xyz) ) continue; // invalid and NaN check

      if (calc_shift && !shift_was_calc){
        shift = xyz;
        shift_was_calc = true;
      }

      // Skip points outside the given box
      if (!lonlat_box.empty()){
        Vector3 llh = geo.datum().cartesian_to_geodetic(xyz);
        if ( !lonlat_box.contains(subvector(llh, 0, 2)))
          continue;
      }

      for (int row = 0; row < DIM; row++)
        data.features(row, points_count) = xyz[row] - shift[row];
      data.features(DIM, points_count) = 1;

      points_count++;
    }
    if (verbose) tpc.report_incremental_progress( inc_amount );
  }
  if (verbose) tpc.report_finished();

  data.features.conservativeResize(Eigen::NoChange, points_count);

  return num_total_points;
}

template<typename T>
int64 load_las_aux(bool verbose,
                  string const& file_name,
                  int num_points_to_load,
                  BBox2 const& lonlat_box,
                  bool calc_shift,
                  Vector3 & shift,
                  GeoReference const& geo,
                  typename PointMatcher<T>::DataPoints & data){

  validateFile(file_name);

  data.features.conservativeResize(DIM+1, num_points_to_load);
  data.featureLabels = form_labels<T>(DIM);

  GeoReference las_georef;
  bool has_georef = asp::georef_from_las(file_name, las_georef);

  std::ifstream ifs;
  ifs.open(file_name.c_str(), std::ios::in | std::ios::binary);
  liblas::ReaderFactory f;
  liblas::Reader reader = f.CreateWithStream(ifs);

  // We will randomly pick or not a point with probability load_ratio
  int64 num_total_points = asp::las_file_size(file_name);
  double load_ratio
    = (double)num_points_to_load/std::max(1.0, (double)num_total_points);

  bool shift_was_calc = false;
  int64 points_count = 0;

  TerminalProgressCallback tpc("asp", "\t--> ");
  int hundred = 100;
  int spacing = num_total_points/hundred;
  double inc_amount = 1.0 / hundred;
  if (verbose) tpc.report_progress(0);

  while (reader.ReadNextPoint()){

    if (points_count >= num_points_to_load) break;

    double r = (double)std::rand()/(double)RAND_MAX;
    if (r > load_ratio) continue;

    liblas::Point const& p = reader.GetPoint();
    Vector3 xyz(p.GetX(), p.GetY(), p.GetZ());
    if (has_georef){
      Vector2 ll = las_georef.point_to_lonlat(subvector(xyz, 0, 2));
      xyz = las_georef.datum().geodetic_to_cartesian(Vector3(ll[0], ll[1], xyz[2]));
    }

    if (calc_shift && !shift_was_calc){
      shift = xyz;
      shift_was_calc = true;
    }

    // Skip points outside the given box
    if (!lonlat_box.empty()){
      Vector3 llh = geo.datum().cartesian_to_geodetic(xyz);
      if ( !lonlat_box.contains(subvector(llh, 0, 2)))
        continue;
    }

    for (int row = 0; row < DIM; row++)
      data.features(row, points_count) = xyz[row] - shift[row];
    data.features(DIM, points_count) = 1;

    if (verbose && points_count%spacing == 0) tpc.report_incremental_progress( inc_amount );

    points_count++;
  }

  if (verbose) tpc.report_finished();

  data.features.conservativeResize(Eigen::NoChange, points_count);

  return num_total_points;
}

template<typename T>
void load_pc(bool verbose,
             string const& file_name,
             int num_points_to_load,
             BBox2 const& lonlat_box,
             bool calc_shift,
             Vector3 & shift,
             GeoReference const& geo,
             typename PointMatcher<T>::DataPoints & data
             ){

  int64 num_total_points = load_pc_aux<T>(verbose,
                                          file_name, num_points_to_load,
                                          lonlat_box, calc_shift, shift,
                                          geo, data);

  int num_loaded_points = data.features.cols();
  if (!lonlat_box.empty()                    &&
      num_loaded_points < num_points_to_load &&
      num_loaded_points < num_total_points){

    // We loaded too few points. Try harder. Need some care here as to not run
    // out of memory.
    num_points_to_load = std::max(4*num_points_to_load, 10000000);
    if (verbose)
      vw_out() << "Too few points were loaded. Trying again." << endl;
    load_pc_aux<T>(verbose,
                   file_name, num_points_to_load, lonlat_box,
                   calc_shift, shift, geo, data);
  }

}

template<typename T>
void load_las(bool verbose,
             string const& file_name,
             int num_points_to_load,
             BBox2 const& lonlat_box,
             bool calc_shift,
             Vector3 & shift,
             GeoReference const& geo,
             typename PointMatcher<T>::DataPoints & data
             ){

  int64 num_total_points = load_las_aux<T>(verbose,
                                          file_name, num_points_to_load,
                                          lonlat_box, calc_shift, shift,
                                          geo, data);

  int num_loaded_points = data.features.cols();
  if (!lonlat_box.empty()                    &&
      num_loaded_points < num_points_to_load &&
      num_loaded_points < num_total_points){

    // We loaded too few points. Try harder. Need some care here as to not run
    // out of memory.
    num_points_to_load = std::max(4*num_points_to_load, 10000000);
    if (verbose)
      vw_out() << "Too few points were loaded. Trying again." << endl;
    load_las_aux<T>(verbose,
                   file_name, num_points_to_load, lonlat_box,
                   calc_shift, shift, geo, data);
  }

}

// Load file from disk and convert to libpointmatcher's format
template<typename T>
void load_file(string const& file_name,
               int num_points_to_load,
               BBox2 const& lonlat_box,
               bool calc_shift,
               Vector3 & shift,
               GeoReference const& geo,
               asp::CsvConv const& csv_conv,
               bool & is_lola_rdr_format,
               double & mean_longitude,
               bool verbose,
               typename PointMatcher<T>::DataPoints & data){

  if (verbose)
    vw_out() << "Reading: " << file_name << endl;

  // We will over-write this below for CSV and DEM files where
  // longitude is available.
  mean_longitude = 0.0;

  string file_type = get_file_type(file_name);
  if (file_type == "DEM")
    load_dem<T>(verbose,
                file_name, num_points_to_load, lonlat_box,
                calc_shift, shift, data);
  else if (file_type == "PC")
    load_pc<T>(verbose,
               file_name, num_points_to_load, lonlat_box, calc_shift, shift,
               geo, data);
  else if (file_type == "LAS")
    load_las<T>(verbose,
                file_name, num_points_to_load, lonlat_box, calc_shift, shift,
                geo, data);
  else if (file_type == "CSV"){
    bool verbose = true;
    load_csv<T>(file_name, num_points_to_load, lonlat_box, verbose,
                calc_shift, shift, geo, csv_conv, is_lola_rdr_format,
                mean_longitude, data
                );
  }else
    vw_throw( ArgumentErr() << "Unknown file type: " << file_name << "\n" );

  if (verbose)
    vw_out() << "Loaded points: " << data.features.cols() << endl;

}

// Calculate the lon-lat bounding box of the points and bias it based
// on max displacement (which is in meters). This is used to throw
// away points in the other cloud which are not within this box.
BBox2 calc_extended_lonlat_bbox(GeoReference const& geo,
                                int num_sample_pts,
                                asp::CsvConv const& C,
                                string const& file_name,
                                double max_disp){

  // If the user does not want to use the max-displacement parameter,
  // or if there is no datum to use to convert to/from lon/lat,
  // there is not much we can do.
  if (max_disp < 0.0 || geo.datum().name() == UNSPECIFIED_DATUM)
    return BBox2();

  validateFile(file_name);
  PointMatcher<RealT>::DataPoints points;

  double mean_longitude = 0.0; // to convert back from xyz to lonlat
  bool verbose = false;
  bool calc_shift = false; // won't shift the points
  Vector3 shift = Vector3(0, 0, 0);
  BBox2 dummy_box;
  bool is_lola_rdr_format;
  // Load a sample of points, hopefully enough to estimate the box
  // reliably.
  load_file<RealT>(file_name, num_sample_pts, dummy_box,
                   calc_shift, shift, geo, C, is_lola_rdr_format,
                   mean_longitude, verbose, points);

  // Bias the xyz points in several directions by max_disp, then
  // convert to lon-lat and grow the box. This is a rough
  // overestimate, but should be good enough.
  BBox2 box;
  for (int col = 0; col < points.features.cols(); col++){
    Vector3 p;
    for (int row = 0; row < DIM; row++) p[row] = points.features(row, col);

    for (int x = -1; x <= 1; x += 2){
      for (int y = -1; y <= 1; y += 2){
        for (int z = -1; z <= 1; z += 2){
          Vector3 q = p + Vector3(x, y, z)*max_disp;
          Vector3 llh = geo.datum().cartesian_to_geodetic(q);
          llh[0] += 360.0*round((mean_longitude - llh[0])/360.0); // 360 deg adjust
          box.grow(subvector(llh, 0, 2));
        }
      }
    }
  }

  return box;
}

double calc_mean(vector<double> const& errs, int len){
  double mean = 0.0;
  for (int i = 0; i < len; i++){
    mean += errs[i];
  }
  if (len == 0) return 0;
  return mean/len;
}

double calc_stddev(vector<double> const& errs, double mean){
  double stddev = 0.0;
  int len = errs.size();
  for (int i = 0; i < len; i++){
    stddev += (errs[i] - mean)*(errs[i] - mean);
  }
  if (len == 0) return 0;
  return sqrt(stddev/len);
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
  vw_out() << "Number of errors: " << len << endl;
  if (len == 0) return;

  double p16 = errs[std::min(len-1, (int)round(len*0.16))];
  double p50 = errs[std::min(len-1, (int)round(len*0.50))];
  double p84 = errs[std::min(len-1, (int)round(len*0.84))];
  vw_out() << label << ": error percentile of smallest errors (meters):"
           << " 16%: " << p16 << ", 50%: " << p50 << ", 84%: " << p84 << endl;

  double a25 = calc_mean(errs, len/4),   a50  = calc_mean(errs, len/2);
  double a75 = calc_mean(errs, 3*len/4), a100 = calc_mean(errs, len);
  vw_out() << label << ": mean of smallest errors (meters):"
           << " 25%: " << a25 << ", 50%: " << a50
           << ", 75%: " << a75 << ", 100%: " << a100 << endl;
}

void dump_llh(string const& file, Datum const& datum,
              DP const & data, Vector3 const& shift){

  vw_out() << "Writing: " << data.features.cols()
           << " points to " << file << std::endl;

  ofstream fs(file.c_str());
  fs.precision(20);
  for (int c = 0; c < data.features.cols(); c++){
    Vector3 xyz;
    for (int r = 0; r < data.features.rows() - 1; r++)
      xyz[r] = data.features(r, c) + shift[r];
    Vector3 llh = datum.cartesian_to_geodetic(xyz);
    fs << llh[0] << ' ' << llh[1] << ' ' << llh[2] << endl;
  }
  fs.close();
}

void save_transforms(Options const& opt,
                     PointMatcher<RealT>::Matrix const& T){

  // Save the transform and its inverse.

  string transFile = opt.out_prefix + "-transform.txt";
  vw_out() << "Writing: " << transFile  << endl;
  ofstream tf(transFile.c_str());
  tf.precision(16);
  tf << T << endl;
  tf.close();

  string iTransFile = opt.out_prefix + "-inverse-transform.txt";
  PointMatcher<RealT>::Matrix invT = T.inverse();
  vw_out() << "Writing: " << iTransFile  << endl;
  ofstream itf(iTransFile.c_str());
  itf.precision(16);
  itf << invT << endl;
  itf.close();

}

void calc_translation_vec(DP const& source, DP const& trans_source,
                          Vector3 & shift, // from planet center to current origin
                          Datum const& datum,
                          Vector3 & source_ctr_vec,
                          Vector3 & source_ctr_llh,
                          Vector3 & trans_xyz,
                          Vector3 & trans_ned,
                          Vector3 & trans_llh){

  Eigen::VectorXd source_ctr
    = source.features.rowwise().sum() / source.features.cols();
  Eigen::VectorXd trans_source_ctr
    = trans_source.features.rowwise().sum() / trans_source.features.cols();

  Vector3 trans_source_ctr_vec;
  for (int row = 0; row < DIM; row++){
    source_ctr_vec[row]       = source_ctr(row, 0);
    trans_source_ctr_vec[row] = trans_source_ctr(row, 0);
  }

  // Make these vectors in reference to the center of the planet
  source_ctr_vec       += shift;
  trans_source_ctr_vec += shift;

  trans_xyz = trans_source_ctr_vec - source_ctr_vec;

  source_ctr_llh = datum.cartesian_to_geodetic(source_ctr_vec);
  Vector3 trans_source_ctr_llh = datum.cartesian_to_geodetic(trans_source_ctr_vec);
  trans_llh = trans_source_ctr_llh - source_ctr_llh;

  Matrix3x3 M = datum.lonlat_to_ned_matrix(subvector(source_ctr_llh, 0, 2));
  trans_ned = M*trans_xyz;
}

void calc_max_displacment(DP const& source, DP const& trans_source){

  double max_obtained_disp = 0.0;
  int numPts = source.features.cols();
  for(int col = 0; col < numPts; col++){
    Vector3 s, t;
    for (int row = 0; row < DIM; row++){
      s[row] = source.features(row, col);
      t[row] = trans_source.features(row, col);
    }
    max_obtained_disp = max(max_obtained_disp, norm_2(s - t));
  }

  vw_out() << "Maximum displacement of source points: "
           << max_obtained_disp << " m" << endl;
}

void save_errors(DP const& point_cloud,
                 PointMatcher<RealT>::Matrix const& errors,
                 string const& output_file,
                 Vector3 const& shift,
                 GeoReference const& geo,
                 asp::CsvConv const& C,
                 bool is_lola_rdr_format,
                 double mean_longitude
                 ){

  // Save the lon, lat, radius/height, and error. Use a format
  // consistent with the input CSV format.

  vw_out() << "Writing: " << output_file << std::endl;

  VW_ASSERT(point_cloud.features.cols() == errors.cols(),
            ArgumentErr() << "Expecting as many errors as source points.");

  ofstream outfile( output_file.c_str() );
  outfile.precision(16);

  // Write the header line
  if (C.csv_format_str != ""){
    outfile << "# ";
    for (map<int, string>::const_iterator it = C.col2name.begin();
         it != C.col2name.end(); it++){
      outfile << it->second << ",";
    }
    outfile << "error (meters)" << endl;
  }else{
    if (is_lola_rdr_format)
      outfile << "# longitude,latitude,radius (km),error (meters)" << endl;
    else
      outfile << "# latitude,longitude,height above datum (meters),error (meters)" << endl;
  }

  int numPts = point_cloud.features.cols();
  for(int col = 0; col < numPts; col++){

    Vector3 P;
    for (int row = 0; row < DIM; row++)
      P[row] = point_cloud.features(row, col) + shift[row];

    if (C.csv_format_str != ""){

      Vector3 csv = cartesian_to_csv(P, geo, mean_longitude, C);
      outfile << csv[0] << ',' << csv[1] << ',' << csv[2]
              << "," << errors(0, col) << endl;

    }else{
      Vector3 llh = geo.datum().cartesian_to_geodetic(P); // lon-lat-height
      llh[0] += 360.0*round((mean_longitude - llh[0])/360.0); // 360 deg adjustment

      if (is_lola_rdr_format)
        outfile << llh[0] << ',' << llh[1] << ',' << norm_2(P)/1000.0
                << "," << errors(0, col) << endl;
      else
        outfile << llh[1] << ',' << llh[0] << ',' << llh[2]
                << "," << errors(0, col) << endl;
    }
  }
  outfile.close();
}

namespace asp{
  Vector3 apply_transform(PointMatcher<RealT>::Matrix const& T, Vector3 const& P){

    Eigen::VectorXd V(4);
    V[0] = P[0]; V[1] = P[1]; V[2] = P[2]; V[3] = 1;
    V = T*V;
    Vector3 Q;
    Q[0] = V[0]; Q[1] = V[1]; Q[2] = V[2];
    return Q;
  }
}

// Apply a transform to the first three coordinates of the cloud
struct TransformPC: public UnaryReturnSameType {
  PointMatcher<RealT>::Matrix m_T;
  TransformPC(PointMatcher<RealT>::Matrix const& T):m_T(T){}
  inline Vector<double> operator()(Vector<double> const& pt) const {

    Vector<double> P = pt; // local copy
    Vector3 xyz = subvector(P, 0, 3);

    if (xyz == Vector3())
      return P; // invalid point

    Vector3 Q = asp::apply_transform(m_T, xyz);
    subvector(P, 0, 3) = Q;

    return P;
  }
};

template<int n>
void save_trans_point_cloud_n(Options const& opt,
                              string input_file,
                              string output_file,
                              PointMatcher<RealT>::Matrix const& T){

  ImageViewRef< Vector<double, n> > point_cloud = asp::read_cloud<n>(input_file);
  asp::block_write_gdal_image(output_file,
                              per_pixel_filter(point_cloud, TransformPC(T)),
                              opt,
                              TerminalProgressCallback("asp", "\t--> "));
}

void save_trans_point_cloud(Options const& opt,
                            string input_file,
                            string out_prefix,
                            GeoReference const& geo,
                            asp::CsvConv const& C,
                            PointMatcher<RealT>::Matrix const& T){

  // Apply a given transform to the point cloud in input file,
  // and save it.

  // Note: We transform the entire point cloud, not just the resampled
  // version used in alignment.

  string file_type = get_file_type(input_file);

  string output_file;
  if (file_type == "CSV")
    output_file = out_prefix + ".csv";
  else if (file_type == "LAS")
    output_file = out_prefix + fs::path(input_file).extension().string();
  else
    output_file = out_prefix + ".tif";
  vw_out() << "Writing: " << output_file << endl;

  if (file_type == "DEM"){

    cartography::GeoReference dem_geo;
    bool is_good = cartography::read_georeference( dem_geo, input_file );
    if (!is_good) vw_throw(ArgumentErr() << "DEM: " << input_file
                           << " does not have a georeference.\n");

    DiskImageView<float> dem(input_file);
    double nodata = numeric_limits<double>::quiet_NaN();
    boost::shared_ptr<DiskImageResource> dem_rsrc
      ( new DiskImageResourceGDAL(input_file) );
    if (dem_rsrc->has_nodata_read()) nodata = dem_rsrc->nodata_read();

    ImageViewRef<Vector3> point_cloud =
      geodetic_to_cartesian( dem_to_geodetic( create_mask(dem, nodata),
                                              dem_geo ),
                             dem_geo.datum() );
    asp::block_write_gdal_image(output_file,
                                per_pixel_filter(point_cloud, TransformPC(T)),
                                opt,
                                TerminalProgressCallback("asp", "\t--> "));

  }else if (file_type == "PC"){

    // Need this logic because we cannot open an image
    // with n channels without knowing n beforehand.
    int nc = get_num_channels(input_file);
    switch(nc){
    case 3:
      save_trans_point_cloud_n<3>(opt, input_file, output_file, T);
      break;
    case 4:
      save_trans_point_cloud_n<4>(opt, input_file, output_file, T);
      break;
    case 6:
      save_trans_point_cloud_n<6>(opt, input_file, output_file, T);
      break;
    default:
      vw_throw( ArgumentErr() << "The point cloud from " << input_file
                << " has " << nc << " channels, which is not supported.\n" );
    }

  }else if (file_type == "LAS"){

    int64 num_total_points = asp::las_file_size(input_file);
    GeoReference las_georef;
    bool has_georef = asp::georef_from_las(input_file, las_georef);

    std::ifstream ifs;
    ifs.open(input_file.c_str(), std::ios::in | std::ios::binary);
    liblas::ReaderFactory f;
    liblas::Reader reader = f.CreateWithStream(ifs);
    liblas::Header const& header = reader.GetHeader();

    std::ofstream ofs;
    ofs.open(output_file.c_str(), std::ios::out | std::ios::binary);
    liblas::Writer writer(ofs, header);

    TerminalProgressCallback tpc("asp", "\t--> ");
    int hundred = 100;
    int spacing = num_total_points/hundred;
    double inc_amount = 1.0 / hundred;
    int64 count = 0;
    while (reader.ReadNextPoint()){

      liblas::Point const& in_las_pt = reader.GetPoint();
      Vector3 P(in_las_pt.GetX(), in_las_pt.GetY(), in_las_pt.GetZ());
      if (has_georef){
        // Go from projected space to xyz
        Vector2 ll = las_georef.point_to_lonlat(subvector(P, 0, 2));
        P = las_georef.datum().geodetic_to_cartesian(Vector3(ll[0], ll[1], P[2]));
      }
      P = asp::apply_transform(T, P);
      if (has_georef){
        // Go from xyz to projected space
        Vector3 llh = las_georef.datum().cartesian_to_geodetic(P);
        subvector(P, 0, 2) = las_georef.lonlat_to_point(subvector(llh, 0, 2));
        P[2] = llh[2];
      }

      liblas::Point out_las_pt(&header);
      out_las_pt.SetCoordinates(P[0], P[1], P[2]);
      writer.WritePoint(out_las_pt);

      if (count%spacing == 0) tpc.report_incremental_progress( inc_amount );

      count++;
    }
    tpc.report_finished();

  }else if (file_type == "CSV"){

    // Write a CSV file in format consistent with the input CSV file.

    BBox2 empty_box;
    bool verbose = false;
    bool calc_shift = true;
    Vector3 shift;
    bool is_lola_rdr_format;
    double mean_longitude;
    DP point_cloud;
    load_csv<RealT>(input_file, numeric_limits<int>::max(),
                    empty_box, verbose, calc_shift, shift,
                    geo, C, is_lola_rdr_format,
                    mean_longitude, point_cloud);

    ofstream outfile( output_file.c_str() );
    outfile.precision(16);

    // Write the header line
    if (C.csv_format_str != ""){
      outfile << "# ";
      for (map<int, string>::const_iterator it = C.col2name.begin();
           it != C.col2name.end(); it++){
        outfile << it->second << ",";
      }
      outfile << endl;
    }else{
      if (is_lola_rdr_format)
        outfile << "# longitude,latitude,radius (km)" << endl;
      else
        outfile << "# latitude,longitude,height above datum (meters)" << endl;
    }

    int numPts = point_cloud.features.cols();
    TerminalProgressCallback tpc("asp", "\t--> ");
    int hundred = 100;
    int spacing = numPts/hundred;
    double inc_amount = 1.0 / hundred;
    for (int col = 0; col < numPts; col++){

      Eigen::VectorXd V(DIM + 1);
      for (int row = 0; row < DIM; row++)
        V[row] = point_cloud.features(row, col) + shift[row];
      V[DIM] = 1;

      // Apply the transform
      V = T*V;

      Vector3 P;
      for (int row = 0; row < DIM; row++) P[row] = V[row];

      if (C.csv_format_str != ""){

        Vector3 csv = cartesian_to_csv(P, geo, mean_longitude, C);
        outfile << csv[0] << ',' << csv[1] << ',' << csv[2] << endl;

      }else{
        Vector3 llh = geo.datum().cartesian_to_geodetic(P); // lon-lat-height
        llh[0] += 360.0*round((mean_longitude - llh[0])/360.0); // 360 deg adjustment

        if (is_lola_rdr_format)
          outfile << llh[0] << ',' << llh[1] << ',' << norm_2(P)/1000.0 << endl;
        else
          outfile << llh[1] << ',' << llh[0] << ',' << llh[2] << endl;
      }

      if (col%spacing == 0) tpc.report_incremental_progress( inc_amount );
    }
    tpc.report_finished();
    outfile.close();

  }else{
    vw_throw( ArgumentErr() << "Unknown file type: " << input_file << "\n" );
  }

}

void debug_save_point_cloud(DP const& point_cloud, GeoReference const& geo,
                            Vector3 const& shift,
                            string const& output_file){

  int numPts = point_cloud.features.cols();

  vw_out() << "Writing: " << output_file << endl;
  ofstream outfile( output_file.c_str() );
  outfile.precision(16);

  for(int col = 0; col < numPts; col++){

    Vector3 P;
    for (int row = 0; row < DIM; row++)
      P[row] = point_cloud.features(row, col) + shift[row];

    Vector3 llh = geo.datum().cartesian_to_geodetic(P); // lon-lat-height
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
    omp_set_num_threads(opt.num_threads);

    asp::CsvConv C;
    parse_csv_format(opt.csv_format_str, opt.csv_proj4_str, C);

    Datum datum(UNSPECIFIED_DATUM, "User Specified Spheroid",
                "Reference Meridian", 1, 1, 0);
    read_datum(opt, C, datum);
    GeoReference geo(datum);

    // Set user's csv_proj4_str if specified
    asp::handle_easting_northing(C, geo);

    // We will use ref_box to bound the source points, and vice-versa.
    // Decide how many samples to pick to estimate these boxes.
    Stopwatch sw0;
    sw0.start();
    int num_sample_pts = std::max(4000000,
                                  std::max(opt.max_num_source_points,
                                           opt.max_num_reference_points)/4);
    BBox2 ref_box, source_box;
    ref_box = calc_extended_lonlat_bbox(geo, num_sample_pts,
                                        C, opt.reference, opt.max_disp);
    source_box = calc_extended_lonlat_bbox(geo, num_sample_pts,
                                           C, opt.source, opt.max_disp);
    // If ref points are offset by 360 degrees in longitude in respect to
    // source points, adjust the ref box to be aligned with the source points,
    // and vice versa.
    double lon_offset = 0.0;
    if (!ref_box.empty() && !source_box.empty()){
      lon_offset = (source_box.min().x() + source_box.max().x())/2.0
        - (ref_box.min().x() + ref_box.max().x())/2.0;
      lon_offset = 360.0*round(lon_offset/360.0);
      ref_box    += Vector2(lon_offset, 0);
      ref_box.crop(source_box); source_box.crop(ref_box); // common area
      source_box -= Vector2(lon_offset, 0);
    }
    sw0.stop();
    if (opt.verbose) vw_out() << "Determination of the intersection "
                              << "of bounding boxes of the reference"
                              << " and source points took "
                              << sw0.elapsed_seconds() << " [s]" << endl;

    // Load the point clouds. We will shift both point clouds by the
    // centroid of the first one to bring them closer to origin.

    // Load the subsampled reference point cloud.
    Vector3 shift;
    bool calc_shift = true;
    bool is_lola_rdr_format = false;     // may get overwritten
    double mean_ref_longitude    = 0.0;  // may get overwritten
    double mean_source_longitude = 0.0;  // may get overwritten
    Stopwatch sw1;
    sw1.start();
    DP ref;
    load_file<RealT>(opt.reference, opt.max_num_reference_points,
                     source_box, // source box is used to bound reference
                     calc_shift, shift, geo, C, is_lola_rdr_format,
                     mean_ref_longitude, opt.verbose, ref);
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
    DP source;
    load_file<RealT>(opt.source, num_source_pts,
                     ref_box, // ref box is used to bound source
                     calc_shift, shift, geo, C, is_lola_rdr_format,
                     mean_source_longitude, opt.verbose, source);
    sw2.stop();
    if (opt.verbose) vw_out() << "Loading the source point cloud took "
                              << sw2.elapsed_seconds() << " [s]" << endl;

    // So far we shifted by first point in first point cloud to reduce
    // the magnitude of all loaded points. Now that we have loaded all
    // points, shift one more time, to place the centroid of the
    // reference at the origin.
    // Note: If this code is ever converting to using floats,
    // the operation below needs to be re-implemented to be accurate.
    int numRefPts = ref.features.cols();
    Eigen::VectorXd meanRef = ref.features.rowwise().sum() / numRefPts;
    ref.features.topRows(DIM).colwise()    -= meanRef.head(DIM);
    source.features.topRows(DIM).colwise() -= meanRef.head(DIM);
    for (int row = 0; row < DIM; row++) shift[row] += meanRef(row);
    if (opt.verbose) vw_out() << "Data shifted internally by subtracting: "
                              << shift << std::endl;


    // The point clouds are shifted, so shift the initial transform as well.
    PointMatcher<RealT>::Matrix initT = apply_shift(opt.init_transform, shift);

    // Filter the reference and initialize the reference tree
    PM::ICP icp;
    Stopwatch sw3;
    sw3.start();
    icp.initRefTree(ref, opt.alignment_method, opt.highest_accuracy,
                    false /*opt.verbose*/);
    sw3.stop();
    if (opt.verbose) vw_out() << "Reference point cloud processing took "
                              << sw3.elapsed_seconds() << " [s]" << endl;

    // Apply the initial guess transform to the source point cloud.
    icp.transformations.apply(source, initT);

    PointMatcher<RealT>::Matrix beg_errors;
    if (opt.max_disp > 0.0){
      // Filter gross outliers
      Stopwatch sw4;
      sw4.start();
      icp.filterGrossOutliersAndCalcErrors(ref, opt.max_disp*opt.max_disp,
                                            source, beg_errors); //in-out
      sw4.stop();
      if (opt.verbose) vw_out() << "Filter gross outliers took "
                                << sw4.elapsed_seconds() << " [s]" << endl;
    }

    random_pc_subsample<RealT>(opt.max_num_source_points, source);
    vw_out() << "Reducing number of source points to " << source.features.cols()
             << endl;

    //dump_llh("ref.csv", datum, ref,    shift);
    //dump_llh("src.csv", datum, source, shift);

    // Calculate the errors before doing ICP
    Stopwatch sw5;
    sw5.start();
    double big = 1e+300;
    icp.filterGrossOutliersAndCalcErrors(ref, big,
                                          source, beg_errors); //in-out
    calc_stats("Input", beg_errors);
    sw5.stop();
    if (opt.verbose) vw_out() << "Initial error computation took "
                              << sw5.elapsed_seconds() << " [s]" << endl;

    // Compute the transformation to align the source to reference.
    Stopwatch sw6;
    sw6.start();
    PointMatcher<RealT>::Matrix Id
      = PointMatcher<RealT>::Matrix::Identity(DIM + 1, DIM + 1);
    if (opt.config_file == ""){
      // Read the options from the command line
      icp.setParams(opt.out_prefix, opt.num_iter, opt.outlier_ratio,
                    (2.0*M_PI/360.0)*opt.diff_rotation_err, // convert to radians
                    opt.diff_translation_err, opt.alignment_method,
                    false/*opt.verbose*/);
    }else{
      vw_out() << "Will read the options from: " << opt.config_file << endl;
      ifstream ifs(opt.config_file.c_str());
      if (!ifs.good())
        vw_throw( ArgumentErr() << "Cannot open configuration file: "
                  << opt.config_file << "\n" );
      icp.loadFromYaml(ifs);
    }
    // We bypass calling ICP if the user explicitely asks for 0 iterations.
    PointMatcher<RealT>::Matrix T = Id;
    if (opt.num_iter > 0){
      T = icp(source, ref, Id,
              opt.compute_translation_only);
      vw_out() << "Match ratio: "
               << icp.errorMinimizer->getWeightedPointUsedRatio() << endl;
    }
    sw6.stop();
    if (opt.verbose) vw_out() << "ICP took "
                              << sw6.elapsed_seconds() << " [s]" << endl;

    // Transform the source to make it close to reference.
    DP trans_source(source);
    icp.transformations.apply(trans_source, T);

    // Calculate by how much points move as result of T
    calc_max_displacment(source, trans_source);
    Vector3 source_ctr_vec, source_ctr_llh;
    Vector3 trans_xyz, trans_ned, trans_llh;
    calc_translation_vec(source, trans_source, shift, geo.datum(),
                         source_ctr_vec, source_ctr_llh,
                         trans_xyz, trans_ned, trans_llh);

    // Calculate the errors after doing ICP
    PointMatcher<RealT>::Matrix end_errors;
    Stopwatch sw7;
    sw7.start();
    icp.filterGrossOutliersAndCalcErrors(ref, big,
                                          trans_source, end_errors); // in-out
    calc_stats("Output", end_errors);
    sw7.stop();
    if (opt.verbose) vw_out() << "Final error computation took "
                              << sw7.elapsed_seconds() << " [s]" << endl;

    // We must apply to T the initial guess transform
    PointMatcher<RealT>::Matrix combinedT = T*initT;

    // Go back to the original coordinate system, undoing the shift
    PointMatcher<RealT>::Matrix globalT = apply_shift(combinedT, -shift);

    // Print statistics
    vw_out() << "Alignment transform (rotation + translation, "
         << "origin is planet center):" << endl << globalT << endl;
    vw_out() << "Centroid of source points (Cartesian, meters): " << source_ctr_vec << std::endl;
    // Swap lat and lon, as we want to print lat first
    std::swap(source_ctr_llh[0], source_ctr_llh[1]);
    vw_out() << "Centroid of source points (lat,lon,z): " << source_ctr_llh << std::endl;
    vw_out() << std::endl;

    vw_out() << "Translation vector (Cartesian, meters): " << trans_xyz << std::endl;
    vw_out() << "Translation vector (North-East-Down, meters): "
         << trans_ned << std::endl;
    vw_out() << "Translation vector magnitude (meters): " << norm_2(trans_xyz)
         << std::endl;
    // Swap lat and lon, as we want to print lat first
    std::swap(trans_llh[0], trans_llh[1]);
    vw_out() << "Translation vector (lat,lon,z): " << trans_llh << std::endl;
    vw_out() << std::endl;

    Matrix3x3 rot;
    for (int r = 0; r < DIM; r++) for (int c = 0; c < DIM; c++)
      rot(r, c) = globalT(r, c);
    Vector3 euler_angles = math::rotation_matrix_to_euler_xyz(rot) * 180/M_PI;
    Vector3 axis_angles = math::matrix_to_axis_angle(rot) * 180/M_PI;
    vw_out() << "Euler angles (degrees): " << euler_angles  << endl;
    vw_out() << "Axis of rotation and angle (degrees): "
         << axis_angles/norm_2(axis_angles) << ' '
         << norm_2(axis_angles) << endl;

    Stopwatch sw8;
    sw8.start();
    save_transforms(opt, globalT);

    if (opt.save_trans_ref){
      string trans_ref_prefix = opt.out_prefix + "-trans_reference";
      save_trans_point_cloud(opt, opt.reference, trans_ref_prefix,
                             geo, C, globalT.inverse());
    }

    if (opt.save_trans_source){
      string trans_source_prefix = opt.out_prefix + "-trans_source";
      save_trans_point_cloud(opt, opt.source, trans_source_prefix,
                             geo, C, globalT);
    }

    save_errors(source, beg_errors,  opt.out_prefix + "-beg_errors.csv",
                shift, geo, C, is_lola_rdr_format, mean_source_longitude);
    save_errors(trans_source, end_errors,  opt.out_prefix + "-end_errors.csv",
                shift, geo, C, is_lola_rdr_format, mean_source_longitude);

    if (opt.verbose) vw_out() << "Writing: " << opt.out_prefix
      + "-iterationInfo.csv" << std::endl;

    sw8.stop();
    if (opt.verbose) vw_out() << "Saving to disk took "
                              << sw8.elapsed_seconds() << " [s]" << endl;

  } ASP_STANDARD_CATCHES;

  return 0;
}
