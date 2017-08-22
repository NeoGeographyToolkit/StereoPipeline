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
#include <asp/Core/InterestPointMatching.h>
#include <liblas/liblas.hpp>

#include <limits>
#include <cstring>

#include <pointmatcher/PointMatcher.h>
#include <ceres/ceres.h>
#include <ceres/loss_function.h>

#include <asp/Tools/pc_align_utils.h>

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

const double BIG_NUMBER = 1e+300; // libpointmatcher does not like here the largest double

// Allows FileIO to correctly read/write these pixel types
namespace vw {
  typedef Vector<float64,6> Vector6;
  template<> struct PixelFormatID<Vector3>   { static const PixelFormatEnum value = VW_PIXEL_GENERIC_3_CHANNEL; };
  template<> struct PixelFormatID<Vector3f>  { static const PixelFormatEnum value = VW_PIXEL_GENERIC_3_CHANNEL; };
  template<> struct PixelFormatID<Vector4>   { static const PixelFormatEnum value = VW_PIXEL_GENERIC_4_CHANNEL; };
  template<> struct PixelFormatID<Vector6>   { static const PixelFormatEnum value = VW_PIXEL_GENERIC_6_CHANNEL; };
}

/// Options container for the pc_align tool
struct Options : public vw::cartography::GdalWriteOptions {
  // Input
  string reference, source, init_transform_file, alignment_method, config_file,
    datum, csv_format_str, csv_proj4_str, match_file;
  PointMatcher<RealT>::Matrix init_transform;
  int    num_iter,
         max_num_reference_points,
         max_num_source_points;
  double diff_translation_err,
         diff_rotation_err,
         max_disp,
         outlier_ratio,
         semi_major,
         semi_minor;
  bool   compute_translation_only,
         dont_use_dem_distances,
         save_trans_source,
         save_trans_ref,
         highest_accuracy,
         verbose;
  std::string initial_ned_translation;
  
  // Output
  string out_prefix;

  Options() : max_disp(-1.0), verbose(true){}

  /// Return true if the reference file is a DEM file and this option is not disabled
  bool use_dem_distances() const { return ( (get_file_type(this->reference) == "DEM") && !dont_use_dem_distances); }
};


void handle_arguments( int argc, char *argv[], Options& opt ) {
  po::options_description general_options("");
  general_options.add_options()
    ("initial-transform",        po::value(&opt.init_transform_file)->default_value(""),
                                 "The file containing the transform to be used as an initial guess. It can come from a previous run of the tool.")
    ("num-iterations",           po::value(&opt.num_iter)->default_value(1000),
                                 "Maximum number of iterations.")
    ("diff-rotation-error",      po::value(&opt.diff_rotation_err)->default_value(1e-8),
                                 "Change in rotation amount below which the algorithm will stop (if translation error is also below bound), in degrees.")
    ("diff-translation-error",   po::value(&opt.diff_translation_err)->default_value(1e-3),
                                 "Change in translation amount below which the algorithm will stop (if rotation error is also below bound), in meters.")
    ("max-displacement",         po::value(&opt.max_disp)->default_value(0.0),
                                 "Maximum expected displacement of source points as result of alignment, in meters (after the initial guess transform is applied to the source points). Used for removing gross outliers in the source point cloud.")
    ("outlier-ratio",            po::value(&opt.outlier_ratio)->default_value(0.75),
                                 "Fraction of source (movable) points considered inliers (after gross outliers further than max-displacement from reference points are removed).")
    ("max-num-reference-points", po::value(&opt.max_num_reference_points)->default_value(100000000),
                                 "Maximum number of (randomly picked) reference points to use.")
    ("max-num-source-points",    po::value(&opt.max_num_source_points)->default_value(100000),
                                 "Maximum number of (randomly picked) source points to use (after discarding gross outliers).")
    ("alignment-method",         po::value(&opt.alignment_method)->default_value("point-to-plane"),
                                 "The type of iterative closest point method to use. [point-to-plane, point-to-point, similarity-point-to-point, least-squares, similarity-least-squares]")
    ("highest-accuracy",         po::bool_switch(&opt.highest_accuracy)->default_value(false)->implicit_value(true),
                                 "Compute with highest accuracy for point-to-plane (can be much slower).")
    ("csv-format",               po::value(&opt.csv_format_str)->default_value(""), asp::csv_opt_caption().c_str())
    ("csv-proj4",                po::value(&opt.csv_proj4_str)->default_value(""),
                                 "The PROJ.4 string to use to interpret the entries in input CSV files.")
    ("datum",                    po::value(&opt.datum)->default_value(""),
                                 "Use this datum for CSV files instead of auto-detecting it. Options: WGS_1984, D_MOON (1,737,400 meters), D_MARS (3,396,190 meters), MOLA (3,396,000 meters), NAD83, WGS72, and NAD27. Also accepted: Earth (=WGS_1984), Mars (=D_MARS), Moon (=D_MOON).")
    ("semi-major-axis",          po::value(&opt.semi_major)->default_value(0),
                                 "Explicitly set the datum semi-major axis in meters.")
    ("semi-minor-axis",          po::value(&opt.semi_minor)->default_value(0),
                                 "Explicitly set the datum semi-minor axis in meters.")
    ("output-prefix,o",          po::value(&opt.out_prefix)->default_value("run/run"),
                                 "Specify the output prefix.")
    ("compute-translation-only", po::bool_switch(&opt.compute_translation_only)->default_value(false)->implicit_value(true),
                                 "Compute the transform from source to reference point cloud as a translation only (no rotation).")
    ("save-transformed-source-points", po::bool_switch(&opt.save_trans_source)->default_value(false)->implicit_value(true),
                                  "Apply the obtained transform to the source points so they match the reference points and save them.")
    ("save-inv-transformed-reference-points", po::bool_switch(&opt.save_trans_ref)->default_value(false)->implicit_value(true),
     "Apply the inverse of the obtained transform to the reference points so they match the source points and save them.")

    ("no-dem-distances",         po::bool_switch(&opt.dont_use_dem_distances)->default_value(false)->implicit_value(true),
                                 "For reference point clouds that are DEMs, don't take advantage of the fact that it is possible to interpolate into this DEM when finding the closest distance to it from a point in the source cloud and hence the error metrics.")
    ("initial-ned-translation", po::value(&opt.initial_ned_translation)->default_value(""),
                                 "Initialize the alignment transform based on a translation with this vector in the North-East-Down coordinate system. Specify it in quotes, separated by spaces or commas.")

    ("match-file", po::value(&opt.match_file)->default_value(""),
     "Compute a translation + rotation + scale transform from the source to the reference point cloud using manually selected point correspondences (obtained for example using stereo_gui).")
    ("config-file",              po::value(&opt.config_file)->default_value(""),
     "This is an advanced option. Read the alignment parameters from a configuration file, in the format expected by libpointmatcher, over-riding the command-line options.");

  //("verbose", po::bool_switch(&opt.verbose)->default_value(false)->implicit_value(true),
  // "Print debug information");

  general_options.add( vw::cartography::GdalWriteOptionsDescription(opt) );

  po::options_description positional("");
  positional.add_options()
    ("reference", po::value(&opt.reference), "The reference (fixed) point cloud/DEM.")
    ("source",    po::value(&opt.source),    "The source (movable) point cloud/DEM.");

  po::positional_options_description positional_desc;
  positional_desc.add("reference", 1);
  positional_desc.add("source",    1);

  string usage("--max-displacement arg [other options] <reference cloud> <source cloud> -o <output prefix>");
  bool allow_unregistered = false;
  std::vector<std::string> unregistered;
  po::variables_map vm =
    asp::check_command_line( argc, argv, opt, general_options, general_options,
                             positional, positional_desc, usage,
                             allow_unregistered, unregistered );

  if ( opt.reference.empty() || opt.source.empty() )
    vw_throw( ArgumentErr() << "Missing input files.\n" << usage << general_options );

  if ( opt.out_prefix.empty() )
    vw_throw( ArgumentErr() << "Missing output prefix.\n" << usage << general_options );

  // There is no need to use max-displacement with custom tie points.
  if (opt.match_file != "")
    opt.max_disp = -1.0;

  if ( opt.max_disp == 0.0 )
    vw_throw( ArgumentErr() << "The max-displacement option was not set. Use -1 if it is desired not to use it.\n"
                            << usage << general_options );

  if ( opt.num_iter < 0 )
    vw_throw( ArgumentErr() << "The number of iterations must be non-negative.\n"
                            << usage << general_options );

  if ( (opt.semi_major != 0 && opt.semi_minor == 0) ||
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
  vw::create_out_dir(opt.out_prefix);

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

    if (opt.init_transform(DIM, DIM) != 1) {
      vw_throw( ArgumentErr() << "The initial transform must have a 1 in the lower-right corner.\n");
    }
  }

  if (opt.alignment_method != "point-to-plane"            &&
      opt.alignment_method != "point-to-point"            &&
      opt.alignment_method != "similarity-point-to-point" &&
      opt.alignment_method != "least-squares"             &&
      opt.alignment_method != "similarity-least-squares"
      )
    vw_throw( ArgumentErr() << "Only the following alignment methods are supported: "
	      << "point-to-plane, point-to-point, similarity-point-to-point, "
	      << "least-squares, and similarity-least-squares.\n"
	      << usage << general_options );

  if ( (opt.alignment_method == "least-squares" ||
	opt.alignment_method == "similarity-least-squares")
       && get_file_type(opt.reference) != "DEM")
    vw_throw( ArgumentErr()
	      << "Least squares alignment can be used only when the "
	      << "reference cloud is a DEM.\n" );
}

/// Try to read the georef/datum info, need it to read CSV files.
void read_georef(Options& opt, asp::CsvConv& csv_conv, GeoReference& geo){

  // Use an initialized datum for the georef, so later we can check
  // if we manage to populate it.
  {
    Datum datum(UNSPECIFIED_DATUM, "User Specified Spheroid",
              "Reference Meridian", 1, 1, 0);
    geo.set_datum(datum);
  }

  bool is_good = false;

  // First, get the datum from the DEM if available.
  string dem_file = "";
  if ( get_file_type(opt.reference) == "DEM" )
    dem_file = opt.reference;
  else if ( get_file_type(opt.source) == "DEM" )
    dem_file = opt.source;
  if (dem_file != ""){
    GeoReference local_geo;
    bool have_georef = cartography::read_georeference(local_geo, dem_file);
    if (!have_georef)
      vw_throw(ArgumentErr() << "DEM: " << dem_file << " does not have a georeference.\n");
    geo = local_geo;
    vw_out() << "Detected datum from " << dem_file << ":\n" << geo.datum() << std::endl;
    is_good = true;
  }

  // Then, try to set it from the pc file if available.
  // Either one, or both or neither of the pc files may have a georef.
  string pc_file = "";
  if ( get_file_type(opt.reference) == "PC" ){
    GeoReference local_geo;
    if (cartography::read_georeference(local_geo, opt.reference)){
      pc_file = opt.reference;
      geo = local_geo;
      vw_out() << "Detected datum from " << pc_file << ":\n" << geo.datum() << std::endl;
      is_good = true;
    }
  }
  if ( get_file_type(opt.source) == "PC" ){
    GeoReference local_geo;
    if (cartography::read_georeference(local_geo, opt.source)){
      pc_file = opt.source;
      geo = local_geo;
      vw_out() << "Detected datum from " << pc_file << ":\n" << geo.datum() << std::endl;
      is_good = true;
    }
  }

  // Then, try to set it from the las file if available.
  // Either one, or both or neither of the las files may have a georef.
  string las_file = "";
  if ( get_file_type(opt.reference) == "LAS" ){
    GeoReference local_geo;
    if (asp::georef_from_las(opt.reference, local_geo)){
      las_file = opt.reference;
      geo = local_geo;
      vw_out() << "Detected datum from " << las_file << ":\n" << geo.datum() << std::endl;
      is_good = true;
    }
  }
  if ( get_file_type(opt.source) == "LAS" ){
    GeoReference local_geo;
    if (asp::georef_from_las(opt.source, local_geo)){
      las_file = opt.source;
      geo = local_geo;
      vw_out() << "Detected datum from " << las_file << ":\n" << geo.datum() << std::endl;
      is_good = true;
    }
  }

  // We should have read in the datum from an input file, but check to see if
  //  we should override it with input parameters.

  if (opt.datum != ""){
    // If the user set the datum, use it.
    Datum datum;
    datum.set_well_known_datum(opt.datum);
    geo.set_datum(datum);
    is_good = true;
  }else if (opt.semi_major > 0 && opt.semi_minor > 0){
    // Otherwise, if the user set the semi-axes, use that.
    Datum datum = Datum("User Specified Datum", "User Specified Spheroid",
                        "Reference Meridian",
                        opt.semi_major, opt.semi_minor, 0.0);
    geo.set_datum(datum);
    is_good = true;
  }

  // This must be the last as it has priority. Use user's csv_proj4 string,
  // to add info to the georef.
  if (csv_conv.parse_georef(geo)) {
    is_good = true;
  }

  if (is_good)
    vw_out() << "Will use datum (for CSV files): " << geo.datum() << std::endl;

  // A lot of care is needed below.
  if (!is_good  && (opt.csv_format_str == "" || csv_conv.get_format() != asp::CsvConv::XYZ) ){
    // There is no DEM/LAS to read the datum from, and the user either
    // did not specify the CSV format (then we set it to lat, lon,
    // height), or it is specified as containing lat, lon, rather than xyz.
    bool has_csv = ( get_file_type(opt.reference) == "CSV" ) ||
                   ( get_file_type(opt.source   ) == "CSV" );
    if (has_csv){
      // We are in trouble, will not be able to convert input lat, lon, to xyz.
      vw_throw( ArgumentErr() << "Cannot detect the datum. "
                              << "Please specify it via --csv-proj4 or --datum or "
                              << "--semi-major-axis and --semi-minor-axis.\n" );
    }else{
      // The inputs have no georef. Will have to write xyz.
      vw_out() << "No datum specified. Will write output CSV files "
               << "in the x,y,z format." << std::endl;
      opt.csv_format_str = "1:x 2:y 3:z";
      csv_conv.parse_csv_format(opt.csv_format_str, opt.csv_proj4_str);
      is_good = true;
    }
  }

  if (!is_good)
    vw::vw_throw( vw::InputErr() << "Datum is required and could not be set.\n");

  return;
}

/// Compute output statistics for pc_align
void calc_stats(string label, PointMatcher<RealT>::Matrix const& dists){

  VW_ASSERT(dists.rows() == 1,
            LogicErr() << "Expecting only one row.");

  vector<double> errs(dists.cols()*dists.rows());
  int count = 0;
  for (int col = 0; col < dists.cols(); col++){
    //for (int row = 0; row < dists.rows(); row++){
    errs[count] = dists(0, col);
    count++;
    //}
  }
  sort(errs.begin(), errs.end());

  int len = errs.size();
  vw_out() << "Number of errors: " << len << endl;
  if (len == 0)
    return;

  double p16 = errs[std::min(len-1, (int)round(len*0.16))];
  double p50 = errs[std::min(len-1, (int)round(len*0.50))];
  double p84 = errs[std::min(len-1, (int)round(len*0.84))];
  vw_out() << label << ": error percentile of smallest errors (meters):"
           << " 16%: " << p16 << ", 50%: " << p50 << ", 84%: " << p84 << endl;

  double a25 = calc_mean(errs,   len/4), a50  = calc_mean(errs, len/2);
  double a75 = calc_mean(errs, 3*len/4), a100 = calc_mean(errs, len);
  vw_out() << label << ": mean of smallest errors (meters):"
           << " 25%: "  << a25 << ", 50%: "  << a50
           << ", 75%: " << a75 << ", 100%: " << a100 << endl;
}


/// Write the output points in lon_lat_height format
void dump_llh(string const& file, Datum const& datum,
              DP const & data, Vector3 const& shift){

  vw_out() << "Writing: "   << data.features.cols()
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

/// Save the transform and its inverse.
void save_transforms(Options const& opt,
                     PointMatcher<RealT>::Matrix const& T){

  string transFile = opt.out_prefix + "-transform.txt";
  vw_out() << "Writing: " << transFile << endl;
  ofstream tf(transFile.c_str());
  tf.precision(16);
  tf << T << endl;
  tf.close();

  string iTransFile = opt.out_prefix + "-inverse-transform.txt";
  PointMatcher<RealT>::Matrix invT = T.inverse();
  vw_out() << "Writing: " << iTransFile << endl;
  ofstream itf(iTransFile.c_str());
  itf.precision(16);
  itf << invT << endl;
  itf.close();
}


/// Extracts the full GCC coordinate of a single point from a LibPointMatcher point cloud.
/// - The shift converts from the normalized coordinate to the actual GCC coordinate.
/// - No bounds checking is performed on the point index.
Vector3 get_cloud_gcc_coord(DP const& point_cloud, vw::Vector3 const& shift, int index) {
  Vector3 gcc_coord;
  for (int row = 0; row < DIM; ++row)
     gcc_coord[row] = point_cloud.features(row, index) + shift[row];
  return gcc_coord;
}


/// Save the lon, lat, radius/height, and error. Use a format
/// consistent with the input CSV format.
void save_errors(DP const& point_cloud,
                 PointMatcher<RealT>::Matrix const& errors,
                 string const& output_file,
                 Vector3 const& shift,
                 GeoReference const& geo,
                 asp::CsvConv const& csv_conv,
                 bool is_lola_rdr_format,
                 double mean_longitude){

  vw_out() << "Writing: " << output_file << std::endl;

  VW_ASSERT(point_cloud.features.cols() == errors.cols(),
            ArgumentErr() << "Expecting as many errors as source points.");

  ofstream outfile( output_file.c_str() );
  outfile.precision(16);

  // Write the header line
  if (csv_conv.is_configured()){
    outfile << "# " << csv_conv.write_header_string(",") << "error (meters)" << endl;
  }else{
    if (is_lola_rdr_format)
      outfile << "# longitude,latitude,radius (km),error (meters)" << endl;
    else
      outfile << "# latitude,longitude,height above datum (meters),error (meters)" << endl;
  }

  // Save the datum, may be useful to know what it was
  if (geo.datum().name() != UNSPECIFIED_DATUM){
    outfile << "# " << geo.datum() << std::endl;
    outfile << "# Projection: " << geo.overall_proj4_str() << std::endl;
  }

  int numPts = point_cloud.features.cols();
  for(int col = 0; col < numPts; col++){
    Vector3 P = get_cloud_gcc_coord(point_cloud, shift, col);

    if (csv_conv.is_configured()){
      Vector3 csv = csv_conv.cartesian_to_csv(P, geo, mean_longitude);
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



void debug_save_point_cloud(DP const& point_cloud, GeoReference const& geo,
                            Vector3 const& shift,
                            string const& output_file){

  int numPts = point_cloud.features.cols();

  vw_out() << "Writing: " << output_file << endl;
  ofstream outfile( output_file.c_str() );
  outfile.precision(16);

  for(int col = 0; col < numPts; col++){
    Vector3 P = get_cloud_gcc_coord(point_cloud, shift, col);

    Vector3 llh = geo.datum().cartesian_to_geodetic(P); // lon-lat-height
    outfile << llh[1] << ',' << llh[0] << ',' << llh[2] << endl;
  }
}

/// Like PM::ICP::filterGrossOutliersAndCalcErrors, except comparing to a DEM instead.
/// - The point cloud is in GCC coordinates with point_cloud_shift subtracted from each point.
/// - The output is put in the "errors" vector for each point.
/// - If there is a problem computing the point error, a very large number is used as a flag.
void calcErrorsWithDem(DP          const& point_cloud,
                       vw::Vector3 const& point_cloud_shift,
                       vw::cartography::GeoReference        const& georef,
                       vw::ImageViewRef< PixelMask<float> > const& dem,
                       std::vector<double> &errors) {

  // Initialize output error storage
  const int num_pts = point_cloud.features.cols();
  errors.resize(num_pts);

  // Loop through every point in the point cloud
  double dem_height_here;
  for(int i=0; i<num_pts; ++i){
    // Extract and un-shift the point to get the real GCC coordinate
    Vector3 gcc_coord = get_cloud_gcc_coord(point_cloud, point_cloud_shift, i);

    // Convert from GDC to GCC
    Vector3 llh = georef.datum().cartesian_to_geodetic(gcc_coord); // lon-lat-height

    // Interpolate the point at this location
    if (!interp_dem_height(dem, georef, llh, dem_height_here)) {
      // If we did not intersect the DEM, record a flag error value here.
      errors[i] = BIG_NUMBER;
    }
    else { // Success, the error is the absolute height difference
      errors[i] = std::abs(llh[2] - dem_height_here);
    }

  } // End loop through all points

}

template<class F>
void extract_rotation_translation(F       * transform, 
				  Quat    & rotation,
				  Vector3 & translation){

  
  Vector3 axis_angle;
  for (int i = 0; i < 3; i++){
    translation[i] = transform[i];
    axis_angle[i]  = transform[i+3];
  }
  rotation = axis_angle_to_quaternion(axis_angle);
}

// Discrepancy between a 3D point with the rotation to be solved
// applied to it, and its projection straight down onto the DEM. Used
// with the least squares method of finding the best transform between
// clouds.
struct PointToDemError {
  PointToDemError(Vector3 const& point,
		 ImageViewRef< PixelMask<float> > const& dem,
		 cartography::GeoReference const& geo):
    m_point(point), m_dem(dem), m_geo(geo){}

  template <typename F>
  bool operator()(const F* const transform, const F* const scale, F* residuals) const {

    // Default residuals are zero, if we can't project into the DEM
    residuals[0] = F(0.0);

    // Extract the translation, and rotation
    Vector3 translation;
    Quat rotation;
    extract_rotation_translation(transform, rotation, translation);

    Vector3 trans_point = scale[0]*rotation.rotate(m_point) + translation;
    
    // Convert from GDC to GCC
    Vector3 llh = m_geo.datum().cartesian_to_geodetic(trans_point); // lon-lat-height

    // Interpolate the point at this location
    double dem_height_here;
    if (!interp_dem_height(m_dem, m_geo, llh, dem_height_here)) {
      // If we did not intersect the DEM, record a flag error value here.
      residuals[0] = F(0.0);
      return true;
    }
    
    residuals[0] = llh[2] - dem_height_here;
    return true;
  }
  
  // Factory to hide the construction of the CostFunction object from
  // the client code.
  static ceres::CostFunction* Create(Vector3 const& point,
				     ImageViewRef< PixelMask<float> > const& dem,
				     vw::cartography::GeoReference const& geo){
    return (new ceres::NumericDiffCostFunction<PointToDemError,
	    ceres::CENTRAL, 1, 6, 1>
	    (new PointToDemError(point, dem, geo)));
  }

  Vector3                                  m_point;
  ImageViewRef< PixelMask<float> > const & m_dem;    // alias
  cartography::GeoReference        const & m_geo;    // alias
};

/// Compute alignment using least squares
PointMatcher<RealT>::Matrix
least_squares_alignment(DP & source_point_cloud, // Should not be modified
			vw::Vector3 const& point_cloud_shift,
			vw::cartography::GeoReference        const& dem_georef,
			vw::ImageViewRef< PixelMask<float> > const& dem_ref,
			Options const& opt) {

  ceres::Problem problem;

  // The final transform as a axis angle and translation pair
  std::vector<double> transform(6, 0.0);

  double scale = 1.0;
  
  // Add a residual block for every source point
  const int num_pts = source_point_cloud.features.cols();

  // Loop through every point in the point cloud
  for(int i = 0; i < num_pts; ++i){
    
    // Extract and un-shift the point to get the real GCC coordinate
    Vector3 gcc_coord = get_cloud_gcc_coord(source_point_cloud, point_cloud_shift, i);

    ceres::CostFunction* cost_function =
      PointToDemError::Create(gcc_coord, dem_ref, dem_georef);
    ceres::LossFunction* loss_function = new ceres::CauchyLoss(0.5); // NULL;
    problem.AddResidualBlock(cost_function, loss_function, &transform[0], &scale);
    
  } // End loop through all points

  if (opt.alignment_method == "least-squares") {
    // Only solve for rotation and translation
    problem.SetParameterBlockConstant(&scale);
  }
  
  ceres::Solver::Options options;
  options.gradient_tolerance = 1e-16;
  options.function_tolerance = 1e-16;
  options.max_num_iterations = opt.num_iter;
  options.minimizer_progress_to_stdout = 1;
  options.num_threads = opt.num_threads;
  options.linear_solver_type = ceres::SPARSE_SCHUR;

  // Solve the problem
  ceres::Solver::Summary summary;
  ceres::Solve(options, &problem, &summary);

  vw_out() << summary.FullReport() << "\n" << std::endl;

  Quat rotation;
  Vector3 translation;
  extract_rotation_translation(&transform[0], rotation, translation);
  Matrix<double,3,3> rot_matrix = rotation.rotation_matrix();
  
  PointMatcher<RealT>::Matrix T = PointMatcher<RealT>::Matrix::Identity(DIM + 1, DIM + 1);
  for (int row = 0; row < DIM; row++){
    for (int col = 0; col < DIM; col++){
      T(row, col) = scale*rot_matrix(col, row);
    }
  }

  for (int row = 0; row < DIM; row++)
    T(row, DIM) = translation[row];

  // This transform is in the world coordinate system (as that's the natural
  // coord system for the DEM). Transform it to the internal shifted coordinate
  // system.
  T = apply_shift(T, point_cloud_shift);

  return T;
}

/// Filters out all points from point_cloud with an error entry higher than cutoff
void filterPointsByError(DP & point_cloud, PointMatcher<RealT>::Matrix &errors, double cutoff) {

  DP input_copy = point_cloud; // Make a copy of the input DP object

  // Init LPM data structure
  const int input_point_count = point_cloud.features.cols();
  if (errors.cols() != input_point_count)
    vw_throw( LogicErr() << "Error: error size does not match point count size!\n");
  point_cloud.features.conservativeResize(DIM+1, input_point_count);
  point_cloud.featureLabels = form_labels<double>(DIM);

  // Loop through all the input points and copy them to the output if they pass the test
  int points_count = 0;
  for (int col = 0; col < input_point_count; ++col) {

    if (errors(0,col) > cutoff) {
      //vw_out() << "Throwing out point " << col << " for having error " << errors(0,col) << "\n";
      continue; // Error too high, don't add this point
    }

    // Copy this point to the output LPM structure
    for (int row = 0; row < DIM; row++)
      point_cloud.features(row, points_count) = input_copy.features(row, col);
    point_cloud.features(DIM, points_count) = 1; // Extend to be a homogenous coordinate
    ++points_count; // Update output point count

  } // End loop through points

  // Finalize the LPM data structure
  point_cloud.features.conservativeResize(Eigen::NoChange, points_count);

}

// Note: The LPM matrix type used to store errors only ever has a single row.

/// Updates an LPM error matrix to use the DEM-based error for each point if it is lower.
void update_best_error(std::vector<double>         const& dem_errors,
                       PointMatcher<RealT>::Matrix      & lpm_errors) {
  int num_points = lpm_errors.cols();
  if (dem_errors.size() != static_cast<size_t>(num_points))
    vw_throw( LogicErr() << "Error: error size does not match point count size!\n");
  //vw_out() << "Updating error...\n";

  // Loop through points
  for(int col = 0; col < num_points; col++){
    // Use the DEM error if it is less
    if (dem_errors[col] < lpm_errors(0,col)) {
      //vw_out() << "DEM error = " << dem_errors[col] << ", LPM error = " << lpm_errors(0,col) << std::endl;
      lpm_errors(0, col) = dem_errors[col];
    }
  }

}


/// Compute the distance from source_point_cloud to the reference points.
double compute_registration_error(DP          const& ref_point_cloud,
                                  DP               & source_point_cloud, // Should not be modified
                                  PM::ICP          & pm_icp_object, // Must already be initialized
                                  vw::Vector3 const& shift,
                                  vw::cartography::GeoReference        const& dem_georef,
                                  vw::ImageViewRef< PixelMask<float> > const& dem_ref,
                                  Options const& opt,
                                  PointMatcher<RealT>::Matrix &error_matrix) {
  Stopwatch sw;
  sw.start();

  // Always start by computing the error using LPM
  // Use a big number to make sure no points are filtered!
  pm_icp_object.filterGrossOutliersAndCalcErrors(ref_point_cloud, BIG_NUMBER,
                                                 source_point_cloud, error_matrix);

  if (opt.use_dem_distances()) {
    // Compute the distance from each point to the DEM
    std::vector<double> dem_errors;
    calcErrorsWithDem(source_point_cloud, shift, dem_georef, dem_ref, dem_errors);

    // For each point use the lower of the two calculated errors.
    update_best_error(dem_errors, error_matrix);

    // We compute the error in two passes for two reasons:
    // 1 - Get the most accurate distance for each point in all cases.
    // 2 - Help fill in distances where the DEM has holes.
  }

  sw.stop();
  return sw.elapsed_seconds();
}

/// Points in source_point_cloud farther than opt.max_disp from the reference cloud are deleted.
void filter_source_cloud(DP          const& ref_point_cloud,
                         DP               & source_point_cloud,
                         PM::ICP          & pm_icp_object, // Must already be initialized
                         vw::Vector3 const& shift,
                         vw::cartography::GeoReference        const& dem_georef,
                         vw::ImageViewRef< PixelMask<float> > const& dem_ref,
                         Options const& opt) {

  // Filter gross outliers
  Stopwatch sw;
  sw.start();

  if (opt.verbose)
    vw_out() << "Filtering gross outliers" << endl;

  PointMatcher<RealT>::Matrix error_matrix;
  try {
    if (opt.use_dem_distances()) {
      // Compute the registration error using the best available means
      compute_registration_error(ref_point_cloud, source_point_cloud, pm_icp_object, shift,
                                 dem_georef, dem_ref, opt, error_matrix);

      filterPointsByError(source_point_cloud, error_matrix, opt.max_disp);
    } else { // LPM only method
        // Points in source_point_cloud further than opt.max_disp from ref_point_cloud are deleted!
        pm_icp_object.filterGrossOutliersAndCalcErrors(ref_point_cloud, opt.max_disp,
                                                       source_point_cloud, error_matrix);
    }
  }catch(const PointMatcher<RealT>::ConvergenceError & e){
    vw_throw( ArgumentErr() << "Error: No points left in source cloud after filtering.\n");
  }


  sw.stop();
  if (opt.verbose)
    vw_out() << "Filtering gross outliers took " << sw.elapsed_seconds() << " [s]" << endl;
}


Eigen::Matrix3d vw_matrix3_to_eigen(vw::Matrix3x3 const& vw_matrix) {
  Eigen::Matrix3d out;
  out(0,0) = vw_matrix(0,0);
  out(0,1) = vw_matrix(0,1);
  out(0,2) = vw_matrix(0,2);
  out(1,0) = vw_matrix(1,0);
  out(1,1) = vw_matrix(1,1);
  out(1,2) = vw_matrix(1,2);
  out(2,0) = vw_matrix(2,0);
  out(2,1) = vw_matrix(2,1);
  out(2,2) = vw_matrix(2,2);
  return out;
}

Eigen::Vector3d vw_vector3_to_eigen(vw::Vector3 const& vw_vector) {
  return Eigen::Vector3d(vw_vector[0], vw_vector[1], vw_vector[2]);
}

// Need this to placate libpointmatcher.
std::string alignment_method_fallback(std::string const& alignment_method){
  if (alignment_method == "least-squares" || alignment_method == "similarity-least-squares") 
    return "point-to-plane";
  return alignment_method;
}
  

// Compute a manual transform based on tie points (interest point matches).
void manual_transform(Options & opt){

  if (get_file_type(opt.reference) != "DEM" ||
      get_file_type(opt.source) != "DEM" )
    vw_throw( ArgumentErr() << "The alignment transform computation based on manually chosen point matches only works for DEMs. Use point2dem to first create DEMs from the input point clouds.\n" );

  vector<vw::ip::InterestPoint> ref_ip, source_ip;
  vw_out() << "Reading match file: " << opt.match_file << "\n";
  vw::ip::read_binary_match_file(opt.match_file, ref_ip, source_ip);

  DiskImageView<float> ref(opt.reference);
  vw::cartography::GeoReference ref_geo;
  bool has_ref_geo = vw::cartography::read_georeference(ref_geo, opt.reference);
  double ref_nodata = -std::numeric_limits<double>::max();
  vw::read_nodata_val(opt.reference, ref_nodata);

  DiskImageView<float> source(opt.source);
  vw::cartography::GeoReference source_geo;
  bool has_source_geo = vw::cartography::read_georeference(source_geo, opt.source);
  double source_nodata = -std::numeric_limits<double>::max();
  vw::read_nodata_val(opt.source, source_nodata);

  if (!has_ref_geo || !has_source_geo)
    vw_throw( ArgumentErr() << "One of the inputs is not a valid DEM.\n" );

  if (DIM != 3)
    vw_throw( ArgumentErr() << "Expecting DIM = 3.\n" );

  // Go from pixels to 3D points
  int num_matches = ref_ip.size();
  vw::Matrix<double> points_ref(DIM, num_matches), points_src(DIM, num_matches);
  typedef vw::math::MatrixCol<vw::Matrix<double> > ColView;
  int count = 0;
  for (int match_id = 0; match_id < num_matches; match_id++) {
    int ref_x = ref_ip[match_id].x;
    int ref_y = ref_ip[match_id].y;
    if (ref_x < 0 || ref_x >= ref.cols()) continue;
    if (ref_y < 0 || ref_y >= ref.rows()) continue;
    double ref_h = ref(ref_x, ref_y);
    // Check for no-data and NaN pixels
    if (ref_h <= ref_nodata || ref_h != ref_h) continue;

    int source_x = source_ip[match_id].x;
    int source_y = source_ip[match_id].y;
    if (source_x < 0 || source_x >= source.cols()) continue;
    if (source_y < 0 || source_y >= source.rows()) continue;
    double source_h = source(source_x, source_y);
    // Check for no-data and NaN pixels
    if (source_h <= source_nodata || source_h != source_h) continue;

    Vector2 ref_ll  = ref_geo.pixel_to_lonlat(Vector2(ref_x, ref_y));
    Vector3 ref_xyz = ref_geo.datum()
      .geodetic_to_cartesian(Vector3(ref_ll[0], ref_ll[1], ref_h));

    Vector2 source_ll = source_geo.pixel_to_lonlat(Vector2(source_x, source_y));
    Vector3 source_xyz = source_geo.datum()
      .geodetic_to_cartesian(Vector3(source_ll[0], source_ll[1], source_h));

    // Store in matrices
    ColView col_ref(points_ref, count); 
    ColView col_src(points_src, count);
    col_ref = ref_xyz;
    col_src = source_xyz;
    
    count++;
  }

  if (count < 3)
    vw_throw( ArgumentErr() << "Not enough valid matches were found.\n");

  // Resize the matrix to keep only the valid points. Find the transform.
  points_src.set_size(DIM, count, true);
  points_ref.set_size(DIM, count, true);
  vw::Matrix3x3 rotation;
  vw::Vector3   translation;
  double        scale;
  asp::find_3D_affine_transform(points_src, points_ref,
                                rotation, translation, scale);

  // Convert to pc_align transform format.
  PointMatcher<RealT>::Matrix globalT = Eigen::MatrixXd::Identity(DIM+1, DIM+1);
  globalT.block(0, 0, DIM, DIM) = vw_matrix3_to_eigen(rotation*scale);
  globalT.block(0, DIM, DIM, 1) = vw_vector3_to_eigen(translation);

  vw_out() << "Computed manual transform from source to reference:\n" << globalT << std::endl;

  // Save the transform to disk
  save_transforms(opt, globalT);

  vw_out() << "The transform file can be passed back to "
	   << "pc_align as an initial guess via --initial-transform. "
	   << "To have pc_align not further refine this transform, invoke it with 0 iterations.\n";
}

void apply_transform_to_cloud(PointMatcher<RealT>::Matrix const& T, DP & point_cloud){
  for (int col = 0; col < point_cloud.features.cols(); col++) {
    point_cloud.features.col(col) = T*point_cloud.features.col(col);
  }
}


// Convert a north-east-down vector at a given location to a vector in reference
// to the center of the Earth and create a translation matrix from that vector. 
PointMatcher<RealT>::Matrix ned_to_caresian_transform(vw::cartography::Datum const& datum,
                                                      std::string& ned_str, 
                                                      vw::Vector3 const & location){

  vw::string_replace(ned_str, ",", " "); // replace any commas
  vw::Vector3 ned = vw::str_to_vec<vw::Vector3>(ned_str);

  vw::Vector3 loc_llh = datum.cartesian_to_geodetic(location);
  vw::Matrix3x3 M = datum.lonlat_to_ned_matrix(subvector(loc_llh, 0, 2));
  vw::Vector3 xyz_shift = inverse(M)*ned;
  
  PointMatcher<RealT>::Matrix T = PointMatcher<RealT>::Matrix::Identity(DIM + 1, DIM + 1);

  // Append the transform
  for (int row = 0; row < DIM; row++)
    T(row, DIM) = xyz_shift[row];

  return T;
}

int main( int argc, char *argv[] ) {

  // Mandatory line for Eigen
  Eigen::initParallel();

  Options opt;
  try {
    handle_arguments( argc, argv, opt );

// TODO: Enable on OSX when clang supports OpenMP!
#if (defined(ASP_OSX_BUILD) && ASP_OSX_BUILD==1)
#else
    // Set the number of threads for OpenMP
    omp_set_num_threads(opt.num_threads);
#endif

    // Parse the csv format string and csv projection string
    asp::CsvConv csv_conv;
    csv_conv.parse_csv_format(opt.csv_format_str, opt.csv_proj4_str);

    // Try to read the georeference/datum info
    GeoReference geo;
    read_georef(opt, csv_conv, geo);

    // Create a manual transform based on user-picked correspondences among clouds
    if (opt.match_file != "") {
      manual_transform(opt);
      return 0;
    }

    // We will use ref_box to bound the source points, and vice-versa.
    // Decide how many samples to pick to estimate these boxes.
    Stopwatch sw0;
    sw0.start();
    int num_sample_pts = std::max(4000000,
                                  std::max(opt.max_num_source_points,
                                           opt.max_num_reference_points)/4);
    num_sample_pts = std::min(9000000, num_sample_pts); // avoid being slow
    
    // Compute GDC bounding box of the source and reference clouds
    vw_out() << "Computing the intersection of the bounding boxes "
             << "of the reference and source points." << endl;
    BBox2 ref_box, source_box;
    ref_box    = calc_extended_lonlat_bbox(geo, num_sample_pts, csv_conv,
                                           opt.reference, opt.max_disp);
    source_box = calc_extended_lonlat_bbox(geo, num_sample_pts, csv_conv,
                                           opt.source,    opt.max_disp);

    // When boxes are huge, it is hard to do the optimization of intersecting
    // them, as they may differ not by 0 or 360, but by 180. Better do nothing
    // in that case. The solution may degrade a bit, as we may load points
    // not in the intersection of the boxes, but at least it won't be wrong.
    // In this case, there is a chance the boxes were computed wrong anyway.
    if (ref_box.width() > 180.0 || source_box.width() > 180.0) {
      vw_out() << "Warning: Your input point clouds are spread over more than half the planet. "
               << "It is suggested that they be cropped, to get more accurate results.\n";
      ref_box = BBox2();
      source_box = BBox2();
    }
    
    vw_out() << "Reference box: " << ref_box << std::endl;
    vw_out() << "Source box:    " << source_box << std::endl;

    // If ref points are offset by 360 degrees in longitude in respect
    // to source points, adjust the ref box to be aligned with the
    // source points, and vice versa.  Note that we will use the ref
    // box to bound the source points, and vice-versa.
    double lon_offset = 0.0;
    if (!ref_box.empty() && !source_box.empty()){
      // Compute the longitude offset
      double source_mean_lon = (source_box.min().x() + source_box.max().x())/2.0;
      double ref_mean_lon    = (ref_box.min().x()    + ref_box.max().x()   )/2.0;
      lon_offset = source_mean_lon - ref_mean_lon;
      lon_offset = 360.0*round(lon_offset/360.0);
      // Apply to both bounding boxes
      ref_box    += Vector2(lon_offset, 0);
      // Intersect them, as pc_align will operate on their common area
      ref_box.crop(source_box);
      source_box.crop(ref_box); // common area
      source_box -= Vector2(lon_offset, 0);

      // Extra adjustments. These are needed since pixel_to_lonlat and
      // cartesian_to_geodetic can disagree by 360 degress. Adjust ref
      // to source and vice-versa.
      adjust_lonlat_bbox(opt.reference, source_box);
      adjust_lonlat_bbox(opt.source, ref_box);
    }
    sw0.stop();
    vw_out() << "Intersection:  " << ref_box << std::endl;
    vw_out() << "Intersection of bounding boxes took " << sw0.elapsed_seconds() << " [s]" << endl;

    // Load the point clouds. We will shift both point clouds by the
    // centroid of the first one to bring them closer to origin.

    // Load the subsampled reference point cloud.
    Vector3 shift;
    bool   calc_shift = true; // Shift points so the first point is (0,0,0)
    bool   is_lola_rdr_format = false;   // may get overwritten
    double mean_ref_longitude    = 0.0;  // may get overwritten
    double mean_source_longitude = 0.0;  // may get overwritten
    Stopwatch sw1;
    sw1.start();
    DP ref_point_cloud;
    load_file<RealT>(opt.reference, opt.max_num_reference_points,
                     source_box, // source box is used to bound reference
                     calc_shift, shift, geo, csv_conv, is_lola_rdr_format,
                     mean_ref_longitude, opt.verbose, ref_point_cloud);
    sw1.stop();
    if (opt.verbose)
      vw_out() << "Loading the reference point cloud took "
               << sw1.elapsed_seconds() << " [s]" << endl;
    //ref_point_cloud.save(outputBaseFile + "_ref.vtk");

    // Load the subsampled source point cloud. If the user wants
    // to filter gross outliers in the source points based on
    // max_disp, load a lot more points than asked, filter based on
    // max_disp, then resample to the number desired by the user.
    int num_source_pts = opt.max_num_source_points;
    if (opt.max_disp > 0.0)
      num_source_pts = max(num_source_pts, 50000000);
    calc_shift = false; // Use the same shift used for the reference point cloud
    Stopwatch sw2;
    sw2.start();
    DP source_point_cloud;
    load_file<RealT>(opt.source, num_source_pts,
                     ref_box, // ref box is used to bound source
                     calc_shift, shift, geo, csv_conv, is_lola_rdr_format,
                     mean_source_longitude, opt.verbose, source_point_cloud);
    sw2.stop();
    if (opt.verbose)
      vw_out() << "Loading the source point cloud took "
               << sw2.elapsed_seconds() << " [s]" << endl;

    // So far we shifted by first point in reference point cloud to reduce
    // the magnitude of all loaded points. Now that we have loaded all
    // points, shift one more time, to place the centroid of the
    // reference at the origin.
    // Note: If this code is ever converting to using floats,
    // the operation below needs to be re-implemented to be accurate.
    int numRefPts = ref_point_cloud.features.cols();
    Eigen::VectorXd meanRef = ref_point_cloud.features.rowwise().sum() / numRefPts;
    ref_point_cloud.features.topRows(DIM).colwise()    -= meanRef.head(DIM);
    source_point_cloud.features.topRows(DIM).colwise() -= meanRef.head(DIM);
    for (int row = 0; row < DIM; row++)
      shift[row] += meanRef(row); // Update the shift variable as well as the points
    if (opt.verbose)
      vw_out() << "Data shifted internally by subtracting: " << shift << std::endl;


    // See if to apply an initial north-east-down translation
    if (opt.initial_ned_translation != "") {
      if (opt.init_transform_file != "")
        vw_throw( ArgumentErr()
                  << "Cannot specify an initial transform both from file and as a NED vector.\n");

      opt.init_transform = ned_to_caresian_transform(geo.datum(),
                                                     opt.initial_ned_translation, 
                                                     shift);
    }
    
    // The point clouds are shifted, so shift the initial transform as well.
    PointMatcher<RealT>::Matrix initT = apply_shift(opt.init_transform, shift);

    // If the reference point cloud came from a DEM, also load the data in DEM format.
    cartography::GeoReference dem_georef;
    vw::ImageViewRef< PixelMask<float> > reference_dem_ref;
    if (opt.use_dem_distances()) {
      vw_out() << "Loading reference as DEM." << endl;
      // Load the dem, then wrap it inside an ImageViewRef object.
      // - This is done because the actual DEM type cannot be created without being initialized.
      InterpolationReadyDem reference_dem(load_interpolation_ready_dem(opt.reference, dem_georef));
      reference_dem_ref.reset(reference_dem);
    }

    // Now all of the input data is loaded.

    // Filter the reference and initialize the reference tree
    double elapsed_time;
    PM::ICP icp; // LibpointMatcher object

    Stopwatch sw3;
    if (opt.verbose)
      vw_out() << "Building the reference cloud tree." << endl;
    sw3.start();
    icp.initRefTree(ref_point_cloud, alignment_method_fallback(opt.alignment_method),
		    opt.highest_accuracy, false /*opt.verbose*/);
    sw3.stop();
    if (opt.verbose)
      vw_out() << "Reference point cloud processing took " << sw3.elapsed_seconds() << " [s]" << endl;

    // Apply the initial guess transform to the source point cloud.
    apply_transform_to_cloud(initT, source_point_cloud);
    
    PointMatcher<RealT>::Matrix beg_errors;
    if (opt.max_disp > 0.0){
      // Filter gross outliers
      filter_source_cloud(ref_point_cloud, source_point_cloud, icp,
                          shift, dem_georef, reference_dem_ref, opt);
    }

    random_pc_subsample<RealT>(opt.max_num_source_points, source_point_cloud);
    vw_out() << "Reducing number of source points to "
             << source_point_cloud.features.cols() << endl;

    //dump_llh("ref.csv", datum, ref_point_cloud,    shift);
    //dump_llh("src.csv", datum, source, shift);

    elapsed_time = compute_registration_error(ref_point_cloud, source_point_cloud, icp,
                                              shift, dem_georef, reference_dem_ref,
					      opt, beg_errors);
    calc_stats("Input", beg_errors);
    if (opt.verbose)
      vw_out() << "Initial error computation took " << elapsed_time << " [s]" << endl;


    // Compute the transformation to align the source to reference.
    Stopwatch sw4;
    sw4.start();
    PointMatcher<RealT>::Matrix Id = PointMatcher<RealT>::Matrix::Identity(DIM + 1, DIM + 1);
    if (opt.config_file == ""){
      // Read the options from the command line
      icp.setParams(opt.out_prefix, opt.num_iter, opt.outlier_ratio,
                    (2.0*M_PI/360.0)*opt.diff_rotation_err, // convert to radians
                    opt.diff_translation_err, alignment_method_fallback(opt.alignment_method),
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
      if (opt.alignment_method != "least-squares" &&
	  opt.alignment_method != "similarity-least-squares") {
	T = icp(source_point_cloud, ref_point_cloud, Id,
		opt.compute_translation_only);
	vw_out() << "Match ratio: "
		 << icp.errorMinimizer->getWeightedPointUsedRatio() << endl;
      }else{
	T = least_squares_alignment(source_point_cloud, shift,
				    dem_georef, reference_dem_ref, opt);
      }
      
    }
    sw4.stop();
    if (opt.verbose)
      vw_out() << "Alignment took " << sw4.elapsed_seconds() << " [s]" << endl;

    // Transform the source to make it close to reference.
    DP trans_source_point_cloud(source_point_cloud);
    apply_transform_to_cloud(T, trans_source_point_cloud);

    // Calculate by how much points move as result of T
    calc_max_displacment(source_point_cloud, trans_source_point_cloud);
    Vector3 source_ctr_vec, source_ctr_llh;
    Vector3 trans_xyz, trans_ned, trans_llh;
    calc_translation_vec(source_point_cloud, trans_source_point_cloud, shift, geo.datum(),
                         source_ctr_vec, source_ctr_llh,
                         trans_xyz, trans_ned, trans_llh);

    // For each point, compute the distance to the nearest reference point.
    PointMatcher<RealT>::Matrix end_errors;
    elapsed_time = compute_registration_error(ref_point_cloud, trans_source_point_cloud, icp,
                                              shift, dem_georef, reference_dem_ref, opt,
					      end_errors);
    calc_stats("Output", end_errors);
    if (opt.verbose)
      vw_out() << "Final error computation took " << elapsed_time << " [s]" << endl;

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
    if (opt.max_disp > 0 && opt.max_disp < norm_2(trans_xyz)) {
      vw_out() << "Warning: The input --max-displacement value is smaller than the "
               << "final observed displacement. It may be advised to increase the former "
               << "and rerun the tool.\n";
    }

    // Swap lat and lon, as we want to print lat first
    std::swap(trans_llh[0], trans_llh[1]);
    vw_out() << "Translation vector (lat,lon,z): " << trans_llh << std::endl;
    vw_out() << std::endl;

    Matrix3x3 rot;
    for (int r = 0; r < DIM; r++)
      for (int c = 0; c < DIM; c++)
        rot(r, c) = globalT(r, c);

    if (opt.alignment_method == "similarity-point-to-point" ||
	opt.alignment_method == "similarity-least-squares"){
      double scale = pow(det(rot), 1.0/3.0);
      for (int r = 0; r < DIM; r++)
        for (int c = 0; c < DIM; c++)
          rot(r, c) /= scale;
      vw_out() << "Scale - 1 = " << (scale-1.0) << std::endl;
    }
    
    Vector3 euler_angles = math::rotation_matrix_to_euler_xyz(rot) * 180/M_PI;
    Vector3 axis_angles = math::matrix_to_axis_angle(rot) * 180/M_PI;
    vw_out() << "Euler angles (degrees): " << euler_angles  << endl;
    vw_out() << "Axis of rotation and angle (degrees): "
             << axis_angles/norm_2(axis_angles) << ' '
             << norm_2(axis_angles) << endl;

    Stopwatch sw5;
    sw5.start();
    save_transforms(opt, globalT);

    if (opt.save_trans_ref){
      string trans_ref_prefix = opt.out_prefix + "-trans_reference";
      save_trans_point_cloud(opt, opt.reference, trans_ref_prefix,
                             geo, csv_conv, globalT.inverse());
    }

    if (opt.save_trans_source){
      string trans_source_prefix = opt.out_prefix + "-trans_source";
      save_trans_point_cloud(opt, opt.source, trans_source_prefix,
                             geo, csv_conv, globalT);
    }

    save_errors(source_point_cloud, beg_errors,  opt.out_prefix + "-beg_errors.csv",
                shift, geo, csv_conv, is_lola_rdr_format, mean_source_longitude);
    save_errors(trans_source_point_cloud, end_errors,  opt.out_prefix + "-end_errors.csv",
                shift, geo, csv_conv, is_lola_rdr_format, mean_source_longitude);

    if (opt.verbose) vw_out() << "Writing: " << opt.out_prefix
      + "-iterationInfo.csv" << std::endl;

    sw5.stop();
    if (opt.verbose) vw_out() << "Saving to disk took "
                              << sw5.elapsed_seconds() << " [s]" << endl;

  } ASP_STANDARD_CATCHES;

  return 0;
}
