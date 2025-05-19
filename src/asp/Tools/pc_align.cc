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

// This tool uses libpointmatcher for alignment.
// https://github.com/ethz-asl/libpointmatcher
// Released under the BSD 3-Clause.

#include <asp/Core/Common.h>
#include <asp/Core/Macros.h>
#include <asp/Core/PointUtils.h>
#include <asp/Core/PdalUtils.h>
#include <asp/Core/InterestPointMatching.h>
#include <asp/PcAlign/pc_align_utils.h>
#include <asp/PcAlign/pc_align_ceres.h>
#include <asp/PcAlign/pc_align_fgr.h>
#include <asp/PcAlign/NuthAlignment.h>

#include <vw/Core/Stopwatch.h>
#include <vw/Math/EulerAngles.h>
#include <vw/FileIO/DiskImageView.h>
#include <vw/Cartography/Datum.h>
#include <vw/Cartography/GeoReference.h>
#include <vw/Cartography/PointImageManipulation.h>
#include <vw/FileIO/DiskImageUtils.h>
#include <vw/Core/CmdUtils.h>
#include <vw/Math/Geometry.h>
#include <vw/InterestPoint/MatcherIO.h>

#include <limits>
#include <cstring>
#include <thread>
#include <omp.h>

#include <boost/filesystem.hpp>
#include <boost/algorithm/string.hpp>
namespace fs = boost::filesystem;
namespace po = boost::program_options;

using namespace vw;
using namespace std;
using namespace vw::cartography;
using namespace asp;

const double BIG_NUMBER = 1e+300; // libpointmatcher does not like here the largest double

/// Options container for the pc_align tool
struct Options: public vw::GdalWriteOptions {
  // Input
  string reference, source, init_transform_file, alignment_method, 
    datum, csv_format_str, csv_srs, match_file, hillshade_options,
    ipfind_options, ipmatch_options, nuth_options, fgr_options, csv_proj4_str;
  Vector2 initial_transform_ransac_params;
  Eigen::MatrixXd init_transform;
  int    num_iter,
         max_num_reference_points,
         max_num_source_points;
  double diff_translation_err, diff_rotation_err,
         max_disp, outlier_ratio, semi_major_axis,
         semi_minor_axis, initial_rotation_angle;
  bool   compute_translation_only, dont_use_dem_distances,
         save_trans_source, save_trans_ref,
         highest_accuracy, verbose, skip_shared_box_estimation;
  std::string initial_ned_translation, hillshading_transform;
  vw::BBox2 ref_copc_win, src_copc_win;
  bool ref_copc_read_all, src_copc_read_all;
  
  // Output
  string out_prefix;

  Options() : max_disp(-1.0), verbose(true){}
  
  /// Return true if the reference file is a DEM file and this option is not disabled
  bool use_dem_distances() const { 
    return (asp::get_cloud_type(this->reference) == "DEM") && !dont_use_dem_distances; 
  }
  
};

void handle_arguments(int argc, char *argv[], Options& opt) {
  po::options_description general_options("");
  general_options.add_options()
    ("initial-transform", po::value(&opt.init_transform_file)->default_value(""),
      "The file containing the transform to be used as an initial guess. It can come from a previous run of the tool.")
    ("num-iterations", po::value(&opt.num_iter)->default_value(1000),
      "Maximum number of iterations.")
    ("diff-rotation-error", po::value(&opt.diff_rotation_err)->default_value(1e-8),
      "Change in rotation amount below which the algorithm will stop (if translation error is also below bound), in degrees.")
    ("diff-translation-error",   po::value(&opt.diff_translation_err)->default_value(1e-3),
      "Change in translation amount below which the algorithm will stop (if rotation error is also below bound), in meters.")
    ("max-displacement", po::value(&opt.max_disp)->default_value(0.0),
      "Maximum expected displacement of source points as result of alignment, in meters (after the initial guess transform is applied to the source points). Used for removing gross outliers in the source point cloud.")
    ("outlier-ratio", po::value(&opt.outlier_ratio)->default_value(0.75),
      "Fraction of source (movable) points considered inliers (after gross outliers further than max-displacement from reference points are removed).")
    ("max-num-reference-points", 
     po::value(&opt.max_num_reference_points)->default_value(100000000),
     "Maximum number of (randomly picked) reference points to use.")
    ("max-num-source-points", po::value(&opt.max_num_source_points)->default_value(100000),
      "Maximum number of (randomly picked) source points to use (after discarding gross outliers).")
    ("alignment-method", po::value(&opt.alignment_method)->default_value("point-to-plane"),
     "Alignment method. Options: point-to-plane, point-to-point, "
     "similarity-point-to-plane, similarity-point-to-point, nuth, fgr, least-squares, "
     "similarity-least-squares.")
    ("highest-accuracy", 
     po::bool_switch(&opt.highest_accuracy)->default_value(false)->implicit_value(true),
     "Compute with highest accuracy for point-to-plane (can be much slower).")
    ("csv-format", po::value(&opt.csv_format_str)->default_value(""),  
     asp::csv_opt_caption().c_str())
    ("csv-srs", po::value(&opt.csv_srs)->default_value(""), 
     "The PROJ or WKT string to use to interpret the entries in input CSV files.")
    ("datum", po::value(&opt.datum)->default_value(""),
     "Use this datum for CSV files instead of auto-detecting it. Options: WGS_1984, D_MOON (1,737,400 meters), D_MARS (3,396,190 meters), MOLA (3,396,000 meters), NAD83, WGS72, and NAD27. Also accepted: Earth (=WGS_1984), Mars (=D_MARS), Moon (=D_MOON).")
    ("semi-major-axis", po::value(&opt.semi_major_axis)->default_value(0),
     "Explicitly set the datum semi-major axis in meters.")
    ("semi-minor-axis", po::value(&opt.semi_minor_axis)->default_value(0),
     "Explicitly set the datum semi-minor axis in meters.")
    ("output-prefix,o", po::value(&opt.out_prefix)->default_value(""),
     "Specify the output prefix.")
    ("compute-translation-only", 
     po::bool_switch(&opt.compute_translation_only)->default_value(false)->implicit_value(true),
     "Compute the transform from source to reference point cloud as a translation only (no rotation).")
    ("save-transformed-source-points", 
     po::bool_switch(&opt.save_trans_source)->default_value(false)->implicit_value(true),
      "Apply the obtained transform to the source points so they match the reference points and save them.")
    ("save-inv-transformed-reference-points", po::bool_switch(&opt.save_trans_ref)->default_value(false)->implicit_value(true),
     "Apply the inverse of the obtained transform to the reference points so they match the source points and save them.")
    ("initial-ned-translation", po::value(&opt.initial_ned_translation)->default_value(""),
     "Initialize the alignment transform based on a translation with this vector in the North-East-Down coordinate system around the centroid of the reference points. Specify it in quotes, separated by spaces or commas.")
    ("initial-rotation-angle", po::value(&opt.initial_rotation_angle)->default_value(0),
     "Initialize the alignment transform as the rotation with this angle (in degrees) around the axis going from the planet center to the centroid of the point cloud. If --initial-ned-translation is also specified, the translation gets applied after the rotation.")
    ("initial-transform-from-hillshading", po::value(&opt.hillshading_transform)->default_value(""), "If both input clouds are DEMs, find interest point matches among their hillshaded versions, and use them to compute an initial transform to apply to the source cloud before proceeding with alignment. Specify here the type of transform, as one of: 'similarity' (rotation + translation + scale), 'rigid' (rotation + translation) or 'translation'. See the options further down for tuning this.")
    ("hillshade-options", po::value(&opt.hillshade_options)->default_value("--azimuth 300 --elevation 20 --align-to-georef"), "Options to pass to the hillshade program when computing the transform from hillshading.")
    ("ipfind-options", po::value(&opt.ipfind_options)->default_value("--ip-per-image 1000000 --interest-operator sift --descriptor-generator sift"), "Options to pass to the ipfind program when computing the transform from hillshading.")
    ("ipmatch-options", po::value(&opt.ipmatch_options)->default_value("--inlier-threshold 100 --ransac-iterations 10000 --ransac-constraint similarity"), "Options to pass to the ipmatch program when computing the transform from hillshading.")
    ("match-file", po::value(&opt.match_file)->default_value(""), "Compute a translation + rotation + scale transform from the source to the reference point cloud using manually selected point correspondences from the reference to the source (obtained for example using stereo_gui). It may be desired to change --initial-transform-ransac-params if it rejects as outliers some manual matches.")
    ("initial-transform-ransac-params", po::value(&opt.initial_transform_ransac_params)->default_value(Vector2(10000, 1.0), "num_iter factor"),
     "When computing an initial transform based on hillshading, use "
     "this number of RANSAC iterations and outlier factor. A smaller factor "
     "will reject more outliers.")
    ("nuth-options", po::value(&opt.nuth_options)->default_value(""),
     "Options to pass to the Nuth and Kaab algorithm.")
    ("fgr-options", po::value(&opt.fgr_options)->default_value("div_factor: 1.4 use_absolute_scale: 0 max_corr_dist: 0.025 iteration_number: 100 tuple_scale: 0.95 tuple_max_cnt: 10000"), 
     "Options to pass to the Fast Global Registration algorithm.")
    ("no-dem-distances", po::bool_switch(&opt.dont_use_dem_distances)->default_value(false)->implicit_value(true),
     "For reference point clouds that are DEMs, don't take advantage of the fact that it is possible to interpolate into this DEM when finding the closest distance to it from a point in the source cloud and hence the error metrics.")
    ("skip-shared-box-estimation", po::bool_switch(&opt.skip_shared_box_estimation)->default_value(false)->implicit_value(true),
     "Do not estimate the shared bounding box of the two clouds. This estimation "
     "can be costly for large clouds but helps with eliminating outliers.")
    ("ref-copc-win", po::value(&opt.ref_copc_win)->default_value(vw::BBox2()),
     "Specify the region to read from the reference cloud, if it is a COPC LAZ file. The "
     "units are based the projection in the file. This is required unless --ref-copc-read- "
     "all is set. Specify as minx miny maxx maxy, or minx maxy maxx miny, with no quotes.")
    ("src-copc-win", po::value(&opt.src_copc_win)->default_value(vw::BBox2()),
     "Specify the region to read from the source cloud, if it is a COPC LAZ file. The "
     "units are based the projection in the file. This is required unless "
     "--src-copc-read-all all is set. Specify as minx miny maxx maxy, "
     "or minx maxy maxx miny, with no quotes. If not set, the --ref-copc-win option will "
     "be used, or otherwise it will be estimated based on the extent of reference points "
     "and the --max-displacement option.")
    ("ref-copc-read-all", po::bool_switch(&opt.ref_copc_read_all)->default_value(false), 
     "Read the full reference COPC file, ignoring the --ref-copc-win option.")
    ("src-copc-read-all", po::bool_switch(&opt.src_copc_read_all)->default_value(false), 
     "Read the full source COPC file, ignoring the --src-copc-win option.")
    ("csv-proj4", po::value(&opt.csv_proj4_str)->default_value(""), 
     "An alias for --csv-srs, for backward compatibility.");

  general_options.add(vw::GdalWriteOptionsDescription(opt));

  po::options_description positional("");
  positional.add_options()
    ("reference", po::value(&opt.reference), "The reference (fixed) point cloud/DEM.")
    ("source",    po::value(&opt.source),    "The source (movable) point cloud/DEM.");

  po::positional_options_description positional_desc;
  positional_desc.add("reference", 1);
  positional_desc.add("source",    1);

  string usage("--max-displacement arg [other options] <reference cloud> <source cloud> "
               "-o <output prefix>");
  bool allow_unregistered = false;
  std::vector<std::string> unregistered;
  po::variables_map vm =
    asp::check_command_line(argc, argv, opt, general_options, general_options,
                            positional, positional_desc, usage,
                            allow_unregistered, unregistered);

  // If the user specifies no options at all, print the help message.
  // Otherwise, print the specific error.
  if (opt.reference.empty() && opt.source.empty() && opt.out_prefix.empty())
    vw_throw( ArgumentErr() << "No input arguments provided.\n" 
             << usage << general_options);

  if (opt.reference.empty() || opt.source.empty())
    vw_throw( ArgumentErr() << "Missing input files.\n");

  if (opt.out_prefix.empty())
    vw_throw( ArgumentErr() << "Missing output prefix.\n");

  if ( opt.max_disp == 0.0 )
    vw_throw( ArgumentErr() << "The max-displacement option was not set. "
              << "Use -1 if it is desired not to use it.\n");

  if (opt.num_iter < 0)
    vw_throw( ArgumentErr() << "The number of iterations must be non-negative.\n");

  if ( (opt.semi_major_axis != 0 && opt.semi_minor_axis == 0) ||
       (opt.semi_minor_axis != 0 && opt.semi_major_axis == 0) ){

    vw_throw( ArgumentErr() << "One of the semi-major or semi-minor axes"
              << " was specified, but not the other one.\n");
  }

  if (opt.semi_major_axis < 0 || opt.semi_minor_axis < 0){
    vw_throw( ArgumentErr() << "The semi-major and semi-minor axes cannot "
                            << "be negative.\n" );
  }

  if (opt.datum != "" && opt.semi_major_axis != 0 && opt.semi_minor_axis != 0 ){
    vw_throw( ArgumentErr() << "Both the datum string and datum semi-axes were "
                            << "specified. At most one needs to be set.\n");
  }

  if ((opt.initial_ned_translation != "" || opt.initial_rotation_angle != 0)
      && opt.init_transform_file != "")
    vw_throw( ArgumentErr()
              << "Cannot specify an initial transform both from a file "
              << "and as a NED vector or rotation angle.\n");

  if ( (opt.hillshading_transform != "" || opt.match_file != "") &&
       (opt.initial_ned_translation != "" || opt.init_transform_file != "" ||
        opt.initial_rotation_angle != 0)) {
    vw_throw( ArgumentErr() << "Cannot both specify an initial transform "
              << "and expect one to be computed automatically.\n");
  }

  // Must specify either csv_srs or csv_proj4_str, but not both. The latter is 
  // for backward compatibility.
  if (!opt.csv_srs.empty() && !opt.csv_proj4_str.empty())
    vw_throw(ArgumentErr() << "Cannot specify both --csv-srs and --csv-proj4.\n");
  if (!opt.csv_proj4_str.empty() && opt.csv_srs.empty())
    opt.csv_srs = opt.csv_proj4_str;

  // Create the output directory
  vw::create_out_dir(opt.out_prefix);

  // Turn on logging to file
  asp::log_to_file(argc, argv, "", opt.out_prefix);

  // Read the initial transform
  opt.init_transform = Eigen::MatrixXd::Identity(DIM + 1, DIM + 1);
  if (opt.init_transform_file != ""){
    asp::read_transform(opt.init_transform, opt.init_transform_file);
    vw_out() << std::setprecision(16) << "Initial guess transform:\n" 
             << opt.init_transform << "\n";
    vw_out() << std::setprecision(8); // undo the higher precision
  }

  if (opt.alignment_method != "point-to-plane"            &&
      opt.alignment_method != "point-to-point"            &&
      opt.alignment_method != "similarity-point-to-point" &&
      opt.alignment_method != "similarity-point-to-plane" &&
      opt.alignment_method != "nuth"                      &&
      opt.alignment_method != "fgr"                       &&
      opt.alignment_method != "least-squares"             &&
      opt.alignment_method != "similarity-least-squares")
    vw_throw( ArgumentErr() << "Only the following alignment methods are supported: "
        << "point-to-plane, point-to-point, similarity-point-to-point, "
      << "similarity-point-to-plane, fgr, least-squares, and similarity-least-squares.\n");

  if (opt.alignment_method != "point-to-plane"            &&
      opt.alignment_method != "point-to-point"            &&
      opt.alignment_method != "similarity-point-to-point" &&
      opt.alignment_method != "similarity-point-to-plane" &&
      opt.alignment_method != "nuth"                      &&
      opt.compute_translation_only) {
    vw_throw( ArgumentErr() << "The option --compute-translation-only is only applicable "
             << "to point-to-plane, point-to-point, similarity-point-to-point, "
             << "similarity-point-to-plane, and nuth alignment.\n");
  }
  
  if ((opt.alignment_method == "least-squares" ||
    opt.alignment_method == "similarity-least-squares")
       && asp::get_cloud_type(opt.reference) != "DEM")
    vw_throw( ArgumentErr()
          << "Least squares alignment can be used only when the "
          << "reference cloud is a DEM.\n" );

  if (opt.alignment_method == "nuth") {
         
    if (asp::get_cloud_type(opt.reference) != "DEM" ||
        asp::get_cloud_type(opt.source)    != "DEM")
      vw_throw(ArgumentErr()
          << "Nuth and Kaab alignment can be used only when both "
          << "the reference and source clouds are DEMs.\n");
      
    if (!opt.init_transform_file.empty() ||
        !opt.initial_ned_translation.empty() ||
        !opt.hillshading_transform.empty())
      vw_throw(ArgumentErr()
          << "Nuth and Kaab alignment cannot be used with an initial transform.\n");  
      
    if (opt.dont_use_dem_distances)
      vw_throw(ArgumentErr()
          << "Nuth and Kaab alignment requires using DEM distances for statistics.\n");
      
    if (!opt.match_file.empty())
      vw_throw(ArgumentErr()
          << "Nuth and Kaab alignment cannot be used with a match file.\n");
    
    if (opt.initial_rotation_angle != 0)
      vw_throw(ArgumentErr()
          << "Nuth and Kaab alignment cannot be used with an initial rotation angle.\n");

     opt.max_num_reference_points /= 25;
     opt.max_num_reference_points = std::max(1000000, opt.max_num_reference_points);
     vw::vw_out() << "For Nuth and Kaab alignment, a smaller reference cloud is "
        << "sufficient. Reducing --max-num-reference-points by about 25x, to: " 
        << opt.max_num_reference_points << ".\n";
  }
      
  int num_iter  = opt.initial_transform_ransac_params[0];
  double factor = opt.initial_transform_ransac_params[1];
  if (num_iter < 1 || factor <= 0.0)
    vw_throw( ArgumentErr() << "Invalid values were provided for "
              << "--initial-transform-ransac-params.\n");

  // Support for minx maxy maxx miny format.
  if (opt.ref_copc_win != BBox2()) {
    if (opt.ref_copc_win.min().y() > opt.ref_copc_win.max().y())
      std::swap(opt.ref_copc_win.min().y(), opt.ref_copc_win.max().y());
  }
  if (opt.src_copc_win != BBox2()) {
    if (opt.src_copc_win.min().y() > opt.src_copc_win.max().y())
      std::swap(opt.src_copc_win.min().y(), opt.src_copc_win.max().y());
  }
  
  // Use ref_copc_win, if src_copc_win is not set
  if (asp::isCopc(opt.source) &&
      opt.src_copc_win == BBox2() &&
      opt.ref_copc_win != BBox2()) {
    vw::vw_out() << "Using --ref-copc-win for --src-copc-win.\n";
    opt.src_copc_win = opt.ref_copc_win;
  }

}

/// Compute output statistics for pc_align
void calc_stats(string label, Eigen::MatrixXd const& dists) {

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

/// Write the output points as xyz values in binary, to be used by
/// https://github.com/IntelVCL/FastGlobalRegistration
void dump_bin(string const& file, DP const & data){

  vw_out() << "Writing: "   << data.features.cols()
           << " points to " << file << "\n";

  FILE* fid = fopen(file.c_str(), "wb");
  int nV = data.features.cols(),
    nDim = 3; // tmp!
  fwrite(&nV, sizeof(int), 1, fid);
  fwrite(&nDim, sizeof(int), 1, fid);
  for (int c = 0; c < data.features.cols(); c++){
    float xyz[3];
    for (int r = 0; r < 3; r++) xyz[r] = data.features(r, c);
    fwrite(xyz, sizeof(float), 3, fid);

    // That code needs features
    fwrite(xyz, sizeof(float), 3, fid);
    
  }
  fclose(fid);
  
}

// Save a cloud to disk for debugging
void debug_save_point_cloud(DP const& point_cloud, GeoReference const& geo,
                            Vector3 const& shift,
                            string const& output_file){

  int numPts = point_cloud.features.cols();

  vw_out() << "Writing: " << numPts << " to " << output_file << endl;
  ofstream outfile( output_file.c_str() );
  outfile.precision(18);

  for (int col = 0; col < numPts; col++){
    Vector3 P = get_cloud_gcc_coord(point_cloud, shift, col);

    Vector3 llh = geo.datum().cartesian_to_geodetic(P); // lon-lat-height
    outfile << llh[1] << ',' << llh[0] << ',' << llh[2] << endl;
  }
  outfile.close();
}

/// Save the transform and its inverse.
void write_transforms(Options const& opt,
                     Eigen::MatrixXd const& T) {

  string transFile = opt.out_prefix + "-transform.txt";
  vw_out() << "Writing: " << transFile << endl;
  write_transform(T, transFile);

  string iTransFile = opt.out_prefix + "-inverse-transform.txt";
  Eigen::MatrixXd invT = T.inverse();
  vw_out() << "Writing: " << iTransFile << endl;
  write_transform(invT, iTransFile);
}

/// Save the lon, lat, radius/height, and error. Use a format
/// consistent with the input CSV format.
void save_errors(DP const& point_cloud,
                 Eigen::MatrixXd const& errors,
                 string const& output_file,
                 Vector3 const& shift,
                 GeoReference const& geo,
                 asp::CsvConv const& csv_conv,
                 bool is_lola_rdr_format,
                 double median_longitude){

  vw_out() << "Writing: " << output_file << "\n";

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
  if (geo.datum().name() != UNSPECIFIED_DATUM) {
    outfile << "# " << geo.datum() << "\n";
    outfile << "# Projection: " << geo.get_wkt() << "\n";
  }

  int numPts = point_cloud.features.cols();
  for (int col = 0; col < numPts; col++){
    Vector3 P = get_cloud_gcc_coord(point_cloud, shift, col);

    if (csv_conv.is_configured()){
      Vector3 csv = csv_conv.cartesian_to_csv(P, geo, median_longitude);
      outfile << csv[0] << ',' << csv[1] << ',' << csv[2]
              << "," << errors(0, col) << endl;
    }else{
      Vector3 llh = geo.datum().cartesian_to_geodetic(P); // lon-lat-height
      llh[0] += 360.0*round((median_longitude - llh[0])/360.0); // 360 deg adjustment

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

/// Like PM::ICP::filterGrossOutliersAndCalcErrors, except comparing to a DEM instead.
/// - The point cloud is in GCC coordinates with point_cloud_shift subtracted from each point.
/// - The output is put in the "errors" vector for each point.
/// - If there is a problem computing the point error, a very large number is used as a flag.
void calcErrorsWithDem(DP                                 const& point_cloud,
                       vw::Vector3                        const& point_cloud_shift,
                       vw::cartography::GeoReference      const& georef,
                       vw::ImageViewRef<PixelMask<float>> const& dem,
                       std::vector<double>                     & errors) {

  // Initialize output error storage
  const std::int64_t num_pts = point_cloud.features.cols();
  errors.resize(num_pts);

  // Loop through every point in the point cloud
  double dem_height_here;
  for (std::int64_t i = 0; i < num_pts; i++){
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

// Updates an LPM error matrix to use the DEM-based error for each point if it is lower.
// Note: The LPM matrix type used to store errors only ever has a single row.
void update_best_error(std::vector<double>         const& dem_errors,
                       Eigen::MatrixXd      & lpm_errors) {
  std::int64_t num_points = lpm_errors.cols();
  if (dem_errors.size() != static_cast<size_t>(num_points))
    vw_throw(LogicErr() << "Error: error size does not match point count size!\n");

  // Loop through points
  for (std::int64_t col = 0; col < num_points; col++){
    // Use the DEM error if it is less
    if (dem_errors[col] < lpm_errors(0,col)) {
      lpm_errors(0, col) = dem_errors[col];
    }
  }

}

/// Compute the distance from source_point_cloud to the reference points.
double compute_registration_error(DP          const& ref_point_cloud,
                                  DP               & source_point_cloud, // Should not be modified
                                  PM::ICP          & pm_icp_object,
                                  vw::Vector3 const& shift,
                                  vw::cartography::GeoReference        const& dem_georef,
                                  vw::ImageViewRef< PixelMask<float> > const& dem_ref,
                                  Options const& opt,
                                  Eigen::MatrixXd &error_matrix) {
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

  Eigen::MatrixXd error_matrix;
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
  }catch(const PointMatcher<double>::ConvergenceError & e){
    vw_throw( ArgumentErr() << "Error: No points left in source cloud after filtering. Consider increasing --max-displacement and/or see the documentation.\n");
  }


  sw.stop();
  if (opt.verbose)
    vw_out() << "Filtering gross outliers took " << sw.elapsed_seconds() << " s\n";
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
  if (alignment_method == "least-squares" || alignment_method == "similarity-least-squares" ||
      alignment_method == "fgr" || alignment_method == "nuth") 
    return "point-to-plane";
  return alignment_method;
}

// Hillshade the reference and source DEMs, and use them to find
// interest point matches among the hillshaded images.  These will be
// used later to find a rotation + translation + scale transform.
std::string find_matches_from_hillshading(Options & opt, std::string const& curr_exec_path) {

  // First, this works only for DEMs
  if (asp::get_cloud_type(opt.reference) != "DEM" ||
      asp::get_cloud_type(opt.source) != "DEM" )
    vw_throw( ArgumentErr() << "Cannot find an initial transform using hillshading "
              << "unless both point clouds are DEMs. Use point2dem to first create "
              << "DEMs from the input point clouds. Then this transform can be used "
              << "with the original clouds.\n" );

  // Find the needed executables
  std::string hillshade_path = vw::program_path("hillshade", curr_exec_path);
  std::string ipfind_path    = vw::program_path("ipfind",    curr_exec_path);
  std::string ipmatch_path   = vw::program_path("ipmatch",   curr_exec_path);

  // Hillshade the reference
  std::string ref_hillshade = opt.out_prefix + "-reference_hillshade.tif";
  std::string cmd = hillshade_path + " " + opt.hillshade_options + " "
    + opt.reference + " -o " + ref_hillshade;
  vw_out() << cmd << "\n";
  std::string ans = vw::exec_cmd(cmd.c_str());
  vw_out() << ans << "\n";
  
  // Hillshade the source
  std::string source_hillshade = opt.out_prefix + "-source_hillshade.tif";
  cmd = hillshade_path + " " + opt.hillshade_options + " "
    + opt.source + " -o " + source_hillshade;
  vw_out() << cmd << "\n";
  ans = vw::exec_cmd(cmd.c_str());
  vw_out() << ans << "\n";

  // IP find
  cmd = ipfind_path + " " + opt.ipfind_options + " " + ref_hillshade + " " + source_hillshade;
  vw_out() << cmd << "\n";
  ans = vw::exec_cmd(cmd.c_str());
  vw_out() << ans << "\n";

  // IP match
  std::string ref_ip    = fs::path(ref_hillshade).replace_extension(".vwip").string();
  std::string source_ip = fs::path(source_hillshade).replace_extension(".vwip").string();

  cmd = ipmatch_path + " " + opt.ipmatch_options + " "
    + ref_hillshade + " " + ref_ip + " " + source_hillshade + " " + source_ip + " -o "
    + opt.out_prefix;
  vw_out() << cmd << "\n";
  ans = vw::exec_cmd(cmd.c_str());
  vw_out() << ans << "\n";

  // The name of the file where the matches are written to
  std::string match_file = vw::ip::match_filename(opt.out_prefix, ref_hillshade, source_hillshade);
  
  return match_file;
}

// Compute an initial source to reference transform based on tie points (interest point matches).
Eigen::MatrixXd
initial_transform_from_match_file(std::string const& ref_file,
                                  std::string const& source_file,
                                  std::string const& match_file,
                                  std::string const& hillshading_transform,
                                  Vector2 initial_transform_ransac_params) {
  
  if (asp::get_cloud_type(ref_file) != "DEM" || asp::get_cloud_type(source_file) != "DEM")
    vw_throw(ArgumentErr() 
             << "The alignment transform computation based on manually chosen "
             << "point matches only works for DEMs. Use point2dem to first create "
             << "DEMs from the input point clouds.\n");

  vector<vw::ip::InterestPoint> ref_ip, source_ip;
  vw_out() << "Reading match file: " << match_file << "\n";
  vw::ip::read_binary_match_file(match_file, ref_ip, source_ip);

  DiskImageView<float> ref(ref_file);
  vw::cartography::GeoReference ref_geo;
  bool has_ref_geo = vw::cartography::read_georeference(ref_geo, ref_file);
  double ref_nodata = -std::numeric_limits<double>::max();
  vw::read_nodata_val(ref_file, ref_nodata);

  DiskImageView<float> source(source_file);
  vw::cartography::GeoReference source_geo;
  bool has_source_geo = vw::cartography::read_georeference(source_geo, source_file);
  double source_nodata = -std::numeric_limits<double>::max();
  vw::read_nodata_val(source_file, source_nodata);

  if (!has_ref_geo || !has_source_geo)
    vw_throw( ArgumentErr() << "One of the inputs is not a valid DEM.\n" );

  if (DIM != 3)
    vw_throw( ArgumentErr() << "Expecting DIM = 3.\n" );

  // Go from pixels to 3D points
  int num_matches = ref_ip.size();
  vw::Matrix<double> points_ref(DIM, num_matches), points_src(DIM, num_matches);
  typedef vw::math::MatrixCol<vw::Matrix<double>> ColView;
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
  bool filter_outliers = true;
  vw::math::find_3D_transform(points_src, points_ref,
                              rotation, translation, scale,
                              hillshading_transform,
                              filter_outliers,
                              initial_transform_ransac_params);

  // Convert to pc_align transform format.
  Eigen::MatrixXd globalT = Eigen::MatrixXd::Identity(DIM+1, DIM+1);
  globalT.block(0, 0, DIM, DIM) = vw_matrix3_to_eigen(rotation*scale);
  globalT.block(0, DIM, DIM, 1) = vw_vector3_to_eigen(translation);

  vw_out() << "Transform computed from source to reference using a match file:\n"
           << globalT << "\n";

  return globalT;
}

void apply_transform_to_cloud(Eigen::MatrixXd const& T, DP & point_cloud){
  for (int col = 0; col < point_cloud.features.cols(); col++) {
    point_cloud.features.col(col) = T*point_cloud.features.col(col);
  }
}

// Load, filter, transform, and resample the source point cloud. The input
// clouds and transform have been shifted by the provided shift value. The 
// same shift will be applied to the produced source cloud.
void processSourceCloud(Options                       const& opt,
                        DP                            const& ref_point_cloud,
                        Eigen::MatrixXd   const& shiftInitT,
                        vw::cartography::GeoReference const& dem_georef,
                        vw::cartography::GeoReference const& geo,
                        asp::CsvConv                  const& csv_conv,
                        BBox2                         const& source_box,
                        vw::ImageViewRef<PixelMask<float>> reference_dem_ref,
                        bool is_lola_rdr_format,
                        double mean_source_longitude,
                        // These won't change but cannot be const
                        PM::ICP      & icp,
                        vw::Vector3  & shift,
                        // Output
                        DP           & source_point_cloud) {

  // Do two attempts, in case too few points are left after filtering. This can
  // happen if the source cloud is very different from the reference cloud.
  std::string tag = "";
  for (int attempt = 1; attempt <= 2; attempt++) {
    
    source_point_cloud = DP(); // Clear the output

    int big_num = 1.0e+6;
    if (attempt == 2) {
      big_num = 50.0e+6;
      tag = "(second attempt) ";
    }
      
    int num_source_pts = opt.max_num_source_points;
    if (opt.max_disp > 0.0)
      num_source_pts = std::max((1+attempt) * num_source_pts, big_num);
    
    // Use the same shift used for the reference point cloud    
    bool calc_shift = false; 
    Stopwatch sw;
    sw.start();
    load_cloud(opt.source, num_source_pts, source_box, 
               opt.src_copc_win, opt.src_copc_read_all,
               calc_shift, shift, geo, csv_conv, is_lola_rdr_format,
               mean_source_longitude, opt.verbose, source_point_cloud);
    sw.stop();
    if (opt.verbose)
      vw_out() << "Loading the source point cloud " << tag << "took "
                << sw.elapsed_seconds() << " s\n";

    // Apply the initial guess transform to the source point cloud.
    apply_transform_to_cloud(shiftInitT, source_point_cloud);
    
    // Filter gross outliers in the source point cloud with max_disp
    try {
      // This can throw an exception if the resulting cloud is empty 
      if (opt.max_disp > 0.0)
        filter_source_cloud(ref_point_cloud, source_point_cloud, icp,
                          shift, dem_georef, reference_dem_ref, opt);
    } catch (std::exception const& e) {
      vw_out() << "Not enough points left in the source cloud after filtering. "
               << "Try loading more.\n";  
      continue;
    }
    
    random_pc_subsample(opt.max_num_source_points, source_point_cloud.features);
    vw_out() << "Reducing number of source points to: "
              << source_point_cloud.features.cols() << "\n";
  
    if (source_point_cloud.features.cols() >= opt.max_num_source_points)
      break;
    
    if (attempt == 1)   
      vw_out() << "Not enough points left in the source cloud after filtering. "
               << "Try loading more.\n";  
  }
  
  // Write the point cloud to disk for debugging
  //debug_save_point_cloud(ref_point_cloud, geo, shift, "ref.csv");
  //dump_bin("ref.bin", ref_point_cloud);
}

// Convert a north-east-down vector at a given location to a vector in reference
// to the center of the Earth and create a translation matrix from that vector. 
Eigen::MatrixXd 
ned_to_cartesian_transform(vw::cartography::Datum const& datum,
                           std::string& ned_str, 
                           vw::Vector3 const & location) {

  vw::string_replace(ned_str, ",", " "); // replace any commas
  vw::Vector3 ned = vw::str_to_vec<vw::Vector3>(ned_str);

  vw::Vector3 loc_llh = datum.cartesian_to_geodetic(location);
  vw::Matrix3x3 NedToEcef = datum.lonlat_to_ned_matrix(loc_llh);
  vw::Vector3 xyz_shift = NedToEcef * ned;
  
  Eigen::MatrixXd T = Eigen::MatrixXd::Identity(DIM + 1, DIM + 1);

  // Append the transform
  for (int row = 0; row < DIM; row++)
    T(row, DIM) = xyz_shift[row];

  return T;
}

// Estimate the centroid of the reference points
vw::Vector3 estimate_ref_cloud_centroid(Options const& opt,
                                        vw::cartography::GeoReference const& geo,
                                        CsvConv const& csv_conv,
                                        std::string const& reference) {
  Stopwatch sw;
  sw.start();
  
  PointMatcherSupport::validateFile(reference);
  PointMatcher<double>::DataPoints points;

  double median_longitude = 0.0; // to convert back from xyz to lonlat
  bool verbose = false;
  bool calc_shift = false; // won't shift the points
  vw::Vector3 shift = vw::Vector3(0, 0, 0);
  vw::BBox2 dummy_box;
  bool is_lola_rdr_format;
  // Load a sample of points, hopefully enough to estimate the centroid
  // reliably.
  int num_sample_pts = 1000000;
  load_cloud(reference, num_sample_pts, dummy_box,
             opt.ref_copc_win, opt.ref_copc_read_all,
             calc_shift, shift, geo, csv_conv, is_lola_rdr_format,
             median_longitude, verbose, points);

  int numRefPts = points.features.cols();
  Eigen::VectorXd meanRef = points.features.rowwise().sum() / numRefPts;
  
  vw::Vector3 centroid;
  for (int it = 0; it < 3; it++)
    centroid[it] = meanRef[it];
  
  sw.stop();
  vw_out() << "Centroid estimation took " << sw.elapsed_seconds() << " s\n";

  return centroid;
}

// Intersect the reference and source boxes, making sure to handle well the potential
// 360 degree offset among the two.
void adjust_and_intersect_ref_source_boxes(BBox2 & ref_box, BBox2 & source_box,
                                           std::string const& reference,
                                           std::string const& source){

  double lon_offset = 0.0;
  
  // Compute the longitude offset
  double source_mean_lon = (source_box.min().x() + source_box.max().x())/2.0;
  double ref_mean_lon    = (ref_box.min().x()    + ref_box.max().x()   )/2.0;
  lon_offset = source_mean_lon - ref_mean_lon;
  lon_offset = 360.0*round(lon_offset/360.0);

  // Move the ref box in the domain of the source box
  ref_box += Vector2(lon_offset, 0);

  // Intersect them, as pc_align will operate on their common area
  ref_box.crop(source_box);
  source_box.crop(ref_box);

  // Move back the ref box to its domain
  ref_box -= Vector2(lon_offset, 0);
  
  // Extra adjustments. These are needed since pixel_to_lonlat and
  // cartesian_to_geodetic can disagree by 360 degrees. Adjust ref
  // to source and vice-versa.
  adjust_lonlat_bbox(reference, ref_box);
  adjust_lonlat_bbox(source, source_box);
}

// See if we need to estimate the proj win of the source points
// based on reference points and max disp. This is needed for COPC
// files when this win is not set by the user.
void checkNeeForSrcProjWin(Options const& opt, 
                           bool & need_src_projwin,
                           vw::cartography::GeoReference & src_georef) {

  need_src_projwin = false;
  src_georef = vw::cartography::GeoReference();
  
  if (asp::isCopc(opt.source) && opt.src_copc_win == vw::BBox2() &&
      !opt.src_copc_read_all && opt.max_disp > 0.0) {

    need_src_projwin = true;
    
    // Sanity check
    bool has_src_georef = asp::georef_from_las(opt.source, src_georef);
    if (!has_src_georef)
      vw_throw(ArgumentErr() 
                << "Cannot read the georeference of the source point cloud.\n");
    
    // Need this later when biasing by max_disp  
    if (!src_georef.is_projected())
      vw_throw(ArgumentErr() 
                << "The source point cloud is not in projected coordinates. "
                << "Set the option: --src-copc-win.");
  }
  
}
                                         
int main(int argc, char *argv[]) {

  // Mandatory line for Eigen
  Eigen::initParallel();

  Options opt;
  try {
    handle_arguments(argc, argv, opt);

    // Set the number of threads for OpenMP. 
    int processor_count = std::thread::hardware_concurrency();
    omp_set_dynamic(0);
    omp_set_num_threads(processor_count);
    
    // Parse the csv format string and csv projection string
    asp::CsvConv csv_conv;
    csv_conv.parse_csv_format(opt.csv_format_str, opt.csv_srs);

    // Try to read the georeference/datum info
    GeoReference geo;
    std::vector<std::string> clouds;
    clouds.push_back(opt.reference);
    clouds.push_back(opt.source);
    read_georef(clouds, opt.datum, opt.csv_srs,  
                opt.semi_major_axis, opt.semi_minor_axis,  
                opt.csv_format_str,  csv_conv, geo);

    // Use hillshading to create a match file
    if (opt.hillshading_transform != "" && opt.match_file == "")
      opt.match_file = find_matches_from_hillshading(opt, argv[0]);
    
    // Create a transform based on a match file, either automatically generated, or
    // user-made (normally with stereo_gui).
    if (opt.match_file != "") {
      if (opt.hillshading_transform == "") 
        opt.hillshading_transform = "similarity";
      opt.init_transform 
        = initial_transform_from_match_file(opt.reference, opt.source,
                                            opt.match_file, opt.hillshading_transform,
                                            opt.initial_transform_ransac_params);
    }

    // See if to apply an initial north-east-down translation relative
    // to the point cloud centroid, and/or a rotation around the axis
    // going from the planet center to the centroid. The rotation
    // happens first.
    if (opt.initial_rotation_angle != 0 || opt.initial_ned_translation != "") {

      vw::Vector3 centroid = estimate_ref_cloud_centroid(opt, geo, csv_conv, opt.reference);

      // Ignore any other initializations so far
      opt.init_transform = Eigen::MatrixXd::Identity(DIM + 1, DIM + 1);

      // Form the rotation around the axis
      Eigen::Vector3d axis(centroid[0], centroid[1], centroid[2]);
      axis.normalize();
      Eigen::Matrix3d rot
        = Eigen::AngleAxisd(opt.initial_rotation_angle * M_PI/180.0, axis).matrix();
      opt.init_transform.block(0, 0, DIM, DIM) = rot;

      // The NED translation
      if (opt.initial_ned_translation != "") 
        opt.init_transform = 
          ned_to_cartesian_transform(geo.datum(), opt.initial_ned_translation, centroid)
          * opt.init_transform;
    }

    // Find the intersection of ref and source bounding boxs. This does not need
    // a lot of samples as it is expanded by max_disp in either case.
    BBox2 ref_box, source_box, trans_ref_box, trans_source_box;
    if (!opt.skip_shared_box_estimation) {
      Stopwatch sw0;
      sw0.start();
      int num_sample_pts = std::max(4000000,
                                    std::max(opt.max_num_source_points,
                                             opt.max_num_reference_points)/4);
      num_sample_pts = std::min(1.0e+6, 1.0 * num_sample_pts); // avoid being slow
      
      // Compute GDC bounding box of the source and reference clouds.
      vw_out() << "Computing the bounding boxes of the reference and source points using " 
               << num_sample_pts << " sample points.\n";

      // See if we have to estimate the proj win of the src points
      bool need_src_projwin = false, dummy_flag = false;
      vw::cartography::GeoReference src_georef, dummy_georef;
      vw::BBox2 src_projwin, dummy_projwin;
      checkNeeForSrcProjWin(opt, need_src_projwin, src_georef);
      
      Eigen::MatrixXd inv_init_trans = opt.init_transform.inverse();
      calc_extended_lonlat_bbox(geo, num_sample_pts, csv_conv,
                                opt.reference, opt.max_disp, inv_init_trans,
                                opt.ref_copc_win, opt.ref_copc_read_all,
                                need_src_projwin, src_georef,
                                ref_box, trans_ref_box, src_projwin); // outputs

      if (need_src_projwin) {
        opt.src_copc_win = src_projwin;
        vw::vw_out() << "Estimated: --src-copc-win "
          << opt.src_copc_win.min().x() << " " << opt.src_copc_win.min().y() << " "
          << opt.src_copc_win.max().x() << " " << opt.src_copc_win.max().y() << "\n";
      }

      calc_extended_lonlat_bbox(geo, num_sample_pts, csv_conv,
                                opt.source, opt.max_disp, opt.init_transform,
                                opt.src_copc_win, opt.src_copc_read_all,
                                dummy_flag, dummy_georef,
                                source_box, trans_source_box, dummy_projwin); // outputs
      sw0.stop();
      vw_out() << "Computation of bounding boxes took " 
              << sw0.elapsed_seconds() << " s\n";

      // When boxes are huge, it is hard to do the optimization of intersecting
      // them, as they may differ not by 0 or 360, but by 180. Better do nothing
      // in that case. The solution may degrade a bit, as we may load points
      // not in the intersection of the boxes, but at least it won't be wrong.
      // In this case, there is a chance the boxes were computed wrong anyway.
      if (ref_box.width() > 180.0 || source_box.width() > 180.0) {
        vw_out() << "Warning: Your input point clouds are spread over more than half "
                  << "the planet. It is suggested that they be cropped, to get more "
                  << "accurate results. Giving up on estimating their bounding boxes "
                  << "and filtering outliers based on them.\n";
                
        ref_box = BBox2();
        source_box = BBox2();
      }
    
      // This is useful to point out issues when the reference and source
      // boxes are shifted by 360 degrees relative to each other.
      vw_out() << "Reference points box: " << ref_box << "\n";
      vw_out() << "Source points box:    " << source_box << "\n";
      
      if (!ref_box.empty() && !source_box.empty()) {
        adjust_and_intersect_ref_source_boxes(ref_box, trans_source_box, 
                                              opt.reference, opt.source);
        adjust_and_intersect_ref_source_boxes(trans_ref_box, source_box, 
                                              opt.reference, opt.source);
      }
      if (ref_box == source_box) {
        // Leave some space here to align the text to the earlier printouts
        vw_out() << "Intersection box:     " << ref_box    << "\n";
      } else {
        // This can happen when the projections are offset by 360 degrees
        vw_out() << "Intersection reference box: " << ref_box    << "\n";
        vw_out() << "Intersection source    box: " << source_box << "\n";
      }
    }
    
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
    load_cloud(opt.reference, opt.max_num_reference_points, ref_box,
               opt.ref_copc_win, opt.ref_copc_read_all,
               calc_shift, shift, geo, csv_conv, is_lola_rdr_format,
               mean_ref_longitude, opt.verbose, ref_point_cloud);
    sw1.stop();
    if (opt.verbose)
      vw_out() << "Loading the reference point cloud took "
               << sw1.elapsed_seconds() << " s\n";
    //ref_point_cloud.save(outputBaseFile + "_ref.vtk");

    // So far we shifted by first point in reference point cloud to reduce
    // the magnitude of all loaded points. Now that we have loaded all
    // points, shift one more time, to place the centroid of the
    // reference at the origin.
    // Note: If this code is ever converting to using floats,
    // the operation below needs to be re-implemented to be accurate.
    int numRefPts = ref_point_cloud.features.cols();
    Eigen::VectorXd meanRef = ref_point_cloud.features.rowwise().sum() / numRefPts;
    ref_point_cloud.features.topRows(DIM).colwise()    -= meanRef.head(DIM);
    for (int row = 0; row < DIM; row++)
      shift[row] += meanRef(row); // Update the shift variable as well as the points
    if (opt.verbose)
      vw_out() << "Data shifted internally by subtracting: " << shift << "\n";

    // The point clouds are shifted, so shift the initial transform as well.
    Eigen::MatrixXd shiftInitT = apply_shift(opt.init_transform, shift);

    // If the reference point cloud came from a DEM, also load the data in DEM format.
    cartography::GeoReference dem_georef;
    vw::ImageViewRef<PixelMask<float>> reference_dem_ref;
    if (opt.use_dem_distances()) {
      // Load the dem, then wrap it inside an ImageViewRef object. This is done
      // because the actual DEM type cannot be created without being
      // initialized.
      InterpolationReadyDem 
      reference_dem(load_interpolation_ready_dem(opt.reference, dem_georef));
      reference_dem_ref.reset(reference_dem);
    }

    // Filter the reference and initialize the reference tree
    double elapsed_time;
    PM::ICP icp; // libpointmatcher object

    Stopwatch sw3;
    if (opt.verbose)
      vw_out() << "Building the reference cloud tree.\n";
    sw3.start();
    icp.initRefTree(ref_point_cloud, alignment_method_fallback(opt.alignment_method),
            opt.highest_accuracy, false /*opt.verbose*/);
    sw3.stop();
    if (opt.verbose)
      vw_out() << "Reference point cloud processing took " << sw3.elapsed_seconds() << " s\n";

   // Load, filter, transform, and resample the source point cloud
   DP source_point_cloud;
   processSourceCloud(opt, ref_point_cloud, shiftInitT, dem_georef, geo, csv_conv, 
                      source_box, reference_dem_ref, is_lola_rdr_format, 
                      mean_source_longitude, icp, shift, 
                      source_point_cloud); // output

    // Make the libpointmatcher error message clearer
    std::string libpointmatcher_error = "no point to minimize";
    std::string pc_align_error = 
       "This likely means that the clouds are too far. Consider increasing the "
       "--max-displacement value to something somewhat larger than the expected "
       "length of the displacement that may be needed to align the clouds.\n";
    
    Eigen::MatrixXd beg_errors;
    try {
      elapsed_time = compute_registration_error(ref_point_cloud, source_point_cloud, icp,
                                                shift, dem_georef, reference_dem_ref,
                                                opt, beg_errors);
    } catch(std::exception const& e) {
      std::string error = e.what();
      if (error.find(libpointmatcher_error) != std::string::npos)
        error += ".\n" + pc_align_error; // clarify the error
      vw_throw(ArgumentErr() << error);
    }
    
    calc_stats("Input", beg_errors);
    if (opt.verbose)
      vw_out() << "Initial error computation took " << elapsed_time << " s\n";

    // Set up the ICP object
    Stopwatch sw4;
    sw4.start();
    bool verbose = false;
    Eigen::MatrixXd Id = Eigen::MatrixXd::Identity(DIM + 1, DIM + 1);
    icp.setParams(opt.out_prefix, opt.num_iter, opt.outlier_ratio,
                  (2.0*M_PI/360.0)*opt.diff_rotation_err, // convert to radians
                  opt.diff_translation_err, 
                  alignment_method_fallback(opt.alignment_method),
                  verbose);

    // Compute the transformation to align the source to reference.
    // We bypass calling ICP if the user explicitly asks for 0 iterations.
    Eigen::MatrixXd T = Id;
    if (opt.num_iter > 0) {
      if (opt.alignment_method == "nuth") {
        T = asp::nuthAlignment(opt.reference, opt.source, opt.out_prefix, 
                               opt.max_disp, opt.num_iter,
                               opt.num_threads, opt.compute_translation_only,
                               opt.nuth_options);
        // Nuth does not use the shift, so apply it after it is returned.
        T = apply_shift(T, shift);
      } else if (opt.alignment_method == "fgr") {
        T = fgr_alignment(source_point_cloud, ref_point_cloud, opt.fgr_options);
      } else if (opt.alignment_method == "point-to-plane" ||
                 opt.alignment_method == "point-to-point" ||
                 opt.alignment_method == "similarity-point-to-point" ||
                 opt.alignment_method == "similarity-point-to-plane") {
        // Use libpointmatcher
        try {
          T = icp(source_point_cloud, ref_point_cloud, Id, opt.compute_translation_only);
        } catch(std::exception const& e) {
          std::string error = e.what();
          if (error.find(libpointmatcher_error) != std::string::npos)
            error += ".\n" + pc_align_error; // clarify the error
          vw_throw(ArgumentErr() << error);
        }
        
        vw_out() << "Match ratio: "
                 << icp.errorMinimizer->getWeightedPointUsedRatio() << endl;
      } else if (opt.alignment_method == "least-squares" ||
                opt.alignment_method == "similarity-least-squares") {
        // Compute alignment using least squares
        T = least_squares_alignment(source_point_cloud, shift,
                                    dem_georef, reference_dem_ref, opt.alignment_method,
                                    opt.num_iter, opt.num_threads);
      } else
        vw_throw( ArgumentErr() << "Unknown alignment method: " << opt.alignment_method);
    }
    sw4.stop();
    if (opt.verbose)
      vw_out() << "Alignment took " << sw4.elapsed_seconds() << " s\n";

    // Transform the source to make it close to reference.
    DP trans_source_point_cloud(source_point_cloud);
    apply_transform_to_cloud(T, trans_source_point_cloud);

    // Calculate by how much points move as result of T
    double max_obtained_disp = calc_max_displacement(source_point_cloud, trans_source_point_cloud);
    Vector3 source_ctr_vec, source_ctr_llh;
    Vector3 trans_xyz, trans_ned, trans_llh;
    vw::Matrix3x3 NedToEcef;
    calc_translation_vec(shiftInitT, source_point_cloud, trans_source_point_cloud, shift,
                         geo.datum(), source_ctr_vec, source_ctr_llh,
                         trans_xyz, trans_ned, trans_llh, NedToEcef);

    // For each point, compute the distance to the nearest reference point.
    Eigen::MatrixXd end_errors;
    elapsed_time 
      = compute_registration_error(ref_point_cloud, trans_source_point_cloud, icp,
                                   shift, dem_georef, reference_dem_ref, opt, 
                                   end_errors);
    calc_stats("Output", end_errors);
    if (opt.verbose)
      vw_out() << "Final error computation took " << elapsed_time << " s\n";

    // Go back to the original coordinate system, by applying the shifted initial
    // transform to the shifted computed transform and then undoing the shift.
    Eigen::MatrixXd globalT = apply_shift(T * shiftInitT, -shift);

    // Print statistics
    vw_out() << std::setprecision(16)
             << "Alignment transform (origin is planet center):\n" << globalT << "\n";
    vw_out() << std::setprecision(8); // undo the higher precision

    vw_out() << "Centroid of source points (Cartesian, meters): " << source_ctr_vec << "\n";
    // Swap lat and lon, as we want to print lat first
    std::swap(source_ctr_llh[0], source_ctr_llh[1]);
    vw_out() << "Centroid of source points (lat,lon,z): " << source_ctr_llh << "\n";
    vw_out() << "\n";

    vw_out() << "Translation vector (Cartesian, meters): " << trans_xyz << "\n";
    vw_out() << "Translation vector (North-East-Down, meters): "
             << trans_ned << "\n";
    vw_out() << "Translation vector magnitude (meters): " << norm_2(trans_xyz)
             << "\n";
    vw::vw_out() << "Maximum displacement of points between the source "
                 << "cloud with any initial transform applied to it and the "
                 << "source cloud after alignment to the reference: " 
                 << max_obtained_disp << " m" << "\n";
    if (opt.max_disp > 0 && opt.max_disp < max_obtained_disp)
      vw_out() << "Warning: The input --max-displacement value is smaller than the "
               << "final observed displacement. It may be advised to increase the former "
               << "and rerun the tool.\n";

    // Swap lat and lon, as we want to print lat first
    std::swap(trans_llh[0], trans_llh[1]);
    vw_out() << "Translation vector (lat,lon,z): " << trans_llh << "\n";
    vw_out() << "\n";

    Matrix3x3 rot;
    for (int r = 0; r < DIM; r++)
      for (int c = 0; c < DIM; c++)
        rot(r, c) = globalT(r, c);

    double scale = pow(det(rot), 1.0/3.0);
    for (int r = 0; r < DIM; r++)
      for (int c = 0; c < DIM; c++)
        rot(r, c) /= scale;

    // Subtract one before printing the scale, to see a lot of digits of precision
    vw_out() << "Transform scale - 1 = " << (scale-1.0) << "\n";
    
    Matrix3x3 rot_NED = inverse(NedToEcef) * rot * NedToEcef;
   
    Vector3 euler_angles = math::rotation_matrix_to_euler_xyz(rot) * 180/M_PI;
    Vector3 euler_angles_NED = math::rotation_matrix_to_euler_xyz(rot_NED) * 180/M_PI;
    Vector3 axis_angles = math::matrix_to_axis_angle(rot) * 180/M_PI;
    vw_out() << "Euler angles (degrees): " << euler_angles  << endl;
    vw_out() << "Euler angles (North-East-Down, degrees): " << euler_angles_NED  << endl;
    vw_out() << "Axis of rotation and angle (degrees): "
             << axis_angles/norm_2(axis_angles) << ' '
             << norm_2(axis_angles) << endl;

    Stopwatch sw5;
    sw5.start();
    write_transforms(opt, globalT);

    if (opt.save_trans_ref) {
      string trans_ref_prefix = opt.out_prefix + "-trans_reference";
      save_trans_point_cloud(opt, opt.reference, trans_ref_prefix,
                             opt.ref_copc_win, opt.ref_copc_read_all,
                             geo, csv_conv, globalT.inverse());
    }

    if (opt.save_trans_source) {
      string trans_source_prefix = opt.out_prefix + "-trans_source";
      save_trans_point_cloud(opt, opt.source, trans_source_prefix,
                             opt.src_copc_win, opt.src_copc_read_all,
                             geo, csv_conv, globalT);
    }

    save_errors(source_point_cloud, beg_errors,  opt.out_prefix + "-beg_errors.csv",
                shift, geo, csv_conv, is_lola_rdr_format, mean_source_longitude);
    save_errors(trans_source_point_cloud, end_errors,  opt.out_prefix + "-end_errors.csv",
                shift, geo, csv_conv, is_lola_rdr_format, mean_source_longitude);

    if (opt.verbose) vw_out() << "Writing: " << opt.out_prefix
      + "-iterationInfo.csv" << "\n";

    sw5.stop();
    if (opt.verbose) vw_out() << "Saving to disk took "
                              << sw5.elapsed_seconds() << " s\n";

  } ASP_STANDARD_CATCHES;

  return 0;
}
