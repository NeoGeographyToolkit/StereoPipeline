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

/// \file jitter_adjust.cc
///
/// Use n adjustments for every camera, placed at several lines in the image
// with interpolation between them. The pdf doc has more info.

// TODO(oalexan1): Move some UsgsAstroLsSensorModel functions from
// here and from LinescanDGModel.cc to CsmUtils.cc.

// TODO(oalexan1): Add two passes and outlier filtering. For now
// try to use clean matches.

#include <asp/Sessions/StereoSessionFactory.h>
#include <asp/Sessions/CameraUtils.h>
#include <asp/Camera/CsmModel.h>
#include <asp/Camera/CsmUtils.h>
#include <asp/Camera/BundleAdjustCamera.h>
#include <asp/Camera/BundleAdjustIsis.h>
#include <asp/Camera/JitterSolveCostFuns.h>
#include <asp/Camera/JitterSolveUtils.h>
#include <asp/Camera/LinescanUtils.h>
#include <asp/Camera/BundleAdjustResiduals.h>
#include <asp/Core/Macros.h>
#include <asp/Core/Common.h>
#include <asp/Core/StereoSettings.h>
#include <asp/Core/BundleAdjustUtils.h>
#include <asp/Core/IpMatchingAlgs.h> // Lightweight header for matching algorithms
#include <asp/Core/SatSimBase.h>
#include <asp/Core/CameraTransforms.h>
#include <asp/Core/ImageUtils.h>
#include <asp/IsisIO/IsisInterface.h>

#include <vw/BundleAdjustment/ControlNetwork.h>
#include <vw/BundleAdjustment/ControlNetworkLoader.h>
#include <vw/Core/Stopwatch.h>
#include <vw/Cartography/CameraBBox.h>
#include <vw/Cartography/GeoReferenceBaseUtils.h>
#include <vw/Camera/CameraImage.h>
#include <usgscsm/UsgsAstroLsSensorModel.h>
#include <usgscsm/UsgsAstroFrameSensorModel.h>
#include <usgscsm/Utilities.h>

#include <xercesc/util/PlatformUtils.hpp>

#include <ceres/ceres.h>
#include <ceres/loss_function.h>

namespace po = boost::program_options;
namespace fs = boost::filesystem;

using namespace vw;
using namespace vw::math;

namespace asp {

struct Options: public asp::BaBaseOptions {
  int num_lines_per_position, num_lines_per_orientation, num_anchor_points_per_image,
    num_anchor_points_per_tile;
  std::string anchor_weight_image;   
  double quat_norm_weight, anchor_weight, roll_weight, yaw_weight;
  std::string anchor_dem;
  int num_anchor_points_extra_lines;
  bool initial_camera_constraint;
  std::map<int, int> orbital_groups;
  double forced_triangulation_distance;
};
    
void handle_arguments(int argc, char *argv[], Options& opt) {

  po::options_description general_options("");
  general_options.add_options()
    ("output-prefix,o",  po::value(&opt.out_prefix), "Prefix for output filenames.")
    ("session-type,t",   po::value(&opt.stereo_session)->default_value(""),
     "Select the stereo session type to use for processing. Usually the program "
     "can select this automatically by the file extension, except for xml cameras. "
     "See the doc for options.")
    ("input-adjustments-prefix",  po::value(&opt.input_prefix),
     "Prefix to read initial adjustments from, written by bundle_adjust. "
     "Not required. Cameras in .json files in ISD or model state format "
     "can be passed in with no adjustments.")
    ("num-lines-per-position", po::value(&opt.num_lines_per_position)->default_value(-1),
     "Resample the input camera positions and velocities, using this many lines per "
     "produced position and velocity. If not set, use the positions and velocities "
     "from the CSM file as they are.")
    ("num-lines-per-orientation", po::value(&opt.num_lines_per_orientation)->default_value(-1),
     "Resample the input camera orientations, using this many lines per produced orientation. "
     "If not set, use the orientations from the CSM file as they are.")
    ("match-first-to-last",
     po::bool_switch(&opt.match_first_to_last)->default_value(false)->implicit_value(true),
     "Match first several images to last several images by extending the logic of "
     "--overlap-limit past the last image to the earliest ones.")
    ("overlap-limit",        po::value(&opt.overlap_limit)->default_value(0),
     "Limit the number of subsequent images to search for matches to the current image "
     "to this value. By default match all images.")
    ("match-files-prefix",  po::value(&opt.match_files_prefix)->default_value(""),
     "Use the match files from this prefix. The order of images in each interest point "
     "match file need not be the same as for input images.")
    ("clean-match-files-prefix",  po::value(&opt.clean_match_files_prefix)->default_value(""),
     "Use as input match files the *-clean.match files from this prefix. The order of "
     "images in each interest point match file need not be the same as for input images.")
    ("isis-cnet", po::value(&opt.isis_cnet)->default_value(""),
     "Read a control network having interest point matches from this binary file "
     "in the ISIS jigsaw format. This can be used with any images and cameras "
     "supported by ASP.")
    ("min-matches", po::value(&opt.min_matches)->default_value(30),
     "Set the minimum  number of matches between images that will be considered.")
    ("max-pairwise-matches", po::value(&opt.max_pairwise_matches)->default_value(10000),
     "Reduce the number of matches per pair of images to at most this "
     "number, by selecting a random subset, if needed. This happens "
     "when setting up the optimization, and before outlier filtering.")
    ("min-triangulation-angle", po::value(&opt.min_triangulation_angle)->default_value(0.1),
     "The minimum angle, in degrees, at which rays must meet at a triangulated point to "
     "accept this point as valid. It must be a positive value.")
    ("max-initial-reprojection-error", po::value(&opt.max_init_reproj_error)->default_value(10),
     "Filter as outliers triangulated points project using initial cameras with error more than "
     "this, measured in pixels. Since jitter corrections are supposed to be small and cameras "
     "bundle-adjusted by now, this value need not be too big.")
    ("robust-threshold", po::value(&opt.robust_threshold)->default_value(0.5),
     "Set the threshold for the Cauchy robust cost function. Increasing this makes "
     "the solver focus harder on the larger errors.")
    ("image-list", po::value(&opt.image_list)->default_value(""),
     "A file containing the list of images, when they are too many to specify on the command line. Use space or newline as separator. See also --camera-list.")
    ("camera-list", po::value(&opt.camera_list)->default_value(""),
     "A file containing the list of cameras, when they are too many to specify on "
     "the command line. If the images have embedded camera information, such as for ISIS, "
     "this file may be omitted, or specify the image names instead of camera names.")
    ("parameter-tolerance",  po::value(&opt.parameter_tolerance)->default_value(1e-12),
     "Stop when the relative error in the variables being optimized is less than this.")
    ("num-iterations",       po::value(&opt.num_iterations)->default_value(500),
     "Set the maximum number of iterations.")
    ("tri-weight", po::value(&opt.tri_weight)->default_value(0.1),
     "The weight to give to the constraint that optimized triangulated points stay "
      "close to original triangulated points. A positive value will help ensure the "
      "cameras do not move too far, but a large value may prevent convergence. It is "
      "suggested to use here 0.1 to 0.5. This will be divided by ground sample distance "
      "(GSD) to convert this constraint to pixel units, since the reprojection errors "
      "are in pixels. See also --tri-robust-threshold. Does not apply to GCP or points "
      "constrained by a DEM.")
    ("tri-robust-threshold",
     po::value(&opt.tri_robust_threshold)->default_value(0.1),
     "The robust threshold to attenuate large differences between initial and "
     "optimized triangulation points, after multiplying them by --tri-weight and "
     "dividing by GSD. This is less than --robust-threshold, as the primary goal "
     "is to reduce pixel reprojection errors, even if that results in big differences "
      "in the triangulated points. It is suggested to not modify this value, "
      "and adjust instead --tri-weight.")
    ("heights-from-dem",   po::value(&opt.heights_from_dem)->default_value(""),
     "Assuming the cameras have already been bundle-adjusted and aligned to a "
     "known DEM, constrain the triangulated points to be close to this DEM. See also "
     "--heights-from-dem-uncertainty.")
    ("heights-from-dem-uncertainty", 
     po::value(&opt.heights_from_dem_uncertainty)->default_value(10.0),
     "The DEM uncertainty (1 sigma, in meters). A smaller value constrain more the "
     "triangulated points to the DEM specified via --heights-from-dem.")
    ("heights-from-dem-robust-threshold",
     po::value(&opt.heights_from_dem_robust_threshold)->default_value(0.1),
     "The robust threshold to use keep the triangulated points close to the DEM if "
      "specified via --heights-from-dem. This is applied after the point differences "
      "are divided by --heights-from-dem-uncertainty. It will attenuate large height "
      "difference outliers. It is suggested to not modify this value, and adjust instead "
      "--heights-from-dem-uncertainty.")
    ("num-anchor-points", po::value(&opt.num_anchor_points_per_image)->default_value(0),
     "How many anchor points to create per image. They will be uniformly distributed.")
    ("num-anchor-points-per-tile", po::value(&opt.num_anchor_points_per_tile)->default_value(0),
     "How many anchor points to create per 1024 x 1024 image tile. They will "
      "be uniformly distributed. Useful when images of vastly different sizes "
      "(such as frame and linescan) are used together.")
    ("anchor-weight", po::value(&opt.anchor_weight)->default_value(0.0),
     "How much weight to give to each anchor point. Anchor points are "
     "obtained by intersecting rays from initial cameras with the DEM given by "
     "--heights-from-dem. A larger weight will make it harder for "
     "the cameras to move, hence preventing unreasonable changes. "
     "Set also --anchor-weight and --anchor-dem.")
    ("anchor-dem",  po::value(&opt.anchor_dem)->default_value(""),
     "Use this DEM to create anchor points.")
    ("num-anchor-points-extra-lines",
     po::value(&opt.num_anchor_points_extra_lines)->default_value(0),
     "Start placing anchor points this many lines before first image line "
     "and after last image line. Applies only to linescan cameras.")
    ("rotation-weight", po::value(&opt.rotation_weight)->default_value(0.0),
     "A higher weight will penalize more deviations from the original camera orientations.")
    ("camera-position-weight", po::value(&opt.camera_position_weight)->default_value(0.0),
     "A soft constraint to keep the camera positions close to the original values. "
     "It is meant to prevent a wholesale shift of the cameras. It can impede "
     "the reduction in reprojection errors. It adjusts to the ground sample distance "
     "and the number of interest points in the images. The computed "
     "discrepancy is attenuated with --camera-position-robust-threshold.")
    ("camera-position-robust-threshold", 
     po::value(&opt.camera_position_robust_threshold)->default_value(0.1),
     "The robust threshold to attenuate large discrepancies between initial and "
     "optimized camera positions with the option --camera-position-weight. "
     "This is less than --robust-threshold, as the primary goal "
     "is to reduce pixel reprojection errors, even if that results in big differences "
     "in the camera positions. It is suggested to not modify this value, "
     "and adjust instead --camera-position-weight.")
    ("quat-norm-weight", po::value(&opt.quat_norm_weight)->default_value(1.0),
     "How much weight to give to the constraint that the norm of each quaternion must be 1.")
    ("roll-weight", po::value(&opt.roll_weight)->default_value(0.0),
     "A weight to penalize the deviation of camera roll orientation as measured from the "
     "along-track direction. Pass in a large value, such as 1e+5. This is best used only with "
     "linescan cameras created with sat_sim.")
    ("yaw-weight", po::value(&opt.yaw_weight)->default_value(0.0),
     "A weight to penalize the deviation of camera yaw orientation as measured from the "
     "along-track direction. Pass in a large value, such as 1e+5. This is best used only "
     "with linescan cameras created with sat_sim.")
    ("weight-image", po::value(&opt.weight_image)->default_value(""),
     "Given a georeferenced image with float values, for each initial triangulated "
     "point find its location in the image and closest pixel value. Multiply the "
     "reprojection errors in the cameras for this point by this weight value. The solver "
     "will focus more on optimizing points with a higher weight. Points that fall "
     "outside the image and weights that are non-positive, NaN, or equal to nodata "
     "will be ignored.")
     ("anchor-weight-image", po::value(&opt.anchor_weight_image)->default_value(""),
     "Weight image for anchor points. Limits where anchor points are placed and their weight. "
     "These weights are additionally multiplied by --anchor-weight. See also --weight-image.")
    ("ip-side-filter-percent",  po::value(&opt.ip_edge_buffer_percent)->default_value(-1.0),
     "Remove matched IPs this percentage from the image left/right sides.")
    ("forced-triangulation-distance", 
     po::value(&opt.forced_triangulation_distance)->default_value(-1),
     "When triangulation fails, for example, when input cameras are inaccurate, "
     "artificially create a triangulation point this far ahead of the camera, in units "
     "of meter.")
    ("update-isis-cubes-with-csm-state", 
     po::bool_switch(&opt.update_isis_cubes_with_csm_state)->default_value(false)->implicit_value(true),
     "Save the model state of optimized CSM cameras as part of the .cub files. Any prior "
     "version and any SPICE data will be deleted. Mapprojected images obtained with prior "
     "version of the cameras must no longer be used in stereo.")
    ("initial-camera-constraint", 
     po::bool_switch(&opt.initial_camera_constraint)->default_value(false),
     "When constraining roll and yaw, measure these not in the satellite along-track/ "
     "across-track/down coordinate system, but relative to the initial camera poses. This "
     "is experimental. Internally, the roll weight will then be applied to the camera "
     "pitch angle (rotation around the camera y axis), because the camera coordinate "
     "system is rotated by 90 degrees in the sensor plane relative to the satellite "
     "coordinate system. The goal is the same, to penalize deviations that are not "
     "aligned with satellite pitch.")
    ;

    general_options.add(vw::GdalWriteOptionsDescription(opt));

  // TODO(oalexan1): This old option may need to be wiped given the newer
  // recent outlier filtering.
  asp::stereo_settings().ip_edge_buffer_percent = opt.ip_edge_buffer_percent;

  po::options_description positional("");
  positional.add_options()
    ("input-files", po::value(&opt.image_files));
  
  po::positional_options_description positional_desc;
  positional_desc.add("input-files", -1);

  std::string usage("<images> <cameras> -o <output prefix> [options]");
  bool allow_unregistered = false;
  std::vector<std::string> unregistered;
  po::variables_map vm =
    asp::check_command_line(argc, argv, opt, general_options, general_options,
                            positional, positional_desc, usage,
                            allow_unregistered, unregistered);

  // Do this check first, as the output prefix is used below many times
  if (opt.out_prefix == "") 
    vw_throw(ArgumentErr() << "Must specify the output prefix.\n" << usage << "\n");

  // Create the output directory
  vw::create_out_dir(opt.out_prefix);

  // Turn on logging to file
  asp::log_to_file(argc, argv, "", opt.out_prefix);

  // Set this before loading cameras, as jitter can be modeled only with CSM
  // cameras.
  asp::stereo_settings().aster_use_csm = true;
  
  if (!opt.image_list.empty()) {
    // Read the images and cameras and put them in 'images_or_cams' to be parsed later
    if (!opt.image_files.empty())
      vw_throw(ArgumentErr() << "The option --image-list was specified, but also "
               << "images or cameras on the command line.\n");
    asp::IntrinsicOptions intr_opts;
    read_image_cam_lists(opt.image_list, opt.camera_list, 
      opt.image_files, opt.camera_files, intr_opts); // outputs
    if (intr_opts.num_sensors != 0 || !intr_opts.cam2sensor.empty()) 
      vw::vw_throw(vw::ArgumentErr() << "Cannot handle intrinsics with jitter_solve.\n");
  } else {
    std::vector<std::string> images_or_cams = opt.image_files;
    bool ensure_equal_sizes = true;
    asp::separate_images_from_cameras(images_or_cams,
                                      opt.image_files, opt.camera_files, // outputs
                                      ensure_equal_sizes); 

    // This is needed when several images are acquired in quick succession
    // and we want to impose roll and yaw constraints given their orbital 
    // trajectory.
    asp::readGroupStructure(images_or_cams, opt.orbital_groups);
  }
  
  // Throw if there are duplicate camera file names.
  asp::check_for_duplicates(opt.image_files, opt.camera_files, opt.out_prefix);
  
  const int num_images = opt.image_files.size();
  
  // Sanity check
  if (opt.image_files.size() != opt.camera_files.size())
    vw_throw(ArgumentErr() << "Must have as many cameras as  have images.\n");
  
  if (opt.image_files.empty())
    vw_throw(ArgumentErr() << "Missing input image files.\n");
  
  if (opt.overlap_limit < 0)
    vw_throw(ArgumentErr() << "Must allow search for matches between "
             << "at least each image and its subsequent one.\n");
  
  // By default, try to match all of the images
  if (opt.overlap_limit == 0)
    opt.overlap_limit = opt.image_files.size();
  
  if (int(!opt.match_files_prefix.empty()) + int(!opt.clean_match_files_prefix.empty())
      + int(!opt.isis_cnet.empty()) != 1) 
    vw_throw(ArgumentErr() << "Must specify precisely one of: --match-files-prefix, "
             << "--clean-match-files-prefix, --isis-cnet.\n");

  if (opt.max_init_reproj_error <= 0.0)
    vw_throw(ArgumentErr() << "Must have a positive --max-initial-reprojection-error.\n");

  if (opt.tri_weight < 0.0) 
    vw_throw(ArgumentErr() << "The value of --tri-weight must be non-negative.\n");

  if (opt.robust_threshold <= 0.0) 
    vw_throw(ArgumentErr() << "The value of --robust-threshold must be positive.\n");

  if (opt.tri_robust_threshold <= 0.0) 
    vw_throw(ArgumentErr() << "The value of --tri-robust-threshold must be positive.\n");
  
  if (opt.heights_from_dem_uncertainty <= 0.0) 
    vw_throw(ArgumentErr() << "The value of --heights-from-dem-uncertainty must be positive.\n");
  
  if (opt.heights_from_dem_robust_threshold <= 0.0) 
    vw_throw(ArgumentErr() << "The value of --heights-from-robust-threshold must be positive.\n");

  if (opt.rotation_weight < 0)
    vw_throw(ArgumentErr() << "Rotation weight must be non-negative.\n");
  
  if (opt.camera_position_weight < 0) 
    vw_throw(ArgumentErr() << "The value of --camera-position-weight must be n"
                           << "non-negative.\n");
    
  if (opt.camera_position_robust_threshold <= 0.0)
    vw_throw(ArgumentErr() << "The value of --camera-position-robust-threshold "
                            << "must be positive.\n");
      
  if (opt.quat_norm_weight <= 0)
    vw_throw(ArgumentErr() << "Quaternion norm weight must be positive.\n");

  if (opt.roll_weight < 0.0)
    vw_throw(ArgumentErr() << "Roll weight must be non-negative.\n");

  if (opt.yaw_weight < 0.0)
    vw_throw(ArgumentErr() << "Yaw weight must be non-negative.\n");

  // Handle the roll/yaw constraint DEM
  if ((opt.roll_weight > 0 || opt.yaw_weight > 0) &&
     opt.heights_from_dem == "" && opt.anchor_dem == "")
      vw::vw_throw(ArgumentErr() << "Cannot use the roll/yaw constraint without a DEM. "
        << "Set either --heights-from-dem or --anchor-dem.\n");

  if (opt.num_anchor_points_per_image < 0)
    vw_throw(ArgumentErr() << "The number of anchor points must be non-negative.\n");
  if (opt.num_anchor_points_per_tile < 0)
    vw_throw(ArgumentErr() << "The number of anchor points per tile must be non-negative.\n");

  // Cannot have anchor points both per image and per tile
  if (opt.num_anchor_points_per_image > 0 && opt.num_anchor_points_per_tile > 0)
    vw_throw(ArgumentErr() << "Cannot have anchor points both per image and per tile.\n");
    
  if (opt.anchor_weight < 0)
    vw_throw(ArgumentErr() << "Anchor weight must be non-negative.\n");

  if (opt.anchor_weight > 0 && opt.anchor_dem.empty()) 
    vw::vw_throw(vw::ArgumentErr() << "If --anchor-weight is positive, set --anchor-dem.\n");
  
  // Turn on logging to file
  asp::log_to_file(argc, argv, "", opt.out_prefix);
  
  return;
}

// Calc the time of first image line, last image line, elapsed time
// between these lines, and elapsed time per line.  This assumes a
// linear relationship between lines and time.
// TODO(oalexan1): This is fragile. Maybe it can be avoided.
void calcTimes(UsgsAstroLsSensorModel const* ls_model,
               double & earlier_line_time, double & later_line_time,
               double & elapsed_time, double & dt_per_line) {

  int numLines = ls_model->m_nLines;
  csm::ImageCoord imagePt;

  asp::toCsmPixel(vw::Vector2(0, 0), imagePt);
  earlier_line_time = ls_model->getImageTime(imagePt);

  asp::toCsmPixel(vw::Vector2(0, numLines - 1), imagePt);
  later_line_time = ls_model->getImageTime(imagePt);

  // See note in resampleModel().
  if (earlier_line_time > later_line_time)
    std::swap(earlier_line_time, later_line_time);
  
  elapsed_time = later_line_time - earlier_line_time;
  dt_per_line = elapsed_time / (numLines - 1.0);

  if (later_line_time <= earlier_line_time)
    vw::vw_throw(vw::ArgumentErr()
                 << "The time of the last line (in scanning order) must be larger than "
                 << "first line time.\n");
  
  return;
}

// Calculate the line index for first and last tabulated position.
// We always expect these to be less than first line index (0), and no less
// than last valid image line index (numLines - 1), respectively.
// TODO(oalexan1): This assumes a linear relationship between time and lines,
// which is fragile. At least need to check that this assumption is satisfied.
void calcFirstLastPositionLines(UsgsAstroLsSensorModel const* ls_model, 
                                double & beg_position_line, double & end_position_line) {

  double earlier_line_time = -1.0, later_line_time = -1.0, 
         elapsed_time = -1.0, dt_per_line = -1.0;
  calcTimes(ls_model, earlier_line_time, later_line_time, elapsed_time,  
               dt_per_line);
  
  // Find time of first and last tabulated position.
  double bt = ls_model->m_t0Ephem;
  double et = bt + (ls_model->m_positions.size()/NUM_XYZ_PARAMS - 1) * ls_model->m_dtEphem;

  // Use the equation: time = earlier_line_time + line * dt_per_line.
  // See note in resampleModel() about scan direction.
  beg_position_line = (bt - earlier_line_time) / dt_per_line;
  end_position_line = (et - earlier_line_time) / dt_per_line;

  // Sanity checks
  if (beg_position_line > 1e-3) // allow for rounding errors 
    vw::vw_throw(vw::ArgumentErr() << "Line of first tabulated position is "
                 << beg_position_line << ", which is after first image line, which is "
                 << 0 << ".\n");
  int numLines = ls_model->m_nLines;
  if (end_position_line < numLines - 1 - 1e-3)  // allow for rounding errors
    vw::vw_throw(vw::ArgumentErr() << "Line of last tabulated position is "
                 << end_position_line << ", which is before last image line, which is "
                 << numLines - 1 << ".\n");
}
  
// Calculate the line index for first and last tabulated orientation.
// We always expect these to be less than first line index (0), and no less
// than last valid image line index (numLines - 1), respectively.
void calcFirstLastOrientationLines(UsgsAstroLsSensorModel const* ls_model, 
                                   double & beg_orientation_line, double & end_orientation_line) {

  double earlier_line_time = -1.0, later_line_time = -1.0, 
         elapsed_time = -1.0, dt_per_line = -1.0;
  calcTimes(ls_model, earlier_line_time, later_line_time, elapsed_time,  
               dt_per_line);
  
  // Find time of first and last tabulated orientation.
  double bt = ls_model->m_t0Quat;
  double et = bt + (ls_model->m_quaternions.size()/NUM_QUAT_PARAMS - 1) * ls_model->m_dtQuat;
  
  // Use the equation: time = earlier_line_time + line * dt_per_line.
  beg_orientation_line = (bt - earlier_line_time) / dt_per_line;
  end_orientation_line = (et - earlier_line_time) / dt_per_line;

  // Sanity checks
  if (beg_orientation_line > 1e-3) // allow for rounding errors 
    vw::vw_throw(vw::ArgumentErr() << "Line of first tabulated orientation is "
                 << beg_orientation_line << ", which is after first image line, which is "
                   << 0 << ".\n");
  int numLines = ls_model->m_nLines;
  if (end_orientation_line < numLines - 1 - 1e-3)  // allow for rounding errors
    vw::vw_throw(vw::ArgumentErr() << "Line of last tabulated orientation is "
                 << end_orientation_line << ", which is before last image line, which is "
                   << numLines - 1 << ".\n");
}

// TODO(oalexan1): Move the function below out of here, to CsmUtils.cc.
// The provided tabulated positions, velocities and quaternions may be too few,
// so resample them with --num-lines-per-position and --num-lines-per-orientation,
// if those are set. Throughout this function the lines are indexed in the order
// they are acquired, which can be the reverse of the order they are eventually
// stored in the file if the scan direction is reverse.
void resampleModel(Options const& opt, UsgsAstroLsSensorModel * ls_model) {
  
  // The positions and quaternions can go way beyond the valid range of image lines,
  // so need to estimate how many of them are within the range.
  
  int numLines = ls_model->m_nLines;
  vw_out() << "Number of lines: " << numLines << ".\n";

  double earlier_line_time = -1.0, later_line_time = -1.0, elapsed_time = -1.0, dt_per_line = -1.0;
  calcTimes(ls_model, earlier_line_time, later_line_time, elapsed_time,  
            dt_per_line);

  // Line index of first and last tabulated position
  double beg_position_line = -1.0, end_position_line = -1.0;
  calcFirstLastPositionLines(ls_model, beg_position_line, end_position_line);
  vw_out() << std::setprecision (17) << "Line of first and last tabulated position: "
           << beg_position_line << ' ' << end_position_line << "\n";

  // Line index of first and last tabulated orientation
  double beg_orientation_line = -1.0, end_orientation_line = -1.0;
  calcFirstLastOrientationLines(ls_model, beg_orientation_line, end_orientation_line);
  vw_out() << std::setprecision (17) << "Line of first and last tabulated orientation: "
           << beg_orientation_line << ' ' << end_orientation_line << "\n";

  double numInputLinesPerPosition = (numLines - 1) * ls_model->m_dtEphem / elapsed_time;
  double numInputLinesPerOrientation = (numLines - 1) * ls_model->m_dtQuat / elapsed_time;
  vw_out() << "Number of image lines per input position: "
           << round(numInputLinesPerPosition) << "\n";
  vw_out() << "Number of image lines per input orientation: "
           << round(numInputLinesPerOrientation) << "\n";

  if (opt.num_lines_per_position > 0) {
    // Resample in such a way that first and last samples are preserved. This is tricky.
    double posFactor = double(numInputLinesPerPosition) / double(opt.num_lines_per_position);
    if (posFactor <= 0.0)
      vw::vw_throw(vw::ArgumentErr() << "Invalid image.\n");

    int numOldMeas = ls_model->m_numPositions / NUM_XYZ_PARAMS;
    int numNewMeas = round(posFactor * (numOldMeas - 1.0)) + 1; // careful here
    numNewMeas = std::max(numNewMeas, 2);

    posFactor = double(numNewMeas - 1.0) / double(numOldMeas - 1.0);
    double currDtEphem = ls_model->m_dtEphem / posFactor;
    double numLinesPerPosition = (numLines - 1.0) * currDtEphem / elapsed_time;
    vw_out() << "Resampled number of lines per position: "
             << numLinesPerPosition << "\n";
    std::vector<double> positions(NUM_XYZ_PARAMS * numNewMeas, 0);
    std::vector<double> velocities(NUM_XYZ_PARAMS * numNewMeas, 0);
    for (int ipos = 0; ipos < numNewMeas; ipos++) {
      double time = ls_model->m_t0Ephem + ipos * currDtEphem;
      asp::interpPositions(ls_model, time, &positions[NUM_XYZ_PARAMS * ipos]);
      asp::interpVelocities(ls_model, time, &velocities[NUM_XYZ_PARAMS * ipos]);
    }
    
    // Overwrite in the model. Time of first tabulated position does not change.
    ls_model->m_dtEphem = currDtEphem;
    ls_model->m_numPositions = positions.size();
    ls_model->m_positions = positions;
    ls_model->m_velocities = velocities;

    // Sanity check
    double new_beg_position_line = -1.0, new_end_position_line = -1.0;
    calcFirstLastPositionLines(ls_model, new_beg_position_line, new_end_position_line);
    if (std::abs(beg_position_line - new_beg_position_line) > 1.0e-3 ||
        std::abs(end_position_line - new_end_position_line) > 1.0e-3)
      vw::vw_throw(vw::ArgumentErr() << "Bookkeeping failure. Resampling was done "
                   << "without preserving first and last tabulated position time.\n");
  }

  if (opt.num_lines_per_orientation > 0) {
    // Resample in such a way that first and last samples are preserved. This is tricky.
    double posFactor = double(numInputLinesPerOrientation) / double(opt.num_lines_per_orientation);
    if (posFactor <= 0.0)
      vw::vw_throw(vw::ArgumentErr() << "Invalid image.\n");

    int numOldMeas = ls_model->m_numQuaternions / NUM_QUAT_PARAMS;
    int numNewMeas = round(posFactor * (numOldMeas - 1.0)) + 1; // careful here
    numNewMeas = std::max(numNewMeas, 2);

    posFactor = double(numNewMeas - 1.0) / double(numOldMeas - 1.0);
    double currDtQuat = ls_model->m_dtQuat / posFactor;
    double numLinesPerOrientation = (numLines - 1.0) * currDtQuat / elapsed_time;
    vw_out() << "Resampled number of lines per orientation: "
             << numLinesPerOrientation << "\n";
    std::vector<double> quaternions(NUM_QUAT_PARAMS * numNewMeas, 0);
    for (int ipos = 0; ipos < numNewMeas; ipos++) {
      double time = ls_model->m_t0Quat + ipos * currDtQuat;
      asp::interpQuaternions(ls_model, time, &quaternions[NUM_QUAT_PARAMS * ipos]);
    }
    
    // Overwrite in the model. Time of first tabulated orientation does not change.
    ls_model->m_dtQuat = currDtQuat;
    ls_model->m_numQuaternions = quaternions.size();
    ls_model->m_quaternions = quaternions;

    // Sanity check
    double new_beg_orientation_line = -1.0, new_end_orientation_line = -1.0;
    calcFirstLastOrientationLines(ls_model, new_beg_orientation_line, new_end_orientation_line);
    if (std::abs(beg_orientation_line - new_beg_orientation_line) > 1.0e-3 ||
        std::abs(end_orientation_line - new_end_orientation_line) > 1.0e-3)
      vw::vw_throw(vw::ArgumentErr() << "Bookkeeping failure. Resampling was done "
                   << "without preserving first and last tabulated orientation time.\n");
  }

  return;
}

// Calculate a set of anchor points uniformly distributed over the image
// Will use opt.num_anchor_points_extra_lines. We append to weight_vec and
// other quantities that were used for reprojection errors for match points.
void calcAnchorPoints(Options                         const & opt,
                      ImageViewRef<PixelMask<double>>         interp_anchor_dem,
                      vw::cartography::GeoReference   const & anchor_georef,
                      std::vector<asp::CsmModel*>     const & csm_models,
                      // Append to these, they already have entries
                      std::vector<std::vector<Vector2>>     & pixel_vec,
                      std::vector<std::vector<double>>      & weight_vec,
                      std::vector<std::vector<int>>         & isAnchor_vec,
                      std::vector<std::vector<int>>         & pix2xyz_index,
                      std::vector<double>                   & orig_tri_points_vec,
                      std::vector<double>                   & tri_points_vec) {

  if (opt.num_anchor_points_per_image <= 0 && opt.num_anchor_points_per_tile <= 0)
    vw::vw_throw(vw::ArgumentErr() << "Expecting a positive number of anchor points.\n");

  // If to use an anchor weight image
  bool have_anchor_weight_image = (!opt.anchor_weight_image.empty());
  vw::ImageViewRef<vw::PixelMask<float>> anchor_weight_image;
  float anchor_weight_image_nodata = -std::numeric_limits<float>::max();
  vw::cartography::GeoReference anchor_weight_image_georef;
  if (have_anchor_weight_image)
    vw::cartography::readGeorefImage(opt.anchor_weight_image,
      anchor_weight_image_nodata, anchor_weight_image_georef, anchor_weight_image);

  int num_cams = opt.camera_models.size();
  for (int icam = 0; icam < num_cams; icam++) {
    
    vw::Vector2 dims = vw::file_image_size(opt.image_files[icam]);
    int numLines   = dims[1];
    int numSamples = dims[0];
    int extra = opt.num_anchor_points_extra_lines;
    {
      UsgsAstroLsSensorModel * ls_model
        = dynamic_cast<UsgsAstroLsSensorModel*>((csm_models[icam]->m_gm_model).get());
      if (ls_model == NULL)
         extra = 0; // extra lines are only for linescan
    }

    // Find how much image area will be taken by each anchor point  
    // Convert to double early on to avoid integer overflow
    double area = double(numSamples) * double(numLines + 2 * extra);
    double area_per_point = 0.0;
    if (opt.num_anchor_points_per_image > 0)
      area_per_point = area / double(opt.num_anchor_points_per_image);
    else
      area_per_point = 1024.0 * 1024.0 / double(opt.num_anchor_points_per_tile);

    double bin_len = sqrt(area_per_point);
    bin_len = std::max(bin_len, 1.0);
    int lenx = ceil(double(numSamples) / bin_len); lenx = std::max(1, lenx);
    int leny = ceil(double(numLines + 2 * extra) / bin_len); leny = std::max(1, leny);

    int numAnchorPoints = 0;
    for (int binx = 0; binx <= lenx; binx++) {
      double posx = binx * bin_len;
      for (int biny = 0; biny <= leny; biny++) {
        double posy = biny * bin_len - extra;
        
        if (posx > numSamples - 1 || posy < -extra || posy > numLines - 1 + extra) 
          continue;
        
        Vector2 pix(posx, posy);
        Vector3 xyz_guess(0, 0, 0);
        
        bool treat_nodata_as_zero = false;
        bool has_intersection = false;
        double height_error_tol = 0.001; // 1 mm should be enough
        double max_abs_tol      = 1e-14; // abs cost fun change b/w iterations
        double max_rel_tol      = 1e-14;
        int num_max_iter        = 50;   // Using many iterations can be very slow
          
        Vector3 dem_xyz = vw::cartography::camera_pixel_to_dem_xyz
          (opt.camera_models[icam]->camera_center(pix),
           opt.camera_models[icam]->pixel_to_vector(pix),
           interp_anchor_dem, anchor_georef, treat_nodata_as_zero, has_intersection,
           height_error_tol, max_abs_tol, max_rel_tol, num_max_iter, xyz_guess);

        if (!has_intersection || dem_xyz == Vector3())
          continue;

        Vector2 pix_out;
        try {
          pix_out = opt.camera_models[icam]->point_to_pixel(dem_xyz);
        } catch (...) {
          continue;
        }
        
        if (norm_2(pix - pix_out) > 10 * height_error_tol)
          continue; // this is likely a bad point

        // If we have a weight image, use it to multiply the weight
        double anchor_weight_from_image = 1.0;
        if (have_anchor_weight_image) {
          vw::PixelMask<float> img_wt 
            = vw::cartography::closestPixelVal(anchor_weight_image, 
                                               anchor_weight_image_georef, 
                                               dem_xyz);
          
          // Skip bad weights
          if (!is_valid(img_wt) || std::isnan(img_wt.child()) || img_wt.child() <= 0.0) 
            continue;
          
          anchor_weight_from_image = img_wt.child();
        }

        pixel_vec[icam].push_back(pix);
        weight_vec[icam].push_back(opt.anchor_weight * anchor_weight_from_image);
        isAnchor_vec[icam].push_back(1);
        
        // The current number of points in tri_points_vec is the index of the next point
        pix2xyz_index[icam].push_back(tri_points_vec.size() / 3);

        // Append every coordinate of dem_xyz to tri_points_vec
        for (int it = 0; it < 3; it++) {
          orig_tri_points_vec.push_back(dem_xyz[it]);
          tri_points_vec.push_back(dem_xyz[it]);
        }
          
        numAnchorPoints++;
      }   
    }

    vw_out() << std::endl;
    vw_out() << "Image file: " << opt.image_files[icam] << std::endl;
    vw_out() << "Lines and samples: " << numLines << ' ' << numSamples << std::endl;
    vw_out() << "Num anchor points per image: " << numAnchorPoints     << std::endl;
  }   
}

// Add the linescan model reprojection error to the cost function
void addLsReprojectionErr(Options          const & opt,
                          UsgsAstroLsSensorModel * ls_model,
                          vw::Vector2      const & observation,
                          double                 * tri_point,
                          double                   weight,
                          ceres::Problem         & problem) {

  // Must grow the number of quaternions and positions a bit
  // because during optimization the 3D point and corresponding
  // pixel may move somewhat.
  double line_extra = opt.max_init_reproj_error + 5.0; // add some more just in case
  csm::ImageCoord imagePt1, imagePt2;
  asp::toCsmPixel(observation - Vector2(0.0, line_extra), imagePt1);
  asp::toCsmPixel(observation + Vector2(0.0, line_extra), imagePt2);
  double time1 = ls_model->getImageTime(imagePt1);
  double time2 = ls_model->getImageTime(imagePt2);

  // Handle quaternions. We follow closely the conventions for UsgsAstroLsSensorModel.
  int numQuatPerObs = 8; // Max num of quaternions used in pose interpolation 
  int numQuat       = ls_model->m_quaternions.size() / NUM_QUAT_PARAMS;
  double quatT0     = ls_model->m_t0Quat;
  double quatDt     = ls_model->m_dtQuat;

  // Starting and ending quat index (ending is exclusive). Based on lagrangeInterp().
  int qindex1      = static_cast<int>((time1 - quatT0) / quatDt);
  int qindex2      = static_cast<int>((time2 - quatT0) / quatDt);
  int begQuatIndex = std::min(qindex1, qindex2) - numQuatPerObs / 2 + 1;
  int endQuatIndex = std::max(qindex1, qindex2) + numQuatPerObs / 2 + 1;

  // Keep in bounds
  begQuatIndex = std::max(0, begQuatIndex);
  endQuatIndex = std::min(endQuatIndex, numQuat);
  if (begQuatIndex >= endQuatIndex)
    vw::vw_throw(vw::ArgumentErr() << "Book-keeping error for quaternions for pixel: " 
      << observation << ". Likely image order is different than camera order.\n"); 

  // Same for positions
  int numPosPerObs = 8;
  int numPos       = ls_model->m_positions.size() / NUM_XYZ_PARAMS;
  double posT0     = ls_model->m_t0Ephem;
  double posDt     = ls_model->m_dtEphem;

  // Starting and ending pos index (ending is exclusive). Based on lagrangeInterp().
  int pindex1 = static_cast<int>((time1 - posT0) / posDt);
  int pindex2 = static_cast<int>((time2 - posT0) / posDt);
  int begPosIndex = std::min(pindex1, pindex2) - numPosPerObs / 2 + 1;
  int endPosIndex = std::max(pindex1, pindex2) + numPosPerObs / 2 + 1;

  // Keep in bounds
  begPosIndex = std::max(0, begPosIndex);
  endPosIndex = std::min(endPosIndex, numPos);
  if (begPosIndex >= endPosIndex)
    vw_throw(ArgumentErr() << "Book-keeping error for positions for pixel: " 
      << observation << ". Likely image order is different than camera order.\n"); 

  ceres::CostFunction* pixel_cost_function =
    LsPixelReprojErr::Create(observation, weight, ls_model,
                              begQuatIndex, endQuatIndex,
                              begPosIndex, endPosIndex);
  ceres::LossFunction* pixel_loss_function = new ceres::CauchyLoss(opt.robust_threshold);

  // The variable of optimization are camera quaternions and positions stored in the
  // camera models, and the triangulated point.
  std::vector<double*> vars;
  for (int it = begQuatIndex; it < endQuatIndex; it++)
    vars.push_back(&ls_model->m_quaternions[it * NUM_QUAT_PARAMS]);
  for (int it = begPosIndex; it < endPosIndex; it++)
    vars.push_back(&ls_model->m_positions[it * NUM_XYZ_PARAMS]);
  vars.push_back(tri_point);
  problem.AddResidualBlock(pixel_cost_function, pixel_loss_function, vars);

  return;   
}

// Add the frame camera model reprojection error to the cost function
void addFrameReprojectionErr(Options             const & opt,
                             UsgsAstroFrameSensorModel * frame_model,
                             vw::Vector2         const & observation,
                             double                    * frame_params,
                             double                    * tri_point,
                             double                      weight,
                             ceres::Problem            & problem) {

  ceres::CostFunction* pixel_cost_function =
    FramePixelReprojErr::Create(observation, weight, frame_model);
  ceres::LossFunction* pixel_loss_function = new ceres::CauchyLoss(opt.robust_threshold);

  // The variable of optimization are camera positions and quaternion stored 
  // in frame_cam_params, in this order, and the triangulated point.
  // This is different from the linescan model, where we can directly access
  // these quantities inside the model, so they need not be stored separately.
  std::vector<double*> vars;
  vars.push_back(&frame_params[0]);              // positions start here
  vars.push_back(&frame_params[NUM_XYZ_PARAMS]); // quaternions start here
  vars.push_back(tri_point);
  problem.AddResidualBlock(pixel_cost_function, pixel_loss_function, vars);

  return;   
}

// Add reprojection errors. Collect data that will be used to add camera
// constraints that scale with the number of reprojection errors and GSD.
void addReprojCamErrs(Options                           const & opt,
                      asp::CRNJ                         const & crn,
                      std::vector<std::vector<Vector2>> const & pixel_vec,
                      std::vector<std::vector<double>>  const & weight_vec,
                      std::vector<std::vector<int>>     const & isAnchor_vec,
                      std::vector<std::vector<int>>     const & pix2xyz_index,
                      std::vector<asp::CsmModel*>       const & csm_models,
                      // Outputs
                      std::vector<double>                     & tri_points_vec,
                      std::vector<double>                     & frame_params,
                      std::vector<double>                     & weight_per_residual,
                      std::vector<std::vector<double>>        & weight_per_cam,
                      std::vector<std::vector<double>>        & count_per_cam,
                      ceres::Problem                          & problem) {

  // Do here two passes, first for non-anchor points and then for anchor ones.
  // This way it is easier to do the bookkeeping when saving the residuals.
  // Note: The same motions as here are repeated in saveJitterResiduals().
  weight_per_cam.resize(2);
  count_per_cam.resize(2);
  for (int pass = 0; pass < 2; pass++) {
    
     weight_per_cam[pass].resize((int)crn.size(), 0.0);
     count_per_cam[pass].resize((int)crn.size(), 0.0);

    for (int icam = 0; icam < (int)crn.size(); icam++) {

      vw::DiskImageView<float> img(opt.image_files[icam]);
      vw::BBox2 image_box = bounding_box(img);
      std::vector<double> this_cam_weights;

      for (size_t ipix = 0; ipix < pixel_vec[icam].size(); ipix++) {

        Vector2 pix_obs    = pixel_vec[icam][ipix];
        double * tri_point = &tri_points_vec[3 * pix2xyz_index[icam][ipix]];
        double pix_wt      = weight_vec[icam][ipix];
        bool isAnchor      = isAnchor_vec[icam][ipix];

        // Pass 0 is without anchor points, while pass 1 uses them
        if ((int)isAnchor != pass) 
          continue;

        // We can have linescan or frame cameras 
        UsgsAstroLsSensorModel * ls_model
          = dynamic_cast<UsgsAstroLsSensorModel*>((csm_models[icam]->m_gm_model).get());
        UsgsAstroFrameSensorModel * frame_model
          = dynamic_cast<UsgsAstroFrameSensorModel*>((csm_models[icam]->m_gm_model).get());
  
        // Note how for the frame model we pass the frame_params for the current camera.
        if (ls_model != NULL)
          addLsReprojectionErr(opt, ls_model, pix_obs, tri_point, pix_wt, problem);
        else if (frame_model != NULL)
          addFrameReprojectionErr(opt, frame_model, pix_obs, 
              &frame_params[icam * (NUM_XYZ_PARAMS + NUM_QUAT_PARAMS)],
              tri_point, pix_wt, problem);                   
        else
          vw::vw_throw(vw::ArgumentErr() << "Unknown camera model.\n");

        // Two residuals were added. Save the corresponding weights.
        for (int c = 0; c < PIXEL_SIZE; c++)
          weight_per_residual.push_back(pix_wt);

        // Anchor points are fixed by definition. They try to prevent
        // the cameras from moving too much from original poses.
        if (isAnchor) 
          problem.SetParameterBlockConstant(tri_point);
        
        // Find the weight to use with the camera constraint
        vw::Vector3 xyz_obs(tri_point[0], tri_point[1], tri_point[2]);
        double gsd = 0.0;
        try {
          gsd = vw::camera::estimatedGSD(opt.camera_models[icam].get(), image_box, 
                                         pix_obs, xyz_obs);
        } catch (...) {
          continue;
        }
        if (gsd <= 0) 
          continue; 

        // The camera position weight depends on the input multiplier, pixel weight, and gsd
        double position_wt = opt.camera_position_weight * pix_wt / gsd;
        this_cam_weights.push_back(position_wt);
      } // end iteration through pixels
      
      // Find the median weight and count. The median is more robust to outliers.
      count_per_cam[pass][icam] = this_cam_weights.size();
      if (count_per_cam[pass][icam] > 0)
        weight_per_cam[pass][icam] = vw::math::destructive_median(this_cam_weights);
      else
        weight_per_cam[pass][icam] = 0.0;
    } // end iteration through cameras
  } // end iteration through passes

  return;
}

// Add the constraint based on DEM
void addDemConstraint(Options                  const& opt,
                      std::vector<vw::Vector3> const& dem_xyz_vec,
                      std::set<int>            const& outliers,
                      vw::ba::ControlNetwork   const& cnet,
                      // Outputs
                      std::vector<double>           & tri_points_vec,
                      std::vector<double>           & weight_per_residual, // append
                      ceres::Problem                & problem) {
  
  double xyz_weight = -1.0, xyz_threshold = -1.0;
    
  if (!opt.heights_from_dem.empty()) {
    xyz_weight = 1.0/opt.heights_from_dem_uncertainty;
    xyz_threshold = opt.heights_from_dem_robust_threshold;
  } else {
    vw::vw_throw(vw::ArgumentErr() << "No input DEM was provided.\n");
  }
  
  if (dem_xyz_vec.size() != cnet.size()) 
    vw_throw(ArgumentErr() << "Must have as many xyz computed from DEM as xyz "
             << "triangulated from match files.\n");
  if (xyz_weight <= 0 || xyz_threshold <= 0)
    vw_throw(ArgumentErr() << "Detected invalid robust threshold or weights.\n");

  int num_tri_points = cnet.size();
  
  // The tri_points_vec must have at least as many points as cnet. It can have anchor points
  // as well.
  if ((int)tri_points_vec.size() < num_tri_points * NUM_XYZ_PARAMS)
    vw_throw(ArgumentErr() << "Too few triangulated points.\n");
  
  for (int ipt = 0; ipt < num_tri_points; ipt++) {
      
    if (cnet[ipt].type() == vw::ba::ControlPoint::GroundControlPoint)
      vw_throw(ArgumentErr() << "Found GCP where not expecting any.\n");

    // Note that we get tri points from dem_xyz_vec, based on the input DEM
    Vector3 observation = dem_xyz_vec.at(ipt);
    if (outliers.find(ipt) != outliers.end() || observation == Vector3(0, 0, 0)) 
      continue; // outlier
      
    ceres::CostFunction* xyz_cost_function = weightedXyzError::Create(observation, xyz_weight);
    ceres::LossFunction* xyz_loss_function = new ceres::CauchyLoss(xyz_threshold);
    double * tri_point = &tri_points_vec[0] + ipt * NUM_XYZ_PARAMS;

    // Add cost function
    problem.AddResidualBlock(xyz_cost_function, xyz_loss_function, tri_point);

    for (int c = 0; c < NUM_XYZ_PARAMS; c++)
      weight_per_residual.push_back(xyz_weight);
  }
}

// Add the constraint to keep triangulated points close to initial values
// This does not need a DEM or alignment
void addTriConstraint(Options                const& opt,
                      std::set<int>          const& outliers,
                      vw::ba::ControlNetwork const& cnet,
                      asp::CRNJ              const& crn,
                      // Outputs
                      std::vector<double>    & tri_points_vec,
                      std::vector<double>    & weight_per_residual, // append
                      ceres::Problem         & problem) {

  // Estimate the GSD for each triangulated point
  std::vector<double> gsds;
  asp::estimateGsdPerTriPoint(opt.image_files, opt.camera_models, crn, 
                              outliers, tri_points_vec, gsds);
  
  int num_tri_points = cnet.size();
  for (int ipt = 0; ipt < num_tri_points; ipt++) {
    if (cnet[ipt].type() == vw::ba::ControlPoint::GroundControlPoint ||
        cnet[ipt].type() == vw::ba::ControlPoint::PointFromDem)
      continue; // Skip GCPs and height-from-dem points which have their own constraint

    if (outliers.find(ipt) != outliers.end()) 
      continue; // skip outliers
      
    double * tri_point = &tri_points_vec[0] + ipt * NUM_XYZ_PARAMS;
    
    // The weight must be inversely proportional to the GSD, to ensure
    // this is in pixel units
    double gsd = gsds[ipt];
    if (gsd <= 0) 
      continue; // GSD calculation failed. Do not use a constraint.
    double weight = opt.tri_weight / gsd;
  
    // Use as constraint the initially triangulated point
    vw::Vector3 observation(tri_point[0], tri_point[1], tri_point[2]);

    ceres::CostFunction* cost_function = weightedXyzError::Create(observation, weight);
    ceres::LossFunction* loss_function = new ceres::CauchyLoss(opt.tri_robust_threshold);
    problem.AddResidualBlock(cost_function, loss_function, tri_point);
    
    for (int c = 0; c < NUM_XYZ_PARAMS; c++)
      weight_per_residual.push_back(opt.tri_weight);
      
  } // End loop through xyz
}

// Add camera constraints that are proportional to the number of reprojection errors.
// This requires going through some of the same motions as in addReprojCamErrs().
void addCamPositionConstraint(Options                      const & opt,
                              std::set<int>                const & outliers,
                              asp::CRNJ                    const & crn,
                              std::vector<asp::CsmModel*>  const & csm_models,
                              std::vector<std::vector<double>> const& weight_per_cam,
                              std::vector<std::vector<double>> const& count_per_cam,
                              // Outputs
                              std::vector<double>                & frame_params,
                              std::vector<double>                & weight_per_residual, 
                              ceres::Problem                     & problem) {

  // First pass is for interest point matches, and second pass is for anchor points
  for (int pass = 0; pass < 2; pass++) {
    for (int icam = 0; icam < (int)crn.size(); icam++) {
      
      double median_wt = weight_per_cam[pass][icam];
      double count = count_per_cam[pass][icam];
      if (count <= 0) 
        continue; // no reprojection errors for this camera
      
      // We know the median weight to use, and how many residuals were added.
      // Based on the CERES loss function formula, adding N loss functions each 
      // with weight w and robust threshold t is equivalent to adding one loss 
      // function with weight sqrt(N)*w and robust threshold sqrt(N)*t.
      // For linescan cameras, then need to subdivide this for individual
      // positions for that camera.
      double combined_wt  = sqrt(count * 1.0) * median_wt;
      double combined_th = sqrt(count * 1.0) * opt.camera_position_robust_threshold;
      UsgsAstroLsSensorModel * ls_model
        = dynamic_cast<UsgsAstroLsSensorModel*>((csm_models[icam]->m_gm_model).get());
      UsgsAstroFrameSensorModel * frame_model
        = dynamic_cast<UsgsAstroFrameSensorModel*>((csm_models[icam]->m_gm_model).get());
        
      if (ls_model != NULL) {
        // There are multiple position parameters per camera. They divide among
        // them the job of minimizing the reprojection error. So need to divide
        // the weight among them.

        // Divide the weight among the positions
        int numPos = ls_model->m_positions.size() / NUM_XYZ_PARAMS;
        double wt = combined_wt / sqrt(numPos * 1.0);
        double th = combined_th / sqrt(numPos * 1.0);
        for (int ip = 0; ip < numPos; ip++) {
          ceres::CostFunction* cost_function
            = weightedTranslationError::Create(&ls_model->m_positions[ip * NUM_XYZ_PARAMS],
                                               wt);
          ceres::LossFunction* loss_function = new ceres::CauchyLoss(th);
          problem.AddResidualBlock(cost_function, loss_function,
                                  &ls_model->m_positions[ip * NUM_XYZ_PARAMS]);
          
          for (int c = 0; c < NUM_XYZ_PARAMS; c++)
            weight_per_residual.push_back(wt);
        }
        
      } else if (frame_model != NULL) {
      
        // Same logic as for bundle_adjust
        // There is only one position per camera
        double * curr_params = &frame_params[icam * (NUM_XYZ_PARAMS + NUM_QUAT_PARAMS)];
        // we will copy from curr_params the initial position
        ceres::CostFunction* cost_function
          = weightedTranslationError::Create(&curr_params[0], combined_wt);
        ceres::LossFunction* loss_function = new ceres::CauchyLoss(combined_th);
        problem.AddResidualBlock(cost_function, loss_function,
                                &curr_params[0]); // translation starts here
        
        for (int c = 0; c < NUM_XYZ_PARAMS; c++)
          weight_per_residual.push_back(combined_wt);
            
      } else {
         vw::vw_throw(vw::ArgumentErr() << "Unknown camera model.\n");
      }
    }
  }
}

void addQuatNormRotationConstraints(
    Options                      const & opt,
    std::set<int>                const & outliers,
    asp::CRNJ                    const & crn,
    std::vector<asp::CsmModel*>  const & csm_models,
    // Outputs
    std::vector<double>                & frame_params,
    std::vector<double>                & weight_per_residual, // append
    ceres::Problem                     & problem) {
  
  // Constrain the rotations
  // TODO(oalexan1): Make this a standalone function
  if (opt.rotation_weight > 0.0) {
    for (int icam = 0; icam < (int)crn.size(); icam++) {

      UsgsAstroLsSensorModel * ls_model
        = dynamic_cast<UsgsAstroLsSensorModel*>((csm_models[icam]->m_gm_model).get());
      UsgsAstroFrameSensorModel * frame_model
        = dynamic_cast<UsgsAstroFrameSensorModel*>((csm_models[icam]->m_gm_model).get());

      if (ls_model != NULL) {
        // There are multiple quaternion parameters per camera
        int numQuat = ls_model->m_quaternions.size() / NUM_QUAT_PARAMS;
        for (int iq = 0; iq < numQuat; iq++) {
          ceres::CostFunction* rotation_cost_function
            = weightedRotationError::Create(&ls_model->m_quaternions[iq * NUM_QUAT_PARAMS],
                                            opt.rotation_weight);
          // We use no loss function, as the quaternions have no outliers
          ceres::LossFunction* rotation_loss_function = NULL;
          problem.AddResidualBlock(rotation_cost_function, rotation_loss_function,
                                  &ls_model->m_quaternions[iq * NUM_QUAT_PARAMS]);
          
          for (int c = 0; c < NUM_QUAT_PARAMS; c++)
            weight_per_residual.push_back(opt.rotation_weight);
        }

      } else if (frame_model != NULL) {
        // There is one quaternion per camera, stored after the translation
        double * curr_params = &frame_params[icam * (NUM_XYZ_PARAMS + NUM_QUAT_PARAMS)];
          
        // Copy from curr_params the initial quaternion
        ceres::CostFunction* rotation_cost_function
          = weightedRotationError::Create(&curr_params[NUM_XYZ_PARAMS], // quat starts here
                                          opt.rotation_weight);
        // Pass the quaternion to optimize to the problem                                  
        // We use no loss function, as the quaternions have no outliers
        ceres::LossFunction* rotation_loss_function = NULL;
        problem.AddResidualBlock(rotation_cost_function, rotation_loss_function,
                                &curr_params[NUM_XYZ_PARAMS]); // quat starts here
        
        for (int c = 0; c < NUM_QUAT_PARAMS; c++)
          weight_per_residual.push_back(opt.rotation_weight);
      } else {
         vw::vw_throw(vw::ArgumentErr() << "Unknown camera model.\n");
      }

    } // end loop through cameras
  }

  // Try to make the norm of quaternions be close to 1
  // TODO(oalexan1): Make this a standalone function
  if (opt.quat_norm_weight > 0.0) {
    for (int icam = 0; icam < (int)crn.size(); icam++) {

      UsgsAstroLsSensorModel * ls_model
        = dynamic_cast<UsgsAstroLsSensorModel*>((csm_models[icam]->m_gm_model).get());
      UsgsAstroFrameSensorModel * frame_model
        = dynamic_cast<UsgsAstroFrameSensorModel*>((csm_models[icam]->m_gm_model).get());

      if (ls_model != NULL) {

        int numQuat = ls_model->m_quaternions.size() / NUM_QUAT_PARAMS;
        for (int iq = 0; iq < numQuat; iq++) {
          ceres::CostFunction* quat_norm_cost_function
            = weightedQuatNormError::Create(opt.quat_norm_weight);
          // We use no loss function, as the quaternions have no outliers
          ceres::LossFunction* quat_norm_loss_function = NULL;
          problem.AddResidualBlock(quat_norm_cost_function, quat_norm_loss_function,
                                  &ls_model->m_quaternions[iq * NUM_QUAT_PARAMS]);
          
          weight_per_residual.push_back(opt.quat_norm_weight); // 1 single residual
        }

      } else if (frame_model != NULL) {

        // There is one quaternion per camera, stored after the translation
        double * curr_params = &frame_params[icam * (NUM_XYZ_PARAMS + NUM_QUAT_PARAMS)];

        ceres::CostFunction* quat_norm_cost_function
          = weightedQuatNormError::Create(opt.quat_norm_weight);
        // We use no loss function, as the quaternions have no outliers
        ceres::LossFunction* quat_norm_loss_function = NULL;
        problem.AddResidualBlock(quat_norm_cost_function, quat_norm_loss_function,
                                &curr_params[NUM_XYZ_PARAMS]); // quat starts here
        
        weight_per_residual.push_back(opt.quat_norm_weight); // 1 single residual

      } else {
         vw::vw_throw(vw::ArgumentErr() << "Unknown camera model.\n");
      }
    }
  }
}

// Add roll / yaw constraints. For linescan, use the whole set of samples for given
// camera model. For frame cameras, use the trajectory of all cameras in the same orbital
// group as the current camera.
void addRollYawConstraint
   (Options                         const& opt,
    asp::CRNJ                       const& crn,
    std::vector<asp::CsmModel*>     const& csm_models,
    vw::cartography::GeoReference   const& georef,
    // Outputs (append to residual)
    std::vector<double>                  & frame_params,
    std::vector<double>                  & weight_per_residual,
    ceres::Problem                       & problem) {
  
  if (opt.roll_weight <= 0.0 && opt.yaw_weight <= 0.0)
     vw::vw_throw(vw::ArgumentErr() 
         << "addRollYawConstraint: The roll or yaw weight must be positive.\n");

  int num_cams = crn.size();

  // Frame cameras can be grouped by orbital portion. Ensure that all cameras
  // belong to a group.
  if (num_cams != int(opt.orbital_groups.size()))
    vw::vw_throw(vw::ArgumentErr() 
         << "addRollYawConstraint: Failed to add each input camera to an orbital group.\n");

  // Create the orbital trajectory for each group of frame cameras
  std::map<int, std::vector<double>> orbital_group_positions;
  std::map<int, std::vector<double>> orbital_group_quaternions;
  formPositionQuatVecPerGroup(opt.orbital_groups, csm_models, 
    orbital_group_positions, orbital_group_quaternions); // outputs

  for (int icam = 0; icam < num_cams; icam++) {

    UsgsAstroLsSensorModel * ls_model
      = dynamic_cast<UsgsAstroLsSensorModel*>((csm_models[icam]->m_gm_model).get());
    UsgsAstroFrameSensorModel * frame_model
      = dynamic_cast<UsgsAstroFrameSensorModel*>((csm_models[icam]->m_gm_model).get());

    if (ls_model != NULL) {
      // Linescan cameras. Use the full sequence of cameras in the model
      // to enforce the roll/yaw constraint for each camera in the sequence.
      int numQuat = ls_model->m_quaternions.size() / NUM_QUAT_PARAMS;

      // Make positions one-to-one with quaternions
      std::vector<double> interp_positions;
      asp::orbitInterpExtrap(ls_model, georef, interp_positions);
      
      for (int iq = 0; iq < numQuat; iq++) {
        ceres::CostFunction* roll_yaw_cost_function
          = weightedRollYawError::Create(interp_positions,
                                         ls_model->m_quaternions,
                                         georef, iq,
                                         opt.roll_weight, opt.yaw_weight, 
                                         opt.initial_camera_constraint);

        // We use no loss function, as the quaternions have no outliers
        ceres::LossFunction* roll_yaw_loss_function = NULL;
        problem.AddResidualBlock(roll_yaw_cost_function, roll_yaw_loss_function,
                                &ls_model->m_quaternions[iq * NUM_QUAT_PARAMS]);
        // The recorded weight should not be 0 as we will divide by it
        weight_per_residual.push_back(opt.roll_weight || 1.0);
        weight_per_residual.push_back(opt.yaw_weight  || 1.0);
      } // end loop through quaternions for given camera
    
    } else if (frame_model != NULL) {
      // Frame cameras. Use the positions and quaternions of the cameras
      // in the same orbital group to enforce the roll/yaw constraint for
      // each camera in the group.
      auto it = opt.orbital_groups.find(icam);
      if (it == opt.orbital_groups.end())
        vw::vw_throw(vw::ArgumentErr() 
           << "addRollYawConstraint: Failed to find orbital group for camera.\n"); 
      int group_id = it->second;

      int index_in_group = indexInGroup(icam, opt.orbital_groups);
      std::vector<double> positions = orbital_group_positions[group_id];
      std::vector<double> quaternions = orbital_group_quaternions[group_id];
      if (positions.size() / NUM_XYZ_PARAMS < 2) {
        // It can happen that we have just one frame camera, but then we just
        // can't add this constraint
        vw::vw_out(vw::WarningMessage) << "Cannot add roll and/or yaw constraint for "
          << "for an orbital group consisting of only one frame camera.\n";
        continue;
      }
        
      ceres::CostFunction* roll_yaw_cost_function
        = weightedRollYawError::Create(positions, quaternions, 
                                   georef, index_in_group,
                                   opt.roll_weight, opt.yaw_weight, 
                                   opt.initial_camera_constraint);

      // We use no loss function, as the quaternions have no outliers
      ceres::LossFunction* roll_yaw_loss_function = NULL;

      // Note how we set the quaternions to be optimized from frame_params.
      // Above, we only cared for initial positions and quaternions.
      double * curr_params = &frame_params[icam * (NUM_XYZ_PARAMS + NUM_QUAT_PARAMS)];
      problem.AddResidualBlock(roll_yaw_cost_function, roll_yaw_loss_function,
                                &curr_params[NUM_XYZ_PARAMS]); // quat starts here

      // The recorded weight should not be 0 as we will divide by it
      weight_per_residual.push_back(opt.roll_weight || 1.0);
      weight_per_residual.push_back(opt.yaw_weight  || 1.0);
    } else {
      vw::vw_throw(vw::ArgumentErr() 
         << "addRollYawConstraint: Expecting CSM linescan or frame cameras.\n");
    }

  } // end loop through cameras

  return;
}

// Apply the input adjustments to the CSM cameras. Resample linescan models.
// Get pointers to the underlying CSM cameras, as need to manipulate
// those directly. This modifies camera_models in place.
void prepareCsmCameras(Options const& opt,
  std::vector<vw::CamPtr>      const& camera_models,
  std::vector<asp::CsmModel*>       & csm_models) {

  // Wipe the output
  csm_models.clear();
  
  for (size_t icam = 0; icam < camera_models.size(); icam++) {
    asp::CsmModel * csm_cam = asp::csm_model(camera_models[icam], opt.stereo_session);

    // Sanity check
    if (csm_cam == NULL)
      vw::vw_throw(vw::ArgumentErr() << "Expecting CSM cameras.\n");
      
    if (!opt.input_prefix.empty())
      asp::applyAdjustmentToCsmCamera(opt.image_files[icam],
                                      opt.camera_files[icam],
                                      opt.input_prefix,
                                      camera_models[icam],
                                      csm_cam);

    // Get the underlying linescan model or frame model
    UsgsAstroLsSensorModel * ls_model
      = dynamic_cast<UsgsAstroLsSensorModel*>((csm_cam->m_gm_model).get());
    UsgsAstroFrameSensorModel * frame_model
      = dynamic_cast<UsgsAstroFrameSensorModel*>((csm_cam->m_gm_model).get());

    if (ls_model == NULL && frame_model == NULL)
      vw_throw(ArgumentErr() << "Expecting the cameras to be of CSM linescan or frame type.\n");

    // Normalize quaternions. Later, the quaternions being optimized will
    // be kept close to being normalized.  This makes it easy to ensure
    // that quaternion interpolation gives good results, especially that
    // some quaternions may get optimized and some not.
    if (ls_model != NULL) {
      asp::normalizeQuaternions(ls_model);
      // The provided tabulated positions, velocities and quaternions may be too few,
      // so resample them with --num-lines-per-position and --num-lines-per-orientation,
      // if those are set.
      resampleModel(opt, ls_model);
    } else if (frame_model != NULL) {
      normalizeQuaternions(frame_model);
    } else {
      vw::vw_throw(vw::ArgumentErr() 
        << "Expecting the cameras to be of CSM linescan or frame type.\n");
    }

    csm_models.push_back(csm_cam);
  }
}

// Create structures for pixels, xyz, and weights, to be used in optimization.
// Later there will be another pass to add weights for the anchor points.
// Here more points may be flagged as outliers.
void createProblemStructure(Options                      const& opt,
                            asp::CRNJ                    const& crn,
                            vw::ba::ControlNetwork       const& cnet, 
                            std::vector<double>          const& tri_points_vec,
                            // Outputs
                            std::set<int>                     & outliers,
                            std::vector<std::vector<Vector2>> & pixel_vec,
                            std::vector<std::vector<double>>  & weight_vec,
                            std::vector<std::vector<int>>     & isAnchor_vec,
                            std::vector<std::vector<int>>     & pix2xyz_index) {

  // If to use a weight image
  bool have_weight_image = (!opt.weight_image.empty());
  vw::ImageViewRef<vw::PixelMask<float>> weight_image;
  float weight_image_nodata = -std::numeric_limits<float>::max();
  vw::cartography::GeoReference weight_image_georef;
  if (have_weight_image)
    vw::cartography::readGeorefImage(opt.weight_image,
      weight_image_nodata, weight_image_georef, weight_image);

  int num_cameras = opt.camera_models.size();

  // Wipe
  pixel_vec.resize(0);
  weight_vec.resize(0);
  isAnchor_vec.resize(0);
  pix2xyz_index.resize(0);
  // Resize
  pixel_vec.resize(num_cameras);
  weight_vec.resize(num_cameras);
  isAnchor_vec.resize(num_cameras);
  pix2xyz_index.resize(num_cameras);

  for (int icam = 0; icam < (int)crn.size(); icam++) {
    for (auto fiter = crn[icam].begin(); fiter != crn[icam].end(); fiter++) {
      
      // The index of the 3D point
      int ipt = (**fiter).m_point_id;
      
      if (outliers.find(ipt) != outliers.end())
        continue; // Skip outliers
      
      // The observed value for the projection of point with index ipt into
      // the camera with index icam.
      Vector2 observation = (**fiter).m_location;

      // Unlike in bundle adjustment, the weight of a pixel is 1.0, rather
      // than 1.0 / pixel_sigma.
      double weight = 1.0;
      
      // If we have a weight image, use it to set the weight
      if (have_weight_image) {
        const double * tri_point = &tri_points_vec[0] + ipt * NUM_XYZ_PARAMS;
        Vector3 ecef(tri_point[0], tri_point[1], tri_point[2]);
        vw::PixelMask<float> img_wt 
          = vw::cartography::closestPixelVal(weight_image, weight_image_georef, ecef);
        
        // Flag bad weights as outliers
        if (!is_valid(img_wt) || std::isnan(img_wt.child()) || img_wt.child() <= 0.0) {
          outliers.insert(ipt);
          continue;
        }
        
        weight = img_wt.child();
      }
      
      pixel_vec[icam].push_back(observation);
      weight_vec[icam].push_back(weight);
      isAnchor_vec[icam].push_back(0);
      pix2xyz_index[icam].push_back(ipt);
    }
  }

  return;
}

// Put the triangulated points in a vector. Update the cnet from the DEM,
// if we have one.
void formTriVec(std::vector<Vector3> const& dem_xyz_vec,
                bool have_dem,
                // Outputs
                ba::ControlNetwork  & cnet,
                std::vector<double> & orig_tri_points_vec,
                std::vector<double> & tri_points_vec) {

  int num_tri_points = cnet.size();
  if (num_tri_points == 0)
    vw::vw_throw(ArgumentErr() << "No triangulated ground points were found.\n"); 

  orig_tri_points_vec.resize(num_tri_points*NUM_XYZ_PARAMS, 0.0);
  tri_points_vec.resize(num_tri_points*NUM_XYZ_PARAMS, 0.0);

  for (int ipt = 0; ipt < num_tri_points; ipt++) {
    // We overwrite the triangulated point when we have an input DEM.
    // It is instructive to examine the pointmap residual file to see
    // what effect that has on residuals.  This point will likely try
    // to move back somewhat to its triangulated position during
    // optimization, depending on the strength of the weight which
    // tries to keep it back in place.
    Vector3 tri_point = cnet[ipt].position();
    
    // The original triangulated point, before the override or optimization
    for (int q = 0; q < NUM_XYZ_PARAMS; q++)
      orig_tri_points_vec[ipt*NUM_XYZ_PARAMS + q] = tri_point[q];
    
    if (have_dem && dem_xyz_vec.at(ipt) != Vector3(0, 0, 0)) {
      tri_point = dem_xyz_vec.at(ipt);

      // Update in the cnet too
      cnet[ipt].set_position(Vector3(tri_point[0], tri_point[1], tri_point[2]));
      
      // Ensure we can track it later
      cnet[ipt].set_type(vw::ba::ControlPoint::PointFromDem); 
    }
    
    for (int q = 0; q < NUM_XYZ_PARAMS; q++)
      tri_points_vec[ipt*NUM_XYZ_PARAMS + q] = tri_point[q];
  }
  return;
}

void saveOptimizedCameraModels(std::string const& out_prefix,
                               std::string const& stereo_session, 
                               std::vector<std::string> const& image_files,
                               std::vector<std::string> const& camera_files,
                               std::vector<vw::CamPtr>  const& camera_models,
                               bool update_isis_cubes_with_csm_state) {

  for (size_t icam = 0; icam < camera_models.size(); icam++) {
    std::string adjustFile = asp::bundle_adjust_file_name(out_prefix,
                                                          image_files[icam],
                                                          camera_files[icam]);
    std::string csmFile = asp::csmStateFile(adjustFile);
    asp::CsmModel * csm_cam = asp::csm_model(camera_models[icam], stereo_session);
    csm_cam->saveState(csmFile);

    if (update_isis_cubes_with_csm_state) {
      // Save the CSM state to the image file. Wipe any spice info.
      std::string image_name = image_files[icam]; 
      std::string plugin_name = csm_cam->plugin_name();
      std::string model_name  = csm_cam->model_name();
      std::string model_state = csm_cam->model_state();
      vw::vw_out() << "Adding updated CSM state to image file: " << image_name << std::endl;
      asp:isis::saveCsmStateToIsisCube(image_name, plugin_name, model_name, model_state);
    }
  }
}

void run_jitter_solve(int argc, char* argv[]) {

  // Parse arguments and perform validation
  Options opt;
  handle_arguments(argc, argv, opt);

  bool approximate_pinhole_intrinsics = false;
  asp::load_cameras(opt.image_files, opt.camera_files, opt.out_prefix, opt,  
                    approximate_pinhole_intrinsics,  
                    // Outputs
                    opt.stereo_session,  // may change
                    opt.single_threaded_cameras,  
                    opt.camera_models);

  // Find the datum.
  // TODO(oalexan1): Integrate this into load_cameras, to avoid loading
  // the cameras twice. Do this also in bundle_adjust.cc.
  bool found_datum = asp::datum_from_cameras(opt.image_files, opt.camera_files,  
                                             opt.stereo_session,  // may change
                                             // Outputs
                                             opt.datum);
  if (!found_datum)
    vw_throw(ArgumentErr() << "No datum was found in the input cameras.\n");
    
  // Apply the input adjustments to the cameras. Resample linescan models.
  // Get pointers to the underlying CSM cameras, as need to manipulate
  // those directly. These will result in changes to the input cameras.
  std::vector<asp::CsmModel*> csm_models;
  prepareCsmCameras(opt, opt.camera_models, csm_models);
  
  // This the right place to record the original camera positions.
  std::vector<vw::Vector3> orig_cam_positions;
  asp::calcCameraCenters(opt.camera_models, orig_cam_positions);
  
  // Make a list of all the image pairs to find matches for. Some quantities
  // below are not needed but are part of the API.
  bool external_matches = true;
  bool got_est_cam_positions = false;
  double position_filter_dist = -1.0;
  std::vector<vw::Vector3> estimated_camera_gcc;
  bool have_overlap_list = false;
  std::set<std::pair<std::string, std::string>> overlap_list;
  std::vector<std::pair<int,int>> all_pairs;
  asp::determine_image_pairs(// Inputs
                             opt.overlap_limit, opt.match_first_to_last,  
                             external_matches,
                             opt.image_files, 
                             got_est_cam_positions, position_filter_dist,
                             estimated_camera_gcc, have_overlap_list, overlap_list,
                             // Output
                             all_pairs);

  // List existing match files. This can take a while.
  vw_out() << "Computing the list of existing match files.\n";
  std::string prefix = asp::match_file_prefix(opt.clean_match_files_prefix,
                                              opt.match_files_prefix,  
                                              opt.out_prefix);
  std::set<std::string> existing_files;
  asp::listExistingMatchFiles(prefix, existing_files);

  // Load match files
  std::map<std::pair<int, int>, std::string> match_files;
  for (size_t k = 0; k < all_pairs.size(); k++) {
    int i = all_pairs[k].first;
    int j = all_pairs[k].second;
    std::string const& image1_path  = opt.image_files[i];  // alias
    std::string const& image2_path  = opt.image_files[j];  // alias
    std::string const& camera1_path = opt.camera_files[i]; // alias
    std::string const& camera2_path = opt.camera_files[j]; // alias

    // Load match files from a different source
    std::string match_file 
      = asp::match_filename(opt.clean_match_files_prefix, opt.match_files_prefix,  
                            opt.out_prefix, image1_path, image2_path);

    // The external match file does not exist, don't try to load it
    if (existing_files.find(match_file) == existing_files.end())
      continue;
    
    match_files[std::make_pair(i, j)] = match_file;
  }
  
  if (match_files.empty() && opt.isis_cnet.empty())
    vw_throw(ArgumentErr() 
             << "No match files or ISIS cnet found. Check if your match "
             << "files exist and if they satisfy the naming convention "
             << "<prefix>-<image1>__<image2>.match.\n");

  // Build control network and perform triangulation with adjusted input cameras
  ba::ControlNetwork cnet("jitter_solve");
  if (opt.isis_cnet != "") {
    asp::IsisCnetData isisCnetData; // isis cnet (if loaded)
    vw::vw_out() << "Reading ISIS control network: " << opt.isis_cnet << "\n";
    asp::loadIsisCnet(opt.isis_cnet, opt.image_files,
                      cnet, isisCnetData); // outputs
  } else {
    bool triangulate_control_points = true;
    bool success = vw::ba::build_control_network(triangulate_control_points,
                                                cnet, // output
                                                opt.camera_models, opt.image_files,
                                                match_files, opt.min_matches,
                                                opt.min_triangulation_angle*(M_PI/180.0),
                                                opt.forced_triangulation_distance,
                                                opt.max_pairwise_matches);
    if (!success)
      vw::vw_throw(vw::ArgumentErr()
              << "Failed to build a control network. Check the bundle adjustment directory "
              << "for matches and if the match files satisfy the naming convention. "
              << "Or, consider removing all .vwip and "
              << ".match files and increasing the number of interest points "
              << "using --ip-per-image or --ip-per-tile, or decreasing --min-matches, "
              << "and then re-running bundle adjustment.\n");
  }
  
  // TODO(oalexan1): Is it possible to avoid using CRNs?
  asp::CRNJ crn;
  crn.from_cnet(cnet);
  
  if ((int)crn.size() != opt.camera_models.size()) 
    vw_throw(ArgumentErr() << "Book-keeping error, the size of CameraRelationNetwork "
             << "must equal the number of images.\n");

  // Flag as outliers points with initial reprojection error bigger than
  // a certain amount. This assumes that the input cameras are very accurate.
  std::set<int> outliers;
  flag_initial_outliers(cnet, crn, opt.camera_models, opt.max_init_reproj_error,  
                        // Output
                        outliers);
  vw_out() << "Removed " << outliers.size() 
    << " outliers based on initial reprojection error.\n";
  
  bool have_dem = (!opt.heights_from_dem.empty());

  // Create anchor xyz with the help of a DEM in two ways.
  // TODO(oalexan1): Study how to best pass the DEM to avoid the code
  // below not being slow. It is not clear if the DEM tiles are cached
  // when passing around an ImageViewRef.
  std::vector<Vector3> dem_xyz_vec;
  vw::cartography::GeoReference dem_georef, anchor_georef;
  ImageViewRef<PixelMask<double>> interp_dem, interp_anchor_dem;
  if (opt.heights_from_dem != "") {
    vw::vw_out() << "Reading the DEM for the --heights-from-dem constraint.\n";
    asp::create_interp_dem(opt.heights_from_dem, dem_georef, interp_dem);
    asp::update_point_from_dem(cnet, crn, outliers, opt.camera_models,
                               dem_georef, interp_dem,  
                               // Output
                               dem_xyz_vec);
  }
  
  if (opt.anchor_dem != "") {
    vw::vw_out() << "Reading the DEM for the --anchor-dem constraint.\n";
    asp::create_interp_dem(opt.anchor_dem, anchor_georef, interp_anchor_dem);
  }

  // Handle the roll/yaw constraint DEM. We already checked that one of thse cases should work
  vw::cartography::GeoReference roll_yaw_georef;
  if (opt.roll_weight > 0 || opt.yaw_weight > 0) {
    if (opt.heights_from_dem != "") {
      roll_yaw_georef = dem_georef;
      vw::vw_out() << "Using the DEM from --heights-from-dem "
                   << "for the roll/yaw constraint.\n";
    } else if (opt.anchor_dem != "") {
      roll_yaw_georef = anchor_georef;
      vw::vw_out() << "Using the DEM from --anchor-dem for the roll/yaw constraint.\n";
    }
  } 

  int num_cameras = opt.camera_models.size();
  if (num_cameras < 2)
    vw_throw(ArgumentErr() << "Expecting at least two input cameras.\n");

  // If some of the input cameras are frame, need to store position and
  // quaternion variables for them outside the camera model, as these are
  // private for UsgsAstroFrameCameraModel, unlike for UsgsAstroLsSensorModel.
  // It is easier to just allocate the space for all cameras, even if it may go
  // unused mostly or at all.
  std::vector<double> frame_params;
  initFrameCameraParams(csm_models, frame_params);

  // Put the triangulated points in a vector. Update the cnet from the DEM,
  // if we have one. Later will add here the anchor points.
  std::vector<double> orig_tri_points_vec, tri_points_vec;
  formTriVec(dem_xyz_vec, have_dem,
    cnet, orig_tri_points_vec, tri_points_vec); // outputs
  
  // Create structures for pixels, xyz, and weights, to be used in optimization
  std::vector<std::vector<Vector2>> pixel_vec;
  std::vector<std::vector<double>> weight_vec;
  std::vector<std::vector<int>> isAnchor_vec;
  std::vector<std::vector<int>> pix2xyz_index;
  createProblemStructure(opt, crn, cnet, tri_points_vec,
                         // Outputs
                         outliers, pixel_vec, 
                         weight_vec, isAnchor_vec, pix2xyz_index);

  // Find anchor points and append to pixel_vec, weight_vec, xyz_vec, etc.
  if ((opt.num_anchor_points_per_image > 0 || opt.num_anchor_points_per_tile > 0) &&
       opt.anchor_weight > 0)
    calcAnchorPoints(opt, interp_anchor_dem, anchor_georef, csm_models,  
                     // Append to these
                     pixel_vec, weight_vec, isAnchor_vec, pix2xyz_index,
                     orig_tri_points_vec, tri_points_vec);
    
    // The above structures must not be resized anymore, as we will get pointers
    // to individual blocks within them.

  // Need this in order to undo the multiplication by weight before saving the residuals
  std::vector<double> weight_per_residual;

  // The problem to solve
  ceres::Problem problem;
  
  // In order to add a proportional camera constraint, we need to know the
  // median weight per camera and their count. These are different for anchor
  // and non-anchor points.  
  std::vector<std::vector<double>> weight_per_cam(2);
  std::vector<std::vector<double>> count_per_cam(2);
  
  // Add reprojection errors. Get back weights_per_cam, count_per_cam.
  addReprojCamErrs(opt, crn, pixel_vec, weight_vec,
                   isAnchor_vec, pix2xyz_index, csm_models,
                   // Outputs
                   tri_points_vec, frame_params, weight_per_residual, 
                   weight_per_cam, count_per_cam, problem);
 
  // Add the DEM constraint. We check earlier that only one
  // of the two options below can be set at a time.
  if (have_dem)
    addDemConstraint(opt, dem_xyz_vec, outliers, cnet,  
                     // Outputs
                     tri_points_vec, 
                     weight_per_residual,  // append
                     problem);

  // Add the constraint to keep triangulated points close to initial values
  // This does not need a DEM or alignment.
  // This must happen after any DEM-based constraint is set, and won't
  // apply to tri points already constrained by the DEM (so it will
  // work only where the DEM is missing).
  if (opt.tri_weight > 0) 
    addTriConstraint(opt, outliers, cnet, crn,
                     // Outputs
                     tri_points_vec,  
                     weight_per_residual,  // append
                     problem);

  // Add the constraint to keep the camera positions close to initial values
  if (opt.camera_position_weight > 0) 
    addCamPositionConstraint(opt, outliers, crn, csm_models, weight_per_cam, count_per_cam,
                             // Outputs
                             frame_params, weight_per_residual, problem);
    
  // Add constraints to keep quat norm close to 1, and make rotations 
  // not change too much
  addQuatNormRotationConstraints(opt, outliers, crn, csm_models,  
                                 // Outputs
                                 frame_params,
                                 weight_per_residual,  // append
                                 problem);

  if (opt.roll_weight > 0 || opt.yaw_weight > 0)
    addRollYawConstraint(opt, crn, csm_models, roll_yaw_georef,
                        frame_params, weight_per_residual, problem); // outputs

  // Save residuals before optimization
  std::string residual_prefix = opt.out_prefix + "-initial_residuals";
  saveJitterResiduals(problem, residual_prefix, opt, cnet, crn, opt.datum,
                 tri_points_vec, outliers, weight_per_residual,
                 pixel_vec, weight_vec, isAnchor_vec, pix2xyz_index);
  
  // Set up the problem
  ceres::Solver::Options options;
  options.gradient_tolerance  = 1e-16;
  options.function_tolerance  = 1e-16;
  options.parameter_tolerance = opt.parameter_tolerance; // default is 1e-12
  options.max_num_iterations                = opt.num_iterations;
  options.max_num_consecutive_invalid_steps = std::max(20, opt.num_iterations/5); // try hard
  options.minimizer_progress_to_stdout      = true;
  if (opt.single_threaded_cameras)
    options.num_threads = 1;
  else
    options.num_threads = opt.num_threads;
  // This is supposed to help with speed in a certain size range
  options.linear_solver_type = ceres::SPARSE_SCHUR;
  options.use_explicit_schur_complement = true; 
  options.linear_solver_type  = ceres::ITERATIVE_SCHUR;
  options.preconditioner_type = ceres::SCHUR_JACOBI;
  options.use_explicit_schur_complement = false; // Only matters with ITERATIVE_SCHUR
  
  // Solve the problem
  vw_out() << "Starting the Ceres optimizer." << std::endl;
  ceres::Solver::Summary summary;
  ceres::Solve(options, &problem, &summary);
  vw_out() << summary.FullReport() << "\n";
  if (summary.termination_type == ceres::NO_CONVERGENCE) 
    vw_out() << "Found a valid solution, but did not reach the actual minimum. "
             << "This is expected, and likely the produced solution is good enough.\n";

  // With the problem solved, update camera_models based on frame_params
  // (applies only to frame cameras, if any)
  updateFrameCameras(csm_models, frame_params);  

  // By now the cameras have been updated in-place. Compute the optimized
  // camera centers.
  std::vector<vw::Vector3> opt_cam_positions;
  asp::calcCameraCenters(opt.camera_models, opt_cam_positions);

  // Save residuals after optimization
  residual_prefix = opt.out_prefix + "-final_residuals";
  saveJitterResiduals(problem, residual_prefix, opt, cnet, crn, opt.datum,
                 tri_points_vec, outliers, weight_per_residual,
                 pixel_vec, weight_vec, isAnchor_vec, pix2xyz_index);

  saveOptimizedCameraModels(opt.out_prefix, opt.stereo_session,
                            opt.image_files, opt.camera_files,
                            opt.camera_models, opt.update_isis_cubes_with_csm_state);
  
  // Compute the change in camera centers.
  std::string cam_offsets_file = opt.out_prefix + "-camera_offsets.txt";
  if (opt.datum.name() != asp::UNSPECIFIED_DATUM) 
    asp::saveCameraOffsets(opt.datum, opt.image_files, 
                           orig_cam_positions, opt_cam_positions,
                           cam_offsets_file); 

  std::string tri_offsets_file = opt.out_prefix + "-triangulation_offsets.txt";     
  asp::saveTriOffsetsPerCamera(opt.image_files, outliers,
                               orig_tri_points_vec, tri_points_vec,
                               crn, tri_offsets_file);
  return;
}

} // end namespace asp
  
int main(int argc, char* argv[]) {

  try {
    xercesc::XMLPlatformUtils::Initialize();
    
    asp::run_jitter_solve(argc, argv);
    
    xercesc::XMLPlatformUtils::Terminate();
  } ASP_STANDARD_CATCHES;

  return 0;
}
