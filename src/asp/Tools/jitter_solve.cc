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

// TODO(oalexan1): Why jitter_solve does not use all threads?

#include <asp/Sessions/StereoSessionFactory.h>
#include <asp/Sessions/CameraUtils.h>
#include <asp/Camera/CsmModel.h>
#include <asp/Camera/CsmUtils.h>
#include <asp/Camera/BundleAdjustCamera.h>
#include <asp/Camera/BundleAdjustIsis.h>
#include <asp/Camera/JitterSolveCostFuns.h>
#include <asp/Camera/JitterSolveUtils.h>
#include <asp/Rig/rig_config.h>
#include <asp/Camera/JitterSolveRigUtils.h>
#include <asp/Camera/LinescanUtils.h>
#include <asp/Camera/BundleAdjustResiduals.h>
#include <asp/Core/Macros.h>
#include <asp/Core/AspProgramOptions.h>
#include <asp/Core/StereoSettings.h>
#include <asp/Core/BaseCameraUtils.h>
#include <asp/Core/BundleAdjustUtils.h>
#include <asp/Core/IpMatchingAlgs.h>
#include <asp/Core/CameraTransforms.h>
#include <asp/Core/ImageUtils.h>
#include <asp/Core/PointUtils.h>
#include <asp/Core/FileUtils.h>
#include <asp/Core/nvm.h>

#include <vw/BundleAdjustment/ControlNetwork.h>
#include <vw/BundleAdjustment/ControlNetworkLoader.h>
#include <vw/Core/Stopwatch.h>
#include <vw/Cartography/CameraBBox.h>
#include <vw/Cartography/GeoReferenceBaseUtils.h>
#include <vw/Camera/CameraImage.h>
#include <vw/Cartography/DatumUtils.h>
#include <vw/FileIO/FileTypes.h>
#include <vw/FileIO/FileUtils.h>

#include <usgscsm/UsgsAstroLsSensorModel.h>
#include <usgscsm/UsgsAstroFrameSensorModel.h>
#include <usgscsm/Utilities.h>

#include <xercesc/util/PlatformUtils.hpp>

namespace po = boost::program_options;
namespace fs = boost::filesystem;

using namespace vw;
using namespace vw::math;

namespace asp {

struct Options: public asp::BaBaseOptions {
  int num_lines_per_position, num_lines_per_orientation, num_anchor_points_per_image,
    num_anchor_points_per_tile;
  std::string anchor_weight_image;   
  std::string anchor_dem, rig_config;
  int num_anchor_points_extra_lines;
  bool initial_camera_constraint, fix_rig_translations, fix_rig_rotations,
    use_initial_rig_transforms;
  double quat_norm_weight, anchor_weight, roll_weight, yaw_weight, smoothness_weight;
  std::map<int, int> cam2group;
};
    
void handle_arguments(int argc, char *argv[], Options& opt, rig::RigSet & rig) {

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
     "Resample the input camera orientations, using this many lines per produced "
     "orientation. If not set, use the orientations from the CSM file as they are.")
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
    ("nvm", po::value(&opt.nvm)->default_value(""),
     "Read a control network having interest point matches from this file in the NVM "
     "format. This can be used with any images and cameras supported by ASP.")
    ("min-matches", po::value(&opt.min_matches)->default_value(30),
     "Set the minimum  number of matches between images that will be considered.")
    ("max-pairwise-matches", po::value(&opt.max_pairwise_matches)->default_value(10000),
     "Reduce the number of matches per pair of images to at most this "
     "number, by selecting a random subset, if needed. This happens "
     "when setting up the optimization, and before outlier filtering.")
    ("min-triangulation-angle", po::value(&opt.min_triangulation_angle)->default_value(0.1),
     "The minimum angle, in degrees, at which rays must meet at a triangulated point to "
     "accept this point as valid. It must be a positive value.")
    ("max-initial-reprojection-error", 
     po::value(&opt.max_init_reproj_error)->default_value(20),
     "Filter as outliers triangulated points project using initial cameras with error more "
     "than this, measured in pixels. Since jitter corrections are supposed to be small and "
     "cameras bundle-adjusted by now, this value need not be too big. Does not apply to "
     "GCP.")
    ("robust-threshold", po::value(&opt.robust_threshold)->default_value(0.5),
     "Set the threshold for the Cauchy robust cost function. Increasing this makes "
     "the solver focus harder on the larger errors.")
    ("image-list", po::value(&opt.image_list)->default_value(""),
     "A file containing the list of images, when they are too many to specify on the "
     "command line. Use space or newline as separator. See also --camera-list.")
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
     "The robust threshold to use to keep the triangulated points close to the DEM if "
      "specified via --heights-from-dem. This is applied after the point differences "
      "are divided by --heights-from-dem-uncertainty. It will attenuate large height "
      "difference outliers. It is suggested to not modify this value, and adjust instead "
      "--heights-from-dem-uncertainty.")
    ("num-anchor-points", po::value(&opt.num_anchor_points_per_image)->default_value(0),
     "How many anchor points to create per image. They will be uniformly distributed.")
    ("num-anchor-points-per-tile", 
     po::value(&opt.num_anchor_points_per_tile)->default_value(0),
     "How many anchor points to create per 1024 x 1024 image tile. They will be uniformly "
     "distributed. Useful when images of vastly different sizes (such as frame and "
     "linescan) are used together.")
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
    ("camera-position-uncertainty",  
     po::value(&opt.camera_position_uncertainty_str)->default_value(""),
     "A file having on each line the image name and the horizontal and vertical camera "
     "position uncertainty (1 sigma, in meters). This strongly constrains the movement of "
     "cameras to within the given values, potentially at the expense of accuracy.")
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
    ("reference-terrain", po::value(&opt.reference_terrain)->default_value(""),
     "An externally provided trustworthy reference terrain to use as a constraint. It can "
     "be either a DEM or a point cloud in CSV format. It must be well-aligned with the "
     "input cameras.")
    ("max-num-reference-points", 
     po::value(&opt.max_num_reference_points)->default_value(50000),
     "Maximum number of (randomly picked) points from the --reference-terrain dataset.")
    ("stereo-prefix-list", po::value(&opt.stereo_prefix_list)->default_value(""),
     "List of stereo prefixes (one per line) having disparities for the "
     "--reference-terrain option.")
    ("reference-terrain-uncertainty", 
     po::value(&opt.reference_terrain_uncertainty)->default_value(1.0),
     "The uncertainty (1 sigma, in meters), for the dataset in --reference-terrain. "
     "A smaller value will result in a stronger constraint. It is suggested to not "
     "not make this too small as it may prevent convergence.")
    ("reference-terrain-robust-threshold",
     po::value(&opt.reference_terrain_robust_threshold)->default_value(0.1),
     "The robust threshold, in pixels, for the option --reference-terrain. It is suggested "
     "to not modify this value, and adjust instead --reference-terrain-uncertainty.")
    ("csv-format", 
     po::value(&opt.csv_format_str)->default_value(""), asp::csv_opt_caption().c_str())
    ("csv-srs", 
     po::value(&opt.csv_srs)->default_value(""),
     "The PROJ or WKT string for interpreting the entries in input CSV files.")
    ("rotation-weight", po::value(&opt.rotation_weight)->default_value(0.0),
     "A higher weight will penalize more deviations from the original camera "
     "orientations. This is not recommended. Use instead ground constraints and "
     "--camera-position-uncertainty.")
    ("mapproj-dem", po::value(&opt.mapproj_dem)->default_value(""),
     "If specified, mapproject every pair of matched interest points onto this DEM "
     "and compute their distance, then percentiles of such distances for each image "
     "vs the rest and each image pair. This is done after bundle adjustment "
     "and outlier removal. Measured in meters.")
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
    ("num-passes",
     po::value(&opt.num_passes)->default_value(2),
     "How many passes of jitter solving to do, with given number of iterations in each "
     "pass.")
    ("rig-config", po::value(&opt.rig_config)->default_value(""),
     "Assume that the cameras are acquired with a set of rigs with this configuration "
     "file. The intrinsics will be read, but not the transforms between sensors, as those "
     "will be auto-computed (unless --use-initial-rig-transforms is set). The optimized "
     "rig, including the sensor transforms, will be saved.")
    ("fix-rig-translations", 
     po::bool_switch(&opt.fix_rig_translations)->default_value(false)->implicit_value(true),
     "Fix the translation component of the transforms between the sensors on a "
     "rig.")
     ("fix-rig-rotations", 
      po::bool_switch(&opt.fix_rig_rotations)->default_value(false)->implicit_value(true),
     "Fix the rotation component of the transforms between the sensors on a "
     "rig.")
    ("use-initial-rig-transforms", 
     po::bool_switch(&opt.use_initial_rig_transforms)->default_value(false)->implicit_value(true),
     "Use the transforms between the sensors (ref_to_sensor_transform) of the rig "
     "given by --rig-config, instead of computing them from the poses of individual "
     "cameras.")
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
     "Weight image for anchor points. Limits where anchor points are placed and their "
     "weight. These weights are additionally multiplied by --anchor-weight. See also "
     "--weight-image.")
     ("smoothness-weight", po::value(&opt.smoothness_weight)->default_value(0.0),
      "A weight to penalize high-frequency changes in the sequence of orientations "
      "in the linescan cameras being optimized. This is internally adjusted based "
      "on the initial curvature of the sequence of orientations. A value of 0.01 "
      "to 0.1 is recommended. This may impede convergence if high. Use with "
      "--camera-position-weight 1e+6 or so, to constrain the camera positions, "
      "to ensure that does not interfere with constraining the orientations.")
    ("initial-camera-constraint", 
     po::bool_switch(&opt.initial_camera_constraint)->default_value(false),
     "When constraining roll and yaw, measure these not in the satellite along-track/ "
     "across-track/down coordinate system, but relative to the initial camera poses. This "
     "is experimental. Internally, the roll weight will then be applied to the camera "
     "pitch angle (rotation around the camera y axis), because the camera coordinate "
     "system is rotated by 90 degrees in the sensor plane relative to the satellite "
     "coordinate system. The goal is the same, to penalize deviations that are not "
     "aligned with satellite pitch.")
    ("fix-gcp-xyz", 
     po::bool_switch(&opt.fix_gcp_xyz)->default_value(false)->implicit_value(true),
     "If the GCP are highly accurate, use this option to not float them during the optimization.")
    ("use-lon-lat-height-gcp-error",
     po::bool_switch(&opt.use_llh_error)->default_value(false)->implicit_value(true),
     "Constrain the triangulated points tied to GCP in the longitude, latitude, and height "
     "space, instead of ECEF. The standard deviations in the GCP file are applied "
     "accordingly.")
    ("accept-provided-mapproj-dem", 
     po::bool_switch(&asp::stereo_settings().accept_provided_mapproj_dem)->default_value(false)->implicit_value(true),
     "Accept the DEM provided on the command line as the one mapprojection was done with, "
     "even if it disagrees with the DEM recorded in the geoheaders of input images.")
    ;
    general_options.add(vw::GdalWriteOptionsDescription(opt));
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

  // Stereo settings must be set after the command line arguments are parsed.
  
  // Set this before loading cameras, as jitter can be modeled only with CSM
  // cameras.
  asp::stereo_settings().aster_use_csm = true;
  // TODO(oalexan1): This old option may need to be wiped given the newer
  // recent outlier filtering.
  asp::stereo_settings().ip_edge_buffer_percent = opt.ip_edge_buffer_percent;

  // Do this check first, as the output prefix is needed to log to file. This
  // will be triggered when called with no arguments, so print the general
  // options, which functions as the help message.
  if (opt.out_prefix == "") 
    vw_throw(ArgumentErr() << "Missing the output prefix.\n" << usage 
             << general_options);

  // Create the output directory
  vw::create_out_dir(opt.out_prefix);

  // Turn on logging to file (after the output directory is created)
  asp::log_to_file(argc, argv, "", opt.out_prefix);

  // For Glog
  google::InitGoogleLogging(argv[0]);

  // This must be done early
  boost::to_lower(opt.stereo_session);

  // Separate out GCP files
  bool rm_from_input_list = true;
  opt.gcp_files = vw::get_files_with_ext(opt.image_files, ".gcp", rm_from_input_list);
  int num_gcp_files = opt.gcp_files.size();
  if (num_gcp_files > 0)
    vw_out() << "Found " << num_gcp_files << " GCP files.\n";

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

    // This is needed when several frame camera images are acquired in quick succession
    asp::readGroupStructure(images_or_cams, opt.cam2group);
  }
  
  // Throw if there are duplicate camera file names.
  asp::check_for_duplicates(opt.image_files, opt.camera_files, opt.out_prefix);
  
  // Sanity check
  const int num_images = opt.image_files.size();
  if (opt.image_files.size() != opt.camera_files.size())
    vw_throw(ArgumentErr() << "Must have as many cameras as  have images.\n");
  
  if (opt.image_files.empty())
    vw_throw(ArgumentErr() << "Missing input image files.\n");
  
  // Must have this early check to print a clear message about unsupported
  // camera before any error thrown by StereoSessionFactory.
  std::string err_str; 
  try {
    std::string input_dem = ""; // No DEM
    bool allow_map_promote = false, quiet = true;
    asp::SessionPtr session;
      session.reset(asp::StereoSessionFactory::create
                      (opt.stereo_session, // may change
                      opt, opt.image_files[0], opt.image_files[0], 
                      opt.camera_files[0], opt.camera_files[0],
                      opt.out_prefix, input_dem,
                      allow_map_promote, quiet));
  } catch (const std::exception& e) {
    // Catch and record any error
    err_str = e.what();
  }
  // First check for unexpected sessions. Only dg, pleiades, aster, csm are allowed.
  if (opt.stereo_session != "dg" && opt.stereo_session != "pleiades" &&
      opt.stereo_session != "aster" && opt.stereo_session != "csm")
    vw_throw(ArgumentErr() << "Session " << opt.stereo_session 
             << " is not supported in jitter_solve. Check your camera files and/or "
             << "specify the -t (--session-type) option.\n");
  // Throw any other errors
  if (err_str != "")
    vw_throw(ArgumentErr() << err_str << "\n");
  
  if (opt.overlap_limit < 0)
    vw_throw(ArgumentErr() << "Must allow search for matches between "
             << "at least each image and its subsequent one.\n");
  
  // By default, try to match all of the images
  if (opt.overlap_limit == 0)
    opt.overlap_limit = opt.image_files.size();
  
  int num_pref = int(!opt.match_files_prefix.empty()) + int(!opt.clean_match_files_prefix.empty())
      + int(!opt.isis_cnet.empty()) + int(!opt.nvm.empty());
  if (num_pref > 1)
    vw_throw(ArgumentErr() << "Must specify no more than one of: --match-files-prefix, "
             << "--clean-match-files-prefix, --isis-cnet, --nvm.\n");
 if (num_pref == 0 && opt.gcp_files.empty())
    vw_throw(ArgumentErr() << "Neither interest point matches nor GCP were passed in.\n");
       
  if (opt.max_init_reproj_error <= 0.0)
    vw_throw(ArgumentErr() << "Must have a positive --max-initial-reprojection-error.\n");

  if (opt.tri_weight < 0.0) 
    vw_throw(ArgumentErr() << "The value of --tri-weight must be non-negative.\n");

  if (opt.robust_threshold <= 0.0) 
    vw_throw(ArgumentErr() << "The value of --robust-threshold must be positive.\n");

  if (opt.tri_robust_threshold <= 0.0) 
    vw_throw(ArgumentErr() << "The value of --tri-robust-threshold must be positive.\n");
  
  // This is a bug fix. The user by mistake passed in an empty height-from-dem string.
  if (!vm["heights-from-dem"].defaulted() && opt.heights_from_dem.empty())
    vw_throw(ArgumentErr() 
             << "The value of --heights-from-dem is empty. "
             << "Then it must not be set at all.\n");
   
   // Same for opt.anchor_dem
   if (!vm["anchor-dem"].defaulted() && opt.anchor_dem.empty())
    vw_throw(ArgumentErr() 
             << "The value of --anchor-dem is empty. Then it must not be set at all.\n");
     
  if (!vm["heights-from-dem-uncertainty"].defaulted() &&
      vm["heights-from-dem"].defaulted())
    vw_throw(ArgumentErr() 
             << "The value of --heights-from-dem-uncertainty is set, "
             << "but --heights-from-dem is not set.\n");

  if (opt.heights_from_dem_uncertainty <= 0.0) 
    vw_throw(ArgumentErr() << "The value of --heights-from-dem-uncertainty must be positive.\n");
  
  if (opt.heights_from_dem_robust_threshold <= 0.0) 
    vw_throw(ArgumentErr() 
             << "The value of --heights-from-robust-threshold must be positive.\n");

  // Options for reference terrain
  if (!vm["reference-terrain"].defaulted() && opt.reference_terrain.empty())
    vw_throw(ArgumentErr() 
             << "The value of --reference-terrain is set and empty. "
             << "Then it must not be set at all.\n");
  if (!vm["reference-terrain-uncertainty"].defaulted() &&
      vm["reference-terrain"].defaulted())
    vw_throw(ArgumentErr() 
             << "The value of --reference-terrain-uncertainty is set, "
             << "but --reference-terrain is not set.\n");
  if (opt.reference_terrain_uncertainty <= 0.0) 
    vw_throw(ArgumentErr() << "The value of --reference-terrain-uncertainty must be "
              << "positive.\n");
  if (opt.reference_terrain_robust_threshold <= 0.0) 
    vw_throw(ArgumentErr() << "The value of --heights-from-robust-threshold must be "
              << "positive.\n");
  if (!opt.reference_terrain.empty()) {
    
    if (opt.stereo_prefix_list.empty())
      vw_throw(ArgumentErr() 
               << "Must set --stereo-prefix-list when --reference-terrain is set.\n");
    if (!opt.rig_config.empty())
      vw_throw(ArgumentErr() 
               << "Cannot use --rig-config with --reference-terrain.\n");
    
    // Must have --csv-format unless the reference terrain is a DEM
    if (opt.csv_format_str.empty()) {
      vw::cartography::GeoReference georef;
      bool has_georef = vw::has_image_extension(opt.reference_terrain) &&
                        vw::cartography::read_georeference(georef, opt.reference_terrain);
      if (!has_georef)
        vw_throw(ArgumentErr() 
                 << "Must set --csv-format when --reference-terrain is set "
                 << "the terrain is not a DEM.\n");
    }
  }
  
  bool have_camera_position_uncertainty = !opt.camera_position_uncertainty_str.empty();
  bool have_datum = true; // Jitter solving always expects a datum
  if (have_camera_position_uncertainty)
   asp::handleCameraPositionUncertainty(opt, have_datum); 

  if (opt.rotation_weight < 0)
    vw_throw(ArgumentErr() << "The rotation weight must be non-negative.\n");
  
  if (opt.camera_position_weight < 0) 
    vw_throw(ArgumentErr() << "The value of --camera-position-weight must be n"
                           << "non-negative.\n");
    
  if (opt.camera_position_robust_threshold <= 0.0)
    vw_throw(ArgumentErr() << "The value of --camera-position-robust-threshold "
                            << "must be positive.\n");
      
  if (opt.quat_norm_weight <= 0)
    vw_throw(ArgumentErr() << "The quaternion norm weight must be positive.\n");

  if (opt.roll_weight < 0.0)
    vw_throw(ArgumentErr() << "The roll weight must be non-negative.\n");

  if (opt.yaw_weight < 0.0)
    vw_throw(ArgumentErr() << "The yaw weight must be non-negative.\n");

  // The smoothness weight must be non-negative
  if (opt.smoothness_weight < 0.0)
    vw_throw(ArgumentErr() << "The smoothness weight must be non-negative.\n");
    
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
    vw_throw(ArgumentErr() << "The anchor weight must be non-negative.\n");

  if ((opt.anchor_weight > 0  || opt.num_anchor_points_per_image > 0 || 
       opt.num_anchor_points_per_tile > 0 || opt.num_anchor_points_extra_lines > 0) &&
      opt.anchor_dem.empty()) 
    vw::vw_throw(vw::ArgumentErr() << "Anchor points parameters have been specified. "
                 << "Must set  --anchor-dem.\n");
  
  // Must have at least one pass
  if (opt.num_passes < 1)
    vw_throw(ArgumentErr() << "Must have at least one pass.\n");
    
  bool have_rig = !opt.rig_config.empty();
  if (have_rig) {
    bool have_rig_transforms = opt.use_initial_rig_transforms;
    rig::readRigConfig(opt.rig_config, have_rig_transforms, rig);
    
    for (size_t i = 0; i < rig.cam_params.size(); i++) {
      auto const& params = rig.cam_params[i];
      if (params.GetDistortion().size() != 0)
        vw::vw_throw(vw::ArgumentErr() << "Distortion is not supported in jitter_solve.\n");
    }
    
    if (opt.roll_weight > 0 || opt.yaw_weight > 0)
      vw::vw_throw(vw::ArgumentErr() << "Cannot use the roll/yaw constraint with a rig.\n");
  }
  
  if (!have_rig && 
      (opt.use_initial_rig_transforms || opt.fix_rig_translations || opt.fix_rig_rotations))
    vw::vw_throw(vw::ArgumentErr() << "Cannot use --use-initial-rig-transforms, "
                 << "--fix-rig-translations, or --fix-rig-rotations without a rig.\n");
  
  // If have both anchor DEM and height-from-dem, and these are difrerent, print
  // a warming that the user should check for their agreement.
  if (!opt.anchor_dem.empty() && !opt.heights_from_dem.empty() &&
      opt.anchor_dem != opt.heights_from_dem)
    vw::vw_out(vw::WarningMessage) 
      << "The values of --anchor-dem and --heights-from-dem are different. "
      << "Check (with geodiff) that these are in agreement.\n";
      
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

  bool warning_printed = false;
  
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

    UsgsAstroLsSensorModel * ls_model
      = dynamic_cast<UsgsAstroLsSensorModel*>((csm_models[icam]->m_gm_model).get());
    if (ls_model == NULL)
       extra = 0; // extra lines are only for linescan

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
           vw::pixel_cast<vw::PixelMask<float>>(interp_anchor_dem), anchor_georef, 
           treat_nodata_as_zero, has_intersection,
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
        
        if (ls_model != NULL) {
          // Anchor points must not be outside the range of tabulated positions and orientations
          csm::ImageCoord imagePt;
          asp::toCsmPixel(pix, imagePt);
          double time    = ls_model->getImageTime(imagePt);
          int numPos     = ls_model->m_positions.size() / NUM_XYZ_PARAMS;
          double posT0   = ls_model->m_t0Ephem;
          double posDt   = ls_model->m_dtEphem;
          int pos_index  = static_cast<int>((time - posT0) / posDt);
          int numQuat    = ls_model->m_quaternions.size() / NUM_QUAT_PARAMS;
          double quatT0  = ls_model->m_t0Quat;
          double quatDt  = ls_model->m_dtQuat;
          int quat_index = static_cast<int>((time - quatT0) / quatDt);
          if (pos_index < 0  || pos_index >= numPos || 
              quat_index < 0 || quat_index >= numQuat) {
            if (!warning_printed) {
              vw::vw_out(vw::WarningMessage) << "Not placing anchor points outside "
                << "the range of tabulated positions and orientations.\n";
              warning_printed = true;
            }
            continue; 
          }
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

// Apply the input adjustments to the CSM cameras. Resample linescan models.
// Get pointers to the underlying CSM cameras, as need to manipulate
// those directly. This modifies camera_models in place.
void initResampleCsmCams(Options                     const& opt,
                         std::vector<vw::CamPtr>     const& camera_models,
                         std::vector<asp::CsmModel*>      & csm_models) {

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
      vw_throw(ArgumentErr() 
               << "Expecting the cameras to be of CSM linescan or frame type.\n");

    // Normalize quaternions. Later, the quaternions being optimized will
    // be kept close to being normalized.  This makes it easy to ensure
    // that quaternion interpolation gives good results, especially that
    // some quaternions may get optimized and some not.
    if (ls_model != NULL) {
      asp::normalizeQuaternions(ls_model);
      // The provided tabulated positions, velocities and quaternions may be too few,
      // so resample them with --num-lines-per-position and --num-lines-per-orientation,
      // if those are set.
      resampleModel(opt.num_lines_per_position, opt.num_lines_per_orientation, ls_model);
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
    
    bool is_gcp = (cnet[ipt].type() == vw::ba::ControlPoint::GroundControlPoint);

    if (have_dem && dem_xyz_vec.at(ipt) != Vector3(0, 0, 0) && !is_gcp) {
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

// Run one pass of solving for jitter. At each pass the cameras we have so far
// are used to triangulate the points and the DEM constraint is refreshed if
// applicable, and then the cameras are optimized. More than one pass
// was shown to improve the accuracy.
void jitterSolvePass(int                                 pass,
                     bool                                have_rig,
                     Options                      const& opt,
                     asp::CRNJ                    const& crn,
                     std::vector<RigCamInfo>      const& rig_cam_info,
                     TimestampMap                 const& timestamp_map,
                     // Outputs
                     ba::ControlNetwork                & cnet,
                     std::set<int>                     & outliers,
                     std::vector<asp::CsmModel*>       & csm_models,
                     std::vector<std::vector<vw::Vector3>> & orig_cam_positions,
                     std::vector<double>               & orig_tri_points_vec,
                     std::vector<std::vector<double>>  & orig_curvatures,
                     rig::RigSet                       & rig,
                     std::vector<double>               & ref_to_curr_sensor_vec) {

  vw::vw_out() << "\nJitter solving pass: " << pass << "\n";
  
  // If some of the input cameras are frame, need to store position and
  // quaternion variables for them outside the camera model.
  // TODO(oalexan1): Revisit this decision now that frame camera
  // params are no longer private.
  std::vector<double> frame_params;
  initFrameCameraParams(csm_models, frame_params);
  
  // Update tri points from DEM and create anchor xyz from DEM.
  bool have_dem = (!opt.heights_from_dem.empty());
  std::vector<Vector3> dem_xyz_vec;
  vw::cartography::GeoReference dem_georef, anchor_georef;
  ImageViewRef<PixelMask<double>> interp_dem, interp_anchor_dem;
  bool warn_only = false; // for jitter solving we always know well the datum
  if (have_dem) {
    vw::vw_out() << "Reading the DEM for the --heights-from-dem constraint.\n";
    asp::create_interp_dem(opt.heights_from_dem, dem_georef, interp_dem);
    vw::checkDatumConsistency(opt.datum, dem_georef.datum(), warn_only);
    asp::update_tri_pts_from_dem(cnet, crn, outliers, opt.camera_models,
                               dem_georef, interp_dem,
                               // Output
                               dem_xyz_vec);
  }
  if (opt.anchor_dem != "") {
    vw::vw_out() << "Reading the DEM for the --anchor-dem constraint.\n";
    asp::create_interp_dem(opt.anchor_dem, anchor_georef, interp_anchor_dem);
    vw::checkDatumConsistency(opt.datum, anchor_georef.datum(), warn_only);
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
  if (num_cameras < 1)
    vw_throw(ArgumentErr() << "Expecting at least one input camera.\n");

  // Put the triangulated points in a vector. Update the cnet from the DEM,
  // if we have one. Later will add here the anchor points.
  std::vector<double> local_orig_tri_points_vec, tri_points_vec;
  formTriVec(dem_xyz_vec, have_dem,
    cnet, local_orig_tri_points_vec, tri_points_vec); // outputs
  
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
                     local_orig_tri_points_vec, tri_points_vec);
  
  // Save the original camera positions and triangulated points for the initial pass
  if (pass == 0) {
    orig_tri_points_vec = local_orig_tri_points_vec;
    asp::calcCameraCenters(opt.stereo_session, opt.camera_models, orig_cam_positions);
  }
      
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
                   have_rig, rig, rig_cam_info, opt.cam2group, timestamp_map,
                   opt.fix_rig_translations, opt.fix_rig_rotations,
                   // Outputs
                   tri_points_vec, frame_params, weight_per_residual, 
                   weight_per_cam, count_per_cam, ref_to_curr_sensor_vec, 
                   problem);
 
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
    
  // Add a cost function meant to tie up to known disparities (option
  // --reference-terrain). The structures below must persist until the end.
  std::vector<int> left_indices, right_indices;
  std::vector<asp::SessionPtr> sessions;
  std::vector<vw::TransformPtr> left_trans, right_trans;
  std::vector<std::string> disp_files;
  std::vector<DispPtr> disp_vec;
  std::vector<vw::Vector3> reference_vec; 
  std::vector<std::vector<int>> ref_indices;
  vw::ImageView<float> mapproj_dem;
  if (opt.reference_terrain != "") {
    asp::parseStereoRuns(opt.stereo_prefix_list, opt.image_files,
                         // Outputs
                         left_indices, right_indices, sessions, left_trans, right_trans,
                         disp_files);
    asp::addReferenceTerrainCostFunction(opt, csm_models, left_indices, right_indices,
                                         left_trans, right_trans, disp_files,
                                         // Outputs
                                         problem, disp_vec, mapproj_dem,
                                         weight_per_residual, // append
                                         reference_vec, ref_indices);
  }

  // Add the GCP constraint. GCP can come from GCP files or ISIS cnet.
  addGcpConstraint(opt, outliers, opt.use_llh_error, opt.fix_gcp_xyz,
                   cnet, tri_points_vec, weight_per_residual, problem); // outputs

  // Add the constraint to keep the camera positions close to initial values.
  // Note that in the second pass the initial values are the ones optimized in
  // the first pass. 
  // TODO(oalexan1): It is not clear how to best handle this. Also revisit
  // for bundle adjustment.
  if (opt.camera_position_uncertainty.size() > 0) 
    addHardCamPositionConstraint(opt, outliers, crn, csm_models, count_per_cam,
                                 opt.anchor_weight,
                                 have_rig, rig, rig_cam_info,
                                 // Outputs
                                 frame_params, weight_per_residual, problem);

  // Add another type of constraint to keep the camera positions close to initial values.
  // The earlier one is recommended as this one was not fully sorted out.
  // TODO(oalexan1): Need to wipe this option. The above does better.
  if (opt.camera_position_weight > 0) 
    addSoftCamPositionConstraint(opt, outliers, crn, csm_models, 
                                 weight_per_cam, count_per_cam,
                                 have_rig, rig, rig_cam_info,
                                 // Outputs
                                 frame_params, weight_per_residual, problem);
    
  // Add constraints to keep quat norm close to 1, and make rotations 
  // not change too much.
  // TODO(oalexan1): Need to parameterize the rotations with axis angle, then
  // convert from / to quaternions.
  addQuatNormRotationConstraints(opt, outliers, crn, csm_models,  
                                 have_rig, rig, rig_cam_info,
                                 opt.quat_norm_weight, 
                                 // Outputs
                                 frame_params,
                                 weight_per_residual,  // append
                                 problem);

  if (opt.roll_weight > 0 || opt.yaw_weight > 0)
    addRollYawConstraint(opt, crn, csm_models, roll_yaw_georef, 
                         opt.cam2group, 
                         opt.initial_camera_constraint,
                         opt.roll_weight, opt.yaw_weight,
                         // Outputs
                         frame_params, weight_per_residual, problem); // outputs

  // Add the smoothness constraint. This is a constraint on the curvature of the
  // sequence of poses.
  if (opt.smoothness_weight > 0)
    asp::addSmoothnessConstraint(opt, csm_models, opt.smoothness_weight,
                                 have_rig, rig, rig_cam_info,
                                 // Outputs
                                 weight_per_residual, orig_curvatures, 
                                 problem);
     
  // Save residuals before optimization
  if (pass == 0) {
    std::string residual_prefix = opt.out_prefix + "-initial_residuals";
    saveJitterResiduals(problem, residual_prefix, opt, cnet, crn, opt.datum,
                   tri_points_vec, outliers, weight_per_residual,
                   pixel_vec, weight_vec, isAnchor_vec, pix2xyz_index,
                   reference_vec, ref_indices);
  }
  
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
  vw_out() << "Starting the Ceres optimizer.\n";
  
  ceres::Solver::Summary summary;
  ceres::Solve(options, &problem, &summary);
  vw_out() << summary.FullReport() << "\n";
  if (summary.termination_type == ceres::NO_CONVERGENCE) 
    vw_out() << "Found a valid solution, but did not reach the actual minimum. "
             << "This is expected, and likely the produced solution is good enough.\n";

  // Update the cameras given the optimized parameters
  updateCameras(have_rig, rig, rig_cam_info, 
                opt.cam2group, timestamp_map, ref_to_curr_sensor_vec, 
                csm_models, frame_params);  

  // By now camera_models has been updated in-place. Compute the optimized
  // camera centers.
  std::vector<std::vector<vw::Vector3>> opt_cam_positions;
  asp::calcCameraCenters(opt.stereo_session, opt.camera_models, opt_cam_positions);

  // Save residuals after optimization
  std::string residual_prefix = opt.out_prefix + "-final_residuals";
  saveJitterResiduals(problem, residual_prefix, opt, cnet, crn, opt.datum,
                 tri_points_vec, outliers, weight_per_residual,
                 pixel_vec, weight_vec, isAnchor_vec, pix2xyz_index,
                 reference_vec, ref_indices);

  // Save the optimized camera models
  saveCsmCameras(opt.out_prefix, opt.stereo_session,
                 opt.image_files, opt.camera_files,
                 opt.camera_models, opt.update_isis_cubes_with_csm_state);
  
  if (have_rig) {
    // Update the rig with the optimized transforms and save it
    asp::updateRig(ref_to_curr_sensor_vec, rig);
    std::string rig_config = opt.out_prefix + "-rig_config.txt"; 
    rig::writeRigConfig(rig_config, have_rig, rig);
  }

  // Write many types of stats. These are done together as they rely on
  // reloading interest point matches, which is expensive.
  bool remove_outliers = true, propagate_errors = false, save_clean_matches = false;
  vw::Vector<double> horizontal_stddev_vec; // not used
  asp::matchFilesProcessing(cnet,
                            asp::BaBaseOptions(opt), // note the slicing
                            opt.camera_models, // these have been updated
                            remove_outliers, outliers, opt.mapproj_dem,
                            propagate_errors, horizontal_stddev_vec, 
                            save_clean_matches, opt.match_files);

  // Compute the change in camera centers
  std::string cam_offsets_file = opt.out_prefix + "-camera_offsets.txt";
  if (opt.datum.name() != asp::UNSPECIFIED_DATUM) 
    asp::saveCameraOffsets(opt.datum, opt.image_files, 
                           orig_cam_positions, opt_cam_positions,
                           cam_offsets_file);

  // Resize the tri_points_vec to eliminate the anchor points that were appended.
  // The number of those can be variable in each pass and those do not contribute
  // to the triangulation offsets. This must be at the end.
  int num_tri_points = cnet.size();
  size_t tri_len = num_tri_points*NUM_XYZ_PARAMS;
  if (orig_tri_points_vec.size() < tri_len || tri_points_vec.size() < tri_len)
    vw_throw(ArgumentErr() << "Expecting more triangulated points.\n");
  orig_tri_points_vec.resize(tri_len);
  tri_points_vec.resize(tri_len);
  std::string tri_offsets_file = opt.out_prefix + "-triangulation_offsets.txt";     
  asp::saveTriOffsetsPerCamera(opt.image_files, outliers,
                               orig_tri_points_vec, tri_points_vec,
                               crn, tri_offsets_file);

} // end jitterSolvePass

void run_jitter_solve(int argc, char* argv[]) {

  // Parse arguments and perform validation
  Options opt;
  rig::RigSet rig;
  handle_arguments(argc, argv, opt, rig);

  // Load the cameras  
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
  asp::SessionPtr session(NULL);
  bool found_datum = asp::datum_from_camera(opt.image_files[0], opt.camera_files[0],
                                             // Outputs
                                             opt.stereo_session, session, opt.datum);
  if (!found_datum)
    vw_throw(ArgumentErr() << "No datum was found in the input cameras.\n");
  
  // Apply the input adjustments to the cameras. Resample linescan models.
  // Get pointers to the underlying CSM cameras, as need to manipulate
  // those directly. These will result in changes to the input cameras.
  std::vector<asp::CsmModel*> csm_models;
  initResampleCsmCams(opt, opt.camera_models, csm_models);

  // Preparations if having a rig
  bool have_rig = (opt.rig_config != "");
  std::vector<RigCamInfo> rig_cam_info;
  std::vector<double> ref_to_curr_sensor_vec;
  TimestampMap timestamp_map;
  if (have_rig)
    populateRigCamInfo(rig, opt.image_files, opt.camera_files, csm_models, 
                       opt.cam2group, opt.use_initial_rig_transforms,
                       // Outputs
                       rig_cam_info, ref_to_curr_sensor_vec, timestamp_map);
  
  // Make a list of all the image pairs to find matches for. Some quantities
  // below are not needed but are part of the API.
  if (opt.isis_cnet.empty() && opt.nvm.empty()) {
    // TODO(oalexan1): Make this into a function
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

    // TODO(oalexan1): Make this into a function
    // Load match files
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
      opt.match_files[std::make_pair(i, j)] = match_file;
    }
  }
    
  // Build control network and perform triangulation with adjusted input cameras
  ba::ControlNetwork cnet("jitter_solve");
  if (opt.isis_cnet != "") {
    asp::IsisCnetData isisCnetData; // isis cnet (if loaded)
    vw::vw_out() << "Reading ISIS control network: " << opt.isis_cnet << "\n";
    asp::loadIsisCnet(opt.isis_cnet, opt.image_files,
                      cnet, isisCnetData); // outputs
  } else if (opt.nvm != "") {
      // Assume the features are stored shifted relative to optical center
      bool nvm_no_shift = false;
      std::vector<Eigen::Affine3d> world_to_cam; // poses will not be used
      std::map<std::string, Eigen::Vector2d> optical_offsets;
      asp::readNvmAsCnet(opt.nvm, opt.image_files, nvm_no_shift, 
                         cnet, world_to_cam, optical_offsets); // outputs
  } else {
    bool triangulate_control_points = true;
    vw::ba::build_control_network(triangulate_control_points,
                                  cnet, // output
                                  opt.camera_models, opt.image_files,
                                  opt.match_files, opt.min_matches,
                                  opt.min_triangulation_angle*(M_PI/180.0),
                                  opt.forced_triangulation_distance,
                                  opt.max_pairwise_matches);
  }
  
  if (!opt.gcp_files.empty()) {
    int num_gcp = vw::ba::add_ground_control_points(cnet, opt.gcp_files, opt.datum);
    checkGcpRadius(opt.datum, cnet);
    vw::vw_out() << "Loaded " << num_gcp << " ground control points.\n";
  }
  
  if (cnet.empty())
      vw::vw_throw(vw::ArgumentErr()
              << "Failed to build a control network. Check the bundle adjustment directory "
              << "for matches and if the match files satisfy the naming convention "
              << "<prefix>-<image1>__<image2>.match. "
              << "Or, if using an .nvm file, ISIS cnet, or GCP, check those.\n");

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
  
  // It is convenient to compute these inside the first pass rather than outside.
  // They should not go out of scope until the end of the program.
  std::vector<std::vector<vw::Vector3>> orig_cam_positions;
  std::vector<double> orig_tri_points_vec;
  std::vector<std::vector<double>> orig_curvatures;

  // Do this many passes
  for (int pass = 0; pass < opt.num_passes; pass++)
    jitterSolvePass(pass, have_rig, opt, crn, rig_cam_info, 
                    timestamp_map,
                    // Outputs
                    cnet, outliers, csm_models,
                    orig_cam_positions,
                    orig_tri_points_vec, orig_curvatures,
                    rig, ref_to_curr_sensor_vec);
  
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
