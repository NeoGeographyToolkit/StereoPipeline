// __BEGIN_LICENSE__
//  Copyright (c) 2009-2026, United States Government as represented by the
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

/// \file RigParseOptions.cc
///
/// Parse and validate rig_calibrator command-line options.

#include <asp/Rig/RigParseOptions.h>
#include <asp/Rig/thread.h>
#include <asp/Rig/RigParseUtils.h>
#include <asp/Rig/rig_config.h>
#include <asp/Rig/basic_algs.h>
#include <asp/Core/AspProgramOptions.h>

#include <vw/Core/Exception.h>
#include <vw/Core/StringUtils.h>

#include <boost/program_options.hpp>

#include <glog/logging.h>

namespace po = boost::program_options;

namespace rig {

// Parse camera_position_uncertainty_str if non-empty
void handleCamPosUncertainty(RigOptions& opt) {

  if (opt.camera_position_uncertainty_str.empty())
    return;

  std::string sep = ",";
  std::vector<double> vals = vw::str_to_std_vec(opt.camera_position_uncertainty_str, sep);

  // Check if values were parsed
  if (vals.empty())
    vw::vw_throw(vw::ArgumentErr() << "Camera position uncertainty string is invalid.\n");

  // If size is 1, add a second value equal to the first. Only the first will be used,
  // but need to respect the api.
  if (vals.size() == 1)
    vals.push_back(vals[0]);

  // Validate that values are positive
  if (vals[0] <= 0.0 || vals[1] <= 0.0)
    vw::vw_throw(vw::ArgumentErr() << "Camera position uncertainty values must be positive.\n");

  opt.camera_position_uncertainty.resize(1);
  opt.camera_position_uncertainty[0] = vw::Vector2(vals[0], vals[1]);
}

void handleRigArgs(int argc, char *argv[], RigOptions& opt) {

  po::options_description general_options("");
  general_options.add_options()
    ("rig-config", po::value(&opt.rig_config)->default_value(""),
     "Read the rig configuration from this file.")
    ("nvm", po::value(&opt.nvm)->default_value(""),
     "Read images and camera poses from this nvm file, as exported by Theia.")
    ("image-sensor-list", po::value(&opt.image_sensor_list)->default_value(""),
     "Read image name, sensor name, and timestamp, from each line in this list. "
     "Alternatively, a directory structure can be used.")
    ("robust-threshold", po::value(&opt.robust_threshold)->default_value(0.5),
     "Residual pixel errors and 3D point residuals (the latter multiplied by "
     "corresponding weight) much larger than this will be logarithmically attenuated "
     "to affect less the cost function.")
    ("num-iterations", po::value(&opt.num_iterations)->default_value(100),
     "How many solver iterations to perform in calibration.")
    ("bracket-len", po::value(&opt.bracket_len)->default_value(0.6),
     "Lookup non-reference cam images only between consecutive ref cam images whose "
     "distance in time is no more than this (in seconds), after adjusting for the "
     "timestamp offset between these cameras. It is assumed the rig moves slowly and "
     "uniformly during this time. A large value here will make the calibrator compute "
     "a poor solution but a small value may prevent enough images being bracketed. See "
     "also --bracket-single-image.")
    ("intrinsics-to-float", po::value(&opt.intrinsics_to_float_str)->default_value(""),
     "Specify which intrinsics to float for each sensor. Example: "
     "'cam1:focal_length,optical_center,distortion cam2:focal_length'.")
    ("camera-poses-to-float", po::value(&opt.camera_poses_to_float_str)->default_value(""),
     "Specify the cameras for which sensors can have their poses floated. Example: "
     "'cam1 cam3'. The documentation has more details.")
    ("depth-to-image-transforms-to-float",
     po::value(&opt.depth_to_image_transforms_to_float_str)->default_value(""),
     "Specify for which sensors to float the depth-to-image transform (if depth data "
     "exists). Example: 'cam1 cam3'.")
    ("fix-rig-translations", po::bool_switch(&opt.fix_rig_translations)->default_value(false),
     "Fix the translation component of the transforms between the sensors on a rig. "
     "Works only when --no-rig is not set.")
    ("fix-rig-rotations", po::bool_switch(&opt.fix_rig_rotations)->default_value(false),
     "Fix the rotation component of the transforms between the sensors on a rig. Works "
     "only when --no-rig is not set.")
    ("float-scale", po::bool_switch(&opt.float_scale)->default_value(false),
     "If to optimize the scale of the clouds, part of depth-to-image transform. If kept "
     "fixed, the configuration of cameras should adjust to respect the given scale. This "
     "parameter should not be used with --affine-depth-to-image when the transform is "
     "affine, rather than rigid and a scale.")
    ("float-timestamp-offsets",
     po::bool_switch(&opt.float_timestamp_offsets)->default_value(false),
     "If to optimize the timestamp offsets among the cameras. This is experimental.")
    ("timestamp-offsets-max-change",
     po::value(&opt.timestamp_offsets_max_change)->default_value(1.0),
     "If floating the timestamp offsets, do not let them change by more than this "
     "(measured in seconds). Existing image bracketing acts as an additional constraint.")
    ("tri-weight", po::value(&opt.tri_weight)->default_value(0.1),
     "The weight to give to the constraint that optimized triangulated points stay "
     "close to original triangulated points. A positive value will help ensure the "
     "cameras do not move too far, but a large value may prevent convergence. This "
     "does not get set for triangulated points at which --heights-from-dem or --mesh "
     "constraints are applied.")
    ("tri-robust-threshold", po::value(&opt.tri_robust_threshold)->default_value(0.1),
     "The robust threshold to use with the triangulation weight. Must be positive.")
    ("use-initial-triangulated-points",
     po::bool_switch(&opt.use_initial_triangulated_points)->default_value(false),
     "Use the triangulated points from the input nvm file. Together with --tri-weight, "
     "this ensures the cameras do not move too far from the initial solution. This will "
     "fail if additional interest point matches are created with --num-overlaps. If "
     "registration is used, the initial triangulated points are transformed appropriately.")
    ("depth-tri-weight", po::value(&opt.depth_tri_weight)->default_value(1000.0),
     "The weight to give to the constraint that depth measurements agree with triangulated "
     "points. Use a bigger number as depth errors are usually on the order of 0.01 meters "
     "while reprojection errors are on the order of 1 pixel.")
    ("mesh", po::value(&opt.mesh)->default_value(""),
     "Use this mesh to help constrain the calibration (in .ply format). Must use a "
     "positive --mesh-tri-weight.")
    ("mesh-tri-weight", po::value(&opt.mesh_tri_weight)->default_value(0.0),
     "A larger value will give more weight to the constraint that triangulated points "
     "stay close to the mesh. Not suggested by default.")
    ("depth-mesh-weight", po::value(&opt.depth_mesh_weight)->default_value(0.0),
     "A larger value will give more weight to the constraint that the depth clouds stay "
     "close to the mesh. Not suggested by default.")
    ("camera-position-uncertainty",
     po::value(&opt.camera_position_uncertainty_str)->default_value(""),
     "Camera position uncertainty (1 sigma, in meters). This strongly constrains the "
     "movement of cameras, potentially at the expense of accuracy. Specify a single value.")
    ("heights-from-dem", po::value(&opt.heights_from_dem)->default_value(""),
     "Use this DEM to constrain the triangulated points. The uncertainty of the DEM is "
     "specified via --heights-from-dem-uncertainty.")
    ("heights-from-dem-uncertainty",
     po::value(&opt.heights_from_dem_uncertainty)->default_value(-1.0),
     "Uncertainty (in meters, 1 sigma) for --heights-from-dem. A smaller value constrains "
     "more the triangulated points to the DEM specified via --heights-from-dem.")
    ("heights-from-dem-robust-threshold",
     po::value(&opt.heights_from_dem_robust_threshold)->default_value(0.1),
     "Robust threshold for residual errors in triangulated points relative to DEM "
     "specified via --heights-from-dem. This is applied after the point differences "
     "are divided by --heights-from-dem-uncertainty. It will attenuate large height "
     "differences. Set to 0 to turn off.")
    ("affine-depth-to-image",
     po::bool_switch(&opt.affine_depth_to_image)->default_value(false),
     "Assume that the depth-to-image transform for each depth + image camera is an "
     "arbitrary affine transform rather than scale * rotation + translation.")
    ("num-passes", po::value(&opt.num_passes)->default_value(2),
     "How many passes of optimization to do. Outliers will be removed after every pass. "
     "Each pass will start with the previously optimized solution as an initial guess. "
     "Mesh intersections (if applicable) and ray triangulation will be recomputed before "
     "each pass.")
    ("initial-max-reprojection-error",
     po::value(&opt.initial_max_reprojection_error)->default_value(300.0),
     "If filtering outliers, remove interest points for which the reprojection error, in "
     "pixels, is larger than this. This filtering happens when matches are created, before "
     "cameras are optimized, and a big value should be used if the initial cameras are not "
     "trusted.")
    ("max-reprojection-error", po::value(&opt.max_reprojection_error)->default_value(25.0),
     "If filtering outliers, remove interest points for which the reprojection error, in "
     "pixels, is larger than this. This filtering happens after each optimization pass "
     "finishes, unless disabled. It is better to not filter too aggressively unless "
     "confident of the solution.")
    ("min-triangulation-angle", po::value(&opt.min_triangulation_angle)->default_value(0.01),
     "If filtering outliers, remove triangulated points for which all rays converging to "
     "it make an angle (in degrees) less than this. Note that some cameras in the rig may "
     "be very close to each other relative to the triangulated points, so care is needed "
     "here.")
    ("out-texture-dir", po::value(&opt.out_texture_dir)->default_value(""),
     "If non-empty and if an input mesh was provided, project the camera images using the "
     "optimized poses onto the mesh and write the obtained .obj files in the given "
     "directory.")
    ("min-ray-dist", po::value(&opt.min_ray_dist)->default_value(0.0),
     "The minimum search distance from a starting point along a ray when intersecting the "
     "ray with a mesh, in meters (if applicable).")
    ("max-ray-dist", po::value(&opt.max_ray_dist)->default_value(100.0),
     "The maximum search distance from a starting point along a ray when intersecting the "
     "ray with a mesh, in meters (if applicable).")
    ("registration", po::bool_switch(&opt.registration)->default_value(false),
     "If true, and registration control points for the sparse map exist and are specified "
     "by --hugin-file and --xyz-file, register all camera poses and the rig transforms "
     "before starting the optimization. For now, the depth-to-image transforms do not "
     "change as result of this, which may be a problem. To apply the registration only, "
     "use zero iterations.")
    ("hugin-file", po::value(&opt.hugin_file)->default_value(""),
     "The path to the hugin .pto file used for registration.")
    ("xyz-file", po::value(&opt.xyz_file)->default_value(""),
     "The path to the xyz file used for registration.")
    ("skip-post-registration", po::bool_switch(&opt.skip_post_registration)->default_value(false),
     "If true and registration to world coordinates takes place, do not apply the "
     "registration again after the cameras are optimized. This is usually not recommended, "
     "unless one is quite confident that other constraints (such as using --tri-weight or "
     "--mesh-tri-weight) are sufficient to keep the cameras from drifting.")
    ("parameter-tolerance", po::value(&opt.parameter_tolerance)->default_value(1e-12),
     "Stop when the optimization variables change by less than this.")
    ("no-rig", po::bool_switch(&opt.no_rig)->default_value(false),
     "Do not assumes the cameras are on a rig. Hence the pose of any camera of any sensor "
     "type may vary on its own and not being tied to other sensor types. See also "
     "--camera-poses-to-float.")
    ("out-dir,o", po::value(&opt.out_prefix)->default_value(""),
     "Save in this directory the camera intrinsics and extrinsics. See also "
     "--save-images-and-depth-clouds, --save-matches, --verbose, and --in-dir.")
    ("no-nvm-matches", po::bool_switch(&opt.no_nvm_matches)->default_value(false),
     "Do not read interest point matches from the nvm file. So read only camera poses. "
     "This implies --num-overlaps is positive, to be able to find new matches.")
    ("camera-poses", po::value(&opt.camera_poses)->default_value(""),
     "Read the images and world-to-camera poses from this list. The same format is used "
     "when this tool saves the updated poses in the output directory. It is preferred to "
     "read the camera poses with the --nvm option, as then interest point matches will be "
     "read as well.")
    ("num-overlaps", po::value(&opt.num_overlaps)->default_value(0),
     "Match an image with this many images (of all camera types) following it in "
     "increasing order of timestamp value. Set to a positive value only if desired to find "
     "more interest point matches than read from the input nvm file. Not suggested by "
     "default. For advanced controls, run: rig_calibrator --help | grep -i sift.")
    ("use-initial-rig-transforms",
     po::bool_switch(&opt.use_initial_rig_transforms)->default_value(false),
     "Use the transforms between the sensors (ref_to_sensor_transform) of the rig "
     "specified via --rig-config to initialize all non-reference camera poses based on the "
     "reference camera poses and the rig transforms. If this option is not set, derive the "
     "rig transforms from the poses of individual cameras.")
    ("bracket-single-image", po::bool_switch(&opt.bracket_single_image)->default_value(false),
     "If more than one image from a given sensor is acquired between two consecutive "
     "reference sensor images, as measured by timestamps, keep only one, choosing the image "
     "that is closest to the midpoint of the interval formed by reference sensor "
     "timestamps. Only applicable without --no-rig.")
    ("extra-list", po::value(&opt.extra_list)->default_value(""),
     "Add to the SfM solution the camera poses for the additional images/depth clouds in "
     "this list. Use bilinear interpolation of poses in time and nearest neighbor "
     "extrapolation (within --bracket-len) and/or the rig constraint to find the new poses "
     "(will be followed by bundle adjustment refinement). This can give incorrect results "
     "if the new images are not very similar or not close in time to the existing ones. "
     "This list can contain entries for the data already present.")
    ("fixed-image-list", po::value(&opt.fixed_image_list_str)->default_value(""),
     "A file having a list of images, one per line, whose cameras will be fixed during "
     "optimization.")
    ("nearest-neighbor-interp",
     po::bool_switch(&opt.nearest_neighbor_interp)->default_value(false),
     "Use nearest neighbor interpolation (in time) when inserting extra camera poses.")
    ("read-nvm-no-shift", po::bool_switch(&opt.read_nvm_no_shift)->default_value(false),
     "Read an nvm file assuming that interest point matches were not shifted to the "
     "origin.")
    ("save-nvm-no-shift", po::bool_switch(&opt.save_nvm_no_shift)->default_value(false),
     "Save the optimized camera poses and inlier interest point matches to "
     "<out dir>/cameras_no_shift.nvm. Interest point matches are not offset relative to "
     "the optical center, which is not standard, but which allows this file to be "
     "self-contained and for the matches to be drawn with stereo_gui.")
    ("save-matches", po::bool_switch(&opt.save_matches)->default_value(false),
     "Save the inlier interest point matches. stereo_gui can be used to visualize these.")
    ("export-to-voxblox", po::bool_switch(&opt.export_to_voxblox)->default_value(false),
     "Save the depth clouds and optimized transforms needed to create a mesh with voxblox "
     "(if depth clouds exist).")
    ("save-pinhole-cameras", po::bool_switch(&opt.save_pinhole_cameras)->default_value(false),
     "Save the optimized cameras in ASP's Pinhole format. The distortion model gets saved "
     "if it is of radtan type (OpenCV radial-tangential distortion model).")
    ("save-transformed-depth-clouds",
     po::bool_switch(&opt.save_transformed_depth_clouds)->default_value(false),
     "Save the depth clouds with the camera transform applied to them to make them be in "
     "world coordinates.")
    ("num-threads", po::value(&opt.num_threads)->default_value(rig::defaultNumThreads()),
     "Number of threads to use.")
    ("num-match-threads", po::value(&opt.num_match_threads)->default_value(8),
     "How many threads to use in feature detection/matching. A large number can use a lot "
     "of memory.")
    ("verbose", po::bool_switch(&opt.verbose)->default_value(false),
     "Print a lot of verbose information about how matching goes.");

  general_options.add(vw::GdalWriteOptionsDescription(opt));
  po::options_description positional("");
  po::positional_options_description positional_desc;
  std::string usage("[options]");
  bool allow_unregistered = false;
  std::vector<std::string> unregistered;
  po::variables_map vm =
    asp::check_command_line(argc, argv, opt, general_options, general_options,
                            positional, positional_desc, usage,
                            allow_unregistered, unregistered);

  // Parse and validate camera position uncertainty
  handleCamPosUncertainty(opt);

  // Validation will happen in parameterValidation()
}

void parameterValidation(RigOptions const& opt) {

  if (opt.robust_threshold <= 0.0)
    LOG(FATAL) << "The robust threshold must be positive.\n";

  if (opt.bracket_len <= 0.0) LOG(FATAL) << "Bracket length must be positive.";

  if (opt.num_overlaps < 1 && (opt.nvm == "" || opt.no_nvm_matches))
    LOG(FATAL) << "No nvm file was specified or it is not desired to read its matches. "
               << "Then must set a positive --num-overlaps to be able to find new "
               << "interest point matches.";

  if (opt.timestamp_offsets_max_change < 0)
    LOG(FATAL) << "The timestamp offsets must be non-negative.";

  if (opt.min_triangulation_angle <= 0.0)
    LOG(FATAL) << "The min triangulation angle must be positive.\n";

  if (opt.depth_tri_weight < 0.0)
    LOG(FATAL) << "The depth weight must non-negative.\n";

  if (opt.mesh_tri_weight < 0.0)
    LOG(FATAL) << "The mesh weight must non-negative.\n";

  if (opt.depth_mesh_weight < 0.0)
    LOG(FATAL) << "The depth mesh weight must non-negative.\n";

  if (opt.tri_weight < 0.0)
    LOG(FATAL) << "The triangulation weight must non-negative.\n";

  if (opt.tri_weight > 0.0 && opt.tri_robust_threshold <= 0.0)
    LOG(FATAL) << "The triangulation robust threshold must be positive.\n";

  // Validate heights-from-dem options  
  if (!opt.heights_from_dem.empty() && opt.heights_from_dem_uncertainty <= 0.0)
    LOG(FATAL) << "The value of --heights-from-dem-uncertainty must be positive.\n";

  if (opt.heights_from_dem.empty() && opt.heights_from_dem_uncertainty > 0.0)
    LOG(FATAL) << "The value of --heights-from-dem-uncertainty is set, "
               << "but --heights-from-dem is not set.\n";

  if (opt.heights_from_dem_robust_threshold <= 0.0)
    LOG(FATAL) << "The value of --heights-from-dem-robust-threshold must be positive.\n";

  // Validate mesh and DEM exclusivity
  if (!opt.mesh.empty() && !opt.heights_from_dem.empty())
    LOG(FATAL) << "Cannot use both --mesh and --heights-from-dem simultaneously. "
               << "These constraints may conflict as they both try to constrain triangulated points "
               << "to different external reference positions.\n";

  if (opt.registration && (opt.xyz_file.empty() || opt.hugin_file.empty()))
    LOG(FATAL) << "In order to register the map, the hugin and xyz file must be specified.";

  if (opt.float_scale && opt.affine_depth_to_image)
    LOG(FATAL) << "The options --float-scale and --affine-depth-to-image should not be used "
               << "together. If the latter is used, the scale is always floated.\n";

  if (opt.no_rig && opt.float_timestamp_offsets)
      LOG(FATAL) << "Cannot float timestamps with option --no-rig.\n";

  if (opt.out_prefix == "")
    LOG(FATAL) << "The output directory was not specified.\n";

  if (opt.out_texture_dir != "" && opt.mesh == "")
      LOG(FATAL) << "Cannot project camera images onto a mesh if a mesh was not provided.\n";

  if (opt.rig_config == "")
    LOG(FATAL) << "Must specify the initial rig configuration via --rig-config.\n";

  if (opt.camera_poses != "" && opt.nvm != "")
    LOG(FATAL) << "Cannot specify both --nvm and --camera-poses.\n";

  if (opt.camera_poses == "" && opt.nvm == "")
    LOG(FATAL) << "Must specify the cameras via --nvm or --camera-poses.\n";

  if (opt.num_overlaps > 0 && opt.use_initial_triangulated_points)
    LOG(FATAL) << "Cannot use the initial triangulated points if new matches are created.\n";

  return;
}

void parseAuxRigOptions(RigOptions& opt, RigSet const& R) {
  rig::parse_intrinsics_to_float(opt.intrinsics_to_float_str, R.cam_names,
                                 opt.intrinsics_to_float);

  rig::parse_camera_names(R.cam_names, opt.camera_poses_to_float_str,
                        opt.camera_poses_to_float);

  rig::parse_camera_names(R.cam_names, opt.depth_to_image_transforms_to_float_str,
                        opt.depth_to_image_transforms_to_float);

  // Read a list of images to keep fixed, if provided
  if (!opt.fixed_image_list_str.empty())
    rig::readList(opt.fixed_image_list_str, opt.fixed_images);
}

} // end namespace rig

