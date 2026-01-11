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

// TODO(oalexan1): Modularize this code! 

// See the ASP documentation for how this tool works.

#include <asp/Rig/RigTypeDefs.h>
#include <asp/Core/AspProgramOptions.h>
#include <asp/Core/BundleAdjustUtils.h>
#include <asp/Rig/triangulation.h> 
#include <asp/Rig/thread.h> 

#include <vw/FileIO/FileUtils.h>
#include <vw/Core/Log.h>
#include <asp/Core/nvm.h>

// TODO(oalexan1): Move these to cost_function.cc, together will
// all logic for the cost function. 
#include <ceres/ceres.h>
#include <ceres/rotation.h>
#include <ceres/problem.h>
#include <ceres/solver.h>
#include <ceres/cost_function.h>
#include <ceres/loss_function.h>
#include <ceres/dynamic_numeric_diff_cost_function.h>
#include <ceres/numeric_diff_cost_function.h>
#include <ceres/autodiff_cost_function.h>

#include <opencv2/imgproc.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/core/utility.hpp>

#include <Rig/basic_algs.h>
#include <Rig/rig_utils.h>
#include <Rig/image_lookup.h>
#include <Rig/system_utils.h>
#include <Rig/transform_utils.h>
#include <Rig/interpolation_utils.h>
#include <Rig/interest_point.h>
#include <Rig/texture_processing.h>
#include <Rig/camera_image.h>
#include <Rig/rig_config.h>
#include <Rig/RigCameraUtils.h>
#include <asp/Rig/RigCostFunction.h>
#include <asp/Rig/rig_io.h>
#include <asp/Rig/RigImageIO.h>

#include <gflags/gflags.h>
#include <glog/logging.h>

#include <Eigen/Geometry>
#include <Eigen/Core>

#include <oneapi/tbb/task_arena.h>
#include <boost/filesystem.hpp>

#include <string>
#include <map>
#include <iostream>
#include <fstream>
#include <tuple>

namespace fs = boost::filesystem;

DEFINE_string(rig_config, "",
              "Read the rig configuration from this file.");

DEFINE_string(nvm, "",
              "Read images and camera poses from this nvm file, as exported by Theia.");

DEFINE_string(image_sensor_list, "",
              "Read image name, sensor name, and timestamp, from each line in this list. "
              "Alternatively, a directory structure can be used.");

DEFINE_double(robust_threshold, 0.5,
              "Residual pixel errors and 3D point residuals (the latter multiplied "
              "by corresponding weight) much larger than this will be "
              "logarithmically attenuated to affect less the cost function.");

DEFINE_int32(num_iterations, 100, "How many solver iterations to perform in calibration.");

DEFINE_double(bracket_len, 0.6,
              "Lookup non-reference cam images only between consecutive ref cam images "
              "whose distance in time is no more than this (in seconds), after adjusting "
              "for the timestamp offset between these cameras. It is assumed the rig "
              "moves slowly and uniformly during this time. A large value here will "
              "make the calibrator compute a poor solution but a small value may prevent "
              "enough images being bracketed. See also --bracket_single_image.");

DEFINE_string(intrinsics_to_float, "", "Specify which intrinsics to float for each sensor. "
              "Example: 'cam1:focal_length,optical_center,distortion cam2:focal_length'.");

DEFINE_string(camera_poses_to_float, "",
              "Specify the cameras for which sensors can have their poses "
              "floated. Example: 'cam1 cam3'. The documentation has more details.");

DEFINE_string(depth_to_image_transforms_to_float, "",
              "Specify for which sensors to float the depth-to-image transform "
              "(if depth data exists). Example: 'cam1 cam3'.");

DEFINE_bool(fix_rig_translations, false,
            "Fix the translation component of the transforms between the sensors on a "
            "rig. Works only when --no-rig is not set.");

DEFINE_bool(fix_rig_rotations, false,
            "Fix the rotation component of the transforms between the sensors on a "
            "rig. Works only when --no-rig is not set.");

DEFINE_bool(float_scale, false,
            "If to optimize the scale of the clouds, part of depth-to-image transform. "
            "If kept fixed, the configuration of cameras should adjust to respect the given "
            "scale. This parameter should not be used with --affine_depth_to_image when the "
            "transform is affine, rather than rigid and a scale.");

DEFINE_bool(float_timestamp_offsets, false,
            "If to optimize the timestamp offsets among the cameras. This is experimental.");

DEFINE_double(timestamp_offsets_max_change, 1.0,
              "If floating the timestamp offsets, do not let them change by more than this "
              "(measured in seconds). Existing image bracketing acts as an additional "
              "constraint.");

DEFINE_double(tri_weight, 0.1,
              "The weight to give to the constraint that optimized triangulated "
              "points stay close to original triangulated points. A positive value will "
              "help ensure the cameras do not move too far, but a large value may prevent "
              "convergence.");

DEFINE_double(tri_robust_threshold, 0.1,
              "The robust threshold to use with the triangulation weight. Must be positive.");

DEFINE_bool(use_initial_triangulated_points, false, "Use the triangulated "
            "points from the input nvm file. Together with --tri-weight, this ensures "
            "the cameras do not move too far from the initial solution. This will fail "
            "if additional interest point matches are created with --num_overlaps."
            "If registration is used, the initial triangulated points are transformed "
            "appropriately.");

DEFINE_double(depth_tri_weight, 1000.0,
              "The weight to give to the constraint that depth measurements agree with "
              "triangulated points. Use a bigger number as depth errors are usually on the "
              "order of 0.01 meters while reprojection errors are on the order of 1 pixel.");

DEFINE_string(mesh, "",
              "Use this mesh to help constrain the calibration (in .ply format). "
              "Must use a positive --mesh_tri_weight.");

DEFINE_double(mesh_tri_weight, 0.0,
              "A larger value will give more weight to the constraint that triangulated "
              "points stay close to the mesh. Not suggested by default.");

DEFINE_double(depth_mesh_weight, 0.0,
              "A larger value will give more weight to the constraint that the depth clouds "
              "stay close to the mesh. Not suggested by default.");

DEFINE_double(camera_position_weight, 0.0,
              "A constraint to keep the camera positions close to initial locations. "
              "A high value can impede convergence. This does not use a robust threshold "
              "(soft cost function).");

DEFINE_bool(affine_depth_to_image, false, "Assume that the depth-to-image transform "
            "for each depth + image camera is an arbitrary affine transform rather than "
            "scale * rotation + translation.");

DEFINE_int32(calibrator_num_passes, 2, "How many passes of optimization to do. Outliers "
              "will be removed after every pass. Each pass will start with the previously "
              "optimized solution as an initial guess. Mesh intersections (if applicable) "
              "and ray triangulation will be recomputed before each pass.");

DEFINE_double(initial_max_reprojection_error, 300.0, "If filtering outliers, remove interest "
              "points for which the reprojection error, in pixels, is larger than this. This "
              "filtering happens when matches are created, before cameras are optimized, and "
              "a big value should be used if the initial cameras are not trusted.");

DEFINE_double(max_reprojection_error, 25.0, "If filtering outliers, remove interest "
              "points for which the reprojection error, in pixels, is larger than this. "
              "This filtering happens after each optimization pass finishes, unless "
              "disabled. It is better to not filter too aggressively unless confident "
              "of the solution.");

DEFINE_double(min_triangulation_angle, 0.01, 
              "If filtering outliers, remove triangulated points "
              "for which all rays converging to it make an angle (in degrees) less than this. "
              "Note that some cameras in the rig may be very close to each other relative to "
              "the triangulated points, so care is needed here.");

DEFINE_string(out_texture_dir, "", "If non-empty and if an input mesh was provided, "
              "project the camera images using the optimized poses onto the mesh "
              "and write the obtained .obj files in the given directory.");

DEFINE_double(min_ray_dist, 0.0, "The minimum search distance from a starting point "
              "along a ray when intersecting the ray with a mesh, in meters (if "
              "applicable).");

DEFINE_double(max_ray_dist, 100.0, "The maximum search distance from a starting point "
              "along a ray when intersecting the ray with a mesh, in meters (if "
              "applicable).");

DEFINE_bool(registration, false,
            "If true, and registration control points for the sparse map exist and "
            "are specified by --hugin_file and --xyz_file, register all camera poses "
            "and the rig transforms before starting the optimization. For now, the "
            "depth-to-image transforms do not change as result of this, which may be a "
            "problem. To apply the registration only, use zero iterations.");

DEFINE_string(hugin_file, "", "The path to the hugin .pto file used for registration.");

DEFINE_string(xyz_file, "", "The path to the xyz file used for registration.");

DEFINE_bool(skip_post_registration, false,
            "If true and registration to world coordinates takes place, do not apply the "
            "registration again after the cameras are optimized. This is usually not "
            "recommended, unless one is quite confident that other constraints "
            "(such as using --tri_weight or --mesh_tri_weight) are sufficient to "
            "keep the cameras from drifting.");

DEFINE_double(parameter_tolerance, 1e-12, "Stop when the optimization variables change by "
              "less than this.");

DEFINE_bool(no_rig, false,
            "Do not assumes the cameras are on a rig. Hence the pose of any "
            "camera of any sensor type may vary on its own and not being tied to other "
            "sensor types. See also --camera_poses_to_float.");

DEFINE_string(out_dir, "",
              "Save in this directory the camera intrinsics and extrinsics. "
              "See also --save-images_and_depth_clouds, --save_matches, --verbose, "
              "and --in_dir.");

DEFINE_bool(no_nvm_matches, false,
            "Do not read interest point matches from the nvm file. So read only "
            "camera poses. This implies --num_overlaps is positive, to be able to "
            "find new matches.");
DEFINE_string(camera_poses, "",
              "Read the images and world-to-camera poses from this list. "
              "The same format is used when this tool saves the updated "
              "poses in the output directory. It is preferred to read the camera "
              "poses with the ``--nvm`` option, as then interest point matches will "
              "be read as well.");  

DEFINE_int32(num_overlaps, 0, "Match an image with this many images (of all camera "
             "types) following it in increasing order of timestamp value. Set to "
             "a positive value only if desired to find more interest point matches "
             "than read from the input nvm file. Not suggested by default. For "
             "advanced controls, run: rig_calibrator --help | grep -i sift.");

DEFINE_bool(use_initial_rig_transforms, false,
            "Use the transforms between the sensors (ref_to_sensor_transform) of the rig "
            "specified via --rig_config to initialize all non-reference camera poses based "
            "on the reference camera poses and the rig transforms. If this option is not "
            "set, derive the rig transforms from the poses of individual cameras.");

DEFINE_bool(bracket_single_image, false,
            "If more than one image from a given sensor is acquired between two "
            "two consecutive reference sensor images, as measured by timestamps, "
            "keep only one, choosing the image that is closest to the midpoint "
            "of the interval formed by reference sensor timestamps. Only applicable "
            "without --no_rig.");

DEFINE_string(extra_list, "",
              "Add to the SfM solution the camera poses for the additional images/depth "
              "clouds in this list. Use bilinear interpolation of poses in time and nearest "
              "neighbor extrapolation (within --bracket_len) and/or the rig constraint to "
              "find the new poses (will be followed by bundle adjustment refinement). "
              "This can give incorrect results if the new images are not very similar "
              "or not close in time to the existing ones. This list can contain entries "
              "for the data already present.");

DEFINE_string(fixed_image_list, "",
              "A file having a list of of images, one per line, whose cameras will be "
              "fixed during optimization.");

DEFINE_bool(nearest_neighbor_interp, false,
            "Use nearest neighbor interpolation (in time) when inserting extra camera poses.");

DEFINE_bool(read_nvm_no_shift, false,
            "Read an nvm file assuming that interest point matches were not shifted "
            "to the origin.");

DEFINE_bool(save_nvm_no_shift, false,
            "Save the optimized camera poses and inlier interest point matches to "
            "<out dir>/cameras_no_shift.nvm. Interest point matches are not offset "
            "relative to the optical center, which is not standard, but which "
            "allows this file to be self-contained and for the matches to be "
            "drawn with stereo_gui.");

DEFINE_bool(save_matches, false,
            "Save the inlier interest point matches. stereo_gui can be used to "
            "visualize these.");

DEFINE_bool(export_to_voxblox, false,
            "Save the depth clouds and optimized transforms needed to create "
            "a mesh with voxblox (if depth clouds exist).");

DEFINE_bool(save_pinhole_cameras, false,
            "Save the optimized cameras in ASP's Pinhole format. "
            "The distortion model gets saved if it is of ``radtan`` type (OpenCV "
            "radial-tangential distortion model).");

DEFINE_bool(save_transformed_depth_clouds, false,
            "Save the depth clouds with the camera transform applied to them to make "
            "them be in world coordinates.");

DEFINE_int32(num_threads, rig::defaultNumThreads(), "Number of threads to use.");

DEFINE_int32(num_match_threads, 8, "How many threads to use in feature "
             "detection/matching. A large number can use a lot of memory.");

DEFINE_bool(verbose, false,
            "Print a lot of verbose information about how matching goes.");

namespace rig {

void parameterValidation() {
    
  if (FLAGS_robust_threshold <= 0.0)
    LOG(FATAL) << "The robust threshold must be positive.\n";

  if (FLAGS_bracket_len <= 0.0) LOG(FATAL) << "Bracket length must be positive.";

  if (FLAGS_num_overlaps < 1 && (FLAGS_nvm == "" || FLAGS_no_nvm_matches))
    LOG(FATAL) << "No nvm file was specified or it is not desired to read its matches. "
               << "Then must set a positive --num_overlaps to be able to find new "
               << "interest point matches.";

  if (FLAGS_timestamp_offsets_max_change < 0)
    LOG(FATAL) << "The timestamp offsets must be non-negative.";

  if (FLAGS_min_triangulation_angle <= 0.0)
    LOG(FATAL) << "The min triangulation angle must be positive.\n";

  if (FLAGS_depth_tri_weight < 0.0)
    LOG(FATAL) << "The depth weight must non-negative.\n";

  if (FLAGS_mesh_tri_weight < 0.0)
    LOG(FATAL) << "The mesh weight must non-negative.\n";

  if (FLAGS_depth_mesh_weight < 0.0)
    LOG(FATAL) << "The depth mesh weight must non-negative.\n";

  if (FLAGS_tri_weight < 0.0)
    LOG(FATAL) << "The triangulation weight must non-negative.\n";

  if (FLAGS_tri_weight > 0.0 && FLAGS_tri_robust_threshold <= 0.0)
    LOG(FATAL) << "The triangulation robust threshold must be positive.\n";

  if (FLAGS_camera_position_weight < 0.0)
    LOG(FATAL) << "The camera position weight must non-negative.\n";
  
  if (FLAGS_registration && (FLAGS_xyz_file.empty() || FLAGS_hugin_file.empty()))
    LOG(FATAL) << "In order to register the map, the hugin and xyz file must be specified.";

  if (FLAGS_float_scale && FLAGS_affine_depth_to_image)
    LOG(FATAL) << "The options --float_scale and --affine_depth_to_image should not be used "
               << "together. If the latter is used, the scale is always floated.\n";

  if (FLAGS_no_rig && FLAGS_float_timestamp_offsets)
      LOG(FATAL) << "Cannot float timestamps with option --no_rig.\n";

  if (FLAGS_out_dir == "")
    LOG(FATAL) << "The output directory was not specified.\n";

  if (FLAGS_out_texture_dir != "" && FLAGS_mesh == "")
      LOG(FATAL) << "Cannot project camera images onto a mesh if a mesh was not provided.\n";

  if (FLAGS_rig_config == "")
    LOG(FATAL) << "Must specify the initial rig configuration via --rig_config.\n";

  if (FLAGS_camera_poses != "" && FLAGS_nvm != "")
    LOG(FATAL) << "Cannot specify both --nvm and --camera_poses.\n";

  if (FLAGS_camera_poses == "" && FLAGS_nvm == "")
    LOG(FATAL) << "Must specify the cameras via --nvm or --camera_poses.\n";

  if (FLAGS_num_overlaps > 0 && FLAGS_use_initial_triangulated_points)
    LOG(FATAL) << "Cannot use the initial triangulated points if new matches are created.\n";
    
  return;
}

} // namespace rig
           
int main(int argc, char** argv) {

  google::InitGoogleLogging(argv[0]);
  google::ParseCommandLineFlags(&argc, &argv, true);
  tbb::task_arena schedule(tbb::task_arena::automatic); // to force linking to tbb

  // Create the output directory, turn on logging, do validation
  std::string out_prefix = FLAGS_out_dir + "/run"; // part of the api
  vw::create_out_dir(out_prefix);
  asp::log_to_file(argc, argv, "", out_prefix);
  rig::parameterValidation();

  rig::RigSet R;
  rig::readRigConfig(FLAGS_rig_config, FLAGS_use_initial_rig_transforms, R);
  
  // Sanity check
  size_t max_num_sensors_per_rig = 0;
  for (size_t rig_it = 0; rig_it < R.cam_set.size(); rig_it++) 
    max_num_sensors_per_rig = std::max(max_num_sensors_per_rig, R.cam_set[rig_it].size()); 
  if (FLAGS_extra_list != "" && FLAGS_num_overlaps < max_num_sensors_per_rig)
    LOG(FATAL) << "If inserting extra images, must have --num_overlaps be at least "
                << "the number of sensors in the rig, and ideally more, to be able "
                << "to tie well the new images with the existing ones.\n";
                 
  // Optionally load the mesh
  mve::TriangleMesh::Ptr mesh;
  std::shared_ptr<mve::MeshInfo> mesh_info;
  std::shared_ptr<tex::Graph> graph;
  std::shared_ptr<BVHTree> bvh_tree;
  if (FLAGS_mesh != "")
    rig::loadMeshBuildTree(FLAGS_mesh, mesh, mesh_info, graph, bvh_tree);

  // Read a list of images to keep fixed, if provided
  std::set<std::string> fixed_images;
  if (!FLAGS_fixed_image_list.empty()) 
    rig::readList(FLAGS_fixed_image_list, fixed_images);

  // world_to_ref has the transforms from the ref cameras to the world,
  // while world_to_cam has the transforms from the world to all cameras,
  // including world_to_ref. Both of these are needed in certain circumstances,
  // and it is very important to always keep these in sync.
  std::vector<Eigen::Affine3d> world_to_ref, world_to_cam;

  // Read camera poses from nvm file or a list.
  // image_data is on purpose stored in vectors of vectors, with each
  // image_data[i] having data in increasing order of timestamps. This
  // way it is fast to find next timestamps after a given one.
  std::vector<rig::MsgMap> image_maps;
  std::vector<rig::MsgMap> depth_maps;
  asp::nvmData nvm;
  rig::readListOrNvm(FLAGS_camera_poses, FLAGS_nvm, 
                     FLAGS_image_sensor_list, FLAGS_extra_list,
                     FLAGS_use_initial_rig_transforms,
                     FLAGS_bracket_len, FLAGS_nearest_neighbor_interp, 
                     FLAGS_read_nvm_no_shift, R,
                     // outputs
                     nvm, image_maps, depth_maps); // out
  
  // Keep here the images, timestamps, and bracketing information
  std::vector<rig::cameraImage> cams;
  //  The range of R.ref_to_cam_timestamp_offsets[cam_type] before
  //  getting out of the bracket.
  std::vector<double> min_timestamp_offset, max_timestamp_offset;
  std::vector<double> ref_timestamps; // Timestamps for the ref cameras
  // Select the images to use. If the rig is used, keep non-ref images
  // only within the bracket.
  rig::lookupImages(// Inputs
                    FLAGS_no_rig, FLAGS_bracket_len,
                    FLAGS_timestamp_offsets_max_change,
                    FLAGS_bracket_single_image, 
                    R, image_maps, depth_maps,
                    // Outputs
                    ref_timestamps, world_to_ref,
                    cams, world_to_cam, min_timestamp_offset, max_timestamp_offset);
  // De-allocate data we no longer need
  image_maps = std::vector<rig::MsgMap>();
  depth_maps = std::vector<rig::MsgMap>();

  if (FLAGS_no_rig && FLAGS_use_initial_rig_transforms)
    LOG(FATAL) << "Cannot use initial rig transforms without a rig.\n";
  
  // If we can use the initial rig transform, compute and overwrite overwrite
  // world_to_cam, the transforms from the world to each non-reference camera.
  // TODO(oalexan1): Test if this works with --no_rig. For now this combination
  // is not allowed.
  if (!FLAGS_no_rig && FLAGS_use_initial_rig_transforms)
    rig::calcWorldToCamWithRig(// Inputs
                               !FLAGS_no_rig,
                               cams, world_to_ref, ref_timestamps,
                               R.ref_to_cam_trans,
                               R.ref_to_cam_timestamp_offsets,
                               // Output
                               world_to_cam);
  
  if (!FLAGS_no_rig && !FLAGS_use_initial_rig_transforms) {
    // If we want a rig, and cannot use the initial rig, use the
    // transforms from the world to each camera, compute the rig
    // transforms.
    rig::calc_rig_trans(cams, world_to_ref, world_to_cam, ref_timestamps,
                              R); // update this
  }
  
  // Determine if a given camera type has any depth information
  int num_cam_types = R.cam_names.size();
  std::vector<bool> has_depth(num_cam_types, false);
  for (size_t cid = 0; cid < cams.size(); cid++) {
    int cam_type = cams[cid].camera_type;
    if (cams[cid].depth_cloud.cols > 0 && cams[cid].depth_cloud.rows > 0)
      has_depth[cam_type] = true;
  }

  // Transform to world coordinates if control points were provided
  Eigen::Affine3d registration_trans;
  registration_trans.matrix() = Eigen::Matrix4d::Identity(); // default
  bool registration_applied = false;
  if (FLAGS_registration && FLAGS_hugin_file != "" && FLAGS_xyz_file != "") {
    // Keep user's R.depth_to_image transforms, and only transform only the image
    // cameras from Theia's abstract coordinate system to world coordinates.
    bool scale_depth = false;
    registration_applied = true;
    applyRegistration(FLAGS_no_rig, scale_depth, FLAGS_hugin_file, FLAGS_xyz_file,
                      has_depth, cams,
                      // Outputs
                      registration_trans, world_to_ref, world_to_cam, R);
  }

  // Put the rig transforms in arrays, so we can optimize them
  std::vector<double> ref_to_cam_vec(num_cam_types * rig::NUM_RIGID_PARAMS, 0.0);
  for (int cam_type = 0; cam_type < num_cam_types; cam_type++)
    rig::rigid_transform_to_array
      (R.ref_to_cam_trans[cam_type],
       &ref_to_cam_vec[rig::NUM_RIGID_PARAMS * cam_type]);

  // Put transforms of the reference cameras in a vector so we can optimize them.
  // TODO(oalexan1): Eliminate world_to_ref. Use only world_to_ref_vec.
  int num_ref_cams = world_to_ref.size();
  if (world_to_ref.size() != ref_timestamps.size())
    LOG(FATAL) << "Must have as many ref cam timestamps as ref cameras.\n";
  std::vector<double> world_to_ref_vec(num_ref_cams * rig::NUM_RIGID_PARAMS);
  for (int cid = 0; cid < num_ref_cams; cid++)
    rig::rigid_transform_to_array(world_to_ref[cid],
                                  &world_to_ref_vec[rig::NUM_RIGID_PARAMS * cid]);
  
  // Need the identity transform for when the cam is the ref cam, and
  // have to have a placeholder for the right bracketing cam which won't be used.
  // These need to have different pointers because CERES wants it that way.
  Eigen::Affine3d identity = Eigen::Affine3d::Identity();
  std::vector<double> ref_identity_vec(rig::NUM_RIGID_PARAMS),
    right_identity_vec(rig::NUM_RIGID_PARAMS);
  rig::rigid_transform_to_array(identity, &ref_identity_vec[0]);
  rig::rigid_transform_to_array(identity, &right_identity_vec[0]);

  // Which intrinsics from which cameras to float. Indexed by cam_type.
  std::vector<std::set<std::string>> intrinsics_to_float;
  rig::parse_intrinsics_to_float(FLAGS_intrinsics_to_float, R.cam_names,
                                       intrinsics_to_float);

  std::set<std::string> camera_poses_to_float;
  rig::parse_camera_names(R.cam_names, 
                                FLAGS_camera_poses_to_float,
                                camera_poses_to_float);

  std::set<std::string> depth_to_image_transforms_to_float;
  rig::parse_camera_names(R.cam_names, 
                          FLAGS_depth_to_image_transforms_to_float,
                          depth_to_image_transforms_to_float);
  
  // Set up the variable blocks to optimize for BracketedDepthError
  int num_depth_params = rig::NUM_RIGID_PARAMS;
  if (FLAGS_affine_depth_to_image)
    num_depth_params = rig::NUM_AFFINE_PARAMS;

  // TODO(oalexan1): affine_depth_to_image must allow
  // fixing the scale. 
  // Separate the initial scale. This is convenient if
  // cam_depth_to_image is scale * rotation + translation and if
  // it is desired to keep the scale fixed. In either case, the scale
  // will be multiplied back when needed.
  std::vector<double> depth_to_image_scales;
  for (int cam_type = 0; cam_type < num_cam_types; cam_type++) {
    double depth_to_image_scale
      = pow(R.depth_to_image[cam_type].matrix().determinant(), 1.0 / 3.0);
    R.depth_to_image[cam_type].linear() /= depth_to_image_scale;
    depth_to_image_scales.push_back(depth_to_image_scale);
  }

  // Put depth_to_image arrays, so we can optimize them
  std::vector<double> depth_to_image_vec(num_cam_types * num_depth_params);
  for (int cam_type = 0; cam_type < num_cam_types; cam_type++) {
    if (FLAGS_affine_depth_to_image)
      rig::affine_transform_to_array
        (R.depth_to_image[cam_type],
         &depth_to_image_vec[num_depth_params * cam_type]);
    else
      rig::rigid_transform_to_array
        (R.depth_to_image[cam_type],
         &depth_to_image_vec[num_depth_params * cam_type]);
  }

  // Put the intrinsics in arrays
  std::vector<double> focal_lengths(num_cam_types);
  std::vector<Eigen::Vector2d> optical_centers(num_cam_types);
  std::vector<Eigen::VectorXd> distortions(num_cam_types);
  for (int it = 0; it < num_cam_types; it++) {
    focal_lengths[it] = R.cam_params[it].GetFocalLength();  // average the two focal lengths
    optical_centers[it] = R.cam_params[it].GetOpticalOffset();
    distortions[it] = R.cam_params[it].GetDistortion();
  }

  // TODO(oalexan1): Eliminate world_to_cam, use only world_to_cam_vec
  // If using no extrinsics, each camera will float separately, using
  // world_to_cam as initial guesses. Use world_to_cam_vec as storage
  // for the camera poses to optimize.
  std::vector<double> world_to_cam_vec;
  if (FLAGS_no_rig) {
    world_to_cam_vec.resize(cams.size() * rig::NUM_RIGID_PARAMS);
    for (size_t cid = 0; cid < cams.size(); cid++)
      rig::rigid_transform_to_array(world_to_cam[cid],
                                    &world_to_cam_vec[rig::NUM_RIGID_PARAMS * cid]);
  }

  // Detect and match features if --num_overlaps > 0. Append the features
  // read from the nvm.
  rig::KeypointVec keypoint_vec;
  rig::PidCidFid pid_to_cid_fid;
  bool filter_matches_using_cams = true;
  std::vector<std::pair<int, int>> input_image_pairs; // will use num_overlaps instead
  // Do not save these matches. Only inlier matches will be saved later.
  bool local_save_matches = false;
  std::vector<Eigen::Vector3d> xyz_vec; // triangulated points go here
  rig::detectMatchAppendFeatures(// Inputs
                           cams, R.cam_params, FLAGS_out_dir, local_save_matches,
                           filter_matches_using_cams, world_to_cam,
                           FLAGS_num_overlaps, input_image_pairs,
                           FLAGS_initial_max_reprojection_error,
                           FLAGS_num_match_threads,
                           FLAGS_read_nvm_no_shift, FLAGS_no_nvm_matches,
                           FLAGS_verbose,
                           // Outputs
                           keypoint_vec, pid_to_cid_fid, xyz_vec, nvm);
  if (pid_to_cid_fid.empty())
    LOG(FATAL) << "No interest points were found. Must specify either "
               << "--nvm or positive --num_overlaps.\n";
  
  // Set up the block sizes
  std::vector<int> bracketed_cam_block_sizes;
  std::vector<int> bracketed_depth_block_sizes;
  std::vector<int> bracketed_depth_mesh_block_sizes;
  std::vector<int> xyz_block_sizes;
  rig::set_up_block_sizes(num_depth_params,
                          bracketed_cam_block_sizes, bracketed_depth_block_sizes,
                          bracketed_depth_mesh_block_sizes, xyz_block_sizes);

  // For a given fid = pid_to_cid_fid[pid][cid], the value
  // pid_cid_fid_inlier[pid][cid][fid] will be non-zero only if this
  // pixel is an inlier. Originally all pixels are inliers. Once an
  // inlier becomes an outlier, it never becomes an inlier again.
  rig::PidCidFidMap pid_cid_fid_inlier;
  
  // TODO(oalexan1): Must initialize all points as inliers outside this function,
  // as now this function resets those.
  rig::flagOutlierByExclusionDist(// Inputs
                                  R.cam_params, cams, pid_to_cid_fid,
                                  keypoint_vec,
                                  // Outputs
                                  pid_cid_fid_inlier);

  // Ensure that the triangulated points are kept in sync with the cameras
  if (FLAGS_use_initial_triangulated_points && registration_applied)
    rig::transformInlierTriPoints(registration_trans, pid_to_cid_fid, 
                                  pid_cid_fid_inlier, xyz_vec);
  
  // Structures needed to intersect rays with the mesh
  rig::PidCidFidToMeshXyz pid_cid_fid_mesh_xyz;
  std::vector<Eigen::Vector3d> pid_mesh_xyz;
  Eigen::Vector3d bad_xyz(1.0e+100, 1.0e+100, 1.0e+100);  // use this to flag invalid xyz

  // TODO(oalexan1): All the logic for one pass should be its own function,
  // as the block below is too big.
  for (int pass = 0; pass < FLAGS_calibrator_num_passes; pass++) {
    std::cout << "\nOptimization pass "
              << pass + 1 << " / " << FLAGS_calibrator_num_passes << "\n";

    // The transforms from the world to all cameras must be updated
    // given the current state of optimization
    // TODO(oalexan1): The call below is likely not necessary since this function
    // is already called earlier, and also whenever a pass finishes, see below.
    rig::calcWorldToCam(// Inputs
                        FLAGS_no_rig, cams, world_to_ref_vec, ref_timestamps, ref_to_cam_vec,
                        world_to_cam_vec, R.ref_to_cam_timestamp_offsets,
                        // Output
                        world_to_cam);

    // Triangulate, unless desired to reuse the initial points
    if (!FLAGS_use_initial_triangulated_points)
      rig::multiViewTriangulation(// Inputs
                                  R.cam_params, cams, world_to_cam, pid_to_cid_fid,
                                  keypoint_vec,
                                  // Outputs
                                  pid_cid_fid_inlier, xyz_vec);

    // This is a copy which won't change
    std::vector<Eigen::Vector3d> xyz_vec_orig;
    if (FLAGS_tri_weight > 0.0) {
      // Better copy manually to ensure no shallow copy
      xyz_vec_orig.resize(xyz_vec.size());
      for (size_t pt_it = 0; pt_it < xyz_vec.size(); pt_it++) {
        for (int coord_it = 0; coord_it < 3; coord_it++) {
          xyz_vec_orig[pt_it][coord_it] = xyz_vec[pt_it][coord_it];
        }
      } 
    }
    
    // Compute where each ray intersects the mesh
    if (FLAGS_mesh != "")
      rig::meshTriangulations(// Inputs
                              R.cam_params, cams, world_to_cam, pid_to_cid_fid,
                              pid_cid_fid_inlier, keypoint_vec, bad_xyz,
                              FLAGS_min_ray_dist, FLAGS_max_ray_dist, mesh, bvh_tree,
                              // Outputs
                              pid_cid_fid_mesh_xyz, pid_mesh_xyz);

    // For a given fid = pid_to_cid_fid[pid][cid], the value
    // pid_cid_fid_to_residual_index[pid][cid][fid] will be the index
    // in the array of residuals (look only at pixel residuals). This
    // structure is populated only for inliers, so its total number of
    // elements changes at each pass.
    rig::PidCidFidMap pid_cid_fid_to_residual_index;
    pid_cid_fid_to_residual_index.resize(pid_to_cid_fid.size());

    // Form the problem
    ceres::Problem problem;
    std::vector<std::string> residual_names;
    std::vector<double> residual_scales;
    rig::setupRigOptProblem(
        // Inputs
        cams, R, ref_timestamps, world_to_cam_vec, world_to_ref_vec,
        ref_to_cam_vec, ref_identity_vec, right_identity_vec, focal_lengths,
        optical_centers, distortions, depth_to_image_vec, depth_to_image_scales,
        keypoint_vec, pid_to_cid_fid, pid_cid_fid_inlier, pid_cid_fid_mesh_xyz,
        pid_mesh_xyz, xyz_vec, xyz_vec_orig,
        // Block sizes
        bracketed_cam_block_sizes, bracketed_depth_block_sizes,
        bracketed_depth_mesh_block_sizes, xyz_block_sizes, num_depth_params,
        // Configuration
        intrinsics_to_float, camera_poses_to_float,
        depth_to_image_transforms_to_float, fixed_images, min_timestamp_offset,
        max_timestamp_offset,
        // Flags
        FLAGS_no_rig, FLAGS_fix_rig_translations, FLAGS_fix_rig_rotations,
        FLAGS_float_timestamp_offsets, FLAGS_float_scale,
        FLAGS_affine_depth_to_image, (FLAGS_mesh != ""),
        FLAGS_robust_threshold, FLAGS_tri_robust_threshold, FLAGS_tri_weight,
        FLAGS_depth_tri_weight, FLAGS_depth_mesh_weight, FLAGS_mesh_tri_weight,
        FLAGS_camera_position_weight,
        // Outputs
        pid_cid_fid_to_residual_index, problem, residual_names, residual_scales);

    // Evaluate the residuals before optimization
    std::vector<double> residuals;
    rig::evalResiduals("before opt", residual_names, residual_scales, problem, residuals);

    if (pass == 0)
      rig::writeResiduals(FLAGS_out_dir, "initial", R.cam_names, cams, keypoint_vec,  
                          pid_to_cid_fid, pid_cid_fid_inlier, pid_cid_fid_to_residual_index,  
                          residuals);
    
    // Solve the problem
    ceres::Solver::Options options;
    ceres::Solver::Summary summary;
    options.linear_solver_type = ceres::ITERATIVE_SCHUR;
    options.num_threads = FLAGS_num_threads; 
    options.max_num_iterations = FLAGS_num_iterations;
    options.minimizer_progress_to_stdout = true;
    options.gradient_tolerance = 1e-16;
    options.function_tolerance = 1e-16;
    options.parameter_tolerance = FLAGS_parameter_tolerance;
    ceres::Solve(options, &problem, &summary);

    // The optimization is done. Right away copy the optimized states
    // to where they belong to keep all data in sync.
    if (!FLAGS_no_rig) {
      // Copy back the reference transforms
      for (int cid = 0; cid < num_ref_cams; cid++)
        rig::array_to_rigid_transform
          (world_to_ref[cid], &world_to_ref_vec[rig::NUM_RIGID_PARAMS * cid]);
    } else {
      // Each camera floats individually. Update world_to_cam from
      // optimized world_to_cam_vec.
      for (size_t cid = 0; cid < cams.size(); cid++) {
        rig::array_to_rigid_transform
          (world_to_cam[cid], &world_to_cam_vec[rig::NUM_RIGID_PARAMS * cid]);
      }
    }

    // Copy back the optimized intrinsics
    for (int cam_type = 0; cam_type < num_cam_types; cam_type++) {
      R.cam_params[cam_type].SetFocalLength(Eigen::Vector2d(focal_lengths[cam_type],
                                                          focal_lengths[cam_type]));
      R.cam_params[cam_type].SetOpticalOffset(optical_centers[cam_type]);
      R.cam_params[cam_type].SetDistortion(distortions[cam_type]);
    }

    // Copy back the optimized extrinsics, whether they were optimized or fixed
    for (int cam_type = 0; cam_type < num_cam_types; cam_type++)
      rig::array_to_rigid_transform
        (R.ref_to_cam_trans[cam_type],
         &ref_to_cam_vec[rig::NUM_RIGID_PARAMS * cam_type]);

    // Copy back the depth to image transforms without scales
    for (int cam_type = 0; cam_type < num_cam_types; cam_type++) {
      if (FLAGS_affine_depth_to_image)
        rig::array_to_affine_transform
          (R.depth_to_image[cam_type],
           &depth_to_image_vec[num_depth_params * cam_type]);
      else
        rig::array_to_rigid_transform(
          R.depth_to_image[cam_type],
          &depth_to_image_vec[num_depth_params * cam_type]);
    }

    // Evaluate the residuals after optimization
    rig::evalResiduals("after opt", residual_names, residual_scales, problem,
                             residuals);

    // Must have up-to-date world_to_cam and residuals to flag the outliers
    rig::calcWorldToCam(// Inputs
                        FLAGS_no_rig, cams, world_to_ref_vec, ref_timestamps, ref_to_cam_vec,
                        world_to_cam_vec, R.ref_to_cam_timestamp_offsets,
                        // Output
                        world_to_cam);

    // Flag outliers after this pass
    rig::flagOutliersByTriAngleAndReprojErr(// Inputs
       FLAGS_min_triangulation_angle, FLAGS_max_reprojection_error,
       pid_to_cid_fid, keypoint_vec,
       world_to_cam, xyz_vec, pid_cid_fid_to_residual_index, residuals,
       // Outputs
       pid_cid_fid_inlier);
    
    rig::writeResiduals(FLAGS_out_dir, "final", R.cam_names, cams, keypoint_vec,  
                        pid_to_cid_fid, pid_cid_fid_inlier,
                        pid_cid_fid_to_residual_index, residuals);
    
  }  // End optimization passes

  // Put back the scale in R.depth_to_image
  for (int cam_type = 0; cam_type < num_cam_types; cam_type++)
    R.depth_to_image[cam_type].linear() *= depth_to_image_scales[cam_type];

  if (FLAGS_save_matches)
    rig::saveInlierMatchPairs(cams, FLAGS_num_overlaps, pid_to_cid_fid,
                              keypoint_vec, pid_cid_fid_inlier, FLAGS_out_dir);

  // Update the transforms from the world to every camera
  rig::calcWorldToCam(  // Inputs
    FLAGS_no_rig, cams, world_to_ref_vec, ref_timestamps, ref_to_cam_vec, world_to_cam_vec,
    R.ref_to_cam_timestamp_offsets,
    // Output
    world_to_cam);

  // Redo the registration unless told not to.
  if (!FLAGS_skip_post_registration && FLAGS_registration &&
      FLAGS_hugin_file != "" && FLAGS_xyz_file != "") {
    // This time adjust the depth-to-image scale to be consistent with optimized cameras
    bool scale_depth = true;
    Eigen::Affine3d registration_trans;
    applyRegistration(FLAGS_no_rig, scale_depth, FLAGS_hugin_file, FLAGS_xyz_file,
                      has_depth, cams,
                      // Outputs
                      registration_trans, world_to_ref, world_to_cam, R);

    // Transform accordingly the triangulated points
    rig::transformInlierTriPoints(registration_trans, pid_to_cid_fid, 
                                        pid_cid_fid_inlier, xyz_vec);
  }
  
  // TODO(oalexan1): Why the call below works without rig:: prepended to it?
  if (FLAGS_out_texture_dir != "")
    rig::meshProjectCameras(R.cam_names, R.cam_params, cams, world_to_cam, 
                                  mesh, bvh_tree, FLAGS_out_texture_dir);

  rig::saveCameraPoses(FLAGS_out_dir, cams, world_to_cam);
  
  bool model_rig = (!FLAGS_no_rig);
  rig::writeRigConfig(FLAGS_out_dir + "/rig_config.txt", model_rig, R);

  std::string nvm_file = FLAGS_out_dir + "/cameras.nvm";
  bool shift_keypoints = true;
  rig::writeInliersToNvm(nvm_file, shift_keypoints, R.cam_params, cams,
                         world_to_cam, keypoint_vec,
                         pid_to_cid_fid, pid_cid_fid_inlier, xyz_vec);
  
  if (FLAGS_save_nvm_no_shift) {
    std::string nvm_file = FLAGS_out_dir + "/cameras_no_shift.nvm";
    bool shift_keypoints = false;
    rig::writeInliersToNvm(nvm_file, shift_keypoints, R.cam_params, cams,
                                 world_to_cam, keypoint_vec,
                                 pid_to_cid_fid, pid_cid_fid_inlier, xyz_vec);
  }

  if (FLAGS_export_to_voxblox)
    rig::exportToVoxblox(R.cam_names, cams, R.depth_to_image,
                               world_to_cam, FLAGS_out_dir);

  if (FLAGS_save_transformed_depth_clouds)
    rig::saveTransformedDepthClouds(R.cam_names, cams, R.depth_to_image,
                                          world_to_cam, FLAGS_out_dir);

  // Save the list of images (useful for bundle_adjust)
  std::string image_list = FLAGS_out_dir + "/image_list.txt";
  rig::saveImageList(cams, image_list); 

  if (FLAGS_save_pinhole_cameras)
    rig::writePinholeCameras(R.cam_names, R.cam_params, cams, 
                                   world_to_cam, FLAGS_out_dir);
  
  std::string conv_angles_file = FLAGS_out_dir + "/convergence_angles.txt";
  rig::savePairwiseConvergenceAngles(pid_to_cid_fid, keypoint_vec,
                                     cams, world_to_cam,  
                                     xyz_vec,  pid_cid_fid_inlier,  
                                     conv_angles_file);
  return 0;
}

