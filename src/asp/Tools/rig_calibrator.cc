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

#include <asp/Core/AspLog.h>
#include <asp/Core/BundleAdjustUtils.h>
#include <asp/Core/nvm.h>
#include <asp/Camera/BundleAdjustCamera.h>
#include <asp/Rig/RigOptions.h>
#include <asp/Rig/RigParseOptions.h>
#include <asp/Rig/RigTypeDefs.h>
#include <asp/Rig/triangulation.h> 
#include <asp/Rig/thread.h> 
#include <asp/Rig/RigParseUtils.h>
#include <asp/Rig/basic_algs.h>
#include <asp/Rig/rig_utils.h>
#include <asp/Rig/image_lookup.h>
#include <asp/Rig/system_utils.h>
#include <asp/Rig/transform_utils.h>
#include <asp/Rig/interpolation_utils.h>
#include <asp/Rig/interest_point.h>
#include <asp/Rig/RigOutlier.h>
#include <asp/Rig/texture_processing.h>
#include <asp/Rig/camera_image.h>
#include <asp/Rig/rig_config.h>
#include <asp/Rig/RigCameraUtils.h>
#include <asp/Rig/RigCostFunction.h>
#include <asp/Rig/RigData.h>
#include <asp/Rig/rig_io.h>
#include <asp/Rig/RigImageIO.h>

#include <vw/FileIO/FileUtils.h>
#include <vw/Core/Log.h>

#include <ceres/ceres.h>
#include <ceres/problem.h>
#include <ceres/solver.h>

#include <opencv2/imgproc.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/core/utility.hpp>

#include <glog/logging.h>

#include <Eigen/Geometry>
#include <Eigen/Core>

#include <oneapi/tbb/task_arena.h>
#include <boost/filesystem.hpp>

#include <string>
#include <map>
#include <iostream>
#include <fstream>

int main(int argc, char** argv) {

  // Some dependencies may still use google logging
  google::InitGoogleLogging(argv[0]);
  // Force linking to tbb for some dependencies
  tbb::task_arena schedule(tbb::task_arena::automatic); 

  rig::RigOptions opt;
  rig::handleRigArgs(argc, argv, opt);
  rig::parameterValidation(opt);

  // Create the output directory, turn on logging
  std::string out_prefix = opt.out_prefix + "/run"; // part of the api
  vw::create_out_dir(out_prefix);
  asp::log_to_file(argc, argv, "", out_prefix);

  // Rig configuration. The rig transforms may not exist yet.
  rig::RigSet R;
  rig::readRigConfig(opt.rig_config, opt.use_initial_rig_transforms, R);
  
  // Sanity check
  size_t max_num_sensors_per_rig = 0;
  for (size_t rig_it = 0; rig_it < R.cam_set.size(); rig_it++) 
    max_num_sensors_per_rig = std::max(max_num_sensors_per_rig, R.cam_set[rig_it].size()); 
  if (opt.extra_list != "" && opt.num_overlaps < max_num_sensors_per_rig)
    LOG(FATAL) << "If inserting extra images, must have --num_overlaps be at least "
                << "the number of sensors in the rig, and ideally more, to be able "
                << "to tie well the new images with the existing ones.\n";
                 
  // Optionally load the mesh
  mve::TriangleMesh::Ptr mesh;
  std::shared_ptr<mve::MeshInfo> mesh_info;
  std::shared_ptr<tex::Graph> graph;
  std::shared_ptr<BVHTree> bvh_tree;
  if (opt.mesh != "")
    rig::loadMeshBuildTree(opt.mesh, mesh, mesh_info, graph, bvh_tree);

  // Read a list of images to keep fixed, if provided
  std::set<std::string> fixed_images;
  if (!opt.fixed_image_list.empty()) 
    rig::readList(opt.fixed_image_list, fixed_images);

  // Read camera poses from nvm file or a list.
  std::vector<rig::MsgMap> image_maps;
  std::vector<rig::MsgMap> depth_maps;
  asp::nvmData nvm;
  rig::readListOrNvm(opt.camera_poses, opt.nvm, 
                     opt.image_sensor_list, opt.extra_list,
                     opt.use_initial_rig_transforms,
                     opt.bracket_len, opt.nearest_neighbor_interp, 
                     opt.read_nvm_no_shift, R,
                     nvm, image_maps, depth_maps); // out
  
  // Poses for all the cameras (extrinsics)
  rig::Extrinsics cams;
  // Keep here the images, timestamps, and bracketing information
  std::vector<rig::cameraImage> imgData;
  //  The range of R.ref_to_cam_timestamp_offsets[cam_type] before
  //  getting out of the bracket.
  std::vector<double> min_timestamp_offset, max_timestamp_offset;
  std::vector<double> ref_timestamps; // Timestamps for the ref cameras
  // Select the images to use. If the rig is used, keep non-ref images
  // only within the bracket.
  rig::lookupImages(// Inputs
                    opt.no_rig, opt.bracket_len,
                    opt.timestamp_offsets_max_change,
                    opt.bracket_single_image, 
                    R, image_maps, depth_maps,
                    // Outputs
                    ref_timestamps, cams.world_to_ref,
                    imgData, cams.world_to_cam, min_timestamp_offset, max_timestamp_offset);
  // De-allocate data we no longer need
  image_maps = std::vector<rig::MsgMap>();
  depth_maps = std::vector<rig::MsgMap>();

  if (opt.no_rig && opt.use_initial_rig_transforms)
    LOG(FATAL) << "Cannot use initial rig transforms without a rig.\n";
  
  // If we can use the initial rig transform, compute and overwrite overwrite
  // cams.world_to_cam, the transforms from the world to each non-reference camera.
  // TODO(oalexan1): Test if this works with --no_rig. For now this combination
  // is not allowed.
  if (!opt.no_rig && opt.use_initial_rig_transforms)
    rig::calcWorldToCamWithRig(// Inputs
                               !opt.no_rig,
                               imgData, cams.world_to_ref, ref_timestamps,
                               R.ref_to_cam_trans,
                               R.ref_to_cam_timestamp_offsets,
                               // Output
                               cams.world_to_cam);
  
  // If desired, initalize the rig based on median of transforms for the sensors
  if (!opt.no_rig && !opt.use_initial_rig_transforms)
    rig::calc_rig_trans(imgData, cams.world_to_ref, cams.world_to_cam, ref_timestamps,
                        R); // out
  
  // Determine if a given camera type has any depth information
  int num_cam_types = R.cam_names.size();
  std::vector<bool> has_depth(num_cam_types, false);
  for (size_t cid = 0; cid < imgData.size(); cid++) {
    int cam_type = imgData[cid].camera_type;
    if (imgData[cid].depth_cloud.cols > 0 && imgData[cid].depth_cloud.rows > 0)
      has_depth[cam_type] = true;
  }

  // Transform to world coordinates if control points were provided. Note:
  // applyRegistration modifies cams.world_to_ref (Affine3d), which will then be
  // converted to state.world_to_ref_vec (primary storage) below.
  Eigen::Affine3d registration_trans;
  registration_trans.matrix() = Eigen::Matrix4d::Identity(); // default
  bool registration_applied = false;
  if (opt.registration && opt.hugin_file != "" && opt.xyz_file != "") {
    // Keep user's R.depth_to_image transforms, and only transform only the image
    // cameras from Theia's abstract coordinate system to world coordinates.
    bool scale_depth = false;
    registration_applied = true;
    applyRegistration(opt.no_rig, scale_depth, opt.hugin_file, opt.xyz_file,
                      has_depth, imgData,
                      // Outputs
                      registration_trans, cams.world_to_ref, cams.world_to_cam, R);
  }

  int num_ref_cams = cams.world_to_ref.size();
  if (cams.world_to_ref.size() != ref_timestamps.size())
    LOG(FATAL) << "Must have as many ref cam timestamps as ref cameras.\n";

  // Which intrinsics from which cameras to float. Indexed by cam_type.
  std::vector<std::set<std::string>> intrinsics_to_float;
  rig::parse_intrinsics_to_float(opt.intrinsics_to_float, R.cam_names,
                                       intrinsics_to_float);

  std::set<std::string> camera_poses_to_float;
  rig::parse_camera_names(R.cam_names, opt.camera_poses_to_float, camera_poses_to_float);

  std::set<std::string> depth_to_image_transforms_to_float;
  rig::parse_camera_names(R.cam_names, opt.depth_to_image_transforms_to_float,
                          depth_to_image_transforms_to_float);
  
  // Set up the variable blocks to optimize for BracketedDepthError
  int num_depth_params = rig::NUM_RIGID_PARAMS;
  if (opt.affine_depth_to_image)
    num_depth_params = rig::NUM_AFFINE_PARAMS;

  // Separate the initial scale. This is convenient if
  // cam_depth_to_image is scale * rotation + translation and if
  // it is desired to keep the scale fixed. In either case, the scale
  // will be multiplied back when needed.
  // Extract and normalize depth-to-image scales. This is done before the optimization
  // loop and the scales are applied back after optimization.
  std::vector<double> depth_to_image_scales;
  // TODO(oalexan1): affine_depth_to_image must allow fixing the scale. 
  for (int cam_type = 0; cam_type < num_cam_types; cam_type++) {
    double depth_to_image_scale
      = pow(R.depth_to_image[cam_type].matrix().determinant(), 1.0 / 3.0);
    R.depth_to_image[cam_type].linear() /= depth_to_image_scale;
    depth_to_image_scales.push_back(depth_to_image_scale);
  }

  // Detect and match features if --num_overlaps > 0. Append the features
  // read from the nvm. Triangulate all points.
  rig::KeypointVec keypoint_vec;
  rig::PidCidFid pid_to_cid_fid;
  bool filter_matches_using_cams = true;
  std::vector<std::pair<int, int>> input_image_pairs; // will use num_overlaps instead
  // Do not save these matches. Only inlier matches will be saved later.
  bool local_save_matches = false;
  std::vector<Eigen::Vector3d> xyz_vec;
  rig::detectAddFeatures(// Inputs
                         imgData, R.cam_params, opt.out_prefix, local_save_matches,
                         filter_matches_using_cams, cams.world_to_cam,
                         opt.num_overlaps, input_image_pairs,
                         opt.initial_max_reprojection_error,
                         opt.num_match_threads,
                         opt.read_nvm_no_shift, opt.no_nvm_matches,
                         opt.verbose,
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

  // Inlier flag. Once an inlier becomes an outlier, it stays that way
  rig::PidCidFidMap pid_cid_fid_inlier;
  
  // TODO(oalexan1): Must initialize all points as inliers outside this function,
  // as now this function resets those.
  rig::flagOutlierByExclusionDist(// Inputs
                                  R.cam_params, imgData, pid_to_cid_fid,
                                  keypoint_vec,
                                  // Outputs
                                  pid_cid_fid_inlier);

  // Ensure that the triangulated points are kept in sync with the cameras
  if (opt.use_initial_triangulated_points && registration_applied)
    rig::transformInlierTriPoints(registration_trans, pid_to_cid_fid, 
                                  pid_cid_fid_inlier, xyz_vec);
  
  // Structures needed to intersect rays with the mesh
  rig::PidCidFidToMeshXyz pid_cid_fid_mesh_xyz;
  std::vector<Eigen::Vector3d> pid_mesh_xyz;
  Eigen::Vector3d bad_xyz(1.0e+100, 1.0e+100, 1.0e+100);  // use this to flag invalid xyz

  // TODO(oalexan1): All the logic for one pass should be its own function,
  // as the block below is too big.
  for (int pass = 0; pass < opt.calibrator_num_passes; pass++) {
    std::cout << "\nOptimization pass "
              << pass + 1 << " / " << opt.calibrator_num_passes << "\n";

    // Optimization state local to this pass. Must update the state from
    // extrinsics, run the optimization, then update back the extrinsics.
    rig::OptState state;

    // Put the intrinsics in arrays
    state.focal_lengths.resize(num_cam_types);
    state.optical_centers.resize(num_cam_types);
    state.distortions.resize(num_cam_types);
    for (int it = 0; it < num_cam_types; it++) {
      state.focal_lengths[it] = R.cam_params[it].GetFocalLength();  // average focal length
      state.optical_centers[it] = R.cam_params[it].GetOpticalOffset();
      state.distortions[it] = R.cam_params[it].GetDistortion();
    }

    // Convert extrinsics to optimization state
    rig::toOptState(cams, R, state, opt.no_rig, opt.affine_depth_to_image, num_depth_params);

    // Update cams.world_to_cam from current _vec state before triangulation
    // TODO(oalexan1): Not clear if this is needed with the current code structure.
    rig::calcWorldToCam(// Inputs
                        opt.no_rig, imgData, state.world_to_ref_vec, ref_timestamps, state.ref_to_cam_vec,
                        state.world_to_cam_vec, R.ref_to_cam_timestamp_offsets,
                        // Output
                        cams.world_to_cam);

    // Triangulate, unless desired to reuse the initial points
    if (!opt.use_initial_triangulated_points)
      rig::multiViewTriangulation(// Inputs
                                  R.cam_params, imgData, cams.world_to_cam, pid_to_cid_fid,
                                  keypoint_vec,
                                  // Outputs
                                  pid_cid_fid_inlier, xyz_vec);

    // This is a copy which won't change
    std::vector<Eigen::Vector3d> xyz_vec_orig;
    if (opt.tri_weight > 0.0) {
      // Better copy manually to ensure no shallow copy
      xyz_vec_orig.resize(xyz_vec.size());
      for (size_t pt_it = 0; pt_it < xyz_vec.size(); pt_it++) {
        for (int coord_it = 0; coord_it < 3; coord_it++) {
          xyz_vec_orig[pt_it][coord_it] = xyz_vec[pt_it][coord_it];
        }
      } 
    }
    
    // Compute where each ray intersects the mesh
    if (opt.mesh != "")
      rig::meshTriangulations(// Inputs
                              R.cam_params, imgData, cams.world_to_cam, pid_to_cid_fid,
                              pid_cid_fid_inlier, keypoint_vec, bad_xyz,
                              opt.min_ray_dist, opt.max_ray_dist, mesh, bvh_tree,
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
        imgData, R, ref_timestamps, state.world_to_cam_vec, state.world_to_ref_vec,
        state.ref_to_cam_vec, state.ref_identity_vec, state.right_identity_vec, state.focal_lengths,
        state.optical_centers, state.distortions, state.depth_to_image_vec, depth_to_image_scales,
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
        opt.no_rig, opt.fix_rig_translations, opt.fix_rig_rotations,
        opt.float_timestamp_offsets, opt.float_scale,
        opt.affine_depth_to_image, (opt.mesh != ""),
        opt.robust_threshold, opt.tri_robust_threshold, opt.tri_weight,
        opt.depth_tri_weight, opt.depth_mesh_weight, opt.mesh_tri_weight,
        opt.camera_position_weight,
        // Outputs
        pid_cid_fid_to_residual_index, problem, residual_names, residual_scales);

    // Evaluate the residuals before optimization
    std::vector<double> residuals;
    rig::evalResiduals("before opt", residual_names, residual_scales, problem, residuals);

    if (pass == 0)
      rig::writeResiduals(opt.out_prefix, "initial", R.cam_names, imgData, keypoint_vec,  
                          pid_to_cid_fid, pid_cid_fid_inlier, pid_cid_fid_to_residual_index,  
                          residuals);
    
    // Solve the problem
    ceres::Solver::Options options;
    ceres::Solver::Summary summary;
    options.linear_solver_type = ceres::ITERATIVE_SCHUR;
    options.num_threads = opt.num_threads; 
    options.max_num_iterations = opt.num_iterations;
    options.minimizer_progress_to_stdout = true;
    options.gradient_tolerance = 1e-16;
    options.function_tolerance = 1e-16;
    options.parameter_tolerance = opt.parameter_tolerance;
    ceres::Solve(options, &problem, &summary);

    // The optimization is done. Convert state back to extrinsics and R.
    rig::fromOptState(state, cams, R, opt.no_rig, opt.affine_depth_to_image, num_depth_params);

    // Copy back the optimized intrinsics
    for (int cam_type = 0; cam_type < num_cam_types; cam_type++) {
      R.cam_params[cam_type].SetFocalLength(Eigen::Vector2d(state.focal_lengths[cam_type],
                                                          state.focal_lengths[cam_type]));
      R.cam_params[cam_type].SetOpticalOffset(state.optical_centers[cam_type]);
      R.cam_params[cam_type].SetDistortion(state.distortions[cam_type]);
    }

    // Must have up-to-date cams.world_to_cam and residuals to flag the outliers
    rig::calcWorldToCam(// Inputs
                        opt.no_rig, imgData, state.world_to_ref_vec, ref_timestamps, state.ref_to_cam_vec,
                        state.world_to_cam_vec, R.ref_to_cam_timestamp_offsets,
                        // Output
                        cams.world_to_cam);

    // Evaluate the residuals after optimization
    rig::evalResiduals("after opt", residual_names, residual_scales, problem,
                       residuals);

    // Flag outliers after this pass using the computed residuals
    rig::flagOutliers(// Inputs
                      opt.min_triangulation_angle, opt.max_reprojection_error,
                      pid_to_cid_fid, keypoint_vec,
                      cams.world_to_cam, xyz_vec, pid_cid_fid_to_residual_index, residuals,
                      // Outputs
                      pid_cid_fid_inlier);
    
    rig::writeResiduals(opt.out_prefix, "final", R.cam_names, imgData, keypoint_vec,  
                        pid_to_cid_fid, pid_cid_fid_inlier,
                        pid_cid_fid_to_residual_index, residuals);
    
  }  // End optimization passes

  // Put back the scale in R.depth_to_image
  for (int cam_type = 0; cam_type < num_cam_types; cam_type++)
    R.depth_to_image[cam_type].linear() *= depth_to_image_scales[cam_type];

  if (opt.save_matches)
    rig::saveInlierMatchPairs(imgData, opt.num_overlaps, pid_to_cid_fid,
                              keypoint_vec, pid_cid_fid_inlier, opt.out_prefix);

  // Redo the registration unless told not to.
  // Note: applyRegistration modifies cams.world_to_ref (Affine3d form), which was updated
  // from state.world_to_ref_vec after the optimization loop above.
  if (!opt.skip_post_registration && opt.registration &&
      opt.hugin_file != "" && opt.xyz_file != "") {
    // This time adjust the depth-to-image scale to be consistent with optimized cameras
    bool scale_depth = true;
    Eigen::Affine3d registration_trans;
    applyRegistration(opt.no_rig, scale_depth, opt.hugin_file, opt.xyz_file,
                      has_depth, imgData,
                      // Outputs
                      registration_trans, cams.world_to_ref, cams.world_to_cam, R);

    // Transform accordingly the triangulated points
    rig::transformInlierTriPoints(registration_trans, pid_to_cid_fid, 
                                        pid_cid_fid_inlier, xyz_vec);
  }
  
  if (opt.out_texture_dir != "")
    rig::meshProjectCameras(R.cam_names, R.cam_params, imgData, cams.world_to_cam, 
                            mesh, bvh_tree, opt.out_texture_dir);

  rig::saveCameraPoses(opt.out_prefix, imgData, cams.world_to_cam);
  
  bool model_rig = (!opt.no_rig);
  rig::writeRigConfig(opt.out_prefix + "/rig_config.txt", model_rig, R);

  std::string nvm_file = opt.out_prefix + "/cameras.nvm";
  bool shift_keypoints = true;
  rig::writeInliersToNvm(nvm_file, shift_keypoints, R.cam_params, imgData,
                         cams.world_to_cam, keypoint_vec,
                         pid_to_cid_fid, pid_cid_fid_inlier, xyz_vec);
  
  if (opt.save_nvm_no_shift) {
    std::string nvm_file = opt.out_prefix + "/cameras_no_shift.nvm";
    bool shift_keypoints = false;
    rig::writeInliersToNvm(nvm_file, shift_keypoints, R.cam_params, imgData,
                                 cams.world_to_cam, keypoint_vec,
                                 pid_to_cid_fid, pid_cid_fid_inlier, xyz_vec);
  }

  if (opt.export_to_voxblox)
    rig::exportToVoxblox(R.cam_names, imgData, R.depth_to_image,
                         cams.world_to_cam, opt.out_prefix);

  if (opt.save_transformed_depth_clouds)
    rig::saveTransformedDepthClouds(R.cam_names, imgData, R.depth_to_image,
                                    cams.world_to_cam, opt.out_prefix);

  // Save the list of images (useful for bundle_adjust)
  std::string image_list = opt.out_prefix + "/image_list.txt";
  rig::saveImageList(imgData, image_list); 

  if (opt.save_pinhole_cameras)
    rig::writePinholeCameras(R.cam_names, R.cam_params, imgData, 
                             cams.world_to_cam, opt.out_prefix);
  
  std::string conv_angles_file = opt.out_prefix + "/convergence_angles.txt";
  rig::savePairwiseConvergenceAngles(pid_to_cid_fid, keypoint_vec,
                                     imgData, cams.world_to_cam,  
                                     xyz_vec,  pid_cid_fid_inlier,  
                                     conv_angles_file);
  return 0;
}

