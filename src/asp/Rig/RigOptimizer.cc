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

#include <asp/Rig/RigOptimizer.h>
#include <asp/Rig/RigCostFunction.h>
#include <asp/Rig/camera_image.h>
#include <asp/Rig/rig_config.h>
#include <asp/Rig/transform_utils.h>
#include <asp/Rig/image_lookup.h>
#include <asp/Rig/basic_algs.h>
#include <asp/Rig/rig_utils.h>
#include <asp/Rig/texture_processing.h>
#include <asp/Rig/triangulation.h>
#include <asp/Rig/RigOutlier.h>

#include <ceres/ceres.h>
#include <ceres/dynamic_numeric_diff_cost_function.h>
#include <ceres/loss_function.h>

#include <iostream>
#include <iomanip>

namespace rig {

// A struct to hold the block sizes for the various cost functions
struct RigBlockSizes {
  std::vector<int> image_block_sizes;
  std::vector<int> depth_block_sizes;
  std::vector<int> depth_mesh_block_sizes;
  std::vector<int> xyz_block_sizes;
};

// Set up block sizes for various cost functions
void set_up_block_sizes(int num_depth_params,
                        RigBlockSizes& block_sizes) {
  // Wipe the outputs
  block_sizes.image_block_sizes.clear();
  block_sizes.depth_block_sizes.clear();
  block_sizes.depth_mesh_block_sizes.clear();
  block_sizes.xyz_block_sizes.clear();

  int num_focal_lengths = 1;      // The x and y focal length are assumed to be the same
  int num_distortion_params = 1;  // will be overwritten later

  // Set up the variable blocks to optimize for BracketedCamError

  block_sizes.image_block_sizes.push_back(rig::NUM_RIGID_PARAMS);
  block_sizes.image_block_sizes.push_back(rig::NUM_RIGID_PARAMS);
  block_sizes.image_block_sizes.push_back(rig::NUM_RIGID_PARAMS);
  block_sizes.image_block_sizes.push_back(rig::NUM_XYZ_PARAMS);
  block_sizes.image_block_sizes.push_back(rig::NUM_SCALAR_PARAMS);
  block_sizes.image_block_sizes.push_back(num_focal_lengths);
  block_sizes.image_block_sizes.push_back(rig::NUM_OPT_CTR_PARAMS);
  block_sizes.image_block_sizes.push_back(num_distortion_params);

  // Set up variable blocks to optimize for BracketedDepthError
  block_sizes.depth_block_sizes.push_back(rig::NUM_RIGID_PARAMS);
  block_sizes.depth_block_sizes.push_back(rig::NUM_RIGID_PARAMS);
  block_sizes.depth_block_sizes.push_back(rig::NUM_RIGID_PARAMS);
  block_sizes.depth_block_sizes.push_back(num_depth_params);
  block_sizes.depth_block_sizes.push_back(rig::NUM_SCALAR_PARAMS);
  block_sizes.depth_block_sizes.push_back(rig::NUM_XYZ_PARAMS);
  block_sizes.depth_block_sizes.push_back(rig::NUM_SCALAR_PARAMS);

  // Set up the variable blocks to optimize for BracketedDepthMeshError
  block_sizes.depth_mesh_block_sizes.push_back(rig::NUM_RIGID_PARAMS);
  block_sizes.depth_mesh_block_sizes.push_back(rig::NUM_RIGID_PARAMS);
  block_sizes.depth_mesh_block_sizes.push_back(rig::NUM_RIGID_PARAMS);
  block_sizes.depth_mesh_block_sizes.push_back(num_depth_params);
  block_sizes.depth_mesh_block_sizes.push_back(rig::NUM_SCALAR_PARAMS);
  block_sizes.depth_mesh_block_sizes.push_back(rig::NUM_SCALAR_PARAMS);

  // Set up the variable blocks to optimize for the mesh xyz
  block_sizes.xyz_block_sizes.push_back(rig::NUM_XYZ_PARAMS);
}

// Evaluate the residuals before and after optimization
void evalResiduals(// Inputs
                   std::string              const& tag,
                   std::vector<std::string> const& residual_names,
                   std::vector<double>      const& residual_scales,
                   // Outputs
                   ceres::Problem     & problem,
                   std::vector<double>& residuals) {

  double total_cost = 0.0;
  ceres::Problem::EvaluateOptions eval_options;
  eval_options.num_threads = 1;
  eval_options.apply_loss_function = false;  // want raw residuals
  problem.Evaluate(eval_options, &total_cost, &residuals, NULL, NULL);

  // Sanity checks, after the residuals are created
  if (residuals.size() != residual_names.size())
    LOG(FATAL) << "There must be as many residual names as residual values.";
  if (residuals.size() != residual_scales.size())
    LOG(FATAL) << "There must be as many residual values as residual scales.";

  // Compensate for the scale
  for (size_t it = 0; it < residuals.size(); it++)
    residuals[it] /= residual_scales[it];

  rig::calcResidualStats(residuals, residual_names, tag);
  return;
}

void addRigReprojCostFun(// Observation
                         Eigen::Vector2d const& dist_ip,
                         double beg_ref_timestamp,
                         double end_ref_timestamp,
                         double cam_timestamp,
                         // Params and variables
                         std::vector<int> const& image_block_sizes,
                         rig::CameraParameters const& cam_params,
                         Eigen::VectorXd& distortion_vec,
                         double* distortion_placeholder_ptr,
                         double* beg_cam_ptr,
                         double* end_cam_ptr,
                         double* ref_to_cam_ptr,
                         double* xyz_ptr,
                         double* timestamp_offset_ptr,
                         double* focal_length_ptr,
                         double* optical_center_ptr,
                         rig::RigSet const& R,
                         int cam_type,
                         std::string const& image_name,
                         std::set<std::string> const& intrinsics_to_float,
                         std::set<std::string> const& camera_poses_to_float,
                         std::set<std::string> const& fixed_images,
                         std::set<double*>& fixed_parameters,
                         ceres::SubsetManifold* constant_transform_manifold,
                         double min_timestamp_offset,
                         double max_timestamp_offset,
                         // Flags
                         bool no_rig, bool fix_rig_translations,
                         bool fix_rig_rotations, bool float_timestamp_offsets,
                         double robust_threshold,
                         // Output
                         ceres::Problem& problem,
                         std::vector<std::string>& residual_names,
                         std::vector<double>& residual_scales) {

  ceres::CostFunction* bracketed_cost_function =
    rig::BracketedCamError::Create(dist_ip, beg_ref_timestamp,
                                   end_ref_timestamp,
                                   cam_timestamp, image_block_sizes,
                                   cam_params);
  ceres::LossFunction* bracketed_loss_function
    = rig::GetLossFunction("cauchy", robust_threshold);

  // Handle the case of no distortion
  double * distortion_ptr = NULL;
  if (distortion_vec.size() > 0)
    distortion_ptr = &distortion_vec[0];
  else
    distortion_ptr = distortion_placeholder_ptr;

  residual_names.push_back(R.cam_names[cam_type] + "_pix_x");
  residual_names.push_back(R.cam_names[cam_type] + "_pix_y");
  residual_scales.push_back(1.0);
  residual_scales.push_back(1.0);
  problem.AddResidualBlock
    (bracketed_cost_function, bracketed_loss_function,
     beg_cam_ptr, end_cam_ptr, ref_to_cam_ptr, xyz_ptr,
     timestamp_offset_ptr,
     focal_length_ptr, optical_center_ptr, distortion_ptr);

  // See which intrinsics to float
  if (intrinsics_to_float.find("focal_length") ==
      intrinsics_to_float.end())
    problem.SetParameterBlockConstant(focal_length_ptr);
  if (intrinsics_to_float.find("optical_center") ==
      intrinsics_to_float.end())
    problem.SetParameterBlockConstant(optical_center_ptr);
  if (intrinsics_to_float.find("distortion")
      == intrinsics_to_float.end() || distortion_vec.size() == 0)
    problem.SetParameterBlockConstant(distortion_ptr);

  if (!no_rig) {
    // See if to float the beg camera, which here will point to the ref cam
    if (camera_poses_to_float.find(R.refSensor(cam_type))
        == camera_poses_to_float.end())
      problem.SetParameterBlockConstant(beg_cam_ptr);
  } else {
    // There is no rig. Then beg_cam_ptr refers to camera
    // for cams[cid], and not to its ref bracketing cam.
    // See if the user wants it floated.
    if (camera_poses_to_float.find(R.cam_names[cam_type])
        == camera_poses_to_float.end())
      problem.SetParameterBlockConstant(beg_cam_ptr);
  }

  // The end cam floats only if the ref cam can float and end cam brackets
  // a non-ref cam and we have a rig. 
  if (camera_poses_to_float.find(R.refSensor(cam_type))
      == camera_poses_to_float.end() ||
      R.isRefSensor(R.cam_names[cam_type]) || no_rig)
    problem.SetParameterBlockConstant(end_cam_ptr);

  // ref_to_cam is kept fixed at the identity if the cam is the ref type or
  // no rig
  if (camera_poses_to_float.find(R.cam_names[cam_type])
      == camera_poses_to_float.end() ||
      R.isRefSensor(R.cam_names[cam_type]) || no_rig) {
    problem.SetParameterBlockConstant(ref_to_cam_ptr);
    fixed_parameters.insert(ref_to_cam_ptr);
  }

  // See if to fix the rig translation or rotation components
  if ((fix_rig_translations || fix_rig_rotations) &&
      fixed_parameters.find(ref_to_cam_ptr) == fixed_parameters.end())
    problem.SetManifold(ref_to_cam_ptr, constant_transform_manifold);

  // See if to fix some images. For that, an image must be in the list,
  // and its camera must be either of ref type or there must be no rig.
  if (!fixed_images.empty() &&
      (fixed_images.find(image_name) != fixed_images.end()) &&
      (R.isRefSensor(R.cam_names[cam_type]) || no_rig))
    problem.SetParameterBlockConstant(beg_cam_ptr);

  if (!float_timestamp_offsets || R.isRefSensor(R.cam_names[cam_type]) ||
      no_rig) {
    // Either we don't float timestamp offsets at all, or the cam is the ref type,
    // or with no extrinsics, when it can't float anyway.
    problem.SetParameterBlockConstant(timestamp_offset_ptr);
  } else {
    problem.SetParameterLowerBound(timestamp_offset_ptr, 0,
                                   min_timestamp_offset);
    problem.SetParameterUpperBound(timestamp_offset_ptr, 0,
                                   max_timestamp_offset);
  }
}

// Ensure that the depth points agree with triangulated points
void addRigDepthTriCostFun(// Observation
                           Eigen::Vector3d const& depth_xyz,
                           double beg_ref_timestamp,
                           double end_ref_timestamp,
                           double cam_timestamp,
                           // Params & Variables
                           std::vector<int> const& depth_block_sizes,
                           int num_depth_params,
                           double* beg_cam_ptr,
                           double* end_cam_ptr,
                           double* ref_to_cam_ptr,
                           double* depth_to_image_ptr,
                           double* depth_to_image_scale_ptr,
                           double* xyz_ptr,
                           double* timestamp_offset_ptr,
                           rig::RigSet const& R,
                           int cam_type,
                           std::set<std::string> const& depth_to_image_transforms_to_float,
                           // Flags
                           bool float_scale,
                           bool affine_depth_to_image,
                           double depth_tri_weight,
                           double robust_threshold,
                           // Output
                           ceres::Problem& problem,
                           std::vector<std::string>& residual_names,
                           std::vector<double>& residual_scales) {

  ceres::CostFunction* bracketed_depth_cost_function
    = rig::BracketedDepthError::Create(depth_tri_weight, depth_xyz,
                                       beg_ref_timestamp, end_ref_timestamp,
                                       cam_timestamp,
                                       depth_block_sizes);

  ceres::LossFunction* bracketed_depth_loss_function
    = rig::GetLossFunction("cauchy", robust_threshold);

  residual_names.push_back("depth_tri_x_m");
  residual_names.push_back("depth_tri_y_m");
  residual_names.push_back("depth_tri_z_m");
  residual_scales.push_back(depth_tri_weight);
  residual_scales.push_back(depth_tri_weight);
  residual_scales.push_back(depth_tri_weight);
  problem.AddResidualBlock
    (bracketed_depth_cost_function, bracketed_depth_loss_function,
     beg_cam_ptr, end_cam_ptr, ref_to_cam_ptr,
     depth_to_image_ptr,
     depth_to_image_scale_ptr,
     xyz_ptr,
     timestamp_offset_ptr);

  // Note that above we already considered fixing some params.
  // We won't repeat that code here.
  // If we model an affine depth to image, fix its scale here,
  // it will change anyway as part of depth_to_image_vec.
  if (!float_scale || affine_depth_to_image) {
    problem.SetParameterBlockConstant(depth_to_image_scale_ptr);
  }

  if (depth_to_image_transforms_to_float.find(R.cam_names[cam_type])
      == depth_to_image_transforms_to_float.end())
    problem.SetParameterBlockConstant(depth_to_image_ptr);
}

// Try to make each mesh intersection agree with corresponding depth
// measurement, if it exists
void addRigDepthMeshCostFun(// Observation
                            Eigen::Vector3d const& depth_xyz,
                            Eigen::Vector3d const& mesh_xyz,
                            double beg_ref_timestamp,
                            double end_ref_timestamp,
                            double cam_timestamp,
                            // Params & Variables
                            std::vector<int> const& depth_mesh_block_sizes,
                            int num_depth_params,
                            double* beg_cam_ptr,
                            double* end_cam_ptr,
                            double* ref_to_cam_ptr,
                            double* depth_to_image_ptr,
                            double* depth_to_image_scale_ptr,
                            double* timestamp_offset_ptr,
                            rig::RigSet const& R,
                            int cam_type,
                            std::set<std::string> const& depth_to_image_transforms_to_float,
                            // Flags
                            bool float_scale,
                            bool affine_depth_to_image,
                            double depth_mesh_weight,
                            double robust_threshold,
                            // Output
                            ceres::Problem& problem,
                            std::vector<std::string>& residual_names,
                            std::vector<double>& residual_scales) {

  ceres::CostFunction* bracketed_depth_mesh_cost_function
    = rig::BracketedDepthMeshError::Create
    (depth_mesh_weight, depth_xyz, mesh_xyz, beg_ref_timestamp,
     end_ref_timestamp, cam_timestamp, depth_mesh_block_sizes);

  ceres::LossFunction* bracketed_depth_mesh_loss_function
    = rig::GetLossFunction("cauchy", robust_threshold);

  residual_names.push_back("depth_mesh_x_m");
  residual_names.push_back("depth_mesh_y_m");
  residual_names.push_back("depth_mesh_z_m");
  residual_scales.push_back(depth_mesh_weight);
  residual_scales.push_back(depth_mesh_weight);
  residual_scales.push_back(depth_mesh_weight);
  problem.AddResidualBlock
    (bracketed_depth_mesh_cost_function, bracketed_depth_mesh_loss_function,
     beg_cam_ptr, end_cam_ptr, ref_to_cam_ptr,
     depth_to_image_ptr,
     depth_to_image_scale_ptr,
     timestamp_offset_ptr);

  // Note that above we already fixed some of these variables.
  // Repeat the fixing of depth variables, however, as the previous block
  // may not take place.
  if (!float_scale || affine_depth_to_image)
    problem.SetParameterBlockConstant(depth_to_image_scale_ptr);

  if (depth_to_image_transforms_to_float.find(R.cam_names[cam_type])
      == depth_to_image_transforms_to_float.end())
    problem.SetParameterBlockConstant(depth_to_image_ptr);
}

void addRigMeshTriCostFun(Eigen::Vector3d const& avg_mesh_xyz,
                         std::vector<int> const& xyz_block_sizes,
                         double mesh_tri_weight,
                         double robust_threshold,
                         double* xyz_ptr,
                         ceres::Problem& problem,
                         std::vector<std::string>& residual_names,
                         std::vector<double>& residual_scales) {

  // Try to make the triangulated point agree with the mesh intersection
  ceres::CostFunction* mesh_cost_function =
    rig::XYZError::Create(avg_mesh_xyz, xyz_block_sizes, mesh_tri_weight);

  ceres::LossFunction* mesh_loss_function =
    rig::GetLossFunction("cauchy", robust_threshold);

  problem.AddResidualBlock(mesh_cost_function, mesh_loss_function,
                           xyz_ptr);

  residual_names.push_back("mesh_tri_x_m");
  residual_names.push_back("mesh_tri_y_m");
  residual_names.push_back("mesh_tri_z_m");
  residual_scales.push_back(mesh_tri_weight);
  residual_scales.push_back(mesh_tri_weight);
  residual_scales.push_back(mesh_tri_weight);
}

void addRigTriCostFun(Eigen::Vector3d const& xyz_orig,
                      std::vector<int> const& xyz_block_sizes,
                      double tri_weight,
                      double robust_threshold,
                      double* xyz_ptr,
                      ceres::Problem& problem,
                      std::vector<std::string>& residual_names,
                      std::vector<double>& residual_scales) {

  // Try to make the triangulated points (and hence cameras) not move too far
  ceres::CostFunction* tri_cost_function =
    rig::XYZError::Create(xyz_orig, xyz_block_sizes, tri_weight);

  ceres::LossFunction* tri_loss_function =
    rig::GetLossFunction("cauchy", robust_threshold);

  problem.AddResidualBlock(tri_cost_function, tri_loss_function,
                           xyz_ptr);

  residual_names.push_back("tri_x_m");
  residual_names.push_back("tri_y_m");
  residual_names.push_back("tri_z_m");
  residual_scales.push_back(tri_weight);
  residual_scales.push_back(tri_weight);
  residual_scales.push_back(tri_weight);
}

// Add the camera position constraints for the ref cams
void addRigCamPosCostFun(// Observation
                         std::vector<rig::cameraImage> const& cams,
                         // Params & Variables
                         rig::RigSet const& R,
                         std::set<std::string> const& camera_poses_to_float,
                         std::vector<double> const& ref_timestamps,
                         std::vector<double>& world_to_cam_vec,
                         std::vector<double>& world_to_ref_vec,
                         std::vector<double>& ref_to_cam_vec,
                         std::vector<double>& ref_identity_vec,
                         std::vector<double>& right_identity_vec,
                         // Flags
                         bool no_rig,
                         double camera_position_uncertainty,
                         // Output
                         ceres::Problem& problem,
                         std::vector<std::string>& residual_names,
                         std::vector<double>& residual_scales) {

  for (size_t cid = 0; cid < cams.size(); cid++) {
    int cam_type = cams[cid].camera_type;
    auto const& sensor_name = R.cam_names[cam_type];

    if (camera_poses_to_float.find(sensor_name)
        == camera_poses_to_float.end()) continue; // sensor not floated

    if (!no_rig && !R.isRefSensor(sensor_name))
      continue; // only ref sensors are floated in a rig

    // Find timestamps and pointers to bracketing cameras ref_to_cam transform.
    // This strongly depends on whether we are using a rig or not.
    double beg_ref_timestamp = -1.0, end_ref_timestamp = -1.0, cam_timestamp = -1.0;
    double *beg_cam_ptr = NULL, *end_cam_ptr = NULL, *ref_to_cam_ptr = NULL;

    rig::calcBracketing(// Inputs
                  no_rig, cid, cam_type, cams, ref_timestamps, R,
                  world_to_cam_vec, world_to_ref_vec, ref_to_cam_vec,
                  ref_identity_vec, right_identity_vec,
                  // Outputs
                  beg_cam_ptr, end_cam_ptr, ref_to_cam_ptr,
                  beg_ref_timestamp, end_ref_timestamp,
                  cam_timestamp);

    ceres::CostFunction* cam_pos_cost_function =
       rig::CamPositionErr::Create(beg_cam_ptr, camera_position_uncertainty);
    ceres::LossFunction* cam_pos_loss_function = NULL; // no robust threshold

    problem.AddResidualBlock(cam_pos_cost_function, cam_pos_loss_function,
                             beg_cam_ptr);

    residual_names.push_back(sensor_name + "_pos_x");
    residual_names.push_back(sensor_name + "_pos_y");
    residual_names.push_back(sensor_name + "_pos_z");
    residual_names.push_back(sensor_name + "_q_x");
    residual_names.push_back(sensor_name + "_q_y");
    residual_names.push_back(sensor_name + "_q_z");
    residual_names.push_back(sensor_name + "_q_w");
    // The scales are weight multipliers, so need to convert uncertainty to weight
    residual_scales.push_back(1.0/camera_position_uncertainty);
    residual_scales.push_back(1.0/camera_position_uncertainty);
    residual_scales.push_back(1.0/camera_position_uncertainty);
    residual_scales.push_back(1.0); // Rotations will not be constrained
    residual_scales.push_back(1.0);
    residual_scales.push_back(1.0);
    residual_scales.push_back(1.0);
  }
}

// Set up the optimization problem for rig calibration
void setupRigOptProblem(// Inputs
                        std::vector<cameraImage> const& cams,
                        RigSet& R,
                        std::vector<double> const& ref_timestamps,
                        OptState& state,
                        std::vector<double>& depth_to_image_scales,
                        KeypointVec const& keypoint_vec,
                        rig::PidCidFid const& pid_to_cid_fid,
                        rig::PidCidFidMap const& pid_cid_fid_inlier,
                        rig::PidCidFidToMeshXyz const& pid_cid_fid_mesh_xyz,
                        std::vector<Eigen::Vector3d> const& pid_mesh_xyz,
                        std::vector<Eigen::Vector3d>& xyz_vec,
                        std::vector<Eigen::Vector3d> const& xyz_vec_orig,
                        rig::RigBlockSizes const& block_sizes,
                        int num_depth_params,
                        std::vector<double> const& min_timestamp_offset,
                        std::vector<double> const& max_timestamp_offset,
                        RigOptions const& opt,
                        // Outputs
                        rig::PidCidFidMap& pid_cid_fid_to_residual_index,
                        ceres::Problem& problem,
                        std::vector<std::string>& residual_names,
                        std::vector<double>& residual_scales) {

  bool has_mesh = (opt.mesh != "");

  // For when we don't have distortion but must get a pointer to
  // distortion for the interface
  double distortion_placeholder = 0.0;
  Eigen::Vector3d bad_xyz(1.0e+100, 1.0e+100, 1.0e+100);

  // Prepare for the case of fixed rig translations and/or rotations
  std::set<double*> fixed_parameters; // to avoid double fixing
  ceres::SubsetManifold* constant_transform_manifold = nullptr;
  rig::setUpFixRigOptions(opt.no_rig, opt.fix_rig_translations,
                          opt.fix_rig_rotations,
                          constant_transform_manifold);

  for (size_t pid = 0; pid < pid_to_cid_fid.size(); pid++) {
    for (auto cid_fid = pid_to_cid_fid[pid].begin();
         cid_fid != pid_to_cid_fid[pid].end(); cid_fid++) {
      int cid = cid_fid->first;
      int fid = cid_fid->second;

      // Deal with inliers only
      if (!rig::getMapValue(pid_cid_fid_inlier, pid, cid, fid))
        continue;

      // Find timestamps and pointers to bracketing cameras ref_to_cam transform.
      // This strongly depends on whether we are using a rig or not.
      int cam_type = cams[cid].camera_type;
      double beg_ref_timestamp = -1.0, end_ref_timestamp = -1.0, cam_timestamp = -1.0;
      double *beg_cam_ptr = NULL, *end_cam_ptr = NULL, *ref_to_cam_ptr = NULL;
      rig::calcBracketing(// Inputs
                          opt.no_rig, cid, cam_type, cams, ref_timestamps, R,
                          state.world_to_cam_vec, state.world_to_ref_vec, state.ref_to_cam_vec,
                          state.ref_identity_vec, state.right_identity_vec,
                          // Outputs
                          beg_cam_ptr, end_cam_ptr, ref_to_cam_ptr,
                          beg_ref_timestamp, end_ref_timestamp,
                          cam_timestamp);

      Eigen::Vector2d dist_ip(keypoint_vec[cid][fid].first,
                              keypoint_vec[cid][fid].second);

      // Remember the index of the pixel residuals about to create
      pid_cid_fid_to_residual_index[pid][cid][fid] = residual_names.size();

      // Add pixel reprojection error cost function
      rig::addRigReprojCostFun(dist_ip, beg_ref_timestamp, end_ref_timestamp,
                               cam_timestamp,
                               block_sizes.image_block_sizes,
                               R.cam_params[cam_type],
                               state.distortions[cam_type], &distortion_placeholder,
                               beg_cam_ptr, end_cam_ptr, ref_to_cam_ptr,
                               &xyz_vec[pid][0],
                               &R.ref_to_cam_timestamp_offsets[cam_type],
                               &state.focal_lengths[cam_type],
                               &state.optical_centers[cam_type][0],
                               R, cam_type, cams[cid].image_name,
                               opt.intrinsics_to_float[cam_type],
                               opt.camera_poses_to_float, opt.fixed_images, fixed_parameters,
                               constant_transform_manifold,
                               min_timestamp_offset[cam_type],
                               max_timestamp_offset[cam_type],
                               opt.no_rig, opt.fix_rig_translations,
                               opt.fix_rig_rotations, opt.float_timestamp_offsets,
                               opt.robust_threshold,
                               problem,
                               residual_names, residual_scales);

      // Add the depth to triangulated point constraint
      Eigen::Vector3d depth_xyz(0, 0, 0);
      bool have_depth_tri_constraint
        = (opt.depth_tri_weight > 0 &&
           rig::depthValue(cams[cid].depth_cloud, dist_ip, depth_xyz));
      if (have_depth_tri_constraint)
        rig::addRigDepthTriCostFun(depth_xyz, beg_ref_timestamp, end_ref_timestamp,
                                   cam_timestamp,
                                   block_sizes.depth_block_sizes,
                                   num_depth_params,
                                   beg_cam_ptr, end_cam_ptr, ref_to_cam_ptr,
                                   &state.depth_to_image_vec[num_depth_params * cam_type],
                                   &depth_to_image_scales[cam_type],
                                   &xyz_vec[pid][0],
                                   &R.ref_to_cam_timestamp_offsets[cam_type],
                                   R, cam_type, opt.depth_to_image_transforms_to_float,
                                   opt.float_scale,
                                   opt.affine_depth_to_image,
                                   opt.depth_tri_weight,
                                   opt.robust_threshold,
                                   problem,
                                   residual_names,
                                   residual_scales);

      // Add the depth to mesh constraint
      bool have_depth_mesh_constraint = false;
      depth_xyz = Eigen::Vector3d(0, 0, 0);
      Eigen::Vector3d mesh_xyz(0, 0, 0);
      if (has_mesh) {
        mesh_xyz = rig::getMapValue(pid_cid_fid_mesh_xyz, pid, cid, fid);
        have_depth_mesh_constraint
          = (opt.depth_mesh_weight > 0 && mesh_xyz != bad_xyz &&
             rig::depthValue(cams[cid].depth_cloud, dist_ip, depth_xyz));
      }

      if (have_depth_mesh_constraint)
        rig::addRigDepthMeshCostFun(depth_xyz, mesh_xyz, beg_ref_timestamp,
                                    end_ref_timestamp, cam_timestamp,
                                    block_sizes.depth_mesh_block_sizes,
                                    num_depth_params, beg_cam_ptr, end_cam_ptr,
                                    ref_to_cam_ptr,
                                    &state.depth_to_image_vec[num_depth_params * cam_type],
                                    &depth_to_image_scales[cam_type],
                                    &R.ref_to_cam_timestamp_offsets[cam_type],
                                    R, cam_type, opt.depth_to_image_transforms_to_float,
                                    opt.float_scale, opt.affine_depth_to_image,
                                    opt.depth_mesh_weight, opt.robust_threshold,
                                    problem, residual_names, residual_scales);
    }  // end iterating over all cid for given pid

    // The constraints below will be for each triangulated point. Skip such a point
    // if all rays converging to it come from outliers.
    bool isTriInlier = false;
    for (auto cid_fid = pid_to_cid_fid[pid].begin();
         cid_fid != pid_to_cid_fid[pid].end(); cid_fid++) {
      int cid = cid_fid->first;
      int fid = cid_fid->second;
      if (rig::getMapValue(pid_cid_fid_inlier, pid, cid, fid)) {
        isTriInlier = true;
        break; // found it to be an inlier, no need to do further checking
      }
    }

    // Add mesh-to-triangulated point constraint
    bool have_mesh_tri_constraint = false;
    Eigen::Vector3d avg_mesh_xyz(0, 0, 0);
    if (has_mesh && isTriInlier) {
      avg_mesh_xyz = pid_mesh_xyz.at(pid);
      if (opt.mesh_tri_weight > 0 && avg_mesh_xyz != bad_xyz)
        have_mesh_tri_constraint = true;
    }
    if (have_mesh_tri_constraint)
      rig::addRigMeshTriCostFun(avg_mesh_xyz, block_sizes.xyz_block_sizes,
                                opt.mesh_tri_weight,
                                opt.robust_threshold,
                                &xyz_vec[pid][0], problem, residual_names,
                                residual_scales);

    // Add the constraint that the triangulated point does not go too far
    if (opt.tri_weight > 0.0 && isTriInlier)
      rig::addRigTriCostFun(xyz_vec_orig[pid], block_sizes.xyz_block_sizes,
                            opt.tri_weight, opt.tri_robust_threshold,
                            &xyz_vec[pid][0], problem,
                            residual_names, residual_scales);

  }  // end iterating over pid

  // Add the camera position constraints for the ref cams. Need to respect
  // the fact that opt.camera_position_uncertainty type is a vector of vectors.
  if (opt.camera_position_uncertainty.size() > 0)
    rig::addRigCamPosCostFun(cams, R, opt.camera_poses_to_float, ref_timestamps,
                             state.world_to_cam_vec, state.world_to_ref_vec, state.ref_to_cam_vec,
                             state.ref_identity_vec, state.right_identity_vec,
                             opt.no_rig, opt.camera_position_uncertainty[0][0],
                             problem, residual_names, residual_scales);
}

// Run an optimization pass for rig calibration
void runOptPass(int pass,
                int num_depth_params,
                rig::RigOptions               const& opt,
                std::vector<rig::cameraImage> const& imgData,
                std::vector<double>           const& ref_timestamps,
                rig::KeypointVec              const& keypoint_vec,
                rig::PidCidFid                const& pid_to_cid_fid,
                std::vector<double>           const& min_timestamp_offset,
                std::vector<double>           const& max_timestamp_offset,
                mve::TriangleMesh::Ptr        const& mesh,
                std::shared_ptr<BVHTree>      const& bvh_tree,
                // Outputs
                std::vector<double>                & depth_to_image_scales,
                rig::Extrinsics                    & cams,
                rig::RigSet                        & R,
                std::vector<Eigen::Vector3d>       & xyz_vec,
                rig::PidCidFidMap                  & pid_cid_fid_inlier) {

  // Set up the block sizes
  rig::RigBlockSizes block_sizes;
  rig::set_up_block_sizes(num_depth_params, block_sizes);

  // Optimization state local to this pass. Must update the state from
  // extrinsics and rig config, run the optimization, then update back the extrinsics.
  rig::OptState state;
  rig::toOptState(cams, R, state, opt.no_rig, opt.affine_depth_to_image, num_depth_params);

  // Update cams.world_to_cam from current state. This is strictly necessary
  // only when the rig is on, as then this data must be derived from the rig
  // and the transforms for the reference sensor.
  rig::calcWorldToCam(// Inputs
                      opt.no_rig, imgData, state.world_to_ref_vec, ref_timestamps,
                      state.ref_to_cam_vec, state.world_to_cam_vec,
                      R.ref_to_cam_timestamp_offsets,
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
  rig::PidCidFidToMeshXyz pid_cid_fid_mesh_xyz;
  std::vector<Eigen::Vector3d> pid_mesh_xyz;
  if (opt.mesh != "") {
    rig::meshTriangulations(// Inputs
                            R.cam_params, imgData, cams.world_to_cam, pid_to_cid_fid,
                            pid_cid_fid_inlier, keypoint_vec,
                            opt.min_ray_dist, opt.max_ray_dist, mesh, bvh_tree,
                            // Outputs
                            pid_cid_fid_mesh_xyz, pid_mesh_xyz);
  }

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
  rig::setupRigOptProblem(imgData, R, ref_timestamps, state, depth_to_image_scales,
                          keypoint_vec, pid_to_cid_fid, pid_cid_fid_inlier, 
                          pid_cid_fid_mesh_xyz, pid_mesh_xyz, xyz_vec, xyz_vec_orig,
                          block_sizes, num_depth_params, 
                          min_timestamp_offset, max_timestamp_offset, opt,
                          // Outputs
                          pid_cid_fid_to_residual_index, problem, residual_names, 
                          residual_scales);

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

  // The optimization is done. Convert state back to extrinsics and R (includes intrinsics).
  rig::fromOptState(state, cams, R, opt.no_rig, opt.affine_depth_to_image, num_depth_params);

  // Update cams.world_to_cam from optimized state. This is strictly necessary
  // only when the rig is on, as then this data must be derived from the rig
  // and the transforms for the reference sensor.
  rig::calcWorldToCam(// Inputs
                      opt.no_rig, imgData, state.world_to_ref_vec, ref_timestamps,
                      state.ref_to_cam_vec, state.world_to_cam_vec,
                      R.ref_to_cam_timestamp_offsets,
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
}

}  // end namespace rig
