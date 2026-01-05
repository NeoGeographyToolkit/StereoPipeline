// __BEGIN_LICENSE__
//  Copyright (c) 2009-2025, United States Government as represented by the
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

#include <asp/Rig/RigCostFunction.h>
#include <asp/Rig/camera_image.h>
#include <asp/Rig/rig_config.h>
#include <asp/Rig/transform_utils.h>
#include <asp/Rig/image_lookup.h>
#include <asp/Rig/basic_algs.h>
#include <asp/Rig/rig_utils.h>

#include <iostream>
#include <iomanip>

namespace rig {

// Set up block sizes for various cost functions
void set_up_block_sizes(int num_depth_params,
                        std::vector<int> & bracketed_cam_block_sizes,
                        std::vector<int> & bracketed_depth_block_sizes,
                        std::vector<int> & bracketed_depth_mesh_block_sizes,
                        std::vector<int> & xyz_block_sizes) {
  // Wipe the outputs
  bracketed_cam_block_sizes.clear();
  bracketed_depth_block_sizes.clear();
  bracketed_depth_mesh_block_sizes.clear();
  xyz_block_sizes.clear();

  int num_focal_lengths = 1;      // The x and y focal length are assumed to be the same
  int num_distortion_params = 1;  // will be overwritten later

  // Set up the variable blocks to optimize for BracketedCamError

  bracketed_cam_block_sizes.push_back(rig::NUM_RIGID_PARAMS);
  bracketed_cam_block_sizes.push_back(rig::NUM_RIGID_PARAMS);
  bracketed_cam_block_sizes.push_back(rig::NUM_RIGID_PARAMS);
  bracketed_cam_block_sizes.push_back(rig::NUM_XYZ_PARAMS);
  bracketed_cam_block_sizes.push_back(rig::NUM_SCALAR_PARAMS);
  bracketed_cam_block_sizes.push_back(num_focal_lengths);
  bracketed_cam_block_sizes.push_back(rig::NUM_OPT_CTR_PARAMS);
  bracketed_cam_block_sizes.push_back(num_distortion_params);

  // Set up variable blocks to optimize for BracketedDepthError
  bracketed_depth_block_sizes.push_back(rig::NUM_RIGID_PARAMS);
  bracketed_depth_block_sizes.push_back(rig::NUM_RIGID_PARAMS);
  bracketed_depth_block_sizes.push_back(rig::NUM_RIGID_PARAMS);
  bracketed_depth_block_sizes.push_back(num_depth_params);
  bracketed_depth_block_sizes.push_back(rig::NUM_SCALAR_PARAMS);
  bracketed_depth_block_sizes.push_back(rig::NUM_XYZ_PARAMS);
  bracketed_depth_block_sizes.push_back(rig::NUM_SCALAR_PARAMS);

  // Set up the variable blocks to optimize for BracketedDepthMeshError
  bracketed_depth_mesh_block_sizes.push_back(rig::NUM_RIGID_PARAMS);
  bracketed_depth_mesh_block_sizes.push_back(rig::NUM_RIGID_PARAMS);
  bracketed_depth_mesh_block_sizes.push_back(rig::NUM_RIGID_PARAMS);
  bracketed_depth_mesh_block_sizes.push_back(num_depth_params);
  bracketed_depth_mesh_block_sizes.push_back(rig::NUM_SCALAR_PARAMS);
  bracketed_depth_mesh_block_sizes.push_back(rig::NUM_SCALAR_PARAMS);

  // Set up the variable blocks to optimize for the mesh xyz
  xyz_block_sizes.push_back(rig::NUM_XYZ_PARAMS);
}

// If applicable, set up the parameters block to fix the rig translations and/or rotations
void setUpFixRigOptions(bool no_rig, bool fix_rig_translations, bool fix_rig_rotations,
                        ceres::SubsetManifold*& constant_transform_manifold) {
  
  constant_transform_manifold = NULL;
  
  int beg = 0, end = 0;
  if (!no_rig && fix_rig_translations) {
    beg = 0; 
    end = 3;
  }
  
  if (!no_rig && fix_rig_rotations) {
    if (!fix_rig_translations)
      beg = 3; // only fix rotation
    end = rig::NUM_RIGID_PARAMS;
  }
  
  // Make a vector that goes from beg to end with increment 1
  std::vector<int> fixed_indices;
  for (int it = beg; it < end; it++)
    fixed_indices.push_back(it);
  
  if (!fixed_indices.empty())
    constant_transform_manifold 
      = new ceres::SubsetManifold(rig::NUM_RIGID_PARAMS, fixed_indices);
}

ceres::LossFunction* GetLossFunction(std::string cost_fun, double th) {
  // Convert to lower-case
  std::transform(cost_fun.begin(), cost_fun.end(), cost_fun.begin(), ::tolower);

  ceres::LossFunction* loss_function = NULL;
  if (cost_fun == "l2")
    loss_function = NULL;
  else if (cost_fun == "huber")
    loss_function = new ceres::HuberLoss(th);
  else if (cost_fun == "cauchy")
    loss_function = new ceres::CauchyLoss(th);
  else if (cost_fun == "l1")
    loss_function = new ceres::SoftLOneLoss(th);
  else
    LOG(FATAL) << "Unknown cost function: " + cost_fun;

  return loss_function;
}
  
BracketedCamError::BracketedCamError(Eigen::Vector2d const& meas_dist_pix,
                  double left_ref_stamp, double right_ref_stamp, double cam_stamp,
                  std::vector<int> const& block_sizes,
                  rig::CameraParameters const& cam_params):
  m_meas_dist_pix(meas_dist_pix),
  m_left_ref_stamp(left_ref_stamp),
  m_right_ref_stamp(right_ref_stamp),
  m_cam_stamp(cam_stamp),
  m_block_sizes(block_sizes),
  m_cam_params(cam_params),
  m_num_focal_lengths(1) {
  // Sanity check
  if (m_block_sizes.size() != 8 || m_block_sizes[0] != NUM_RIGID_PARAMS ||
      m_block_sizes[1] != NUM_RIGID_PARAMS || m_block_sizes[2] != NUM_RIGID_PARAMS ||
      m_block_sizes[3] != NUM_XYZ_PARAMS || m_block_sizes[4] != NUM_SCALAR_PARAMS ||
      m_block_sizes[5] != m_num_focal_lengths ||
      m_block_sizes[6] != NUM_OPT_CTR_PARAMS ||
      m_block_sizes[7] != 1  // This will be overwritten shortly
  ) {
    LOG(FATAL) << "BracketedCamError: The block sizes were not set up properly.\n";
  }

  // Set the correct distortion size. This cannot be done in the interface for now.
  m_block_sizes[7] = m_cam_params.GetDistortion().size();
}

// Call to work with ceres::DynamicNumericDiffCostFunction.
bool BracketedCamError::operator()(double const* const* parameters, double* residuals) const {
  Eigen::Affine3d world_to_cam_trans =
    calcWorldToCamBase(parameters[0],  // beg_world_to_ref_t
                       parameters[1],  // end_world_to_ref_t
                       parameters[2],  // ref_to_cam_trans
                       m_left_ref_stamp, m_right_ref_stamp,
                       parameters[4][0],  // ref_to_cam_offset
                       m_cam_stamp);

  // World point
  Eigen::Vector3d X(parameters[3][0], parameters[3][1], parameters[3][2]);

  // Make a deep copy which we will modify
  rig::CameraParameters cam_params = m_cam_params;
  Eigen::Vector2d focal_vector = Eigen::Vector2d(parameters[5][0], parameters[5][0]);
  Eigen::Vector2d optical_center(parameters[6][0], parameters[6][1]);
  Eigen::VectorXd distortion(m_block_sizes[7]);
  for (int i = 0; i < m_block_sizes[7]; i++) distortion[i] = parameters[7][i];
  cam_params.SetFocalLength(focal_vector);
  cam_params.SetOpticalOffset(optical_center);
  cam_params.SetDistortion(distortion);

  // Convert world point to given cam coordinates
  X = world_to_cam_trans * X;

  // Project into the image
  Eigen::Vector2d undist_pix
    = cam_params.GetFocalVector().cwiseProduct(X.hnormalized());
  Eigen::Vector2d curr_dist_pix;
  cam_params.Convert<rig::UNDISTORTED_C, rig::DISTORTED>
    (undist_pix, &curr_dist_pix);

  // Compute the residuals
  residuals[0] = curr_dist_pix[0] - m_meas_dist_pix[0];
  residuals[1] = curr_dist_pix[1] - m_meas_dist_pix[1];

  return true;
}

// Factory to hide the construction of the CostFunction object from the client code.
ceres::CostFunction*
BracketedCamError::Create(Eigen::Vector2d const& meas_dist_pix, double left_ref_stamp, 
                          double right_ref_stamp, double cam_stamp, 
                          std::vector<int> const& block_sizes,
                          rig::CameraParameters const& cam_params) {

  ceres::DynamicNumericDiffCostFunction<BracketedCamError>* cost_function =
    new ceres::DynamicNumericDiffCostFunction<BracketedCamError>
    (new BracketedCamError(meas_dist_pix, left_ref_stamp, right_ref_stamp,
                            cam_stamp, block_sizes, cam_params));

  cost_function->SetNumResiduals(NUM_PIX_PARAMS);

  // The camera wrapper knows all of the block sizes to add, except
  // for distortion, which is last
  for (size_t i = 0; i + 1 < block_sizes.size(); i++)  // note the i + 1
    cost_function->AddParameterBlock(block_sizes[i]);

  // The distortion block size is added separately as it is variable
  cost_function->AddParameterBlock(cam_params.GetDistortion().size());

  return cost_function;
}
  
BracketedDepthError::BracketedDepthError(double weight, Eigen::Vector3d const& meas_depth_xyz,
                    double left_ref_stamp, double right_ref_stamp, double cam_stamp,
                    std::vector<int> const& block_sizes):
  m_weight(weight),
  m_meas_depth_xyz(meas_depth_xyz),
  m_left_ref_stamp(left_ref_stamp),
  m_right_ref_stamp(right_ref_stamp),
  m_cam_stamp(cam_stamp),
  m_block_sizes(block_sizes) {
  // Sanity check
  if (m_block_sizes.size() != 7             ||
      m_block_sizes[0] != NUM_RIGID_PARAMS  ||
      m_block_sizes[1] != NUM_RIGID_PARAMS  ||
      m_block_sizes[2] != NUM_RIGID_PARAMS  ||
      (m_block_sizes[3] != NUM_RIGID_PARAMS && m_block_sizes[3] != NUM_AFFINE_PARAMS) ||
      m_block_sizes[4] != NUM_SCALAR_PARAMS ||
      m_block_sizes[5] != NUM_XYZ_PARAMS    ||
      m_block_sizes[6] != NUM_SCALAR_PARAMS) {
    LOG(FATAL) << "BracketedDepthError: The block sizes were not set up properly.\n";
  }
}

// Call to work with ceres::DynamicNumericDiffCostFunction.
bool BracketedDepthError::operator()(double const* const* parameters, 
                                     double* residuals) const {
  // Current world to camera transform
  Eigen::Affine3d world_to_cam_trans =
    calcWorldToCamBase(parameters[0],  // beg_world_to_ref_t
                       parameters[1],  // end_world_to_ref_t
                       parameters[2],  // ref_to_cam_trans
                       m_left_ref_stamp, m_right_ref_stamp,
                       parameters[6][0],  // ref_to_cam_offset
                       m_cam_stamp);

  // The current transform from the depth point cloud to the camera image
  Eigen::Affine3d depth_to_image;
  if (m_block_sizes[3] == NUM_AFFINE_PARAMS)
    array_to_affine_transform(depth_to_image, parameters[3]);
  else
    array_to_rigid_transform(depth_to_image, parameters[3]);

  // Apply the scale
  double depth_to_image_scale = parameters[4][0];
  depth_to_image.linear() *= depth_to_image_scale;

  // Convert from depth cloud coordinates to cam coordinates
  Eigen::Vector3d M = depth_to_image * m_meas_depth_xyz;

  // Convert to world coordinates
  M = world_to_cam_trans.inverse() * M;

  // Triangulated world point
  Eigen::Vector3d X(parameters[5][0], parameters[5][1], parameters[5][2]);

  // Compute the residuals
  for (size_t it = 0; it < NUM_XYZ_PARAMS; it++) {
    residuals[it] = m_weight * (X[it] - M[it]);
  }

  return true;
}

// Factory to hide the construction of the CostFunction object from the client code.
ceres::CostFunction* BracketedDepthError::Create(double weight, 
                                                 Eigen::Vector3d const& meas_depth_xyz,
                                                 double left_ref_stamp, 
                                                 double right_ref_stamp,
                                                 double cam_stamp, 
                                                 std::vector<int> const& block_sizes) {
  ceres::DynamicNumericDiffCostFunction<BracketedDepthError>* cost_function =
    new ceres::DynamicNumericDiffCostFunction<BracketedDepthError>
    (new BracketedDepthError(weight, meas_depth_xyz, left_ref_stamp, right_ref_stamp,
                            cam_stamp, block_sizes));

  // The residual size is always the same.
  cost_function->SetNumResiduals(NUM_XYZ_PARAMS);

  for (size_t i = 0; i < block_sizes.size(); i++)
    cost_function->AddParameterBlock(block_sizes[i]);

  return cost_function;
}

BracketedDepthMeshError::BracketedDepthMeshError(double weight,
                        Eigen::Vector3d const& meas_depth_xyz,
                        Eigen::Vector3d const& mesh_xyz,
                        double left_ref_stamp, double right_ref_stamp, double cam_stamp,
                        std::vector<int> const& block_sizes):
  m_weight(weight),
  m_meas_depth_xyz(meas_depth_xyz),
  m_mesh_xyz(mesh_xyz),
  m_left_ref_stamp(left_ref_stamp),
  m_right_ref_stamp(right_ref_stamp),
  m_cam_stamp(cam_stamp),
  m_block_sizes(block_sizes) {
  // Sanity check
  if (m_block_sizes.size() != 6 ||
      m_block_sizes[0] != NUM_RIGID_PARAMS  ||
      m_block_sizes[1] != NUM_RIGID_PARAMS  ||
      m_block_sizes[2] != NUM_RIGID_PARAMS  ||
      (m_block_sizes[3] != NUM_RIGID_PARAMS  && m_block_sizes[3] != NUM_AFFINE_PARAMS) ||
      m_block_sizes[4] != NUM_SCALAR_PARAMS ||
      m_block_sizes[5] != NUM_SCALAR_PARAMS) {
    LOG(FATAL) << "BracketedDepthMeshError: The block sizes were not set up properly.\n";
  }
}

// Call to work with ceres::DynamicNumericDiffCostFunction.
bool BracketedDepthMeshError::operator()(double const* const* parameters, double* residuals) const {
  // Current world to camera transform
  Eigen::Affine3d world_to_cam_trans =
    calcWorldToCamBase(parameters[0],  // beg_world_to_ref_t
                       parameters[1],  // end_world_to_ref_t
                       parameters[2],  // ref_to_cam_trans
                       m_left_ref_stamp, m_right_ref_stamp,
                       parameters[5][0],  // ref_to_cam_offset
                       m_cam_stamp);

  // The current transform from the depth point cloud to the camera image
  Eigen::Affine3d depth_to_image;
  if (m_block_sizes[3] == NUM_AFFINE_PARAMS)
    array_to_affine_transform(depth_to_image, parameters[3]);
  else
    array_to_rigid_transform(depth_to_image, parameters[3]);

  // Apply the scale
  double depth_to_image_scale = parameters[4][0];
  depth_to_image.linear() *= depth_to_image_scale;

  // Convert from depth cloud coordinates to cam coordinates
  Eigen::Vector3d M = depth_to_image * m_meas_depth_xyz;

  // Convert to world coordinates
  M = world_to_cam_trans.inverse() * M;

  // Compute the residuals
  for (size_t it = 0; it < NUM_XYZ_PARAMS; it++) {
    residuals[it] = m_weight * (m_mesh_xyz[it] - M[it]);
  }

  return true;
}

// Factory to hide the construction of the CostFunction object from the client code.
ceres::CostFunction* BracketedDepthMeshError::Create(double weight,
                                    Eigen::Vector3d const& meas_depth_xyz,
                                    Eigen::Vector3d const& mesh_xyz,
                                    double left_ref_stamp, double right_ref_stamp,
                                    double cam_stamp, std::vector<int> const& block_sizes) {
  ceres::DynamicNumericDiffCostFunction<BracketedDepthMeshError>* cost_function =
    new ceres::DynamicNumericDiffCostFunction<BracketedDepthMeshError>
    (new BracketedDepthMeshError(weight, meas_depth_xyz, mesh_xyz,
                                  left_ref_stamp, right_ref_stamp,
                                  cam_stamp, block_sizes));

  // The residual size is always the same.
  cost_function->SetNumResiduals(NUM_XYZ_PARAMS);

  for (size_t i = 0; i < block_sizes.size(); i++)
    cost_function->AddParameterBlock(block_sizes[i]);

  return cost_function;
}

XYZError::XYZError(Eigen::Vector3d const& ref_xyz, std::vector<int> const& block_sizes, double weight)
    : m_ref_xyz(ref_xyz), m_block_sizes(block_sizes), m_weight(weight) {
  // Sanity check
  if (m_block_sizes.size() != 1 || m_block_sizes[0] != NUM_XYZ_PARAMS)
    LOG(FATAL) << "XYZError: The block sizes were not set up properly.\n";
}

// Call to work with ceres::DynamicNumericDiffCostFunction.
// Takes array of arrays as parameters.
// TODO(oalexan1): May want to use the analytical Ceres cost function
bool XYZError::operator()(double const* const* parameters, double* residuals) const {
  // Compute the residuals
  for (int it = 0; it < NUM_XYZ_PARAMS; it++)
    residuals[it] = m_weight * (parameters[0][it] - m_ref_xyz[it]);

  return true;
}

// Factory to hide the construction of the CostFunction object from the client code.
ceres::CostFunction* XYZError::Create(Eigen::Vector3d const& ref_xyz,
                                    std::vector<int> const& block_sizes,
                                    double weight) {
  ceres::DynamicNumericDiffCostFunction<XYZError>* cost_function =
    new ceres::DynamicNumericDiffCostFunction<XYZError>
    (new XYZError(ref_xyz, block_sizes, weight));

  // The residual size is always the same
  cost_function->SetNumResiduals(NUM_XYZ_PARAMS);

  // The camera wrapper knows all of the block sizes to add.
  for (size_t i = 0; i < block_sizes.size(); i++) {
    cost_function->AddParameterBlock(block_sizes[i]);
  }
  return cost_function;
}

CamPositionErr::CamPositionErr(const double * init_world_to_cam, double weight):
  m_weight(weight) {

  // Make a copy, as later the value at the pointer will change
  m_init_position = calc_cam_position(init_world_to_cam);
}

bool CamPositionErr::operator()(double const* const* parameters, double* residuals) const {
  Eigen::Vector3d curr_cam_position = calc_cam_position(parameters[0]);
  for (size_t p = 0; p < NUM_XYZ_PARAMS; p++)
      residuals[p] = m_weight * (curr_cam_position[p] - m_init_position[p]);
  for (size_t p = NUM_XYZ_PARAMS; p < rig::NUM_RIGID_PARAMS; p++)
    residuals[p] = 0; // for rotations
  return true;
}

// Factory to hide the construction of the CostFunction object from the client code.
ceres::CostFunction* CamPositionErr::Create(const double * init_world_to_cam, double weight) {

  ceres::DynamicNumericDiffCostFunction<CamPositionErr>* cost_function =
    new ceres::DynamicNumericDiffCostFunction<CamPositionErr>
    (new CamPositionErr(init_world_to_cam, weight));

  // The residual size is always the same
  cost_function->SetNumResiduals(rig::NUM_RIGID_PARAMS);

  // The camera wrapper knows all of the block sizes to add.
  // The full parameter has the rotation too.
  cost_function->AddParameterBlock(rig::NUM_RIGID_PARAMS); 

  return cost_function;
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
                         std::vector<int> const& bracketed_cam_block_sizes,
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
                                   cam_timestamp, bracketed_cam_block_sizes,
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
                           std::vector<int> const& bracketed_depth_block_sizes,
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
                                       bracketed_depth_block_sizes);

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
                            std::vector<int> const& bracketed_depth_mesh_block_sizes,
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
     end_ref_timestamp, cam_timestamp, bracketed_depth_mesh_block_sizes);

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
                         double camera_position_weight,
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
       rig::CamPositionErr::Create(beg_cam_ptr, camera_position_weight);
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
    residual_scales.push_back(camera_position_weight);
    residual_scales.push_back(camera_position_weight);
    residual_scales.push_back(camera_position_weight);
    residual_scales.push_back(1.0); // Rotations will not be constrained
    residual_scales.push_back(1.0);
    residual_scales.push_back(1.0);
    residual_scales.push_back(1.0);
  }
}

// TODO(oalexan1): Must have a struct for all options Must have structs for 
// groups of other related data, e.g., camera params, etc.
void setupRigOptProblem(
    // Inputs
    std::vector<cameraImage> const& cams,
    RigSet& R,
    std::vector<double> const& ref_timestamps,
    std::vector<double>& world_to_cam_vec,
    std::vector<double>& world_to_ref_vec,
    std::vector<double>& ref_to_cam_vec,
    std::vector<double>& ref_identity_vec,
    std::vector<double>& right_identity_vec,
    std::vector<double>& focal_lengths,
    std::vector<Eigen::Vector2d>& optical_centers,
    std::vector<Eigen::VectorXd>& distortions,
    std::vector<double>& depth_to_image_vec,
    std::vector<double>& depth_to_image_scales,
    KeypointVec const& keypoint_vec,
    rig::PidCidFid const& pid_to_cid_fid,
    rig::PidCidFidMap const& pid_cid_fid_inlier,
    rig::PidCidFidToMeshXyz const& pid_cid_fid_mesh_xyz,
    std::vector<Eigen::Vector3d> const& pid_mesh_xyz,
    std::vector<Eigen::Vector3d>& xyz_vec,
    std::vector<Eigen::Vector3d> const& xyz_vec_orig,
    // Block sizes
    std::vector<int> const& bracketed_cam_block_sizes,
    std::vector<int> const& bracketed_depth_block_sizes,
    std::vector<int> const& bracketed_depth_mesh_block_sizes,
    std::vector<int> const& xyz_block_sizes,
    int num_depth_params,
    // Configuration
    std::vector<std::set<std::string>> const& intrinsics_to_float,
    std::set<std::string> const& camera_poses_to_float,
    std::set<std::string> const& depth_to_image_transforms_to_float,
    std::set<std::string> const& fixed_images,
    std::vector<double> const& min_timestamp_offset,
    std::vector<double> const& max_timestamp_offset,
    // Logic flags
    bool no_rig,
    bool fix_rig_translations,
    bool fix_rig_rotations,
    bool float_timestamp_offsets,
    bool float_scale,
    bool affine_depth_to_image,
    bool has_mesh,
    double robust_threshold,
    double tri_robust_threshold,
    double tri_weight,
    double depth_tri_weight,
    double depth_mesh_weight,
    double mesh_tri_weight,
    double camera_position_weight,
    // Outputs
    rig::PidCidFidMap& pid_cid_fid_to_residual_index,
    ceres::Problem& problem,
    std::vector<std::string>& residual_names,
    std::vector<double>& residual_scales) {

  // For when we don't have distortion but must get a pointer to
  // distortion for the interface
  double distortion_placeholder = 0.0;
  Eigen::Vector3d bad_xyz(1.0e+100, 1.0e+100, 1.0e+100);

  // Prepare for the case of fixed rig translations and/or rotations
  std::set<double*> fixed_parameters; // to avoid double fixing
  ceres::SubsetManifold* constant_transform_manifold = nullptr;
  rig::setUpFixRigOptions(no_rig, fix_rig_translations,
                          fix_rig_rotations,
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
                          no_rig, cid, cam_type, cams, ref_timestamps, R,
                          world_to_cam_vec, world_to_ref_vec, ref_to_cam_vec,
                          ref_identity_vec, right_identity_vec,
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
                               bracketed_cam_block_sizes, R.cam_params[cam_type],
                               distortions[cam_type], &distortion_placeholder,
                               beg_cam_ptr, end_cam_ptr, ref_to_cam_ptr,
                               &xyz_vec[pid][0],
                               &R.ref_to_cam_timestamp_offsets[cam_type],
                               &focal_lengths[cam_type], &optical_centers[cam_type][0],
                               R, cam_type, cams[cid].image_name,
                               intrinsics_to_float[cam_type],
                               camera_poses_to_float, fixed_images, fixed_parameters,
                               constant_transform_manifold,
                               min_timestamp_offset[cam_type],
                               max_timestamp_offset[cam_type],
                               no_rig, fix_rig_translations,
                               fix_rig_rotations, float_timestamp_offsets,
                               robust_threshold,
                               problem,
                               residual_names, residual_scales);

      // Add the depth to triangulated point constraint
      Eigen::Vector3d depth_xyz(0, 0, 0);
      bool have_depth_tri_constraint
        = (depth_tri_weight > 0 &&
           rig::depthValue(cams[cid].depth_cloud, dist_ip, depth_xyz));
      if (have_depth_tri_constraint)
        rig::addRigDepthTriCostFun(depth_xyz, beg_ref_timestamp, end_ref_timestamp,
                                   cam_timestamp,
                                   bracketed_depth_block_sizes, num_depth_params,
                                   beg_cam_ptr, end_cam_ptr, ref_to_cam_ptr,
                                   &depth_to_image_vec[num_depth_params * cam_type],
                                   &depth_to_image_scales[cam_type],
                                   &xyz_vec[pid][0],
                                   &R.ref_to_cam_timestamp_offsets[cam_type],
                                   R, cam_type, depth_to_image_transforms_to_float,
                                   float_scale,
                                   affine_depth_to_image,
                                   depth_tri_weight,
                                   robust_threshold,
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
          = (depth_mesh_weight > 0 && mesh_xyz != bad_xyz &&
             rig::depthValue(cams[cid].depth_cloud, dist_ip, depth_xyz));
      }

      if (have_depth_mesh_constraint)
        rig::addRigDepthMeshCostFun(depth_xyz, mesh_xyz, beg_ref_timestamp,
                                    end_ref_timestamp, cam_timestamp,
                                    bracketed_depth_mesh_block_sizes,
                                    num_depth_params, beg_cam_ptr, end_cam_ptr,
                                    ref_to_cam_ptr,
                                    &depth_to_image_vec[num_depth_params * cam_type],
                                    &depth_to_image_scales[cam_type],
                                    &R.ref_to_cam_timestamp_offsets[cam_type],
                                    R, cam_type, depth_to_image_transforms_to_float,
                                    float_scale, affine_depth_to_image,
                                    depth_mesh_weight, robust_threshold,
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
      if (mesh_tri_weight > 0 && avg_mesh_xyz != bad_xyz)
        have_mesh_tri_constraint = true;
    }
    if (have_mesh_tri_constraint)
      rig::addRigMeshTriCostFun(avg_mesh_xyz, xyz_block_sizes,
                                mesh_tri_weight,
                                robust_threshold,
                                &xyz_vec[pid][0], problem, residual_names,
                                residual_scales);

    // Add the constraint that the triangulated point does not go too far
    if (tri_weight > 0.0 && isTriInlier)
      rig::addRigTriCostFun(xyz_vec_orig[pid], xyz_block_sizes, tri_weight,
                            tri_robust_threshold, &xyz_vec[pid][0], problem,
                            residual_names, residual_scales);

  }  // end iterating over pid

  // Add the camera position constraints for the ref cams
  if (camera_position_weight > 0.0)
    rig::addRigCamPosCostFun(cams, R, camera_poses_to_float, ref_timestamps,
                             world_to_cam_vec, world_to_ref_vec, ref_to_cam_vec,
                             ref_identity_vec, right_identity_vec,
                             no_rig, camera_position_weight,
                             problem, residual_names, residual_scales);
}

}  // end namespace rig