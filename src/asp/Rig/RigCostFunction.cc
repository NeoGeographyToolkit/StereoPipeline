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

#include <asp/Rig/RigCostFunction.h>
#include <asp/Rig/camera_image.h>
#include <asp/Rig/rig_config.h>
#include <asp/Rig/transform_utils.h>
#include <asp/Rig/image_lookup.h>
#include <asp/Rig/basic_algs.h>
#include <asp/Rig/rig_utils.h>

// temporary!!!
#include <asp/Rig/texture_processing.h>
#include <asp/Rig/triangulation.h>
#include <asp/Rig/RigOutlier.h>

#include <iostream>
#include <iomanip>

namespace rig {

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
      m_block_sizes[7] != 1) { // This will be overwritten shortly
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

}  // end namespace rig
