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

#ifndef RIG_COST_FUNCTION_H_
#define RIG_COST_FUNCTION_H_

#include <ceres/ceres.h>
#include <ceres/dynamic_numeric_diff_cost_function.h>
#include <ceres/loss_function.h>

#include <vector>
#include <string>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <Rig/RigCameraParams.h>
#include <Rig/rig_config.h>
#include <Rig/transform_utils.h>

namespace rig {

// Forward declarations
class cameraImage;
class RigSet;

// Set up block sizes for various cost functions
void set_up_block_sizes(int num_depth_params,
                        std::vector<int> & bracketed_cam_block_sizes,
                        std::vector<int> & bracketed_depth_block_sizes,
                        std::vector<int> & bracketed_depth_mesh_block_sizes,
                        std::vector<int> & xyz_block_sizes);

// If applicable, set up the parameters block to fix the rig translations and/or rotations
void setUpFixRigOptions(bool no_rig, bool fix_rig_translations, bool fix_rig_rotations,
                        ceres::SubsetManifold*& constant_transform_manifold);

ceres::LossFunction* GetLossFunction(std::string cost_fun, double th);

// An error function minimizing the error of projecting
// an xyz point into a camera that is bracketed by
// two reference cameras. The precise timestamp offset
// between them is also floated.
struct BracketedCamError {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  BracketedCamError(Eigen::Vector2d const& meas_dist_pix,
                    double left_ref_stamp, double right_ref_stamp, double cam_stamp,
                    std::vector<int> const& block_sizes,
                    rig::CameraParameters const& cam_params);

  // Call to work with ceres::DynamicNumericDiffCostFunction.
  bool operator()(double const* const* parameters, double* residuals) const;

  // Factory to hide the construction of the CostFunction object from the client code.
  static ceres::CostFunction*
  Create(Eigen::Vector2d const& meas_dist_pix, double left_ref_stamp, double right_ref_stamp,
         double cam_stamp, std::vector<int> const& block_sizes,
         rig::CameraParameters const& cam_params);

 private:
  Eigen::Vector2d m_meas_dist_pix;             // Measured distorted current camera pixel
  double m_left_ref_stamp, m_right_ref_stamp;  // left and right ref cam timestamps
  double m_cam_stamp;                          // Current cam timestamp
  std::vector<int> m_block_sizes;
  rig::CameraParameters m_cam_params;
  int m_num_focal_lengths;
};  // End class BracketedCamError

// An error function minimizing the product of a given weight and the
// error between a triangulated point and a measured depth point. The
// depth point needs to be transformed to world coordinates first. For
// that one has to do pose interpolation.
struct BracketedDepthError {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  BracketedDepthError(double weight, Eigen::Vector3d const& meas_depth_xyz,
                      double left_ref_stamp, double right_ref_stamp, double cam_stamp,
                      std::vector<int> const& block_sizes);

  // Call to work with ceres::DynamicNumericDiffCostFunction.
  bool operator()(double const* const* parameters, double* residuals) const;

  // Factory to hide the construction of the CostFunction object from the client code.
  static ceres::CostFunction* Create(double weight, Eigen::Vector3d const& meas_depth_xyz,
                                     double left_ref_stamp, double right_ref_stamp,
                                     double cam_stamp, std::vector<int> const& block_sizes);

 private:
  double m_weight;                             // How much weight to give to this constraint
  Eigen::Vector3d m_meas_depth_xyz;            // Measured depth measurement
  double m_left_ref_stamp, m_right_ref_stamp;  // left and right ref cam timestamps
  double m_cam_stamp;                          // Current cam timestamp
  std::vector<int> m_block_sizes;
};  // End class BracketedDepthError

// An error function minimizing the product of a given weight and the
// error between a mesh point and a transformed measured depth point. The
// depth point needs to be transformed to world coordinates first. For
// that one has to do pose interpolation.
struct BracketedDepthMeshError {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  BracketedDepthMeshError(double weight,
                          Eigen::Vector3d const& meas_depth_xyz,
                          Eigen::Vector3d const& mesh_xyz,
                          double left_ref_stamp, double right_ref_stamp, double cam_stamp,
                          std::vector<int> const& block_sizes);

  // Call to work with ceres::DynamicNumericDiffCostFunction.
  bool operator()(double const* const* parameters, double* residuals) const;

  // Factory to hide the construction of the CostFunction object from the client code.
  static ceres::CostFunction* Create(double weight,
                                     Eigen::Vector3d const& meas_depth_xyz,
                                     Eigen::Vector3d const& mesh_xyz,
                                     double left_ref_stamp, double right_ref_stamp,
                                     double cam_stamp, std::vector<int> const& block_sizes);

 private:
  double m_weight;                             // How much weight to give to this constraint
  Eigen::Vector3d m_meas_depth_xyz;            // Measured depth measurement
  Eigen::Vector3d m_mesh_xyz;                  // Point on preexisting mesh
  double m_left_ref_stamp, m_right_ref_stamp;  // left and right ref cam timestamps
  double m_cam_stamp;                          // Current cam timestamp
  std::vector<int> m_block_sizes;
};  // End class BracketedDepthMeshError

// An error function minimizing a weight times the distance from a
// variable xyz point to a fixed reference xyz point.
struct XYZError {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  XYZError(Eigen::Vector3d const& ref_xyz, std::vector<int> const& block_sizes, double weight);

  // Call to work with ceres::DynamicNumericDiffCostFunction.
  // Takes array of arrays as parameters.
  // TODO(oalexan1): May want to use the analytical Ceres cost function
  bool operator()(double const* const* parameters, double* residuals) const;

  // Factory to hide the construction of the CostFunction object from the client code.
  static ceres::CostFunction* Create(Eigen::Vector3d const& ref_xyz,
                                     std::vector<int> const& block_sizes,
                                     double weight);

 private:
  Eigen::Vector3d m_ref_xyz;  // reference xyz
  std::vector<int> m_block_sizes;
  double m_weight;
};  // End class XYZError

/// A Ceres cost function. The residual is the difference between the
/// initial position and optimized position, multiplied by given weight.
/// The variable has the rotations as well, but those are ignored.
struct CamPositionErr {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  CamPositionErr(const double * init_world_to_cam, double weight);

  bool operator()(double const* const* parameters, double* residuals) const;

  // Factory to hide the construction of the CostFunction object from the client code.
  static ceres::CostFunction* Create(const double * init_world_to_cam, double weight);

  Eigen::Vector3d m_init_position;
  double m_weight;
};
 
// Evaluate the residuals before and after optimization
void evalResiduals(// Inputs
                   std::string              const& tag, 
                   std::vector<std::string> const& residual_names,
                   std::vector<double>      const& residual_scales,
                   // Outputs
                   ceres::Problem     & problem, 
                   std::vector<double>& residuals);

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
    std::vector<double>& residual_scales);

}  // namespace rig

#endif  // RIG_COST_FUNCTION_H_