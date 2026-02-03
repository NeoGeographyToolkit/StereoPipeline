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

#ifndef ASP_RIG_OPT_H
#define ASP_RIG_OPT_H

#include <asp/Rig/RigCameraParams.h>
#include <asp/Rig/rig_config.h>
#include <asp/Rig/transform_utils.h>
#include <asp/Rig/RigTypeDefs.h>
#include <asp/Rig/RigData.h>
#include <asp/Rig/RigOptions.h>
#include <asp/Rig/RigCostFunction.h>

#include <asp/Rig/texture_processing.h>

#include <ceres/ceres.h>
#include <ceres/dynamic_numeric_diff_cost_function.h>
#include <ceres/loss_function.h>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <vector>
#include <string>

namespace rig {

// Forward declarations
class cameraImage;
class RigSet;

// Evaluate the residuals before and after optimization
void evalResiduals(// Inputs
                   std::string              const& tag, 
                   std::vector<std::string> const& residual_names,
                   std::vector<double>      const& residual_scales,
                   // Outputs
                   ceres::Problem     & problem, 
                   std::vector<double>& residuals);

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
                        std::vector<double>& residual_scales);

// Run an optimization pass for rig calibration
void runOptPass(int pass,
                int num_depth_params,
                rig::RigOptions               const& opt,
                std::vector<rig::cameraImage> const& imgData,
                std::vector<double>           const& ref_timestamps,
                rig::KeypointVec              const& keypoint_vec,
                rig::PidCidFid                const& pid_to_cid_fid,
                rig::RigBlockSizes            const& block_sizes,
                std::vector<double>           const& min_timestamp_offset,
                std::vector<double>           const& max_timestamp_offset,
                mve::TriangleMesh::Ptr        const& mesh,
                std::shared_ptr<BVHTree>      const& bvh_tree,
                // Outputs
                std::vector<double>                & depth_to_image_scales,
                rig::Extrinsics                    & cams,
                rig::RigSet                        & R,
                std::vector<Eigen::Vector3d>       & xyz_vec,
                rig::PidCidFidMap                  & pid_cid_fid_inlier);

}  // namespace rig

#endif  // ASP_RIG_OPT_H
