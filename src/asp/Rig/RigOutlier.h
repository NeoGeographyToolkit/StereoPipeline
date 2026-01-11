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

#ifndef ASP_RIG_RIG_OUTLIER_H
#define ASP_RIG_RIG_OUTLIER_H

#include <asp/Rig/RigTypeDefs.h>
#include <vector>
#include <Eigen/Core>
#include <Eigen/Geometry>

namespace rig {

class CameraParameters;
class cameraImage;

void flagOutlierByExclusionDist(// Inputs
                                std::vector<rig::CameraParameters> const& cam_params,
                                std::vector<rig::cameraImage>      const& cams,
                                rig::PidCidFid                     const& pid_to_cid_fid,
                                rig::KeypointVec                   const& keypoint_vec,
                                // Outputs
                                PidCidFidMap                            & pid_cid_fid_inlier);

void flagOutliersByTriAngleAndReprojErr
(// Inputs
 double min_triangulation_angle, double max_reprojection_error,
 rig::PidCidFid               const& pid_to_cid_fid,
 rig::KeypointVec             const& keypoint_vec,
 std::vector<Eigen::Affine3d> const& world_to_cam, 
 std::vector<Eigen::Vector3d> const& xyz_vec,
 PidCidFidMap                 const& pid_cid_fid_to_residual_index,
 std::vector<double>          const& residuals,
 // Outputs
 PidCidFidMap                      & pid_cid_fid_inlier);

}  // namespace rig

#endif  // ASP_RIG_RIG_OUTLIER_H
