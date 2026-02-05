/* Copyright (c) 2017-2026, United States Government, as represented by the
 * Administrator of the National Aeronautics and Space Administration.
 * 
 * All rights reserved.
 * 
 * The Astrobee platform is licensed under the Apache License, Version 2.0
 * (the "License"); you may not use this file except in compliance with the
 * License. You may obtain a copy of the License at
 * 
 *     http://www.apache.org/licenses/LICENSE-2.0
 * 
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the
 * License for the specific language governing permissions and limitations
 * under the License.
 */

#ifndef ASP_RIG_DEM_H
#define ASP_RIG_DEM_H

#include <asp/Rig/RigTypeDefs.h>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <vector>
#include <string>

namespace rig {

// Forward declarations
class CameraParameters;
class cameraImage;

// Update triangulated points with DEM heights. This function shoots rays from
// cameras through keypoints, intersects them with a DEM, averages the results,
// and projects them vertically onto the DEM surface.
// There is a version of this with the same name for bundle adjustment.
void updateTriPtsFromDem(std::vector<rig::CameraParameters> const& cam_params,
                         std::vector<rig::cameraImage>      const& cams,
                         std::vector<Eigen::Affine3d>       const& world_to_cam,
                         rig::PidCidFid                     const& pid_to_cid_fid,
                         PidCidFidMap                       const& pid_cid_fid_inlier,
                         rig::KeypointVec                   const& keypoint_vec,
                         std::vector<Eigen::Vector3d>       const& xyz_vec_orig,
                         std::string                        const& dem_filename,
                         // Outputs
                         std::vector<Eigen::Vector3d>            & dem_xyz_vec);

}  // namespace rig

#endif  // ASP_RIG_DEM_H