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

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <vector>
#include <string>

namespace rig {

// Forward declarations
class cameraImage;
class RigSet;

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
