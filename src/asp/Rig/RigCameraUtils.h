/* Copyright (c) 2017, United States Government, as represented by the
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

#ifndef ASP_RIG_CAMERA_UTILS_H
#define ASP_RIG_CAMERA_UTILS_H

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <string>
#include <vector>
#include <algorithm>

namespace camera {
  // Forward declaration
  class CameraParameters;
}

namespace rig {
  class cameraImage;
}

// Functionality for undistorting and re-distorting images
namespace camera {

// A utility for saving a camera in a format ASP understands. For now do not save
// the distortion.
void writePinholeCamera(camera::CameraParameters const& cam_params,
                        Eigen::Affine3d          const& world_to_cam,
                        std::string              const& filename);
  
// Save the optimized cameras in ASP's Pinhole format. For now do not save
// the distortion model.
// TODO(oalexan1): Move this somewhere else.
void writePinholeCameras(std::vector<std::string>              const& cam_names,
                         std::vector<camera::CameraParameters> const& cam_params,
                         std::vector<rig::cameraImage>         const& cams,
                         std::vector<Eigen::Affine3d>          const& world_to_cam,
                         std::string                           const& out_dir);

}  // namespace camera

#endif  // ASP_RIG_CAMERA_UTILS_H
