// __BEGIN_LICENSE__
//  Copyright (c) 2009-2013, United States Government as represented by the
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

/// \file BundleAdjustEigen.h
/// Functions used in bundle adjustment that use Eigen matrices.
/// Keep these separate to make compilation faster.
#ifndef __BUNDLE_ADJUST_EIGEN_H__
#define __BUNDLE_ADJUST_EIGEN_H__

#include <asp/Camera/BundleAdjustCamera.h>

#include <Eigen/Dense>

#include <string>
#include <vector>
#include <set>

namespace asp {
  
// Export the latest xyz values from param_storage to a vector of Vector3
void exportTriPoints(asp::BAParams                const& param_storage, 
                     std::vector<Eigen::Vector3d>      & tri_vec);

// Calculate the optical offsets for each image. For pinhole cameras,
// read these from the camera model. For other cameras, use half of the
// image size.
void calcOpticalOffsets(std::vector<std::string>          const& image_files,
                        std::vector<vw::CamPtr>           const& cams,
                        std::map<std::string, Eigen::Vector2d> & optical_offsets);

// Given a pinhole camera model, find the world-to-camera transform
Eigen::Affine3d calcWorldToCam(vw::camera::PinholeModel const& pin);

// Based on the current camera models, calculate the world-to-camera transforms.
// For pinhole cameras, fetch them directly from the camera models. For adjusted
// pinhole cameras, must combine the adjustments with the pose. For others,
// return the identity matrix.
void calcCameraPoses(std::vector<vw::CamPtr>      const& cams,
                     std::vector<Eigen::Affine3d>      & world_to_cam);

// Once bundle adjustment is done, export the outlier list, camera poses,
// triangulated points, and optical offsets, and write the cnet to an nvm file.
void saveNvm(asp::BaBaseOptions                     const& opt, 
                  vw::ba::ControlNetwork            const& cnet,
                  asp::BAParams                     const& param_storage,
                  std::vector<Eigen::Affine3d>           & world_to_cam,
                  std::map<std::string, Eigen::Vector2d> & optical_offsets);

// Given pinhole cameras and camera-to-world transforms, update the camera poses
// in the pinhole cameras.
void updateCameraPoses(std::string                  const& session, 
                       std::vector<Eigen::Affine3d> const& world_to_cam,
                       std::vector<vw::CamPtr>           & cams);
  
} // end namespace asp

#endif // __BUNDLE_ADJUST_EIGEN_H__