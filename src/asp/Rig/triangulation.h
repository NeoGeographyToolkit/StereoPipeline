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

#ifndef ASP_RIG_TRIANGULATION_H
#define ASP_RIG_TRIANGULATION_H

#include <vector>
#include <map>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <asp/Rig/camera_image.h> // For rig::cameraImage
#include <asp/Rig/RigCameraParams.h> // For camera::CameraParameters

namespace rig {

// Triangulate a 3D point from multiple 2D observations and camera poses. This
// version takes a single focal length and a map of camera IDs to keypoints.
void Triangulate(bool rm_invalid_xyz, double focal_length,
                 std::vector<Eigen::Affine3d> const& world_to_cam,
                 std::vector<Eigen::Matrix2Xd> const& cid_to_keypoint_map,
                 std::vector<std::map<int, int>> * pid_to_cid_fid,
                 std::vector<Eigen::Vector3d> * pid_to_xyz);

// Triangulate rays emanating from given undistorted and centered pixels for a
// pair of cameras.
Eigen::Vector3d TriangulatePair(double focal_length1, double focal_length2,
                                Eigen::Affine3d const& world_to_cam1,
                                Eigen::Affine3d const& world_to_cam2,
                                Eigen::Vector2d const& pix1,
                                Eigen::Vector2d const& pix2);

// Triangulate n rays emanating from given undistorted and centered pixels. This
// version takes vectors of focal lengths, camera poses, and pixels.
Eigen::Vector3d Triangulate(std::vector<double>          const& focal_length_vec,
                            std::vector<Eigen::Affine3d> const& world_to_cam_vec,
                            std::vector<Eigen::Vector2d> const& pix_vec);

// Perform multi-view triangulation to compute 3D points from 2D keypoints
// across multiple cameras.
void multiViewTriangulation(std::vector<camera::CameraParameters>   const& cam_params,
                            std::vector<rig::cameraImage>     const& cams,
                            std::vector<Eigen::Affine3d>            const& world_to_cam,
                            std::vector<std::map<int, int>>         const& pid_to_cid_fid,
                            std::vector<std::vector<std::pair<float, float>>>
                            const& keypoint_vec,
                            // Outputs
                            std::vector<std::map<int, std::map<int, int>>>&
                            pid_cid_fid_inlier,
                            std::vector<Eigen::Vector3d>& xyz_vec);

// A triangulated point that is equal to (0, 0, 0), inf, or NaN, is not good
bool isGoodTri(Eigen::Vector3d const& P);

} // end namespace rig

#endif // ASP_RIG_TRIANGULATION_H
