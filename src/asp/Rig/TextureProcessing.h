/* Copyright (c) 2021-2026, United States Government, as represented by the
 * Administrator of the National Aeronautics and Space Administration.
 *
 * All rights reserved.
 *
 * The "ISAAC - Integrated System for Autonomous and Adaptive Caretaking
 * platform" software is licensed under the Apache License, Version 2.0
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

#ifndef ASP_RIG_TEXTURE_PROCESSING_H
#define ASP_RIG_TEXTURE_PROCESSING_H

#include <asp/Rig/RigCameraParams.h>
#include <asp/Rig/rig_utils.h>
#include <asp/Rig/camera_image.h>

#include <Eigen/Geometry>
#include <Eigen/Core>

// texrecon includes
#include <mve/mesh_io_ply.h>
#include <mve/mesh_info.h>
#include <mve/mesh.h>
#include <mve/image.h>
#include <acc/bvh_tree.h>
#include <tex/timer.h>
#include <tex/texturing.h>
#include <tex/tri.h>
#include <tex/texture_patch.h>
#include <tex/rectangular_bin.h>
#include <tex/material_lib.h>
#include <util/exception.h>
#include <math/vector.h>

// OpenCV
#include <opencv2/highgui/highgui.hpp>

#include <vector>
#include <map>
#include <limits>
#include <string>

typedef acc::BVHTree<unsigned int, math::Vec3f> BVHTree;

namespace rig {

// Small utilities
inline Eigen::Vector3d badMeshXyz() {
  return Eigen::Vector3d(1.0e+100, 1.0e+100, 1.0e+100);
}
Eigen::Vector3d vec3f_to_eigen(math::Vec3f const& v);
math::Vec3f eigen_to_vec3f(Eigen::Vector3d const& V);

// Load and prepare a mesh
void loadMeshBuildTree(std::string const& mesh_file, 
                       mve::TriangleMesh::Ptr& mesh,
                       std::shared_ptr<mve::MeshInfo>& mesh_info,
                       std::shared_ptr<tex::Graph>& graph,
                       std::shared_ptr<BVHTree>& bvh_tree);

// Put an textured mesh obj file in a string
void formObjCustomUV(mve::TriangleMesh::ConstPtr mesh, 
                     std::vector<Eigen::Vector3i> const& face_vec,
                     std::map<int, Eigen::Vector2d> const& uv_map,
                     std::string const& out_prefix, 
                     std::string& obj_str);

void formMtl(std::string const& out_prefix, std::string& mtl_str);

// Find where ray emanating from a distorted pixel intersects a mesh. Return true
// on success.
bool ray_mesh_intersect(Eigen::Vector2d const& dist_pix,
                        rig::CameraParameters const& cam_params,
                        Eigen::Affine3d const& world_to_cam,
                        mve::TriangleMesh::Ptr const& mesh,
                        std::shared_ptr<BVHTree> const& bvh_tree,
                        double min_ray_dist, double max_ray_dist,
                        // Output
                        Eigen::Vector3d& intersection);

void meshProject(mve::TriangleMesh::Ptr const& mesh, std::shared_ptr<BVHTree> const& bvh_tree,
                 cv::Mat const& image,
                 Eigen::Affine3d const& world_to_cam, rig::CameraParameters const& cam_params,
                 std::string const& out_prefix);

void meshProjectCameras(std::vector<std::string> const& cam_names,
                        std::vector<rig::CameraParameters> const& cam_params,
                        std::vector<rig::cameraImage> const& cam_images,
                        std::vector<Eigen::Affine3d> const& world_to_cam,
                        mve::TriangleMesh::Ptr const& mesh,
                        std::shared_ptr<BVHTree> const& bvh_tree,
                        std::string const& out_dir);

void meshTriangulations(// Inputs
  std::vector<rig::CameraParameters> const& cam_params,
  std::vector<rig::cameraImage>      const& cams,
  std::vector<Eigen::Affine3d>       const& world_to_cam,
  rig::PidCidFid                     const& pid_to_cid_fid,
  PidCidFidMap                       const& pid_cid_fid_inlier,
  rig::KeypointVec                   const& keypoint_vec,
  double min_ray_dist, double max_ray_dist,
  mve::TriangleMesh::Ptr             const& mesh, std::shared_ptr<BVHTree> const& bvh_tree,
  // Outputs
  rig::PidCidFidToMeshXyz                 & pid_cid_fid_mesh_xyz,
  std::vector<Eigen::Vector3d>            & pid_mesh_xyz);
  
}  // namespace rig

#endif  // ASP_RIG_TEXTURE_PROCESSING_H
