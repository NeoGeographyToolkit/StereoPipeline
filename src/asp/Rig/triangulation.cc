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

#include <asp/Rig/triangulation.h>
#include <asp/Rig/basic_algs.h>
#include <asp/Rig/system_utils.h>
#include <asp/Rig/camera_image.h>
#include <asp/Rig/RigCameraParams.h>
#include <asp/OpenMVG/projection.hpp>
#include <asp/OpenMVG/triangulation_nview.hpp>

namespace rig {

void Triangulate(bool rm_invalid_xyz, double focal_length,
                 std::vector<Eigen::Affine3d> const& world_to_cam,
                 rig::CidToKeypointMatVec const& cid_to_keypoint_map,
                 rig::PidToCidFidVec * pid_to_cid_fid,
                 std::vector<Eigen::Vector3d> * pid_to_xyz) {

  Eigen::Matrix3d k;
  k << focal_length, 0, 0,
    0, focal_length, 0,
    0, 0, 1;

  // Build p matrices for all of the cameras. aspOpenMVG::Triangulation
  // will be holding pointers to all of the cameras.
  std::vector<aspOpenMVG::Mat34> cid_to_p(world_to_cam.size());
  for (size_t cid = 0; cid < cid_to_p.size(); cid++) {
    aspOpenMVG::P_From_KRt(k, world_to_cam[cid].linear(),
                        world_to_cam[cid].translation(), &cid_to_p[cid]);
  }

  pid_to_xyz->resize(pid_to_cid_fid->size());
  for (int pid = pid_to_cid_fid->size() - 1; pid >= 0; pid--) {
    aspOpenMVG::Triangulation tri;
    for (std::pair<int, int> const& cid_fid : pid_to_cid_fid->at(pid)) {
      tri.add(cid_to_p[cid_fid.first],  // they're holding a pointer to this
              cid_to_keypoint_map[cid_fid.first].col(cid_fid.second));
    }
    Eigen::Vector3d solution = tri.compute();
    if ( rm_invalid_xyz && (std::isnan(solution[0]) || tri.minDepth() < 0) ) {
      pid_to_xyz->erase(pid_to_xyz->begin() + pid);
      pid_to_cid_fid->erase(pid_to_cid_fid->begin() + pid);
    } else {
      pid_to_xyz->at(pid) = solution;
    }
  }

}

// Triangulate rays emanating from given undistorted and centered pixels
Eigen::Vector3d TriangulatePair(double focal_length1, double focal_length2,
                                Eigen::Affine3d const& world_to_cam1,
                                Eigen::Affine3d const& world_to_cam2,
                                Eigen::Vector2d const& pix1,
                                Eigen::Vector2d const& pix2) {
  Eigen::Matrix3d k1;
  k1 << focal_length1, 0, 0, 0, focal_length1, 0, 0, 0, 1;

  Eigen::Matrix3d k2;
  k2 << focal_length2, 0, 0, 0, focal_length2, 0, 0, 0, 1;

  aspOpenMVG::Mat34 cid_to_p1, cid_to_p2;
  aspOpenMVG::P_From_KRt(k1, world_to_cam1.linear(), world_to_cam1.translation(), &cid_to_p1);
  aspOpenMVG::P_From_KRt(k2, world_to_cam2.linear(), world_to_cam2.translation(), &cid_to_p2);

  aspOpenMVG::Triangulation tri;
  tri.add(cid_to_p1, pix1);
  tri.add(cid_to_p2, pix2);

  Eigen::Vector3d solution = tri.compute();
  return solution;
}

// Triangulate n rays emanating from given undistorted and centered pixels
Eigen::Vector3d Triangulate(std::vector<double>          const& focal_length_vec,
                            std::vector<Eigen::Affine3d> const& world_to_cam_vec,
                            std::vector<Eigen::Vector2d> const& pix_vec) {
  if (focal_length_vec.size() != world_to_cam_vec.size() ||
      focal_length_vec.size() != pix_vec.size())
    LOG(FATAL) << "All inputs to Triangulate() must have the same size.";

  if (focal_length_vec.size() <= 1)
    LOG(FATAL) << "At least two rays must be passed to Triangulate().";

  aspOpenMVG::Triangulation tri;

  for (size_t it = 0; it < focal_length_vec.size(); it++) {
    Eigen::Matrix3d k;
    k << focal_length_vec[it], 0, 0, 0, focal_length_vec[it], 0, 0, 0, 1;

    aspOpenMVG::Mat34 cid_to_p;
    aspOpenMVG::P_From_KRt(k, world_to_cam_vec[it].linear(),
                        world_to_cam_vec[it].translation(),
                        &cid_to_p);

    tri.add(cid_to_p, pix_vec[it]);
  }

  Eigen::Vector3d solution = tri.compute();
  return solution;
}

void multiViewTriangulation(// Inputs
                            std::vector<rig::CameraParameters>   const& cam_params,
                            std::vector<rig::cameraImage>     const& cams,
                            std::vector<Eigen::Affine3d>            const& world_to_cam,
                            rig::PidToCidFidVec         const& pid_to_cid_fid,
                            rig::KeypointVec
                            const& keypoint_vec,
                            // Outputs
                            PidCidFid& pid_cid_fid_inlier,
                            std::vector<Eigen::Vector3d>& xyz_vec) {

  if (cams.size() != world_to_cam.size()) 
    LOG(FATAL) << "Expecting as many images as cameras.\n";
  
  xyz_vec.clear();
  xyz_vec.resize(pid_to_cid_fid.size());
  for (size_t pid = 0; pid < pid_to_cid_fid.size(); pid++)
    xyz_vec[pid] = Eigen::Vector3d(0, 0, 0); // initialize to 0
  
  for (size_t pid = 0; pid < pid_to_cid_fid.size(); pid++) {
    std::vector<double> focal_length_vec;
    std::vector<Eigen::Affine3d> world_to_cam_aff_vec;
    std::vector<Eigen::Vector2d> pix_vec;

    for (auto cid_fid = pid_to_cid_fid[pid].begin(); cid_fid != pid_to_cid_fid[pid].end();
         cid_fid++) {
      int cid = cid_fid->first;
      int fid = cid_fid->second;

      // Triangulate inliers only
      if (!rig::getMapValue(pid_cid_fid_inlier, pid, cid, fid))
        continue;

      Eigen::Vector2d dist_ip(keypoint_vec[cid][fid].first, keypoint_vec[cid][fid].second);
      Eigen::Vector2d undist_ip;
      cam_params[cams[cid].camera_type].Convert<rig::DISTORTED, rig::UNDISTORTED_C>
        (dist_ip, &undist_ip);

      focal_length_vec.push_back(cam_params[cams[cid].camera_type].GetFocalLength());
      world_to_cam_aff_vec.push_back(world_to_cam[cid]);
      pix_vec.push_back(undist_ip);
    }

    if (pix_vec.size() < 2) {
      // If after outlier filtering less than two rays are left, can't triangulate.
      // Must set all features for this pid to outliers.
      for (auto cid_fid = pid_to_cid_fid[pid].begin();
           cid_fid != pid_to_cid_fid[pid].end();
           cid_fid++) {
        int cid = cid_fid->first;
        int fid = cid_fid->second;
        rig::setMapValue(pid_cid_fid_inlier, pid, cid, fid, 0);
      }

      // Nothing else to do
      continue;
    }

    // Triangulate n rays emanating from given undistorted and centered pixels
    xyz_vec[pid] = rig::Triangulate(focal_length_vec, world_to_cam_aff_vec,
                                          pix_vec);

    bool bad_xyz = false;
    for (int c = 0; c < xyz_vec[pid].size(); c++) {
      if (std::isinf(xyz_vec[pid][c]) || std::isnan(xyz_vec[pid][c])) 
        bad_xyz = true;
    }
    if (bad_xyz) {
      // if triangulation failed, must set all features for this pid to outliers.
      for (auto cid_fid = pid_to_cid_fid[pid].begin(); cid_fid != pid_to_cid_fid[pid].end();
           cid_fid++) {
        int cid = cid_fid->first;
        int fid = cid_fid->second;
        rig::setMapValue(pid_cid_fid_inlier, pid, cid, fid, 0);
      }
    }
    
  } // end iterating over triangulated points
  
  return;
}


// A triangulated point that is equal to (0, 0, 0), inf, or NaN, is not good.
bool isGoodTri(Eigen::Vector3d const& P) {
  for (int c = 0; c < P.size(); c++) {
    if (std::isinf(P[c]) || std::isnan(P[c]))
      return false;
  }
  
  if (P[0] == 0 && P[1] == 0 && P[2] == 0) 
    return false;
  
  return true;
}

} // end namespace rig