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

#include <asp/Rig/RigOutlier.h>
#include <asp/Rig/RigCameraParams.h>
#include <asp/Rig/camera_image.h>
#include <asp/Rig/basic_algs.h>

#include <iostream>
#include <iomanip>

namespace rig {

void flagOutlierByExclusionDist(// Inputs
                                std::vector<rig::CameraParameters> const& cam_params,
                                std::vector<rig::cameraImage>      const& cams,
                                rig::PidCidFid                     const& pid_to_cid_fid,
                                rig::KeypointVec                   const& keypoint_vec,
                                // Outputs
                                PidCidFidMap                            & pid_cid_fid_inlier) {

  // Initialize the output
  pid_cid_fid_inlier.resize(pid_to_cid_fid.size());

  // Iterate though interest point matches
  int num_excl = 0;
  for (size_t pid = 0; pid < pid_to_cid_fid.size(); pid++) {
    for (auto cid_fid = pid_to_cid_fid[pid].begin(); cid_fid != pid_to_cid_fid[pid].end();
         cid_fid++) {
      int cid = cid_fid->first;
      int fid = cid_fid->second;
      int cam_type = cams[cid].camera_type;

      // Initially there are inliers only
      pid_cid_fid_inlier[pid][cid][fid] = 1;

      // Flag as outliers pixels at the image boundary.
      Eigen::Vector2d dist_pix(keypoint_vec[cid][fid].first,
                               keypoint_vec[cid][fid].second);
      Eigen::Vector2i dist_size = cam_params[cam_type].GetDistortedSize();
      Eigen::Vector2i dist_crop_size = cam_params[cam_type].GetDistortedCropSize();
      // Note that if dist_crop_size equals dist_size, which is image
      // size, no outliers are flagged
      if (std::abs(dist_pix[0] - dist_size[0] / 2.0) > dist_crop_size[0] / 2.0  ||
          std::abs(dist_pix[1] - dist_size[1] / 2.0) > dist_crop_size[1] / 2.0) {
        rig::setMapValue(pid_cid_fid_inlier, pid, cid, fid, 0);
        num_excl++;
      }
    }
  }

  std::cout << "Removed " << num_excl << " features based on distorted_crop_size region.\n";
  
  return;
}

// Flag outliers by triangulation angle and reprojection error.  It is
// assumed that the cameras in world_to_cam are up-to-date given the
// current state of optimization, and that the residuals (including
// the reprojection errors) have also been updated beforehand.
void flagOutliersByTriAngleAndReprojErr(// Inputs
  double min_triangulation_angle, double max_reprojection_error,
  rig::PidCidFid               const& pid_to_cid_fid,
  rig::KeypointVec             const& keypoint_vec,
  std::vector<Eigen::Affine3d> const& world_to_cam, 
  std::vector<Eigen::Vector3d> const& xyz_vec,
  PidCidFidMap                 const& pid_cid_fid_to_residual_index,
  std::vector<double>          const& residuals,
  // Outputs
  PidCidFidMap                      & pid_cid_fid_inlier) {

  // Must deal with outliers by triangulation angle before
  // removing outliers by reprojection error, as the latter will
  // exclude some rays which form the given triangulated points.
  int num_outliers_small_angle = 0;
  for (size_t pid = 0; pid < pid_to_cid_fid.size(); pid++) {
    // Find the largest angle among any two intersecting rays
    double max_rays_angle = 0.0;
    bool point_checked = false;
    for (auto cid_fid1 = pid_to_cid_fid[pid].begin();
         cid_fid1 != pid_to_cid_fid[pid].end(); cid_fid1++) {
      int cid1 = cid_fid1->first;
      int fid1 = cid_fid1->second;

      // Deal with inliers only
      if (!rig::getMapValue(pid_cid_fid_inlier, pid, cid1, fid1)) 
        continue;

      Eigen::Vector3d cam_ctr1 = (world_to_cam[cid1].inverse()) * Eigen::Vector3d(0, 0, 0);
      Eigen::Vector3d ray1 = xyz_vec[pid] - cam_ctr1;
      ray1.normalize();

      for (auto cid_fid2 = pid_to_cid_fid[pid].begin();
           cid_fid2 != pid_to_cid_fid[pid].end(); cid_fid2++) {
        int cid2 = cid_fid2->first;
        int fid2 = cid_fid2->second;

        // Look at each cid and next cids
        if (cid2 <= cid1)
          continue;

        // Deal with inliers only
        if (!rig::getMapValue(pid_cid_fid_inlier, pid, cid2, fid2))
           continue;
        point_checked = true;

        Eigen::Vector3d cam_ctr2 = (world_to_cam[cid2].inverse()) * Eigen::Vector3d(0, 0, 0);
        Eigen::Vector3d ray2 = xyz_vec[pid] - cam_ctr2;
        ray2.normalize();

        double curr_angle = (180.0 / M_PI) * acos(ray1.dot(ray2));
        if (std::isnan(curr_angle) || std::isinf(curr_angle)) continue;
        max_rays_angle = std::max(max_rays_angle, curr_angle);
      }
    }

    if (max_rays_angle >= min_triangulation_angle)
      continue;  // This is a good triangulated point, with large angle of convergence
     
    if (!point_checked)
      continue; // this point was an outlier to start with 
       
    // Flag as outliers all the features for this cid and increment the counter
    num_outliers_small_angle++;
    for (auto cid_fid = pid_to_cid_fid[pid].begin();
         cid_fid != pid_to_cid_fid[pid].end(); cid_fid++) {
      int cid = cid_fid->first;
      int fid = cid_fid->second;
      rig::setMapValue(pid_cid_fid_inlier, pid, cid, fid, 0);
    }
  }

  std::cout << std::setprecision(4) 
            << "Removed " << num_outliers_small_angle 
            << " triangulated points out of " << pid_to_cid_fid.size() 
            << " (" << (100.0 * num_outliers_small_angle) / pid_to_cid_fid.size() << ")"
            << " by ray convergence angle.\n";
  
  int num_outliers_reproj = 0;
  int num_total_features = 0;
  for (size_t pid = 0; pid < pid_to_cid_fid.size(); pid++) {
    for (auto cid_fid = pid_to_cid_fid[pid].begin();
         cid_fid != pid_to_cid_fid[pid].end(); cid_fid++) {
      int cid = cid_fid->first;
      int fid = cid_fid->second;

      // Deal with inliers only
      if (!rig::getMapValue(pid_cid_fid_inlier, pid, cid, fid)) 
        continue;

      num_total_features++;

      // Find the pixel residuals
      size_t residual_index = rig::getMapValue(pid_cid_fid_to_residual_index, pid, cid, fid);
      if (residuals.size() <= residual_index + 1) LOG(FATAL) << "Too few residuals.\n";

      double res_x = residuals[residual_index + 0];
      double res_y = residuals[residual_index + 1];
      // NaN values will never be inliers if the comparison is set as below
      bool is_good = (Eigen::Vector2d(res_x, res_y).norm() <= max_reprojection_error);
      if (!is_good) {
        num_outliers_reproj++;
        rig::setMapValue(pid_cid_fid_inlier, pid, cid, fid, 0);
      }
    }
  }

  std::cout << std::setprecision(4) << "Removed " << num_outliers_reproj
            << " outlier features using reprojection error, out of " << num_total_features
            << " (" << (100.0 * num_outliers_reproj) / num_total_features << "%)\n";

  return;
}

}  // end namespace rig
