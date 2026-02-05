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

#include <asp/Rig/RigDem.h>
#include <asp/Rig/RigCameraUtils.h>
#include <asp/Rig/RigCameraParams.h>
#include <asp/Rig/CameraImage.h>
#include <asp/Rig/BasicAlgs.h>
#include <asp/Core/BundleAdjustUtils.h>
#include <asp/Core/ImageUtils.h>

#include <vw/Image/Interpolation.h>
#include <vw/Core/Log.h>
#include <vw/Core/Stopwatch.h>
#include <vw/Cartography/CameraBBox.h>
#include <vw/Cartography/GeoReference.h>
#include <vw/Core/ProgressCallback.h>

// This has logic that makes heavy use of both rig and asp functionality

namespace rig {

// Update triangulated points with DEM heights (rig-specific version). See the
// There is a version of this with the same name for bundle adjustment. Triangulated
// points will also be overwritten by DEM points for each successfully found DEM point.
void updateTriPtsFromDem(std::vector<rig::CameraParameters> const& cam_params,
                         std::vector<rig::cameraImage>      const& cams,
                         std::vector<Eigen::Affine3d>       const& world_to_cam,
                         rig::PidCidFid                     const& pid_to_cid_fid,
                         PidCidFidMap                       const& pid_cid_fid_inlier,
                         rig::KeypointVec                   const& keypoint_vec,
                         std::string                        const& dem_filename,
                         // Outputs
                         std::vector<Eigen::Vector3d>            & xyz_vec_orig,
                         std::vector<Eigen::Vector3d>            & xyz_vec,
                         std::vector<Eigen::Vector3d>            & dem_xyz_vec) {

  // Load DEM for height constraints
  vw::vw_out() << "Updating triangulated points with DEM heights.\n";
  vw::cartography::GeoReference dem_georef;
  vw::ImageViewRef<vw::PixelMask<double>> masked_dem;
  asp::create_masked_dem(dem_filename, dem_georef, masked_dem);

  // Initialize output
  int num_tri_points = pid_to_cid_fid.size();
  dem_xyz_vec.resize(num_tri_points, Eigen::Vector3d(0, 0, 0));

  // Create interpolated DEM for pixel lookups
  vw::PixelMask<double> invalid_val;
  vw::ImageViewRef<vw::PixelMask<double>> interp_dem
   = vw::interpolate(masked_dem, vw::BilinearInterpolation(),
                     vw::ValueEdgeExtension<vw::PixelMask<float>>(invalid_val));

  // Progress reporting
  vw::TerminalProgressCallback tpc("asp", "\t--> ");
  double inc_amount = 1.0 / std::max(1, num_tri_points);
  tpc.report_progress(0);

  for (int pid = 0; pid < num_tri_points; pid++) {
    tpc.report_incremental_progress(inc_amount);

    // Get the initial triangulated point as a guess
    vw::Vector3 xyz_guess(xyz_vec_orig[pid][0], xyz_vec_orig[pid][1], xyz_vec_orig[pid][2]);

    // Skip invalid initial points
    if (xyz_guess == vw::Vector3(0, 0, 0))
      continue;

    vw::Vector3 accum_xyz(0, 0, 0);
    int num_intersections = 0;

    // Iterate through all camera observations for this triangulated point
    for (const auto& cid_fid: pid_to_cid_fid[pid]) {

      int cid = cid_fid.first;
      int fid = cid_fid.second;

      // Skip outliers
      if (!rig::getMapValue(pid_cid_fid_inlier, pid, cid, fid))
        continue;

      // Get the distorted pixel observation
      std::pair<float, float> kp_pair = keypoint_vec[cid][fid];
      Eigen::Vector2d dist_pix(kp_pair.first, kp_pair.second);

      // Calculate camera center and ray direction
      Eigen::Vector3d cam_ctr, cam_dir;
      rig::calcCamCtrDir(cam_params[cams[cid].camera_type], dist_pix, world_to_cam[cid],
                         cam_ctr, cam_dir);

      // Convert Eigen types to VW types for DEM intersection
      vw::Vector3 cam_ctr_vw(cam_ctr[0], cam_ctr[1], cam_ctr[2]);
      vw::Vector3 ray_dir_vw(cam_dir[0], cam_dir[1], cam_dir[2]);

      // Intersect ray with DEM
      bool treat_nodata_as_zero = false;
      bool has_intersection = false;
      double height_error_tol = 0.001; // 1 mm
      double max_abs_tol      = 1e-14;
      double max_rel_tol      = 1e-14;
      int num_max_iter        = 25;
      vw::Vector3 dem_xyz = vw::cartography::camera_pixel_to_dem_xyz
        (cam_ctr_vw, ray_dir_vw,
         vw::pixel_cast<vw::PixelMask<float>>(masked_dem),
         dem_georef, treat_nodata_as_zero, has_intersection,
         height_error_tol, max_abs_tol, max_rel_tol, num_max_iter, xyz_guess);

      if (!has_intersection)
        continue;

      accum_xyz += dem_xyz;
      num_intersections++;
    }

    // Average the successful intersections
    vw::Vector3 dem_xyz(0, 0, 0);
    if (num_intersections > 0)
      dem_xyz = accum_xyz / double(num_intersections);

    if (dem_xyz == vw::Vector3(0, 0, 0))
      continue; // Skip invalid points

    // Project vertically onto DEM
    vw::Vector3 observation = dem_xyz;
    if (asp::updatePointHeightFromDem(dem_georef, interp_dem, observation))
      dem_xyz = observation;

    // Convert from VW to Eigen
    dem_xyz_vec[pid] = Eigen::Vector3d(dem_xyz[0], dem_xyz[1], dem_xyz[2]);

    // If dem_xyz_vec[pid] is not zero, also overwrite xyz_vec and xyz_vec_orig,
    // so the triangulated points can start on the DEM
    if (dem_xyz_vec[pid] != Eigen::Vector3d(0, 0, 0)) {
      xyz_vec[pid] = dem_xyz_vec[pid];
      xyz_vec_orig[pid] = dem_xyz_vec[pid];
    }
  }

  tpc.report_finished();
}

}  // namespace rig
