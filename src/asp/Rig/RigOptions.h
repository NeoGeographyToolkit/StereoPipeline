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

/// \file RigOptions.h
///
/// Options structure for rig_calibrator, inheriting common options from BaBaseOptions.

#ifndef __ASP_RIG_OPTIONS_H__
#define __ASP_RIG_OPTIONS_H__

#include <asp/Camera/BundleAdjustCamera.h>
#include <string>

namespace rig {

struct RigOptions: public asp::BaBaseOptions {
  std::string rig_config, image_sensor_list, intrinsics_to_float_str,
    camera_poses_to_float_str, depth_to_image_transforms_to_float_str,
    mesh, hugin_file, xyz_file, camera_poses, extra_list,
    out_texture_dir;
  double bracket_len, depth_tri_weight, mesh_tri_weight, depth_mesh_weight,
    timestamp_offsets_max_change,
    initial_max_reprojection_error, max_reprojection_error,
    min_ray_dist, max_ray_dist;
  int num_overlaps, num_match_threads, calibrator_num_passes;
  bool fix_rig_translations, fix_rig_rotations, float_scale,
    float_timestamp_offsets, use_initial_triangulated_points,
    affine_depth_to_image, registration, skip_post_registration, no_rig,
    no_nvm_matches, use_initial_rig_transforms, bracket_single_image,
    nearest_neighbor_interp, read_nvm_no_shift, save_nvm_no_shift,
    save_matches, export_to_voxblox, save_pinhole_cameras,
    save_transformed_depth_clouds, verbose;

  // Parsed options
  std::vector<std::set<std::string>> intrinsics_to_float;
  std::set<std::string> camera_poses_to_float;
  std::set<std::string> depth_to_image_transforms_to_float;
  std::set<std::string> fixed_images;

  RigOptions(): asp::BaBaseOptions(),
    bracket_len(0.6), depth_tri_weight(1000.0),
    mesh_tri_weight(0.0), depth_mesh_weight(0.0),
    timestamp_offsets_max_change(1.0),
    initial_max_reprojection_error(300.0), max_reprojection_error(25.0),
    min_ray_dist(0.0), max_ray_dist(100.0),
    num_overlaps(0), num_match_threads(8), calibrator_num_passes(2),
    fix_rig_translations(false), fix_rig_rotations(false), float_scale(false),
    float_timestamp_offsets(false), use_initial_triangulated_points(false),
    affine_depth_to_image(false), registration(false), skip_post_registration(false),
    no_rig(false), no_nvm_matches(false), use_initial_rig_transforms(false),
    bracket_single_image(false), nearest_neighbor_interp(false),
    read_nvm_no_shift(false), save_nvm_no_shift(false), save_matches(false),
    export_to_voxblox(false), save_pinhole_cameras(false),
    save_transformed_depth_clouds(false), verbose(false) {}
};

} // namespace rig

#endif // __ASP_RIG_OPTIONS_H__
