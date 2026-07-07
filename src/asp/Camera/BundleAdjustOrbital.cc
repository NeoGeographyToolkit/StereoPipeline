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

/// \file BundleAdjustOrbital.cc

#include <asp/Camera/BundleAdjustOrbital.h>
#include <asp/Camera/BundleAdjustOptions.h>
#include <asp/Camera/BaState.h>
#include <asp/Core/FileUtils.h>

#include <vw/Math/Quaternion.h>
#include <vw/Core/Exception.h>
#include <vw/Core/Log.h>

// Turn off warnings from ceres
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-local-typedefs"
#include <ceres/ceres.h>
#include <ceres/manifold.h>
#pragma GCC diagnostic pop

#include <boost/algorithm/string.hpp>

#include <map>
#include <string>
#include <vector>

namespace asp {

// Apply an orbital group rigid pose (axis_angle + translation, rotation about the
// group centroid) to a camera's original position. See the header for the formula.
vw::Vector3 applyOrbitalGroupPose(double const* pose,
                                  vw::Vector3 const& orig_position,
                                  vw::Vector3 const& centroid) {
  vw::Vector3 axis_angle(pose[0], pose[1], pose[2]);
  vw::Vector3 translation(pose[3], pose[4], pose[5]);
  vw::Quat rot = vw::math::axis_angle_to_quaternion(axis_angle);
  return centroid + rot.rotate(orig_position - centroid) + translation;
}

// Build the orbital groups from --orbital-group-list. See that option for more info.
void buildOrbitalGroups(asp::BaOptions const& opt,
                        std::vector<vw::CamPtr> const& orig_cams,
                        asp::OrbitalGroups & groups) {

  groups = asp::OrbitalGroups(); // reset
  if (opt.orbital_group_list.empty())
    return;

  int num_cams = opt.image_files.size();

  // Map from image name to camera index
  std::map<std::string, int> image2cam;
  for (int i = 0; i < num_cams; i++)
    image2cam[opt.image_files[i]] = i;

  // Split the comma-separated list of group files
  std::vector<std::string> group_files;
  boost::split(group_files, opt.orbital_group_list, boost::is_any_of(","));

  groups.cam2group.assign(num_cams, -1);
  groups.num_groups = 0;
  for (size_t g = 0; g < group_files.size(); g++) {
    std::string group_file = group_files[g];
    boost::trim(group_file);
    if (group_file.empty())
      continue;

    std::vector<std::string> images;
    asp::read_list(group_file, images);
    int count = 0;
    for (size_t j = 0; j < images.size(); j++) {
      auto it = image2cam.find(images[j]);
      if (it == image2cam.end())
        vw::vw_throw(vw::ArgumentErr() << "Orbital group file '" << group_file
                     << "' has image '" << images[j]
                     << "' that is not among the input images.\n");
      int icam = it->second;
      if (groups.cam2group[icam] >= 0)
        vw::vw_throw(vw::ArgumentErr() << "Image '" << images[j]
                     << "' is listed in more than one orbital group.\n");
      groups.cam2group[icam] = groups.num_groups;
      count++;
    }
    if (count == 0)
      vw::vw_throw(vw::ArgumentErr() << "Orbital group file '" << group_file
                   << "' has no valid images.\n");
    groups.num_groups++;
  }

  // Original positions per camera, and the per-group centroid of those positions
  groups.init_pos.resize(num_cams);
  for (int i = 0; i < num_cams; i++)
    groups.init_pos[i] = orig_cams[i]->camera_center(vw::Vector2());

  groups.centroid.assign(groups.num_groups, vw::Vector3());
  std::vector<int> group_count(groups.num_groups, 0);
  for (int i = 0; i < num_cams; i++) {
    int g = groups.cam2group[i];
    if (g < 0)
      continue;
    groups.centroid[g] += groups.init_pos[i];
    group_count[g]++;
  }
  for (int g = 0; g < groups.num_groups; g++) {
    if (group_count[g] > 0)
      groups.centroid[g] /= double(group_count[g]);
  }

  // Initialize the group poses to identity (all zeros): the solve starts on the
  // trusted orbit, and each pose can then move the whole orbit rigidly.
  groups.group_pose.assign(groups.num_groups * 6, 0.0);

  vw::vw_out() << "Orbital groups: found " << groups.num_groups
               << " orbit(s). Camera positions in each will move as one rigid pose.\n";
}

// For cameras in an orbital group, fix the per-camera translation (components 0,1,2
// of the camera block); the rotation (3,4,5) stays free. Each grouped camera gets
// its own manifold so Ceres can own them without a double free.
void fixGroupedCameraTranslations(asp::OrbitalGroups const& groups,
                                  asp::BaState & ba_state,
                                  ceres::Problem & problem) {

  if (groups.num_groups <= 0)
    return;

  int num_cameras = ba_state.num_cameras();
  for (int icam = 0; icam < num_cameras; icam++) {
    if (!groups.grouped(icam))
      continue;
    double * cam_ptr = ba_state.get_camera_ptr(icam);
    if (!problem.HasParameterBlock(cam_ptr))
      continue; // no residual references it (e.g. no matches)
    std::vector<int> constant_translation = {0, 1, 2};
    ceres::SubsetManifold * trans_manifold
      = new ceres::SubsetManifold(asp::NUM_CAMERA_PARAMS, constant_translation);
    problem.SetManifold(cam_ptr, trans_manifold);
  }
}

// After the solve, write each grouped camera's derived position back into its
// translation adjustment, so all downstream code sees the correct positions.
void updateGroupedCameraPositions(asp::OrbitalGroups & groups,
                                  asp::BaState & ba_state) {

  if (groups.num_groups <= 0)
    return;

  int num_cameras = ba_state.num_cameras();
  for (int icam = 0; icam < num_cameras; icam++) {
    if (!groups.grouped(icam))
      continue;
    vw::Vector3 derived = applyOrbitalGroupPose(groups.pose_ptr(icam),
                                                groups.init_pos[icam],
                                                groups.centroid_of(icam));
    vw::Vector3 adj = derived - groups.init_pos[icam]; // translation adjustment
    double * cam_ptr = ba_state.get_camera_ptr(icam);
    cam_ptr[0] = adj[0];
    cam_ptr[1] = adj[1];
    cam_ptr[2] = adj[2];
  }
}

} // end namespace asp
