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

/// \file BundleAdjustOrbital.h
///

// Orbital grouping for bundle_adjust. Tie the frame cameras of one orbit to a
// single shared 6-DOF rigid pose, so their positions move as one rigid
// trajectory rather than independently. See the --orbital-group-list option
// for the user-facing description.

#ifndef __ASP_CAMERA_BUNDLE_ADJUST_ORBITAL_H__
#define __ASP_CAMERA_BUNDLE_ADJUST_ORBITAL_H__

#include <vw/Camera/CameraModel.h>
#include <vw/Math/Vector.h>

#include <vector>

// Forward declaration
namespace ceres { class Problem; }

namespace asp {

struct BaOptions;
struct BaParams;

// Apply an orbital group rigid pose to a camera's original position. The pose is a
// 6-element array [axis_angle(3), translation(3)]. The rotation is applied about the
// group centroid (for conditioning), then the translation is added:
//   pos = centroid + R(axis_angle) * (orig_position - centroid) + translation.
// At pose = all zeros this returns orig_position (identity), so the solve starts on
// the trusted orbit.
vw::Vector3 applyOrbitalGroupPose(double const* pose,
                                  vw::Vector3 const& orig_position,
                                  vw::Vector3 const& centroid);

// Structure describing the orbital grouping of the cameras. Several framelet frame
// cameras acquired along one orbit are tied together and their positions are all
// derived from a single shared 6-DOF rigid pose (that group's entry in group_pose).
// Orientations stay per-camera free (not touched here). This replaces N free framelet
// positions with one rigid trajectory per orbit.
struct OrbitalGroups {
  std::vector<int> cam2group;          // per camera, group id, or -1 if ungrouped
  int num_groups;                      // number of orbital groups
  std::vector<double> group_pose;      // num_groups*6, [axis_angle(3), t(3)], optimized
  std::vector<vw::Vector3> init_pos;   // per camera, original (initial) position, constant
  std::vector<vw::Vector3> centroid;   // per group, centroid of that group's positions

  OrbitalGroups(): num_groups(0) {}

  // Is this camera part of an orbital group
  bool grouped(int icam) const {
    return num_groups > 0 && !cam2group.empty() && cam2group[icam] >= 0;
  }
  // Pointer to this camera's group pose block (the shared, optimized 6-DOF)
  double* pose_ptr(int icam) {
    return &group_pose[cam2group[icam] * 6];
  }
  // The centroid of the group this camera belongs to
  vw::Vector3 centroid_of(int icam) const {
    return centroid[cam2group[icam]];
  }
};

// Build the orbital groups from --orbital-group-list. See that option for more info.
void buildOrbitalGroups(asp::BaOptions const& opt,
                        std::vector<vw::CamPtr> const& orig_cams,
                        asp::OrbitalGroups & groups);

// For cameras in an orbital group, the position is derived from the shared group
// pose, so fix the per-camera translation (components 0,1,2 of the camera block);
// the rotation (3,4,5) stays free. Call after residuals are added, before the solve.
void fixGroupedCameraTranslations(asp::OrbitalGroups const& groups,
                                  asp::BaParams & param_storage,
                                  ceres::Problem & problem);

// After the solve, the optimized position lives in the shared group pose (the
// per-camera translation was fixed). Write the derived position back into each
// camera's translation adjustment, so all downstream code (final residuals, camera
// writing) sees the correct positions with no further changes.
void updateGroupedCameraPositions(asp::OrbitalGroups & groups,
                                  asp::BaParams & param_storage);

} // end namespace asp

#endif // __ASP_CAMERA_BUNDLE_ADJUST_ORBITAL_H__
