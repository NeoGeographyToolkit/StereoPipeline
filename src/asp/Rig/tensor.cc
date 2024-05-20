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
// TODO(oalexan1): Most of this must be wiped.

// TODO(oalexan1): Move track logic to track.cc. This file must be
// further broken up into several files, for example, ba.cc,
// interest_point.cc, triangulation.cc, merge_maps.cc, etc.

#include <Rig/tensor.h>
#include <Rig/ransac.h>
#include <Rig/reprojection.h>
#include <Rig/sparse_mapping.h>
#include <Rig/essential.h>
#include <Rig/matching.h>
#include <Rig/basic_algs.h>
#include <Rig/thread.h>
#include <Rig/nvm.h>
#include <Rig/rig_config.h>
#include <Rig/camera_image.h>
#include <Rig/image_lookup.h>
#include <Rig/tracks.h>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <ceres/ceres.h>
#include <gflags/gflags.h>
#include <glog/logging.h>

// Get rid of warning beyond our control
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
#pragma GCC diagnostic ignored "-Wunused-function"
#pragma GCC diagnostic push
#include <OpenMVG/projection.hpp>
#include <OpenMVG/triangulation_nview.hpp>
#include <OpenMVG/numeric.h>
#include <OpenMVG/tracks.hpp>
#pragma GCC diagnostic pop

#include <opencv2/features2d/features2d.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <sys/stat.h>
#include <fstream>
#include <set>
#include <thread>
#include <vector>
#include <mutex>
#include <functional>
#include <cstdio>

namespace sparse_mapping {
// Two minor and local utility functions
std::string print_vec(double a) {
  char st[256];
  snprintf(st, sizeof(st), "%7.4f", a);
  return std::string(st);
}
std::string print_vec(Eigen::Vector3d a) {
  char st[256];
  snprintf(st, sizeof(st), "%7.4f %7.4f %7.4f", a[0], a[1], a[2]);
  return std::string(st);
}

// Extract a submap in-place.
void ExtractSubmap(std::vector<std::string> const& images_to_keep,
                   rig::nvmData & nvm) {

  // Sanity check. The images to keep must exist in the original map.
  std::map<std::string, int> image2cid;
  for (size_t cid = 0; cid < nvm.cid_to_filename.size(); cid++)
    image2cid[nvm.cid_to_filename[cid]] = cid;
  for (size_t cid = 0; cid < images_to_keep.size(); cid++) {
    if (image2cid.find(images_to_keep[cid]) == image2cid.end())
      std::cout << "Warning: Could not find in the input map the image: "
                << images_to_keep[cid] << "\n";
  }

  // To extract the submap-in place, it is simpler to reorder the images
  // to extract to be in the same order as in the map. Keep those in
  // local vector 'keep'.
  std::vector<std::string> keep;
  {
    std::set<std::string> keep_set;
    for (size_t cid = 0; cid < images_to_keep.size(); cid++)
      keep_set.insert(images_to_keep[cid]);
    for (size_t cid = 0; cid < nvm.cid_to_filename.size(); cid++) {
      if (keep_set.find(nvm.cid_to_filename[cid]) != keep_set.end())
        keep.push_back(nvm.cid_to_filename[cid]);
    }
  }

  // Map each image we keep to its index
  std::map<std::string, int> keep2cid;
  for (size_t cid = 0; cid < keep.size(); cid++)
    keep2cid[keep[cid]] = cid;

  // The map from the old cid to the new cid
  std::map<int, int> cid2cid;
  for (size_t cid = 0; cid < nvm.cid_to_filename.size(); cid++) {
    auto it = keep2cid.find(nvm.cid_to_filename[cid]);
    if (it == keep2cid.end()) continue;  // current image is not in the final submap
    cid2cid[cid] = it->second;
  }

  // Sanity checks. All the kept images must be represented in cid2cid,
  // and the values in cid2cid must be consecutive.
  if (cid2cid.size() != keep.size() || cid2cid.empty())
    LOG(FATAL) << "Cannot extract a submap. Check your inputs. Maybe some images "
               << "are duplicated or none are in the map.";
  for (auto it = cid2cid.begin(); it != cid2cid.end(); it++) {
    auto it2 = it; it2++;
    if (it2 == cid2cid.end()) continue;
    if (it->second + 1 != it2->second || cid2cid.begin()->second != 0 )
      LOG(FATAL) << "Cannot extract a submap. Check if the images "
                 << "you want to keep are in the same order as in the original map.";
  }

  // Over-write the data in-place. Should be safe with the checks done above.
  int num_cid = keep.size();
  for (size_t cid = 0; cid < nvm.cid_to_filename.size(); cid++) {
    if (cid2cid.find(cid) == cid2cid.end()) continue;
    size_t new_cid = cid2cid[cid];
    nvm.cid_to_filename[new_cid]     = nvm.cid_to_filename[cid];
    nvm.cid_to_keypoint_map[new_cid] = nvm.cid_to_keypoint_map[cid];
    nvm.cid_to_cam_t_global[new_cid] = nvm.cid_to_cam_t_global[cid];
  }
  nvm.cid_to_filename.resize(num_cid);
  nvm.cid_to_keypoint_map.resize(num_cid);
  nvm.cid_to_cam_t_global.resize(num_cid);

  // Create new pid_to_cid_fid and pid_to_xyz.
  std::vector<std::map<int, int>> pid_to_cid_fid;
  std::vector<Eigen::Vector3d> pid_to_xyz;
  for (size_t pid = 0; pid < nvm.pid_to_cid_fid.size(); pid++) {
    auto const& cid_fid = nvm.pid_to_cid_fid[pid];  // alias
    std::map<int, int> cid_fid2;
    for (auto it = cid_fid.begin(); it != cid_fid.end(); it++) {
      int cid = it->first;
      if (cid2cid.find(cid) == cid2cid.end()) continue;  // not an image we want to keep
      cid_fid2[cid2cid[cid]] = it->second; // fid does not change
    }
    if (cid_fid2.size() <= 1) continue;  // tracks must have size at least 2
    pid_to_cid_fid.push_back(cid_fid2);
    pid_to_xyz.push_back(nvm.pid_to_xyz[pid]);
  }
  nvm.pid_to_cid_fid = pid_to_cid_fid;
  nvm.pid_to_xyz = pid_to_xyz;

  // If the input nvm has optical centers, keep the subset for the submap.
  if (!nvm.optical_centers.empty()) {
    std::map<std::string, Eigen::Vector2d> optical_centers;
    for (size_t cid = 0; cid < images_to_keep.size(); cid++) {
      auto it = nvm.optical_centers.find(images_to_keep[cid]);
      if (it == nvm.optical_centers.end()) 
        LOG(FATAL) << "Cannot find the optical centers for the images in the submap.";
      optical_centers[images_to_keep[cid]] = it->second;
    }
    nvm.optical_centers = optical_centers; // overwrite in place
  }
   
  std::cout << "Number of images in the extracted map: " << nvm.cid_to_filename.size() << "\n";
  std::cout << "Number of tracks in the extracted map: " << nvm.pid_to_cid_fid.size() << "\n";

  return;
}

}  // namespace sparse_mapping
