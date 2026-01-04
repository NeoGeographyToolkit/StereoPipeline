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

#include <asp/Rig/nvmUtils.h>
#include <asp/Rig/system_utils.h>
#include <asp/Rig/basic_algs.h>
#include <asp/Rig/triangulation.h>
#include <asp/Rig/RigTypeDefs.h>
#include <asp/OpenMVG/tracks.hpp>

#include <glog/logging.h>

#include <iostream>
#include <string>
#include <set>

namespace rig {

// The nvm file produced by Theia can have files in arbitrary order. Find the map
// which will help bring the cid in the correct order.
void findCidReorderMap(asp::nvmData const& nvm,
                       std::vector<rig::cameraImage> const& cams,
                       // output
                       std::map<int, int> & nvm_cid_to_cams_cid) {
  
  // Wipe the output
  nvm_cid_to_cams_cid.clear();
                         
  // First find how to map each cid from nvm to cid in 'cams'.
  std::map<std::string, int> nvm_image_name_to_cid;
  for (size_t nvm_cid = 0; nvm_cid < nvm.cid_to_filename.size(); nvm_cid++) {
    nvm_image_name_to_cid[nvm.cid_to_filename[nvm_cid]] = nvm_cid;
  }
  
  std::set<std::string> cam_set;
  for (size_t cid = 0; cid < cams.size(); cid++) {
    std::string const& image_name = cams[cid].image_name;
    cam_set.insert(image_name);

    auto nvm_it = nvm_image_name_to_cid.find(image_name);
    if (nvm_it == nvm_image_name_to_cid.end()) 
      LOG(FATAL) << "Could not look up image: " << image_name
                 << " in the input nvm file. Likely the input reconstruction "
                 << "is incomplete.\n";
    int nvm_cid = nvm_it->second;
    nvm_cid_to_cams_cid[nvm_cid] = cid;
  }

  // This is an important sanity check. Warn the user when not all images
  // from the NVM file are among the ones being used.
  if (cams.size() < nvm.cid_to_filename.size()) {
    std::cout << "Warning: Some input images are not present among the images being used. "
              << "Perhaps they were removed during bracketing. Only images bracketed in time "
              << "by reference sensor images can be used. Excluded images:\n";
    for (size_t nvm_it = 0; nvm_it < nvm.cid_to_filename.size(); nvm_it++) {
      if (cam_set.find(nvm.cid_to_filename[nvm_it]) == cam_set.end()) 
        std::cout << nvm.cid_to_filename[nvm_it] << std::endl;
    }
  }
  
  return;
}

// For nvm data that has the keypoints shifted relative to the optical
// center, undo this shift when 'undo_shift' is true. So, add the optical center.
// When 'undo_shift' is false, subtract the optical center.
void shiftKeypoints(bool undo_shift, rig::RigSet const& R,
                    asp::nvmData & nvm) { // output

  for (size_t cid = 0; cid < nvm.cid_to_filename.size(); cid++) {
    std::string const& image_name = nvm.cid_to_filename[cid]; // alias

    auto it = nvm.optical_centers.find(image_name);
    if (it == nvm.optical_centers.end()) 
      LOG(FATAL) << "Could not find optical center for image: " << image_name << ".\n";
    Eigen::Vector2d keypoint_offset = it->second;
    
    int num_fid = nvm.cid_to_keypoint_map[cid].cols();
    for (int fid = 0; fid < num_fid; fid++) {
      if (undo_shift) 
        nvm.cid_to_keypoint_map.at(cid).col(fid) += keypoint_offset;
      else
        nvm.cid_to_keypoint_map.at(cid).col(fid) -= keypoint_offset;
    }
  }

  return;
}
  
// Transform nvm matches. Account for the fact that the nvm file will
// likely have the images in different order than in the 'cams'
// vector, and may have more such images, as later we may have used
// bracketing to thin them out. Also many need to add a keypoint
// offset.
// TODO(oalexan1): Integrate this with transformAppendNvm().
void transformNvm(// Inputs
                  std::vector<rig::cameraImage>   const& cams,
                  std::vector<Eigen::Vector2d>          const& keypoint_offsets,
                  asp::nvmData                          const& nvm,
                  // Outputs
                  rig::PidToCidFidVec & pid_to_cid_fid,
                  rig::KeypointVec & keypoint_vec,
                  std::vector<Eigen::Vector3d> & xyz_vec) {

  // Sanity checks
  if (!keypoint_vec.empty() && keypoint_vec.size() != cams.size()) 
    LOG(FATAL) << "There must be as many sets of keypoints as images, or none at all.\n";
  if (nvm.pid_to_cid_fid.size() != nvm.pid_to_xyz.size()) 
    LOG(FATAL) << "There must be as many tracks as triangulated points for them.\n";
  
  // Wipe the outputs
  pid_to_cid_fid.clear();
  keypoint_vec.clear();
  keypoint_vec.resize(cams.size());
  xyz_vec.clear();

  // Find how to map each cid from nvm to cid in 'cams'.
  std::map<int, int> nvm_cid_to_cams_cid;
  rig::findCidReorderMap(nvm, cams,
                               nvm_cid_to_cams_cid); // output
  
  // Get new pid_to_cid_fid and keypoint_vec. Note that we ignore the triangulated
  // points in nvm.pid_to_xyz. Triangulation will be redone later.
  for (size_t pid = 0; pid < nvm.pid_to_cid_fid.size(); pid++) {

    // Ignore triangulated points that are NaN, Inf, or (0, 0, 0).
    if (!rig::isGoodTri(nvm.pid_to_xyz[pid])) 
      continue;
      
    std::map<int, int> out_cid_fid;
    for (auto cid_fid = nvm.pid_to_cid_fid[pid].begin();
         cid_fid != nvm.pid_to_cid_fid[pid].end(); cid_fid++) {

      int nvm_cid = cid_fid->first;
      int nvm_fid = cid_fid->second;
      Eigen::Vector2d keypoint = nvm.cid_to_keypoint_map.at(nvm_cid).col(nvm_fid);

      // Find cid value in 'cams' based on cid value in the nvm
      auto it = nvm_cid_to_cams_cid.find(nvm_cid);
      if (it == nvm_cid_to_cams_cid.end()) 
        continue; // this image may not have been within brackets and was thrown out
      int cid = it->second;
      
      // Add the optical center shift, if needed
      keypoint += keypoint_offsets[cid];

      int fid = keypoint_vec[cid].size(); // this is before we add the keypoint
      out_cid_fid[cid] = fid;
      // After the push back, size is fid + 1
      keypoint_vec[cid].push_back(std::make_pair(keypoint[0], keypoint[1])); 
    }

    // Keep only the tracks with at least two matches, and corresponding xyz
    if (out_cid_fid.size() > 1) {
      pid_to_cid_fid.push_back(out_cid_fid);
      xyz_vec.push_back(nvm.pid_to_xyz[pid]);
    }
    
  } // end iterating over nvm pid
}

// Helper function to find a keypoint for given iterator and update cid in the merged
// map with repetitions removed (via cid2cid).
bool updateCidFindKeypoint(std::map<int, int>::const_iterator map_it,
                           rig::CidToKeypointMatVec const& cid_to_keypoint_map,
                           std::map<int, int>            const& cid2cid,
                           std::vector<Eigen::Vector2d>  const& keypoint_offsets,
                           int cid_shift,
                           // outputs
                           int                                & cid, 
                           std::pair<float, float>            & K) {

  // Start with the cid in the input map
  cid = map_it->first;
  int fid = map_it->second;
  Eigen::Vector2d const& P = cid_to_keypoint_map.at(cid).col(fid); // alias
  K = std::pair<float, float>(P[0], P[1]); // the keypoint
  
  // Let the cid value point to the merged map, before removing repetitions
  cid += cid_shift; // only for map B that gets appended, which starts later

  // Add the keypoint offset, using the updated cid
  K.first  += keypoint_offsets.at(cid)[0];
  K.second += keypoint_offsets.at(cid)[1];
  
  // cid gets replaced by cid2cid[cid], the value after removing repetitions
  auto it = cid2cid.find(cid);
  if (it == cid2cid.end())
    return false; // cid2cid is missing this keypoint, just ignore it

  cid = it->second;

  return true;
}

// Given some tracks read from nvm from disk, append the ones from
// nvm. Some remapping is needed.  given that 'fid' values already
// exist for the given tracks and that the nvm read from disk
// may have the images in different order. New keypoints are recorded
// with the help of fid_count and merged_keypoint_map.
// Note that keypoint_offsets are applied before the cid2cid transform gets used!
// This is very error-prone!
// TODO(oalexan1): cid_shift and keypoint_offsets should be applied outside
// this function as they make it hard to understand.
void transformAppendNvm(// Append from these
                        rig::PidToCidFidVec  const& nvm_pid_to_cid_fid,
                        rig::CidToKeypointMatVec    const& nvm_cid_to_keypoint_map,
                        std::map<int, int>               const& cid2cid,
                        std::vector<Eigen::Vector2d>     const& keypoint_offsets,
                        int cid_shift,
                        size_t num_out_cams,
                        // Outputs, append to these 
                        std::vector<int> & fid_count,
                        std::vector<std::map<std::pair<float, float>, int>>
                        & merged_keypoint_map,
                        rig::PidToCidFidVec & pid_to_cid_fid) {

  // Sanity checks
  if (num_out_cams != fid_count.size()) 
    LOG(FATAL) << "Keypoint count was not initialized correctly.\n";
  if (num_out_cams != merged_keypoint_map.size()) 
    LOG(FATAL) << "Keypoint map was not initialized correctly.\n";
  if (num_out_cams < rig::maxMapVal(cid2cid) + 1)
    LOG(FATAL) << "Unexpected value for the size of the output map.\n";

  for (size_t pid = 0; pid < nvm_pid_to_cid_fid.size(); pid++) {

    std::map<int, int> out_cid_fid;
    auto const& cid_fid = nvm_pid_to_cid_fid[pid]; // alias
    for (auto map_it = cid_fid.begin(); map_it != cid_fid.end(); map_it++) {

      int cid = -1; // will change soon
      std::pair<float, float> K;
      bool ans = updateCidFindKeypoint(map_it, nvm_cid_to_keypoint_map, cid2cid,
                                       keypoint_offsets, cid_shift,  
                                       cid, K);
      if (!ans) 
        continue; // could not lookup cid in cid2cid
      
      // TODO(oalexan1): Use below the function findFid(), with testing.
      // Insert K in the keypoint map and increment the count,
      // unless it already exists. In either case get the fid.
      auto key_it = merged_keypoint_map.at(cid).find(K);
      int fid = -1;
      if (key_it != merged_keypoint_map.at(cid).end()) {
        fid = key_it->second;
      } else {
        // fid is the last count, then increment the count
        fid = fid_count[cid];
        merged_keypoint_map.at(cid)[K] = fid_count[cid];
        fid_count[cid]++;
      }
      
      // Create the track
      out_cid_fid[cid] = fid;
    }
    
    // Append the transformed track
    if (out_cid_fid.size() > 1)
      pid_to_cid_fid.push_back(out_cid_fid);
  }
  
  return;
}

} // end namespace rig
