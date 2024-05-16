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

// TODO(oalexan1): Move here track logic from tensor.cc and interest_point.cc.

#include <Rig/camera_image.h>
#include <Rig/tracks.h>

// Get rid of warning beyond our control
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
#pragma GCC diagnostic ignored "-Wunused-function"
#pragma GCC diagnostic push
#include <OpenMVG/projection.hpp>
#include <OpenMVG/triangulation_nview.hpp>
#include <OpenMVG/numeric.h>
#include <OpenMVG/tracks.hpp>
#pragma GCC diagnostic pop

#include <set>
#include <vector>
#include <map>

namespace rig {
  
// Given tracks in a map C that has images from one map A followed by
// images from second map named B, split the tracks that have at least
// two features in each map into tracks for the two maps. For the
// second one, need to shift the cid to start from 0 rather than from
// end of first map. The resulting vectors of tracks must be in
// one-to-one correspondence.
void splitTracksOneToOne(// Inputs
                         int num_acid, // number of images in map A
                         TrackT                              const & C_pid_to_cid_fid, 
                         KeypointVecT                        const & C_keypoint_vec, 
                         std::vector<rig::cameraImage> const & C_cams,
                         // Outputs
                         TrackT                                    & A_pid_to_cid_fid, 
                         TrackT                                    & B_pid_to_cid_fid, 
                         KeypointVecT                              & A_keypoint_vec, 
                         KeypointVecT                              & B_keypoint_vec, 
                         std::vector<rig::cameraImage>       & A_cams, 
                         std::vector<rig::cameraImage>       & B_cams) {
  
  // Wipe the outputs
  A_pid_to_cid_fid.clear();
  B_pid_to_cid_fid.clear();
  A_keypoint_vec.clear();
  B_keypoint_vec.clear();
  A_cams.clear();
  B_cams.clear();
  
  for (size_t pid = 0; pid < C_pid_to_cid_fid.size(); pid++) {

    auto & cid_fid = C_pid_to_cid_fid[pid];
    std::map<int, int> A_cid_fid, B_cid_fid;
    
    for (auto map_it = cid_fid.begin(); map_it != cid_fid.end(); map_it++) {
      int cid = map_it->first;
      int fid = map_it->second;
      if (cid < num_acid) 
        A_cid_fid[cid] = fid; // belongs to A
      else
        B_cid_fid[cid - num_acid] = fid; // belongs to B
    }
    
    if (A_cid_fid.size() > 1 && B_cid_fid.size() > 1) {
      // This is a shared track, that we break in two. Each obtained track
      // must have at least two images.
      A_pid_to_cid_fid.push_back(A_cid_fid);
      B_pid_to_cid_fid.push_back(B_cid_fid);
    }
  }
  
  // Break up the keypoint vec and the images
  for (size_t cid = 0; cid < C_keypoint_vec.size(); cid++) {
    if (cid < num_acid) {
      A_keypoint_vec.push_back(C_keypoint_vec[cid]);
      A_cams.push_back(C_cams[cid]);
    } else {
      B_keypoint_vec.push_back(C_keypoint_vec[cid]);
      B_cams.push_back(C_cams[cid]);
    }
  }

  return;
}

} // end namespace rig
