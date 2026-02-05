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

// TODO(oalexan1): Move here track logic from interest_point.cc.

#include <asp/Rig/CameraImage.h>
#include <asp/Rig/tracks.h>
#include <asp/Rig/RigTypeDefs.h>

// Get rid of warning beyond our control
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
#pragma GCC diagnostic ignored "-Wunused-function"
#pragma GCC diagnostic push
#include <OpenMVG/numeric.h>
#include <OpenMVG/projection.hpp>
#include <OpenMVG/triangulation_nview.hpp>
#include <asp/OpenMVG/tracks.hpp>
#pragma GCC diagnostic pop

#include <glog/logging.h>

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
                         rig::PidCidFid                const& C_pid_to_cid_fid, 
                         rig::KeypointVec              const& C_keypoint_vec, 
                         std::vector<rig::cameraImage> const& C_cams,
                         // Outputs
                         rig::PidCidFid                     & A_pid_to_cid_fid, 
                         rig::PidCidFid                     & B_pid_to_cid_fid, 
                         rig::KeypointVec                   & A_keypoint_vec, 
                         rig::KeypointVec                   & B_keypoint_vec, 
                         std::vector<rig::cameraImage>      & A_cams, 
                         std::vector<rig::cameraImage>      & B_cams) {
  
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

// Build tracks from pairs  
void buildTracks(aspOpenMVG::matching::PairWiseMatches const& match_map,
                 rig::PidCidFid                             & pid_to_cid_fid) { // output

  pid_to_cid_fid.clear(); // wipe the output
  
  aspOpenMVG::tracks::TracksBuilder trackBuilder;
  trackBuilder.Build(match_map);  // Build:  Efficient fusion of correspondences
  trackBuilder.Filter();          // Filter: Remove tracks that have conflict
  // trackBuilder.ExportToStream(std::cout);
  // Export tracks as a map (each entry is a sequence of imageId and featureIndex):
  //  {TrackIndex => {(imageIndex, featureIndex), ... ,(imageIndex, featureIndex)}
  aspOpenMVG::tracks::STLMAPTracks map_tracks;
  trackBuilder.ExportToSTL(map_tracks);
  trackBuilder = aspOpenMVG::tracks::TracksBuilder();   // wipe it

  if (map_tracks.empty())
    LOG(FATAL) << "No tracks left after filtering. Perhaps images "
               << "are too dis-similar?\n";
  
  // Populate the filtered tracks
  size_t num_elems = map_tracks.size();
  pid_to_cid_fid.clear();
  pid_to_cid_fid.resize(num_elems);
  size_t curr_id = 0;
  for (auto itr = map_tracks.begin(); itr != map_tracks.end(); itr++) {
    for (auto itr2 = (itr->second).begin(); itr2 != (itr->second).end(); itr2++) {
      pid_to_cid_fid[curr_id][itr2->first] = itr2->second;
    }
    curr_id++;
  }

  return;
}

// Remove duplicate tracks. There can still be two tracks with one contained
// in the other or otherwise having shared elements. 
void rmDuplicateTracks(rig::PidCidFid & pid_to_cid_fid) {

  int num_tracks = pid_to_cid_fid.size();
  // vector to set, and back
  std::set<std::map<int, int>> track_set(pid_to_cid_fid.begin(), pid_to_cid_fid.end());
  pid_to_cid_fid.assign(track_set.begin(), track_set.end());

  int diff = num_tracks - int(pid_to_cid_fid.size());
  std::cout << "Removed " << diff << " duplicate tracks ("
            << 100.0 * double(diff)/double(num_tracks) << "%)\n";
}

} // end namespace rig
