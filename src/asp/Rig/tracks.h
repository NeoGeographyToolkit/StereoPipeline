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

#ifndef RIG_CALIBRATOR_TRACKS_H_
#define RIG_CALIBRATOR_TRACKS_H_

#include <set>
#include <vector>
#include <map>

// TODO(oalexan1): Move here all tracks logic from interest_point.cc and tensor.cc.

namespace aspOpenMVG {
  namespace matching {
    class PairWiseMatches;
  }
}

namespace rig {

typedef std::vector<std::vector<std::pair<float, float>>> KeypointVecT;
typedef std::vector<std::map<int, int>> TrackT;
class cameraImage;

void buildTracks(aspOpenMVG::matching::PairWiseMatches const& match_map,
                 std::vector<std::map<int, int>>& pid_to_cid_fid);

// Remove duplicate tracks. There can still be two tracks with one contained
// in the other or otherwise having shared elements. 
void rmDuplicateTracks(std::vector<std::map<int, int>> & pid_to_cid_fid);

// See tracks.cc for the doc
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
                         std::vector<rig::cameraImage>       & B_cams);
  
}  // namespace rig

#endif  // RIG_CALIBRATOR_TRACKS_H_
