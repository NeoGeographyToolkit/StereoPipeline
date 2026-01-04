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
#ifndef ASP_RIG_TYPE_DEFS_H
#define ASP_RIG_TYPE_DEFS_H

#include <map>
#include <vector>
#include <Eigen/Core>
#include <Eigen/Geometry>

// A small header to hold type definitions for the rig calibrator
namespace rig {

  // Vector of keypoints for each image
  typedef std::vector<std::vector<std::pair<float, float>>> KeypointVec;  

  // Maps Point ID -> Camera ID -> Feature ID
  typedef std::vector<std::map<int, int>> PidCidFid; 

  // PidCidFid with extra info
  typedef std::vector<std::map<int, std::map<int, int>>> PidCidFidMap;
  
  // Maps Point ID -> Camera ID -> Feature ID -> 3D Mesh Intersection Point
  typedef std::vector<std::map<int, std::map<int, Eigen::Vector3d>>> PidCidFidToMeshXyz;

  // Vector of Eigen Matrices holding keypoints (NVM format)
  typedef std::vector<Eigen::Matrix2Xd> CidToKeypointMatVec;

  // Maps Image ID pair -> List of convergence angles
  typedef std::map<std::pair<int, int>, std::vector<double>> PairwiseConvergenceAngles;

}  // namespace rig

#endif  // ASP_RIG_TYPE_DEFS_H

