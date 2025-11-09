/* Copyright (c) 2021, United States Government, as represented by the
 * Administrator of the National Aeronautics and Space Administration.
 *
 * All rights reserved.
 *
 * The "ISAAC - Integrated System for Autonomous and Adaptive Caretaking
 * platform" software is licensed under the Apache License, Version 2.0
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

#ifndef INTERPOLATION_UTILS_H_
#define INTERPOLATION_UTILS_H_

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <vector>
#include <map>

namespace rig {

// Given two poses aff0 and aff1, and 0 <= alpha <= 1, do linear interpolation.
Eigen::Affine3d linearInterp(double alpha, Eigen::Affine3d const& aff0,
                               Eigen::Affine3d const& aff1);

// Given two poses aff0 and aff1, and t0 <= t <= t1, do linear interpolation.
Eigen::Affine3d linearInterp(double t0, double t, double t1, Eigen::Affine3d const& aff0,
                             Eigen::Affine3d const& aff1);
  
// Given a set of poses indexed by time, interpolate or extrapolate
// (within range of bracket_len) at a set of target timestamps. Go
// forward in time both in the input and the target, which makes the
// complexity linear rather than quadratic.
void interpOrExtrap(std::map<double, Eigen::Affine3d> const& input_poses,
                    std::map<double, std::string> const& target,
                    double bracket_len, bool nearest_neighbor,
                    // Outputs
                    std::vector<std::string> & found_images,
                    std::vector<Eigen::Affine3d> & found_poses);
  
}  // namespace rig

#endif  // INTERPOLATION_UTILS_H_
