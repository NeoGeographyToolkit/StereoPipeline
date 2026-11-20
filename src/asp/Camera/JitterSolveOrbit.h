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

// Orbit constraint for jitter_solve. The position samples of all linescan cameras
// in an orbital group are pulled toward one smooth curve, modeled in ECEF as a
// polynomial of the along-track coordinate, giving the across-track and vertical
// offsets.

#ifndef __ASP_CAMERA_JITTER_SOLVE_ORBIT_H__
#define __ASP_CAMERA_JITTER_SOLVE_ORBIT_H__

#include <vw/Math/Vector.h>

#include <string>
#include <vector>

namespace ceres {
  class Problem;
  class CostFunction;
}

namespace asp {

class CsmModel;

// One orbital group. The frame has fixed ECEF axes: along-track, across-track,
// and up, with the origin at the mean of the position samples. Units are meters.
struct OrbitGroup {
  std::vector<int> cams;          // camera indices in this group
  vw::Vector3 origin;             // mean of the samples, ECEF
  vw::Vector3 along, across, up;  // unit vectors, ECEF
  double half_len = 0.0;          // max abs along-track coordinate, for scaling
  int degree = 1;                 // polynomial degree in the along-track coordinate
  // Polynomial coefficients, first degree+1 for the across-track offset, then
  // degree+1 for the height. These are optimized.
  std::vector<double> coeffs;
};

// Map an ECEF point to the local orbit frame (along, across, height).
vw::Vector3 orbitFrameCoords(OrbitGroup const& group, vw::Vector3 const& ecef);

// Evaluate the across-track offset and height of the orbit curve at the given
// along-track coordinate.
vw::Vector2 orbitCurve(OrbitGroup const& group, double along);

// Build the orbital groups from --orbital-group-list (comma-separated files, each
// listing the images of one group), and fit the initial curve to the current
// position samples.
void buildOrbitGroups(std::string const& orbital_group_list,
                      int degree,
                      std::vector<std::string> const& image_files,
                      std::vector<asp::CsmModel*> const& csm_models,
                      // Outputs
                      std::vector<OrbitGroup> & groups);

// The cost function pulling one position sample to the orbit curve. Its
// parameter blocks are the sample position (ECEF) and the curve coefficients.
ceres::CostFunction * createOrbitDevError(OrbitGroup const& group,
                                          vw::Vector3 const& init_ecef,
                                          vw::Vector2 const& uncertainty,
                                          double power);

// Add a residual per position sample that pulls it to its group's curve. The
// horizontal and vertical uncertainties are in meters (1 sigma).
void addOrbitConstraint(std::vector<asp::CsmModel*> const& csm_models,
                        vw::Vector2 const& uncertainty, double power,
                        // Outputs
                        std::vector<OrbitGroup> & groups,
                        std::vector<double> & weight_per_residual,
                        ceres::Problem & problem);

// Per position sample, the across-track and vertical deviation from the curve.
struct OrbitSampleDev {
  int cam = -1, sample = -1;
  double along = 0.0, across_dev = 0.0, vert_dev = 0.0;
};

// Compute the deviations of the current position samples from the current curve
void orbitDeviations(std::vector<OrbitGroup> const& groups,
                     std::vector<asp::CsmModel*> const& csm_models,
                     // Outputs
                     std::vector<std::vector<OrbitSampleDev>> & devs);

// Print a summary of the deviations, per group
void printOrbitDeviations(std::string const& tag,
                          std::vector<std::vector<OrbitSampleDev>> const& devs);

// Write the deviations before and after the solve, per sample and per camera
void saveOrbitDeviations(std::string const& out_prefix,
                         std::vector<std::string> const& image_files,
                         std::vector<OrbitGroup> const& groups,
                         std::vector<std::vector<OrbitSampleDev>> const& init_devs,
                         std::vector<std::vector<OrbitSampleDev>> const& final_devs);

} // end namespace asp

#endif // __ASP_CAMERA_JITTER_SOLVE_ORBIT_H__
