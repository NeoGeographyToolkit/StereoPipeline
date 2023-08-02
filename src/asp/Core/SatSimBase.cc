// __BEGIN_LICENSE__
//  Copyright (c) 2009-2013, United States Government as represented by the
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

// Very low-level functions used in sat_sim.cc and jitter_solve.cc. 

#include <asp/Core/SatSimBase.h>
#include <vw/Cartography/GeoReferenceUtils.h>

using namespace vw::cartography;

namespace asp {

// A small number to help convert directions from being in projected space to
// ECEF (the transform between these is nonlinear). Do not use a small value,
// as in ECEF these will be large numbers and we may have precision issues.
// The value 0.01 was tested well. Measured in meters.
double satSimDelta() { 
return 0.01;
}

// Assemble the cam2world matrix from the along track, across track, and down vectors
// Note how we swap the first two columns and flip one sign. We went the along
// direction to be the camera y direction
void assembleCam2WorldMatrix(vw::Vector3 const& along, 
                             vw::Vector3 const& across, 
                             vw::Vector3 const& down,
                             // Output
                             vw::Matrix3x3 & cam2world) {

  for (int row = 0; row < 3; row++) {
    cam2world(row, 0) = along[row];
    cam2world(row, 1) = across[row];
    cam2world(row, 2) = down[row];
  }
 return;
}

// Return the matrix of rotation in the xy plane, from camera to satellite body
vw::Matrix3x3 rotationXY() {

  vw::Matrix3x3 T;
  // Set all elements to zero
  for (int row = 0; row < 3; row++)
    for (int col = 0; col < 3; col++)
      T(row, col) = 0.0;
  
  T(0, 1) = 1;
  T(1, 0) = -1;
  T(2, 2) = 1;

  return T;
}

// Given two end points in projected coordinates determining a satellite 
// trajectory, find the along and across vectors in projected coordinates.
void calcProjAlongAcross(vw::Vector3 const& first_proj,
                         vw::Vector3 const& last_proj,
                         vw::Vector3      & proj_along,
                         vw::Vector3      & proj_across) {

  proj_along = last_proj - first_proj;
  
  // Sanity check. 
  if (proj_along == vw::Vector3())
    vw::vw_throw(vw::ArgumentErr()
       << "The first and last camera positions are the same. It is not possible find the camera orientation. Specify at the very least two positions that are close but distinct.\n");

  // Normalize
  proj_along = proj_along / norm_2(proj_along);

  // One more sanity check
  if (std::max(std::abs(proj_along[0]), std::abs(proj_along[1])) < 1e-6)
    vw::vw_throw(vw::ArgumentErr()
      << "It appears that the satellite is aiming for the ground or "
      << "the orbital segment is too short. Correct the orbit end points.\n");

  // Find the across-track direction, parallel to the ground, in projected coords
  proj_across = vw::math::cross_prod(proj_along, vw::Vector3(0, 0, 1));
  proj_across = proj_across / norm_2(proj_across);
}

// Make these vectors have norm 1, and make across perpendicular to along
void normalizeOrthogonalizeAlongAcross(vw::Vector3 & along, vw::Vector3 & across) {
    
  // Normalize
  along = along / norm_2(along);
  across = across / norm_2(across);

  // Ensure that across is perpendicular to along
  across = across - dot_prod(along, across) * along;

  // Normalize again
  across = across / norm_2(across);
}

// Find normalized along and across directions in ECEF given these values in
// projected coordinates. Use centered difference to compute the along and
// across track points This achieves higher quality results.
void calcEcefAlongAcross(vw::cartography::GeoReference const& dem_georef,
                         double delta,
                         vw::Vector3 const& proj_along,
                         vw::Vector3 const& proj_across,
                         vw::Vector3 & proj_pt,
                         // Outputs
                         vw::Vector3 & along,
                         vw::Vector3 & across) {

  // Check if proj_along and proj_across are perpendicular and have norm 1
  double dot = dot_prod(proj_along, proj_across);
  if (std::abs(dot) > 1e-6) 
    vw::vw_throw(vw::ArgumentErr()
      << "calcEcefAlongAcross: proj_along and proj_across are not perpendicular.\n");
  if (std::abs(norm_2(proj_along)  - 1.0) > 1e-6 || 
      std::abs(norm_2(proj_across) - 1.0) > 1e-6) 
    vw::vw_throw(vw::ArgumentErr()
      << "calcEcefAlongAcross: either proj_along or proj_across does not have norm 1.\n");

  vw::Vector3 L1 = proj_pt - delta * proj_along; // along track point
  vw::Vector3 C1 = proj_pt - delta * proj_across; // across track point
  vw::Vector3 L2 = proj_pt + delta * proj_along; // along track point
  vw::Vector3 C2 = proj_pt + delta * proj_across; // across track point

  // Convert to cartesian
  L1 = vw::cartography::projToEcef(dem_georef, L1);
  C1 = vw::cartography::projToEcef(dem_georef, C1);
  L2 = vw::cartography::projToEcef(dem_georef, L2);
  C2 = vw::cartography::projToEcef(dem_georef, C2);

  // Create the along track and across track vectors
  along  = L2 - L1;
  across = C2 - C1;

  // Make these vector have norm 1, and make across perpendicular to along
  asp::normalizeOrthogonalizeAlongAcross(along, across);
}

} // end namespace asp
