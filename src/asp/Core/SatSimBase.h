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

/// \file SatSimBase.h

// Very low-level functions used in sat_sim.cc and jitter_solve.cc. 

#ifndef __ASP_CORE_SATSIM_BASE_H__
#define __ASP_CORE_SATSIM_BASE_H__

#include <vw/Cartography/GeoReference.h>

namespace asp {

  // A small number to help convert directions from being in projected space to
  // ECEF (the transform between these is nonlinear). Do not use a small value,
  // as in ECEF these will be large numbers and we may have precision issues.
  // The value 0.01 was tested well. Measured in meters.
  double satSimDelta();

  // Assemble the cam2world matrix from the along track, across track, and down vectors
  // Note how we swap the first two columns and flip one sign. We went the along
  // direction to be the camera y direction
  void assembleCam2WorldMatrix(vw::Vector3 const& along, 
                              vw::Vector3 const& across, 
                              vw::Vector3 const& down,
                              // Output
                              vw::Matrix3x3 & cam2world);

  // Return the matrix of rotation in the xy plane, from camera to satellite body
  vw::Matrix3x3 rotationXY();

// Make these vectors have norm 1, and make across perpendicular to along
void normalizeOrthogonalizeAlongAcross(vw::Vector3 & along, vw::Vector3 & across);

// Given two end points in projected coordinates determining a satellite 
// trajectory, find the along and across vectors in projected coordinates.
void calcProjAlongAcross(vw::Vector3 const& first_proj,
                         vw::Vector3 const& last_proj,
                         vw::Vector3      & proj_along,
                         vw::Vector3      & proj_across);

// Find normalized along and across directions in ECEF given these values in
// projected coordinates. Use centered difference to compute the along and
// across track points This achieves higher quality results.
void calcEcefAlongAcross(vw::cartography::GeoReference const& dem_georef,
                         double delta,
                         vw::Vector3 const& proj_along,
                         vw::Vector3 const& proj_across,
                         vw::Vector3 const& proj_pt,
                         // Outputs
                         vw::Vector3 & along,
                         vw::Vector3 & across);

} // end namespace asp

#endif//__ASP_CORE_SATSIM_BASE_H__
