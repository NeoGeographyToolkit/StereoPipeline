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

/// \file SatSimTrajectory.h
/// Synthetic trajectory generation and orbit/attitude calculation for sat_sim.

#ifndef __ASP_CAMERA_SATSIM_TRAJECTORY_H__
#define __ASP_CAMERA_SATSIM_TRAJECTORY_H__

#include <asp/Camera/SatSim.h>

#include <vw/Cartography/GeoReference.h>
#include <vw/Image/ImageViewRef.h>
#include <vw/Image/PixelMask.h>
#include <vw/Math/Vector.h>
#include <vw/Math/Matrix.h>

#include <vector>
#include <string>

namespace rig {
  class RigSet;
}

namespace asp {

// Calculate orbit length between two projected points
double calcOrbitLength(vw::Vector3 const& first_proj, vw::Vector3 const& last_proj,
                       vw::GeoReference const& georef);

// Calculate jitter amplitude at a given projected point
vw::Vector3 calcJitterAmplitude(SatSimOptions const& opt,
                                vw::Vector3 const& curr_proj,
                                double orbit_len);

// Adjust sensor orientations for sensor ground offsets and rotation angles
void adjustRigForOffsets(SatSimOptions const& opt,
                         vw::GeoReference const& dem_georef,
                         std::vector<vw::Vector3> const& positions,
                         std::vector<vw::Matrix3x3> const& cam2world,
                         rig::RigSet & rig);

// Generate camera poses along the satellite trajectory
void genCamPoses(SatSimOptions & opt,
                 vw::GeoReference const& dem_georef,
                 vw::ImageViewRef<vw::PixelMask<float>> dem,
                 double height_guess,
                 bool have_rig,
                 // Outputs
                 int                        & first_pos,
                 double                     & first_line_time,
                 double                     & orbit_len,
                 std::vector<vw::Vector3>   & positions,
                 std::vector<vw::Matrix3x3> & cam2world,
                 std::vector<vw::Matrix3x3> & cam2world_no_jitter,
                 std::vector<vw::Matrix3x3> & ref_cam2world,
                 std::vector<double>        & cam_times,
                 rig::RigSet                & rig);

} // namespace asp

#endif // __ASP_CAMERA_SATSIM_TRAJECTORY_H__
