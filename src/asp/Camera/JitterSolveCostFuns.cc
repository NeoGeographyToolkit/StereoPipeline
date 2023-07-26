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

// Cost functions used in solving for jitter. These need access to the camera models,
// so they are stored in the Camera folder.

#include <asp/Camera/JitterSolveCostFuns.h>
#include <asp/Core/CameraTransforms.h>
#include <asp/Core/SatSimBase.h>

#include <vw/Cartography/GeoReferenceUtils.h>

namespace asp {

// Call to work with ceres::DynamicCostFunction.
bool LsPixelReprojErr::operator()(double const * const * parameters, 
                                  double * residuals) const {

  try {
    // Make a copy of the model, as we will update quaternion and position values
    // that are being modified now. This may be expensive.
    UsgsAstroLsSensorModel cam = *m_ls_model;

    // Update the relevant quaternions in the local copy
    int shift = 0;
    for (int qi = m_begQuatIndex; qi < m_endQuatIndex; qi++) {
      for (int coord = 0; coord < NUM_QUAT_PARAMS; coord++) {
        cam.m_quaternions[NUM_QUAT_PARAMS * qi + coord]
          = parameters[qi + shift - m_begQuatIndex][coord];
      }
    }

    // Same for the positions. Note how we move forward in the parameters array,
    // as this is after the quaternions
    shift += (m_endQuatIndex - m_begQuatIndex);
    for (int pi = m_begPosIndex; pi < m_endPosIndex; pi++) {
      for (int coord = 0; coord < NUM_XYZ_PARAMS; coord++) {
        cam.m_positions[NUM_XYZ_PARAMS * pi + coord]
          = parameters[pi + shift - m_begPosIndex][coord];
      }
    }

    // Move forward in the array of parameters, then recover the triangulated point
    shift += (m_endPosIndex - m_begPosIndex);
    csm::EcefCoord P;
    P.x = parameters[shift][0];
    P.y = parameters[shift][1];
    P.z = parameters[shift][2];

    // Project in the camera with high precision. Do not use here
    // anything lower than 1e-8, as the linescan model will then
    // return junk.
    double desired_precision = asp::DEFAULT_CSM_DESIRED_PRECISISON;
    csm::ImageCoord imagePt = cam.groundToImage(P, desired_precision);

    // Convert to what ASP expects
    vw::Vector2 pix;
    asp::fromCsmPixel(pix, imagePt);

    residuals[0] = m_weight*(pix[0] - m_observation[0]);
    residuals[1] = m_weight*(pix[1] - m_observation[1]);
    
  } catch (std::exception const& e) {
    residuals[0] = g_big_pixel_value;
    residuals[1] = g_big_pixel_value;
    return true; // accept the solution anyway
  }

  return true;
}

// Constructor for weightedRollYawError. See the .h file for the documentation.
weightedRollYawError::weightedRollYawError
                  (std::vector<double>       const& positions, 
                   std::vector<double>           const& quaternions,
                   vw::cartography::GeoReference const& georef,
                   int cur_pos, double rollWeight, double yawWeight,
                   bool initial_camera_constraint): 
                   m_rollWeight(rollWeight), m_yawWeight(yawWeight), 
                   m_initial_camera_constraint(initial_camera_constraint) {

    int num_pos = positions.size()/NUM_XYZ_PARAMS;
    int num_quat = quaternions.size()/NUM_QUAT_PARAMS;
    if (num_pos != num_quat)
      vw::vw_throw(vw::ArgumentErr() 
        << "weightedRollYawError: Expecting the same number of positions and quaternions.\n");
    if (cur_pos < 0 || cur_pos >= num_pos)
      vw::vw_throw(vw::ArgumentErr() 
        << "weightedRollYawError: Expecting position index in range.\n");

    // Find the nearest neighbors of the current position
    int beg_pos = std::max(0, cur_pos - 1);
    int end_pos = std::min(num_pos - 1, cur_pos + 1);
    if (beg_pos >= end_pos)
      vw::vw_throw(vw::ArgumentErr() 
        << "weightedRollYawError: Expecting at least 2 camera positions.\n");

    // Find the segment along which the cameras are located, in projected coordinates
    // Here we mirror the logic from SatSim.cc
    int b = beg_pos * NUM_XYZ_PARAMS;
    int c = cur_pos * NUM_XYZ_PARAMS;
    int e = end_pos * NUM_XYZ_PARAMS;
    vw::Vector3 beg_pt(positions[b], positions[b+1], positions[b+2]);
    vw::Vector3 cur_pt(positions[c], positions[c+1], positions[c+2]);
    vw::Vector3 end_pt(positions[e], positions[e+1], positions[e+2]);

    // Orbital points before the current one, the current one, and after the
    // current one, in projected coordinates
    vw::Vector3 beg_proj = vw::cartography::ecefToProj(georef, beg_pt);
    vw::Vector3 cur_proj = vw::cartography::ecefToProj(georef, cur_pt);
    vw::Vector3 end_proj = vw::cartography::ecefToProj(georef, end_pt);
    
    // Find satellite along and across track directions in projected coordinates
    vw::Vector3 proj_along, proj_across;
    asp::calcProjAlongAcross(beg_proj, end_proj, proj_along, proj_across);

    // Find along and across in ECEF
    vw::Vector3 along, across;
    asp::calcEcefAlongAcross(georef, asp::satSimDelta(), 
                              proj_along, proj_across, cur_proj,
                              along, across); // outputs

    // Find the z vector as perpendicular to both along and across
    vw::Vector3 down = vw::math::cross_prod(along, across);
    down = down / norm_2(down);

    // Find the rotation matrix from satellite to world coordinates, and 90
    // degree in-camera rotation. It is assumed, as in sat_sim, that:
    // cam2world = sat2World * rollPitchYaw * rotXY.
    asp::assembleCam2WorldMatrix(along, across, down, m_sat2World);
    m_rotXY = asp::rotationXY();

    // Initial camera rotation matrix, before we optimize it
    m_initCam2World = asp::quaternionToMatrix(&quaternions[cur_pos*NUM_QUAT_PARAMS]);
}

  // See the .h file for the documentation.
  bool weightedRollYawError::operator()(double const * const * parameters, 
                                       double * residuals) const {

    // Convert to rotation matrix. Order of quaternion is x, y, z, w.  
    vw::Matrix3x3 cam2world = asp::quaternionToMatrix(parameters[0]);

    if (m_initial_camera_constraint) {
      // Find the new camera orientation relative to the initial camera, not
      // relative to the satellite along-track direction. Then find the roll and
      // yaw from it. This is experimental.
      vw::Matrix3x3 cam2cam =  vw::math::inverse(cam2world) * m_initCam2World;

      double roll, pitch, yaw;
      rollPitchYawFromRotationMatrix(cam2cam, roll, pitch, yaw);

      // Fix for roll / yaw being determined with +/- 180 degree ambiguity.
      roll  = roll  - 180.0 * round(roll  / 180.0);
      pitch = pitch - 180.0 * round(pitch / 180.0);
      yaw   = yaw   - 180.0 * round(yaw   / 180.0);

      // Roll, pitch, yaw in camera coordinates are pitch, roll, yaw in satellite
      // coordinates. So adjust below accordingly.
      // CERES is very tolerant if one of the weights used below is 0. So there is
      // no need to use a special cost function for such cases.
      residuals[0] = pitch * m_rollWeight; // per above, swap roll and pitch
      residuals[1] = yaw  * m_yawWeight;

      return true;
    }

    vw::Matrix3x3 rollPitchYaw  
      = vw::math::inverse(m_sat2World) * cam2world * vw::math::inverse(m_rotXY);

    double roll, pitch, yaw;
    rollPitchYawFromRotationMatrix(rollPitchYaw, roll, pitch, yaw);

    // Fix for roll / yaw being determined with +/- 180 degree ambiguity.
    roll = roll - 180.0 * round(roll / 180.0);
    pitch = pitch - 180.0 * round(pitch / 180.0);
    yaw  = yaw  - 180.0 * round(yaw  / 180.0);

    // CERES is very tolerant if one of the weights used below is 0. So there is
    // no need to use a special cost function for such cases.
    residuals[0] = roll * m_rollWeight;
    residuals[1] = yaw  * m_yawWeight;

    return true;
  }

} // end namespace asp
