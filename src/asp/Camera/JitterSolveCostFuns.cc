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

} // end namespace asp
