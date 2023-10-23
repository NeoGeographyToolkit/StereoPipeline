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

/// \file CsmUtils.h

// Functions used for handling CSM camera models.

#ifndef __ASP_CAMERA_CSM_UTILS_H__
#define __ASP_CAMERA_CSM_UTILS_H__

#include <vw/Math/Matrix.h>


#include <string>
#include <iostream>
#include <map>

class UsgsAstroFrameSensorModel;
class UsgsAstroLsSensorModel;

namespace vw {
  namespace cartography {
    class Datum;
    class GeoReference;
  }
}

namespace asp {

const int NUM_XYZ_PARAMS  = 3;
const int NUM_QUAT_PARAMS = 4;
const int PIXEL_SIZE      = 2;

class CsmModel;

// Normalize quaternions in UsgsAstroLsSensorModel.
void normalizeQuaternions(UsgsAstroLsSensorModel * ls_model);

// Normalize quaternions in UsgsAstroFrameSensorModel.
void normalizeQuaternions(UsgsAstroFrameSensorModel * frame_model);

// Get quaternions. This duplicates the UsgsAstroLsSensorModel function as that one is private
void interpQuaternions(UsgsAstroLsSensorModel * ls_model, double time,
                       double q[4]);

// Get positions. Based on the UsgsAstroLsSensorModel code.
void interpPositions(UsgsAstroLsSensorModel * ls_model, double time,
                     double pos[3]);

// Get positions. Based on the UsgsAstroLsSensorModel code.
void interpVelocities(UsgsAstroLsSensorModel * ls_model, double time,
                      double vel[3]);

// Nearest neighbor interpolation into a sequence of vectors of length
// vectorLength, stored one after another in valueArray. The result
// goes in valueVector. Analogous to lagrangeInterp() in CSM.
void nearestNeibInterp(const int &numTimes, const double *valueArray,
                       const double &startTime, const double &delTime,
                       const double &time, const int &vectorLength,
                       double *valueVector);

// Find interpolated/extrapolated positions at all camera pose times.
// See the implementation for more details.
void orbitInterpExtrap(UsgsAstroLsSensorModel const * ls_model,
                       vw::cartography::GeoReference const& geo,
                       std::vector<double> & positions_out);

// Populate the CSM model with the given camera positions and orientations.
// TODO(oalexan1): Use this in LinescanDGModel.cc.
void populateCsmLinescan(double first_line_time, double dt_line, 
                         double t0_ephem, double dt_ephem,
                         double t0_quat, double dt_quat, 
                         double focal_length,
                         vw::Vector2                const & optical_center,
                         vw::Vector2i               const & image_size,
                         vw::cartography::Datum     const & datum, 
                         std::string                const & sensor_id, 
                         std::vector<vw::Vector3>   const & positions,
                         std::vector<vw::Vector3>   const & velocities,
                         std::vector<vw::Matrix3x3> const & cam2world,
                         // Outputs
                         asp::CsmModel                    & model);

} // end namespace asp

#endif//__ASP_CAMERA_CSM_UTILS_H__