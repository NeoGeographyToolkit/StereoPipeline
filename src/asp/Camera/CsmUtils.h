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

// Populate the CSM model with the given camera positions and orientations. Note
// that num_cams_in_image is the number of cameras within the desired orbital segment
// of length orbit_len, and also the number of cameras for which we have
// image lines. We will have extra cameras beyond that segment to make it 
// easy to interpolate the camera position and orientation at any time and also to 
// solve for jitter. The indices in positions and cam2world can go beyond
// [0, num_cams_in_image). When it is in this interval, we are recording
// image lines.
// TODO(oalexan1): This is not generic enough. 
// TODO(oalexan1): Use this in LinescanDGModel.cc.
void populateCsmLinescan(int                                  num_cams_in_image,         
                         double                               orbit_len, 
                         double                               velocity,
                         double                               focal_length,
                         vw::Vector2                  const & detector_origin,
                         vw::Vector2i                 const & image_size,
                         vw::cartography::Datum       const & datum, 
                         std::string                  const & sensor_id, 
                         std::map<int, vw::Vector3>   const & positions,
                         std::map<int, vw::Matrix3x3> const & cam2world,
                         // Outputs
                         asp::CsmModel & model);

} // end namespace asp

#endif//__ASP_CAMERA_CSM_UTILS_H__