// __BEGIN_LICENSE__
//  Copyright (c) 2009-2025, United States Government as represented by the
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

/// \file ProjectiveCamApprox.h

// Approximate a camera model with a projective transform

#ifndef __ASP_CORE_PROJECTIVE_CAM_APPROX_H__
#define __ASP_CORE_PROJECTIVE_CAM_APPROX_H__

#include <vw/Math/Vector.h>

namespace asp {

// Compute the best-fitting projective transform that maps a set of 3D ground points
// to 2D image points. See also: applyProjTrans().
void calcProjTrans(std::vector<vw::Vector2> const& imagePts,
                   std::vector<vw::Vector3> const& groundPts,
                   std::vector<double> & transformCoeffs);

// Apply a projective transform to a ground point and compute the image pixel
vw::Vector2 applyProjTrans(vw::Vector3 const& xyz,
                           std::vector<double> const& transformCoeffs);

} // End namespace asp

#endif // __ASP_CORE_PROJECTIVE_CAM_APPROX_H__

