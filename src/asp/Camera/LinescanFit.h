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

/// \file LinescanFit.h

// Find linescan rotations given matrix of sight vectors

#ifndef __ASP_CAMERA_LINESCAN_FIT_H__
#define __ASP_CAMERA_LINESCAN_FIT_H__

#include <vw/Math/Matrix.h>
#include <vw/Math/Vector.h>

namespace asp {

// Find the rotation matrices, focal length, and optical center,
// that best fit a 2D matrix of sight vectors. Uses for ASTER.
void fitBestRotationsIntrinsics(
   std::vector<std::vector<vw::Vector3>> const& world_sight_mat,
   vw::Vector2i const& image_size, int d_col,
   // Outputs
   double & focal_length, vw::Vector2 & optical_center,
   std::vector<vw::Matrix<double,3,3>> & rotation_vec);
  
} // end namespace asp

#endif//__ASP_CAMERA_LINESCAN_FIT_H__