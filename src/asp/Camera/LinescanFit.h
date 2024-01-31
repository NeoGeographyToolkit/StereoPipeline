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

class CsmModel;

// Fit a CSM sensor with distortion to given tabulated sight directions.
// This is specific to ASTER.
void fitAsterLinescanCsmModel(
       std::string const& sensor_id, 
       vw::cartography::Datum const& datum,
       vw::Vector2i const& image_size,
       std::vector<vw::Vector3> const& sat_pos,
       std::vector<std::vector<vw::Vector3>> const& world_sight_mat,
       int min_col, int min_row,
       int d_col, int d_row, 
       // This model will be modified
       asp::CsmModel & csm_model);
  
} // end namespace asp

#endif//__ASP_CAMERA_LINESCAN_FIT_H__
