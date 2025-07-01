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

/// \file CsmModelFit.h

#ifndef __ASP_CAMERA_CSM_MODEL_FIT_H__
#define __ASP_CAMERA_CSM_MODEL_FIT_H__

#include <vw/Math/Matrix.h>
#include <vw/Math/Vector.h>

// Forward declaration
namespace vw {
  namespace cartography {
    class Datum;
  }
}

namespace asp {

class CsmModel;

typedef std::vector<std::vector<vw::Vector3>> SightMatT;

// Fit a CSM sensor with distortion to given tabulated sight directions.
// This is specific to ASTER.
void fitCsmLinescan(
       std::string const& sensor_id, 
       vw::cartography::Datum const& datum,
       vw::Vector2i const& image_size,
       std::vector<vw::Vector3> const& sat_pos,
       SightMatT const& world_sight_mat,
       int min_col, int min_row,
       int d_col, int d_row, 
       bool fit_distortion,
       // This model will be modified
       asp::CsmModel & csm_model);
  
// Create pixel samples. Make sure to sample the pixel at (width - 1, height - 1).
void createPixelSamples(int width, int height, int num_pixel_samples,
                        std::vector<vw::Vector2> & pix_samples);
 
// Refine a CSM frame camera model using a a set of ground points projecting at given pixels
void refineCsmFrameFit(std::vector<vw::Vector2> const& pixels,
                       std::vector<vw::Vector3> const& directions,
                       std::string const& refine_intrinsics,
                       asp::CsmModel & csm_model); // output

// Fit a CSM camera to an optical bar camera. The .cc file has more details.
void fitCsmLinescanToOpticalBar(std::string const& camFile,
                                vw::Vector2i const& imageFize,
                                vw::cartography::Datum const& datum,
                                asp::CsmModel & csm);
  
} // end namespace asp

#endif // __ASP_CAMERA_CSM_MODEL_FIT_H__
