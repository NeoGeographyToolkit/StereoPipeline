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

/// \file RpcUtils.h

// RPC model utilities

#ifndef __ASP_CAMERA_RPC_UTILS_H__
#define __ASP_CAMERA_RPC_UTILS_H__

#include <vw/Camera/CameraModel.h>
#include <vw/Math/Matrix.h>
#include <vw/Math/BBox.h>

#include <string>
#include <vector>

namespace vw {
  namespace cartography {
    class Datum;
    class GeoReference;
  }
}

namespace asp {

// Sample the llh box and shoot the 3D points into the camera.
// Filter by image box.
void sample_llh_pix_bbox(vw::BBox2 const& ll_box,
                         vw::Vector2 const& h_range,
                         int num_samples,
                         vw::cartography::Datum const& datum,
                         vw::CamPtr cam,
                         vw::BBox2 const& image_box,
                         // Outputs
                         std::vector<vw::Vector3> & all_llh,
                         std::vector<vw::Vector2> & all_pixels);

// Add pixel and llh samples along the perimeter and diagonals of image_box.
// Constrain by the ll box.
void add_perimeter_diag_points(vw::BBox2 const& image_box, 
                               vw::cartography::Datum const& datum,
                               vw::CamPtr cam,
                               vw::BBox2 const& ll, // lon-lat box
                               vw::Vector2 const& H, // height range
                               // Outputs (append to these)
                               std::vector<vw::Vector3> & all_llh,
                               std::vector<vw::Vector2> & all_pixels);

} // end namespace asp

#endif //__ASP_CAMERA_RPC_UTILS_H__
