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


/// \file CameraResectioning.h
///

#ifndef __ASP_CAMERA_CAMERA_RESECTIONING_H__
#define __ASP_CAMERA_CAMERA_RESECTIONING_H__

#include <vw/Math/Vector.h>
#include <boost/shared_ptr.hpp>

#include <vector>
#include <string>

namespace vw {
  class CameraModel;
  class PinholeModel;
  class Datum;
}

namespace asp {

/// Use OpenCV to find a Pinhole camera's position and orientation
/// based on image pixels and corresponding ground positions
void findCameraPose(std::vector<vw::Vector3> const& ground_points, 
                    std::vector<vw::Vector2> const& pixel_observations,
                    vw::PinholeModel & cam);

/// Find the best camera that fits the given ground points and pixel observations,
/// with optional known or estimated camera center, and refine it.
void fit_camera_to_xyz_ht(vw::Vector3 const& known_cam_ctr,
                          vw::Vector3 const& estim_cam_ctr,
                          std::string const& camera_type,
                          bool refine_camera,
                          std::vector<vw::Vector3> const& xyz_vec,
                          std::vector<double> const& pixel_values,
                          double cam_height, double cam_weight, double cam_ctr_weight,
                          vw::Datum const& datum,
                          bool verbose,
                          boost::shared_ptr<vw::CameraModel> & out_cam);

} // end namespace asp

#endif // __ASP_CAMERA_CAMERA_RESECTIONING_H__
