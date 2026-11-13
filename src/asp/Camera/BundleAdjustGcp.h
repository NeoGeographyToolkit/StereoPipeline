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

/// \file BundleAdjustGcp.h
///
/// GCP-based camera initialization and transform, and GCP sanity checks,
/// for bundle_adjust.

#ifndef __BUNDLE_ADJUST_GCP_H__
#define __BUNDLE_ADJUST_GCP_H__

#include <asp/Camera/BundleAdjustParamMap.h>
#include <asp/Core/BundleAdjustUtils.h>

#include <vw/BundleAdjustment/ControlNetwork.h>
#include <vw/Cartography/Datum.h>

#include <string>
#include <vector>

namespace asp {

/// Generate a warning if the GCP are really far from the tie points, to catch
/// the common lat/lon swap in a GCP file. The cnet must already be triangulated.
void check_gcp_dists(boost::shared_ptr<vw::ControlNetwork> const& cnet_ptr);

/// Initialize the position and orientation of each pinhole camera model using
///  a least squares error transform to match the provided camera positions.
/// - This function overwrites the camera parameters in-place
bool init_pinhole_model_with_camera_positions
(boost::shared_ptr<vw::ControlNetwork> const& cnet,
 std::vector<vw::CamPtr> & camera_models,
 std::vector<std::string> const& image_files,
 std::vector<vw::Vector3> const & estimated_camera_gcc);

/// Initialize the position and orientation of a pinhole camera model using
/// GCP. It invokes OpenCV's PnP functionality.
void init_camera_using_gcp(boost::shared_ptr<vw::ControlNetwork> const& cnet_ptr,
                           std::vector<vw::CamPtr> & camera_models);

/// Initialize the position and orientation of each pinhole camera model using
///  a least squares error transform to match the provided control points file.
/// This function overwrites the camera parameters in-place. It works
/// if at least three GCP are seen in no less than two images.
void transform_cameras_with_shared_gcp(boost::shared_ptr<vw::ControlNetwork> const& cnet_ptr,
                       std::vector<vw::CamPtr> & camera_models);

// Given at least two images, each having at least 3 GCP that are not seen in other
// images, find and apply a transform to the camera system based on them.
void transform_cameras_with_indiv_image_gcp
  (boost::shared_ptr<vw::ControlNetwork> const& cnet_ptr,
   std::vector<vw::CamPtr> & camera_models);

// Sanity check. This does not prevent the user from setting the wrong datum,
// but it can catch unreasonable height values for GCP.
void checkGcpRadius(vw::Datum const& datum,
                    vw::ControlNetwork const& cnet);

} // end namespace asp

#endif // __BUNDLE_ADJUST_GCP_H__
