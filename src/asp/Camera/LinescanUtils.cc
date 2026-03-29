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

// Functions that know about the many kinds of linescan cameras

#include <asp/Camera/LinescanUtils.h>
#include <asp/Camera/CsmModel.h>

namespace asp {

// Whether the session uses a CSM linescan model.
bool isLinescanCsmSession(std::string const& session) {
  return (session == "csm"      || session == "dg"      ||
          session == "pleiades" || session == "spot"     ||
          session == "perusat"  || session == "aster"    ||
          session == "spot5");
}

// Find the underlying CSM camera. Applies to all linescan CSM sessions.
// All linescan CSM models (Pleiades, DG, PeruSat, Spot, ASTER) inherit
// directly from CsmModel, so a simple dynamic_cast suffices.
asp::CsmModel * csm_model(boost::shared_ptr<vw::camera::CameraModel> cam) {

  // In some places the camera models have an external adjustment.
  // If that is the case, this function only works if the adjustment
  // is the identity, as it does not know how to apply it.
  vw::camera::AdjustedCameraModel *adj_cam
    = dynamic_cast<vw::camera::AdjustedCameraModel*>(cam.get());
  if (adj_cam != NULL) {
    vw::Matrix4x4 ecef_transform = adj_cam->ecef_transform();
    // must be the identity
    if (ecef_transform != vw::math::identity_matrix<4>())
      vw::vw_throw(vw::ArgumentErr()
        << "A CSM camera with an adjustment such as coming from "
        << "--bundle-adjust-prefix detected. Not supported in this workflow.\n");
  }

  asp::CsmModel * csm_model
    = dynamic_cast<asp::CsmModel*>(vw::camera::unadjusted_model(cam.get()));

  if (csm_model == NULL)
      vw::vw_throw(vw::ArgumentErr() << "Expected a CSM camera model.");

  return csm_model;
}

} // end namespace asp