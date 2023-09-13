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

/// \file LinescanUtils.h

// Functions that know about the many kinds of linescan cameras

#ifndef __ASP_CAMERA_LINESCAN_UTILS_H__
#define __ASP_CAMERA_LINESCAN_UTILS_H__

#include <vw/Camera/CameraModel.h>

namespace asp {

  // Find the underlying CSM camera. Applies only to CSM, Pleiades, and DG.
  class CsmModel;
  CsmModel * csm_model(boost::shared_ptr<vw::camera::CameraModel> cam,
                      std::string const& stereo_session);

} // end namespace asp

#endif //__ASP_CAMERA_LINESCAN_UTILS_H__