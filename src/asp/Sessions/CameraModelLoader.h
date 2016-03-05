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


/// \file CameraModelLoader.h
///

#ifndef __STEREO_SESSION_CAMERAMODELLOADER_H__
#define __STEREO_SESSION_CAMERAMODELLOADER_H__


#include <vw/Camera.h>
// TODO: Break this up. Each of these functions must go back to their
// individual Session directories rather than being collected here.

namespace asp {

  // !!! This class is not meant to be invoked directly !!!
  // Use instead the interface load_camera_model in StereoSessionConcrete.tcc.

  class CameraModelLoader {
  public:

    typedef boost::shared_ptr<vw::camera::CameraModel> CameraModelPtr;

    // Setup/teardown code is handled here
    CameraModelLoader();
    ~CameraModelLoader();

    // Camera model loading functions
    // All private functions!
    CameraModelPtr load_rpc_camera_model    (std::string const& path) const;
    CameraModelPtr load_dg_camera_model     (std::string const& path) const;
    CameraModelPtr load_pinhole_camera_model(std::string const& path) const;
    CameraModelPtr load_isis_camera_model   (std::string const& path) const;
  }; // End class CameraModelLoader

} // end namespace asp

#endif // __STEREO_SESSION_CAMERAMODELLOADER_H__
