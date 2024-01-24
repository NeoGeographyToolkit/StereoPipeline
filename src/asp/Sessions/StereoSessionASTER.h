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


/// \file StereoSessionASTER.h
///
/// This a session to support ASTER satellite images.

#ifndef __STEREO_SESSION_ASTER_H__
#define __STEREO_SESSION_ASTER_H__

#include <asp/Sessions/StereoSessionGdal.h>

namespace asp {

/// Session class for ASTER images.
class StereoSessionASTER : public StereoSessionGdal {
  
public:
  StereoSessionASTER(){}
  virtual ~StereoSessionASTER(){}
  
  virtual std::string name() const { return "aster"; }
  
  /// Simple factory function
  static StereoSession* construct() { return new StereoSessionASTER; }

  /// Fetch the RPC models. We want to use those for ip matching, as they are faster.
  void rpc_camera_models(boost::shared_ptr<vw::camera::CameraModel> &cam1,
                         boost::shared_ptr<vw::camera::CameraModel> &cam2);
  
protected:
  /// Function to load a camera model of the particular type.
  virtual boost::shared_ptr<vw::camera::CameraModel>
  load_camera_model(std::string const& image_file, 
                    std::string const& camera_file,
                    std::string const& ba_prefix, 
                    vw::Vector2 pixel_offset) const;
};

} // End namespace asp

#endif//__STEREO_SESSION_ASTER_H__
