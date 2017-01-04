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
#include <vw/Stereo/StereoModel.h>

#include <asp/Core/StereoSettings.h>
#include <asp/Core/InterestPointMatching.h>
#include <asp/Core/AffineEpipolar.h>

namespace asp {

/// Generic stereoSession implementation for images which we can read/write with GDAL.
/// - This class adds a "preprocessing hook" which aligns and normalizes the images using the specified methods.
class StereoSessionASTER : public StereoSessionGdal<DISKTRANSFORM_TYPE_MATRIX,   
                                                    STEREOMODEL_TYPE_ASTER> {
  
public:
  StereoSessionASTER(){}
  virtual ~StereoSessionASTER(){}
  
  virtual std::string name() const { return "aster"; }
  
  /// Simple factory function
  static StereoSession* construct() { return new StereoSessionASTER; }
  
  /// This function will be over-written for ASTER
  virtual void main_or_rpc_camera_models(boost::shared_ptr<vw::camera::CameraModel> &cam1,
                                         boost::shared_ptr<vw::camera::CameraModel> &cam2);
  
};
  

} // End namespace asp

#endif//__STEREO_SESSION_ASTER_H__
