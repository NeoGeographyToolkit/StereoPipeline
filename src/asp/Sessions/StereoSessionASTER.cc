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


/// \file StereoSessionASTER.cc
///
#include <asp/Camera/LinescanASTERModel.h>
#include <asp/Sessions/StereoSessionASTER.h>

#include <vw/Camera/CameraModel.h>

namespace asp {

  boost::shared_ptr<vw::camera::CameraModel>  StereoSessionASTER::load_camera_model
    (std::string const& image_file, std::string const& camera_file, 
     std::string const& ba_prefix, vw::Vector2 pixel_offset) const {
    return load_adjusted_model(m_camera_loader.load_ASTER_camera_model(camera_file),
                              image_file, camera_file, ba_prefix, pixel_offset);
  }
} // End namespace asp
