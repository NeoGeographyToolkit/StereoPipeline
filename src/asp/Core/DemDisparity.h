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


/// \file DEMDisparity.h
///

#ifndef __DEM_DISPARITY_H__
#define __DEM_DISPARITY_H__

#include <boost/smart_ptr/shared_ptr.hpp>

// Forward declaration
namespace asp {
  class ASPGlobalOptions;
}
namespace vw {
  namespace camera {
    class CameraModel;
  }
}

namespace asp {

  /// Use a DEM to get the low-res disparity
  void produce_dem_disparity(ASPGlobalOptions & opt,
                             boost::shared_ptr<vw::camera::CameraModel> left_camera_model,
                             boost::shared_ptr<vw::camera::CameraModel> right_camera_model,
                             std::string session_name
                             );

}

#endif
