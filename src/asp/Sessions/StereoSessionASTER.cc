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
#include <vw/Image/ImageMath.h>
#include <vw/Image/Manipulation.h>
#include <vw/Image/MaskViews.h>
#include <vw/Image/Transform.h>

//#include <asp/Core/StereoSettings.h>
#include <asp/Core/InterestPointMatching.h>
#include <asp/Core/AffineEpipolar.h>
#include <asp/Camera/LinescanASTERModel.h>
#include <asp/Sessions/ResourceLoader.h>
#include <asp/Sessions/StereoSessionASTER.h>


#include <iostream>
#include <string>
#include <vector>
#include <algorithm>

using namespace vw;
using namespace asp;


namespace pt = boost::posix_time;
namespace fs = boost::filesystem;

// Allows FileIO to correctly read/write these pixel types
namespace vw {
  template<> struct PixelFormatID<Vector3>   { static const PixelFormatEnum value = VW_PIXEL_GENERIC_3_CHANNEL; };
  template<> struct PixelFormatID<Vector2f>  { static const PixelFormatEnum value = VW_PIXEL_GENERIC_2_CHANNEL; };
}


namespace asp {

  /// Fetch the RPC models for aster. We want to use those, for ip matching, as they are faster.
  void StereoSessionASTER::
  main_or_rpc_camera_models(boost::shared_ptr<vw::camera::CameraModel> &cam1,
                            boost::shared_ptr<vw::camera::CameraModel> &cam2){

    std::cout << "---now in rpc_camera_models" << std::endl;
    
    boost::shared_ptr<vw::camera::CameraModel> base_cam1, base_cam2;
    this->camera_models(base_cam1, base_cam2);
    
    ASTERCameraModel * aster_cam1 = dynamic_cast<ASTERCameraModel*>(base_cam1.get());
    if (aster_cam1 == NULL) vw_throw( ArgumentErr() << "ASTER camera models are expected." );
    cam1 = aster_cam1->get_rpc_model();
    
    ASTERCameraModel * aster_cam2 = dynamic_cast<ASTERCameraModel*>(base_cam2.get());
    if (aster_cam2 == NULL) vw_throw( ArgumentErr() << "ASTER camera models are expected." );
    cam2 = aster_cam2->get_rpc_model();
    
  }

} // End namespace asp
