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
#include <vw/Camera/CameraModel.h>

//#include <asp/Core/StereoSettings.h>
#include <asp/Core/InterestPointMatching.h>
#include <asp/Core/AffineEpipolar.h>
#include <asp/Camera/LinescanASTERModel.h>
#include <asp/Sessions/StereoSessionASTER.h>


#include <iostream>
#include <string>
#include <vector>
#include <algorithm>

using namespace vw;
using namespace vw::camera;
using namespace asp;


namespace pt = boost::posix_time;
namespace fs = boost::filesystem;

namespace asp {
  
  // This function will get the RPC models approximating the ASTER
  // models.  We want to use the RPC models for ip matching, as they
  // are way faster.  That does not affect the accuracy of the final
  // DEM, as interest points are only used to guess rough alignment
  // transforms and an initial search range.
  void StereoSessionASTER::rpc_camera_models(boost::shared_ptr<vw::camera::CameraModel> &cam1,
                                             boost::shared_ptr<vw::camera::CameraModel> &cam2){
    
    // First load the ASTER models
    boost::shared_ptr<vw::camera::CameraModel> base_cam1, base_cam2;
    this->camera_models(base_cam1, base_cam2);
    
    // Check if the models are adjusted, as loaded by load_camera_model() below.
    AdjustedCameraModel* adj_cam1 = dynamic_cast<AdjustedCameraModel*>(base_cam1.get());
    AdjustedCameraModel* adj_cam2 = dynamic_cast<AdjustedCameraModel*>(base_cam2.get());
    if (adj_cam1 == NULL || adj_cam2 == NULL)
      vw_throw(ArgumentErr() << "An adjusted model expected in StereoSessionASTER.");

    // Strip the adjustments
    boost::shared_ptr<vw::camera::CameraModel> unadj_cam1, unadj_cam2;
    unadj_cam1 = adj_cam1->unadjusted_model();
    unadj_cam2 = adj_cam2->unadjusted_model();

    // Sanity check
    ASTERCameraModel * aster_cam1 = dynamic_cast<ASTERCameraModel*>(unadj_cam1.get());
    ASTERCameraModel * aster_cam2 = dynamic_cast<ASTERCameraModel*>(unadj_cam2.get());
    if (aster_cam1 == NULL || aster_cam2 == NULL)
      vw_throw(ArgumentErr() << "An ASTER camera model is expected in StereoSessionASTER.");

    // Get the RPC models
    boost::shared_ptr<vw::camera::CameraModel> rpc_cam1 = aster_cam1->get_rpc_model();
    boost::shared_ptr<vw::camera::CameraModel> rpc_cam2 = aster_cam2->get_rpc_model();

    // Copy the adjustments over to the RPC models. This is very
    // important, as the options --left-image-crop-win and
    // --right-image-crop-win change the value of pixel_offset()
    // below, and bundle adjustment may have been done, which affects
    // translation() and rotation().
    cam1 = boost::shared_ptr<vw::camera::CameraModel>
      (new AdjustedCameraModel(rpc_cam1, adj_cam1->translation(), adj_cam1->rotation(),
                               adj_cam1->pixel_offset(), adj_cam1->scale()));
    cam2 = boost::shared_ptr<vw::camera::CameraModel>
      (new AdjustedCameraModel(rpc_cam2, adj_cam2->translation(), adj_cam2->rotation(),
                               adj_cam2->pixel_offset(), adj_cam2->scale()));
  }
  
  boost::shared_ptr<vw::camera::CameraModel>  StereoSessionASTER::load_camera_model
    (std::string const& image_file, std::string const& camera_file, Vector2 pixel_offset) const{

    return load_adjusted_model(m_camera_loader.load_ASTER_camera_model(camera_file),
                              image_file, camera_file, pixel_offset);
  }
} // End namespace asp
