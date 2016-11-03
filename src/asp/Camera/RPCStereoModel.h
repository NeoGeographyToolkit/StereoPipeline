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


#ifndef __ASP_CAMERA_RPCSTEREOMODEL_H__
#define __ASP_CAMERA_RPCSTEREOMODEL_H__

#include <vw/Stereo/DisparityMap.h>
#include <vw/Stereo/StereoModel.h>

// TODO: Why not just include the file in the .h file?
// forward declaration
namespace vw {
  namespace camera {
    class CameraModel;
  }
}

namespace asp {

  /// Derived StereoModel class implementing the RPC camera model.
  /// - Using a seperate class allows us to get a speed improvement in ray generation.
  class RPCStereoModel: public vw::stereo::StereoModel {

  public:

    //------------------------------------------------------------------
    // Constructors / Destructors
    //------------------------------------------------------------------
    RPCStereoModel(std::vector<const vw::camera::CameraModel *> const& cameras,
                   bool least_squares_refine = false,
                   double angle_tol = 0.0):
      vw::stereo::StereoModel(cameras, least_squares_refine, angle_tol){}
      
    RPCStereoModel(vw::camera::CameraModel const* camera_model1,
                   vw::camera::CameraModel const* camera_model2,
                   bool least_squares_refine = false,
                   double angle_tol = 0.0):
      vw::stereo::StereoModel(camera_model1, camera_model2, least_squares_refine, angle_tol){}
    
    virtual ~RPCStereoModel() {}
    
    //------------------------------------------------------------------
    // Public Methods
    //------------------------------------------------------------------
    
    // Note: This is a re-implementation of StereoModel::operator().
    virtual vw::Vector3 operator()(std::vector<vw::Vector2> const& pixVec, vw::Vector3& errorVec ) const;

    virtual vw::Vector3 operator()(vw::Vector2 const& pix1,
                                   vw::Vector2 const& pix2,
                                   double& error) const;
  };
  
} // namespace asp

#endif  // __ASP_CAMERA_RPCSTEREOMODEL_H__
