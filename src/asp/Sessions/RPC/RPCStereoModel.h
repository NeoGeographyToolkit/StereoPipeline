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


#ifndef __ASP_RPC_RPCSTEREOMODEL_H__
#define __ASP_RPC_RPCSTEREOMODEL_H__

#include <vw/Stereo/DisparityMap.h>
#include <vw/Stereo/StereoModel.h>

// forward declaration
namespace vw {
  namespace camera {
    class CameraModel;
  }
}

namespace asp {

  class RPCStereoModel: public vw::stereo::StereoModel {

  public:

    //------------------------------------------------------------------
    // Constructors / Destructors
    //------------------------------------------------------------------
    RPCStereoModel(vw::camera::CameraModel const* camera_model1,
                   vw::camera::CameraModel const* camera_model2,
                   bool least_squares_refine = false):
      vw::stereo::StereoModel(camera_model1, camera_model2, least_squares_refine){}

    //------------------------------------------------------------------
    // Public Methods
    //------------------------------------------------------------------

    /// Apply a stereo model to a single pair of image coordinates.
    /// Returns an xyz point.  The error is set to -1 if the rays were
    /// parallel or divergent, otherwise it returns the 2-norm of the
    /// distance between the rays at their nearest point of
    /// intersection.
    virtual vw::Vector3 operator()(vw::Vector2 const& pix1, vw::Vector2 const& pix2,
                                   vw::Vector3& errorVec) const;
    virtual vw::Vector3 operator()(vw::Vector2 const& pix1, vw::Vector2 const& pix2,
                                   double& error) const;

  };

} // namespace asp

#endif  // __ASP_RPC_RPCSTEREOMODEL_H__
