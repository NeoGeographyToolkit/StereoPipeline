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


/// \file StereoSessionRPC.h
///

#ifndef __STEREO_SESSION_RPC_H__
#define __STEREO_SESSION_RPC_H__

#include <asp/Camera/RPCStereoModel.h>
#include <asp/Sessions/StereoSessionGdal.h>

namespace asp {

  /// Derived StereoSession class using the RPC camera model.
  class StereoSessionRPC : public StereoSessionGdal {
  public:

    StereoSessionRPC(){};
    virtual ~StereoSessionRPC(){};

    virtual std::string name() const { return "rpc"; }

    /// Simple factory function.
    static StereoSession* construct() { return new StereoSessionRPC; }

    /// Returns the target datum to use for a given camera model.
    virtual vw::cartography::Datum get_datum(const vw::camera::CameraModel* cam,
                                             bool use_sphere_for_non_earth) const {
      const asp::RPCModel *rpc_cam
        = dynamic_cast<const asp::RPCModel*>(vw::camera::unadjusted_model(cam));
      if (rpc_cam == NULL) 
        vw::vw_throw(vw::ArgumentErr() << "Could not get an RPC camera model as expected.");
      
      // The RPC model knows its datum
      return rpc_cam->datum();
    }
    
  protected:
    /// Function to load a camera model of the particular type.
    virtual boost::shared_ptr<vw::camera::CameraModel>
    load_camera_model(std::string const& image_file, 
                      std::string const& camera_file,
                      vw::Vector2 pixel_offset) const {
      return load_rpc_camera_model(image_file, camera_file, pixel_offset);
    }
  }; // End class StereoSessionRPC

} // namespace asp

#endif // __STEREO_SESSION_RPC_H__
