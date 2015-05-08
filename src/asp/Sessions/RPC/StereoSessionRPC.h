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

#include <asp/Sessions/DG/StereoSessionDG.h>
#include <asp/Sessions/RPC/RPCStereoModel.h>

// TODO: Should not derive from DG!  RPC is used for non-DG files!

namespace asp {

  // TODO: Why not just include this in the .h file?
  // Forward declaration
  class RPCModel;

  // TODO: Do not derive this from DG?
  //   Currently this is really a "DG_using_RPC_model" class.

  /// Derived StereoSession class using the RPC camera model.
  class StereoSessionRPC : public StereoSessionDG {
  public:
    StereoSessionRPC(){};
    virtual ~StereoSessionRPC(){};

    // Produces a camera model from the images
    virtual boost::shared_ptr<vw::camera::CameraModel>
    camera_model( std::string const& image_file,
                  std::string const& camera_file);

    virtual std::string name() const { return "rpc"; }

    typedef asp::RPCStereoModel stereo_model_type;

    /// Simple factory function.
    static StereoSession* construct() { return new StereoSessionRPC; }

    /// Load an RPC camera model from an image file / camera file pair.
    static RPCModel* read_rpc_model( std::string const& image_file,
                                     std::string const& camera_file );
  };

} // namespace asp

#endif // __STEREO_SESSION_RPC_H__
