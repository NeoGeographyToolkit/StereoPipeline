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


/// \file StereoSessionRPC.cc
///

#include <vw/Camera/CameraModel.h>

#include <asp/Sessions/RPC/StereoSessionRPC.h>
#include <asp/Sessions/DG/XML.h>
#include <asp/Sessions/RPC/RPCModel.h>

#include <string>

#include <boost/smart_ptr/shared_ptr.hpp>

using namespace vw;
using namespace asp;

namespace asp {

  // Provide our camera model
  boost::shared_ptr<camera::CameraModel>
  StereoSessionRPC::camera_model(std::string const& image_file,
                                 std::string const& camera_file ) {
    return boost::shared_ptr<camera::CameraModel>(read_rpc_model(image_file,
                                                                 camera_file));
  }

  // Helper function to read RPC models.
  RPCModel* StereoSessionRPC::read_rpc_model( std::string const& image_file,
                                              std::string const& camera_file ) {
    RPCModel* rpc_model = NULL;
    try {
      rpc_model = new RPCModel( image_file );
    } catch ( NotFoundErr const& err ) {}

    if ( !rpc_model ) {

      if (camera_file == ""){
        vw_throw( NotFoundErr()
                  << "RPCModel: Could not find the RPC model in " << image_file
                  << ", and no XML camera file was provided.\n" );
      }

      RPCXML rpc_xml;
      rpc_xml.read_from_file( camera_file );
      rpc_model = new RPCModel( *rpc_xml.rpc_ptr() ); // Copy the value

      // We don't catch an error here because the user will need to
      // know of a failure at this point. We previously opened the
      // xml file safely before.
    }
    return rpc_model;
  }
} // namespace asp
