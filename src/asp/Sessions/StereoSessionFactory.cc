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


/// \file StereoSessionFactory.cc
///

// This include must exist for linking purposes
#include <asp/Sessions/StereoSessionFactory.h>

#include <asp/Sessions/StereoSessionFactory.h>
#include <asp/Sessions/StereoSessionDGMapRPC.h>
#include <asp/Sessions/StereoSessionIsis.h>
#include <asp/Sessions/StereoSessionNadirPinhole.h>
#include <asp/Sessions/StereoSessionPinhole.h>
#include <asp/Sessions/StereoSessionRPC.h>
#include <asp/Sessions/StereoSessionSpot.h>
#include <asp/Sessions/StereoSessionASTER.h>


#include <vw/FileIO/DiskImageResourceRaw.h>
#include <asp/Camera/SPOT_XML.h>
#include <asp/Camera/ASTER_XML.h>

namespace asp{

StereoSession* StereoSessionFactory::create(std::string        & session_type, // in-out variable
                                              vw::cartography::GdalWriteOptions const& options,
                                              std::string const& left_image_file,
                                              std::string const& right_image_file,
                                              std::string const& left_camera_file,
                                              std::string const& right_camera_file,
                                              std::string const& out_prefix,
                                              std::string const& input_dem,
                                              const bool allow_map_promote) {

    // Known user session types are:
    // DG, RPC, ISIS, Pinhole, NadirPinhole
    //
    // Hidden sessions are:
    // DGMapRPC, Blank (Guessing)

    // Try to guess the session if not provided
    std::string actual_session_type = session_type;
    boost::to_lower(actual_session_type);
    if (actual_session_type.empty()) {
      if (asp::has_pinhole_extension(left_camera_file ) || // TODO: Fix this dangerous code!
          asp::has_pinhole_extension(right_camera_file)   ) {
        actual_session_type = "pinhole";
      }
      if (boost::iends_with(boost::to_lower_copy(left_image_file  ), ".cub") ||
          boost::iends_with(boost::to_lower_copy(right_image_file ), ".cub") ||
          boost::iends_with(boost::to_lower_copy(left_camera_file ), ".cub") ||
          boost::iends_with(boost::to_lower_copy(right_camera_file), ".cub") ) {
        actual_session_type = "isis";
      }
      if (boost::iends_with(boost::to_lower_copy(left_camera_file ), ".xml") ||
          boost::iends_with(boost::to_lower_copy(right_camera_file), ".xml") ) {
        actual_session_type = "dg";
      }
      if (boost::iends_with(boost::to_lower_copy(left_camera_file ), ".dim") ||
          boost::iends_with(boost::to_lower_copy(right_camera_file), ".dim") ) {
        actual_session_type = "spot5";
      }
    }

    try {
      if (actual_session_type.empty()) {
        // RPC can be in the main file or it can be in the camera file.
        // DG sessions are always RPC sessions because they contain that
        //   as an extra camera model. Thus this RPC check must happen last.
        StereoSessionRPC session;
        boost::shared_ptr<vw::camera::CameraModel>
          left_model  = session.camera_model(left_image_file,  left_camera_file ),
          right_model = session.camera_model(right_image_file, right_camera_file);
        actual_session_type = "rpc";
      }
    } catch (vw::NotFoundErr const& e) {
      // If it throws, it wasn't RPC
    } catch (...) {
      // It didn't even have XML!
    }

    if (allow_map_promote) {
      if (!input_dem.empty() && actual_session_type == "dg") {
        // User says DG .. but also gives a DEM.
        actual_session_type = "dgmaprpc";
        VW_OUT(vw::DebugMessage,"asp") << "Changing session type to: dgmaprpc" << std::endl;
      }
      if (!input_dem.empty() && actual_session_type == "rpc") {
        // User says RPC .. but also gives a DEM.
        actual_session_type = "rpcmaprpc";
        VW_OUT(vw::DebugMessage,"asp") << "Changing session type to: rpcmaprpc" << std::endl;
      }
      if (!input_dem.empty() && actual_session_type == "pinhole") {
        // User says PINHOLE .. but also gives a DEM.
        actual_session_type = "pinholemappinhole";
        VW_OUT(vw::DebugMessage,"asp") << "Changing session type to: pinholemappinhole" << std::endl;
      }
      if (!input_dem.empty() && actual_session_type == "isis") {
        // User says ISIS .. but also gives a DEM.
        actual_session_type = "isismapisis";
        VW_OUT(vw::DebugMessage,"asp") << "Changing session type to: isismapisis" << std::endl;
      }
      if (!input_dem.empty() && actual_session_type == "spot5") {
        // User says SPOT5 .. but also gives a DEM.
        actual_session_type = "spot5maprpc";
        VW_OUT(vw::DebugMessage,"asp") << "Changing session type to: spot5maprpc" << std::endl;
      }
      if (!input_dem.empty() && actual_session_type == "aster") {
        // User says ASTER .. but also gives a DEM.
        actual_session_type = "astermaprpc";
        VW_OUT(vw::DebugMessage,"asp") << "Changing session type to: astermaprpc" << std::endl;
      }
    } // End map promotion section

    // We should know the session type by now.
    VW_ASSERT(!actual_session_type.empty(),
              vw::ArgumentErr() << "Could not determine stereo session type. "
              << "Please set it explicitly using the -t switch.\n"
              << "Options include: [pinhole isis dg rpc spot5 aster pinholemappinhole isismapisis dgmaprpc rpcmaprpc spot5maprpc astermaprpc].\n");
    VW_OUT(vw::DebugMessage,"asp") << "Using session: " << actual_session_type << ".\n";

    // Compare the current session name to all recognized types
    // - Only one of these will ever get triggered
    StereoSession* session_new = 0;
    if (actual_session_type == "dg")
      session_new = StereoSessionDG::construct();
    else if (actual_session_type == "dgmaprpc")
        session_new = StereoSessionDGMapRPC::construct();
    else if (actual_session_type == "nadirpinhole")
      session_new = StereoSessionNadirPinhole::construct();
    else if (actual_session_type == "pinhole")
      session_new = StereoSessionPinhole::construct();
    else if (actual_session_type == "rpc")
      session_new = StereoSessionRPC::construct();
    else if (actual_session_type == "rpcmaprpc")
      session_new = StereoSessionRPCMapRPC::construct();
    else if (actual_session_type == "pinholemappinhole")
      session_new = StereoSessionPinholeMapPinhole::construct();
    else if (actual_session_type == "spot5maprpc")
        session_new = StereoSessionSpot5MapRPC::construct();
    else if (actual_session_type == "astermaprpc")
        session_new = StereoSessionASTERMapRPC::construct();
#if defined(ASP_HAVE_PKG_ISISIO) && ASP_HAVE_PKG_ISISIO == 1
    else if (actual_session_type == "isis")
      session_new = StereoSessionIsis::construct();
    else if (actual_session_type == "isismapisis")
      session_new = StereoSessionIsisMapIsis::construct();
#endif
    else if (actual_session_type == "spot5")
      session_new = StereoSessionSpot::construct();
    else if (actual_session_type == "aster")
      session_new = StereoSessionASTER::construct();
    if (session_new == 0)
      vw_throw(vw::NoImplErr() << "Unsuppported stereo session type: " << session_type);

    session_new->initialize( options,         // Initialize the new object
                             left_image_file,  right_image_file,
                             left_camera_file, right_camera_file,
                             out_prefix, input_dem );
    session_type = session_new->name();
    return session_new;
} // End function create()

  
} // end namespace asp
