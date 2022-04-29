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
#include <asp/Sessions/StereoSessionMapProj.h>
#include <asp/Sessions/StereoSessionIsis.h>
#include <asp/Sessions/StereoSessionNadirPinhole.h>
#include <asp/Sessions/StereoSessionPinhole.h>
#include <asp/Sessions/StereoSessionRPC.h>
#include <asp/Sessions/StereoSessionASTER.h>

#include <vw/FileIO/DiskImageResourceRaw.h>
#include <vw/Camera/CameraUtilities.h>
#include <asp/Camera/SPOT_XML.h>
#include <asp/Camera/ASTER_XML.h>
#include <vw/Camera/OpticalBarModel.h>

namespace asp{

  StereoSession* StereoSessionFactory::create(std::string      & session_type, // in-out variable
                                              vw::cartography::GdalWriteOptions const& options,
                                              std::string const& left_image_file,
                                              std::string const& right_image_file,
                                              std::string const& left_camera_file,
                                              std::string const& right_camera_file,
                                              std::string const& out_prefix,
                                              std::string const& input_dem,
                                              const bool allow_map_promote) {
    
    // Known user session types are:
    // DG, RPC, ISIS, Pinhole, NadirPinhole, OpticalBar
    //
    // Hidden sessions are:
    // DGMapRPC, Blank (Guessing)
    
    // Try to guess the session if not provided
    std::string actual_session_type = session_type;
    bool quiet = true;
    boost::to_lower(actual_session_type);
    if (actual_session_type.empty()) {
      if (asp::has_pinhole_extension(left_camera_file ) || // TODO: Fix this dangerous code!
          asp::has_pinhole_extension(right_camera_file)   ) {
        // There can be several types of .tsai files
        std::string error_pinhole, error_opticalbar;
        try {
          boost::shared_ptr<vw::camera::CameraModel> P = 
            vw::camera::load_pinhole_camera_model(left_camera_file);
          actual_session_type = "pinhole";
        }catch (std::exception & e) {
          error_pinhole = e.what();
          try {
            vw::camera::OpticalBarModel P;
            P.read(left_camera_file);
            actual_session_type = "opticalbar";
          }catch (std::exception & e) {
            error_opticalbar = e.what();
            vw_throw(vw::NoImplErr() << "Could not read the camera model " <<
                     left_camera_file << " as pinhole. "
                     << "The error was: " << error_pinhole << "\n"
                     << "Could not read it as opticalbar model either, the error was: "
                     << error_opticalbar);
          }
        }
      } else if (boost::iends_with(boost::to_lower_copy(left_camera_file ), ".json") ||
          boost::iends_with(boost::to_lower_copy(right_camera_file), ".json") ) {
        actual_session_type = "csm";
      }else if (boost::iends_with(boost::to_lower_copy(left_image_file  ), ".cub") ||
		boost::iends_with(boost::to_lower_copy(right_image_file ), ".cub") ||
		boost::iends_with(boost::to_lower_copy(left_camera_file ), ".cub") ||
		boost::iends_with(boost::to_lower_copy(right_camera_file), ".cub") ) {
        actual_session_type = "isis";
      } else if (boost::iends_with(boost::to_lower_copy(left_camera_file ), ".dim") ||
                 boost::iends_with(boost::to_lower_copy(right_camera_file), ".dim") ) {
        actual_session_type = "spot5";
      }else if (boost::iends_with(boost::to_lower_copy(left_camera_file ), ".xml") ||
		boost::iends_with(boost::to_lower_copy(right_camera_file), ".xml") ) {

        // Here we have several options
        if (actual_session_type.empty()) {
          // Try DG exact linescan model
          try {
            StereoSessionDG session;
            boost::shared_ptr<vw::camera::CameraModel>
              left_model  = session.camera_model(left_image_file,  left_camera_file, quiet),
              right_model = session.camera_model(right_image_file, right_camera_file, quiet);
            actual_session_type = "dg";
          } catch (...) {}
        }
        
        if (actual_session_type.empty()) {
          // Try PeruSat exact linescan model
          try {
            StereoSessionPeruSat session;
            boost::shared_ptr<vw::camera::CameraModel>
              left_model  = session.camera_model(left_image_file,  left_camera_file, quiet),
              right_model = session.camera_model(right_image_file, right_camera_file, quiet);
            actual_session_type = "perusat";
          } catch (...) {}
        }
      } // end considering the xml extension case

      // Try RPC, which can either have xml cameras or no cameras at all (if embedded
      // in the tif files).
      if (actual_session_type.empty()) {
        try {
          StereoSessionRPC session;
          boost::shared_ptr<vw::camera::CameraModel>
            left_model  = session.camera_model(left_image_file,  left_camera_file, quiet),
            right_model = session.camera_model(right_image_file, right_camera_file, quiet);
          actual_session_type = "rpc";
        } catch (...) {}
      }
    }
    
    if (allow_map_promote) {
      if (!input_dem.empty() && actual_session_type == "dg") {
        // User says DG but also gives a DEM.
        actual_session_type = "dgmaprpc";
        VW_OUT(vw::DebugMessage,"asp") << "Changing session type to: dgmaprpc.\n";
      }
      if (!input_dem.empty() && actual_session_type == "rpc") {
        // User says RPC but also gives a DEM.
        actual_session_type = "rpcmaprpc";
        VW_OUT(vw::DebugMessage,"asp") << "Changing session type to: rpcmaprpc.\n";
      }
      if (!input_dem.empty() && actual_session_type == "pinhole") {
        // User says PINHOLE but also gives a DEM.
        actual_session_type = "pinholemappinhole";
        VW_OUT(vw::DebugMessage,"asp") << "Changing session type to: pinholemappinhole.\n";
      }
      if (!input_dem.empty() && actual_session_type == "opticalbar") {
        // User says OPTICAL BAR but also gives a DEM.
        actual_session_type = "opticalbarmapopticalbar";
        VW_OUT(vw::DebugMessage,"asp") << "Changing session type to: opticalbarmapopticalbar.\n";
      }
      if (!input_dem.empty() && actual_session_type == "csm") {
        // User says CSM but also gives a DEM.
        actual_session_type = "csmmapcsm";
        VW_OUT(vw::DebugMessage,"asp") << "Changing session type to: csmmapcsm.\n";
      }
      if (!input_dem.empty() && actual_session_type == "isis") {
        // User says ISIS but also gives a DEM.
        actual_session_type = "isismapisis";
        VW_OUT(vw::DebugMessage,"asp") << "Changing session type to: isismapisis.\n";
      }
      if (!input_dem.empty() && actual_session_type == "spot5") {
        // User says SPOT5 but also gives a DEM.
        actual_session_type = "spot5maprpc";
        VW_OUT(vw::DebugMessage,"asp") << "Changing session type to: spot5maprpc.\n";
      }
      if (!input_dem.empty() && actual_session_type == "aster") {
        // User says ASTER but also gives a DEM.
        actual_session_type = "astermaprpc";
        VW_OUT(vw::DebugMessage,"asp") << "Changing session type to: astermaprpc.\n";
      }
      // Quetly switch from nadirpinhole to pinhole for mapprojected images
      if (!input_dem.empty() && actual_session_type == "nadirpinhole") {
        // User says nadirpinhole but also gives a DEM.
        actual_session_type = "pinholemappinhole";
        VW_OUT(vw::DebugMessage,"asp") << "Changing session type to: pinhole.\n";
      }
      
    } // End map promotion section

    if (!input_dem.empty() && actual_session_type == "perusat") {
      // User says PeruSat-1 but also gives a DEM, so the images were mapprojected.
      // If the mapprojection was done with the exact model, stereo becomes
      // painfully slow. If it was done with the RPC model, things become hard
      // to manage, and stereo needs to know both the exact and RPC model
      // and those are in different files. Hence, just don't allow mapprojected
      // images in this case.
      vw_throw(vw::NoImplErr() << "Stereo with mapprojected images and the PeruSat-1 " <<
               "linescan model is not implemented. Use instead the RPC model.");
    }
    
    // We should know the session type by now.
    VW_ASSERT(!actual_session_type.empty(),
              vw::ArgumentErr() << "Could not determine stereo session type. "
              << "Please set it explicitly using the -t switch.\n"
              << "Options include: [nadirpinhole pinhole isis dg rpc spot5 aster perusat opticalbar csm pinholemappinhole isismapisis dgmaprpc rpcmaprpc spot5maprpc astermaprpc opticalbarmapopticalbar csmmapcsm].\n");
    vw::vw_out() << "Using session: " << actual_session_type << "\n";

    // Compare the current session name to all recognized types
    // - Only one of these will ever get triggered
    StereoSession* session = NULL;
    if (actual_session_type == "dg")
      session = StereoSessionDG::construct();
    else if (actual_session_type == "dgmaprpc")
        session = StereoSessionDGMapRPC::construct();
    else if (actual_session_type == "nadirpinhole")
      session = StereoSessionNadirPinhole::construct();
    else if (actual_session_type == "pinhole")
      session = StereoSessionPinhole::construct();
    else if (actual_session_type == "rpc")
      session = StereoSessionRPC::construct();
    else if (actual_session_type == "rpcmaprpc")
      session = StereoSessionRPCMapRPC::construct();
    else if (actual_session_type == "pinholemappinhole")
      session = StereoSessionPinholeMapPinhole::construct();
    else if (actual_session_type == "opticalbarmapopticalbar")
      session = StereoSessionBarMapBar::construct();
    else if (actual_session_type == "spot5maprpc")
        session = StereoSessionSpot5MapRPC::construct();
    else if (actual_session_type == "astermaprpc")
        session = StereoSessionASTERMapRPC::construct();
#if defined(ASP_HAVE_PKG_ISISIO) && ASP_HAVE_PKG_ISISIO == 1
    else if (actual_session_type == "isis")
      session = StereoSessionIsis::construct();
    else if (actual_session_type == "isismapisis")
      session = StereoSessionIsisMapIsis::construct();
#endif
    else if (actual_session_type == "spot5")
      session = StereoSessionSpot::construct();
    else if (actual_session_type == "perusat")
      session = StereoSessionPeruSat::construct();
    else if (actual_session_type == "aster")
      session = StereoSessionASTER::construct();
    else if (actual_session_type == "opticalbar")
      session = StereoSessionOpticalBar::construct();
    else if (actual_session_type == "csm")
      session = StereoSessionCsm::construct();
    else if (actual_session_type == "csmmapcsm")
      session = StereoSessionCsmMapCsm::construct();
    if (session == 0)
      vw_throw(vw::NoImplErr() << "Unsupported stereo session type: " << actual_session_type);

    session->initialize(options,         // Initialize the new object
                        left_image_file,  right_image_file,
                        left_camera_file, right_camera_file,
                        out_prefix, input_dem);
    session_type = session->name(); // update the session name 
    return session;
} // End function create()

  
} // end namespace asp
