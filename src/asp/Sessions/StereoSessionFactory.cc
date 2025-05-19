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

#include <asp/Sessions/StereoSessionFactory.h>
#include <asp/Sessions/StereoSessionMapProj.h>
#include <asp/Sessions/StereoSessionIsis.h>
#include <asp/Sessions/StereoSessionNadirPinhole.h>
#include <asp/Sessions/StereoSessionPinhole.h>
#include <asp/Sessions/StereoSessionRPC.h>
#include <asp/Sessions/StereoSessionASTER.h>
#include <asp/Camera/SPOT_XML.h>
#include <asp/Camera/ASTER_XML.h>
#include <asp/asp_config.h>

#if defined(ASP_HAVE_PKG_ISIS) && ASP_HAVE_PKG_ISIS == 1
#include <asp/IsisIO/IsisInterface.h>
#endif // ASP_HAVE_PKG_ISIS

#include <vw/FileIO/DiskImageResourceRaw.h>
#include <vw/Camera/CameraUtilities.h>
#include <vw/Camera/OpticalBarModel.h>
#include <vw/FileIO/FileTypes.h>

namespace asp {

// Parse the ortho heights (terrain heights) from the cameras, if they exist.
// If opt_heights is already set, it has priority. Otherwise this will
// populate those heights from the cameras.
void handleOrthoHeights(std::string const& left_image_file, 
                        std::string const& right_image_file,
                        std::string const& left_camera_file, 
                        std::string const& right_camera_file,
                        bool total_quiet,
                        vw::Vector2 & opt_heights) {
  
  bool quiet = true;
  StereoSessionRPC session;
  vw::CamPtr cam1 = session.camera_model(left_image_file,  left_camera_file, quiet);
  vw::CamPtr cam2 = session.camera_model(right_image_file, right_camera_file, quiet);
  
  asp::RPCModel const* rpc1 
    = dynamic_cast<asp::RPCModel const*>(vw::camera::unadjusted_model(cam1.get()));
  asp::RPCModel const* rpc2 
    = dynamic_cast<asp::RPCModel const*>(vw::camera::unadjusted_model(cam2.get()));
  
  if ((rpc1 == NULL) != (rpc2 == NULL))
    vw::vw_throw(vw::ArgumentErr() << "Expecting both or neither cameras to be RPC.\n");
  if (rpc1 == NULL || rpc2 == NULL) 
    return;
   
  double cam_ht1 = rpc1->m_terrain_height;
  double cam_ht2 = rpc2->m_terrain_height;
  
  // Left
  if (!std::isnan(opt_heights[0]) && !std::isnan(cam_ht1) && opt_heights[0] != cam_ht1
      && !total_quiet)
    vw::vw_out() << "Will use first entry in --ortho-heights instead of "
      << "the corresponding entry in the left camera file.\n";
   if (std::isnan(opt_heights[0]) && !std::isnan(cam_ht1))
    opt_heights[0] = cam_ht1;
  // Right
  if (!std::isnan(opt_heights[1]) && !std::isnan(cam_ht2) && opt_heights[1] != cam_ht2 &&
      !total_quiet)
    vw::vw_out() << "Will use second entry in --ortho-heights instead of "
      << "the corresponding entry in the right camera file.\n";
  if (std::isnan(opt_heights[1]) && !std::isnan(cam_ht2))
    opt_heights[1] = cam_ht2;
}

StereoSession* StereoSessionFactory::create(std::string      & session_type, // in-out
                                            vw::GdalWriteOptions const& options,
                                            std::string const& left_image_file,
                                            std::string const& right_image_file,
                                            std::string const& left_camera_file,
                                            std::string const& right_camera_file,
                                            std::string const& out_prefix,
                                            std::string const& input_dem,
                                            bool allow_map_promote, 
                                            bool total_quiet) {

  // Known user session types are:
  // DG, RPC, ISIS, Pinhole, NadirPinhole, OpticalBar, etc.
  //
  // Hidden sessions are:
  // DGMapRPC, Blank (Guessing)

  // Try to guess the session if not provided
  std::string actual_session_type = session_type;
  bool quiet = true;
  boost::to_lower(actual_session_type);
  if (actual_session_type.empty()) {
    if (vw::has_pinhole_extension(left_camera_file) ||
        vw::has_pinhole_extension(right_camera_file)) {
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
    } else if (vw::has_isd_extension(left_camera_file) ||
                vw::has_isd_extension(right_camera_file)) {
      actual_session_type = "csm";
#if defined(ASP_HAVE_PKG_ISIS) && ASP_HAVE_PKG_ISIS == 1
    } else if (boost::iends_with(boost::to_lower_copy(left_image_file), ".cub") &&
               left_camera_file == "" &&
               asp::isis::IsisCubeHasCsmBlob(left_image_file)) {
      // The image has a CSM blob and there is no separate camera file
      actual_session_type = "csm";
    } else if (boost::iends_with(boost::to_lower_copy(left_camera_file), ".cub") &&
               asp::isis::IsisCubeHasCsmBlob(left_camera_file)) {
      // The camera file has a CSM blob
      actual_session_type = "csm";
#endif // ASP_HAVE_PKG_ISIS
    } else if (boost::iends_with(boost::to_lower_copy(left_image_file  ), ".cub") ||
                boost::iends_with(boost::to_lower_copy(right_image_file ), ".cub") ||
                boost::iends_with(boost::to_lower_copy(left_camera_file ), ".cub") ||
                boost::iends_with(boost::to_lower_copy(right_camera_file), ".cub")) {
      actual_session_type = "isis";
    } else if (boost::iends_with(boost::to_lower_copy(left_camera_file ), ".dim") ||
                boost::iends_with(boost::to_lower_copy(right_camera_file), ".dim")) {
      actual_session_type = "spot5";
    } else if (boost::iends_with(boost::to_lower_copy(left_camera_file ), ".xml") ||
                boost::iends_with(boost::to_lower_copy(right_camera_file), ".xml")) {

      // Here we have several options for .xml files. Note that a
      // Digital Globe xml file has both linescan and RPC
      // models. The logic below favors the linescan sensor.
      if (actual_session_type.empty()) {
        
        // TODO(oalexan1): Try to peek in the xml file instead of doing this exhaustive
        // checking.
        
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

      if (actual_session_type.empty()) {
        // Try Pleiades exact linescan model
        try {
          StereoSessionPleiades session;
          boost::shared_ptr<vw::camera::CameraModel>
            left_model  = session.camera_model(left_image_file,  left_camera_file, quiet),
            right_model = session.camera_model(right_image_file, right_camera_file, quiet);
          actual_session_type = "pleiades";
        } catch (...) {}
      }
      
      if (actual_session_type.empty()) {
        // Try the ASTER exact linescan model
        try {
          StereoSessionASTER session;
          boost::shared_ptr<vw::camera::CameraModel>
            left_model  = session.camera_model(left_image_file,  left_camera_file, quiet),
            right_model = session.camera_model(right_image_file, right_camera_file, quiet);
          actual_session_type = "aster";
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

  // If the session name starts with rpc, read the ortho heights, if available.
  if (boost::to_lower_copy(actual_session_type).find("rpc") == 0) 
    handleOrthoHeights(left_image_file, right_image_file, 
                       left_camera_file, right_camera_file,
                       total_quiet, asp::stereo_settings().ortho_heights);
  vw::Vector2 heights = asp::stereo_settings().ortho_heights;
  bool have_heights = (!std::isnan(heights[0]) && !std::isnan(heights[1]));
  
  if (allow_map_promote) {
    
    // The case of images mapprojected onto some surface of constant height
    if (have_heights) {
      size_t pos = actual_session_type.find("map");
      if (pos != std::string::npos)
        actual_session_type = actual_session_type.substr(0, pos); // wipe map and after
      actual_session_type = actual_session_type + "map" + actual_session_type;
    }

    if (!input_dem.empty() && actual_session_type == "dg") {
      // User says DG but also gives a DEM.
      // Mapprojection can happen either with DG or RPC cameras
      std::string cam_tag = "CAMERA_MODEL_TYPE";
      std::string l_cam_type 
        = vw::cartography::read_header_string(left_image_file, cam_tag);
      if (l_cam_type == "dg")
        actual_session_type = "dgmapdg";
      else
        actual_session_type = "dgmaprpc"; // used also when l_cam_type is empty
    }
    if (!input_dem.empty() && actual_session_type == "rpc") {
      // User says RPC but also gives a DEM.
      actual_session_type = "rpcmaprpc";
    }
    if (!input_dem.empty() && actual_session_type == "pinhole") {
      // User says PINHOLE but also gives a DEM.
      actual_session_type = "pinholemappinhole";
    }
    if (!input_dem.empty() && actual_session_type == "opticalbar") {
      // User says OPTICAL BAR but also gives a DEM.
      actual_session_type = "opticalbarmapopticalbar";
    }
    if (!input_dem.empty() && actual_session_type == "csm") {
      // User says CSM but also gives a DEM.
      // Mapprojection can happen either with csm or RPC cameras (the latter for DG)
      std::string cam_tag = "CAMERA_MODEL_TYPE";
      std::string l_cam_type = vw::cartography::read_header_string(left_image_file, cam_tag);
      if (l_cam_type == "rpc")
        actual_session_type = "csmmaprpc";
      else
        actual_session_type = "csmmapcsm"; // used also when l_cam_type is empty
    }
    if (!input_dem.empty() && actual_session_type == "isis") {
      // User says ISIS but also gives a DEM.
      actual_session_type = "isismapisis";
    }
    if (!input_dem.empty() && actual_session_type == "spot5") {
      // User says SPOT5 but also gives a DEM.
      actual_session_type = "spot5maprpc";
    }
    if (!input_dem.empty() && actual_session_type == "aster") {
      // User says ASTER but also gives a DEM.
      // Mapprojection can happen either with ASTER or RPC cameras 
      std::string cam_tag = "CAMERA_MODEL_TYPE";
      std::string l_cam_type 
        = vw::cartography::read_header_string(left_image_file, cam_tag);
      if (l_cam_type == "aster")
        actual_session_type = "astermapaster";
      else
        actual_session_type = "astermaprpc"; // used also when l_cam_type is empty
    }
    if (!input_dem.empty() && actual_session_type == "pleiades") {
      // User says Pleiades but also gives a DEM.
      actual_session_type = "pleiadesmappleiades";
    }
    
    // Quietly switch from nadirpinhole to pinhole for mapprojected images
    if (!input_dem.empty() && actual_session_type == "nadirpinhole") {
      // User says nadirpinhole but also gives a DEM.
      actual_session_type = "pinholemappinhole";
    }
    
  } // End map promotion section

  if (!input_dem.empty() && actual_session_type == "perusat") {
    // User says PeruSat-1 or Pleiades but also gives a DEM, so the images were mapprojected.
    // If the mapprojection was done with the exact model, stereo becomes
    // painfully slow. If it was done with the RPC model, things become hard
    // to manage, and stereo needs to know both the exact and RPC model
    // and those are in different files. Hence, just don't allow mapprojected
    // images in this case.
    vw_throw(vw::NoImplErr() << "Stereo with mapprojected images and the "
              << "PeruSat-1 linescan model is not implemented. "
              << "Use instead the RPC model.");
  }
  
  // We should know the session type by now.
  VW_ASSERT(!actual_session_type.empty(),
            vw::ArgumentErr() << "Could not determine stereo session type. "
            << "Please set it explicitly with the --session-type option.\n");
  
  if (!total_quiet)
    vw::vw_out() << "Using session: " << actual_session_type << "\n";

  // Compare the current session name to all recognized types
  // - Only one of these will ever get triggered
  StereoSession* session = NULL;
  if (actual_session_type == "dg")
    session = StereoSessionDG::construct();
  else if (actual_session_type == "dgmaprpc")
      session = StereoSessionDGMapRPC::construct();
  else if (actual_session_type == "dgmapdg")
      session = StereoSessionDGMapDG::construct();
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
  else if (actual_session_type == "astermapaster")
      session = StereoSessionASTERMapASTER::construct();
  else if (actual_session_type == "astermaprpc")
      session = StereoSessionASTERMapRPC::construct();
  else if (actual_session_type == "pleiadesmappleiades")
      session = StereoSessionPleiadesMapPleiades::construct();
#if defined(ASP_HAVE_PKG_ISIS) && ASP_HAVE_PKG_ISIS == 1
  else if (actual_session_type == "isis")
    session = StereoSessionIsis::construct();
  else if (actual_session_type == "isismapisis")
    session = StereoSessionIsisMapIsis::construct();
#endif
  else if (actual_session_type == "spot5")
    session = StereoSessionSpot::construct();
  else if (actual_session_type == "perusat")
    session = StereoSessionPeruSat::construct();
  else if (actual_session_type == "pleiades")
    session = StereoSessionPleiades::construct();
  else if (actual_session_type == "aster")
    session = StereoSessionASTER::construct();
  else if (actual_session_type == "opticalbar")
    session = StereoSessionOpticalBar::construct();
  else if (actual_session_type == "csm")
    session = StereoSessionCsm::construct();
  else if (actual_session_type == "csmmapcsm")
    session = StereoSessionCsmMapCsm::construct();
  else if (actual_session_type == "csmmaprpc")
    session = StereoSessionCsmMapRpc::construct();
  if (session == 0)
    vw_throw(vw::NoImplErr() << "Unsupported stereo session type: "
              << actual_session_type);

  session->initialize(options,         // Initialize the new object
                      left_image_file,  right_image_file,
                      left_camera_file, right_camera_file,
                      out_prefix, input_dem);
  session_type = session->name(); // update the session name 

// This is important so that the user does not load RPC cameras while thinking
// ASTER or DG models are loaded. 
if (session_type.find("rpc") != std::string::npos && 
    asp::stereo_settings().aster_use_csm)
    vw_throw(vw::ArgumentErr() 
              << "The --aster-use-csm option must be used only with the "
              << "ASTER session (-t aster).\n");
  
  return session;
} // End function create()

} // end namespace asp
