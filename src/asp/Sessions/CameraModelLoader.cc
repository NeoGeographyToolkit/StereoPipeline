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


/// \file CameraModelLoader.cc
///

#include <asp/Core/Common.h>
#include <asp/Core/StereoSettings.h>
#include <asp/Camera/CsmModel.h>
#include <asp/Camera/LinescanDGModel.h>
#include <asp/Camera/LinescanSpotModel.h>
#include <asp/Camera/LinescanPeruSatModel.h>
#include <asp/Camera/LinescanPleiadesModel.h>
#include <asp/Camera/LinescanASTERModel.h>
#include <asp/Sessions/CameraModelLoader.h>
#include <asp/Camera/RPCModel.h>
#include <asp/Camera/RPC_XML.h>

#if defined(ASP_HAVE_PKG_ISIS) && ASP_HAVE_PKG_ISIS == 1
#include <asp/IsisIO/Equation.h>
#include <asp/IsisIO/IsisCameraModel.h>
#endif // ASP_HAVE_PKG_ISIS

#include <vw/Camera/Extrinsics.h>
#include <vw/Camera/CameraUtilities.h>
#include <vw/Core/Exception.h>
#include <vw/Core/Log.h>
#include <vw/FileIO/FileUtils.h>
#include <vw/Math/EulerAngles.h>
#include <vw/Math/Matrix.h>
#include <vw/Camera/OpticalBarModel.h>
#include <vw/Core/Stopwatch.h>
#include <vw/FileIO/FileTypes.h>

#include <xercesc/util/PlatformUtils.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>

#include <map>
#include <utility>
#include <string>
#include <ostream>
#include <limits>

namespace asp {

CameraModelLoader::CameraModelLoader() {
  xercesc::XMLPlatformUtils::Initialize();
}

CameraModelLoader::~CameraModelLoader() {
  xercesc::XMLPlatformUtils::Terminate();
}

boost::shared_ptr<vw::camera::CameraModel>
CameraModelLoader::load_pinhole_camera_model(std::string const& path) const {
  return vw::camera::load_pinhole_camera_model(path);
}

// Load an RPC camera file
// - TODO: Move to another file
boost::shared_ptr<vw::camera::CameraModel>
CameraModelLoader::load_rpc_camera_model(std::string const& path) const {
  
  // Try the default loading method
  RPCModel* rpc_model = NULL;
  try {
    RPCXML rpc_xml; // This is for reading XML files
    rpc_xml.read_from_file(path);
    rpc_model = new RPCModel(*rpc_xml.rpc_ptr()); // Copy the value
  } catch (...) {
  }
  if (!rpc_model) { // The default loading method failed, try the backup method.
    rpc_model = new RPCModel(path); // Invoke the GDAL driver for the image file
  }
  
  // We don't catch an error here because the user will need to
  // know of a failure at this point.
  return boost::shared_ptr<asp::RPCModel>(rpc_model);
}

// Load a DG camera file
boost::shared_ptr<vw::camera::CameraModel>
CameraModelLoader::load_dg_camera_model(std::string const& path) const {
  // Redirect to the call from LinescanDGModel.h file
  return vw::CamPtr(load_dg_camera_model_from_xml(path));
}

// Load a spot5 camera file
boost::shared_ptr<vw::camera::CameraModel>
CameraModelLoader::load_spot5_camera_model(std::string const& path) const {
  vw::vw_out() << "Loading a SPOT5 camera model can take a few minutes.\n";
  vw::Stopwatch sw;
  sw.start();
  vw::CamPtr ans = load_spot5_camera_model_from_xml(path);
  sw.stop();
  vw::vw_out() << "Load time: " << sw.elapsed_seconds() << " seconds.\n";

  return ans;
}

// Load a PeruSat linescan camera file
boost::shared_ptr<vw::camera::CameraModel>
CameraModelLoader::load_perusat_camera_model(std::string const& path) const {
  return vw::CamPtr(load_perusat_camera_model_from_xml(path));
}

// Load a Pleiades linescan camera file
boost::shared_ptr<vw::camera::CameraModel>
CameraModelLoader::load_pleiades_camera_model(std::string const& path) const {
  return vw::CamPtr(load_pleiades_camera_model_from_xml(path));
}

// Load a ASTER camera file
boost::shared_ptr<vw::camera::CameraModel>
CameraModelLoader::load_ASTER_camera_model(std::string const& path) const {
  // This model file also needs the RPC model as an initial guess
  boost::shared_ptr<vw::camera::CameraModel> rpc_model = load_rpc_camera_model(path);

  return vw::CamPtr(load_ASTER_camera_model_from_xml(path, rpc_model));
}

// Load an ISIS camera model
boost::shared_ptr<vw::camera::CameraModel>
CameraModelLoader::load_isis_camera_model(std::string const& path) const {
#if defined(ASP_HAVE_PKG_ISIS) && ASP_HAVE_PKG_ISIS == 1
  return vw::CamPtr(new vw::camera::IsisCameraModel(path));
#endif
  // If ISIS was not enabled in the build, just throw an exception.
  vw::vw_throw(vw::NoImplErr()
               << "Cannot load ISIS files because ISIS was not enabled in the build.\n");

} // End function load_isis_camera_model()

// Load an optical bar camera file
boost::shared_ptr<vw::camera::CameraModel>
CameraModelLoader::load_optical_bar_camera_model(std::string const& path) const {
  return vw::CamPtr(new vw::camera::OpticalBarModel(path));
}

// Load a CSM camera file
boost::shared_ptr<vw::camera::CameraModel> 
CameraModelLoader::load_csm_camera_model(std::string const& path) const {
  
  boost::shared_ptr<asp::CsmModel> cam_ptr(new asp::CsmModel());
  if (vw::has_isd_extension(path)) {
    // Load a .json file
    cam_ptr->load_model(path);
    return vw::CamPtr(cam_ptr);
  }
  
  // The CSM model is embedded in the cub file
#if defined(ASP_HAVE_PKG_ISIS) && ASP_HAVE_PKG_ISIS == 1
  std::string modelState = asp::isis::csmStateFromIsisCube(path);
  bool recreateModel = true; 
  cam_ptr->setModelFromStateString(modelState, recreateModel);
#endif // ASP_HAVE_PKG_ISIS

  return vw::CamPtr(cam_ptr);
}

} // end namespace asp
