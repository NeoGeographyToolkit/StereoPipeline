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

#include <vw/Camera.h>
#include <vw/Camera/Extrinsics.h>
#include <vw/Core/Exception.h>
#include <vw/Core/Log.h>
#include <vw/Math/EulerAngles.h>
#include <vw/Math/Matrix.h>
#include <xercesc/util/PlatformUtils.hpp>

#include <asp/Core/Common.h>
#include <asp/Core/StereoSettings.h>
#include <asp/IsisIO/Equation.h>
#include <asp/IsisIO/IsisCameraModel.h>
#include <asp/Camera/LinescanDGModel.h>
#include <asp/Sessions/CameraModelLoader.h>
#include <asp/Camera/RPCModel.h>
#include <boost/date_time/posix_time/posix_time.hpp>

#include <map>
#include <utility>
#include <string>
#include <ostream>
#include <limits>

// TODO: Break this up. Each of these functions must go back to their
// individual Session directories rather than being collected here.

namespace asp {

CameraModelLoader::CameraModelLoader()
{
  xercesc::XMLPlatformUtils::Initialize();
}

CameraModelLoader::~CameraModelLoader()
{
  xercesc::XMLPlatformUtils::Terminate();
}


/// Load an RPC camera file
boost::shared_ptr<vw::camera::CameraModel> CameraModelLoader::load_rpc_camera_model(std::string const& path) const
{
  // Try the default loading method
  RPCModel* rpc_model = NULL;
  try {
    RPCXML rpc_xml; // This is for reading XML files
    rpc_xml.read_from_file(path);
    rpc_model = new RPCModel(*rpc_xml.rpc_ptr()); // Copy the value
  } catch (...) {}
  if (!rpc_model) // The default loading method failed, try the backup method.
  {
    rpc_model = new RPCModel(path); // This is for reading .tif files
  }

  // We don't catch an error here because the user will need to
  // know of a failure at this point.
  return boost::shared_ptr<asp::RPCModel>(rpc_model);
}


/// Load a DG camera file
boost::shared_ptr<vw::camera::CameraModel> CameraModelLoader::load_dg_camera_model(std::string const& path) const
{
  bool correct_velocity_aberration = !stereo_settings().disable_correct_velocity_aberration;

  // Redirect to the call from LinescanDGModel.h file
  return CameraModelPtr(load_dg_camera_model_from_xml(path, correct_velocity_aberration));
}





/// Load an ISIS camera model
boost::shared_ptr<vw::camera::CameraModel> CameraModelLoader::load_isis_camera_model(std::string const& path) const
{
#if defined(ASP_HAVE_PKG_ISISIO) && ASP_HAVE_PKG_ISISIO == 1
  return CameraModelPtr(new vw::camera::IsisCameraModel(path));
#endif
  // If ISIS was not enabled in the build, just throw an exception.
  vw::vw_throw( vw::NoImplErr() << "\nCannot load ISIS files because ISIS was not enabled in the build!.\n");

} // End function load_isis_camera_model()

} // end namespace asp
