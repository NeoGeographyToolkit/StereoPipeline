// __BEGIN_LICENSE__
//  Copyright (c) 2009-2012, United States Government as represented by the
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


// Wholesale copy from orthoproject.cc. Longer term, this needs to be dealt with properly.



/// \file reconstruct_aux.cc
///
#include <iostream>
#include <string>
#include <fstream>
#include <vector>

#include <vw/Camera.h>
#include <vw/Cartography.h>
#include <vw/Core.h>
#include <vw/FileIO.h>
#include <vw/Image.h>
#include <vw/Math.h>
#include <vw/Math/Functors.h>
#include <vw/Photometry.h>
using namespace std;
using namespace vw::camera;
using namespace vw::cartography;
using namespace vw::math;
using namespace vw::photometry;
using namespace vw;

#include <asp/Core/Macros.h>
#include <asp/Core/Common.h>
#include <asp/Sessions.h>
#include <asp/IsisIO/DiskImageResourceIsis.h>
#include <asp/IsisIO/IsisCameraModel.h>
#include <asp/IsisIO/IsisAdjustCameraModel.h>
#include <boost/tokenizer.hpp>
namespace po = boost::program_options;
namespace fs = boost::filesystem;
using namespace std;
struct Options : asp::BaseOptions {
  Options(){}
  std::string image_file, camera_model_file, stereo_session;
};

int extractDRGFromCube(bool useDEMTiles, double metersPerPixel, std::string DEMTilesDir, std::string DEMFile,
                      std::string cubeFile, std::string isis_adjust_file, std::string outputDrgFile,
                      Vector3 & sunPosition, Vector3 & spacecraftPosition
                      ){

  // Extract the sun/spacecraft position from the cube. If, in
  // addition, the file isis_adjust_file is specified, extract
  // the adjusted spacecraft position from that file instead.
  
  Options opt;
  try {

    if ( !fs::exists(isis_adjust_file) ){
      std::cout << "WARNING: The ISIS adjust file " << isis_adjust_file
                << " is missing, will extract the unadjusted spacecraft position from the cube file "
                << cubeFile << std::endl;
      isis_adjust_file = cubeFile;
    }

    opt.image_file        = cubeFile;
    opt.camera_model_file = isis_adjust_file;

    // Create a fresh stereo session and query it for the camera models.
    asp::StereoSession::register_session_type("rmax", &asp::StereoSessionRmax::construct);

#if defined(ASP_HAVE_PKG_ISISIO) && ASP_HAVE_PKG_ISISIO == 1
    asp::StereoSession::register_session_type("isis", &asp::StereoSessionIsis::construct);
    
    opt.stereo_session = "isis";

    // Okay, here's a total hack.  We create a stereo session where both
    // of the imagers and images are the same, because we want to take
    // advantage of the stereo pipeline's ability to generate camera
    // models for various missions.  Hence, we create two identical
    // camera models, but only one is used.  The last empty strings
    // are dummy arguments.
    typedef boost::scoped_ptr<asp::StereoSession> SessionPtr;
    SessionPtr session( asp::StereoSession::create(opt.stereo_session) );
    session->initialize(opt, opt.image_file, opt.image_file,
                        opt.camera_model_file, opt.camera_model_file,
                        "", "","","","" );
    boost::shared_ptr<camera::CameraModel> camera_model;
    camera_model = session->camera_model(opt.image_file, opt.camera_model_file);
    spacecraftPosition = camera_model->camera_center(Vector2());
    
    vw::camera::IsisCameraModel M(opt.image_file);
    sunPosition = M.sun_position();

    std::cout << "sun and spacecraft positions are: " << sunPosition << ' ' << spacecraftPosition << std::endl;
    
#else
    std::cout << "ERROR: The ISIS package is missing. Cannot extract " <<
      "sun and spacecraft position from an ISIS cube file. " << std::endl;
    exit(1);
#endif
    
  } ASP_STANDARD_CATCHES;
  
  return 0;
}


