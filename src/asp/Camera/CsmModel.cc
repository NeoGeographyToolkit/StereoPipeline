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



#include <asp/Core/StereoSettings.h>

#include <asp/Camera/CsmModel.h>

// TODO: Check
#include <csm/Plugin.h>
#include <boost/dll.hpp>


namespace dll = boost::dll;

using namespace vw;

namespace asp {

// -----------------------------------------------------------------
// Helper functions
  
EcefCoord vectorToEcefCoord(Vector3 v) {
  EcefCoord c;
  c.x = v[0];
  c.y = v[1];
  c.z = v[2];
  return c;
}

ImageCoord vectorToImageCoord(Vector2 v) {
  ImageCoord c;
  c.sample = v[0];
  c.line   = v[1];
  return c;
}

Vector3 ecefCoordToVector(EcefCoord c) {
  Vector3 v;
  v[0] = c.x;
  v[1] = c.y;
  v[2] = c.z;
  return c;
}

Vector2 imageCoordToVector(ImageCoord c) {
  Vector2 v;
  v[0] = c.sample;
  v[1] = c.line;
  return v;
}


// -----------------------------------------------------------------
// CsmModel class functions

CsmModel::CsmModel() {

  // TODO: Fix!
  boost::filesystem::path csm_dll_path ("/home/smcmich1/repo/CSM-CameraModel/install/lib/libcsmapi.so" );
  boost::filesystem::path usgs_dll_path("/home/smcmich1/repo/CSM-CameraModel/install/lib/libusgscsm.so");
  
  boost::filesystem::path fixture_dll_path("/home/smcmich1/repo/CSM-Swig/install/lib/libfixturecsm.so");
  
  std::vector<std::string> sections, symbols;
  /*
  boost::dll::library_info csm_info  (csm_dll_path);
  boost::dll::library_info usgs_info(usgs_dll_path);
  sections = csm_info.sections();
  std::cout << "CSM sections: \n";
  for (size_t i=0; i<sections.size(); ++i)
    std::cout << " -> " << sections[i] << std::endl;
  
  symbols = csm_info.symbols();
  std::cout << "\n\nCSM symbols: \n";
  for (size_t i=0; i<symbols.size(); ++i)
    std::cout << " -> " << symbols[i] << std::endl;
  
  sections = usgs_info.sections();
  std::cout << "\n\n\n\nUSGS sections: \n";
  for (size_t i=0; i<sections.size(); ++i)
    std::cout << " -> " << sections[i] << std::endl;
  
  symbols = usgs_info.symbols();
  std::cout << "\n\nUSGS symbols: \n";
  for (size_t i=0; i<symbols.size(); ++i)
    std::cout << " -> " << symbols[i] << std::endl;
  */
  
  boost::dll::library_info fixture_info(fixture_dll_path);
  sections = fixture_info.sections();
  std::cout << "\n\n\n\nFixture sections: \n";
  for (size_t i=0; i<sections.size(); ++i)
    std::cout << " -> " << sections[i] << std::endl;
  
  // TODO: All of the libs need to be on LD_LIBRARY_PATH or equivalent!
  
/*
  csm_plugin = dll::import<csm::Plugin>(
      csm_dll_path,
      ""//,                     // name of the symbol to import
      //dll::load_mode::append_decorations          // append `.so` or `.dll` to the name
  );
*/
/*
  m_csm_plugin = dll::import<csm::Plugin>(
      usgs_dll_path,
      "UsgsAstroFramePlugin"//,                     // name of the symbol to import
      //dll::load_mode::append_decorations          // append `.so` or `.dll` to the name
  );
  */
  
  std::cout << "Plugin name = " << m_csm_plugin->getPluginName() << std::endl;
  
  
  //m_csm_pointer
  
}

CsmModel::~CsmModel() {
  
}
/*
bool load_model(std::string const& isd_path) {

  isd_path
  Isd isd_data;
  const std::string model_name = "TODO";
  
  m_csm_pointer = m_csm_plugin->constructModelFromISD(isd_data, model_name);
  if (!m_csm_pointer) {
    vw_out() << "Failed to construct CSM model from ISD file: " << isd_path << std::endl;
    return false;
  }
}
*/
void throw_if_not_init() const {
  if (!m_csm_pointer)
    vw_throw( ArgumentErr() << "CsmModel: Sensor model has not been loaded yet!" );
}

Vector2 CsmModel::point_to_pixel (Vector3 const& point) const {
  throw_if_not_init();

  csm::EcefCoord ecef = vectorToEcefCoord(point);
  ImageCoord imagePt = m_csm_pointer->groundToImage(ecef);
  
  return imageCoordToVector(imagePt);
}

Vector3 CsmModel::pixel_to_vector(Vector2 const& pix) const {
  throw_if_not_init();

  csm::ImageCoord imagePt = vectorToImageCoord(pix);

  csm::EcefLocus locus = imageToRemoteImagingLocus(imagePt);
      //double desiredPrecision = 0.001,
      //double* achievedPrecision = NULL,
      //WarningList* warnings = NULL)

  Vector3 dir = ecefCoordToVector(locus.direction);
  return dir;
}

Vector3 CsmModel::camera_center(Vector2 const& pix) const {
  throw_if_not_init();
  
  csm::ImageCoord imagePt = vectorToImageCoord(pix);
  csm::EcefCoord  ecef    = m_csm_pointer->getSensorPosition(imagePt);
  
  return ecefCoordToVector(ecef);
}
/*
Quaternion<double> CsmModel::camera_pose(Vector2 const& pix) const {
  throw_if_not_init();
  
  return Quaternion<double>();
}
*/

} // end namespace asp

