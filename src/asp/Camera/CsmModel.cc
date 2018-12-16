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


csm::EcefCoord vectorToEcefCoord(Vector3 v) {
  csm::EcefCoord c;
  c.x = v[0];
  c.y = v[1];
  c.z = v[2];
  return c;
}

csm::ImageCoord vectorToImageCoord(Vector2 v) {
  csm::ImageCoord c;
  c.samp = v[0];
  c.line = v[1];
  return c;
}

Vector3 ecefCoordToVector(csm::EcefCoord c) {
  Vector3 v;
  v[0] = c.x;
  v[1] = c.y;
  v[2] = c.z;
  return v;
}

Vector2 imageCoordToVector(csm::ImageCoord c) {
  Vector2 v;
  v[0] = c.samp;
  v[1] = c.line;
  return v;
}


// -----------------------------------------------------------------
// CsmModel class functions

CsmModel::CsmModel() {

  // TODO: Move the loading functionality out of the constructor.
  
  // TODO: Need to build against the base library, automatically detect
  //       the other library.

  // Use either these two together...
  boost::filesystem::path base_dll_path("/home/smcmich1/repo/csm-3.0.3.1/linux64/lib/libcsmapi.so.3");  
  boost::filesystem::path fixture_dll_path("/home/smcmich1/repo/CSM-Swig/install/lib/libfixturecsm.so");

  // Or use these two.
  // - TODO: May want to use base dll + usgs dll but that will require some USGS CMake changes.
  boost::filesystem::path csm_dll_path ("/home/smcmich1/repo/CSM-CameraModel/install/lib/libcsmapi.so.3" );
  boost::filesystem::path usgs_dll_path("/home/smcmich1/repo/CSM-CameraModel/install/lib/libusgscsm.so");

  
  
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
  /*
  boost::dll::library_info fixture_info(fixture_dll_path);
  sections = fixture_info.sections();
  std::cout << "\n\n\n\nFixture sections: \n";
  for (size_t i=0; i<sections.size(); ++i)
    std::cout << " -> " << sections[i] << std::endl;
  
  symbols = fixture_info.symbols();
  std::cout << "\n\nFixture symbols: \n";
  for (size_t i=0; i<symbols.size(); ++i)
    std::cout << " -> " << symbols[i] << std::endl;
  */
  
  // TODO: Find and load all of these only once?
  
  // Get the DLL in memory, causing it to automatically register itself
  //  with the main Plugin interface.
  //boost::dll::shared_library lib_fixture(fixture_dll_path);
  vw_out() << "Loading CSM plugin: " << usgs_dll_path << std::endl;
  boost::dll::shared_library lib_usgs(usgs_dll_path);
  
  /*
  m_csm_plugin = dll::import<csm::Plugin>(
      base_dll_path,
      "Plugin"//,                     // name of the symbol to import
      //dll::load_mode::append_decorations          // append `.so` or `.dll` to the name
  );
  */
/*
  m_csm_plugin = dll::import<csm::Plugin>(
      fixture_dll_path,
      "Plugin"//,                     // name of the symbol to import
      //dll::load_mode::append_decorations          // append `.so` or `.dll` to the name
  );
*/
/*
  m_csm_plugin = dll::import<csm::Plugin>(
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
  
  //std::cout << "Plugin name = " << m_csm_plugin->getPluginName() << std::endl;
  
  // TODO: Make this a setting
  //std::string plugin_folder = "/home/smcmich1/repo/CSM-Swig/install/lib/";
  
  //vw_out() << "Looking for plugins in folder: " << plugin_folder << std::endl;
  
  //csm::Plugin::setDataDirectory(plugin_folder);
  
  csm::PluginList available_plugins = csm::Plugin::getList();
  vw_out() << "Detected " << available_plugins.size() << " CSM plugins.\n";
  
  csm::PluginList::iterator iter;
  for (iter=available_plugins.begin(); iter!=available_plugins.end(); ++iter) {
    std::cout << "  -->  " << (*iter)->getPluginName() << std::endl;
    size_t num_models = (*iter)->getNumModels();
    std::cout << "    - Num models = " << num_models << std::endl;
    for (size_t i=0; i<num_models; ++i) {
      std::cout << "      -> " << (*iter)->getModelName(i)
                << ", family =  " << (*iter)->getModelFamily(i) << std::endl;
    }
  }
  
  // TODO: Mechanism to choose which plugin and sensor model to load.
  
  std::string modelName = "USGS_ASTRO_FRAME_SENSOR_MODEL";
  
  // Load the selected plugin
  m_csm_plugin = available_plugins.front();
  
  std::cout << "Loaded plugin: " << m_csm_plugin->getPluginName() << std::endl;
  
  // TODO: Load ISD data
  Isd imageSupportData;
  
  
  // TODO: Construct a sensor model from the ISD data
/*
  csm::WarningList warnings;
  csm::Model* new_model
    = m_csm_plugin->constructModelFromISD(imageSupportData, modelName, &warnings);
  
  // Display any warnings generated when loading the sensor model.
  csm::WarningList w_iter;
  for (w_iter = warnings.begin(); w_iter!=warnings.end(); ++w_iter) {
    vw_out() << "CSM Warning: " << w_iter->getMessage() << std::endl;
  }
  if (!new_model) // Handle load failure.
    vw::vw_throw( vw::ArgumentErr() << "Failed to load CSM sensor model!");
  
  // TODO: Are all sensor models going to be this type?
  //       Otherwise we can use the result of getModelFamily() to choose the class.
  // Cast the model we got to the child class with the needed functionality.
  csm::RasterGM* raster_model = dynamic_cast<csm::RasterGM*>(new_model);
  if (!raster_model) // Handle load failure.
    vw::vw_throw( vw::ArgumentErr() << "Failed to cast CSM sensor model to raster type!");
    
  m_csm_model.reset(raster_model); // We will handle cleanup of the model.
*/
  
}

CsmModel::~CsmModel() {
  // Don't need to do any cleanup here!
}
/*
bool CsmModel::load_model(std::string const& isd_path) {

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
void CsmModel::throw_if_not_init() const {
  if (!m_csm_model)
    vw_throw( ArgumentErr() << "CsmModel: Sensor model has not been loaded yet!" );
}

// TODO: Check the warnings

Vector2 CsmModel::point_to_pixel (Vector3 const& point) const {
  throw_if_not_init();

  csm::EcefCoord  ecef    = vectorToEcefCoord(point);
  csm::ImageCoord imagePt = m_csm_model->groundToImage(ecef);
    //double desiredPrecision = 0.001,
  //double* achievedPrecision = NULL,
  //WarningList* warnings = NULL)
  
  return imageCoordToVector(imagePt);
}

Vector3 CsmModel::pixel_to_vector(Vector2 const& pix) const {
  throw_if_not_init();

  csm::ImageCoord imagePt = vectorToImageCoord(pix);

  // This function generates the vector from the camera at the camera origin,
  //  there is a different call that gets the vector near the ground.
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
  csm::EcefCoord  ecef    = m_csm_model->getSensorPosition(imagePt);

  return ecefCoordToVector(ecef);
}
/*
Quaternion<double> CsmModel::camera_pose(Vector2 const& pix) const {
  throw_if_not_init();
  
  return Quaternion<double>();
}
*/

} // end namespace asp

