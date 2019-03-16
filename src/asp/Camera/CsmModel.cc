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


#include <vw/FileIO/FileUtils.h>

#include <asp/Core/StereoSettings.h>
#include <asp/Camera/CsmModel.h>

#include <boost/dll.hpp>
#include <boost/filesystem.hpp>


// From the CSM base interface library
#include <csm/csm.h>
#include <csm/Plugin.h>
#include <csm/RasterGM.h>

namespace dll = boost::dll;

using namespace vw;

namespace asp {

vw::Mutex csm_init_mutex;


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

Vector3 ecefVectorToVector(csm::EcefVector c) {
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
}

CsmModel::CsmModel(std::string const& isd_path) {
  load_model(isd_path);
}

CsmModel::~CsmModel() {
  // Don't need to do any cleanup here!
}

bool CsmModel::file_has_isd_extension(std::string const& path) {
  std::string ext = vw::get_extension(path);
  return ((ext == ".json") || (ext == ".isd"));
}

std::string CsmModel::get_csm_plugin_folder(){

  // Look up the CSM_PLUGIN_PATH environmental variable.
  // - It is set in the "libexec/libexec-funcs.sh" deployed file.
  char * data_folder = getenv("CSM_PLUGIN_PATH");
  if (data_folder == NULL){
    vw_throw( ArgumentErr() << "The environmental variable CSM_PLUGIN_PATH was not set. "
              << "It should point to the 'share' directory of your ASP distribution.\n" );
  }

  std::string full_path = std::string(data_folder);
  if (!boost::filesystem::exists(full_path)){
    vw_throw( ArgumentErr() << "Could not find CSM plugin folder: " << full_path << ".\n"
              << "Check the value of environmental variable CSM_PLUGIN_PATH.");
  }

  return full_path;
}


size_t CsmModel::find_csm_plugins(std::vector<std::string> &plugins) {
  plugins.clear();
  const std::string folder = get_csm_plugin_folder();

  size_t num_dlls = vw::get_files_in_folder(folder, plugins, ".so");
  if (num_dlls == 0) // Try again using the Mac extension.
    num_dlls = vw::get_files_in_folder(folder, plugins, ".dylib");

  for (size_t i=0; i<num_dlls; ++i) {
    boost::filesystem::path p(folder);
    p /= plugins[i];
    plugins[i] = p.string();
  }
  
  return num_dlls;
}


void CsmModel::print_available_models() {

  csm::PluginList available_plugins = csm::Plugin::getList();
  vw_out() << "Detected " << available_plugins.size() << " available CSM plugin(s).\n";

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
}



// This function is not kept out of the header file to hide CSM dependencies.
/// Look through all of the loaded plugins and find one that is compatible with
///  the provided ISD.
const csm::Plugin* find_plugin_for_isd(csm::Isd const& support_data,
                                 std::string   & model_name,
                                 std::string   & model_family,
                                 bool            show_warnings) {

  // Loop through the available plugins.
  csm::PluginList::iterator iter;
  csm::PluginList available_plugins = csm::Plugin::getList();
  for (iter=available_plugins.begin(); iter!=available_plugins.end(); ++iter) {
    const csm::Plugin* csm_plugin = (*iter);

    // For each plugin, loop through the available models.
    size_t num_models = csm_plugin->getNumModels();
    for (size_t i=0; i<num_models; ++i) {
      std::string this_model_name = (*iter)->getModelName(i);

      // Check if we can construct a camera with the ISD and this plugin/model.
      csm::WarningList warnings;
      csm::WarningList::const_iterator w_iter;
      if (csm_plugin->canModelBeConstructedFromISD(support_data, this_model_name, &warnings)) {
        model_name   = this_model_name;
        model_family = csm_plugin->getModelFamily(i);
        return csm_plugin; // Found a plugin that will work!
      }
      // Optionally print the reasons why we could not load it.
      if (show_warnings)
        for (w_iter = warnings.begin(); w_iter!=warnings.end(); ++w_iter) {
          vw_out() << "CSM Warning: " << w_iter->getMessage() << std::endl;
        }
    } // End loop through models
  } // End loop through plugins

  // Did not find a match!
  model_name   = "";
  model_family = "";
  return 0;
} // End function find_plugin_for_isd


void CsmModel::initialize_plugins() {

  // Only let one thread at a time in here.
  vw::Mutex::Lock lock(csm_init_mutex);

  // If we already have plugins loaded, don't do initialization again.
  csm::PluginList available_plugins = csm::Plugin::getList();
  if (!available_plugins.empty())
    return;

  vw_out() << "Initializing CSM plugins...\n";
  
  // Find all of the available CSM plugin DLL files.
  std::vector<std::string> plugin_files;
  size_t num_plugin_files = find_csm_plugins(plugin_files);
  vw_out() << "Found " << num_plugin_files << " CSM plugin files.\n";

  // Load all of the plugins.
  for (size_t i=0; i<num_plugin_files; ++i) {
    // Get the DLL in memory, causing it to automatically register itself
    //  with the main Plugin interface.
    //boost::dll::shared_library lib_fixture(fixture_dll_path);
    vw_out() << "Loading CSM plugin: " << plugin_files[i] << std::endl;
    boost::dll::shared_library lib_usgs(plugin_files[i]);
  }

  //csm::Plugin::setDataDirectory(plugin_folder);

  print_available_models();
}


bool CsmModel::load_model(std::string const& isd_path) {

/*
  // Use either these two together...
  boost::filesystem::path base_dll_path("/home/smcmich1/repo/csm-3.0.3.1/linux64/lib/libcsmapi.so.3");  
  boost::filesystem::path fixture_dll_path("/home/smcmich1/repo/CSM-Swig/install/lib/libfixturecsm.so");

  // Or use these two.
  // - TODO: May want to use base dll + usgs dll but that will require some USGS CMake changes.
  boost::filesystem::path csm_dll_path ("/home/smcmich1/repo/CSM-CameraModel/install/lib/libcsmapi.so.3" ); // -> This is set in the CMakeLists file.
  boost::filesystem::path usgs_dll_path("/home/smcmich1/repo/CSM-CameraModel/install/lib/libusgscsm.so"); // -> This is dynamically loaded.
 */

  // This only happens the first time it is called.
  initialize_plugins();

  // Load ISD data
  csm::Isd support_data(isd_path);

  // Check each available CSM plugin until we find one that can handle the ISD.
  std::string model_name, model_family;
  const csm::Plugin* csm_plugin = find_plugin_for_isd(support_data, model_name, model_family, false);

  // If we did not find a plugin that would work, go through them again and print error
  //  messages for each plugin that fails.
  if (csm_plugin == 0) {
    find_plugin_for_isd(support_data, model_name, model_family, true);
    vw::vw_throw( vw::ArgumentErr() << "Unable to construct a camera model for the ISD file "
                        << isd_path << " using any of the loaded CSM plugins!");
  }

  vw_out() << "Using plugin: " << csm_plugin->getPluginName() 
           << " with model name " << model_name << std::endl;

  // Now try to construct the camera model
  csm::WarningList warnings;
  csm::Model* new_model
    = csm_plugin->constructModelFromISD(support_data, model_name, &warnings);

  // Error checking
  csm::WarningList::const_iterator w_iter;
  for (w_iter = warnings.begin(); w_iter!=warnings.end(); ++w_iter) {
    vw_out() << "CSM Warning: " << w_iter->getMessage() << std::endl;
  }
  if (!new_model) // Handle load failure.
    vw::vw_throw( vw::ArgumentErr() << "Failed to load CSM sensor model from file: "
                                    << isd_path);

  // TODO: Are all sensor models going to be this type?
  //       Otherwise we can use the result of getModelFamily() to choose the class.
  // Cast the model we got to the child class with the needed functionality.
  csm::RasterGM* raster_model = dynamic_cast<csm::RasterGM*>(new_model);
  if (!raster_model) // Handle load failure.
    vw::vw_throw( vw::ArgumentErr() << "Failed to cast CSM sensor model to raster type!");

  m_csm_model.reset(raster_model); // We will handle cleanup of the model.
  std::cout << "Done setting up the CSM model\n";
}

void CsmModel::throw_if_not_init() const {
  if (!m_csm_model)
    vw_throw( ArgumentErr() << "CsmModel: Sensor model has not been loaded yet!" );
}

// TODO: Check the warnings


vw::Vector2 CsmModel::get_image_size() const {
  throw_if_not_init();

  csm::ImageVector size = m_csm_model->getImageSize();
  return Vector2(size.samp, size.line);
}

Vector2 CsmModel::point_to_pixel (Vector3 const& point) const {
  throw_if_not_init();

  csm::EcefCoord  ecef    = vectorToEcefCoord(point);
  
  double desiredPrecision = 0.01;
  double achievedPrecision;
  csm::WarningList warnings;
  std::cout << "call\n";
  csm::ImageCoord imagePt = m_csm_model->groundToImage(ecef, desiredPrecision,
                                                       &achievedPrecision,
                                                       &warnings);

  csm::WarningList::const_iterator w_iter;
  for (w_iter = warnings.begin(); w_iter!=warnings.end(); ++w_iter) {
    vw_out() << "CSM Warning: " << w_iter->getMessage() << std::endl;
  }

  return imageCoordToVector(imagePt);
}

Vector3 CsmModel::pixel_to_vector(Vector2 const& pix) const {
  throw_if_not_init();

  csm::ImageCoord imagePt = vectorToImageCoord(pix);

  // This function generates the vector from the camera at the camera origin,
  //  there is a different call that gets the vector near the ground.
  csm::EcefLocus locus = m_csm_model->imageToRemoteImagingLocus(imagePt);
      //double desiredPrecision = 0.001,
      //double* achievedPrecision = NULL,
      //WarningList* warnings = NULL)

  Vector3 dir = ecefVectorToVector(locus.direction);
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

