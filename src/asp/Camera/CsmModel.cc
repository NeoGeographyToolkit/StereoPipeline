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

#include <asp/Camera/CsmModel.h>
#include <asp/Camera/CsmUtils.h>
#include <asp/Core/StereoSettings.h>
#include <asp/Core/ProjectiveCamApprox.h>

#include <vw/Camera/PinholeModel.h>
#include <vw/Camera/LensDistortion.h>

#include <boost/dll.hpp>
#include <boost/dll/runtime_symbol_info.hpp>
#include <boost/filesystem.hpp>
#include <boost/algorithm/string/case_conv.hpp>
#include <boost/version.hpp>
#include <boost/config.hpp>

// From the CSM base interface library
#include <csm/csm.h>
#include <csm/Plugin.h>
#include <csm/RasterGM.h>
#include <nlohmann/json.hpp>

// USGSCSM linescan
#include <usgscsm/UsgsAstroFrameSensorModel.h>
#include <usgscsm/UsgsAstroLsSensorModel.h>
#include <usgscsm/UsgsAstroPushFrameSensorModel.h>
#include <usgscsm/UsgsAstroSarSensorModel.h>
#include <usgscsm/Utilities.h>

#include <ale/Rotation.h>
#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <streambuf>

namespace dll = boost::dll;
namespace fs = boost::filesystem;
using json = nlohmann::json;

using namespace vw;

namespace asp {

// This was discussed with the USGS folks. To convert from ISIS to ASP
// pixels we subtract 1.0. To convert from CSM pixels we have to
// subtract only 0.5.
const vw::Vector2 ASP_TO_CSM_SHIFT(0.5, 0.5);

enum USGSCSM_MODEL_TYPE {
  USGSCSM_FRAME_MODEL,
  USGSCSM_LINESCAN_MODEL,
  USGSCSM_PUSHFRAME_MODEL,
  USGSCSM_SAR_MODEL
};

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

// Auxiliary functions to convert a pixel from ASP conventions to what CSM
// expects and vice versa
void toCsmPixel(vw::Vector2 const& pix, csm::ImageCoord & csm) {
  csm.samp = pix[0] + ASP_TO_CSM_SHIFT[0];
  csm.line = pix[1] + ASP_TO_CSM_SHIFT[1];
}
void fromCsmPixel(vw::Vector2 & pix, csm::ImageCoord const& csm) {
  pix[0] = csm.samp - ASP_TO_CSM_SHIFT[0];
  pix[1] = csm.line - ASP_TO_CSM_SHIFT[1];
}

Vector3 ecefCoordToVector(csm::EcefCoord const& c) {
  Vector3 v;
  v[0] = c.x;
  v[1] = c.y;
  v[2] = c.z;
  return v;
}

Vector3 ecefVectorToVector(csm::EcefVector const& c) {
  Vector3 v;
  v[0] = c.x;
  v[1] = c.y;
  v[2] = c.z;
  return v;
}

Vector2 imageCoordToVector(csm::ImageCoord const& c) {
  Vector2 v;
  v[0] = c.samp;
  v[1] = c.line;
  return v;
}

// -----------------------------------------------------------------
// Constructor
CsmModel::CsmModel():m_semi_major_axis(0.0),
                     m_semi_minor_axis(0.0),
                     m_sun_position(vw::Vector3()),
                     // Do not make the precision lower than 1e-8. CSM can give
                     // junk results when it is too low.
                     m_desired_precision(asp::DEFAULT_CSM_DESIRED_PRECISION),
                     m_maxApproxCamPixErr(-1.0) {
}
                                      
// Call the default constructor to initalize the member variables, then load
// from file.
CsmModel::CsmModel(std::string const& isd_path): CsmModel() {
  load_model(isd_path);
}

// Note: This class copy constructor is shallow. To make a deep copy
// use the deep_copy() function.

CsmModel::~CsmModel() {
  // nothing to do.
}

std::string CsmModel::get_csm_plugin_folder() {
  // Look up the CSM_PLUGIN_PATH environmental variable.
  // It is set in the "libexec/libexec-funcs.sh" deploy file.
  // If the plugin is not found in CSM_PLUGIN_PATH, look at ISISROOT.
  std::string plugin_path;
  char * plugin_path_arr = getenv("CSM_PLUGIN_PATH");

  char * isis_root = getenv("ISISROOT");
  if (isis_root == NULL)
    vw_throw(vw::ArgumentErr() << "The variable ISISROOT was not set.\n");
  
  if (plugin_path_arr != NULL && std::string(plugin_path_arr) != ""){
    plugin_path = std::string(plugin_path_arr);

  }else{
    // This is for when ASP is installed without the deploy file.
    // vw_out() << "The environmental variable CSM_PLUGIN_PATH was not set.\n";
    fs::path try_path(isis_root);
    try_path /= "lib";
    plugin_path = try_path.string();
    //vw_out() << "Looking in " << plugin_path << ".\n";
  }

  if (!fs::exists(plugin_path)){
    vw_throw(ArgumentErr() << "Could not find CSM plugin folder: " << plugin_path << ".\n"
              << "Check the value of the environmental variable CSM_PLUGIN_PATH.");
  }

  return plugin_path;
}

// The original idea here was to look at every library in the plugins
// directory and load the valid plugins. For now however there is just
// one plugin, usgscsm, and it is stored in 'lib', among thousands
// of other inapplicable libraries. Hence just pick that one.  One day
// we will have a dedicated plugins directory.
size_t CsmModel::find_csm_plugins(std::vector<std::string> &plugins) {

  plugins.clear();
  const std::string folder = get_csm_plugin_folder();
  std::string ext;
  std::vector<std::string> potential_plugins;
  std::string platform = std::string(BOOST_PLATFORM);
  boost::to_lower(platform);
  if (std::string(platform).find("linux") != std::string::npos)
    ext = ".so";
  else if (std::string(platform).find("mac") != std::string::npos) 
    ext = ".dylib";
  else
    vw_throw(ArgumentErr() << "Unknown operating system: " << BOOST_PLATFORM << "\n");

#if 0
  size_t potential_num_dlls = vw::get_files_in_folder(folder, potential_plugins, ext);
  for (size_t i = 0; i < potential_num_dlls; i++) {
    if (potential_plugins[i] != "libusgscsm" + ext) {
      continue;
    }
    
    fs::path p(folder);
    p /= potential_plugins[i];
    plugins.push_back(p.string());
  }
#endif

  fs::path p(folder);
  p /= "libusgscsm" + ext;
  std::string plugin = p.string();
  if (!fs::exists(plugin)) 
    vw_throw(ArgumentErr() << "Cannot find plugin: " <<plugin <<
              ". Set CSM_PLUGIN_PATH to the directory where the plugins are stored.\n");
  plugins.push_back(plugin);

  return plugins.size();
}

void CsmModel::print_available_models() {
  csm::PluginList available_plugins = csm::Plugin::getList();
  // vw_out() << "Detected " << available_plugins.size() << " available CSM plugin(s).\n";

  csm::PluginList::iterator iter;
  for (iter = available_plugins.begin(); iter != available_plugins.end(); iter++) {
    vw_out() << "  -->  " << (*iter)->getPluginName() << std::endl;
    size_t num_models = (*iter)->getNumModels();
    vw_out() << "    - Num models = " << num_models << std::endl;
    for (size_t i = 0; i < num_models; i++) {
      vw_out() << "      -> " << (*iter)->getModelName(i)
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
  csm::PluginList plugins = csm::Plugin::getList();
  for (iter = plugins.begin(); iter != plugins.end(); iter++) {
    const csm::Plugin* csm_plugin = (*iter);

    // For each plugin, loop through the available models.
    size_t num_models = csm_plugin->getNumModels();
    for (size_t i = 0; i < num_models; i++) {

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
          vw_out() << "CSM warning: " << w_iter->getMessage() << std::endl;
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
  csm::PluginList plugins = csm::Plugin::getList();
  if (!plugins.empty())
    return;
  
  //vw_out() << "Initializing CSM plugins...\n";

  // Find all of the available CSM plugin DLL files.
  std::vector<std::string> plugin_files;
  size_t num_plugin_files = find_csm_plugins(plugin_files);
  //vw_out() << "Found " << num_plugin_files << " CSM plugin files.\n";

  // Load all of the plugins.
  for (size_t i = 0; i < num_plugin_files; i++) {
    // Get the DLL in memory, causing it to automatically register itself
    //  with the main Plugin interface.
    vw_out() << "Loading CSM plugin: " << plugin_files[i] << std::endl;
    boost::dll::shared_library lib_usgs(plugin_files[i]);
  }

  //csm::Plugin::setDataDirectory(plugin_folder); // Don't think we need this.

  print_available_models();
}

// Read the semi-major and semi-minor axes
void CsmModel::read_ellipsoid_from_isd(std::string const& isd_path) {

  // Load and parse the json file
  std::ifstream ifs(isd_path);
  json json_isd;
  try {
    ifs >> json_isd;
  } catch(...) {
    vw::vw_throw(vw::ArgumentErr() << "Cannot open file: " << isd_path << "\n");
  }
  
  // Read the semi-major axis
  m_semi_major_axis = 0.0;
  try {
    m_semi_major_axis = json_isd.at("radii").at("semimajor");
  } catch (...){
  }

  // Read the semi-minor axis
  m_semi_minor_axis = 0.0;
  try {
    m_semi_minor_axis = json_isd.at("radii").at("semiminor");
  } catch (...){
  }

  // Read the unit
  std::string unit;
  try {
    unit = json_isd.at("radii").at("unit");
  } catch (...){
  }
  boost::to_lower(unit);

  // Convert from km to m if need be
  if (unit == "km") {
    m_semi_major_axis *= 1000.0;
    m_semi_minor_axis *= 1000.0;
  } else if (unit != "m") {
    vw::vw_throw(vw::ArgumentErr() << "Unknown unit for the ellipsoid radii in "
                  << isd_path << ". The read value is: " << unit);
  }

  // Sanity check
  if (m_semi_major_axis <= 0.0 || m_semi_minor_axis <= 0.0) 
    vw::vw_throw(vw::ArgumentErr() << "Could not read positive semi-major "
                 << "and semi-minor axies from:  " << isd_path
                 << ". The read values are: "
                 << m_semi_major_axis << ' ' << m_semi_minor_axis);
}

/// Read and cache the sun position. This is an expensive operation.
/// TODO(oalexan1): See if one can avoid creating and parsing a string file.
/// Maybe by now the Sun position is a public member in each model type.
/// This will work for USGSCSM models, but maybe not for others. It is assumed
/// here that the sun does not move noticeably in the sky during the brief time
/// the picture is taken. 
/// TODO(oalexan1): This returns a single Sun position per camera. It appears
/// that linescan cameras can return a line-dependent Sun position. It is 
/// not clear if that has any value, given how quickly an image is taken.
void readCsmSunPosition(boost::shared_ptr<csm::RasterGM> const& gm_model,
                        vw::Vector3 & sun_position) {

  if (gm_model.get() == NULL)
    vw::vw_throw(vw::ArgumentErr() 
                 << "CsmModel::readCsmSunPosition() failed because " 
                 << "the model is not initialized.\n");
    
  std::string modelState = gm_model->getModelState();
  nlohmann::json j = stateAsJson(modelState);
  if (j.find("m_sunPosition") == j.end())
    vw::vw_throw(vw::ArgumentErr() 
                 << "The Sun position was not found in the CSM model state.\n");
    
  std::vector<double> sun_pos = j["m_sunPosition"].get<std::vector<double>>();
  if (sun_pos.size() < 3)
    vw::vw_throw(vw::ArgumentErr() 
                  << "The Sun position must be a vector of size >= 3.\n");

  for (size_t it = 0; it < 3; it++) 
    sun_position[it] = sun_pos[it];
}

/// Load the camera model from an ISD file or model state.
void CsmModel::load_model(std::string const& isd_path) {
  
  std::string line;
  {
    // Peek inside the file to see if it is an isd or a model state.
    // A model state file starts with an easily identifiable string.
    std::ifstream ifs(isd_path);
    ifs >> line;
  }
  bool is_model_state = (line == UsgsAstroFrameSensorModel::_SENSOR_MODEL_NAME     || 
                         line == UsgsAstroLsSensorModel::_SENSOR_MODEL_NAME        ||
                         line == UsgsAstroPushFrameSensorModel::_SENSOR_MODEL_NAME ||
                         line == UsgsAstroSarSensorModel::_SENSOR_MODEL_NAME);

  if (!is_model_state) 
    CsmModel::load_model_from_isd(isd_path);
  else
    CsmModel::loadModelFromStateFile(isd_path);
  
  CsmModel::createApproxCam();
}

/// Load the model from ISD. Read the ellipsoid, sun position, and
/// m_plugin_name. 
//
// See setModelFromStateString() for a different construction method. These must
// be kept in sync.
void CsmModel::load_model_from_isd(std::string const& isd_path) {
  // This only happens the first time it is called.
  initialize_plugins();

  // Load ISD data
  csm::Isd support_data(isd_path);

  CsmModel::read_ellipsoid_from_isd(isd_path);
 
  // Check each available CSM plugin until we find one that can handle the ISD.
  std::string model_name, model_family;
  const csm::Plugin* csm_plugin = find_plugin_for_isd(support_data, model_name,
                                                      model_family, false);

  // If we did not find a plugin that would work, go through them again and print error
  //  messages for each plugin that fails.
  if (csm_plugin == 0) {
    find_plugin_for_isd(support_data, model_name, model_family, true);
    vw::vw_throw(vw::ArgumentErr() 
                 << "Unable to construct a camera model for the ISD file "
                 << isd_path << " using any of the loaded CSM plugins!");
  }
  
  // Remember the plugin name. It will be needed to add a model state to a cub file.
  m_plugin_name = csm_plugin->getPluginName();
  
  // This is verbose
  //vw::vw_out() << "Using plugin: " << this->plugin_name() 
  //             << " with model name " << model_name << std::endl;

  // Now try to construct the camera model
  csm::WarningList warnings;
  csm::Model* csm_model
    = csm_plugin->constructModelFromISD(support_data, model_name, &warnings);

  // Error checking
  csm::WarningList::const_iterator w_iter;
  for (w_iter = warnings.begin(); w_iter!=warnings.end(); ++w_iter) {
    vw_out() << "CSM warning: " << w_iter->getMessage() << std::endl;
  }

  // Handle load failure
  if (!csm_model)
    vw::vw_throw(vw::ArgumentErr() << "Failed to load CSM sensor model from file: "
                 << isd_path);
  
  // TODO: Are all sensor models going to be this type (RasterGM)?
  //       Otherwise we can use the result of getModelFamily() to choose the class.
  // Cast the model we got to the child class with the needed functionality.
  csm::RasterGM* gm_model = dynamic_cast<csm::RasterGM*>(csm_model);

   // Handle load failure
  if (!gm_model)
    vw::vw_throw(vw::ArgumentErr() << "Failed to cast CSM sensor model to raster type!");
  
  m_gm_model.reset(gm_model); // The smart pointer will handle memory management
  
  // This must happen after gm model is set
  readCsmSunPosition(m_gm_model, m_sun_position);

  // This is a bug fix.
  normalizeLinescanQuaternions();  
}

/// Load the camera model from a model state written to disk.
/// A model state is obtained from an ISD model by pre-processing
/// and combining its data in a form ready to be used.
void CsmModel::loadModelFromStateFile(std::string const& state_file) {

  // Read the state as one string
  std::ifstream ifs(state_file.c_str());
  std::string model_state;
  ifs.seekg(0, std::ios::end);   
  model_state.reserve(ifs.tellg());
  ifs.seekg(0, std::ios::beg);
  model_state.assign((std::istreambuf_iterator<char>(ifs)),
             std::istreambuf_iterator<char>());
  ifs.close();

  bool recreate_model = true;
  CsmModel::setModelFromStateString(model_state, recreate_model);
}

// This should not be used directly. The function setModelFromStateString() 
// below also reads the semi-axes and the Sun position.
template<class ModelT>
void setModelFromStateStringAux(bool recreate_model,
                                std::string const& model_state,
                                boost::shared_ptr<csm::RasterGM> & gm_model) {

  if (recreate_model) {

    csm::RasterGM* new_gm_model = NULL;
    ModelT * specific_model = new ModelT;
    specific_model->replaceModelState(model_state);
    new_gm_model = dynamic_cast<csm::RasterGM*>(specific_model);
    
    // Handle load failure
    if (!new_gm_model)
      vw::vw_throw(vw::ArgumentErr() << "Failed to cast CSM model to raster type.");
    
    // This will wipe any preexisting model. Prior gm_model pointer will become invalid.
    gm_model.reset(new_gm_model); 
    
  } else {
    
    // Update existing model. This does not destroy gm_model.
    ModelT * specific_model = static_cast<ModelT*>(gm_model.get());
    if (specific_model == NULL)
      vw::vw_throw(vw::ArgumentErr() << "Incorrect model type passed in.\n");
    specific_model->replaceModelState(model_state);
    
  }

  return;
}

// Ensure the linescan model quaternions are always normalized and do not
// suddenly flip sign. This is a bug fix.
void CsmModel::normalizeLinescanQuaternions() {
  throw_if_not_init();
  UsgsAstroLsSensorModel * ls_model
    = dynamic_cast<UsgsAstroLsSensorModel*>(m_gm_model.get());
  if (ls_model != NULL)
    asp::normalizeQuaternions(ls_model);
}
  
/// Load the camera model from a model state written to disk. A model state is
/// obtained from an ISD model by pre-processing and combining its data in a
/// form ready to be used. Use recreate_model = false if desired to just update
/// an existing model.
///
/// Read the ellipsoid, sun position, and m_plugin_name. 
///
/// See also load_model_from_isd() for a different construction method. These
/// must be kept in sync.
void CsmModel::setModelFromStateString(std::string const& model_state, 
                                       bool recreate_model) {
  
  // TODO(oalexan1): Use the usgscsm function
  // constructModelFromState() after that package pushes a new version
  // (currently there are compile-time issues with it).
  
  // See which model to load, then cast it to RasterGM. This could
  // have been simpler if the USGSCSM models shared a base class where
  // all shared functionality would be shared.
  if (model_state.rfind(UsgsAstroFrameSensorModel::_SENSOR_MODEL_NAME, 0) == 0) {

    setModelFromStateStringAux<UsgsAstroFrameSensorModel>
      (recreate_model, model_state, m_gm_model);
    
  } else if (model_state.rfind(UsgsAstroLsSensorModel::_SENSOR_MODEL_NAME, 0) == 0) {
    
    setModelFromStateStringAux<UsgsAstroLsSensorModel>
      (recreate_model, model_state, m_gm_model);
    
  } else if (model_state.rfind(UsgsAstroPushFrameSensorModel::_SENSOR_MODEL_NAME, 0) == 0) {
    
    setModelFromStateStringAux<UsgsAstroPushFrameSensorModel>
      (recreate_model, model_state, m_gm_model);
    
  } else if (model_state.rfind(UsgsAstroSarSensorModel::_SENSOR_MODEL_NAME, 0) == 0) {
    
    setModelFromStateStringAux<UsgsAstroSarSensorModel>
      (recreate_model, model_state, m_gm_model);
    
  } else {
    vw::vw_throw(vw::ArgumentErr() << "Could not create CSM model from state string.\n");
  }

  // Get the plugin name
  csm::PluginList plugins = csm::Plugin::getList();
  if (plugins.size() == 0)
    vw::vw_throw(vw::ArgumentErr() << "Could not find CSM plugins.\n");
    
  if (plugins.size() > 1)
    vw::vw_out(vw::WarningMessage) 
      << "Expected to find one CSM plugin, found: " << plugins.size() << ".\n";

  for (auto iter = plugins.begin(); iter != plugins.end(); iter++) {
    const csm::Plugin* csm_plugin = (*iter);
    m_plugin_name = csm_plugin->getPluginName();
  }
  
  // Set the semi-axes from json (cannot pull it from the usgs models
  // as these figure as private in some of them).
  auto j = stateAsJson(model_state);
  m_semi_major_axis = j["m_majorAxis"];
  m_semi_minor_axis = j["m_minorAxis"];

  // Sanity check
  if (m_semi_major_axis <= 0.0 || m_semi_minor_axis <= 0.0) 
    vw::vw_throw(vw::ArgumentErr() << "Could not read positive semi-major "
                 << "and semi-minor axies from state string.");
    
  // This must happen after gm model is set
  readCsmSunPosition(m_gm_model, m_sun_position);
  
  // This is a bug fix.
  normalizeLinescanQuaternions();  
}
  
void CsmModel::throw_if_not_init() const {
  if (!m_gm_model)
    vw_throw(ArgumentErr() << "CsmModel: Sensor model has not been initialized.");
}

// TODO: Check all of the warnings

vw::Vector2 CsmModel::get_image_size() const {
  throw_if_not_init();

  csm::ImageVector size = m_gm_model->getImageSize();
  return Vector2(size.samp, size.line);
}

vw::Vector3 CsmModel::target_radii() const {
  return vw::Vector3(m_semi_major_axis,  // x
                     m_semi_major_axis,  // y
                     m_semi_minor_axis); // z
}

Vector2 CsmModel::point_to_pixel(Vector3 const& point) const {
  throw_if_not_init();

  csm::EcefCoord  ecef = vectorToEcefCoord(point);

  double achievedPrecision = -1.0;
  csm::WarningList warnings;
  csm::WarningList * warnings_ptr = NULL;

  // Do not show warnings, it becomes too verbose
  bool show_warnings = false;
  if (show_warnings) 
    warnings_ptr = &warnings;

  csm::ImageCoord imagePt = m_gm_model->groundToImage(ecef, m_desired_precision,
						       &achievedPrecision, warnings_ptr);

  if (show_warnings) {
    csm::WarningList::const_iterator w_iter;
    for (w_iter = warnings.begin(); w_iter!=warnings.end(); ++w_iter) {
      vw_out() << "CSM warning: " << w_iter->getMessage() << std::endl;
    }
  }

  vw::Vector2 pix = imageCoordToVector(imagePt) - ASP_TO_CSM_SHIFT;
  
  // This is a bugfix for when points far from the field of view project
  // incorrectly into the camera.
  if (m_maxApproxCamPixErr > 0)
    return this->correctWithApproxCam(pix, point);
    
  return pix;
}

vw::Vector3 CsmModel::pixel_to_vector(vw::Vector2 const& pix) const {
  throw_if_not_init();

  csm::ImageCoord imagePt;
  toCsmPixel(pix, imagePt);
  
  double achievedPrecision = -1.0; // will be modified in the function
  csm::EcefLocus locus = m_gm_model->imageToRemoteImagingLocus(imagePt,
                                                               m_desired_precision,
                                                               &achievedPrecision);
  Vector3 dir = ecefVectorToVector(locus.direction);
  return dir;

#if 0  
  // This alternative approach gives the same results as above, except
  // for the SAR model, which has curved rays, and for MSL, whose 
  // location is below the zero datum. 
  
  // This code is kept in case it is necessary to revisit the SAR model.
  // Camera center
  csm::EcefCoord  ctr = m_gm_model->getSensorPosition(imagePt);

  // Ground point. Note how we use the 0 height above datum.
  // The precise height value matters only for the SAR model, when the rays
  // are curved, which violates a fundamental assumption in ASP.
  double groundHeight      = 0.0;
  csm::EcefCoord groundPt
    = m_gm_model->imageToGround(imagePt, groundHeight, m_desired_precision,
                                 &achievedPrecision);

  // Normalized direction
  Vector3 dir0 = ecefCoordToVector(groundPt) - ecefCoordToVector(ctr);
  dir0 = dir0 / norm_2(dir0);
  return dir0;
#endif

}

Vector3 CsmModel::camera_center(Vector2 const& pix) const {
  throw_if_not_init();

  csm::ImageCoord imagePt = vectorToImageCoord(pix + ASP_TO_CSM_SHIFT);
  csm::EcefCoord  ecef    = m_gm_model->getSensorPosition(imagePt);

  return ecefCoordToVector(ecef);
}

// Apply a transform to the model state in json format
template<class ModelT>
void applyTransformToState(ModelT const * model,
                           vw::Matrix4x4 const& transform,
                           // Output
                           std::string & modelState) {

  // Applying a scale is not supported in any usgscsm sensors for now.
  double scale = pow(det(transform), 1.0/3.0);
  if (std::abs(scale - 1.0) > 1e-6)
    vw_throw(ArgumentErr()
             << "CSM camera models do not support applying a transform with a scale.\n");

  // Extract the rotation and convert it to ale::Rotation
  vw::Matrix3x3 rotation_matrix = submatrix(transform, 0, 0, 3, 3);
  std::vector<double> rotation_vec;
  for (int row = 0; row < 3; row++) {
    for (int col = 0; col < 3; col++) {
      rotation_vec.push_back(rotation_matrix(row, col));
    }
  }
  ale::Rotation r(rotation_vec);
  
  // Extract the translation
  ale::Vec3d t(transform(0, 3), transform(1, 3), transform(2, 3));
  
  model->applyTransformToState(r, t, modelState);

  return;
}

void applyTransformToState(csm::RasterGM const* gm_model,
                           vw::Matrix4x4 const& transform,
                           // Output
                           std::string & modelState) {
  
  // Need to consider each model type separately
  bool success = false;
  UsgsAstroFrameSensorModel const* frame_model
    = dynamic_cast<UsgsAstroFrameSensorModel const*>(gm_model);
  if (!success && frame_model != NULL) {
    applyTransformToState(frame_model, transform, modelState);
    success = true;
  }
  
  UsgsAstroLsSensorModel const* ls_model
    = dynamic_cast<UsgsAstroLsSensorModel const*>(gm_model);
  if (!success && ls_model != NULL) {
    applyTransformToState(ls_model, transform, modelState);
    success = true;
  }

  UsgsAstroPushFrameSensorModel const* pf_model
    = dynamic_cast<UsgsAstroPushFrameSensorModel const*>(gm_model);
  if (!success && pf_model != NULL) {
    applyTransformToState(pf_model, transform, modelState);
    success = true;
  }

  UsgsAstroSarSensorModel const* sar_model
    = dynamic_cast<UsgsAstroSarSensorModel const*>(gm_model);
  if (!success && sar_model != NULL) {
    applyTransformToState(sar_model, transform, modelState);
    success = true;
  }

  if (!success)
    vw_throw(vw::ArgumentErr()
             << "CsmModel::applyTransformedState(): Unknown CSM model type.\n");
}

// Save model state
void CsmModel::saveState(std::string const& json_state_file) const {
  
  throw_if_not_init();

  csm::RasterGM const* gm_model
    = dynamic_cast<csm::RasterGM const*>(this->m_gm_model.get());
    
  std::string modelState = gm_model->getModelState();
  std::ofstream ofs(json_state_file.c_str());
  ofs << modelState << std::endl;
  ofs.close();

  return;
}

// Apply a transform to a CSM model
void CsmModel::applyTransform(vw::Matrix4x4 const& transform) {

  throw_if_not_init();
  
  csm::RasterGM const* gm_model
    = dynamic_cast<csm::RasterGM const*>(this->m_gm_model.get());
  
  std::string modelState = gm_model->getModelState();
  
  applyTransformToState(gm_model, transform,  
                        // Output
                        modelState);

  bool recreate_model = false; // don't want to destroy the model
  setModelFromStateString(modelState, recreate_model);
}
 
std::string CsmModel::plugin_name() const {
  if (m_plugin_name.empty())
    vw_throw(ArgumentErr() << "CsmModel: Plugin name has not been set yet.");
  return m_plugin_name;
}

std::string CsmModel::model_name() const {
  throw_if_not_init();
  return m_gm_model->getModelName();
}

std::string CsmModel::model_state() const {
  throw_if_not_init();
  return m_gm_model->getModelState();
}

// Convert -0 to 0. The -0 seems to be a quick. Have to return a copy 
// due to these being json fields.
std::vector<double> stripSign(std::vector<double> const & vals) {
  std::vector<double> out_vals = vals;
  for (size_t i = 0; i < vals.size(); i++)
    if (std::abs(vals[i]) < 1e-16)
      out_vals[i] = 0.0;
      
  return out_vals;
}  

// Create a CSM frame camera model. This requires a lot of bookkeeping. Use
// cam_test to compare such model with ASP's Pinhole model with same data. That
// is created as: vw::camera::PinholeModel pin(C, R, focal_length,
// focal_length, cx, cy);
void CsmModel::createFrameModel(int cols, int rows,  // in pixels
        double cx, double cy, // col and row of optical center, in units of pixel pitch
        double focal_length,  // in units of pixel pitch
        double semi_major_axis, double semi_minor_axis, // in meters
        vw::Vector3 const& C, // camera center
        vw::Matrix3x3 const& R, // camera to world rotation matrix
        std::string const& distortionType,
        std::vector<double> const& distortion,
        double ephem_time,
        vw::Vector3 const& sun_position,
        std::string const& serial_number,
        std::string const& target_name,
        double pixel_pitch) {
        
  // Make a copy of R as an Eigen matrix, and convert to quaternion
  Eigen::Matrix3d R_copy;
  for (int r = 0; r < 3; r++){
    for (int c = 0; c < 3; c++)
      R_copy(r, c) = R(r, c);
  }
  Eigen::Quaterniond q(R_copy);

  // Creating a frame model requires populating a json file
  UsgsAstroFrameSensorModel cam;
  cam.reset();
  std::string state = cam.getModelState();
  nlohmann::json j = stateAsJson(state);

  j["m_sensorName"] = "csm";
  j["m_platformName"] = "csm";
  j["m_majorAxis"] = semi_major_axis;
  j["m_minorAxis"] = semi_minor_axis;
  j["m_minElevation"] = -10000.0; // -10 km
  j["m_maxElevation"] = 10000.0;  // 10 km

  // Here a particular choice is assumed for converting from sensor plane
  // coordinates to pixels, which is compatible with the ASP Pinhole model.
  j["m_iTransL"] = std::vector<double>({0.0, 0.0, 1.0 / pixel_pitch});
  j["m_iTransS"] = std::vector<double>({0.0, 1.0 / pixel_pitch, 0.0});
  j["m_focalLength"] = focal_length; 
  
  // Note the order (row, col), and how we must divide by pixel pitch
  j["m_ccdCenter"] = std::vector<double>({cy / pixel_pitch, cx / pixel_pitch});
  j["m_pixelPitch"] = pixel_pitch;
  j["m_nLines"] = rows;
  j["m_nSamples"] = cols;

  // Set the distortion.  
  if (distortionType.empty()) {
    // Let default distortion be radial, with zero distortion. Avoid transverse
    // distortion, as that needs a lot of care in setting the coefficients.
    j["m_distortionType"] = DistortionType::RADIAL;
    j["m_opticalDistCoeffs"] = std::vector<double>(3, 0.0);
  } else if (distortionType == "radial") {
    if (distortion.size() != 3)
      vw::vw_throw(ArgumentErr() 
                   << "Distortion coefficients for the radial distortion "
                   << "model must be of size 3, in the order k1, k2, k3. "
                   << "Got the size: " << distortion.size() << "\n");
    j["m_distortionType"] = DistortionType::RADIAL;
    j["m_opticalDistCoeffs"] = distortion;
  } else if (distortionType == "radtan") {
    if (distortion.size() != 5)
      vw::vw_throw(ArgumentErr() 
                   << "Distortion coefficients for the radtan distortion "
                   << "model must be of size 5, in the order k1, k2, p1, p2, k3. "
                   << "Got the size: " << distortion.size() << "\n");
    j["m_distortionType"] = DistortionType::RADTAN;
    j["m_opticalDistCoeffs"] = distortion;
  } else if (distortionType == "transverse") {
    j["m_distortionType"] = DistortionType::TRANSVERSE;
    if (distortion.size() != 20)
      vw::vw_throw(ArgumentErr() 
                   << "Distortion coefficients for the transverse distortion "
                   << "model must be of size 20. Thse are the coefficients of a "
                   << "polynomial of degree 3 in x and y. "
                   << "Got the size: " << distortion.size() << "\n");
    j["m_opticalDistCoeffs"] = distortion;
  } else {
    vw_throw(ArgumentErr() << "Unknown distortion type: " << distortionType << ".\n");
  }

  // Need to apply this offset to make CSM agree with ASP's Pinhole
  j["m_startingDetectorLine"] = -0.5;
  j["m_startingDetectorSample"] = -0.5;

  // Part of the API    
  j["m_focalLengthEpsilon"] = 1.0; 

  // Copied from UsgsAstroFrameSensorModel.cpp  
  double det = j["m_iTransL"][1].get<double>() * j["m_iTransS"][2].get<double>() 
                - j["m_iTransL"][2].get<double>() * j["m_iTransS"][1].get<double>();
  j["m_transX"][1] = j["m_iTransL"][1].get<double>() / det;
  j["m_transX"][2] = -j["m_iTransS"][1].get<double>() / det;
  j["m_transX"][0] = -(j["m_transX"][1].get<double>() * j["m_iTransL"][0].get<double>() 
                       + j["m_transX"][2].get<double>() * j["m_iTransS"][0].get<double>());
  j["m_transY"][1] = -j["m_iTransL"][2].get<double>() / det;
  j["m_transY"][2] = j["m_iTransS"][2].get<double>() / det;
  j["m_transY"][0] = -(j["m_transY"][1].get<double>() * j["m_iTransL"][0].get<double>() 
                       + j["m_transY"][2].get<double>() * j["m_iTransS"][0].get<double>());
  
  // Fix a quirk with -0. Cannot modify in-place the json fields, hence the copy.
  j["m_transX"] = stripSign(j["m_transX"]);
  j["m_transY"] = stripSign(j["m_transY"]);
  
  // Set the translation and quaternion. The quaternion is stored as x, y, z, w.
  j["m_currentParameterValue"] = std::vector<double>({C[0], C[1], C[2], 
                                                     q.x(), q.y(), q.z(), q.w()});
  
  j["m_ephemerisTime"] = ephem_time;
  j["m_sunPosition"]   = std::vector<double>({sun_position[0],
                                              sun_position[1], 
                                              sun_position[2]});
  j["m_imageIdentifier"] = serial_number;
  
  // Set the target name in the json
  j["m_targetName"] = target_name;
  
  // Update the state string and create the CSM model
  state = cam.getModelName() + "\n" + j.dump(2);
  bool recreate_model = true;
  setModelFromStateString(state, recreate_model);
  
  // This is a temporary fix for the function replaceModelState()
  // in UsgsAstroFrameSensorModel forgetting the target name.
  // Pull request submitted.
  set_target_name(target_name);
}

// Create a CSM frame camera model from pinhole camera model.
void CsmModel::createFrameModel(vw::camera::PinholeModel const& pin_model,
                                int cols, int rows,  // in pixels
                                double semi_major_axis, double semi_minor_axis, // in meters
                                std::string const& distortionType, 
                                std::vector<double> const& distortion,
                                double ephem_time,
                                vw::Vector3 const& sun_position,
                                std::string const& serial_number,
                                std::string const& target_name) {

  // These are all in units of pixel pitch
  vw::Vector2 focal_length = pin_model.focal_length();
  vw::Vector2 opt_ctr = pin_model.point_offset();
  double pixel_pitch = pin_model.pixel_pitch();
  
  // Find the average focal length
  double f = (focal_length[0] + focal_length[1])/2.0;
  
  this->createFrameModel(cols, rows, opt_ctr[0], opt_ctr[1], f, 
                         semi_major_axis, semi_minor_axis,
                         pin_model.camera_center(), 
                         pin_model.get_rotation_matrix(),
                         distortionType, distortion,
                         ephem_time, sun_position,
                         serial_number, target_name,
                         pixel_pitch);
}

// Approximate conversion to a pinhole model. Will be exact only for the radtan
// lens distortion and no unusual line or sample adjustments in CSM. Compare
// these with cam_test.
// TODO(oalexan1): This code is not used and not tested.
vw::camera::PinholeModel CsmModel::pinhole() const {
  
  // Camera center
  double x = 0, y = 0, z = 0;
  this->frame_position(x, y, z);
  vw::Vector3 cam_ctr(x, y, z);
  
  // Camera orientation
  double qx = 0, qy = 0, qz = 0, qw = 0;
  this->frame_quaternion(qx, qy, qz, qw);
  Eigen::Quaterniond q(qw, qx, qy, qz);
  Eigen::Matrix3d R = q.toRotationMatrix();
  vw::Matrix3x3 cam_rot;
  for (int r = 0; r < 3; r++)
    for (int c = 0; c < 3; c++)
      cam_rot(r, c) = R(r, c);
  
  // Focal length, in units of pixel pitch
  double f = this->focal_length();
  
  // CSM optical center is always in pixels. Have to convert to pixel pitch units.
  vw::Vector2 optical_center = this->optical_center() * this->frame_pixel_pitch();

  // Create a pinhole model with zero distortion
  vw::camera::PinholeModel pin(cam_ctr, cam_rot, f, f, 
                               optical_center[0], optical_center[1],
                               NULL, this->frame_pixel_pitch());

  // Distortion
  DistortionType dist_type = this->distortion_type();
  if (dist_type == DistortionType::RADTAN) {

    // Must have 5 coefficients
    std::vector<double> dist = this->distortion();
    if (dist.size() != 5)
      vw_throw(ArgumentErr() << "Expected 5 distortion coefficients for radtan model.\n");

    // Copy to VW vector
    vw::Vector<double> coeffs;
    coeffs.set_size(dist.size());
    for (size_t i = 0; i < dist.size(); i++)
      coeffs[i] = dist[i];
      
    vw::camera::TsaiLensDistortion distModel(coeffs);
    pin.set_lens_distortion(&distModel);
  }
  
  return pin;
}

// Must have some macros here to avoid a lot of boilerplate code
#define CSM_FRAME_GET(PARAM, NAME, VAL)                           \
  /* Try frame */                                                 \
  success = false;                                                \
  csm::RasterGM const* gm_model                                   \
    = dynamic_cast<csm::RasterGM const*>(this->m_gm_model.get()); \
  {                                                               \
    UsgsAstroFrameSensorModel const* frame_model                  \
      = dynamic_cast<UsgsAstroFrameSensorModel const*>(gm_model); \
    if (!success && frame_model != NULL) {                        \
      VAL = frame_model->PARAM;                                   \
      success = true;                                             \
    }                                                             \
  }

#define CSM_FRAME_SET(PARAM, NAME, VAL)                     \
  success = false;                                          \
  csm::RasterGM * gm_model                                  \
    = dynamic_cast<csm::RasterGM*>(this->m_gm_model.get()); \
  /* Try frame */                                           \
  {                                                         \
    UsgsAstroFrameSensorModel * frame_model                 \
      = dynamic_cast<UsgsAstroFrameSensorModel*>(gm_model); \
    if (!success && frame_model != NULL) {                  \
      frame_model->PARAM = VAL;                             \
      success = true;                                       \
   }                                                        \
  }

#define CSM_LINESCAN_GET(PARAM, NAME, VAL)                        \
  /* Try linescan */                                              \
  success = false;                                                \
  {                                                               \
    UsgsAstroLsSensorModel const* ls_model                        \
      = dynamic_cast<UsgsAstroLsSensorModel const*>(gm_model);    \
    if (!success && ls_model != NULL) {                           \
      VAL = ls_model->PARAM;                                      \
      success = true;                                             \
    }                                                             \
  }                                                               \
  /* Fail otherwise. There's no chance we will need this */       \
  /* with SAR or pushbroom models */                              \
  if (!success)                                                   \
    vw_throw(vw::ArgumentErr()                                    \
         << "CSM model " << NAME << " can be handled only "       \
         << "for linescan and frame cameras.\n");

#define CSM_LINESCAN_SET(PARAM, NAME, VAL)                  \
  /* Try linescan */                                        \
  success = false;                                          \
  {                                                         \
    UsgsAstroLsSensorModel * ls_model                       \
      = dynamic_cast<UsgsAstroLsSensorModel*>(gm_model);    \
    if (!success && ls_model != NULL) {                     \
      ls_model->PARAM = VAL;                                \
      success = true;                                       \
    }                                                       \
  }                                                         \
  /* Fail otherwise. There's no chance we will need this */ \
  /* with SAR or pushbroom models. */                       \
  if (!success)                                             \
    vw_throw(vw::ArgumentErr()                              \
         << "CSM model " << NAME << " can be handled only " \
         << "for linescan and frame cameras.\n");

// Get distortion
std::vector<double> CsmModel::distortion() const {
  std::vector<double> dist;
  bool success = false;
  CSM_FRAME_GET(m_opticalDistCoeffs, "distortion", dist)
  if (success) 
    return dist;
  CSM_LINESCAN_GET(m_opticalDistCoeffs, "distortion", dist)
  return dist;
}

// Get distortion type
DistortionType CsmModel::distortion_type() const {
  DistortionType dist_type;
  bool success = false;
  CSM_FRAME_GET(m_distortionType, "distortion type", dist_type)
  if (success) 
    return dist_type;
  CSM_LINESCAN_GET(m_distortionType, "distortion type", dist_type)
  return dist_type;
}

// Set distortion type
void CsmModel::set_distortion_type(DistortionType dist_type) {
  bool success = false;
  CSM_FRAME_SET(m_distortionType, "distortion type", dist_type)
  if (success) 
    return;
  CSM_LINESCAN_SET(m_distortionType, "distortion type", dist_type)
  return;
}

// Set camera position in ECEF (only for frame cameras)
void CsmModel::set_frame_position(double x, double y, double z) {
  
  throw_if_not_init();
  
  UsgsAstroFrameSensorModel * frame_model
      = dynamic_cast<UsgsAstroFrameSensorModel*>(m_gm_model.get());
  if (frame_model == NULL)
    vw_throw(ArgumentErr() 
             << "CsmModel: Cannot set camera position for non-frame camera.\n");
  
  frame_model->m_currentParameterValue[0] = x;
  frame_model->m_currentParameterValue[1] = y;
  frame_model->m_currentParameterValue[2] = z;
    
  return;
}

// Get the camera position in ECEF (only for frame cameras)
void CsmModel::frame_position(double & x, double & y, double & z) const {
  
  throw_if_not_init();
  
  UsgsAstroFrameSensorModel const* frame_model
      = dynamic_cast<UsgsAstroFrameSensorModel const*>(m_gm_model.get());
  if (frame_model == NULL)
    vw_throw(ArgumentErr() << "CsmModel: Cannot get camera position for non-frame camera.\n");
  
  x = frame_model->m_currentParameterValue[0];
  y = frame_model->m_currentParameterValue[1];
  z = frame_model->m_currentParameterValue[2];
    
  return;
}

// Set the camera quaternion (only for frame cameras)
void CsmModel::set_frame_quaternion(double qx, double qy, double qz, double qw) {

  throw_if_not_init();
  
  UsgsAstroFrameSensorModel * frame_model
      = dynamic_cast<UsgsAstroFrameSensorModel*>(m_gm_model.get());
  if (frame_model == NULL)
    vw_throw(ArgumentErr() << "CsmModel: Cannot set camera quaternion for non-frame camera.\n");
  
  frame_model->m_currentParameterValue[3] = qx;
  frame_model->m_currentParameterValue[4] = qy;
  frame_model->m_currentParameterValue[5] = qz;
  frame_model->m_currentParameterValue[6] = qw;
  
  return;
}

// Get the camera quaternion (only for frame cameras)
void CsmModel::frame_quaternion(double & qx, double & qy, double & qz, double & qw) const {
  
  throw_if_not_init();
  
  UsgsAstroFrameSensorModel const* frame_model
      = dynamic_cast<UsgsAstroFrameSensorModel const*>(m_gm_model.get());
  if (frame_model == NULL)
    vw_throw(ArgumentErr() 
             << "CsmModel: Cannot get camera quaternion for non-frame camera.\n");
  
  qx = frame_model->m_currentParameterValue[3];
  qy = frame_model->m_currentParameterValue[4];
  qz = frame_model->m_currentParameterValue[5];
  qw = frame_model->m_currentParameterValue[6];
  
  return;
}

// Get the camera position in ECEF (only for frame cameras)
double CsmModel::frame_pixel_pitch() const {
  
  throw_if_not_init();
  
  UsgsAstroFrameSensorModel const* frame_model
      = dynamic_cast<UsgsAstroFrameSensorModel const*>(m_gm_model.get());
  if (frame_model == NULL)
    vw_throw(ArgumentErr() 
             << "CsmModel: Cannot get pixel pitch for non-frame camera.\n");

  // Check that m_iTransL and m_iTransS are set as in createFrameModel()
  if (frame_model->m_iTransL[0] != 0.0 || frame_model->m_iTransL[1] != 0.0)
    vw_throw(ArgumentErr() 
             << "CsmModel: m_iTransL must have first two elements equal to zero.\n");
  if (frame_model->m_iTransS[0] != 0.0 || frame_model->m_iTransS[2] != 0.0)
    vw_throw(ArgumentErr() 
             << "CsmModel: m_iTransS must have first and third elements equal to zero.\n");
  if (frame_model->m_iTransL[2] <= 0.0 || 
      frame_model->m_iTransS[1] <= 0.0 ||
      frame_model->m_iTransL[2] != frame_model->m_iTransS[1]) {
    vw_throw(ArgumentErr() 
             << "CsmModel: m_iTransL[2] and m_iTransS[1] must be positive and equal.\n");
  }
  
  return (1.0/frame_model->m_iTransL[2] + 1.0/frame_model->m_iTransS[1]) / 2.0; 
}
  
// Set quaternions (only for linescan cameras)
void CsmModel::set_linescan_quaternions(std::vector<double> const& quaternions) {
  
  throw_if_not_init();
  
  csm::RasterGM * gm_model = dynamic_cast<csm::RasterGM*>(this->m_gm_model.get());
  
  int num_quaternions = quaternions.size(); // total number of coefficients
  bool success = false;
  CSM_LINESCAN_SET(m_numQuaternions, "num quaternions", num_quaternions)
  CSM_LINESCAN_SET(m_quaternions, "quaternions", quaternions)
}

// Set the distortion. Need to consider each model type separately.
void CsmModel::set_distortion(std::vector<double> const& dist) {
  bool success = false;
  CSM_FRAME_SET(m_opticalDistCoeffs, "distortion", dist)
  if (success) 
    return;
  CSM_LINESCAN_SET(m_opticalDistCoeffs, "distortion", dist)
  return;
}

// Get the focal length
double CsmModel::focal_length() const {
  double focal_length = 0.0;
  bool success = false;
  CSM_FRAME_GET(m_focalLength, "focal length", focal_length)
  if (success) 
    return focal_length;
  CSM_LINESCAN_GET(m_focalLength, "focal length", focal_length)
  return focal_length;
}

// Set the focal length
void CsmModel::set_focal_length(double focal_length) {
  bool success = false;
  CSM_FRAME_SET(m_focalLength, "focal length", focal_length)
  if (success) 
    return;
  CSM_LINESCAN_SET(m_focalLength, "focal length", focal_length)
  return;
}

// Get the optical center as sample, line. Different logic is needed for frame
// and linescan cameras. Will return (m_ccdCenter[1], m_ccdCenter[0]) for frame,
// and (m_detectorSampleOrigin, m_detectorLineOrigin) for linescan.
// This is always in units of pixels, not mm.
vw::Vector2 CsmModel::optical_center() const {
  vw::Vector2 optical_center;

  std::vector<double> ccd_center;
  bool success = false;
  CSM_FRAME_GET(m_ccdCenter, "optical center", ccd_center)
  if (success) 
    return vw::Vector2(ccd_center[1], ccd_center[0]); // note the order (sample, line)

  CSM_LINESCAN_GET(m_detectorSampleOrigin, "detector sample", optical_center[0])
  CSM_LINESCAN_GET(m_detectorLineOrigin,   "detector line",   optical_center[1])
  
  return optical_center;
}

// Set the optical center as sample, line. Different logic is needed for frame
// and linescan cameras.
void CsmModel::set_optical_center(vw::Vector2 const& optical_center) {
  bool success = false;
  auto ccd_center = std::vector<double>({optical_center[1], optical_center[0]});
  CSM_FRAME_SET(m_ccdCenter, "optical center", ccd_center)
  if (success) 
    return;

  CSM_LINESCAN_SET(m_detectorSampleOrigin, "detector sample", optical_center[0])
  CSM_LINESCAN_SET(m_detectorLineOrigin,   "detector line",   optical_center[1])

  return; 
}

// Get quaternions (only for linescan cameras)
std::vector<double> CsmModel::linescan_quaternions() const {
  
  throw_if_not_init();
  csm::RasterGM * gm_model = dynamic_cast<csm::RasterGM*>(this->m_gm_model.get());
  
  std::vector<double> quaternions;
  bool success = false;
  CSM_LINESCAN_GET(m_quaternions, "quaternions", quaternions)
  return quaternions;
}

// Set target name (only for frame cameras)
void CsmModel::set_target_name(std::string const& target_name) {
  
  throw_if_not_init();
  
  UsgsAstroFrameSensorModel * frame_model
      = dynamic_cast<UsgsAstroFrameSensorModel*>(m_gm_model.get());
  if (frame_model != NULL)
    frame_model->m_targetName = target_name;
  
  return;
}
  
// Get target name (only for frame cameras)
std::string CsmModel::target_name() const {
  
  throw_if_not_init();
  
  UsgsAstroFrameSensorModel const* frame_model
      = dynamic_cast<UsgsAstroFrameSensorModel const*>(m_gm_model.get());
  if (frame_model != NULL)
    return frame_model->m_targetName;

  // Fallback measure
  return "";
}

// Create a deep copy of the model, so don't just copy the shared pointer.
void CsmModel::deep_copy(boost::shared_ptr<CsmModel> & copy) const {
  // Initialize the output
  copy.reset(new CsmModel);
  // Then make a deep copy
  this->deep_copy(*copy.get());
}

void CsmModel::deep_copy(CsmModel & copy) const {

  throw_if_not_init();
  
  // Start with a shallow copy. Then make a deep copy of m_gm_model.
  copy = *this;

  // Frame case
  UsgsAstroFrameSensorModel const* frame_model 
    = dynamic_cast<UsgsAstroFrameSensorModel const*>(m_gm_model.get());
  if (frame_model != NULL) {
    UsgsAstroFrameSensorModel * new_frame_model 
      = new UsgsAstroFrameSensorModel(*frame_model);
    copy.m_gm_model.reset(new_frame_model);
    return;
  }

  // Linescan case
  UsgsAstroLsSensorModel const* ls_model 
    = dynamic_cast<UsgsAstroLsSensorModel const*>(m_gm_model.get());
  if (ls_model != NULL) {
    UsgsAstroLsSensorModel * new_ls_model = new UsgsAstroLsSensorModel(*ls_model);
    copy.m_gm_model.reset(new_ls_model);
    return;
  }

  // Pushframe case
  UsgsAstroPushFrameSensorModel const* pf_model 
    = dynamic_cast<UsgsAstroPushFrameSensorModel const*>(m_gm_model.get());
  if (pf_model != NULL) {
    UsgsAstroPushFrameSensorModel * new_pf_model 
      = new UsgsAstroPushFrameSensorModel(*pf_model);
    copy.m_gm_model.reset(new_pf_model);
    return;
  }

  // SAR case
  UsgsAstroSarSensorModel const* sar_model 
    = dynamic_cast<UsgsAstroSarSensorModel const*>(m_gm_model.get());
  if (sar_model != NULL) {
    UsgsAstroSarSensorModel * new_sar_model = new UsgsAstroSarSensorModel(*sar_model);
    copy.m_gm_model.reset(new_sar_model);
    return;
  }

  // Throw an error
  vw_throw(ArgumentErr() << "CsmModel::deep_copy(): Unknown CSM model type.\n");
}

vw::Vector3 CsmModel::sun_position() const {
  if (m_sun_position == vw::Vector3())
    vw::vw_throw(vw::ArgumentErr() 
                 << "CsmModel::sun_position() returns the Sun position as being "
                 << "at the planet center. This is a programmer error.\n");
  return m_sun_position;
}

bool CsmModel::isFrameCam() const {
  throw_if_not_init();
  csm::RasterGM const* gm_model
    = dynamic_cast<csm::RasterGM const*>(this->m_gm_model.get());
  if (gm_model == NULL)
    return false;
  UsgsAstroFrameSensorModel const* frame_model
    = dynamic_cast<UsgsAstroFrameSensorModel const*>(gm_model);
  if (frame_model == NULL)
    return false;
    
  return true;
}

// Get the datum from the CSM model. It is suggested to use if possible
// the function StereoSessionCsm::get_datum() which calls this one, as
// that one also knows about the image and can find the datum name.
// If the spheroid name is not known, use "unknown".
vw::cartography::Datum CsmModel::get_datum_csm(std::string spheroid_name, 
                                               bool use_sphere_for_non_earth) const {

  std::string datum_name = "D_" + spheroid_name; // may be refined later

  // Read the ellipsoid radii
  vw::Vector3 radii = this->target_radii();
  double radius1 = (radii[0] + radii[1]) / 2; // average the x and y axes (semi-major) 
  double radius2 = radius1;

  // Auto-guess the datum if not available
  vw::cartography::Datum wgs84("WGS84");
  vw::cartography::Datum moon("D_MOON");
  vw::cartography::Datum mars("D_MARS");
  bool is_wgs84 = (std::abs(wgs84.semi_major_axis() - radius1)  < 1e-7 &&
                   std::abs(wgs84.semi_minor_axis() - radii[2]) < 1e-7);
  bool is_moon =  (std::abs(moon.semi_major_axis()  - radius1)  < 1e-7 &&
                   std::abs(moon.semi_minor_axis()  - radii[2]) < 1e-7);
  bool is_mars =  (std::abs(mars.semi_major_axis()  - radius1)  < 1e-7 &&
                   std::abs(mars.semi_minor_axis()  - radii[2]) < 1e-7);
  
  if (boost::to_lower_copy(spheroid_name).find("unknown") != std::string::npos ||
      spheroid_name.empty()) {
    // Unknown datum. Try to fill in the name from above.
    if (is_wgs84)
      return wgs84;
    if (is_moon)
      return moon;
    if (is_mars)
      return mars;
  }
  
  // For Earth always use two radii. The logic below should distinguish Venus.
  bool has_earth_radius = (std::abs(radius1/wgs84.semi_major_axis() - 1.0) < 0.05);
  if (!use_sphere_for_non_earth || has_earth_radius)
    radius2 = radii[2]; // let the semi-minor axis be distinct from the semi-major axis
  
  vw::cartography::Datum datum(datum_name, spheroid_name,
                               "Reference Meridian", radius1, radius2, 0);
  
  return datum;
  
}

// Create a projective approximation of the camera, if linescan and having
// radtan distortion. This helps project into the camera ground points very
// far from the field of view. See the .h file for more info. Code adapted from
// UsgsAstroLsSensorModel.cc.
// It may work better to compare the linescan model with distortion with the one
// without distortion. This may be slower though.
void CsmModel::createApproxCam() {

  throw_if_not_init();
  UsgsAstroLsSensorModel * ls_model
    = dynamic_cast<UsgsAstroLsSensorModel*>(m_gm_model.get());
  if (ls_model == NULL)
    return;
  if (ls_model->m_distortionType != DistortionType::RADTAN)
    return;

  csm::EcefCoord refPt = m_gm_model->getReferencePoint();
  double desired_precision = 1e-3;
  double height = computeEllipsoidElevation(refPt.x, refPt.y, refPt.z, 
                                            m_semi_major_axis, m_semi_minor_axis,
                                            desired_precision);
  if (std::isnan(height))
    return;

  vw::Vector2 imageSize = CsmModel::get_image_size();
  double numCols = imageSize[0];
  double numRows = imageSize[1];

  // Use 10 samples along each row and column
  int numSamples = 10.0;
  
  // Sample at two heights (these get added to the ellipsoid height from above).
  std::vector<double> height_delta = {-100.0, 100.0};
  std::vector<vw::Vector2> imagePixels;
  std::vector<vw::Vector3> groundPts;
  
  // Iterate over height_delta 
  for (size_t ht_iter = 0; ht_iter < height_delta.size(); ht_iter++) {
    
    double curr_height = height + height_delta[ht_iter];
    
    // Iterate over the samples for given height  
    for (int col_samp = 0; col_samp < numSamples; col_samp++) {
      for (int row_samp = 0; row_samp < numSamples; row_samp++) {
  
        vw::Vector2 pix((numCols - 1.0) * (double(col_samp) / (numSamples - 1.0)),
                        (numRows - 1.0) * (double(row_samp) / (numSamples - 1.0)));
        vw::Vector3 xyz = 
          vw::cartography::datum_intersection(m_semi_major_axis + curr_height,
                                              m_semi_minor_axis + curr_height,
                                              this->camera_center(pix),
                                              this->pixel_to_vector(pix));
        
        // Print a warning and quit
        if (xyz == vw::Vector3()) {
          vw::vw_out(vw::WarningMessage)
            << "Failed to create an approximate camera model that may help "
            << "with projecting into the camera pixels far from the field of view.";
            return;
        }
      
        imagePixels.push_back(pix);
        groundPts.push_back(xyz);
      } // end iterate over rows
    } // end iterate over columns
  } // end iterate over height deltas
  
  asp::calcProjTrans(imagePixels, groundPts, m_approxCamCoeffs);
  
  // Test. Iterate over xyz, find pixel, compare.
  m_maxApproxCamPixErr = 0.0;
  for (size_t i = 0; i < imagePixels.size(); i++) {
    vw::Vector2 pix = imagePixels[i];
    vw::Vector3 xyz = groundPts[i];
    vw::Vector2 pix2 = asp::applyProjTrans(xyz, m_approxCamCoeffs);
    
    double err = vw::math::norm_2(pix - pix2);
    m_maxApproxCamPixErr = std::max(m_maxApproxCamPixErr, err);
  }
  m_maxApproxCamPixErr = std::max(m_maxApproxCamPixErr, 10.0); // ensure it is not small
}

// This is a bugfix for when points far from the field of view project
// incorrectly into the camera.
vw::Vector2 CsmModel::correctWithApproxCam(vw::Vector2 const& pix, 
                                           vw::Vector3 const& xyz) const {

  // Find the approximate projection based on a projective transform
  vw::Vector2 apix = asp::applyProjTrans(xyz, m_approxCamCoeffs);

  // If the exact pixel is in the image box, and the approx one is out of the
  // box, need a closer study.
  vw::Vector2 imageSize = this->get_image_size();
  bool pixIn = (0 <= pix[0] && pix[0] < imageSize[0] &&
                0 <= pix[1] && pix[1] < imageSize[1]);
  bool apixIn = (0 <= apix[0] && apix[0] < imageSize[0] &&
                 0 <= apix[1] && apix[1] < imageSize[1]);
  
  if (pixIn && !apixIn) {
    
    // Likely the exact projection is not accurate
    double dist = vw::math::norm_2(pix - apix);  
    if (m_maxApproxCamPixErr > 0 && dist > 1.5 * m_maxApproxCamPixErr) {
      
      double gsd = 0.0;
      UsgsAstroLsSensorModel * ls_model
          = dynamic_cast<UsgsAstroLsSensorModel*>(m_gm_model.get());
      if (ls_model != NULL)
        gsd = ls_model->m_gsd;
      
      if (gsd > 0) {
        // Do a geometric check. Project from the camera to the ground and see
        // if we get close enough to the input xyz.
        double dist_to_ground = vw::math::norm_2(xyz - this->camera_center(pix)); 
        vw::Vector3 xyz2 = this->camera_center(pix) 
          + this->pixel_to_vector(pix) * dist_to_ground;
        if (vw::math::norm_2(xyz - xyz2) > 10 * gsd) 
          return apix; // Return the approximate pixel
      }
    }
  }

  // Return the exact pixel
  return pix;
}

} // end namespace asp
