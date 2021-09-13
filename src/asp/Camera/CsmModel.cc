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
#include <usgscsm/UsgsAstroLsSensorModel.h>
#include <usgscsm/UsgsAstroFrameSensorModel.h>
#include <usgscsm/UsgsAstroSarSensorModel.h>
#include <usgscsm/Utilities.h>

#include <ale/Rotation.h>
#include <Eigen/Geometry>

#include <streambuf>

namespace dll = boost::dll;
namespace fs = boost::filesystem;
using json = nlohmann::json;

// This was discussed with the USGS folks. To convert from ISIS to ASP
// pixels we subtract 1.0. To convert from CSM pixels we have to
// subtract only 0.5.
const vw::Vector2 ASP_TO_CSM_SHIFT(0.5, 0.5);

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
CsmModel::CsmModel():m_semi_major_axis(0.0), m_semi_minor_axis(0.0) {
}

CsmModel::CsmModel(std::string const& isd_path) {
  load_model(isd_path);
}

CsmModel::~CsmModel() {
  // nothing to do.
}

bool CsmModel::file_has_isd_extension(std::string const& path) {
  std::string ext = vw::get_extension(path);
  return ((ext == ".json") || (ext == ".isd"));
}

std::string CsmModel::get_csm_plugin_folder(){

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
    vw_throw( ArgumentErr() << "Could not find CSM plugin folder: " << plugin_path << ".\n"
              << "Check the value of the environmental variable CSM_PLUGIN_PATH.");
  }

  return plugin_path;
}

// The original idea here was to look at every library in the plugins
// directory and load the valid plugins. For now however there is just
// one plugin, libusgscsm, and it is stored in 'lib', among thousands
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
    vw_throw( ArgumentErr() << "Cannot find plugin: " <<plugin <<
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
  ifs >> json_isd;

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
    vw::vw_throw( vw::ArgumentErr() << "Unknown unit for the ellipsoid radii in "
                  << isd_path << ". The read value is: " << unit);
  }

  // Sanity check
  if (m_semi_major_axis <= 0.0 || m_semi_minor_axis <= 0.0) 
    vw::vw_throw( vw::ArgumentErr() << "Could not read positive semi-major "
                  << "and semi-minor axies from:  " << isd_path
                  << ". The read values are: "
                  << m_semi_major_axis << ' ' << m_semi_minor_axis);
}

// Apply a rotation to a vector of quaternions.
// Maybe one day this will move to the usgscsm package.
void applyRotationToQuatVec(ale::Rotation const& r, std::vector<double> & quaternions) {
  int num_quat = quaternions.size();
  
  for (int it = 0; it < num_quat/4; it++) {

    double * q = &quaternions[4*it];

    // Note that quaternion in q is stored in order x, y, z, w, while
    // the rotation matrix wants it as w, x, y, z.
    
    std::vector<double> trans_q = (r * ale::Rotation(q[3], q[0], q[1], q[2])).toQuaternion();

    // Modify in-place
    q[0] = trans_q[1];
    q[1] = trans_q[2];
    q[2] = trans_q[3];
    q[3] = trans_q[0];
  }
  
}

// Apply a rotation and translation to a vector of xyz vectors.
// Maybe one day this will move to the usgscsm package.
void applyRotationTranslationToXyzVec(ale::Rotation const& r, ale::Vec3d const& t,
                                      std::vector<double> & xyz) {
  
  int num_xyz = xyz.size();
  for (int it = 0; it < num_xyz/3; it++) {
    
    double * p = &xyz[3*it];

    ale::Vec3d p_vec(p[0], p[1], p[2]);
    
    // Apply the rotation
    p_vec = r(p_vec);

    // Apply the translation
    p_vec += t;

    // Modify in-place
    p[0] = p_vec.x;
    p[1] = p_vec.y;
    p[2] = p_vec.z;
    
  }
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
  bool is_model_state = (line == UsgsAstroFrameSensorModel::_SENSOR_MODEL_NAME || 
                         line == UsgsAstroLsSensorModel::_SENSOR_MODEL_NAME    ||
                         line == UsgsAstroSarSensorModel::_SENSOR_MODEL_NAME);

  if (!is_model_state) 
    CsmModel::load_model_from_isd(isd_path);
  else
    CsmModel::load_model_from_state(isd_path);
}
  
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
    vw::vw_throw( vw::ArgumentErr() << "Unable to construct a camera model for the ISD file "
                        << isd_path << " using any of the loaded CSM plugins!");
  }

  vw_out() << "Using plugin: " << csm_plugin->getPluginName() 
           << " with model name " << model_name << std::endl;

  // Now try to construct the camera model
  csm::WarningList warnings;
  csm::Model* csm_model
    = csm_plugin->constructModelFromISD(support_data, model_name, &warnings);

  // Error checking
  csm::WarningList::const_iterator w_iter;
  for (w_iter = warnings.begin(); w_iter!=warnings.end(); ++w_iter) {
    vw_out() << "CSM Warning: " << w_iter->getMessage() << std::endl;
  }

  // Handle load failure
  if (!csm_model)
    vw::vw_throw(vw::ArgumentErr() << "Failed to load CSM sensor model from file: "
                 << isd_path);
  
  // TODO: Are all sensor models going to be this type (RasterGM)?
  //       Otherwise we can use the result of getModelFamily() to choose the class.
  // Cast the model we got to the child class with the needed functionality.
  csm::RasterGM* raster_model = dynamic_cast<csm::RasterGM*>(csm_model);

   // Handle load failure
  if (!raster_model)
    vw::vw_throw( vw::ArgumentErr() << "Failed to cast CSM sensor model to raster type!");
  
  m_csm_model.reset(raster_model); // We will handle cleanup of the model.
}

/// Load the camera model from a model state written to disk.
/// A model state is obtained from an ISD model by pre-processing
/// and combining its data in a form ready to be used.
void CsmModel::load_model_from_state(std::string const& state_path) {

  // Read the state as one string
  std::ifstream ifs(state_path.c_str());
  std::string model_state;
  ifs.seekg(0, std::ios::end);   
  model_state.reserve(ifs.tellg());
  ifs.seekg(0, std::ios::beg);
  model_state.assign((std::istreambuf_iterator<char>(ifs)),
             std::istreambuf_iterator<char>());
  ifs.close();

  // See which model to load, then cast it to RasterGM. This could
  // have been simpler if the USGSCSM models shared a base class where
  // all shared functionality would be shared.
  csm::RasterGM* raster_model = NULL;
  if (model_state.rfind(UsgsAstroFrameSensorModel::_SENSOR_MODEL_NAME, 0) == 0) {
    
    UsgsAstroFrameSensorModel * model = new UsgsAstroFrameSensorModel;
    model->replaceModelState(model_state);
    raster_model = dynamic_cast<csm::RasterGM*>(model);
    
  } else if (model_state.rfind(UsgsAstroLsSensorModel::_SENSOR_MODEL_NAME, 0) == 0) {
    
    UsgsAstroLsSensorModel * model = new UsgsAstroLsSensorModel;
    model->replaceModelState(model_state);
    raster_model = dynamic_cast<csm::RasterGM*>(model);
    
  } else if (model_state.rfind(UsgsAstroSarSensorModel::_SENSOR_MODEL_NAME, 0) == 0) {
    
    UsgsAstroSarSensorModel * model = new UsgsAstroSarSensorModel;
    model->replaceModelState(model_state);
    raster_model = dynamic_cast<csm::RasterGM*>(model);
    
  } else {
    vw::vw_throw(vw::ArgumentErr() << "Could not load the model state from: "
                 << state_path << ".\n");
  }

  // Set the semi-axes from json (cannot pull it from the usgs models
  // as these figure as private in some of them).
  auto j = stateAsJson(model_state);
  m_semi_major_axis = j["m_majorAxis"];
  m_semi_minor_axis = j["m_minorAxis"];

  // Sanity check
  if (m_semi_major_axis <= 0.0 || m_semi_minor_axis <= 0.0) 
    vw::vw_throw( vw::ArgumentErr() << "Could not read positive semi-major "
                  << "and semi-minor axies from:  " << state_path
                  << ". The read values are: "
                  << m_semi_major_axis << ' ' << m_semi_minor_axis);

  // Handle load failure
  if (!raster_model)
    vw::vw_throw(vw::ArgumentErr() << "Failed to cast linescan model to raster type.");
  
  m_csm_model.reset(raster_model); // We will handle cleanup of the model
}
  
void CsmModel::throw_if_not_init() const {
  if (!m_csm_model)
    vw_throw( ArgumentErr() << "CsmModel: Sensor model has not been loaded yet!" );
}

// TODO: Check all of the warnings

vw::Vector2 CsmModel::get_image_size() const {
  throw_if_not_init();

  csm::ImageVector size = m_csm_model->getImageSize();
  return Vector2(size.samp, size.line);
}

Vector2 CsmModel::point_to_pixel(Vector3 const& point) const {
  throw_if_not_init();

  csm::EcefCoord  ecef = vectorToEcefCoord(point);

  // TODO(oalexan1): Think about how many digits of precision are
  // needed here.  For bundle adjustment may need many. But it will
  // slow down mapproject.
  double desiredPrecision = 1e-8;
  double achievedPrecision;
  csm::WarningList warnings;
  csm::WarningList * warnings_ptr = NULL;

  // Do not show warnings, it becomes too verbose
  bool show_warnings = false;
  if (show_warnings) 
    warnings_ptr = &warnings;

  csm::ImageCoord imagePt = m_csm_model->groundToImage(ecef, desiredPrecision,
						       &achievedPrecision, warnings_ptr);

  if (show_warnings) {
    csm::WarningList::const_iterator w_iter;
    for (w_iter = warnings.begin(); w_iter!=warnings.end(); ++w_iter) {
      vw_out() << "CSM Warning: " << w_iter->getMessage() << std::endl;
    }
  }

  return imageCoordToVector(imagePt) - ASP_TO_CSM_SHIFT;
}

Vector3 CsmModel::pixel_to_vector(Vector2 const& pix) const {
  throw_if_not_init();

  csm::ImageCoord imagePt = vectorToImageCoord(pix + ASP_TO_CSM_SHIFT);

  // Camera center
  csm::EcefCoord  ctr = m_csm_model->getSensorPosition(imagePt);

  // Ground point
  double desiredPrecision  = 1e-8; 
  double achievedPrecision = -1.0; // will be modified in the function
  double groundHeight      = 0.0;
  csm::EcefCoord groundPt = m_csm_model->imageToGround(imagePt, groundHeight, desiredPrecision,
                                                       &achievedPrecision);
  

  // Normalized direction
  Vector3 dir0 = ecefCoordToVector(groundPt) - ecefCoordToVector(ctr);
  dir0 = dir0 / norm_2(dir0);
  
  return dir0;

#if 0
  // This does not work correctly for the SAR sensor.
  // TODO(oalexan1): When this is fixed, go back to the code below as that is
  // the more efficient way of doing it, presumably.
  
  // This function generates the vector from the camera at the camera origin,
  //  there is a different call that gets the vector near the ground.
  csm::EcefLocus locus = m_csm_model->imageToRemoteImagingLocus(imagePt,
                                                                desiredPrecision,
                                                                &achievedPrecision);
  Vector3 dir = ecefVectorToVector(locus.direction);

  return dir;
#endif
}

Vector3 CsmModel::camera_center(Vector2 const& pix) const {
  throw_if_not_init();

  csm::ImageCoord imagePt = vectorToImageCoord(pix + ASP_TO_CSM_SHIFT);
  csm::EcefCoord  ecef    = m_csm_model->getSensorPosition(imagePt);

  return ecefCoordToVector(ecef);
}

// Apply a transform to the model and save the transformed state as a JSON file.
void CsmModel::save_transformed_json_state(std::string const& json_state_file,
                                           vw::Matrix4x4 const& transform) {

  csm::RasterGM* raster_model = dynamic_cast<csm::RasterGM*>(this->m_csm_model.get());

  // TODO(oalexan1): Make this work for Frame and SAR
  
  UsgsAstroLsSensorModel * ls_model
    = dynamic_cast<UsgsAstroLsSensorModel*>(this->m_csm_model.get());
  
  if (ls_model == NULL) {
    vw_out() << "Saving the adjusted JSON state is implemented only "
             << "for usgscsm linescan camera models.\n";
    return;
  }

  std::string modelState = ls_model->getModelState();

  nlohmann::json j = stateAsJson(modelState);

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

  // Zero translation
  ale::Vec3d zero_t(0, 0, 0);
  
  // Sensor rotations
  // TODO(oalexan1): When USGSCSM releases the version having these
  // functions then use those.
  std::vector<double> quaternions = j["m_quaternions"].get<std::vector<double>>();
  asp::applyRotationToQuatVec(r, quaternions);
  j["m_quaternions"] = quaternions;
  
  // Sensor positions
  std::vector<double> positions = j["m_positions"].get<std::vector<double>>();;
  asp::applyRotationTranslationToXyzVec(r, t, positions);
  j["m_positions"] = positions;
  
  // For velocities, the translation does not get added
  std::vector<double> velocities = j["m_velocities"].get<std::vector<double>>();;
  asp::applyRotationTranslationToXyzVec(r, zero_t, velocities);
  j["m_velocities"] = velocities;
  
  // We do not change the sun position or velocity. The idea is that
  // the sun is so far, that minor adjustments in camera location
  // won't affect where the Sun is.

  int indent = 2; // Write on multiple lines rather than using a single giant line
  std::string stateString = ls_model->getModelName() + "\n" + j.dump(indent);

  vw_out() << "Writing adjusted JSON state: " << json_state_file << std::endl;
  std::ofstream ofs(json_state_file.c_str());
  ofs << stateString << std::endl;
  ofs.close();

  return;
}
  
} // end namespace asp

