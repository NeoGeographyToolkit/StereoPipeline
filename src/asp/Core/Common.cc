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

#include <asp/Core/Common.h>
#include <asp/Core/StereoSettings.h>
#include <asp/Core/EnvUtils.h>

#include <asp/asp_date_config.h>

#include <vw/Core/Log.h>
#include <vw/Core/System.h>
#include <vw/Math/BBox.h>
#include <vw/FileIO/DiskImageResource.h>
#include <vw/FileIO/FileUtils.h>
#include <vw/FileIO/FileTypes.h>

#include <boost/algorithm/string.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/filesystem/operations.hpp>
#include <boost/filesystem/path.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/program_options.hpp>
#include <boost/dll.hpp>
#include <boost/algorithm/string/replace.hpp>

// TODO(oalexan1): Move this to VW in the cartography module.
#include <gdal_version.h>
#if defined(VW_HAVE_PKG_GDAL) && VW_HAVE_PKG_GDAL==1
#include "ogr_spatialref.h"
#endif

#include <map>
#include <sstream>
#include <string>
#include <vector>
#include <unistd.h>

// These variables must never go out of scope or else the
// environmental variables set by them using putenv() will disappear.
// Use C style strings, rather than std::string, as then putenv() and
// getenv() give valgrind warnings.
namespace asp {
  // TODO(oalexan1): Wipe al these
  const int COMMON_BUF_SIZE = 5120;
  char ISISROOT_ENV_STR[COMMON_BUF_SIZE];
  char QT_PLUGIN_PATH_ENV_STR[COMMON_BUF_SIZE];
  char GDAL_DATA_ENV_STR[COMMON_BUF_SIZE];
  char GDAL_DRIVER_PATH_ENV_STR[COMMON_BUF_SIZE];
  char PROJ_LIB_ENV_STR[COMMON_BUF_SIZE]; // older proj api
  char PROJ_DATA_ENV_STR[COMMON_BUF_SIZE]; // newer proj api
  char LC_ALL_STR[COMMON_BUF_SIZE];
  char LANG_STR[COMMON_BUF_SIZE];
}

using namespace vw;
namespace po = boost::program_options;
namespace fs = boost::filesystem;

// Given a vector of files, with each file being an image, camera,
// or a text file having images or cameras, return the list of
// all found images and cameras. This is a local auxiliary 
// function not exposed in the header file.
void readImagesCamsOrLists(std::vector<std::string> const & in,
                           std::vector<std::string>       & out) {

  // Wipe the output
  out.clear();

  for (size_t i = 0; i < in.size(); i++) {

    if (vw::has_image_extension(in[i]) || vw::has_cam_extension(in[i])) {
    
      // Simply append the image or camera to the list
      out.push_back(in[i]);
      
    } else {

      // Read the list, append all entries from it
      std::string ext = vw::get_extension(in[i]);
      if (ext == ".txt") {

        std::vector<std::string> list;
        asp::read_list(in[i], list);
        for (size_t j = 0; j < list.size(); j++) 
            out.push_back(list[j]);
            
      } else if (boost::iends_with(in[i], ".adjust")) {
        vw_throw(ArgumentErr() << "The file " << in[i] << " is an adjustment. "
                  << "Use the original cameras and the option "
                  << "--bundle-adjust-prefix.\n");
      } else {
        vw::vw_throw(vw::ArgumentErr() << "Unknown file type passed on input: "
          << in[i] << ".\n");
      }
    }
  }

  return;
} 

// Given a list of images/cameras and/or lists of such things, put the images
// and the cameras in separate vectors.
void asp::separate_images_from_cameras(std::vector<std::string> const& inputs,
                                       std::vector<std::string>      & images,
                                       std::vector<std::string>      & cameras,
                                       bool ensure_equal_sizes){

  // There are N images and possibly N camera paths.
  // There are several situations:
  // 1. img1.cub ... imgN.cub for ISIS with non-proj images
  // 2. img1.tif ... imgN.tif img1.cub ... imgN.cub for ISIS with proj images
  // 3. img1.tif ... imgN.tif for RPC with embedded RPC in the tif files 
  // 4. img1.tif ... imgN.tif cam1 .... camN for all other cases.

  // Consider the case when some of the inputs are lists of images/cameras
  std::vector<std::string> inputs2;
  readImagesCamsOrLists(inputs, inputs2);

  // Check that all files exist
  for (size_t i = 0; i < inputs2.size(); i++) {
    if (!fs::exists(inputs2[i])) {
      vw_throw(ArgumentErr() << "Cannot find the file: " << inputs2[i] << ".\n");
      return;
    }
  }
  
  // Images and cameras may be interleaved. Separate them.
  images.clear();
  cameras.clear();
  for (size_t i = 0; i < inputs2.size(); i++) {
    if (vw::has_image_extension(inputs2[i]))
      images.push_back(inputs2[i]);
    else if (vw::has_cam_extension(inputs2[i]))
      cameras.push_back(inputs2[i]);
    else if (boost::iends_with(inputs2[i], ".adjust"))
      vw_throw(ArgumentErr() << "The file " << inputs2[i] << " is an adjustment. "
                << "Use the original cameras and the option "
                << "--bundle-adjust-prefix.\n");  
    else 
      vw_throw(ArgumentErr() << "Unknown file type passed on input: "
                << inputs2[i] << ".\n");
  }

  // Then concatenate them again, but with the images first and the cameras
  // second.
  inputs2.clear();
  for (size_t i = 0; i < images.size(); i++)  inputs2.push_back(images[i]);
  for (size_t i = 0; i < cameras.size(); i++) inputs2.push_back(cameras[i]);
  images.clear();
  cameras.clear();

  // See if we have cub files and/or camera files (.cub files are also cameras)  
  bool has_cub    = false;
  bool has_nocub  = false;
  bool has_cam    = false;
  for (size_t i = 0; i < inputs2.size(); i++) {
    std::string ext = vw::get_extension(inputs2[i]);
    if (ext == ".cub")                      has_cub   = true;
    if (ext != ".cub")                      has_nocub = true;
    if (vw::has_cam_extension(inputs2[i])) has_cam   = true;
  }
  
  // Let the first half of the data be images, and the second half be cameras.
  // Unless we have only .cub files, when all the data are images.
  if ((has_cub && !has_nocub) || (!has_cam)) {
    // Only cubes, or only non-cameras, cases 1 and 3 above
    for (size_t i=0; i < inputs2.size(); ++i) 
      images.push_back(inputs2[i]);
  } else {
    // Images and cameras (cameras could be cubes)
    if (inputs2.size() % 2 != 0)
      vw_throw(ArgumentErr() << "Expecting as many images as cameras.\n");
    
    int half = inputs2.size()/2;
    for (int i = 0;    i < half;   i++) images.push_back(inputs2[i]);
    for (int i = half; i < 2*half; i++) cameras.push_back(inputs2[i]);
  }
  
  // Verification for images
  for (size_t i = 0; i < images.size(); i++) {
    if (!vw::has_image_extension(images[i])) {
      vw_throw(ArgumentErr() << "Expecting an image, got: " << images[i] << ".\n");
    }
  }

  // Verification for cameras
  for (size_t i = 0; i < cameras.size(); i++) {
    if (!vw::has_cam_extension(cameras[i])) {
      vw_throw(ArgumentErr() << "Expecting a camera, got: " << cameras[i] << ".\n");
    }
  }

  if (images.size() != cameras.size() && !cameras.empty()) {
    vw_throw(ArgumentErr() << "Expecting the number of images and cameras to agree.\n");
  }

  if (ensure_equal_sizes) {
    while (cameras.size() < images.size())
      cameras.push_back("");
  }
  
  return;
}

/// Parse 'VAR1=VAL1 VAR2=VAL2' into a map. Note that we append to the map,
/// so it may have some items there beforehand.
void asp::parse_append_metadata(std::string const& metadata,
                                std::map<std::string, std::string> & keywords){
  
  std::istringstream is(metadata);
  std::string meta, var, val;
  while (is >> meta){
    boost::replace_all(meta, "=", " ");  // replace equal with space
    std::istringstream is2(meta);
    if (!(is2 >> var >> val)) 
      vw_throw(ArgumentErr() << "Could not parse: " << meta << "\n");
    keywords[var] = val;
  }
}

// Print time function
std::string asp::current_posix_time_string() {
  return boost::posix_time::to_simple_string(boost::posix_time::second_clock::local_time());
}

// Run a system command and append the output to a given file
void asp::run_cmd_app_to_file(std::string cmd, std::string file){
  std::string full_cmd;
  full_cmd = "echo '" + cmd + "' >> " + file; // echo the command to run
  int code = system(full_cmd.c_str());
  full_cmd = cmd + " >> " + file + " 2>&1";
  code = system(full_cmd.c_str());
  full_cmd = "echo '' >> " + file; // append a newline
  code = system(full_cmd.c_str());
}

std::string asp::extract_prog_name(std::string const& prog_str){

  // Get program name without path and leading 'lt-'.
  std::string prog_name = fs::path(prog_str).stem().string();
  std::string pref = "lt-";
  size_t lp = pref.size();
  if (prog_name.size() >= lp && prog_name.substr(0, lp) == pref)
    prog_name = prog_name.substr(lp, prog_name.size() - lp);

  return prog_name;
}

void asp::log_to_file(int argc, char *argv[],
                      std::string stereo_default_filename,
                      std::string out_prefix){

  // Log some system info to a file, then copy the vw log
  // info to that file as well.

  if (out_prefix == "")
    vw::vw_throw(vw::ArgumentErr() << "Output prefix was not set.\n");

  // Create the output directory if not present
  vw::create_out_dir(out_prefix);

  std::string prog_name = extract_prog_name(argv[0]);

  // Create the log file and open it in write mode
  std::ostringstream os;
  int pid = getpid();
  std::string timestamp = 
      boost::posix_time::to_iso_string(boost::posix_time::second_clock::local_time());
  std::string clean_timestamp = timestamp.substr(4, 9); // Trim off the year
  clean_timestamp.replace(4, 1, 1, '-'); // Replace T with -
  clean_timestamp.insert (2, 1,    '-'); // Insert - between month and day
  os << out_prefix << "-log-" << prog_name << "-" 
     << clean_timestamp << "-" << pid << ".txt";
  std::string log_file = os.str();
  vw_out() << "Writing log: " << log_file << std::endl;
  std::ofstream lg(log_file.c_str());

  // Write the version
  lg << "ASP " << ASP_VERSION << "\n";

#if defined(ASP_COMMIT_ID)
    lg << "Build ID: " << ASP_COMMIT_ID << "\n";
#endif
#if defined(ASP_BUILD_DATE)
    lg << "Build date: " << ASP_BUILD_DATE << "\n";
#endif

    lg << "\n"; // leave some separation
    
    // Write the program name and its arguments
    for (int s = 0; s < argc; s++) {
      std::string token = std::string(argv[s]);
      // Skip adding empty spaces
      if (token == " ")
        continue;
      // Use quotes if there are spaces
      if (token.find(" ") != std::string::npos || token.find("\t") != std::string::npos) 
        token = '"' + token + '"';
      lg << token + " ";
    }
    
  lg << std::endl << std::endl;

  // Must ensure to close the file handle before further appending to
  // it below.
  lg.close();

  // System calls. Not all will succeed on all machines.
  asp::run_cmd_app_to_file("uname -a", log_file);
  if (fs::exists("/proc/meminfo")) 
    asp::run_cmd_app_to_file("cat /proc/meminfo 2>/dev/null | grep MemTotal", log_file);
  if (fs::exists("/proc/cpuinfo")) 
    asp::run_cmd_app_to_file("cat /proc/cpuinfo 2>/dev/null | tail -n 25", log_file);
  
  // The line below is for MacOSX
  asp::run_cmd_app_to_file("sysctl -a hw 2>/dev/null | grep -E \"ncpu|byteorder|memsize|cpufamily|cachesize|mmx|sse|machine|model\" | grep -v ipv6", log_file);
  if (stereo_default_filename != "" && fs::exists(stereo_default_filename)) {
    std::string cmd = "cat " + stereo_default_filename + " 2>/dev/null";
    asp::run_cmd_app_to_file(cmd, log_file);
  }

  // Save the current .vwrc
  char * home_ptr = getenv("HOME");
  if (home_ptr) {
    std::string home = home_ptr;
    std::string vwrc = home + "/.vwrc";
    if (fs::exists(vwrc)) {
      std::string cmd = "cat " + vwrc + " 2>/dev/null";
      asp::run_cmd_app_to_file(cmd, log_file);
    }
  }
  
  // Copy all the info going to the console to log_file as well,
  // except the progress bar.
  boost::shared_ptr<vw::LogInstance> current_log(new vw::LogInstance(log_file));
  current_log->rule_set() = vw_log().console_log().rule_set();
  current_log->rule_set().add_rule(0, "*.progress");
  vw_log().add(current_log);
}

// A function to set the environmental variables ISISROOT, QT_PLUGIN_PATH,
// GDAL_DATA, and PROJ_LIB. In packaged build mode, set these with the help of the
// base directory of the distribution.
// In dev mode, use the ASP_DEPS_DIR macro or otherwise the
// ASP_DEPS_DIR environmental variable. These are needed especially for the
// conda build, when the ASP executables don't have a wrapper around them.
// For the tarball build, some of this logic is duplicated in the script
// in BinaryBuilder/dist-add/libexec/libexec-funcs.sh which is then called
// by the wrapper.
namespace asp {
void set_asp_env_vars() {
    
  // Find the path to the base of the package and see if it works.
  std::string base_dir = boost::dll::program_location().parent_path().parent_path().string();
  if (!fs::exists(base_dir + "/IsisPreferences")) {
    base_dir = ASP_DEPS_DIR; // This is defined at compile time
    if (!fs::exists(base_dir + "/IsisPreferences")) {
      // If nothing works, try the ASP_DEPS_DIR env variable
      char * ptr = getenv("ASP_DEPS_DIR");
      if (ptr) 
        base_dir = ptr;
      if (ptr == NULL || !fs::exists(base_dir + "/IsisPreferences")) {
        vw::vw_throw(vw::ArgumentErr() << "Cannot find the directory having IsisPreferences. "
                     << "Try setting it as the environmental variable ASP_DEPS_DIR.");
      }
    }
  }

  // Set ISISROOT and check for IsisPreferences
  // ISISROOT_ENV_STR = "ISISROOT=" + base_dir;
  snprintf(ISISROOT_ENV_STR, COMMON_BUF_SIZE, "ISISROOT=%s", base_dir.c_str());
  asp::setEnvVar("ISISROOT", base_dir);
  if (!fs::exists(std::string(getenv("ISISROOT")) + "/IsisPreferences")) 
    vw::vw_throw(vw::ArgumentErr() << "Cannot find IsisPreferences in "
                 << getenv("ISISROOT"));
  
  // Set QT_PLUGIN_PATH as the path to /plugins
  // QT_PLUGIN_PATH_ENV_STR = "QT_PLUGIN_PATH=" + base_dir + "/plugins";
  snprintf(QT_PLUGIN_PATH_ENV_STR, COMMON_BUF_SIZE, "QT_PLUGIN_PATH=%s%s",
           base_dir.c_str(), "/plugins");
  asp::setEnvVar("QT_PLUGIN_PATH", base_dir + "/plugins");
  if (!fs::exists(std::string(getenv("QT_PLUGIN_PATH"))))
    vw::vw_throw(vw::ArgumentErr() << "Cannot find Qt plugins in " 
                 << getenv("QT_PLUGIN_PATH"));

  // Set GDAL_DATA and check for share/gdal
  snprintf(GDAL_DATA_ENV_STR, COMMON_BUF_SIZE, "GDAL_DATA=%s%s",
           base_dir.c_str(), "/share/gdal");
  asp::setEnvVar("GDAL_DATA", base_dir + "/share/gdal");  
  if (!fs::exists(std::string(getenv("GDAL_DATA")))) 
    vw::vw_throw(vw::ArgumentErr() << "Cannot find GDAL data in "
                 << getenv("GDAL_DATA"));

  // Set GDAL_DRIVER_PATH and check for share/gdal. There are two locations, because
  // BinaryBuilder moves the plugins from lib/gdalplugins to lib.
  // TODO(oalexan1): Figure out why this happens.
  snprintf(GDAL_DRIVER_PATH_ENV_STR, COMMON_BUF_SIZE, "GDAL_DRIVER_PATH=%s%s:%s%s",
           base_dir.c_str(), "/lib/gdalplugins", base_dir.c_str(), "/lib");
  asp::setEnvVar("GDAL_DRIVER_PATH", base_dir + "/lib/gdalplugins:" + base_dir + "/lib");  
  
  // Older proj api
  // Set PROJ_LIB and check for share/proj
  snprintf(PROJ_LIB_ENV_STR, COMMON_BUF_SIZE, "PROJ_LIB=%s%s",
           base_dir.c_str(), "/share/proj");
  asp::setEnvVar("PROJ_LIB", base_dir + "/share/proj");
  if (!fs::exists(std::string(getenv("PROJ_LIB")))) 
    vw::vw_throw(vw::ArgumentErr() << "Cannot find PROJ data in "
                 << getenv("PROJ_LIB"));

  // Newer proj api
  // Set PROJ_DATA and check for share/proj
  snprintf(PROJ_DATA_ENV_STR, COMMON_BUF_SIZE, "PROJ_DATA=%s%s",
           base_dir.c_str(), "/share/proj");
  asp::setEnvVar("PROJ_DATA", base_dir + "/share/proj");
  if (!fs::exists(std::string(getenv("PROJ_DATA")))) 
    vw::vw_throw(vw::ArgumentErr() << "Cannot find PROJ data in "
                 << getenv("PROJ_DATA"));

  // Force the US English locale as long as ASP is running to avoid
  // ISIS choking on a decimal separator which shows up as a comma for 
  // some reason.
  snprintf(LC_ALL_STR, COMMON_BUF_SIZE, "LC_ALL=en_US.UTF-8");
  asp::setEnvVar("LC_ALL", "en_US.UTF-8");  
  
  snprintf(LANG_STR, COMMON_BUF_SIZE, "LANG=en_US.UTF-8");
  asp::setEnvVar("LANG", "en_US.UTF-8");
}
} // end namespace asp

// User should only put the arguments to their application in the
// usage_comment argument. We'll finish filling in the repeated information.
po::variables_map
asp::check_command_line(int argc, char *argv[], vw::GdalWriteOptions& opt,
                        po::options_description const& public_options,
                        po::options_description const& all_public_options,
                        po::options_description const& positional_options,
                        po::positional_options_description const& positional_desc,
                        std::string & usage_comment,
                        bool allow_unregistered,
                        std::vector<std::string> & unregistered) {

  unregistered.clear();

  // Ensure that opt gets all needed fields from vw::GdalWriteOptionsDescription().
  // This is needed not only for stereo, but for all tools using vw::GdalWriteOptions.
  stereo_settings().initialize(opt);

  // Finish filling in the usage_comment.
  std::ostringstream ostr;
  ostr << "Usage: " << argv[0] << " " << usage_comment << "\n\n";
  ostr << "  [ASP " << ASP_VERSION << "]\n";
#if defined(ASP_BUILD_DATE)
  ostr << "  Build date: " << ASP_BUILD_DATE << "\n";
#endif
  ostr << "\n";
  
  usage_comment = ostr.str();

  asp::set_asp_env_vars();
  
  // We distinguish between all_public_options, which is all the
  // options we must parse, even if we don't need some of them, and
  // public_options, which are the options specifically used by the
  // current tool, and for which we also print the help message.
  po::variables_map vm;
  try {
    po::options_description all_options;
    all_options.add(all_public_options).add(positional_options);

    if (allow_unregistered) {
      po::parsed_options parsed = 
        po::command_line_parser(argc, argv).options(all_options).allow_unregistered()
         .style(po::command_line_style::unix_style).run();
      unregistered = collect_unrecognized(parsed.options, po::include_positional);
      po::store(parsed, vm);
    } else {
      po::parsed_options parsed = 
        po::command_line_parser(argc, argv).options(all_options).positional(positional_desc)
         .style(po::command_line_style::unix_style).run();
      po::store(parsed, vm);
    }

    po::notify(vm);
  } catch (po::error const& e) {
    vw::vw_throw(vw::ArgumentErr() << "Error parsing input:\n"
                  << e.what() << "\n" << usage_comment << public_options);
  }

  // We really don't want to use BIGTIFF unless we have to. It's
  // hard to find viewers for bigtiff.
  if (vm.count("no-bigtiff")) {
    opt.gdal_options["BIGTIFF"] = "NO";
  } else {
    opt.gdal_options["BIGTIFF"] = "IF_SAFER";
  }

  if (vm.count("help")) {
    vw::vw_out() << usage_comment << public_options << "\n";
    exit(0);
  }

  if (vm.count("version")) {
    std::ostringstream ostr;
    ostr << ASP_PACKAGE_STRING  << "\n";
#if defined(ASP_COMMIT_ID)
    ostr << "  Build ID: " << ASP_COMMIT_ID << "\n";
#endif
#if defined(ASP_BUILD_DATE)
    ostr << "  Build date: " << ASP_BUILD_DATE << "\n";
#endif
    ostr << "\nBuilt against:\n  " << VW_PACKAGE_STRING << "\n";
#if defined(VW_COMMIT_ID)
    ostr << "    Build ID: " << VW_COMMIT_ID << "\n";
#endif
#if defined(ASP_HAVE_PKG_ISISIO) && ASP_HAVE_PKG_ISISIO == 1
    ostr << "  USGS ISIS " << ASP_ISIS_VERSION << "\n";
#endif
    ostr << "  Boost C++ Libraries " << ASP_BOOST_VERSION << "\n";
    ostr << "  GDAL " << GDAL_RELEASE_NAME << " | " << GDAL_RELEASE_DATE << "\n";
    vw::vw_out() << ostr.str() << "\n";
    exit(0);
  }

  opt.setVwSettingsFromOpt();
  
  return vm;
}

// TODO(oalexan1): Move this to VW in the cartography module
void asp::set_srs_string(std::string srs_string, bool have_user_datum,
                         vw::cartography::Datum const& user_datum,
                         vw::cartography::GeoReference & georef) {

  // When an EPSG code is provided, store the name so that
  //  it shows up when the GeoReference object is written
  //  out to disk.
  if (srs_string.find("EPSG") != std::string::npos)
    georef.set_projcs_name(srs_string);

  // Set srs_string into given georef. Note that this may leave the
  // georef's affine transform inconsistent.

  // TODO: The line below needs more thought
  if (srs_string == "")
    srs_string = "+proj=longlat";

  // TODO(oalexan1): Use below datum.get_wkt().
  // But then cannot concatenate the wkt. Need to have a way
  // of reconciling the two wkt. May need to first call 
  // set_datum, and then set_wkt. The latter will 
  // try to reconcile the wkt with the prior datum.
  // But this needs testing, especially in cases when 
  // srs_string lacks the datum info.
  if (have_user_datum)
    srs_string += " " + user_datum.proj4_str();
  
  // TODO(oalexan1): Rename set_wkt to set_srs.
  // TODO(oalexan1): Must wipe all this and call directly georef.set_wkt.
  // TODO(oalexan1): Deal with datum name!
  vw::cartography::GeoReference input_georef = georef;
  OGRSpatialReference gdal_spatial_ref;
  if (gdal_spatial_ref.SetFromUserInput(srs_string.c_str()) != OGRERR_NONE)
    vw::vw_throw(vw::ArgumentErr() << "Failed to parse: \"" << srs_string << "\".");
  char *wkt_str_tmp = NULL;
  gdal_spatial_ref.exportToWkt(&wkt_str_tmp);
  srs_string = wkt_str_tmp;
  CPLFree(wkt_str_tmp);
  georef.set_wkt(srs_string);
}

// Write a vector of strings from a file, one per line.
void asp::write_list(std::string const& file, std::vector<std::string> const & list) {
  std::ofstream fh(file.c_str());
  for (size_t i = 0; i < list.size(); i++)
    fh << list[i] << std::endl;
  fh.close();
}

// Read a vector of strings from a file, with spaces and newlines acting as separators.
// Throw an exception if the list is empty.
void asp::read_list(std::string const& file, std::vector<std::string> & list) {
  list.clear();
  std::ifstream fh(file);
  std::string val;
  while (fh >> val)
    list.push_back(val);

  if (list.empty())
    vw_throw(ArgumentErr() << "Could not read any entries from: " << file << ".\n");

  fh.close();
}

// Read a vector of doubles from a file
void asp::read_vec(std::string const& filename, std::vector<double> & vals) {
  vals.clear();
  std::ifstream ifs(filename.c_str());
  if (!ifs.good()) 
    vw_throw(ArgumentErr() << "Could not open file: " << filename);
  
  double val;
  while (ifs >> val)
    vals.push_back(val);
  ifs.close();
}

// Read the target name (planet name) from the plain text portion of an ISIS cub
// file or from a CSM file. Will return "UNKNOWN" if the target name cannot be
// found.
std::string asp::read_target_name(std::string const& filename) {

  std::string target = "UNKNOWN";
  
  std::ifstream handle;
  handle.open(filename.c_str());
  if (handle.fail())
    return target; // No luck
  
  std::string line;
  int count = 0;
  while (getline(handle, line, '\n')) {
    if (line == "End") 
      return target; // No luck, reached the end of the text part of the cub

    // If the input file is a .tif rather than a .cub, it could have several
    // GB and not have this info anyway, so exist early.
    count++;
    if (count > 1000)
      break;
    
    // Find the line having TargetName
    boost::to_lower(line);
    if (line.find("targetname") == std::string::npos) 
      continue;

    // Replace the equal sign with a space and read the second
    // non-space entry from this line
    boost::replace_all(line, "=", " ");
    std::istringstream iss(line);
    std::string val;
    if (! (iss >> val >> target)) 
      continue;

    // Wipe any commas, quotes, or spaces (this is for CSM files)
    boost::replace_all(target, ",", "");
    boost::replace_all(target, "\"", "");
    boost::replace_all(target, " ", "");
    
    boost::to_upper(target);
    
    // If empty, return UNKNOWN
    if (target == "")
       target = "UNKNOWN";
    
    // Found a target, no need to go on   
    return target;
  }
  
  return target;
}

namespace boost {
namespace program_options {

  // Custom value semantics, these explain how many tokens should be ingested.
  
  // 2 values
  typed_2_value<vw::Vector2i>*
  value(vw::Vector2i* v) {
    typed_2_value<vw::Vector2i>* r =
      new typed_2_value<vw::Vector2i>(v);
    return r;
  }
  typed_2_value<vw::Vector2>*
  value(vw::Vector2* v) {
    typed_2_value<vw::Vector2>* r =
      new typed_2_value<vw::Vector2>(v);
    return r;
  }

  // 3 values
  typed_3_value<vw::Vector3i>*
  value(vw::Vector3i* v) {
    typed_3_value<vw::Vector3i>* r =
      new typed_3_value<vw::Vector3i>(v);
    return r;
  }
  typed_3_value<vw::Vector3>*
  value(vw::Vector3* v) {
    typed_3_value<vw::Vector3>* r =
      new typed_3_value<vw::Vector3>(v);
    return r;
  }

  // 4 values
  typed_4_value<vw::BBox2i>*
  value(vw::BBox2i* v) {
    typed_4_value<vw::BBox2i>* r =
      new typed_4_value<vw::BBox2i>(v);
    return r;
  }
  typed_4_value<vw::BBox2>*
  value(vw::BBox2* v) {
    typed_4_value<vw::BBox2>* r =
      new typed_4_value<vw::BBox2>(v);
    return r;
  }

  // 6 values
  typed_6_value<vw::BBox3>*
  value(vw::BBox3* v) {
    typed_6_value<vw::BBox3>* r =
      new typed_6_value<vw::BBox3>(v);
    return r;
  }

  // Custom validators which describe how text is turned into values

  // Validator for Vector2i
  template <>
  void validate(boost::any& v,
                 const std::vector<std::string>& values,
                 vw::Vector2i*, long) {
    validators::check_first_occurrence(v);

    // Concatenate and then split again, so that the user can mix
    // comma and space delimited values.
    std::string joined = boost::algorithm::join(values, " ");
    std::vector<std::string> cvalues;
    boost::split(cvalues, joined, is_any_of(", "), boost::token_compress_on);

    if (cvalues.size() != 2)
      boost::throw_exception(invalid_syntax(invalid_syntax::missing_parameter));

    try {
      Vector2i output(boost::lexical_cast<int32>(cvalues[0]),
                       boost::lexical_cast<int32>(cvalues[1]));
      v = output;
    } catch (boost::bad_lexical_cast const& e) {
      boost::throw_exception(validation_error(validation_error::invalid_option_value));
    }
  }

  // Validator for Vector2
  template <>
  void validate(boost::any& v,
                 const std::vector<std::string>& values,
                 vw::Vector2*, long) {
    validators::check_first_occurrence(v);

    // Concatenate and then split again, so that the user can mix
    // comma and space delimited values.
    std::string joined = boost::algorithm::join(values, " ");
    std::vector<std::string> cvalues;
    boost::split(cvalues, joined, is_any_of(", "), boost::token_compress_on);

    if (cvalues.size() != 2)
      boost::throw_exception(invalid_syntax(invalid_syntax::missing_parameter));

    try {
      Vector2 output(boost::lexical_cast<double>(cvalues[0]),
                      boost::lexical_cast<double>(cvalues[1]));
      v = output;
    } catch (boost::bad_lexical_cast const& e) {
      boost::throw_exception(validation_error(validation_error::invalid_option_value));
    }
  }

  // Validator for Vector3i
  template <>
  void validate(boost::any& v,
                 const std::vector<std::string>& values,
                 vw::Vector3i*, long) {
    validators::check_first_occurrence(v);

    // Concatenate and then split again, so that the user can mix
    // comma and space delimited values.
    std::string joined = boost::algorithm::join(values, " ");
    std::vector<std::string> cvalues;
    boost::split(cvalues, joined, is_any_of(", "), boost::token_compress_on);

    if (cvalues.size() != 3)
      boost::throw_exception(invalid_syntax(invalid_syntax::missing_parameter));

    try {
      Vector3i output(boost::lexical_cast<int32>(cvalues[0]),
                      boost::lexical_cast<int32>(cvalues[1]),
                      boost::lexical_cast<int32>(cvalues[2]));
      v = output;
    } catch (boost::bad_lexical_cast const& e) {
      boost::throw_exception(validation_error(validation_error::invalid_option_value));
    }
  }

  // Validator for Vector3
  template <>
  void validate(boost::any& v,
                const std::vector<std::string>& values,
                vw::Vector3*, long) {
    validators::check_first_occurrence(v);

    // Concatenate and then split again, so that the user can mix
    // comma and space delimited values.
    std::string joined = boost::algorithm::join(values, " ");
    std::vector<std::string> cvalues;
    boost::split(cvalues, joined, is_any_of(", "), boost::token_compress_on);

    if (cvalues.size() != 3)
      boost::throw_exception(invalid_syntax(invalid_syntax::missing_parameter));

    try {
      Vector3 output(boost::lexical_cast<double>(cvalues[0]),
                     boost::lexical_cast<double>(cvalues[1]),
                     boost::lexical_cast<double>(cvalues[2]));

      v = output;
    } catch (boost::bad_lexical_cast const& e) {
      boost::throw_exception(validation_error(validation_error::invalid_option_value));
    }
  }

  // Validator for BBox2i
  template <>
  void validate(boost::any& v,
                 const std::vector<std::string>& values,
                 vw::BBox2i*, long) {
    validators::check_first_occurrence(v);

    // Concatenate and then split again, so that the user can mix
    // comma and space delimited values.
    std::string joined = boost::algorithm::join(values, " ");
    std::vector<std::string> cvalues;
    boost::split(cvalues, joined, is_any_of(", "), boost::token_compress_on);

    if (cvalues.size() != 4)
      boost::throw_exception(invalid_syntax(invalid_syntax::missing_parameter));

    try {
      BBox2i output(Vector2i(boost::lexical_cast<int32>(cvalues[0]),
                              boost::lexical_cast<int32>(cvalues[1])),
                    Vector2i(boost::lexical_cast<int32>(cvalues[2]),
                              boost::lexical_cast<int32>(cvalues[3])));
      v = output;
    } catch (boost::bad_lexical_cast const& e) {
      boost::throw_exception(validation_error(validation_error::invalid_option_value));
    }
  }

  // Validator for BBox2
  template <>
  void validate(boost::any& v,
                 const std::vector<std::string>& values,
                 vw::BBox2*, long) {
    validators::check_first_occurrence(v);

    // Concatenate and then split again, so that the user can mix
    // comma and space delimited values.
    std::string joined = boost::algorithm::join(values, " ");
    std::vector<std::string> cvalues;
    boost::split(cvalues, joined, is_any_of(", "), boost::token_compress_on);

    if (cvalues.size() != 4)
      boost::throw_exception(invalid_syntax(invalid_syntax::missing_parameter));

    try {
      BBox2 output(Vector2(boost::lexical_cast<double>(cvalues[0]),
                            boost::lexical_cast<double>(cvalues[1])),
                   Vector2(boost::lexical_cast<double>(cvalues[2]),
                            boost::lexical_cast<double>(cvalues[3])));
      v = output;
    } catch (boost::bad_lexical_cast const& e) {
      boost::throw_exception(validation_error(validation_error::invalid_option_value));
    }
  }

  // Validator for BBox3
  template <>
  void validate(boost::any& v,
                 const std::vector<std::string>& values,
                 vw::BBox3*, long) {
    validators::check_first_occurrence(v);

    // Concatenate and then split again, so that the user can mix
    // comma and space delimited values.
    std::string joined = boost::algorithm::join(values, " ");
    std::vector<std::string> cvalues;
    boost::split(cvalues, joined, is_any_of(", "), boost::token_compress_on);

    if (cvalues.size() != 6)
      boost::throw_exception(invalid_syntax(invalid_syntax::missing_parameter));

    try {
      BBox3 output(Vector3(boost::lexical_cast<double>(cvalues[0]),
                            boost::lexical_cast<double>(cvalues[1]),
                            boost::lexical_cast<double>(cvalues[2])),
                   Vector3(boost::lexical_cast<double>(cvalues[3]),
                            boost::lexical_cast<double>(cvalues[4]),
                            boost::lexical_cast<double>(cvalues[5])));
      v = output;
    } catch (boost::bad_lexical_cast const& e) {
      boost::throw_exception(validation_error(validation_error::invalid_option_value));
    }
  }

}}
