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

#include <asp/Core/EnvUtils.h>
#include <vw/Core/Exception.h>

#include <vw/vw_config.h> // must come before asp_config.h, defines VW_BOOST_VERSION
#include <asp/asp_config.h> // defines ASP_HAVE_PKG_ISIS

#include <boost/filesystem.hpp>
#include <boost/dll.hpp>

#include <cstdlib>
#include <iostream>
#include <unistd.h>

namespace fs = boost::filesystem;

namespace asp {

// Set an env var and check for success. Use the setenv() function,
// which is not problematic, as putenv().
void setEnvVar(std::string const& var, std::string const& val) {
  
  bool overwrite = true;
   if (setenv(var.c_str(), val.c_str(), overwrite) != 0)
    vw::vw_throw(vw::ArgumentErr() << "Failed to set environment variable: " << var
              << " to value: " << val << ".\n");
}

// A function to set the environmental variables ISISROOT, QT_PLUGIN_PATH,
// GDAL_DATA, and PROJ_LIB. In packaged build mode, set these with the help of
// the base directory of the distribution. These are needed especially for the
// conda build, when the ASP executables don't have a wrapper around them. For
// the tarball build, some of this logic is duplicated in the script in
// BinaryBuilder/dist-add/libexec/libexec-funcs.sh which is then called by the
// wrapper.
void set_asp_env_vars() {
    
  // Find the path to the base of the package and see if it works.
  std::string base_dir = boost::dll::program_location().parent_path().parent_path().string();

#if defined(ASP_HAVE_PKG_ISIS) && ASP_HAVE_PKG_ISIS == 1
  // Set ISISROOT and check for IsisPreferences
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
  asp::setEnvVar("ISISROOT", base_dir);
  if (!fs::exists(std::string(getenv("ISISROOT")) + "/IsisPreferences")) 
    vw::vw_throw(vw::ArgumentErr() << "Cannot find IsisPreferences in "
                 << getenv("ISISROOT"));
#endif // ASP_HAVE_PKG_ISIS
  
  // Set QT_PLUGIN_PATH as the path to /plugins
  asp::setEnvVar("QT_PLUGIN_PATH", base_dir + "/plugins");
  if (!fs::exists(std::string(getenv("QT_PLUGIN_PATH")))) {
    base_dir = ASP_DEPS_DIR; // This is defined at compile time
    asp::setEnvVar("QT_PLUGIN_PATH", base_dir + "/plugins");
  }
  if (!fs::exists(std::string(getenv("QT_PLUGIN_PATH"))))
    vw::vw_throw(vw::ArgumentErr() << "Cannot find Qt plugins in " 
                 << getenv("QT_PLUGIN_PATH"));

  // Set GDAL_DATA and check for share/gdal
  asp::setEnvVar("GDAL_DATA", base_dir + "/share/gdal");  
  if (!fs::exists(std::string(getenv("GDAL_DATA")))) 
    vw::vw_throw(vw::ArgumentErr() << "Cannot find GDAL data in "
                 << getenv("GDAL_DATA"));

  // Set GDAL_DRIVER_PATH and check for share/gdal. There are two locations, because
  // BinaryBuilder moves the plugins from lib/gdalplugins to lib.
  // TODO(oalexan1): Figure out why this happens.
  asp::setEnvVar("GDAL_DRIVER_PATH", base_dir + "/lib/gdalplugins:" + base_dir + "/lib");  
  
  // Older proj api
  // Set PROJ_LIB and check for share/proj
  asp::setEnvVar("PROJ_LIB", base_dir + "/share/proj");
  if (!fs::exists(std::string(getenv("PROJ_LIB")))) 
    vw::vw_throw(vw::ArgumentErr() << "Cannot find PROJ data in "
                 << getenv("PROJ_LIB"));

  // Newer proj api
  // Set PROJ_DATA and check for share/proj
  asp::setEnvVar("PROJ_DATA", base_dir + "/share/proj");
  if (!fs::exists(std::string(getenv("PROJ_DATA")))) 
    vw::vw_throw(vw::ArgumentErr() << "Cannot find PROJ data in "
                 << getenv("PROJ_DATA"));

  // Force the US English locale as long as ASP is running to avoid
  // ISIS choking on a decimal separator which shows up as a comma for 
  // some reason.
  asp::setEnvVar("LC_ALL", "en_US.UTF-8");  
  asp::setEnvVar("LANG", "en_US.UTF-8");
}

} // end namespace asp
