// __BEGIN_LICENSE__
//  Copyright (c) 2009-2026, United States Government as represented by the
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

#include <asp/Core/AspLog.h>
#include <asp/asp_date_config.h>
#include <asp/asp_config.h>

#include <vw/Core/Log.h>
#include <vw/Core/System.h>
#include <vw/Core/Exception.h>
#include <vw/FileIO/FileUtils.h>

#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/filesystem/operations.hpp>
#include <boost/filesystem/path.hpp>
#include <boost/shared_ptr.hpp>

#include <sstream>
#include <fstream>
#include <unistd.h>

namespace fs = boost::filesystem;

std::string asp::current_posix_time_string() {
  return boost::posix_time::to_simple_string(boost::posix_time::second_clock::local_time());
}

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
  vw::vw_out() << "Writing log: " << log_file << std::endl;
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
  current_log->rule_set() = vw::vw_log().console_log().rule_set();
  current_log->rule_set().add_rule(0, "*.progress");
  vw::vw_log().add(current_log);
}
