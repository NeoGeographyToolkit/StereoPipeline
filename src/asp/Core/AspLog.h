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

/// \file AspLog.h
///
/// Logging utilities for ASP tools (no boost::program_options dependency).

#ifndef __ASP_CORE_ASP_LOG_H__
#define __ASP_CORE_ASP_LOG_H__

#include <string>

namespace asp {

  /// Print time function
  std::string current_posix_time_string();

  /// Run a system command and append the output to a given file
  void run_cmd_app_to_file(std::string cmd, std::string file);

  /// Get program name without path and leading 'lt-'.
  std::string extract_prog_name(std::string const& prog_str);

  /// Write logs to a file
  void log_to_file(int argc, char *argv[],
                   std::string stereo_default_filename,
                   std::string output_prefix);

} // end namespace asp

#endif // __ASP_CORE_ASP_LOG_H__
