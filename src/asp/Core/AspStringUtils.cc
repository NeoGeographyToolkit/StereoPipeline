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

/// Low-level string utilities.

// TODO(oalexan1): Move more logic here from Common.cc which is too big.

#include <asp/Core/AspStringUtils.h>
#include <vw/Core/Exception.h>

#include <boost/algorithm/string.hpp>

#include <iostream>
#include <fstream>

namespace asp {

// Given the string dgmaprpc, extract the parts 'dg' and 'rpc'.
void parseCamTypes(std::string const& session_name,
                   // Outputs
                   std::string & tri_cam_type, 
                   std::string & mapproj_cam_type) {

  std::string sep = "map";
  std::size_t it = session_name.find(sep);
  if (it == std::string::npos) 
      vw::vw_throw(vw::ArgumentErr() << "Could not find the string \"" << sep << "\" in the session name: \"" << session_name << "\".\n");

  tri_cam_type = session_name.substr(0, it);

  it += sep.size();
  mapproj_cam_type = session_name.substr(it, session_name.size());
} 

// Given a file having keys like "run:", followed by values, extract
// the keys and values in a map from keys to vectors of strings.
void parseKeysVals(std::string const& file, 
                   std::map<std::string, std::vector<std::string>> & keys_vals) {

  std::ifstream ifs(file.c_str());
  if (!ifs.good()) 
    vw::vw_throw(vw::ArgumentErr() << "Could not open file: " << file << "\n");

  // Wipe the output map
  keys_vals.clear();
  
  // Read the file line by line
  std::string line;
  while (getline(ifs, line)) {
    boost::trim(line);
    if (line.empty()) continue;
    if (line[0] == '#') continue; // Skip comments

    // Split the line into key and value
    std::vector<std::string> parts;
    boost::split(parts, line, boost::is_any_of(" \t"), boost::token_compress_on);

    if (parts.empty())
      continue;

    // Add the key and value to the map
    std::string key = parts[0];
    parts.erase(parts.begin());
    keys_vals[key] = parts;
  }
  
}

} // end namespace asp
