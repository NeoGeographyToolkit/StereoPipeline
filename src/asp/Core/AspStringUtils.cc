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

/// Parse 'VAR1=VAL1 VAR2=VAL2' into a map. Note that we append to the map,
/// so it may have some items there beforehand. The value is everything after
/// the first equal sign, including any further equal signs and spaces, just as
/// GDAL handles its -mo option. A whitespace-separated token that has an equal
/// sign starts a new key; one without an equal sign extends the value of the
/// current key. So 'VAR1=value with spaces VAR2=plain' yields VAR1="value with
/// spaces" and VAR2="plain". Pass several pairs in one quoted string, or repeat
/// the option.
void parse_append_metadata(std::string const& metadata,
                           std::map<std::string, std::string> & keywords) {

  std::istringstream is(metadata);
  std::string token, var;
  bool have_var = false;
  while (is >> token) {
    size_t equal_pos = token.find('=');

    // A token with no equal sign, or with one only at the start (so it has
    // no variable name), continues the value of the current key.
    if (equal_pos == std::string::npos || equal_pos == 0) {
      if (!have_var)
        vw::vw_throw(vw::ArgumentErr() << "Could not parse metadata: " << metadata << "\n");
      keywords[var] += " " + token;
      continue;
    }

    // This token starts a new key=value pair. The value must be non-empty.
    if (equal_pos + 1 >= token.size())
      vw::vw_throw(vw::ArgumentErr() << "Could not parse: " << token << "\n");

    var = token.substr(0, equal_pos);
    keywords[var] = token.substr(equal_pos + 1);
    have_var = true;
  }
}

} // end namespace asp
