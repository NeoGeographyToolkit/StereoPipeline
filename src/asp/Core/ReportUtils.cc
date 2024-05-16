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

/// \file ReportUtils.cc
///

// Utilities for reading report files

#include <asp/Core/ReportUtils.h>
#include <vw/Core/Exception.h>
#include <vw/Core/Log.h>
#include <boost/algorithm/string/replace.hpp>
#include <boost/algorithm/string/case_conv.hpp>

namespace asp {

// See the header for the function description.
void readReportFile(std::string const& report_file,
                    std::set<std::string> const& str_col_names,
                    std::set<std::string> const& num_col_names,
                    // Outputs
                    std::map<std::string, std::vector<std::string>> & str_map,
                    std::map<std::string, std::vector<double>> & num_map) {

  // Open the file for reading, and throw an error if it fails
  vw::vw_out() << "Reading: " << report_file << "\n";
  std::ifstream file(report_file.c_str());
  if (!file.is_open())
    vw::vw_throw(vw::ArgumentErr() << "Failed to open file: " << report_file << "\n");

  // Wipe the outputs
  str_map.clear();
  num_map.clear();
  
  // Read the header
  std::string header;
  std::getline(file, header);
  // Make lowercase
  boost::to_lower(header);
  // Replace all pound signs and commas with spaces
  boost::replace_all(header, "#", " ");
  boost::replace_all(header, ",", " ");
  // Split the header into tokens
  std::istringstream is(header);
  std::vector<std::string> colNames;  
  std::string val;
  while (is >> val)
    colNames.push_back(val);
    
  // Parse each line
  std::string line;
  while (std::getline(file, line)) {

    // Make lowercase
    boost::to_lower(line);
    // Replace all commas with spaces
    boost::replace_all(line, ",", " ");
    // Split the line into tokens
    std::istringstream is(line);
    std::vector<std::string> tokens;
    while (is >> val)
      tokens.push_back(val);
      
    // Skip empty lines
    if (tokens.empty()) 
      continue;
    // Skip lines starting with a pound sign
    if (tokens[0][0] == '#') 
      continue;
      
    // Must have as many tokens as columns
    if (tokens.size() != colNames.size()) 
      vw::vw_throw(vw::ArgumentErr() << "Line has " << tokens.size() << " values. "
                   << "Expected: " << colNames.size() << "\n");
      
    // Iterate over tokens. If the token is a string, add it to the string map.
    // If it is a number, add it to the number map.
    for (size_t i = 0; i < tokens.size(); i++) {
      // check if colNames[i] is in str_col_names 
      if (str_col_names.find(colNames[i]) != str_col_names.end()) {
        
        str_map[colNames[i]].push_back(tokens[i]);
        
      } else if (num_col_names.find(colNames[i]) != num_col_names.end()) {
        
        // Must be a number. Add it to the number map. Do a simple sanity check.
        auto const& t = tokens[i];
        if (!isdigit(t[0]) && t[0] != '-' && t[0] != '.')
          vw::vw_throw(vw::ArgumentErr() << "Expected a number, got: " << t << "\n");
        num_map[colNames[i]].push_back(atof(t.c_str()));
      }
    }
  }  
  
  // Check each desired column is present in the file
  for (std::string const& col: str_col_names) {
    if (str_map.find(col) == str_map.end())
      vw::vw_throw(vw::ArgumentErr() << "Column " << col << " not found in: "
                    << report_file << "\n");
  }
  for (std::string const& col: num_col_names) {
    if (num_map.find(col) == num_map.end())
      vw::vw_throw(vw::ArgumentErr() << "Column " << col << " not found in: "
                    << report_file << "\n");
  }
  
  // All columns must have the same size
  size_t num_rows = str_map.begin()->second.size();
  for (auto const& p: str_map) {
    if (p.second.size() != num_rows)
      vw::vw_throw(vw::ArgumentErr() << "Column " << p.first << " has " << p.second.size()
                    << " rows, expected: " << num_rows << "\n");
  }
  for (auto const& p: num_map) {
    if (p.second.size() != num_rows)
      vw::vw_throw(vw::ArgumentErr() << "Column " << p.first << " has " << p.second.size()
                    << " rows, expected: " << num_rows << "\n");
  }
}
    
} // end namespace asp

