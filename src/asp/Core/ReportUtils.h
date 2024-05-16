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

/// \file ReportUtils.h
///

// Utilities for reading report files

#ifndef __ASP_CORE_REPORT_UTILS_H__
#define __ASP_CORE_REPORT_UTILS_H__

#include <string>
#include <vector>
#include <map>
#include <set>
#include <iostream>
#include <fstream>

namespace asp {

// Given a report file with the first line being a header having column names,
// read it into a map. The keys will be the column names, the values will be
// vectors of strings or doubles. The desired names of columns and their types
// are passed in as arguments.
void readReportFile(std::string const& report_file,
                    std::set<std::string> const& str_col_names,
                    std::set<std::string> const& num_col_names,
                    // Outputs
                    std::map<std::string, std::vector<std::string>> & str_map,
                    std::map<std::string, std::vector<double>> & num_map);
                    
  
} // End namespace asp

#endif
