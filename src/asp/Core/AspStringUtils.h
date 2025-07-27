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

/// \file AspStringUtils.h
/// Low-level string utilities.

#ifndef __ASP_CORE_ASP_STRING_UTILS_H__
#define __ASP_CORE_ASP_STRING_UTILS_H__

#include <string>
#include <map>
#include <vector>

namespace asp {

// Given the string dgmaprpc, extract the parts 'dg' and 'rpc'.
void parseCamTypes(std::string const& session_name,
                   // Outputs
                   std::string & tri_cam_type, 
                   std::string & mapproj_cam_type);

// Given a file having keys like "run:", and values like what follows, extract
// the keys and values in a map from keys to vectors of strings.
void parseKeysVals(std::string const& file, 
                   std::map<std::string, std::vector<std::string>> & keys_vals);

// Parse 'VAR1=VAL1 VAR2=VAL2' into a map. Note that we append to the map,
// so it may have some items there beforehand.
void parse_append_metadata(std::string const& metadata,
                            std::map<std::string, std::string> & keywords);

} // end namespace asp


#endif //__ASP_CORE_ASP_STRING_UTILS_H__
