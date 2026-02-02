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

/// \file RigParseOptions.h
///
/// Parse and validate rig_calibrator command-line options.

#ifndef __ASP_RIG_RIG_PARSE_OPTIONS_H__
#define __ASP_RIG_RIG_PARSE_OPTIONS_H__

#include <asp/Rig/RigOptions.h>

namespace rig {
  
struct RigSet;
  
// Process the rig_calibrator options using boost::program_options
void handleRigArgs(int argc, char *argv[], RigOptions& opt);

// Validate rig_calibrator options
void parameterValidation(RigOptions const& opt);

// Parse auxiliary rig options
void parseAuxRigOptions(RigOptions& opt, RigSet const& R);
  
} // namespace rig

#endif // __ASP_RIG_RIG_PARSE_OPTIONS_H__
