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

#include <cstdlib>
#include <iostream>
#include <unistd.h>

namespace asp {

// Set an env var and check for success. Use the setenv() function,
// which is not problematic, as putenv().
void setEnvVar(std::string const& var, std::string const& val) {
  
  bool overwrite = true;
   if (setenv(var.c_str(), val.c_str(), overwrite) != 0)
    vw::vw_throw(vw::ArgumentErr() << "Failed to set environment variable: " << var
              << " to value: " << val << ".\n");
}

} // end namespace asp
