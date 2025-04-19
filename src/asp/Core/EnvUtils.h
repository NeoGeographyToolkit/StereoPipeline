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

#ifndef __CORE_ENV_UTILS_H__
#define __CORE_ENV_UTILS_H__

#include <string>

namespace asp {

// Set an env var and check for success. Use the setenv() function,
// which is not problematic, as putenv().
void setEnvVar(std::string const& var, std::string const& val);

// Set some env vars needed by all ASP programs. See the .cc for details.
void set_asp_env_vars();

} //end namespace asp

#endif//__CORE_ENV_UTILS_H__

