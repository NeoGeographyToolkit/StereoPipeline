// __BEGIN_LICENSE__
//  Copyright (c) 2006-2013, United States Government as represented by the
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


/// \file MatchList.h
///
/// Interest point matching logic.
///
#ifndef __ASP_CORE_GCP_H__
#define __ASP_CORE_GCP_H__

#include <asp/Core/MatchList.h>

namespace asp { 

  // Write a GCP file. Can throw exceptions.
  void writeGCP(std::vector<std::string> const& image_files,
                std::string const& gcp_file,
                std::string const& dem_file,
                asp::MatchList const& matchlist,
                double xyz_sigma);

} // namespace asp

#endif  // __ASP_CORE_GCP_H__
