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

/// \file DataLoader.h
///

// Keep here some logic for handling data which depends on Eigen
// to speed up compilation of bundle_adjust.

#ifndef __ASP_CORE_DATA_LOADER_H__
#define __ASP_CORE_DATA_LOADER_H__

#include <vector>
#include <string>
#include <vw/Math/Vector.h>

namespace vw {
  namespace cartography {
    class GeoReference;
  }
}

namespace asp {
void load_csv_or_dem(std::string const& csv_format_str, std::string const& csv_srs,
                     std::string const& reference_terrain,
                     int max_num_reference_points,
                     vw::cartography::GeoReference & geo, // may change
                     // Outputs
                     std::vector<vw::Vector3> & ref_data);
}

#endif // __ASP_CORE_DATA_LOADER_H__
