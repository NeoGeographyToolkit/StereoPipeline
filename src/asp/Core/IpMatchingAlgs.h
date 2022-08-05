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

// This header file is to be very light-weight and only include
// definitions of high-level interest point matching algorithms, and
// no template-based logic. Those should go in
// InterestPointMatching.h, which will be included in
// IpMatchingAlgs.cc. The goal is to include in most places this
// lightweight header rather than InterestPointMatching.h.

// TODO(oalexan1): Move here more algorithms.

#ifndef __ASP_CORE_IP_MATCHING_ALGS_H__
#define __ASP_CORE_IP_MATCHING_ALGS_H__

#include <vector>
#include <vw/Math/Transform.h> // defines vw::TransformPtr

namespace vw {
  namespace ip {
    class InterestPoint;
  }
}

namespace asp {

/// Apply alignment transform to ip. Not to be used with mapprojected images.
void align_ip(vw::TransformPtr const& tx_left,
              vw::TransformPtr const& tx_right,
              std::vector<vw::ip::InterestPoint> & ip_left,
              std::vector<vw::ip::InterestPoint> & ip_right);

// Heuristics for where to load the match file from  
std::string match_filename(std::string const& clean_match_files_prefix,
                           std::string const& match_files_prefix,
                           std::string const& out_prefix,
                           std::string const& image1_path,
                           std::string const& image2_path,
                           bool allow_missing_match_file = false);
} // End namespace asp

#endif//__ASP_CORE_IP_MATCHING_ALGS_H__
