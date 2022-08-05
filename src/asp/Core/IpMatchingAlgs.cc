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

#include <asp/Core/IpMatchingAlgs.h>         // Lightweight header
#include <vw/InterestPoint/InterestData.h>
#include <vw/InterestPoint/Matcher.h>
#include <boost/filesystem.hpp>
using namespace vw;

namespace asp {

/// Apply alignment transform to ip. Not to be used with mapprojected images.
void align_ip(vw::TransformPtr const& tx_left,
              vw::TransformPtr const& tx_right,
              std::vector<vw::ip::InterestPoint> & ip_left,
              std::vector<vw::ip::InterestPoint> & ip_right) {

  // Loop through all the IP we found
  for (size_t i = 0; i < ip_left.size(); i++) {
    // Apply the alignment transforms to the recorded IP
    Vector2 l = tx_left->forward (Vector2(ip_left [i].x,  ip_left [i].y));
    Vector2 r = tx_right->forward(Vector2(ip_right[i].x,  ip_right[i].y));

    ip_left [i].x = l[0];
    ip_left [i].y = l[1];
    ip_left [i].ix = l[0];
    ip_left [i].iy = l[1];
    
    ip_right[i].x = r[0];
    ip_right[i].y = r[1];
    ip_right[i].ix = r[0];
    ip_right[i].iy = r[1];
  }

  return;
} // End align_ip

// Heuristics for where to load the match file from  
std::string match_filename(std::string const& clean_match_files_prefix,
                           std::string const& match_files_prefix,
                           std::string const& out_prefix,
                           std::string const& image1_path,
                           std::string const& image2_path,
                           bool allow_missing_match_file) {
  
  if (clean_match_files_prefix != "") {

    std::string match_file = vw::ip::clean_match_filename(clean_match_files_prefix, image1_path,
                                                          image2_path);
    if (!boost::filesystem::exists(match_file) && !allow_missing_match_file) 
      vw_throw(ArgumentErr() << "Missing IP file: " << match_file);
    return match_file;
    
  } else if (match_files_prefix != "") {
    
    std::string match_file = vw::ip::match_filename(match_files_prefix, image1_path,
                                                    image2_path);
    if (!boost::filesystem::exists(match_file) && !allow_missing_match_file) 
      vw_throw(ArgumentErr() << "Missing IP file: " << match_file);
    return match_file;
    
  }
  
  return vw::ip::match_filename(out_prefix, image1_path, image2_path);
}
  
} // end namespace asp
