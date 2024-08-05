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
#include <set>
#include <vw/Math/Transform.h> // defines vw::TransformPtr

namespace vw {
  namespace ip {
    class InterestPoint;
  }
  namespace ba {
    class ControlNetwork;
  }

  namespace camera {
    class CameraModel;
  }
}

namespace asp {

// Find the match file taking into account --match-files-prefix and
// --clean-match-files-prefix.
std::string stereo_match_filename(std::string const& left_cropped_file,
                                  std::string const& right_cropped_file,
                                  std::string const& out_prefix);

// Compute ip between L.tif and R.tif produced by stereo.
void compute_ip_LR(std::string const & out_prefix);

// Outlier removal based on the disparity of interest points.
// Points with x or y disparity not within the 100-'pct' to 'pct'
// percentile interval expanded by 'factor' will be removed as
// outliers. Overwrite the ip in place.
void filter_ip_by_disparity(double pct,    // for example, 90.0
                            double factor, // for example, 3.0
                            bool quiet,    // if not to print a lot of text
                            std::vector<vw::ip::InterestPoint> & left_ip,
                            std::vector<vw::ip::InterestPoint> & right_ip);
  
// Estimate the "spread" of IP coverage in an image.
// - Returns a value between 0 and 1.
// - Breaks the image into tiles and checks how many tiles have at least N IP.
double calc_ip_coverage_fraction(std::vector<vw::ip::InterestPoint> const& ip,
                                 vw::Vector2i const& image_size, int tile_size=1024,
                                 int min_ip_per_tile=2);
  
/// Apply alignment transform to ip. Not to be used with mapprojected images.
void align_ip(vw::TransformPtr const& tx_left,
              vw::TransformPtr const& tx_right,
              std::vector<vw::ip::InterestPoint> & ip_left,
              std::vector<vw::ip::InterestPoint> & ip_right);

// Undo the alignment of interest points. If tx_left and tx_right are null,
// that will mean there is no alignment to undo.
void unalign_ip(vw::TransformPtr tx_left,
               vw::TransformPtr  tx_right,
               std::vector<vw::ip::InterestPoint> const& ip1_in,
               std::vector<vw::ip::InterestPoint> const& ip2_in,
               std::vector<vw::ip::InterestPoint> & ip1_out,
               std::vector<vw::ip::InterestPoint> & ip2_out);

// Heuristics for match file prefix
std::string match_file_prefix(std::string const& clean_match_files_prefix,
                              std::string const& match_files_prefix,
                              std::string const& out_prefix);
  
// Heuristics for where to load the match file from  
std::string match_filename(std::string const& clean_match_files_prefix,
                           std::string const& match_files_prefix,
                           std::string const& out_prefix,
                           std::string const& image1_path,
                           std::string const& image2_path);

/// The unwarped disparity file name.
std::string unwarped_disp_file(std::string const& prefix, std::string const& left_image,
                               std::string const& right_image);

// Find and sort the convergence angles for given cameras and interest points
void convergence_angles(vw::camera::CameraModel const * left_cam,
                        vw::camera::CameraModel const * right_cam,
                        std::vector<vw::ip::InterestPoint> const& left_ip,
                        std::vector<vw::ip::InterestPoint> const& right_ip,
                        std::vector<double> & sorted_angles);

// Find all match files stored on disk having this prefix
void listExistingMatchFiles(std::string const& prefix,
                            std::set<std::string> & existing_files);

// Given a pair of indicies, return all the matches between them.
void matchesForPair(vw::ba::ControlNetwork const& cnet,
                    int left_cid, int right_cid,
                    std::vector<vw::ip::InterestPoint> & left_ip,
                    std::vector<vw::ip::InterestPoint> & right_ip);
  
} // End namespace asp

#endif//__ASP_CORE_IP_MATCHING_ALGS_H__
