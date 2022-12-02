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
#include <vw/Camera/CameraModel.h>
#include <boost/filesystem.hpp>
using namespace vw;
namespace fs = boost::filesystem;

namespace asp {

// Outlier removal based on the disparity of interest points.
// Points with x or y disparity not within the 100-'pct' to 'pct'
// percentile interval expanded by 'factor' will be removed as
// outliers. Overwrite the ip in place.
void filter_ip_by_disparity(double pct,    // for example, 90.0
                            double factor, // for example, 3.0
                            bool quiet,    // if not to print a lot of text
                            std::vector<vw::ip::InterestPoint> & left_ip,
                            std::vector<vw::ip::InterestPoint> & right_ip){

  double pct_fraction = 1.0 - pct/100.0;
  double bx, ex, by, ey;
  std::vector<double> dispx, dispy;
  for (size_t it = 0; it < left_ip.size(); it++) {
    dispx.push_back(right_ip[it].x - left_ip[it].x);
    dispy.push_back(right_ip[it].y - left_ip[it].y);
  }
  vw::math::find_outlier_brackets(dispx, pct_fraction, factor, bx, ex);
  vw::math::find_outlier_brackets(dispy, pct_fraction, factor, by, ey);
    
  //vw_out() << "Outlier statistics by disparity in x: b = " << bx << ", e = " << ex << ".\n";
  //vw_out() << "Outlier statistics by disparity in y: b = " << by << ", e = " << ey << ".\n";
    
  // Remove the bad ip 
  size_t good_it = 0;
  for (size_t it = 0; it < left_ip.size(); it++) {
    if (dispx[it] < bx || dispx[it] > ex) continue;
    if (dispy[it] < by || dispy[it] > ey) continue;
    left_ip [good_it] = left_ip[it];
    right_ip[good_it] = right_ip[it];
    good_it++;
  }

  if (!quiet)
    vw_out() << "Removed " << left_ip.size() - good_it
             << " outliers based on percentiles of differences of interest "
             << "points with --outlier-removal-params.\n";
  
  left_ip.resize(good_it);
  right_ip.resize(good_it);

  return;
}


double calc_ip_coverage_fraction(std::vector<ip::InterestPoint> const& ip,
                                 vw::Vector2i const& image_size, int tile_size,
                                 int min_ip_per_tile) {

  if (tile_size < 1)
    vw_throw(LogicErr() << "calc_ip_coverage_fraction: tile size is " << tile_size);

  // Generate a grid of ROIs covering the entire image
  BBox2i full_bbox(Vector2i(0,0), image_size);
  bool include_partials = false;
  std::vector<BBox2i> rois;
  rois = subdivide_bbox(full_bbox, tile_size, tile_size, include_partials);
  const size_t num_rois = rois.size();
  if (num_rois == 0)
    return 0; // Cannot have any coverage in the degenerate case!
    
  // Pack all IP into a list for speed
  std::list<Vector2i> ip_list;
  for (size_t i=0; i<ip.size(); ++i) {
    ip_list.push_back(Vector2i(ip[i].x, ip[i].y));
  }
    
  size_t num_filled_rois = 0;
  for (size_t i=0; i<num_rois; ++i) { // Loop through ROIs
    int ip_in_roi = 0;
      
    // Check if each point is in this ROI
    std::list<Vector2i>::iterator iter;
    for (iter=ip_list.begin(); iter!=ip_list.end(); ++iter) {
        
      // If the IP is in the ROI, remove it from the IP list so it
      // does not get searched again.
      if (rois[i].contains(*iter)) {
        iter = ip_list.erase(iter);
        ++ip_in_roi;
        --iter;
      }
    } // End IP loop
    if (ip_in_roi > min_ip_per_tile)
      ++num_filled_rois;
  }// End ROI loop

  return static_cast<double>(num_filled_rois) / static_cast<double>(num_rois);
}
  
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

// Heuristics for match file prefix
std::string match_file_prefix(std::string const& clean_match_files_prefix,
                              std::string const& match_files_prefix,
                              std::string const& out_prefix) {
  
  if (clean_match_files_prefix != "")
    return clean_match_files_prefix;
  else if (match_files_prefix != "")
    return match_files_prefix;
  return out_prefix; 
}
  
// Heuristics for where to load the match file from  
std::string match_filename(std::string const& clean_match_files_prefix,
                           std::string const& match_files_prefix,
                           std::string const& out_prefix,
                           std::string const& image1_path,
                           std::string const& image2_path) {

  std::string curr_prefix = asp::match_file_prefix(clean_match_files_prefix,
                                              match_files_prefix,  
                                              out_prefix);
  
  return vw::ip::match_filename(curr_prefix, image1_path, image2_path);
}

// Find and sort the convergence angles for given cameras and interest points
void convergence_angles(vw::camera::CameraModel const * left_cam,
                        vw::camera::CameraModel const * right_cam,
                        std::vector<vw::ip::InterestPoint> const& left_ip,
                        std::vector<vw::ip::InterestPoint> const& right_ip,
                        std::vector<double> & sorted_angles) {

  int num_ip = left_ip.size();
  sorted_angles.clear();
  for (int ip_it = 0; ip_it < num_ip; ip_it++) {
    Vector2 lip(left_ip[ip_it].x,  left_ip[ip_it].y);
    Vector2 rip(right_ip[ip_it].x, right_ip[ip_it].y);
    double angle = 0.0;
    try {
      angle = (180.0 / M_PI) * acos(dot_prod(left_cam->pixel_to_vector(lip),
                                             right_cam->pixel_to_vector(rip)));
    } catch(...) {
      // Projection into camera may not always succeed
      continue;
    }
      
    if (std::isnan(angle)) 
      continue;
      
    sorted_angles.push_back(angle);
  }
  
  std::sort(sorted_angles.begin(), sorted_angles.end());
}

// Find all match files stored on disk having this prefix
void listExistingMatchFiles(std::string const& prefix,
                            std::set<std::string> & existing_files) {
  existing_files.clear();
  
  fs::path dirName = fs::path(prefix).parent_path().string();
  for (auto i = fs::directory_iterator(dirName); i != fs::directory_iterator(); i++) {
    if (fs::is_directory(i->path())) // skip dirs
      continue;
    std::string filename = i->path().string();
    if (filename.find(".match") != std::string::npos)
      existing_files.insert(filename);
  }
}
  
} // end namespace asp
