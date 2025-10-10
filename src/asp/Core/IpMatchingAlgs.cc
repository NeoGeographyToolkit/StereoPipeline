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

#include <asp/Core/InterestPointMatching.h>
#include <asp/Core/IpMatchingAlgs.h>
#include <asp/Core/StereoSettings.h>
#include <asp/Core/FileUtils.h>

#include <vw/InterestPoint/InterestData.h>
#include <vw/InterestPoint/MatcherIO.h>
#include <vw/Camera/CameraModel.h>
#include <vw/BundleAdjustment/ControlNetwork.h>
#include <vw/FileIO/DiskImageView.h>

#include <boost/filesystem.hpp>

using namespace vw;
namespace fs = boost::filesystem;

namespace asp {

// Find the match file taking into account --match-files-prefix and
// --clean-match-files-prefix.
std::string stereo_match_filename(std::string const& left_cropped_file,
                                  std::string const& right_cropped_file,
                                  std::string const& out_prefix) {
  
  // Define the file name containing IP match information.
  bool crop_left  = (stereo_settings().left_image_crop_win  != BBox2i(0, 0, 0, 0));
  bool crop_right = (stereo_settings().right_image_crop_win != BBox2i(0, 0, 0, 0));
  
  // See if can use an externally provided match file
  std::string match_filename;
  if (!crop_left && !crop_right)
    match_filename 
      = asp::match_filename(stereo_settings().clean_match_files_prefix,
                            stereo_settings().match_files_prefix,  
                            out_prefix, left_cropped_file, right_cropped_file);
  
  // If the user wants to use an external match file, it better exist
  bool external_matches = (!stereo_settings().clean_match_files_prefix.empty() ||
                           !stereo_settings().match_files_prefix.empty());
  if (external_matches && !boost::filesystem::exists(match_filename)) 
    vw_throw(ArgumentErr() << "Missing IP file: " << match_filename);
  
  // Fall back to creating one if no luck
  if (match_filename == "" || !boost::filesystem::exists(match_filename))
      match_filename = vw::ip::match_filename(out_prefix, left_cropped_file, right_cropped_file);
  
  return match_filename;
}

// Compute ip between L.tif and R.tif produced by stereo.
void compute_ip_LR(std::string const & out_prefix) {
  
  const std::string left_aligned_image_file  = out_prefix + "-L.tif";
  const std::string right_aligned_image_file = out_prefix + "-R.tif";
  
  // The L-R match file. TODO(oalexan1): May need to consider the
  // --match-files-prefix and --clean-match-files-prefix options. 
  // For that, need to integrate with the logic for finding the match file
  // for non-mapprojected images. So this function should be called from
  // StereoSession::preprocessing_hook() where we know left_cropped_file, etc.
  // Must then also test with ISIS mapprojected images.
  std::string match_filename = vw::ip::match_filename(out_prefix, "L.tif", "R.tif");

  // Make sure the match file is newer than these files
  std::vector<std::string> ref_list;
  ref_list.push_back(left_aligned_image_file);
  ref_list.push_back(right_aligned_image_file);

  bool crop_left  = (asp::stereo_settings().left_image_crop_win  != BBox2i(0, 0, 0, 0));
  bool crop_right = (asp::stereo_settings().right_image_crop_win != BBox2i(0, 0, 0, 0));
  bool rebuild = (!first_is_newer(match_filename, ref_list) || crop_left || crop_right);
  if (!crop_left && !crop_right &&
      (asp::stereo_settings().force_reuse_match_files ||
       asp::stereo_settings().clean_match_files_prefix != "" ||
       asp::stereo_settings().match_files_prefix != ""))
    rebuild = false; // Do not rebuild with externally provided match files
    
  if (fs::exists(match_filename) && !rebuild) {
    vw_out() << "Cached IP match file found: " << match_filename << std::endl;
    return;
  }

  vw_out() << "Computing interest points matches.\n";
  
  // Load the images
  boost::shared_ptr<DiskImageResource> 
    left_rsrc(DiskImageResourcePtr(left_aligned_image_file)),
    right_rsrc(DiskImageResourcePtr(right_aligned_image_file));

  std::string left_ip_filename  = ip::ip_filename(out_prefix, left_aligned_image_file);
  std::string right_ip_filename = ip::ip_filename(out_prefix, right_aligned_image_file);

  // Read the no-data values written to disk previously when
  // the normalized left and right sub-images were created.
  float left_nodata_value  = std::numeric_limits<float>::quiet_NaN();
  float right_nodata_value = std::numeric_limits<float>::quiet_NaN();
  if (left_rsrc->has_nodata_read ()) left_nodata_value  = left_rsrc->nodata_read ();
  if (right_rsrc->has_nodata_read()) right_nodata_value = right_rsrc->nodata_read();
  
  // These images can be big, so use ImageViewRef
  ImageViewRef<float> left_image  = DiskImageView<float>(left_rsrc);
  ImageViewRef<float> right_image = DiskImageView<float>(right_rsrc);

  // Mask the nodata
  ImageViewRef<PixelMask<float>> left_masked_image
    = create_mask(left_image, left_nodata_value);
  ImageViewRef<PixelMask<float>> right_masked_image
    = create_mask(right_image, right_nodata_value);
  
  // Read the masks
  std::string left_mask_file  = out_prefix + "-lMask.tif";
  std::string right_mask_file = out_prefix + "-rMask.tif";
  ImageViewRef<PixelMask<uint8>> left_mask 
    = create_mask(DiskImageView<uint8>(left_mask_file), 0);
  ImageViewRef<PixelMask<uint8>> right_mask
    = create_mask(DiskImageView<uint8>(right_mask_file), 0);
   
  // The logic further down cannot handle NaN, so fix that
  if (std::isnan(left_nodata_value)) 
    left_nodata_value = -std::numeric_limits<float>::max();
  if (std::isnan(right_nodata_value))
    right_nodata_value = -std::numeric_limits<float>::max();
    
  // It is important to apply the masks so that not to find interest points
  // in areas where the images are invalid.
  left_masked_image = intersect_mask(left_masked_image, left_mask);
  right_masked_image = intersect_mask(right_masked_image, right_mask);
  left_image = apply_mask(left_masked_image, left_nodata_value);
  right_image = apply_mask(right_masked_image, right_nodata_value);
  
  // No interest point operations have been performed before
  vw_out() << "\t    * Detecting interest points.\n";

  // TODO: Depending on alignment method, we can tailor the IP filtering strategy.
  double thresh_factor = asp::stereo_settings().ip_inlier_factor; // 1.0/15 by default
  
  // This range is extra large to handle elevation differences. That can 
  // be an issue with mapprojected images.
  // TODO(oalexan1): Must use everywhere a single choice of parameters
  // for ip matching with homography.
  const int inlier_threshold = 1000.0 * thresh_factor; // 200 by default
  size_t number_of_jobs = 1;
  bool use_cached_ip = false;
  bool success = asp::homography_ip_matching(left_image, right_image,
                                             asp::stereo_settings().ip_per_tile,
                                             inlier_threshold, match_filename,
                                             number_of_jobs,
                                             left_ip_filename, right_ip_filename,
                                             use_cached_ip,
                                             left_nodata_value, right_nodata_value);

  if (!success)
    vw_throw(ArgumentErr() << "Could not find interest points.\n");

  return;
}

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

// Undo the alignment of interest points. If tx_left and tx_right are null,
// that will mean there is no alignment to undo.
void unalign_ip(vw::TransformPtr tx_left,
               vw::TransformPtr  tx_right,
               std::vector<vw::ip::InterestPoint> const& ip1_in,
               std::vector<vw::ip::InterestPoint> const& ip2_in,
               std::vector<vw::ip::InterestPoint> & ip1_out,
               std::vector<vw::ip::InterestPoint> & ip2_out) {

  // Init the output vectors
  ip1_out.clear();
  ip2_out.clear();
  int num_ip = ip1_in.size();
  ip1_out.reserve(num_ip);
  ip2_out.reserve(num_ip);

  if (int(tx_left.get() != NULL) + int(tx_right != NULL) == 1)
    vw_throw(ArgumentErr() << "Either both or none of the transforms must be set.\n");

  // This function can be called with both unaligned and aligned interest points
  bool aligned_ip = (tx_left.get() != NULL && tx_right != NULL);

  for (size_t i = 0; i < num_ip; i++) {

    // We must not both apply a transform and a scale at the same time
    // as these are meant to do the same thing in different circumstances.
    Vector2 p1 = Vector2(ip1_in[i].x, ip1_in[i].y);
    Vector2 p2 = Vector2(ip2_in[i].x, ip2_in[i].y);
    
    if (aligned_ip) {
      // Unalign
      p1 = tx_left->reverse (p1);
      p2 = tx_right->reverse(p2);
    }
    
    // First push the original ip
    ip1_out.push_back(ip1_in[i]);
    ip2_out.push_back(ip2_in[i]);
    
    // Then adjust x and y
    ip1_out.back().x = p1[0];
    ip1_out.back().y = p1[1];
    ip2_out.back().x = p2[0];
    ip2_out.back().y = p2[1];
    
    // Same for ix and iy, for consistency
    ip1_out.back().ix = p1[0];
    ip1_out.back().iy = p1[1];
    ip2_out.back().ix = p2[0];
    ip2_out.back().iy = p2[1];
  }
  
  // ip_in and ip_out must have same size.
  if (ip1_in.size() != ip1_out.size())
    vw_throw(ArgumentErr() << "Aligned and unaligned interest points have different sizes.\n");
    
  return;
}

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

  if (clean_match_files_prefix != "")
    return vw::ip::clean_match_filename(curr_prefix, image1_path, image2_path);

  return vw::ip::match_filename(curr_prefix, image1_path, image2_path);
}

/// The unwarped disparity file name
std::string unwarped_disp_file(std::string const& prefix, std::string const& left_image,
                               std::string const& right_image){
    
  std::string match_file = vw::ip::match_filename(prefix, left_image, right_image);

  std::string disp_file = boost::filesystem::path(match_file).replace_extension("").string();
  return disp_file + "-unaligned-D.tif";
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

// Find all match files stored on disk having this prefix. This is much faster
// than trying to see if any combination of images results in a match file.
void listExistingMatchFiles(std::string const& prefix,
                            std::set<std::string> & existing_files) {

  existing_files.clear();
  fs::path dirName = fs::path(".");
  try {
    dirName = fs::path(prefix).parent_path();
  } catch(...) {}
  
  // This is a fix for an output prefix which is of the form "run" rather than
  // "run/run".
  bool add_dot = false;
  if (dirName.string() == "") {
    dirName = fs::path(".");
    add_dot = true;
  }
  
  // Iterate over all files in the directory
  for (auto i = fs::directory_iterator(dirName); i != fs::directory_iterator(); i++) {
    
    if (fs::is_directory(i->path())) // skip dirs
      continue;
    std::string filename = i->path().string();
    if (filename.find(".match") == std::string::npos) // keep only match files
      continue;
      
    if (add_dot && filename.size() >= 2 && filename[0] == '.' && filename[1] == '/' &&
        filename.find(prefix) != 0) {
      // Had to temporarily replace prefix*.match with ./prefix*.match so
      // boost can list the current directory. Remove the dot (and slash) now.
      filename = filename.substr(2, filename.size()-2);
    }
      
    existing_files.insert(filename);
  }
}

// Given a pair of indices, return all the matches between them.
void matchesForPair(vw::ba::ControlNetwork const& cnet,
                    int left_cid, int right_cid,
                    std::vector<vw::ip::InterestPoint> & left_ip,
                    std::vector<vw::ip::InterestPoint> & right_ip) {

  // Wipe the outputs
  left_ip.clear();
  right_ip.clear();
  
  // Iterate over all control points in cnet
  for (int ipt = 0; ipt < cnet.size(); ipt++) {
    
    // Iterate over all measures for the current control point 
    bool has_left = false, has_right = false;
    vw::ip::InterestPoint lip, rip;
    for (auto m = cnet[ipt].begin(); m != cnet[ipt].end(); m++) {
      int cid = m->image_id();
      
      if (cid == left_cid) {
        has_left = true;
        lip.x = m->position()[0];
        lip.y = m->position()[1];
      } else if (cid == right_cid) {
        has_right = true;
        rip.x = m->position()[0];
        rip.y = m->position()[1];
      }
    }
    
    if (has_left && has_right) {
      left_ip.push_back(lip);
      right_ip.push_back(rip);
    }
  }
  
  return;
}
 
} // end namespace asp
