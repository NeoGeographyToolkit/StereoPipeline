// __BEGIN_LICENSE__
//  Copyright (c) 2009-2025, United States Government as represented by the
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

// \file BundleAdjustSession.cc
// Bundle adjustment functions that need StereoSession

#include <asp/Sessions/BundleAdjustSession.h>
#include <asp/Sessions/StereoSession.h>
#include <asp/Sessions/StereoSessionFactory.h>
#include <asp/Camera/BundleAdjustOptions.h>
#include <asp/Core/ImageNormalization.h>
#include <asp/Core/ImageUtils.h>
#include <asp/Core/IpMatchingAlgs.h>
#include <asp/Core/FileUtils.h>

#include <vw/InterestPoint/MatcherIO.h>
#include <vw/Cartography/GeoTransform.h>

namespace asp {

using namespace vw;

// A wrapper around ip matching. Can also work with NULL cameras.
void ba_match_ip(asp::BaOptions & opt, asp::SessionPtr session,
                 std::string const& image1_path,  std::string const& image2_path,
                 vw::camera::CameraModel* cam1,   vw::camera::CameraModel* cam2,
                 std::string const& match_filename) {

  boost::shared_ptr<DiskImageResource>
    rsrc1(vw::DiskImageResourcePtr(image1_path)),
    rsrc2(vw::DiskImageResourcePtr(image2_path));
  if ((rsrc1->channels() > 1) || (rsrc2->channels() > 1))
    vw_throw(ArgumentErr()
             << "Error: Input images can only have a single channel!\n\n");
  float nodata1, nodata2;
  asp::get_nodata_values(rsrc1, rsrc2, nodata1, nodata2);

  // IP matching may not succeed for all pairs

  // Get masked views of the images to get statistics from. If the user provided
  // a custom no-data value, values no more than that are masked.
  ImageViewRef<float> image1_view = DiskImageView<float>(rsrc1);
  ImageViewRef<float> image2_view = DiskImageView<float>(rsrc2);
  ImageViewRef<PixelMask<float>> masked_image1, masked_image2;
  float user_nodata = asp::stereo_settings().nodata_value;
  if (!std::isnan(user_nodata)) {
    masked_image1 = create_mask_less_or_equal(image1_view, user_nodata);
    masked_image2 = create_mask_less_or_equal(image2_view, user_nodata);
  } else {
    masked_image1 = create_mask(image1_view, nodata1);
    masked_image2 = create_mask(image2_view, nodata2);
  }

  // Since we computed statistics earlier, this will just be loading files.
  vw::Vector<vw::float32,6> image1_stats, image2_stats;
  image1_stats = asp::gather_stats(masked_image1, image1_path,
                                   opt.out_prefix, image1_path,
                                   asp::stereo_settings().force_reuse_match_files);
  image2_stats = asp::gather_stats(masked_image2, image2_path,
                                   opt.out_prefix, image2_path,
                                   asp::stereo_settings().force_reuse_match_files);

  // Rough homography cannot use / cache vwip
  std::string vwip_file1, vwip_file2; 
  if (opt.enable_rough_homography) {
    vw::vw_out(vw::WarningMessage)
      << "Option --enable-rough-homography is set. Will not save interest "
      << "point matches per image before matching (vwip), as these depend "
      << "on the pair of images used.\n";
  } else {
    vwip_file1 = ip::ip_filename(opt.out_prefix, image1_path);
    vwip_file2 = ip::ip_filename(opt.out_prefix, image2_path);
  }
  
  // For mapprojected images and given the overlap params,
  // can restrict the matching to a smaller region.
  vw::BBox2 bbox1, bbox2;
  if (opt.pct_for_overlap >= 0) {
    vw::cartography::GeoReference georef1, georef2;
    bool has_georef1 = vw::cartography::read_georeference(georef1, image1_path);
    bool has_georef2 = vw::cartography::read_georeference(georef2, image2_path);
    if (has_georef1 && has_georef2) {
      bbox1 = vw::bounding_box(masked_image1);
      bbox2 = vw::bounding_box(masked_image2);
      // Expand the boxes by pct
      expand_box_by_pct(bbox1, opt.pct_for_overlap);
      expand_box_by_pct(bbox2, opt.pct_for_overlap);
      // Transform each box to the other image's pixel coordinates
      vw::cartography::GeoTransform trans(georef1, georef2);
      vw::BBox2 trans_bbox1 = trans.forward_bbox(bbox1);
      vw::BBox2 trans_bbox2 = trans.reverse_bbox(bbox2);
      // The first box will be the 2nd transformed box, then cropped
      // to bounding box of first image
      bbox1 = trans_bbox2;
      bbox1.crop(bounding_box(masked_image1));
      // Same logic for the second box.
      bbox2 = trans_bbox1;
      bbox2.crop(bounding_box(masked_image2));
    }
  }

  // The match files (.match) are cached unless the images or camera
  // are newer than them.
  session->ip_matching(image1_path, image2_path,
                       Vector2(masked_image1.cols(), masked_image1.rows()),
                       image1_stats, image2_stats,
                       nodata1, nodata2, cam1, cam2, match_filename,
                       vwip_file1, vwip_file2, bbox1, bbox2);
}

// Create matches among the mapprojected images (or use any such matches created
// beforehand manually by the user), and undo the mapprojection. All matches are
// saved to files.
void matches_from_mapproj_images(int i, int j,
                                 asp::BaOptions& opt, asp::SessionPtr session,
                                 std::vector<std::string> const& map_files,
                                 std::string mapproj_dem,
                                 vw::cartography::GeoReference const& dem_georef,
                                 vw::ImageViewRef<vw::PixelMask<double>> & interp_dem,
                                 std::string const& match_filename) {

  vw::cartography::GeoReference georef1, georef2;
  vw_out() << "Reading georef from " << map_files[i] << ' ' << map_files[j] << std::endl;
  bool is_good1 = vw::cartography::read_georeference(georef1, map_files[i]);
  bool is_good2 = vw::cartography::read_georeference(georef2, map_files[j]);
  if (!is_good1 || !is_good2)
    vw_throw(ArgumentErr() << "Error: Cannot read georeference.\n");

  std::string image1_path  = opt.image_files[i];
  std::string image2_path  = opt.image_files[j];
  if (boost::filesystem::exists(match_filename)) {
    vw_out() << "Using cached match file: " << match_filename << "\n";
    return;
  }

  // TODO(oalexan1):  What if mapprojection was done with a different camera
  // model? Such as RPC vs DG? Also must check the DEM and adjust prefix! Same
  // as when undoing mapprojection during stereo.
  if (opt.skip_matching)
    return;

  // If the match file does not exist, create it. The user can create this manually
  // too.
  std::string map_match_file = ip::match_filename(opt.out_prefix,
                                                  map_files[i], map_files[j]);
  try {
    asp::ba_match_ip(opt, session, map_files[i], map_files[j],
                     NULL, NULL, // no cameras
                     map_match_file);
  } catch (const std::exception& e) {
    vw_out() << "Could not find interest points between images "
             << map_files[i] << " and " << map_files[j] << std::endl;
    vw_out(WarningMessage) << e.what() << std::endl;
    return;
  } //End try/catch

  if (!boost::filesystem::exists(map_match_file)) {
    vw_out() << "Missing: " << map_match_file << "\n";
    return;
  }

  vw_out() << "Reading: " << map_match_file << std::endl;
  std::vector<ip::InterestPoint> ip1,     ip2;
  std::vector<ip::InterestPoint> ip1_cam, ip2_cam;
  ip::read_binary_match_file(map_match_file, ip1, ip2);

  // Undo the map-projection
  vw::CamPtr left_map_proj_cam, right_map_proj_cam;
  session->read_mapproj_cams(map_files[i], map_files[j],
                             opt.camera_files[i], opt.camera_files[j],
                             mapproj_dem, session->name(),
                             left_map_proj_cam, right_map_proj_cam);

  for (size_t ip_iter = 0; ip_iter < ip1.size(); ip_iter++) {
    vw::ip::InterestPoint P1 = ip1[ip_iter];
    vw::ip::InterestPoint P2 = ip2[ip_iter];
    if (!asp::projected_ip_to_raw_ip(P1, interp_dem, left_map_proj_cam,
                                     georef1, dem_georef))
      continue;
    if (!asp::projected_ip_to_raw_ip(P2, interp_dem, right_map_proj_cam,
                                      georef2, dem_georef))
      continue;

    ip1_cam.push_back(P1);
    ip2_cam.push_back(P2);
  }

  vw::vw_out() << "Saving " << ip1_cam.size() << " matches.\n";

  vw::vw_out() << "Writing: " << match_filename << std::endl;
  vw::ip::write_binary_match_file(match_filename, ip1_cam, ip2_cam);

} // End function matches_from_mapproj_images()

void findPairwiseMatches(asp::BaOptions & opt, // will change
                         std::vector<std::string> const& map_files,
                         std::string const& mapproj_dem,
                         std::vector<vw::Vector3> const& estimated_camera_gcc,
                         bool need_no_matches) {

  int num_images = opt.image_files.size();
  const bool got_est_cam_positions =
    (estimated_camera_gcc.size() == static_cast<size_t>(num_images));

  // When prior matches are used, accept them in any order
  bool external_matches = (!opt.clean_match_files_prefix.empty() ||
                           !opt.match_files_prefix.empty() ||
                           opt.skip_matching);

  // Make a list of all the image pairs to find matches for. When using external
  // matches, try to read read both image1__image2 and image2__image1 matches.
  std::vector<std::pair<int,int>> all_pairs;
  if (!need_no_matches)
    asp::determine_image_pairs(// Inputs
                               opt.overlap_limit, opt.match_first_to_last,
                               external_matches,
                               opt.image_files, got_est_cam_positions,
                               opt.position_filter_dist, estimated_camera_gcc,
                               opt.have_overlap_list, opt.overlap_list,
                               // Output
                               all_pairs);

  // Need this information in parallel_bundle_adjust for load balancing.
  if (opt.query_num_image_pairs) {
    vw::vw_out() << "num_image_pairs," << all_pairs.size() << "\n";
    exit(0);
  }

  // Assign the pairs (match files) which this job should compute, when called
  // from parallel_bundle_adjust. TODO(oalexan1): It is hard to understand the
  // logic here.  
  size_t per_job = all_pairs.size() / opt.num_parallel_jobs; // Round down
  size_t remainder = all_pairs.size() % opt.num_parallel_jobs;
  size_t curr_job_start_index = 0, curr_job_num_tasks = 0;
  for (size_t i = 0; i <= opt.job_id; i++) {
    curr_job_num_tasks = per_job;
    if (i < remainder)
      curr_job_num_tasks++;
    curr_job_start_index += curr_job_num_tasks;
  }
  curr_job_start_index -= curr_job_num_tasks;

  // Sanity check
  if (opt.num_parallel_jobs == 1) {
    if (curr_job_start_index != 0 || curr_job_num_tasks != all_pairs.size())
      vw::vw_throw(vw::ArgumentErr() << "Book-keeping failure in bundle_adjust.\n");
  }

  std::vector<std::pair<int,int>> curr_job_pairs;
  for (size_t i = 0; i < curr_job_num_tasks; i++)
    curr_job_pairs.push_back(all_pairs[i+curr_job_start_index]);

  // When using match-files-prefix or clean_match_files_prefix, form the list of
  // match files, rather than searching for them exhaustively on disk, which can
  // get very slow.
  std::set<std::string> existing_files;
  if (external_matches && !need_no_matches) {
    std::string prefix = asp::match_file_prefix(opt.clean_match_files_prefix,
                                                opt.match_files_prefix,
                                                opt.out_prefix);
    vw_out() << "Computing the list of existing match files.\n";
    asp::listExistingMatchFiles(prefix, existing_files);
  }

  vw::cartography::GeoReference dem_georef;
  ImageViewRef<PixelMask<double>> interp_dem;
  if (mapproj_dem != "")
      asp::create_interp_dem(mapproj_dem, dem_georef, interp_dem);

  // Process the selected pairs
  // TODO(oalexan1): This block must be a function.
  for (size_t k = 0; k < curr_job_pairs.size(); k++) {

    if (need_no_matches)
      continue;

    const int i = curr_job_pairs[k].first;
    const int j = curr_job_pairs[k].second;

    std::string const& image1_path  = opt.image_files[i];  // alias
    std::string const& image2_path  = opt.image_files[j];  // alias
    std::string const& camera1_path = opt.camera_files[i]; // alias
    std::string const& camera2_path = opt.camera_files[j]; // alias

    // See if perhaps to load match files from a different source
    std::string match_file
      = asp::match_filename(opt.clean_match_files_prefix, opt.match_files_prefix,
                            opt.out_prefix, image1_path, image2_path);

    // The external match file does not exist, don't try to load it
    if (external_matches && existing_files.find(match_file) == existing_files.end())
      continue;

    opt.match_files[std::make_pair(i, j)] = match_file;

    // If we skip matching (which is the case, among other situations, when
    // using external matches), there's no point in checking if the match
    // files are recent.
    bool inputs_changed = false;
    if (!opt.skip_matching) {
      inputs_changed = (!asp::first_is_newer(match_file,
                                             image1_path,  image2_path,
                                             camera1_path, camera2_path));

      // We make an exception and not rebuild if explicitly asked
      if (asp::stereo_settings().force_reuse_match_files &&
          boost::filesystem::exists(match_file))
        inputs_changed = false;
    }

    if (!inputs_changed) {
      vw_out() << "\t--> Using cached match file: " << match_file << "\n";
      continue;
    }

    // Read no-data
    boost::shared_ptr<DiskImageResource>
      rsrc1(vw::DiskImageResourcePtr(image1_path)),
      rsrc2(vw::DiskImageResourcePtr(image2_path));
    if ((rsrc1->channels() > 1) || (rsrc2->channels() > 1))
      vw_throw(ArgumentErr() << "Error: Input images can only have a single channel!\n\n");
    float nodata1, nodata2;
    asp::get_nodata_values(rsrc1, rsrc2, nodata1, nodata2);

    // Set up the stereo session
    asp::SessionPtr
      session(asp::StereoSessionFactory::create(opt.stereo_session, // may change
                                                opt, image1_path, image2_path,
                                                camera1_path, camera2_path,
                                                opt.out_prefix));

    // Find matches between image pairs. This may not always succeed.
    try {
      if (opt.mapprojected_data == "")
        asp::ba_match_ip(opt, session, image1_path, image2_path,
                         opt.camera_models[i].get(),
                         opt.camera_models[j].get(),
                         match_file);
      else
        asp::matches_from_mapproj_images(i, j, opt, session, map_files, mapproj_dem,
                                         dem_georef, interp_dem, match_file);

      // Compute the coverage fraction
      std::vector<ip::InterestPoint> ip1, ip2;
      ip::read_binary_match_file(match_file, ip1, ip2);
      int right_ip_width = rsrc1->cols() *
                            static_cast<double>(100-opt.ip_edge_buffer_percent)/100.0;
      Vector2i ip_size(right_ip_width, rsrc1->rows());
      double ip_coverage = asp::calc_ip_coverage_fraction(ip2, ip_size);
      vw_out() << "IP coverage fraction = " << ip_coverage << std::endl;
    } catch (const std::exception& e) {
      vw_out() << "Could not find interest points between images "
                << opt.image_files[i] << " and " << opt.image_files[j] << std::endl;
      vw_out(WarningMessage) << e.what() << std::endl;
    } //End try/catch
  } // End loop through all input image pairs

} // end function findPairwiseMatches()

} // End namespace asp
