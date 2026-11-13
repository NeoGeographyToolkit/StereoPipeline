// __BEGIN_LICENSE__
//  Copyright (c) 2009-2026, United States Government as represented by the
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

/// \file BundleAdjustMatchFiles.cc
///

#include <asp/Camera/BundleAdjustCamera.h>
#include <asp/Camera/BundleAdjustMatchFiles.h>
#include <asp/Core/IpMatchingAlgs.h>
#include <asp/Core/ImageUtils.h>

#include <vw/BundleAdjustment/ControlNetwork.h>
#include <vw/Cartography/CameraBBox.h>
#include <vw/InterestPoint/MatcherIO.h>
#include <vw/Image/ImageChannels.h>

#include <boost/filesystem.hpp>

#include <map>
#include <set>
#include <string>
#include <tuple>
#include <utility>
#include <vector>

using namespace vw;

namespace asp {

// Mapproject interest points onto a DEM and find the norm of their
// disagreement in meters. It is assumed that dem_georef
// was created by bilinear interpolation. The cameras must be with
// the latest adjustments applied to them.
void calcPairMapprojOffsets(int left_cam_index, int right_cam_index,
                            std::vector<vw::CamPtr>            const& optimized_cams,
                            std::vector<vw::InterestPoint> const& left_ip,
                            std::vector<vw::InterestPoint> const& right_ip,
                            vw::GeoReference      const& dem_georef,
                            vw::ImageViewRef<vw::PixelMask<double>> const& interp_dem,
                            // Will append below
                            std::vector<vw::Vector<float, 4>>       & mapprojPoints,
                            std::vector<float>                      & mapprojOffsets) {

  // Wipe mapprojOffsets
  mapprojOffsets.clear();

  // Will append to mapprojPoints, so don't wipe it

  for (size_t ip_it = 0; ip_it < left_ip.size(); ip_it++) {

    bool treat_nodata_as_zero = false;
    bool has_intersection = false;
    double height_error_tol = 0.001; // 1 mm should be enough
    Vector3 xyz_guess;

    Vector2 left_pix(left_ip[ip_it].x, left_ip[ip_it].y);
    Vector3 left_dem_xyz = vw::camera_pixel_to_dem_xyz
      (optimized_cams[left_cam_index]->camera_center(left_pix),
       optimized_cams[left_cam_index]->pixel_to_vector(left_pix),
       vw::pixel_cast<vw::PixelMask<float>>(interp_dem), dem_georef,
       treat_nodata_as_zero, has_intersection,
       height_error_tol, xyz_guess);
    if (!has_intersection)
      continue;

    // Do the same for right. Use left pixel as initial guess
    xyz_guess = left_dem_xyz;
    Vector2 right_pix(right_ip[ip_it].x, right_ip[ip_it].y);
    Vector3 right_dem_xyz = vw::camera_pixel_to_dem_xyz
      (optimized_cams[right_cam_index]->camera_center(right_pix),
       optimized_cams[right_cam_index]->pixel_to_vector(right_pix),
       vw::pixel_cast<vw::PixelMask<float>>(interp_dem), dem_georef,
       treat_nodata_as_zero, has_intersection,
       height_error_tol, xyz_guess);
    if (!has_intersection)
      continue;

    Vector3 mid_pt = (left_dem_xyz + right_dem_xyz)/2.0;
    double dist = norm_2(left_dem_xyz - right_dem_xyz);

    // Keep in the same structure both the midpoint between these two
    // mapprojected ip, as lon-lat,height, and their distance, as
    // later the bookkeeping of mapprojOffsets will be different.
    // Float precision is enough, and will save on memory.
    Vector<float, 4> point;
    subvector(point, 0, 3) = dem_georef.datum().cartesian_to_geodetic(mid_pt);
    point[3] = dist;

    mapprojPoints.push_back(point);
    mapprojOffsets.push_back(dist);
  }

  return;
}

// This is called either with original or inlier ip
void processMatchPair(size_t left_index, size_t right_index,
                      std::vector<vw::InterestPoint> const& left_ip,
                      std::vector<vw::InterestPoint> const& right_ip,
                      std::vector<vw::CamPtr> const& optimized_cams,
                      vw::GeoReference const& mapproj_dem_georef,
                      vw::ImageViewRef<vw::PixelMask<double>> const& interp_mapproj_dem,
                      vw::Datum const& datum,
                      bool save_mapproj_match_points_offsets,
                      bool propagate_errors,
                      vw::Vector<double> const& horizontal_stddev_vec,
                      vw::BathyData const& bathy_data,
                      // Outputs
                      // Will append to entities below
                      std::vector<asp::MatchPairStats>  & convAngles,
                      std::vector<vw::Vector<float, 4>> & mapprojPoints,
                      std::vector<asp::MatchPairStats> & mapprojOffsets,
                      std::vector<std::vector<float>>  & mapprojOffsetsPerCam,
                      std::vector<asp::HorizVertErrorStats> & horizVertErrors,
                      std::vector<vw::Vector<double, 5>> & uncertPoints) {

  convAngles.push_back(asp::MatchPairStats()); // add an element, then populate it
  std::vector<double> sorted_angles;
  asp::convergence_angles(optimized_cams[left_index].get(), optimized_cams[right_index].get(),
                          left_ip, right_ip, sorted_angles);
  convAngles.back().populate(left_index, right_index, sorted_angles);

  if (save_mapproj_match_points_offsets) {
    std::vector<float> localMapprojOffsets;
    asp::calcPairMapprojOffsets(left_index, right_index,
                                optimized_cams,
                                left_ip, right_ip,
                                mapproj_dem_georef, interp_mapproj_dem,
                                mapprojPoints, // will append here
                                localMapprojOffsets);
    mapprojOffsets.push_back(asp::MatchPairStats()); // add an elem, then populate it
    mapprojOffsets.back().populate(left_index, right_index, localMapprojOffsets);
    for (size_t map_it = 0; map_it < localMapprojOffsets.size(); map_it++) {
      mapprojOffsetsPerCam[left_index].push_back(localMapprojOffsets[map_it]);
      mapprojOffsetsPerCam[right_index].push_back(localMapprojOffsets[map_it]);
    }
  }

  if (propagate_errors) {
    // Ensure her that proper values are passed for the input std devs
    horizVertErrors.push_back(asp::HorizVertErrorStats()); // add an elem, then populate it
    asp::propagatedErrorStats(left_index, right_index,
                              optimized_cams[left_index].get(),
                              optimized_cams[right_index].get(),
                              left_ip, right_ip,
                              horizontal_stddev_vec[left_index],
                              horizontal_stddev_vec[right_index],
                              datum,
                              bathy_data,
                              uncertPoints,
                              horizVertErrors.back());
  }

  return;
}

// Calculate convergence angles. Remove the outliers flagged earlier,
// if remove_outliers is true. Compute offsets of mapprojected matches,
// if a DEM is given. These are done together as they rely on
// reloading interest point matches, which is expensive so the matches
// are used for both operations.
typedef std::tuple<float, float, float, float> Quadruplet;

// Iterate over the control network and record, for each image, the (control point
// index, measure index) of every surviving inlier measure, sorted by control point.
// Optionally record image pairs contributing to at least one surviving measure.
// This uses a linear per-image index rather than quadratic pairwise storage.
void build_cnet_image_index(vw::ControlNetwork const& cnet,
                            bool remove_outliers,
                            bool need_match_pairs,
                            std::set<int> const& outliers,
                            std::set<std::pair<int, int>> const& outlier_obs,
                            std::map<int, std::vector<std::pair<int, int>>> & image_index,
                            std::set<std::pair<int, int>> & match_pairs) {

  image_index.clear();
  match_pairs.clear();

  if (!remove_outliers && !need_match_pairs)
    return;

  for (int ipt = 0; ipt < cnet.size(); ipt++) {
    // Skip outliers
    if (outliers.find(ipt) != outliers.end())
      continue;
    // Skip gcp
    if (cnet[ipt].type() == ControlPoint::GroundControlPoint)
      continue;

    int num_meas = cnet[ipt].size();
    // Record the surviving measures of this point, by image.
    for (int m = 0; m < num_meas; m++) {
      int img = cnet[ipt][m].image_id();
      // Skip measurements dropped as per-observation outliers, so the clean
      // match files and the stats reflect only the measurements that survived.
      if (outlier_obs.find(std::make_pair(img, ipt)) != outlier_obs.end())
        continue;
      image_index[img].push_back(std::make_pair(ipt, m));
    }
    // Record the image pairs this point contributes to. Only needed to create
    // match files from scratch, so skip the quadratic loop otherwise.
    if (need_match_pairs) {
      for (int i1 = 0; i1 < num_meas; i1++) {
        for (int i2 = 0; i2 < num_meas; i2++) {
          int a = cnet[ipt][i1].image_id(), b = cnet[ipt][i2].image_id();
          if (a == b)
            continue;
          if (outlier_obs.find(std::make_pair(a, ipt)) != outlier_obs.end() ||
              outlier_obs.find(std::make_pair(b, ipt)) != outlier_obs.end())
            continue;
          match_pairs.insert(std::make_pair(a, b));
        }
      }
    }
  }

  // Sort each image's measures by control point index for the merge below.
  for (auto & kv: image_index)
    std::sort(kv.second.begin(), kv.second.end());
}

// Reconstruct, for a single image pair, the map from a match's position
// quadruplet to its (left, right) interest-point scales, by intersecting the two
// images' sorted point lists. Built on demand for one pair at a time, so memory
// stays bounded. The std::map reproduces the exact sorted, de-duplicated per-pair
// structure used below, so the clean match files are byte-identical to before.
void makePairQuads(vw::ControlNetwork const& cnet,
                   std::map<int, std::vector<std::pair<int, int>>> const& image_index,
                   int left_index, int right_index,
                   std::map<Quadruplet, std::pair<float, float>> & quads) {
  quads.clear();
  auto la = image_index.find(left_index), ra = image_index.find(right_index);
  if (la == image_index.end() || ra == image_index.end())
    return;
  std::vector<std::pair<int, int>> const& lv = la->second;
  std::vector<std::pair<int, int>> const& rv = ra->second;
  size_t i = 0, j = 0;
  while (i < lv.size() && j < rv.size()) {
    if (lv[i].first < rv[j].first) { i++; continue; }
    if (rv[j].first < lv[i].first) { j++; continue; }
    // Same control point: a match for this image pair.
    int ipt = lv[i].first;
    auto const& ml = cnet[ipt][lv[i].second];
    auto const& mr = cnet[ipt][rv[j].second];
    quads[Quadruplet(ml.position()[0], ml.position()[1],
                     mr.position()[0], mr.position()[1])]
      = std::make_pair(ml.sigma()[0], mr.sigma()[0]);
    i++; j++;
  }
}

void matchFilesProcessing(vw::ControlNetwork       const& cnet,
                          asp::BaBaseOptions           const& opt,
                          std::vector<vw::CamPtr>      const& optimized_cams,
                          bool                                remove_outliers,
                          std::set<int>                const& outliers,
                          std::set<std::pair<int, int>> const& outlier_obs,
                          std::string                  const& mapproj_dem,
                          bool                                propagate_errors,
                          vw::Vector<double>           const& horizontal_stddev_vec,
                          bool                                save_clean_matches,
                          std::map<std::pair<int, int>, std::string> const& match_files,
                          bool                                matches_as_txt) {

  vw_out() << "Creating reports.\n";

  std::vector<vw::Vector<float, 4>> mapprojPoints; // all points, not just stats
  std::vector<asp::MatchPairStats> convAngles, mapprojOffsets;
  std::vector<std::vector<float>> mapprojOffsetsPerCam;
  std::vector<asp::HorizVertErrorStats> horizVertErrors;
  std::vector<vw::Vector<double, 5>> uncertPoints; // per-match propagated errors

  // Wipe the outputs
  mapprojPoints.clear();
  convAngles.clear();
  mapprojOffsets.clear();
  mapprojOffsetsPerCam.clear();
  horizVertErrors.clear();

  bool save_mapproj_match_points_offsets = (!mapproj_dem.empty());
  vw::GeoReference mapproj_dem_georef;
  ImageViewRef<PixelMask<double>> interp_mapproj_dem;
  if (save_mapproj_match_points_offsets)
    asp::create_interp_dem(mapproj_dem, mapproj_dem_georef, interp_mapproj_dem);

  int num_cameras = opt.image_files.size();
  mapprojOffsetsPerCam.resize(num_cameras);

  // A per-image index: for each image, the (control point index, local measure
  // index) of every surviving measure in it, sorted by control point index. This
  // is linear in the number of measures.
  std::map<int, std::vector<std::pair<int, int>>> image_index;
  // The set of image pairs sharing at least one surviving measure. Needed only to
  // enumerate match files when there are no input match files (ISIS cnet or nvm).
  std::set<std::pair<int, int>> match_pairs;
  bool need_match_pairs = (!opt.isis_cnet.empty() || !opt.nvm.empty());
  build_cnet_image_index(cnet, remove_outliers, need_match_pairs,
                         outliers, outlier_obs, image_index, match_pairs);

  // If we read the matches from an ISIS cnet or nvm, there are no match files.
  // Then create them. 
  std::map<std::pair<int, int>, std::string> local_match_files = match_files;
  if (opt.isis_cnet != "" || opt.nvm != "") {
    // iterate over match pairs
    local_match_files.clear();
    for (auto const& pr: match_pairs) {
      int left_index  = pr.first;
      int right_index = pr.second;
      // When creating match files from scratch, let the first index
      // be less than the second.
      if (left_index > right_index)
        continue;
      std::string match_file
        = vw::match_filename(opt.out_prefix,
                             opt.image_files[left_index],
                             opt.image_files[right_index],
                             matches_as_txt);
      local_match_files[std::make_pair(left_index, right_index)] = match_file;
    }
  }

  // Print only the first few match files, to avoid verbosity.
  size_t const max_verbose_match_msgs = 100;
  size_t num_match_msgs = 0;

  // Work on individual image pairs
  for (const auto& match_it: local_match_files) {

    std::pair<int, int> cam_pair   = match_it.first;
    std::string         match_file = match_it.second;
    size_t left_index  = cam_pair.first;
    size_t right_index = cam_pair.second;
    if (left_index == right_index)
      vw::vw_throw(vw::ArgumentErr() << "Bookkeeping failure. Cannot have interest point "
                   << "matches between an image and itself.\n");

    // Reconstruct this pair's position->scale map from the control network, once,
    // for the uses below. Empty if the pair shares no surviving measures.
    std::map<Quadruplet, std::pair<float, float>> curr_quads;
    makePairQuads(cnet, image_index, (int)left_index, (int)right_index, curr_quads);

    std::vector<vw::InterestPoint> orig_left_ip, orig_right_ip;
    if (opt.isis_cnet != "" || opt.nvm != "") {
      // Must create the matches from the cnet.
      auto const& match_pair = curr_quads; // alias
      // Iterate over the quadruplets and build matches. The scale is not tracked
      // for an ISIS cnet or nvm, so use a unit scale.
      for (auto const& q: match_pair) {
        auto const& pos = q.first; // position quadruplet (x1, y1, x2, y2)
        double s = 1.0; // scale
        orig_left_ip.push_back(vw::InterestPoint(std::get<0>(pos), std::get<1>(pos), s));
        orig_right_ip.push_back(vw::InterestPoint(std::get<2>(pos), std::get<3>(pos), s));
      }

      // Write the matches formed from the cnet to disk
      if (opt.output_cnet_type == "match-files") {
        if (num_match_msgs < max_verbose_match_msgs)
          vw::vw_out() << "Writing: " << match_file << "\n";
        vw::write_match_file(match_file, orig_left_ip, orig_right_ip, matches_as_txt);
      }

    } else if (remove_outliers && !curr_quads.empty()) {
      // The matches were already read into the control network by
      // build_control_network, so do not re-read them from disk here, which is
      // slow at scale over a network file system. Rebuild the per-pair interest
      // points from the control network: the positions are the measure
      // positions and the scale is the measure sigma.
      auto const& quads = curr_quads;
      for (auto const& q: quads) {
        auto const& pos   = q.first;  // position quadruplet (x1, y1, x2, y2)
        auto const& scale = q.second; // interest point scale (left, right)
        orig_left_ip.push_back
          (vw::InterestPoint(std::get<0>(pos), std::get<1>(pos), scale.first));
        orig_right_ip.push_back
          (vw::InterestPoint(std::get<2>(pos), std::get<3>(pos), scale.second));
      }
    } else {
      // Read existing matches. Skip over match files that don't exist.
      if (!boost::filesystem::exists(match_file)) {
        if (num_match_msgs < max_verbose_match_msgs)
          vw_out() << "Skipping non-existent match file: " << match_file << "\n";
        continue;
      }
      // Read the original IP, to ensure later we write to disk only the subset
      // of the IP from the control network which are part of these original
      // ones.
      vw::read_match_file(match_file, orig_left_ip, orig_right_ip, matches_as_txt);
    }

    num_match_msgs++;
    if (num_match_msgs == max_verbose_match_msgs)
      vw::vw_out() << "Processed " << max_verbose_match_msgs
                   << " match files. Not printing more individual ones.\n";

    // Create a new convergence angle storage struct
    asp::MatchPairStats & convAngle = convAngles.back(); // alias
    if (!remove_outliers) {
      // Do some processing with orig ip. Otherwise this will be done below
      // with the inlier ip.
      processMatchPair(left_index, right_index,
                       orig_left_ip, orig_right_ip,
                       optimized_cams,
                       mapproj_dem_georef, interp_mapproj_dem, opt.datum,
                       save_mapproj_match_points_offsets,
                       propagate_errors, horizontal_stddev_vec, opt.bathy_data,
                       // Will append to entities below
                       convAngles, mapprojPoints, mapprojOffsets, mapprojOffsetsPerCam,
                       horizVertErrors, uncertPoints);
      // Since no outliers are removed, nothing else to do
      continue;
    }
    // Keep only inliers and non-gcp. GCP are used in optimization but are not
    // part of the originally found interest point matches.
    std::vector<vw::InterestPoint> left_ip, right_ip;
    for (size_t ip_iter = 0; ip_iter < orig_left_ip.size(); ip_iter++) {
      Quadruplet q(orig_left_ip[ip_iter].x, orig_left_ip[ip_iter].y,
                   orig_right_ip[ip_iter].x, orig_right_ip[ip_iter].y);
      auto const& match_pair = curr_quads; // alias
      if (match_pair.find(q) == match_pair.end())
        continue;

      // We do not copy descriptors, those take storage
      left_ip.push_back(vw::InterestPoint(orig_left_ip[ip_iter].x,
                                          orig_left_ip[ip_iter].y,
                                          orig_left_ip[ip_iter].scale));
      right_ip.push_back(vw::InterestPoint(orig_right_ip[ip_iter].x,
                                           orig_right_ip[ip_iter].y,
                                           orig_right_ip[ip_iter].scale));
    }

    // Filter by disparity
    // TODO(oalexan1): Note that this does not update the outliers set. Likely this
    // processing needs to move where other outlier filtering logic is.
    bool quiet = true; // Otherwise too many messages are printed
    if (opt.remove_outliers_params[0] > 0 && opt.remove_outliers_params[1] > 0.0) {
      // The typical value of 75 for opt.remove_outliers_params[1] may be too low.
      // Adjust it. pct = 75 becomes pct = 90. pct = 100 becomes pct = 100. So,
      // if starting under 100, it gets closer to 100 but stays under it.
      double pct = opt.remove_outliers_params[0];
      pct = 100.0 * (pct + 150.0) / 250.0;
      asp::filter_ip_by_disparity(pct, opt.remove_outliers_params[1],
                                  quiet, left_ip, right_ip);
    }

    if (num_cameras == 2) {
      // Compute the coverage fraction
      Vector2i right_image_size = file_image_size(opt.image_files[1]);
      int right_ip_width = right_image_size[0]*
        static_cast<double>(100.0 - std::max(opt.ip_edge_buffer_percent, 0))/100.0;
      Vector2i ip_size(right_ip_width, right_image_size[1]);
      double ip_coverage = asp::calc_ip_coverage_fraction(right_ip, ip_size);
      // Careful with the line below, it gets used in process_icebridge_batch.py.
      vw_out() << "IP coverage fraction after cleaning = " << ip_coverage << "\n";
    }

    // Process the inlier ip
    processMatchPair(left_index, right_index, left_ip, right_ip,
                     optimized_cams, mapproj_dem_georef, interp_mapproj_dem,
                     opt.datum,
                     save_mapproj_match_points_offsets,
                     propagate_errors, horizontal_stddev_vec, opt.bathy_data,
                     // Will append to entities below
                     convAngles, mapprojPoints, mapprojOffsets, mapprojOffsetsPerCam,
                     horizVertErrors, uncertPoints);

    if (opt.output_cnet_type != "match-files" || !save_clean_matches)
      continue; // Do not write match files

    // Make a clean copy of the file
    std::string clean_match_file = vw::clean_match_filename(match_file, matches_as_txt);
    if (opt.clean_match_files_prefix != "") {
      // Ensure "clean" does not show up twice
      clean_match_file = match_file;
      // Write the clean match file in the current dir, not where it was read from
      clean_match_file.replace(0, opt.clean_match_files_prefix.size(), opt.out_prefix);
    } else if (opt.match_files_prefix != "") {
      // Write the clean match file in the current dir, not where it was read from
      clean_match_file.replace(0, opt.match_files_prefix.size(), opt.out_prefix);
    }

    vw_out() << "Saving " << left_ip.size() << " filtered interest points.\n";
    vw_out() << "Writing: " << clean_match_file << "\n";
    vw::write_match_file(clean_match_file, left_ip, right_ip, matches_as_txt);

  } // End loop through the match files

  // Also find convergence angles from GCP, but only for pairs not already
  // covered by a match file above. So the report is the union of the two.
  {
    std::map<std::pair<int, int>,
             std::pair<std::vector<vw::InterestPoint>,
                       std::vector<vw::InterestPoint>>> gcp_pairs;
    for (int ipt = 0; ipt < cnet.size(); ipt++) {
      if (outliers.find(ipt) != outliers.end())
        continue;
      if (cnet[ipt].type() != ControlPoint::GroundControlPoint)
        continue;
      for (auto m1 = cnet[ipt].begin(); m1 != cnet[ipt].end(); m1++) {
        for (auto m2 = cnet[ipt].begin(); m2 != cnet[ipt].end(); m2++) {
          int left_index  = m1->image_id();
          int right_index = m2->image_id();
          if (left_index >= right_index)
            continue; // one direction only, and skip self-pairs
          // Skip pairs already handled through a match file above
          if (local_match_files.find(std::make_pair(left_index, right_index))
                != local_match_files.end() ||
              local_match_files.find(std::make_pair(right_index, left_index))
                != local_match_files.end())
            continue;
          auto & pr = gcp_pairs[std::make_pair(left_index, right_index)];
          pr.first.push_back
            (vw::InterestPoint(m1->position()[0], m1->position()[1], 1.0));
          pr.second.push_back
            (vw::InterestPoint(m2->position()[0], m2->position()[1], 1.0));
        }
      }
    }
    for (auto const& it: gcp_pairs) {
      int left_index  = it.first.first;
      int right_index = it.first.second;
      convAngles.push_back(asp::MatchPairStats());
      std::vector<double> sorted_angles;
      asp::convergence_angles(optimized_cams[left_index].get(),
                              optimized_cams[right_index].get(),
                              it.second.first, it.second.second, sorted_angles);
      convAngles.back().populate(left_index, right_index, sorted_angles);
    }
  }

  // Save the produced files
  std::string conv_angles_file = opt.out_prefix + "-convergence_angles.txt";
  asp::saveConvergenceAngles(conv_angles_file, convAngles, opt.image_files);

  if (!opt.mapproj_dem.empty())
    asp::saveMapprojOffsets(opt.out_prefix,
                            mapproj_dem_georef,
                            mapprojPoints,
                            mapprojOffsets,
                            mapprojOffsetsPerCam, // will change
                            opt.image_files);

  if (opt.propagate_errors) {
    std::string horiz_vert_errors_file = opt.out_prefix + "-triangulation_uncertainty.txt";
    asp::saveHorizVertErrors(horiz_vert_errors_file, horizVertErrors, opt.image_files);

    // Per-match propagated errors with their ground location, for gridding or
    // plotting (:numref:`error_propagation`). Very analogous to pointmap.csv.
    std::string pts_file = opt.out_prefix + "-triangulation_uncertainty_points.csv";
    vw_out() << "Writing: " << pts_file << "\n";
    std::ofstream ofs(pts_file.c_str());
    ofs << "# lon, lat, height_above_datum, horizontal_stddev, vertical_stddev\n";
    ofs << "# " << opt.datum << "\n";
    ofs.precision(17); // full double precision, as in pointmap.csv
    for (size_t it = 0; it < uncertPoints.size(); it++)
      ofs << uncertPoints[it][0] << ", " << uncertPoints[it][1] << ", "
          << uncertPoints[it][2] << ", " << uncertPoints[it][3] << ", "
          << uncertPoints[it][4] << "\n";
    ofs.close();
  }

  return;
}

} // end namespace asp
