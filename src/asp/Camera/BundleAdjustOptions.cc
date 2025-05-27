// __BEGIN_LICENSE__
//  Copyright (c) 2009-2024, United States Government as represented by the
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

// \file BundleAdjustOptions.cc

// Options for bundle adjustment.
#include <asp/Camera/BundleAdjustOptions.h>
#include <asp/Core/StereoSettings.h>
#include <asp/Core/ImageUtils.h>

#include <string>

namespace asp {

void BaOptions::copy_to_asp_settings() const {
  asp::stereo_settings().ip_detect_method        = ip_detect_method;
  asp::stereo_settings().epipolar_threshold      = epipolar_threshold;
  asp::stereo_settings().ip_inlier_factor        = ip_inlier_factor;
  asp::stereo_settings().ip_uniqueness_thresh    = ip_uniqueness_thresh;
  asp::stereo_settings().num_scales              = num_scales;
  asp::stereo_settings().nodata_value            = nodata_value;
  asp::stereo_settings().aster_use_csm           = aster_use_csm;
  asp::stereo_settings().ip_per_tile             = ip_per_tile;
  asp::stereo_settings().ip_per_image            = ip_per_image;
  asp::stereo_settings().matches_per_tile        = matches_per_tile;
  asp::stereo_settings().matches_per_tile_params = matches_per_tile_params;
  asp::stereo_settings().no_datum                = no_datum;
  asp::stereo_settings().use_least_squares       = false; // never true with ba

  // Note that by default rough homography and tri filtering are disabled
  // as input cameras may be too inaccurate for that.
  asp::stereo_settings().skip_rough_homography      = !enable_rough_homography;
  asp::stereo_settings().disable_tri_filtering      = !enable_tri_filtering;

  // Do not pass this as it will results in filtering by elevation and lonlat
  // with unoptimized cameras. We will do that filtering with optimized
  // cameras later.
  //asp::stereo_settings().elevation_limit            = elevation_limit;
  //asp::stereo_settings().lon_lat_limit              = lon_lat_limit;

  asp::stereo_settings().individually_normalize     = individually_normalize;
  asp::stereo_settings().force_reuse_match_files    = force_reuse_match_files;
  asp::stereo_settings().min_triangulation_angle    = min_triangulation_angle;
  asp::stereo_settings().ip_triangulation_max_error = ip_triangulation_max_error;
  asp::stereo_settings().ip_num_ransac_iterations   = ip_num_ransac_iterations;
  asp::stereo_settings().ip_edge_buffer_percent     = ip_edge_buffer_percent;
  asp::stereo_settings().ip_debug_images            = ip_debug_images;
  asp::stereo_settings().ip_normalize_tiles         = ip_normalize_tiles;
  asp::stereo_settings().flann_method               = flann_method;
  asp::stereo_settings().propagate_errors           = propagate_errors;
  asp::stereo_settings().ip_nodata_radius           = ip_nodata_radius;
  
  // The setting below is not used, but populate it for completeness
  asp::stereo_settings().horizontal_stddev          = vw::Vector2(horizontal_stddev,
                                                                  horizontal_stddev);
}

// Read the mapprojected data, if having the option --mapprojected-data.
void setupMapprojectedData(asp::BaOptions & opt,
                           bool need_no_matches,
                           std::vector<std::string> & map_files,
                           std::string & mapproj_dem) {

  // Clear the outputs
  map_files.clear();
  mapproj_dem.clear();
  
  if (!need_no_matches) {
    if (!opt.mapprojected_data_list.empty()) {
      asp::read_list(opt.mapprojected_data_list, map_files);
      opt.mapprojected_data = "non-empty"; // put a token value, to make it non-empty
    } else if (opt.mapprojected_data != "") {
      std::istringstream is(opt.mapprojected_data);
      std::string file;
      while (is >> file)
        map_files.push_back(file);
    }
    
    if (!opt.mapprojected_data.empty()) {
      // Read the mapproj DEM from the first image geoheader or as the last element
      // in the list.
      if (opt.image_files.size() + 1 != map_files.size() &&
          opt.image_files.size()     != map_files.size()) 
        vw::vw_throw(vw::ArgumentErr() << "Error: Expecting as many mapprojected images as "
          << "cameras and, potentially, a DEM at the end of the mapprojected list.\n");

      if (opt.image_files.size() == map_files.size()) {
        // Read the DEM from first image header  
        std::string adj_key, img_file_key, cam_type_key, cam_file_key, dem_file_key;
        std::string adj_prefix_raw, image_file_raw, cam_type, cam_file_raw, dem_file;
        asp::read_mapproj_header(map_files.at(0),
                                  adj_key, img_file_key, cam_type_key, 
                                  cam_file_key, dem_file_key,
                                  adj_prefix_raw, image_file_raw, cam_type,
                                  cam_file_raw, 
                                  mapproj_dem); // read here
      } else if (opt.image_files.size() + 1 == map_files.size()) {
        // Pull out the dem from the list
        mapproj_dem = map_files.back();
        map_files.erase(map_files.end() - 1);
      }
    }
    
    // The mapproj DEM must be non-empty
    if (!map_files.empty() && mapproj_dem.empty())
      vw::vw_throw(vw::ArgumentErr()
                << "Interest point matching with mapprojected images requires a DEM. "
                << "None found either on the command line or in a geoheader of any "
                << "mapprojected image.\n");
  }
  
  if (!map_files.empty()) {
    if (!opt.input_prefix.empty() || !opt.initial_transform_file.empty() ||
        need_no_matches)
      vw::vw_throw(vw::ArgumentErr()
                << "Cannot use mapprojected data with initial adjustments, "
                << "an initial transform, or ISIS cnet or NVM input.\n");
  }
}

} // end namespace asp
