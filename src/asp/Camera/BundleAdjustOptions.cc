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

// \file BundleAdjustOptions.cc

// Options for bundle adjustment.
#include <asp/Camera/BundleAdjustOptions.h>
#include <asp/Core/StereoSettings.h>

#include <string>

namespace asp {
  
void BaOptions::copy_to_asp_settings() const {
  asp::stereo_settings().ip_matching_method         = ip_detect_method;
  asp::stereo_settings().epipolar_threshold         = epipolar_threshold;
  asp::stereo_settings().ip_inlier_factor           = ip_inlier_factor;
  asp::stereo_settings().ip_uniqueness_thresh       = ip_uniqueness_thresh;
  asp::stereo_settings().num_scales                 = num_scales;
  asp::stereo_settings().nodata_value               = nodata_value;

  asp::stereo_settings().aster_use_csm = aster_use_csm;
  asp::stereo_settings().ip_per_tile = ip_per_tile;
  asp::stereo_settings().ip_per_image = ip_per_image;
  asp::stereo_settings().matches_per_tile = matches_per_tile;
  asp::stereo_settings().matches_per_tile_params = matches_per_tile_params;
  asp::stereo_settings().no_datum = no_datum;
  asp::stereo_settings().use_least_squares = false; // never true with ba

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
  // The setting below is not used, but populate it for completeness
  asp::stereo_settings().horizontal_stddev          = vw::Vector2(horizontal_stddev,
                                                                  horizontal_stddev);
}

} // end namespace asp
