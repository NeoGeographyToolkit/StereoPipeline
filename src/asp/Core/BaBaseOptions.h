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

/// \file BaBaseOptions.h
///
/// Base options structure shared by bundle_adjust, jitter_solve, and rig_calibrator.

#ifndef __ASP_CORE_BA_BASE_OPTIONS_H__
#define __ASP_CORE_BA_BASE_OPTIONS_H__

#include <asp/Core/Bathymetry.h>
#include <vw/FileIO/GdalWriteOptions.h>
#include <vw/Math/Vector.h>
#include <vw/Math/BBox.h>
#include <vw/Camera/CameraModel.h>
#include <vw/Cartography/Datum.h>

#include <string>
#include <vector>
#include <set>
#include <map>

namespace asp {

// This must be const or else there's a crash
const std::string UNSPECIFIED_DATUM = "unspecified_datum";

/// These are the different camera modes that bundle_adjust supports.
enum BACameraType {BaCameraType_Pinhole    = 0,
                   BaCameraType_OpticalBar = 1,
                   BaCameraType_CSM        = 2,
                   BaCameraType_Other      = 3};

// Options shared by bundle_adjust, jitter_solve, and rig_calibrator
struct BaBaseOptions: public vw::GdalWriteOptions {
  std::string out_prefix, stereo_session, input_prefix, match_files_prefix,
    clean_match_files_prefix, heights_from_dem, reference_terrain, mapproj_dem, weight_image,
    isis_cnet, nvm, nvm_no_shift, output_cnet_type,
    image_list, camera_list, mapprojected_data_list,
    fixed_image_list_str, camera_position_uncertainty_str;
  int overlap_limit, min_matches, max_pairwise_matches, num_iterations,
    ip_edge_buffer_percent, max_num_reference_points, num_passes;
  std::set<std::pair<std::string, std::string>> overlap_list;
  std::string overlap_list_file, auto_overlap_params, datum_str, proj_str,
    csv_format_str, csv_srs, csv_proj4_str, disparity_list, stereo_prefix_list,
    match_pair_sigma;
  bool have_overlap_list, propagate_errors, match_first_to_last, single_threaded_cameras,
    update_isis_cubes_with_csm_state, save_adjusted_rpc, fix_gcp_xyz, use_llh_error;
  double forced_triangulation_distance, min_triangulation_angle, max_triangulation_angle,
    max_init_reproj_error, robust_threshold, parameter_tolerance;
  double heights_from_dem_uncertainty, reference_terrain_weight,
    reference_terrain_uncertainty, reference_terrain_robust_threshold,
    heights_from_dem_robust_threshold, camera_weight, rotation_weight,
    camera_position_weight, camera_position_robust_threshold,
    tri_weight, tri_robust_threshold, camera_position_uncertainty_power,
    max_disp_error;
  std::vector<vw::Vector2> camera_position_uncertainty;
  vw::Vector<double, 4> remove_outliers_params;
  BACameraType camera_type;
  std::vector<std::string> image_files, camera_files, gcp_files;
  std::vector<vw::CamPtr> camera_models;
  std::map<std::pair<int, int>, std::string> match_files;
  std::map<std::pair<int, int>, double> match_sigmas;
  vw::cartography::Datum datum;
  vw::BBox2 proj_win; // Limit input triangulated points to this projwin
  double horizontal_stddev;
  vw::Vector<double> horizontal_stddev_vec; // may come from cameras or user
  vw::BathyData bathy_data;

  BaBaseOptions():
   forced_triangulation_distance(-1),
   min_triangulation_angle(0.01), camera_position_weight(0.0),
   camera_position_robust_threshold(0.0), camera_weight(-1.0),
   rotation_weight(0.0), tri_weight(0.1), tri_robust_threshold(0.1),
   robust_threshold(0.5), min_matches(0),
   num_iterations(0), num_passes(0),
   overlap_limit(0), have_overlap_list(false), propagate_errors(false),
   match_first_to_last(false), single_threaded_cameras(false),
   update_isis_cubes_with_csm_state(false),
   fix_gcp_xyz(false), use_llh_error(false),
   camera_type(BaCameraType_Other), max_num_reference_points(-1),
   datum(vw::cartography::Datum(asp::UNSPECIFIED_DATUM,
                                "User Specified Spheroid",
                                "Reference Meridian", 1, 1, 0)) {}
};

} // namespace asp

#endif // __ASP_CORE_BA_BASE_OPTIONS_H__