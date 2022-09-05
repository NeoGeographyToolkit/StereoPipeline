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


/// \file Camerautils.h
///

#ifndef __STEREO_SESSION_CAMERAUTILS_H__
#define __STEREO_SESSION_CAMERAUTILS_H__

#include <vector>
#include <string>

#include <vw/Camera/CameraModel.h>
#include <vw/FileIO/GdalWriteOptions.h>

namespace vw {
  namespace cartography {
    class Datum;
  }
  namespace ip {
    class InterestPoint;
  }
}

namespace asp {

// Options shared by bundle_adjust and jitter_solve
struct BaBaseOptions: public vw::GdalWriteOptions {
  std::string out_prefix, stereo_session, input_prefix, match_files_prefix,
    clean_match_files_prefix, ref_dem, heights_from_dem;
  int overlap_limit, min_matches, max_pairwise_matches, num_iterations,
    ip_edge_buffer_percent;
  bool match_first_to_last, single_threaded_cameras;
  double min_triangulation_angle, max_init_reproj_error, robust_threshold, parameter_tolerance;
  double ref_dem_weight, ref_dem_robust_threshold, heights_from_dem_weight,
    heights_from_dem_robust_threshold, camera_weight, rotation_weight, translation_weight;
  vw::Vector2 remove_outliers_by_disp_params;
  
  std::vector<std::string> image_files, camera_files;
  std::vector<boost::shared_ptr<vw::camera::CameraModel>> camera_models;
  std::map<std::pair<int, int>, std::string> match_files;

  BaBaseOptions(): min_triangulation_angle(0.0), camera_weight(-1.0),
                   rotation_weight(0.0), translation_weight(0.0),
                   robust_threshold(0.0), min_matches(0),
                   num_iterations(0), overlap_limit(0) {}
};
  
// Load cameras from given image and camera files
void load_cameras(std::vector<std::string> const& image_files,
                  std::vector<std::string> const& camera_files,
                  std::string const& out_prefix, 
                  vw::GdalWriteOptions const& opt,
                  bool approximate_pinhole_intrinsics,
                  // Outputs
                  std::string & stereo_session, // may change
                  bool & single_threaded_cameras,
                  std::vector<boost::shared_ptr<vw::camera::CameraModel>> & camera_models);
  
// Find the datum based on cameras. For stereo session pinhole will return WGS84.
void datum_from_cameras(std::vector<std::string> const& image_files,
                        std::vector<std::string> const& camera_files,
                        std::string & stereo_session, // may change
                        // Outputs
                        vw::cartography::Datum & datum);
  
// Find and sort the convergence angles for given cameras and interest points
void convergence_angles(vw::camera::CameraModel const * left_cam,
                        vw::camera::CameraModel const * right_cam,
                        std::vector<vw::ip::InterestPoint> const& left_ip,
                        std::vector<vw::ip::InterestPoint> const& right_ip,
                        std::vector<double> & sorted_angles);
} // end namespace asp

#endif // __STEREO_SESSION_CAMERAUTILS_H__
