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

#ifndef __ASP_CORE_EPIPOLAR_IP_MATCHING_H__
#define __ASP_CORE_EPIPOLAR_IP_MATCHING_H__

#include <vw/InterestPoint/InterestPoint.h>
#include <vw/Camera/CameraModel.h>
#include <vw/Cartography/Datum.h>
#include <vw/Image/ImageViewRef.h>
#include <vw/Math/Matrix.h>
#include <vw/Math/Vector.h>

#include <vector>
#include <map>
#include <set>

namespace asp {

// Choose the method used for IP matching
enum DetectIpMethod {
  DETECT_IP_METHOD_INTEGRAL = 0,
  DETECT_IP_METHOD_SIFT     = 1,
  DETECT_IP_METHOD_ORB      = 2,
  DETECT_IP_METHOD_AKAZE    = 3,
  DETECT_IP_METHOD_BRISK    = 4,
  DETECT_IP_METHOD_KAZE     = 5
};

// Given a value x, find all n such that x is in the interval 
// [n*tile_size - extra, (n+1)*tile_size + extra]. We assume
// that extra is at most half of tile size, so there are at most 
// 3 such indices.
void tile_extra_indices(double x, int tile_size, int extra, std::vector<int> & indices);

// Divide the left image and right aligned image into corresponding tiles.
void group_ip_in_tiles(std::vector<vw::InterestPoint> const& ip1_copy,
                       std::vector<vw::InterestPoint> const& ip2_copy,
                       vw::Matrix<double> align_matrix,
                       // Outputs
                       std::map<std::pair<int, int>, std::vector<vw::InterestPoint>> & ip1_vec,
                       std::map<std::pair<int, int>, std::vector<vw::InterestPoint>> & ip2_vec);

void append_new_matches(std::vector<vw::InterestPoint> const& local_matched_ip1,
                        std::vector<vw::InterestPoint> const& local_matched_ip2,
                        std::set<std::pair<float, float>> & seen1,
                        std::set<std::pair<float, float>> & seen2,
                        std::vector<vw::InterestPoint> & matched_ip1,
                        std::vector<vw::InterestPoint> & matched_ip2);

void pick_subset(int max_pairwise_matches, 
                 std::vector<vw::InterestPoint> & ip1, 
                 std::vector<vw::InterestPoint> & ip2);

// Match ip without a datum, cameras, epipolar lines.
void match_ip_no_datum(std::vector<vw::InterestPoint> const& ip1_copy,
                       std::vector<vw::InterestPoint> const& ip2_copy,
                       DetectIpMethod detect_method, double uniqueness_threshold, 
                       bool quiet,
                       // Outputs
                       std::vector<vw::InterestPoint>& matched_ip1,
                       std::vector<vw::InterestPoint>& matched_ip2);

// Match interest points per tile
void matches_per_tile(bool have_datum, 
                      bool single_threaded_camera,
                      DetectIpMethod detect_method,
                      std::vector<vw::InterestPoint> const& ip1_copy, 
                      std::vector<vw::InterestPoint> const& ip2_copy,
                      vw::CameraModel* cam1,
                      vw::CameraModel* cam2,
                      vw::Datum const& datum,
                      size_t number_of_jobs,
                      double epipolar_threshold,
                      double uniqueness_threshold,
                      vw::Matrix<double> const& align_matrix,
                      // Outputs
                      std::vector<vw::InterestPoint>& matched_ip1,
                      std::vector<vw::InterestPoint>& matched_ip2);

void epipolar_ip_matching_task(bool single_threaded_camera,
                               DetectIpMethod detect_method, 
                               int number_of_jobs,
                               double epipolar_threshold,
                               double uniqueness_threshold,
                               vw::Datum const& datum,
                               bool quiet,
                               vw::CameraModel* cam1,
                               vw::CameraModel* cam2,
                               vw::InterestPointList const& ip1,
                               vw::InterestPointList const& ip2,
                               // Outputs
                               std::vector<vw::InterestPoint>& matched_ip1,
                               std::vector<vw::InterestPoint>& matched_ip2);

bool epipolar_ip_matching(bool single_threaded_camera,
                          vw::InterestPointList const& ip1,
                          vw::InterestPointList const& ip2,
                          vw::CameraModel* cam1,
                          vw::CameraModel* cam2,
                          vw::ImageViewRef<float> const& image1,
                          vw::ImageViewRef<float> const& image2,
                          vw::Datum const& datum,
                          size_t number_of_jobs,
                          double epipolar_threshold,
                          double uniqueness_threshold,
                          double nodata1, double nodata2,
                          bool match_per_tile, 
                          vw::Matrix<double> const& align_matrix,
                          // Outputs
                          std::vector<vw::InterestPoint>& matched_ip1,
                          std::vector<vw::InterestPoint>& matched_ip2);

} // namespace asp

#endif // __ASP_CORE_EPIPOLAR_IP_MATCHING_H__
