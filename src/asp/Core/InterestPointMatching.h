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

#ifndef __ASP_CORE_INTEREST_POINT_MATCHING_H__
#define __ASP_CORE_INTEREST_POINT_MATCHING_H__

#include <asp/Core/EpipolarIpMatching.h>
#include <asp/Core/IpFilterUtils.h>

#include <vw/Image/ImageView.h>
#include <vw/Image/ImageViewRef.h>
#include <vw/Camera/CameraModel.h>
#include <vw/InterestPoint/InterestPoint.h>
#include <vw/Cartography/Datum.h>
#include <vw/Cartography/GeoReference.h>
#include <vw/Math/Transform.h>
#include <vw/Math/Matrix.h>

#include <vector>
#include <string>

namespace asp {

// Debug utility to write out matches on top of the images
void write_match_image(std::string const& out_file_name,
                       vw::ImageViewRef<float> const& image1,
                       vw::ImageViewRef<float> const& image2,
                       std::vector<vw::InterestPoint> const& matched_ip1,
                       std::vector<vw::InterestPoint> const& matched_ip2);

// Calls ip matching assuming a datum. We may or not apply a homography to
// make right image like left image. This is useful so that both images have
// similar scale and similar affine qualities.
// - Only the left image will use an IP file since the right image is
//   modified.
bool match_ip_with_datum(bool single_threaded_camera,
                         bool use_rough_homography,
                         vw::CameraModel* cam1,
                         vw::CameraModel* cam2,
                         vw::ImageViewRef<float> const& image1,
                         vw::ImageViewRef<float> const& image2,
                         int ip_per_tile,
                         vw::Datum const& datum,
                         std::string const& match_filename,
                         size_t number_of_jobs,
                         double epipolar_threshold,
                         double uniqueness_threshold,
                         std::string const left_vwip_file = "",
                         std::string const right_vwip_file = "",
                         double nodata1 = std::numeric_limits<double>::quiet_NaN(),
                         double nodata2 = std::numeric_limits<double>::quiet_NaN());

// Match the ip and save the match file. No epipolar constraint
// is used in this mode.
void match_ip_no_datum(vw::InterestPointList const& ip1,
                       vw::InterestPointList const& ip2,
                       vw::ImageViewRef<float> const& image1,
                       vw::ImageViewRef<float> const& image2,
                       size_t number_of_jobs,
                       // Outputs
                       std::vector<vw::InterestPoint>& matched_ip1,
                       std::vector<vw::InterestPoint>& matched_ip2,
                       std::string const& match_file);

// Do IP matching, return the best translation+scale fitting functor.
vw::Matrix<double>
translation_ip_matching(vw::ImageView<vw::PixelGray<float>> const& image1,
                        vw::ImageView<vw::PixelGray<float>> const& image2,
                        int ip_per_tile,
                        std::string const left_vwip_file = "",
                        std::string const right_vwip_file = "",
                        double nodata1 = std::numeric_limits<double>::quiet_NaN(),
                        double nodata2 = std::numeric_limits<double>::quiet_NaN());

// Match IP using homography alignment.
bool homography_ip_matching(vw::ImageViewRef<float> const& image1,
                            vw::ImageViewRef<float> const& image2,
                            int ip_per_tile,
                            int inlier_threshold,
                            std::string const& match_filename,
                            size_t number_of_jobs,
                            std::string const left_vwip_file = "",
                            std::string const right_vwip_file = "",
                            bool use_cached_ip = false,
                            double nodata1 = std::numeric_limits<double>::quiet_NaN(),
                            double nodata2 = std::numeric_limits<double>::quiet_NaN(),
                            vw::BBox2i const& bbox1 = vw::BBox2i(),
                            vw::BBox2i const& bbox2 = vw::BBox2i(),
                            bool filter_by_ground_loc = false,
                            vw::GeoReference const& georef1 = vw::GeoReference(),
                            vw::GeoReference const& georef2 = vw::GeoReference());

// Homography rectification helper
vw::Vector2i
homography_rectification(bool adjust_left_image_size,
                         bool tight_inlier_threshold,
                         vw::Vector2i const& left_size,
                         vw::Vector2i const& right_size,
                         std::vector<vw::InterestPoint> const& left_ip,
                         std::vector<vw::InterestPoint> const& right_ip,
                         vw::Matrix<double>& left_matrix,
                         vw::Matrix<double>& right_matrix);

// Check if homography matrix is well-conditioned
void check_homography_matrix(vw::Matrix<double> const& H,
                             int left_size, int right_size,
                             int num_inliers);

// Create interest points from valid D_sub values and make them full scale
void aligned_ip_from_D_sub(vw::ImageViewRef<vw::PixelMask<vw::Vector2f>> const& sub_disp,
                           vw::Vector2 const& upsample_scale,
                           std::vector<vw::InterestPoint>& left_ip,
                           std::vector<vw::InterestPoint>& right_ip);

// Detect interest points
void detect_ip(vw::InterestPointList& ip,
               vw::ImageViewRef<float> const& image,
               int ip_per_image, int ip_per_tile, std::string const vwip_file, 
               double nodata, bool use_cached_ip);

// Detect IP in a pair of images and apply rudimentary filtering.
// Returns false if either image ended up with zero IP.
bool detect_ip_pair(vw::InterestPointList& ip1,
                    vw::InterestPointList& ip2,
                    vw::ImageViewRef<float> const& image1,
                    vw::ImageViewRef<float> const& image2,
                    int ip_per_image, int ip_per_tile,
                    std::string const left_vwip_file,
                    std::string const right_vwip_file,
                    double nodata1, double nodata2,
                    bool use_cached_ip);

// Detect interest points. Return also the rough homography that aligns the right image
// to the left.
bool detect_ip_aligned_pair(vw::CameraModel* cam1,
                            vw::CameraModel* cam2,
                            vw::ImageViewRef<float> const& image1,
                            vw::ImageViewRef<float> const& image2,
                            int ip_per_image, int ip_per_tile,
                            vw::Datum const& datum,
                            std::string const left_vwip_file,
                            double nodata1, double nodata2,
                            // Outputs
                            vw::InterestPointList& ip1,
                            vw::InterestPointList& ip2,
                            vw::Matrix<double>& rough_homography);

// Detect interest points and use a simple matching technique.
void detect_match_ip(std::vector<vw::InterestPoint>& matched_ip1,
                     std::vector<vw::InterestPoint>& matched_ip2,
                     vw::ImageViewRef<float> const& image1,
                     vw::ImageViewRef<float> const& image2,
                     int ip_per_tile, size_t number_of_jobs,
                     std::string const left_vwip_file = "",
                     std::string const right_vwip_file = "",
                     bool use_cached_ip = false,
                     double nodata1 = std::numeric_limits<double>::quiet_NaN(),
                     double nodata2 = std::numeric_limits<double>::quiet_NaN(),
                     std::string const& match_file = "",
                     vw::BBox2i const& bbox1 = vw::BBox2i(),
                     vw::BBox2i const& bbox2 = vw::BBox2i(),
                     bool filter_by_ground_loc = false,
                     vw::GeoReference const& georef1 = vw::GeoReference(),
                     vw::GeoReference const& georef2 = vw::GeoReference());

} // namespace asp

#endif // __ASP_CORE_INTEREST_POINT_MATCHING_H__
