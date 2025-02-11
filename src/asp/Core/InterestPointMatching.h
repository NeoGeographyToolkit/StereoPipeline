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

#ifndef __ASP_CORE_INTEREST_POINT_MATCHING_H__
#define __ASP_CORE_INTEREST_POINT_MATCHING_H__

#include <vw/Image/ImageViewBase.h>
#include <vw/Camera/CameraModel.h>
#include <vw/InterestPoint/InterestData.h>
#include <vw/Cartography/Datum.h>
#include <vw/Math/Transform.h>

namespace asp {

// Debug utility to write out a matches on top of the images
void write_match_image(std::string const& out_file_name,
                       vw::ImageViewRef<float> const& image1,
                       vw::ImageViewRef<float> const& image2,
                       std::vector<vw::ip::InterestPoint> const& matched_ip1,
                       std::vector<vw::ip::InterestPoint> const& matched_ip2);

  // Calls ip matching assuming a datum. We may or not apply a homography to
  // make right image like left image. This is useful so that both images have
  // similar scale and similar affine qualities.
  // - Only the left image will use an IP file since the right image is
  //   modified.
  bool match_ip_with_datum(bool single_threaded_camera,
                             bool use_rough_homography,
                             vw::camera::CameraModel* cam1,
                             vw::camera::CameraModel* cam2,
                             vw::ImageViewRef<float> const& image1,
                             vw::ImageViewRef<float> const& image2,
                             int ip_per_tile,
                             vw::cartography::Datum const& datum,
                             std::string const& match_filename,
                             size_t number_of_jobs,
                             double epipolar_threshold,
                             double uniqueness_threshold,
                             std::string const left_file_path ="",
                             std::string const right_file_path ="",
                             double nodata1 = std::numeric_limits<double>::quiet_NaN(),
                             double nodata2 = std::numeric_limits<double>::quiet_NaN());

  // Do IP matching, return, the best translation+scale fitting functor.
  vw::Matrix<double>
    translation_ip_matching(vw::ImageView<vw::PixelGray<float>> const& image1,
                            vw::ImageView<vw::PixelGray<float>> const& image2,
                            int ip_per_tile,
                            std::string const left_file_path ="",
                            std::string const right_file_path="",
                            double nodata1 = std::numeric_limits<double>::quiet_NaN(),
                            double nodata2 = std::numeric_limits<double>::quiet_NaN());

  //-------------------------------------------------------------------------------------------
  // These are IP filtering functions.

  /// Optionally remove pixels from the opposite sides of the images.
  /// - This is a very simple filter for the left/right image common case.
  void side_ip_filtering(vw::ip::InterestPointList& ip1,
                         vw::ip::InterestPointList& ip2,
                         vw::BBox2i const& bbox1, vw::BBox2i const& bbox2);

  /// IP matching that uses clustering on triangulation error to
  /// determine inliers.  Check output this filter can fail.
  ///
  /// Input and output is the valid indices. Valid indices must have
  /// something to start with.
  bool tri_ip_filtering(std::vector<vw::ip::InterestPoint> const& ip1,
                        std::vector<vw::ip::InterestPoint> const& ip2,
                        vw::camera::CameraModel* cam1,
                        vw::camera::CameraModel* cam2,
                        std::list<size_t>& valid_indices);

  /// 4 stddev filtering. Deletes any disparity measurement that is 4
  /// stddev away from the measurements of it's local neighbors. We
  /// kill off worse offender one at a time until everyone is compliant.
  bool stddev_ip_filtering(std::vector<vw::ip::InterestPoint> const& ip1,
          std::vector<vw::ip::InterestPoint> const& ip2,
          std::list<size_t>& valid_indices);


  // Filter IP by ensuring that the triangulated IP are in the given lon-lat-height box
  size_t filter_ip_by_lonlat_and_elevation(vw::TransformPtr         tx_left,
                                           vw::TransformPtr         tx_right,
                                           vw::camera::CameraModel* left_camera_model,
                                           vw::camera::CameraModel* right_camera_model,
                                           vw::cartography::Datum const& datum,
                                           std::vector<vw::ip::InterestPoint> const& ip1_in,
                                           std::vector<vw::ip::InterestPoint> const& ip2_in,
                                           vw::Vector2 const & elevation_limit,
                                           vw::BBox2   const & lon_lat_limit,
                                           std::vector<vw::ip::InterestPoint> & ip1_out,
                                           std::vector<vw::ip::InterestPoint> & ip2_out);

  // Filter ip by triangulation error, reprojection error, and height range
  void filter_ip_using_cameras(std::vector<vw::ip::InterestPoint> & ip1,
                               std::vector<vw::ip::InterestPoint> & ip2,
                               vw::camera::CameraModel const* cam1,
                               vw::camera::CameraModel const* cam2,
                               vw::cartography::Datum  const& datum,
                               double pct, double factor);

  /// Filter IP points by how reasonably the disparity can change along rows
  /// - Returns the number of points remaining after filtering.
  size_t filter_ip_homog(std::vector<vw::ip::InterestPoint> const& ip1_in,
                         std::vector<vw::ip::InterestPoint> const& ip2_in,
                         std::vector<vw::ip::InterestPoint>      & ip1_out,
                         std::vector<vw::ip::InterestPoint>      & ip2_out,
                         int inlier_threshold = 1);

  // Filter IP using a given DEM and max height difference.  Assume that
  // the interest points have alignment applied to them (either via a
  // transform or from mapprojection).
  void ip_filter_using_dem(std::string              const & ip_filter_using_dem,
                           vw::TransformPtr                 tx_left,
                           vw::TransformPtr                 tx_right,
                           boost::shared_ptr<vw::camera::CameraModel> left_camera_model,
                           boost::shared_ptr<vw::camera::CameraModel> right_camera_model,
                           std::vector<vw::ip::InterestPoint> & left_aligned_ip,
                           std::vector<vw::ip::InterestPoint> & right_aligned_ip);

  // Estimate the search range by finding the median disparity and
  // creating a box of given dimensions around it. This assumes aligned
  // interest points.
  vw::BBox2 search_range_using_spread(double max_disp_spread,
                                     std::vector<vw::ip::InterestPoint> const& left_ip,
                                     std::vector<vw::ip::InterestPoint> const& right_ip);

  //-----------------------------------------------------------------------------
  // Miscellaneous IP functions

  /// Find a rough homography that maps right to left using the camera
  /// and datum information.
  /// - This intersects rays with the datum, then projects them into the other camera.
  vw::Matrix<double>
  rough_homography_fit(vw::camera::CameraModel* cam1,
            vw::camera::CameraModel* cam2,
            vw::BBox2i const& box1, vw::BBox2i const& box2,
            vw::cartography::Datum const& datum);

  /// Homography rectification that aligns the right image to the left
  /// image via a homography transform. It returns a vector2i of the
  /// ideal cropping size to use for the left and right image. The left
  /// transform is actually just a translation that sets origin to the
  /// shared corner of left and right.
  vw::Vector2i
  homography_rectification(bool adjust_left_image_size,
                           bool tight_inlier_threshold,
          vw::Vector2i const& left_size,
          vw::Vector2i const& right_size,
          std::vector<vw::ip::InterestPoint> const& left_ip,
          std::vector<vw::ip::InterestPoint> const& right_ip,
          vw::Matrix<double>& left_matrix,
          vw::Matrix<double>& right_matrix);

  // Create interest points from valid D_sub values and make them full scale
  // (while still having potentially a global alignment applied to them).
  void aligned_ip_from_D_sub(
    vw::ImageViewRef<vw::PixelMask<vw::Vector2f>> const & sub_disp,
    vw::Vector2                                   const & upsample_scale,
    std::vector<vw::ip::InterestPoint>                  & left_ip,
    std::vector<vw::ip::InterestPoint>                  & right_ip);

  // Homography IP matching
  // This applies only the homography constraint. Not the best.
  // Can optionally restrict the matching to given image bounding boxes,
  // which can result in big efficiency gains.
  bool homography_ip_matching(vw::ImageViewRef<float> const& image1,
                              vw::ImageViewRef<float> const& image2,
                              int ip_per_tile,
                              int inlier_threshold,
                              std::string const& match_filename,
                              size_t number_of_jobs,
                              std::string const  left_file_path,
                              std::string const  right_file_path,
                              double nodata1, double nodata2,
                              vw::BBox2i const& bbox1 = vw::BBox2i(),
                              vw::BBox2i const& bbox2 = vw::BBox2i());

// Match the ip and save the match file. No epipolar constraint
// is used in this mode.
void match_ip_no_datum(vw::ip::InterestPointList const& ip1,
                   vw::ip::InterestPointList const& ip2,
                   vw::ImageViewRef<float> const& image1,
                   vw::ImageViewRef<float> const& image2,
                   size_t number_of_jobs,
                   // Outputs
                   std::vector<vw::ip::InterestPoint>& matched_ip1,
                   std::vector<vw::ip::InterestPoint>& matched_ip2,
                   std::string const& match_file);

  // Choose the method used for IP matching
  enum DetectIpMethod {DETECT_IP_METHOD_INTEGRAL = 0,
                       DETECT_IP_METHOD_SIFT     = 1,
                       DETECT_IP_METHOD_ORB      = 2};

/// Detect interest points
/// This is not meant to be used directly. Use ip_matching() or
/// homography_ip_matching().
void detect_ip(vw::ip::InterestPointList& ip,
               vw::ImageViewRef<float> const& image,
               int ip_per_tile, std::string const file_path, double nodata);

// Detect IP in a pair of images and apply rudimentary filtering.
// Returns false if either image ended up with zero IP.
bool detect_ip_pair(vw::ip::InterestPointList& ip1,
                    vw::ip::InterestPointList& ip2,
                    vw::ImageViewRef<float> const& image1,
                    vw::ImageViewRef<float> const& image2,
                    int ip_per_tile,
                    std::string const left_file_path,
                    std::string const right_file_path,
                    double nodata1, double nodata2);

// Detect interest points. Return also the rough homography that aligns the right image
// to the left.
bool detect_ip_aligned_pair(vw::camera::CameraModel* cam1,
  vw::camera::CameraModel* cam2,
  vw::ImageViewRef<float> const& image1,
  vw::ImageViewRef<float> const& image2,
  int ip_per_tile,
  vw::cartography::Datum const& datum,
  std::string const left_file_path,
  double nodata1, double nodata2,
  // Outputs
  vw::ip::InterestPointList& ip1,
  vw::ip::InterestPointList& ip2,
  vw::Matrix<double> &rough_homography);

/// Detect interest points and use a simple matching technique.
/// This is not meant to be used directly. Use ip_matching().
void detect_match_ip(std::vector<vw::ip::InterestPoint>& matched_ip1,
                     std::vector<vw::ip::InterestPoint>& matched_ip2,
                     vw::ImageViewRef<float> const& image1,
                     vw::ImageViewRef<float> const& image2,
                     int ip_per_tile, size_t number_of_jobs,
                     std::string const left_file_path ="",
                     std::string const right_file_path="",
                     double nodata1 = std::numeric_limits<double>::quiet_NaN(),
                     double nodata2 = std::numeric_limits<double>::quiet_NaN(),
                     std::string const& match_file = "");

// A debug routine to save images with matches on top of them.
void write_match_image(std::string const& out_file_name,
                       vw::ImageViewRef<float> const& image1,
                       vw::ImageViewRef<float> const& image2,
                       std::vector<vw::ip::InterestPoint> const& matched_ip1,
                       std::vector<vw::ip::InterestPoint> const& matched_ip2);

} // End namespace asp

#endif//__ASP_CORE_INTEREST_POINT_MATCHING_H__
