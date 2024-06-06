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

#ifndef __ASP_CORE_INTEREST_POINT_MATCHING_H__
#define __ASP_CORE_INTEREST_POINT_MATCHING_H__

#include <vw/Core/Stopwatch.h>
#include <vw/Image/ImageViewBase.h>
#include <vw/Image/MaskViews.h>
#include <vw/Camera/CameraModel.h>
#include <vw/InterestPoint/Matcher.h>
#include <vw/InterestPoint/InterestData.h>
#include <vw/InterestPoint/IntegralDetector.h>
#include <vw/InterestPoint/InterestPointUtils.h>
#include <vw/Cartography/Datum.h>
#include <vw/Math/RANSAC.h>
#include <vw/Math/Geometry.h>
#include <vw/FileIO/FileUtils.h>
#include <vw/Core/Stopwatch.h>

#include <asp/Core/StereoSettings.h>
#include <boost/foreach.hpp>
#include <boost/math/special_functions/fpclassify.hpp>

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
  bool ip_matching_with_datum(bool single_threaded_camera,
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
void match_ip_pair(vw::ip::InterestPointList const& ip1, 
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
///
/// This is not meant to be used directly. Use ip_matching() or
/// homography_ip_matching().
// TODO(oalexan1): Cast this to ImageViewRef<float> and move to the .cc.
// Verify if this affects performance and results. It should not.
// Example: vw::ImageViewRef<float> im = vw::pixel_cast<float>(image);
template <class Image1T>
void detect_ip(vw::ip::InterestPointList& ip,
	       vw::ImageViewBase<Image1T> const& image,
	       int ip_per_tile, std::string const file_path, double nodata) {
  ip.clear();

  // If the images are cropped, redo the ip
  // TODO(oalexan1): This won't handle well the case when the input
  // images changed on disk. 
  bool crop_left  = (stereo_settings().left_image_crop_win  != vw::BBox2i(0, 0, 0, 0));
  bool crop_right = (stereo_settings().right_image_crop_win != vw::BBox2i(0, 0, 0, 0));
  bool rebuild = crop_left || crop_right;
  
  // If a valid file_path was provided, just try to read in the IP's from that file.
  if ((file_path != "") && (boost::filesystem::exists(file_path)) && !rebuild) {
    vw::vw_out() << "\t    Reading interest points from file: " << file_path << std::endl;
    ip = vw::ip::read_binary_ip_file_list(file_path);
    vw::vw_out() << "\t    Found interest points: " << ip.size() << std::endl;
    return;
  }
  
  vw::Stopwatch sw1;
  sw1.start();

  // Automatically determine how many ip we need. Can be overridden below
  // either by --ip-per-image or --ip-per-tile (the latter takes priority).
  double tile_size = 1024.0;
  vw::BBox2i box = vw::bounding_box(image.impl());
  double number_tiles = (box.width() / tile_size) * (box.height() / tile_size);

  int ip_per_image = 5000; // default
  if (stereo_settings().ip_per_image > 0) 
    ip_per_image = stereo_settings().ip_per_image; 
  
  size_t points_per_tile = double(ip_per_image) / number_tiles;
  if (points_per_tile > 5000) points_per_tile = 5000;
  if (points_per_tile < 50  ) points_per_tile = 50;

  // See if to override with ip per tile
  if (ip_per_tile != 0)
    points_per_tile = ip_per_tile;

  // Record the current number of ip per tile. Later this can be used
  // for a subsequent attempt, if this one failed.
  asp::stereo_settings().ip_per_tile = points_per_tile;
  
  vw::vw_out() << "\t    Using " << points_per_tile 
    << " interest points per tile (1024^2 px).\n";

  const bool has_nodata = !boost::math::isnan(nodata);
  
  // Load the detection method from stereo_settings.
  // - This relies on a direct match in the enum integer value.
  DetectIpMethod detect_method 
    = static_cast<DetectIpMethod>(stereo_settings().ip_matching_method);

  // Detect interest points.
  // - Due to templated types we need to duplicate a bunch of code here
  if (detect_method == DETECT_IP_METHOD_INTEGRAL) {
    // Zack's custom detector
    int num_scales = stereo_settings().num_scales;
    if (num_scales <= 0) 
      num_scales = vw::ip::IntegralInterestPointDetector
        <vw::ip::OBALoGInterestOperator>::IP_DEFAULT_SCALES;
    else
      vw::vw_out() << "\t    Using " << num_scales 
        << " scales in OBALoG interest point detection.\n";

    vw::ip::IntegralAutoGainDetector detector(points_per_tile, num_scales);

    // This detector can't handle a mask so if there is nodata just set those pixels to zero.

    vw::vw_out() << "\t    Detecting IP\n";
    if (!has_nodata)
      ip = detect_interest_points(image.impl(), detector, points_per_tile);
    else
      ip = detect_interest_points(apply_mask(create_mask_less_or_equal(image.impl(),nodata)), detector, points_per_tile);
  } else {

    // Initialize the OpenCV detector.  Conveniently we can just pass in the type argument.
    // - If VW was not build with OpenCV, this call will just throw an exception.
    vw::ip::OpenCvIpDetectorType cv_method = vw::ip::OPENCV_IP_DETECTOR_TYPE_SIFT;
    if (detect_method == DETECT_IP_METHOD_ORB)
      cv_method = vw::ip::OPENCV_IP_DETECTOR_TYPE_ORB;

    // The opencv detector only works if the inputs are normalized, so do it here if it was not done before.
    // - If the images are already normalized most of the data will be in the 0-1 range.
    // - This normalize option will normalize PER-TILE which has different results than
    //   the whole-image normalization that ASP normally uses.
    bool opencv_normalize = stereo_settings().skip_image_normalization;
    if (stereo_settings().ip_normalize_tiles)
      opencv_normalize = true;
    if (opencv_normalize)
      vw::vw_out() << "\t    Using per-tile image normalization for IP detection...\n";

    bool build_opencv_descriptors = true;
    vw::ip::OpenCvInterestPointDetector detector(cv_method, opencv_normalize, build_opencv_descriptors, points_per_tile);

    // These detectors do accept a mask so use one if applicable.

    vw::vw_out() << "\t    Detecting IP\n";
    if (!has_nodata)
      ip = detect_interest_points(image.impl(), detector, points_per_tile);
    else
      ip = detect_interest_points(create_mask_less_or_equal(image.impl(),nodata), detector, points_per_tile);
  } // End OpenCV case

  sw1.stop();
  vw::vw_out(vw::DebugMessage,"asp") << "Detect interest points elapsed time: "
                                     << sw1.elapsed_seconds() << " s." << std::endl;
           
  if (!boost::math::isnan(nodata)) {
    vw::vw_out() << "\t    Removing IP near nodata with radius "
             << stereo_settings().ip_nodata_radius << std::endl;
    remove_ip_near_nodata(image.impl(), nodata, ip, stereo_settings().ip_nodata_radius);
  }

  // For the two OpenCV options we already built the descriptors, so
  // only do this for the integral method.
  if (detect_method == DETECT_IP_METHOD_INTEGRAL) {
    vw::Stopwatch sw2;
    sw2.start();
    vw::vw_out() << "\t    Building descriptors" << std::endl;
    vw::ip::SGradDescriptorGenerator descriptor;
    if (!has_nodata)
      describe_interest_points(image.impl(), descriptor, ip);
    else
      describe_interest_points(apply_mask(create_mask_less_or_equal(image.impl(),nodata)), descriptor, ip);
    sw2.stop();
    vw::vw_out(vw::DebugMessage,"asp") << "Building descriptors elapsed time: "
                                       << sw2.elapsed_seconds() << " s." << std::endl;
  }

  vw::vw_out() << "\t    Found interest points: " << ip.size() << std::endl;

  // If a file path was provided, record the IP to disk.
  if (file_path != "") {
    vw::vw_out() << "\t    Recording interest points to file: " << file_path << std::endl;
    vw::ip::write_binary_ip_file(file_path, ip);
  }

  return;
}

/// Detect IP in a pair of images and apply rudimentary filtering.
/// - Returns false if either image ended up with zero IP.
template <class Image1T, class Image2T>
bool detect_ip_pair(vw::ip::InterestPointList& ip1, 
		    vw::ip::InterestPointList& ip2,  
		    vw::ImageViewBase<Image1T> const& image1,
		    vw::ImageViewBase<Image2T> const& image2,
		    int ip_per_tile,
		    std::string const left_file_path,
		    std::string const right_file_path,
		    double nodata1, double nodata2) {
  vw::Stopwatch sw1;
  sw1.start();

  // Detect interest points in the two images
  vw::vw_out() << "\t    Looking for IP in left image.\n";
  detect_ip(ip1, image1.impl(), ip_per_tile, left_file_path, nodata1);
  vw::vw_out() << "\t    Looking for IP in right image.\n";
  detect_ip(ip2, image2.impl(), ip_per_tile, right_file_path, nodata2);
  
  if (stereo_settings().ip_debug_images) {
    vw::vw_out() << "\t    Writing detected IP debug images. " << std::endl;
    write_ip_debug_image("ASP_IP_detect_debug1.tif", image1, ip1, !boost::math::isnan(nodata1), nodata1);
    write_ip_debug_image("ASP_IP_detect_debug2.tif", image2, ip2, !boost::math::isnan(nodata2), nodata2);
  }
  
  side_ip_filtering(ip1, ip2, bounding_box(image1), bounding_box(image2));
  
  sw1.stop();
  vw::vw_out() << "Elapsed time in ip detection: " << sw1.elapsed_seconds() << " s.\n";

  return ((ip1.size() > 0) && (ip2.size() > 0));
}

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
template <class Image1T, class Image2T>
void detect_match_ip(std::vector<vw::ip::InterestPoint>& matched_ip1,
                     std::vector<vw::ip::InterestPoint>& matched_ip2,
                     vw::ImageViewBase<Image1T> const& image1,
                     vw::ImageViewBase<Image2T> const& image2,
                     int ip_per_tile, size_t number_of_jobs,
                     std::string const left_file_path ="",
                     std::string const right_file_path="",
                     double nodata1 = std::numeric_limits<double>::quiet_NaN(),
                     double nodata2 = std::numeric_limits<double>::quiet_NaN(),
                     std::string const& match_file = "") {

  // Detect interest points in the two images
  vw::ip::InterestPointList ip1, ip2;
  detect_ip_pair(ip1, ip2, image1, image2, ip_per_tile,
                 left_file_path, right_file_path, nodata1, nodata2);

  // Cast to float to make this compile
  vw::ImageViewRef<float> im1 = vw::pixel_cast<float>(image1);
  vw::ImageViewRef<float> im2 = vw::pixel_cast<float>(image2);

  // Match the ip and save the match file. No epipolar constraint
  // is used in this mode.
  match_ip_pair(ip1, ip2, im1, im2, number_of_jobs,
    matched_ip1, matched_ip2, match_file); // outputs

} // End function detect_match_ip

// A debug routine to save images with matches on top of them.
void write_match_image(std::string const& out_file_name,
                       vw::ImageViewRef<float> const& image1,
                       vw::ImageViewRef<float> const& image2,
                       std::vector<vw::ip::InterestPoint> const& matched_ip1,
                       std::vector<vw::ip::InterestPoint> const& matched_ip2);

} // End namespace asp

#endif//__ASP_CORE_INTEREST_POINT_MATCHING_H__
