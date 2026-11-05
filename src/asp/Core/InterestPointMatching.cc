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

#include <asp/Core/InterestPointMatching.h>
#include <asp/Core/EpipolarIpMatching.h>
#include <asp/Core/IpFilterUtils.h>
#include <asp/Core/StereoSettings.h>

#include <vw/InterestPoint/IntegralDetector.h>
#include <vw/InterestPoint/InterestPointUtils.h>
#include <vw/InterestPoint/OpenCvDetector.h>
#include <vw/InterestPoint/Matcher.h>
#include <vw/InterestPoint/MatcherIO.h>
#include <vw/Math/GaussianClustering.h>
#include <vw/Math/RANSAC.h>
#include <vw/Image/MaskViews.h>
#include <vw/Cartography/CameraBBox.h>
#include <vw/Cartography/GeoReference.h>
#include <vw/Cartography/GeoTransform.h>
#include <vw/Stereo/StereoModel.h>
#include <vw/Mosaic/ImageComposite.h>
#include <vw/Image/AlgorithmFunctions.h>
#include <vw/Core/Stopwatch.h>
#include <vw/Math/Geometry.h>
#include <vw/FileIO/FileUtils.h>

#include <boost/smart_ptr/shared_ptr.hpp>
#include <boost/foreach.hpp>

#include <algorithm>
#include <cmath>
#include <set>
#include <unordered_map>

using namespace vw;

namespace fs = boost::filesystem;
namespace asp {

// Detect interest points. Use a cached ip file if available and if the
// use_cached_ip flag is set. This is not meant to be used directly. Use
// ip_matching() or homography_ip_matching(). The caller is responsible for
// checking if this cached file is not older than the input image. 
void detect_ip(vw::InterestPointList& ip,
               vw::ImageViewRef<float> const& image,
               int ip_per_image, int ip_per_tile, 
               std::string const vwip_file, 
               double nodata, 
               bool use_cached_ip) {
  if (use_cached_ip && fs::exists(vwip_file)) {

    // Sanity check for when the vwip need to be redone     
    if ((asp::stereo_settings().left_image_crop_win  != BBox2i(0, 0, 0, 0) ||
         asp::stereo_settings().right_image_crop_win != BBox2i(0, 0, 0, 0)) ||
         !asp::stereo_settings().skip_rough_homography) 
      vw::vw_throw(vw::ArgumentErr() 
                   << "Cannot use cached ip with --left-image-crop-win, "
                   << "--right-image-crop-win, or --skip-rough-homography.\n");
    
    vw::vw_out() << "\t    Loading cached interest points file: " << vwip_file << "\n";
    ip = vw::read_binary_ip_file_list(vwip_file);
    return;
  }
  
  vw::Stopwatch sw1;
  sw1.start();

  ip.clear();

  // Automatically determine how many ip we need. Can be overridden below
  // either by --ip-per-image or --ip-per-tile (the latter takes priority).
  double tile_size = 1024.0;
  vw::BBox2i box = vw::bounding_box(image.impl());
  double number_tiles = (box.width() / tile_size) * (box.height() / tile_size);
  
  // Must have not have both ip per image and ip per tile set
  if (ip_per_image > 0 && ip_per_tile > 0)
    vw::vw_throw(vw::ArgumentErr()
      << "Cannot set both --ip-per-image and --ip-per-tile.\n");
  
  if (ip_per_tile == 0) {
    // Create ip_per_tile from ip_per_image

    if (ip_per_image == 0)
      ip_per_image = 5000; // default

    ip_per_tile = double(ip_per_image) / number_tiles;
    if (ip_per_tile > 5000) ip_per_tile = 5000;
    if (ip_per_tile < 50) ip_per_tile = 50;
  }

  vw::vw_out() << "\t    Using " << ip_per_tile << " interest points per tile (1024^2 px).\n";

  const bool has_nodata = !std::isnan(nodata);

  // Load the detection method from stereo_settings.
  // - This relies on a direct match in the enum integer value.
  DetectIpMethod detect_method
    = static_cast<DetectIpMethod>(stereo_settings().ip_detect_method);

  // Detect interest points.
  // - Due to templated types we need to duplicate a bunch of code here
  if (detect_method == DETECT_IP_METHOD_INTEGRAL) {
    // Zack's custom detector
    int num_scales = stereo_settings().num_scales;
    if (num_scales <= 0)
      num_scales = vw::IntegralAutoGainDetector::IP_DEFAULT_SCALES;
    else
      vw::vw_out() << "\t    Using " << num_scales
        << " scales in OBALoG interest point detection.\n";

    vw::IntegralAutoGainDetector detector(ip_per_tile, num_scales);

    // This detector can't handle a mask so if there is nodata just set those pixels to zero.

    vw::vw_out() << "\t    Detecting IP\n";
    if (!has_nodata)
      ip = detect_interest_points(pixel_cast<float>(image.impl()), detector, ip_per_tile);
    else
      ip = detect_interest_points(pixel_cast<float>(apply_mask(create_mask(image.impl(),nodata))), detector, ip_per_tile);
  } else {

    // Initialize the OpenCV detector.  Conveniently we can just pass in the type argument.
    // - If VW was not build with OpenCV, this call will just throw an exception.
    vw::OpenCvIpDetectorType cv_method = vw::OPENCV_IP_DETECTOR_TYPE_SIFT;
    if (detect_method == DETECT_IP_METHOD_ORB)
      cv_method = vw::OPENCV_IP_DETECTOR_TYPE_ORB;
    else if (detect_method == DETECT_IP_METHOD_AKAZE)
      cv_method = vw::OPENCV_IP_DETECTOR_TYPE_AKAZE;
    else if (detect_method == DETECT_IP_METHOD_BRISK)
      cv_method = vw::OPENCV_IP_DETECTOR_TYPE_BRISK;
    else if (detect_method == DETECT_IP_METHOD_KAZE)
      cv_method = vw::OPENCV_IP_DETECTOR_TYPE_KAZE;

    // The opencv detector only works if the inputs are normalized, so do it here if it was not done before.
    // - If the images are already normalized most of the data will be in the 0-1 range.
    // - This normalize option will normalize PER-TILE which has different results than
    //   the whole-image normalization that ASP normally uses.
    bool opencv_normalize = stereo_settings().skip_image_normalization;
    if (stereo_settings().ip_normalize_tiles)
      opencv_normalize = true;
    if (opencv_normalize)
      vw::vw_out() << "\t    Using per-tile image normalization for interest points detection...\n";

    bool build_opencv_descriptors = true;
    vw::OpenCvInterestPointDetector detector(cv_method, opencv_normalize, build_opencv_descriptors, ip_per_tile);

    // These detectors do accept a mask so use one if applicable.

    vw::vw_out() << "\t    Detecting IP\n";
    if (!has_nodata)
      ip = detect_interest_points(pixel_cast<float>(image.impl()), detector, ip_per_tile);
    else
      ip = detect_interest_points(pixel_cast<float>(create_mask(image.impl(),nodata)), detector, ip_per_tile);
  } // End OpenCV case

  sw1.stop();
  vw::vw_out(vw::DebugMessage,"asp") << "Detect interest points elapsed time: "
                                     << sw1.elapsed_seconds() << " s." << std::endl;
  
  if (!std::isnan(nodata)) {
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
    vw::SGradDescriptorGenerator descriptor;
    if (!has_nodata)
      describe_interest_points(image.impl(), descriptor, ip);
    else
      describe_interest_points(apply_mask(create_mask(image.impl(),nodata)), descriptor, ip);
    sw2.stop();
    vw::vw_out(vw::DebugMessage,"asp") << "Building descriptors elapsed time: "
                                       << sw2.elapsed_seconds() << " s." << std::endl;
  }

  vw::vw_out() << "\t    Number of interest points: " << ip.size() << std::endl;

  // If a file path was provided, record the IP (before matching) to disk.
  if (vwip_file != "") {
    vw::vw_out() << "\t    Writing interest points: " << vwip_file << "\n";
    vw::write_binary_ip_file(vwip_file, ip);
  }

  return;
}

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
                    bool use_cached_ip) {

  vw::Stopwatch sw1;
  sw1.start();

  // Detect interest points in the two images
  vw::vw_out() << "\t    Looking for interest points in left image.\n";
  detect_ip(ip1, vw::pixel_cast<float>(image1), 
            ip_per_image, ip_per_tile, left_vwip_file, 
            nodata1, use_cached_ip);
  vw::vw_out() << "\t    Looking for interest points in right image.\n";
  detect_ip(ip2, vw::pixel_cast<float>(image2), 
            ip_per_image, ip_per_tile, right_vwip_file,
            nodata2, use_cached_ip);

  if (stereo_settings().ip_debug_images) {
    vw::vw_out() << "\t    Writing detected IP debug images. " << std::endl;
    write_ip_debug_image("ASP_IP_detect_debug1.tif", image1, ip1, !std::isnan(nodata1), nodata1);
    write_ip_debug_image("ASP_IP_detect_debug2.tif", image2, ip2, !std::isnan(nodata2), nodata2);
  }

  side_ip_filtering(ip1, ip2, bounding_box(image1), bounding_box(image2));

  sw1.stop();
  vw::vw_out() << "Elapsed time in ip detection: " << sw1.elapsed_seconds() << " s.\n";

  return ((ip1.size() > 0) && (ip2.size() > 0));
}

/// Detect interest points and use a simple matching technique.
/// This is not meant to be used directly. Use ip_matching().
void detect_match_ip(std::vector<vw::InterestPoint>& matched_ip1,
                     std::vector<vw::InterestPoint>& matched_ip2,
                     vw::ImageViewRef<float> const& image1,
                     vw::ImageViewRef<float> const& image2,
                     int ip_per_tile, size_t number_of_jobs,
                     std::string const left_vwip_file,
                     std::string const right_vwip_file,
                     bool use_cached_ip,
                     double nodata1,
                     double nodata2,
                     std::string const& match_file,
                     vw::BBox2i const& bbox1,
                     vw::BBox2i const& bbox2,
                     bool filter_by_ground_loc,
                     vw::GeoReference const& georef1,
                     vw::GeoReference const& georef2) {

  // Detect interest points in the two images. This is done on the full image,
  // so the per-image vwip files are full-image and pair-independent (cacheable).
  vw::InterestPointList ip1, ip2;
  detect_ip_pair(ip1, ip2, image1, image2,
                 asp::stereo_settings().ip_per_image, ip_per_tile,
                 left_vwip_file, right_vwip_file, nodata1, nodata2,
                 use_cached_ip);

  // For mapprojected images, drop interest points with no geographically close
  // counterpart in the other image (--ip-match-radius), before correspondence.
  if (filter_by_ground_loc)
    filter_ip_by_ground_loc(ip1, ip2, georef1, georef2,
                            asp::stereo_settings().ip_match_radius);

  // If overlap boxes are given (e.g. the overlap region of mapprojected
  // images), confine matching to them by dropping interest points outside the
  // boxes before correspondence. Detection above was full-image, so this does
  // not affect the cached vwip and is correct for every pair the image is in.
  if (!bbox1.empty()) {
    vw::InterestPointList kept1;
    for (auto const& ip : ip1)
      if (bbox1.contains(vw::Vector2i(int(ip.x), int(ip.y))))
        kept1.push_back(ip);
    ip1 = kept1;
  }
  if (!bbox2.empty()) {
    vw::InterestPointList kept2;
    for (auto const& ip : ip2)
      if (bbox2.contains(vw::Vector2i(int(ip.x), int(ip.y))))
        kept2.push_back(ip);
    ip2 = kept2;
  }

  // Match the ip and save the match file. No epipolar constraint
  // is used in this mode.
  match_ip_no_datum(ip1, ip2, image1, image2, number_of_jobs,
                    matched_ip1, matched_ip2, match_file); // outputs

} // End function detect_match_ip

void check_homography_matrix(Matrix<double> const& H,
                             int left_size, int right_size, int num_inliers) {

  // Ensure H has at least 2 rows and 2 columns
  if (H.rows() < 2 || H.cols() < 2)
    vw::vw_throw(ArgumentErr() << "InterestPointMatching: Homography matrix is too small.\n");
    return;

  // Sanity checks. If these fail, most likely the two images are too different
  // for stereo to succeed.
  if (num_inliers < std::min(left_size, right_size)/2) {
    vw_out(WarningMessage) << "InterestPointMatching: The number of inliers is less "
                           << "than 1/2 of the number of points. The inputs may be invalid.\n";
  }

  double det = fabs(H(0, 0)*H(1, 1) - H(0, 1)*H(1, 0));
  if (det <= 0.1 || det >= 10.0) {
    vw_out(WarningMessage) << "InterestPointMatching: The determinant of the 2x2 submatrix "
                           << "of the homography matrix " << H << " is " << det << ". "
                           << "There could be a large scale discrepancy among the input images "
                           << "or the inputs may be an invalid stereo pair.\n";
  }
}

// Find a rough homography that maps right to left using the camera
// and datum information.  More precisely, take a set of pixels in
// the left camera image, project them onto the ground and back
// project them into the right camera image. Then to the
// reverse. This will help find a rough correspondence between
// the pixels in the two camera images.
Matrix<double>
rough_homography_fit(vw::CameraModel* cam1,
                     vw::CameraModel* cam2,
                     BBox2i const& box1, BBox2i const& box2,
                     vw::Datum const& datum) {

  vw::Stopwatch sw;
  sw.start();

  // Bounce several points off the datum and fit an affine.
  std::vector<Vector3> left_points, right_points;
  int num = 100;
  left_points.reserve(2*num*num);
  right_points.reserve(2*num*num);

  // Report progress, as this may be slow
  TerminalProgressCallback tpc("", "\tRough homography--> ");
  tpc.report_progress(0);
  double inc_amount = 1.0 / double(num) / double(num);

  for (int i = 0; i < num; i++) {
    for (int j = 0; j < num; j++) {
      try {
        Vector2 l(double(box1.width()  - 1) * i / (num-1.0),
                   double(box1.height() - 1) * j / (num-1.0));

        Vector3 intersection = vw::datum_intersection(datum, cam1, l);
        if (intersection == Vector3())
          continue;

        Vector2 r = cam2->point_to_pixel(intersection);

        if (box2.contains(r)) {
          left_points.push_back(Vector3(l[0],l[1],1));
          right_points.push_back(Vector3(r[0],r[1],1));
        }
      } catch (...) {}

      try {
        Vector2 r(double(box2.width()  - 1) * i / (num-1.0),
                  double(box2.height() - 1) * j / (num-1.0));

        Vector3 intersection = vw::datum_intersection(datum, cam2, r);
        if (intersection == Vector3())
          continue;

        Vector2 l = cam1->point_to_pixel(intersection);

        if (box1.contains(l)) {
          left_points.push_back(Vector3(l[0],l[1],1));
          right_points.push_back(Vector3(r[0],r[1],1));
        }
      } catch (...) {}
      tpc.report_incremental_progress(inc_amount);
    }
  }
  tpc.report_finished();

  // At most 5000 points should be enough for rough homography as it is a simple transform.
  vw::pick_pair_subset(left_points, right_points, 5000);

  vw_out() << "Projected " << left_points.size()
           << " rays for rough homography.\n";

  if (left_points.empty() || right_points.empty())
    vw_throw(ArgumentErr() << "InterestPointMatching: rough_homography_fit failed to generate points! Examine your images, or consider using the options --skip-rough-homography and --no-datum.\n");

  // The --ip-inlier-factor is 0.2 by default in stereo and bundle_adjust. 
  // This results in inlier_th = 0.1 * diagonal. May be too high for large images,
  // but the rough homography is well-behaved, since it uses cameras, and not
  // interest points, so it should be fine.
  double thresh_factor = stereo_settings().ip_inlier_factor;
  double inlier_th = norm_2(Vector2(box1.width(), box1.height())) * (0.5 * thresh_factor);
  inlier_th = std::max(inlier_th, 30.0); // don't allow it to be too small

  int min_inliers = left_points.size()/2;
  bool reduce_num_if_no_fit = true;

  // Using 1000 iterations is excessive here given that the these produced
  // interest points are not too noisy. This also takes a long time.
  int num_iter = stereo_settings().ip_num_ransac_iterations/10 + 50; // about 150

  vw_out() << "\t    Rough homography inlier threshold: " << inlier_th << "\n";

  // Use RANSAC to determine a good homography transform between the images
  vw_out() << "Estimating rough homography using RANSAC with "
    << num_iter << " iterations.\n";
  typedef vw::HomographyFittingFunctor hfit_func;
  vw::RandomSampleConsensus<hfit_func, vw::InterestPointErrorMetric>
    ransac(hfit_func(), vw::InterestPointErrorMetric(),
           num_iter, inlier_th, min_inliers, reduce_num_if_no_fit);

  Matrix<double> H = ransac(right_points, left_points);
  std::vector<size_t> indices = ransac.inlier_indices(H, right_points, left_points);
  check_homography_matrix(H, left_points.size(), right_points.size(), indices.size());
  vw_out() << "Number of inliers: " << indices.size() << ".\n";

  sw.stop();
  vw_out() << "Rough homography fit elapsed time: " << sw.elapsed_seconds() << " s.\n";

  return H;
}

// Filter ip with homography. This is a wrapper around RANSAC.
void filter_ip_homog(std::vector<vw::InterestPoint> const& ip1_in,
                     std::vector<vw::InterestPoint> const& ip2_in,
                     double inlier_th,
                     // Outputs
                     vw::Matrix<double> & H,
                     std::vector<size_t> & indices) {

  // Initialize the outputs. Ensure H is a valid matrix, in case ransac fails.
  H = vw::identity_matrix<3>();
  H.set_identity();
  indices.clear();

  vw::vw_out() << "\t    Filtering interest point matches using homography.\n";
  try {

    auto ransac_ip1 = iplist_to_vectorlist(ip1_in);
    auto ransac_ip2 = iplist_to_vectorlist(ip2_in);

    int min_inliers = ransac_ip1.size()/2;
    bool reduce_num_if_no_fit = true;

    vw_out() << "\t    Homography ip filter inlier threshold: " << inlier_th << "\n";
    vw_out() << "\t    RANSAC iterations:                     "
             << stereo_settings().ip_num_ransac_iterations << "\n";

    typedef vw::RandomSampleConsensus<vw::HomographyFittingFunctor,
      vw::InterestPointErrorMetric> RansacT;
    RansacT ransac(vw::HomographyFittingFunctor(),
                   vw::InterestPointErrorMetric(),
                   stereo_settings().ip_num_ransac_iterations,
                   inlier_th,
                   min_inliers, reduce_num_if_no_fit);
    H = ransac(ransac_ip2, ransac_ip1); // 2 then 1 is used here for legacy reasons
    indices = ransac.inlier_indices(H, ransac_ip2, ransac_ip1);
    vw::vw_out() << "Homography matrix:\n" << H << "\n";
    vw_out() << "Number of inliers: " << indices.size() << ".\n";
    check_homography_matrix(H, ransac_ip1.size(), ransac_ip2.size(), indices.size());
    
  } catch (const vw::RANSACErr& e) {
    vw_out() << "RANSAC failed: " << e.what() << "\n";
    return;
  }

}

// Compute the homography matrix (in right_matrix). The left matrix
// will have a translation corresponding to a crop.
Vector2i homography_rectification(bool adjust_left_image_size,
                                  bool tight_inlier_threshold,
                                  Vector2i const& left_size,
                                  Vector2i const& right_size,
                                  std::vector<vw::InterestPoint> const& left_ip,
                                  std::vector<vw::InterestPoint> const& right_ip,
                                  vw::Matrix<double>& left_matrix,
                                  vw::Matrix<double>& right_matrix) {

  // This number is 1/15 by default in stereo, and 0.2 in bundle_adjust. The
  // latter is more tolerant of outliers, but for stereo this can result in a
  // huge search range for disparity, which is not good.

  // TODO(oalexan1): This number can be too large for large images. Maybe need
  // to use the square root of the diagonal of the image size. Also, in some
  // related places a constant factor is used, like 30. Need to make that
  // uniform.
  double thresh_factor = stereo_settings().ip_inlier_factor;
  double inlier_th = norm_2(Vector2(left_size.x(),left_size.y())) * (0.5*thresh_factor);

  // When determining matches per tile, a loose threshold can result in inaccurate
  // determination of the homography. We'd rather have it accurate here and throw
  // away some good matches than the other way around. Later these matches will
  // be replaced with matches per tile once the homography transform is found.
  if (tight_inlier_threshold) {
    inlier_th = std::min(0.05*norm_2(Vector2(left_size.x(),left_size.y())), inlier_th);
    inlier_th = std::min(inlier_th, 200.0);
    inlier_th = std::max(inlier_th, 50.0);
  }

  Matrix<double> H = vw::identity_matrix<3>();
  std::vector<size_t> indices;
  filter_ip_homog(left_ip, right_ip, inlier_th,
                  H, indices); // outputs

  // TODO(oalexan1): A percentile-based filter may help here, after finding
  // the inliers. It should be based on estimating H * right - left.
  // That because the inlier threshold is kind of arbitrary.
  // If outliers are found, need to recompute H based on inliers, which is quick.
  check_homography_matrix(H, left_ip.size(), right_ip.size(), indices.size());

  // Set right to a homography that has been refined just to our inliers.
  // May be adjusted with a translation below.
  // TODO(oalexan1): It is not clear if adjusting a homography transform's
  // translation coefficients is the right way of applying a translation to it,
  // because it has a denominator too.
  left_matrix  = vw::identity_matrix<3>();
  right_matrix = H;

  // Work out the ideal render size
  BBox2i output_bbox, right_bbox;
  output_bbox.grow(Vector2i(0, 0));
  output_bbox.grow(Vector2i(left_size.x(), 0));
  output_bbox.grow(Vector2i(0, left_size.y()));
  output_bbox.grow(left_size);

  if (adjust_left_image_size) {
    // Crop the left and right images to the shared region. This is
    // done for efficiency.  It may not be always desirable though,
    // as in this case we lose the one-to-one correspondence between
    // original input left image pixels and output disparity/point
    // cloud pixels.
    Vector3 temp = right_matrix*Vector3(0,0,1);
    temp /= temp.z();
    right_bbox.grow(subvector(temp,0,2));
    temp = right_matrix*Vector3(right_size.x(),0,1);
    temp /= temp.z();
    right_bbox.grow(subvector(temp,0,2));
    temp = right_matrix*Vector3(0,right_size.y(),1);
    temp /= temp.z();
    right_bbox.grow(subvector(temp,0,2));
    temp = right_matrix*Vector3(right_size.x(),right_size.y(),1);
    temp /= temp.z();
    right_bbox.grow(subvector(temp,0,2));

    output_bbox.crop(right_bbox);

    //  Move the ideal render size to be aligned up with origin
    left_matrix (0,2) -= output_bbox.min().x();
    right_matrix(0,2) -= output_bbox.min().x();
    left_matrix (1,2) -= output_bbox.min().y();
    right_matrix(1,2) -= output_bbox.min().y();
  }

  return Vector2i(output_bbox.width(), output_bbox.height());
}

// Create interest points from valid D_sub values and make them full scale
// (while still having potentially a global alignment applied to them).
void aligned_ip_from_D_sub(vw::ImageViewRef<vw::PixelMask<vw::Vector2f>> const & sub_disp,
                           vw::Vector2                                   const & upsample_scale,
                           std::vector<vw::InterestPoint>                  & left_ip,
                           std::vector<vw::InterestPoint>                  & right_ip) {

  left_ip.clear();
  right_ip.clear();

  for (int col = 0; col < sub_disp.cols(); col++) {
    for (int row = 0; row < sub_disp.rows(); row++) {
      vw::PixelMask<vw::Vector2f> disp = sub_disp(col, row);

      if (!is_valid(disp)) continue;

      Vector2 left_pix(col, row);
      Vector2 right_pix = left_pix + disp.child();

      left_pix  = elem_prod(left_pix, upsample_scale);
      right_pix = elem_prod(right_pix, upsample_scale);

      vw::InterestPoint left, right;
      left.x  = left_pix.x();   left.y = left_pix.y();
      right.x = right_pix.x(); right.y = right_pix.y();

      left_ip.push_back(left);
      right_ip.push_back(right);
    }
  }
}

// Do IP matching, return, the best translation+scale fitting functor.
vw::Matrix<double> translation_ip_matching(vw::ImageView<vw::PixelGray<float>> const& image1,
                                           vw::ImageView<vw::PixelGray<float>> const& image2,
                                           int ip_per_tile,
                                           std::string const  left_vwip_file,
                                           std::string const  right_vwip_file,
                                           double nodata1, double nodata2) {

  using namespace vw;

  std::vector<vw::InterestPoint> matched_ip1, matched_ip2;
  size_t number_of_jobs = 1;
  bool use_cached_ip = false;
  detect_match_ip(matched_ip1, matched_ip2,
                  vw::pixel_cast<float>(image1),
                  vw::pixel_cast<float>(image2),
                  ip_per_tile, number_of_jobs,
                  left_vwip_file, right_vwip_file, 
                  use_cached_ip,
                  nodata1, nodata2);

  std::vector<Vector3> ransac_ip1 = iplist_to_vectorlist(matched_ip1);
  std::vector<Vector3> ransac_ip2 = iplist_to_vectorlist(matched_ip2);
  vw_out(DebugMessage,"asp") << "\t--> Removed "
                             << matched_ip1.size() - ransac_ip1.size()
                             << " duplicate matches.\n";

  Matrix<double> T;
  std::vector<size_t> indices;
  try {

    vw::RandomSampleConsensus<vw::TranslationScaleFittingFunctor, vw::InterestPointErrorMetric>
      ransac(vw::TranslationScaleFittingFunctor(),
             vw::InterestPointErrorMetric(),
             stereo_settings().ip_num_ransac_iterations,
             10, ransac_ip1.size()/2, true);
    T = ransac(ransac_ip2, ransac_ip1);
    indices = ransac.inlier_indices(T, ransac_ip2, ransac_ip1);
  } catch (...) {
    vw_out(WarningMessage,"console") << "Automatic Alignment Failed! Proceed with caution...\n";
    T = vw::identity_matrix<3>();
  }

  { // Keeping only inliers
    std::vector<vw::InterestPoint> inlier_ip1, inlier_ip2;
    for (size_t i = 0; i < indices.size(); i++) {
      inlier_ip1.push_back(matched_ip1[indices[i]]);
      inlier_ip2.push_back(matched_ip2[indices[i]]);
    }
    matched_ip1 = inlier_ip1;
    matched_ip2 = inlier_ip2;
  }

  return T;
}

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
                            std::string const  left_vwip_file,
                            std::string const  right_vwip_file,
                            bool use_cached_ip,
                            double nodata1, double nodata2,
                            vw::BBox2i const& bbox1,
                            vw::BBox2i const& bbox2,
                            bool filter_by_ground_loc,
                            vw::GeoReference const& georef1,
                            vw::GeoReference const& georef2) {

  vw_out() << "\t--> Matching interest points using homography.\n";

  // Detect interest points on the FULL image (so the per-image vwip is cacheable
  // and pair-independent) and, if overlap boxes are given (mapprojected images),
  // confine the matching to those boxes by dropping interest points outside them
  // before correspondence. This is done inside detect_match_ip. The earlier
  // approach cropped the image before detection, which made the cached vwip
  // pair-dependent and caused most pairs in a multi-image run to find no matches.
  std::vector<vw::InterestPoint> matched_ip1, matched_ip2;
  detect_match_ip(matched_ip1, matched_ip2,
                  image1, image2,
                  ip_per_tile, number_of_jobs,
                  left_vwip_file, right_vwip_file,
                  use_cached_ip,
                  nodata1, nodata2,
                  "", // match_file: written below after homography filtering
                  bbox1, bbox2,
                  filter_by_ground_loc, georef1, georef2);
  if (matched_ip1.size() == 0 || matched_ip2.size() == 0)
    return false;

  // Filter the interest points using a homography constraint
  Stopwatch sw;
  sw.start();
  Matrix<double> H = vw::identity_matrix<3>();
  std::vector<size_t> indices;
  filter_ip_homog(matched_ip1, matched_ip2, inlier_threshold,
                  H, indices); // outputs
  sw.stop();
  vw_out() << "Filtering time: " << sw.elapsed_seconds() << " seconds.\n";
  int num_left = indices.size();
  if (num_left == 0)
    return false;

  // Form the filtered interest points
  std::vector<vw::InterestPoint> final_ip1(num_left), final_ip2(num_left);
  for (size_t i = 0; i < num_left; i++) {
    final_ip1[i] = matched_ip1[indices[i]];
    final_ip2[i] = matched_ip2[indices[i]];
  }
    
  if (stereo_settings().ip_debug_images) {
    vw_out() << "\t    Writing post-homography IP match debug image.\n";
    write_match_image("InterestPointMatching__ip_matching_debug2.tif",
                      image1, image2, final_ip1, final_ip2);
  }

  vw_out() << "\t    * Writing match file: " << match_filename << "\n";
  bool matches_as_txt = stereo_settings().matches_as_txt;
  vw::write_match_file(match_filename, final_ip1, final_ip2, matches_as_txt);

  return true;
}

// Detect interest points. Return also the rough homography that aligns the right image
// to the left.
bool detect_ip_aligned_pair(vw::CameraModel* cam1,
                            vw::CameraModel* cam2,
                            vw::ImageViewRef<float> const& image1,
                            vw::ImageViewRef<float> const& image2,
                            int ip_per_per_image,
                            int ip_per_tile,
                            vw::Datum const& datum,
                            std::string const left_vwip_file,
                            double nodata1, double nodata2,
                            // Outputs
                            vw::InterestPointList& ip1,
                            vw::InterestPointList& ip2,
                            vw::Matrix<double> & rough_homography) {

  BBox2i box1 = bounding_box(image1), box2 = bounding_box(image2);

  try {
    // Homography is defined in the original camera coordinates
    rough_homography = rough_homography_fit(cam1, cam2, box1, box2, datum);
  } catch(...) {
    vw_out() << "Rough homography fit failed, trying with identity transform. " << std::endl;
    rough_homography.set_identity(3);
  }

  // Remove the main translation and solve for BBox that fits the
  // image. If we used the translation from the solved homography with
  // poorly position cameras, the right image might be moved out of frame.
  rough_homography(0,2) = rough_homography(1,2) = 0;
  vw_out() << "Aligning right to left for interest points capture using rough homography: "
	   << rough_homography << std::endl;

  // Find the image of the right image bounding box after applying the rough homography
  HomographyTransform rough_trans(rough_homography);
  BBox2i trans_box2 = rough_trans.forward_bbox(box2);

  // Check to see if this rough homography works
  VW_ASSERT(box1.intersects(trans_box2),
     LogicErr() << "The rough homography alignment based on datum and camera "
        << "geometry shows that input images do not overlap at all. Unable to proceed. "
        << "Examine your images, or consider using the option --skip-rough-homography.\n");

  // Adjust the homography transform so that the right image bounding box has
  // (0, 0) as the minimum. We will count on this below and outside of this
  // function. See Transform.h for definition of homography transform.
  for (int i = 0; i < 3; i++) {
    for (int j = 0; j < 2; j++) {
      rough_homography(j, i) -= trans_box2.min()[j] * rough_homography(2, i);
    }
  }

  // Update these after the adjustment to the homography transform
  rough_trans = HomographyTransform(rough_homography);
  trans_box2 = rough_trans.forward_bbox(box2);

  // Detect interest points for the left and (transformed) right image.
  // - It is important that we use NearestPixelInterpolation in the
  //   next step. Using anything else will interpolate nodata values
  //   and stop them from being masked out.
  // TODO(oalexan1): Would it be better to pass masked images and use interpolation?
  // TODO(oalexan1): Likely bilinear interpolation is better for the IP detection.
  auto ext = ValueEdgeExtension<float>(std::isnan(nodata2) ? 0 : nodata2);
  std::string right_vwip_file = ""; // Don't record IP from transformed images
  bool use_cached_ip = false;
  if (!detect_ip_pair(ip1, ip2, image1,
                      crop(transform(image2, rough_trans, ext,
                                     NearestPixelInterpolation()), trans_box2),
                      ip_per_per_image, ip_per_tile, left_vwip_file, right_vwip_file,
                      nodata1, nodata2, use_cached_ip)) {
    vw_out() << "Unable to detect interest points." << std::endl;
    return false;
  }

  // Undo the rough homography transform for the interest points
  vw::InterestPointList::iterator ip_it;
  for (ip_it = ip2.begin(); ip_it != ip2.end(); ip_it++) {
    Vector2 pt(ip_it->x, ip_it->y);
    pt = rough_trans.reverse(pt);
    ip_it->ix = ip_it->x = pt.x();
    ip_it->iy = ip_it->y = pt.y();
  }

  return true;
} // End function detect_ip_aligned_pair


// A debug routine to save images with matches on top of them.
void write_match_image(std::string const& out_file_name,
                       vw::ImageViewRef<float> const& image1,
                       vw::ImageViewRef<float> const& image2,
                       std::vector<vw::InterestPoint> const& matched_ip1,
                       std::vector<vw::InterestPoint> const& matched_ip2) {

  // Skip image pairs with no matches.
  if (matched_ip1.empty())
    return;

  // Work out the scaling to produce the subsampled images. These
  // values are choosen just allow a reasonable rendering time.
  float sub_scale  = sqrt(1500.0 * 1500.0 / float(image1.impl().cols() * image1.impl().rows()));
  sub_scale += sqrt(1500.0 * 1500.0 / float(image2.impl().cols() * image2.impl().rows()));
  sub_scale /= 2;
  //if (sub_scale > 1)
  sub_scale = 1;

  vw::ImageComposite<vw::PixelRGB<vw::uint8>> composite;
  //composite.insert(vw::pixel_cast_rescale<vw::PixelRGB<vw::uint8> >(resample(normalize(image1), sub_scale)), 0, 0);
  //composite.insert(vw::pixel_cast_rescale<vw::PixelRGB<vw::uint8> >(resample(normalize(image2), sub_scale)), vw::int32(image1.impl().cols() * sub_scale), 0);
  composite.insert(vw::pixel_cast_rescale<vw::PixelRGB<vw::uint8> >(image1), 0, 0);
  composite.insert(vw::pixel_cast_rescale<vw::PixelRGB<vw::uint8> >(image2), vw::int32(image1.impl().cols() * sub_scale), 0);
  composite.set_draft_mode(true);
  composite.prepare();

  // Rasterize the composite so that we can draw on it.
  vw::ImageView<vw::PixelRGB<vw::uint8>> comp = composite;

  // Draw a red line between matching interest points
  for (size_t k = 0; k < matched_ip1.size(); k++) {
    vw::Vector2f start(matched_ip1[k].x, matched_ip1[k].y);
    vw::Vector2f end(matched_ip2[k].x+image1.impl().cols(), matched_ip2[k].y);
    start *= sub_scale;
    end   *= sub_scale;
    float inc_amt = 1/norm_2(end-start);
    for (float r=0; r<1.0; r+=inc_amt) {
      int i = (int)(0.5 + start.x() + r*(end.x()-start.x()));
      int j = (int)(0.5 + start.y() + r*(end.y()-start.y()));
      if (i >=0 && j >=0 && i < comp.cols() && j < comp.rows())
        comp(i,j) = vw::PixelRGB<vw::uint8>(255, 0, 0);
    }
  }

  vw::vw_out() << "Writing: " << out_file_name << std::endl;
  boost::shared_ptr<vw::DiskImageResource> rsrc(vw::DiskImageResource::create(out_file_name, comp.format()));
  vw::block_write_image(*rsrc, comp, vw::TerminalProgressCallback("", ""));
}

// See the .h file for documentation.
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
                         std::string const left_file_path,
                         std::string const right_file_path,
                         double nodata1,
                         double nodata2) {

  bool matches_as_txt = stereo_settings().matches_as_txt;

  if (use_rough_homography)
    vw_out() << "\t    Using rough homography.\n";
  else
    vw_out() << "\t    Skipping rough homography.\n";

  // This call aligns the right image to the left image then detects IPs in the two images.
  // Undo the alignment transform in the ip before returning, and return the transform.
  vw::InterestPointList ip1, ip2;
  vw::Matrix<double> rough_homography = vw::identity_matrix<3>();
  bool use_cached_ip = false;
  if (use_rough_homography) 
    detect_ip_aligned_pair(cam1, cam2, image1, image2,
                           asp::stereo_settings().ip_per_image, ip_per_tile, 
                           datum, left_file_path, nodata1, nodata2,
                           ip1, ip2, rough_homography); // Outputs
  else
     detect_ip_pair(ip1, ip2, image1, image2,
                    asp::stereo_settings().ip_per_image, ip_per_tile, 
                    left_file_path, right_file_path,
                    nodata1, nodata2, use_cached_ip);

  // Match the detected IPs which are in the original image coordinates.
  std::vector<vw::InterestPoint> matched_ip1, matched_ip2;
  bool match_per_tile = false;
  vw::Matrix<double> align_matrix = vw::identity_matrix<3>();
  vw::Stopwatch sw2;
  sw2.start();
  bool inlier = epipolar_ip_matching(single_threaded_camera, ip1, ip2, cam1, cam2, 
                                     image1, image2,  datum, number_of_jobs, 
                                     epipolar_threshold, uniqueness_threshold, 
                                     nodata1, nodata2,
                                     match_per_tile, align_matrix, 
                                     matched_ip1, matched_ip2); // Outputs
  sw2.stop();
  vw_out() << "Elapsed time in ip matching: " << sw2.elapsed_seconds() << " s.\n";

  if (!inlier)
    return false;

  // Use the interest points that we found to compute an aligning homography
  // transform for the two images. This is always needed when finding matches
  // per tile or with rough homography.
  bool adjust_left_image_size = false, tight_inlier_threshold = false;
  Matrix<double> matrix1 = vw::identity_matrix<3>();
  Matrix<double> matrix2 = vw::identity_matrix<3>();
  if (use_rough_homography || asp::stereo_settings().matches_per_tile > 0) {
    vw_out() << "\t    Computing homography transform.\n";
    vw::Stopwatch sw3;
    sw3.start();
    bool tight_inlier_threshold = (asp::stereo_settings().matches_per_tile > 0);
    homography_rectification(adjust_left_image_size, tight_inlier_threshold,
			   image1.get_size(), image2.get_size(),
			   matched_ip1, matched_ip2, matrix1, matrix2);
    sw3.stop();
    vw_out() << "Elapsed time in homography computation: " << sw3.elapsed_seconds() << " s.\n";
  }
  if (use_rough_homography && 
      sum(abs(submatrix(rough_homography,0,0,2,2) - submatrix(matrix2,0,0,2,2))) > 4) {
    vw_out() << "Homography transform has largely different scale and skew "
             << "compared with the rough homography. Homography transform is " 
	           << matrix2 << ". Examine your images, or consider using the option "
             << "--skip-rough-homography.\n";
    //return false;
  }

  // Second pass if doing matches per tile
  if (asp::stereo_settings().matches_per_tile > 0) {
    // We will use the homography matrix from left to right ip, stored in matrix2.
    // It is expected that matrix1 is the identity matrix.
    if (matrix1 != vw::identity_matrix<3>())
      vw::vw_throw(ArgumentErr() << "Expecting identity matrix for left image alignment.\n");

    match_per_tile = true;
    align_matrix = matrix2; // use the homography matrix
    vw::Stopwatch sw4;
    sw4.start();
    bool inlier =
      epipolar_ip_matching(single_threaded_camera,
        ip1, ip2, cam1, cam2, image1, image2, 
        datum, number_of_jobs, epipolar_threshold, uniqueness_threshold,
        nodata1, nodata2,
        match_per_tile, align_matrix,
        matched_ip1, matched_ip2); // Outputs
    sw4.stop();
    vw_out() << "Elapsed time in ip matching when using tiles: " 
      << sw4.elapsed_seconds() << " s.\n";

    if (!inlier)
      return false;
  }

  // Write the matches to disk
  vw_out() << "\t    * Writing match file: " << match_filename << "\n";
  vw::write_match_file(match_filename, matched_ip1, matched_ip2, matches_as_txt);
  return inlier;
}

// Match the ip and save the match file. No datum or epipolar constraint
// is used in this mode.
void match_ip_no_datum(vw::InterestPointList const& ip1,
                       vw::InterestPointList const& ip2,
                       vw::ImageViewRef<float> const& image1,
                       vw::ImageViewRef<float> const& image2,
                       size_t number_of_jobs,
                       // Outputs
                       std::vector<vw::InterestPoint>& matched_ip1,
                       std::vector<vw::InterestPoint>& matched_ip2,
                       std::string const& match_file) {

  matched_ip1.clear(); 
  matched_ip2.clear();

  std::vector<vw::InterestPoint> ip1_copy, ip2_copy;
  ip1_copy.reserve(ip1.size());
  ip2_copy.reserve(ip2.size());
  std::copy(ip1.begin(), ip1.end(), std::back_inserter(ip1_copy));
  std::copy(ip2.begin(), ip2.end(), std::back_inserter(ip2_copy));

  DetectIpMethod detect_method 
    = static_cast<DetectIpMethod>(stereo_settings().ip_detect_method);

  // Best point must be closer than the next best point
  double uniqueness_threshold = stereo_settings().ip_uniqueness_thresh;
  vw_out() << "\t--> Uniqueness threshold: " << uniqueness_threshold << "\n";

  bool quiet = false;
  vw::Stopwatch sw1;
  sw1.start();
  match_ip_no_datum(ip1_copy, ip2_copy, detect_method, uniqueness_threshold, quiet,
                    matched_ip1, matched_ip2); // outputs
  sw1.stop();
  vw_out() << "Elapsed time in ip matching: " << sw1.elapsed_seconds() << " s.\n";

  // Remove ip duplicates. Complexity is O(n log n).
  vw::remove_duplicates(matched_ip1, matched_ip2);

  if (asp::stereo_settings().matches_per_tile != 0) {

    // Use the interest points that we found to compute an aligning
    // homography transform for the two images.
    vw_out() << "\t    Computing homography transform.\n";
    bool adjust_left_image_size = false;
    bool tight_inlier_threshold = true; // this is necessary for large images
    Matrix<double> matrix1, matrix2;
    vw::Stopwatch sw2;
    sw2.start();
    homography_rectification(adjust_left_image_size, tight_inlier_threshold,
                             image1.get_size(), image2.get_size(),
                             matched_ip1, matched_ip2, matrix1, matrix2);
    sw2.stop();
    vw_out() << "Elapsed time in homography computation: " << sw2.elapsed_seconds() << " s.\n";

    // We will use the homography matrix from left to right ip, stored in matrix2.
    // It is expected that matrix1 is the identity matrix.
    if (matrix1 != vw::identity_matrix<3>())
      vw::vw_throw(ArgumentErr() << "Expecting identity matrix for left image alignment.\n");
    vw::Matrix<double> align_matrix = matrix2; 

    // Initialize some quantities that won't be used but are part of the API.
    bool have_datum = false, single_threaded_camera = false;
    double epipolar_threshold = -1, nodata1 = -1, nodata2 = -1;
    vw::CameraModel *cam1 = NULL, *cam2 = NULL;
    vw::Datum datum;
    vw::Stopwatch sw3;
    sw3.start();
    vw_out() << "\t    Performing matching per tile.\n";
    matches_per_tile(have_datum, single_threaded_camera, detect_method,
                     ip1_copy, ip2_copy, cam1, cam2, datum,
                     number_of_jobs, epipolar_threshold, uniqueness_threshold,
                     align_matrix, 
                     matched_ip1, matched_ip2); // outputs
    sw3.stop();
    vw_out() << "Elapsed time in ip matching when using tiles: " 
      << sw3.elapsed_seconds() << " s.\n";
  }

  vw_out() << "\t    Matched points: " << matched_ip1.size() << std::endl;

  if (stereo_settings().ip_debug_images) {
    vw_out() << "\t    Writing IP initial match debug image.\n";
    // Cast to float to make this compile
    ImageViewRef<float> im1 = vw::pixel_cast<float>(image1);
    ImageViewRef<float> im2 = vw::pixel_cast<float>(image2);
    write_match_image("InterestPointMatching__ip_matching_debug.tif",
                      im1, im2, matched_ip1, matched_ip2);
  }

  // Save ip
  if (match_file != "") {
    // Create the output directory
    vw::create_out_dir(match_file);
    vw_out() << "Writing: " << match_file << std::endl;
    bool matches_as_txt = stereo_settings().matches_as_txt;
    vw::write_match_file(match_file, matched_ip1, matched_ip2, matches_as_txt);
  }

  return;
}

} // end namespace asp
