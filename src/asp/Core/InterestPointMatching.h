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

// TODO(oalexan1): There's too much template-heavy logic here which
// slows down compilation. Move most of these to .cc but check if the
// performance suffers if switching to ImageViewRef instead of templates.

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

#include <asp/Core/StereoSettings.h>
#include <boost/foreach.hpp>
#include <boost/math/special_functions/fpclassify.hpp>

// // TODO(oalexan1): This function should live somewhere else.It was
// pulled from vw->tools->ipmatch.cc. Move it InterestPointUtils.cc
// in VisionWorkbench, but after removing templates from most functions
// in this file.
#include <vw/Mosaic/ImageComposite.h>
template <typename Image1T, typename Image2T>
void write_match_image(std::string const& out_file_name,
                       vw::ImageViewBase<Image1T> const& image1,
                       vw::ImageViewBase<Image2T> const& image2,
                       std::vector<vw::ip::InterestPoint> const& matched_ip1,
                       std::vector<vw::ip::InterestPoint> const& matched_ip2) {
  vw::vw_out() << "\t    Starting write_match_image " << std::endl;

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

  vw::mosaic::ImageComposite<vw::PixelRGB<vw::uint8> > composite;
  //composite.insert(vw::pixel_cast_rescale<vw::PixelRGB<vw::uint8> >(resample(normalize(image1), sub_scale)), 0, 0);
  //composite.insert(vw::pixel_cast_rescale<vw::PixelRGB<vw::uint8> >(resample(normalize(image2), sub_scale)), vw::int32(image1.impl().cols() * sub_scale), 0);
  composite.insert(vw::pixel_cast_rescale<vw::PixelRGB<vw::uint8> >(image1), 0, 0);
  composite.insert(vw::pixel_cast_rescale<vw::PixelRGB<vw::uint8> >(image2), vw::int32(image1.impl().cols() * sub_scale), 0);
  composite.set_draft_mode(true);
  composite.prepare();

  vw::vw_out() << "\t    rasterize composite " << std::endl;

  // Rasterize the composite so that we can draw on it.
  vw::ImageView<vw::PixelRGB<vw::uint8> > comp = composite;

  vw::vw_out() << "\t    Draw lines "<<  std::endl;

  // Draw a red line between matching interest points
  for (size_t k = 0; k < matched_ip1.size(); ++k) {
    vw::Vector2f start(matched_ip1[k].x, matched_ip1[k].y);
    vw::Vector2f end(matched_ip2[k].x+image1.impl().cols(), matched_ip2[k].y);
    start *= sub_scale;
    end   *= sub_scale;
    float inc_amt = 1/norm_2(end-start);
    for (float r=0; r<1.0; r+=inc_amt){
      int i = (int)(0.5 + start.x() + r*(end.x()-start.x()));
      int j = (int)(0.5 + start.y() + r*(end.y()-start.y()));
      if (i >=0 && j >=0 && i < comp.cols() && j < comp.rows())
        comp(i,j) = vw::PixelRGB<vw::uint8>(255, 0, 0);
    }
  }

  vw::vw_out() << "\t    Write to disk "  <<std::endl;
  boost::scoped_ptr<vw::DiskImageResource> rsrc(vw::DiskImageResource::create(out_file_name, comp.format()));
  block_write_image(*rsrc, comp, vw::TerminalProgressCallback("tools.ipmatch", "Writing Debug:"));
}

namespace asp {


  //-------------------------------------------------------------------------------------------
  // These are the top level IP matching functions that detect IP, match them, and then write a match file to disk.

  /// Detect IP, use a simple matching technique, then apply a homography constraint.
///
/// This applies only the homography constraint. Not the best...
template <class Image1T, class Image2T>
bool homography_ip_matching(vw::ImageViewBase<Image1T> const& image1,
			    vw::ImageViewBase<Image2T> const& image2,
			    int ip_per_tile,
			    std::string const& output_name,
			    int inlier_threshold=10,
			    std::string const left_file_path ="",
			    std::string const right_file_path="",
			    double nodata1 = std::numeric_limits<double>::quiet_NaN(),
			    double nodata2 = std::numeric_limits<double>::quiet_NaN());  
  
  /// Smart IP matching that uses clustering on triangulation and
/// datum information to determine inliers.
///
/// Left and Right TX define transforms that have been performed on
/// the images that that camera data doesn't know about. (ie scaling).
template <class Image1T, class Image2T>
bool ip_matching_no_align(bool single_threaded_camera,
			  vw::camera::CameraModel* cam1,
			  vw::camera::CameraModel* cam2,
			  vw::ImageViewBase<Image1T> const& image1,
			  vw::ImageViewBase<Image2T> const& image2,
			  int ip_per_tile,
			  vw::cartography::Datum const& datum,
			  double epipolar_threshold,
			  double uniqueness_threshold,
			  std::string const& output_name,
			  std::string const  left_file_path ="",
			  std::string const  right_file_path="",
			  double nodata1 = std::numeric_limits<double>::quiet_NaN(),
			  double nodata2 = std::numeric_limits<double>::quiet_NaN());
  
  // Calls ip matching above but with an additional step where we
  // apply a homography to make right image like left image. This is
  // useful so that both images have similar scale and similar affine qualities.
  // - Only the left image will use an IP file since the right image is modified.
  template <class Image1T, class Image2T>
  bool ip_matching_w_alignment(bool single_threaded_camera,
                               vw::camera::CameraModel* cam1,
                               vw::camera::CameraModel* cam2,
                               vw::ImageViewBase<Image1T> const& image1,
                               vw::ImageViewBase<Image2T> const& image2,
                               int ip_per_tile,
                               vw::cartography::Datum const& datum,
                               std::string const& output_name,
                               double epipolar_threshold,
                               double uniqueness_threshold,
                               std::string const left_file_path ="",
                               double nodata1 = std::numeric_limits<double>::quiet_NaN(),
                               double nodata2 = std::numeric_limits<double>::quiet_NaN());
  
  // Do IP matching, return, the best translation+scale fitting functor.
  vw::Matrix<double> translation_ip_matching(vw::ImageView<vw::PixelGray<float>> const& image1,
					     vw::ImageView<vw::PixelGray<float>> const& image2,
					     int ip_per_tile,
					     std::string const  left_file_path ="",
					     std::string const  right_file_path="",
					     double nodata1 =
                                             std::numeric_limits<double>::quiet_NaN(),
					     double nodata2 =
                                             std::numeric_limits<double>::quiet_NaN());

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
			   vw::Vector2i const& left_size,
			   vw::Vector2i const& right_size,
			   std::vector<vw::ip::InterestPoint> const& left_ip,
			   std::vector<vw::ip::InterestPoint> const& right_ip,
			   vw::Matrix<double>& left_matrix,
			   vw::Matrix<double>& right_matrix);

  // Create interest points from valid D_sub values and make them full scale
  // (while still having potentially a global alignment applied to them).
  void aligned_ip_from_D_sub(vw::ImageViewRef<vw::PixelMask<vw::Vector2f>> const & sub_disp,
                             vw::Vector2                                   const & upsample_scale,
                             std::vector<vw::ip::InterestPoint>                  & left_ip, 
                             std::vector<vw::ip::InterestPoint>                  & right_ip);
  
  /// The unwarped disparity file name.
  /// We put it here because this file knows about vw::ip. That is why it should not
  /// in Common.h which should not know so much.
  std::string unwarped_disp_file(std::string const& prefix, std::string const& left_image,
                                 std::string const& right_image);

  //-------------------------------------------------------------------------------------------
  // Lower level IP detection functions

  /// Detect interest points
  ///
  /// This is not meant to be used directly. Use ip_matching() or
  /// homography_ip_matching().
  template <class Image1T>
  void detect_ip(vw::ip::InterestPointList& ip1, 
		 vw::ImageViewBase<Image1T> const& image,
		 int ip_per_tile,
		 std::string const file_path="",
		 double nodata = std::numeric_limits<double>::quiet_NaN());

  /// Detect IP in a pair of images and apply rudimentary filtering.
  /// - Returns false if either image ended up with zero IP.
  template <class Image1T, class Image2T>
  bool detect_ip_pair(vw::ip::InterestPointList& ip1, 
		      vw::ip::InterestPointList& ip2,  
		      vw::ImageViewBase<Image1T> const& image1,
		      vw::ImageViewBase<Image2T> const& image2,
		      int ip_per_tile,
		      std::string const left_file_path ="",
		      std::string const right_file_path="",
		      double nodata1 = std::numeric_limits<double>::quiet_NaN(),
		      double nodata2 = std::numeric_limits<double>::quiet_NaN());


  /// Find IP's for a pair as above but with an additional step where we
  /// apply a homography to make right image like left image. This is
  /// useful so that both images have similar scale and similar affine qualities.
  template <class Image1T, class Image2T>
  bool detect_ip_aligned_pair(vw::camera::CameraModel* cam1,
			      vw::camera::CameraModel* cam2,
			      vw::ImageViewBase<Image1T> const& image1,
			      vw::ImageViewBase<Image2T> const& image2,
			      int ip_per_tile,
			      vw::cartography::Datum const& datum,
			      vw::ip::InterestPointList& ip1,   // Output IP.
			      vw::ip::InterestPointList& ip2,  
			      vw::Matrix<double> &rough_homography, // The homography that was used.
			      std::string const left_file_path ="",
			      double nodata1 = std::numeric_limits<double>::quiet_NaN(),
			      double nodata2 = std::numeric_limits<double>::quiet_NaN());

  /// Use epipolar line matching with the provided IP.
  template <class Image1T, class Image2T>
  bool epipolar_ip_matching(bool single_threaded_camera,
			    vw::ip::InterestPointList const& ip1,
			    vw::ip::InterestPointList const& ip2,
			    vw::camera::CameraModel* cam1,
			    vw::camera::CameraModel* cam2,
			    vw::ImageViewBase<Image1T> const& image1,
			    vw::ImageViewBase<Image2T> const& image2,
			    vw::cartography::Datum const& datum,
			    double epipolar_threshold,
			    double uniqueness_threshold,
			    std::vector<vw::ip::InterestPoint>& matched_ip1, // Output matched IP.
			    std::vector<vw::ip::InterestPoint>& matched_ip2,
			    double nodata1 = std::numeric_limits<double>::quiet_NaN(),
			    double nodata2 = std::numeric_limits<double>::quiet_NaN());


  /// Detect interest points and use a simple matching technique.
  /// This is not meant to be used directly. Use ip_matching().
  template <class Image1T, class Image2T>
  void detect_match_ip(std::vector<vw::ip::InterestPoint>& matched_ip1,
                       std::vector<vw::ip::InterestPoint>& matched_ip2,
                       vw::ImageViewBase<Image1T> const& image1,
                       vw::ImageViewBase<Image2T> const& image2,
                       int    ip_per_tile,
                       std::string const left_file_path ="",
                       std::string const right_file_path="",
                       double nodata1 = std::numeric_limits<double>::quiet_NaN(),
                       double nodata2 = std::numeric_limits<double>::quiet_NaN(),
                       std::string const& match_file = "");

  // Choose the method used for IP matching
  enum DetectIpMethod { DETECT_IP_METHOD_INTEGRAL = 0,
                        DETECT_IP_METHOD_SIFT     = 1,
                        DETECT_IP_METHOD_ORB      = 2};

  /// Takes interest points and then finds the nearest 10 matches
  /// according to their IP descriptors. It then
  /// filters them by whom are closest to the epipolar line via a
  /// threshold. The first 2 are then selected to be a match if
  /// their descriptor distance is sufficiently far apart.
  class EpipolarLinePointMatcher {
    bool   m_single_threaded_camera;
    double m_uniqueness_threshold, m_epipolar_threshold;
    vw::cartography::Datum m_datum;

  public:
    /// Constructor.
    EpipolarLinePointMatcher(bool single_threaded_camera,
			     double uniqueness_threshold, double inlier_threshold,
			     vw::cartography::Datum const& datum);

    /// This only returns the indicies
    /// - ip_detect_method must match the method used to obtain the interest points
    void operator()(vw::ip::InterestPointList const& ip1,
		    vw::ip::InterestPointList const& ip2,
		    DetectIpMethod  ip_detect_method,
		    vw::camera::CameraModel        * cam1,
		    vw::camera::CameraModel        * cam2,
		    std::vector<size_t>            & output_indices) const;

    /// Work out an epipolar line from interest point. Returns the
    /// coefficients for the following line equation: ax + by + c = 0
    static vw::Vector3 epipolar_line(vw::Vector2            const& feature,
				     vw::cartography::Datum const& datum,
				     vw::camera::CameraModel     * cam_ip,
				     vw::camera::CameraModel     * cam_obj,
				     bool                        & success);

    /// Calculate distance between a line of equation ax + by + c = 0
    static double distance_point_line(vw::Vector3 const& line, vw::Vector2 const& point);

    friend class EpipolarLineMatchTask;
  };

  //-------------------------------------------------------------------------------------------
  // Implementations below

template <class Image1T>
void detect_ip(vw::ip::InterestPointList& ip,
	       vw::ImageViewBase<Image1T> const& image,
	       int ip_per_tile, std::string const file_path, double nodata) {
  using namespace vw;
  ip.clear();

  // If the images are cropped, redo the ip
  // TODO(oalexan1): This won't handle well the case when the input
  // images changed on disk. 
  bool crop_left  = (stereo_settings().left_image_crop_win  != BBox2i(0, 0, 0, 0));
  bool crop_right = (stereo_settings().right_image_crop_win != BBox2i(0, 0, 0, 0));
  bool rebuild = crop_left || crop_right;
  
  // If a valid file_path was provided, just try to read in the IP's from that file.
  if ((file_path != "") && (boost::filesystem::exists(file_path)) && !rebuild) {
    vw_out() << "\t    Reading interest points from file: " << file_path << std::endl;
    ip = ip::read_binary_ip_file_list(file_path);
    vw_out() << "\t    Found interest points: " << ip.size() << std::endl;
    return;
  }
  
  Stopwatch sw;
  sw.start();

  // Automatically determine how many ip we need. Can be overridden below
  // either by --ip-per-image or --ip-per-tile (the latter takes priority).
  double tile_size = 1024.0;
  BBox2i box = bounding_box(image.impl());
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

  vw_out() << "\t    Using " << points_per_tile << " interest points per tile (1024^2 px).\n";

  const bool has_nodata = !boost::math::isnan(nodata);
  
  // Load the detection method from stereo_settings.
  // - This relies on a direct match in the enum integer value.
  DetectIpMethod detect_method = static_cast<DetectIpMethod>(stereo_settings().ip_matching_method);

  // Detect interest points.
  // - Due to templated types we need to duplicate a bunch of code here
  if (detect_method == DETECT_IP_METHOD_INTEGRAL) {
    // Zack's custom detector
    int num_scales = stereo_settings().num_scales;
    if (num_scales <= 0) 
      num_scales = vw::ip::IntegralInterestPointDetector
        <vw::ip::OBALoGInterestOperator>::IP_DEFAULT_SCALES;
    else
      vw_out() << "\t    Using " << num_scales << " scales in OBALoG interest point detection.\n";

    vw::ip::IntegralAutoGainDetector detector(points_per_tile, num_scales);

    // This detector can't handle a mask so if there is nodata just set those pixels to zero.

    vw_out() << "\t    Detecting IP\n";
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
      vw_out() << "\t    Using per-tile image normalization for IP detection...\n";

    bool build_opencv_descriptors = true;
    vw::ip::OpenCvInterestPointDetector detector(cv_method, opencv_normalize, build_opencv_descriptors, points_per_tile);

    // These detectors do accept a mask so use one if applicable.

    vw_out() << "\t    Detecting IP\n";
    if (!has_nodata)
      ip = detect_interest_points(image.impl(), detector, points_per_tile);
    else
      ip = detect_interest_points(create_mask_less_or_equal(image.impl(),nodata), detector, points_per_tile);
  } // End OpenCV case

  sw.stop();
  vw_out(DebugMessage,"asp") << "Detect interest points elapsed time: "
			     << sw.elapsed_seconds() << " s." << std::endl;

  if (!boost::math::isnan(nodata)) {
    vw_out() << "\t    Removing IP near nodata with radius "
             << stereo_settings().ip_nodata_radius << std::endl;
    remove_ip_near_nodata(image.impl(), nodata, ip, stereo_settings().ip_nodata_radius);
  }

  // For the two OpenCV options we already built the descriptors, so
  // only do this for the integral method.
  if (detect_method == DETECT_IP_METHOD_INTEGRAL) {
    sw.start();
    vw_out() << "\t    Building descriptors" << std::endl;
    ip::SGradDescriptorGenerator descriptor;
    if (!has_nodata)
      describe_interest_points(image.impl(), descriptor, ip);
    else
      describe_interest_points(apply_mask(create_mask_less_or_equal(image.impl(),nodata)), descriptor, ip);

    vw_out(DebugMessage,"asp") << "Building descriptors elapsed time: "
                               << sw.elapsed_seconds() << " s." << std::endl;
  }

  vw_out() << "\t    Found interest points: " << ip.size() << std::endl;

  // If a file path was provided, record the IP to disk.
  if (file_path != "") {
    vw_out() << "\t    Recording interest points to file: " << file_path << std::endl;
    ip::write_binary_ip_file(file_path, ip);
  }
}

template <class Image1T, class Image2T>
bool detect_ip_pair(vw::ip::InterestPointList& ip1, 
		    vw::ip::InterestPointList& ip2,  
		    vw::ImageViewBase<Image1T> const& image1,
		    vw::ImageViewBase<Image2T> const& image2,
		    int ip_per_tile,
		    std::string const left_file_path,
		    std::string const right_file_path,
		    double nodata1, double nodata2) {
  using namespace vw;

  // Detect Interest Points
  vw_out() << "\t    Looking for IP in left image...\n";
  detect_ip(ip1, image1.impl(), ip_per_tile, left_file_path, nodata1);
  vw_out() << "\t    Looking for IP in right image...\n";
  detect_ip(ip2, image2.impl(), ip_per_tile, right_file_path, nodata2);
  
  if (stereo_settings().ip_debug_images) {
    vw_out() << "\t    Writing detected IP debug images. " << std::endl;
    write_ip_debug_image("ASP_IP_detect_debug1.tif", image1, ip1, !boost::math::isnan(nodata1), nodata1);
    write_ip_debug_image("ASP_IP_detect_debug2.tif", image2, ip2, !boost::math::isnan(nodata2), nodata2);
  }
  
  side_ip_filtering(ip1, ip2, bounding_box(image1), bounding_box(image2));
  
  return ((ip1.size() > 0) && (ip2.size() > 0));
}

template <class Image1T, class Image2T>
bool detect_ip_aligned_pair(vw::camera::CameraModel* cam1,
			    vw::camera::CameraModel* cam2,
			    vw::ImageViewBase<Image1T> const& image1,
			    vw::ImageViewBase<Image2T> const& image2,
			    int ip_per_tile,
			    vw::cartography::Datum const& datum,
			    vw::ip::InterestPointList& ip1,
			    vw::ip::InterestPointList& ip2,
			    vw::Matrix<double> &rough_homography,
			    std::string const left_file_path,
			    double nodata1, double nodata2) {

  using namespace vw;

  // No longer supporting input transforms, set them to identity.
  const TransformRef left_tx (TranslateTransform(Vector2(0,0)));
  const TransformRef right_tx(TranslateTransform(Vector2(0,0)));

  BBox2i box1 = bounding_box(image1.impl()),
    box2 = bounding_box(image2.impl());

  try {
    // Homography is defined in the original camera coordinates
    rough_homography =  rough_homography_fit(cam1, cam2, left_tx.reverse_bbox(box1),
					     right_tx.reverse_bbox(box2), datum);
  } catch(...) {
    vw_out() << "Rough homography fit failed, trying with identity transform. " << std::endl;
    rough_homography.set_identity(3);
  }

  // Remove the main translation and solve for BBox that fits the
  // image. If we used the translation from the solved homography with
  // poorly position cameras, the right image might be moved out of frame.
  rough_homography(0,2) = rough_homography(1,2) = 0;
  vw_out() << "Aligning right to left for IP capture using rough homography: " 
	   << rough_homography << std::endl;
  
  { // Check to see if this rough homography works
    HomographyTransform func(rough_homography);
    VW_ASSERT(box1.intersects(func.forward_bbox(box2)),
	      LogicErr() << "The rough homography alignment based on datum and camera geometry shows that input images do not overlap at all. Unable to proceed. Examine your images, or consider using the option --skip-rough-homography.\n");
  }

  TransformRef tx(compose(right_tx, HomographyTransform(rough_homography)));
  BBox2i raster_box = tx.forward_bbox(right_tx.reverse_bbox(box2));
  tx = TransformRef(compose(TranslateTransform(-raster_box.min()),
                            right_tx, HomographyTransform(rough_homography)));
  raster_box -= Vector2i(raster_box.min());
  
  // Detect interest points for the left and (transformed) right image.
  // - It is important that we use NearestPixelInterpolation in the
  //   next step. Using anything else will interpolate nodata values
  //   and stop them from being masked out.
  if (!detect_ip_pair(ip1, ip2,
                      image1.impl(),
                      crop(transform(image2.impl(), compose(tx, inverse(right_tx)),
                                     ValueEdgeExtension<typename Image2T::pixel_type>(boost::math::isnan(nodata2) ? 0 : nodata2),
                                     NearestPixelInterpolation()),
                           raster_box),
                      ip_per_tile, left_file_path, "", // Don't record IP from transformed images.
                      nodata1, nodata2)) {
    vw_out() << "Unable to detect interest points." << std::endl;
    return false;
  }

  // Factor the transform out of the right interest points
  ip::InterestPointList::iterator ip_it;
  for (ip_it = ip2.begin(); ip_it != ip2.end(); ++ip_it) {
    Vector2 pt(ip_it->x, ip_it->y);
    pt = tx.reverse(pt);
    ip_it->ix = ip_it->x = pt.x();
    ip_it->iy = ip_it->y = pt.y();
  }

  return true;
} // End function detect_ip_aligned_pair

template <class Image1T, class Image2T>
bool epipolar_ip_matching(bool single_threaded_camera,
			  vw::ip::InterestPointList const& ip1,
			  vw::ip::InterestPointList const& ip2,
			  vw::camera::CameraModel* cam1,
			  vw::camera::CameraModel* cam2,
			  vw::ImageViewBase<Image1T> const& image1,
			  vw::ImageViewBase<Image2T> const& image2,
			  vw::cartography::Datum const& datum,
			  double epipolar_threshold,
			  double uniqueness_threshold,
			  std::vector<vw::ip::InterestPoint>& matched_ip1,
			  std::vector<vw::ip::InterestPoint>& matched_ip2,
			  double nodata1, double nodata2) {
  using namespace vw;
  
  matched_ip1.clear();
  matched_ip2.clear();

  // Match interest points forward/backward .. constraining on epipolar line
  DetectIpMethod detect_method = static_cast<DetectIpMethod>(stereo_settings().ip_matching_method);
  std::vector<size_t> forward_match, backward_match;
  vw_out() << "\t--> Matching interest points using the epipolar line." << std::endl;
  vw_out() << "\t    Uniqueness threshold: " << uniqueness_threshold << "\n";
  vw_out() << "\t    Epipolar threshold:   " << epipolar_threshold   << "\n";
  
  EpipolarLinePointMatcher matcher(single_threaded_camera,
				   uniqueness_threshold, epipolar_threshold, datum);
  vw_out() << "\t    Matching forward" << std::endl;
  matcher(ip1, ip2, detect_method, cam1, cam2, forward_match);
  vw_out() << "\t    ---> Obtained " << forward_match.size() << " matches." << std::endl;
  vw_out() << "\t    Matching backward" << std::endl;
  matcher(ip2, ip1, detect_method, cam2, cam1, backward_match);
  vw_out() << "\t    ---> Obtained " << backward_match.size() << " matches." << std::endl;

  // Perform circle consistency check
  size_t valid_count = 0;
  const size_t NULL_INDEX = (size_t)(-1);
  for (size_t i = 0; i < forward_match.size(); i++) {
    if (forward_match[i] != NULL_INDEX) {
      if (backward_match[forward_match[i]] != i) {
        forward_match[i] = NULL_INDEX;
      } else {
        valid_count++;
      }
    }
  }
  vw_out() << "\t    Matched " << valid_count << " points." << std::endl;

  if (valid_count == 0)
    return false;

  // Produce listing of valid indices that agree with forward and backward matching
  matched_ip1.reserve(valid_count); // Get our allocations out of the way.
  matched_ip2.reserve(valid_count);
  {
    ip::InterestPointList::const_iterator ip1_it = ip1.begin(), ip2_it = ip2.begin();
    for (size_t i = 0; i < forward_match.size(); i++) {
      if (forward_match[i] != NULL_INDEX) {
        matched_ip1.push_back(*ip1_it);
        ip2_it = ip2.begin();
        std::advance(ip2_it, forward_match[i]);
        matched_ip2.push_back(*ip2_it);
      }
      ip1_it++;
    }
  }

  if (stereo_settings().ip_debug_images) {
    vw_out() << "\t    Writing IP match debug image prior to geometric filtering.\n";
    write_match_image("InterestPointMatching__ip_matching_debug.tif",
                      image1, image2, matched_ip1, matched_ip2);
  }

  // Apply filtering of IP by a selection of assumptions. Low
  // triangulation error, agreement with klt tracking, and local
  // neighbors are the same neighbors in both images.
  std::list<size_t> good_indices;
  for (size_t i = 0; i < matched_ip1.size(); i++) {
    good_indices.push_back(i);
  }
  if (!stereo_settings().disable_tri_filtering) {
    if (!tri_ip_filtering(matched_ip1, matched_ip2,
			  cam1, cam2, good_indices)){
      vw_out() << "No interest points left after triangulation filtering." << std::endl;
      return false;
    }
  }
  if (!stddev_ip_filtering(matched_ip1, matched_ip2, good_indices)) {
    vw_out() << "No interest points left after stddev filtering." << std::endl;
    return false;
  }

  // Record new list that contains only the inliers.
  vw_out() << "\t    Reduced matches to " << good_indices.size() << "\n";
  std::vector<ip::InterestPoint> buffer(good_indices.size());

  // Subselect and copy ip1
  size_t w_index = 0;
  BOOST_FOREACH(size_t index, good_indices) {
    Vector2 l(matched_ip1[index].x, matched_ip1[index].y);
    matched_ip1[index].ix = matched_ip1[index].x = l.x();
    matched_ip1[index].iy = matched_ip1[index].y = l.y();
    buffer[w_index] = matched_ip1[index];
    w_index++;
  }
  matched_ip1 = buffer;

  // Subselect and copy ip2
  w_index = 0;
  BOOST_FOREACH(size_t index, good_indices) {
    Vector2 r(matched_ip2[index].x, matched_ip2[index].y);
    matched_ip2[index].ix = matched_ip2[index].x = r.x();
    matched_ip2[index].iy = matched_ip2[index].y = r.y();
    buffer[w_index] = matched_ip2[index];
    w_index++;
  }
  matched_ip2 = buffer;

  // The interest points are not aligned here, so there's no need to align them
  vw::TransformPtr tx1(NULL);
  vw::TransformPtr tx2(NULL);
    
  // If options are set, filter by elevation.
  if (stereo_settings().elevation_limit[0] < stereo_settings().elevation_limit[1] ||
      !stereo_settings().lon_lat_limit.empty()) {

    std::vector<ip::InterestPoint> matched_ip1_out, matched_ip2_out;
    filter_ip_by_lonlat_and_elevation(tx1, tx2,
                                      cam1, cam2,
                                      datum,  matched_ip1, matched_ip2,
                                      stereo_settings().elevation_limit,  
                                      stereo_settings().lon_lat_limit,  
                                      matched_ip1_out, matched_ip2_out);
    matched_ip1 = matched_ip1_out;
    matched_ip2 = matched_ip2_out;
  } // End elevation filtering

  if (stereo_settings().ip_debug_images) {
    vw_out() << "\t    Writing IP match debug image after geometric filtering.\n";
    write_match_image("InterestPointMatching__ip_matching_debug2.tif",
                      image1, image2, matched_ip1, matched_ip2);
  }

  return true;
} // End function epipolar_ip_matching

template <class Image1T, class Image2T>
void detect_match_ip(std::vector<vw::ip::InterestPoint>& matched_ip1,
		     std::vector<vw::ip::InterestPoint>& matched_ip2,
		     vw::ImageViewBase<Image1T> const& image1,
		     vw::ImageViewBase<Image2T> const& image2,
		     int ip_per_tile,
		     std::string const left_file_path,
		     std::string const right_file_path,
		     double nodata1, double nodata2, std::string const& match_file) {
  using namespace vw;

  // Detect Interest Points
  vw::ip::InterestPointList ip1, ip2;
  detect_ip_pair(ip1, ip2, image1.impl(), image2.impl(), ip_per_tile,
                 left_file_path, right_file_path, nodata1, nodata2);
  
  // Match the interset points using the default matcher

  // Replace the IP lists with IP vectors
  std::vector<vw::ip::InterestPoint> ip1_copy, ip2_copy;
  ip1_copy.reserve(ip1.size());
  ip2_copy.reserve(ip2.size());
  std::copy(ip1.begin(), ip1.end(), std::back_inserter(ip1_copy));
  std::copy(ip2.begin(), ip2.end(), std::back_inserter(ip2_copy));

  DetectIpMethod detect_method = static_cast<DetectIpMethod>(stereo_settings().ip_matching_method);

  // Best point must be closer than the next best point
  double adj_uniqueness_thresh = (0.8/0.7)*stereo_settings().ip_uniqueness_thresh;
  adj_uniqueness_thresh = std::min(0.99, adj_uniqueness_thresh);
  vw_out() << "\t--> Adjusted uniqueness threshold: " << adj_uniqueness_thresh << "\n";
  // TODO: Should probably unify the ip::InterestPointMatcher class
  // with the EpipolarLinePointMatcher class!
  if (detect_method != DETECT_IP_METHOD_ORB) {
    // For all L2Norm distance metrics
    ip::InterestPointMatcher<ip::L2NormMetric,ip::NullConstraint> matcher(adj_uniqueness_thresh);
    matcher(ip1_copy, ip2_copy, matched_ip1, matched_ip2,
	    TerminalProgressCallback("asp", "\t   Matching: "));
  }
  else {
    // For Hamming distance metrics
    ip::InterestPointMatcher<ip::HammingMetric,ip::NullConstraint> matcher(adj_uniqueness_thresh);
    matcher(ip1_copy, ip2_copy, matched_ip1, matched_ip2,
	    TerminalProgressCallback("asp", "\t   Matching: "));
  }

  ip::remove_duplicates(matched_ip1, matched_ip2);

  vw_out() << "\n\t    Matched points: " << matched_ip1.size() << std::endl;

  if (stereo_settings().ip_debug_images) {
    vw_out() << "\t    Writing IP initial match debug image.\n";
    write_match_image("InterestPointMatching__ip_matching_debug.tif",
                      image1, image2, matched_ip1, matched_ip2);
  }

  // Save ip
  if (match_file != "") {
    // Create the output directory
    vw::create_out_dir(match_file);
    vw_out() << "Writing: " << match_file << std::endl;
    ip::write_binary_match_file(match_file, matched_ip1, matched_ip2);
  }
  
} // End function detect_match_ip


// Homography IP matching
//
// This applies only the homography constraint. Not the best...
template <class Image1T, class Image2T>
bool homography_ip_matching(vw::ImageViewBase<Image1T> const& image1,
			    vw::ImageViewBase<Image2T> const& image2,
			    int    ip_per_tile,
			    int    inlier_threshold,
			    std::string const& output_name,
			    std::string const  left_file_path,
			    std::string const  right_file_path,
			    double nodata1, double nodata2) {

  using namespace vw;

  vw_out() << "\t--> Matching interest points using homography.\n";

  std::vector<ip::InterestPoint> matched_ip1, matched_ip2;
  detect_match_ip(matched_ip1, matched_ip2,
		  image1.impl(), image2.impl(),
		  ip_per_tile,
		  left_file_path, right_file_path,
		  nodata1, nodata2);
  if (matched_ip1.size() == 0 || matched_ip2.size() == 0)
    return false;
    
  std::vector<ip::InterestPoint> final_ip1, final_ip2;
  size_t num_left = filter_ip_homog(matched_ip1, matched_ip2, final_ip1, final_ip2,
                                    inlier_threshold);
  if (num_left == 0)
    return false;

  if (stereo_settings().ip_debug_images) {
    vw_out() << "\t    Writing post-homography IP match debug image.\n";
    write_match_image("InterestPointMatching__ip_matching_debug2.tif",
                      image1, image2, final_ip1, final_ip2);
  }

  vw_out() << "\t    * Writing match file: " << output_name << "\n";
  ip::write_binary_match_file(output_name, final_ip1, final_ip2);

  return true;
}

template <class Image1T, class Image2T>
bool ip_matching_no_align(bool single_threaded_camera,
			  vw::camera::CameraModel* cam1,
			  vw::camera::CameraModel* cam2,
			  vw::ImageViewBase<Image1T> const& image1,
			  vw::ImageViewBase<Image2T> const& image2,
			  int ip_per_tile,
			  vw::cartography::Datum const& datum,
			  double epipolar_threshold,
			  double uniqueness_threshold,
			  std::string const& output_name,
			  std::string const  left_file_path,
			  std::string const  right_file_path,
			  double nodata1,
			  double nodata2) {
  using namespace vw;
  
  // Find IP
  vw::ip::InterestPointList ip1, ip2;
  detect_ip_pair(ip1, ip2, image1.impl(), image2.impl(),
                 ip_per_tile, left_file_path, right_file_path,
                 nodata1, nodata2);

  // Match them
  std::vector<ip::InterestPoint> matched_ip1, matched_ip2;
  if (!epipolar_ip_matching(single_threaded_camera,
			    ip1, ip2, cam1, cam2, image1, image2,
			    datum, epipolar_threshold, uniqueness_threshold,
			    matched_ip1, matched_ip2, nodata1, nodata2))
    return false;

  // Write to disk
  vw_out() << "\t    * Writing match file: " << output_name << "\n";
  ip::write_binary_match_file(output_name, matched_ip1, matched_ip2);

  return true;
}

template <class Image1T, class Image2T>
bool ip_matching_w_alignment(bool single_threaded_camera,
			     vw::camera::CameraModel* cam1,
			     vw::camera::CameraModel* cam2,
			     vw::ImageViewBase<Image1T> const& image1,
			     vw::ImageViewBase<Image2T> const& image2,
			     int ip_per_tile,
			     vw::cartography::Datum const& datum,
			     std::string const& output_name,
			     double epipolar_threshold,
			     double uniqueness_threshold,
			     std::string const left_file_path,
			     double nodata1,
			     double nodata2) {

  using namespace vw;

  vw_out() << "Performing IP matching with alignment." << std::endl;

  // This call aligns the right image to the left image then detects IPs in the two images.
  vw::ip::InterestPointList ip1, ip2;
  Matrix<double> rough_homography;
  detect_ip_aligned_pair(cam1, cam2, image1.impl(), image2.impl(),
                         ip_per_tile, datum, ip1, ip2, rough_homography, 
                         left_file_path, nodata1, nodata2);


  // Match the detected IPs which are in the original image coordinates.
  std::vector<ip::InterestPoint> matched_ip1, matched_ip2;
  bool inlier =
    epipolar_ip_matching(single_threaded_camera,
			 ip1, ip2,
			 cam1, cam2,
			 image1.impl(), image2.impl(),
			 datum, epipolar_threshold, uniqueness_threshold,
			 matched_ip1, matched_ip2,
			 nodata1, nodata2);
  if (!inlier)
    return false;

  // Write the matches to disk
  vw_out() << "\t    * Writing match file: " << output_name << "\n";
  ip::write_binary_match_file(output_name, matched_ip1, matched_ip2);

  // Use the interest points that we found to compute an aligning
  // homography transform for the two images.
  // - This is just a sanity check.
  bool adjust_left_image_size = true;
  Matrix<double> matrix1, matrix2;
  homography_rectification(adjust_left_image_size,
			   image1.get_size(), image2.get_size(),
			   matched_ip1, matched_ip2, matrix1, matrix2);
  if (sum(abs(submatrix(rough_homography,0,0,2,2) - submatrix(matrix2,0,0,2,2))) > 4) {
    vw_out() << "Post homography has largely different scale and skew from rough fit. Post solution is " 
	     << matrix2 << ". Examine your images, or consider using the option --skip-rough-homography.\n";
    //return false;
  }

  return inlier;
}

} // End namespace asp

#endif//__ASP_CORE_INTEREST_POINT_MATCHING_H__
