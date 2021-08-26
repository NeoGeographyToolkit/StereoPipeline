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
#include <vw/Cartography/Datum.h>
#include <vw/Mosaic/ImageComposite.h>
#include <vw/Math/RANSAC.h>
#include <vw/Math/Geometry.h>
#include <vw/FileIO/FileUtils.h>

#include <asp/Core/StereoSettings.h>
#include <boost/foreach.hpp>
#include <boost/math/special_functions/fpclassify.hpp>

// TODO: This function should live somewhere else!  It was pulled from vw->tools->ipmatch.cc
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

  /// Calls ip matching above but with an additional step where we
/// apply a homography to make right image like left image. This is
/// useful so that both images have similar scale and similar affine qualities.
/// - Only the left image will use an IP file since the right image is modified.
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
  vw::Matrix<double> translation_ip_matching(vw::ImageView<float> const& image1,
					     vw::ImageView<float> const& image2,
					     int ip_per_tile,
					     std::string const  left_file_path ="",
					     std::string const  right_file_path="",
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
  bool
  tri_ip_filtering(std::vector<vw::ip::InterestPoint> const& ip1,
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
  size_t filter_ip_by_lonlat_and_elevation
  (vw::camera::CameraModel* left_camera_model,
   vw::camera::CameraModel* right_camera_model,
   vw::cartography::Datum const& datum,
   std::vector<vw::ip::InterestPoint> const& ip1_in,
   std::vector<vw::ip::InterestPoint> const& ip2_in,
   double ip_scale,
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
  
  // Outlier removal based on the disparity of interest points.
  // Points with x or y disparity not within the 100-'pct' to 'pct'
  // percentile interval expanded by 'factor' will be removed as
  // outliers. Overwrite the ip in place.
  void filter_ip_by_disparity(double pct, // for example, 90.0
                              double factor, // for example, 3.0
                              std::vector<vw::ip::InterestPoint> & left_ip,
                              std::vector<vw::ip::InterestPoint> & right_ip);

  /// Filter IP points by how reasonably the disparity can change along rows
  /// - Returns the number of points remaining after filtering.
  size_t filter_ip_homog(std::vector<vw::ip::InterestPoint> const& ip1_in,
                         std::vector<vw::ip::InterestPoint> const& ip2_in,
                         std::vector<vw::ip::InterestPoint>      & ip1_out,
                         std::vector<vw::ip::InterestPoint>      & ip2_out,
                         int inlier_threshold = 1);

  //-------------------------------------------------------------------------------------------
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
  
  /// Estimate the "spread" of IP coverage in an image.
  /// - Returns a value between 0 and 1.
  /// - Breaks the image into tiles and checks how many tiles have at least N IP.
  double calc_ip_coverage_fraction(std::vector<vw::ip::InterestPoint> const& ip,
                                   vw::Vector2i const& image_size, int tile_size=1024,
                                   int min_ip_per_tile=2);

  /// The unwarped disparity file name.
  /// We put it here because this file knows about vw::ip. That is why it should not
  /// in Common.h which should not know so much.
  std::string unwarped_disp_file(std::string const& prefix, std::string const& left_image,
                                 std::string const& right_image);


  //-------------------------------------------------------------------------------------------
  // Lower level IP detection functions

  /// Detect interest points
  ///
  /// This is not meant to be used directly. Please use ip_matching() or
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
  ///
  /// This is not meant to be used directly. Please use ip_matching
  template <class Image1T, class Image2T>
  void detect_match_ip(std::vector<vw::ip::InterestPoint>& matched_ip1,
		       std::vector<vw::ip::InterestPoint>& matched_ip2,
		       vw::ImageViewBase<Image1T> const& image1,
		       vw::ImageViewBase<Image2T> const& image2,
		       int ip_per_tile,
		       std::string const left_file_path ="",
		       std::string const right_file_path="",
		       double nodata1 = std::numeric_limits<double>::quiet_NaN(),
		       double nodata2 = std::numeric_limits<double>::quiet_NaN(),
		       std::string const& match_file = "");

  //-------------------------------------------------------------------------------------------


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

  // Include the template function definitions.
#include <asp/Core/InterestPointMatching.tcc>

} // End namespace asp

#endif//__ASP_CORE_INTEREST_POINT_MATCHING_H__
