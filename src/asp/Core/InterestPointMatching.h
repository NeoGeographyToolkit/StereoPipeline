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

#include <vw/Core.h>
#include <vw/Core/Stopwatch.h>
#include <vw/Math.h>
#include <vw/Image/ImageViewBase.h>
#include <vw/Image/MaskViews.h>
#include <vw/Camera/CameraModel.h>
#include <vw/Mosaic.h>
#include <vw/InterestPoint/Matcher.h>
#include <vw/InterestPoint/InterestData.h>
#include <vw/InterestPoint/IntegralDetector.h>
#include <vw/Cartography/Datum.h>

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
  if ( sub_scale > 1 )
    sub_scale = 1;

  vw::mosaic::ImageComposite<vw::PixelRGB<vw::uint8> > composite;
  composite.insert(vw::pixel_cast_rescale<vw::PixelRGB<vw::uint8> >(resample(normalize(image1), sub_scale)), 0, 0 );
  composite.insert(vw::pixel_cast_rescale<vw::PixelRGB<vw::uint8> >(resample(normalize(image2), sub_scale)), vw::int32(image1.impl().cols() * sub_scale), 0 );
  composite.set_draft_mode( true );
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
    for (float r=0; r<1.0; r+=inc_amt ){
      int i = (int)(0.5 + start.x() + r*(end.x()-start.x()));
      int j = (int)(0.5 + start.y() + r*(end.y()-start.y()));
      if (i >=0 && j >=0 && i < comp.cols() && j < comp.rows())
	comp(i,j) = vw::PixelRGB<vw::uint8>(255, 0, 0);
    }
  }

  vw::vw_out() << "\t    Write to disk "  <<std::endl;
  boost::scoped_ptr<vw::DiskImageResource> rsrc( vw::DiskImageResource::create(out_file_name, comp.format()) );
  block_write_image( *rsrc, comp, vw::TerminalProgressCallback( "tools.ipmatch", "Writing Debug:" ) );
}


// TODO: This function should live somewhere else!
template <typename ImageT, typename IPT>
void write_point_image(std::string const& out_file_name,
		       vw::ImageViewBase<ImageT> const& image,
		       IPT const& ip) {
  vw::vw_out() << "\t    Starting write_match_image " << std::endl;

  // Skip image pairs with no matches.
  if (ip.empty())
    return;

  // Work out the scaling to produce the subsampled images. These
  // values are choosen just allow a reasonable rendering time.
  float sub_scale  = sqrt(1500.0 * 1500.0 / float(image.impl().cols() * image.impl().rows()));
	sub_scale /= 2;
  if ( sub_scale > 1 )
    sub_scale = 1;

  vw::ImageView<vw::PixelRGB<vw::uint8> > canvas =
	vw::pixel_cast_rescale<vw::PixelRGB<vw::uint8> >(resample(normalize(image), sub_scale));

  vw::vw_out() << "\t    Draw points "<<  std::endl;

  // Draw all the interest points
  for (typename IPT::const_iterator iter=ip.begin(); iter != ip.end(); ++iter) {
    vw::Vector2f pt(iter->x, iter->y);
    pt *= sub_scale;

    // Draw a little square
    // TODO: - Error checking!
    for (size_t i=pt[0]-1; i<=pt[0]+1; ++i )
      for (size_t j=pt[1]-1; j<=pt[1]+1; ++j )
	canvas(i,j) = vw::PixelRGB<vw::uint8>(255, 0, 0);
  }

  vw::vw_out() << "\t    Write to disk "  <<std::endl;
  boost::scoped_ptr<vw::DiskImageResource> rsrc( vw::DiskImageResource::create(out_file_name, canvas.format()) );
  block_write_image( *rsrc, canvas, vw::TerminalProgressCallback( "tools.ipmatch", "Writing Debug:" ) );
}





namespace asp {


  // Choose the method used for IP matching
  enum DetectIpMethod { DETECT_IP_METHOD_INTEGRAL = 0,
			DETECT_IP_METHOD_SIFT     = 1,
			DETECT_IP_METHOD_ORB      = 2};

  /// Takes interest points and then finds the nearest 10 matches
  /// according to their IP descriptiors. It then
  /// filters them by whom are closest to the epipolar line via a
  /// threshold. The first 2 are then selected to be a match if
  /// their descriptor distance is sufficiently far apart.
  class EpipolarLinePointMatcher {
    bool   m_single_threaded_camera;
    double m_uniqueness_threshold, m_epipolar_threshold;
    vw::cartography::Datum m_datum;

  public:
    /// Constructor.
    EpipolarLinePointMatcher( bool single_threaded_camera,
			      double uniqueness_threshold, double inlier_threshold,
			      vw::cartography::Datum const& datum);

    /// This only returns the indicies
    /// - ip_detect_method must match the method used to obtain the interest points
    void operator()( vw::ip::InterestPointList const& ip1,
		     vw::ip::InterestPointList const& ip2,
		     DetectIpMethod  ip_detect_method,
		     vw::camera::CameraModel        * cam1,
		     vw::camera::CameraModel        * cam2,
		     vw::TransformRef          const& tx1,
		     vw::TransformRef          const& tx2,
		     std::vector<size_t>            & output_indices ) const;

    /// Work out an epipolar line from interest point. Returns the
    /// coefficients for the following line equation: ax + by + c = 0
    static vw::Vector3 epipolar_line( vw::Vector2            const& feature,
				      vw::cartography::Datum const& datum,
				      vw::camera::CameraModel     * cam_ip,
				      vw::camera::CameraModel     * cam_obj,
				      bool                        & success);

    /// Calculate distance between a line of equation ax + by + c = 0
    static double distance_point_line( vw::Vector3 const& line,
				       vw::Vector2 const& point );

    friend class EpipolarLineMatchTask;
  };

  /// Tool to remove points on or within radius px of nodata pixels.
  /// Note: A nodata pixel is one for which pixel <= nodata.
  template <class ImageT>
  void remove_ip_near_nodata( vw::ImageViewBase<ImageT> const& image,
                              double nodata,
                              vw::ip::InterestPointList& ip_list,
                              int radius = 1 );

  /// Find a rough homography that maps right to left using the camera
  /// and datum information.
  /// - This intersects rays with the datum, then projects them into the other camera.
  vw::Matrix<double>
  rough_homography_fit( vw::camera::CameraModel* cam1,
			vw::camera::CameraModel* cam2,
			vw::BBox2i const& box1, vw::BBox2i const& box2,
			vw::cartography::Datum const& datum );

  /// Homography rectification that aligns the right image to the left
  /// image via a homography transform. It returns a vector2i of the
  /// ideal cropping size to use for the left and right image. The left
  /// transform is actually just a translation that sets origin to the
  /// shared corner of left and right.
  vw::Vector2i
  homography_rectification( bool adjust_left_image_size,
			    vw::Vector2i const& left_size,
			    vw::Vector2i const& right_size,
			    std::vector<vw::ip::InterestPoint> const& left_ip,
			    std::vector<vw::ip::InterestPoint> const& right_ip,
			    vw::Matrix<double>& left_matrix,
			    vw::Matrix<double>& right_matrix );

  /// Detect InterestPoints
  ///
  /// This is not meant to be used directly. Please use ip_matching() or
  /// the dumb homography_ip_matching().
  template <class List1T, class List2T, class Image1T, class Image2T>
  void detect_ip( List1T& ip1, List2T& ip2,
		  vw::ImageViewBase<Image1T> const& image1,
		  vw::ImageViewBase<Image2T> const& image2,
		  int ip_per_tile,
		  double nodata1 = std::numeric_limits<double>::quiet_NaN(),
		  double nodata2 = std::numeric_limits<double>::quiet_NaN() );

  /// Detect and Match Interest Points
  ///
  /// This is not meant to be used directly. Please use ip_matching
  template <class Image1T, class Image2T>
  void detect_match_ip( std::vector<vw::ip::InterestPoint>& matched_ip1,
			std::vector<vw::ip::InterestPoint>& matched_ip2,
			vw::ImageViewBase<Image1T> const& image1,
			vw::ImageViewBase<Image2T> const& image2,
			int ip_per_tile,
			double nodata1 = std::numeric_limits<double>::quiet_NaN(),
			double nodata2 = std::numeric_limits<double>::quiet_NaN() );

  /// Homography IP matching
  ///
  /// This applies only the homography constraint. Not the best...
  template <class Image1T, class Image2T>
  bool homography_ip_matching( vw::ImageViewBase<Image1T> const& image1,
			       vw::ImageViewBase<Image2T> const& image2,
			       int ip_per_tile,
			       std::string const& output_name,
			       int inlier_threshold=10,
			       double nodata1 = std::numeric_limits<double>::quiet_NaN(),
			       double nodata2 = std::numeric_limits<double>::quiet_NaN() );

  /// IP matching that uses clustering on triangulation error to
  /// determine inliers.  Check output this filter can fail.
  ///
  /// Input and output is the valid indices. Valid indices must have
  /// something to start with.
  bool
  tri_ip_filtering( std::vector<vw::ip::InterestPoint> const& ip1,
		    std::vector<vw::ip::InterestPoint> const& ip2,
		    vw::camera::CameraModel* cam1,
		    vw::camera::CameraModel* cam2,
		    std::list<size_t>& valid_indices,
		    vw::TransformRef const& left_tx  = vw::TransformRef(vw::TranslateTransform(0,0)),
		    vw::TransformRef const& right_tx = vw::TransformRef(vw::TranslateTransform(0,0)) );

  /// 4 stddev filtering. Deletes any disparity measurement that is 4
  /// stddev away from the measurements of it's local neighbors. We
  /// kill off worse offender one at a time until everyone is compliant.
  bool stddev_ip_filtering( std::vector<vw::ip::InterestPoint> const& ip1,
			    std::vector<vw::ip::InterestPoint> const& ip2,
			    std::list<size_t>& valid_indices );

  // Filter IP by ensuring that the triangulated IP are in the given lon-lat-height box
  size_t filter_ip_by_lonlat_and_elevation
  (vw::camera::CameraModel* left_camera_model,
   vw::camera::CameraModel* right_camera_model,
   vw::cartography::Datum        const& datum,
   std::vector<vw::ip::InterestPoint> const& ip1_in,
   std::vector<vw::ip::InterestPoint> const& ip2_in,
   vw::TransformRef const& left_tx, vw::TransformRef const& right_tx, 
   double ip_scale,
   vw::Vector2 const & elevation_limit,
   vw::BBox2   const & lon_lat_limit,
   std::vector<vw::ip::InterestPoint> & ip1_out,
   std::vector<vw::ip::InterestPoint> & ip2_out);

  /// Smart IP matching that uses clustering on triangulation and
  /// datum information to determine inliers.
  ///
  /// Left and Right TX define transforms that have been performed on
  /// the images that that camera data doesn't know about. (ie scaling).
  template <class Image1T, class Image2T>
  bool ip_matching( bool single_threaded_camera,
		    vw::camera::CameraModel* cam1,
		    vw::camera::CameraModel* cam2,
		    vw::ImageViewBase<Image1T> const& image1,
		    vw::ImageViewBase<Image2T> const& image2,
		    int ip_per_tile,
		    vw::cartography::Datum const& datum,
		    std::string const& output_name,
		    double epipolar_threshold,
		    double uniqueness_threshold,
		    double nodata1 = std::numeric_limits<double>::quiet_NaN(),
		    double nodata2 = std::numeric_limits<double>::quiet_NaN(),
		    vw::TransformRef const& left_tx  = vw::TransformRef(vw::TranslateTransform(0,0)),
		    vw::TransformRef const& right_tx = vw::TransformRef(vw::TranslateTransform(0,0)),
		    bool transform_to_original_coord = true );

  /// Calls ip matching above but with an additional step where we
  /// apply a homography to make right image like left image. This is
  /// useful so that both images have similar scale and similar affine qualities.
  template <class Image1T, class Image2T>
  bool ip_matching_w_alignment( bool single_threaded_camera,
				vw::camera::CameraModel* cam1,
				vw::camera::CameraModel* cam2,
				vw::ImageViewBase<Image1T> const& image1,
				vw::ImageViewBase<Image2T> const& image2,
				int ip_per_tile,
				vw::cartography::Datum const& datum,
				std::string const& output_name,
				double epipolar_threshold,
				double uniqueness_threshold,
				double nodata1 = std::numeric_limits<double>::quiet_NaN(),
				double nodata2 = std::numeric_limits<double>::quiet_NaN(),
				vw::TransformRef const& left_tx  = vw::TransformRef(vw::TranslateTransform(0,0)),
				vw::TransformRef const& right_tx = vw::TransformRef(vw::TranslateTransform(0,0)) );

// ==============================================================================================
// Function definitions
// TODO: Move to .tcc file



  // Tool to remove points on or within 1 px of nodata pixels.
  // Note: A nodata pixel is one for which pixel <= nodata.
  template <class ImageT>
  void remove_ip_near_nodata( vw::ImageViewBase<ImageT> const& image,   double nodata,
                              vw::ip::InterestPointList      & ip_list, int    radius ){

    using namespace vw;
    size_t prior_ip = ip_list.size();

    typedef ImageView<typename ImageT::pixel_type> CropImageT;
    const int width = 2*radius+1;
    CropImageT subsection(width,width);

    // Get shrunk bounding box
    BBox2i bound = bounding_box( image.impl() );
    bound.contract(radius); 
    // Loop through all the points
    for ( ip::InterestPointList::iterator ip = ip_list.begin(); ip != ip_list.end(); ++ip ) {
      
      // Remove the point if it was too close to the image borders
      if ( !bound.contains( Vector2i(ip->ix,ip->iy) ) ) {
        ip = ip_list.erase(ip);
        ip--;
        continue;
      }

      // Remove the point if any invalid pixels are too close to it
      subsection = crop( image.impl(), ip->ix-radius, ip->iy-radius, width, width );
      for ( typename CropImageT::iterator pixel = subsection.begin();
	    pixel != subsection.end(); pixel++ ) {
        if (*pixel <= nodata) {
          ip = ip_list.erase(ip);
          ip--;
          break;
        }
      }
    }
    VW_OUT( DebugMessage, "asp" ) << "Removed " << prior_ip - ip_list.size()
				  << " interest points due to their proximity to nodata values."
				  << std::endl << "Nodata value used " << nodata << std::endl;
  } // End function remove_ip_near_nodata

  // Detect InterestPoints
  //
  /// This is not meant to be used directly. Please use ip_matching() or
  /// the dumb homography_ip_matching().
  template <class List1T, class List2T, class Image1T, class Image2T>
  void detect_ip( List1T& ip1, List2T& ip2,
		  vw::ImageViewBase<Image1T> const& image1,
		  vw::ImageViewBase<Image2T> const& image2,
		  int ip_per_tile,
		  double nodata1,
		  double nodata2 ) {
    using namespace vw;
    BBox2i box1 = bounding_box(image1.impl());
    ip1.clear();
    ip2.clear();

    Stopwatch sw;
    sw.start();

    // Automatically determine how many ip we need
    float  number_boxes    = (box1.width() / 1024.f) * (box1.height() / 1024.f);
    size_t points_per_tile = 5000.f / number_boxes;
    if ( points_per_tile > 5000 ) points_per_tile = 5000;
    if ( points_per_tile < 50   ) points_per_tile = 50;

    // See if to override with manual value
    if (ip_per_tile != 0)
      points_per_tile = ip_per_tile;

    vw_out() << "Using " << points_per_tile
	     << " interest points per tile (1024^2 px).\n";

    // Load the detection method from stereo_settings.
    // - This relies on a direct match in the enum integer value.
    DetectIpMethod detect_method = static_cast<DetectIpMethod>(stereo_settings().ip_matching_method);

    // Detect Interest Points
    // - Due to templated types we need to duplicate a bunch of code here
    if (detect_method == DETECT_IP_METHOD_INTEGRAL) {
      // Zack's custom detector
      int num_scales = stereo_settings().num_scales;
      if (num_scales <= 0) 
        num_scales = vw::ip::IntegralInterestPointDetector
          <vw::ip::OBALoGInterestOperator>::IP_DEFAULT_SCALES;
      else
        vw_out() << "Using " << num_scales << " scales in OBALoG interest point detection.\n";
      
      vw::ip::IntegralAutoGainDetector detector( points_per_tile, num_scales );
      
      vw_out() << "\t    Processing left image" << std::endl;
      if ( boost::math::isnan(nodata1) )
        ip1 = detect_interest_points( image1.impl(), detector );
      else
        ip1 = detect_interest_points( apply_mask(create_mask_less_or_equal(image1.impl(),nodata1)), detector );
      vw_out() << "\t    Processing right image" << std::endl;
      if ( boost::math::isnan(nodata2) )
        ip2 = detect_interest_points( image2.impl(), detector );
      else
        ip2 = detect_interest_points( apply_mask(create_mask_less_or_equal(image2.impl(),nodata2)), detector );
    } else {

      // Initialize the OpenCV detector.  Conveniently we can just pass in the type argument.
      // - If VW was not build with OpenCV, this call will just throw an exception.
      vw::ip::OpenCvIpDetectorType cv_method = vw::ip::OPENCV_IP_DETECTOR_TYPE_SIFT;
      if (detect_method == DETECT_IP_METHOD_ORB)
        cv_method = vw::ip::OPENCV_IP_DETECTOR_TYPE_ORB;

      // The opencv detector only works if the inputs are normalized, so do it here if it was not done before.
      // - If the images are already normalized most of the data will be in the 0-1 range.
      bool opencv_normalize = stereo_settings().skip_image_normalization;
      if (opencv_normalize)
        vw_out() << "Normalizing OpenCV images...\n";

      bool build_opencv_descriptors = true;
      vw::ip::OpenCvInterestPointDetector detector(cv_method, opencv_normalize, build_opencv_descriptors, points_per_tile);

      vw_out() << "\t    Processing left image" << std::endl;
      if ( boost::math::isnan(nodata1) )
        ip1 = detect_interest_points( image1.impl(), detector );
      else
        ip1 = detect_interest_points( apply_mask(create_mask_less_or_equal(image1.impl(),nodata1)), detector );
      vw_out() << "\t    Processing right image" << std::endl;
      if ( boost::math::isnan(nodata2) )
        ip2 = detect_interest_points( image2.impl(), detector );
      else
        ip2 = detect_interest_points( apply_mask(create_mask_less_or_equal(image2.impl(),nodata2)), detector );
    } // End OpenCV case

    sw.stop();
    vw_out(DebugMessage,"asp") << "Detect interest points elapsed time: "
			       << sw.elapsed_seconds() << " s." << std::endl;

    //// DEBUG - Draw out the point matches pre-geometric filtering
    //vw_out() << "\t    Writing IP debug images! " << std::endl;
    //write_point_image("InterestPointMatching__ip_detect_debug1.tif", image1, ip1);
    //write_point_image("InterestPointMatching__ip_detect_debug2.tif", image2, ip2);


    sw.start();

    vw_out() << "\t    Removing IP near nodata" << std::endl;
    const int NODATA_RADIUS = 4;
    if ( !boost::math::isnan(nodata1) )
      remove_ip_near_nodata( image1.impl(), nodata1, ip1, NODATA_RADIUS );

    if ( !boost::math::isnan(nodata2) )
      remove_ip_near_nodata( image2.impl(), nodata2, ip2, NODATA_RADIUS );

    sw.stop();
    vw_out(DebugMessage,"asp") << "Remove IP elapsed time: "
			       << sw.elapsed_seconds() << " s." << std::endl;

    sw.start();

    // For the two OpenCV options we already built the descriptors, so only do this for the integral method.
    if (detect_method == DETECT_IP_METHOD_INTEGRAL) {
      vw_out() << "\t    Building descriptors" << std::endl;
      ip::SGradDescriptorGenerator descriptor;
      if ( boost::math::isnan(nodata1) )
        describe_interest_points( image1.impl(), descriptor, ip1 );
      else
        describe_interest_points( apply_mask(create_mask_less_or_equal(image1.impl(),nodata1)), descriptor, ip1 );
      if ( boost::math::isnan(nodata2) )
        describe_interest_points( image2.impl(), descriptor, ip2 );
      else
        describe_interest_points( apply_mask(create_mask_less_or_equal(image2.impl(),nodata2)), descriptor, ip2 );

      vw_out(DebugMessage,"asp") << "Building descriptors elapsed time: "
				 << sw.elapsed_seconds() << " s." << std::endl;
    }

    vw_out() << "\t    Found interest points:\n" << "\t      left: " << ip1.size() << std::endl;
    vw_out() << "\t     right: " << ip2.size() << std::endl;
  }

  // Detect and Match Interest Points
  //
  // This is not meant to be used directly. Please use ip matching
  template <class Image1T, class Image2T>
  void detect_match_ip( std::vector<vw::ip::InterestPoint>& matched_ip1,
			std::vector<vw::ip::InterestPoint>& matched_ip2,
			vw::ImageViewBase<Image1T> const& image1,
			vw::ImageViewBase<Image2T> const& image2,
			int ip_per_tile,
			double nodata1,
			double nodata2) {
    using namespace vw;

    // Detect Interest Points
    ip::InterestPointList ip1, ip2;
    detect_ip( ip1, ip2, image1.impl(), image2.impl(),
	       ip_per_tile, nodata1, nodata2 );

    // Match the interset points using the default matcher
    vw_out() << "\t--> Matching interest points\n";

    // Replace the IP lists with IP vectors
    std::vector<vw::ip::InterestPoint> ip1_copy, ip2_copy;
    ip1_copy.reserve( ip1.size() );
    ip2_copy.reserve( ip2.size() );
    std::copy( ip1.begin(), ip1.end(), std::back_inserter( ip1_copy ) );
    std::copy( ip2.begin(), ip2.end(), std::back_inserter( ip2_copy ) );

    DetectIpMethod detect_method = static_cast<DetectIpMethod>(stereo_settings().ip_matching_method);

    // Best point must be closer than the next best point
    vw_out() << "Uniqueness threshold: " << stereo_settings().ip_uniqueness_thresh << "\n";
    const double uniqueness_threshold = (0.8/0.7)*stereo_settings().ip_uniqueness_thresh;  // adj
    
    if (detect_method != DETECT_IP_METHOD_ORB) {
      // For all L2Norm distance metrics
      ip::InterestPointMatcher<ip::L2NormMetric,ip::NullConstraint> matcher(uniqueness_threshold);
      matcher( ip1_copy, ip2_copy, matched_ip1, matched_ip2,
	       TerminalProgressCallback( "asp", "\t   Matching: " ));
    }
    else {
      // For Hamming distance metrics
      ip::InterestPointMatcher<ip::HammingMetric,ip::NullConstraint> matcher(uniqueness_threshold);
      matcher( ip1_copy, ip2_copy, matched_ip1, matched_ip2,
	       TerminalProgressCallback( "asp", "\t   Matching: " ));
    }

    ip::remove_duplicates( matched_ip1, matched_ip2 );

    //// DEBUG - Draw out the point matches pre-geometric filtering
    //vw_out() << "\t    Writing IP debug image! " << std::endl;
    //write_match_image("InterestPointMatching__ip_matching_debug.tif",
    //                  image1, image2,
    //                  matched_ip1, matched_ip2);

    vw_out() << "\n\t    Matched points: " << matched_ip1.size() << std::endl;
  }

  // Homography IP matching
  //
  // This applies only the homography constraint. Not the best...
  template <class Image1T, class Image2T>
  bool homography_ip_matching( vw::ImageViewBase<Image1T> const& image1,
			       vw::ImageViewBase<Image2T> const& image2,
			       int ip_per_tile,
			       std::string const& output_name,
			       int inlier_threshold,
			       double nodata1,
			       double nodata2 ) {

    using namespace vw;

    std::vector<ip::InterestPoint> matched_ip1, matched_ip2;
    detect_match_ip( matched_ip1, matched_ip2,
		     image1.impl(), image2.impl(),
		     ip_per_tile,
		     nodata1, nodata2 );
    if ( matched_ip1.size() == 0 || matched_ip2.size() == 0 )
      return false;
    std::vector<Vector3> ransac_ip1 = iplist_to_vectorlist(matched_ip1),
			 ransac_ip2 = iplist_to_vectorlist(matched_ip2);
    std::vector<size_t> indices;
    try {
      typedef math::RandomSampleConsensus<math::HomographyFittingFunctor, math::InterestPointErrorMetric> RansacT;
      const int    MIN_NUM_OUTPUT_INLIERS = ransac_ip1.size()/2;
      const int    NUM_ITERATIONS         = 100;
      RansacT ransac( math::HomographyFittingFunctor(),
		      math::InterestPointErrorMetric(), NUM_ITERATIONS,
		      inlier_threshold,
		      MIN_NUM_OUTPUT_INLIERS, true
		      );
      Matrix<double> H(ransac(ransac_ip2,ransac_ip1)); // 2 then 1 is used here for legacy reasons
      vw_out() << "\t--> Homography: " << H << "\n";
      indices = ransac.inlier_indices(H,ransac_ip2,ransac_ip1);
    } catch (const math::RANSACErr& e ) {
      vw_out() << "RANSAC Failed: " << e.what() << "\n";
      return false;
    }

    std::vector<ip::InterestPoint> final_ip1, final_ip2;
    BOOST_FOREACH( size_t& index, indices ) {
      final_ip1.push_back(matched_ip1[index]);
      final_ip2.push_back(matched_ip2[index]);
    }


    //// DEBUG - Draw out the point matches pre-geometric filtering
    //vw_out() << "\t    Writing IP debug image2! " << std::endl;
    //write_match_image("InterestPointMatching__ip_matching_debug2.tif",
    //                  image1, image2,
    //                  final_ip1, final_ip2);

    vw_out() << "\t    * Writing match file: " << output_name << "\n";
    ip::write_binary_match_file(output_name, final_ip1, final_ip2);
    return true;
  }

  // Smart IP matching that uses clustering on triangulation and
  // datum information to determine inliers.
  //
  // Left and Right TX define transforms that have been performed on
  // the images that that camera data doesn't know about. (ie scaling).
  template <class Image1T, class Image2T>
  bool ip_matching( bool single_threaded_camera,
		    vw::camera::CameraModel* cam1,
		    vw::camera::CameraModel* cam2,
		    vw::ImageViewBase<Image1T> const& image1,
		    vw::ImageViewBase<Image2T> const& image2,
		    int ip_per_tile,
		    vw::cartography::Datum const& datum,
		    std::string const& output_name,
		    double epipolar_threshold,
		    double uniqueness_threshold,
		    double nodata1,
		    double nodata2,
		    vw::TransformRef const& left_tx,
		    vw::TransformRef const& right_tx,
		    bool transform_to_original_coord) {
    using namespace vw;

    // Detect interest points
    ip::InterestPointList ip1, ip2;
    detect_ip( ip1, ip2, image1.impl(), image2.impl(),
	       ip_per_tile,
	       nodata1, nodata2 );
    if ( ip1.size() == 0 || ip2.size() == 0 ){
      vw_out() << "Unable to detect interest points." << std::endl;
      return false;
    }

    // Match interest points forward/backward .. constraining on epipolar line
    DetectIpMethod detect_method = static_cast<DetectIpMethod>(stereo_settings().ip_matching_method);
    std::vector<size_t> forward_match, backward_match;
    vw_out() << "\t--> Matching interest points" << std::endl;

    vw_out() << "Uniqueness threshold: " << uniqueness_threshold << "\n";
    vw_out() << "Inlier threshold:     " << epipolar_threshold   << "\n";
    
    EpipolarLinePointMatcher matcher(single_threaded_camera,
				     uniqueness_threshold, epipolar_threshold, datum );
    vw_out() << "\t    Matching Forward" << std::endl;
    matcher( ip1, ip2, detect_method, cam1, cam2, left_tx, right_tx, forward_match );
    vw_out() << "\t    ---> Obtained " << forward_match.size() << " matches." << std::endl;
    vw_out() << "\t    Matching Backward" << std::endl;
    matcher( ip2, ip1, detect_method, cam2, cam1, right_tx, left_tx, backward_match );
    vw_out() << "\t    ---> Obtained " << backward_match.size() << " matches." << std::endl;

    // Perform circle consistency check
    size_t valid_count = 0;
    const size_t NULL_INDEX = (size_t)(-1);
    for ( size_t i = 0; i < forward_match.size(); i++ ) {
      if ( forward_match[i] != NULL_INDEX ) {
        if ( backward_match[forward_match[i]] != i ) {
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
    std::vector<ip::InterestPoint> matched_ip1, matched_ip2;
    matched_ip1.reserve( valid_count ); // Get our allocations out of the way.
    matched_ip2.reserve( valid_count );
    {
      ip::InterestPointList::const_iterator ip1_it = ip1.begin(), ip2_it = ip2.begin();
      for ( size_t i = 0; i < forward_match.size(); i++ ) {
        if ( forward_match[i] != NULL_INDEX ) {
          matched_ip1.push_back( *ip1_it );
          ip2_it = ip2.begin();
          std::advance( ip2_it, forward_match[i] );
          matched_ip2.push_back( *ip2_it );
        }
        ip1_it++;
      }
    }

    //// DEBUG - Draw out the point matches pre-geometric filtering
    //vw_out() << "\t    Writing IP debug image! " << std::endl;
    //write_match_image("InterestPointMatching__ip_matching_debug.tif",
    //                  image1, image2,
    //                  matched_ip1, matched_ip2);

    // Apply filtering of IP by a selection of assumptions. Low
    // triangulation error, agreement with klt tracking, and local
    // neighbors are the same neighbors in both images.
    std::list<size_t> good_indices;
    for ( size_t i = 0; i < matched_ip1.size(); i++ ) {
      good_indices.push_back(i);
    }
    if (!tri_ip_filtering( matched_ip1, matched_ip2,
			   cam1, cam2, good_indices, left_tx, right_tx ) ){
      vw_out() << "No interest points left after triangulation filtering." << std::endl;
      return false;
    }
    if (!stddev_ip_filtering( matched_ip1, matched_ip2, good_indices ) ) {
      vw_out() << "No interest points left after stddev filtering." << std::endl;
      return false;
    }

    // Record new list that contains only the inliers.
    vw_out() << "\t    Reduced matches to " << good_indices.size() << "\n";
    std::vector<ip::InterestPoint> buffer( good_indices.size() );

    // Subselect, Transform, Copy, Matched Ip1
    size_t w_index = 0;
    BOOST_FOREACH( size_t index, good_indices ) {
      Vector2 l( matched_ip1[index].x, matched_ip1[index].y );
      if ( transform_to_original_coord )
        l = left_tx.reverse( l );
      matched_ip1[index].ix = matched_ip1[index].x = l.x();
      matched_ip1[index].iy = matched_ip1[index].y = l.y();
      buffer[w_index] = matched_ip1[index];
      w_index++;
    }
    matched_ip1 = buffer;

    // Subselect, Transform, Copy, Matched ip2
    w_index = 0;
    BOOST_FOREACH( size_t index, good_indices ) {
      Vector2 r( matched_ip2[index].x, matched_ip2[index].y );
      if ( transform_to_original_coord )
        r = right_tx.reverse( r );
      matched_ip2[index].ix = matched_ip2[index].x = r.x();
      matched_ip2[index].iy = matched_ip2[index].y = r.y();
      buffer[w_index] = matched_ip2[index];
      w_index++;
    }
    matched_ip2 = buffer;

    if (stereo_settings().elevation_limit[0] < stereo_settings().elevation_limit[1] ||
	!stereo_settings().lon_lat_limit.empty()) {

      vw::TransformRef local_left_tx = vw::TransformRef(vw::TranslateTransform(0,0));
      vw::TransformRef local_right_tx = vw::TransformRef(vw::TranslateTransform(0,0));
      if (!transform_to_original_coord) {
	// The transform to original coords was not applied, need to take it into account.
	local_left_tx  = left_tx;
	local_right_tx = right_tx;
      }

      std::vector<ip::InterestPoint> matched_ip1_out, matched_ip2_out;
      double ip_scale = 1.0; // left_tx and right_tx already have any scale info
      filter_ip_by_lonlat_and_elevation(cam1, cam2,  
					datum,  matched_ip1, matched_ip2, 
					local_left_tx, local_right_tx,  ip_scale,  
					stereo_settings().elevation_limit,  
					stereo_settings().lon_lat_limit,  
					matched_ip1_out, matched_ip2_out);
      matched_ip1 = matched_ip1_out;
      matched_ip2 = matched_ip2_out;
    }
    
    vw_out() << "\t    * Writing match file: " << output_name << "\n";
    ip::write_binary_match_file( output_name, matched_ip1, matched_ip2 );

    //// DEBUG - Draw out the point matches pre-geometric filtering
    //vw_out() << "\t    Writing IP debug image! " << std::endl;
    //write_match_image("InterestPointMatching__ip_matching_debug2.tif",
    //                  image1, image2,
    //                  matched_ip1, matched_ip2);

    return true;
  }

  template <class Image1T, class Image2T>
  bool ip_matching_w_alignment( bool single_threaded_camera,
				vw::camera::CameraModel* cam1,
				vw::camera::CameraModel* cam2,
				vw::ImageViewBase<Image1T> const& image1,
				vw::ImageViewBase<Image2T> const& image2,
				int ip_per_tile,
				vw::cartography::Datum const& datum,
				std::string const& output_name,
				double epipolar_threshold,
				double uniqueness_threshold,
				double nodata1,
				double nodata2,
				vw::TransformRef const& left_tx,
				vw::TransformRef const& right_tx ) {

    using namespace vw;

    VW_OUT( DebugMessage, "asp" ) << "Performing IP matching with alignment " << std::endl;

    BBox2i box1 = bounding_box(image1.impl()), box2 = bounding_box(image2.impl());

    Matrix<double> rough_homography;
    try {
      // Homography is defined in the original camera coordinates
      rough_homography =  rough_homography_fit( cam1, cam2, left_tx.reverse_bbox(box1),
						right_tx.reverse_bbox(box2), datum );
    } catch(...) {
      VW_OUT( DebugMessage, "asp" ) << "Rough homography fit failed, trying with identity transform. " << std::endl;
      rough_homography.set_identity(3);
    }

    // Remove the main translation and solve for BBox that fits the
    // image. If we used the translation from the solved homography with
    // poorly position cameras, the right image might be moved out of frame.
    rough_homography(0,2) = rough_homography(1,2) = 0;
    VW_OUT( DebugMessage, "asp" ) << "Aligning right to left for IP capture using rough homography: " << rough_homography << std::endl;

    { // Check to see if this rough homography works
      HomographyTransform func( rough_homography );
      VW_ASSERT( box1.intersects( func.forward_bbox( box2 ) ),
		 LogicErr() << "The rough homography alignment based on datum and camera geometry shows that input images do not overlap at all. Unable to proceed. Examine your images, or consider using the option --skip-rough-homography.\n" );
    }

    TransformRef tx( compose(right_tx, HomographyTransform(rough_homography)) );
    BBox2i raster_box = tx.forward_bbox( right_tx.reverse_bbox(box2) );
    tx = TransformRef(compose(TranslateTransform(-raster_box.min()),
			      right_tx, HomographyTransform(rough_homography)));
    raster_box -= Vector2i(raster_box.min());


    // With a transform applied to the input images, try to match interest points between them.
    // - It is important that we use NearestPixelInterpolation in the
    //   next step. Using anything else will interpolate nodata values
    //   and stop them from being masked out.
    bool inlier =
      ip_matching( single_threaded_camera,
		   cam1, cam2, image1.impl(),
		   crop(transform(image2.impl(), compose(tx, inverse(right_tx)),
				  ValueEdgeExtension<typename Image2T::pixel_type>(boost::math::isnan(nodata2) ? 0 : nodata2),
				  NearestPixelInterpolation()), raster_box),
		   ip_per_tile,
		   datum, output_name, epipolar_threshold, uniqueness_threshold,
		   nodata1, nodata2, left_tx, tx );
    if (!inlier)
      return inlier;

    // For some reason ip_matching writes its points to file instead
    // of returning them, so read them from disk.
    std::vector<ip::InterestPoint> ip1_copy, ip2_copy;
    ip::read_binary_match_file( output_name, ip1_copy, ip2_copy );

    // Use the interest points that we found to compute an aligning
    // homography transform for the two images
    bool adjust_left_image_size = true;
    Matrix<double> matrix1, matrix2;
    homography_rectification( adjust_left_image_size,
			      raster_box.size(), raster_box.size(),
			      ip1_copy, ip2_copy, matrix1, matrix2 );
    if ( sum(abs(submatrix(rough_homography,0,0,2,2) - submatrix(matrix2,0,0,2,2))) > 4 ) {
      vw_out() << "Post homography has largely different scale and skew from rough fit. Post solution is " << matrix2 << ". Examine your images, or consider using the option --skip-rough-homography.\n";
      //return false;
    }

    return inlier;
  }

  // Do IP matching, return, the best translation+scale fitting functor.
  vw::Matrix<double> translation_ip_matching(vw::ImageView<float> const& image1,
                                             vw::ImageView<float> const& image2,
                                             int ip_per_tile,
                                             double nodata1, double nodata2);
  
} // End namespace asp

#endif//__ASP_CORE_INTEREST_POINT_MATCHING_H__
