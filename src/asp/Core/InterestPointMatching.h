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

#include <asp/Core/IntegralAutoGainDetector.h>

#include <vw/Core.h>
#include <vw/Core/Stopwatch.h>
#include <vw/Math.h>
#include <vw/Image/ImageViewBase.h>
#include <vw/Image/MaskViews.h>
#include <vw/Camera/CameraModel.h>
#include <vw/InterestPoint/Matcher.h>
#include <vw/InterestPoint/InterestData.h>
#include <vw/Cartography/Datum.h>

#include <boost/foreach.hpp>
#include <boost/math/special_functions/fpclassify.hpp>

namespace asp {

  // Takes interest points and then finds the nearest 10 matches. It
  // filters them by whom are closest to the epipolar line via a
  // threshold. The remaining 2 or then selected to be a match if
  // their distance meets the other threshold.
  class EpipolarLinePointMatcher {
    double m_threshold, m_epipolar_threshold;
    vw::cartography::Datum m_datum;

  public:
    EpipolarLinePointMatcher( double threshold, double epipolar_threshold,
                              vw::cartography::Datum const& datum );

    // This only returns the indicies
    void operator()( vw::ip::InterestPointList const& ip1,
                     vw::ip::InterestPointList const& ip2,
                     vw::camera::CameraModel* cam1,
                     vw::camera::CameraModel* cam2,
                     vw::TransformRef const& tx1,
                     vw::TransformRef const& tx2,
                     std::vector<size_t>& output_indices ) const;

    // Work out an epipolar line from interest point. Returns the
    // coefficients for the following line equation: ax + by + c = 0
    static vw::Vector3 epipolar_line( vw::Vector2 const& feature,
                                      vw::cartography::Datum const& datum,
                                      vw::camera::CameraModel* cam_ip,
                                      vw::camera::CameraModel* cam_obj );

    // Calculate distance between a line of equation ax + by + c = 0
    static double distance_point_line( vw::Vector3 const& line,
                                       vw::Vector2 const& point );

    friend class EpipolarLineMatchTask;
  };

  // Tool to remove points on or within 1 px of nodata pixels.
  // Note: A nodata pixel is one for which pixel <= nodata.
  template <class ImageT>
  void remove_ip_near_nodata( vw::ImageViewBase<ImageT> const& image,
                              double nodata,
                              vw::ip::InterestPointList& ip_list ){

    using namespace vw;
    size_t prior_ip = ip_list.size();

    typedef ImageView<typename ImageT::pixel_type> CropImageT;
    CropImageT subsection(3,3);

    BBox2i bound = bounding_box( image.impl() );
    bound.contract(1);
    for ( ip::InterestPointList::iterator ip = ip_list.begin();
          ip != ip_list.end(); ++ip ) {
      if ( !bound.contains( Vector2i(ip->ix,ip->iy) ) ) {
        ip = ip_list.erase(ip);
        ip--;
        continue;
      }

      subsection =
        crop( image.impl(), ip->ix-1, ip->iy-1, 3, 3 );
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
                                  << std::endl << "Nodata value used "
                                  << nodata << std::endl;
  }

  // Find a rough homography that maps right to left using the camera
  // and datum information.
  vw::Matrix<double>
  rough_homography_fit( vw::camera::CameraModel* cam1,
                        vw::camera::CameraModel* cam2,
                        vw::BBox2i const& box1, vw::BBox2i const& box2,
                        vw::cartography::Datum const& datum );

  // Homography rectification that aligns the right image to the left
  // image via a homography transform. It returns a vector2i of the
  // ideal cropping size to use for the left and right image. The left
  // transform is actually just a translation that sets origin to the
  // shared corner of left and right.
  vw::Vector2i
  homography_rectification( vw::Vector2i const& left_size,
                            vw::Vector2i const& right_size,
                            std::vector<vw::ip::InterestPoint> const& left_ip,
                            std::vector<vw::ip::InterestPoint> const& right_ip,
                            vw::Matrix<double>& left_matrix,
                            vw::Matrix<double>& right_matrix );

  // Detect InterestPoints
  //
  // This is not meant to be used directly. Please use ip_matching or
  // the dumb homography ip matching.
  template <class List1T, class List2T, class Image1T, class Image2T>
  void detect_ip( List1T& ip1, List2T& ip2,
                  vw::ImageViewBase<Image1T> const& image1,
                  vw::ImageViewBase<Image2T> const& image2,
                  double nodata1 = std::numeric_limits<double>::quiet_NaN(),
                  double nodata2 = std::numeric_limits<double>::quiet_NaN() ) {
    using namespace vw;
    BBox2i box1 = bounding_box(image1.impl());
    ip1.clear();
    ip2.clear();

    Stopwatch sw;
    sw.start();

    // Detect Interest Points
    float number_boxes = (box1.width() / 1024.f) * (box1.height() / 1024.f);
    size_t points_per_tile = 5000.f / number_boxes;
    if ( points_per_tile > 5000 ) points_per_tile = 5000;
    if ( points_per_tile < 50 ) points_per_tile = 50;
    VW_OUT( DebugMessage, "asp" ) << "Setting IP code to search " << points_per_tile << " IP per tile (1024^2 px).\n";
    asp::IntegralAutoGainDetector detector( points_per_tile );
    vw_out() << "\t    Processing Left" << std::endl;
    if ( boost::math::isnan(nodata1) )
      ip1 = detect_interest_points( image1.impl(), detector );
    else
      ip1 = detect_interest_points( apply_mask(create_mask_less_or_equal(image1.impl(),nodata1)), detector );
    vw_out() << "\t    Processing Right" << std::endl;
    if ( boost::math::isnan(nodata2) )
      ip2 = detect_interest_points( image2.impl(), detector );
    else
      ip2 = detect_interest_points( apply_mask(create_mask_less_or_equal(image2.impl(),nodata2)), detector );

    sw.stop();
    vw_out(DebugMessage,"asp") << "Detect interest points elapsed time: "
                               << sw.elapsed_seconds() << " s." << std::endl;

    sw.start();

    vw_out() << "\t    Removing IP near nodata" << std::endl;
    if ( !boost::math::isnan(nodata1) )
      remove_ip_near_nodata( image1.impl(), nodata1, ip1 );

    if ( !boost::math::isnan(nodata2) )
      remove_ip_near_nodata( image2.impl(), nodata2, ip2 );

    sw.stop();
    vw_out(DebugMessage,"asp") << "Remove IP elapsed time: "
                               << sw.elapsed_seconds() << " s." << std::endl;

    sw.start();

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

    vw_out() << "\t    Found interest points:\n"
             << "\t      left: " << ip1.size() << std::endl;
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
                        double nodata1 = std::numeric_limits<double>::quiet_NaN(),
                        double nodata2 = std::numeric_limits<double>::quiet_NaN() ) {
    using namespace vw;

    // Detect Interest Points
    ip::InterestPointList ip1, ip2;
    detect_ip( ip1, ip2, image1.impl(), image2.impl(), nodata1, nodata2 );

    // Match the interset points using the default matcher
    vw_out() << "\t--> Matching interest points\n";
    ip::InterestPointMatcher<ip::L2NormMetric,ip::NullConstraint> matcher(0.5);
    std::vector<ip::InterestPoint> ip1_copy, ip2_copy;
    ip1_copy.reserve( ip1.size() );
    ip2_copy.reserve( ip2.size() );
    std::copy( ip1.begin(), ip1.end(), std::back_inserter( ip1_copy ) );
    std::copy( ip2.begin(), ip2.end(), std::back_inserter( ip2_copy ) );
    matcher( ip1_copy, ip2_copy, matched_ip1, matched_ip2,
             TerminalProgressCallback( "asp", "\t   Matching: " ));
    ip::remove_duplicates( matched_ip1, matched_ip2 );
    vw_out() << "\t    Matched points: " << matched_ip1.size() << std::endl;
  }

  // Homography IP matching
  //
  // This applies only the homography constraint. Not the best...
  template <class Image1T, class Image2T>
  bool homography_ip_matching( vw::ImageViewBase<Image1T> const& image1,
                               vw::ImageViewBase<Image2T> const& image2,
                               std::string const& output_name,
                               double nodata1 = std::numeric_limits<double>::quiet_NaN(),
                               double nodata2 = std::numeric_limits<double>::quiet_NaN() ) {

    using namespace vw;

    std::vector<ip::InterestPoint> matched_ip1, matched_ip2;
    detect_match_ip( matched_ip1, matched_ip2,
                     image1.impl(), image2.impl(),
                     nodata1, nodata2 );
    if ( matched_ip1.size() == 0 || matched_ip2.size() == 0 )
      return false;
    std::vector<Vector3> ransac_ip1 = iplist_to_vectorlist(matched_ip1),
      ransac_ip2 = iplist_to_vectorlist(matched_ip2);
    std::vector<size_t> indices;
    try {
      typedef math::RandomSampleConsensus<math::HomographyFittingFunctor, math::InterestPointErrorMetric> RansacT;
      RansacT ransac( math::HomographyFittingFunctor(),
                      math::InterestPointErrorMetric(), 100,
                      norm_2(Vector2(bounding_box(image1.impl()).size()))/100.0,
                      ransac_ip1.size()/2, true
                      );
      Matrix<double> H(ransac(ransac_ip1,ransac_ip2));
      vw_out() << "\t--> Homography: " << H << "\n";
      indices = ransac.inlier_indices(H,ransac_ip1,ransac_ip2);
    } catch (const math::RANSACErr& e ) {
      vw_out() << "RANSAC Failed: " << e.what() << "\n";
      return false;
    }

    std::vector<ip::InterestPoint> final_ip1, final_ip2;
    BOOST_FOREACH( size_t& index, indices ) {
      final_ip1.push_back(matched_ip1[index]);
      final_ip2.push_back(matched_ip2[index]);
    }

    ip::write_binary_match_file(output_name, final_ip1, final_ip2);
    return true;
  }

  // IP matching that uses clustering on triangulation error to determine inliers.
  // Check output this filter can fail.
  //
  // Input and output is the valid indices. Valid indices must have something to start with.
  bool
  tri_ip_filtering( std::vector<vw::ip::InterestPoint> const& ip1,
                    std::vector<vw::ip::InterestPoint> const& ip2,
                    vw::camera::CameraModel* cam1,
                    vw::camera::CameraModel* cam2,
                    std::list<size_t>& valid_indices,
                    vw::TransformRef const& left_tx = vw::TransformRef(vw::TranslateTransform(0,0)),
                    vw::TransformRef const& right_tx = vw::TransformRef(vw::TranslateTransform(0,0)) );

  bool
  stddev_ip_filtering( std::vector<vw::ip::InterestPoint> const& ip1,
                       std::vector<vw::ip::InterestPoint> const& ip2,
                       std::list<size_t>& valid_indices );

  // Smart IP matching that uses clustering on triangulation and
  // datum information to determine inliers.
  //
  // Left and Right TX define transforms that have been performed on
  // the images that that camera data doesn't know about. (ie
  // scaling).
  template <class Image1T, class Image2T>
  bool ip_matching( vw::camera::CameraModel* cam1,
                    vw::camera::CameraModel* cam2,
                    vw::ImageViewBase<Image1T> const& image1,
                    vw::ImageViewBase<Image2T> const& image2,
                    vw::cartography::Datum const& datum,
                    std::string const& output_name,
                    double nodata1 = std::numeric_limits<double>::quiet_NaN(),
                    double nodata2 = std::numeric_limits<double>::quiet_NaN(),
                    vw::TransformRef const& left_tx = vw::TransformRef(vw::TranslateTransform(0,0)),
                    vw::TransformRef const& right_tx = vw::TransformRef(vw::TranslateTransform(0,0)),
                    bool transform_to_original_coord = true ) {
    using namespace vw;

    // Detect interest points
    ip::InterestPointList ip1, ip2;
    detect_ip( ip1, ip2, image1.impl(), image2.impl(),
               nodata1, nodata2 );
    if ( ip1.size() == 0 || ip2.size() == 0 ){
      vw_out() << "Unable to detect interest points." << std::endl;
      return false;
    }

    // Match interest points forward/backward .. constraining on epipolar line
    std::vector<size_t> forward_match, backward_match;
    vw_out() << "\t--> Matching interest points" << std::endl;
    EpipolarLinePointMatcher matcher( 0.5, norm_2(Vector2(image1.impl().cols(),image1.impl().rows()))/20, datum );
    vw_out() << "\t    Matching Forward" << std::endl;
    matcher( ip1, ip2, cam1, cam2, left_tx, right_tx, forward_match );
    vw_out() << "\t    Matching Backward" << std::endl;
    matcher( ip2, ip1, cam2, cam1, right_tx, left_tx, backward_match );

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
    if (!stddev_ip_filtering( matched_ip1, matched_ip2,
                              good_indices ) ) {
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

    ip::write_binary_match_file( output_name, matched_ip1, matched_ip2 );

    return true;
  }

  // Calls ip matching above but with an additional step where we
  // apply a homogrpahy to make right image like left image. This is
  // useful so that both images have similar scale and similar affine qualities.
  template <class Image1T, class Image2T>
  bool ip_matching_w_alignment( vw::camera::CameraModel* cam1,
                                vw::camera::CameraModel* cam2,
                                vw::ImageViewBase<Image1T> const& image1,
                                vw::ImageViewBase<Image2T> const& image2,
                                vw::cartography::Datum const& datum,
                                std::string const& output_name,
                                double nodata1 = std::numeric_limits<double>::quiet_NaN(),
                                double nodata2 = std::numeric_limits<double>::quiet_NaN(),
                                vw::TransformRef const& left_tx = vw::TransformRef(vw::TranslateTransform(0,0)),
                                vw::TransformRef const& right_tx = vw::TransformRef(vw::TranslateTransform(0,0)) ) {

    using namespace vw;
    BBox2i box1 = bounding_box(image1.impl()), box2 = bounding_box(image2.impl());

    // Homography is defined in the original camera coordinates
    Matrix<double> rough_homography =
      rough_homography_fit( cam1, cam2, left_tx.reverse_bbox(box1),
                            right_tx.reverse_bbox(box2), datum );

    // Remove the main translation and solve for BBox that fits the
    // image. If we used the translation from the solved homography with
    // poorly position cameras, the right image might be moved out of
    // frame.
    rough_homography(0,2) = rough_homography(1,2) = 0;
    VW_OUT( DebugMessage, "asp" ) << "Aligning right to left for IP capture using rough homography: " << rough_homography << std::endl;

    { // Check to see if this rough homography works
      HomographyTransform func( rough_homography );
      VW_ASSERT( box1.intersects( func.forward_bbox( box2 ) ),
                 LogicErr() << "The rough homography alignment based on datum and camera geometry shows that input images do not overlap at all. Unable to proceed.\n" );
    }

    TransformRef tx( compose(right_tx, HomographyTransform(rough_homography)) );
    BBox2i raster_box = tx.forward_bbox( right_tx.reverse_bbox(box2) );
    tx = TransformRef(compose(TranslateTransform(-raster_box.min()),
                              right_tx, HomographyTransform(rough_homography)));
    raster_box -= Vector2i(raster_box.min());

    // It is important that we use NearestPixelInterpolation in the
    // next step. Using anything else will interpolate nodata values
    // and stop them from being masked out.
    bool inlier =
      ip_matching( cam1, cam2, image1.impl(),
                   crop(transform(image2.impl(), compose(tx, inverse(right_tx)),
                                  ValueEdgeExtension<typename Image2T::pixel_type>(boost::math::isnan(nodata2) ? 0 : nodata2),
                                  NearestPixelInterpolation()), raster_box),
                   datum, output_name, nodata1, nodata2, left_tx, tx );

    std::vector<ip::InterestPoint> ip1_copy, ip2_copy;
    ip::read_binary_match_file( output_name, ip1_copy, ip2_copy );
    Matrix<double> matrix1, matrix2;
    homography_rectification( raster_box.size(), raster_box.size(),
                              ip1_copy, ip2_copy, matrix1, matrix2 );
    if ( sum(abs(submatrix(rough_homography,0,0,2,2) - submatrix(matrix2,0,0,2,2))) > 4 ) {
      VW_OUT( DebugMessage, "asp" ) << "Post homography has largely different scale and skew from rough fit. Post solution is " << matrix2 << "\n";
      exit(0);
      return false;
    }

    return inlier;
  }

}

#endif//__ASP_CORE_INTEREST_POINT_MATCHING_H__
