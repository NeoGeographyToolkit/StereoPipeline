// __BEGIN_LICENSE__
// Copyright (C) 2006-2011 United States Government as represented by
// the Administrator of the National Aeronautics and Space Administration.
// All Rights Reserved.
// __END_LICENSE__

#ifndef __ASP_CORE_INTEREST_POINT_MATCHING_H__
#define __ASP_CORE_INTEREST_POINT_MATCHING_H__

#include <asp/Core/GaussianClustering.h>
#include <asp/Core/IntegralAutoGainDetector.h>

#include <vw/Core.h>
#include <vw/Math.h>
#include <vw/Image/ImageViewBase.h>
#include <vw/Image/MaskViews.h>
#include <vw/Camera/CameraModel.h>
#include <vw/Cartography/Datum.h>
#include <vw/InterestPoint/Matcher.h>
#include <vw/InterestPoint/InterestData.h>
#include <vw/Stereo/StereoModel.h>

#include <boost/foreach.hpp>
#include <boost/math/special_functions/fpclassify.hpp>

namespace asp {

  // Interesect a pixel against the datum
  vw::Vector3 datum_intersection( vw::cartography::Datum const& datum,
                                  vw::camera::CameraModel* model,
                                  vw::Vector2 const& pix );

  // Find a rough homography that maps right to left using the camera
  // and datum information.
  vw::Matrix<double>
  rough_homography_fit( vw::camera::CameraModel* cam1,
                        vw::camera::CameraModel* cam2,
                        vw::BBox2i const& box1, vw::BBox2i const& box2,
                        vw::cartography::Datum const& datum );

  // Homography fit to interest points
  vw::Matrix<double>
  homography_fit( std::vector<vw::ip::InterestPoint> const& ip1,
                  std::vector<vw::ip::InterestPoint> const& ip2,
                  vw::BBox2i const& image_size );

  // Tool to remove points on or within 1 px of nodata
  template <class ImageT>
  void remove_ip_near_nodata( vw::ImageViewBase<ImageT> const& image_base,
                              double nodata,
                              vw::ip::InterestPointList& ip_list ){
    using namespace vw;
    ImageT image = image_base.impl();
    size_t prior_ip = ip_list.size();

    typedef ImageView<typename ImageT::pixel_type> CropImageT;
    CropImageT subsection(3,3);

    BBox2i bound = bounding_box( image );
    bound.contract(1);
    for ( ip::InterestPointList::iterator ip = ip_list.begin();
          ip != ip_list.end(); ++ip ) {
      if ( !bound.contains( Vector2i(ip->ix,ip->iy) ) ) {
        ip = ip_list.erase(ip);
        ip--;
        continue;
      }

      subsection =
        crop( image, ip->ix-1, ip->iy-1, 3, 3 );
      for ( typename CropImageT::iterator pixel = subsection.begin();
            pixel != subsection.end(); pixel++ ) {
        if (*pixel == nodata) {
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

  // Smart IP matching that using clustering on triangulation and
  // datum information to determine inliers.
  //
  // Left and Right TX define transforms that have been performed on
  // the images that that camera data doesn't know about. (ie
  // scaling).
  template <class Image1T, class Image2T>
  bool ip_matching( vw::camera::CameraModel* cam1,
                    vw::camera::CameraModel* cam2,
                    vw::ImageViewBase<Image1T> const& image1_base,
                    vw::ImageViewBase<Image2T> const& image2_base,
                    vw::cartography::Datum const& datum,
                    std::string const& output_name,
                    double nodata1 = std::numeric_limits<double>::quiet_NaN(),
                    double nodata2 = std::numeric_limits<double>::quiet_NaN(),
                    vw::TransformRef const& left_tx = vw::TransformRef(vw::TranslateTransform(0,0)),
                    vw::TransformRef const& right_tx = vw::TransformRef(vw::TranslateTransform(0,0))) {
    using namespace vw;
    Image1T image1 = image1_base.impl();
    Image2T image2 = image2_base.impl();
    BBox2i box1 = bounding_box(image1), box2 = bounding_box(image2);

    // Detect Interest Points
    ip::InterestPointList ip1, ip2;
    float number_boxes = (box1.width() / 1024.f) * (box1.height() / 1024.f);
    size_t points_per_tile = 5000.f / number_boxes;
    if ( points_per_tile > 5000 ) points_per_tile = 5000;
    if ( points_per_tile < 50 ) points_per_tile = 50;
    VW_OUT( DebugMessage, "asp" ) << "Setting IP code to search " << points_per_tile << " IP per tile (1024^2 px).\n";
    asp::IntegralAutoGainDetector detector( points_per_tile );
    vw_out() << "\t    Processing Left\n";
    if ( boost::math::isnan(nodata1) )
      ip1 = detect_interest_points( image1, detector );
    else
      ip1 = detect_interest_points( apply_mask(create_mask(image1,nodata1)), detector );
    vw_out() << "\t    Processing Right\n";
    if ( boost::math::isnan(nodata2) )
      ip2 = detect_interest_points( image2, detector );
    else
      ip2 = detect_interest_points( apply_mask(create_mask(image2,nodata2)), detector );

    if ( !boost::math::isnan(nodata1) )
      remove_ip_near_nodata( image1, nodata1, ip1 );

    if ( !boost::math::isnan(nodata2) )
      remove_ip_near_nodata( image2, nodata2, ip2 );

    vw_out() << "\t    Building Descriptors\n";
    ip::SGradDescriptorGenerator descriptor;
    if ( boost::math::isnan(nodata1) )
      descriptor(image1, ip1 );
    else
      descriptor( apply_mask(create_mask(image1,nodata1)), ip1 );
    if ( boost::math::isnan(nodata2) )
      descriptor(image2, ip2 );
    else
      descriptor( apply_mask(create_mask(image2,nodata2)), ip2 );

    vw_out() << "\t    Found interest points:\n"
             << "\t      left: " << ip1.size() << "\n";
    vw_out() << "\t     right: " << ip2.size() << "\n";

    vw_out() << "\t--> Matching interest points\n";
    ip::InterestPointMatcher<ip::L2NormMetric,ip::NullConstraint> matcher(0.5);
    std::vector<ip::InterestPoint> matched_ip1, matched_ip2, ip1_copy, ip2_copy;
    ip1_copy.reserve( ip1.size() );
    ip2_copy.reserve( ip2.size() );
    BOOST_FOREACH( ip::InterestPoint const& ip, ip1 )
      ip1_copy.push_back(ip);
    BOOST_FOREACH( ip::InterestPoint const& ip, ip2 )
      ip2_copy.push_back(ip);
    matcher( ip1_copy, ip2_copy, matched_ip1, matched_ip2, false,
             TerminalProgressCallback( "asp", "\t   Matching: " ));
    ip::remove_duplicates( matched_ip1, matched_ip2 );
    vw_out() << "\t    Matched points: " << matched_ip1.size() << "\n";

    // Remove IP that don't triangulate well or triangulate well away
    // from the datum.
    std::vector<Vector2> error_samples( matched_ip1.size() );
    std::vector<double> normalized_samples;
    normalized_samples.reserve( matched_ip1.size() );
    Vector2 error_scaling, error_offset;

    // Create the 'error' samples. Which are triangulation error and
    // distance to sphere.
    stereo::StereoModel model( cam1, cam2 );
    for (size_t i = 0; i < matched_ip1.size(); i++ ) {
      Vector3 geodetic =
        datum.cartesian_to_geodetic( model( left_tx.reverse(Vector2( matched_ip1[i].x, matched_ip1[i].y )),
                                            right_tx.reverse(Vector2(matched_ip2[i].x,
                                                               matched_ip2[i].y)),
                                            error_samples[i].x() ) );
      error_samples[i].y() = fabs(geodetic[2]);
    }

    // Find the mean and std deviation error and distance from sphere
    // so that the sample can be combined a norm_2 together without
    // one dimension getting unfair waiting.
    {
      namespace ba = boost::accumulators;
      typedef ba::accumulator_set<double, ba::stats<ba::tag::variance> > acc_set;
      acc_set tri_stat, height_stat;
      BOOST_FOREACH( Vector2 const& sample, error_samples ) {
        tri_stat( sample.x() );
        height_stat( sample.y() );
      }

      // output =  scale * ( input  - offset )
      error_offset = Vector2( ba::mean( tri_stat ), ba::mean( height_stat ) );
      error_scaling = sqrt( Vector2( ba::variance( tri_stat ),
                                     ba::variance( height_stat ) ) );
      error_scaling = elem_quot(sqrt(2.), error_scaling);

      // Apply this scaling to create the normalized samples
      BOOST_FOREACH( Vector2 const& sample, error_samples ) {
        normalized_samples.push_back( norm_2( elem_prod(error_scaling,
                                                        (sample - error_offset ) ) ) );
      }
    }

    std::vector<std::pair<Vector<double>, Vector<double> > > clustering =
      asp::gaussian_clustering< std::vector<double> >( normalized_samples.begin(), normalized_samples.end(), 2 );
    if ( clustering[0].first[0] > clustering[1].first[0] /*Other cluster has lower tri error*/ &&
         clustering[1].second[0] != 0 /*Other cluster has non-zero variance (ie non empty set)*/ )
      std::swap( clustering[0], clustering[1] );

    vw_out() << "\t    Inlier cluster:\n"
             << "\t      Triangulation Err: " << clustering.front().first[0] / error_scaling[0] + error_offset[0]
             << " +- " << sqrt( clustering.front().second[0] / error_scaling[0] ) << " meters\n"
             << "\t      Altitude         : " << clustering.front().first[0] / error_scaling[1] + error_offset[1]
             << " +- " << sqrt( clustering.front().second[0] / error_scaling[1] ) << " meters\n";

    // Record indices of points that match our clustering result
    std::list<size_t> good_indices;
    for (size_t i = 0; i < matched_ip1.size(); i++ ) {
      double scalar1 = 1.0 / sqrt( 2.0 * M_PI * clustering.front().second[0] );
      double scalar2 = 1.0 / sqrt( 2.0 * M_PI * clustering.back().second[0] );
      if ( (scalar1 * exp( (-((normalized_samples[i]-clustering.front().first[0]) *
                              (normalized_samples[i]-clustering.front().first[0]))) /
                           (2 * clustering.front().second[0] ) ) )   >
           (scalar2 * exp( (-((normalized_samples[i]-clustering.back().first[0]) *
                              (normalized_samples[i]-clustering.back().first[0]))) /
                           (2 * clustering.back().second[0] ) ) ) ||
           normalized_samples[i] < clustering.front().first[0] ) {
        good_indices.push_back(i);
      }
    }

    // Record new list that contains only the inliers.
    vw_out() << "\t    Reduced matches to " << good_indices.size() << "\n";
    ip1_copy.resize(0); ip2_copy.resize(0);
    BOOST_FOREACH( size_t index, good_indices ) {
      Vector2 l = left_tx.reverse( Vector2( matched_ip1[index].x, matched_ip1[index].y ) );
      Vector2 r = right_tx.reverse( Vector2( matched_ip2[index].x, matched_ip2[index].y ) );
      matched_ip1[index].ix = matched_ip1[index].x = l.x();
      matched_ip1[index].iy = matched_ip1[index].y = l.y();
      matched_ip2[index].ix = matched_ip2[index].x = r.x();
      matched_ip2[index].iy = matched_ip2[index].y = r.y();
      ip1_copy.push_back( matched_ip1[index] );
      ip2_copy.push_back( matched_ip2[index] );
    }

    ip::write_binary_match_file( output_name, ip1_copy, ip2_copy );

    // Determine if we just wrote nothing but outliers
    // If the variance on triangulation is ungodly highy
    if ( clustering.front().second[0] > 1e6 )
      return false;

    return true;
  }

  template <class Image1T, class Image2T>
  bool ip_matching_w_alignment( vw::camera::CameraModel* cam1,
                                vw::camera::CameraModel* cam2,
                                vw::ImageViewBase<Image1T> const& image1_base,
                                vw::ImageViewBase<Image2T> const& image2_base,
                                vw::cartography::Datum const& datum,
                                std::string const& output_name,
                                double nodata1 = std::numeric_limits<double>::quiet_NaN(),
                                double nodata2 = std::numeric_limits<double>::quiet_NaN(),
                                vw::TransformRef const& left_tx = vw::TransformRef(vw::TranslateTransform(0,0)),
                                vw::TransformRef const& right_tx = vw::TransformRef(vw::TranslateTransform(0,0)) ) {
    using namespace vw;
    Image1T image1 = image1_base.impl();
    Image2T image2 = image2_base.impl();
    BBox2i box1 = bounding_box(image1), box2 = bounding_box(image2);

    Matrix<double> homography =
      rough_homography_fit( cam1, cam2, left_tx.reverse_bbox(box1),
                            right_tx.reverse_bbox(box2), datum );

    // Remove the main translation and solve for BBox that fits the
    // image. If we used the translation from the solved homography with
    // poorly position cameras, the right image might be moved out of
    // frame.
    homography(0,2) = homography(1,2) = 0;
    VW_OUT( DebugMessage, "asp" ) << "Aligning right to left for IP capture using rough homography: " << homography << std::endl;

    { // Check to see if this rough homography works
      HomographyTransform func( homography );
      VW_ASSERT( box1.intersects( func.forward_bbox( box2 ) ),
                 LogicErr() << "The rough homography alignment based on datum and camera geometry shows that input images do not overlap at all. Unable to proceed.\n" );
    }

    TransformRef tx( compose(HomographyTransform(homography), right_tx) );
    BBox2i raster_box = tx.forward_bbox( right_tx.reverse_bbox(box2) );
    tx = TransformRef(compose(HomographyTransform(homography), right_tx,
                              TranslateTransform(-raster_box.min())));
    raster_box -= raster_box.min();

    // It is important that we use NearestPixelInterpolation in the
    // next step. Using anything else will interpolate nodata values
    // and stop them from being masked out.
    bool inlier =
      ip_matching( cam1, cam2, image1,
                   crop(transform(image2, compose(inverse(right_tx), tx),
                                  ValueEdgeExtension<typename Image2T::pixel_type>(boost::math::isnan(nodata2) ? 0 : nodata2),
                                  NearestPixelInterpolation()), raster_box),
                   datum, output_name, nodata1, nodata2, left_tx, tx );

    std::vector<ip::InterestPoint> ip1_copy, ip2_copy;
    ip::read_binary_match_file( output_name, ip1_copy, ip2_copy );
    Matrix<double> post_fit =
      homography_fit( ip2_copy, ip1_copy, raster_box );
    if ( sum(abs(submatrix(homography,0,0,2,2) - submatrix(post_fit,0,0,2,2))) > 4 ) {
      VW_OUT( DebugMessage, "asp" ) << "Post homography has largely different scale and skew from rough fit. Post solution is " << post_fit << "\n";
      return false;
    }

    return inlier;
  }

}

#endif//__ASP_CORE_INTEREST_POINT_MATCHING_H__
