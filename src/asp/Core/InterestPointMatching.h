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
#include <vw/Camera/CameraModel.h>
#include <vw/Cartography/Datum.h>
#include <vw/InterestPoint/Matcher.h>
#include <vw/InterestPoint/InterestData.h>
#include <vw/Stereo/StereoModel.h>

#include <boost/foreach.hpp>

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
                    vw::TransformRef const& left_tx = vw::TransformRef(vw::TranslateTransform(0,0)),
                    vw::TransformRef const& right_tx = vw::TransformRef(vw::TranslateTransform(0,0))) {
    using namespace vw;
    Image1T image1 = image1_base.impl();
    Image2T image2 = image2_base.impl();
    BBox2i box1 = bounding_box(image1), box2 = bounding_box(image2);

    // Detect Interest Points
    ip::InterestPointList ip1, ip2;
    float number_boxes = (box1.width() / 1024.f) * (box1.height() / 1024.f);
    size_t points_per_tile = 2500.f / number_boxes;
    if ( points_per_tile > 2500 ) points_per_tile = 2500;
    asp::IntegralAutoGainDetector detector( points_per_tile );
    vw_out() << "\t    Processing Left\n";
    ip1 = detect_interest_points( image1, detector );
    vw_out() << "\t    Processing Right\n";
    ip2 = detect_interest_points( image2, detector );

    vw_out() << "\t    Building Descriptors\n";
    ip::SGradDescriptorGenerator descriptor;
    descriptor(image1, ip1 );
    descriptor(image2, ip2 );

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

    // Remove IP that don't triangulate well or triangulate well away from the datum.
    std::vector<Vector2> error_samples( matched_ip1.size() );

    stereo::StereoModel model( cam1, cam2 );
    for (size_t i = 0; i < matched_ip1.size(); i++ ) {
      Vector3 geodetic =
        datum.cartesian_to_geodetic( model( left_tx.reverse(Vector2( matched_ip1[i].x, matched_ip1[i].y )),
                                            right_tx.reverse(Vector2(matched_ip2[i].x,
                                                               matched_ip2[i].y)),
                                            error_samples[i].x() ) );
      error_samples[i].y() = fabs(geodetic[2]);
    }

    std::vector<std::pair<Vector<double>, Vector<double> > > clustering =
      asp::gaussian_clustering< std::vector<Vector2>, 2 >( error_samples.begin(), error_samples.end(), 2 );
    if ( clustering[0].first[0] > clustering[1].first[0] /*Other cluster has lower tri error*/ &&
         clustering[1].second[0] != 0 /*Other cluster has non-zero variance (ie non empty set)*/ )
      std::swap( clustering[0], clustering[1] );

    vw_out() << "\t    Inlier cluster:\n"
             << "\t      Mean: " << clustering.front().first << "\n"
             << "\t      Var : " << clustering.front().second << "\n";

    // Record indices of points that match our clustering result
    std::list<size_t> good_indices;
    for (size_t i = 0; i < matched_ip1.size(); i++ ) {
      Vector2 scalar1 = elem_quot( 1.0, sqrt( 2.0 * M_PI * clustering.front().second ) );
      Vector2 scalar2 = elem_quot( 1.0, sqrt( 2.0 * M_PI * clustering.back().second ) );
      if ( (prod(elem_prod(scalar1, exp( elem_quot(-elem_prod(error_samples[i]-clustering.front().first,
                                                              error_samples[i]-clustering.front().first),
                                                   2 * clustering.front().second ) ) ) ) >
            prod(elem_prod(scalar2, exp( elem_quot(-elem_prod(error_samples[i]-clustering.back().first,
                                                              error_samples[i]-clustering.back().first),
                                                   2 * clustering.back().second ) ) ) ) ) ||
           (error_samples[i].x() < clustering.front().first[0] &&
            error_samples[i].y() < clustering.front().first[1] ) ) {
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
    TransformRef tx( compose(HomographyTransform(homography), right_tx) );
    BBox2i raster_box = tx.forward_bbox( right_tx.reverse_bbox(box2) );
    tx = TransformRef(compose(HomographyTransform(homography), right_tx,
                              TranslateTransform(-raster_box.min())));
    raster_box -= raster_box.min();

    bool inlier =
      ip_matching( cam1, cam2, image1,
                   crop(transform(image2, compose(inverse(right_tx), tx) ), raster_box),
                   datum, output_name, left_tx, tx );

    std::vector<ip::InterestPoint> ip1_copy, ip2_copy;
    ip::read_binary_match_file( output_name, ip1_copy, ip2_copy );
    Matrix<double> post_fit =
      homography_fit( ip2_copy, ip1_copy, raster_box );
    if ( sum(abs(submatrix(homography,0,0,2,2) - submatrix(post_fit,0,0,2,2))) > 5 ) {
      VW_OUT( DebugMessage, "asp" ) << "Post homography has largely different scale and skew from rough fit. Post solution is " << post_fit << "\n";
      return false;
    }

    return inlier;
  }

}

#endif//__ASP_CORE_INTEREST_POINT_MATCHING_H__
