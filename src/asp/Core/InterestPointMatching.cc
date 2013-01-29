// __BEGIN_LICENSE__
//  Copyright (c) 2009-2012, United States Government as represented by the
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
#include <asp/Core/GaussianClustering.h>
#include <vw/Math/RANSAC.h>
#include <vw/Cartography/Datum.h>
#include <vw/Stereo/StereoModel.h>

using namespace vw;

namespace asp {

  Vector3 datum_intersection( cartography::Datum const& datum,
                              camera::CameraModel* model,
                              Vector2 const& pix ) {
    using namespace vw;

    // Intersect the ray back-projected from the camera with the
    // datum, which is a spheroid. To simplify the calculations, scale
    // everything in such a way that the spheroid becomes a
    // sphere. Scale back at the end of computation.

    double z_scale = datum.semi_major_axis() / datum.semi_minor_axis();

    Vector3 ccenter = model->camera_center( pix );
    Vector3 cvec  = model->pixel_to_vector( pix );
    ccenter.z() *= z_scale;
    cvec.z() *= z_scale;
    cvec = normalize(cvec);
    double radius_2 = datum.semi_major_axis() *
      datum.semi_major_axis();
    double alpha = -dot_prod(ccenter, cvec );
    Vector3 projection = ccenter + alpha*cvec;
    if ( norm_2_sqr(projection) > radius_2 ) {
      // did not intersect
      return Vector3();
    }

    alpha -= sqrt( radius_2 -
                   norm_2_sqr(projection) );
    return elem_prod(ccenter + alpha * cvec, Vector3(1,1,1.0/z_scale));
  }

  void check_homography_matrix(Matrix<double>       const& H,
                               std::vector<Vector3> const& left_points,
                               std::vector<Vector3> const& right_points,
                               std::vector<size_t>  const& indices
                               ){

    // Sanity checks. If these fail, most likely the two images are too different
    // for stereo to succeed.
    if ( indices.size() < std::min( right_points.size(), left_points.size() )/2 ){
      vw_throw( ArgumentErr() << "InterestPointMatching: The number of inliers is less than 1/2 of the number of points. Invalid stereo pair.\n" );
    }

    double det = H(0, 0)*H(1, 1) - H(0, 1)*H(1, 0);
    if (det <= 0.1 || det >= 10.0){
      vw_throw( ArgumentErr() << "InterestPointMatching: The determinant of the 2x2 submatrix of the homography matrix " << H << " is too small or too large. Invalid stereo pair.\n" );
    }

  }

  // Find a rough homography that maps right to left using the camera
  // and datum information.  More precisely, take a set of pixels in
  // the left camera image, project them onto the ground and back
  // project them into the right camera image. Then to the
  // reverse. This will help find a rough correspondence between
  // the pixels in the two camera images.
  Matrix<double>
  rough_homography_fit( camera::CameraModel* cam1,
                        camera::CameraModel* cam2,
                        BBox2i const& box1, BBox2i const& box2,
                        cartography::Datum const& datum ) {

    // Bounce several points off the datum and fit an affine.
    std::vector<Vector3> left_points, right_points;
    left_points.reserve(20000);
    right_points.reserve(20000);
    for ( size_t i = 0; i < 100; i++ ) {
      for ( size_t j = 0; j < 100; j++ ) {
        try {
          Vector2 l( double(box1.width() - 1) * i / 99.0,
                     double(box1.height() - 1) * j / 99.0 );

          Vector3 intersection =
            datum_intersection( datum, cam1, l );
          if ( intersection == Vector3() )
            continue;

          Vector2 r = cam2->point_to_pixel( intersection );

          if ( box2.contains( r ) ){
            left_points.push_back( Vector3(l[0],l[1],1) );
            right_points.push_back( Vector3(r[0],r[1],1) );
          }
        } catch (camera::PixelToRayErr const& e ) {}
        try {
          Vector2 r( double(box2.width() - 1) * i / 99.0,
                     double(box2.height() - 1) * j / 99.0 );

          Vector3 intersection =
            datum_intersection( datum, cam2, r );
          if ( intersection == Vector3() )
            continue;

          Vector2 l = cam1->point_to_pixel( intersection );

          if ( box1.contains( l ) ) {
            left_points.push_back( Vector3(l[0],l[1],1) );
            right_points.push_back( Vector3(r[0],r[1],1) );
          }
        } catch (camera::PixelToRayErr const& e ) {}
      }
    }

    typedef math::HomographyFittingFunctor hfit_func;
    math::RandomSampleConsensus<hfit_func, math::InterestPointErrorMetric>
      ransac( hfit_func(), math::InterestPointErrorMetric(),
              100, // num iterations
              norm_2(Vector2(box1.height(),box1.width())) / 10, // inlier threshold
              left_points.size()/2 // min output inliers
              );

    Matrix<double> H = ransac( right_points, left_points );
    std::vector<size_t> indices = ransac.inlier_indices(H, right_points, left_points);

    VW_OUT( DebugMessage, "asp" ) << "Projected " << left_points.size()
                                  << " rays for rough homography.\n";
    VW_OUT( DebugMessage, "asp" ) << "Number of inliers: " << indices.size() << ".\n";

    check_homography_matrix(H, left_points, right_points, indices);

    return H;
  }

  vw::Matrix<double>
  homography_fit( std::vector<vw::ip::InterestPoint> const& ip1,
                  std::vector<vw::ip::InterestPoint> const& ip2,
                  vw::BBox2i const& image_size ) {
    using namespace vw;

    std::vector<Vector3>  copy1, copy2;
    copy1.reserve( ip1.size() );
    copy2.reserve( ip1.size() );
    for ( size_t i = 0; i < ip1.size(); i++ ) {
      copy1.push_back( Vector3(ip1[i].x, ip1[i].y, 1) );
      copy2.push_back( Vector3(ip2[i].x, ip2[i].y, 1) );
    }

    typedef math::HomographyFittingFunctor hfit_func;
    math::RandomSampleConsensus<hfit_func, math::InterestPointErrorMetric>
      ransac( hfit_func(), math::InterestPointErrorMetric(),
              100, // num iter
              norm_2(Vector2(image_size.width(),image_size.height())) / 10, // inlier threshold
              copy1.size()/2 // min output inliers
              );

    Matrix<double> H = ransac(copy1, copy2);
    std::vector<size_t> indices = ransac.inlier_indices(H, copy1, copy2);
    check_homography_matrix(H, copy1, copy2, indices);

    return hfit_func()(copy1, copy2, H);
  }

  bool
  tri_and_alt_ip_filtering( std::vector<ip::InterestPoint> const& matched_ip1,
                            std::vector<ip::InterestPoint> const& matched_ip2,
                            vw::camera::CameraModel* cam1,
                            vw::camera::CameraModel* cam2,
                            vw::cartography::Datum const& datum,
                            std::list<size_t>& output,
                            vw::TransformRef const& left_tx,
                            vw::TransformRef const& right_tx ) {
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

    // Determine if we just wrote nothing but outliers
    // If the variance on triangulation is ungodly highy
    if ( clustering.front().second[0] > 1e6 )
      return false;

    vw_out() << "\t    Inlier cluster:\n"
             << "\t      Triangulation Err: " << clustering.front().first[0] / error_scaling[0] + error_offset[0]
             << " +- " << sqrt( clustering.front().second[0] / error_scaling[0] ) << " meters\n"
             << "\t      Altitude         : " << clustering.front().first[0] / error_scaling[1] + error_offset[1]
             << " +- " << sqrt( clustering.front().second[0] / error_scaling[1] ) << " meters\n";

    // Record indices of points that match our clustering result
    output.clear();
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
        output.push_back(i);
      }
    }

    return true;
  }

}
