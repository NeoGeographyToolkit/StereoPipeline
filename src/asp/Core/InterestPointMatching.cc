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

  // Find a rough homography that maps right to left using the camera
  // and datum information.
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

          if ( box2.contains( r ) ) {
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

    VW_OUT( DebugMessage, "asp" ) << "Projected " << left_points.size()
                                  << " rays for rough homography.\n";

    typedef math::HomographyFittingFunctor hfit_func;
    math::RandomSampleConsensus<hfit_func, math::InterestPointErrorMetric> ransac( hfit_func(), math::InterestPointErrorMetric(), norm_2(Vector2(box1.height(),box1.width())) / 10 );
    Matrix<double> H = ransac( right_points, left_points );
    std::vector<size_t> indices = ransac.inlier_indices(H, right_points, left_points);

    // Sanity checks. If these fail, most likely the two images are too different
    // for stereo to succeed.
    if ( indices.size() < std::min( right_points.size(), left_points.size() )/3 ){
      vw_throw( ArgumentErr() << "InterestPointMatching: The number of inliers is less than 1/3 of the number of points. Invalid stereo pair.\n" );
    }
    
    double det = H(0, 0)*H(1, 1) - H(0, 1)*H(1, 0);
    if (det <= 0.0 || det > 2.0){
      vw_throw( ArgumentErr() << "InterestPointMatching: The determinant of homography matrix is negative or too large. Invalid stereo pair.\n" );
    }
    
    return H;
  }

  vw::Matrix<double>
  homography_fit( std::vector<vw::ip::InterestPoint> const& ip1,
                  std::vector<vw::ip::InterestPoint> const& ip2,
                  vw::BBox2i const& image_size ) {
    using namespace vw;

    typedef math::HomographyFittingFunctor hfit_func;
    math::RandomSampleConsensus<hfit_func, math::InterestPointErrorMetric> ransac( hfit_func(), math::InterestPointErrorMetric(), norm_2(Vector2(image_size.width(),image_size.height())) / 10 );
    std::vector<Vector3>  copy1, copy2;
    copy1.reserve( ip1.size() );
    copy2.reserve( ip1.size() );
    for ( size_t i = 0; i < ip1.size(); i++ ) {
      copy1.push_back( Vector3(ip1[i].x, ip1[i].y, 1) );
      copy2.push_back( Vector3(ip2[i].x, ip2[i].y, 1) );
    }
    Matrix<double> ransac_result = ransac(copy1,copy2);
    return hfit_func()(copy1,copy2,ransac_result);
  }

}
