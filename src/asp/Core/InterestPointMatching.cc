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
#include <vw/Math/RANSAC.h>

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

}
