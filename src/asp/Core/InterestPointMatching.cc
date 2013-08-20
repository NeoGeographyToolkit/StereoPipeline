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


#include <asp/Core/InterestPointMatching.h>
#include <asp/Core/GaussianClustering.h>
#include <vw/Math/RANSAC.h>
#include <vw/Cartography/CameraBBox.h>
#include <vw/Stereo/StereoModel.h>

using namespace vw;

namespace asp {

  Vector3 EpipolarLinePointMatcher::epipolar_line( Vector2 const& feature,
                                                   cartography::Datum const& datum,
                                                   camera::CameraModel* cam_ip,
                                                   camera::CameraModel* cam_obj ) {
    Vector3 p0 = cartography::datum_intersection( datum, cam_ip, feature );
    Vector3 p1 = p0 + 10*cam_ip->pixel_to_vector( feature );
    Vector2 ep0 = cam_obj->point_to_pixel( p0 );
    Vector2 ep1 = cam_obj->point_to_pixel( p1 );
    Matrix<double> matrix( 2, 3 );
    select_col( matrix, 2 ) = Vector2(1,1);
    matrix(0,0) = ep0.x();
    matrix(0,1) = ep0.y();
    matrix(1,0) = ep1.x();
    matrix(1,1) = ep1.y();
    return select_col(nullspace( matrix ),0);
  }

  double EpipolarLinePointMatcher::distance_point_line( Vector3 const& line,
                                                        Vector2 const& point ) {
    return fabs( line.x() * point.x() +
                 line.y() * point.y() +
                 line.z() ) /
      norm_2( subvector( line, 0, 2 ) );
  }

  class EpipolarLineMatchTask : public Task, private boost::noncopyable {
    typedef ip::InterestPointList::const_iterator IPListIter;
    math::FLANNTree<float>& m_tree;
    IPListIter m_start, m_end;
    ip::InterestPointList const& m_ip_other;
    camera::CameraModel *m_cam1, *m_cam2;
    TransformRef m_tx1, m_tx2;
    EpipolarLinePointMatcher const& m_matcher;
    Mutex& m_camera_mutex;
    std::vector<size_t>::iterator m_output;
  public:
    EpipolarLineMatchTask( math::FLANNTree<float>& tree,
                           ip::InterestPointList::const_iterator start,
                           ip::InterestPointList::const_iterator end,
                           ip::InterestPointList const& ip2,
                           camera::CameraModel* cam1,
                           camera::CameraModel* cam2,
                           TransformRef const& tx1,
                           TransformRef const& tx2,
                           EpipolarLinePointMatcher const& matcher,
                           Mutex& camera_mutex,
                           std::vector<size_t>::iterator output ) :
    m_tree(tree), m_start(start), m_end(end), m_ip_other(ip2),
      m_cam1(cam1), m_cam2(cam2), m_tx1(tx1), m_tx2(tx2),
      m_matcher( matcher ), m_camera_mutex(camera_mutex), m_output(output) {}

    void operator()() {
      Vector<int> indices(10);
      Vector<float> distances(10);

      for ( IPListIter ip = m_start; ip != m_end; ip++ ) {
        Vector2 ip_org_coord = m_tx1.reverse( Vector2( ip->x, ip->y ) );
        Vector3 line_eq;

        // Can't assume the camera is safe (ISIS)
        {
          Mutex::Lock lock( m_camera_mutex );
          line_eq = m_matcher.epipolar_line( ip_org_coord, m_matcher.m_datum, m_cam1, m_cam2 );
        }

        std::vector<std::pair<float,int> > kept_indices;
        kept_indices.reserve(10);
        m_tree.knn_search( ip->descriptor, indices, distances, 10 );

        for ( size_t i = 0; i < 10; i++ ) {
          IPListIter ip2_it = m_ip_other.begin();
          std::advance( ip2_it, indices[i] );
          Vector2 ip2_org_coord = m_tx2.reverse( Vector2( ip2_it->x, ip2_it->y ) );
          double distance = m_matcher.distance_point_line( line_eq, ip2_org_coord );
          if ( distance < m_matcher.m_epipolar_threshold ) {
            kept_indices.push_back( std::pair<float,int>( distances[i], indices[i] ) );
          }
        }

        if ( ( kept_indices.size() > 2 &&
               kept_indices[0].first < m_matcher.m_threshold * kept_indices[1].first ) ||
             kept_indices.size() == 1 ){
          *m_output++ = kept_indices[0].second;
        } else {
          *m_output++ = (size_t)(-1);
        }
      }
    }
  };

  void EpipolarLinePointMatcher::operator()( ip::InterestPointList const& ip1,
                                             ip::InterestPointList const& ip2,
                                             camera::CameraModel* cam1,
                                             camera::CameraModel* cam2,
                                             TransformRef const& tx1,
                                             TransformRef const& tx2,
                                             std::vector<size_t>& output_indices ) const {
    typedef ip::InterestPointList::const_iterator IPListIter;

    Timer total_time("Total elapsed time", DebugMessage, "interest_point");
    size_t ip1_size = ip1.size(), ip2_size = ip2.size();

    output_indices.clear();
    if (!ip1_size || !ip2_size) {
      vw_out(InfoMessage,"interest_point") << "KD-Tree: no points to match, exiting\n";
      return;
    }

    // Build the output indices
    output_indices.resize( ip1_size );

    // Make the storage structure required by FLANN. FLANN really only
    // holds a bunch of pointers to this structure. It should be
    // possible to modify FLANN so that it doesn't need a copy.
    Matrix<float> ip2_matrix( ip2_size, ip2.begin()->size() );
    Matrix<float>::iterator ip2_matrix_it = ip2_matrix.begin();
    BOOST_FOREACH( ip::InterestPoint const& ip, ip2 )
      ip2_matrix_it = std::copy( ip.begin(), ip.end(), ip2_matrix_it );

    math::FLANNTree<float > kd( ip2_matrix );
    vw_out(InfoMessage,"interest_point") << "FLANN-Tree created. Searching...\n";

    FifoWorkQueue matching_queue;
    Mutex camera_mutex;

    // Jobs set to 2x the number of cores. This is just incase all jobs are not equal.
    size_t number_of_jobs = vw_settings().default_num_threads() * 2;
    IPListIter start_it = ip1.begin();
    std::vector<size_t>::iterator output_it = output_indices.begin();

    for ( size_t i = 0; i < number_of_jobs - 1; i++ ) {
      IPListIter end_it = start_it;
      std::advance( end_it, ip1_size / number_of_jobs );
      boost::shared_ptr<Task>
        match_task( new EpipolarLineMatchTask( kd, start_it, end_it,
                                               ip2, cam1, cam2, tx1, tx2, *this,
                                               camera_mutex, output_it ) );
      matching_queue.add_task( match_task );
      start_it = end_it;
      std::advance( output_it, ip1_size / number_of_jobs );
    }
    boost::shared_ptr<Task>
      match_task( new EpipolarLineMatchTask( kd, start_it, ip1.end(),
                                             ip2, cam1, cam2, tx1, tx2, *this,
                                             camera_mutex, output_it ) );
    matching_queue.add_task( match_task );
    matching_queue.join_all();
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
            cartography::datum_intersection( datum, cam1, l );
          if ( intersection == Vector3() )
            continue;

          Vector2 r = cam2->point_to_pixel( intersection );

          if ( box2.contains( r ) ){
            left_points.push_back( Vector3(l[0],l[1],1) );
            right_points.push_back( Vector3(r[0],r[1],1) );
          }
        }
        catch (camera::PixelToRayErr   const& e ) {}
        catch (camera::PointToPixelErr const& e ) {}

        try {
          Vector2 r( double(box2.width() - 1) * i / 99.0,
                     double(box2.height() - 1) * j / 99.0 );

          Vector3 intersection =
            cartography::datum_intersection( datum, cam2, r );
          if ( intersection == Vector3() )
            continue;

          Vector2 l = cam1->point_to_pixel( intersection );

          if ( box1.contains( l ) ) {
            left_points.push_back( Vector3(l[0],l[1],1) );
            right_points.push_back( Vector3(r[0],r[1],1) );
          }
        }
        catch (camera::PixelToRayErr   const& e ) {}
        catch (camera::PointToPixelErr const& e ) {}
      }
    }

    typedef math::HomographyFittingFunctor hfit_func;
    math::RandomSampleConsensus<hfit_func, math::InterestPointErrorMetric>
      ransac( hfit_func(), math::InterestPointErrorMetric(),
              100, // num iterations
              norm_2(Vector2(box1.width(),box1.height())) / 10, // inlier threshold
              left_points.size()/2 // min output inliers
              );
    Matrix<double> H = ransac( right_points, left_points );
    std::vector<size_t> indices = ransac.inlier_indices(H, right_points, left_points);
    check_homography_matrix(H, left_points, right_points, indices);

    VW_OUT( DebugMessage, "asp" ) << "Projected " << left_points.size()
                                  << " rays for rough homography.\n";
    VW_OUT( DebugMessage, "asp" ) << "Number of inliers: " << indices.size() << ".\n";

    return H;
  }

  Vector2i
  homography_rectification( Vector2i const& left_size,
                            Vector2i const& right_size,
                            std::vector<ip::InterestPoint> const& left_ip,
                            std::vector<ip::InterestPoint> const& right_ip,
                            vw::Matrix<double>& left_matrix,
                            vw::Matrix<double>& right_matrix ) {

    std::vector<Vector3>  right_copy, left_copy;
    right_copy.reserve( right_ip.size() );
    left_copy.reserve( right_ip.size() );
    for ( size_t i = 0; i < right_ip.size(); i++ ) {
      right_copy.push_back( Vector3(right_ip[i].x, right_ip[i].y, 1) );
      left_copy.push_back( Vector3(left_ip[i].x, left_ip[i].y, 1) );
    }

    typedef math::HomographyFittingFunctor hfit_func;
    math::RandomSampleConsensus<hfit_func, math::InterestPointErrorMetric>
      ransac( hfit_func(), math::InterestPointErrorMetric(),
              100, // num iter
              norm_2(Vector2(left_size.x(),left_size.y())) / 10, // inlier threshold
              left_copy.size()/2 // min output inliers
              );
    Matrix<double> H = ransac(right_copy, left_copy);
    std::vector<size_t> indices = ransac.inlier_indices(H, right_copy, left_copy);
    check_homography_matrix(H, left_copy, right_copy, indices);

    // Set right to a homography that has been refined just to our inliers
    left_matrix = math::identity_matrix<3>();
    right_matrix = hfit_func()(right_copy, left_copy, H);

    // Work out the ideal render size
    BBox2i output_bbox, right_bbox;
    output_bbox.grow( Vector2i(0,0) );
    output_bbox.grow( Vector2i(left_size.x(),0) );
    output_bbox.grow( Vector2i(0,left_size.y()) );
    output_bbox.grow( left_size );
    Vector3 temp = right_matrix*Vector3(0,0,1);
    temp /= temp.z();
    right_bbox.grow( subvector(temp,0,2) );
    temp = right_matrix*Vector3(right_size.x(),0,1);
    temp /= temp.z();
    right_bbox.grow( subvector(temp,0,2) );
    temp = right_matrix*Vector3(0,right_size.y(),1);
    temp /= temp.z();
    right_bbox.grow( subvector(temp,0,2) );
    temp = right_matrix*Vector3(right_size.x(),right_size.y(),1);
    temp /= temp.z();
    right_bbox.grow( subvector(temp,0,2) );
    output_bbox.crop( right_bbox );

    //  Move the ideal render size to be aligned up with origin
    left_matrix(0,2) -= output_bbox.min().x();
    right_matrix(0,2) -= output_bbox.min().x();
    left_matrix(1,2) -= output_bbox.min().y();
    right_matrix(1,2) -= output_bbox.min().y();

    return Vector2i( output_bbox.width(), output_bbox.height() );
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
    typedef std::vector<double> ArrayT;
    ArrayT error_samples( matched_ip1.size() ), alt_samples( matched_ip1.size() );

    // Create the 'error' samples. Which are triangulation error and
    // distance to sphere.
    stereo::StereoModel model( cam1, cam2 );
    for (size_t i = 0; i < matched_ip1.size(); i++ ) {
      Vector3 geodetic =
        datum.cartesian_to_geodetic( model( left_tx.reverse(Vector2( matched_ip1[i].x, matched_ip1[i].y )),
                                            right_tx.reverse(Vector2(matched_ip2[i].x,
                                                                     matched_ip2[i].y)),
                                            error_samples[i] ) );
      alt_samples[i] = geodetic.z();
    }

    typedef std::vector<std::pair<Vector<double>, Vector<double> > > ClusterT;
    ClusterT error_clusters =
      asp::gaussian_clustering<ArrayT>( error_samples.begin(),
                                        error_samples.end(), 2 );
    ClusterT alt_clusters =
      asp::gaussian_clustering<ArrayT>( alt_samples.begin(),
                                        alt_samples.end(), 2 );

    // The best triangulation error and altitude clusters are ones
    // that have small standard deviations. They are focused on the
    // tight pack of inliers. Bring the smaller std-dev cluster to the
    // front as it is what we are interested in.
    if ( error_clusters.front().second[0] > error_clusters.back().second[0] &&
         error_clusters.back().second[0] != 0 )
      std::swap( error_clusters[0], error_clusters[1] );
    if ( alt_clusters.front().second[0] > alt_clusters.back().second[0] &&
         alt_clusters.back().second[0] != 0 )
      std::swap( alt_clusters[0], alt_clusters[1] );

    // Determine if we just wrote nothing but outliers (the variance
    // on triangulation is too high).
    if ( error_clusters.front().second[0] > 1e6 )
      return false;

    vw_out() << "\t    Inlier cluster:\n"
             << "\t      Triangulation Err: " << error_clusters.front().first[0]
             << " +- " << sqrt( error_clusters.front().second[0] ) << " meters\n";

    // Determine if we should disable the altitude constraint if the
    // standard deviations are not wild different. If everything is an
    // inlier .. the mixture model will just seperate different planar
    // regions of the image.
    bool disable_alt_check = false;
    if ( fabs( log10( alt_clusters.front().second[0] ) - log10( alt_clusters.back().second[0] ) ) < 1 ) {
      disable_alt_check = true;
    } else {
      vw_out() << "\t      Altitude         : " << alt_clusters.front().first[0]
               << " +- " << sqrt( alt_clusters.front().second[0] ) << " meters\n";
    }

    // Record indices of points that match our clustering result
    output.clear();
    const double escalar1 = 1.0 / sqrt( 2.0 * M_PI * error_clusters.front().second[0] ); // outside exp of normal eq
    const double escalar2 = 1.0 / sqrt( 2.0 * M_PI * error_clusters.back().second[0] );
    const double escalar3 = 1.0 / (2 * error_clusters.front().second[0] ); // inside exp of normal eq
    const double escalar4 = 1.0 / (2 * error_clusters.back().second[0] );
    const double ascalar1 = 1.0 / sqrt( 2.0 * M_PI * alt_clusters.front().second[0] );
    const double ascalar2 = 1.0 / sqrt( 2.0 * M_PI * alt_clusters.back().second[0] );
    const double ascalar3 = 1.0 / (2 * alt_clusters.front().second[0] );
    const double ascalar4 = 1.0 / (2 * alt_clusters.back().second[0] );
    for (size_t i = 0; i < matched_ip1.size(); i++ ) {
      double err_diff_front = error_samples[i]-error_clusters.front().first[0];
      double err_diff_back = error_samples[i]-error_clusters.back().first[0];
      double alt_diff_front = alt_samples[i]-alt_clusters.front().first[0];
      double alt_diff_back = alt_samples[i]-alt_clusters.back().first[0];
      // Is this point an inlier in terms of triangulation error?
      bool error_inlier =
        (escalar1 * exp( (-err_diff_front * err_diff_front) * escalar3 ) ) >
        (escalar2 * exp( (-err_diff_back * err_diff_back) * escalar4 ) ) ||
        error_samples[i] < error_clusters.front().first[0];

      if ( error_inlier ) {
        std::cout << i << " " << error_samples[i] << " " << alt_samples[i] << " " << error_clusters.front().first[0]  << std::endl;
      }

      // Is this point an inlier in terms of altitude against world datum?
      bool alt_inlier =
        disable_alt_check ||
        (
         (ascalar1 * exp( (-alt_diff_front * alt_diff_front) * ascalar3 ) ) >
         (ascalar2 * exp( (-alt_diff_back * alt_diff_back ) * ascalar4 ) ) &&
         fabs(alt_diff_front) < 3 * sqrt(alt_clusters.front().second[0]) );

      if ( error_inlier && alt_inlier ) {
        output.push_back(i);
      }
    }

    // See what happens if we apply altitude thresholding here ...
    alt_samples.clear();
    BOOST_FOREACH( size_t i, output ) {
      double error;
      Vector3 geodetic =
        datum.cartesian_to_geodetic( model( left_tx.reverse(Vector2( matched_ip1[i].x, matched_ip1[i].y )),
                                            right_tx.reverse(Vector2(matched_ip2[i].x,
                                                                     matched_ip2[i].y)),
                                            error ) );
      alt_samples.push_back( geodetic.z() );
    }
    alt_clusters =
      asp::gaussian_clustering<ArrayT>( alt_samples.begin(),
                                        alt_samples.end(), 2 );
    if ( alt_clusters.front().second[0] > alt_clusters.back().second[0] &&
         alt_clusters.back().second[0] != 0 )
      std::swap( alt_clusters[0], alt_clusters[1] );
    BOOST_FOREACH( float a, alt_samples ) {
      std::cout << a << std::endl;
    }

    return true;
  }

}
