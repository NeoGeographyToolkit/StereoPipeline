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
#include <vw/Math/GaussianClustering.h>
#include <vw/Math/RANSAC.h>
#include <vw/Cartography/CameraBBox.h>
#include <vw/Stereo/StereoModel.h>

using namespace vw;

namespace asp {

  int g_ip_num_errors = 0;
  Mutex g_ip_mutex;


//-------------------------------------------------------------------------------------------------
// Class EpipolarLinePointMatcher

  EpipolarLinePointMatcher::EpipolarLinePointMatcher( bool   single_threaded_camera,
                                                      double uniqueness_threshold,
                                                      double epipolar_threshold,
                                                      vw::cartography::Datum const& datum) :
    m_single_threaded_camera(single_threaded_camera), m_uniqueness_threshold(uniqueness_threshold),
    m_epipolar_threshold(epipolar_threshold), m_datum(datum) {
    // Detect some problems before they result in strange math errors
    if (epipolar_threshold < 1)
      vw_throw( ArgumentErr() << "EpipolarLinePointMatcher: epipolar threshold is < 1.\n" );
    if (uniqueness_threshold < 0.1)
      vw_throw( ArgumentErr() << "EpipolarLinePointMatcher: uniqueness threshold is < 0.1.\n" );
    if (uniqueness_threshold > 0.99)
      vw_throw( ArgumentErr() << "EpipolarLinePointMatcher: uniqueness threshold is > 0.99.\n" );
  }

  Vector3 EpipolarLinePointMatcher::epipolar_line( Vector2 const& feature,
						   cartography::Datum const& datum,
						   camera::CameraModel* cam_ip,
						   camera::CameraModel* cam_obj,
						   bool & success) {
    success = true;

    // Watch out for errors thrown when projecting into the camera
    try{

      // Intersect the interest point pixel with the datum
      Vector3 p0 = cartography::datum_intersection( datum, cam_ip, feature );

      if (p0 == Vector3()){ // No intersection
        success = false;
        return Vector3();
      }

      Vector3 p1  = p0 + 10*cam_ip->pixel_to_vector( feature ); // Extend the point below the datum
      Vector2 ep0 = cam_obj->point_to_pixel( p0 ); // Project the intersection and extension into the other camera
      Vector2 ep1 = cam_obj->point_to_pixel( p1 );
      Matrix<double> matrix( 2, 3 );
      select_col( matrix, 2 ) = Vector2(1,1);
      matrix(0,0) = ep0.x();
      matrix(0,1) = ep0.y();
      matrix(1,0) = ep1.x();
      matrix(1,1) = ep1.y();

      if (matrix != matrix){ // Got back NaN values. Can't proceed.
        success = false;
        return Vector3();
      }

      // If the input matrix is bad this can result in some weird errors!
      Matrix<double> nsp = nullspace( matrix );
      if (nsp.cols() <= 0 || nsp.rows() <= 0){ // Failed to find the nullspace
        success = false;
        return Vector3();
      }

      return select_col(nsp,0);

    } catch (std::exception const& e) {
      Mutex::Lock lock( g_ip_mutex );
      g_ip_num_errors++;
      if (g_ip_num_errors < 100) {
        vw_out(ErrorMessage) << e.what() << std::endl;
      }else if (g_ip_num_errors == 100) {
        vw_out() << "Will print no more error messages about failing to find epipolar line.\n";
      }
    }

    success = false;
    return Vector3();
  }

  double EpipolarLinePointMatcher::distance_point_line( Vector3 const& line,
							Vector2 const& point ) {
    return fabs( line.x() * point.x() +
		 line.y() * point.y() +
		 line.z() ) /
      norm_2( subvector( line, 0, 2 ) );
  }

  // Local class definition -----
  class EpipolarLineMatchTask : public Task, private boost::noncopyable {
    typedef ip::InterestPointList::const_iterator IPListIter;
    bool                            m_single_threaded_camera;
    bool                            m_use_uchar_tree;
    math::FLANNTree<float        >& m_tree_float;
    math::FLANNTree<unsigned char>& m_tree_uchar;
    IPListIter                      m_start, m_end;
    ip::InterestPointList const&    m_ip_other;
    camera::CameraModel            *m_cam1, *m_cam2;
    TransformRef                    m_tx1, m_tx2;
    EpipolarLinePointMatcher const& m_matcher;
    Mutex&                          m_camera_mutex;
    std::vector<size_t>::iterator   m_output;
  public:
    EpipolarLineMatchTask( bool single_threaded_camera,
			   bool use_uchar_tree,
			   math::FLANNTree<float        >& tree_float,
			   math::FLANNTree<unsigned char>& tree_uchar,
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
      m_single_threaded_camera(single_threaded_camera),
      m_use_uchar_tree(use_uchar_tree), m_tree_float(tree_float), m_tree_uchar(tree_uchar),
      m_start(start), m_end(end), m_ip_other(ip2),
      m_cam1(cam1), m_cam2(cam2), m_tx1(tx1), m_tx2(tx2),
      m_matcher( matcher ), m_camera_mutex(camera_mutex), m_output(output) {}

    void operator()() {

      const size_t NUM_MATCHES_TO_FIND = 10;
      Vector<int   > indices  (NUM_MATCHES_TO_FIND);
      Vector<double> distances(NUM_MATCHES_TO_FIND);

      for ( IPListIter ip = m_start; ip != m_end; ip++ ) {
        Vector2 ip_org_coord = m_tx1.reverse( Vector2( ip->x, ip->y ) );
        Vector3 line_eq;

        // Find the equation that describes the epipolar line
        bool found_epipolar = false;
        if (m_single_threaded_camera){
          // ISIS camera is single-threaded
          Mutex::Lock lock( m_camera_mutex );
          line_eq = m_matcher.epipolar_line( ip_org_coord, m_matcher.m_datum, m_cam1, m_cam2, found_epipolar);
        }else{
          line_eq = m_matcher.epipolar_line( ip_org_coord, m_matcher.m_datum, m_cam1, m_cam2, found_epipolar);
        }

        if (!found_epipolar) {
          *m_output++ = (size_t)(-1); // Failed to find a match, return a flag!
          continue; // Skip to the next IP
        }

        // Use FLANN tree to find the N nearest neighbors according to the IP region descriptor?
        std::vector<std::pair<float,int> > kept_indices;
        kept_indices.reserve(NUM_MATCHES_TO_FIND);

        // Call the correct FLANN tree for the matching type
        size_t num_matches_valid = 0;
        if (m_use_uchar_tree) {
          vw::Vector<unsigned char> uchar_descriptor(ip->descriptor.size());
          for (size_t i=0; i<ip->descriptor.size(); ++i)
            uchar_descriptor[i] = static_cast<unsigned char>(ip->descriptor[i]);
          num_matches_valid = m_tree_uchar.knn_search( uchar_descriptor, indices, distances, NUM_MATCHES_TO_FIND );
        } else {
          num_matches_valid = m_tree_float.knn_search( ip->descriptor, indices, distances, NUM_MATCHES_TO_FIND );
        }

        if (num_matches_valid < 1) {
          *m_output++ = (size_t)(-1); // Failed to find a match, return a flag!
          continue; // Skip to the next IP
        }

        //vw_out() << "For descriptor: " << ip->descriptor << std::endl;
        //vw_out() << num_matches_valid << " Best match distances: " << distances << std::endl;
        //vw_out() << "Indices: " << indices << std::endl;

        // Loop through the N "nearest" points and keep only the ones within
        //   m_matcher.m_epipolar_threshold pixel distance from the epipolar line
        const double EPIPOLAR_BAND_EXPANSION = 200;
        double small_epipolar_threshold = m_matcher.m_epipolar_threshold;
        double large_epipolar_threshold = small_epipolar_threshold + EPIPOLAR_BAND_EXPANSION;
        for ( size_t i = 0; i < num_matches_valid; i++ ) {
          IPListIter ip2_it = m_ip_other.begin();
          std::advance( ip2_it, indices[i] );

          if (found_epipolar){
            Vector2 ip2_org_coord = m_tx2.reverse( Vector2( ip2_it->x, ip2_it->y ) );
            double  line_distance = m_matcher.distance_point_line( line_eq, ip2_org_coord );
            if ( line_distance < large_epipolar_threshold ) {
              if ( line_distance < small_epipolar_threshold )
                kept_indices.push_back( std::pair<float,int>( distances[i], indices[i] ) );
              else // In between thresholds
                kept_indices.push_back( std::pair<float,int>( distances[i], -1 ) );
            }
            else {
              Vector2 ip1_coord( ip->x, ip->y );
              //double normDist = norm_2(ip1_coord - ip2_org_coord);
              //vw_out() << "Discarding match between " << ip1_coord << " and " << ip2_org_coord
              //        << " because distance is " << line_distance << " and threshold is "
              //        << m_matcher.m_epipolar_threshold << " norm dist = " << normDist<<"\n";
            }
          }
        } // End loop for match prunining

        // If we only found one match or the first descriptor match is much better than the second
        if ( ( (kept_indices.size() > 2)     &&
               (kept_indices[0].second >= 0) &&
                     (kept_indices[0].first < m_matcher.m_uniqueness_threshold * kept_indices[1].first) )
              || (kept_indices.size() == 1) ){
          *m_output++ = kept_indices[0].second; // Return the first of the matches we found
          //vw_out() << "Kept distance: " << kept_indices[0].first << std::endl;
        } else { // No matches or no clear winner
          *m_output++ = (size_t)(-1); // Failed to find a match, return a flag!
        }
      } // End loop through IP
    } // End function operator()

  }; // End class EpipolarLineMatchTask -------------------

  void EpipolarLinePointMatcher::operator()( ip::InterestPointList const& ip1,
					     ip::InterestPointList const& ip2,
					     DetectIpMethod  ip_detect_method,
					     camera::CameraModel        * cam1,
					     camera::CameraModel        * cam2,
					     TransformRef          const& tx1,
					     TransformRef          const& tx2,
					     std::vector<size_t>        & output_indices ) const {
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

    // Set up FLANNTree objects of all the different types we may need.
    math::FLANNTree<float        > kd_float;
    math::FLANNTree<unsigned char> kd_uchar;

    Matrix<float        > ip2_matrix_float;
    Matrix<unsigned char> ip2_matrix_uchar;

    // Pack the IP descriptors into a matrix and feed it to the chosen FLANNTree object
    const bool use_uchar_FLANN = (ip_detect_method == DETECT_IP_METHOD_ORB);
    if (use_uchar_FLANN) {
      ip_list_to_matrix(ip2, ip2_matrix_uchar);
      kd_uchar.load_match_data( ip2_matrix_uchar, vw::math::FLANN_DistType_Hamming );
    }else {
      ip_list_to_matrix(ip2, ip2_matrix_float);
      kd_float.load_match_data( ip2_matrix_float,  vw::math::FLANN_DistType_L2 );
    }

    vw_out(InfoMessage,"interest_point") << "FLANN-Tree created. Searching...\n";

    FifoWorkQueue matching_queue; // Create a thread pool object
    Mutex camera_mutex;

    // Jobs set to 2x the number of cores. This is just incase all jobs are not equal.
    // The total number of interest points will be divided up among the jobs.
    size_t number_of_jobs = vw_settings().default_num_threads() * 2;

    // Robustness fix
    if (ip1_size < number_of_jobs)
      number_of_jobs = ip1_size;

    // Get input and output iterators
    IPListIter start_it = ip1.begin();
    std::vector<size_t>::iterator output_it = output_indices.begin();

    for ( size_t i = 0; i < number_of_jobs - 1; i++ ) { // For each job...
      // Update iterators and launch the job.
      IPListIter end_it = start_it;
      std::advance( end_it, ip1_size / number_of_jobs );
      boost::shared_ptr<Task>
	match_task( new EpipolarLineMatchTask( m_single_threaded_camera,
					       use_uchar_FLANN, kd_float, kd_uchar,
					       start_it, end_it,
					       ip2, cam1, cam2, tx1, tx2, *this,
					       camera_mutex, output_it ) );
      matching_queue.add_task( match_task );
      start_it = end_it;
      std::advance( output_it, ip1_size / number_of_jobs );
    }
    boost::shared_ptr<Task>
      match_task( new EpipolarLineMatchTask( m_single_threaded_camera,
					     use_uchar_FLANN, kd_float, kd_uchar,
					     start_it, ip1.end(),
					     ip2, cam1, cam2, tx1, tx2, *this,
					     camera_mutex, output_it ) );
    matching_queue.add_task( match_task );
    matching_queue.join_all(); // Wait for all the jobs to finish.
  }

// End class EpipolarLinePointMatcher
//---------------------------------------------------------------------------------------

  void check_homography_matrix(Matrix<double>       const& H,
			       std::vector<Vector3> const& left_points,
			       std::vector<Vector3> const& right_points,
			       std::vector<size_t>  const& indices
			       ){

    // Sanity checks. If these fail, most likely the two images are too different
    // for stereo to succeed.
    if ( indices.size() < std::min( right_points.size(), left_points.size() )/2 ){
      vw_out(WarningMessage) << "InterestPointMatching: The number of inliers is less "
                             << "than 1/2 of the number of points. The inputs may be invalid.\n";
    }

    double det = fabs(H(0, 0)*H(1, 1) - H(0, 1)*H(1, 0));
    if (det <= 0.1 || det >= 10.0){
      vw_out(WarningMessage) << "InterestPointMatching: The determinant of the 2x2 submatrix "
                             << "of the homography matrix " << H << " is " << det
                             << ". There could be a large scale discrepancy among the input images "
                             << "or the inputs may be an invalid stereo pair.\n";
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
    int num = 100;
    left_points.reserve(2*num*num);
    right_points.reserve(2*num*num);

    // Report progress, as this may be slow
    TerminalProgressCallback tpc("", "\tRough homography--> ");
    tpc.report_progress(0);
    double inc_amount = 1.0 / double(num) / double(num);
    
    for (int i = 0; i < num; i++ ) {
      for ( int j = 0; j < num; j++ ) {
        try {
          Vector2 l( double(box1.width()  - 1) * i / (num-1.0),
	             double(box1.height() - 1) * j / (num-1.0) );

          Vector3 intersection = cartography::datum_intersection( datum, cam1, l );
          if ( intersection == Vector3() )
            continue;

          Vector2 r = cam2->point_to_pixel( intersection );

          if ( box2.contains( r ) ){
            left_points.push_back(  Vector3(l[0],l[1],1) );
            right_points.push_back( Vector3(r[0],r[1],1) );
          }
        }
        catch (...) {}

        try {
          Vector2 r( double(box2.width()  - 1) * i / (num-1.0),
	             double(box2.height() - 1) * j / (num-1.0) );

          Vector3 intersection = cartography::datum_intersection( datum, cam2, r );
          if ( intersection == Vector3() )
            continue;

          Vector2 l = cam1->point_to_pixel( intersection );

          if ( box1.contains( l ) ) {
            left_points.push_back( Vector3(l[0],l[1],1) );
            right_points.push_back( Vector3(r[0],r[1],1) );
          }
        }
        catch (...) {}
        tpc.report_incremental_progress( inc_amount );
      }
    }
    tpc.report_finished();
    
    if (left_points.empty() || right_points.empty())
      vw_throw( ArgumentErr() << "InterestPointMatching: rough_homography_fit failed to generate points! Examine your images, or consider using the option --skip-rough-homography.\n" );

    double thresh_factor = stereo_settings().ip_inlier_factor; // 1/15 by default

    typedef math::HomographyFittingFunctor hfit_func;
    math::RandomSampleConsensus<hfit_func, math::InterestPointErrorMetric>
      ransac( hfit_func(), math::InterestPointErrorMetric(),
	      100, // num iterations
	      norm_2(Vector2(box1.width(),box1.height())) * (1.5*thresh_factor), // inlier threshold
	      left_points.size()/2 // min output inliers
	      );
    Matrix<double> H = ransac( right_points, left_points );
    std::vector<size_t> indices = ransac.inlier_indices(H, right_points, left_points);
    check_homography_matrix(H, left_points, right_points, indices);

    vw_out() << "Projected " << left_points.size()
             << " rays for rough homography.\n";
    vw_out() << "Number of inliers: " << indices.size() << ".\n";
    
    return H;
  }

  Vector2i
  homography_rectification( bool adjust_left_image_size,
			    Vector2i const& left_size,
			    Vector2i const& right_size,
			    std::vector<ip::InterestPoint> const& left_ip,
			    std::vector<ip::InterestPoint> const& right_ip,
			    vw::Matrix<double>& left_matrix,
			    vw::Matrix<double>& right_matrix ) {
    // Reformat the interest points for RANSAC
    std::vector<Vector3>  right_copy = iplist_to_vectorlist(right_ip),
			  left_copy  = iplist_to_vectorlist(left_ip);

    double thresh_factor = stereo_settings().ip_inlier_factor; // 1/15 by default
    
    // Use RANSAC to determine a good homography transform between the images
    math::RandomSampleConsensus<math::HomographyFittingFunctor, math::InterestPointErrorMetric>
      ransac( math::HomographyFittingFunctor(),
	      math::InterestPointErrorMetric(),
	      100, // num iter
              norm_2(Vector2(left_size.x(),left_size.y())) * (1.5*thresh_factor), // inlier thresh
	      left_copy.size()*2/3 // min output inliers
	      );
    Matrix<double> H = ransac(right_copy, left_copy);
    std::vector<size_t> indices = ransac.inlier_indices(H, right_copy, left_copy);
    check_homography_matrix(H, left_copy, right_copy, indices);

    // Set right to a homography that has been refined just to our inliers
    left_matrix  = math::identity_matrix<3>();
    right_matrix = math::HomographyFittingFunctor()(right_copy, left_copy, H);

    // Work out the ideal render size
    BBox2i output_bbox, right_bbox;
    output_bbox.grow( Vector2i(0,0) );
    output_bbox.grow( Vector2i(left_size.x(),0) );
    output_bbox.grow( Vector2i(0,left_size.y()) );
    output_bbox.grow( left_size );

    if (adjust_left_image_size){
      // Crop the left and right images to the shared region. This is
      // done for efficiency.  It may not be always desirable though,
      // as in this case we lose the one-to-one correspondence between
      // original input left image pixels and output disparity/point
      // cloud pixels.
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
      left_matrix (0,2) -= output_bbox.min().x();
      right_matrix(0,2) -= output_bbox.min().x();
      left_matrix (1,2) -= output_bbox.min().y();
      right_matrix(1,2) -= output_bbox.min().y();
    }

    return Vector2i( output_bbox.width(), output_bbox.height() );
  }

  bool
  tri_ip_filtering( std::vector<ip::InterestPoint> const& matched_ip1,
                    std::vector<ip::InterestPoint> const& matched_ip2,
                    vw::camera::CameraModel* cam1,
                    vw::camera::CameraModel* cam2,
                    std::list<size_t>& valid_indices,
                    vw::TransformRef const& left_tx,
                    vw::TransformRef const& right_tx ) {
    typedef std::vector<double> ArrayT;
    ArrayT error_samples( valid_indices.size() );

    // Create the 'error' samples. Which are triangulation error and distance to sphere.
    double angle_tol = vw::stereo::StereoModel::robust_1_minus_cos(stereo_settings().min_triangulation_angle*M_PI/180);

    stereo::StereoModel model( cam1, cam2, stereo_settings().use_least_squares, angle_tol );
    size_t count = 0;
    const double HIGH_ERROR = 9999999;
    BOOST_FOREACH( size_t i, valid_indices ) {
      model( left_tx.reverse (Vector2(matched_ip1[i].x, matched_ip1[i].y)),
             right_tx.reverse(Vector2(matched_ip2[i].x, matched_ip2[i].y)), error_samples[count] );
      // The call returns exactly zero error to indicate a failed ray intersection
      //  so replace it in those cases with a very high error
      if (error_samples[count] == 0)
        error_samples[count] = HIGH_ERROR;
      //vw_out() << "error_samples["<< count <<"] = " << error_samples[count] << std::endl;
      //vw_out() << "diff = " << Vector2(matched_ip1[i].x, matched_ip1[i].y) - 
      //                         Vector2(matched_ip2[i].x, matched_ip2[i].y)                                
      count++;
    }
    VW_ASSERT( count == valid_indices.size(),
	       vw::MathErr() << "tri_ip_filtering: Programmer error. Count indices not aligned." );

    typedef std::vector<std::pair<Vector<double>, Vector<double> > > ClusterT;
    const size_t NUM_CLUSTERS = 2;
    ClusterT error_clusters = gaussian_clustering<ArrayT>(error_samples.begin(), error_samples.end(), NUM_CLUSTERS);

    // The best triangulation error is the one that has the smallest
    // standard deviations. They are focused on the tight pack of
    // inliers. Bring the smaller std-dev cluster to the front as it
    // is what we are interested in.OB
    if ( (error_clusters.front().second[0] > error_clusters.back().second[0]       ) &&
         (error_clusters.back().second[0] != std::numeric_limits<double>::epsilon())   )
      std::swap( error_clusters[0], error_clusters[1] );

    // Determine if we just wrote nothing but outliers (the variance
    // on triangulation is too high).
    if ( error_clusters.front().second[0] > 1e6 ) {
      //vw_out() << "DEBUG Dumping error samples:" << std::endl;
      //for (size_t i=0; i<error_samples.size(); ++i)
      //  vw_out() << matched_ip1[i].x << ", " << matched_ip1[i].y << ", " << error_samples[i] << std::endl;

      vw_out() << "\t    Unable to find inlier cluster, keeping best 70% of points.\n";

      // Instead of failing, take our best guess at which points are inliers
      // - Sort the error, then find an approximate percentile.
      const double CUTOFF_PERCENTILE = 0.7;
      std::vector<double> sorted_error(error_samples);
      std::sort(sorted_error.begin(), sorted_error.end());
      size_t last_good_index = static_cast<double>(sorted_error.size()) * CUTOFF_PERCENTILE;
      double cutoff_value = 0;
      if (last_good_index < sorted_error.size())
        cutoff_value = sorted_error[last_good_index];

      // Treat all points below the new cutoff_value as inliers
      std::list<size_t> filtered_indices;
      size_t c=0;
      for ( std::list<size_t>::iterator i = valid_indices.begin(); i != valid_indices.end(); i++ )
      {
        if (error_samples[c] < cutoff_value)
          filtered_indices.push_back(*i);
        ++c;
      }
      valid_indices = filtered_indices;
      return (!valid_indices.empty());
    } // End of "nothing but outliers" case

    vw_out() << "\t    Inlier cluster:\n"
             << "\t      Triangulation Err: " << error_clusters.front().first[0]
             << " +- " << sqrt( error_clusters.front().second[0] ) << " meters\n";

    // Record indices of points that match our clustering result
    const double escalar1 = 1.0 / sqrt( 2.0 * M_PI * error_clusters.front().second[0] ); // outside exp of normal eq
    const double escalar2 = 1.0 / sqrt( 2.0 * M_PI * error_clusters.back().second[0] );
    const double escalar3 = 1.0 / (2 * error_clusters.front().second[0] ); // inside exp of normal eq
    const double escalar4 = 1.0 / (2 * error_clusters.back().second[0] );
    size_t error_idx = 0;
    size_t prior_valid_size = valid_indices.size();
    for ( std::list<size_t>::iterator i = valid_indices.begin(); i != valid_indices.end(); i++ ) {
      double err_diff_front = error_samples[error_idx]-error_clusters.front().first[0];
      double err_diff_back  = error_samples[error_idx]-error_clusters.back().first[0];

      if (!((escalar1 * exp( (-err_diff_front * err_diff_front) * escalar3 ) ) >
            (escalar2 * exp( (-err_diff_back  * err_diff_back ) * escalar4 ) ) ||
          error_samples[error_idx] < error_clusters.front().first[0]) ) {
        // It's an outlier!
        //vw_out() << "Removing error_samples["<< error_idx <<"] = " << error_samples[error_idx] << std::endl;
        i = valid_indices.erase(i);
        i--; // For loop is going to increment this back up
      }
      error_idx++;
    }
    VW_ASSERT( prior_valid_size == error_idx,
	       vw::MathErr() << "tri_ip_filtering: Programmer error. Indices don't seem to be aligned." );

    return (!valid_indices.empty());
  }

  bool
  stddev_ip_filtering( std::vector<vw::ip::InterestPoint> const& ip1,
		       std::vector<vw::ip::InterestPoint> const& ip2,
		       std::list<size_t>& valid_indices ) {
    const int NUM_STD_FILTER = 4;
    // 4 stddev filtering. Deletes any disparity measurement that is 4
    // stddev away from the measurements of it's local neighbors. We
    // kill off worse offender one at a time until everyone is compliant.
    bool deleted_something;
    do {
      deleted_something = false;
      Matrix<float> locations1( valid_indices.size(), 2 );
      size_t count = 0;
      std::vector<size_t > reverse_lookup  ( valid_indices.size() );
      std::vector<Vector2> disparity_vector( valid_indices.size() );
      BOOST_FOREACH( size_t index, valid_indices ) {
        locations1( count, 0 ) = ip1[index].x;
        locations1( count, 1 ) = ip1[index].y;
        reverse_lookup  [ count ] = index;
        disparity_vector[ count ] = Vector2(ip2[index].x,ip2[index].y) -
			            Vector2(ip1[index].x,ip1[index].y);
        count++;
      }
      math::FLANNTree<float> tree1;
      tree1.load_match_data( locations1, vw::math::FLANN_DistType_L2);

      std::pair<double,size_t> worse_index;
      worse_index.first = 0;
      for ( size_t i = 0; i < valid_indices.size(); i++ ) {
        Vector<int   > indices;
        Vector<double> distance;
        const int NUM_INDICES_TO_GET = 11;
        tree1.knn_search( select_row( locations1, i ),
		          indices, distance, NUM_INDICES_TO_GET );

	      // Bugfix: If there are too few inputs, in rare occasions
	      // some of the outputs are invalid. Not always. Could not
	      // figure this out in reasonable time, the logic is somewhere
	      // deep inside FLANN. Just discard the bad results.
	      std::vector<int> good_indices;
	      for (size_t j = 0; j < indices.size(); j++) {
	        if ((indices[j] < 0) || (indices[j] >= (int)disparity_vector.size()))
                  continue;
	        good_indices.push_back(indices[j]);
	      }

	      // Make an average of the disparities around us and not our own measurement
	      Vector2 sum;
	      for ( size_t j = 1; j < good_indices.size(); j++ ) {
	        sum += disparity_vector[ good_indices[j] ];
	      }
	      sum = normalize( sum );

	      // Project all disparities along the new gradient
	      Vector<double> projections( good_indices.size() );
	      for ( size_t j = 0; j < good_indices.size(); j++ ) {
	        projections[j] = dot_prod( disparity_vector[good_indices[j]], sum );
	      }
	      double mean   = 0;
	      double stddev = 0;
	      for ( size_t j = 1; j < good_indices.size(); j++ ) {
	        mean += projections[j];
	        stddev += projections[j]*projections[j];
	      }
	      mean /= good_indices.size() - 1;
	      stddev = sqrt( stddev / ( good_indices.size() - 1 ) - mean*mean );

	      double std_distance = fabs(projections[0]-mean)/stddev;
	      if ( std_distance > worse_index.first ) {
	        worse_index.first = std_distance;
	        worse_index.second = i;
	      }
      } // End loop through valid indices
      if ( worse_index.first > NUM_STD_FILTER ) {
        std::list<size_t>::iterator it = valid_indices.begin();
        std::advance( it, worse_index.second );
        valid_indices.erase( it );
        deleted_something = true;
      }
      // If we ended up deleting everything, just quit here and return 0.
      if (valid_indices.empty())
        return 0;
    } while( deleted_something );

    return valid_indices.size();
  }

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
   std::vector<vw::ip::InterestPoint> & ip2_out){

    // Handle case where the elevation or lonlat range are not set
    const size_t num_ip = ip1_in.size();                              
    if (elevation_limit[1] <= elevation_limit[0] && lon_lat_limit.empty()) {
      ip1_out = ip1_in;
      ip2_out = ip2_in;
      return num_ip;
    }

    if (elevation_limit[0] < elevation_limit[1]) 
      vw_out() << "\t    * Applying elevation restriction: " << elevation_limit[0]
	       << " to " << elevation_limit[1] << ".\n";

    if (!lon_lat_limit.empty()) 
      vw_out() << "\t    * Applying lon-lat restriction: " << lon_lat_limit.min()
	       << " to " << lon_lat_limit.max() << ".\n";
    
    // Init output vectors
    ip1_out.clear();
    ip2_out.clear();
    ip1_out.reserve(num_ip);
    ip2_out.reserve(num_ip);

    // Set up stereo model
    double angle_tolerance = vw::stereo::StereoModel::robust_1_minus_cos(
                                stereo_settings().min_triangulation_angle*M_PI/180);
    vw::stereo::StereoModel model(left_camera_model, right_camera_model, 
                                  stereo_settings().use_least_squares, angle_tolerance );
    
    // For each interest point, compute the height and only keep it if the height falls within
    // the specified range.
    for (size_t i=0; i<num_ip; ++i) {
      double error;

      // We must not both apply a transform and a scale at the same time
      // as these are meant to do the same thing in different circumstances.
      Vector2 p1 = left_tx.reverse (Vector2(ip1_in[i].x, ip1_in[i].y));
      Vector2 p2 = right_tx.reverse(Vector2(ip2_in[i].x, ip2_in[i].y));
      Vector3 pt  = model(p1/ip_scale, p2/ip_scale, error);

      Vector3 llh = datum.cartesian_to_geodetic(pt);
      if ( (elevation_limit[0] < elevation_limit[1]) && 
	   ( (llh[2] < elevation_limit[0]) || (llh[2] > elevation_limit[1]) ) ) {
        //vw_out() << "Removing IP diff: " << p2 - p1 << " with llh " << llh << std::endl;
        continue;
      }
      
      Vector2 lon_lat = subvector(llh, 0, 2);
      if ( (!lon_lat_limit.empty()) && (!lon_lat_limit.contains(lon_lat)) ) {
	continue; 
      }
      
      //vw_out() << "Keeping IP diff: " << p2 - p1 << " with llh " << llh << std::endl;
      ip1_out.push_back(ip1_in[i]);
      ip2_out.push_back(ip2_in[i]);
    }
    vw_out() << "\t    * Removed " << ip1_in.size() - ip1_out.size() << " ip using elevation/lonlat filtering.\n";
    
    return ip1_out.size();
} // End filter_ip_by_elevation

  // Do IP matching, return, the best translation+scale fitting functor.
  vw::Matrix<double> translation_ip_matching(vw::ImageView<float> const& image1,
                                              vw::ImageView<float> const& image2,
                                              int ip_per_tile,
                                              double nodata1, double nodata2) {

    using namespace vw;

    std::vector<ip::InterestPoint> matched_ip1, matched_ip2;
    detect_match_ip(matched_ip1, matched_ip2, image1,  image2, ip_per_tile,
                    nodata1, nodata2);

    std::vector<Vector3> ransac_ip1 = iplist_to_vectorlist(matched_ip1);
    std::vector<Vector3> ransac_ip2 = iplist_to_vectorlist(matched_ip2);
    vw_out(DebugMessage,"asp") << "\t--> Removed "
                               << matched_ip1.size() - ransac_ip1.size()
                               << " duplicate matches.\n";

    Matrix<double> T;
    std::vector<size_t> indices;
    try {

      vw::math::RandomSampleConsensus<vw::math::TranslationScaleFittingFunctor, vw::math::InterestPointErrorMetric> ransac( vw::math::TranslationScaleFittingFunctor(), vw::math::InterestPointErrorMetric(), 100, 10, ransac_ip1.size()/2, true);
      T = ransac( ransac_ip2, ransac_ip1 );
      indices = ransac.inlier_indices(T, ransac_ip2, ransac_ip1 );
    } catch (...) {
      vw_out(WarningMessage,"console") << "Automatic Alignment Failed! Proceed with caution...\n";
      T = vw::math::identity_matrix<3>();
    }

    { // Keeping only inliers
      std::vector<ip::InterestPoint> inlier_ip1, inlier_ip2;
      for ( size_t i = 0; i < indices.size(); i++ ) {
        inlier_ip1.push_back( matched_ip1[indices[i]] );
        inlier_ip2.push_back( matched_ip2[indices[i]] );
      }
      matched_ip1 = inlier_ip1;
      matched_ip2 = inlier_ip2;
    }

    return T;

  }
  
}
