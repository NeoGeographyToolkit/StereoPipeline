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
#include <vw/Mosaic/ImageComposite.h>
#include <vw/Image/AlgorithmFunctions.h>
#include <vw/Math/RandomSet.h> // Move this when moving the random subset code
#include <vw/Core/Stopwatch.h>

using namespace vw;

namespace asp {

int g_ip_num_errors = 0;
Mutex g_ip_mutex;
  
/// Takes interest points and then finds the nearest 10 matches
/// according to their IP descriptors. It then
/// filters them by whom are closest to the epipolar line via a
/// threshold. The first 2 are then selected to be a match if
/// their descriptor distance is sufficiently far apart.
// TODO(oalexan1): Move this to VW as low-level? 
// VW has the related class InterestPointMatcher.
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
      size_t          number_of_jobs,
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

EpipolarLinePointMatcher::EpipolarLinePointMatcher(bool   single_threaded_camera,
                                                   double uniqueness_threshold,
                                                   double epipolar_threshold,
                                                   vw::cartography::Datum const& datum):
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
  
Vector3 EpipolarLinePointMatcher::epipolar_line(Vector2 const& feature,
                                                cartography::Datum const& datum,
                                                camera::CameraModel* cam_ip,
                                                camera::CameraModel* cam_obj,
                                                bool & success) {
  success = true;

  // Watch out for errors thrown when projecting into the camera
  try{

    // Intersect the interest point pixel with the datum
    Vector3 p0 = cartography::datum_intersection(datum, cam_ip, feature);

    if (p0 == Vector3()) { // No intersection
      success = false;
      return Vector3();
    }

    Vector3 p1  = p0 + 10*cam_ip->pixel_to_vector(feature); // Extend the point below the datum
    Vector2 ep0 = cam_obj->point_to_pixel(p0); // Project the intersection and extension into the other camera
    Vector2 ep1 = cam_obj->point_to_pixel(p1);
    Matrix<double> matrix( 2, 3 );
    select_col( matrix, 2 ) = Vector2(1,1);
    matrix(0,0) = ep0.x();
    matrix(0,1) = ep0.y();
    matrix(1,0) = ep1.x();
    matrix(1,1) = ep1.y();

    if (matrix != matrix) { // Got back NaN values. Can't proceed.
      success = false;
      return Vector3();
    }

    // If the input matrix is bad this can result in some weird errors!
    Matrix<double> nsp = nullspace(matrix);
    if (nsp.cols() <= 0 || nsp.rows() <= 0){ // Failed to find the nullspace
      success = false;
      return Vector3();
    }

    return select_col(nsp,0);

  } catch (std::exception const& e) {
    // Turn this off, it can be verbose
    // Mutex::Lock lock(g_ip_mutex);
    // g_ip_num_errors++;
    // if (g_ip_num_errors < 100) {
    //  vw_out(ErrorMessage) << e.what() << std::endl;
    // }else if (g_ip_num_errors == 100) {
    //  vw_out() << "Will print no more error messages about failing to find epipolar line.\n";
    // }
  }

  success = false;
  return Vector3();
}

// TODO(oalexan1): Use std::abs() instead of fabs(). May change the result.
double EpipolarLinePointMatcher::distance_point_line(Vector3 const& line,
                                                     Vector2 const& point) {
  return fabs(line.x() * point.x() + line.y() * point.y() + line.z() ) /
    norm_2(subvector(line, 0, 2));
}

// Local class definition
class EpipolarLineMatchTask: public Task, private boost::noncopyable {
  typedef ip::InterestPointList::const_iterator IPListIter;
  bool                            m_single_threaded_camera;
  bool                            m_use_uchar_tree;
  math::FLANNTree<float        >& m_tree_float;
  math::FLANNTree<unsigned char>& m_tree_uchar;
  IPListIter                      m_start, m_end;
  ip::InterestPointList const&    m_ip_other;
  camera::CameraModel            *m_cam1, *m_cam2;
  EpipolarLinePointMatcher const& m_matcher;
  Mutex&                          m_camera_mutex;
  std::vector<size_t>::iterator   m_output;
public:
  EpipolarLineMatchTask(bool single_threaded_camera,
                        bool use_uchar_tree,
                        math::FLANNTree<float>        & tree_float,
                        math::FLANNTree<unsigned char>& tree_uchar,
                        ip::InterestPointList::const_iterator start,
                        ip::InterestPointList::const_iterator end,
                        ip::InterestPointList const& ip2,
                        vw::camera::CameraModel* cam1,
                        vw::camera::CameraModel* cam2,
                        EpipolarLinePointMatcher const& matcher,
                        Mutex& camera_mutex,
                        std::vector<size_t>::iterator output):
    m_single_threaded_camera(single_threaded_camera),
    m_use_uchar_tree(use_uchar_tree), m_tree_float(tree_float), m_tree_uchar(tree_uchar),
    m_start(start), m_end(end), m_ip_other(ip2),
    m_cam1(cam1), m_cam2(cam2),
    m_matcher( matcher), m_camera_mutex(camera_mutex), m_output(output) {}

  void operator()() {

    const size_t NUM_MATCHES_TO_FIND = 10;
    Vector<int> indices(NUM_MATCHES_TO_FIND);
    Vector<double> distances(NUM_MATCHES_TO_FIND);

    for (IPListIter ip = m_start; ip != m_end; ip++) {
      Vector2 ip_org_coord = Vector2(ip->x, ip->y);
      Vector3 line_eq;

      // Find the equation that describes the epipolar line
      bool found_epipolar = false;
      if (m_single_threaded_camera){
        // ISIS camera is single-threaded
        Mutex::Lock lock(m_camera_mutex);
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
        for (size_t i=0; i<ip->descriptor.size(); i++)
          uchar_descriptor[i] = static_cast<unsigned char>(ip->descriptor[i]);
        num_matches_valid = m_tree_uchar.knn_search(uchar_descriptor, indices, distances, 
                                                    NUM_MATCHES_TO_FIND);
      } else {
        num_matches_valid = m_tree_float.knn_search(ip->descriptor, indices, distances, 
                                                    NUM_MATCHES_TO_FIND);
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
        auto ip2_it = m_ip_other.begin();
        std::advance( ip2_it, indices[i] );

        if (found_epipolar){
          Vector2 ip2_org_coord = Vector2( ip2_it->x, ip2_it->y );
          double  line_distance = m_matcher.distance_point_line( line_eq, ip2_org_coord );
          if (line_distance < large_epipolar_threshold) {
            if (line_distance < small_epipolar_threshold)
              kept_indices.push_back(std::pair<float,int>(distances[i], indices[i]));
            else // In between thresholds
              kept_indices.push_back(std::pair<float,int>(distances[i], -1));
          }
          else {
            Vector2 ip1_coord(ip->x, ip->y);
            //double normDist = norm_2(ip1_coord - ip2_org_coord);
            //vw_out() << "Discarding match between " << ip1_coord << " and " << ip2_org_coord
            //        << " because distance is " << line_distance << " and threshold is "
            //        << m_matcher.m_epipolar_threshold << " norm dist = " << normDist<<"\n";
          }
        }
      } // End loop for match pruning

        // If we only found one match or the first descriptor match is much better than the second
      if (((kept_indices.size() > 2)     &&
            (kept_indices[0].second >= 0) &&
            (kept_indices[0].first < m_matcher.m_uniqueness_threshold * kept_indices[1].first))
           || (kept_indices.size() == 1)) {
        *m_output++ = kept_indices[0].second; // Return the first of the matches we found
        //vw_out() << "Kept distance: " << kept_indices[0].first << std::endl;
      } else { // No matches or no clear winner
        *m_output++ = (size_t)(-1); // Failed to find a match, return a flag!
      }
    } // End loop through IP
  } // End function operator()

}; // End class EpipolarLineMatchTask -------------------

void EpipolarLinePointMatcher::operator()(ip::InterestPointList const& ip1,
                                          ip::InterestPointList const& ip2,
                                          DetectIpMethod  ip_detect_method,
                                          size_t          number_of_jobs,
                                          camera::CameraModel        * cam1,
                                          camera::CameraModel        * cam2,
                                          std::vector<size_t>        & output_indices) const {
  typedef ip::InterestPointList::const_iterator IPListIter;

  Timer total_time("Total elapsed time", DebugMessage, "interest_point");
  size_t ip1_size = ip1.size(), ip2_size = ip2.size();

  output_indices.clear();
  if (!ip1_size || !ip2_size) {
    vw_out(InfoMessage,"interest_point") << "KD-Tree: no points to match, exiting\n";
    return;
  }

  // Build the output indices
  output_indices.resize(ip1_size);

  // Set up FLANNTree objects of all the different types we may need.
  math::FLANNTree<float>         kd_float;
  math::FLANNTree<unsigned char> kd_uchar;

  Matrix<float>         ip2_matrix_float;
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

  // Robustness fix
  if (ip1_size < number_of_jobs)
    number_of_jobs = ip1_size;

  // Get input and output iterators
  IPListIter start_it = ip1.begin();
  std::vector<size_t>::iterator output_it = output_indices.begin();

  for (size_t i = 0; i < number_of_jobs - 1; i++) {
    // Update iterators and launch the job.
    IPListIter end_it = start_it;
    std::advance(end_it, ip1_size / number_of_jobs);
    boost::shared_ptr<Task>
      match_task(new EpipolarLineMatchTask(m_single_threaded_camera,
                                           use_uchar_FLANN, kd_float, kd_uchar,
                                           start_it, end_it,
                                           ip2, cam1, cam2, *this,
                                           camera_mutex, output_it));
    matching_queue.add_task( match_task );
    start_it = end_it;
    std::advance(output_it, ip1_size / number_of_jobs);
  }
  // Launch the last job.
  // TODO(oalexan1): Integrate this into the above loop.
  boost::shared_ptr<Task>
    match_task(new EpipolarLineMatchTask(m_single_threaded_camera,
                                         use_uchar_FLANN, kd_float, kd_uchar,
                                         start_it, ip1.end(),
                                         ip2, cam1, cam2, *this,
                                         camera_mutex, output_it));
  matching_queue.add_task(match_task);
  matching_queue.join_all(); // Wait for all the jobs to finish.
}

// End class EpipolarLinePointMatcher
//---------------------------------------------------------------------------------------

// Match a set of interest points. This can be invoked for all interest points
// in the image, then multiple threads are used, for for partial subsets,
// when this should be called with a single thread, as mulitple such calls
// may be running in parallel.
void epipolar_ip_matching_task(bool single_threaded_camera,
        DetectIpMethod detect_method, 
        double epipolar_threshold,
        double uniqueness_threshold,
        int number_of_jobs,
        vw::cartography::Datum const& datum,
        bool quiet,
			  vw::camera::CameraModel* cam1,
			  vw::camera::CameraModel* cam2,
        vw::ip::InterestPointList const& ip1,
        vw::ip::InterestPointList const& ip2,
        // Outputs
        std::vector<vw::ip::InterestPoint>& matched_ip1,
        std::vector<vw::ip::InterestPoint>& matched_ip2) {

  std::vector<size_t> forward_match, backward_match;
  EpipolarLinePointMatcher matcher(single_threaded_camera,
				   uniqueness_threshold, epipolar_threshold, datum);

  if (!quiet)           
    vw_out() << "\t    Matching forward" << std::endl;
  
  matcher(ip1, ip2, detect_method, number_of_jobs, cam1, cam2, forward_match); 

  if (!quiet) {
    vw_out() << "\t    ---> Obtained " << forward_match.size() << " matches." << std::endl;
    vw_out() << "\t    Matching backward" << std::endl;
  }

  matcher(ip2, ip1, detect_method, number_of_jobs, cam2, cam1, backward_match);
  
  if (!quiet)
    vw_out() << "\t    ---> Obtained " << backward_match.size() << " matches." << std::endl;

  // Perform circle consistency check
  size_t valid_count = 0;
  const size_t NULL_INDEX = (size_t)(-1);
  for (size_t i = 0; i < forward_match.size(); i++) {
    if (forward_match[i] != NULL_INDEX) {
      if (backward_match[forward_match[i]] != i) {
        forward_match[i] = NULL_INDEX;
      } else {
        valid_count++;
      }
    }
  }

  // Produce listing of valid indices that agree with forward and backward matching
  matched_ip1.reserve(valid_count); // Get our allocations out of the way.
  matched_ip2.reserve(valid_count);

  auto ip1_it = ip1.begin(), ip2_it = ip2.begin();
  for (size_t i = 0; i < forward_match.size(); i++) {
    if (forward_match[i] != NULL_INDEX) {
      matched_ip1.push_back(*ip1_it);
      ip2_it = ip2.begin();
      std::advance(ip2_it, forward_match[i]);
      matched_ip2.push_back(*ip2_it);
    }
    ip1_it++;
  }

  return;
}

// Given a value x, find all n such that x is in the interval 
// [n*tile_size - extra, (n+1)*tile_size + extra]. We assume
// that extra is at most half of tile size, so there are at most 
// 3 such indices.
void tile_extra_indices(double x, int tile_size, int extra, std::vector<int> & indices) {

  // This check should not be reached as the input parameters are validated.
  if (extra > tile_size/2 || extra < 0)
    vw_throw(ArgumentErr() << "tile_indices: extra is too large or too small.\n");

  // wipe the output
  indices.clear();

  int n = (int)floor(x / double(tile_size));
  // iterate in -1, 0, 1
  for (int i = -1; i <= 1; i++) {
    int m = n + i;
    if (m*tile_size - extra <= x && x <= (m+1)*tile_size + extra) {
      indices.push_back(m);
    }
  }

  return;
}

// Divide the left image and right aligned image into corresponding tiles. Find
// the vector interest points in each left tile, and the corresponding one in
// each right tile. These per-tile vectors will later be matched. The returned
// interest points do not have alignment applied to them, that is only used
// during processing. The division into tiles is virtual, as no actual tiles get
// created. TODO(oalexan1): Tiles on the right and bottom edges may be small. We
// do not take that into account. This may result in interest points there be
// too dense.
void group_ip_in_tiles(std::vector<vw::ip::InterestPoint> const& ip1_copy,
                       std::vector<vw::ip::InterestPoint> const& ip2_copy,
                       vw::Matrix<double> align_matrix,
                       // Outputs
                       std::map<std::pair<int, int>, std::vector<vw::ip::InterestPoint>> & ip1_vec,
                       std::map<std::pair<int, int>, std::vector<vw::ip::InterestPoint>> & ip2_vec) {

  // Wipe the output vectors
  ip1_vec.clear();
  ip2_vec.clear();

  std::cout << "alignment transform is " << align_matrix << std::endl;
  HomographyTransform align_trans(align_matrix);

  std::cout << "--ip per tile is " << asp::stereo_settings().ip_per_tile << std::endl;
  std::cout << "matches per tile is " << asp::stereo_settings().matches_per_tile << std::endl;
  std::cout << "ip per image is " << asp::stereo_settings().ip_per_image << std::endl;
  
  std::cout << "will do matches per tile!\n";
  std::cout << "For last tile, must expand inward!\n";

  // We will have tiles overlap, to ensure no interest points
  // fall between the cracks. That may happen if alignment is not perfect.
  Vector2i params = asp::stereo_settings().matches_per_tile_params;
  int tile_size = params[0];
  int extra = (params[1] - params[0])/2;

  // Map each ip to a box. 
  std::cout << "size of copy is " << ip1_copy.size() << std::endl;
  std::cout << "don't make copy, iterate over original ip using foreach!\n";
  for (int i = 0; i < (int)ip1_copy.size(); i++) {
    auto & ip = ip1_copy[i];
    vw::Vector2 pt(ip.x, ip.y);
    // Doing floor on purpose, so ip in [0, tile_size]^2 go to first box
    int cx = floor(pt[0]/double(tile_size));
    int cy = floor(pt[1]/double(tile_size));
    auto p = std::make_pair(cx, cy);
    ip1_vec[p].push_back(ip1_copy[i]);
  }
  // Repeat for ip2. First bring to image1 coordinates
  for (int i = 0; i < (int)ip2_copy.size(); i++) {
    auto & ip = ip2_copy[i];

    // Bring to left image coordinates
    vw::Vector2 pt(ip.x, ip.y); 
    pt = align_trans.forward(pt);

    // Find all tiles of size tile_size with padding of extra on each side to
    // which this belongs. For example, assume for the moment that tiles are in
    // 1D. Then, the value 3 * tile_size - 0.8 * extra will belong to both tile
    // 3 and tile 2. Value 3 * tile_size + 0.7 * extra will belong to both tile
    // 3 and tile 4.
    std::vector<int> x_indices, y_indices;
    tile_extra_indices(pt[0], tile_size, extra, x_indices);
    tile_extra_indices(pt[1], tile_size, extra, y_indices);
    for (int cx : x_indices) {
      for (int cy : y_indices) {
        auto p = std::make_pair(cx, cy);
        ip2_vec[p].push_back(ip2_copy[i]);
      }
    }
  }

  return;
}

void append_new_matches(
          // Inputs
          std::vector<vw::ip::InterestPoint> const& local_matched_ip1,
          std::vector<vw::ip::InterestPoint> const& local_matched_ip2,
          // Outputs (append)
          std::set<std::pair<float, float>> & set1,
          std::set<std::pair<float, float>> & set2,
          std::vector<vw::ip::InterestPoint> & matched_ip1,
          std::vector<vw::ip::InterestPoint> & matched_ip2) {

  for (int i = 0; i < (int)local_matched_ip1.size(); i++) {
    auto & ip1 = local_matched_ip1[i];
    auto & ip2 = local_matched_ip2[i];
    auto p1 = std::make_pair(ip1.x, ip1.y);
    auto p2 = std::make_pair(ip2.x, ip2.y);
    if (set1.find(p1) == set1.end() && set2.find(p2) == set2.end()) {
      matched_ip1.push_back(ip1);
      matched_ip2.push_back(ip2);
      set1.insert(p1);
      set2.insert(p2);
    }
  }
  return;
}

// Make the code above a function for picking a subset
// TODO(oalexan1): Call it in ControlNetworkLoader.cc
void pick_subset(int max_pairwise_matches, 
  std::vector<ip::InterestPoint> & ip1, 
  std::vector<ip::InterestPoint> & ip2) {

  std::vector<int> subset;
  vw::math::pick_random_indices_in_range(ip1.size(), max_pairwise_matches, subset);
  std::sort(subset.begin(), subset.end()); // sort the indices; not strictly necessary

  std::vector<ip::InterestPoint> ip1_full, ip2_full;
  ip1_full.swap(ip1);
  ip2_full.swap(ip2);

  ip1.resize(max_pairwise_matches);
  ip2.resize(max_pairwise_matches);
  for (size_t it = 0; it < subset.size(); it++) {
    ip1[it] = ip1_full[subset[it]];
    ip2[it] = ip2_full[subset[it]];
  }
}

// TODO(oalexan1): Break up this function as it is too long.
bool epipolar_ip_matching(bool single_threaded_camera,
        vw::ip::InterestPointList const& ip1,
        vw::ip::InterestPointList const& ip2,
        vw::camera::CameraModel* cam1,
        vw::camera::CameraModel* cam2,
        vw::ImageViewRef<float> const& image1,
        vw::ImageViewRef<float> const& image2,
        vw::cartography::Datum const& datum,
        double epipolar_threshold,
        double uniqueness_threshold,
        double nodata1, double nodata2,
        bool match_per_tile, 
        vw::Matrix<double> const& align_matrix, // used only if match_per_tile is true 
        // Outputs
        std::vector<vw::ip::InterestPoint>& matched_ip1,
        std::vector<vw::ip::InterestPoint>& matched_ip2) {
  
  matched_ip1.clear();
  matched_ip2.clear();

  std::cout << "remove the copy\n";
  std::vector<vw::ip::InterestPoint> ip1_copy, ip2_copy;
  ip1_copy.reserve(ip1.size());
  ip2_copy.reserve(ip2.size());
  std::copy(ip1.begin(), ip1.end(), std::back_inserter(ip1_copy));
  std::copy(ip2.begin(), ip2.end(), std::back_inserter(ip2_copy));

  // Match interest points forward/backward .. constraining on epipolar line
  DetectIpMethod detect_method 
    = static_cast<DetectIpMethod>(stereo_settings().ip_matching_method);

  // Sanity check
  if (match_per_tile && asp::stereo_settings().matches_per_tile <= 0)
  vw::vw_throw(vw::ArgumentErr()
    << "epipolar_ip_matching: matches_per_tile must be positive.\n");

  vw_out() << "\t--> Matching interest points using the epipolar line." << std::endl;
  vw_out() << "\t    Uniqueness threshold: " << uniqueness_threshold << "\n";
  vw_out() << "\t    Epipolar threshold:   " << epipolar_threshold   << "\n";
  
  // Jobs set to 2x the number of cores. This is just incase all jobs are not equal.
  // The total number of interest points will be divided up among the jobs.
  size_t number_of_jobs = vw_settings().default_num_threads() * 2;
#if __APPLE__
  // Fix due to OpenBLAS crashing and/or giving different results
  // each time. May need to be revisited.
  number_of_jobs = std::min(int(vw_settings().default_num_threads()), 1);
  vw_out() << "\t    Using " << number_of_jobs << " thread(s) for matching.\n";
#endif

  // TODO(oalexan1): Must use one job if using multiple tiles
  // TODO(oalexan1): Tile size is 1024x1024. 
  // To put right image ip in tile need to apply alignment. May also need to 
  // Make the tiles have some overlap then remove duplicates lr and rl.

  // TODO(oalexan1): Mapproject both images to same extent. Divide into tiles,
  // run on each tile, combine, make unique, l-r, and r-l. Save to disk. 
  // If that works, consider estimating the alignment. 

  // Note that actually the code to change is not here, but in homography!

  // Do matching
  if (!match_per_tile) {
    bool quiet = false; // has to be true if many such jobs exist
    epipolar_ip_matching_task(single_threaded_camera, detect_method, 
          epipolar_threshold, uniqueness_threshold, number_of_jobs, 
          datum, quiet, cam1, cam2, ip1, ip2,
          matched_ip1, matched_ip2); // outputs
  } else {
    bool quiet = false;
    std::cout << "must make quiet be true\n";
    number_of_jobs = 1;

    std::cout << "alignment transform is " << align_matrix << std::endl;
    std::cout << "---matches per tile is " << asp::stereo_settings().matches_per_tile <<  std::endl;

    std::map<std::pair<int, int>, std::vector<vw::ip::InterestPoint>> ip1_vec, ip2_vec;

    std::cout << "alignment transform is " << align_matrix << std::endl;

    group_ip_in_tiles(ip1_copy, ip2_copy, align_matrix, 
      ip1_vec, ip2_vec); // outputs

    std::cout << "--now here\n";
    std::cout << "ip1 vec size is " << ip1_vec.size() << std::endl;
    std::cout << "ip2 vec size is " << ip2_vec.size() << std::endl;

    matched_ip1.clear(); 
    matched_ip2.clear();
    std::set<std::pair<float, float>> set1, set2; // to help find unique matches
    // Now iterate over each set of ip in ip1_vec
    for (auto it1 = ip1_vec.begin(); it1 != ip1_vec.end(); it1++) {

      std::cout << "--now start in tile\n";

      // Find the corresponding set of ip in ip1_vec and ip2_vec
      auto key = it1->first;
      auto it2 = ip2_vec.find(key);
      if (it2 == ip2_vec.end()) 
        continue; // no matches in this tile

      std::vector<vw::ip::InterestPoint> & tile_ip1 = it1->second;
      std::vector<vw::ip::InterestPoint> & tile_ip2 = it2->second;

      // Copy from std::vector<vw::ip::InterestPoint> to 
      // vw::ip::InterestPointList
      std::cout << "Must remove this copy\n";
      vw::ip::InterestPointList ip1_list, ip2_list;
      std::cout << "---input size is " << tile_ip1.size() << std::endl;
      std::cout << "---input size is " << tile_ip2.size() << std::endl;
      for (size_t i = 0; i < tile_ip1.size(); i++)
        ip1_list.push_back(tile_ip1[i]);
      for (size_t i = 0; i < tile_ip2.size(); i++)
        ip2_list.push_back(tile_ip2[i]);
      
      std::cout << "--output size is " << ip1_list.size() << std::endl;
      std::cout << "--output size is " << ip2_list.size() << std::endl;
      std::cout << "--list size is " << ip1_list.size() << std::endl;
      std::vector<vw::ip::InterestPoint> local_matched_ip1, local_matched_ip2;
      try {
        epipolar_ip_matching_task(single_threaded_camera, detect_method, 
            epipolar_threshold, uniqueness_threshold, number_of_jobs, 
            datum, quiet, cam1, cam2, ip1_list, ip2_list,
            local_matched_ip1, local_matched_ip2); // outputs
      }catch(...){
        // This need not succeed
        continue;
      }
      std::cout << "end of epipolar_ip_matching_task\n";

      // Add to the global list

      std::cout << "number of matches in box is " << local_matched_ip1.size() << std::endl;
      std::cout << "number of matches in box is " << local_matched_ip2.size() << std::endl;
      std::cout << "---now 1\n";
      std::cout << "matches per tile is " << asp::stereo_settings().matches_per_tile << std::endl;
      if (local_matched_ip1.size() > asp::stereo_settings().matches_per_tile)
        pick_subset(asp::stereo_settings().matches_per_tile, 
          local_matched_ip1, local_matched_ip2);

      std::cout << "now 2\n";
      
      std::cout << "will append new matches\n"; 
      append_new_matches(local_matched_ip1, local_matched_ip2,
         set1, set2, matched_ip1, matched_ip2); // append here
      std::cout << "--end of append new matches\n";

    } // end iterating over sets of ip
  } // end if matches per tile

  vw_out() << "\t    Matched " << matched_ip1.size() << " points." << std::endl;

  if (matched_ip1.empty())
    return false;

  // Write out debug image
  if (stereo_settings().ip_debug_images) {
    vw_out() << "\t    Writing IP match debug image prior to geometric filtering.\n";
    write_match_image("InterestPointMatching__ip_matching_debug.tif",
                      image1, image2, matched_ip1, matched_ip2);
  }

  // TODO(oalexan1): All the filtering logic below must be in a separate function
  // that will be called here.

  // Apply filtering of IP by a selection of assumptions. Low
  // triangulation error, agreement with klt tracking, and local
  // neighbors are the same neighbors in both images.
  std::list<size_t> good_indices;
  for (size_t i = 0; i < matched_ip1.size(); i++)
    good_indices.push_back(i);
  
  if (!stereo_settings().disable_tri_filtering) {
    if (!tri_ip_filtering(matched_ip1, matched_ip2,
			  cam1, cam2, good_indices)){
      vw_out() << "No interest points left after triangulation filtering." << std::endl;
      return false;
    }
  }
  if (!stddev_ip_filtering(matched_ip1, matched_ip2, good_indices)) {
    vw_out() << "No interest points left after stddev filtering." << std::endl;
    return false;
  }

  // Record new list that contains only the inliers.
  vw_out() << "\t    Reduced matches to " << good_indices.size() << "\n";
  std::vector<ip::InterestPoint> buffer(good_indices.size());

  // Subselect and copy ip1
  size_t w_index = 0;
  BOOST_FOREACH(size_t index, good_indices) {
    Vector2 l(matched_ip1[index].x, matched_ip1[index].y);
    matched_ip1[index].ix = matched_ip1[index].x = l.x();
    matched_ip1[index].iy = matched_ip1[index].y = l.y();
    buffer[w_index] = matched_ip1[index];
    w_index++;
  }
  matched_ip1 = buffer;

  // Subselect and copy ip2
  w_index = 0;
  BOOST_FOREACH(size_t index, good_indices) {
    Vector2 r(matched_ip2[index].x, matched_ip2[index].y);
    matched_ip2[index].ix = matched_ip2[index].x = r.x();
    matched_ip2[index].iy = matched_ip2[index].y = r.y();
    buffer[w_index] = matched_ip2[index];
    w_index++;
  }
  matched_ip2 = buffer;

  // The interest points are not aligned here, so there's no need to align them
  vw::TransformPtr tx1(NULL);
  vw::TransformPtr tx2(NULL);
    
  // If options are set, filter by elevation.
  if (stereo_settings().elevation_limit[0] < stereo_settings().elevation_limit[1] ||
      !stereo_settings().lon_lat_limit.empty()) {

    std::vector<ip::InterestPoint> matched_ip1_out, matched_ip2_out;
    filter_ip_by_lonlat_and_elevation(tx1, tx2,
                                      cam1, cam2,
                                      datum,  matched_ip1, matched_ip2,
                                      stereo_settings().elevation_limit,  
                                      stereo_settings().lon_lat_limit,  
                                      matched_ip1_out, matched_ip2_out);
    matched_ip1 = matched_ip1_out;
    matched_ip2 = matched_ip2_out;
  } // End elevation filtering

  if (stereo_settings().ip_debug_images) {
    vw_out() << "\t    Writing IP match debug image after geometric filtering.\n";
    write_match_image("InterestPointMatching__ip_matching_debug2.tif",
                      image1, image2, matched_ip1, matched_ip2);
  }

  return true;
} // End function epipolar_ip_matching

void check_homography_matrix(Matrix<double>       const& H,
                             std::vector<Vector3> const& left_points,
                             std::vector<Vector3> const& right_points,
                             std::vector<size_t>  const& indices) {

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
rough_homography_fit(camera::CameraModel* cam1,
                     camera::CameraModel* cam2,
                     BBox2i const& box1, BBox2i const& box2,
                     cartography::Datum const& datum) {
    
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
        Vector2 r(double(box2.width()  - 1) * i / (num-1.0),
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
  vw_out() << "Projected " << left_points.size()
           << " rays for rough homography.\n";
    
  if (left_points.empty() || right_points.empty())
    vw_throw( ArgumentErr() << "InterestPointMatching: rough_homography_fit failed to generate points! Examine your images, or consider using the options --skip-rough-homography and --no-datum.\n" );

  double thresh_factor = stereo_settings().ip_inlier_factor; // 1/15 by default
  typedef math::HomographyFittingFunctor hfit_func;
  int min_num_inliers = left_points.size()/2;
  vw_out() << "Estimating rough homography using RANSAC with " 
    << asp::stereo_settings().ip_num_ransac_iterations << " iterations.\n";
  Stopwatch sw;
  sw.start();
  double inlier_thresh = norm_2(Vector2(box1.width(),box1.height())) * (1.5*thresh_factor);
  math::RandomSampleConsensus<hfit_func, math::InterestPointErrorMetric>
    ransac(hfit_func(), math::InterestPointErrorMetric(),
           asp::stereo_settings().ip_num_ransac_iterations,
           inlier_thresh, min_num_inliers);

  Matrix<double> H = ransac(right_points, left_points);
  std::vector<size_t> indices = ransac.inlier_indices(H, right_points, left_points);
  check_homography_matrix(H, left_points, right_points, indices);

  vw_out() << "Number of inliers: " << indices.size() << ".\n";
  sw.stop();
  vw_out() << "RANSAC time: " << sw.elapsed_seconds() << " seconds.\n";

  return H;
}

Vector2i homography_rectification(bool adjust_left_image_size,
                                  Vector2i const& left_size,
                                  Vector2i const& right_size,
                                  std::vector<ip::InterestPoint> const& left_ip,
                                  std::vector<ip::InterestPoint> const& right_ip,
                                  vw::Matrix<double>& left_matrix,
                                  vw::Matrix<double>& right_matrix) {
  // Reformat the interest points for RANSAC
  std::vector<Vector3>  right_copy = iplist_to_vectorlist(right_ip),
    left_copy  = iplist_to_vectorlist(left_ip);

  double thresh_factor = stereo_settings().ip_inlier_factor; // 1/15 by default
    
  // Use RANSAC to determine a good homography transform between the images
  int min_inliers = left_copy.size()*2/3;
  double inlier_th = norm_2(Vector2(left_size.x(),left_size.y())) * (1.5*thresh_factor);
  math::RandomSampleConsensus<math::HomographyFittingFunctor, math::InterestPointErrorMetric>
    ransac(math::HomographyFittingFunctor(),
            math::InterestPointErrorMetric(),
            stereo_settings().ip_num_ransac_iterations,
            inlier_th, min_inliers);
  Matrix<double> H = ransac(right_copy, left_copy);
  std::vector<size_t> indices = ransac.inlier_indices(H, right_copy, left_copy);
  check_homography_matrix(H, left_copy, right_copy, indices);

  // Set right to a homography that has been refined just to our inliers.
  // May be adjusted with a translation below.
  // TODO(oalexan1): It is not clear if adjusting a homography transform's
  // translation coefficients is the right way of applying a translation to it,
  // because it has a denominator too.
  left_matrix  = math::identity_matrix<3>();
  right_matrix = math::HomographyFittingFunctor()(right_copy, left_copy, H);

  // Work out the ideal render size
  BBox2i output_bbox, right_bbox;
  output_bbox.grow(Vector2i(0, 0));
  output_bbox.grow(Vector2i(left_size.x(), 0));
  output_bbox.grow(Vector2i(0, left_size.y()));
  output_bbox.grow(left_size);

  if (adjust_left_image_size) {
    // Crop the left and right images to the shared region. This is
    // done for efficiency.  It may not be always desirable though,
    // as in this case we lose the one-to-one correspondence between
    // original input left image pixels and output disparity/point
    // cloud pixels.
    Vector3 temp = right_matrix*Vector3(0,0,1);
    temp /= temp.z();
    right_bbox.grow(subvector(temp,0,2));
    temp = right_matrix*Vector3(right_size.x(),0,1);
    temp /= temp.z();
    right_bbox.grow(subvector(temp,0,2));
    temp = right_matrix*Vector3(0,right_size.y(),1);
    temp /= temp.z();
    right_bbox.grow(subvector(temp,0,2));
    temp = right_matrix*Vector3(right_size.x(),right_size.y(),1);
    temp /= temp.z();
    right_bbox.grow(subvector(temp,0,2));

    output_bbox.crop(right_bbox );

    //  Move the ideal render size to be aligned up with origin
    left_matrix (0,2) -= output_bbox.min().x();
    right_matrix(0,2) -= output_bbox.min().x();
    left_matrix (1,2) -= output_bbox.min().y();
    right_matrix(1,2) -= output_bbox.min().y();
  }

  return Vector2i(output_bbox.width(), output_bbox.height());
}

/// Remove points in/out of a bounding box depending on "remove_outside".
/// - Returns the number of points removed.
size_t remove_ip_bbox(vw::BBox2i const& roi, vw::ip::InterestPointList & ip_list,
                      bool remove_outside){
  // Loop through all the points
  size_t num_removed = 0;
  vw::ip::InterestPointList::iterator ip;
  for (ip = ip_list.begin(); ip != ip_list.end(); ++ip) {
    
    if (roi.contains(vw::Vector2i(ip->ix,ip->iy)) xor remove_outside) {
      ip = ip_list.erase(ip);
      ++num_removed;
      --ip;
    }
  }
  return num_removed;
} // End function remove_ip_bbox
  
void side_ip_filtering(vw::ip::InterestPointList& ip1, 
                       vw::ip::InterestPointList& ip2,  
                       vw::BBox2i const& bbox1, vw::BBox2i const& bbox2) {
      
  // Filter out IP from the opposite sides of the two images.
  // - Would be better to just pass an ROI into the IP detector!
  if (stereo_settings().ip_edge_buffer_percent <= 0)
    return;
      
  // Figure out removal bboxes
  double percent  = static_cast<double>(stereo_settings().ip_edge_buffer_percent)/100.0;
  int   width_left  = floor(static_cast<double>(bbox1.width()) * percent);
  int   width_right = floor(static_cast<double>(bbox2.width()) * percent);
  BBox2 bbox_left (width_left, 0, bbox1.width()-width_left,  bbox1.height());
  BBox2 bbox_right(0,          0, bbox2.width()-width_right, bbox2.height());
  bool  remove_outside = true;
    
  // Remove the points
  size_t num_removed_left  = remove_ip_bbox(bbox_left,  ip1, remove_outside);
  size_t num_removed_right = remove_ip_bbox(bbox_right, ip2, remove_outside);
  vw_out() << "Removed: " << num_removed_left << " points from the left side of the left image and "
           << num_removed_right << " points from the right side of the right image.\n";
} // End side IP filtering
  
bool tri_ip_filtering( std::vector<ip::InterestPoint> const& matched_ip1,
                  std::vector<ip::InterestPoint> const& matched_ip2,
                  vw::camera::CameraModel* cam1,
                  vw::camera::CameraModel* cam2,
                  std::list<size_t>& valid_indices ) {
  typedef std::vector<double> ArrayT;
  ArrayT error_samples( valid_indices.size() );

  // Create the 'error' samples. Which are triangulation error and distance to sphere.
  double angle_tol = vw::stereo::StereoModel::robust_1_minus_cos(stereo_settings().min_triangulation_angle*M_PI/180);

  stereo::StereoModel model( cam1, cam2, stereo_settings().use_least_squares, angle_tol );
  size_t count = 0;
  const double HIGH_ERROR = 9999999;
  BOOST_FOREACH( size_t i, valid_indices ) {
    model( Vector2(matched_ip1[i].x, matched_ip1[i].y),
           Vector2(matched_ip2[i].x, matched_ip2[i].y), error_samples[count] );
    // The call returns exactly zero error to indicate a failed ray intersection
    //  so replace it in those cases with a very high error
    if (error_samples[count] == 0)
      error_samples[count] = HIGH_ERROR;
    count++;
  }
  VW_ASSERT(count == valid_indices.size(),
            vw::MathErr() << "Error: tri_ip_filtering: count indices not aligned.");

  typedef std::vector<std::pair<Vector<double>, Vector<double> > > ClusterT;
  const size_t NUM_CLUSTERS = 2;
  ClusterT error_clusters = gaussian_clustering<ArrayT>(error_samples.begin(), 
      error_samples.end(), NUM_CLUSTERS);

  // The best triangulation error is the one that has the smallest
  // standard deviations. They are focused on the tight pack of
  // inliers. Bring the smaller std-dev cluster to the front as it
  // is what we are interested in.OB
  if ((error_clusters.front().second[0] > error_clusters.back().second[0]) &&
      (error_clusters.back().second[0] != std::numeric_limits<double>::epsilon()))
    std::swap( error_clusters[0], error_clusters[1] );

  // Determine if we just wrote nothing but outliers (the variance
  // on triangulation is too high).
  if ( error_clusters.front().second[0] > 1e6 ) {

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
    // The maximum triangulation error still applies!
    if ((asp::stereo_settings().ip_triangulation_max_error > 0) && 
        (cutoff_value > asp::stereo_settings().ip_triangulation_max_error))
      cutoff_value = asp::stereo_settings().ip_triangulation_max_error;

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
           << "\t      Triangulation error: " << error_clusters.front().first[0]
           << " +- " << sqrt( error_clusters.front().second[0] ) << " meters\n";

  // Record indices of points that match our clustering result
  const double escalar1 = 1.0 / sqrt( 2.0 * M_PI * error_clusters.front().second[0] ); // outside exp of normal eq
  const double escalar2 = 1.0 / sqrt( 2.0 * M_PI * error_clusters.back().second[0] );
  const double escalar3 = 1.0 / (2 * error_clusters.front().second[0] ); // inside exp of normal eq
  const double escalar4 = 1.0 / (2 * error_clusters.back().second[0] );
  size_t error_idx        = 0;
  size_t prior_valid_size = valid_indices.size();
  size_t outlier_count    = 0;
  for ( std::list<size_t>::iterator i = valid_indices.begin(); i != valid_indices.end(); i++ ) {
    double err_diff_front = error_samples[error_idx]-error_clusters.front().first[0];
    double err_diff_back  = error_samples[error_idx]-error_clusters.back ().first[0];

    if ( !((escalar1 * exp( (-err_diff_front * err_diff_front) * escalar3 ) ) >
           (escalar2 * exp( (-err_diff_back  * err_diff_back ) * escalar4 ) ) ||
           error_samples[error_idx] < error_clusters.front().first[0]          ) ||
         ( (asp::stereo_settings().ip_triangulation_max_error > 0) &&
           (error_samples[error_idx] > asp::stereo_settings().ip_triangulation_max_error) ) ) {
      // It's an outlier!
      //vw_out() << "Removing error_samples["<< error_idx <<"] = " << error_samples[error_idx] << std::endl;
      i = valid_indices.erase(i);
      i--; // For loop is going to increment this back up
      ++outlier_count;
    }
    error_idx++;
  }
  VW_ASSERT( prior_valid_size == error_idx,
             vw::MathErr() << "tri_ip_filtering: Programmer error. Indices don't seem to be aligned." );

  vw_out() << "\t      Removed " << outlier_count << " points in triangulation filtering.\n";
  return (!valid_indices.empty());
}

bool stddev_ip_filtering( std::vector<vw::ip::InterestPoint> const& ip1,
                          std::vector<vw::ip::InterestPoint> const& ip2,
                          std::list<size_t>& valid_indices ) {
  const int NUM_STD_FILTER = 4;
  // 4 stddev filtering. Deletes any disparity measurement that is 4
  // stddev away from the measurements of it's local neighbors. We
  // kill off worse offender one at a time until everyone is compliant.
  bool deleted_something;
  size_t num_deleted = 0;
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
    tree1.load_match_data(locations1, vw::math::FLANN_DistType_L2);

    std::pair<double,size_t> worse_index;
    worse_index.first = 0;
    for ( size_t i = 0; i < valid_indices.size(); i++ ) {
      Vector<int   > indices;
      Vector<double> distance;
      const int NUM_INDICES_TO_GET = 11;
      tree1.knn_search(select_row( locations1, i ),
                       indices, distance, NUM_INDICES_TO_GET);

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
      ++num_deleted;
    }
    // If we ended up deleting everything, just quit here and return 0.
    if (valid_indices.empty())
      return 0;
  } while( deleted_something );

  vw_out() << "\t      Removed " << num_deleted << " points in stddev filtering.\n";
  return valid_indices.size();
}

size_t filter_ip_by_lonlat_and_elevation(vw::TransformPtr         tx_left,
                                         vw::TransformPtr         tx_right,
                                         vw::camera::CameraModel* left_camera_model,
                                         vw::camera::CameraModel* right_camera_model,
                                         vw::cartography::Datum        const& datum,
                                         std::vector<vw::ip::InterestPoint> const& ip1_in,
                                         std::vector<vw::ip::InterestPoint> const& ip2_in,
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
    vw_out() << "\t    * Applying elevation restriction. Height range: " << elevation_limit[0]
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
  double angle_tolerance = vw::stereo::StereoModel::robust_1_minus_cos
    (stereo_settings().min_triangulation_angle*M_PI/180);
  vw::stereo::StereoModel model(left_camera_model, right_camera_model, 
                                stereo_settings().use_least_squares, angle_tolerance);
    
  // This function can be called with both unaligned and aligned interest points
  bool aligned_ip = (tx_left.get() != NULL && tx_right != NULL);

  // For each interest point, compute the height and only keep it if the height falls within
  // the specified range.
  for (size_t i = 0; i < num_ip; i++) {

    // We must not both apply a transform and a scale at the same time
    // as these are meant to do the same thing in different circumstances.
    Vector2 p1 = Vector2(ip1_in[i].x, ip1_in[i].y);
    Vector2 p2 = Vector2(ip2_in[i].x, ip2_in[i].y);

    if (aligned_ip) {
      // Unalign
      p1 = tx_left->reverse (p1);
      p2 = tx_right->reverse(p2);
    }
      
    // Triangulate
    double err = -1.0;
    Vector3 xyz(0.0, 0.0, 0.0);
    try {
      xyz = model(p1, p2, err);
    } catch(...) {
      xyz = Vector3();
    }

    if (err <= 0.0 || xyz == Vector3()) {
      // Triangulation failed
      continue;
    }
      
    Vector3 llh = datum.cartesian_to_geodetic(xyz);
    if ( (elevation_limit[0] < elevation_limit[1]) && 
         ( (llh[2] < elevation_limit[0]) || (llh[2] > elevation_limit[1]) ) ) {
      // vw_out() << "Removing IP diff: " << p2 - p1 << " with llh " << llh << std::endl;
      continue;
    }
      
    Vector2 lon_lat = subvector(llh, 0, 2);
    if ( (!lon_lat_limit.empty()) && (!lon_lat_limit.contains(lon_lat)) ) {
      continue; 
    }
      
    // vw_out() << "Keeping IP diff: " << p2 - p1 << " with llh " << llh << std::endl;
    ip1_out.push_back(ip1_in[i]);
    ip2_out.push_back(ip2_in[i]);
  }
  vw_out() << "\t    * Removed " << ip1_in.size() - ip1_out.size()
           << " ip using elevation/lonlat filtering.\n";
    
  return ip1_out.size();
} // End filter_ip_by_elevation
  
// Filter ip by triangulation error and height range.
// This assumes the interest points are for the original images, without alignment
// or mapprojection.
// TODO(oalexan1): Add here the ability to reverse the alignment
void filter_ip_using_cameras(std::vector<vw::ip::InterestPoint> & ip1,
                             std::vector<vw::ip::InterestPoint> & ip2,
                             vw::camera::CameraModel const * cam1,
                             vw::camera::CameraModel const * cam2,
                             vw::cartography::Datum  const & datum,
                             double pct, double factor) {

  std::vector<double> tri_errors(ip1.size()), heights(ip1.size());
    
  // Compute the triangulation errors
  double angle_tol = vw::stereo::StereoModel
    ::robust_1_minus_cos(stereo_settings().min_triangulation_angle*M_PI/180);
  stereo::StereoModel model(cam1, cam2, stereo_settings().use_least_squares, angle_tol);
  double HIGH_ERROR = std::numeric_limits<double>::max();
  for (size_t i = 0; i < ip1.size(); i++) {
    Vector3 xyz;
    try {
      xyz = model(Vector2(ip1[i].x, ip1[i].y), Vector2(ip2[i].x, ip2[i].y),
                  tri_errors[i]);
    } catch(...) {
      xyz = Vector3();
    }
      
    // The call returns the zero tri error and zero xyz to indicate a
    // failed ray intersection so replace it in those cases with a
    // very high error.
    if (tri_errors[i] == 0 || xyz == Vector3()) {
      tri_errors[i]  = HIGH_ERROR;
      heights[i] = HIGH_ERROR;
      continue;
    }

    // Find the height above datum
    Vector3 llh = datum.cartesian_to_geodetic(xyz);
    heights[i] = llh[2];
  }

  // Put the valid heights in a vector
  std::vector<double> vals;
  vals.reserve(ip1.size());
  vals.clear();
  for (size_t i = 0; i < ip1.size(); i++) {
    if (heights[i] >= HIGH_ERROR) continue;
    vals.push_back(heights[i]);
  }

  // Find the outlier brackets
  double pct_fraction = 1.0 - pct/100.0;
  double b = -1.0, e = -1.0;
  vw::math::find_outlier_brackets(vals, pct_fraction, factor, b, e);
    
  // Apply the outlier threshold
  int count = 0;
  for (size_t i = 0; i < ip1.size(); i++) {
    if (heights[i] >= HIGH_ERROR) continue;
    if (heights[i] < b || heights[i] > e) {
      heights[i] = HIGH_ERROR;
      count++;
    }
  }
  vw_out() << "Number (and fraction) of removed outliers by the height check: "
           << count << " (" << double(count)/ip1.size() << ").\n";

  // Find the valid tri errors. Make use of the fact that we already filtered by height.
  vals.clear();
  for (size_t i = 0; i < ip1.size(); i++) {
    if (tri_errors[i] >= HIGH_ERROR || heights[i] >= HIGH_ERROR) // already invalid
      continue;
    vals.push_back(tri_errors[i]);
  }

  // Find the outlier brackets. Since the triangulation errors, unlike
  // the heights, are usually rather uniform, adjust pct from 95 to
  // 75.
  double pct2 = (75.0/95.0) * pct;
  double pct_fraction2 = 1.0 - pct2/100.0;
  // Show some lenience below as due to jitter some errors could be somewhat bigger
  double factor2 = 2.0 * factor;
  b = -1.0; e = -1.0;
  vw::math::find_outlier_brackets(vals, pct_fraction2, factor2, b, e);

  // Apply the outlier threshold
  count = 0;
  for (size_t i = 0; i < ip1.size(); i++) {
    if (heights[i] >= HIGH_ERROR || tri_errors[i] >= HIGH_ERROR) continue; // already invalid
    // We will ignore b, as the triangulation errors are non-negative.
    if (tri_errors[i] > e) {
      tri_errors[i] = HIGH_ERROR;
      count++;
    }
  }
    
  vw_out() << "Number (and fraction) of removed outliers by the triangulation error check: "
           << count << " (" << double(count)/ip1.size() << ").\n";
    
  // Copy the outliers in place
  count = 0;
  for (size_t i = 0; i < ip1.size(); i++) {
    if (tri_errors[i] >= HIGH_ERROR || heights[i] >= HIGH_ERROR) 
      continue;

    ip1[count] = ip1[i];
    ip2[count] = ip2[i];
    count++;
  }

  ip1.resize(count);
  ip2.resize(count);
}
  
size_t filter_ip_homog(std::vector<ip::InterestPoint> const& ip1_in,
                       std::vector<ip::InterestPoint> const& ip2_in,
                       std::vector<ip::InterestPoint>      & ip1_out,
                       std::vector<ip::InterestPoint>      & ip2_out,
                       int inlier_threshold) {

  std::vector<size_t> indices;
  try {
    
    std::vector<Vector3> ransac_ip1 = iplist_to_vectorlist(ip1_in),
      ransac_ip2 = iplist_to_vectorlist(ip2_in);

    vw_out() << "\t    Inlier threshold:                     " << inlier_threshold << "\n";
    vw_out() << "\t    RANSAC iterations:                    "
             << stereo_settings().ip_num_ransac_iterations << "\n";
    typedef math::RandomSampleConsensus<math::HomographyFittingFunctor,
      math::InterestPointErrorMetric> RansacT;
    const int    MIN_NUM_OUTPUT_INLIERS = ransac_ip1.size()/2;
    RansacT ransac(math::HomographyFittingFunctor(),
                   math::InterestPointErrorMetric(),
                   stereo_settings().ip_num_ransac_iterations,
                   inlier_threshold,
                   MIN_NUM_OUTPUT_INLIERS, true);
    Matrix<double> H(ransac(ransac_ip2,ransac_ip1)); // 2 then 1 is used here for legacy reasons
    //vw_out() << "\t--> Homography: " << H << "\n";
    indices = ransac.inlier_indices(H,ransac_ip2,ransac_ip1);
  } catch (const math::RANSACErr& e ) {
    vw_out() << "RANSAC failed: " << e.what() << "\n";
    return false;
  }

  // Assemble the remaining interest points
  const size_t num_left = indices.size();
  std::vector<ip::InterestPoint> final_ip1, final_ip2;
  ip1_out.resize(num_left);
  ip2_out.resize(num_left);
  for (size_t i=0; i<num_left; ++i) {
    size_t index = indices[i];
    ip1_out[i] = ip1_in[index];
    ip2_out[i] = ip2_in[index];
  }

  return num_left;
}


// Filter IP using a given DEM and max height difference.  Assume that
// the interest points have alignment applied to them (either via a
// transform or from mapprojection).
void ip_filter_using_dem(std::string              const & ip_filter_using_dem,
                         vw::TransformPtr                 tx_left,
                         vw::TransformPtr                 tx_right,
                         boost::shared_ptr<vw::camera::CameraModel> left_camera_model, 
                         boost::shared_ptr<vw::camera::CameraModel> right_camera_model,
                         std::vector<vw::ip::InterestPoint> & left_aligned_ip,
                         std::vector<vw::ip::InterestPoint> & right_aligned_ip) {

  vw_out() << "Filtering interest point matches with --ip-filter-using-dem.\n";
  
  std::string dem_file;
  double max_height_diff = -1.0;
  std::istringstream is(ip_filter_using_dem);
  if (!(is >> dem_file >> max_height_diff)) 
    vw_throw(ArgumentErr() << "Could not parse correctly option --ip-filter-using-dem.\n");

  if (max_height_diff <= 0.0) 
    vw_throw(ArgumentErr() << "Positive height diff value expected in --ip-filter-using-dem.\n");

  // Read the DEM and supporting data
  vw::cartography::GeoReference dem_georef;
  DiskImageView<float> dem_disk_image(dem_file);
  ImageViewRef<PixelMask<float>> dem;
  boost::shared_ptr<DiskImageResource> dem_rsrc(DiskImageResourcePtr(dem_file));
  if (dem_rsrc->has_nodata_read())
    dem = create_mask(dem_disk_image, dem_rsrc->nodata_read());
  else
    dem = pixel_cast<PixelMask<float>>(dem_disk_image); // all pixels are valid

  bool has_georef = vw::cartography::read_georeference(dem_georef, dem_file);
  if (!has_georef)
    vw_throw(ArgumentErr() << "There is no georeference information in: "
             << dem_file << ".\n" );
  
  // An invalid pixel value used for edge extension
  PixelMask<float> nodata_pix(0); nodata_pix.invalidate();
  ValueEdgeExtension<PixelMask<float>> nodata_ext(nodata_pix); 
  
  // Set up for interpolation. Out-of-range pixels are declared to be invalid.
  ImageViewRef<PixelMask<float>> interp_dem
    = interpolate(dem, BilinearInterpolation(), nodata_ext);

  // Set up the stereo model for doing triangulation
  double angle_tol = vw::stereo::StereoModel
    ::robust_1_minus_cos(stereo_settings().min_triangulation_angle*M_PI/180);
  stereo::StereoModel model(left_camera_model.get(), right_camera_model.get(),
                            stereo_settings().use_least_squares, angle_tol);

  std::set<int> invalid_indices;
  for (size_t it = 0; it < left_aligned_ip.size(); it++) {

    // Undo the alignment or mapprojection
    Vector2 left_pix  = tx_left->reverse (Vector2(left_aligned_ip[it].x,  left_aligned_ip[it].y));
    Vector2 right_pix = tx_right->reverse(Vector2(right_aligned_ip[it].x, right_aligned_ip[it].y));

    // Triangulate
    double err = -1.0;
    Vector3 xyz(0.0, 0.0, 0.0);
    try {
      xyz = model(left_pix, right_pix, err);
    } catch(...) {
      xyz = Vector3();
    }

    if (err <= 0.0 || xyz == Vector3()) {
      invalid_indices.insert(it);
      continue;
    }

    Vector3 llh = dem_georef.datum().cartesian_to_geodetic(xyz);

    // This was tested to give correct results with the DEM
    // having its longitude in both [-180, 0] and [180, 360].
    Vector2 pix = dem_georef.lonlat_to_pixel(Vector2(llh[0], llh[1]));

    PixelMask<float> dem_val = interp_dem(pix.x(), pix.y());

    if (!is_valid(dem_val) || std::abs(dem_val.child() - llh[2]) > max_height_diff)
      invalid_indices.insert(it);
  }

  int num_total   = left_aligned_ip.size();
  int num_invalid = invalid_indices.size();
  int num_valid   = num_total - num_invalid;
  vw_out() << "Removed " << num_invalid << " interest points out of " << num_total << " ("
           << 100.0 * double(num_invalid) / num_total << " pct).\n";

  // Copy back only the valid pixels in-place
  int good_it = 0;
  for (int it = 0; it < num_total; it++) {
    if (invalid_indices.find(it) != invalid_indices.end())
      continue;
      
    left_aligned_ip[good_it]  = left_aligned_ip[it];
    right_aligned_ip[good_it] = right_aligned_ip[it];
    good_it++;
  }

  if (good_it != num_valid)
    vw_throw(ArgumentErr() << "Book-keeping failure in ip filtering using DEM.\n");
    
  left_aligned_ip.resize(num_valid);
  right_aligned_ip.resize(num_valid);

  return;
}

// Estimate the search range by finding the median disparity and
// creating a box of given dimensions around it. This assumes aligned
// interest points. Note that this box may be an overestimate,
// so it should be intersected with a previously existing box.
vw::BBox2 search_range_using_spread(double max_disp_spread,
                                    std::vector<vw::ip::InterestPoint> const& left_ip,
                                    std::vector<vw::ip::InterestPoint> const& right_ip) {
    std::vector<double> dx, dy;
  
    for (size_t i = 0; i < left_ip.size(); i++) {
    double diffX = right_ip[i].x - left_ip[i].x;
    double diffY = right_ip[i].y - left_ip[i].y;
    dx.push_back(diffX);
    dy.push_back(diffY);
  }
  if (dx.empty())
    vw_throw(vw::ArgumentErr() << "No interest points left.");
  
  std::sort(dx.begin(), dx.end());
  std::sort(dy.begin(), dy.end());
  double mid_x = dx[dx.size()/2]; // median
  double mid_y = dy[dy.size()/2];
  
  double half = max_disp_spread / 2.0;
  vw::BBox2 spread_box(mid_x - half, mid_y - half, max_disp_spread, max_disp_spread);

  return spread_box;
}

// Create interest points from valid D_sub values and make them full scale
// (while still having potentially a global alignment applied to them).
void aligned_ip_from_D_sub(vw::ImageViewRef<vw::PixelMask<vw::Vector2f>> const & sub_disp,
                           vw::Vector2                                   const & upsample_scale,
                           std::vector<vw::ip::InterestPoint>                  & left_ip, 
                           std::vector<vw::ip::InterestPoint>                  & right_ip) {

  left_ip.clear();
  right_ip.clear();
    
  for (int col = 0; col < sub_disp.cols(); col++) {
    for (int row = 0; row < sub_disp.rows(); row++) {
      vw::PixelMask<vw::Vector2f> disp = sub_disp(col, row);

      if (!is_valid(disp)) continue;
        
      Vector2 left_pix(col, row);
      Vector2 right_pix = left_pix + disp.child();

      left_pix  = elem_prod(left_pix, upsample_scale);
      right_pix = elem_prod(right_pix, upsample_scale);

      vw::ip::InterestPoint left, right;
      left.x  = left_pix.x();   left.y = left_pix.y();
      right.x = right_pix.x(); right.y = right_pix.y();

      left_ip.push_back(left);
      right_ip.push_back(right);
    }
  }
}
  
// Do IP matching, return, the best translation+scale fitting functor.
vw::Matrix<double> translation_ip_matching(vw::ImageView<vw::PixelGray<float>> const& image1,
                                           vw::ImageView<vw::PixelGray<float>> const& image2,
                                           int ip_per_tile,
                                           std::string const  left_file_path,
                                           std::string const  right_file_path,
                                           double nodata1, double nodata2) {

  using namespace vw;

  std::vector<ip::InterestPoint> matched_ip1, matched_ip2;
  detect_match_ip(matched_ip1, matched_ip2, image1,  image2, ip_per_tile,
                  left_file_path, right_file_path, nodata1, nodata2);
  
  std::vector<Vector3> ransac_ip1 = iplist_to_vectorlist(matched_ip1);
  std::vector<Vector3> ransac_ip2 = iplist_to_vectorlist(matched_ip2);
  vw_out(DebugMessage,"asp") << "\t--> Removed "
                             << matched_ip1.size() - ransac_ip1.size()
                             << " duplicate matches.\n";

  Matrix<double> T;
  std::vector<size_t> indices;
  try {

    vw::math::RandomSampleConsensus<vw::math::TranslationScaleFittingFunctor, vw::math::InterestPointErrorMetric>
      ransac(vw::math::TranslationScaleFittingFunctor(),
             vw::math::InterestPointErrorMetric(),
             stereo_settings().ip_num_ransac_iterations,
             10, ransac_ip1.size()/2, true);
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

// Homography IP matching
// This applies only the homography constraint. Not the best.
bool homography_ip_matching(
      vw::ImageViewRef<float> const& image1,
      vw::ImageViewRef<float> const& image2,
      int    ip_per_tile,
      int    inlier_threshold,
      std::string const& match_filename,
      std::string const  left_file_path,
      std::string const  right_file_path,
      double nodata1, double nodata2) {

  vw_out() << "\t--> Matching interest points using homography.\n";

  std::vector<ip::InterestPoint> matched_ip1, matched_ip2;
  detect_match_ip(matched_ip1, matched_ip2,
		  image1, image2,
		  ip_per_tile,
		  left_file_path, right_file_path,
		  nodata1, nodata2);
  if (matched_ip1.size() == 0 || matched_ip2.size() == 0)
    return false;
    
  std::vector<ip::InterestPoint> final_ip1, final_ip2;
  size_t num_left = filter_ip_homog(matched_ip1, matched_ip2, final_ip1, final_ip2,
                                    inlier_threshold);
  if (num_left == 0)
    return false;

  if (stereo_settings().ip_debug_images) {
    vw_out() << "\t    Writing post-homography IP match debug image.\n";
    write_match_image("InterestPointMatching__ip_matching_debug2.tif",
                      image1, image2, final_ip1, final_ip2);
  }

  vw_out() << "\t    * Writing match file: " << match_filename << "\n";
  ip::write_binary_match_file(match_filename, final_ip1, final_ip2);

  return true;
}

// Detect interest points. Return also the rough homography that aligns the right image
// to the left.
bool detect_ip_aligned_pair(vw::camera::CameraModel* cam1,
  vw::camera::CameraModel* cam2,
  vw::ImageViewRef<float> const& image1,
  vw::ImageViewRef<float> const& image2,
  int ip_per_tile,
  vw::cartography::Datum const& datum,
  std::string const left_file_path,
  double nodata1, double nodata2,
  // Outputs
  vw::ip::InterestPointList& ip1,
  vw::ip::InterestPointList& ip2,
  vw::Matrix<double> &rough_homography) {

  BBox2i box1 = bounding_box(image1), box2 = bounding_box(image2);

  try {
    // Homography is defined in the original camera coordinates
    rough_homography = rough_homography_fit(cam1, cam2, box1, box2, datum);
  } catch(...) {
    vw_out() << "Rough homography fit failed, trying with identity transform. " << std::endl;
    rough_homography.set_identity(3);
  }
  std::cout << "xxx4 " << rough_homography << std::endl;

  // Remove the main translation and solve for BBox that fits the
  // image. If we used the translation from the solved homography with
  // poorly position cameras, the right image might be moved out of frame.
  rough_homography(0,2) = rough_homography(1,2) = 0;
  vw_out() << "Aligning right to left for IP capture using rough homography: " 
	   << rough_homography << std::endl;

  // Find the image of the right image bounding box after applying the rough homography  
  HomographyTransform rough_trans(rough_homography);
  BBox2i trans_box2 = rough_trans.forward_bbox(box2);

  // Check to see if this rough homography works
  VW_ASSERT(box1.intersects(trans_box2),
     LogicErr() << "The rough homography alignment based on datum and camera "
        << "geometry shows that input images do not overlap at all. Unable to proceed. "
        << "Examine your images, or consider using the option --skip-rough-homography.\n");

  // Adjust the homography transform so that the right image bounding box has
  // (0, 0) as the minimum. We will count on this below and outside of this
  // function. See Transform.h for definition of homography transform.
  for (int i = 0; i < 3; i++) {
    for (int j = 0; j < 2; j++) {
      rough_homography(j, i) -= trans_box2.min()[j] * rough_homography(2, i);
    }
  }

  // Update these after the adjustment to the homography transform
  rough_trans = HomographyTransform(rough_homography);
  trans_box2 = rough_trans.forward_bbox(box2);

  // Detect interest points for the left and (transformed) right image.
  // - It is important that we use NearestPixelInterpolation in the
  //   next step. Using anything else will interpolate nodata values
  //   and stop them from being masked out.
  // TODO(oalexan1): Would it be better to pass masked images and use interpolation?
  auto ext = ValueEdgeExtension<float>(boost::math::isnan(nodata2) ? 0 : nodata2);
  std::string right_file_path = ""; // Don't record IP from transformed images
  if (!detect_ip_pair(ip1, ip2, image1,
                      crop(transform(image2, rough_trans, ext,
                                     NearestPixelInterpolation()), trans_box2),
                      ip_per_tile, left_file_path, right_file_path,
                      nodata1, nodata2)) {
    vw_out() << "Unable to detect interest points." << std::endl;
    return false;
  }

  // Undo the rough homography transform for the interest points
  ip::InterestPointList::iterator ip_it;
  for (ip_it = ip2.begin(); ip_it != ip2.end(); ip_it++) {
    Vector2 pt(ip_it->x, ip_it->y);
    pt = rough_trans.reverse(pt);
    ip_it->ix = ip_it->x = pt.x();
    ip_it->iy = ip_it->y = pt.y();
  }

  return true;
} // End function detect_ip_aligned_pair

// See the .h file for documentation.
bool ip_matching_with_datum(
            bool single_threaded_camera,
            bool use_rough_homography,
            vw::camera::CameraModel* cam1,
            vw::camera::CameraModel* cam2,
            vw::ImageViewRef<float> const& image1,
            vw::ImageViewRef<float> const& image2,
            int ip_per_tile,
            vw::cartography::Datum const& datum,
            std::string const& match_filename,
            double epipolar_threshold,
            double uniqueness_threshold,
            std::string const left_file_path,
            std::string const right_file_path,
            double nodata1,
            double nodata2) {

  if (use_rough_homography)
    vw_out() << "\t    Using rough homography.\n";
  else
    vw_out() << "\t    Skipping rough homography.\n";

  // This call aligns the right image to the left image then detects IPs in the two images.
  // Undo the alignment transform in the ip before returning, and return the transform.
  vw::ip::InterestPointList ip1, ip2;
  vw::Matrix<double> rough_homography = vw::math::identity_matrix<3>();
  if (use_rough_homography) 
    detect_ip_aligned_pair(cam1, cam2, image1, image2,
                           ip_per_tile, datum, left_file_path, nodata1, nodata2,
                           ip1, ip2, rough_homography); // Outputs
  else
     detect_ip_pair(ip1, ip2, image1, image2,
                 ip_per_tile, left_file_path, right_file_path,
                 nodata1, nodata2);

  std::cout << "finished with detect_ip_aligned_pair" << std::endl;

  // Match the detected IPs which are in the original image coordinates.
  std::vector<ip::InterestPoint> matched_ip1, matched_ip2;
  bool match_per_tile = false;
  vw::Matrix<double> align_matrix = vw::math::identity_matrix<3>();
  bool inlier =
    epipolar_ip_matching(single_threaded_camera,
			 ip1, ip2, cam1, cam2, image1, image2, 
       datum, epipolar_threshold, uniqueness_threshold, nodata1, nodata2,
       match_per_tile, align_matrix,
			 matched_ip1, matched_ip2); // Outputs
  if (!inlier)
    return false;

  std::cout << "finished with epipolar_ip_matching" << std::endl;

  // Use the interest points that we found to compute an aligning
  // homography transform for the two images.
  // - This is just a sanity check.
  bool adjust_left_image_size = false;
  Matrix<double> matrix1, matrix2;
  homography_rectification(adjust_left_image_size,
			   image1.get_size(), image2.get_size(),
			   matched_ip1, matched_ip2, matrix1, matrix2);
  std::cout << "matrix 1 = " << matrix1 << std::endl;
  std::cout << "matrix 2 = " << matrix2 << std::endl;
  if (use_rough_homography && 
      sum(abs(submatrix(rough_homography,0,0,2,2) - submatrix(matrix2,0,0,2,2))) > 4) {
    vw_out() << "Homography transform has largely different scale and skew "
             << "compared with the rough homography. Homography transform is " 
	           << matrix2 << ". Examine your images, or consider using the option "
             << "--skip-rough-homography.\n";
    //return false;
  }

  // Second pass if doing matches per tile
  if (asp::stereo_settings().matches_per_tile > 0) {
    std::cout << "---now here---" << std::endl;
    // We will use the homography matrix from left to right ip, stored in matrix2.
    // It is expected that matrix1 is the identity matrix.
    if (matrix1 != vw::math::identity_matrix<3>())
      vw::vw_throw( ArgumentErr() << "Expecting identity matrix for left image alignment.\n");

    match_per_tile = true;
    align_matrix = matrix2; // use the homography matrix

    bool inlier =
    epipolar_ip_matching(single_threaded_camera,
			 ip1, ip2, cam1, cam2, image1, image2, 
       datum, epipolar_threshold, uniqueness_threshold,
			 nodata1, nodata2,
       match_per_tile, align_matrix,
			 matched_ip1, matched_ip2); // Outputs
    if (!inlier)
      return false;
  }

  // Write the matches to disk
  vw_out() << "\t    * Writing match file: " << match_filename << "\n";
  ip::write_binary_match_file(match_filename, matched_ip1, matched_ip2);

  return inlier;
}

// Use one of the two modalities to detect interest points.
void match_ip_task(std::vector<vw::ip::InterestPoint> const& ip1_copy,
                   std::vector<vw::ip::InterestPoint> const& ip2_copy,
                   DetectIpMethod detect_method, double th, bool quiet,
                   // Outputs
                   std::vector<vw::ip::InterestPoint>& matched_ip1,
                   std::vector<vw::ip::InterestPoint>& matched_ip2) {

  // TODO: Should probably unify the ip::InterestPointMatcher class
  // with the EpipolarLinePointMatcher class!
  if (detect_method != DETECT_IP_METHOD_ORB) {
    // For all L2Norm distance metrics
    vw::ip::InterestPointMatcher<vw::ip::L2NormMetric,ip::NullConstraint> matcher(th);
    matcher(ip1_copy, ip2_copy, matched_ip1, matched_ip2,
	    TerminalProgressCallback("asp", "\t   Matching: "), quiet);
  } else {
    // For Hamming distance metrics
    vw::ip::InterestPointMatcher<ip::HammingMetric,ip::NullConstraint> matcher(th);
    matcher(ip1_copy, ip2_copy, matched_ip1, matched_ip2,
	    TerminalProgressCallback("asp", "\t   Matching: "), quiet);
  }

  return;
}

// Match the ip and save the match file. No epipolar constraint
// is used in this mode.
void match_ip_pair(vw::ip::InterestPointList const& ip1, 
                   vw::ip::InterestPointList const& ip2,
                   vw::ImageViewRef<float> const& image1,
                   vw::ImageViewRef<float> const& image2,
                   // Outputs
                   std::vector<vw::ip::InterestPoint>& matched_ip1,
                   std::vector<vw::ip::InterestPoint>& matched_ip2,
                   std::string const& match_file) {


  matched_ip1.clear(); 
  matched_ip2.clear();

  std::vector<vw::ip::InterestPoint> ip1_copy, ip2_copy;
  ip1_copy.reserve(ip1.size());
  ip2_copy.reserve(ip2.size());
  std::copy(ip1.begin(), ip1.end(), std::back_inserter(ip1_copy));
  std::copy(ip2.begin(), ip2.end(), std::back_inserter(ip2_copy));

  DetectIpMethod detect_method 
    = static_cast<DetectIpMethod>(stereo_settings().ip_matching_method);

  // Best point must be closer than the next best point
  double th = stereo_settings().ip_uniqueness_thresh;
  vw_out() << "\t--> Uniqueness threshold: " << th << "\n";

  std::cout << "---matches per tile is " << asp::stereo_settings().matches_per_tile << std::endl;
  
  bool quiet = false;
  match_ip_task(ip1_copy, ip2_copy, detect_method, th, quiet,
    matched_ip1, matched_ip2); // outputs
  std::cout << "--Set here matcher task, verbose or not, and number of jobs.\n";
  std::cout << "Then break up into tiles, and call for each tile:\n";
  std::cout << "no multithreading yet, so use 1 job. Just checking that it works.\n";
  std::cout << "Must make unique here!\n";
  std::cout << "This is O(N^2)!\n";
  vw::ip::remove_duplicates(matched_ip1, matched_ip2);

  if (asp::stereo_settings().matches_per_tile != 0) {

    // Use the interest points that we found to compute an aligning
    // homography transform for the two images.
    // - This is just a sanity check.
    bool adjust_left_image_size = false;
    Matrix<double> matrix1, matrix2;
    homography_rectification(adjust_left_image_size,
          image1.get_size(), image2.get_size(),
          matched_ip1, matched_ip2, matrix1, matrix2);
    std::cout << "matrix 1 = " << matrix1 << std::endl;
    std::cout << "matrix 2 = " << matrix2 << std::endl;

    std::cout << "---now here---" << std::endl;
    // We will use the homography matrix from left to right ip, stored in matrix2.
    // It is expected that matrix1 is the identity matrix.
    if (matrix1 != vw::math::identity_matrix<3>())
      vw::vw_throw( ArgumentErr() << "Expecting identity matrix for left image alignment.\n");

    // Wipe the prior matches
    matched_ip1.clear(); 
    matched_ip2.clear();

    vw::Matrix<double> align_matrix = matrix2; // use the homography matrix
    std::cout << "alignment transform is " << align_matrix << std::endl;

    std::map<std::pair<int, int>, std::vector<vw::ip::InterestPoint>> ip1_vec, ip2_vec;
    group_ip_in_tiles(ip1_copy, ip2_copy, align_matrix, 
      ip1_vec, ip2_vec); // outputs

    std::cout << "--now here\n";
    std::cout << "ip1 vec size is " << ip1_vec.size() << std::endl;
    std::cout << "ip2 vec size is " << ip2_vec.size() << std::endl;

    bool quiet = true;  
    std::set<std::pair<float, float>> set1, set2; // to help find unique matches
    // Now iterate over each set of ip in ip1_vec
    for (auto it1 = ip1_vec.begin(); it1 != ip1_vec.end(); it1++) {

      // Find the corresponding set of ip in ip1_vec and ip2_vec
      auto key = it1->first;
      auto it2 = ip2_vec.find(key);
      if (it2 == ip2_vec.end()) 
        continue; // no matches in this tile

      std::vector<vw::ip::InterestPoint> & tile_ip1 = it1->second;
      std::vector<vw::ip::InterestPoint> & tile_ip2 = it2->second;
      std::vector<vw::ip::InterestPoint> local_matched_ip1, local_matched_ip2;
      try {
        match_ip_task(tile_ip1, tile_ip2, detect_method, th, quiet,
                      local_matched_ip1, local_matched_ip2); // outputs
      } catch (const vw::Exception& e) {
        // This can fail
        std::cout << "Caught exception! " << e.what() << std::endl;
        continue;
      }

      std::cout << "number of matches in box is " << local_matched_ip1.size() << std::endl;
      std::cout << "number of matches in box is " << local_matched_ip2.size() << std::endl;

      if (local_matched_ip1.size() > asp::stereo_settings().matches_per_tile)
        pick_subset(asp::stereo_settings().matches_per_tile, 
          local_matched_ip1, local_matched_ip2);
       
      append_new_matches(local_matched_ip1, local_matched_ip2,
         set1, set2, matched_ip1, matched_ip2); // append here
    } // end iterating over sets of ip
  }

  vw_out() << "\n\t    Matched points: " << matched_ip1.size() << std::endl;

  if (stereo_settings().ip_debug_images) {
    vw_out() << "\t    Writing IP initial match debug image.\n";
    // Cast to float to make this compile
    ImageViewRef<float> im1 = vw::pixel_cast<float>(image1);
    ImageViewRef<float> im2 = vw::pixel_cast<float>(image2);
    write_match_image("InterestPointMatching__ip_matching_debug.tif",
                      im1, im2, matched_ip1, matched_ip2);
  }

  // Save ip
  if (match_file != "") {
    // Create the output directory
    vw::create_out_dir(match_file);
    vw_out() << "Writing: " << match_file << std::endl;
    vw::ip::write_binary_match_file(match_file, matched_ip1, matched_ip2);
  }

  return;
}

// A debug routine to save images with matches on top of them.
void write_match_image(std::string const& out_file_name,
                       vw::ImageViewRef<float> const& image1,
                       vw::ImageViewRef<float> const& image2,
                       std::vector<vw::ip::InterestPoint> const& matched_ip1,
                       std::vector<vw::ip::InterestPoint> const& matched_ip2) {

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

  vw::mosaic::ImageComposite<vw::PixelRGB<vw::uint8>> composite;
  //composite.insert(vw::pixel_cast_rescale<vw::PixelRGB<vw::uint8> >(resample(normalize(image1), sub_scale)), 0, 0);
  //composite.insert(vw::pixel_cast_rescale<vw::PixelRGB<vw::uint8> >(resample(normalize(image2), sub_scale)), vw::int32(image1.impl().cols() * sub_scale), 0);
  composite.insert(vw::pixel_cast_rescale<vw::PixelRGB<vw::uint8> >(image1), 0, 0);
  composite.insert(vw::pixel_cast_rescale<vw::PixelRGB<vw::uint8> >(image2), vw::int32(image1.impl().cols() * sub_scale), 0);
  composite.set_draft_mode(true);
  composite.prepare();

  // Rasterize the composite so that we can draw on it.
  vw::ImageView<vw::PixelRGB<vw::uint8>> comp = composite;

  // Draw a red line between matching interest points
  for (size_t k = 0; k < matched_ip1.size(); k++) {
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

  vw::vw_out() << "Writing: " << out_file_name << std::endl;
  boost::scoped_ptr<vw::DiskImageResource> rsrc(vw::DiskImageResource::create(out_file_name, comp.format()));
  vw::block_write_image(*rsrc, comp, vw::TerminalProgressCallback("", ""));
}

} // end namespace asp
