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
#include <asp/Core/StereoSettings.h>

#include <vw/Math/GaussianClustering.h>
#include <vw/Math/RANSAC.h>
#include <vw/Cartography/CameraBBox.h>
#include <vw/Stereo/StereoModel.h>
#include <vw/Mosaic/ImageComposite.h>
#include <vw/Image/AlgorithmFunctions.h>
#include <vw/InterestPoint/MatcherIO.h>
#include <vw/InterestPoint/Matcher.h>
#include <vw/Math/RandomSet.h> // TODO(oalexan1): Rm this when moving the random subset code
#include <vw/Core/Stopwatch.h>
#include <vw/FileIO/FileUtils.h>

// Move some code here from InterestPointMatching.cc, as that one is slow to compile.
// The logic here is mostly for matching, and not for ip detection or filtering.

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

  /// This only returns the indices
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
    vw_throw(ArgumentErr() << "EpipolarLinePointMatcher: epipolar threshold is < 1.\n");
  if (uniqueness_threshold < 0.1)
    vw_throw(ArgumentErr() << "EpipolarLinePointMatcher: uniqueness threshold is < 0.1.\n");
  if (uniqueness_threshold > 0.99)
    vw_throw(ArgumentErr() << "EpipolarLinePointMatcher: uniqueness threshold is > 0.99.\n");
}
  
Vector3 EpipolarLinePointMatcher::epipolar_line(Vector2 const& feature,
                                                cartography::Datum const& datum,
                                                camera::CameraModel* cam_ip,
                                                camera::CameraModel* cam_obj,
                                                bool & success) {
  success = true;

  // Watch out for errors thrown when projecting into the camera
  try {

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
  math::FLANNTree<float>        & m_tree_float;
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
      if (m_single_threaded_camera) {
        // ISIS camera is single-threaded
        Mutex::Lock lock(m_camera_mutex);
        line_eq = m_matcher.epipolar_line( ip_org_coord, m_matcher.m_datum, 
          m_cam1, m_cam2, found_epipolar);
      }else{
        line_eq = m_matcher.epipolar_line( ip_org_coord, m_matcher.m_datum, 
          m_cam1, m_cam2, found_epipolar);
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
  math::FLANNTree<float>         kd_float(asp::stereo_settings().flann_method);
  math::FLANNTree<unsigned char> kd_uchar(asp::stereo_settings().flann_method);

  Matrix<float>         ip2_matrix_float;
  Matrix<unsigned char> ip2_matrix_uchar;

  // Pack the IP descriptors into a matrix and feed it to the chosen FLANNTree object
  const bool use_uchar_FLANN = (ip_detect_method == DETECT_IP_METHOD_ORB);
  if (use_uchar_FLANN) {
    ip_list_to_matrix(ip2, ip2_matrix_uchar);
    kd_uchar.load_match_data(ip2_matrix_uchar, vw::math::FLANN_DistType_Hamming);
  }else {
    ip_list_to_matrix(ip2, ip2_matrix_float);
    kd_float.load_match_data(ip2_matrix_float,  vw::math::FLANN_DistType_L2);
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
// when this should be called with a single thread, as multiple such calls
// may be running in parallel.
void epipolar_ip_matching_task(bool single_threaded_camera,
        DetectIpMethod detect_method, 
        int number_of_jobs,
        double epipolar_threshold,
        double uniqueness_threshold,
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

  HomographyTransform align_trans(align_matrix);

  // We will have tiles overlap, to ensure no interest points
  // fall between the cracks. That may happen if alignment is not perfect.
  Vector2i params = asp::stereo_settings().matches_per_tile_params;
  vw_out() << "Tile size used in matching: " << params[0] << "\n";
  vw_out() << "Expanded tile size:         " << params[1] << "\n";
  
  int tile_size = params[0];
  int extra = (params[1] - params[0])/2;

  // Map each ip to a box. 
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
          std::set<std::pair<float, float>> & seen1,
          std::set<std::pair<float, float>> & seen2,
          std::vector<vw::ip::InterestPoint> & matched_ip1,
          std::vector<vw::ip::InterestPoint> & matched_ip2) {

  for (int i = 0; i < (int)local_matched_ip1.size(); i++) {
    auto & ip1 = local_matched_ip1[i];
    auto & ip2 = local_matched_ip2[i];
    auto p1 = std::make_pair(ip1.x, ip1.y);
    auto p2 = std::make_pair(ip2.x, ip2.y);
    if (seen1.find(p1) == seen1.end() && seen2.find(p2) == seen2.end()) {
      matched_ip1.push_back(ip1);
      matched_ip2.push_back(ip2);
      seen1.insert(p1);
      seen2.insert(p2);
    }
  }
  return;
}

// TODO(oalexan1): Move this function out of here.
// TODO(oalexan1): Call this function in ControlNetworkLoader.cc
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

// Match ip without a datum, cameras, epipolar lines.
void match_ip_no_datum(std::vector<vw::ip::InterestPoint> const& ip1_copy,
                       std::vector<vw::ip::InterestPoint> const& ip2_copy,
                       DetectIpMethod detect_method, double uniqueness_threshold, 
                       bool quiet,
                       // Outputs
                       std::vector<vw::ip::InterestPoint>& matched_ip1,
                       std::vector<vw::ip::InterestPoint>& matched_ip2) {

  // TODO: Should probably unify the ip::InterestPointMatcher class
  // with the EpipolarLinePointMatcher class!
  if (detect_method != DETECT_IP_METHOD_ORB) {
    // For all L2Norm distance metrics
    vw::ip::InterestPointMatcher<vw::ip::L2NormMetric,ip::NullConstraint> 
      matcher(asp::stereo_settings().flann_method, uniqueness_threshold);
    matcher(ip1_copy, ip2_copy, matched_ip1, matched_ip2,
	    TerminalProgressCallback("asp", "\t   Matching: "), quiet);
  } else {
    // For Hamming distance metrics
    vw::ip::InterestPointMatcher<ip::HammingMetric,ip::NullConstraint> 
      matcher(asp::stereo_settings().flann_method, uniqueness_threshold);
    matcher(ip1_copy, ip2_copy, matched_ip1, matched_ip2,
	    TerminalProgressCallback("asp", "\t   Matching: "), quiet);
  }

  return;
}

// Local class definition
class MatchPerTileTask: public Task, private boost::noncopyable {

  int m_start_tile, m_num_tiles_per_job;
  bool m_have_datum, m_single_threaded_camera;
  DetectIpMethod m_detect_method;
  std::map<std::pair<int, int>, std::vector<vw::ip::InterestPoint>> const& m_ip1_vec;
  std::map<std::pair<int, int>, std::vector<vw::ip::InterestPoint>> const& m_ip2_vec;
  vw::camera::CameraModel* m_cam1;
  vw::camera::CameraModel* m_cam2;
  vw::cartography::Datum const& m_datum;
  double m_epipolar_threshold;
  double m_uniqueness_threshold;
  vw::Matrix<double> const& m_align_matrix;
  int m_matches_per_tile;
  // Outputs
  std::set<std::pair<float, float>> & m_seen1;
  std::set<std::pair<float, float>> & m_seen2;
  std::vector<vw::ip::InterestPoint> & m_matched_ip1;
  std::vector<vw::ip::InterestPoint> & m_matched_ip2;
  Mutex& m_match_mutex;

public:
  MatchPerTileTask(int start_tile, int num_tiles_per_job,
                   bool have_datum, 
                   bool single_threaded_camera,
                   DetectIpMethod detect_method,
                   std::map<std::pair<int, int>, std::vector<vw::ip::InterestPoint>> 
                   const& ip1_vec,
                   std::map<std::pair<int, int>, std::vector<vw::ip::InterestPoint>> 
                   const& ip2_vec,
                   vw::camera::CameraModel* cam1,
                   vw::camera::CameraModel* cam2,
                   vw::cartography::Datum const& datum,
                   double epipolar_threshold,
                   double uniqueness_threshold,
                   vw::Matrix<double> const& align_matrix,
                   int matches_per_tile,
                   // Outputs
                   std::set<std::pair<float, float>> & seen1, 
                   std::set<std::pair<float, float>> & seen2, 
                   std::vector<vw::ip::InterestPoint> & matched_ip1,
                   std::vector<vw::ip::InterestPoint> & matched_ip2,
                   Mutex& match_mutex):
    m_start_tile(start_tile), 
    m_num_tiles_per_job(num_tiles_per_job),
    m_have_datum(have_datum),
    m_single_threaded_camera(single_threaded_camera),
    m_detect_method(detect_method),
    m_ip1_vec(ip1_vec), m_ip2_vec(ip2_vec),
    m_cam1(cam1), m_cam2(cam2), m_datum(datum),
    m_epipolar_threshold(epipolar_threshold),
    m_uniqueness_threshold(uniqueness_threshold),
    m_align_matrix(align_matrix),
    m_matches_per_tile(matches_per_tile),
    m_seen1(seen1), m_seen2(seen2),
    m_matched_ip1(matched_ip1), m_matched_ip2(matched_ip2),
    m_match_mutex(match_mutex) {}

  void operator()() {
  
    bool quiet = true;

    // Do the tiles within the range [m_start_tile, m_start_tile + m_num_tiles_per_job)
    int tile_id = -1;
    for (auto it1 = m_ip1_vec.begin(); it1 != m_ip1_vec.end(); it1++) {

      // Do only the tiles within desired range
      tile_id++;
      if (tile_id < m_start_tile) continue;
      if (tile_id >= m_start_tile + m_num_tiles_per_job) break;

      // Find the corresponding set of ip in ip1_vec and ip2_vec
      auto key = it1->first;
      auto it2 = m_ip2_vec.find(key);
      if (it2 == m_ip2_vec.end()) 
        continue; // no matches in this tile

      std::vector<vw::ip::InterestPoint> const & tile_ip1 = it1->second; // alias
      std::vector<vw::ip::InterestPoint> const & tile_ip2 = it2->second; // alias

      std::vector<vw::ip::InterestPoint> local_matched_ip1, local_matched_ip2;
      try {
        if (m_have_datum) {
          // Copy from std::vector<vw::ip::InterestPoint> to vw::ip::InterestPointList
          // TODO(oalexan1): Avoid this copy
          vw::ip::InterestPointList ip1_list, ip2_list;
          for (size_t i = 0; i < tile_ip1.size(); i++)
              ip1_list.push_back(tile_ip1[i]);
          for (size_t i = 0; i < tile_ip2.size(); i++)
              ip2_list.push_back(tile_ip2[i]);
          int local_number_of_jobs = 1;
          if (m_single_threaded_camera) {
            // ISIS camera is single-threaded, prevent a crash
            Mutex::Lock lock(m_match_mutex);
            epipolar_ip_matching_task(m_single_threaded_camera, m_detect_method, 
              local_number_of_jobs, m_epipolar_threshold, m_uniqueness_threshold, 
              m_datum, quiet, m_cam1, m_cam2, ip1_list, ip2_list,
              local_matched_ip1, local_matched_ip2); // outputs
          } else {
            // Multi-threaded camera
            epipolar_ip_matching_task(m_single_threaded_camera, m_detect_method, 
              local_number_of_jobs, m_epipolar_threshold, m_uniqueness_threshold, 
              m_datum, quiet, m_cam1, m_cam2, ip1_list, ip2_list,
              local_matched_ip1, local_matched_ip2); // outputs
          }
        
        } else {
          match_ip_no_datum(tile_ip1, tile_ip2, m_detect_method, m_uniqueness_threshold, 
            quiet, local_matched_ip1, local_matched_ip2); // outputs
        }
      } catch(...) {
        // This need not succeed
        return;
      }

      // Remove some ip if too many
      if (local_matched_ip1.size() > m_matches_per_tile)
      pick_subset(m_matches_per_tile, local_matched_ip1, local_matched_ip2);

      {
        // Update the global output variables
        Mutex::Lock lock(m_match_mutex);
        append_new_matches(local_matched_ip1, local_matched_ip2,
          m_seen1, m_seen2, m_matched_ip1, m_matched_ip2); // append here
      }
  }

    return;
  } // End function operator()

}; // End class MatchPerTileTask

// Do interest point matching per tile, with and without having a datum. The
// alignment matrix is used to bring the interest points in image2 to image1
// coordinates. Use stereo_settings().matches_per_tile_params.
void matches_per_tile(bool have_datum, 
                      bool single_threaded_camera,
                      DetectIpMethod detect_method,
                      std::vector<vw::ip::InterestPoint> const& ip1_copy, 
                      std::vector<vw::ip::InterestPoint> const& ip2_copy,
                      vw::camera::CameraModel* cam1,
                      vw::camera::CameraModel* cam2,
                      vw::cartography::Datum const& datum,
                      size_t number_of_jobs,
                      double epipolar_threshold,
                      double uniqueness_threshold,
                      vw::Matrix<double> const& align_matrix,
                      // Outputs
                      std::vector<vw::ip::InterestPoint>& matched_ip1,
                      std::vector<vw::ip::InterestPoint>& matched_ip2) {
  
  // Wipe the prior matches
  matched_ip1.clear(); 
  matched_ip2.clear();

  // TODO(oalexan1): Add multi-threading here.
  bool quiet = true;
  std::map<std::pair<int, int>, std::vector<vw::ip::InterestPoint>> ip1_vec, ip2_vec;
  group_ip_in_tiles(ip1_copy, ip2_copy, align_matrix, 
      ip1_vec, ip2_vec); // outputs

  // Adjust the number of jobs
  int num_tiles = ip1_vec.size();
  if (num_tiles < number_of_jobs) 
    number_of_jobs = num_tiles;

  // The number of tiles per job
  int num_tiles_per_job = ceil(double(num_tiles) / number_of_jobs);
  if (num_tiles_per_job == 0) 
    num_tiles_per_job = 1;
  int start_tile = 0;

   // To help find unique matches
  std::set<std::pair<float, float>> seen1, seen2;

  // Iterate over each set of ip in ip1_vec
  FifoWorkQueue matching_queue; // Create a thread pool object
  Mutex match_mutex;
  for (size_t i = 0; i < number_of_jobs; i++) {
    boost::shared_ptr<Task> match_task(new 
    MatchPerTileTask(start_tile, num_tiles_per_job,
                     have_datum, single_threaded_camera, 
                     detect_method, ip1_vec, ip2_vec, cam1, cam2, datum, 
                     epipolar_threshold, uniqueness_threshold, align_matrix,
                     asp::stereo_settings().matches_per_tile,
                     seen1, seen2, matched_ip1, matched_ip2, match_mutex));
    matching_queue.add_task(match_task);
    start_tile += num_tiles_per_job; // for the next job

  } // end iterating over sets of ip

  matching_queue.join_all(); // Wait for all jobs to finish

  return;
} // end if matches per tile

// TODO(oalexan1): Break up this function as it is too long.
bool epipolar_ip_matching(bool single_threaded_camera,
        vw::ip::InterestPointList const& ip1,
        vw::ip::InterestPointList const& ip2,
        vw::camera::CameraModel* cam1,
        vw::camera::CameraModel* cam2,
        vw::ImageViewRef<float> const& image1,
        vw::ImageViewRef<float> const& image2,
        vw::cartography::Datum const& datum,
        size_t number_of_jobs,
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

  // TODO(oalexan1): Avoid this copy
  std::vector<vw::ip::InterestPoint> ip1_copy, ip2_copy;
  ip1_copy.reserve(ip1.size());
  ip2_copy.reserve(ip2.size());
  std::copy(ip1.begin(), ip1.end(), std::back_inserter(ip1_copy));
  std::copy(ip2.begin(), ip2.end(), std::back_inserter(ip2_copy));

  // Match interest points forward/backward .. constraining on epipolar line
  DetectIpMethod detect_method 
    = static_cast<DetectIpMethod>(stereo_settings().ip_detect_method);

  // Sanity check
  if (match_per_tile && asp::stereo_settings().matches_per_tile <= 0)
  vw::vw_throw(vw::ArgumentErr()
    << "epipolar_ip_matching: matches_per_tile must be positive.\n");

  if (!match_per_tile) {
    vw_out() << "\t--> Matching interest points using the epipolar line." << std::endl;
    vw_out() << "\t    Uniqueness threshold: " << uniqueness_threshold << "\n";
    vw_out() << "\t    Epipolar threshold:   " << epipolar_threshold   << "\n";
  } else {
    // This will be printed on the second pass
    vw_out() << "\t--> Performing matching per tile.\n";
  }
  
  // Do matching
  if (!match_per_tile) {
    bool quiet = false; // has to be true if many such jobs exist
    epipolar_ip_matching_task(single_threaded_camera, detect_method, 
          number_of_jobs, epipolar_threshold, uniqueness_threshold, 
          datum, quiet, cam1, cam2, ip1, ip2,
          matched_ip1, matched_ip2); // outputs
  } else {
    bool have_datum = true;
    matches_per_tile(have_datum, single_threaded_camera, detect_method,
                     ip1_copy, ip2_copy, cam1, cam2, datum,
                     number_of_jobs, epipolar_threshold, uniqueness_threshold,
                     align_matrix, 
                     matched_ip1, matched_ip2); // outputs
  }

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

// See the .h file for documentation.
bool match_ip_with_datum(bool single_threaded_camera,
                         bool use_rough_homography,
                         vw::camera::CameraModel* cam1,
                         vw::camera::CameraModel* cam2,
                         vw::ImageViewRef<float> const& image1,
                         vw::ImageViewRef<float> const& image2,
                         int ip_per_tile,
                         vw::cartography::Datum const& datum,
                         std::string const& match_filename,
                         size_t number_of_jobs,
                         double epipolar_threshold,
                         double uniqueness_threshold,
                         std::string const left_file_path,
                         std::string const right_file_path,
                         double nodata1,
                         double nodata2) {

  bool matches_as_txt = stereo_settings().matches_as_txt;

  if (use_rough_homography)
    vw_out() << "\t    Using rough homography.\n";
  else
    vw_out() << "\t    Skipping rough homography.\n";

  // This call aligns the right image to the left image then detects IPs in the two images.
  // Undo the alignment transform in the ip before returning, and return the transform.
  vw::ip::InterestPointList ip1, ip2;
  vw::Matrix<double> rough_homography = vw::math::identity_matrix<3>();
  bool use_cached_ip = false;
  if (use_rough_homography) 
    detect_ip_aligned_pair(cam1, cam2, image1, image2,
                           asp::stereo_settings().ip_per_image, ip_per_tile, 
                           datum, left_file_path, nodata1, nodata2,
                           ip1, ip2, rough_homography); // Outputs
  else
     detect_ip_pair(ip1, ip2, image1, image2,
                    asp::stereo_settings().ip_per_image, ip_per_tile, 
                    left_file_path, right_file_path,
                    nodata1, nodata2, use_cached_ip);

  // Match the detected IPs which are in the original image coordinates.
  std::vector<ip::InterestPoint> matched_ip1, matched_ip2;
  bool match_per_tile = false;
  vw::Matrix<double> align_matrix = vw::math::identity_matrix<3>();
  vw::Stopwatch sw2;
  sw2.start();
  bool inlier = epipolar_ip_matching(single_threaded_camera, ip1, ip2, cam1, cam2, 
                                     image1, image2,  datum, number_of_jobs, 
                                     epipolar_threshold, uniqueness_threshold, 
                                     nodata1, nodata2,
                                     match_per_tile, align_matrix, 
                                     matched_ip1, matched_ip2); // Outputs
  sw2.stop();
  vw_out() << "Elapsed time in ip matching: " << sw2.elapsed_seconds() << " s.\n";

  if (!inlier)
    return false;

  // Use the interest points that we found to compute an aligning homography
  // transform for the two images. This is always needed when finding matches
  // per tile or with rough homography.
  bool adjust_left_image_size = false, tight_inlier_threshold = false;
  Matrix<double> matrix1 = vw::math::identity_matrix<3>();
  Matrix<double> matrix2 = vw::math::identity_matrix<3>();
  if (use_rough_homography || asp::stereo_settings().matches_per_tile > 0) {
    vw_out() << "\t    Computing homography transform.\n";
    vw::Stopwatch sw3;
    sw3.start();
    bool tight_inlier_threshold = (asp::stereo_settings().matches_per_tile > 0);
    std::cout << "----qqqq2\n";
    homography_rectification(adjust_left_image_size, tight_inlier_threshold,
			   image1.get_size(), image2.get_size(),
			   matched_ip1, matched_ip2, matrix1, matrix2);
    sw3.stop();
    vw_out() << "Elapsed time in homography computation: " << sw3.elapsed_seconds() << " s.\n";
  }
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
    // We will use the homography matrix from left to right ip, stored in matrix2.
    // It is expected that matrix1 is the identity matrix.
    if (matrix1 != vw::math::identity_matrix<3>())
      vw::vw_throw( ArgumentErr() << "Expecting identity matrix for left image alignment.\n");

    match_per_tile = true;
    align_matrix = matrix2; // use the homography matrix
    vw::Stopwatch sw4;
    sw4.start();
    bool inlier =
      epipolar_ip_matching(single_threaded_camera,
        ip1, ip2, cam1, cam2, image1, image2, 
        datum, number_of_jobs, epipolar_threshold, uniqueness_threshold,
        nodata1, nodata2,
        match_per_tile, align_matrix,
        matched_ip1, matched_ip2); // Outputs
    sw4.stop();
    vw_out() << "Elapsed time in ip matching when using tiles: " 
      << sw4.elapsed_seconds() << " s.\n";

    if (!inlier)
      return false;
  }

  // Write the matches to disk
  std::cout << "---writing matches ppp7\n";
  vw_out() << "\t    * Writing match file: " << match_filename << "\n";
  ip::write_match_file(match_filename, matched_ip1, matched_ip2, matches_as_txt);
  std::cout << "--done writing matches ppp7\n";
  return inlier;
}

// Match the ip and save the match file. No datum or epipolar constraint
// is used in this mode.
void match_ip_no_datum(vw::ip::InterestPointList const& ip1,
                       vw::ip::InterestPointList const& ip2,
                       vw::ImageViewRef<float> const& image1,
                       vw::ImageViewRef<float> const& image2,
                       size_t number_of_jobs,
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
    = static_cast<DetectIpMethod>(stereo_settings().ip_detect_method);

  // Best point must be closer than the next best point
  double uniqueness_threshold = stereo_settings().ip_uniqueness_thresh;
  vw_out() << "\t--> Uniqueness threshold: " << uniqueness_threshold << "\n";

  bool quiet = false;
  vw::Stopwatch sw1;
  sw1.start();
  match_ip_no_datum(ip1_copy, ip2_copy, detect_method, uniqueness_threshold, quiet,
                    matched_ip1, matched_ip2); // outputs
  sw1.stop();
  vw_out() << "Elapsed time in ip matching: " << sw1.elapsed_seconds() << " s.\n";

  // Remove ip duplicates. Complexity is O(n log n).
  vw::ip::remove_duplicates(matched_ip1, matched_ip2);

  if (asp::stereo_settings().matches_per_tile != 0) {

    // Use the interest points that we found to compute an aligning
    // homography transform for the two images.
    vw_out() << "\t    Computing homography transform.\n";
    bool adjust_left_image_size = false;
    bool tight_inlier_threshold = true; // this is necessary for large images
    Matrix<double> matrix1, matrix2;
    vw::Stopwatch sw2;
    sw2.start();
    std::cout << "---qqq3\n";
    homography_rectification(adjust_left_image_size, tight_inlier_threshold,
                             image1.get_size(), image2.get_size(),
                             matched_ip1, matched_ip2, matrix1, matrix2);
    sw2.stop();
    vw_out() << "Elapsed time in homography computation: " << sw2.elapsed_seconds() << " s.\n";

    // We will use the homography matrix from left to right ip, stored in matrix2.
    // It is expected that matrix1 is the identity matrix.
    if (matrix1 != vw::math::identity_matrix<3>())
      vw::vw_throw( ArgumentErr() << "Expecting identity matrix for left image alignment.\n");
    vw::Matrix<double> align_matrix = matrix2; 

    // Initialize some quantities that won't be used but are part of the API.
    bool have_datum = false, single_threaded_camera = false;
    double epipolar_threshold = -1, nodata1 = -1, nodata2 = -1;
    vw::camera::CameraModel *cam1 = NULL, *cam2 = NULL;
    vw::cartography::Datum datum;
    vw::Stopwatch sw3;
    sw3.start();
    vw_out() << "\t    Performing matching per tile.\n";
    matches_per_tile(have_datum, single_threaded_camera, detect_method,
                     ip1_copy, ip2_copy, cam1, cam2, datum,
                     number_of_jobs, epipolar_threshold, uniqueness_threshold,
                     align_matrix, 
                     matched_ip1, matched_ip2); // outputs
    sw3.stop();
    vw_out() << "Elapsed time in ip matching when using tiles: " 
      << sw3.elapsed_seconds() << " s.\n";
  }

  vw_out() << "\t    Matched points: " << matched_ip1.size() << std::endl;

  if (stereo_settings().ip_debug_images) {
    vw_out() << "\t    Writing IP initial match debug image.\n";
    // Cast to float to make this compile
    ImageViewRef<float> im1 = vw::pixel_cast<float>(image1);
    ImageViewRef<float> im2 = vw::pixel_cast<float>(image2);
    write_match_image("InterestPointMatching__ip_matching_debug.tif",
                      im1, im2, matched_ip1, matched_ip2);
  }

  // Save ip
  std::cout << "---will save ipqwqedqqq\n";
  if (match_file != "") {
    // Create the output directory
    vw::create_out_dir(match_file);
    vw_out() << "Writing: " << match_file << std::endl;
    bool matches_as_txt = stereo_settings().matches_as_txt;
    vw::ip::write_match_file(match_file, matched_ip1, matched_ip2, matches_as_txt);
  }

  return;
}

} // end namespace asp
