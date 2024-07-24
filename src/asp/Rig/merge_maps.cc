/* Copyright (c) 2017, United States Government, as represented by the
 * Administrator of the National Aeronautics and Space Administration.
 *
 * All rights reserved.
 *
 * The Astrobee platform is licensed under the Apache License, Version 2.0
 * (the "License"); you may not use this file except in compliance with the
 * License. You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the
 * License for the specific language governing permissions and limitations
 * under the License.
 */

#include <ceres/ceres.h>
#include <gflags/gflags.h>
#include <glog/logging.h>
// Get rid of warning beyond our control
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
#pragma GCC diagnostic ignored "-Wunused-function"
#pragma GCC diagnostic push
#include <OpenMVG/projection.hpp>
#include <OpenMVG/triangulation_nview.hpp>
#include <OpenMVG/numeric.h>
#include <OpenMVG/tracks.hpp>
#pragma GCC diagnostic pop

#include <Rig/essential.h>
#include <Rig/merge_maps.h>
#include <Rig/rig_config.h>
#include <Rig/camera_image.h>
#include <Rig/image_lookup.h>
#include <Rig/tracks.h>
#include <Rig/nvm.h>
#include <Rig/basic_algs.h>
#include <Rig/ransac.h>
#include <Rig/transform_utils.h>
#include <Rig/rig_config.h>
#include <Rig/interest_point.h>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <set>
#include <thread>
#include <vector>
#include <mutex>
#include <fstream>

DECLARE_int32(num_threads); // defined externally

namespace rig {

// TODO(oalexan1): This code is not used but may be useful one day
// As result of matching some images in A to some images in B, we must
// now merge some tracks in A with some tracks in B, as those tracks
// correspond physically to the same point in space. A track in
// C.pid_to_cid_fid_ tells us which track in A.pid_to_cid_fid_ is tied
// with which track in B.pid_to_cid_fid_. If it turns out one track in
// A should be merged with multiple tracks in B or vice-versa, select
// just one candidate from each map, based on who got most votes. Note
// that here it is easier to work with A.cid_fid_to_pid_ rather than
// A.pid_to_cid_fid_.
void FindPidCorrespondences(std::vector<std::map<int, int>> const& A_cid_fid_to_pid,
                            std::vector<std::map<int, int>> const& B_cid_fid_to_pid,
                            std::vector<std::map<int, int>> const& C_pid_to_cid_fid,
                            int num_acid,  // How many images are in A
                            std::map<int, int> * A2B, std::map<int, int> * B2A) {
  A2B->clear();
  B2A->clear();

  std::map<int, std::map<int, int>> VoteMap;
  for (int pid = 0; pid < static_cast<int>(C_pid_to_cid_fid.size()); pid++) {
    // This track has some cid indices from A (those < num_acid)
    // and some from B (those >= num_acid). Ignore all other combinations.
    auto const& cid_fid_c = C_pid_to_cid_fid[pid];  // alias
    for (auto it_a = cid_fid_c.begin(); it_a != cid_fid_c.end(); it_a++) {
      for (auto it_b = it_a; it_b != cid_fid_c.end(); it_b++) {
        int cid_a = it_a->first, fid_a = it_a->second;
        int cid_b = it_b->first, fid_b = it_b->second;
        if (cid_a >= num_acid) continue;
        if (cid_b <  num_acid) continue;

        // Subtract num_acid from cid_b so it becomes a cid in B.
        cid_b -= num_acid;

        auto it_fida = A_cid_fid_to_pid[cid_a].find(fid_a);
        if (it_fida == A_cid_fid_to_pid[cid_a].end()) continue;

        auto it_fidb = B_cid_fid_to_pid[cid_b].find(fid_b);
        if (it_fidb == B_cid_fid_to_pid[cid_b].end()) continue;

        int pid_a = it_fida->second;
        int pid_b = it_fidb->second;

        VoteMap[pid_a][pid_b]++;
      }
    }
  }

  // For each pid in A, keep the pid in B with most votes
  std::map<int, std::map<int, int>> B2A_Version0;  // still not fully one-to-one
  for (auto it_a = VoteMap.begin(); it_a != VoteMap.end(); it_a++) {
    auto & M = it_a->second;  // all pid_b corresp to given pid_a with their votes
    int pid_a = it_a->first;
    int best_pid_b = -1;
    int max_vote = -1;
    for (auto it_b = M.begin(); it_b != M.end(); it_b++) {
      int pid_b = it_b->first;
      int vote = it_b->second;
      if (vote > max_vote) {
        best_pid_b = pid_b;
        max_vote = vote;
      }
    }
    B2A_Version0[best_pid_b][pid_a] = max_vote;
  }

  // And vice-versa
  for (auto it_b = B2A_Version0.begin(); it_b != B2A_Version0.end(); it_b++) {
    int pid_b = it_b->first;
    auto & M = it_b->second;
    int best_pid_a = -1;
    int max_vote = -1;
    for (auto it_a = M.begin(); it_a != M.end(); it_a++) {
      int pid_a = it_a->first;
      int vote = it_a->second;
      if (vote > max_vote) {
        best_pid_a = pid_a;
        max_vote = vote;
      }
    }

    (*A2B)[best_pid_a] = pid_b;  // track from A and track from B
    (*B2A)[pid_b] = best_pid_a;  // track from B and track from A
  }
}

#if 0
  // TODO(oalexan1): This code may need to be wiped as no longer useful.
  
// If two maps share images, can match tracks between the maps
// just based on that, which is fast.
void findTracksForSharedImages(sparse_mapping::SparseMap * A_in,
                               sparse_mapping::SparseMap * B_in,
                               // Outputs
                               std::map<int, int> & A2B,
                               std::map<int, int> & B2A) {
  // Wipe the outputs
  A2B.clear();
  B2A.clear();

  // Create aliases to not use pointers all the time.
  sparse_mapping::SparseMap & A = *A_in;
  sparse_mapping::SparseMap & B = *B_in;

  size_t num_acid = A.cid_to_filename_.size();
  size_t num_bcid = B.cid_to_filename_.size();

  // Map from file name to cid
  std::map<std::string, int> A_file_to_cid, B_file_to_cid;
  for (size_t cid = 0; cid < num_acid; cid++)
    A_file_to_cid[A.cid_to_filename_[cid]] = cid;
  for (size_t cid = 0; cid < num_bcid; cid++)
    B_file_to_cid[B.cid_to_filename_[cid]] = cid;

  // Iterate through A's cid_fid_to_pid_ and find matches in B.
  int num_shared_cid = 0;
  for (size_t cid_a = 0; cid_a < A.cid_fid_to_pid_.size(); cid_a++) {
    std::string filename = A.cid_to_filename_[cid_a];
    auto it = B_file_to_cid.find(filename);
    if (it == B_file_to_cid.end())
      continue;

    num_shared_cid++;

    // The corresponding camera id in the second map
    size_t cid_b = it->second;

    if (A.cid_to_keypoint_map_[cid_a] != B.cid_to_keypoint_map_[cid_b])
      LOG(FATAL) << "The input maps don't have the same features. "
                 << "They need to be rebuilt.";

    auto a_fid_to_pid = A.cid_fid_to_pid_[cid_a];
    auto b_fid_to_pid = B.cid_fid_to_pid_[cid_b];

    // Find tracks corresponding to same cid_fid
    for (auto it_a = a_fid_to_pid.begin(); it_a != a_fid_to_pid.end(); it_a++) {
      int pid_a = it_a->second;
      int fid = it_a->first;  // shared fid
      auto it_b = b_fid_to_pid.find(fid);
      if (it_b == b_fid_to_pid.end()) {
        // This fid is not in second image. This is fine. A feature in a current image
        // may match to features in one image but not in another.
        continue;
      }

      int pid_b = it_b->second;

      A2B[pid_a] = pid_b;
    }
  }

  // Now create B2A
  for (auto it = A2B.begin(); it != A2B.end(); it++) {
    B2A[it->second] = it->first;
  }

  // Just in case, recreate A2B, to avoid issues when the original
  // A2B mapped multiple A pids to same B pid.
  A2B.clear();
  for (auto it = B2A.begin(); it != B2A.end(); it++) {
    A2B[it->second] = it->first;
  }


  LOG(INFO) << "Number of shared images in the two maps: "
            << num_shared_cid << std::endl;
  LOG(INFO) << "Number of shared tracks: " << A2B.size() << std::endl;

  // Sanity check
  if (num_shared_cid <= 0 || A2B.size() <= 5)
    LOG(FATAL) << "Not enough shared images or features among the two maps. "
               << "Run without the --fast option.";
}
#endif

  
// Merge the camera poses from both maps using the cid2cid map of
// indices. By now the poses are in the same coordinate system but
// some show up in both maps.
void MergePoses(std::map<int, int> & cid2cid,
                std::vector<Eigen::Affine3d> & cid_to_cam_t_global) {
    
  // The total number of output cameras (last new cid value + 1)
  int num_out_cams = rig::maxMapVal(cid2cid) + 1;
  
  // Each blob will be original cids that end up being a single cid
  // after identifying repeat images.
  std::vector<std::set<int>> blobs(num_out_cams);
  for (size_t cid = 0; cid < cid_to_cam_t_global.size(); cid++)
    blobs[cid2cid[cid]].insert(cid);

  // To merge cid_to_cam_t_global, find the average rotation and translation
  // from the two maps.
  std::vector<Eigen::Affine3d> cid_to_cam_t_global2(num_out_cams);
  for (size_t c = 0; c < blobs.size(); c++) {
    if (blobs[c].size() == 1) {
      cid_to_cam_t_global2[c] = cid_to_cam_t_global[*blobs[c].begin()];
    } else {
      int num = blobs[c].size();

      // All cams to merge get equal weight
      std::vector<double> W(num, 1.0/num);

      std::vector<Eigen::Quaternion<double>> Q(num);
      cid_to_cam_t_global2[c].translation() << 0.0, 0.0, 0.0;
      int pos = -1;
      for (auto it = blobs[c].begin(); it != blobs[c].end(); it++) {
        pos++;
        int cid = *it;
        Q[pos] = Eigen::Quaternion<double>(cid_to_cam_t_global[cid].linear());

        cid_to_cam_t_global2[c].translation()
          += W[pos]*cid_to_cam_t_global[cid].translation();
      }
      Eigen::Quaternion<double> S = rig::slerp_n(W, Q);
      cid_to_cam_t_global2[c].linear() = S.toRotationMatrix();
    }
  }

  // Return the updated poses
  cid_to_cam_t_global = cid_to_cam_t_global2;
  
  return;
}

// Choose the images to match and load them. It is assumed that in image_files
// we have the images from the first and then he second maps to merge.
void setupLoadMatchingImages(std::vector<std::string> const& image_files,
                             rig::RigSet const& R,
                             std::string const& image_sensor_list, 
                             int map1_len, int map2_len,
                             int num_image_overlaps_at_endpoints,
                             // Outputs
                             std::vector<std::pair<int, int>> & image_pairs,
                             std::vector<rig::cameraImage> & cams) {

  // sanity check
  if (map1_len + map2_len != image_files.size()) 
    LOG(FATAL) << "Book-keeping error, total number of images is not right.\n";
  
  // Initialize the outputs
  image_pairs.clear();
  cams.resize(image_files.size());
  
  std::set<int> map1_search, map2_search;  // use sets to avoid duplicates
  int num = num_image_overlaps_at_endpoints;

  // Images in map1 to search for matches in map2
  for (int cid = 0; cid < num; cid++)
    if (cid < map1_len) map1_search.insert(cid);
  for (int cid = map1_len-num; cid < map1_len; cid++)
    if (cid >= 0) map1_search.insert(cid);

  // Images in map2 to search for matches in map1. Add map1_len since we will
  // match map1 and map2 inside of the merged map.
  for (int cid = 0; cid < num; cid++)
    if (cid < map2_len) map2_search.insert(map1_len + cid);
  for (int cid = map2_len-num; cid < map2_len; cid++)
    if (cid >= 0) map2_search.insert(map1_len + cid);

  // The indices in the merged map between which we need matches. Do not match
  // an image with itself. That can happen if the maps to merge
  // have shared images.
  for (auto it1 = map1_search.begin(); it1 != map1_search.end() ; it1++) {
    for (auto it2 = map2_search.begin(); it2 != map2_search.end(); it2++) {
      if (*it1 == *it2)
        LOG(FATAL) << "Book-keeping failure in map merging.";
      if (image_files[*it1] == image_files[*it2])
        continue;
      image_pairs.push_back(std::make_pair(*it1, *it2));
    }
  }

  // Allocate a structure having an entry for all images, but load
  // only those for which we need to find matches.
  if (!map1_search.empty() && !map2_search.empty()) 
    std::cout << "Loading images to match." << std::endl;
  
  // Infer the sensor type (and timestamp, which is not used)
  std::vector<int> cam_types;
  std::vector<double> timestamps;
  bool flexible_strategy = true; // can handle with and without separate attributes
  rig::readImageSensorTimestamp(image_sensor_list, image_files, R.cam_names, 
                                      flexible_strategy,
                                      // Outputs
                                      cam_types, timestamps);
  
  for (size_t cid = 0; cid < image_files.size(); cid++) {
    auto & c = cams[cid]; // alias
    // Populate most fields. All we need is the image data and camera type.
    c.image_name = image_files[cid];
    c.camera_type = cam_types[cid];
    c.timestamp = timestamps[cid];
    if (map1_search.find(cid) != map1_search.end() ||
        map2_search.find(cid) != map2_search.end()) {
      c.image = cv::imread(c.image_name, cv::IMREAD_GRAYSCALE);
    }
  }
}

// Compute the transform from the B map to the A map by finding the median
// transform based on the shared images
Eigen::Affine3d computeTransformFromBToA(const rig::nvmData& A,
                                         const rig::nvmData& B) {
  // Calc all transforms from B poses to A poses
  std::vector<Eigen::MatrixXd> B2A_vec;
  
  // Put the B poses in a map
  std::map<std::string, Eigen::Affine3d> B_world2cam;
  for (size_t cid = 0; cid < B.cid_to_cam_t_global.size(); cid++)
    B_world2cam[B.cid_to_filename[cid]] = B.cid_to_cam_t_global[cid];
  
  // Find the transform from B to A based on shared poses
  for (size_t cid = 0; cid < A.cid_to_filename.size(); cid++) {
    auto b_it = B_world2cam.find(A.cid_to_filename[cid]);
    if (b_it == B_world2cam.end()) 
      continue;
    
    auto const& A_world2cam = A.cid_to_cam_t_global[cid];
    auto const& B_world2cam = b_it->second;
    
    // Go from world of B to world of A
    B2A_vec.push_back( ((A_world2cam.inverse()) * B_world2cam).matrix() );
  }

  // Find the median transform, for robustness
  Eigen::Affine3d B2A_trans;
  B2A_trans.matrix() = rig::median_matrix(B2A_vec);
  
  return B2A_trans;
}

// Given a set of points in 3D, heuristically estimate what it means
// for two points to be "not far" from each other. The logic is to
// find a bounding box of an inner cluster and multiply that by 0.1.
double estimateCloseDistance(std::vector<Eigen::Vector3d> const& vec) {
  Eigen::Vector3d range;
  int num_pts = vec.size();
  if (num_pts <= 0)
    LOG(FATAL) << "Empty set of points.\n";  // to avoid a segfault

  std::vector<double> vals(num_pts);
  for (int it = 0; it < range.size(); it++) {  // iterate in each coordinate
    // Sort all values in given coordinate
    for (int p = 0; p < num_pts; p++)
      vals[p] = vec[p][it];
    std::sort(vals.begin(), vals.end());

    // Find some percentiles
    int min_p = round(num_pts*0.25);
    int max_p = round(num_pts*0.75);
    if (min_p >= num_pts) min_p = num_pts - 1;
    if (max_p >= num_pts) max_p = num_pts - 1;
    double min_val = vals[min_p], max_val = vals[max_p];
    range[it] = 0.1*(max_val - min_val);
  }

  // Find the average of all ranges
  double range_val = 0.0;
  for (int it = 0; it < range.size(); it++)
    range_val += range[it];
  range_val /= range.size();

  return range_val;
}
// This fitting functor attempts to find a rotation + translation + scale transformation
// between two vectors of points.
struct TranslationRotationScaleFittingFunctor {
  typedef Eigen::Affine3d result_type;

  /// A transformation requires 3 inputs and 3 outputs to make a fit.
  size_t min_elements_needed_for_fit() const { return 3; }

  result_type operator() (std::vector<Eigen::Vector3d> const& in_vec,
                          std::vector<Eigen::Vector3d> const& out_vec) const {
    // check consistency
    if (in_vec.size() != out_vec.size())
      LOG(FATAL) << "There must be as many inputs as outputs to be "
                 << "able to compute a transform between them.\n";
    if (in_vec.size() < min_elements_needed_for_fit())
      LOG(FATAL) << "Cannot compute a transformation. Insufficient data.\n";

    Eigen::Matrix3Xd in_mat  = Eigen::MatrixXd(3, in_vec.size());
    Eigen::Matrix3Xd out_mat = Eigen::MatrixXd(3, in_vec.size());
    for (size_t it = 0; it < in_vec.size(); it++) {
      in_mat.col(it)  = in_vec[it];
      out_mat.col(it) = out_vec[it];
    }
    result_type out_trans;
    rig::Find3DAffineTransform(in_mat, out_mat, &out_trans);
    return out_trans;
  }
};

// How well does the given transform do to map p1 to p2.
struct TransformError {
  double operator() (Eigen::Affine3d const& T, Eigen::Vector3d const& p1,
                     Eigen::Vector3d const& p2) const {
    return (T*p1 - p2).norm();
  }
};
  
// Estimate the transform from B_xyz_vec to A_xyz_vec using RANSAC.
// A lot of outliers are possible.
Eigen::Affine3d findMapToMapTransform(const std::vector<Eigen::Vector3d>& A_xyz_vec,
                                      const std::vector<Eigen::Vector3d>& B_xyz_vec,
                                      const double close_dist) {

  double inlier_threshold = estimateCloseDistance(A_xyz_vec);
  if (close_dist > 0.0)
    inlier_threshold = close_dist; // user-set value
  
  std::cout << "3D points are declared to be rather close if their distance is " 
            << inlier_threshold << " meters (option --close_dist). "
            << "Using this as inlier threshold.\n";
  
  int  num_iterations = 1000;
  int  min_num_output_inliers = A_xyz_vec.size()/2;
  bool reduce_min_num_output_inliers_if_no_fit = true;  // If too many outliers
  bool increase_threshold_if_no_fit = false;  // better fail than give bad results
  std::vector<size_t> inlier_indices;
  Eigen::Affine3d B2A_trans;
  try {
    RandomSampleConsensus<TranslationRotationScaleFittingFunctor, TransformError>
      ransac(TranslationRotationScaleFittingFunctor(),
             TransformError(), num_iterations,
             inlier_threshold, min_num_output_inliers,
             reduce_min_num_output_inliers_if_no_fit, increase_threshold_if_no_fit);
    B2A_trans = ransac(B_xyz_vec, A_xyz_vec);
    inlier_indices
      = ransac.inlier_indices(B2A_trans, B_xyz_vec, A_xyz_vec);
  } catch(std::exception const& e) {
    LOG(FATAL) << e.what() << "\n" << "Consider adjusting --close_dist or "
               << "taking a closer look at your maps. They should have "
               << "mostly images with non-small baseline.";
  }

  std::cout << "Number of RANSAC inliers: " << inlier_indices.size() << " ("
            << (100.0 * inlier_indices.size()) / A_xyz_vec.size() << " %)\n";

  return B2A_trans;
}

// Find how to map cids which may have repetition, to new unique cid.
// Also sort the new cid by image name.
void findCidToCid(std::vector<std::string> const& cid_to_filename,
                  std::map<int, int> & cid2cid, int & num_out_cams) {
  
  // Wipe the outputs
  cid2cid.clear();
  num_out_cams = 0;
  
  std::vector<std::string> sorted = cid_to_filename; // make a copy
  std::sort(sorted.begin(), sorted.end());
  
  // The new index of each image after rm repetitions
  std::map<std::string, int> image2cid;  
  for (size_t cid = 0; cid < sorted.size(); cid++) {
    std::string img = sorted[cid];
    if (image2cid.find(img) == image2cid.end()) {
      image2cid[img] = num_out_cams;
      num_out_cams++;
    }
  }

  // The index of the cid after removing the repetitions
  for (size_t cid = 0; cid < cid_to_filename.size(); cid++)
    cid2cid[cid] = image2cid[cid_to_filename[cid]];
  if (num_out_cams != rig::maxMapVal(cid2cid) + 1) // sanity check
    LOG(FATAL) << "Book-keeping error in findCidToCid().\n";
}

// Merge the camera names using cid2cid, which remaps the cid to remove repetitions
// and sort the images by time.
void mergeCameraNames(std::vector<std::string> & cid_to_filename,
                      const std::map<int, int> & cid2cid, 
                      int num_out_cams) {
  std::vector<std::string> cid_to_filename2(num_out_cams);
  for (size_t cid = 0; cid < cid_to_filename.size(); cid++) {
    auto it = cid2cid.find(cid);
    if (it == cid2cid.end())
      LOG(FATAL) << "cid2cid does not contain cid " << cid << ".\n";
    cid_to_filename2.at(it->second) = cid_to_filename[cid];
  }
  
  cid_to_filename = cid_to_filename2;
}

// Merge the camera poses using cid2cid, which remaps the cids to remove repetitions.
void mergeCameraPoses(std::vector<rig::cameraImage> &C_cams,
                      std::map<int, int> const& cid2cid,
                      int num_out_cams) {
  std::vector<rig::cameraImage> merged_cams(num_out_cams);
  for (size_t cid = 0; cid < C_cams.size(); cid++) {
    auto it = cid2cid.find(cid);
    if (it == cid2cid.end())
      LOG(FATAL) << "cid2cid does not contain cid " << cid << ".\n";
    merged_cams[it->second] = C_cams[cid];
  }

  C_cams = merged_cams;
}

// Merge two maps. See sfm_merge.cc. Approach: Find matches among
// several images in map A and several in map B, based on
// num_image_overlaps_at_endpoints. Then build tracks (so merging the
// pairwise matches into tracks). If a track is partially in A and
// partially in B, (with at least two features in each), that makes it
// possible to find a triangulated point in A and one in B for that
// track. Doing RANSAC between them will find the transform from B to
// A. Then merge the transformed poses, remove the repeated images,
// and concatenate and remove repetitions from tracks.  If the
// -fast_merge flag is used, find the transform between the maps using
// shared poses.  It is assumed features in keypoint maps are not
// shifted relative to the optical center. The caller is responsible
// to ensure that.
// TODO(oalexan1): Modularize and move to some new tracks.cc file,
// together with other logic from interest_point.cc.
void MergeMaps(rig::nvmData const& A,
               rig::nvmData const& B,
               rig::RigSet const& R,
               int num_image_overlaps_at_endpoints,
               bool fast_merge,
               bool no_transform,
               double close_dist,
               std::string const& image_sensor_list, 
               rig::nvmData & C) { // output merged map

  // Wipe the output
  C = rig::nvmData();

  if (fast_merge && num_image_overlaps_at_endpoints > 0) {
    std::cout << "Setting number of image overlaps at end points to zero, "
              << "as fast merging is used.\n";
    num_image_overlaps_at_endpoints = 0;
  }
  
  // Merge things that make sense to merge and are easy to do. Later
  // some of these will be shrunk if the input maps have shared data.
  int num_acid = A.cid_to_filename.size();
  int num_bcid = B.cid_to_filename.size();
  int num_ccid = num_acid + num_bcid;

  // Concatenate the images from A and B into C.
  C.cid_to_filename.clear();
  C.cid_to_filename.insert(C.cid_to_filename.end(),
                           A.cid_to_filename.begin(), A.cid_to_filename.end());
  C.cid_to_filename.insert(C.cid_to_filename.end(),
                           B.cid_to_filename.begin(), B.cid_to_filename.end());

  std::vector<rig::cameraImage> C_cams;
  std::vector<std::pair<int, int>> image_pairs;
  setupLoadMatchingImages(C.cid_to_filename, R, image_sensor_list, 
                          num_acid, num_bcid,  
                          num_image_overlaps_at_endpoints,  
                          image_pairs, C_cams); // Outputs
  
  Eigen::Affine3d B2A_trans = Eigen::Affine3d::Identity();
  if (fast_merge || no_transform) {

    // This will be empty, as we add no new features, during merging,
    // but ensure it has the right size.
    C.cid_to_keypoint_map.clear();
    C.cid_to_keypoint_map.resize(C.cid_to_filename.size());

    // Compute the transform from the B map to the A map by finding the median
    // transform based on the shared images
    if (!no_transform) 
      B2A_trans = computeTransformFromBToA(A, B);

  } else {
    // Find corresponding triangulated points between the maps which will
    // result in the transform between them.
  
    // Find features among matching images
    std::string out_dir = "";
    bool save_matches = false;
    int num_overlaps = 0; // will use image_pairs
    rig::KeypointVecT C_keypoint_vec;
    int initial_max_reprojection_error = -1; // won't be used
    bool verbose = false;
    bool filter_matches_using_cams = false; // do not have a single camera set yet
    bool read_nvm_no_shift = true; // not used, part of the api
    bool no_nvm_matches = true; // not used, part of the api
    rig::nvmData empty_nvm; // not used, part of the api
    C.cid_to_cam_t_global.resize(C.cid_to_filename.size()); // won't be used
    std::cout << "Number of image pairs to match: " << image_pairs.size() << std::endl;
    std::vector<Eigen::Vector3d> local_xyz_vec; // not used
    rig::detectMatchFeatures(// Inputs
                                   C_cams, R.cam_params, out_dir, save_matches,  
                                   filter_matches_using_cams,  
                                   C.cid_to_cam_t_global,
                                   num_overlaps, image_pairs,
                                   initial_max_reprojection_error, FLAGS_num_threads,  
                                   read_nvm_no_shift, no_nvm_matches, verbose,  
                                   // Outputs
                                   C_keypoint_vec, C.pid_to_cid_fid, local_xyz_vec,
                                   empty_nvm);

    // Split intro corresponding tracks in the two maps
    std::vector<std::map<int, int>> A_pid_to_cid_fid, B_pid_to_cid_fid;
    rig::KeypointVecT A_keypoint_vec, B_keypoint_vec;
    std::vector<rig::cameraImage> A_cams, B_cams;
    rig::splitTracksOneToOne(// Inputs
                                   num_acid, C.pid_to_cid_fid, C_keypoint_vec, C_cams,  
                                   // Outputs
                                   A_pid_to_cid_fid, B_pid_to_cid_fid,
                                   A_keypoint_vec, B_keypoint_vec,  
                                   A_cams, B_cams);
    
#if 1
    // TODO(oalexan1): This should be a function called findMatchingTriPoints().
    // Flag as outliers features outside of the distorted crop box
    std::vector<std::map<int, std::map<int, int>>> A_pid_cid_fid_inlier,
      B_pid_cid_fid_inlier;
    rig::flagOutlierByExclusionDist(// Inputs
                                          R.cam_params, A_cams, A_pid_to_cid_fid,
                                          A_keypoint_vec,
                                          // Outputs
                                          A_pid_cid_fid_inlier);
    rig::flagOutlierByExclusionDist(// Inputs
                                          R.cam_params, B_cams, B_pid_to_cid_fid,
                                          B_keypoint_vec,
                                          // Outputs
                                          B_pid_cid_fid_inlier);

    // Find triangulated points
    std::vector<Eigen::Vector3d> A_xyz_vec, B_xyz_vec; // triangulated points go here
    rig::multiViewTriangulation(// Inputs
                                      R.cam_params, A_cams, A.cid_to_cam_t_global,
                                      A_pid_to_cid_fid,
                                      A_keypoint_vec,
                                      // Outputs
                                      A_pid_cid_fid_inlier, A_xyz_vec);
    rig::multiViewTriangulation(// Inputs
                                      R.cam_params, B_cams,
                                      B.cid_to_cam_t_global,
                                      B_pid_to_cid_fid,
                                      B_keypoint_vec,
                                      // Outputs
                                      B_pid_cid_fid_inlier, B_xyz_vec);
  
    // Keep only the good points
    int count = 0;
    for (size_t pid = 0; pid < A_xyz_vec.size(); pid++) {
      if (!rig::isGoodTri(A_xyz_vec[pid]) || !rig::isGoodTri(B_xyz_vec[pid]))
        continue;
      A_xyz_vec[count] = A_xyz_vec[pid];
      B_xyz_vec[count] = B_xyz_vec[pid];
      count++;
    }
    A_xyz_vec.resize(count);
    B_xyz_vec.resize(count);
#endif

    // Use the triangulated points and RANSAC to find the B to A transform
    B2A_trans = findMapToMapTransform(A_xyz_vec, B_xyz_vec, close_dist);

    // Convert keypoints to nvm format, updating C.cid_to_keypoint_map.
    C.cid_to_keypoint_map.resize(C.cid_to_filename.size());
    for (size_t cid = 0; cid < C.cid_to_filename.size(); cid++)
      rig::vec2eigen(C_keypoint_vec[cid], C.cid_to_keypoint_map[cid]);
    C_keypoint_vec = rig::KeypointVecT (); // wipe this
    
  } // end finding the transform using matches

  // In either case print the found transform
  std::cout << "Affine transform from second map to first map:\n";
  std::cout << "Rotation + scale:\n" << B2A_trans.linear()  << "\n";
  std::cout << "Translation: " << B2A_trans.translation().transpose() << "\n";
  
  // Bring the B map cameras in the A map coordinate system. Do not modify
  // B.cid_to_cam_t_global, but make a copy of it in B_trans_world2cam.
  std::vector<Eigen::Affine3d> B_trans_world2cam = B.cid_to_cam_t_global;
  rig::TransformCameras(B2A_trans, B_trans_world2cam);

  // Append all to the C map. Note how we use the transformed B.
  C.cid_to_cam_t_global.clear();
  C.cid_to_cam_t_global.insert(C.cid_to_cam_t_global.end(),
                           A.cid_to_cam_t_global.begin(), A.cid_to_cam_t_global.end());
  C.cid_to_cam_t_global.insert(C.cid_to_cam_t_global.end(),
                               B_trans_world2cam.begin(), B_trans_world2cam.end());

  //  Find how to map cid to new cid which will not have
  //  repetition. Also sort by image name.
  std::map<int, int> cid2cid;
  int num_out_cams = 0;
  findCidToCid(C.cid_to_filename, cid2cid, num_out_cams);

  // Update C.cid_to_cam_t_global by merging poses for same images in the two maps
  MergePoses(cid2cid, C.cid_to_cam_t_global);

  // Merge the camera names too, based on cid2cid
  mergeCameraNames(C.cid_to_filename, cid2cid, num_out_cams);
  
#if 1
  // By now we have 3 maps: A, B, and the new one in C having shared
  // tracks. Each of these has its own images and indices, and C
  // has repeated indices too, and need to merge them all into
  // a single set of tracks. We will not merge tracks, just remove
  // duplicates.
  // Factor this block out and call it mergeTracks().
  // Note that keypoint_offsets are applied before the cid2cid transform gets used!
  // There must be enough for all the input cameras.
  // This is very error-prone!
  std::vector<Eigen::Vector2d> keypoint_offsets(num_acid + num_bcid,
                                                Eigen::Vector2d(0, 0));
  std::vector<std::map<std::pair<float, float>, int>> merged_keypoint_map(num_out_cams);
  std::vector<int> find_count(num_out_cams, 0); // how many keypoints so far
  std::vector<std::map<int, int>> merged_pid_to_cid_fid;
  // Add A
  int cid_shift = 0; // A and C start with same images, so no shift
  rig::transformAppendNvm(A.pid_to_cid_fid, A.cid_to_keypoint_map,  
                                cid2cid, keypoint_offsets, cid_shift, num_out_cams,
                                // Append below
                                find_count, merged_keypoint_map,
                                merged_pid_to_cid_fid);
  // Add B
  cid_shift = num_acid; // the B map starts later
  rig::transformAppendNvm(B.pid_to_cid_fid, B.cid_to_keypoint_map,  
                                cid2cid, keypoint_offsets, cid_shift, num_out_cams,  
                                // Append below
                                find_count, merged_keypoint_map,
                                merged_pid_to_cid_fid);
  // Add C
  cid_shift = 0; // no shift, C is consistent with itself
  rig::transformAppendNvm(C.pid_to_cid_fid, C.cid_to_keypoint_map,  
                                cid2cid, keypoint_offsets, cid_shift, num_out_cams,  
                                // Append below
                                find_count, merged_keypoint_map,
                                merged_pid_to_cid_fid);

  // Overwrite C.pid_to_cid_fid after the merge
  C.pid_to_cid_fid = merged_pid_to_cid_fid;
  merged_pid_to_cid_fid = std::vector<std::map<int, int>>();

  // Remove duplicate tracks
  rig::rmDuplicateTracks(C.pid_to_cid_fid);

  // Update C.cid_to_keypoint_map. This has the same data as
  // merged_keypoint_map but need to reverse key and value and use
  // an Eigen::Matrix.
  C.cid_to_keypoint_map.clear();
  C.cid_to_keypoint_map.resize(num_out_cams);
  for (int cid = 0; cid < num_out_cams; cid++) {
    auto const& map = merged_keypoint_map[cid]; // alias
    C.cid_to_keypoint_map[cid] = Eigen::MatrixXd(2, map.size());
    for (auto map_it = map.begin(); map_it != map.end(); map_it++) {
      std::pair<float, float> const& K = map_it->first; 
      int fid = map_it->second;
      C.cid_to_keypoint_map.at(cid).col(fid) = Eigen::Vector2d(K.first, K.second);
    }
  }
#endif
  
  // Merge the camera poses as well (remove duplicates)
  mergeCameraPoses(C_cams, cid2cid, num_out_cams);
  
  // Create C_keypoint_vec. Same info as C.cid_to_keypoint_map but different structure.
  rig::KeypointVecT C_keypoint_vec;
  C_keypoint_vec.resize(num_out_cams);
  for (int cid = 0; cid < num_out_cams; cid++)
    rig::eigen2vec(C.cid_to_keypoint_map[cid], C_keypoint_vec[cid]);

  // Flag outliers
  std::vector<std::map<int, std::map<int, int>>> C_pid_cid_fid_inlier;
  rig::flagOutlierByExclusionDist(// Inputs
                                        R.cam_params, C_cams, C.pid_to_cid_fid,
                                        C_keypoint_vec,
                                        // Outputs
                                        C_pid_cid_fid_inlier);

  // Triangulate the merged tracks with merged cameras
  rig::multiViewTriangulation(// Inputs
                                    R.cam_params, C_cams, C.cid_to_cam_t_global,
                                    C.pid_to_cid_fid, C_keypoint_vec,
                                    // Outputs
                                    C_pid_cid_fid_inlier, C.pid_to_xyz);

  // TODO(oalexan1): Should one remove outliers from tri points
  // and C.pid_to_cid_fid?
}
  
}

