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

// TODO(oalexan1): Move track logic to track.cc. This file must be
// further broken up into several files, for example, ba.cc,
// interest_point.cc, triangulation.cc, merge_maps.cc, etc.

#include <Rig/tensor.h>
#include <Rig/ransac.h>
#include <Rig/reprojection.h>
#include <Rig/sparse_mapping.h>
#include <Rig/sparse_map.h>
#include <Rig/essential.h>
#include <Rig/matching.h>
#include <Rig/basic_algs.h>
#include <Rig/thread.h>
#include <Rig/nvm.h>
#include <Rig/rig_config.h>
#include <Rig/camera_image.h>
#include <Rig/image_lookup.h>
#include <Rig/tracks.h>

#include <Eigen/Core>
#include <Eigen/Geometry>
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

#include <opencv2/features2d/features2d.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <sys/stat.h>
#include <fstream>
#include <set>
#include <thread>
#include <vector>
#include <mutex>
#include <functional>
#include <cstdio>

DEFINE_int32(min_valid, 20,
             "Minimum number of valid inlier matches required to keep matches for "
             "a given image pair.");

DEFINE_int32(max_pairwise_matches, 2000,
             "Maximum number of pairwise matches in an image pair to keep.");

DEFINE_int32(num_subsequent_images, std::numeric_limits<int32_t>::max()/2,  // avoid overflow
             "When no vocabulary tree is provided, match every image against this "
             "many subsequent images.");
DEFINE_int32(match_all_rate, -1,  // avoid overflow
             "If nonnegative, match one of every match_all_rate images to every other image.");
DEFINE_bool(skip_filtering, false,
            "Skip filtering of outliers after bundle adjustment.");
DEFINE_bool(skip_adding_new_matches_on_merging, false,
            "When merging maps, do not take advantage of performed matching to add new tracks.");
DEFINE_double(reproj_thresh, 5.0,
              "Filter points with re-projection error higher than this.");

// bundle adjustment phase parameters
DECLARE_int32(num_threads); // defined externally

DEFINE_int32(max_num_iterations, 1000,
             "Maximum number of iterations for bundle adjustment solver.");
DEFINE_int32(num_ba_passes, 5,
             "How many times to run bundle adjustment, removing outliers each time.");
DEFINE_string(cost_function, "Cauchy",
              "Choose a bundle adjustment cost function from: Cauchy, PseudoHuber, Huber, L1, L2.");
DEFINE_double(cost_function_threshold, 2.0,
              "Threshold to use with some cost functions, e.g., Cauchy.");
DEFINE_int32(first_ba_index, 0,
             "Vary only cameras starting with this index during bundle adjustment.");
DEFINE_int32(last_ba_index, std::numeric_limits<int>::max(),
             "Vary only cameras ending with this index during bundle adjustment.");
DEFINE_bool(silent_matching, false,
            "Do not print a lot of verbose info when matching.");

namespace sparse_mapping {
// Two minor and local utility functions
std::string print_vec(double a) {
  char st[256];
  snprintf(st, sizeof(st), "%7.4f", a);
  return std::string(st);
}
std::string print_vec(Eigen::Vector3d a) {
  char st[256];
  snprintf(st, sizeof(st), "%7.4f %7.4f %7.4f", a[0], a[1], a[2]);
  return std::string(st);
}

// Read matches from disk into OpenMVG's format
int ReadMatches(std::string const& matches_file,
                aspOpenMVG::matching::PairWiseMatches * match_map) {
  (*match_map).clear();
  std::ifstream imfile(matches_file.c_str());
  if (!imfile.good())
    LOG(FATAL) << "Could not read: " << matches_file << ". The matching step needs to be redone.";

  LOG(INFO) << "Reading: " << matches_file;
  std::string line;
  int num_matches = 0;
  while (std::getline(imfile, line)) {
    // replace '_' with ' '
    char * ptr = const_cast<char*>(line.c_str());
    for (size_t it = 0; it < line.size(); it++)
      if (ptr[it] == '_') ptr[it] = ' ';
    int i0, mi0, j0, mj0;
    if (sscanf(line.c_str(), "%d %d %d %d\n", &i0, &mi0, &j0, &mj0) != 4) continue;
    size_t i = i0, mi = mi0, j = j0, mj = mj0;
    std::pair<size_t, size_t> P(i, j);
    if ((*match_map).find(P) == (*match_map).end())
      (*match_map)[P] = std::vector<aspOpenMVG::matching::IndMatch>();
    aspOpenMVG::matching::IndMatch M(mi, mj);
    (*match_map)[P].push_back(M);
    num_matches++;
  }
  return num_matches;
}

void WriteMatches(aspOpenMVG::matching::PairWiseMatches const& match_map,
                  std::string const& matches_file) {
  // Save the matches to disk in the format: cidi_fidi cidj_fidj
  LOG(INFO) << "Writing: " << matches_file;
  std::ofstream mfile(matches_file.c_str());
  for (aspOpenMVG::matching::PairWiseMatches::const_iterator iter = match_map.begin();
       iter != match_map.end(); ++iter) {
    const size_t & I = iter->first.first;
    const size_t & J = iter->first.second;
    const std::vector<aspOpenMVG::matching::IndMatch> & matchVec = iter->second;
    // We have correspondences between I and J image indices
    for (size_t k = 0; k < matchVec.size(); ++k) {
      mfile << I << "_" << matchVec[k].i_ << " "
            << J << "_" << matchVec[k].j_ << std::endl;
    }
  }
  mfile.close();
}

// Filter the matches by a geometric constraint. Compute the essential matrix.
void BuildMapFindEssentialAndInliers(Eigen::Matrix2Xd const& keypoints1,
                                     Eigen::Matrix2Xd const& keypoints2,
                                     std::vector<cv::DMatch> const& matches,
                                     camera::CameraParameters const& camera_params,
                                     bool compute_inliers_only,
                                     size_t cam_a_idx, size_t cam_b_idx,
                                     std::mutex * match_mutex,
                                     CIDPairAffineMap * relative_affines,
                                     std::vector<cv::DMatch> * inlier_matches,
                                     bool compute_rays_angle,
                                     double * rays_angle) {
  // Initialize the outputs
  inlier_matches->clear();
  if (compute_rays_angle)
    *rays_angle = 0.0;

  int pt_count = matches.size();
  Eigen::MatrixXd observationsa(2, pt_count);
  Eigen::MatrixXd observationsb(2, pt_count);
  for (int i = 0; i < pt_count; i++) {
    observationsa.col(i) = keypoints1.col(matches[i].queryIdx);
    observationsb.col(i) = keypoints2.col(matches[i].trainIdx);
  }

  std::pair<size_t, size_t> image_size(camera_params.GetUndistortedSize()[0],
                                       camera_params.GetUndistortedSize()[1]);
  Eigen::Matrix3d k = camera_params.GetIntrinsicMatrix<camera::UNDISTORTED_C>();

  Eigen::Matrix3d e;
  // Calculate the essential matrix
  std::vector<size_t> vec_inliers;
  double error_max = std::numeric_limits<double>::max();
  double max_expected_error = 2.5;

  if (!interest_point::RobustEssential(k, k, observationsa, observationsb,
                                       &e, &vec_inliers,
                                       image_size, image_size,
                                       &error_max,
                                       max_expected_error)) {
    VLOG(2) << cam_a_idx << " " << cam_b_idx
            << " | Estimation of essential matrix failed!\n";
    return;
  }

  if (vec_inliers.size() < static_cast<size_t>(FLAGS_min_valid)) {
    VLOG(2) << cam_a_idx << " " << cam_b_idx
            << " | Failed to get enough inliers " << vec_inliers.size();
    return;
  }

  if (compute_inliers_only) {
    // We only need to know which interest points are inliers and not the
    // R and T matrices.
    int num_inliers = vec_inliers.size();
    inlier_matches->clear();
    inlier_matches->reserve(num_inliers);
    std::vector<Eigen::Matrix2Xd> observations2(2, Eigen::Matrix2Xd(2, num_inliers));
    for (int i = 0; i < num_inliers; i++) {
      inlier_matches->push_back(matches[vec_inliers[i]]);
    }
    return;
  }

  // Estimate the best possible R & T from the found Essential Matrix
  Eigen::Matrix3d r;
  Eigen::Vector3d t;
  if (!interest_point::EstimateRTFromE(k, k, observationsa, observationsb,
                                       e, vec_inliers,
                                       &r, &t)) {
    VLOG(2) << cam_a_idx << " " << cam_b_idx
            << " | Failed to extract RT from E";
    return;
  }

  VLOG(2) << cam_a_idx << " " << cam_b_idx << " | Inliers from E: "
          << vec_inliers.size() << " / " << observationsa.cols();

  // Get the observations corresponding to inliers
  // TODO(ZACK): We could reuse everything.
  int num_inliers = vec_inliers.size();
  std::vector<Eigen::Matrix2Xd> observations2(2, Eigen::Matrix2Xd(2, num_inliers));
  for (int i = 0; i < num_inliers; i++) {
    observations2[0].col(i) = observationsa.col(vec_inliers[i]);
    observations2[1].col(i) = observationsb.col(vec_inliers[i]);
  }

  // Refine the found T and R via bundle adjustment
  ceres::Solver::Options options;
  options.linear_solver_type = ceres::ITERATIVE_SCHUR;
  options.max_num_iterations = 200;
  options.logging_type = ceres::SILENT;
  options.num_threads = FLAGS_num_threads;
  ceres::Solver::Summary summary;
  std::vector<Eigen::Affine3d> cameras(2);
  cameras[0].setIdentity();
  cameras[1].linear() = r;
  cameras[1].translation() = t;
  Eigen::Matrix3Xd pid_to_xyz(3, observations2[0].cols());
  double error;
  int num_pts_behind_camera = 0;
  for (ptrdiff_t i = 0; i < observations2[0].cols(); i++) {
    pid_to_xyz.col(i) =
      sparse_mapping::TriangulatePoint
      (Eigen::Vector3d(observations2[0](0, i), observations2[0](1, i),
                       camera_params.GetFocalLength()),
       Eigen::Vector3d(observations2[1](0, i), observations2[1](1, i),
                       camera_params.GetFocalLength()),
       r, t, &error);
    Eigen::Vector3d P = pid_to_xyz.col(i);
    Eigen::Vector3d Q = r*P + t;
    if (P[2] <= 0 || Q[2] <= 0) {
      num_pts_behind_camera++;
    }
  }
  VLOG(2) << "Pair " << cam_a_idx  << ' ' << cam_b_idx
          << ": number of points behind cameras: "
          << num_pts_behind_camera << "/" <<  observations2[0].cols()
          << " (" << round((100.0*num_pts_behind_camera) / observations2[0].cols())
          << "%)";

  sparse_mapping::BundleAdjustSmallSet(observations2, camera_params.GetFocalLength(), &cameras,
                                       &pid_to_xyz, new ceres::CauchyLoss(0.5), options,
                                       &summary);

  if (!summary.IsSolutionUsable()) {
    LOG(ERROR) << cam_a_idx << " " << cam_b_idx << " | Failed to refine RT with bundle adjustment";
    return;
  }
  VLOG(2) << summary.BriefReport();

  if (compute_rays_angle) {
    // Compute the median angle between rays.
    std::vector<double> angles;
    Eigen::Vector3d ctr0 = cameras[0].inverse().translation();
    Eigen::Vector3d ctr1 = cameras[1].inverse().translation();

    for (ptrdiff_t i = 0; i < observations2[0].cols(); i++) {
      Eigen::Vector3d P =
        sparse_mapping::TriangulatePoint
        (Eigen::Vector3d(observations2[0](0, i), observations2[0](1, i),
                         camera_params.GetFocalLength()),
         Eigen::Vector3d(observations2[1](0, i), observations2[1](1, i),
                         camera_params.GetFocalLength()),
         cameras[1].linear(), cameras[1].translation(), &error);

      Eigen::Vector3d X0 = ctr0 - P;
      Eigen::Vector3d X1 = ctr1 - P;
      double l0 = X0.norm(), l1 = X1.norm();
      double angle;
      // TODO(oalexan1): Integrate this code with the other angle computation
      // code.
      if (l0 == 0 || l1 == 0) {
        angle = 0.0;
      } else {
        double dot = X0.dot(X1)/l0/l1;
        dot = std::min(dot, 1.0);
        dot = std::max(-1.0, dot);
        angle = (180.0/M_PI)*acos(dot);
      }
      angles.push_back(angle);
    }
    // Median rays angle
    if (angles.size() >= static_cast<size_t>(2*FLAGS_min_valid))
      *rays_angle = angles[angles.size()/2];
  }

  // Give the solution
  Eigen::Affine3d result = cameras[1] * cameras[0].inverse();
  result.translation().normalize();

  // Must use a lock to protect this map shared among the threads
  CHECK(match_mutex) << "Forgot to provide the mutex lock.";
  CHECK(relative_affines) << "Forgot to provide relative_affines argument.";
  match_mutex->lock();
  relative_affines->insert(std::make_pair(std::make_pair(cam_a_idx, cam_b_idx),
                                        result));
  match_mutex->unlock();

  cv::Mat valid = cv::Mat::zeros(pt_count, 1, CV_8UC1);
  for (size_t i = 0; i < vec_inliers.size(); i++) {
    valid.at<uint8_t>(vec_inliers[i], 0) = 1;
  }

  // Count the number of inliers
  int32_t num_of_inliers =
    std::accumulate(valid.begin<uint8_t>(), valid.end<uint8_t>(), 0);

  // Keep about FLAGS_max_pairwise_matches inliers. This is to speed
  // up map generation so that we don't have to bother with a 1000
  // matches between consecutive images.
  if (FLAGS_max_pairwise_matches < num_of_inliers) {
    std::vector<double> dist;
    for (size_t query_index = 0; query_index < matches.size(); query_index++) {
      if (valid.at<uint8_t>(query_index, 0) > 0)
        dist.push_back(matches[query_index].distance);
    }
    std::sort(dist.begin(), dist.end());
    double max_dist = dist[FLAGS_max_pairwise_matches - 1];
    for (size_t query_index = 0; query_index < matches.size(); query_index++) {
      if (valid.at<uint8_t>(query_index, 0) > 0 &&
          matches[query_index].distance > max_dist) {
        valid.at<uint8_t>(query_index, 0) = 0;
      }
    }
    num_of_inliers
      = std::accumulate(valid.begin<uint8_t>(), valid.end<uint8_t>(), 0);
  }

  // Copy the inliers only
  inlier_matches->clear();
  inlier_matches->reserve(num_of_inliers);
  for (size_t m = 0; m < matches.size(); m++) {
    if (valid.at<uint8_t>(m, 0) > 0) {
      inlier_matches->push_back(matches[m]);
    }
  }
}

void BuildMapPerformMatching(aspOpenMVG::matching::PairWiseMatches * match_map,
                             std::vector<Eigen::Matrix2Xd > const& cid_to_keypoint_map,
                             std::vector<cv::Mat> const& cid_to_descriptor_map,
                             camera::CameraParameters const& camera_params,
                             CIDPairAffineMap * relative_affines,
                             std::mutex * match_mutex,
                             int i /*query cid index*/, int j /*train cid index*/,
                             bool compute_rays_angle, double * rays_angle) {
  Eigen::Matrix2Xd const& keypoints1 = cid_to_keypoint_map[i];
  Eigen::Matrix2Xd const& keypoints2 = cid_to_keypoint_map[j];

  std::vector<cv::DMatch> matches, inlier_matches;
  interest_point::FindMatches(cid_to_descriptor_map[i],
                              cid_to_descriptor_map[j],
                              &matches);

  // Do a check and verify that we meet our minimum before the
  // essential matrix fitting.
  if (static_cast<int32_t>(matches.size()) < FLAGS_min_valid) {
    if (!FLAGS_silent_matching) LOG(INFO) << i << " " << j << " | Failed to find enough matches " << matches.size();
    return;
  }

  bool compute_inliers_only = false;
  BuildMapFindEssentialAndInliers(keypoints1, keypoints2, matches,
                                  camera_params, compute_inliers_only,
                                  i, j,
                                  match_mutex,
                                  relative_affines,
                                  &inlier_matches,
                                  compute_rays_angle, rays_angle);

  if (static_cast<int32_t>(inlier_matches.size()) < FLAGS_min_valid) {
    if (!FLAGS_silent_matching)
      LOG(INFO) << i << " " << j << " | Failed to find enough inlier matches "
                << inlier_matches.size();
    return;
  }

  if (!FLAGS_silent_matching) LOG(INFO) << i << " " << j << " success " << inlier_matches.size();

  std::vector<aspOpenMVG::matching::IndMatch> mvg_matches;
  for (std::vector<cv::DMatch>::value_type const& match : inlier_matches)
    mvg_matches.push_back(aspOpenMVG::matching::IndMatch(match.queryIdx, match.trainIdx));
  match_mutex->lock();
  (*match_map)[ std::make_pair(i, j) ] = mvg_matches;
  match_mutex->unlock();
}

// Create the initial map by feature matching and essential affine computation.
// TODO(oalexan1): Remove the logic which requires saving intermediate results to disk.
// Keep it all in memory.
void MatchFeatures(const std::string & essential_file,
                   const std::string & matches_file,
                   sparse_mapping::SparseMap * s) {
  sparse_mapping::CIDPairAffineMap relative_affines;

  // Iterate through the cid pairings
  rig::ThreadPool thread_pool;
  std::mutex match_mutex;

  aspOpenMVG::matching::PairWiseMatches match_map;
  for (size_t cid = 0; cid < s->cid_to_keypoint_map_.size(); cid++) {
    std::vector<int> indices, queried_indices;
    // No matches in the db, or no db was provided.
    if ( s->cid_to_cid_.find(cid) != s->cid_to_cid_.end() ) {
      // See if perhaps we know which images to match to from a
      // previous map
      std::set<int> & matches = s->cid_to_cid_.find(cid)->second;
      for (auto it = matches.begin(); it != matches.end() ; it++) {
        indices.push_back(*it);
      }
    } else {
      // No way out, try matching brute force to subsequent images
      int subsequent = FLAGS_num_subsequent_images;
      if (FLAGS_match_all_rate > 0 && cid % FLAGS_match_all_rate == 0)
        subsequent = static_cast<int>(s->cid_to_keypoint_map_.size());
      int end = std::min(static_cast<int>(cid) + subsequent + 1,
                         static_cast<int>(s->cid_to_keypoint_map_.size()));
      for (int j = cid + 1; j < end; j++) {
        // Use subsequent images
        indices.push_back(j);
      }
    }

    bool compute_rays_angle = false;
    double rays_angle;
    for (size_t j = 0; j < indices.size(); j++) {
      // Need the check below for loop closing to pass in unit tests
      if (s->cid_to_filename_[cid] != s->cid_to_filename_[indices[j]]) {
        thread_pool.AddTask(&sparse_mapping::BuildMapPerformMatching,
                            &match_map,
                            s->cid_to_keypoint_map_,
                            s->cid_to_descriptor_map_,
                            std::cref(s->camera_params_),
                            &relative_affines,
                            &match_mutex,
                            cid, indices[j],
                            compute_rays_angle, &rays_angle);
      }
    }
  }
  thread_pool.Join();

  LOG(INFO) << "Number of affines found: " << relative_affines.size() << "\n";

  // Write the solution
  sparse_mapping::WriteAffineCSV(relative_affines, essential_file);

  WriteMatches(match_map, matches_file);

  // Initial cameras based on the affines (won't be used later,
  // just for visualization purposes).
  int num_images = s->cid_to_filename_.size();
  (s->cid_to_cam_t_global_).resize(num_images);
  (s->cid_to_cam_t_global_)[0].setIdentity();
  for (int cid = 1; cid < num_images; cid++) {
    std::pair<int, int> P(cid-1, cid);
    if (relative_affines.find(P) != relative_affines.end())
      (s->cid_to_cam_t_global_)[cid] = relative_affines[P]*(s->cid_to_cam_t_global_)[cid-1];
    else
      (s->cid_to_cam_t_global_)[cid] = (s->cid_to_cam_t_global_)[cid-1];  // no choice
  }
}

void BuildTracks(bool rm_invalid_xyz,
                 const std::string & matches_file,
                 sparse_mapping::SparseMap * s) {
  aspOpenMVG::matching::PairWiseMatches match_map;
  ReadMatches(matches_file, &match_map);

  // Build tracks using the interface tracksbuilder
  aspOpenMVG::tracks::TracksBuilder trackBuilder;
  trackBuilder.Build(match_map);  // Build:  Efficient fusion of correspondences
  trackBuilder.Filter();          // Filter: Remove tracks that have conflict
  // trackBuilder.ExportToStream(std::cout);
  aspOpenMVG::tracks::STLMAPTracks map_tracks;
  // Export tracks as a map (each entry is a sequence of imageId and featureIndex):
  //  {TrackIndex => {(imageIndex, featureIndex), ... ,(imageIndex, featureIndex)}
  trackBuilder.ExportToSTL(map_tracks);

  // TODO(oalexan1): Print how many pairwise matches were there before
  // and after filtering tracks.

  if (map_tracks.empty())
    LOG(FATAL) << "No tracks left after filtering. Perhaps images are too dis-similar?\n";

  size_t num_elems = map_tracks.size();
  // Populate back the filtered tracks.
  (s->pid_to_cid_fid_).clear();
  (s->pid_to_cid_fid_).resize(num_elems);
  size_t curr_id = 0;
  for (auto itr = map_tracks.begin(); itr != map_tracks.end(); itr++) {
    for (auto itr2 = (itr->second).begin(); itr2 != (itr->second).end(); itr2++) {
      (s->pid_to_cid_fid_)[curr_id][itr2->first] = itr2->second;
    }
    curr_id++;
  }

  // Triangulate. The results should be quite inaccurate, we'll redo this
  // later. This step is mostly for consistency.
  sparse_mapping::Triangulate(rm_invalid_xyz,
                              s->camera_params_.GetFocalLength(),
                              s->cid_to_cam_t_global_,
                              s->cid_to_keypoint_map_,
                              &(s->pid_to_cid_fid_),
                              &(s->pid_to_xyz_),
                              &(s->cid_fid_to_pid_));


  // Wipe file that is no longer needed
  try {
    std::remove(matches_file.c_str());
  }catch(...) {}

  // PrintTrackStats(s->pid_to_cid_fid_, "track building");
}

// TODO(oalexan1): This very naive code can use serious performance
// improvements.  Each time we add a new camera we triangulate all
// points. We bundle-adjust the last several cameras, but while seeing
// (and keeping fixed) all the earlier cameras. It is sufficient to
// both triangulate and see during bundle adjustment only the several
// most similar cameras. Fixing these would need careful testing for
// both map quality and run-time before and after the fix.
void IncrementalBA(std::string const& essential_file,
                   sparse_mapping::SparseMap * s) {
  // Do incremental bundle adjustment.

  // Optimize only the last several cameras, their number varies
  // between min_num_cams and max_num_cams.

  // TODO(oalexan1): Need to research how many previous cameras we
  // need for loop closure.
  int min_num_cams = 4;
  int max_num_cams = 128;

  // Read in all the affine R|t combinations between cameras
  sparse_mapping::CIDPairAffineMap relative_affines;
  sparse_mapping::ReadAffineCSV(essential_file,
                                &relative_affines);

  int num_images = s->cid_to_filename_.size();

  // Track and camera info up to the current cid
  std::vector<std::map<int, int>> pid_to_cid_fid_local;
  std::vector<Eigen::Affine3d> cid_to_cam_t_local;
  std::vector<Eigen::Vector3d> pid_to_xyz_local;
  std::vector<std::map<int, int>> cid_fid_to_pid_local;

  bool rm_invalid_xyz = true;

  for (int cid = 1; cid < num_images; cid++) {
    // The array of cameras so far including this one
    cid_to_cam_t_local.resize(cid + 1);
    for (int c = 0; c < cid; c++)
      cid_to_cam_t_local[c] = s->cid_to_cam_t_global_[c];

    // Add a new camera. Obtain it based on relative affines. Here we assume
    // the current camera is similar to the previous one.
    std::pair<int, int> P(cid-1, cid);
    if (relative_affines.find(P) != relative_affines.end())
      cid_to_cam_t_local[cid] = relative_affines[P]*cid_to_cam_t_local[cid-1];
    else
      cid_to_cam_t_local[cid] = cid_to_cam_t_local[cid-1];  // no choice

    // Restrict tracks to images up to cid.
    pid_to_cid_fid_local.clear();
    for (size_t p = 0; p < s->pid_to_cid_fid_.size(); p++) {
      std::map<int, int> & long_track = s->pid_to_cid_fid_[p];
      std::map<int, int> track;
      for (std::map<int, int>::iterator it = long_track.begin();
           it != long_track.end() ; it++) {
        if (it->first <= cid)
          track[it->first] = it->second;
      }

      // This is absolutely essential, using tracks of length >= 3
      // only greatly increases the reliability.
      if ( (cid == 1 && track.size() > 1) || track.size() > 2 )
        pid_to_cid_fid_local.push_back(track);
    }

    // Perform triangulation of all points. Multiview triangulation is
    // used.
    pid_to_xyz_local.clear();
    std::vector<std::map<int, int>> cid_fid_to_pid_local;
    sparse_mapping::Triangulate(rm_invalid_xyz,
                                s->camera_params_.GetFocalLength(),
                                cid_to_cam_t_local,
                                s->cid_to_keypoint_map_,
                                &pid_to_cid_fid_local,
                                &pid_to_xyz_local,
                                &cid_fid_to_pid_local);

    ceres::Solver::Options options;
    options.linear_solver_type = ceres::ITERATIVE_SCHUR;
    options.max_num_iterations = 500;
    options.logging_type = ceres::SILENT;
    options.num_threads = FLAGS_num_threads;
    ceres::Solver::Summary summary;
    ceres::LossFunction* loss = new ceres::CauchyLoss(0.5);

    // If cid+1 is divisible by 2^k, do at least 2^k cameras, ending
    // with camera cid.  E.g., if current camera index is 23 = 3*8-1, do at
    // least 8 cameras, so cameras 16, ..., 23. This way, we will try
    // to occasionally do more than just several close cameras.
    int val = cid+1;
    int offset = 1;
    while (val % 2 == 0) {
      val /= 2;
      offset *= 2;
    }
    offset = std::min(offset, max_num_cams);

    int start = cid-offset+1;
    start = std::min(cid-min_num_cams+1, start);
    if (start < 0) start = 0;

    LOG(INFO) << "Optimizing cameras from " << start << " to " << cid << " (total: "
        << cid-start+1 << ")";

    sparse_mapping::BundleAdjust(pid_to_cid_fid_local, s->cid_to_keypoint_map_,
                                 s->camera_params_.GetFocalLength(),
                                 &cid_to_cam_t_local, &pid_to_xyz_local,
                                 s->user_pid_to_cid_fid_,
                                 s->user_cid_to_keypoint_map_,
                                 &(s->user_pid_to_xyz_),
                                 loss, options, &summary,
                                 start, cid);

    // Copy back
    for (int c = 0; c <= cid; c++)
      s->cid_to_cam_t_global_[c] = cid_to_cam_t_local[c];
  }

  // Triangulate all points
  sparse_mapping::Triangulate(rm_invalid_xyz,
                              s->camera_params_.GetFocalLength(),
                              s->cid_to_cam_t_global_,
                              s->cid_to_keypoint_map_,
                              &(s->pid_to_cid_fid_),
                              &(s->pid_to_xyz_),
                              &(s->cid_fid_to_pid_));

  // Wipe file that is no longer needed
  try {
    std::remove(essential_file.c_str());
  }catch(...) {}
}

// Close loop after incremental BA
void CloseLoop(sparse_mapping::SparseMap * s) {
  // Consider a map with n images, where the first and the last image
  // are very similar. We would like to create a closed loop sequence
  // of cameras. To do that, after the last image we append the first several
  // images (parameter num_repeat_images), and do incremental BA.
  // We get a loop which ideally would perfectly overlap with itself at
  // the ends, but in practice does not. If the drift is small however,
  // in this function we identify repeated images and pids, and blend
  // the repeated camera positions and orientations. After this, the
  // user is meant to do another bundle adjustment step.

  // This process won't perform well if the drift is huge. In that case
  // the user is expected to redo the SFM calculations, perhaps
  // using better-positioned interest points, or or more of them.

  int num_images = s->cid_to_filename_.size();
  std::vector<std::string> & images = s->cid_to_filename_;  // alias
  std::map<std::string, int> image_map;

  // The first image to repeat is images[s1], it will also show
  // at position s2. The last image to repeat is image[e1],
  // it will also show at position e2.
  int s1 = -1, e1 = -1, s2 = -1, e2 = -1;
  bool has_repeat = false;
  for (int cid = 0; cid < num_images; cid++) {
    std::string image = images[cid];

    std::map<std::string, int>::iterator it = image_map.find(image);
    if (it == image_map.end()) {
      if (has_repeat) {
        // No more repeat images.
        break;
      }
      image_map[image] = cid;
      continue;
    }

    if (!has_repeat) {
      has_repeat = true;
      s1 = it->second;
      s2 = cid;
    }
    e1 = it->second;
    e2 = cid;
  }

  if (!has_repeat) {
    LOG(INFO) << "Could not find a loop to close";
    return;
  }

  // Sanity checks
  if (s1 < 0 || s2 < 0)
    LOG(FATAL) << "Could not find repeat images, failure in CloseLoop()";
  if (s2 - s1 != e2 - e1 || s1 >= e1)
    LOG(FATAL) << "Book-keeping failure in CloseLoop()";
  if (s1 != 0)
    LOG(FATAL) << "Situation not implemented in CloseLoop()";
  if (images[s1] != images[s2] || images[e1] != images[e2])
    LOG(FATAL) << "Expecting repeat images in CloseLoop().";

  // Blend the cameras. Start by giving full weight to the repeated
  // cameras, and gradually shifting the weight to the original
  // cameras.
  int pad = 0.1*(e1-s1);  // to make the blending a bit gentler
  double den = e1 - s1 - 2*pad;
  for (int cid = s1; cid <= e1; cid++) {
    int cid2 = cid - s1 + s2;
    double wt1 = (cid - s1 - pad)/den;
    if (wt1 < 0.0) wt1 = 0.0;
    if (wt1 > 1.0) wt1 = 1.0;
    double wt2 = 1.0 - wt1;

    // Blend rotations
    Eigen::Quaternion<double> q1(s->cid_to_cam_t_global_[cid].linear());
    Eigen::Quaternion<double> q2(s->cid_to_cam_t_global_[cid2].linear());
    Eigen::Quaternion<double> q = q2.slerp(wt1, q1);
    Eigen::Matrix3d R = q.toRotationMatrix();

    // Blend translations
    s->cid_to_cam_t_global_[cid].translation()
      = wt1*s->cid_to_cam_t_global_[cid].translation()
      + wt2*s->cid_to_cam_t_global_[cid2].translation();
    s->cid_to_cam_t_global_[cid].linear() = R;
  }

  // Merge the pids after identifying repeat images
  sparse_mapping::MergePids(e1, s2, &(s->pid_to_cid_fid_));

  // Wipe the now redundant info
  s->cid_to_filename_.resize(s2);
  s->cid_to_keypoint_map_.resize(s2);
  s->cid_to_cam_t_global_.resize(s2);
  s->cid_to_descriptor_map_.resize(s2);

  // sparse_mapping::PrintPidStats(s->pid_to_cid_fid_);
  bool rm_invalid_xyz = true;
  sparse_mapping::Triangulate(rm_invalid_xyz,
                              s->camera_params_.GetFocalLength(),
                              s->cid_to_cam_t_global_,
                              s->cid_to_keypoint_map_,
                              &(s->pid_to_cid_fid_),
                              &(s->pid_to_xyz_),
                              &(s->cid_fid_to_pid_));
}

void BundleAdjust(bool fix_all_cameras, sparse_mapping::SparseMap * map,
                  std::set<int> const& fixed_cameras) {
  for (int i = 0; i < FLAGS_num_ba_passes; i++) {
    LOG(INFO) << "Beginning bundle adjustment, pass: " << i << ".\n";

    // perform bundle adjustment
    ceres::Solver::Options options;
    // options.linear_solver_type = ceres::SPARSE_SCHUR; // Need to be building SuiteSparse
    options.linear_solver_type = ceres::ITERATIVE_SCHUR;
    // What should the preconditioner be?
    options.num_threads = FLAGS_num_threads;
    options.max_num_iterations = FLAGS_max_num_iterations;
    options.minimizer_progress_to_stdout = true;
    ceres::Solver::Summary summary;
    ceres::LossFunction* loss
      = sparse_mapping::GetLossFunction(FLAGS_cost_function,
                                        FLAGS_cost_function_threshold);
    sparse_mapping::BundleAdjustment(map, loss, options, &summary,
                                     FLAGS_first_ba_index, FLAGS_last_ba_index,
                                     fix_all_cameras, fixed_cameras);

    LOG(INFO) << summary.FullReport() << "\n";
    LOG(INFO) << "Starting average reprojection error: "
              << summary.initial_cost / map->GetNumObservations();
    LOG(INFO) << "Final average reprojection error:    "
              << summary.final_cost / map->GetNumObservations();
  }
}

void BundleAdjustment(sparse_mapping::SparseMap * s,
                      ceres::LossFunction* loss,
                      const ceres::Solver::Options & options,
                      ceres::Solver::Summary* summary,
                      int first, int last, bool fix_all_cameras,
                      std::set<int> const& fixed_cameras) {
  sparse_mapping::BundleAdjust(s->pid_to_cid_fid_, s->cid_to_keypoint_map_,
                               s->camera_params_.GetFocalLength(),
                               &(s->cid_to_cam_t_global_),
                               &(s->pid_to_xyz_),
                               s->user_pid_to_cid_fid_, s->user_cid_to_keypoint_map_,
                               &(s->user_pid_to_xyz_),
                               loss, options, summary, first, last, fix_all_cameras,
                               fixed_cameras);

  // First do BA, and only afterwards remove outliers.
  if (!FLAGS_skip_filtering) {
    FilterPID(FLAGS_reproj_thresh,  s->camera_params_, s->cid_to_cam_t_global_,
              s->cid_to_keypoint_map_, &(s->pid_to_cid_fid_), &(s->pid_to_xyz_));
    s->InitializeCidFidToPid();
  }

  // sparse_mapping::PrintPidStats(s->pid_to_cid_fid_);
  // PrintTrackStats(s->pid_to_cid_fid_, "bundle adjustment and filtering");
}
  
// Extract a submap in-place.
void ExtractSubmap(std::vector<std::string> const& images_to_keep,
                   rig::nvmData & nvm) {

  // Sanity check. The images to keep must exist in the original map.
  std::map<std::string, int> image2cid;
  for (size_t cid = 0; cid < nvm.cid_to_filename.size(); cid++)
    image2cid[nvm.cid_to_filename[cid]] = cid;
  for (size_t cid = 0; cid < images_to_keep.size(); cid++) {
    if (image2cid.find(images_to_keep[cid]) == image2cid.end())
      std::cout << "Warning: Could not find in the input map the image: "
                << images_to_keep[cid] << "\n";
  }

  // To extract the submap-in place, it is simpler to reorder the images
  // to extract to be in the same order as in the map. Keep those in
  // local vector 'keep'.
  std::vector<std::string> keep;
  {
    std::set<std::string> keep_set;
    for (size_t cid = 0; cid < images_to_keep.size(); cid++)
      keep_set.insert(images_to_keep[cid]);
    for (size_t cid = 0; cid < nvm.cid_to_filename.size(); cid++) {
      if (keep_set.find(nvm.cid_to_filename[cid]) != keep_set.end())
        keep.push_back(nvm.cid_to_filename[cid]);
    }
  }

  // Map each image we keep to its index
  std::map<std::string, int> keep2cid;
  for (size_t cid = 0; cid < keep.size(); cid++)
    keep2cid[keep[cid]] = cid;

  // The map from the old cid to the new cid
  std::map<int, int> cid2cid;
  for (size_t cid = 0; cid < nvm.cid_to_filename.size(); cid++) {
    auto it = keep2cid.find(nvm.cid_to_filename[cid]);
    if (it == keep2cid.end()) continue;  // current image is not in the final submap
    cid2cid[cid] = it->second;
  }

  // Sanity checks. All the kept images must be represented in cid2cid,
  // and the values in cid2cid must be consecutive.
  if (cid2cid.size() != keep.size() || cid2cid.empty())
    LOG(FATAL) << "Cannot extract a submap. Check your inputs. Maybe some images "
               << "are duplicated or none are in the map.";
  for (auto it = cid2cid.begin(); it != cid2cid.end(); it++) {
    auto it2 = it; it2++;
    if (it2 == cid2cid.end()) continue;
    if (it->second + 1 != it2->second || cid2cid.begin()->second != 0 )
      LOG(FATAL) << "Cannot extract a submap. Check if the images "
                 << "you want to keep are in the same order as in the original map.";
  }

  // Over-write the data in-place. Should be safe with the checks done above.
  int num_cid = keep.size();
  for (size_t cid = 0; cid < nvm.cid_to_filename.size(); cid++) {
    if (cid2cid.find(cid) == cid2cid.end()) continue;
    size_t new_cid = cid2cid[cid];
    nvm.cid_to_filename[new_cid]     = nvm.cid_to_filename[cid];
    nvm.cid_to_keypoint_map[new_cid] = nvm.cid_to_keypoint_map[cid];
    nvm.cid_to_cam_t_global[new_cid] = nvm.cid_to_cam_t_global[cid];
  }
  nvm.cid_to_filename.resize(num_cid);
  nvm.cid_to_keypoint_map.resize(num_cid);
  nvm.cid_to_cam_t_global.resize(num_cid);

  // Create new pid_to_cid_fid and pid_to_xyz.
  std::vector<std::map<int, int>> pid_to_cid_fid;
  std::vector<Eigen::Vector3d> pid_to_xyz;
  for (size_t pid = 0; pid < nvm.pid_to_cid_fid.size(); pid++) {
    auto const& cid_fid = nvm.pid_to_cid_fid[pid];  // alias
    std::map<int, int> cid_fid2;
    for (auto it = cid_fid.begin(); it != cid_fid.end(); it++) {
      int cid = it->first;
      if (cid2cid.find(cid) == cid2cid.end()) continue;  // not an image we want to keep
      cid_fid2[cid2cid[cid]] = it->second; // fid does not change
    }
    if (cid_fid2.size() <= 1) continue;  // tracks must have size at least 2
    pid_to_cid_fid.push_back(cid_fid2);
    pid_to_xyz.push_back(nvm.pid_to_xyz[pid]);
  }
  nvm.pid_to_cid_fid = pid_to_cid_fid;
  nvm.pid_to_xyz = pid_to_xyz;

  // If the input nvm has optical centers, keep the subset for the submap.
  if (!nvm.optical_centers.empty()) {
    std::map<std::string, Eigen::Vector2d> optical_centers;
    for (size_t cid = 0; cid < images_to_keep.size(); cid++) {
      auto it = nvm.optical_centers.find(images_to_keep[cid]);
      if (it == nvm.optical_centers.end()) 
        LOG(FATAL) << "Cannot find the optical centers for the images in the submap.";
      optical_centers[images_to_keep[cid]] = it->second;
    }
    nvm.optical_centers = optical_centers; // overwrite in place
  }
   
  std::cout << "Number of images in the extracted map: " << nvm.cid_to_filename.size() << "\n";
  std::cout << "Number of tracks in the extracted map: " << nvm.pid_to_cid_fid.size() << "\n";

  return;
}


void PrintTrackStats(std::vector<std::map<int, int>>const& pid_to_cid_fid,
                       std::string const& step) {
  LOG(INFO) << "Track statistics after: " << step;

  double track_len = 0.0;
  for (size_t pid = 0; pid < pid_to_cid_fid.size(); pid++)
    track_len += pid_to_cid_fid[pid].size();
  double avg_len = track_len / pid_to_cid_fid.size();

  LOG(INFO) << "Number of tracks (points in the control network): " << pid_to_cid_fid.size();
  LOG(INFO) << "Total length of all tracks: " << track_len;
  LOG(INFO) << "Average track length: " << avg_len;

  std::map<int, int> stats;
  for (size_t pid = 0; pid < pid_to_cid_fid.size(); pid++)
    stats[pid_to_cid_fid[pid].size()]++;
  for (std::map<int, int>::const_iterator it = stats.begin(); it != stats.end() ; it++) {
    LOG(INFO) << "Track length and their number: "
              << it->first << ' ' << it->second;
  }
}

// I/O Functions
template <class IterT>
void WriteCIDPairAffineIterator(IterT it,
                                IterT end,
                                std::ofstream* file) {
  Eigen::IOFormat fmt(Eigen::FullPrecision, 0, " ", "\n", "", "", "", "");
  while (it != end) {
    *file << it->first.first << " " << it->first.second << std::endl;
    *file << it->second.affine().format(fmt) << std::endl;
    it++;
  }
}

template <class IterT>
void ReadAffine(std::ifstream* file,
                IterT output_iter) {
  std::string line[4];
  std::getline(*file, line[0]);
  std::getline(*file, line[1]);
  std::getline(*file, line[2]);
  std::getline(*file, line[3]);
  if (line[0].empty())
    return;

  int i, j;
  Eigen::Matrix3d r;
  Eigen::Vector3d t;
  {
    std::stringstream ss(line[0]);
    ss >> i >> j;
  }

  for (int k = 0; k < 3; k++) {
    std::stringstream ss(line[k + 1]);
    ss >> r(k, 0) >> r(k, 1) >> r(k, 2) >> t[k];
  }

  Eigen::Affine3d affine;
  affine.linear() = r;
  affine.translation() = t;
  *output_iter = std::make_pair(std::make_pair(i, j),
                                affine);
}

// Use a back inserter with this if you haven't previously allocated enough space.
template <class IterT>
void PushBackCIDPairAffine(std::ifstream* file,
                           IterT output_iter,
                           IterT output_iter_end) {
  do {
    ReadAffine(file, output_iter);
    output_iter++;
  } while (file->good() && output_iter != output_iter_end);
}

template <class IterT>
void PushBackCIDPairAffine(std::ifstream* file,
                           IterT iter) {
  do {
    ReadAffine(file, iter);
    iter++;
  } while (file->good());
}

void WriteAffineCSV(CIDPairAffineMap const& relative_affines,
                    std::string const& output_filename) {
  LOG(INFO) << "Writing: " << output_filename;
  std::ofstream f(output_filename, std::ofstream::out);
  WriteCIDPairAffineIterator(relative_affines.begin(),
                             relative_affines.end(),
                             &f);
  f.close();
}
void WriteAffineCSV(CIDAffineTupleVec const& relative_affines,
                    std::string const& output_filename) {
  LOG(INFO) << "Writing: " << output_filename;
  std::ofstream f(output_filename, std::ofstream::out);
  for (CIDAffineTupleVec::value_type const& tuple : relative_affines) {
    f << "Tuple:" << std::endl;
    WriteCIDPairAffineIterator(tuple.begin(), tuple.end(), &f);
  }
  f.close();
}
void ReadAffineCSV(std::string const& input_filename,
                   CIDPairAffineMap* relative_affines) {
  LOG(INFO) << "Reading: " << input_filename;
  std::ifstream f(input_filename, std::ifstream::in);
  if (!f.good())
    LOG(FATAL) << "Could no read: " << input_filename << ". Must redo the matching step.";
  relative_affines->clear();
  PushBackCIDPairAffine(&f, std::inserter(*relative_affines, relative_affines->begin()));
  f.close();
}
void ReadAffineCSV(std::string const& input_filename,
                   CIDAffineTupleVec* relative_affines) {
  std::ifstream f(input_filename, std::ifstream::in);
  if (!f.good())
    LOG(FATAL) << "Could no read: " << input_filename << ". Must redo the matching step.";
  relative_affines->clear();
  std::string line;
  std::getline(f, line);
  while (!line.empty()) {
    relative_affines->push_back({});
    PushBackCIDPairAffine(&f, relative_affines->back().begin(),
                          relative_affines->back().end());
    std::getline(f, line);
  }
  f.close();
}

void Triangulate(bool rm_invalid_xyz, double focal_length,
                 std::vector<Eigen::Affine3d> const& cid_to_cam_t_global,
                 std::vector<Eigen::Matrix2Xd> const& cid_to_keypoint_map,
                 std::vector<std::map<int, int>> * pid_to_cid_fid,
                 std::vector<Eigen::Vector3d> * pid_to_xyz,
                 std::vector<std::map<int, int>> * cid_fid_to_pid) {
  Eigen::Matrix3d k;
  k << focal_length, 0, 0,
    0, focal_length, 0,
    0, 0, 1;

  // Build p matrices for all of the cameras. aspOpenMVG::Triangulation
  // will be holding pointers to all of the cameras.
  std::vector<aspOpenMVG::Mat34> cid_to_p(cid_to_cam_t_global.size());
  for (size_t cid = 0; cid < cid_to_p.size(); cid++) {
    aspOpenMVG::P_From_KRt(k, cid_to_cam_t_global[cid].linear(),
                        cid_to_cam_t_global[cid].translation(), &cid_to_p[cid]);
  }

  pid_to_xyz->resize(pid_to_cid_fid->size());
  for (int pid = pid_to_cid_fid->size() - 1; pid >= 0; pid--) {
    aspOpenMVG::Triangulation tri;
    for (std::pair<int, int> const& cid_fid : pid_to_cid_fid->at(pid)) {
      tri.add(cid_to_p[cid_fid.first],  // they're holding a pointer to this
              cid_to_keypoint_map[cid_fid.first].col(cid_fid.second));
    }
    Eigen::Vector3d solution = tri.compute();
    if ( rm_invalid_xyz && (std::isnan(solution[0]) || tri.minDepth() < 0) ) {
      pid_to_xyz->erase(pid_to_xyz->begin() + pid);
      pid_to_cid_fid->erase(pid_to_cid_fid->begin() + pid);
    } else {
      pid_to_xyz->at(pid) = solution;
    }
  }

  // Must always keep the book-keeping correct
  sparse_mapping::InitializeCidFidToPid(cid_to_cam_t_global.size(),
                                        *pid_to_cid_fid,
                                        cid_fid_to_pid);
}

}  // namespace sparse_mapping
