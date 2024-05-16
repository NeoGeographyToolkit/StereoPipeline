/* Copyright (c) 2021, United States Government, as represented by the
 * Administrator of the National Aeronautics and Space Administration.
 *
 * All rights reserved.
 *
 * The "ISAAC - Integrated System for Autonomous and Adaptive Caretaking
 * platform" software is licensed under the Apache License, Version 2.0
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

// TODO(oalexan1): Move track logic to tracks.cc.

#include <Rig/basic_algs.h>
#include <Rig/interest_point.h>
#include <Rig/camera_image.h>
#include <Rig/system_utils.h>
#include <Rig/thread.h>
#include <Rig/matching.h>
#include <Rig/transform_utils.h>
#include <Rig/interpolation_utils.h>
#include <Rig/rig_config.h>
#include <Rig/random_set.h>
#include <Rig/image_lookup.h>
#include <Rig/nvm.h>

#include <Rig/RigCameraParams.h>

#include <opencv2/xfeatures2d.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>

// Get rid of warnings beyond our control
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
#pragma GCC diagnostic ignored "-Wunused-function"
#pragma GCC diagnostic push
#include <OpenMVG/projection.hpp>
#include <OpenMVG/triangulation_nview.hpp>
#include <OpenMVG/tracks.hpp>
#pragma GCC diagnostic pop

#include <boost/filesystem.hpp>

#include <iostream>
#include <fstream>
#include <iomanip>

namespace fs = boost::filesystem;

// SIFT is doing so much better than SURF for haz cam images.
DEFINE_string(feature_detector, "SIFT", "The feature detector to use. SIFT or SURF.");
DEFINE_int32(sift_nFeatures, 10000, "Number of SIFT features.");
DEFINE_int32(sift_nOctaveLayers, 3, "Number of SIFT octave layers.");
DEFINE_double(sift_contrastThreshold, 0.02,
              "SIFT contrast threshold");  // decrease for more ip
DEFINE_double(sift_edgeThreshold, 10, "SIFT edge threshold.");
DEFINE_double(sift_sigma, 1.6, "SIFT sigma.");

DECLARE_int32(max_pairwise_matches); // declared externally

namespace rig {

void detectFeatures(const cv::Mat& image, bool verbose,
                    // Outputs
                    cv::Mat* descriptors, Eigen::Matrix2Xd* keypoints) {
  bool histogram_equalization = false;

  // If using histogram equalization, need an extra image to store it
  cv::Mat* image_ptr = const_cast<cv::Mat*>(&image);
  cv::Mat hist_image;
  if (histogram_equalization) {
    cv::equalizeHist(image, hist_image);
    image_ptr = &hist_image;
  }

  std::vector<cv::KeyPoint> storage;

  if (FLAGS_feature_detector == "SIFT") {
    cv::Ptr<cv::SIFT> sift =
      cv::SIFT::create(FLAGS_sift_nFeatures, FLAGS_sift_nOctaveLayers,
                                    FLAGS_sift_contrastThreshold,
                                    FLAGS_sift_edgeThreshold, FLAGS_sift_sigma);
    sift->detect(image, storage);
    sift->compute(image, storage, *descriptors);

  } else if (FLAGS_feature_detector == "SURF") {
    interest_point::FeatureDetector detector("SURF");
    detector.Detect(*image_ptr, &storage, descriptors);

    // Undo the shift in the detector
    for (cv::KeyPoint& key : storage) {
      key.pt.x += image.cols / 2.0;
      key.pt.y += image.rows / 2.0;
    }

  } else {
    LOG(FATAL) << "Unknown feature detector: " << FLAGS_feature_detector;
  }

  if (verbose) std::cout << "Features detected " << storage.size() << std::endl;

  // Copy to data structures expected by subsequent code
  keypoints->resize(2, storage.size());
  Eigen::Vector2d output;
  for (size_t j = 0; j < storage.size(); j++) {
    keypoints->col(j) = Eigen::Vector2d(storage[j].pt.x, storage[j].pt.y);
  }
}

void reduceMatches(std::vector<InterestPoint> & left_ip,
                   std::vector<InterestPoint> & right_ip) {
  
  // pick a random subset
  std::vector<int> subset;
  rig::pick_random_indices_in_range(left_ip.size(), FLAGS_max_pairwise_matches, subset);
  std::sort(subset.begin(), subset.end()); // sort the indices; not strictly necessary
  
  std::vector<InterestPoint> left_ip_full, right_ip_full;
  left_ip_full.swap(left_ip);
  right_ip_full.swap(right_ip);
  
  left_ip.resize(FLAGS_max_pairwise_matches);
  right_ip.resize(FLAGS_max_pairwise_matches);
  for (size_t it = 0; it < subset.size(); it++) {
    left_ip[it] = left_ip_full[subset[it]];
    right_ip[it] = right_ip_full[subset[it]];
  }
}
  
// This really likes haz cam first and nav cam second
// Note: The function matchFeaturesWithCams() is used instead.
void matchFeatures(std::mutex* match_mutex,
                   int left_image_index, int right_image_index,
                   cv::Mat const& left_descriptors, cv::Mat const& right_descriptors,
                   Eigen::Matrix2Xd const& left_keypoints,
                   Eigen::Matrix2Xd const& right_keypoints, bool verbose,
                   // output
                   MATCH_PAIR* matches) {
  std::vector<cv::DMatch> cv_matches;
  interest_point::FindMatches(left_descriptors, right_descriptors, &cv_matches);

  std::vector<cv::Point2f> left_vec;
  std::vector<cv::Point2f> right_vec;
  for (size_t j = 0; j < cv_matches.size(); j++) {
    int left_ip_index = cv_matches.at(j).queryIdx;
    int right_ip_index = cv_matches.at(j).trainIdx;

    // Get the keypoints from the good matches
    left_vec.push_back(cv::Point2f(left_keypoints.col(left_ip_index)[0],
                                   left_keypoints.col(left_ip_index)[1]));
    right_vec.push_back(cv::Point2f(right_keypoints.col(right_ip_index)[0],
                                    right_keypoints.col(right_ip_index)[1]));
  }

  if (left_vec.empty()) return;

  // These may need some tweaking but works reasonably well.
  double ransacReprojThreshold = 20.0;
  cv::Mat inlier_mask;
  int maxIters = 10000;
  double confidence = 0.8;

  // affine2D works better than homography
  // cv::Mat H = cv::findHomography(left_vec, right_vec, cv::RANSAC,
  // ransacReprojThreshold, inlier_mask, maxIters, confidence);
  cv::Mat H = cv::estimateAffine2D(left_vec, right_vec, inlier_mask, cv::RANSAC,
                                   ransacReprojThreshold, maxIters, confidence);

  std::vector<InterestPoint> left_ip, right_ip;
  for (size_t j = 0; j < cv_matches.size(); j++) {
    int left_ip_index = cv_matches.at(j).queryIdx;
    int right_ip_index = cv_matches.at(j).trainIdx;

    if (inlier_mask.at<uchar>(j, 0) == 0) continue;

    cv::Mat left_desc = left_descriptors.row(left_ip_index);
    cv::Mat right_desc = right_descriptors.row(right_ip_index);

    InterestPoint left;
    left.setFromCvKeypoint(left_keypoints.col(left_ip_index), left_desc);

    InterestPoint right;
    right.setFromCvKeypoint(right_keypoints.col(right_ip_index), right_desc);

    left_ip.push_back(left);
    right_ip.push_back(right);
  }

  // Update the shared variable using a lock
  match_mutex->lock();

  // Print the verbose message inside the lock, otherwise the text
  // may get messed up.
  if (verbose)
    std::cout << "Number of matches for pair "
              << left_image_index << ' ' << right_image_index << ": "
              << left_ip.size() << std::endl;

  if (FLAGS_max_pairwise_matches >= 0 && (int)left_ip.size() > FLAGS_max_pairwise_matches)
    reduceMatches(left_ip, right_ip);

  *matches = std::make_pair(left_ip, right_ip);
  match_mutex->unlock();
}

// Match features while assuming that the input cameras can be used to filter out
// outliers by reprojection error.
// TODO(oalexan1): This can be fragile. What if input cameras have poor pointing info?
void matchFeaturesWithCams(std::mutex* match_mutex, 
                           int left_image_index, int right_image_index,
                           camera::CameraParameters const& left_params,
                           camera::CameraParameters const& right_params,
                           bool filter_matches_using_cams,
                           Eigen::Affine3d const& left_world_to_cam,
                           Eigen::Affine3d const& right_world_to_cam,
                           double reprojection_error,
                           cv::Mat const& left_descriptors,
                           cv::Mat const& right_descriptors,
                           Eigen::Matrix2Xd const& left_keypoints,
                           Eigen::Matrix2Xd const& right_keypoints,
                           bool verbose,
                           // output
                           MATCH_PAIR* matches) {
  // Match by using descriptors first
  std::vector<cv::DMatch> cv_matches;
  interest_point::FindMatches(left_descriptors, right_descriptors, &cv_matches);

  // Do filtering
  std::vector<cv::Point2f> left_vec;
  std::vector<cv::Point2f> right_vec;
  std::vector<cv::DMatch> filtered_cv_matches;
  for (size_t j = 0; j < cv_matches.size(); j++) {
    int left_ip_index = cv_matches.at(j).queryIdx;
    int right_ip_index = cv_matches.at(j).trainIdx;


    // We may not always have the cameras to use in filtering
    if (filter_matches_using_cams) {
      Eigen::Vector2d dist_left_ip(left_keypoints.col(left_ip_index)[0],
                                   left_keypoints.col(left_ip_index)[1]);

      Eigen::Vector2d dist_right_ip(right_keypoints.col(right_ip_index)[0],
                                    right_keypoints.col(right_ip_index)[1]);

      Eigen::Vector2d undist_left_ip;
      Eigen::Vector2d undist_right_ip;
      left_params.Convert<camera::DISTORTED,  camera::UNDISTORTED_C>
        (dist_left_ip, &undist_left_ip);
      right_params.Convert<camera::DISTORTED, camera::UNDISTORTED_C>
        (dist_right_ip, &undist_right_ip);

      Eigen::Vector3d X =
        rig::TriangulatePair(left_params.GetFocalLength(),
                                   right_params.GetFocalLength(),
                                   left_world_to_cam, right_world_to_cam,
                                   undist_left_ip, undist_right_ip);

      // Project back into the cameras
      Eigen::Vector3d left_cam_X = left_world_to_cam * X;
      Eigen::Vector2d undist_left_pix
        = left_params.GetFocalVector().cwiseProduct(left_cam_X.hnormalized());
      Eigen::Vector2d dist_left_pix;
      left_params.Convert<camera::UNDISTORTED_C, camera::DISTORTED>(undist_left_pix,
                                                                    &dist_left_pix);

      Eigen::Vector3d right_cam_X = right_world_to_cam * X;
      Eigen::Vector2d undist_right_pix
        = right_params.GetFocalVector().cwiseProduct(right_cam_X.hnormalized());
      Eigen::Vector2d dist_right_pix;
      right_params.Convert<camera::UNDISTORTED_C, camera::DISTORTED>(undist_right_pix,
                                                                     &dist_right_pix);

      // Filter out points whose reprojection error is too big
      bool is_good = ((dist_left_ip - dist_left_pix).norm() <= reprojection_error &&
                      (dist_right_ip - dist_right_pix).norm() <= reprojection_error);

      // If any values above are Inf or NaN, is_good will be false as well
      if (!is_good) continue;
    }
    
    // Get the keypoints from the good matches
    left_vec.push_back(cv::Point2f(left_keypoints.col(left_ip_index)[0],
                                   left_keypoints.col(left_ip_index)[1]));
    right_vec.push_back(cv::Point2f(right_keypoints.col(right_ip_index)[0],
                                    right_keypoints.col(right_ip_index)[1]));

    filtered_cv_matches.push_back(cv_matches[j]);
  }

  if (left_vec.empty()) return;

  // Filter using geometry constraints
  // These may need some tweaking but works reasonably well.
  double ransacReprojThreshold = 20.0;
  cv::Mat inlier_mask;
  int maxIters = 10000;
  double confidence = 0.8;

  // affine2D works better than homography
  // cv::Mat H = cv::findHomography(left_vec, right_vec, cv::RANSAC,
  // ransacReprojThreshold, inlier_mask, maxIters, confidence);
  // TODO(oalexan1): The logic in BuildMapFindEssentialAndInliers()
  // is much better at eliminating outliers, but is likely slower.
  cv::Mat H = cv::estimateAffine2D(left_vec, right_vec, inlier_mask, cv::RANSAC,
                                   ransacReprojThreshold, maxIters, confidence);

  std::vector<InterestPoint> left_ip, right_ip;
  for (size_t j = 0; j < filtered_cv_matches.size(); j++) {
    int left_ip_index = filtered_cv_matches.at(j).queryIdx;
    int right_ip_index = filtered_cv_matches.at(j).trainIdx;

    if (inlier_mask.at<uchar>(j, 0) == 0) continue;

    cv::Mat left_desc = left_descriptors.row(left_ip_index);
    cv::Mat right_desc = right_descriptors.row(right_ip_index);

    InterestPoint left;
    left.setFromCvKeypoint(left_keypoints.col(left_ip_index), left_desc);

    InterestPoint right;
    right.setFromCvKeypoint(right_keypoints.col(right_ip_index), right_desc);

    left_ip.push_back(left);
    right_ip.push_back(right);
  }

  if (FLAGS_max_pairwise_matches >= 0 && (int)left_ip.size() > FLAGS_max_pairwise_matches)
    reduceMatches(left_ip, right_ip);
  
  // Update the shared variable using a lock
  match_mutex->lock();

  // Print the verbose message inside the lock, otherwise the text
  // may get messed up.
  if (verbose)
    std::cout << "Number of matches for pair "
              << left_image_index << ' ' << right_image_index << ": "
              << left_ip.size() << std::endl;

  *matches = std::make_pair(left_ip, right_ip);
  match_mutex->unlock();
}
  
void writeIpRecord(std::ofstream& f, InterestPoint const& p) {
  f.write(reinterpret_cast<const char*>(&(p.x)), sizeof(p.x));
  f.write(reinterpret_cast<const char*>(&(p.y)), sizeof(p.y));
  f.write(reinterpret_cast<const char*>(&(p.ix)), sizeof(p.ix));
  f.write(reinterpret_cast<const char*>(&(p.iy)), sizeof(p.iy));
  f.write(reinterpret_cast<const char*>(&(p.orientation)), sizeof(p.orientation));
  f.write(reinterpret_cast<const char*>(&(p.scale)), sizeof(p.scale));
  f.write(reinterpret_cast<const char*>(&(p.interest)), sizeof(p.interest));
  f.write(reinterpret_cast<const char*>(&(p.polarity)), sizeof(p.polarity));
  f.write(reinterpret_cast<const char*>(&(p.octave)), sizeof(p.octave));
  f.write(reinterpret_cast<const char*>(&(p.scale_lvl)), sizeof(p.scale_lvl));
  uint64_t size = p.size();
  f.write(reinterpret_cast<const char*>((&size)), sizeof(uint64));
  for (size_t i = 0; i < p.descriptor.size(); ++i)
    f.write(reinterpret_cast<const char*>(&(p.descriptor[i])), sizeof(p.descriptor[i]));
}

// Write matches to disk
void writeMatchFile(std::string match_file, std::vector<InterestPoint> const& ip1,
                    std::vector<InterestPoint> const& ip2) {
  std::ofstream f;
  f.open(match_file.c_str(), std::ios::binary | std::ios::out);
  std::vector<InterestPoint>::const_iterator iter1 = ip1.begin();
  std::vector<InterestPoint>::const_iterator iter2 = ip2.begin();
  uint64 size1 = ip1.size();
  uint64 size2 = ip2.size();
  f.write(reinterpret_cast<const char*>(&size1), sizeof(uint64));
  f.write(reinterpret_cast<const char*>(&size2), sizeof(uint64));
  for (; iter1 != ip1.end(); ++iter1) writeIpRecord(f, *iter1);
  for (; iter2 != ip2.end(); ++iter2) writeIpRecord(f, *iter2);
  f.close();
}

// TODO(oalexan1): Duplicate code
void Triangulate(bool rm_invalid_xyz, double focal_length,
                 std::vector<Eigen::Affine3d> const& cid_to_cam_t_global,
                 std::vector<Eigen::Matrix2Xd> const& cid_to_keypoint_map,
                 std::vector<std::map<int, int>> * pid_to_cid_fid,
                 std::vector<Eigen::Vector3d> * pid_to_xyz) {
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

}
  
// Triangulate rays emanating from given undistorted and centered pixels
Eigen::Vector3d TriangulatePair(double focal_length1, double focal_length2,
                                Eigen::Affine3d const& world_to_cam1,
                                Eigen::Affine3d const& world_to_cam2,
                                Eigen::Vector2d const& pix1,
                                Eigen::Vector2d const& pix2) {
  Eigen::Matrix3d k1;
  k1 << focal_length1, 0, 0, 0, focal_length1, 0, 0, 0, 1;

  Eigen::Matrix3d k2;
  k2 << focal_length2, 0, 0, 0, focal_length2, 0, 0, 0, 1;

  aspOpenMVG::Mat34 cid_to_p1, cid_to_p2;
  aspOpenMVG::P_From_KRt(k1, world_to_cam1.linear(), world_to_cam1.translation(), &cid_to_p1);
  aspOpenMVG::P_From_KRt(k2, world_to_cam2.linear(), world_to_cam2.translation(), &cid_to_p2);

  aspOpenMVG::Triangulation tri;
  tri.add(cid_to_p1, pix1);
  tri.add(cid_to_p2, pix2);

  Eigen::Vector3d solution = tri.compute();
  return solution;
}

// Triangulate n rays emanating from given undistorted and centered pixels
Eigen::Vector3d Triangulate(std::vector<double>          const& focal_length_vec,
                            std::vector<Eigen::Affine3d> const& world_to_cam_vec,
                            std::vector<Eigen::Vector2d> const& pix_vec) {
  if (focal_length_vec.size() != world_to_cam_vec.size() ||
      focal_length_vec.size() != pix_vec.size())
    LOG(FATAL) << "All inputs to Triangulate() must have the same size.";

  if (focal_length_vec.size() <= 1)
    LOG(FATAL) << "At least two rays must be passed to Triangulate().";

  aspOpenMVG::Triangulation tri;

  for (size_t it = 0; it < focal_length_vec.size(); it++) {
    Eigen::Matrix3d k;
    k << focal_length_vec[it], 0, 0, 0, focal_length_vec[it], 0, 0, 0, 1;

    aspOpenMVG::Mat34 cid_to_p;
    aspOpenMVG::P_From_KRt(k, world_to_cam_vec[it].linear(),
                        world_to_cam_vec[it].translation(),
                        &cid_to_p);

    tri.add(cid_to_p, pix_vec[it]);
  }

  Eigen::Vector3d solution = tri.compute();
  return solution;
}

// Form the match file name. Assume the input images are of the form
// cam_name/image.jpg. Use the ASP convention of the match file being
// run/run-image1__image2.match. This assumes all input images are unique.
std::string matchFileName(std::string const& match_dir,
                          std::string const& left_image, 
                          std::string const& right_image,
                          std::string const& suffix) {
  std::string left_cam_name
    = boost::filesystem::path(left_image).parent_path().stem().string();
  std::string right_cam_name
    = boost::filesystem::path(right_image).parent_path().stem().string();

  if (left_cam_name == "" || right_cam_name == "")
    LOG(FATAL) << "The image name must have the form cam_name/image. Got: "
               << left_image << " and " << right_image << ".\n";

  std::string left_stem = boost::filesystem::path(left_image).stem().string();
  std::string right_stem = boost::filesystem::path(right_image).stem().string();
  std::string match_file = match_dir + "/run-" + left_stem + "__"
    + right_stem + suffix + ".match";

  return match_file;
}

// Build tracks from pairs  
void buildTracks(aspOpenMVG::matching::PairWiseMatches const& match_map,
                 std::vector<std::map<int, int>>& pid_to_cid_fid) { // output

  pid_to_cid_fid.clear(); // wipe the output
  
  aspOpenMVG::tracks::TracksBuilder trackBuilder;
  trackBuilder.Build(match_map);  // Build:  Efficient fusion of correspondences
  trackBuilder.Filter();          // Filter: Remove tracks that have conflict
  // trackBuilder.ExportToStream(std::cout);
  // Export tracks as a map (each entry is a sequence of imageId and featureIndex):
  //  {TrackIndex => {(imageIndex, featureIndex), ... ,(imageIndex, featureIndex)}
  aspOpenMVG::tracks::STLMAPTracks map_tracks;
  trackBuilder.ExportToSTL(map_tracks);
  trackBuilder = aspOpenMVG::tracks::TracksBuilder();   // wipe it

  if (map_tracks.empty())
    LOG(FATAL) << "No tracks left after filtering. Perhaps images "
               << "are too dis-similar?\n";
  
  // Populate the filtered tracks
  size_t num_elems = map_tracks.size();
  pid_to_cid_fid.clear();
  pid_to_cid_fid.resize(num_elems);
  size_t curr_id = 0;
  for (auto itr = map_tracks.begin(); itr != map_tracks.end(); itr++) {
    for (auto itr2 = (itr->second).begin(); itr2 != (itr->second).end(); itr2++) {
      pid_to_cid_fid[curr_id][itr2->first] = itr2->second;
    }
    curr_id++;
  }

  return;
}

// The nvm file produced by Theia can have files in arbitrary order. Find the map
// which will help bring the cid in the correct order.
void findCidReorderMap(nvmData const& nvm,
                       std::vector<rig::cameraImage> const& cams,
                       // output
                       std::map<int, int> & nvm_cid_to_cams_cid) {
  
  // Wipe the output
  nvm_cid_to_cams_cid.clear();
                         
  // First find how to map each cid from nvm to cid in 'cams'.
  std::map<std::string, int> nvm_image_name_to_cid;
  for (size_t nvm_cid = 0; nvm_cid < nvm.cid_to_filename.size(); nvm_cid++) {
    nvm_image_name_to_cid[nvm.cid_to_filename[nvm_cid]] = nvm_cid;
  }
  
  std::set<std::string> cam_set;
  for (size_t cid = 0; cid < cams.size(); cid++) {
    std::string const& image_name = cams[cid].image_name;
    cam_set.insert(image_name);

    auto nvm_it = nvm_image_name_to_cid.find(image_name);
    if (nvm_it == nvm_image_name_to_cid.end()) 
      LOG(FATAL) << "Could not look up image: " << image_name
                 << " in the input nvm file. Likely the input reconstruction "
                 << "is incomplete.\n";
    int nvm_cid = nvm_it->second;
    nvm_cid_to_cams_cid[nvm_cid] = cid;
  }

  // This is an important sanity check. Warn the user when not all images
  // from the NVM file are among the ones being used.
  if (cams.size() < nvm.cid_to_filename.size()) {
    std::cout << "Warning: Some input images are not present among the images being used. "
              << "Perhaps they were removed during bracketing. Only images bracketed in time "
              << "by reference sensor images can be used. Excluded images:\n";
    for (size_t nvm_it = 0; nvm_it < nvm.cid_to_filename.size(); nvm_it++) {
      if (cam_set.find(nvm.cid_to_filename[nvm_it]) == cam_set.end()) 
        std::cout << nvm.cid_to_filename[nvm_it] << std::endl;
    }
  }
  
  return;
}

// For nvm data that has the keypoints shifted relative to the optical
// center, undo this shift when 'undo_shift' is true. So, add the optical center.
// When 'undo_shift' is false, subtract the optical center.
void shiftKeypoints(bool undo_shift, rig::RigSet const& R,
                    rig::nvmData & nvm) { // output

  for (size_t cid = 0; cid < nvm.cid_to_filename.size(); cid++) {
    std::string const& image_name = nvm.cid_to_filename[cid]; // alias

    auto it = nvm.optical_centers.find(image_name);
    if (it == nvm.optical_centers.end()) 
      LOG(FATAL) << "Could not find optical center for image: " << image_name << ".\n";
    Eigen::Vector2d keypoint_offset = it->second;
    
    int num_fid = nvm.cid_to_keypoint_map[cid].cols();
    for (int fid = 0; fid < num_fid; fid++) {
      if (undo_shift) 
        nvm.cid_to_keypoint_map.at(cid).col(fid) += keypoint_offset;
      else
        nvm.cid_to_keypoint_map.at(cid).col(fid) -= keypoint_offset;
    }
  }

  return;
}
  
// Transform nvm matches. Account for the fact that the nvm file will
// likely have the images in different order than in the 'cams'
// vector, and may have more such images, as later we may have used
// bracketing to thin them out. Also many need to add a keypoint
// offset.
// TODO(oalexan1): Integrate this with transformAppendNvm().
void transformNvm(// Inputs
                  std::vector<rig::cameraImage>   const& cams,
                  std::vector<Eigen::Vector2d>          const& keypoint_offsets,
                  nvmData                               const& nvm,
                  // Outputs
                  std::vector<std::map<int, int>> & pid_to_cid_fid,
                  std::vector<std::vector<std::pair<float, float>>> & keypoint_vec,
                  std::vector<Eigen::Vector3d> & xyz_vec) {

  // Sanity checks
  if (!keypoint_vec.empty() && keypoint_vec.size() != cams.size()) 
    LOG(FATAL) << "There must be as many sets of keypoints as images, or none at all.\n";
  if (nvm.pid_to_cid_fid.size() != nvm.pid_to_xyz.size()) 
    LOG(FATAL) << "There must be as many tracks as triangulated points for them.\n";
  
  // Wipe the outputs
  pid_to_cid_fid.clear();
  keypoint_vec.clear();
  keypoint_vec.resize(cams.size());
  xyz_vec.clear();

  // Find how to map each cid from nvm to cid in 'cams'.
  std::map<int, int> nvm_cid_to_cams_cid;
  rig::findCidReorderMap(nvm, cams,
                               nvm_cid_to_cams_cid); // output
  
  // Get new pid_to_cid_fid and keypoint_vec. Note that we ignore the triangulated
  // points in nvm.pid_to_xyz. Triangulation will be redone later.
  for (size_t pid = 0; pid < nvm.pid_to_cid_fid.size(); pid++) {

    // Ignore triangulated points that are NaN, Inf, or (0, 0, 0).
    if (!rig::isGoodTri(nvm.pid_to_xyz[pid])) 
      continue;
      
    std::map<int, int> out_cid_fid;
    for (auto cid_fid = nvm.pid_to_cid_fid[pid].begin();
         cid_fid != nvm.pid_to_cid_fid[pid].end(); cid_fid++) {

      int nvm_cid = cid_fid->first;
      int nvm_fid = cid_fid->second;
      Eigen::Vector2d keypoint = nvm.cid_to_keypoint_map.at(nvm_cid).col(nvm_fid);

      // Find cid value in 'cams' based on cid value in the nvm
      auto it = nvm_cid_to_cams_cid.find(nvm_cid);
      if (it == nvm_cid_to_cams_cid.end()) 
        continue; // this image may not have been within brackets and was thrown out
      int cid = it->second;
      
      // Add the optical center shift, if needed
      keypoint += keypoint_offsets[cid];

      int fid = keypoint_vec[cid].size(); // this is before we add the keypoint
      out_cid_fid[cid] = fid;
      // After the push back, size is fid + 1
      keypoint_vec[cid].push_back(std::make_pair(keypoint[0], keypoint[1])); 
    }

    // Keep only the tracks with at least two matches, and corresponding xyz
    if (out_cid_fid.size() > 1) {
      pid_to_cid_fid.push_back(out_cid_fid);
      xyz_vec.push_back(nvm.pid_to_xyz[pid]);
    }
    
  } // end iterating over nvm pid
}

// Helper function to find a keypoint for given iterator and update cid in the merged
// map with repetitions removed (via cid2cid).
bool updateCidFindKeypoint(std::map<int, int>::const_iterator map_it,
                           std::vector<Eigen::Matrix2Xd> const& cid_to_keypoint_map,
                           std::map<int, int>            const& cid2cid,
                           std::vector<Eigen::Vector2d>  const& keypoint_offsets,
                           int cid_shift,
                           // outputs
                           int                                & cid, 
                           std::pair<float, float>            & K) {

  // Start with the cid in the input map
  cid = map_it->first;
  int fid = map_it->second;
  Eigen::Vector2d const& P = cid_to_keypoint_map.at(cid).col(fid); // alias
  K = std::pair<float, float>(P[0], P[1]); // the keypoint
  
  // Let the cid value point to the merged map, before removing repetitions
  cid += cid_shift; // only for map B that gets appended, which starts later

  // Add the keypoint offset, using the updated cid
  K.first  += keypoint_offsets.at(cid)[0];
  K.second += keypoint_offsets.at(cid)[1];
  
  // cid gets replaced by cid2cid[cid], the value after removing repetitions
  auto it = cid2cid.find(cid);
  if (it == cid2cid.end())
    return false; // cid2cid is missing this keypoint, just ignore it

  cid = it->second;

  return true;
}

// Add keypoints from a map, appending to existing keypoints. Take into
// account how this map's cid gets transformed to the new map cid.
// Note that keypoint_offsets are applied before the cid2cid transform gets used!
// This is very error-prone!
// TODO(oalexan1): addKeypoints() can be merged into addMatchPairs().
// Just add keypoints and update the counter as they are found.
void addKeypoints(// Append from these
                  std::vector<std::map<int, int>>  const& pid_to_cid_fid,
                  std::vector<Eigen::Matrix2Xd>    const& cid_to_keypoint_map,
                  std::map<int, int>               const& cid2cid,
                  std::vector<Eigen::Vector2d>     const& keypoint_offsets,
                  int cid_shift,
                  size_t num_out_cams,
                  // Outputs, append to these 
                  std::vector<int> & keypoint_count,
                  std::vector<std::map<std::pair<float, float>, int>>
                  & merged_keypoint_map) {

  // Sanity checks
  if (num_out_cams != keypoint_count.size()) 
    LOG(FATAL) << "Keypoint count was not initialized correctly.\n";
  if (num_out_cams != merged_keypoint_map.size()) 
    LOG(FATAL) << "Keypoint map was not initialized correctly.\n";
  if (num_out_cams != rig::maxMapVal(cid2cid) + 1)
    LOG(FATAL) << "Unexpected value for the size of the output map.\n";

  for (size_t pid = 0; pid < pid_to_cid_fid.size(); pid++) {

    auto const& cid_fid = pid_to_cid_fid[pid]; // alias
    for (auto map_it = cid_fid.begin(); map_it != cid_fid.end(); map_it++) {

      int cid = -1; // will change soon
      std::pair<float, float> K;
      bool ans = updateCidFindKeypoint(map_it, cid_to_keypoint_map, cid2cid,
                                       keypoint_offsets, cid_shift,  
                                       cid, K);
      if (!ans) 
        continue;
      
      // Insert K in the keypoint map and increment the count,
      // unless it already exists
      auto & key_map = merged_keypoint_map.at(cid); // alias, will be changed
      if (key_map.find(K) != key_map.end()) 
        continue; // exists already

      key_map[K] = keypoint_count[cid];
      keypoint_count[cid]++;
    }
  }

  return;
}

// Break up each track of keypoints of length N into N pairs, (T0,
// T1), (T1, T2), ,,. (T(N-1), T0). Find their indices in the merged
// set of keypoints. Repeat this for each input map to merge and
// accumulate the pairs. Later these will be combined into new tracks
// and any repeated data will be fused. This is very tied to the
// addKeypoints() function.
// Note that keypoint_offsets are applied before the cid2cid transform gets used!
// This is very error-prone! // TODO(oalexan1): Remove that logic and the shift
// as well from this function to make it easier to understand.
// TODO(oalexan1): Incorporate here the addKeypoints() logic which
// mirrors a lot of this.
// TODO(oalexan1): Test at some point using this to rebuild and merge the tracks in
// a merged map. The concern is that it will result in conflicting tracks
// which will be removed, so this should be an option.
void addMatchPairs(// Append from these
                   std::vector<std::map<int, int>>  const& pid_to_cid_fid,
                   std::vector<Eigen::Matrix2Xd>    const& cid_to_keypoint_map,
                   std::map<int, int>               const& cid2cid,
                   std::vector<Eigen::Vector2d>     const& keypoint_offsets,
                   std::vector<std::map<std::pair<float, float>, int>>
                   const& merged_keypoint_map, 
                   int cid_shift, size_t num_out_cams,
                   aspOpenMVG::matching::PairWiseMatches & match_map) { // append here

  // Sanity checks
  if (num_out_cams != merged_keypoint_map.size()) 
    LOG(FATAL) << "Keypoint map was not initialized correctly.\n";
  if (num_out_cams != rig::maxMapVal(cid2cid) + 1)
    LOG(FATAL) << "Unexpected value for the size of the output map.\n";
  
  for (size_t pid = 0; pid < pid_to_cid_fid.size(); pid++) {
    
    auto const& cid_fid = pid_to_cid_fid[pid]; // alias
    for (auto map_it1 = cid_fid.begin(); map_it1 != cid_fid.end(); map_it1++) {
      
      // First element in the pair
      int cid1 = -1; // will change soon
      std::pair<float, float> K1;
      bool ans = updateCidFindKeypoint(map_it1, cid_to_keypoint_map, cid2cid,
                                       keypoint_offsets, cid_shift,    
                                       cid1, K1); // output
      if (!ans)
        continue;
      
      // Find the second element in the pair. If at the end of the track,
      // and the track length is more than 2, use the earliest element.
      auto map_it2 = map_it1;
      map_it2++;
      if (map_it2 == cid_fid.end()) {
        
        if (cid_fid.size() <= 2) 
          continue; // Already added (T0, T1), no need to add (T1, T0).
      
        map_it2 = cid_fid.begin();
      }
      int cid2 = -1; // will change soon
      std::pair<float, float> K2;
      ans = updateCidFindKeypoint(map_it2, cid_to_keypoint_map, cid2cid,
                                  keypoint_offsets, cid_shift,  
                                  cid2, K2); // output
      if (!ans) 
        continue;
      
      // No point in adding a match from an image to itself
      if (cid1 == cid2) 
        continue;

      // Find the fid indices
      auto it1 = merged_keypoint_map.at(cid1).find(K1);
      auto it2 = merged_keypoint_map.at(cid2).find(K2);
      if (it1 == merged_keypoint_map.at(cid1).end() ||
          it2 == merged_keypoint_map.at(cid2).end())
        LOG(FATAL) << "Could not look up a keypoint. That is unexpected.\n";
      int fid1 = it1->second;
      int fid2 = it2->second;
      
      // Append the pair
      std::vector<aspOpenMVG::matching::IndMatch> & mvg_matches
        = match_map[std::make_pair(cid1, cid2)]; // alias
      mvg_matches.push_back(aspOpenMVG::matching::IndMatch(fid1, fid2));
    } // end iterating over the track
  } 
  return;
}

// Given some tracks read from nvm from disk, append the ones from
// nvm. Some remapping is needed.  given that 'fid' values already
// exist for the given tracks and that the nvm read from disk
// may have the images in different order. New keypoints are recorded
// with the help of fid_count and merged_keypoint_map.
// Note that keypoint_offsets are applied before the cid2cid transform gets used!
// This is very error-prone!
// TODO(oalexan1): cid_shift and keypoint_offsets should be applied outside
// this function as they make it hard to understand.
void transformAppendNvm(// Append from these
                        std::vector<std::map<int, int>>  const& nvm_pid_to_cid_fid,
                        std::vector<Eigen::Matrix2Xd>    const& nvm_cid_to_keypoint_map,
                        std::map<int, int>               const& cid2cid,
                        std::vector<Eigen::Vector2d>     const& keypoint_offsets,
                        int cid_shift,
                        size_t num_out_cams,
                        // Outputs, append to these 
                        std::vector<int> & fid_count,
                        std::vector<std::map<std::pair<float, float>, int>>
                        & merged_keypoint_map,
                        std::vector<std::map<int, int>> & pid_to_cid_fid) {

  // Sanity checks
  if (num_out_cams != fid_count.size()) 
    LOG(FATAL) << "Keypoint count was not initialized correctly.\n";
  if (num_out_cams != merged_keypoint_map.size()) 
    LOG(FATAL) << "Keypoint map was not initialized correctly.\n";
  if (num_out_cams < rig::maxMapVal(cid2cid) + 1)
    LOG(FATAL) << "Unexpected value for the size of the output map.\n";

  for (size_t pid = 0; pid < nvm_pid_to_cid_fid.size(); pid++) {

    std::map<int, int> out_cid_fid;
    auto const& cid_fid = nvm_pid_to_cid_fid[pid]; // alias
    for (auto map_it = cid_fid.begin(); map_it != cid_fid.end(); map_it++) {

      int cid = -1; // will change soon
      std::pair<float, float> K;
      bool ans = updateCidFindKeypoint(map_it, nvm_cid_to_keypoint_map, cid2cid,
                                       keypoint_offsets, cid_shift,  
                                       cid, K);
      if (!ans) 
        continue; // could not lookup cid in cid2cid
      
      // TODO(oalexan1): Use below the function findFid(), with testing.
      // Insert K in the keypoint map and increment the count,
      // unless it already exists. In either case get the fid.
      auto key_it = merged_keypoint_map.at(cid).find(K);
      int fid = -1;
      if (key_it != merged_keypoint_map.at(cid).end()) {
        fid = key_it->second;
      } else {
        // fid is the last count, then increment the count
        fid = fid_count[cid];
        merged_keypoint_map.at(cid)[K] = fid_count[cid];
        fid_count[cid]++;
      }
      
      // Create the track
      out_cid_fid[cid] = fid;
    }
    
    // Append the transformed track
    if (out_cid_fid.size() > 1)
      pid_to_cid_fid.push_back(out_cid_fid);
  }
  
  return;
}
  
// Helper function to add an ip to the keypoint map if not there.
// In either case find its fid, which is the keypoint map value.
void findFid(std::pair<float, float> const & ip,
             int cid,
             // Outputs
             std::vector<std::map<std::pair<float, float>, int>> & keypoint_map,
             std::vector<int> & fid_count,
             int & fid) {
  
  fid = -1; // initialize the output
  
  auto it = keypoint_map[cid].find(ip);
  if (it != keypoint_map[cid].end()) {
    fid = it->second;
  } else {
    keypoint_map[cid][ip] = fid_count[cid];
    fid = fid_count[cid];
    fid_count[cid]++;
  }
  return;
}

// Remove duplicate tracks. There can still be two tracks with one contained
// in the other or otherwise having shared elements. 
void rmDuplicateTracks(std::vector<std::map<int, int>> & pid_to_cid_fid) {

  int num_tracks = pid_to_cid_fid.size();
  // vector to set, and back
  std::set<std::map<int, int>> track_set(pid_to_cid_fid.begin(), pid_to_cid_fid.end());
  pid_to_cid_fid.assign(track_set.begin(), track_set.end());

  int diff = num_tracks - int(pid_to_cid_fid.size());
  std::cout << "Removed " << diff << " duplicate tracks ("
            << 100.0 * double(diff)/double(num_tracks) << "%)\n";
}
  
void detectMatchFeatures(// Inputs
                         std::vector<rig::cameraImage> const& cams,
                         std::vector<camera::CameraParameters> const& cam_params,
                         std::string const& out_dir, bool save_matches,
                         bool filter_matches_using_cams,
                         std::vector<Eigen::Affine3d> const& world_to_cam,
                         int num_overlaps,
                         std::vector<std::pair<int, int>> const& input_image_pairs, 
                         int initial_max_reprojection_error, int num_match_threads,
                         bool read_nvm_no_shift, bool no_nvm_matches, bool verbose,
                         // Outputs
                         std::vector<std::vector<std::pair<float, float>>>& keypoint_vec,
                         std::vector<std::map<int, int>>& pid_to_cid_fid,
                         std::vector<Eigen::Vector3d> & xyz_vec,
                         rig::nvmData & nvm) {
  // Wipe the outputs
  keypoint_vec.clear();
  pid_to_cid_fid.clear();
  xyz_vec.clear();

  if (!no_nvm_matches) 
    std::cout << "Tracks read from disk: " << nvm.pid_to_cid_fid.size() << std::endl;
  
  // Sanity check: the offsets from the nvm must agree with the optical centers
  // The offsets may be empty if the cameras were read from a list
  if (!read_nvm_no_shift && !no_nvm_matches && !nvm.optical_centers.empty()) {
    for (size_t cid = 0; cid < cams.size(); cid++) {
      Eigen::Vector2d offset = cam_params[cams[cid].camera_type].GetOpticalOffset();
      auto it = nvm.optical_centers.find(cams[cid].image_name);
      if (it == nvm.optical_centers.end())
        continue; // offsets may be missing when extra images are added
      Eigen::Vector2d nvm_offset = it->second;
      if ((offset - nvm_offset).norm() > 1e-8)
        LOG(FATAL) << "Optical centers read from the nvm file do not agree with the "
                   << "ones from the rig configuration for image:  " 
                   << cams[cid].image_name << ".\n";
    }
  }
  
  size_t num_images = cams.size();
  if (num_overlaps == 0 && !no_nvm_matches) {
    // Add the optical center shift, if needed
    // Here keypoint_offsets are for the cid in the order in cams, not in nvm.
    // TODO(oalexan1): This is confusing and needs to be dealt with at some point.
    std::vector<Eigen::Vector2d> keypoint_offsets(num_images);
    for (int cid = 0; cid < num_images; cid++) {
      if (read_nvm_no_shift)
        keypoint_offsets[cid] = Eigen::Vector2d(0, 0);
      else
        keypoint_offsets[cid] = cam_params[cams[cid].camera_type].GetOpticalOffset();
    }
    
    // If we do not need to create new matches, just reorganize the ones read in
    // and quit.
    rig::transformNvm(// Inputs
                            cams, keypoint_offsets, nvm,  
                            // Outputs
                            pid_to_cid_fid, keypoint_vec, xyz_vec);
    nvm = rig::nvmData(); // no longer needed
    return;
  }

#if 1
  // TODO(oalexan1): This should be a function called detectMatchFeatures().
  // The parent function should be detectMatchFeaturesAppendNvm().
  
  // Detect features using multiple threads. Too many threads may result
  // in high memory usage.
  std::ostringstream oss;
  oss << num_match_threads;
  std::string num_threads = oss.str();
  google::SetCommandLineOption("num_threads", num_threads.c_str());
  if (!gflags::GetCommandLineOption("num_threads", &num_threads))
    LOG(FATAL) << "Failed to get the value of --num_threads in Astrobee software.\n";
  std::cout << "Using " << num_threads << " threads for feature detection/matching.\n";

  std::cout << "Detecting features." << std::endl;

  std::vector<cv::Mat> cid_to_descriptor_map;
  std::vector<Eigen::Matrix2Xd> cid_to_keypoint_map;
  cid_to_descriptor_map.resize(num_images);
  cid_to_keypoint_map.resize(num_images);
  {
    // Make the thread pool go out of scope when not needed to not use up memory
    rig::ThreadPool thread_pool;
    for (size_t it = 0; it < num_images; it++) {
      thread_pool.AddTask
        (&rig::detectFeatures,    // multi-threaded  // NOLINT
         // rig::detectFeatures(  // single-threaded // NOLINT
         cams[it].image, verbose, &cid_to_descriptor_map[it], &cid_to_keypoint_map[it]);
    }
    thread_pool.Join();
  }

  // Find which image pairs to match
  std::vector<std::pair<int, int>> image_pairs;
  if (!input_image_pairs.empty()) {
    // Use provided image pairs instead of num_overlaps
    image_pairs = input_image_pairs;
  } else {
    for (size_t it1 = 0; it1 < num_images; it1++) {
      int end = std::min(num_images, it1 + num_overlaps + 1);
      for (size_t it2 = it1 + 1; it2 < end; it2++) {
        image_pairs.push_back(std::make_pair(it1, it2));
      }
    }
  }
  
  MATCH_MAP matches;
  { // Deallocate local variables as soon as they are not needed
    std::cout << "Matching features." << std::endl;
    rig::ThreadPool thread_pool;
    std::mutex match_mutex;
    for (size_t pair_it = 0; pair_it < image_pairs.size(); pair_it++) {
      auto pair = image_pairs[pair_it];
      int left_image_it = pair.first, right_image_it = pair.second;
      thread_pool.AddTask
        (&rig::matchFeaturesWithCams,   // multi-threaded  // NOLINT
         // rig::matchFeaturesWithCams( // single-threaded // NOLINT
         &match_mutex, left_image_it, right_image_it,
         cam_params[cams[left_image_it].camera_type],
         cam_params[cams[right_image_it].camera_type],
         filter_matches_using_cams,
         world_to_cam[left_image_it], world_to_cam[right_image_it],
         initial_max_reprojection_error,
         cid_to_descriptor_map[left_image_it], cid_to_descriptor_map[right_image_it],
         cid_to_keypoint_map[left_image_it], cid_to_keypoint_map[right_image_it], verbose,
         &matches[pair]);
    }
    thread_pool.Join();
  }
  cid_to_keypoint_map = std::vector<Eigen::Matrix2Xd>(); // wipe, no longer needed
  cid_to_descriptor_map = std::vector<cv::Mat>();  // Wipe, no longer needed

  if (save_matches) {
    if (out_dir.empty())
      LOG(FATAL) << "Cannot save matches if no output directory was provided.\n";

    std::string match_dir = out_dir + "/matches";
    rig::createDir(match_dir);

    for (auto it = matches.begin(); it != matches.end(); it++) {
      std::pair<int, int> cid_pair = it->first;
      rig::MATCH_PAIR const& match_pair = it->second;

      int left_cid = cid_pair.first;
      int right_cid = cid_pair.second;
      std::string const& left_image = cams[left_cid].image_name; // alias
      std::string const& right_image = cams[right_cid].image_name; // alias

      std::string suffix = "";
      std::string match_file = matchFileName(match_dir, left_image, right_image, suffix);
      std::cout << "Writing: " << left_image << " " << right_image << " "
                << match_file << std::endl;
      rig::writeMatchFile(match_file, match_pair.first, match_pair.second);
    }
  }
  
  // Collect all keypoints in keypoint_map, and put the fid (indices of keypoints) in
  // match_map. It will be used to find the tracks.
  std::vector<std::map<std::pair<float, float>, int>> keypoint_map(num_images);
  std::vector<int> fid_count(num_images, 0); // track the fid of keypoints when adding them
  aspOpenMVG::matching::PairWiseMatches match_map;
  for (auto it = matches.begin(); it != matches.end(); it++) {
    std::pair<int, int> const& cid_pair = it->first;     // alias

    int left_cid = cid_pair.first;
    int right_cid = cid_pair.second;

    rig::MATCH_PAIR const& match_pair = it->second;  // alias
    std::vector<rig::InterestPoint> const& left_ip_vec = match_pair.first;
    std::vector<rig::InterestPoint> const& right_ip_vec = match_pair.second;

    for (size_t ip_it = 0; ip_it < left_ip_vec.size(); ip_it++) {
      auto left_ip  = std::make_pair(left_ip_vec[ip_it].x,  left_ip_vec[ip_it].y);
      auto right_ip = std::make_pair(right_ip_vec[ip_it].x, right_ip_vec[ip_it].y);

      // Add an ip to the keypoint map if not there. In either case find its fid.
      int left_fid = -1, right_fid = -1;
      findFid(left_ip, left_cid,
              keypoint_map, fid_count, left_fid); // may change
      findFid(right_ip, right_cid,
              keypoint_map, fid_count, right_fid); // may change

      match_map[cid_pair].push_back(aspOpenMVG::matching::IndMatch(left_fid, right_fid));
    }
  }
  matches.clear(); matches = MATCH_MAP(); // mo longer needed

  // If feature A in image I matches feather B in image J, which
  // matches feature C in image K, then (A, B, C) belong together in
  // a track, and will have a single triangulated xyz. Build such a track.
  buildTracks(match_map, pid_to_cid_fid);
  std::cout << "Tracks obtained after matching: " << pid_to_cid_fid.size() << std::endl;
  match_map = aspOpenMVG::matching::PairWiseMatches();  // wipe this, no longer needed

#endif // this should be end of function detectMatchFeatures()

  // Append tracks being read from nvm. This turned out to work better than to try
  // to merge these tracks with the ones from the pairwise matching above, as the latter
  // would make many good tracks disappear.
  if (!no_nvm_matches) {
    // Find how to map each cid from nvm to cid in 'cams'.
    std::map<int, int> nvm_cid_to_cams_cid;
    rig::findCidReorderMap(nvm, cams,
                                 nvm_cid_to_cams_cid); // output

    // Add the optical center shift, if needed.
    std::vector<Eigen::Vector2d> keypoint_offsets(nvm.cid_to_filename.size());
    for (size_t nvm_cid = 0; nvm_cid < nvm.cid_to_filename.size(); nvm_cid++) {

      // This is a bugfix. The transformAppendNvm() function below
      // expects that keypoint_offsets be indexed by nvm_cid and not
      // reordered cid in cams.
      // TODO(oalexan1): This is confusing. The keypoint offsets in transformAppendNvm()
      // better be applied after nvm_cid_to_cams_cid is applied. Also merge
      // with function transformNvm() and remove the latter.
      auto it = nvm_cid_to_cams_cid.find(nvm_cid);
      if (it == nvm_cid_to_cams_cid.end()) {
        // This nvm_cid does not exist in cams. A warning about that
        // was given in findCidReorderMap(). Give a value to this
        // but it won't be used.
        keypoint_offsets[nvm_cid] = Eigen::Vector2d(0, 0);
        continue;
      }
      
      int cams_cid = it->second;
      if (read_nvm_no_shift)
        keypoint_offsets[nvm_cid] = Eigen::Vector2d(0, 0);
      else
        keypoint_offsets[nvm_cid]
          = cam_params[cams[cams_cid].camera_type].GetOpticalOffset();
    }


    // Add the nvm matches. Unlike the transformNvm() function above,
    // the keypoints are shared with the newly created matches. Later
    // that will be used to remove duplicates.
    int cid_shift = 0; // part of the API
    rig::transformAppendNvm(nvm.pid_to_cid_fid, nvm.cid_to_keypoint_map,  
                                  nvm_cid_to_cams_cid,
                                  keypoint_offsets, cid_shift, num_images,
                                  fid_count, keypoint_map, pid_to_cid_fid); // append
  }
  
  // Create keypoint_vec from keypoint_map. That just reorganizes the data
  // to the format expected later.
  // TODO(oalexan1): This must be a function called keypointMapToVec().
  // TODO(oalexan1): Use std::vector<Eigen::Matrix2Xd> as in nvm instead.
  keypoint_vec.clear();
  keypoint_vec.resize(num_images);
  for (size_t cid = 0; cid < num_images; cid++) {
    auto const& map = keypoint_map[cid]; // alias
    keypoint_vec[cid].resize(map.size());
    for (auto ip_it = map.begin(); ip_it != map.end(); ip_it++) {
      auto const& ip = ip_it->first;  // alias
      int fid = ip_it->second;
      keypoint_vec[cid][fid] = ip;
    }
  }
  
  // De-allocate data not needed anymore
  nvm = rig::nvmData(); // no longer needed
  keypoint_map.clear(); keypoint_map.shrink_to_fit();

  // Remove duplicate tracks. Those can happen since additional tracks being
  // created in matching can duplicate tracks read from disk. Ideally also
  // shorter tracks contained in longer tracks should be removed, and tracks
  // that can be merged without conflicts should be merged.
  rmDuplicateTracks(pid_to_cid_fid);

  return;
}

void multiViewTriangulation(// Inputs
                            std::vector<camera::CameraParameters>   const& cam_params,
                            std::vector<rig::cameraImage>     const& cams,
                            std::vector<Eigen::Affine3d>            const& world_to_cam,
                            std::vector<std::map<int, int>>         const& pid_to_cid_fid,
                            std::vector<std::vector<std::pair<float, float>>>
                            const& keypoint_vec,
                            // Outputs
                            std::vector<std::map<int, std::map<int, int>>>&
                            pid_cid_fid_inlier,
                            std::vector<Eigen::Vector3d>& xyz_vec) {

  if (cams.size() != world_to_cam.size()) 
    LOG(FATAL) << "Expecting as many images as cameras.\n";
  
  xyz_vec.clear();
  xyz_vec.resize(pid_to_cid_fid.size());
  for (size_t pid = 0; pid < pid_to_cid_fid.size(); pid++)
    xyz_vec[pid] = Eigen::Vector3d(0, 0, 0); // initialize to 0
  
  for (size_t pid = 0; pid < pid_to_cid_fid.size(); pid++) {
    std::vector<double> focal_length_vec;
    std::vector<Eigen::Affine3d> world_to_cam_aff_vec;
    std::vector<Eigen::Vector2d> pix_vec;

    for (auto cid_fid = pid_to_cid_fid[pid].begin(); cid_fid != pid_to_cid_fid[pid].end();
         cid_fid++) {
      int cid = cid_fid->first;
      int fid = cid_fid->second;

      // Triangulate inliers only
      if (!rig::getMapValue(pid_cid_fid_inlier, pid, cid, fid))
        continue;

      Eigen::Vector2d dist_ip(keypoint_vec[cid][fid].first, keypoint_vec[cid][fid].second);
      Eigen::Vector2d undist_ip;
      cam_params[cams[cid].camera_type].Convert<camera::DISTORTED, camera::UNDISTORTED_C>
        (dist_ip, &undist_ip);

      focal_length_vec.push_back(cam_params[cams[cid].camera_type].GetFocalLength());
      world_to_cam_aff_vec.push_back(world_to_cam[cid]);
      pix_vec.push_back(undist_ip);
    }

    if (pix_vec.size() < 2) {
      // If after outlier filtering less than two rays are left, can't triangulate.
      // Must set all features for this pid to outliers.
      for (auto cid_fid = pid_to_cid_fid[pid].begin();
           cid_fid != pid_to_cid_fid[pid].end();
           cid_fid++) {
        int cid = cid_fid->first;
        int fid = cid_fid->second;
        rig::setMapValue(pid_cid_fid_inlier, pid, cid, fid, 0);
      }

      // Nothing else to do
      continue;
    }

    // Triangulate n rays emanating from given undistorted and centered pixels
    xyz_vec[pid] = rig::Triangulate(focal_length_vec, world_to_cam_aff_vec,
                                          pix_vec);

    bool bad_xyz = false;
    for (int c = 0; c < xyz_vec[pid].size(); c++) {
      if (std::isinf(xyz_vec[pid][c]) || std::isnan(xyz_vec[pid][c])) 
        bad_xyz = true;
    }
    if (bad_xyz) {
      // if triangulation failed, must set all features for this pid to outliers.
      for (auto cid_fid = pid_to_cid_fid[pid].begin(); cid_fid != pid_to_cid_fid[pid].end();
           cid_fid++) {
        int cid = cid_fid->first;
        int fid = cid_fid->second;
        rig::setMapValue(pid_cid_fid_inlier, pid, cid, fid, 0);
      }
    }
    
  } // end iterating over triangulated points
  
  return;
}
  
// Given all the merged and filtered tracks in pid_cid_fid, for each
// image pair cid1 and cid2 with cid1 < cid2 < cid1 + num_overlaps + 1,
// save the matches of this pair which occur in the set of tracks.
void saveInlierMatchPairs(// Inputs
                           std::vector<rig::cameraImage> const& cams,
                           int num_overlaps,
                           std::vector<std::map<int, int>> const& pid_to_cid_fid,
                           std::vector<std::vector<std::pair<float, float>>>
                           const& keypoint_vec,
                           std::vector<std::map<int, std::map<int, int>>>
                           const& pid_cid_fid_inlier,
                           std::string const& out_dir) {

  MATCH_MAP matches;

  for (size_t pid = 0; pid < pid_to_cid_fid.size(); pid++) {
    for (auto cid_fid1 = pid_to_cid_fid[pid].begin();
         cid_fid1 != pid_to_cid_fid[pid].end(); cid_fid1++) {
      int cid1 = cid_fid1->first;
      int fid1 = cid_fid1->second;

      for (auto cid_fid2 = pid_to_cid_fid[pid].begin();
           cid_fid2 != pid_to_cid_fid[pid].end(); cid_fid2++) {
        int cid2 = cid_fid2->first;
        int fid2 = cid_fid2->second;

        // When num_overlaps == 0, we save only matches read from nvm rather
        // ones made wen this tool was run.
        bool is_good = (cid1 < cid2 && (num_overlaps == 0 || cid2 < cid1 + num_overlaps + 1));
        if (!is_good)
          continue;

        // Consider inliers only
        if (!rig::getMapValue(pid_cid_fid_inlier, pid, cid1, fid1) ||
            !rig::getMapValue(pid_cid_fid_inlier, pid, cid2, fid2))
          continue;

        auto cid_pair = std::make_pair(cid1, cid2);

        InterestPoint ip1(keypoint_vec[cid1][fid1].first, keypoint_vec[cid1][fid1].second);
        InterestPoint ip2(keypoint_vec[cid2][fid2].first, keypoint_vec[cid2][fid2].second);

        matches[cid_pair].first.push_back(ip1);
        matches[cid_pair].second.push_back(ip2);
      }
    }
  }  // End iterations over pid

  for (auto it = matches.begin(); it != matches.end(); it++) {
    auto & cid_pair = it->first;
    rig::MATCH_PAIR const& match_pair = it->second;

    int left_cid = cid_pair.first;
    int right_cid = cid_pair.second;

    std::string match_dir = out_dir + "/matches";
    rig::createDir(match_dir);

    std::string suffix = "";
    std::string match_file = rig::matchFileName(match_dir,
                                                      cams[left_cid].image_name,
                                                      cams[right_cid].image_name,
                                                      suffix);

    std::cout << "Writing: " << match_file << std::endl;
    rig::writeMatchFile(match_file, match_pair.first, match_pair.second);
  }
  
  // The image names (without directory) must be unique, or else bundle
  // adjustment will fail.
  std::set<std::string> base_names;
  for (size_t it = 0; it < cams.size(); it++) {
    std::string base_name = fs::path(cams[it].image_name).filename().string(); 
    if (base_names.find(base_name) != base_names.end())
      LOG(FATAL) << "Non-unique image name (without directory): " << base_name 
                 << ". It will not be possible to use these matches with bundle_adjust.";
    base_names.insert(base_name);
  }
  
}

// TODO(oalexan1): All the logic below has little to do with interest
// points, and should be distributed to some other existing or new files.
  
// TODO(oalexan1): Move this to transform_utils.
// Apply a given transform to the given set of cameras.
// We assume that the transform is of the form
// T(x) = scale * rotation * x + translation
void TransformCameras(Eigen::Affine3d const& T,
                      std::vector<Eigen::Affine3d> &world_to_cam) {
  
  // Inverse of rotation component
  double scale = pow(T.linear().determinant(), 1.0 / 3.0);
  Eigen::MatrixXd Tinv = (T.linear()/scale).inverse();

  for (size_t cid = 0; cid < world_to_cam.size(); cid++) {
    world_to_cam[cid].linear() = world_to_cam[cid].linear()*Tinv;
    world_to_cam[cid].translation() = scale*world_to_cam[cid].translation() -
      world_to_cam[cid].linear()*T.translation();
  }
}

// TODO(oalexan1): Move this to transform_utils.
// Apply same transform as above to points
void TransformPoints(Eigen::Affine3d const& T, std::vector<Eigen::Vector3d> *xyz) {
  for (size_t pid = 0; pid < (*xyz).size(); pid++)
    (*xyz)[pid] = T * (*xyz)[pid];
}

// TODO(oalexan1): Move this to transform_utils.
// Apply a given transform to the specified xyz points, and adjust accordingly the cameras
// for consistency. We assume that the transform is of the form
// A(x) = scale * rotation * x + translation
void TransformCamerasAndPoints(Eigen::Affine3d const& A,
                               std::vector<Eigen::Affine3d> *cid_to_cam_t,
                               std::vector<Eigen::Vector3d> *xyz) {
  TransformCameras(A, *cid_to_cam_t);
  TransformPoints(A, xyz);
}
  
// TODO(oalexan1): Move this to transform_utils.
// Apply a registration transform to a rig. The only thing that
// changes is scale, as the rig transforms are between coordinate
// systems of various cameras.
void TransformRig(Eigen::Affine3d const& T, std::vector<Eigen::Affine3d> & ref_to_cam_trans) {
  double scale = pow(T.linear().determinant(), 1.0 / 3.0);
  for (size_t cam_type = 0; cam_type < ref_to_cam_trans.size(); cam_type++) 
    ref_to_cam_trans[cam_type].translation() *= scale;
}

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

// TODO(oalexan1): Move this to transform_utils.
// Find the 3D transform from an abstract coordinate system to the
// world, given control points (pixel matches) and corresponding 3D
// measurements. It is assumed all images are acquired with the same camera.
Eigen::Affine3d registrationTransform(std::string                  const& hugin_file,
                                      std::string                  const& xyz_file,
                                      camera::CameraParameters     const& cam_params,
                                      std::vector<std::string>     const& cid_to_filename,
                                      std::vector<Eigen::Affine3d> const& world_to_cam_trans) { 
  
  // Get the interest points in the images, and their positions in
  // the world coordinate system, as supplied by a user.
  // Parse and concatenate that information from multiple files.
  std::vector<std::string> images;
  Eigen::MatrixXd user_ip;
  Eigen::MatrixXd user_xyz;
  
  ParseHuginControlPoints(hugin_file, &images, &user_ip);
  ParseXYZ(xyz_file, &user_xyz);

  int num_points = user_ip.cols();
  if (num_points != user_xyz.cols())
    LOG(FATAL) << "Could not parse an equal number of control "
               << "points and xyz coordinates. Their numbers are "
               << num_points << " vs " << user_xyz.cols() << ".\n";


  std::map<std::string, int> filename_to_cid;
  for (size_t cid = 0; cid < cid_to_filename.size(); cid++)
    filename_to_cid[cid_to_filename[cid]] = cid;

  // Wipe images that are missing from the map
  std::map<int, int> cid2cid;
  int good_cid = 0;
  for (size_t cid = 0; cid < images.size(); cid++) {
    std::string image = images[cid];
    if (filename_to_cid.find(image) == filename_to_cid.end()) {
      LOG(WARNING) << "Will ignore image missing from map: " << image;
      continue;
    }
    cid2cid[cid] = good_cid;
    images[good_cid] = images[cid];
    good_cid++;
  }
  images.resize(good_cid);

  // Remove points corresponding to images missing from map
  int good_pid = 0;
  for (int pid = 0; pid < num_points; pid++) {
    int id1 = user_ip(0, pid);
    int id2 = user_ip(1, pid);
    if (cid2cid.find(id1) == cid2cid.end() || cid2cid.find(id2) == cid2cid.end()) {
      continue;
    }
    user_ip.col(good_pid) = user_ip.col(pid);
    user_xyz.col(good_pid) = user_xyz.col(pid);
    good_pid++;
  }
  user_ip.conservativeResize(Eigen::NoChange_t(), good_pid);
  user_xyz.conservativeResize(Eigen::NoChange_t(), good_pid);
  num_points = good_pid;
  for (int pid = 0; pid < num_points; pid++) {
    int id1 = user_ip(0, pid);
    int id2 = user_ip(1, pid);
    if (cid2cid.find(id1) == cid2cid.end() || cid2cid.find(id2) == cid2cid.end())
      LOG(FATAL) << "Book-keeping failure in registration.";
    user_ip(0, pid) = cid2cid[id1];
    user_ip(1, pid) = cid2cid[id2];
  }


  if (num_points < 3) 
    LOG(FATAL) << "Must have at least 3 points to apply registration. Got: "
               << num_points << "\n";
  
  // Iterate over the control points in the hugin file. Copy the
  // control points to the list of user keypoints, and create the
  // corresponding user_pid_to_cid_fid.
  std::vector<Eigen::Matrix2Xd> user_cid_to_keypoint_map;
  std::vector<std::map<int, int> > user_pid_to_cid_fid;
  user_cid_to_keypoint_map.resize(cid_to_filename.size());
  user_pid_to_cid_fid.resize(num_points);
  for (int pid = 0; pid < num_points; pid++) {
    // Left and right image indices
    int id1 = user_ip(0, pid);
    int id2 = user_ip(1, pid);

    // Sanity check
    if (id1 < 0 || id2 < 0 ||
        id1 >= static_cast<int>(images.size()) ||
        id2 >= static_cast<int>(images.size()) )
      LOG(FATAL) << "Invalid image indices in the hugin file: " << id1 << ' ' << id2;

    // Find the corresponding indices in the map where these keypoints will go to
    if (filename_to_cid.find(images[id1]) == filename_to_cid.end())
      LOG(FATAL) << "File missing from map: " << images[id1];
    if (filename_to_cid.find(images[id2]) == filename_to_cid.end())
      LOG(FATAL) << "File missing from map: " << images[id2];
    int cid1 = filename_to_cid[images[id1]];
    int cid2 = filename_to_cid[images[id2]];

    // Append to the keypoints for cid1
    Eigen::Matrix<double, 2, -1> &M1 = user_cid_to_keypoint_map[cid1];  // alias
    Eigen::Matrix<double, 2, -1> N1(M1.rows(), M1.cols()+1);
    N1 << M1, user_ip.block(2, pid, 2, 1);  // left image pixel x and pixel y
    M1.swap(N1);

    // Append to the keypoints for cid2
    Eigen::Matrix<double, 2, -1> &M2 = user_cid_to_keypoint_map[cid2];  // alias
    Eigen::Matrix<double, 2, -1> N2(M2.rows(), M2.cols()+1);
    N2 << M2, user_ip.block(4, pid, 2, 1);  // right image pixel x and pixel y
    M2.swap(N2);

    // The corresponding user_pid_to_cid_fid
    user_pid_to_cid_fid[pid][cid1] = user_cid_to_keypoint_map[cid1].cols()-1;
    user_pid_to_cid_fid[pid][cid2] = user_cid_to_keypoint_map[cid2].cols()-1;
  }

  // Apply undistortion
  Eigen::Vector2d output;
  for (size_t cid = 0; cid < user_cid_to_keypoint_map.size(); cid++) {
    for (int i = 0; i < user_cid_to_keypoint_map[cid].cols(); i++) {
      cam_params.Convert<camera::DISTORTED, camera::UNDISTORTED_C>
        (user_cid_to_keypoint_map[cid].col(i), &output);
      user_cid_to_keypoint_map[cid].col(i) = output;
    }
  }

  // Triangulate to find the coordinates of the current points
  // in the virtual coordinate system
  std::vector<Eigen::Vector3d> unreg_pid_to_xyz;
  bool rm_invalid_xyz = false;  // there should be nothing to remove hopefully
  Triangulate(rm_invalid_xyz,
              cam_params.GetFocalLength(),
              world_to_cam_trans,
              user_cid_to_keypoint_map,
              &user_pid_to_cid_fid,
              &unreg_pid_to_xyz);

  double mean_err = 0;
  for (int i = 0; i < user_xyz.cols(); i++) {
    Eigen::Vector3d a = unreg_pid_to_xyz[i];
    Eigen::Vector3d b = user_xyz.col(i);
    mean_err += (a-b).norm();
  }
  mean_err /= user_xyz.cols();
  std::cout << "Mean absolute error before registration: " << mean_err << " meters" << std::endl;
  std::cout << "Un-transformed computed xyz -- measured xyz -- error diff -- error norm (meters)"
            << std::endl;

  for (int i = 0; i < user_xyz.cols(); i++) {
    Eigen::Vector3d a = unreg_pid_to_xyz[i];
    Eigen::Vector3d b = user_xyz.col(i);
    std::cout << print_vec(a) << " -- "
              << print_vec(b) << " -- "
              << print_vec(a-b) << " -- "
              << print_vec((a - b).norm())
              << std::endl;
  }

  // Find the transform from the computed map coordinate system
  // to the world coordinate system.
  int np = unreg_pid_to_xyz.size();
  Eigen::Matrix3Xd in(3, np);
  for (int i = 0; i < np; i++)
    in.col(i) = unreg_pid_to_xyz[i];

  Eigen::Affine3d registration_trans;  
  Find3DAffineTransform(in, user_xyz, &registration_trans);

  mean_err = 0.0;
  for (int i = 0; i < user_xyz.cols(); i++)
    mean_err += (registration_trans*in.col(i) - user_xyz.col(i)).norm();
  mean_err /= user_xyz.cols();

  // We don't use LOG(INFO) below, as it does not play well with
  // Eigen.
  double scale = pow(registration_trans.linear().determinant(), 1.0 / 3.0);
  std::cout << "Registration transform (to measured world coordinates)." << std::endl;
  std::cout << "Rotation:\n" << registration_trans.linear() / scale << std::endl;
  std::cout << "Scale:\n" << scale << std::endl;
  std::cout << "Translation:\n" << registration_trans.translation().transpose()
            << std::endl;

  std::cout << "Mean absolute error after registration: "
            << mean_err << " meters" << std::endl;

  std::cout << "Transformed computed xyz -- measured xyz -- "
            << "error diff - error norm (meters)" << std::endl;
  for (int i = 0; i < user_xyz.cols(); i++) {
    Eigen::Vector3d a = registration_trans*in.col(i);
    Eigen::Vector3d b = user_xyz.col(i);
    int id1 = user_ip(0, i);
    int id2 = user_ip(1, i);

    std::cout << print_vec(a) << " -- "
              << print_vec(b) << " -- "
              << print_vec(a - b) << " -- "
              << print_vec((a - b).norm()) << " -- "
              << images[id1] << ' '
              << images[id2] << std::endl;
  }

  return registration_trans;
}
  
// Write an image with 3 floats per pixel. OpenCV's imwrite() cannot do that.
void saveXyzImage(std::string const& filename, cv::Mat const& img) {
  if (img.depth() != CV_32F)
    LOG(FATAL) << "Expecting an image with float values\n";
  if (img.channels() != 3) LOG(FATAL) << "Expecting 3 channels.\n";

  std::ofstream f;
  f.open(filename.c_str(), std::ios::binary | std::ios::out);
  if (!f.is_open()) LOG(FATAL) << "Cannot open file for writing: " << filename << "\n";

  // Assign these to explicit variables so we know their type and size in bytes
  // TODO(oalexan1): Replace below with int32_t and check that it is same thing.
  int rows = img.rows, cols = img.cols, channels = img.channels();

  // TODO(oalexan1): Avoid C-style cast. Test if
  // reinterpret_cast<char*> does the same thing.
  f.write((char*)(&rows), sizeof(rows));         // NOLINT
  f.write((char*)(&cols), sizeof(cols));         // NOLINT
  f.write((char*)(&channels), sizeof(channels)); // NOLINT

  for (int row = 0; row < rows; row++) {
    for (int col = 0; col < cols; col++) {
      cv::Vec3f const& P = img.at<cv::Vec3f>(row, col);  // alias
      // TODO(oalexan1): See if using reinterpret_cast<char*> does the same
      // thing.
      for (int c = 0; c < channels; c++)
        f.write((char*)(&P[c]), sizeof(P[c])); // NOLINT
    }
  }

  return;
}

// Save images and depth clouds to disk
void saveImagesAndDepthClouds(std::vector<rig::cameraImage> const& cams) {
  for (size_t it = 0; it < cams.size(); it++) {

    std::cout << "Writing: " << cams[it].image_name << std::endl;
    cv::imwrite(cams[it].image_name, cams[it].image);

    if (cams[it].depth_cloud.cols > 0 && cams[it].depth_cloud.rows > 0) {
      std::cout << "Writing: " << cams[it].depth_name << std::endl;
      rig::saveXyzImage(cams[it].depth_name, cams[it].depth_cloud);
    }
  }

  return;
}

// Read an image with 3 floats per pixel. OpenCV's imread() cannot do that.
void readXyzImage(std::string const& filename, cv::Mat & img) {
  std::ifstream f;
  f.open(filename.c_str(), std::ios::binary | std::ios::in);
  if (!f.is_open()) LOG(FATAL) << "Cannot open file for reading: " << filename << "\n";

  int rows, cols, channels;
  // TODO(oalexan1): Replace below with int32_t and check that it is same thing.
  f.read((char*)(&rows), sizeof(rows));         // NOLINT
  f.read((char*)(&cols), sizeof(cols));         // NOLINT
  f.read((char*)(&channels), sizeof(channels)); // NOLINT

  img = cv::Mat::zeros(rows, cols, CV_32FC3);

  for (int row = 0; row < rows; row++) {
    for (int col = 0; col < cols; col++) {
      cv::Vec3f P;
      // TODO(oalexan1): See if using reinterpret_cast<char*> does the same
      // thing.
      for (int c = 0; c < channels; c++)
        f.read((char*)(&P[c]), sizeof(P[c])); // NOLINT
      img.at<cv::Vec3f>(row, col) = P;
    }
  }

  return;
}

// Parse a file each line of which contains a filename, a sensor name, and a timestamp.
// Then reorder the data based on input file names. 
bool parseImageSensorList(std::string const& image_sensor_list,
                          std::vector<std::string> const& image_files,
                          std::vector<std::string> const& cam_names,
                          bool flexible_strategy,
                          // Outputs
                          std::vector<int> & cam_types,
                          std::vector<double> & timestamps) {

  // Wipe the outputs
  cam_types.clear(); cam_types.resize(image_files.size());
  timestamps.clear(); timestamps.resize(image_files.size());
     
  // Open the file
  std::ifstream f(image_sensor_list.c_str());
  if (!f.is_open()) {
    if (flexible_strategy)
      return false;
    else
      LOG(FATAL) << "Cannot open file for reading: " << image_sensor_list << "\n";
  }

  std::cout << "Reading: " << image_sensor_list << std::endl;
  
  // Go from cam name to cam type
  std::map<std::string, int> cam_name_to_type;
  for (size_t it = 0; it < cam_names.size(); it++)
    cam_name_to_type[cam_names[it]] = it;
    
  // Must put the data to read in maps, as they may not be in the same
  // order as in the input image_files that we must respect.
  std::map<std::string, int> image_to_cam_type;
  std::map<std::string, double> image_to_timestamp;
  std::string line;
  while (getline(f, line)) {
    
    // if line starts with comment or has only white spaces, skip it
    if (line.empty() || line[0] == '#') continue;
    if (line.find_first_not_of(" \t\n\v\f\r") == std::string::npos) continue;
     
    std::string image_file;
    double timestamp = 0.0;
    std::string cam_name;
    std::istringstream iss(line);
    if (!(iss >> image_file >> cam_name >> timestamp)) {
      if (flexible_strategy)
        return false;
      else 
       LOG(FATAL) << "Cannot parse: " << image_sensor_list << "\n";
    }
    
    // Must not have duplicate image_file
    if (image_to_cam_type.find(image_file) != image_to_cam_type.end())
      LOG(FATAL) << "Duplicate image file: " << image_file << " in: "
                 << image_sensor_list << "\n";
    if (image_to_timestamp.find(image_file) != image_to_timestamp.end())
      LOG(FATAL) << "Duplicate image file: " << image_file << " in: "
                 << image_sensor_list << "\n";
    
    // Look up the sensor name    
    auto it = cam_name_to_type.find(cam_name);
    if (it == cam_name_to_type.end())
      LOG(FATAL) << "Cannot find sensor name: " << cam_name << "\n";
    image_to_cam_type[image_file] = it->second;
    image_to_timestamp[image_file] = timestamp;
  }
  
  // Must now add them in the order of image_files
  for (size_t img_it = 0; img_it < image_files.size(); img_it++) {
    auto type_it = image_to_cam_type.find(image_files[img_it]);
    auto time_it = image_to_timestamp.find(image_files[img_it]);
    if (type_it == image_to_cam_type.end() || time_it == image_to_timestamp.end())
      LOG(FATAL) << "Cannot find image file: " << image_files[img_it] 
                 << " in list: " << image_sensor_list << "\n";
    cam_types[img_it] = type_it->second;
    timestamps[img_it] = time_it->second;
  }
    
  return true;
}

// For each image, find its sensor name and timestamp. The info can be in a list or
// from the directory structure. If flexible_strategy is true, then 
// can try from list first, and if that fails, then from directory structure.
void readImageSensorTimestamp(std::string const& image_sensor_list, 
                              std::vector<std::string> const& image_files,
                              std::vector<std::string> const& cam_names,
                              bool flexible_strategy,
                              // Outputs
                              std::vector<int> & cam_types,
                              std::vector<double> & timestamps) {

  // Parse the image sensor list if it is not empty
  bool success = false;
  if (image_sensor_list != "") {
    bool success = parseImageSensorList(image_sensor_list, image_files, cam_names,
                                        flexible_strategy, 
                                        cam_types, timestamps); // outputs
    if (success)
      return;
    if (!success && !flexible_strategy)
      LOG(FATAL) << "Cannot parse the image sensor list: " << image_sensor_list << "\n";
  }
  
  // Use the directory structure
  // TODO(oalexan1): Also parse the file name without a directory structure.
  // Clear the outputs
  cam_types.clear(); cam_types.resize(image_files.size());
  timestamps.clear(); timestamps.resize(image_files.size());
  
  for (size_t it = 0; it < image_files.size(); it++) {
    int cam_type = 0;
    double timestamp = 0.0;
    try {
      findCamTypeAndTimestamp(image_files[it], cam_names,  
                              cam_type, timestamp); // outputs
    } catch (std::exception const& e) {
        LOG(FATAL) << "Could not infer sensor type and image timestamp. See the naming "
                   << "convention, and check your images. Detailed message:\n" << e.what();
    }
    
    cam_types[it] = cam_type;
    timestamps[it] = timestamp;
  }
}

// Find the name of the camera type of the images used in registration.
// The registration images must all be acquired with the same sensor.  
std::string registrationCamName(std::string const& hugin_file,
                                std::vector<std::string> const& cam_names,
                                std::vector<rig::cameraImage> const & cams) {

  std::vector<std::string> images;
  Eigen::MatrixXd user_ip;
  Eigen::MatrixXd user_xyz;
  ParseHuginControlPoints(hugin_file, &images, &user_ip);

  // Must create a map from the image name in cams to sensor type
  std::map<std::string, int> image_to_cam_type;
  for (size_t cid = 0; cid < cams.size(); cid++)
    image_to_cam_type[cams[cid].image_name] = cams[cid].camera_type;
  
  std::set<std::string> sensors;
  for (size_t cid = 0; cid < images.size(); cid++) {
    // Find the image in the map
    auto it = image_to_cam_type.find(images[cid]);
    if (it == image_to_cam_type.end())
      LOG(FATAL) << "Cannot find image: " << images[cid] 
        << " from the Hugin file having control points in the input SfM map.\n";
      
     sensors.insert(cam_names.at(it->second));
  }
  
  if (sensors.size() != 1) 
    LOG(FATAL) << "All images used in registration must be for the same sensor. "
               << "Check the registration file: " << hugin_file << ".\n";
  
  return *sensors.begin();
}

void readImageEntry(// Inputs
                    std::string const& image_file,
                    Eigen::Affine3d const& world_to_cam,
                    std::vector<std::string> const& cam_names,
                    int cam_type,
                    double timestamp,
                    // Outputs
                    std::vector<std::map<double, rig::ImageMessage>> & image_maps,
                    std::vector<std::map<double, rig::ImageMessage>> & depth_maps) {
  
  // Aliases
  std::map<double, ImageMessage> & image_map = image_maps[cam_type];
  std::map<double, ImageMessage> & depth_map = depth_maps[cam_type];

  if (image_map.find(timestamp) != image_map.end())
    std::cout << "WARNING: Duplicate timestamp " << std::setprecision(17) << timestamp
                 << " for sensor id " << cam_type << "\n";
  
  // Read the image as grayscale, in order for feature matching to work
  // For texturing, texrecon should use the original color images.
  //std::cout << "Reading: " << image_file << std::endl;
  image_map[timestamp].image        = cv::imread(image_file, cv::IMREAD_GRAYSCALE);
  image_map[timestamp].name         = image_file;
  image_map[timestamp].timestamp    = timestamp;
  image_map[timestamp].world_to_cam = world_to_cam;

  // Sanity check
  if (depth_map.find(timestamp) != depth_map.end())
    LOG(WARNING) << "Duplicate timestamp " << std::setprecision(17) << timestamp
                 << " for sensor id " << cam_type << "\n";

  // Read the depth data, if present
  std::string depth_file = fs::path(image_file).replace_extension(".pc").string();
  if (fs::exists(depth_file)) {
    //std::cout << "Reading: " << depth_file << std::endl;
    rig::readXyzImage(depth_file, depth_map[timestamp].image);
    depth_map[timestamp].name      = depth_file;
    depth_map[timestamp].timestamp = timestamp;
  }
}

// Add poses for the extra desired images based on interpolation, extrapolation,
// and/or the rig transform.
void calcExtraPoses(std::string const& extra_list, bool use_initial_rig_transforms,
                    double bracket_len, bool nearest_neighbor_interp,
                    rig::RigSet const& R,
                    // Append here
                    std::vector<std::string>     & cid_to_filename,
                    std::vector<int>             & cam_types,
                    std::vector<double>          & timestamps,
                    std::vector<Eigen::Affine3d> & cid_to_cam_t_global) {

  // Put the existing poses in a map
  std::map<int, std::map<double, Eigen::Affine3d>> existing_world_to_cam;
  std::set<std::string> existing_images;

  for (size_t image_it = 0; image_it < cid_to_filename.size(); image_it++) {
    auto const& image_file = cid_to_filename[image_it];
    existing_images.insert(image_file); 
    int cam_type = cam_types[image_it];
    double timestamp = timestamps[image_it];
    Eigen::Affine3d world_to_cam = cid_to_cam_t_global[image_it];
    existing_world_to_cam[cam_type][timestamp] = world_to_cam;

    if (use_initial_rig_transforms) {
      // Use the rig constraint to find the poses for the other sensors on the rig
      // First go to the ref sensor
      double ref_timestamp = timestamp - R.ref_to_cam_timestamp_offsets[cam_type];

      // Careful here with transform directions and order
      Eigen::Affine3d cam_to_ref = R.ref_to_cam_trans[cam_type].inverse();
      Eigen::Affine3d world_to_ref = cam_to_ref * world_to_cam;

      // Now do all the sensors on that rig. Note how we do the reverse of the above
      // timestamp and camera operations, but not just for the given cam_type,
      // but for any sensor on the rig.
      for (size_t sensor_it = 0; sensor_it < R.ref_to_cam_trans.size(); sensor_it++) {

        if (R.rigId(sensor_it) != R.rigId(cam_type)) 
          continue; // stay within the current rig
        
        // Initialize the map if needed
        if (existing_world_to_cam.find(sensor_it) == existing_world_to_cam.end())
          existing_world_to_cam[sensor_it] = std::map<double, Eigen::Affine3d>();

        // Add an entry, unless one already exists
        std::map<double, Eigen::Affine3d> & map = existing_world_to_cam[sensor_it]; // alias
        // TODO(oalexan1): Any issues with numerical precision of timestamps?
        double curr_timestamp = ref_timestamp + R.ref_to_cam_timestamp_offsets[sensor_it];
        if (map.find(curr_timestamp) == map.end())
          existing_world_to_cam[sensor_it][curr_timestamp]
            = R.ref_to_cam_trans[sensor_it] * world_to_ref;
      }
    }
  }
  
  // Read the extra image names. Ignore the ones already existing.
  std::ifstream f(extra_list.c_str());
  std::vector<std::string> extra_images;
  if (!f.is_open())
    LOG(FATAL) << "Cannot open file for reading: " << extra_list << "\n";
  std::string line;
  while (getline(f, line)) {
    if (line.empty() || line[0] == '#') continue;
    std::string image_file;
    std::istringstream iss(line);
    if (!(iss >> image_file))
      LOG(FATAL) << "Cannot parse the image file in: " << extra_list << "\n";
    if (existing_images.find(image_file) != existing_images.end()) 
      continue; // this image already exists
    extra_images.push_back(image_file);
  }
  
  // Infer the timestamp and sensor type for the extra images
  std::vector<int> extra_cam_types;
  std::vector<double> extra_timestamps;
  bool flexible_strategy = true; // can handle with and without separate attributes
  readImageSensorTimestamp(extra_list, extra_images, R.cam_names, 
                           flexible_strategy,
                           extra_cam_types, extra_timestamps); // outputs
  
  // Save the new images in a map, to ensure they are sorted.
  // Also need maps for cam types and timestamps, to be able to associate
  // image names with these
  std::map<int, std::map<double, std::string>> extra_map; 
  std::map<std::string, int> extra_cam_types_map;
  std::map<std::string, double> extra_timestamps_map;
  for (size_t image_it = 0; image_it < extra_images.size(); image_it++) {
    std::string image_file = extra_images[image_it]; 
    int cam_type = extra_cam_types[image_it];
    double curr_timestamp = extra_timestamps[image_it];
    extra_map[cam_type][curr_timestamp] = image_file;
    extra_cam_types_map[image_file] = cam_type;
    extra_timestamps_map[image_file] = curr_timestamp;
  }

  // Iterate over each sensor type and interpolate or extrapolate into existing data
  for (auto sensor_it = extra_map.begin(); sensor_it != extra_map.end(); sensor_it++) {

    int cam_type = sensor_it->first;
    std::map<double, std::string> & target_map = sensor_it->second; // alias
    
    // Look up existing poses to be used for interpolation/extrapolation
    std::map<double, Eigen::Affine3d> & input_map = existing_world_to_cam[cam_type]; // alias
    if (input_map.empty()) {
      std::string msg = std::string("Cannot find camera poses for sensor: ")
        + R.cam_names[cam_type] + " as the data is insufficient.\n";
      if (!use_initial_rig_transforms) 
        msg += std::string("If the rig configuration file has an initial rig, consider ")
          + "using the option --use_initial_rig_transforms.\n";
      std::cout << msg;
      continue;
    }

    std::vector<std::string> found_images;
    std::vector<Eigen::Affine3d> found_poses;
    interpOrExtrap(input_map, target_map, bracket_len, nearest_neighbor_interp,
                   found_images, found_poses); // outputs

    for (size_t found_it = 0; found_it < found_images.size(); found_it++) {
      cid_to_filename.push_back(found_images[found_it]);
      cid_to_cam_t_global.push_back(found_poses[found_it]);
      
      // Add the cam type and timestamp
      auto type_it = extra_cam_types_map.find(found_images[found_it]);
      if (type_it == extra_cam_types_map.end())
        LOG(FATAL) << "Cannot find cam type for image: " << found_images[found_it] << "\n";
      cam_types.push_back(type_it->second);
      auto time_it = extra_timestamps_map.find(found_images[found_it]);
      if (time_it == extra_timestamps_map.end())
        LOG(FATAL) << "Cannot find timestamp for image: " << found_images[found_it] << "\n";
      timestamps.push_back(time_it->second);
    }
  }
}

void readCameraPoses(// Inputs
                     std::string const& camera_poses_file,
                     // Outputs
                     nvmData & nvm) {
  
  // Clear the outputs
  nvm = nvmData();

  // Open the file
  std::cout << "Reading: " << camera_poses_file << std::endl;
  std::ifstream f(camera_poses_file.c_str());
  if (!f.is_open())
    LOG(FATAL) << "Cannot open file for reading: " << camera_poses_file << "\n";
  
  std::string line;
  while (getline(f, line)) {
    if (line.empty() || line[0] == '#') continue;
    
    std::string image_file;
    std::istringstream iss(line);
    if (!(iss >> image_file))
      LOG(FATAL) << "Cannot parse the image file in: "
                 << camera_poses_file << "\n";
    
    // Read the camera to world transform
    Eigen::VectorXd vals(12);
    double val = -1.0;
    int count = 0;
    while (iss >> val) {
      if (count >= 12) break;
      vals[count] = val;
      count++;
    }
    
    if (count != 12)
      LOG(FATAL) << "Expecting 12 values for the transform on line:\n" << line << "\n";
    
    Eigen::Affine3d world_to_cam = vecToAffine(vals);
    nvm.cid_to_cam_t_global.push_back(world_to_cam);
    nvm.cid_to_filename.push_back(image_file);
  }
}

// TODO(oalexan1): Move this to fileio.cc.  
// Read camera information and images from a list or from an NVM file.
// Can interpolate/extrapolate poses for data from an extra list.
// Only later we will consider if the features are shifted or not in the nvm.
void readListOrNvm(// Inputs
                   std::string const& camera_poses_list,
                   std::string const& nvm_file,
                   std::string const& image_sensor_list,
                   std::string const& extra_list,
                   bool use_initial_rig_transforms,
                   double bracket_len, bool nearest_neighbor_interp,
                   bool read_nvm_no_shift,
                   rig::RigSet const& R,
                   // Outputs
                   nvmData & nvm,
                   std::vector<std::map<double, rig::ImageMessage>> & image_maps,
                   std::vector<std::map<double, rig::ImageMessage>> & depth_maps) {

  // Wipe the outputs
  image_maps.clear();
  depth_maps.clear();
  image_maps.resize(R.cam_names.size());
  depth_maps.resize(R.cam_names.size());
  
  if (int(camera_poses_list.empty()) + int(nvm_file.empty()) != 1)
    LOG(FATAL) << "Must specify precisely one of --camera-poses or --nvm.\n";

  if (camera_poses_list != "") {
    rig::readCameraPoses(// Inputs
                               camera_poses_list,  
                               // Outputs
                               nvm);
  } else {
    rig::ReadNvm(nvm_file, 
                       nvm.cid_to_keypoint_map,  
                       nvm.cid_to_filename,  
                       nvm.pid_to_cid_fid,  
                       nvm.pid_to_xyz,  
                       nvm.cid_to_cam_t_global);
    if (!read_nvm_no_shift) {
      std::string offsets_file = rig::offsetsFilename(nvm_file);
      rig::readNvmOffsets(offsets_file, nvm.optical_centers); 
      // Must have as many offsets as images
      if (nvm.optical_centers.size() != nvm.cid_to_filename.size())
        LOG(FATAL) << "Expecting as many optical centers as images.\n";
    }
  }
  
  // Infer the timestamp and sensor type from list or directory structure
  std::vector<int> cam_types;
  std::vector<double> timestamps;
  bool flexible_strategy = false;
  readImageSensorTimestamp(image_sensor_list, nvm.cid_to_filename, R.cam_names, 
                           flexible_strategy,
                           cam_types, timestamps); // outputs
  
  // Extra poses need be be added right after reading the original ones,
  // to ensure the same book-keeping is done for all of them. The extra
  // entries do not mess up the bookkeeping of pid_to_cid_fid, etc,
  // if their cid is larger than the ones read from NVM.
  if (extra_list != "")
    calcExtraPoses(extra_list, use_initial_rig_transforms, bracket_len,
                   nearest_neighbor_interp, R,
                   // Append here
                   nvm.cid_to_filename, cam_types, timestamps, nvm.cid_to_cam_t_global);

  std::cout << "Reading the images.\n";
  for (size_t it = 0; it < nvm.cid_to_filename.size(); it++) {
    // Aliases
    auto const& image_file = nvm.cid_to_filename[it];
    auto const& world_to_cam = nvm.cid_to_cam_t_global[it];
    readImageEntry(image_file, world_to_cam, R.cam_names,  
                   cam_types[it], timestamps[it],
                   // Outputs
                   image_maps, depth_maps);
  }

  return;
}

void flagOutlierByExclusionDist(// Inputs
                                std::vector<camera::CameraParameters> const& cam_params,
                                std::vector<rig::cameraImage> const& cams,
                                std::vector<std::map<int, int>> const& pid_to_cid_fid,
                                std::vector<std::vector<std::pair<float, float>>>
                                const& keypoint_vec,
                                // Outputs
                                std::vector<std::map<int, std::map<int, int>>> &
                                pid_cid_fid_inlier) {

  // Initialize the output
  pid_cid_fid_inlier.resize(pid_to_cid_fid.size());

  // Iterate though interest point matches
  int num_excl = 0;
  for (size_t pid = 0; pid < pid_to_cid_fid.size(); pid++) {
    for (auto cid_fid = pid_to_cid_fid[pid].begin(); cid_fid != pid_to_cid_fid[pid].end();
         cid_fid++) {
      int cid = cid_fid->first;
      int fid = cid_fid->second;
      int cam_type = cams[cid].camera_type;

      // Initially there are inliers only
      pid_cid_fid_inlier[pid][cid][fid] = 1;

      // Flag as outliers pixels at the image boundary.
      Eigen::Vector2d dist_pix(keypoint_vec[cid][fid].first,
                               keypoint_vec[cid][fid].second);
      Eigen::Vector2i dist_size = cam_params[cam_type].GetDistortedSize();
      Eigen::Vector2i dist_crop_size = cam_params[cam_type].GetDistortedCropSize();
      // Note that if dist_crop_size equals dist_size, which is image
      // size, no outliers are flagged
      if (std::abs(dist_pix[0] - dist_size[0] / 2.0) > dist_crop_size[0] / 2.0  ||
          std::abs(dist_pix[1] - dist_size[1] / 2.0) > dist_crop_size[1] / 2.0) {
        rig::setMapValue(pid_cid_fid_inlier, pid, cid, fid, 0);
        num_excl++;
      }
    }
  }

  std::cout << "Removed " << num_excl << " features based on distorted_crop_size region.\n";
  
  return;
}

// Flag outliers by triangulation angle and reprojection error.  It is
// assumed that the cameras in world_to_cam are up-to-date given the
// current state of optimization, and that the residuals (including
// the reprojection errors) have also been updated beforehand.
void flagOutliersByTriAngleAndReprojErr(// Inputs
  double min_triangulation_angle, double max_reprojection_error,
  std::vector<std::map<int, int>> const& pid_to_cid_fid,
  std::vector<std::vector<std::pair<float, float>>> const& keypoint_vec,
  std::vector<Eigen::Affine3d> const& world_to_cam, 
  std::vector<Eigen::Vector3d> const& xyz_vec,
  std::vector<std::map<int, std::map<int, int>>> const& pid_cid_fid_to_residual_index,
  std::vector<double> const& residuals,
  // Outputs
  std::vector<std::map<int, std::map<int, int>>>& pid_cid_fid_inlier) {

  // Must deal with outliers by triangulation angle before
  // removing outliers by reprojection error, as the latter will
  // exclude some rays which form the given triangulated points.
  int num_outliers_small_angle = 0;
  for (size_t pid = 0; pid < pid_to_cid_fid.size(); pid++) {
    // Find the largest angle among any two intersecting rays
    double max_rays_angle = 0.0;
    bool point_checked = false;
    for (auto cid_fid1 = pid_to_cid_fid[pid].begin();
         cid_fid1 != pid_to_cid_fid[pid].end(); cid_fid1++) {
      int cid1 = cid_fid1->first;
      int fid1 = cid_fid1->second;

      // Deal with inliers only
      if (!rig::getMapValue(pid_cid_fid_inlier, pid, cid1, fid1)) 
        continue;

      Eigen::Vector3d cam_ctr1 = (world_to_cam[cid1].inverse()) * Eigen::Vector3d(0, 0, 0);
      Eigen::Vector3d ray1 = xyz_vec[pid] - cam_ctr1;
      ray1.normalize();

      for (auto cid_fid2 = pid_to_cid_fid[pid].begin();
           cid_fid2 != pid_to_cid_fid[pid].end(); cid_fid2++) {
        int cid2 = cid_fid2->first;
        int fid2 = cid_fid2->second;

        // Look at each cid and next cids
        if (cid2 <= cid1)
          continue;

        // Deal with inliers only
        if (!rig::getMapValue(pid_cid_fid_inlier, pid, cid2, fid2))
           continue;
        point_checked = true;

        Eigen::Vector3d cam_ctr2 = (world_to_cam[cid2].inverse()) * Eigen::Vector3d(0, 0, 0);
        Eigen::Vector3d ray2 = xyz_vec[pid] - cam_ctr2;
        ray2.normalize();

        double curr_angle = (180.0 / M_PI) * acos(ray1.dot(ray2));
        if (std::isnan(curr_angle) || std::isinf(curr_angle)) continue;
        max_rays_angle = std::max(max_rays_angle, curr_angle);
      }
    }

    if (max_rays_angle >= min_triangulation_angle)
      continue;  // This is a good triangulated point, with large angle of convergence
     
    if (!point_checked)
      continue; // this point was an outlier to start with 
       
    // Flag as outliers all the features for this cid and increment the counter
    num_outliers_small_angle++;
    for (auto cid_fid = pid_to_cid_fid[pid].begin();
         cid_fid != pid_to_cid_fid[pid].end(); cid_fid++) {
      int cid = cid_fid->first;
      int fid = cid_fid->second;
      rig::setMapValue(pid_cid_fid_inlier, pid, cid, fid, 0);
    }
  }

  std::cout << std::setprecision(4) 
            << "Removed " << num_outliers_small_angle 
            << " triangulated points out of " << pid_to_cid_fid.size() 
            << " (" << (100.0 * num_outliers_small_angle) / pid_to_cid_fid.size() << "%)"
            << " by ray convergence angle.\n";
  
  int num_outliers_reproj = 0;
  int num_total_features = 0;
  for (size_t pid = 0; pid < pid_to_cid_fid.size(); pid++) {
    for (auto cid_fid = pid_to_cid_fid[pid].begin();
         cid_fid != pid_to_cid_fid[pid].end(); cid_fid++) {
      int cid = cid_fid->first;
      int fid = cid_fid->second;

      // Deal with inliers only
      if (!rig::getMapValue(pid_cid_fid_inlier, pid, cid, fid)) 
        continue;

      num_total_features++;

      // Find the pixel residuals
      size_t residual_index = rig::getMapValue(pid_cid_fid_to_residual_index, pid, cid, fid);
      if (residuals.size() <= residual_index + 1) LOG(FATAL) << "Too few residuals.\n";

      double res_x = residuals[residual_index + 0];
      double res_y = residuals[residual_index + 1];
      // NaN values will never be inliers if the comparison is set as below
      bool is_good = (Eigen::Vector2d(res_x, res_y).norm() <= max_reprojection_error);
      if (!is_good) {
        num_outliers_reproj++;
        rig::setMapValue(pid_cid_fid_inlier, pid, cid, fid, 0);
      }
    }
  }

  std::cout << std::setprecision(4) << "Removed " << num_outliers_reproj
            << " outlier features using reprojection error, out of " << num_total_features
            << " (" << (100.0 * num_outliers_reproj) / num_total_features << "%)\n";

  return;
}

// Find convergence angles between every pair of images and save to disk their percentiles
// assumed that the cameras in world_to_cam are up-to-date given the
// current state of optimization, and that the residuals (including
// the reprojection errors) have also been updated beforehand.
void savePairwiseConvergenceAngles(// Inputs
  std::vector<std::map<int, int>> const& pid_to_cid_fid,
  std::vector<std::vector<std::pair<float, float>>> const& keypoint_vec,
  std::vector<rig::cameraImage> const& cams,
  std::vector<Eigen::Affine3d> const& world_to_cam,
  std::vector<Eigen::Vector3d> const& xyz_vec,
  std::vector<std::map<int, std::map<int, int>>> const& pid_cid_fid_inlier,
  std::string const& conv_angles_file) {

  std::map<std::pair<int, int>, std::vector<double>> conv_angles;
  
  for (size_t pid = 0; pid < pid_to_cid_fid.size(); pid++) {

    for (auto cid_fid1 = pid_to_cid_fid[pid].begin();
         cid_fid1 != pid_to_cid_fid[pid].end(); cid_fid1++) {
      int cid1 = cid_fid1->first;
      int fid1 = cid_fid1->second;

      // Deal with inliers only
      if (!rig::getMapValue(pid_cid_fid_inlier, pid, cid1, fid1)) continue;

      Eigen::Vector3d cam_ctr1 = (world_to_cam[cid1].inverse()) * Eigen::Vector3d(0, 0, 0);
      Eigen::Vector3d ray1 = xyz_vec[pid] - cam_ctr1;
      ray1.normalize();

      for (auto cid_fid2 = pid_to_cid_fid[pid].begin();
           cid_fid2 != pid_to_cid_fid[pid].end(); cid_fid2++) {
        int cid2 = cid_fid2->first;
        int fid2 = cid_fid2->second;

        // Look at each cid and next cids
        if (cid2 <= cid1)
          continue;

        // Deal with inliers only
        if (!rig::getMapValue(pid_cid_fid_inlier, pid, cid2, fid2)) continue;

        Eigen::Vector3d cam_ctr2 = (world_to_cam[cid2].inverse()) * Eigen::Vector3d(0, 0, 0);
        Eigen::Vector3d ray2 = xyz_vec[pid] - cam_ctr2;
        ray2.normalize();

        // Calculate the convergence angle
        double conv_angle = (180.0 / M_PI) * acos(ray1.dot(ray2));
        if (std::isnan(conv_angle) || std::isinf(conv_angle)) continue;

        // Add to the image pair
        std::pair<int, int> pair(cid1, cid2);
        conv_angles[pair].push_back(conv_angle);
      }
    }
  }

  // Sort the convergence angles per pair
  std::cout << "Writing: " << conv_angles_file << std::endl;
  std::ofstream ofs(conv_angles_file.c_str());
  ofs << "# Convergence angle percentiles (in degrees) for each image pair having matches\n";
  ofs << "# left_image right_image 25% 50% 75% num_angles_per_pair\n";
  ofs.precision(17);
  for (auto it = conv_angles.begin(); it != conv_angles.end(); it++) {

    // Sort the values first
    std::vector<double> & vals = it->second; // alias
    std::sort(vals.begin(), vals.end());
    int len = vals.size();
    
    int cid1 = (it->first).first;
    int cid2 = (it->first).second;
    ofs << cams[cid1].image_name << ' ' << cams[cid2].image_name << ' '
        << vals[0.25 * len] << ' ' << vals[0.5 * len] << ' ' << vals[0.75*len] << ' '
        << len << std::endl;
  }
  ofs.close();

  return;
}

// Save the list of images, for use with bundle_adjust.
void saveImageList(std::vector<rig::cameraImage> const& cams,
                   std::string const& image_list) {

  // Create the directory having image_list
  std::string dir = fs::path(image_list).parent_path().string();
  rig::createDir(dir);
  
  std::cout << "Writing: " << image_list << std::endl;
  std::ofstream ofs(image_list.c_str());
  for (size_t it = 0; it < cams.size(); it++)
    ofs << cams[it].image_name << std::endl;
  ofs.close();

  return;
}

// Apply a transform to inlier triangulated points  
void transformInlierTriPoints(// Inputs
  Eigen::Affine3d const& trans,
  std::vector<std::map<int, int>> const& pid_to_cid_fid,
  std::vector<std::map<int, std::map<int, int>>> const& pid_cid_fid_inlier,
  std::vector<Eigen::Vector3d> & xyz_vec) { // output
  
  if (pid_to_cid_fid.size() != pid_cid_fid_inlier.size())
    LOG(FATAL) << "Expecting as many inlier flags as there are tracks.\n";
  if (pid_to_cid_fid.size() != xyz_vec.size()) 
    LOG(FATAL) << "Expecting as many tracks as there are triangulated points.\n";

  for (size_t pid = 0; pid < pid_to_cid_fid.size(); pid++) {

    bool isInlierXyz = false;
    for (auto cid_fid1 = pid_to_cid_fid[pid].begin();
         cid_fid1 != pid_to_cid_fid[pid].end(); cid_fid1++) {
      int cid1 = cid_fid1->first;
      int fid1 = cid_fid1->second;

      // Deal with inliers only
      if (!rig::getMapValue(pid_cid_fid_inlier, pid, cid1, fid1)) continue;

      isInlierXyz = true;
      break;
    }

    if (isInlierXyz) 
      xyz_vec[pid] = trans * xyz_vec[pid];
  }
  
  return;
}

// A triangulated point that is equal to (0, 0, 0), inf, or NaN, is not good.
bool isGoodTri(Eigen::Vector3d const& P) {
  for (int c = 0; c < P.size(); c++) {
    if (std::isinf(P[c]) || std::isnan(P[c]))
      return false;
  }
  
  if (P[0] == 0 && P[1] == 0 && P[2] == 0) 
    return false;
  
  return true;
}
  
}  // end namespace rig
