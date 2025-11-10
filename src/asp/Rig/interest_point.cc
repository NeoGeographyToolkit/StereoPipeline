#include <asp/Core/nvm.h>
#include <asp/Rig/basic_algs.h>
#include <asp/Rig/interest_point.h>
#include <asp/Rig/camera_image.h>
#include <asp/Rig/system_utils.h>
#include <asp/Rig/thread.h>
#include <asp/Rig/matching.h>
#include <asp/Rig/transform_utils.h>
#include <asp/Rig/interpolation_utils.h>
#include <asp/Rig/rig_config.h>
#include <asp/Rig/image_lookup.h>
#include <asp/Rig/tracks.h>
#include <asp/Rig/RigCameraParams.h>
#include <asp/Rig/nvmUtils.h>
#include <asp/Rig/rig_io.h>
#include <asp/Rig/triangulation.h> // Added for moved functions

#include <vw/InterestPoint/MatcherIO.h>
#include <vw/Math/RandomSet.h>

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
DEFINE_int32(max_pairwise_matches, 2000,
             "Maximum number of pairwise matches in an image pair to keep.");
DEFINE_double(goodness_ratio, 0.8,
              "A smaller value keeps fewer but more reliable float descriptor matches.");

namespace rig {

// Copy IP information from an OpenCV KeyPoint object.
void setFromCvKeypoint(Eigen::Vector2d const& key, cv::Mat const& cv_descriptor,
                       vw::ip::InterestPoint& ip) {
  ip.x = key[0];
  ip.y = key[1];
  ip.ix = round(ip.x);
  ip.iy = round(ip.y);
  ip.interest = 0;
  ip.octave = 0;
  ip.scale_lvl = 1;
  ip.scale = 1;
  ip.orientation = 0;
  ip.polarity = false;

  if (cv_descriptor.rows != 1 || cv_descriptor.cols < 2)
    LOG(FATAL) << "The descriptors must be in one row, and have at least two columns.";

  ip.descriptor.set_size(cv_descriptor.cols);
  for (size_t it = 0; it < ip.descriptor.size(); it++) {
    ip.descriptor[it] = cv_descriptor.at<float>(0, it);
  }
}
 
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
    rig::FeatureDetector detector("SURF");
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

void reduceMatches(std::vector<vw::ip::InterestPoint> & left_ip,
                   std::vector<vw::ip::InterestPoint> & right_ip) {
  
  // pick a random subset
  std::vector<int> subset;
  vw::math::pick_random_indices_in_range(left_ip.size(), FLAGS_max_pairwise_matches, subset);
  std::sort(subset.begin(), subset.end()); // sort the indices (not strictly necessary)
  
  std::vector<vw::ip::InterestPoint> left_ip_full, right_ip_full;
  left_ip_full.swap(left_ip);
  right_ip_full.swap(right_ip);
  
  left_ip.resize(FLAGS_max_pairwise_matches);
  right_ip.resize(FLAGS_max_pairwise_matches);
  for (size_t it = 0; it < subset.size(); it++) {
    left_ip[it] = left_ip_full[subset[it]];
    right_ip[it] = right_ip_full[subset[it]];
  }
}
  
// descriptor is what opencv descriptor was used to make the descriptors
// the descriptor maps are the features in the two images
// matches is output to contain the matching features between the two images
void FindMatches(const cv::Mat & img1_descriptor_map,
                  const cv::Mat & img2_descriptor_map, std::vector<cv::DMatch> * matches) {
  CHECK(img1_descriptor_map.depth() ==
        img2_descriptor_map.depth())
    << "Mixed descriptor types. Did you mash BRISK with SIFT/SURF?";

  // Check for early exit conditions
  matches->clear();
  if (img1_descriptor_map.rows == 0 ||
      img2_descriptor_map.rows == 0)
    return;

  // Traditional floating point descriptor
  cv::FlannBasedMatcher matcher;
  std::vector<std::vector<cv::DMatch> > possible_matches;
  matcher.knnMatch(img1_descriptor_map, img2_descriptor_map, possible_matches, 2);
  matches->clear();
  matches->reserve(possible_matches.size());
  for (std::vector<cv::DMatch> const& best_pair : possible_matches) {
    if (best_pair.size() == 1) {
      // This was the only best match, push it.
      matches->push_back(best_pair.at(0));
    } else {
      // Push back a match only if it is 25% better than the next best.
      if (best_pair.at(0).distance < FLAGS_goodness_ratio * best_pair.at(1).distance) {
        matches->push_back(best_pair[0]);
      }
    }
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
  rig::FindMatches(left_descriptors, right_descriptors, &cv_matches);

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

  std::vector<vw::ip::InterestPoint> left_ip, right_ip;
  for (size_t j = 0; j < cv_matches.size(); j++) {
    if (inlier_mask.at<uchar>(j, 0) == 0) continue;

    int left_ip_index = cv_matches.at(j).queryIdx;
    int right_ip_index = cv_matches.at(j).trainIdx;

    cv::Mat left_desc = left_descriptors.row(left_ip_index);
    cv::Mat right_desc = right_descriptors.row(right_ip_index);

    vw::ip::InterestPoint left;
    setFromCvKeypoint(left_keypoints.col(left_ip_index), left_desc, left);

    vw::ip::InterestPoint right;
    setFromCvKeypoint(right_keypoints.col(right_ip_index), right_desc, right);

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
  rig::FindMatches(left_descriptors, right_descriptors, &cv_matches);

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
  cv::Mat H = cv::estimateAffine2D(left_vec, right_vec, inlier_mask, cv::RANSAC,
                                   ransacReprojThreshold, maxIters, confidence);

  std::vector<vw::ip::InterestPoint> left_ip, right_ip;
  for (size_t j = 0; j < filtered_cv_matches.size(); j++) {
    int left_ip_index = filtered_cv_matches.at(j).queryIdx;
    int right_ip_index = filtered_cv_matches.at(j).trainIdx;

    if (inlier_mask.at<uchar>(j, 0) == 0) continue;

    cv::Mat left_desc = left_descriptors.row(left_ip_index);
    cv::Mat right_desc = right_descriptors.row(right_ip_index);

    vw::ip::InterestPoint left;
    setFromCvKeypoint(left_keypoints.col(left_ip_index), left_desc, left);

    vw::ip::InterestPoint right;
    setFromCvKeypoint(right_keypoints.col(right_ip_index), right_desc, right);

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

// Detects features, matches them between image pairs, and builds tracks.
// 
//  This function handles the entire feature detection and matching pipeline:
//  1. Detects keypoints and descriptors in all images using multiple threads.
//  2. Determines which image pairs to match (based on overlap or input pairs).
//  3. Matches features between pairs, filtering with camera geometry if enabled.
//  4. Optionally saves matches to disk.
//  5. Builds keypoint maps and feature-ID (fid) counts.
//  6. Builds connectivity tracks (pid_to_cid_fid) from the pairwise matches.
void detectMatchFeatures(// Inputs
    std::vector<rig::cameraImage> const& cams,
    std::vector<camera::CameraParameters> const& cam_params,
    std::string const& out_dir, bool save_matches,
    bool filter_matches_using_cams,
    std::vector<Eigen::Affine3d> const& world_to_cam,
    int num_overlaps,
    std::vector<std::pair<int, int>> const& input_image_pairs,
    int initial_max_reprojection_error, int num_match_threads,
    bool verbose,
    // Outputs
    std::vector<std::map<std::pair<float, float>, int>>& keypoint_map,
    std::vector<int>& fid_count,
    std::vector<std::map<int, int>>& pid_to_cid_fid) {

  // Initialize outputs
  size_t num_images = cams.size();
  keypoint_map.clear();
  fid_count.clear();
  pid_to_cid_fid.clear();
  keypoint_map.resize(num_images);
  fid_count.resize(num_images, 0); // Important: initialize count to 0

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
        (&rig::detectFeatures,     // multi-threaded  // NOLINT
         // rig::detectFeatures(   // single-threaded // NOLINT
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
        (&rig::matchFeaturesWithCams,    // multi-threaded  // NOLINT
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
  cid_to_descriptor_map = std::vector<cv::Mat>();    // Wipe, no longer needed

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
      std::string match_file = rig::matchFileName(match_dir, left_image, right_image, suffix);
      std::cout << "Writing: " << left_image << " " << right_image << " "
                << match_file << std::endl;
      vw::ip::write_binary_match_file(match_file, match_pair.first, match_pair.second);
    }
  }
  
  // Collect all keypoints in keypoint_map, and put the fid (indices of keypoints) in
  // match_map. It will be used to find the tracks.
  aspOpenMVG::matching::PairWiseMatches match_map;
  for (auto it = matches.begin(); it != matches.end(); it++) {
    std::pair<int, int> const& cid_pair = it->first;      // alias

    int left_cid = cid_pair.first;
    int right_cid = cid_pair.second;

    rig::MATCH_PAIR const& match_pair = it->second;   // alias
    std::vector<vw::ip::InterestPoint> const& left_ip_vec = match_pair.first;
    std::vector<vw::ip::InterestPoint> const& right_ip_vec = match_pair.second;

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
}
         
void detectMatchappendFeatures(// Inputs
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
                         asp::nvmData & nvm) {

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
    nvm = asp::nvmData(); // no longer needed
    return;
  }

  // Detect and match features
  std::vector<std::map<std::pair<float, float>, int>> keypoint_map;
  std::vector<int> fid_count;
  detectMatchFeatures( // Inputs
                      cams, cam_params, out_dir, save_matches,
                      filter_matches_using_cams,
                      world_to_cam, num_overlaps,
                      input_image_pairs,
                      initial_max_reprojection_error, num_match_threads,
                      verbose,
                      // Outputs
                      keypoint_map, fid_count, pid_to_cid_fid);
  
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
  nvm = asp::nvmData(); // no longer needed
  keypoint_map.clear(); keypoint_map.shrink_to_fit();

  // Remove duplicate tracks. Those can happen since additional tracks being
  // created in matching can duplicate tracks read from disk. Ideally also
  // shorter tracks contained in longer tracks should be removed, and tracks
  // that can be merged without conflicts should be merged.
  rmDuplicateTracks(pid_to_cid_fid);

  return;
}

// TODO(oalexan1): All the logic below has little to do with interest
// points, and should be distributed to some other existing or new files.
  
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
  rig::Triangulate(rm_invalid_xyz,
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

}  // end namespace rig
