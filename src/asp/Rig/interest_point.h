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

#ifndef INTEREST_POINT_H_
#define INTEREST_POINT_H_

#include <vw/InterestPoint/InterestData.h>

#include <opencv2/imgproc.hpp>
#include <glog/logging.h>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <string>
#include <vector>
#include <map>
#include <mutex>
#include <utility>

namespace camera {
  class CameraParameters;
}

namespace aspOpenMVG {
  namespace matching {
    class PairWiseMatches;
  }
}

namespace asp {
  class nvmData;
}

namespace rig {

class cameraImage;
class RigSet;
class ImageMessage;

// Copy IP information from an OpenCV KeyPoint object.
void setFromCvKeypoint(Eigen::Vector2d const& key, cv::Mat const& cv_descriptor,
                       vw::ip::InterestPoint& ip);

typedef std::pair<std::vector<vw::ip::InterestPoint>, std::vector<vw::ip::InterestPoint>> MATCH_PAIR;
typedef std::map<std::pair<int, int>, rig::MATCH_PAIR> MATCH_MAP;

void detectFeatures(const cv::Mat& image, bool verbose,
                    // Outputs
                    cv::Mat* descriptors, Eigen::Matrix2Xd* keypoints);

// This really likes haz cam first and nav cam second
void matchFeatures(std::mutex* match_mutex, int left_image_index, int right_image_index,
                   cv::Mat const& left_descriptors, cv::Mat const& right_descriptors,
                   Eigen::Matrix2Xd const& left_keypoints,
                   Eigen::Matrix2Xd const& right_keypoints, bool verbose,
                   // Output
                   MATCH_PAIR* matches);

// Form the match file name. Assume the input images are of the form
// cam_name/image.jpg. Use the ASP convention of the match file being
// run/run-image1__image2.match. This assumes all input images are unique.
// TODO(oalexan1): Duplicate code.
std::string matchFileName(std::string const& match_dir,
                          std::string const& left_image, std::string const& right_image,
                          std::string const& suffix);

// Routines for reading & writing interest point match files
// TODO(oalexan1): Duplicate code.

// TODO(oalexan1): Duplicate code
void Triangulate(bool rm_invalid_xyz, double focal_length,
                 std::vector<Eigen::Affine3d> const& world_to_cam,
                 std::vector<Eigen::Matrix2Xd> const& cid_to_keypoint_map,
                 std::vector<std::map<int, int>> * pid_to_cid_fid,
                 std::vector<Eigen::Vector3d> * pid_to_xyz);
  
// Triangulate two rays emanating from given undistorted and centered pixels
Eigen::Vector3d TriangulatePair(double focal_length1, double focal_length2,
                                Eigen::Affine3d const& world_to_cam1,
                                Eigen::Affine3d const& world_to_cam2, Eigen::Vector2d const& pix1,
                                Eigen::Vector2d const& pix2);

// Triangulate n rays emanating from given undistorted and centered pixels
Eigen::Vector3d Triangulate(std::vector<double> const& focal_length_vec,
                            std::vector<Eigen::Affine3d> const& world_to_cam_vec,
                            std::vector<Eigen::Vector2d> const& pix_vec);

// TODO(oalexan1): Move this to transform_utils.  Find the 3D
// transform from an abstract coordinate system to the world, given
// control points (pixel matches) and corresponding 3D
// measurements. It is assumed all images are acquired with the same
// camera.
Eigen::Affine3d registrationTransform(std::string                  const& hugin_file,
                                      std::string                  const& xyz_file,
                                      camera::CameraParameters     const& cam_params,
                                      std::vector<std::string>     const& cid_to_filename,
                                      std::vector<Eigen::Affine3d> const& world_to_cam_trans); 

// TODO(oalexan1): Move to transform_utils
// Find the name of the camera of the images used in registration.
// The registration images must all be acquired with the same sensor.  
std::string registrationCamName(std::string const& hugin_file,
                                std::vector<std::string> const& cam_names,
                                std::vector<rig::cameraImage> const & cams);
  

  
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
                         asp::nvmData & nvm);

void multiViewTriangulation(// Inputs
                            std::vector<camera::CameraParameters>  const& cam_params,
                            std::vector<rig::cameraImage>    const& cams,
                            std::vector<Eigen::Affine3d>           const& world_to_cam,
                            std::vector<std::map<int, int>>        const& pid_to_cid_fid,
                            std::vector<std::vector<std::pair<float, float>>> const& keypoint_vec,
                            // Outputs
                            std::vector<std::map<int, std::map<int, int>>>& pid_cid_fid_inlier,
                            std::vector<Eigen::Vector3d>& xyz_vec);

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
                           std::string const& out_dir);

// Save the list of images, for use with bundle_adjust.
void saveImageList(std::vector<rig::cameraImage> const& cams,
                   std::string const& image_list);

// Write an image with 3 floats per pixel. OpenCV's imwrite() cannot do that.
void saveXyzImage(std::string const& filename, cv::Mat const& img);

// Save images and depth clouds to disk
void saveImagesAndDepthClouds(std::vector<cameraImage> const& cams);

// For nvm data that has the keypoints shifted relative to the optical
// center, undo this shift when 'undo_shift' is true. So, add the optical center.
// When 'undo_shift' is false, subtract the optical center.
void shiftKeypoints(bool undo_shift, rig::RigSet const& R,
                    asp::nvmData & nvm); // output
  
// Read camera information and images from a list or from an NVM file.
// Can interpolate/extrapolate poses for data from an extra list.  
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
                   asp::nvmData & nvm,
                   std::vector<std::map<double, rig::ImageMessage>> & image_maps,
                   std::vector<std::map<double, rig::ImageMessage>> & depth_maps);
  
// Break up each track of keypoints of length N into N pairs, (T0,
// T1), (T1, T2), ,,. (T(N-1), T0). Find their indices in the merged
// set of keypoints. Repeat this for each input map to merge and
// accumulate the pairs. Later these will be combined into new tracks
// and any repeated data will be fused. This is very tied to the
// addKeypoints() function.
// Note that keypoint_offsets are applied before the cid2cid transform gets used!
// This is very error-prone!
void addMatchPairs(// Append from these
                   std::vector<std::map<int, int>>  const& pid_to_cid_fid,
                   std::vector<Eigen::Matrix2Xd>    const& cid_to_keypoint_map,
                   std::map<int, int>               const& cid2cid,
                   std::vector<Eigen::Vector2d>     const& keypoint_offsets,
                   std::vector<std::map<std::pair<float, float>, int>>
                   const& merged_keypoint_map, 
                   int cid_shift, size_t num_out_cams,
                   aspOpenMVG::matching::PairWiseMatches & match_map); // append here

// Given some tracks read from nvm from disk, append the ones from
// nvm. Some remapping is needed.  given that 'fid' values already
// exist for the given tracks and that the nvm read from disk
// may have the images in different order. New keypoints are recorded
// with the help of fid_count and merged_keypoint_map.
// Note that keypoint_offsets are applied before the cid2cid transform gets used!
// This is very error-prone!
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
                        std::vector<std::map<int, int>> & pid_to_cid_fid);
  
// Add keypoints from a map, appending to existing keypoints. Take into
// account how this map's cid gets transformed to the new map cid.
// Note that keypoint_offsets are applied before the cid2cid transform gets used!
// This is very error-prone!
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
                  & merged_keypoint_map);

// Remove duplicate tracks. There can still be two tracks with one contained
// in the other or otherwise having shared elements. 
void rmDuplicateTracks(std::vector<std::map<int, int>> & pid_to_cid_fid);
  
void flagOutlierByExclusionDist(// Inputs
                                std::vector<camera::CameraParameters> const& cam_params,
                                std::vector<rig::cameraImage> const& cams,
                                std::vector<std::map<int, int>> const& pid_to_cid_fid,
                                std::vector<std::vector<std::pair<float, float>>>
                                const& keypoint_vec,
                                // Outputs
                                std::vector<std::map<int, std::map<int, int>>>& pid_cid_fid_inlier);

void flagOutliersByTriAngleAndReprojErr
(// Inputs
 double min_triangulation_angle, double max_reprojection_error,
 std::vector<std::map<int, int>> const& pid_to_cid_fid,
 std::vector<std::vector<std::pair<float, float>>> const& keypoint_vec,
 std::vector<Eigen::Affine3d> const& world_to_cam, std::vector<Eigen::Vector3d> const& xyz_vec,
 std::vector<std::map<int, std::map<int, int>>> const& pid_cid_fid_to_residual_index,
 std::vector<double> const& residuals,
 // Outputs
 std::vector<std::map<int, std::map<int, int>>>& pid_cid_fid_inlier);

void savePairwiseConvergenceAngles(// Inputs
  std::vector<std::map<int, int>> const& pid_to_cid_fid,
  std::vector<std::vector<std::pair<float, float>>> const& keypoint_vec,
  std::vector<rig::cameraImage> const& cams,
  std::vector<Eigen::Affine3d> const& world_to_cam,
  std::vector<Eigen::Vector3d> const& xyz_vec,
  std::vector<std::map<int, std::map<int, int>>> const& pid_cid_fid_inlier,
  std::string const& conv_angles_file);

// Apply a transform to inlier triangulated points  
void transformInlierTriPoints(// Inputs
  Eigen::Affine3d const& trans,
  std::vector<std::map<int, int>> const& pid_to_cid_fid,
  std::vector<std::map<int, std::map<int, int>>> const& pid_cid_fid_inlier,
  std::vector<Eigen::Vector3d> & xyz_vec); // output

  // A triangulated point that is equal to (0, 0, 0), inf, or NaN, is not good.
  bool isGoodTri(Eigen::Vector3d const& P);
  
}  // namespace rig

#endif  // INTEREST_POINT_H_
