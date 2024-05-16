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

#ifndef SPARSE_MAPPING_TENSOR_H_
#define SPARSE_MAPPING_TENSOR_H_

#include <Rig/RigCameraModel.h>
#include <Eigen/Geometry>
#include <ceres/ceres.h>

#include <array>
#include <functional>
#include <map>
#include <set>
#include <string>
#include <utility>
#include <vector>
#include <limits>
#include <memory>
#include <mutex>

EIGEN_DEFINE_STL_VECTOR_SPECIALIZATION(std::array<std::pair<std::pair<int, int>, Eigen::Affine3d>, 3>)

namespace cv {
  class Mat;
  class DMatch;
}

namespace rig {
  struct nvmData;
  struct RigSet;
}

namespace sparse_mapping {

  typedef std::map<std::pair<int, int>, Eigen::Affine3d, std::less<std::pair<int, int>>,
                   Eigen::aligned_allocator<std::pair<std::pair<int, int> const, Eigen::Affine3d>>>
                   CIDPairAffineMap;
  typedef std::array<std::pair<std::pair<int, int>, Eigen::Affine3d>, 3> CIDAffineTuple;
  typedef std::vector<CIDAffineTuple, Eigen::aligned_allocator<CIDAffineTuple>> CIDAffineTupleVec;

  class SparseMap;

  // functions for building a map

  /**
   * Create the initial map by feature matching and essential affine computation.
   **/
  void MatchFeatures(const std::string & essential_file, const std::string & matches_file,
                     sparse_mapping::SparseMap * s);

  /**
   * Build the tracks based on the matches
   **/
  void BuildTracks(bool rm_invalid_xyz,
                   const std::string & matches_file,
                   sparse_mapping::SparseMap * s);

  /**
   * Incremental bundle adjustment.
   **/
  void IncrementalBA(std::string const& essential_file,
                     sparse_mapping::SparseMap * s);

  /**
   * Close a loop with repeated images.
   **/
  void CloseLoop(sparse_mapping::SparseMap * s);

  /**
   * Improve the map with bundle adjustment. Vary only the cameras
   * between given indices.
   **/
  void BundleAdjust(bool fix_all_cameras, sparse_mapping::SparseMap * map,
                    std::set<int> const& fixed_cameras = std::set<int>());

  void BundleAdjustment(sparse_mapping::SparseMap * s,
                        ceres::LossFunction * loss,
                        const ceres::Solver::Options & options,
                        ceres::Solver::Summary * summary,
                        int first = 0, int last = std::numeric_limits<int>::max(),
                        bool fix_all_cameras = false,
                        std::set<int> const& fixed_cameras = std::set<int>());

  // Extract a submap in-place.
  void ExtractSubmap(std::vector<std::string> const& images_to_keep,
                     rig::nvmData & nvm);

  // I/O Functions for writing and reading affine solutions.
  void WriteAffineCSV(CIDPairAffineMap const& relative_affines,
                      std::string const& output_filename);
  void WriteAffineCSV(CIDAffineTupleVec const& relative_affines,
                      std::string const& output_filename);
  void ReadAffineCSV(std::string const& input_filename,
                     CIDPairAffineMap* relative_affines);
  void ReadAffineCSV(std::string const& input_filename,
                     CIDAffineTupleVec* relative_affines);

  // Other auxiliary functions

  void PrintTrackStats(std::vector<std::map<int, int>>const& pid_to_cid_fid,
                       std::string const& step);

  void BuildMapFindEssentialAndInliers(const Eigen::Matrix2Xd & keypoints1,
                                       const Eigen::Matrix2Xd & keypoints2,
                                       const std::vector<cv::DMatch> & matches,
                                       camera::CameraParameters const& camera_params,
                                       bool compute_inliers_only,
                                       size_t cam_a_idx, size_t cam_b_idx,
                                       std::mutex * match_mutex,
                                       CIDPairAffineMap * relative_b_t_a,
                                       std::vector<cv::DMatch> * inlier_matches,
                                       bool compute_rays_angle,
                                       double * rays_angle);

  // Helper utility to speed up our query times into the map. The
  // SparseMap object appears to have this ability now. Possibly don't
  // need this function anymore.
  void GenerateCIDToPIDFIDMap(std::vector<std::map<int, int>> const& pid_to_cid_fid,
                              size_t num_of_cameras,
                              std::vector<std::map<int, int>> * cid_to_pid_fid);

  // List all the possible tuples (cameras paired in threes) that are
  // available from the found camera pairwise affines.
  void GenerateTupleListing(CIDPairAffineMap const& relative_affines,
                            std::set<std::tuple<int, int, int>> * tuple_listing);

  // Triangulates all points given camera positions. This is better
  // than what is in sparse map as it uses multiple view information.
  void Triangulate(bool rm_invalid_xyz, double focal_length,
                   std::vector<Eigen::Affine3d> const& cid_to_cam_t_global,
                   std::vector<Eigen::Matrix2Xd> const& cid_to_keypoint_map,
                   std::vector<std::map<int, int>> * pid_to_cid_fid,
                   std::vector<Eigen::Vector3d> * pid_to_xyz,
                   std::vector<std::map<int, int>> * cid_fid_to_pid);

}  // namespace sparse_mapping

#endif  // SPARSE_MAPPING_TENSOR_H_
