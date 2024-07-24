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

// Logic for nvm files and the sfm solution
#ifndef NVM_H_
#define NVM_H_

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <vector>
#include <map>

namespace camera {
  // Forward declaration
  class CameraParameters;
}

namespace rig {

// Forward declarations
class cameraImage;
class ImageMessage;
}

namespace rig {

struct nvmData {
  std::vector<Eigen::Matrix2Xd>    cid_to_keypoint_map;
  std::vector<std::string>         cid_to_filename;
  std::vector<std::map<int, int>>  pid_to_cid_fid;
  std::vector<Eigen::Vector3d>     pid_to_xyz;
  std::vector<Eigen::Affine3d>     world_to_cam;
  // Interest points in the nvm file are shifted relative to optical centers
  std::map<std::string, Eigen::Vector2d> optical_centers;
};

// Read cameras and interest points from an nvm file  
void ReadNvm(std::string               const & input_filename,
             std::vector<Eigen::Matrix2Xd>   & cid_to_keypoint_map,
             std::vector<std::string>        & cid_to_filename,
             std::vector<std::map<int, int>> & pid_to_cid_fid,
             std::vector<Eigen::Vector3d>    & pid_to_xyz,
             std::vector<Eigen::Affine3d>    & world_to_cam);

// Write the inliers in nvm format. The keypoints are shifted relative to the optical
// center, as written by Theia.
void writeInliersToNvm
(std::string                                       const& nvm_file,
 bool                                                     shift_keypoints,
 std::vector<camera::CameraParameters>             const& cam_params,
 std::vector<rig::cameraImage>               const& cams,
 std::vector<Eigen::Affine3d>                      const& world_to_cam,
 std::vector<std::vector<std::pair<float, float>>> const& keypoint_vec,
 std::vector<std::map<int, int>>                   const& pid_to_cid_fid,
 std::vector<std::map<int, std::map<int, int>>>    const& pid_cid_fid_inlier,
 std::vector<Eigen::Vector3d>                      const& xyz_vec);

// A function to create the offsets filename from the nvm filename
std::string offsetsFilename(std::string const& nvm_filename);

// A function to read nvm offsets (optical center per image). On each line there
// must be the image name, then the optical center column, then row. Read into
// an std::map, with the key being the image name, and the value being vector2
// of the optical center. Interest point matches are shifted relative to this.
void readNvmOffsets(std::string const& offset_path,
                    std::map<std::string, Eigen::Vector2d> & offsets);
  
// Write the optical center offsets to a file. The format is the image name,
// then the optical center column, then row. 
void writeNvmOffsets(std::string const& offset_path,
                     std::map<std::string, Eigen::Vector2d> const& offsets);
  
// Write an nvm file. Keypoints may or may not be shifted relative to the optical center.
void WriteNvm(std::vector<Eigen::Matrix2Xd> const& cid_to_keypoint_map,
              std::vector<std::string> const& cid_to_filename,
              std::vector<std::map<int, int>> const& pid_to_cid_fid,
              std::vector<Eigen::Vector3d> const& pid_to_xyz,
              std::vector<Eigen::Affine3d> const& world_to_cam,
              std::string const& output_filename);

// Given a map from current cid to new cid, apply this map to the nvm. This can
// extract a submap and/or reorder data.
void remapNvm(std::map<int, int>                const& cid2cid,
              // Outputs
              std::vector<Eigen::Matrix2Xd>          & cid_to_keypoint_map,
              std::vector<std::string>               & cid_to_filename,
              std::vector<std::map<int, int>>        & pid_to_cid_fid,
              std::vector<Eigen::Vector3d>           & pid_to_xyz,
              std::vector<Eigen::Affine3d>           & world_to_cam,
              std::map<std::string, Eigen::Vector2d> & optical_centers);

// Extract a submap in-place from an nvm object.
void ExtractSubmap(std::vector<std::string> const& images_to_keep,
                    rig::nvmData & nvm);
  
// A utility for saving a camera in a format ASP understands. For now do not save
// the distortion.
// TODO(oalexan1): Move this somewhere else.
void writePinholeCamera(camera::CameraParameters const& cam_params,
                        Eigen::Affine3d          const& world_to_cam,
                        std::string              const& filename);
  
// Save the optimized cameras in ASP's Pinhole format. For now do not save
// the distortion model.
// TODO(oalexan1): Move this somewhere else.
void writePinholeCameras(std::vector<std::string>              const& cam_names,
                         std::vector<camera::CameraParameters> const& cam_params,
                         std::vector<rig::cameraImage>   const& cams,
                         std::vector<Eigen::Affine3d>          const& world_to_cam,
                         std::string                           const& out_dir);
  
}  // namespace rig

#endif  // NVM_H_
