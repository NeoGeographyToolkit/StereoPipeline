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


/// \file Nvm.h

#ifndef __ASP_CORE_NVM_H__
#define __ASP_CORE_NVM_H__

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <map>
#include <vector>
#include <string>

namespace asp {

struct nvmData {
  std::vector<Eigen::Matrix2Xd>    cid_to_keypoint_map;
  std::vector<std::string>         cid_to_filename;
  std::vector<std::map<int, int>>  pid_to_cid_fid;
  std::vector<Eigen::Vector3d>     pid_to_xyz;
  std::vector<Eigen::Affine3d>     cid_to_cam_t_global;
};

// A wrapper to carry fewer things around
void ReadNVM(std::string const& input_filename, nvmData & nvm);
  
// Reads the NVM control network format. The interest points may or may not
// be shifted relative to optical center. The user is responsible for knowing that.
// If a filename having extension _offset.txt instead of .nvm exists, read
// from it the optical center offsets and apply them.
void ReadNVM(std::string const& input_filename,
             std::vector<Eigen::Matrix2Xd> * cid_to_keypoint_map,
             std::vector<std::string> * cid_to_filename,
             std::vector<std::map<int, int>> * pid_to_cid_fid,
             std::vector<Eigen::Vector3d> * pid_to_xyz,
             std::vector<Eigen::Affine3d> * cid_to_cam_t_global);

void WriteNVM(std::vector<Eigen::Matrix2Xd> const& cid_to_keypoint_map,
              std::vector<std::string> const& cid_to_filename,
              std::vector<double> const& focal_lengths,
              std::vector<std::map<int, int>> const& pid_to_cid_fid,
              std::vector<Eigen::Vector3d> const& pid_to_xyz,
              std::vector<Eigen::Affine3d> const& cid_to_cam_t_global,
              std::string const& output_filename);
  
} // end namespace asp

#endif//__ASP_CORE_NVM_H__
