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
/// Handle NVM files, which are used to store SfM results.

#ifndef __ASP_CORE_NVM_H__
#define __ASP_CORE_NVM_H__

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <map>
#include <vector>
#include <string>
#include <set>
namespace vw {
  namespace ba {
    class ControlNetwork;
  }
}

namespace asp {

struct nvmData {
  std::vector<Eigen::Matrix2Xd>    cid_to_keypoint_map;
  std::vector<std::string>         cid_to_filename;
  std::vector<std::map<int, int>>  pid_to_cid_fid;
  std::vector<Eigen::Vector3d>     pid_to_xyz;
  std::vector<Eigen::Affine3d>     world_to_cam;
  std::vector<double>              focal_lengths;
  // Optical center per image, kept in a separate file, maybe in different order.
  // Interest points in the nvm file are shifted relative to this.
  // Set these to 0 if there is no shift relative to the optical center.
  std::map<std::string, Eigen::Vector2d> optical_centers;
};

// A function to read nvm offsets. On each line there must be the image name,
// then the optical center column, then row. Read into an std::map, with the
// key being the image name, and the value being vector2 of the optical center.
void readNvmOffsets(std::string const& offset_path,
                    std::map<std::string, Eigen::Vector2d> & offsets);

// Read an NVM file. Any offset is applied upon reading.
void readNvm(std::string const& input_filename, bool nvm_no_shift, nvmData & nvm);

// Write an NVM file. Subtract from the interest points the given offset.
// The offsets are saved in a separate file.
void writeNvm(nvmData const& nvm, std::string const& output_filename);

// Read an NVM file into the VisionWorkbench control network format. The flag
// nvm_no_shift, if true, means that the interest points are not shifted
// relative to the optical center, so can be read as is.
void readNvmAsCnet(std::string const& input_filename, 
                   bool nvm_no_shift,
                   vw::ba::ControlNetwork & cnet,
                   std::vector<Eigen::Affine3d> & world_to_cam,
                   std::map<std::string, Eigen::Vector2d> & offsets);

// Create an nvm from a cnet. There is no shift in the interest points. That is
// applied only on loading and saving. Optionally, updated triangulated points
// and outlier flags can be passed in.
void cnetToNvm(vw::ba::ControlNetwork                 const& cnet,
               std::map<std::string, Eigen::Vector2d> const& offsets,
               std::vector<Eigen::Affine3d>           const& world_to_cam,
               // Output
               nvmData & nvm,
               // Optional updated triangulated points and outlier flags
               std::vector<Eigen::Vector3d> const& tri_vec = std::vector<Eigen::Vector3d>(),
               std::set<int> const& outliers = std::set<int>());
  
// Convert nvm to cnet
void nvmToCnet(nvmData const& nvm, 
               // Outputs
               vw::ba::ControlNetwork                 & cnet,
               std::map<std::string, Eigen::Vector2d> & offsets,
               std::vector<Eigen::Affine3d>           & world_to_cam);

// Write a cnet to an NVM file. On writing, the feature matches from the cnet will be
// shifted relative to the optical center. The optical center offsets are saved
// to a separate file.
void writeCnetAsNvm(vw::ba::ControlNetwork const& cnet,
                    std::map<std::string, Eigen::Vector2d> const& optical_offsets,
                    std::vector<Eigen::Affine3d> const& world_to_cam,
                    std::string const& output_filename);
  
} // end namespace asp

#endif//__ASP_CORE_NVM_H__
