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

#include <Rig/nvm.h>
#include <Rig/RigCameraParams.h>
#include <Rig/RigRpcDistortion.h>
#include <Rig/camera_image.h>
#include <Rig/basic_algs.h>
#include <Rig/system_utils.h>

#include <vw/Core/Exception.h>
#include <vw/Core/Log.h>
#include <vw/FileIO/FileUtils.h>
#include <vw/BundleAdjustment/ControlNetwork.h>

#include <boost/filesystem.hpp>
#include <glog/logging.h>

#include <iostream>
#include <fstream>

namespace fs = boost::filesystem;

namespace rig {

// Reads the NVM control network format. This function does not apply
// the optical offsets. Use instead the function that does.
void readNvm(std::string               const & input_filename,
             std::vector<Eigen::Matrix2Xd>   & cid_to_keypoint_map,
             std::vector<std::string>        & cid_to_filename,
             std::vector<std::map<int, int>> & pid_to_cid_fid,
             std::vector<Eigen::Vector3d>    & pid_to_xyz,
             std::vector<Eigen::Affine3d>    & world_to_cam,
             std::vector<double>             & focal_lengths) {

  vw::vw_out() << "Reading: " << input_filename << "\n";
  std::ifstream f(input_filename, std::ios::in);
  std::string token;
  std::getline(f, token);
  
  // Assert that we start with our NVM token
  if (token.compare(0, 6, "NVM_V3") != 0) {
    vw::vw_throw(vw::ArgumentErr() << "File doesn't start with NVM token.");
  }

  // Read the number of cameras
  ptrdiff_t number_of_cid;
  f >> number_of_cid;
  if (number_of_cid < 1) {
    vw::vw_throw(vw::ArgumentErr() << "NVM file is missing cameras.");
  }

  // Resize all our structures to support the number of cameras we now expect
  cid_to_keypoint_map.resize(number_of_cid);
  cid_to_filename.resize(number_of_cid);
  world_to_cam.resize(number_of_cid);
  focal_lengths.resize(number_of_cid);
  for (ptrdiff_t cid = 0; cid < number_of_cid; cid++) {
    // Clear keypoints from map. We'll read these in shortly
    cid_to_keypoint_map.at(cid).resize(Eigen::NoChange_t(), 2);

    // Read the line that contains camera information
    std::string image_name; 
    double focal, dist1, dist2;
    Eigen::Quaterniond q;
    Eigen::Vector3d c;
    f >> image_name >> focal;
    f >> q.w() >> q.x() >> q.y() >> q.z();
    f >> c[0] >> c[1] >> c[2] >> dist1 >> dist2;
    cid_to_filename.at(cid) = image_name;
    focal_lengths.at(cid) = focal;

    // Solve for t, which is part of the affine transform
    Eigen::Matrix3d r = q.matrix();
    world_to_cam.at(cid).linear() = r;
    world_to_cam.at(cid).translation() = -r * c;
  }

  // Read the number of points
  ptrdiff_t number_of_pid;
  f >> number_of_pid;
  if (number_of_pid < 1)
    vw::vw_throw(vw::ArgumentErr() << "The NVM file has no triangulated points.");

  // Read the point
  pid_to_cid_fid.resize(number_of_pid);
  pid_to_xyz.resize(number_of_pid);
  Eigen::Vector3d xyz;
  Eigen::Vector3i color;
  Eigen::Vector2d pt;
  ptrdiff_t cid, fid;
  for (ptrdiff_t pid = 0; pid < number_of_pid; pid++) {
    pid_to_cid_fid.at(pid).clear();

    ptrdiff_t number_of_measures;
    f >> xyz[0] >> xyz[1] >> xyz[2] >>
      color[0] >> color[1] >> color[2] >> number_of_measures;
    pid_to_xyz.at(pid) = xyz;
    for (ptrdiff_t m = 0; m < number_of_measures; m++) {

      f >> cid >> fid >> pt[0] >> pt[1];
      pid_to_cid_fid.at(pid)[cid] = fid;

      if (cid_to_keypoint_map.at(cid).cols() <= fid)
        cid_to_keypoint_map.at(cid).conservativeResize(Eigen::NoChange_t(), fid + 1);
      
      cid_to_keypoint_map.at(cid).col(fid) = pt;
    }

    if (!f.good())
      vw::vw_throw(vw::ArgumentErr() << "Unable to read file: " << input_filename);
  }
}

// A function to create the offsets filename from the nvm filename
std::string offsetsFilename(std::string const& nvm_filename) {
  int file_len = nvm_filename.size(); // cast to int to make subtraction safe
  // The length must be at least 5, as it must end with .nvm
  if (file_len < 5) 
    vw::vw_throw(vw::ArgumentErr() << "Invalid nvm filename: " << nvm_filename << ".\n");
  return nvm_filename.substr(0, std::max(file_len - 4, 0)) + "_offsets.txt";
}

// A function to read nvm offsets (optical center per image). On each line there
// must be the image name, then the optical center column, then row. Read into
// an std::map, with the key being the image name, and the value being vector2
// of the optical center. Interest point matches are shifted relative to this.
void readNvmOffsets(std::string const& offset_path,
                    std::map<std::string, Eigen::Vector2d> & offsets) {

  // Wipe the output
  offsets.clear();
  
  std::ifstream offset_fh(offset_path.c_str());
  if (!offset_fh.good())
    vw::vw_throw(vw::ArgumentErr() << "Cannot find optical offsets file: "
                 << offset_path << ".\n");
  
  vw::vw_out() << "Reading optical centers: " << offset_path << std::endl;
  
  std::string name;
  double x, y;
  while (offset_fh >> name >> x >> y) {
    // Check for repeated entries
    if (offsets.find(name) != offsets.end())
      vw::vw_throw(vw::ArgumentErr() << "Repeated optical center entry for image: "
                   << name << ".\n");
    offsets[name] = Eigen::Vector2d(x, y);
  }
}

// Write the optical center offsets to a file. The format is the image name,
// then the optical center column, then row. 
void writeNvmOffsets(std::string const& offset_path,
                     std::map<std::string, Eigen::Vector2d> const& offsets) {

  std::ofstream offset_fh(offset_path.c_str());
  offset_fh.precision(17); // double precision
  
  if (!offset_fh.good())
    vw::vw_throw(vw::ArgumentErr() << "Cannot write optical offsets file: "
                 << offset_path << ".\n");
  
  vw::vw_out() << "Writing optical centers: " << offset_path << std::endl;
  for (auto it = offsets.begin(); it != offsets.end(); it++)
    offset_fh << it->first << ' ' << it->second[0] << ' ' << it->second[1] << std::endl;
}

// Write the inliers in nvm format. The keypoints are shifted relative to the optical
// center, as written by Theia if shift_keypoints is specified.
// We handle properly the case when a (cid, fid) shows up in many tracks
// (this was a bug).
void writeInliersToNvm
(std::string                                       const& nvm_file,
 bool                                                     shift_keypoints, 
 std::vector<camera::CameraParameters>             const& cam_params,
 std::vector<rig::cameraImage>               const& cams,
 std::vector<Eigen::Affine3d>                      const& world_to_cam,
 std::vector<std::vector<std::pair<float, float>>> const& keypoint_vec,
 std::vector<std::map<int, int>>                   const& pid_to_cid_fid,
 std::vector<std::map<int, std::map<int, int>>>    const& pid_cid_fid_inlier,
 std::vector<Eigen::Vector3d>                      const& xyz_vec) {
  
  // Sanity checks
  if (world_to_cam.size() != cams.size())
    LOG(FATAL) << "Expecting as many world-to-camera transforms as cameras.\n";
  if (world_to_cam.size() != keypoint_vec.size()) 
    LOG(FATAL) << "Expecting as many sets of keypoints as cameras.\n";  
  if (pid_to_cid_fid.size() != pid_cid_fid_inlier.size())
    LOG(FATAL) << "Expecting as many inlier flags as there are tracks.\n";
  if (pid_to_cid_fid.size() != xyz_vec.size()) 
    LOG(FATAL) << "Expecting as many tracks as there are triangulated points.\n";

  // Initialize the keypoints in expected format. Copy the filenames
  // and focal lengths.
  std::vector<Eigen::Matrix2Xd> cid_to_keypoint_map(keypoint_vec.size());
  std::vector<std::string> cid_to_filename(keypoint_vec.size());
  for (size_t cid = 0; cid < cams.size(); cid++) {
    cid_to_keypoint_map[cid] = Eigen::MatrixXd(2, keypoint_vec[cid].size());
    cid_to_filename[cid] = cams[cid].image_name;
  }
  
  // Copy over only inliers, and tracks of length >= 2.
  std::vector<std::map<int, int>> nvm_pid_to_cid_fid;
  std::vector<Eigen::Vector3d> nvm_pid_to_xyz;

  for (size_t pid = 0; pid < pid_to_cid_fid.size(); pid++) {

    std::map<int, int> nvm_cid_fid;
    for (auto cid_fid = pid_to_cid_fid[pid].begin();
         cid_fid != pid_to_cid_fid[pid].end(); cid_fid++) {
      int cid = cid_fid->first;
      int fid = cid_fid->second;

      // Keep inliers only
      if (!rig::getMapValue(pid_cid_fid_inlier, pid, cid, fid))
        continue;

      Eigen::Vector2d dist_ip(keypoint_vec[cid][fid].first,
                              keypoint_vec[cid][fid].second);

      // Offset relative to the optical center
      if (shift_keypoints) 
        dist_ip -= cam_params[cams[cid].camera_type].GetOpticalOffset();

      // Add this to the keypoint map for cid in the location at fid_count[cid]
      cid_to_keypoint_map.at(cid).col(fid) = dist_ip;
      nvm_cid_fid[cid] = fid;
    }

    // Keep only tracks with at least two points
    if (nvm_cid_fid.size() > 1) {
      nvm_pid_to_cid_fid.push_back(nvm_cid_fid);
      nvm_pid_to_xyz.push_back(xyz_vec[pid]);
    }
  }
  
  writeNvm(cid_to_keypoint_map, cid_to_filename, nvm_pid_to_cid_fid,  
           nvm_pid_to_xyz, world_to_cam, nvm_file);

  // Write the optical center per image
  if (shift_keypoints) {
    // Remove .nvm and add new suffix
    std::string offset_path = changeFileSuffix(nvm_file, "_offsets.txt");
    std::cout << "Writing optical center per image: " << offset_path << std::endl;
    std::ofstream offset_fh(offset_path.c_str());
    offset_fh.precision(17);   // Save with the highest precision
    for (size_t cid = 0; cid < cid_to_filename.size(); cid++) {
      auto const& optical_center = cam_params[cams[cid].camera_type].GetOpticalOffset();
      offset_fh << cid_to_filename[cid] << " "
                << optical_center[0] << " " << optical_center[1] << "\n";
    }
  }
}

// Write an nvm file. Note that a single focal length is assumed and no distortion.
// Those are ignored, and only camera poses, matches, and keypoints are used.
// It is assumed that the interest points are shifted relative to the optical center.
// Write the optical center separately.
void writeNvm(std::vector<Eigen::Matrix2Xd> const& cid_to_keypoint_map,
              std::vector<std::string> const& cid_to_filename,
              std::vector<double> const& focal_lengths,
              std::vector<std::map<int, int>> const& pid_to_cid_fid,
              std::vector<Eigen::Vector3d> const& pid_to_xyz,
              std::vector<Eigen::Affine3d> const& world_to_cam,
              std::map<std::string, Eigen::Vector2d> const& optical_centers,
              std::string const& output_filename) {

  // Ensure that the output directory having this file exists
  vw::create_out_dir(output_filename);

  vw::vw_out() << "Writing: " << output_filename << std::endl;
  
  std::fstream f(output_filename, std::ios::out);
  f.precision(17); // double precision
  f << "NVM_V3\n";

  if (cid_to_filename.size() != cid_to_keypoint_map.size())
    vw::vw_throw(vw::ArgumentErr() << "Unequal number of filenames and keypoints.");
  if (pid_to_cid_fid.size() != pid_to_xyz.size())
    vw::vw_throw(vw::ArgumentErr() 
                 << "Unequal number of pid_to_cid_fid and xyz measurements.");
  if (cid_to_filename.size() != world_to_cam.size())
    vw::vw_throw(vw::ArgumentErr() << "Unequal number of filename and camera transforms.");
  
  // Write camera information
  f << cid_to_filename.size() << std::endl;
  for (size_t cid = 0; cid < cid_to_filename.size(); cid++) {

    // World-to-camera rotation quaternion
    Eigen::Quaterniond q(world_to_cam[cid].rotation());

    // Camera center in world coordinates
    Eigen::Vector3d t(world_to_cam[cid].translation());
    Eigen::Vector3d camera_center =
      -world_to_cam[cid].rotation().inverse() * t;

    double focal_length = 1.0;
    if (!focal_lengths.empty())
      focal_length = focal_lengths[cid];

    f << cid_to_filename[cid] << " " << focal_length
      << " " << q.w() << " " << q.x() << " " << q.y() << " " << q.z() << " "
      << camera_center[0] << " " << camera_center[1] << " "
      << camera_center[2] << " " << "0 0\n"; // zero distortion, not used
  }

  // Write the number of points
  f << pid_to_cid_fid.size() << std::endl;

  for (size_t pid = 0; pid < pid_to_cid_fid.size(); pid++) {
    f << pid_to_xyz[pid][0] << " " << pid_to_xyz[pid][1] << " "
      << pid_to_xyz[pid][2] << " 0 0 0 "
      << pid_to_cid_fid[pid].size();

    if (pid_to_cid_fid[pid].size() <= 1)
      vw::vw_throw(vw::ArgumentErr() << "PID " << pid << " has "
                   << pid_to_cid_fid[pid].size() << " measurements.");
    
    for (std::map<int, int>::const_iterator it = pid_to_cid_fid[pid].begin();
         it != pid_to_cid_fid[pid].end(); it++) {
    
      auto cid = it->first;
      auto fid = it->second;
    
      // Find the offset for this image
      Eigen::Vector2d offset = Eigen::Vector2d::Zero();
      if (!optical_centers.empty()) {
        auto map_it = optical_centers.find(cid_to_filename[cid]);
        if (map_it == optical_centers.end()) {
          vw::vw_throw(vw::ArgumentErr() << "Cannot find optical offset for image "
                      << cid_to_filename[cid] << "\n");
        }
        offset = map_it->second;
      }
      
      // Write with the offset subtracted
      f << " " << cid << " " << fid << " "
        << cid_to_keypoint_map[it->first].col(it->second)[0] - offset[0] << " "
        << cid_to_keypoint_map[it->first].col(it->second)[1] - offset[1];
    }
    f << std::endl;
  }

  // Close the file
  f.flush();
  f.close();
  
  if (!optical_centers.empty()) {
    // Write the optical center offsets
    std::string offset_path = offsetsFilename(output_filename);
    writeNvmOffsets(offset_path, optical_centers);
  }
}

// Write an nvm file. Keypoints are written as-is, and maybe shifted or not
// relative to the optical center. No focal length info is known, so it is set
// to 1.0.
void writeNvm(std::vector<Eigen::Matrix2Xd> const& cid_to_keypoint_map,
              std::vector<std::string> const& cid_to_filename,
              std::vector<std::map<int, int>> const& pid_to_cid_fid,
              std::vector<Eigen::Vector3d> const& pid_to_xyz,
              std::vector<Eigen::Affine3d> const& world_to_cam,
              std::string const& output_filename) {

  // Part of the API
  std::vector<double> focal_lengths;
  std::map<std::string, Eigen::Vector2d> const optical_centers;
  
  writeNvm(cid_to_keypoint_map, cid_to_filename, focal_lengths,
           pid_to_cid_fid, pid_to_xyz, world_to_cam, optical_centers,
           output_filename);
}

// Given a map from current cid to new cid, apply this map to the nvm. This can
// extract a submap and/or reorder data.
void remapNvm(std::map<int, int>                  const& cid2cid,
                // Outputs
                std::vector<Eigen::Matrix2Xd>          & cid_to_keypoint_map,
                std::vector<std::string>               & cid_to_filename,
                std::vector<std::map<int, int>>        & pid_to_cid_fid,
                std::vector<Eigen::Vector3d>           & pid_to_xyz,
                std::vector<Eigen::Affine3d>           & world_to_cam,
                std::vector<double>                    & focal_lengths,
                std::map<std::string, Eigen::Vector2d> & optical_centers) {

  // cid2d must be non-empty
  if (cid2cid.empty())
    LOG(FATAL) << "Cannot reorder a control network with an empty map.";
    
  // Find the min and max value in the map
  int min_out_cid = cid2cid.begin()->second;
  int max_out_cid = cid2cid.begin()->second;
  for (auto const& p : cid2cid) {
    min_out_cid = std::min(min_out_cid, p.second);
    max_out_cid = std::max(max_out_cid, p.second);
  }

  // Min cid must be non-negative
  if (min_out_cid < 0)
    LOG(FATAL) << "Cannot reorder a control network with negative cids.";
  
  // Check that all values in cid2cid are unique
  std::set<int> values;
  for (auto const& p : cid2cid) {
    if (values.find(p.second) != values.end())
      LOG(FATAL) << "Cannot reorder a control network with repeated cids.";
    values.insert(p.second);
  }
  
  // Allocate space for the output  
  int num_out = max_out_cid + 1;
  std::vector<Eigen::Matrix2Xd>   cid_to_keypoint_map_out(num_out);
  std::vector<std::string>        cid_to_filename_out(num_out);
  std::vector<Eigen::Affine3d>    world_to_cam_out(num_out);
  std::vector<double>             focal_lengths_out(num_out);
  std::vector<std::map<int, int>> pid_to_cid_fid_out;
  std::vector<Eigen::Vector3d>    pid_to_xyz_out;
  std::map<std::string, Eigen::Vector2d> optical_centers_out;
  
  for (size_t cid = 0; cid < cid_to_filename.size(); cid++) {
    auto it = cid2cid.find(cid);
    if (it == cid2cid.end()) 
      continue;
    size_t new_cid = it->second;
    cid_to_keypoint_map_out[new_cid] = cid_to_keypoint_map[cid];
    cid_to_filename_out[new_cid]     = cid_to_filename[cid];
    world_to_cam_out[new_cid]        = world_to_cam[cid];
    focal_lengths_out[new_cid]       = focal_lengths[cid];
    
    if (!optical_centers.empty()) {
      auto it2 = optical_centers.find(cid_to_filename[cid]);
      if (it2 != optical_centers.end())
        optical_centers_out[cid_to_filename[cid]] = it2->second;
    }
  }              
  
  // Create new pid_to_cid_fid and pid_to_xyz.
  for (size_t pid = 0; pid < pid_to_cid_fid.size(); pid++) {
    auto const& cid_fid = pid_to_cid_fid[pid];  // alias
    std::map<int, int> cid_fid_out;
    for (auto it = cid_fid.begin(); it != cid_fid.end(); it++) {
      int cid = it->first;
      auto cid2cid_it = cid2cid.find(cid);
      if (cid2cid_it == cid2cid.end()) 
        continue;  // not an image we want to keep
      int new_cid = cid2cid_it->second;  
      cid_fid_out[new_cid] = it->second; // fid does not change
    }
    
    if (cid_fid_out.size() <= 1) 
      continue;  // tracks must have size at least 2
    
    pid_to_cid_fid_out.push_back(cid_fid_out);
    pid_to_xyz_out.push_back(pid_to_xyz[pid]);
  }

  // Copy the output to the input
  cid_to_keypoint_map = cid_to_keypoint_map_out;
  cid_to_filename     = cid_to_filename_out;
  world_to_cam        = world_to_cam_out;
  focal_lengths       = focal_lengths_out;
  pid_to_cid_fid      = pid_to_cid_fid_out;
  pid_to_xyz          = pid_to_xyz_out;
  optical_centers     = optical_centers_out;
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

  // Remap the nvm
  rig::remapNvm(cid2cid, nvm.cid_to_keypoint_map, nvm.cid_to_filename,
                nvm.pid_to_cid_fid, nvm.pid_to_xyz, nvm.world_to_cam,
                nvm.focal_lengths, nvm.optical_centers);
  
  std::cout << "Number of images in the extracted map: " << nvm.cid_to_filename.size() << "\n";
  std::cout << "Number of tracks in the extracted map: " << nvm.pid_to_cid_fid.size() << "\n";

  return;
}

// For an image like image_dir/my_cam/image.png, create the file
// out_dir/image.tsai.
// TODO(oalexan1): Move this out of here.
std::string pinholeFile(std::string const& out_dir, 
                        std::string const& sensor_name, 
                        std::string const& image_file) {

  std::string base = fs::path(image_file).stem().string();
  
  // Ensure that the sensor name is part of the pinhole file name,
  // to ensure uniqueness.
  if (base.find(sensor_name) == std::string::npos)
    base = base + "_" + sensor_name;

  return out_dir + "/" + sensor_name + "/" + base + ".tsai";
}
  
// Write a line of the form: name = a b c
// TODO(oalexan1): Move this out of here.
void write_param_vec(std::string const& param_name, std::ofstream & os, 
                     Eigen::VectorXd const& vals) {

  os << param_name << " = ";
  for (size_t p = 0; p < vals.size(); p++){
    os << vals[p];
    if (p + 1 < vals.size()) 
      os << " "; // write a whitespace after each number except the last
  }
  os << "\n";
}
  
// A utility for saving a camera in a format ASP understands. 
// TODO(oalexan1): Move this somewhere else.
void writePinholeCamera(camera::CameraParameters const& cam_params,
                        Eigen::Affine3d const& world_to_cam,
                        std::string const& filename) {

  // Go from world_to_cam to cam_to_world
  Eigen::MatrixXd T = world_to_cam.inverse().matrix();

  // Must create the directory having the output file
  std::string out_dir = fs::path(filename).parent_path().string();
  rig::createDir(out_dir);
  
  std::ofstream ofs(filename);
  ofs.precision(17);
  ofs << "VERSION_4\n";
  ofs << "PINHOLE\n";
  ofs << "fu = " << cam_params.GetFocalVector()[0] << "\n";
  ofs << "fv = " << cam_params.GetFocalVector()[1] << "\n";
  ofs << "cu = " << cam_params.GetOpticalOffset()[0] << "\n";
  ofs << "cv = " << cam_params.GetOpticalOffset()[1] << "\n";
  ofs << "u_direction = 1 0 0\n";
  ofs << "v_direction = 0 1 0\n";
  ofs << "w_direction = 0 0 1\n";
  ofs << "C = " << T(0, 3) << ' ' << T(1, 3) << ' ' << T(2, 3) << "\n";
  ofs << "R = "
      << T(0, 0) << ' ' << T(0, 1) << ' ' << T(0, 2) << ' '
      << T(1, 0) << ' ' << T(1, 1) << ' ' << T(1, 2) << ' '
      << T(2, 0) << ' ' << T(2, 1) << ' ' << T(2, 2) << "\n";
  ofs << "pitch = 1\n";
  
  auto dist = cam_params.GetDistortion();
  // Dist size is 0, 1, 4, or 5
  if (dist.size() == 0) {
    ofs << "NULL\n";
  } else if (dist.size() == 1) {
    ofs << "FOV\n";
    ofs << "k1 = " << dist[0] << "\n";
  } else if (dist.size() == 4 && cam_params.m_distortion_type == camera::FISHEYE_DISTORTION) {
    ofs << "FISHEYE\n";
    ofs << "k1 = " << dist[0] << "\n";
    ofs << "k2 = " << dist[1] << "\n";
    ofs << "k3 = " << dist[2] << "\n";
    ofs << "k4 = " << dist[3] << "\n";
  } else if (dist.size() == 4 || dist.size() == 5) {
    ofs << "TSAI\n";
    ofs << "k1 = " << dist[0] << "\n";
    ofs << "k2 = " << dist[1] << "\n";
    ofs << "p1 = " << dist[2] << "\n";
    ofs << "p2 = " << dist[3] << "\n";
    if (dist.size() > 4)
      ofs << "k3 = " << dist[4] << "\n";
  } else if (dist.size() > 5) {
    // RPC
    Eigen::VectorXd num_x, den_x, num_y, den_y;
    rig::unpack_params(dist, num_x, den_x, num_y, den_y);
    prepend_1(den_x);
    prepend_1(den_y);
    write_param_vec("distortion_num_x", ofs, num_x);
    write_param_vec("distortion_den_x", ofs, den_x);
    write_param_vec("distortion_num_y", ofs, num_y);
    write_param_vec("distortion_den_y", ofs, den_y);

  } else {
    LOG(FATAL) << "Expecting 0, 1, 4, or 5 distortion coefficients.\n";
  }
  
  ofs.close();
  
}
  
// Save the optimized cameras in ASP's Pinhole format. 
// TODO(oalexan1): Move this somewhere else.
void writePinholeCameras(std::vector<std::string>              const& cam_names,
                         std::vector<camera::CameraParameters> const& cam_params,
                         std::vector<rig::cameraImage>   const& cams,
                         std::vector<Eigen::Affine3d>          const& world_to_cam,
                         std::string                           const& out_dir) {

  // Sanity checks
  if (world_to_cam.size() != cams.size())
    LOG(FATAL) << "Expecting as many world-to-camera transforms as cameras.\n";

  std::string pinhole_dir = out_dir + "/pinhole";
  std::cout << "Writing pinhole cameras to: " << pinhole_dir << std::endl;

  std::vector<std::string> pinholeCamFiles; 
  for (size_t cid = 0; cid < cams.size(); cid++) {
    std::string const& image_file = cams[cid].image_name;
    int cam_type = cams[cid].camera_type;
    std::string sensor_name = cam_names[cam_type]; 
    std::string camFile = rig::pinholeFile(pinhole_dir, sensor_name, image_file);
    rig::writePinholeCamera(cam_params[cam_type],  
                                  world_to_cam[cid], camFile);
    pinholeCamFiles.push_back(camFile);
  }

  // Must ensure that all camera names without directory are unique
  std::set<std::string> base_names;
  for (size_t it = 0; it < pinholeCamFiles.size(); it++) {
    std::string base_name = fs::path(pinholeCamFiles[it]).filename().string();
    if (base_names.find(base_name) != base_names.end())
      LOG(FATAL) << "Non-unique camera name (without directory): " << base_name 
                 << ". It will not be possible to use these cameras with bundle_adjust.\n";
    base_names.insert(base_name);
  }
  
  // Also save their list. Useful for bundle adjustment.
  std::string camera_list = out_dir + "/camera_list.txt";
  std::string dir = fs::path(camera_list).parent_path().string();
  rig::createDir(dir);
  
  std::cout << "Writing: " << camera_list << std::endl;
  std::ofstream ofs(camera_list.c_str());
  for (size_t it = 0; it < pinholeCamFiles.size(); it++)
    ofs << pinholeCamFiles[it] << "\n";
  ofs.close();
  
  return;
}

// Reads the NVM control network format. If a filename having extension
// _offset.txt instead of .nvm exists, read from it the optical center offsets
// and apply them.
void readNvm(std::string                       const& input_filename,
             bool                                     nvm_no_shift,
             std::vector<Eigen::Matrix2Xd>          & cid_to_keypoint_map,
             std::vector<std::string>               & cid_to_filename,
             std::vector<std::map<int, int>>        & pid_to_cid_fid,
             std::vector<Eigen::Vector3d>           & pid_to_xyz,
             std::vector<Eigen::Affine3d>           & world_to_cam,
             std::vector<double>                    & focal_lengths,
             std::map<std::string, Eigen::Vector2d> & offsets) {

  // Read the offsets (optical centers) to apply to the interest points,
  // if applicable.
  std::string offset_path = offsetsFilename(input_filename);
  std::ifstream offset_fh(offset_path.c_str());
  
  bool have_offsets = false;
  offsets.clear();
  
  if (nvm_no_shift) {
    have_offsets = false;
    if (offset_fh.good()) 
      vw::vw_out() << "Ignoring the optical center offsets from: " << offset_path
                   << std::endl;
  } else {
    if (!offset_fh.good())
      vw::vw_throw(vw::ArgumentErr() << "Cannot find optical offsets file: "
                   << offset_path << "\n");
    
    readNvmOffsets(offset_path, offsets);  
    have_offsets = true;
  }
  
  // Read the nvm as is, without applying any offsets
  rig::readNvm(input_filename, cid_to_keypoint_map, cid_to_filename,
               pid_to_cid_fid, pid_to_xyz, world_to_cam, focal_lengths);

  // If no offsets, use zero offsets
  if (nvm_no_shift) {
    for (ptrdiff_t cid = 0; cid < cid_to_filename.size(); cid++)
      offsets[cid_to_filename.at(cid)] = Eigen::Vector2d(0, 0);
  }
 
  // If we have the offsets, apply them to the interest points
  if (have_offsets) {
    for (ptrdiff_t cid = 0; cid < cid_to_filename.size(); cid++) {
      std::string name = cid_to_filename.at(cid);
      auto off_it = offsets.find(name);
      if (off_it == offsets.end())
        vw::vw_throw(vw::ArgumentErr() << "Cannot find optical offset for image: "
                      << name << "\n");
      Eigen::Vector2d offset = off_it->second;
      for (ptrdiff_t fid = 0; fid < cid_to_keypoint_map.at(cid).cols(); fid++) {
        cid_to_keypoint_map.at(cid).col(fid) += offset;
      }
    }
  } // end applying offsets
  
}

// A wrapper to carry fewer things around
void readNvm(std::string const& input_filename, 
             bool nvm_no_shift,
             rig::nvmData & nvm) {
  readNvm(input_filename,
          nvm_no_shift,
          nvm.cid_to_keypoint_map,
          nvm.cid_to_filename,
          nvm.pid_to_cid_fid,
          nvm.pid_to_xyz,
          nvm.world_to_cam,
          nvm.focal_lengths,
          nvm.optical_centers);
}

// A wrapper for writing an nvm file
void writeNvm(rig::nvmData const& nvm, std::string const& output_filename) {
  writeNvm(nvm.cid_to_keypoint_map,
          nvm.cid_to_filename,
          nvm.focal_lengths,
          nvm.pid_to_cid_fid,
          nvm.pid_to_xyz,
          nvm.world_to_cam,
          nvm.optical_centers,
          output_filename);
}

// Convert nvm to cnet
void nvmToCnet(rig::nvmData const& nvm, 
               // Outputs
               vw::ba::ControlNetwork                 & cnet,
               std::map<std::string, Eigen::Vector2d> & offsets,
               std::vector<Eigen::Affine3d>           & world_to_cam) {

  // Wipe the output
  cnet = vw::ba::ControlNetwork("ASP_control_network");

  // Add the images
  for (size_t cid = 0; cid < nvm.cid_to_filename.size(); cid++)
    cnet.add_image_name(nvm.cid_to_filename[cid]);

  // Add the points
  for (size_t pid = 0; pid < nvm.pid_to_cid_fid.size(); pid++) {
    vw::ba::ControlPoint cp;
    Eigen::Vector3d const& P = nvm.pid_to_xyz[pid];
    cp.set_position(vw::Vector3(P[0], P[1], P[2]));
    cp.set_type(vw::ba::ControlPoint::TiePoint); // this is the default

    for (const auto& cid_fid: nvm.pid_to_cid_fid[pid]) {
      int cid = cid_fid.first;
      int fid = cid_fid.second;
      double x = nvm.cid_to_keypoint_map[cid].col(fid)[0];
      double y = nvm.cid_to_keypoint_map[cid].col(fid)[1];
      double sigma = 1.0;
      cp.add_measure(vw::ba::ControlMeasure(x, y, sigma, sigma, cid));
    }
   
    cnet.add_control_point(cp);
  }
  
  offsets = nvm.optical_centers;
  world_to_cam = nvm.world_to_cam;
}

// Create an nvm from a cnet. There is no shift in the interest points. That is
// applied only on loading and saving. Optionally, updated triangulated points
// and outlier flags can be passed in.
void cnetToNvm(vw::ba::ControlNetwork                 const& cnet,
               std::map<std::string, Eigen::Vector2d> const& offsets,
               std::vector<Eigen::Affine3d>           const& world_to_cam,
               // Output
               rig::nvmData & nvm,
               // Optional updated triangulated points and outlier flags
               std::vector<Eigen::Vector3d> const& tri_vec,
               std::set<int> const& outliers) {
               
  // Sanity check
  if (offsets.size() != world_to_cam.size())
    vw::vw_throw(vw::ArgumentErr() << "cnetToNvm: Mismatch in offsets and world_to_cam.\n");
  if (cnet.get_image_list().size() != world_to_cam.size())
    vw::vw_throw(vw::ArgumentErr() << "cnetToNvm: Mismatch in cnet and world_to_cam.\n");  
  
  // If tri_vec is not empty, must have the same size as the cnet
  int num_points = cnet.size();
  if (!tri_vec.empty() && (int)tri_vec.size() != num_points)
    vw::vw_throw(vw::ArgumentErr() << "cnetToNvm: Mismatch in tri_vec and cnet.\n");
    
  // Wipe the output 
  nvm = rig::nvmData();

  nvm.cid_to_filename = cnet.get_image_list();
  nvm.focal_lengths.resize(nvm.cid_to_filename.size(), 1.0); // dummy focal length
  nvm.world_to_cam = world_to_cam;
  nvm.optical_centers = offsets;

  // Allocate storage only for the inliers 
  int num_inliers = num_points - outliers.size();
  nvm.pid_to_xyz.resize(num_inliers);
  nvm.pid_to_cid_fid.resize(num_inliers);
  
  // Copy the triangulated points
  int inlier_count = 0;
  for (int pid = 0; pid < num_points; pid++) {
    
    // Skip outliers
    if (outliers.find(pid) != outliers.end())
      continue;
      
    if (tri_vec.empty()) {
      vw::Vector3 P = cnet[pid].position();
      nvm.pid_to_xyz[inlier_count] = Eigen::Vector3d(P[0], P[1], P[2]);
    } else {
      nvm.pid_to_xyz[inlier_count] = tri_vec[pid];
    }
    inlier_count++;
    
    // Sanity check
    if (inlier_count > num_points)
      vw::vw_throw(vw::ArgumentErr() << "cnetToNvm: Book-keeping failure inlier count.\n");
  }

  // Iterate through the control points and get the feature matches
  // Put all interest points per image in a map, so later we can
  // give them an id.
  int num_images = nvm.cid_to_filename.size();
  typedef std::pair<double, double> Pair; // 2D point with comparison operator
  std::vector<std::map<Pair, int>> keypoint_map(num_images);
  for (int pid = 0; pid < num_points; pid++) {
    
    // Skip outliers
    if (outliers.find(pid) != outliers.end())
      continue;
      
    vw::ba::ControlPoint const& cp = cnet[pid];
    // Iterate through the measures
    for (int m = 0; m < cp.size(); m++) {
      vw::ba::ControlMeasure const& cm = cp[m];
      int cid = cm.image_id();
      vw::Vector2 pix = cm.position();
      
      // Add to the keypoint map
      Pair key = std::make_pair(pix[0], pix[1]);
      // If the key is not in the map, add it with an id that is the current size of the map
      if (keypoint_map[cid].find(key) == keypoint_map[cid].end())
        keypoint_map[cid][key] = keypoint_map[cid].size();
    }
  }

  // Now fill in the keypoint map
  nvm.cid_to_keypoint_map.resize(nvm.cid_to_filename.size());
  for (int cid = 0; cid < num_images; cid++)
    nvm.cid_to_keypoint_map[cid].resize(2, keypoint_map[cid].size());
  
  inlier_count = 0;   
  for (int pid = 0; pid < num_points; pid++) {
    vw::ba::ControlPoint const& cp = cnet[pid];
    
    // Skip outliers
    if (outliers.find(pid) != outliers.end())
      continue;

    // Iterate through the measures
    for (int m = 0; m < cp.size(); m++) {
      vw::ba::ControlMeasure const& cm = cp[m];
      int cid = cm.image_id();
      vw::Vector2 pix = cm.position();
      Pair key = std::make_pair(pix[0], pix[1]);
      // The key is guaranteed to be in the map
      auto it = keypoint_map[cid].find(key);
      if (it == keypoint_map[cid].end())
         vw::vw_throw(vw::ArgumentErr() << "cnetToNvm: Unexpected key not found.\n");
      int fid = it->second;  
      nvm.pid_to_cid_fid[inlier_count][cid] = fid;
      nvm.cid_to_keypoint_map[cid].col(fid) = Eigen::Vector2d(pix[0], pix[1]);
    }
    inlier_count++;
  }  

  return;
}

// Reorder the nvm to agree with a given image list
void remapNvm(std::vector<std::string> const& image_files,
              rig::nvmData & nvm) {

  if (image_files.size() != nvm.cid_to_filename.size())
    vw::vw_throw(vw::ArgumentErr() 
                  << "The nvm and input images do not have the same files.\n");

  // Find the cid for each image in the nvm file
  std::map<std::string, int> nvm_file2cid;
  for (size_t i = 0; i < nvm.cid_to_filename.size(); i++)
    nvm_file2cid[nvm.cid_to_filename[i]] = i;
  // Same for the image files
  std::map<std::string, int> image_file2cid;
  for (size_t i = 0; i < image_files.size(); i++)
    image_file2cid[image_files[i]] = i;

  // These must have the same size
  if (nvm_file2cid.size() != image_file2cid.size())
    vw::vw_throw(vw::ArgumentErr() 
                  << "The nvm and input images do not have the same files.\n");
    
  // Create cid2cid map that will help reorder then nvm to agree with the 
  // image files
  std::map<int, int> cid2cid;
  for (size_t i = 0; i < image_files.size(); i++) {
    std::string image_file = image_files[i];
    if (nvm_file2cid.find(image_file) == nvm_file2cid.end())
      vw::vw_throw(vw::ArgumentErr() << "Cannot find image: " << image_file
                    << " in the nvm.\n");
    cid2cid[nvm_file2cid[image_file]] = i;
  }  

  // Remap the nvm
  rig::remapNvm(cid2cid, nvm.cid_to_keypoint_map, nvm.cid_to_filename,
                nvm.pid_to_cid_fid, nvm.pid_to_xyz, nvm.world_to_cam,
                nvm.focal_lengths, nvm.optical_centers);
}

// Read an NVM file into the VisionWorkbench control network format. The flag
// nvm_no_shift, if true, means that the interest points are not shifted
// relative to the optical center, so can be read as is.
void readNvmAsCnet(std::string const& input_filename, 
                   std::vector<std::string> const& image_files,
                   bool nvm_no_shift,
                   vw::ba::ControlNetwork & cnet,
                   std::vector<Eigen::Affine3d> & world_to_cam,
                   std::map<std::string, Eigen::Vector2d> & optical_offsets) {

  // Read the NVM file
  rig::nvmData nvm;
  rig::readNvm(input_filename, nvm_no_shift, nvm);

  // Must ensure the nvm image list agrees with the input image list
  if (!image_files.empty()) 
    remapNvm(image_files, nvm);
  
  // Convert to a control network
  rig::nvmToCnet(nvm, cnet, optical_offsets, world_to_cam);
}

// Write a cnet to an NVM file. On writing, the feature matches from the cnet will be
// shifted relative to the optical center. The optical center offsets are saved
// to a separate file.
void writeCnetAsNvm(vw::ba::ControlNetwork const& cnet,
                    std::map<std::string, Eigen::Vector2d> const& optical_offsets,
                    std::vector<Eigen::Affine3d> const& world_to_cam,
                    std::string const& output_filename) {

  // Convert to an nvm
  rig::nvmData nvm;
  rig::cnetToNvm(cnet, optical_offsets, world_to_cam, nvm);

  // Write the nvm
  rig::writeNvm(nvm, output_filename);
}
  
}  // end namespace rig
