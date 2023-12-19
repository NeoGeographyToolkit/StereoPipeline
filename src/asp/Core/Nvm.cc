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

#include <asp/Core/Nvm.h>
#include <vw/Core/Exception.h>
#include <vw/Core/Log.h>
#include <vw/FileIO/FileUtils.h>
#include <vw/BundleAdjustment/ControlNetwork.h>

#include <fstream>
#include <iostream>

namespace asp {

// A wrapper to carry fewer things around
void ReadNVM(std::string const& input_filename, 
             bool nvm_no_shift,
             nvmData & nvm) {
  ReadNVM(input_filename,
          nvm_no_shift,
          &nvm.cid_to_keypoint_map,
          &nvm.cid_to_filename,
          &nvm.pid_to_cid_fid,
          &nvm.pid_to_xyz,
          &nvm.cid_to_cam_t_global);
}
  
// Reads the NVM control network format. The interest points may or may not
// be shifted relative to optical center. The user is responsible for knowing that.
// If a filename having extension _offset.txt instead of .nvm exists, read
// from it the optical center offsets and apply them.
void ReadNVM(std::string const& input_filename,
             bool nvm_no_shift,
             std::vector<Eigen::Matrix2Xd> * cid_to_keypoint_map,
             std::vector<std::string> * cid_to_filename,
             std::vector<std::map<int, int>> * pid_to_cid_fid,
             std::vector<Eigen::Vector3d> * pid_to_xyz,
             std::vector<Eigen::Affine3d> * cid_to_cam_t_global) {

  // Read the offsets (optical centers) to apply to the interest points,
  // if applicable.
  int file_len = input_filename.size(); // cast to int to make subtraction safe
  std::string offset_path 
    = input_filename.substr(0, std::max(file_len - 4, 0)) + "_offsets.txt";
  std::ifstream offset_fh(offset_path.c_str());
  
  bool have_offsets = false;
  std::map<std::string, Eigen::Vector2d> offsets;
  if (nvm_no_shift) {
    have_offsets = false;
    if (offset_fh.good()) 
      vw::vw_out() << "When reading " << input_filename << ", "
                   << "ignoring the optical center offsets from: " << offset_path
                   << std::endl;
  } else {
    if (!offset_fh.good())
      vw::vw_throw(vw::ArgumentErr() << "Cannot find optical offsets file: "
                   << offset_path << ". Consider reading the nvm file with "
                   << "the no-shift option or specify the offsets.\n");
      
    vw::vw_out() << "Read and apply optical offsets from: " << offset_path << std::endl;
    
    std::string name;
    double x, y;
    while (offset_fh >> name >> x >> y)
      offsets[name] = Eigen::Vector2d(x, y);
    
    have_offsets = true;
  }
  
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
  cid_to_keypoint_map->resize(number_of_cid);
  cid_to_filename->resize(number_of_cid);
  cid_to_cam_t_global->resize(number_of_cid);
  for (ptrdiff_t cid = 0; cid < number_of_cid; cid++) {
    // Clear keypoints from map. We'll read these in shortly
    cid_to_keypoint_map->at(cid).resize(Eigen::NoChange_t(), 2);

    // Read the line that contains camera information
    double focal, dist1, dist2;
    Eigen::Quaterniond q;
    Eigen::Vector3d c;
    f >> token >> focal;
    f >> q.w() >> q.x() >> q.y() >> q.z();
    f >> c[0] >> c[1] >> c[2] >> dist1 >> dist2;
    cid_to_filename->at(cid) = token;

    // Solve for t, which is part of the affine transform
    Eigen::Matrix3d r = q.matrix();
    cid_to_cam_t_global->at(cid).linear() = r;
    cid_to_cam_t_global->at(cid).translation() = -r * c;
  }

  // Read the number of points
  ptrdiff_t number_of_pid;
  f >> number_of_pid;
  if (number_of_pid < 1)
    vw::vw_throw(vw::ArgumentErr() << "The NVM file has no triangulated points.");

  // Read the point
  pid_to_cid_fid->resize(number_of_pid);
  pid_to_xyz->resize(number_of_pid);
  Eigen::Vector3d xyz;
  Eigen::Vector3i color;
  Eigen::Vector2d pt;
  ptrdiff_t cid, fid;
  for (ptrdiff_t pid = 0; pid < number_of_pid; pid++) {
    pid_to_cid_fid->at(pid).clear();

    ptrdiff_t number_of_measures;
    f >> xyz[0] >> xyz[1] >> xyz[2] >>
      color[0] >> color[1] >> color[2] >> number_of_measures;
    pid_to_xyz->at(pid) = xyz;
    for (ptrdiff_t m = 0; m < number_of_measures; m++) {
      f >> cid >> fid >> pt[0] >> pt[1];

      // Apply the optical center offset if it exists
      if (have_offsets) {
        auto map_it = offsets.find(cid_to_filename->at(cid));
        if (map_it == offsets.end()) {
          vw::vw_throw(vw::ArgumentErr() << "Cannot find optical offset for image "
                       << cid_to_filename->at(cid) << "\n");
        }
        pt[0] += (map_it->second)[0];
        pt[1] += (map_it->second)[1];
      }
      
      pid_to_cid_fid->at(pid)[cid] = fid;

      if (cid_to_keypoint_map->at(cid).cols() <= fid)
        cid_to_keypoint_map->at(cid).conservativeResize(Eigen::NoChange_t(), fid + 1);
      
      cid_to_keypoint_map->at(cid).col(fid) = pt;
    }

    if (!f.good())
      vw::vw_throw(vw::ArgumentErr() << "Unable to correctly read PID: " << pid);
  }
}

// Write an nvm file. Note that a single focal length is assumed and no distortion.
// Those are ignored, and only camera poses, matches, and keypoints are used.
// Features are written as is, without shifting them relative to the optical center.
void WriteNVM(std::vector<Eigen::Matrix2Xd> const& cid_to_keypoint_map,
              std::vector<std::string> const& cid_to_filename,
              std::vector<double> const& focal_lengths,
              std::vector<std::map<int, int>> const& pid_to_cid_fid,
              std::vector<Eigen::Vector3d> const& pid_to_xyz,
              std::vector<Eigen::Affine3d> const& cid_to_cam_t_global,
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
    vw::vw_throw(vw::ArgumentErr() << "Unequal number of pid_to_cid_fid and xyz measurements.");
  if (cid_to_filename.size() != cid_to_cam_t_global.size())
    vw::vw_throw(vw::ArgumentErr() << "Unequal number of filename and camera transforms.");
  
  // Write camera information
  f << cid_to_filename.size() << std::endl;
  for (size_t cid = 0; cid < cid_to_filename.size(); cid++) {

    // World-to-camera rotation quaternion
    Eigen::Quaterniond q(cid_to_cam_t_global[cid].rotation());

    // Camera center in world coordinates
    Eigen::Vector3d t(cid_to_cam_t_global[cid].translation());
    Eigen::Vector3d camera_center =
      - cid_to_cam_t_global[cid].rotation().inverse() * t;

    f << cid_to_filename[cid] << " " << focal_lengths[cid]
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
      f << " " << it->first << " " << it->second << " "
        << cid_to_keypoint_map[it->first].col(it->second)[0] << " "
        << cid_to_keypoint_map[it->first].col(it->second)[1];
    }
    f << std::endl;
  }

  // Close the file
  f.flush();
  f.close();
}

// Read an NVM file into the VisionWorkbench control network format. The flag
// nvm_no_shift, if true, means that the interest points are not shifted
// relative to the optical center, so can be read as is.
void readNvmAsCnet(std::string const& input_filename, 
                   bool nvm_no_shift,
                   vw::ba::ControlNetwork & cnet) {

  // Wipe the output
  cnet = vw::ba::ControlNetwork("ASP_control_network");
  
  // Read the NVM file
  nvmData nvm;
  ReadNVM(input_filename, nvm_no_shift, nvm);

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
}

} // end namespace asp
