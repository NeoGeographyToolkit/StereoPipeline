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
#include <Rig/nvm.h>
#include <vw/Core/Exception.h>
#include <vw/Core/Log.h>
#include <vw/FileIO/FileUtils.h>
#include <vw/BundleAdjustment/ControlNetwork.h>

#include <fstream>
#include <iostream>

namespace asp {

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

// A function to create the offsets filename from the nvm filename
std::string offsetsFilename(std::string const& nvm_filename) {
  int file_len = nvm_filename.size(); // cast to int to make subtraction safe
  // The length must be at least 5, as it must end with .nvm
  if (file_len < 5) 
    vw::vw_throw(vw::ArgumentErr() << "Invalid nvm filename: " << nvm_filename << ".\n");
  return nvm_filename.substr(0, std::max(file_len - 4, 0)) + "_offsets.txt";
}

// Reads the NVM control network format. The interest points may or may not
// be shifted relative to optical center. The user is responsible for knowing that.
// If a filename having extension _offset.txt instead of .nvm exists, read
// from it the optical center offsets and apply them. So, the interest points
// that are read in have the offset applied.
void readNvm(std::string                       const& input_filename,
             bool                                     nvm_no_shift,
             std::vector<Eigen::Matrix2Xd>          & cid_to_keypoint_map,
             std::vector<std::string>               & cid_to_filename,
             std::vector<std::map<int, int>>        & pid_to_cid_fid,
             std::vector<Eigen::Vector3d>           & pid_to_xyz,
             std::vector<Eigen::Affine3d>           & world_to_cam,
             std::vector<double>                    & focal_lengths,
             std::map<std::string, Eigen::Vector2d> & offsets) {

  vw::vw_out() << "Reading: " << input_filename << "\n";

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

      // Apply the optical center offset if it exists
      if (have_offsets) {
        auto map_it = offsets.find(cid_to_filename.at(cid));
        if (map_it == offsets.end()) {
          vw::vw_throw(vw::ArgumentErr() << "Cannot find optical offset for image "
                       << cid_to_filename.at(cid) << "\n");
        }
        pt[0] += (map_it->second)[0];
        pt[1] += (map_it->second)[1];
      }
      
      pid_to_cid_fid.at(pid)[cid] = fid;

      if (cid_to_keypoint_map.at(cid).cols() <= fid)
        cid_to_keypoint_map.at(cid).conservativeResize(Eigen::NoChange_t(), fid + 1);
      
      cid_to_keypoint_map.at(cid).col(fid) = pt;
    }

    if (!f.good())
      vw::vw_throw(vw::ArgumentErr() << "Unable to correctly read PID: " << pid);
  }

 // If no offsets, use zero offsets
 if (nvm_no_shift) {
    for (ptrdiff_t cid = 0; cid < number_of_cid; cid++)
      offsets[cid_to_filename.at(cid)] = Eigen::Vector2d(0, 0);
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
    
      auto cid = it->first;
      auto fid = it->second;
    
      // Find the offset for this image
      auto map_it = optical_centers.find(cid_to_filename[cid]);
      if (map_it == optical_centers.end()) {
        vw::vw_throw(vw::ArgumentErr() << "Cannot find optical offset for image "
                     << cid_to_filename[cid] << "\n");
      }
      Eigen::Vector2d offset = map_it->second;
      
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
  
  // Write the optical center offsets
  std::string offset_path = offsetsFilename(output_filename);
  writeNvmOffsets(offset_path, optical_centers);
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
                nvm.optical_centers);
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
  asp::readNvm(input_filename, nvm_no_shift, nvm);

  // Must ensure the nvm image list agrees with the input image list
  if (!image_files.empty()) 
    remapNvm(image_files, nvm);
  
  // Convert to a control network
  asp::nvmToCnet(nvm, cnet, optical_offsets, world_to_cam);
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
  asp::cnetToNvm(cnet, optical_offsets, world_to_cam, nvm);

  // Write the nvm
  asp::writeNvm(nvm, output_filename);
}

} // end namespace asp
