// __BEGIN_LICENSE__
//  Copyright (c) 2009-2026, United States Government as represented by the
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

#include <asp/Rig/rig_io.h>
#include <asp/Rig/RigImageIO.h>
#include <asp/Rig/RigParseUtils.h>
#include <asp/Rig/detector.h>
#include <asp/Rig/camera_image.h>
#include <asp/Rig/system_utils.h>
#include <asp/Rig/basic_algs.h>
#include <asp/Rig/rig_config.h>
#include <asp/Rig/RigCameraParams.h>
#include <asp/Rig/interpolation_utils.h>
#include <asp/Rig/transform_utils.h>
#include <asp/Core/nvm.h>

#include <vw/InterestPoint/MatcherIO.h>
#include <vw/Core/Log.h>

#include <boost/filesystem.hpp>
#include <opencv2/imgcodecs.hpp>
#include <glog/logging.h>

#include <fstream>
#include <iostream>
#include <iomanip>
#include <string>
#include <vector>
#include <set>

namespace fs = boost::filesystem;

namespace rig {

// For each image, find its sensor name and timestamp. The info can be in a list or
// from the file or directory structure. If flexible_strategy is true, then 
// can try from list first, and if that fails, then from file/directory structure.
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
  
  // Clear the outputs
  cam_types.clear(); cam_types.resize(image_files.size());
  timestamps.clear(); timestamps.resize(image_files.size());
  
  for (size_t it = 0; it < image_files.size(); it++) {
    int cam_type = 0;
    double timestamp = 0.0;
    try {
      std::string group; // not used 
      findCamTypeAndTimestamp(image_files[it], cam_names,  
                              cam_type, timestamp, group); // outputs
    } catch (std::exception const& e) {
        LOG(FATAL) << "Could not infer sensor type and image timestamp. See the naming "
                   << "convention, and check your images. Detailed message:\n" << e.what();
    }
    
    cam_types[it] = cam_type;
    timestamps[it] = timestamp;
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
                    std::vector<Eigen::Affine3d> & world_to_cam) {

  // Put the existing poses in a map
  std::map<int, std::map<double, Eigen::Affine3d>> existing_world_to_cam;
  std::set<std::string> existing_images;

  for (size_t image_it = 0; image_it < cid_to_filename.size(); image_it++) {
    auto const& image_file = cid_to_filename[image_it];
    existing_images.insert(image_file); 
    int cam_type = cam_types[image_it];
    double timestamp = timestamps[image_it];
    Eigen::Affine3d world2cam = world_to_cam[image_it];
    existing_world_to_cam[cam_type][timestamp] = world2cam;

    if (use_initial_rig_transforms) {
      // Use the rig constraint to find the poses for the other sensors on the rig
      // First go to the ref sensor
      double ref_timestamp = timestamp - R.ref_to_cam_timestamp_offsets[cam_type];

      // Careful here with transform directions and order
      Eigen::Affine3d cam_to_ref = R.ref_to_cam_trans[cam_type].inverse();
      Eigen::Affine3d world_to_ref = cam_to_ref * world2cam;

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
    auto & input_map = existing_world_to_cam[cam_type]; // alias
    if (input_map.empty()) {
      std::string msg = std::string("Cannot find camera poses for sensor: ")
        + R.cam_names[cam_type] + " as the data is insufficient.\n";
      if (!use_initial_rig_transforms) 
        msg += std::string("If the rig configuration file has an initial rig, consider ")
          + "using the option --use_initial_rig_transforms.\n";
      vw::vw_out() << msg;
      continue;
    }

    std::vector<std::string> found_images;
    std::vector<Eigen::Affine3d> found_poses;
    interpOrExtrap(input_map, target_map, bracket_len, nearest_neighbor_interp,
                   found_images, found_poses); // outputs

    for (size_t found_it = 0; found_it < found_images.size(); found_it++) {
      cid_to_filename.push_back(found_images[found_it]);
      world_to_cam.push_back(found_poses[found_it]);
      
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
                     asp::nvmData & nvm) {
  
  // Clear the outputs
  nvm = asp::nvmData();

  // Open the file
  vw::vw_out() << "Reading: " << camera_poses_file << std::endl;
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
    
    Eigen::Affine3d world2cam = vecToAffine(vals);
    nvm.world_to_cam.push_back(world2cam);
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
                   asp::nvmData & nvm,
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
    asp::readNvm(nvm_file, 
                 nvm.cid_to_keypoint_map,  
                 nvm.cid_to_filename,  
                 nvm.pid_to_cid_fid,  
                 nvm.pid_to_xyz,  
                 nvm.world_to_cam,
                 nvm.focal_lengths);
    if (!read_nvm_no_shift) {
      std::string offsets_file = asp::offsetsFilename(nvm_file);
      asp::readNvmOffsets(offsets_file, nvm.optical_centers); 
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
                   nvm.cid_to_filename, cam_types, timestamps, nvm.world_to_cam);

  vw::vw_out() << "Reading the images.\n";
  for (size_t it = 0; it < nvm.cid_to_filename.size(); it++) {
    // Aliases
    auto const& image_file = nvm.cid_to_filename[it];
    auto const& world2cam = nvm.world_to_cam[it];
    readImageEntry(image_file, world2cam, R.cam_names,  
                   cam_types[it], timestamps[it],
                   // Outputs
                   image_maps, depth_maps);
  }

  return;
}

// Form the match file name using the ASP convention.
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
  
// Given all the merged and filtered tracks in pid_cid_fid, for each
// image pair cid1 and cid2 with cid1 < cid2 < cid1 + num_overlaps + 1,
// save the matches of this pair which occur in the set of tracks.
void saveInlierMatchPairs(// Inputs
                          std::vector<rig::cameraImage> const& cams,
                          int num_overlaps,
                          rig::PidCidFid                const& pid_to_cid_fid,
                          rig::KeypointVec              const& keypoint_vec,
                          PidCidFidMap                  const& pid_cid_fid_inlier,
                          std::string                   const& out_dir) {

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

        vw::ip::InterestPoint ip1(keypoint_vec[cid1][fid1].first, keypoint_vec[cid1][fid1].second);
        vw::ip::InterestPoint ip2(keypoint_vec[cid2][fid2].first, keypoint_vec[cid2][fid2].second);

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
    vw::ip::write_binary_match_file(match_file, match_pair.first, match_pair.second);
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
  
} // end function saveInlierMatchPairs

// Find convergence angles between every pair of images and save to disk their
// percentiles assumed that the cameras in world_to_cam are up-to-date given the
// current state of optimization, and that the residuals (including the
// reprojection errors) have also been updated beforehand.
void savePairwiseConvergenceAngles(// Inputs
  rig::PidCidFid                const& pid_to_cid_fid,
  rig::KeypointVec              const& keypoint_vec,
  std::vector<rig::cameraImage> const& cams,
  std::vector<Eigen::Affine3d>  const& world_to_cam,
  std::vector<Eigen::Vector3d>  const& xyz_vec,
  PidCidFidMap                  const& pid_cid_fid_inlier,
  std::string                   const& conv_angles_file) {

  rig::PairwiseConvergenceAngles conv_angles;
  
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
  ofs << "# left_image right_image 25% 50% 75% num_matches\n";
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
} // end function savePairwiseConvergenceAngles
  
}  // namespace rig

