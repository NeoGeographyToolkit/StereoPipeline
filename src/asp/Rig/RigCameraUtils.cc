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

#include <asp/Rig/RigCameraUtils.h>
#include <asp/Rig/RigCameraParams.h>
#include <asp/Rig/camera_image.h>
#include <Rig/system_utils.h>

#include <vw/Core/Exception.h>
#include <vw/Core/Log.h>

#include <boost/filesystem.hpp>
#include <glog/logging.h>

#include <Eigen/Dense>

namespace fs = boost::filesystem;

namespace camera {

// Write a line of the form: name = a b c
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

// For an image like image_dir/my_cam/image.png, create the file
// out_dir/image.tsai.
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
 

// A utility for saving a camera in a format ASP understands. 
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
    rig::prepend_1(den_x);
    rig::prepend_1(den_y);
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
    std::string camFile = pinholeFile(pinhole_dir, sensor_name, image_file);
    camera::writePinholeCamera(cam_params[cam_type], world_to_cam[cid], camFile);
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

} // end namespace camera