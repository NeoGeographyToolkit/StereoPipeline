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


/// \file BundleAdjustCamera.cc
///

#include <asp/Camera/BundleAdjustCamera.h>

#include <string>

// TODO(oalexan1): Move most of BundleAdjustCamera.h code to here, and put it
// all in the asp namespace.

// Given an input pinhole camera and param changes, apply those, returning
// the new camera. Note that all intrinsic parameters are stored as multipliers
// in BAParamStorage.
vw::camera::PinholeModel transformedPinholeCamera(int camera_index,
                                                  BAParamStorage const& param_storage,
                                                  vw::camera::PinholeModel const& in_cam) {

  // Start by making a copy of the camera. Note that this does not make a copy of the
  // distortion params, as that's a pointer. So will have to make a copy of it further down.
  vw::camera::PinholeModel out_cam = in_cam;

  double const* pos_pose_ptr   = param_storage.get_camera_ptr(camera_index);
  double const* center_ptr     = param_storage.get_intrinsic_center_ptr    (camera_index);
  double const* focus_ptr      = param_storage.get_intrinsic_focus_ptr     (camera_index);
  double const* distortion_ptr = param_storage.get_intrinsic_distortion_ptr(camera_index);

  // Update position and pose
  CameraAdjustment pos_pose_info(pos_pose_ptr);
  out_cam.set_camera_center(pos_pose_info.position());
  out_cam.set_camera_pose  (pos_pose_info.pose    ());

  // Update the lens distortion parameters. Note how we make a new copy of the distortion object.
  boost::shared_ptr<LensDistortion> distortion = out_cam.lens_distortion()->copy();
  vw::Vector<double> lens = distortion->distortion_parameters();
  for (size_t i=0; i<lens.size(); ++i)
    lens[i] *= distortion_ptr[i];
  distortion->set_distortion_parameters(lens);
  out_cam.set_lens_distortion(distortion.get());

  // Update the center and focus
  Vector2 old_center = out_cam.point_offset();
  Vector2 old_focus  = out_cam.focal_length();
  out_cam.set_point_offset(Vector2(center_ptr[0]*old_center[0],
                                  center_ptr[1]*old_center[1]), false);
  double new_focus = old_focus[0]*focus_ptr[0];
  out_cam.set_focal_length(Vector2(new_focus,new_focus), true); // Recompute internals.
  
  return out_cam;
}

// Given an input optical bar camera and param changes, apply those, returning
// the new camera.
vw::camera::OpticalBarModel transformedOpticalBarCamera(int camera_index,
                                                        BAParamStorage const& param_storage,
                                                        vw::camera::OpticalBarModel const& in_cam) {
  
  // Start by making a copy of the camera.
  vw::camera::OpticalBarModel out_cam = in_cam;

  double const* pos_pose_ptr  = param_storage.get_camera_ptr(camera_index);
  double const* center_ptr    = param_storage.get_intrinsic_center_ptr    (camera_index);
  double const* focus_ptr     = param_storage.get_intrinsic_focus_ptr     (camera_index);
  double const* intrinsic_ptr = param_storage.get_intrinsic_distortion_ptr(camera_index);

  // Update position and pose
  CameraAdjustment pos_pose_info(pos_pose_ptr);
  out_cam.set_camera_center(pos_pose_info.position());
  out_cam.set_camera_pose  (pos_pose_info.pose    ());

  // All intrinsic parameters are stored as multipliers!

  // Update the other intrinsic parameters.
  out_cam.set_speed              (out_cam.get_speed()*intrinsic_ptr[0]);
  out_cam.set_motion_compensation(out_cam.get_motion_compensation()*intrinsic_ptr[1]);
  out_cam.set_scan_time          (out_cam.get_scan_time()*intrinsic_ptr[2]);

  // Update the center and focus
  Vector2 old_center = out_cam.get_optical_center();
  float   old_focus  = out_cam.get_focal_length();
  out_cam.set_optical_center(Vector2(center_ptr[0]*old_center[0],
                                    center_ptr[1]*old_center[1]));
  double new_focus = old_focus*focus_ptr[0];
  out_cam.set_focal_length(new_focus);

  return out_cam;
}


// Save convergence angle percentiles for each image pair having matches
void saveConvergenceAngles(std::string const& conv_angles_file,
                           std::vector<asp::convAngle> const& convAngles,
                           std::vector<std::string> const& imageFiles) {

  vw_out() << "Writing: " << conv_angles_file << "\n";
  std::ofstream ofs (conv_angles_file.c_str());
  ofs << " # Convergence angle percentiles (in degrees) for each image pair having matches\n";
  ofs << " # left_image right_image 25% 50% 75% num_angles_per_pair\n";
  ofs.precision(17);
  for (size_t conv_it = 0; conv_it < convAngles.size(); conv_it++) {
    auto const & c = convAngles[conv_it]; // alias
    ofs << imageFiles[c.left_cam_index] << ' ' << imageFiles[c.right_cam_index] << ' '
        << c.angle25 << ' ' << c.angle50 << ' '  << c.angle75 << ' ' << c.num_angles << "\n";
  }
  ofs.close();
  
}
