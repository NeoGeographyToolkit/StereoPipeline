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

// Validate that bundle_adjust rotates a velocity-mode optical bar camera
// rigidly. The pose at the sweep start and at the sweep end must turn together,
// so each ray turns by the same rotation and the camera centers do not move.

#include <asp/Core/BaState.h>
#include <asp/Camera/BundleAdjustCostFuns.h>
#include <asp/Camera/BundleAdjustParamMap.h>
#include <asp/Camera/BundleAdjustTransform.h>

#include <vw/Camera/OpticalBarModel.h>
#include <vw/Cartography/BathyData.h>
#include <vw/Math/Quaternion.h>
#include <vw/Math/Vector.h>

#include <test/Helpers.h>

#include <boost/make_shared.hpp>

#include <vector>
#include <algorithm>

namespace asp {

// A velocity-mode camera with roughly a 90 degree sweep, looking down from
// orbit, with a sweep end pose that differs from the start one.
vw::OpticalBarModel makeOpticalBarCam(bool scan_left_to_right) {

  vw::Vector2i image_size(2400, 300);
  vw::Vector2 center(1200, 150);
  vw::Vector2 pitch(1e-3, 1e-3);
  double focal_length = 1.5;
  double scan_time = 1.0;
  double forward_tilt = 0.0;
  double speed = 0.0;
  double mcf = 0.005;
  bool have_velocity_vec = true;
  vw::Vector3 position(6.54e6, 1.0e4, -2.0e4);
  vw::Vector3 velocity(30.0, 4500.0, -200.0);

  // Camera z axis about along -x, so the camera looks at the Earth
  vw::Vector3 init_pose(0.01, -M_PI/2.0 + 0.02, 0.03);

  // A sweep end pose differing by about 0.3 degrees
  vw::Quat sweep = vw::axis_angle_to_quaternion(vw::Vector3(0.004, -0.002, 0.003));
  vw::Quat init_quat = vw::axis_angle_to_quaternion(init_pose);
  vw::Vector3 final_pose = (init_quat * sweep).axis_angle();

  return vw::OpticalBarModel(image_size, center, pitch, focal_length, scan_time,
                             scan_left_to_right, forward_tilt, position, init_pose,
                             speed, mcf, have_velocity_vec, velocity, final_pose);
}

// Pixels spanning the image, including the corners and the sweep ends
std::vector<vw::Vector2> samplePixels(vw::OpticalBarModel const& cam) {
  vw::Vector2i sz = cam.get_image_size();
  std::vector<vw::Vector2> pixels;
  int num = 7;
  for (int i = 0; i < num; i++) {
    for (int j = 0; j < num; j++) {
      double x = (sz[0] - 1.0) * i / (num - 1.0);
      double y = (sz[1] - 1.0) * j / (num - 1.0);
      pixels.push_back(vw::Vector2(x, y));
    }
  }
  return pixels;
}

// A rotation of about 0.6 degrees about a generic axis, as a solver would apply
vw::Quat testRotation() {
  return vw::axis_angle_to_quaternion(vw::Vector3(0.006, -0.004, 0.008));
}

// Largest angle, in radians, between the rays of the first camera after
// rotation and the rays of the second camera. Also find the largest
// difference between the camera centers.
void compareCams(vw::OpticalBarModel const& cam, vw::Quat const& rot,
                 vw::OpticalBarModel const& out_cam,
                 // Outputs
                 double & max_angle, double & max_ctr_diff) {
  max_angle = 0.0;
  max_ctr_diff = 0.0;
  vw::Matrix3x3 R = rot.rotation_matrix();
  std::vector<vw::Vector2> pixels = samplePixels(cam);
  for (size_t i = 0; i < pixels.size(); i++) {
    vw::Vector3 expected_dir = R * cam.pixel_to_vector(pixels[i]);
    vw::Vector3 dir = out_cam.pixel_to_vector(pixels[i]);
    double angle = vw::norm_2(vw::cross_prod(expected_dir, dir));
    max_angle = std::max(max_angle, angle);
    double ctr_diff = vw::norm_2(cam.camera_center(pixels[i]) -
                                       out_cam.camera_center(pixels[i]));
    max_ctr_diff = std::max(max_ctr_diff, ctr_diff);
  }
}

// Pack the camera into a bundle adjustment state, then rotate its sweep start
// pose there, as the solver does.
asp::BaState rotatedState(vw::OpticalBarModel const& cam, vw::Quat const& rot) {
  int num_points = 1;
  int num_cameras = 1;
  bool using_intrinsics = true;
  asp::IntrinsicOptions intr_opts;
  asp::BaState ba_state(num_points, num_cameras, using_intrinsics,
                        asp::NUM_OPTICAL_BAR_EXTRA_PARAMS, intr_opts);
  int icam = 0;
  asp::pack_optical_bar_to_arrays(cam, icam, ba_state);

  double* pos_pose_ptr = ba_state.get_camera_ptr(icam);
  vw::Vector3 new_pose = (rot * cam.camera_pose()).axis_angle();
  for (int i = 0; i < 3; i++)
    pos_pose_ptr[3 + i] = new_pose[i];

  return ba_state;
}

// The no-argument pose must be the sweep start, also for a right to left
// scan, where pixel (0, 0) is the sweep end.
TEST(OpticalBarRigid, CameraPoseIsSweepStart) {
  for (int s = 0; s < 2; s++) {
    bool scan_left_to_right = (s == 0);
    vw::OpticalBarModel cam = makeOpticalBarCam(scan_left_to_right);
    vw::Vector3 init_pose(0.01, -M_PI/2.0 + 0.02, 0.03);
    vw::Vector3 diff = cam.camera_pose().axis_angle() - init_pose;
    EXPECT_LT(vw::norm_2(diff), 1e-12);

    // For a right to left scan, the pose at pixel (0, 0) is not the start one
    if (!scan_left_to_right) {
      vw::Vector3 end_diff = cam.camera_pose(vw::Vector2(0, 0)).axis_angle() - init_pose;
      EXPECT_GT(vw::norm_2(end_diff), 1e-3);
    }
  }
}

// A camera saved and read back gives the same rays, for both scan directions
TEST(OpticalBarRigid, WriteReadRoundTrip) {
  for (int s = 0; s < 2; s++) {
    bool scan_left_to_right = (s == 0);
    vw::OpticalBarModel cam = makeOpticalBarCam(scan_left_to_right);
    vw::test::UnlinkName file("optical_bar_rigid.tsai");
    cam.write(file);
    vw::OpticalBarModel cam2(file);
    double max_angle = 0.0, max_ctr_diff = 0.0;
    vw::Quat identity(1, 0, 0, 0);
    compareCams(cam, identity, cam2, max_angle, max_ctr_diff);
    EXPECT_LT(max_angle, 1e-9);
    EXPECT_LT(max_ctr_diff, 1e-3);
  }
}

// The camera bundle_adjust writes out turns rigidly
TEST(OpticalBarRigid, TransformedCameraIsRigid) {
  vw::Quat rot = testRotation();
  for (int s = 0; s < 2; s++) {
    bool scan_left_to_right = (s == 0);
    vw::OpticalBarModel cam = makeOpticalBarCam(scan_left_to_right);
    asp::BaState ba_state = rotatedState(cam, rot);
    int icam = 0;
    vw::OpticalBarModel out_cam = asp::transformedOpticalBarCamera(icam, ba_state, cam);

    double max_angle = 0.0, max_ctr_diff = 0.0;
    compareCams(cam, rot, out_cam, max_angle, max_ctr_diff);
    EXPECT_LT(max_angle, 1e-12);
    EXPECT_LT(max_ctr_diff, 1e-6);

    // The test has teeth: turning only the sweep start, with the end pose held,
    // is not rigid and is off by much more.
    vw::OpticalBarModel start_only = cam;
    start_only.set_camera_pose(rot * cam.camera_pose());
    compareCams(cam, rot, start_only, max_angle, max_ctr_diff);
    EXPECT_GT(max_angle, 1e-3);
  }
}

// The cost function sees the same rigidly rotated camera. A point placed along
// a rotated ray must project back to its pixel.
TEST(OpticalBarRigid, CostFunctionIsRigid) {
  vw::Quat rot = testRotation();
  vw::Matrix3x3 R = rot.rotation_matrix();
  vw::BathyData bathy_data;
  for (int s = 0; s < 2; s++) {
    bool scan_left_to_right = (s == 0);
    vw::OpticalBarModel cam = makeOpticalBarCam(scan_left_to_right);
    asp::BaState ba_state = rotatedState(cam, rot);
    int icam = 0;
    boost::shared_ptr<vw::OpticalBarModel> cam_ptr
      = boost::make_shared<vw::OpticalBarModel>(cam);
    asp::BaOpticalBarCam ba_cam(cam_ptr, bathy_data, icam);

    double range = 2.0e5;
    double max_pix_diff = 0.0;
    std::vector<vw::Vector2> pixels = samplePixels(cam);
    for (size_t i = 0; i < pixels.size(); i++) {
      vw::Vector3 dir = R * cam.pixel_to_vector(pixels[i]);
      vw::Vector3 pt = cam.camera_center(pixels[i]) + range * dir;
      double point[3] = {pt[0], pt[1], pt[2]};
      std::vector<double const*> param_blocks;
      param_blocks.push_back(point);
      param_blocks.push_back(ba_state.get_camera_ptr(icam));
      param_blocks.push_back(ba_state.get_intrinsic_center_ptr(icam));
      param_blocks.push_back(ba_state.get_intrinsic_focus_ptr(icam));
      param_blocks.push_back(ba_state.get_intrinsic_distortion_ptr(icam));
      vw::Vector2 pix = ba_cam.evaluate(param_blocks);
      max_pix_diff = std::max(max_pix_diff, vw::norm_2(pix - pixels[i]));
    }
    EXPECT_LT(max_pix_diff, 1e-3);
  }
}

} // end namespace asp
