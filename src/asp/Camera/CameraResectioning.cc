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

/// \file CameraResectioning.cc

#include <asp/Camera/CameraResectioning.h>

#include <opencv2/calib3d/calib3d.hpp>
#include <Eigen/Dense>

#include <string>

using namespace vw;
using namespace vw::camera;

namespace asp {

void RodriguesToRotation(Eigen::Vector3d const& vector,
                         Eigen::Matrix3d & rotation) {
  double angle = vector.norm();
  Eigen::AngleAxisd aa(angle, vector / angle);
  rotation = aa.matrix();
}
  
/// Use OpenCV to find a Pinhole camera's position and orientation
/// based on image pixels and corresponding ground positions
void findCameraPose(std::vector<vw::Vector3> const& ground_points, 
                    std::vector<vw::Vector2> const& pixel_observations,
                    vw::camera::PinholeModel & cam) {

  if (ground_points.size() != pixel_observations.size())
    vw::vw_throw(vw::ArgumentErr()
                 << "There must be as many ground points as pixel observations.\n");

  if (ground_points.size() < 4) 
    vw::vw_throw(vw::ArgumentErr()
                 << "Must have at least four points per camera to be able to "
                 << "find a camera's orientation.\n");

  Vector2 focal_length   = cam.focal_length();
  double pixel_pitch     = cam.pixel_pitch();
  Vector2 optical_offset = cam.point_offset();

  // Intrinsics
  cv::Mat intrinsics(3, 3, cv::DataType<double>::type, 0.0);
  intrinsics.at<double>(0, 0) = focal_length[0]   / pixel_pitch;
  intrinsics.at<double>(1, 1) = focal_length[1]   / pixel_pitch;
  intrinsics.at<double>(0, 2) = optical_offset[0] / pixel_pitch;
  intrinsics.at<double>(1, 2) = optical_offset[1] / pixel_pitch;
  intrinsics.at<double>(2, 2) = 1.0;

  // Assume no distortion, as that one is hard to communicate.
  // This should give a good enough initial camera. Later it will be
  // refined with bundle adjustment taking into account the distortion.
  cv::Mat distortion(4, 1, cv::DataType<double>::type, cv::Scalar(0));

  // Convert to OpenCV format
  std::vector<cv::Point2d> cv_pixel_observations;
  std::vector<cv::Point3d> cv_ground_points;
  for (size_t it = 0; it < ground_points.size(); it++) {
    auto & V = ground_points[it];
    auto & P = pixel_observations[it];
    cv_ground_points.push_back(cv::Point3d(V[0], V[1], V[2]));
    cv_pixel_observations.push_back(cv::Point2d(P[0], P[1]));
  }
  
  // Call PnP
  bool useExtrinsicGuess = false;
  int iterationsCount = 1000; // This algorithm is cheap, let it try hard
  float reprojectionError = 20.0; // because of un-modeled distortion, relax things here
  double confidence = 0.95;
  cv::Mat rvec(3, 1, cv::DataType<double>::type, cv::Scalar(0)); // Rodrigues rotation 
  cv::Mat tvec(3, 1, cv::DataType<double>::type, cv::Scalar(0)); // translation
  bool result = cv::solvePnPRansac(cv_ground_points, cv_pixel_observations,
                                   intrinsics, distortion,
                                   rvec, tvec, // outputs
                                   useExtrinsicGuess, iterationsCount, reprojectionError,
                                   confidence);
  if (!result)
    vw::vw_throw(vw::ArgumentErr()
                 << "Failed to find camera orientation using pixel and ground data.\n");

  // Convert obtained rotation
  Eigen::Matrix3d rotation;
  RodriguesToRotation(Eigen::Vector3d(rvec.at<double>(0), rvec.at<double>(1), 
                                      rvec.at<double>(2)), rotation);

  // Make world2cam into cam2world
  Eigen::Matrix3d cam2world = rotation.inverse(); 
  Eigen::Vector3d cam_ctr = -rotation.inverse() *
    Eigen::Vector3d(tvec.at<double>(0), tvec.at<double>(1), tvec.at<double>(2));

  // Convert Eigen matrix and vector to VW
  vw::Matrix3x3 rot;
  for (int row = 0; row < 3; row++) {
    for (int col = 0; col < 3; col++) {
      rot(row, col) = cam2world(row, col);
    }
  }
  vw::Vector3 ctr;
  for (int row = 0; row < 3; row++)
    ctr[row] = cam_ctr[row];

  // Apply the transform to the camera
  cam.set_camera_pose(rot);
  cam.set_camera_center(ctr);

  return;
}
  
} // end namespace asp
