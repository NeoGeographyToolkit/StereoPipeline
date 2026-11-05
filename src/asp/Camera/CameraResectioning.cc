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

/// \file CameraResectioning.cc

#include <asp/Camera/CameraResectioning.h>
#include <vw/Camera/PinholeModel.h>
#include <vw/Camera/CameraModel.h>
#include <vw/Cartography/Datum.h>
#include <vw/Math/BBox.h>
#include <vw/Math/Geometry.h>
#include <vw/Math/Statistics.h>
#include <vw/Camera/OpticalBarModel.h>
#include <vw/Cartography/CameraDatumFit.h>
#include <vw/Core/Log.h>

#include <opencv2/calib3d/calib3d.hpp>
#include <Eigen/Dense>

#include <string>

using namespace vw;

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
                    vw::PinholeModel & cam) {

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
  vw::BBox2 img_box;
  std::vector<cv::Point2d> cv_pixel_observations;
  std::vector<cv::Point3d> cv_ground_points;
  for (size_t it = 0; it < ground_points.size(); it++) {
    auto & V = ground_points[it];
    auto & P = pixel_observations[it];
    img_box.grow(P);
    cv_ground_points.push_back(cv::Point3d(V[0], V[1], V[2]));
    cv_pixel_observations.push_back(cv::Point2d(P[0], P[1]));
  }
  
  // Call PnP. Because of un-modeled distortion and inaccuracy of picking GCP,
  // relax the tolerance. But still want to throw out worst outliers, for example,
  // if the inputs are produced with interest-point matching, which can have a
  // lot of outliers.
  bool useExtrinsicGuess = false;
  int iterationsCount = 1000; // This algorithm is cheap, let it try hard
  double len = std::max(img_box.width(), img_box.height());
  float reprojectionError = std::max(20.0, 0.10 * len);
  double confidence = 0.75;
  cv::Mat rvec(3, 1, cv::DataType<double>::type, cv::Scalar(0)); // Rodrigues rotation 
  cv::Mat tvec(3, 1, cv::DataType<double>::type, cv::Scalar(0)); // translation
  bool result = cv::solvePnPRansac(cv_ground_points, cv_pixel_observations,
                                   intrinsics, distortion,
                                   rvec, tvec, // outputs
                                   useExtrinsicGuess, iterationsCount, reprojectionError,
                                   confidence);
  if (!result)
    vw::vw_throw(vw::ArgumentErr()
                 << "Failed to find the camera pose using pixel and ground data. "
                 << "The provided inputs may be inaccurate.\n");

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

/// Find the best camera that fits the given ground points and pixel observations,
/// with optional known or estimated camera center, and refine it.
void fit_camera_to_xyz_ht(vw::Vector3 const& known_cam_ctr,
                          vw::Vector3 const& estim_cam_ctr,
                          std::string const& camera_type,
                          bool refine_camera,
                          std::vector<vw::Vector3> const& xyz_vec,
                          std::vector<double> const& pixel_values,
                          double cam_height, double cam_weight, double cam_ctr_weight,
                          vw::Datum const& datum,
                          bool verbose,
                          boost::shared_ptr<vw::CameraModel> & out_cam) {

  // Create fake points in space at given distance from this camera's
  // center and corresponding actual points on the ground. Use 500
  // km, just some height not too far from actual satellite height.
  double ht = 500000.0;
  int num_pts = pixel_values.size()/2;
  vw::Matrix<double> in, out;
  std::vector<vw::Vector2> pixel_vec;
  in.set_size(3, num_pts);
  out.set_size(3, num_pts);
  for (int col = 0; col < in.cols(); col++) {
    if (estim_cam_ctr != Vector3(0, 0, 0)) {
      // We know the camera center. Use that.
      // TODO(oalexan1): Problem with RPC!
      ht = norm_2(xyz_vec[col] - estim_cam_ctr);
    }
    Vector2 pix = Vector2(pixel_values[2*col], pixel_values[2*col+1]);
    Vector3 a = out_cam->camera_center(Vector2(0, 0)) + ht * out_cam->pixel_to_vector(pix);
    pixel_vec.push_back(pix);
    for (int row = 0; row < in.rows(); row++) {
      in(row, col)  = a[row];
      out(row, col) = xyz_vec[col][row];
    }
  }

  // Apply a transform to the camera so that the fake points are on top of the real points
  Matrix<double, 3, 3> rotation;
  Vector3 translation;
  double scale;
  find_3D_transform(in, out, rotation, translation, scale);
  if (camera_type == "opticalbar") {
    ((vw::OpticalBarModel*)out_cam.get())->apply_transform(rotation,
                                   translation, scale);
    if (known_cam_ctr != Vector3(0, 0, 0))
    ((vw::OpticalBarModel*)out_cam.get())->set_camera_center(known_cam_ctr);
  } else if (camera_type == "pinhole") {

    // The logic based on fake points can give junk results. Use instead the
    // state-of-the-art OpenCV solver. TODO(oalexan1): Need to consider using
    // this solution also for OpticalBar.
    try {
      asp::findCameraPose(xyz_vec, pixel_vec, *(PinholeModel*)out_cam.get());
    } catch(std::exception const& e) {
      vw_out() << "Failed to find the camera pose using OpenCV. Falling back "
                << "to ASP's internal approach. This is not as robust. "
                << "Check your inputs and validate the produced camera.\n";
      // Fall back to previous logic
      ((PinholeModel*)out_cam.get())->apply_transform(rotation, translation, scale);
    }

    // Overwrite the solved camera center with what is surely known
    if (known_cam_ctr != Vector3(0, 0, 0))
      ((PinholeModel*)out_cam.get())->set_camera_center(known_cam_ctr);
  }

  // Print out some errors
  if (verbose) {
    std::vector<double> errors;
    for (size_t corner_it = 0; corner_it < num_pts; corner_it++) {
      vw::Vector2 pix1 = out_cam.get()->point_to_pixel(xyz_vec[corner_it]);
      vw::Vector2 pix2 = Vector2(pixel_values[2*corner_it], pixel_values[2*corner_it+1]);
      errors.push_back(norm_2(pix1 - pix2));
    }
    vw::vw_out() << "Median pixel projection error in the coarse camera: "
                 << vw::median_in_place(errors) << "\n";
  }

  Vector3 xyz0 = out_cam.get()->camera_center(vw::Vector2());

  // Solve a little optimization problem to make the points on the ground project
  // as much as possible exactly into the image corners.
  if (refine_camera) {
    if (camera_type == "opticalbar")
      fitOpticalBar(xyz_vec, cam_height, cam_weight, cam_ctr_weight, datum, pixel_values,
                             *((vw::OpticalBarModel*)out_cam.get()));
    else if (camera_type == "pinhole")
      fitPinhole(xyz_vec, cam_height, cam_weight, cam_ctr_weight, datum, pixel_values,
                          *((PinholeModel*)out_cam.get()));
    else
      vw_throw(ArgumentErr() << "Unknown camera type: " << camera_type << ".\n");

    if (verbose) {
      std::vector<double> errors;
      for (size_t corner_it = 0; corner_it < num_pts; corner_it++) {
        vw::Vector2 pix1 = out_cam.get()->point_to_pixel(xyz_vec[corner_it]);
        vw::Vector2 pix2 = Vector2(pixel_values[2*corner_it], pixel_values[2*corner_it+1]);
        errors.push_back(norm_2(pix1 - pix2));
      }
      vw::vw_out() << "Median pixel projection error in the refined camera: "
                   << vw::median_in_place(errors) << "\n";
    }

  } // End camera refinement case
}

} // end namespace asp
