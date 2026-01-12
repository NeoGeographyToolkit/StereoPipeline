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

#include <asp/Rig/RigMath.h>
#include <Eigen/SVD>
#include <glog/logging.h>
#include <iostream>
#include <vector>
#include <algorithm>
#include <cmath>

namespace rig {

// Implement some heuristic to find the maximum rotation angle that can result
// from applying the given transform. It is assumed that the transform is not
// too different from the identity.
double maxRotationAngle(Eigen::Affine3d const& T) {
  Eigen::Vector3d angles = T.linear().eulerAngles(0, 1, 2);

  // Angles close to +/-pi can result even if the matrix is close to identity
  for (size_t it = 0; it < 3; it++)
    angles[it] = std::min(std::abs(angles[it]), std::abs(M_PI - std::abs(angles[it])));
  double angle_norm = (180.0 / M_PI) * angles.norm();
  return angle_norm;
}

// Compute the azimuth and elevation for a (normal) vector
void normalToAzimuthAndElevation(Eigen::Vector3d const& normal,
                                 double& azimuth, double& elevation) {
  if (normal.x() == 0 && normal.y() == 0) {
    azimuth = 0.0;
    if (normal.z() >= 0.0)
      elevation = M_PI / 2.0;
    else
      elevation = -M_PI / 2.0;
  } else {
    azimuth = atan2(normal.y(), normal.x());
    elevation = atan2(normal.z(), Eigen::Vector2d(normal.x(), normal.y()).norm());
  }
}

// Compute a normal vector based on the azimuth and elevation angles
void azimuthAndElevationToNormal(Eigen::Vector3d& normal, double azimuth, double elevation) {
  double ca = cos(azimuth), sa = sin(azimuth);
  double ce = cos(elevation), se = sin(elevation);
  normal = Eigen::Vector3d(ca * ce, sa * ce, se);
}

// Snap the normal to the plane (and the plane itself) to make
// all angles multiple of 45 degrees with the coordinate axes.
void snapPlaneNormal(Eigen::Vector3d& plane_normal) {
  double azimuth, elevation;
  normalToAzimuthAndElevation(plane_normal, azimuth, elevation);

  // Snap to multiple of 45 degrees
  double radian45 = M_PI / 4.0;
  azimuth = radian45 * round(azimuth / radian45);
  elevation = radian45 * round(elevation / radian45);

  azimuthAndElevationToNormal(plane_normal, azimuth, elevation);
}

// Find the best fitting plane to a set of points
void bestFitPlane(const std::vector<Eigen::Vector3d>& points, Eigen::Vector3d& centroid,
                  Eigen::Vector3d& plane_normal) {
  // Copy coordinates to  matrix in Eigen format
  size_t num_points = points.size();
  Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> coord(3, num_points);

  for (size_t i = 0; i < num_points; i++) coord.col(i) = points[i];

  // calculate centroid
  centroid = Eigen::Vector3d(coord.row(0).mean(), coord.row(1).mean(), coord.row(2).mean());

  // subtract centroid
  for (size_t it = 0; it < 3; it++) coord.row(it).array() -= centroid(it);

  // We only need the left-singular matrix here
  // https://math.stackexchange.com/questions/99299/best-fitting-plane-given-a-set-of-points
  auto svd = coord.jacobiSvd(Eigen::ComputeThinU | Eigen::ComputeThinV);

  plane_normal = svd.matrixU().rightCols<1>();
}

// Gamma correction for x between 0 and 1.
// https://en.wikipedia.org/wiki/SRGB#Specification_of_the_transformation
double gamma(double x) {
  // return pow(x, 1.0/2.6);

  if (x <= 0.0031308) return 12.92 * x;

  return 1.055 * pow(x, 1.0 / 2.4) - 0.055;
}

double inv_gamma(double x) {
  // return pow(x, 2.6);

  if (x <= 0.04045) return x / 12.92;

  return pow((x + 0.055) / (1.055), 2.4);
}

// Apply the inverse gamma transform to images, multiply them by
// max_iso_times_exposure/ISO/exposure_time to adjust for
// lightning differences, then apply the gamma transform back.
void exposureCorrection(double max_iso_times_exposure, double iso, double exposure,
                        cv::Mat const& input_image, cv::Mat& output_image) {
  double scale = max_iso_times_exposure / iso / exposure;

  // Make an image of the same type
  input_image.copyTo(output_image);

  // Apply the inverse gamma correction, multiply by scale,
  // and apply the correction back
#pragma omp parallel for
  for (int row = 0; row < input_image.rows; row++) {
    for (int col = 0; col < input_image.cols; col++) {
      cv::Vec3b b = input_image.at<cv::Vec3b>(row, col);

      cv::Vec3b c;
      for (int color = 0; color < 3; color++) {
        double x = 255.0 * gamma(inv_gamma(static_cast<double>(b[color]) / 255.0) * scale);
        c[color] = std::min(round(x), 255.0);
      }
      output_image.at<cv::Vec3b>(row, col) = c;
    }
  }
}

}  // namespace rig
