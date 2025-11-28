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

#include <asp/Rig/RigCameraParams.h>
#include <asp/Rig/RigRpcDistortion.h>

#include <Eigen/Dense>
#include <gflags/gflags.h>

#include <iostream>

#include <glog/logging.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/core/eigen.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <fstream>
#include <iostream>

rig::CameraParameters::CameraParameters(Eigen::Vector2i const& image_size,
    Eigen::Vector2d const& focal_length,
    Eigen::Vector2d const& optical_center,
    Eigen::VectorXd const& distortion,
    DistortionType distortion_type) {
  SetDistortedSize(image_size);
  SetDistortedCropSize(image_size);
  SetUndistortedSize(image_size);
  m_focal_length = focal_length;
  m_optical_offset = optical_center;
  m_crop_offset.setZero();
  SetDistortion(distortion);
  m_distortion_type = distortion_type;
}

void rig::CameraParameters::SetDistortedSize(Eigen::Vector2i const& image_size) {
  m_distorted_image_size = image_size;
  m_distorted_half_size = image_size.cast<double>() / 2.0;
}

const Eigen::Vector2i& rig::CameraParameters::GetDistortedSize() const {
  return m_distorted_image_size;
}

const Eigen::Vector2d& rig::CameraParameters::GetDistortedHalfSize() const {
  return m_distorted_half_size;
}

void rig::CameraParameters::SetDistortedCropSize(Eigen::Vector2i const& crop_size) {
  m_distorted_crop_size = crop_size;
}

const Eigen::Vector2i& rig::CameraParameters::GetDistortedCropSize() const {
  return m_distorted_crop_size;
}

void rig::CameraParameters::SetUndistortedSize(Eigen::Vector2i const& image_size) {
  m_undistorted_size = image_size;
  m_undistorted_half_size = image_size.cast<double>() / 2.0;
}

const Eigen::Vector2i& rig::CameraParameters::GetUndistortedSize() const {
  return m_undistorted_size;
}

const Eigen::Vector2d& rig::CameraParameters::GetUndistortedHalfSize() const {
  return m_undistorted_half_size;
}

void rig::CameraParameters::SetCropOffset(Eigen::Vector2i const& crop) {
  m_crop_offset = crop;
}

const Eigen::Vector2i& rig::CameraParameters::GetCropOffset() const {
  return m_crop_offset;
}

void rig::CameraParameters::SetOpticalOffset(Eigen::Vector2d const& offset) {
  m_optical_offset = offset;
}

const Eigen::Vector2d& rig::CameraParameters::GetOpticalOffset() const {
  return m_optical_offset;
}

void rig::CameraParameters::SetFocalLength(Eigen::Vector2d const& f) {
  m_focal_length = f;
}

double rig::CameraParameters::GetFocalLength() const {
  return m_focal_length.mean();
}

const Eigen::Vector2d& rig::CameraParameters::GetFocalVector() const {
  return m_focal_length;
}

void rig::CameraParameters::SetDistortion(Eigen::VectorXd const& distortion) {

  // Reset this. Will be needed only with RPC distortion.
  m_rpc = rig::RPCLensDistortion(); 
  
  m_distortion_coeffs = distortion;

  // Ensure variables are initialized
  m_distortion_precalc1 = 0;
  m_distortion_precalc2 = 0;
  m_distortion_precalc3 = 0;

  switch (m_distortion_coeffs.size()) {
  case 0:
    // No lens distortion!
    break;
  case 1:
    // FOV model
    // inverse alpha
    m_distortion_precalc1 = 1 / distortion[0];
    // Inside tangent function
    m_distortion_precalc2 = 2 * tan(distortion[0] / 2);
    break;
  case 4:
    // Fall through intended.
  case 5:
    // Tsai model
    // There doesn't seem like there are any precalculations we can use.
    break;
  default:
    m_rpc.set_distortion_parameters(m_distortion_coeffs);
  }
}

const Eigen::VectorXd& rig::CameraParameters::GetDistortion() const {
  return m_distortion_coeffs;
}

// Typedefs for distortion and Jacobian of distortion function signatures
typedef std::function<Eigen::Vector2d(Eigen::Vector2d const&, Eigen::VectorXd const&)> FunT;
typedef std::function<Eigen::VectorXd(Eigen::Vector2d const&, Eigen::VectorXd const&, 
                                      double, FunT)> JacT;

// Apply the fisheye distortion model. Input and output are normalized pixels.
Eigen::Vector2d fishEyeDistortionNorm(Eigen::Vector2d const& P, Eigen::VectorXd const& dist) {
  
  double k1 = dist[0];
  double k2 = dist[1];
  double k3 = dist[2];
  double k4 = dist[3];

  double x = P[0];
  double y = P[1];  
  double r2 = x*x + y*y;
  double r = sqrt(r2);
  double theta = atan(r);

  double theta1 = theta*theta;   // theta^2
  double theta2 = theta1*theta1; // theta^4
  double theta3 = theta2*theta1; // theta^6
  double theta4 = theta2*theta2; // theta^8
  double theta_d = theta*(1 + k1*theta1 + k2*theta2 + k3*theta3 + k4*theta4);
  
  // Careful with the case where r is very small
  double scale = 1.0;
  if (r > 1e-8)
    scale = theta_d / r;

  return Eigen::Vector2d(x*scale, y*scale);
}

// Find the Jacobian of a function at a given point using numerical differentiation.
// A good value for the step is 1e-6. Note that above we use a tolerance of 1e-8
// when dividing floating point values, which is way smaller than this step.
Eigen::VectorXd numericalJacobian(Eigen::Vector2d const& P,
                                  Eigen::VectorXd const& dist,
                                  double step, FunT func) {

  // The Jacobian has 4 elements.
  Eigen::VectorXd jacobian(4);
  
  // First column
  Eigen::Vector2d JX = (func(P + Eigen::Vector2d(step, 0), dist) - 
                        func(P - Eigen::Vector2d(step, 0), dist)) / (2*step);
  // Second column
  Eigen::Vector2d JY = (func(P + Eigen::Vector2d(0, step), dist) - 
                        func(P - Eigen::Vector2d(0, step), dist)) / (2*step);
  
  // Put in the jacobian matrix
  jacobian[0] = JX[0];
  jacobian[1] = JY[0];
  jacobian[2] = JX[1];
  jacobian[3] = JY[1];  
  
  return jacobian;
}

// Find X solving func(X) - Y = 0. Use the Newton-Raphson method.
// Update X as X - (func(X) - Y) * J^-1, where J is the Jacobian of func(X).
Eigen::Vector2d newtonRaphson(Eigen::Vector2d const& Y,
                              Eigen::VectorXd const& distortion,
                              FunT func, JacT jac) {

  // The step for differentiating the function (1e-6) should be larger
  // than the tolerance for finding the function value (1e-8).
  double step = 1e-6;
     
  // Initial guess for the root
  Eigen::Vector2d X = Y;

  int count = 1, maxTries = 20;
  while (count < maxTries) {
    
    Eigen::Vector2d F = func(X, distortion) - Y;
    
    // Compute the Jacobian
    Eigen::VectorXd J(4);
    J = jac(X, distortion, step, func);
    
    // Find the determinant
    double det = J[0]*J[3] - J[1]*J[2];
    if (fabs(det) < 1e-6) {
      // Near-zero determinant. Cannot continue. Return most recent result.
      return X;
    } 

    Eigen::Vector2d DX;
    DX[0] = (J[3]*F[0] - J[1]*F[1]) / det;
    DX[1] = (J[0]*F[1] - J[2]*F[0]) / det;
    
    // Update X
    X -= DX;
    
    // If DX is small enough, we are done
    if (DX.norm() < 1e-6)
      return X;
      
    count++;
   }
  return X;
}

void rig::CameraParameters::DistortCentered(Eigen::Vector2d const& undistorted_c,
                                               Eigen::Vector2d* distorted_c) const {
  // We assume that input x and y are pixel values that have
  // undistorted_len_x/2.0 and undistorted_len_y/2.0 subtracted from
  // them. The outputs will have distorted_len_x/2.0 and
  // distorted_len_y/2.0 subtracted from them.
  if (m_distortion_coeffs.size() == 0) {
    // There is no distortion
    *distorted_c = undistorted_c + m_optical_offset - m_distorted_half_size;
  } else if (m_distortion_coeffs.size() == 1) {
    // This is the FOV model
    Eigen::Vector2d norm = undistorted_c.cwiseQuotient(m_focal_length);
    double ru = norm.norm();
    double rd = atan(ru * m_distortion_precalc2) * m_distortion_precalc1;
    double conv;
    if (ru > 1e-8) {
      conv = rd / ru;
    } else {
      conv = 1;
    }
    *distorted_c = (m_optical_offset - m_distorted_half_size) +
      conv * norm.cwiseProduct(m_focal_length);
      
  } else if (m_distortion_coeffs.size() == 4 && m_distortion_type == FISHEYE_DISTORTION) {
  
    // Fisheye lens distortion
    // https://docs.opencv.org/4.x/db/d58/group__calib3d__fisheye.html Note: The
    // function cv::fisheye::distortPoints() assumes that the inputs have the
    // image midpoint subtracted and are divided by the focal length, while its
    // output are pixels that are not centered.
    
    // Normalize the pixel
    Eigen::Vector2d norm = undistorted_c.cwiseQuotient(m_focal_length);

    // Apply the distortion to the normalized pixel
    *distorted_c = fishEyeDistortionNorm(norm, m_distortion_coeffs);
   
   // Scale by the focal length and add the optical offset
   *distorted_c = distorted_c->cwiseProduct(m_focal_length) +
     (m_optical_offset - m_distorted_half_size);
    
  } else if (m_distortion_coeffs.size() == 4 ||
             m_distortion_coeffs.size() == 5) {
    // Tsai lens distortion (radial-tangential, implemented in OpenCV)
    double k1 = m_distortion_coeffs[0];
    double k2 = m_distortion_coeffs[1];
    double p1 = m_distortion_coeffs[2];
    double p2 = m_distortion_coeffs[3];
    double k3 = 0;
    if (m_distortion_coeffs.size() == 5)
      k3 = m_distortion_coeffs[4];

    // To relative coordinates
    Eigen::Vector2d norm = undistorted_c.cwiseQuotient(m_focal_length);
    double r2 = norm.squaredNorm();

    // Radial distortion
    double radial_dist = 1 + k1 * r2 + k2 * r2 * r2 + k3 * r2 * r2 * r2;
    *distorted_c = radial_dist * norm;

    // Tangential distortion
    *distorted_c +=
      Eigen::Vector2d(2 * p1 * norm[0] * norm[1] + p2 * (r2 + 2 * norm[0] * norm[0]),
                      p1 * (r2 + 2 * norm[1] * norm[1]) + 2 * p2 * norm[0] * norm[1]);

    // Back to absolute coordinates
    *distorted_c = distorted_c->cwiseProduct(m_focal_length) +
      (m_optical_offset - m_distorted_half_size);
  } else {
  
    // RPC 
    // Normalize the pixel
    Eigen::Vector2d norm = undistorted_c.cwiseQuotient(m_focal_length);

    // Apply the distortion to the normalized pixel
    *distorted_c = rig::compute_rpc(norm, m_distortion_coeffs);
   
    // Scale by the focal length and add the optical offset
    *distorted_c = distorted_c->cwiseProduct(m_focal_length) +
     (m_optical_offset - m_distorted_half_size);
  }
}

void rig::CameraParameters::UndistortCentered(Eigen::Vector2d const& distorted_c,
                                                 Eigen::Vector2d *undistorted_c) const {
  if (m_distortion_coeffs.size() == 0) {
    // No lens distortion
    *undistorted_c = distorted_c - (m_optical_offset - m_distorted_half_size);
  } else if (m_distortion_coeffs.size() == 1) {
    // FOV lens distortion
    Eigen::Vector2d norm =
      (distorted_c - (m_optical_offset - m_distorted_half_size)).cwiseQuotient(m_focal_length);
    double rd = norm.norm();
    double ru = tan(rd * m_distortion_coeffs[0]) / m_distortion_precalc2;
    double conv = 1.0;
    if (rd > 1e-8)
      conv = ru / rd;
    *undistorted_c = conv * norm.cwiseProduct(m_focal_length);
    
  } else if (m_distortion_coeffs.size() == 4 && m_distortion_type == FISHEYE_DISTORTION) {
  
    // Center and normalize
    Eigen::Vector2d norm =
    (distorted_c - (m_optical_offset - m_distorted_half_size)).cwiseQuotient(m_focal_length);

     // Find the normalized undistorted pixel using Newton-Raphson
     Eigen::Vector2d U = newtonRaphson(norm, m_distortion_coeffs,
                                       fishEyeDistortionNorm, numericalJacobian);
     
     // Undo the normalization
     *undistorted_c = U.cwiseProduct(m_focal_length);
      
  } else if (m_distortion_coeffs.size() == 4 ||
             m_distortion_coeffs.size() == 5) {
    // Tsai lens distortion (radial-tangential)
    // TODO(oalexan1): Must test this! There is apparently an issue with 
    // the OpenCV implementation! Must use the numerical Jacobian here!
    cv::Mat src(1, 1, CV_64FC2);
    cv::Mat dst(1, 1, CV_64FC2);
    Eigen::Map<Eigen::Vector2d> src_map(src.ptr<double>()), dst_map(dst.ptr<double>());
    cv::Mat dist_int_mat(3, 3, cv::DataType<double>::type),
      undist_int_mat(3, 3, cv::DataType<double>::type);
    cv::Mat cvdist;
    cv::eigen2cv(m_distortion_coeffs, cvdist);
    cv::eigen2cv(GetIntrinsicMatrix<DISTORTED>(), dist_int_mat);
    cv::eigen2cv(GetIntrinsicMatrix<UNDISTORTED>(), undist_int_mat);
    src_map = distorted_c + m_distorted_half_size;
    // Note: cv::undistortPoints() has an error of about half a pixel
    // TODO(oalexan1): Need to investigate this further, and maybe
    // change the implementation to using Newton-Raphson.
    cv::undistortPoints(src, dst, dist_int_mat, cvdist, cv::Mat(), undist_int_mat);
    *undistorted_c = dst_map - m_undistorted_half_size;
  } else {
    // RPC lens distortion
    // Center and normalize
    Eigen::Vector2d norm =
    (distorted_c - (m_optical_offset - m_distorted_half_size)).cwiseQuotient(m_focal_length);

     // Find the normalized undistorted pixel using Newton-Raphson
     Eigen::Vector2d U = newtonRaphson(norm, m_distortion_coeffs,
                                       rig::compute_rpc, numericalJacobian);
     
     // Undo the normalization
     *undistorted_c = U.cwiseProduct(m_focal_length);
  }
}

// The 'scale' variable is useful when we have the distortion model for a given
// image, and want to apply it to a version of that image at a different resolution,
// with 'scale' being the ratio of the width of the image at different resolution
// and the one at the resolution at which the distortion model is computed.
void rig::CameraParameters::GenerateRemapMaps(cv::Mat* remap_map, double scale) {
  remap_map->create(scale*m_undistorted_size[1], scale*m_undistorted_size[0], CV_32FC2);
  Eigen::Vector2d undistorted, distorted;
  for (undistorted[1] = 0; undistorted[1] < scale*m_undistorted_size[1]; undistorted[1]++) {
    for (undistorted[0] = 0; undistorted[0] < scale*m_undistorted_size[0]; undistorted[0]++) {
      Convert<UNDISTORTED, DISTORTED>(undistorted/scale, &distorted);
      remap_map->at<cv::Vec2f>(undistorted[1], undistorted[0])[0] = scale*distorted[0];
      remap_map->at<cv::Vec2f>(undistorted[1], undistorted[0])[1] = scale*distorted[1];
    }
  }
}


namespace rig {

  // Conversion function helpers
#define DEFINE_CONVERSION(TYPEA, TYPEB) \
  template <> \
  void rig::CameraParameters::Convert<TYPEA, TYPEB>(Eigen::Vector2d const& input, Eigen::Vector2d *output) const

  DEFINE_CONVERSION(RAW, DISTORTED) {
    *output = input - m_crop_offset.cast<double>();
  }
  DEFINE_CONVERSION(DISTORTED, RAW) {
    *output = input + m_crop_offset.cast<double>();
  }
  DEFINE_CONVERSION(UNDISTORTED_C, DISTORTED_C) {
    DistortCentered(input, output);
  }
  DEFINE_CONVERSION(DISTORTED_C, UNDISTORTED_C) {
    UndistortCentered(input, output);
  }
  DEFINE_CONVERSION(UNDISTORTED, UNDISTORTED_C) {
    *output = input - m_undistorted_half_size;
  }
  DEFINE_CONVERSION(UNDISTORTED_C, UNDISTORTED) {
    *output = input + m_undistorted_half_size;
  }
  DEFINE_CONVERSION(DISTORTED, UNDISTORTED) {
    Convert<DISTORTED_C, UNDISTORTED_C>(input - m_distorted_half_size, output);
    *output += m_undistorted_half_size;
  }
  DEFINE_CONVERSION(UNDISTORTED, DISTORTED) {
    Eigen::Vector2d centered_output;
    Convert<UNDISTORTED, UNDISTORTED_C>(input, output);
    Convert<UNDISTORTED_C, DISTORTED_C>(*output, &centered_output);
    *output = centered_output + m_distorted_half_size;
  }
  DEFINE_CONVERSION(DISTORTED, DISTORTED_C) {
    *output = input - m_distorted_half_size;
  }
  DEFINE_CONVERSION(DISTORTED, UNDISTORTED_C) {
    Convert<DISTORTED_C, UNDISTORTED_C>(input - m_distorted_half_size, output);
  }
  DEFINE_CONVERSION(UNDISTORTED_C, DISTORTED) {
    Convert<UNDISTORTED_C, DISTORTED_C>(input, output);
    *output += m_distorted_half_size;
  }
  DEFINE_CONVERSION(UNDISTORTED, DISTORTED_C) {
    Eigen::Vector2d centered_output;
    Convert<UNDISTORTED, UNDISTORTED_C>(input, output);
    Convert<UNDISTORTED_C, DISTORTED_C>(*output, &centered_output);
    *output = centered_output;
  }

#undef DEFINE_CONVERSION

  // Helper functions to give the intrinsic matrix
#define DEFINE_INTRINSIC(TYPE) \
  template <> \
  Eigen::Matrix3d rig::CameraParameters::GetIntrinsicMatrix<TYPE>() const

  DEFINE_INTRINSIC(RAW) {
    Eigen::Matrix3d k = m_focal_length.homogeneous().asDiagonal();
    k.block<2, 1>(0, 2) = m_optical_offset + m_crop_offset.cast<double>();
    return k;
  }
  DEFINE_INTRINSIC(DISTORTED) {
    Eigen::Matrix3d k = m_focal_length.homogeneous().asDiagonal();
    k.block<2, 1>(0, 2) = m_optical_offset;
    return k;
  }
  DEFINE_INTRINSIC(DISTORTED_C) {
    Eigen::Matrix3d k = m_focal_length.homogeneous().asDiagonal();
    k.block<2, 1>(0, 2) = m_optical_offset - m_distorted_half_size;
    return k;
  }
  DEFINE_INTRINSIC(UNDISTORTED) {
    Eigen::Matrix3d k = m_focal_length.homogeneous().asDiagonal();
    k.block<2, 1>(0, 2) = m_undistorted_half_size;
    return k;
  }
  DEFINE_INTRINSIC(UNDISTORTED_C) {
    Eigen::Matrix3d k = m_focal_length.homogeneous().asDiagonal();
    return k;
  }

#undef DEFINE_INTRINSIC

}  // end namespace rig

