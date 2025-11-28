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

#ifndef ASP_RIG_CAMERA_PARAMS_H
#define ASP_RIG_CAMERA_PARAMS_H

#include <asp/Rig/RigRpcDistortion.h>

#include <Eigen/Core>

#include <string>
#include <vector>
#include <algorithm>

// Forward declare mat type so that we don't have to include OpenCV if we're
// not going to use it.
namespace cv {
  class Mat;
}  // end namespace cv

// Functionality for undistorting and re-distorting images
namespace rig {

  // Definition of camera frames
  //
  // RAW - This is how the image came to us.
  // DISTORTED - Similar to RAW, but we've cropped out everything that is
  // non-image
  // DISTORTED_C - Same as DISTORTED, but the origin is the center of the image
  // UNDISTORTED - A new camera frame that is create by remove lens distortion
  // and placing the optical center directly center of the image.
  // UNDISTORTED_C - Same as UNDISTORTED, but the origin is the center of the image
  enum {
    RAW,
    DISTORTED,
    DISTORTED_C,
    UNDISTORTED,
    UNDISTORTED_C
  };

  enum DistortionType {
    NO_DISTORTION,      // no distortion params
    FOV_DISTORTION,     // 1 distortion param
    FISHEYE_DISTORTION, // 4 distortion params
    RADTAN_DISTORTION,  // 4 or 5 distortion params
    RPC_DISTORTION      // many distortion params
  };
  
  // These are the names of the distortion models as strings in the config files
  const std::string NO_DISTORTION_STR      = "no_distortion";
  const std::string FOV_DISTORTION_STR     = "fov";
  const std::string FISHEYE_DISTORTION_STR = "fisheye";
  const std::string RADTAN_DISTORTION_STR  = "radtan";
  const std::string RPC_DISTORTION_STR     = "rpc";

  class CameraParameters {
   public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

    CameraParameters() {} // empty constructor; will create an undefined model
    
    CameraParameters(Eigen::Vector2i const& image_size,
                     Eigen::Vector2d const& focal_length,
                     Eigen::Vector2d const& optical_center,
                     Eigen::VectorXd const& distortion,
                     DistortionType distortion_type);

    // Used to create a remap table. This table is the same size as the
    // UNDISTORTED image, where every pixel's value is the corresponding pixel
    // location in the DISTORTED image.
    void GenerateRemapMaps(cv::Mat* remap_map, double scale = 1.0);

    // Conversion utilities
    template <int SRC, int DEST>
    void Convert(Eigen::Vector2d const& input, Eigen::Vector2d *output) const {
      throw("Please use the explicitly specified conversions by using the correct enum.");
    }

    // Utility to create intrinsic matrix for the correct coordinate frame
    template <int FRAME>
    Eigen::Matrix3d GetIntrinsicMatrix() const {
      throw("Please use the explicitly specified frame by using the correct enum.");
    }

    // Image size info. We don't give direct access because if the size
    // changes, we need to recompute some temporary variables.
    //   These apply to coordinate frames DISTORTED, DISTORTED_C
    void SetDistortedSize(Eigen::Vector2i const& image_size);
    const Eigen::Vector2i& GetDistortedSize() const;
    const Eigen::Vector2d& GetDistortedHalfSize() const;

    // Domain of validity of distortion model (normally all image)
    // Centered around image center.
    void SetDistortedCropSize(Eigen::Vector2i const& crop_size);
    const Eigen::Vector2i& GetDistortedCropSize() const;

    //   These apply to UNDISTORTED, UNDISTORTED_C
    void SetUndistortedSize(Eigen::Vector2i const& image_size);
    const Eigen::Vector2i& GetUndistortedSize() const;
    const Eigen::Vector2d& GetUndistortedHalfSize() const;

    // Start of DISTORTED inside RAW frame
    void SetCropOffset(Eigen::Vector2i const& crop);
    const Eigen::Vector2i& GetCropOffset() const;
    // Optical offset in DISTORTED frame
    void SetOpticalOffset(Eigen::Vector2d const& offset);
    const Eigen::Vector2d& GetOpticalOffset() const;

    // Helper functions for focal length. Unless pixels are square,
    // the focal length in x may differ than the one in y. Note that
    // they are equal for undistorted images. Note that
    // GetFocalLength() returns the mean of the two focal lengths
    // while GetFocalVector() returns the vector.
    void SetFocalLength(Eigen::Vector2d const&);
    double GetFocalLength() const;
    const Eigen::Vector2d& GetFocalVector() const;

    // This will change the lens distortion type based on the size of the
    // vector. Size 0, no lens distortion, Size 1, FOV, Size 4 or 5, TSAI.
    void SetDistortion(Eigen::VectorXd const& distortion);
    const Eigen::VectorXd& GetDistortion() const;

    rig::RPCLensDistortion m_rpc;

    // Comparison operator
    friend bool operator== (CameraParameters const& A, CameraParameters const& B) {
      return (A.m_crop_offset          == B.m_crop_offset          &&
              A.m_distorted_image_size == B.m_distorted_image_size &&
              A.m_distorted_crop_size  == B.m_distorted_crop_size  &&
              A.m_undistorted_size     == B.m_undistorted_size     &&
              A.m_distorted_half_size  == B.m_distorted_half_size  &&
              A.m_focal_length         == B.m_focal_length         &&
              A.m_optical_offset       == B.m_optical_offset       &&
              A.m_distortion_coeffs    == B.m_distortion_coeffs    &&
              A.m_distortion_type      == B.m_distortion_type      &&
              A.m_distortion_precalc1  == B.m_distortion_precalc1  &&
              A.m_distortion_precalc2  == B.m_distortion_precalc2  &&
              A.m_distortion_precalc3  == B.m_distortion_precalc3);
    }
    
    // Make this public to not have to use set and get functions
    DistortionType m_distortion_type;

    private:
    // Converts UNDISTORTED_C to DISTORTED_C
    void DistortCentered(Eigen::Vector2d const& undistorted_c,
                          Eigen::Vector2d* distorted_c) const;
    // Converts DISTORTED_C to UNDISTORTED_C
    void UndistortCentered(Eigen::Vector2d const& distorted_c,
                            Eigen::Vector2d* undistorted_c) const;

    // Members
    Eigen::Vector2i m_crop_offset;             // Start of DISTORTED in RAW frame
    Eigen::Vector2i m_distorted_image_size;    // Applies to DISTORTED, DISTORTED_C
      
    // Domain of validity of distortion model (normally all image).
    // Centered at the image center.
    Eigen::Vector2i m_distorted_crop_size;
    
    // Applies to UNDISTORTED, UNDISTORTED_C, this is bigger than distorted image
    // size because we need to expand the sensor to capture the unravelling from
    // lens distortion removal.      
    Eigen::Vector2i m_undistorted_size;  
    
    Eigen::Vector2d m_distorted_half_size, m_undistorted_half_size;
    
    // Focal length, in pixels
    Eigen::Vector2d m_focal_length;
    
    // Optical offset in DISTORTED frame
    Eigen::Vector2d m_optical_offset;

    // Distortion coefficients are in an arbitrary sized vector. The length of
    // the vector tells us what lens distortion model we are using. Length 0 =
    // No lens distortion, ideal camera. Length 1 = FOV model, Length 4:
    // fisheye or TSAI/OpenCV model, Length 5 = TSAI/OpenCV model.
    Eigen::VectorXd m_distortion_coeffs;
    double m_distortion_precalc1, m_distortion_precalc2, m_distortion_precalc3;
  };

#define DECLARE_CONVERSION(TYPEA, TYPEB) \
  template <>  \
  void CameraParameters::Convert<TYPEA, TYPEB>(Eigen::Vector2d const& input, Eigen::Vector2d *output) const
  DECLARE_CONVERSION(RAW, DISTORTED);
  DECLARE_CONVERSION(DISTORTED, RAW);
  DECLARE_CONVERSION(UNDISTORTED_C, DISTORTED_C);
  DECLARE_CONVERSION(UNDISTORTED_C, DISTORTED);
  DECLARE_CONVERSION(UNDISTORTED_C, UNDISTORTED);
  DECLARE_CONVERSION(UNDISTORTED, UNDISTORTED_C);
  DECLARE_CONVERSION(UNDISTORTED, DISTORTED);
  DECLARE_CONVERSION(UNDISTORTED, DISTORTED_C);
  DECLARE_CONVERSION(DISTORTED, DISTORTED_C);
  DECLARE_CONVERSION(DISTORTED_C, UNDISTORTED_C);
  DECLARE_CONVERSION(DISTORTED, UNDISTORTED);
  DECLARE_CONVERSION(DISTORTED, UNDISTORTED_C);
#undef DECLARE_CONVERSION

#define DECLARE_INTRINSIC(TYPE) \
  template <>  \
  Eigen::Matrix3d CameraParameters::GetIntrinsicMatrix<TYPE>() const
  DECLARE_INTRINSIC(RAW);
  DECLARE_INTRINSIC(DISTORTED);
  DECLARE_INTRINSIC(DISTORTED_C);
  DECLARE_INTRINSIC(UNDISTORTED);
  DECLARE_INTRINSIC(UNDISTORTED_C);
#undef DECLARE_INTRINSIC
}  // namespace rig

#endif  // ASP_RIG_CAMERA_PARAMS_H
