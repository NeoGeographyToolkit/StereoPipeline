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


#include <vw/Core/Exception.h>
#include <asp/Core/OpenCVUtils.h>

#include <Eigen/Core>
#include <opencv2/core/eigen.hpp>

#include <iostream>

namespace asp {

  // Return the OpenCV matrix type as a string
  std::string cvMatType(cv::Mat const& M) {

    int inttype = M.type();

    std::string r, a;
    
    uchar depth = inttype & CV_MAT_DEPTH_MASK;
    uchar chans = 1 + (inttype >> CV_CN_SHIFT);

    switch (depth) {
    case CV_8U:  r = "8U"; break;
    case CV_8S:  r = "8S"; break;
    case CV_16U: r = "16U"; break;
    case CV_16S: r = "16S"; break;
    case CV_32S: r = "32S"; break;
    case CV_32F: r = "32F"; break;
    case CV_64F: r = "64F"; break;
    default:     r = "User"; break;
    }   
    r += "C";
    r += (chans+'0');

    return r;
  }

  // Convert a matrix from OpenCV to the VisionWorkbench matrix type.
  // Implemented only for double precision matrix (should be easy to
  // extend also for single precision and various integer types).
  vw::Matrix<double> cvMatToVwMat(cv::Mat const& M) {

    std::string matType = asp::cvMatType(M);
    if (matType != "64FC1") {
      vw::vw_throw(vw::NoImplErr() << "cvMatToVwMat() implemented only for "
                   << "double-precision OpenCV matrices (64FC1). Got: " << matType);
      
    }
    
    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> EM;
    cv::cv2eigen(M, EM);

    vw::Matrix<double> VM(EM.rows(), EM.cols());

    for (int row = 0; row < EM.rows(); row++) {
      for (int col = 0; col < EM.cols(); col++) {
        VM(row, col) = EM(row, col);
      }
    }

    return VM;
  }
  

  // Scale an image from [0, 1] to [0, 255], round, and clamp.
  // NaN values are not affected.
  void formScaledByteCVImage(vw::ImageViewRef<float> in, cv::Mat & out) {
    
    out = cv::Mat(in.rows(), in.cols(), CV_8UC1, cv::Scalar(0));

    // Note how we read from in(col, row) but write to out(row, col)
    
    for (int row = 0; row < in.rows(); row++) {
      for (int col = 0; col < in.cols(); col++) {
        int val = round(255.0 * in(col, row));
        if (val < 0) 
          val = 0;
        if (val > 255)
          val = 255;
      
        out.at<uint8_t>(row, col) = val;
      }
    }

    return;
  }

  // Scale a masked image from [0, 1] to [0, 255], round, and clamp.
  // Invalid values get set to 0 in the output. Masking likely casts NaN
  // to invalid data so they end up being set to 0.
  void maskedToScaledByteCvImage(vw::ImageViewRef<vw::PixelMask<float>> in, cv::Mat & out) {
    vw::ImageViewRef<float> img = vw::apply_mask(in, 0.0f);
    formScaledByteCVImage(img, out);
  }

  // Insert an image as a block at a desired location in a bigger image
  void cvInsertBlock(cv::Mat const& input_image, int extra_x,
                     int extra_y, cv::Mat& output_image) {
    input_image.copyTo(output_image(cv::Range(extra_y, extra_y + input_image.rows),
                                    cv::Range(extra_x, extra_x + input_image.cols)));
  }
  
} // end namespace asp
