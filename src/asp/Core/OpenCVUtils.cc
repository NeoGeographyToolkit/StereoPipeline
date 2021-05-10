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
  
  
}
