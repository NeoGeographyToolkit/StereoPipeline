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


/// \file OpenCVUtils.h
///

#ifndef __ASP_CORE_OPENCVUTILS_H__
#define __ASP_CORE_OPENCVUTILS_H__

#include <string>

#include <vw/Math/Matrix.h>
#include <vw/Image/ImageViewRef.h>
#include <opencv2/core.hpp>

namespace asp {
  
  // Return the OpenCV matrix type as a string
  std::string cvMatType(cv::Mat const& M);
  
  // Convert a matrix from OpenCV to the VisionWorkbench matrix type.
  // Implemented only for double precision matrix.
  vw::Matrix<double> cvMatToVwMat(cv::Mat const& M);
  
  // Scale an image from [0, 1] to [0, 255], round, and clamp.
  // NaN values are not affected.
  void formScaledByteCVImage(vw::ImageViewRef<float> in, cv::Mat & out);
  
  // Insert an image as a block at a desired location in a bigger image
  void cvInsertBlock(cv::Mat const& input_image, int extra_x,
                     int extra_y, cv::Mat& output_image);
}

#endif //__ASP_CORE_OPENCVUTILS_H__
