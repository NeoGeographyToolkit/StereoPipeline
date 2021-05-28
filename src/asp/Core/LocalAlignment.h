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


/// \file LocalAlignment.h
///

#ifndef __LOCAL_ALIGNMENT_H__
#define __LOCAL_ALIGNMENT_H__

#include <vw/Math/BBox.h>
#include <vw/Math/Matrix.h>
#include <vw/Image/ImageViewRef.h>

namespace asp {

  // Algorithm to perform local alignment. Approach:
  //  - Given the global interest points and the left crop window, find
  //    the right crop window.
  //  - Crop the globally aligned images to these crop windows and find
  //    the interest points for the crops
  //  - Use the interest points to find the local alignment
  //  - Apply the composition of the global and local alignment to the
  //    original unaligned images to find the locally aligned images
  //  - Save the locally aligned images to disk
  //  - Estimate the search range for the locally aligned images

  class ASPGlobalOptions; // forward declaration
  
  void local_alignment(ASPGlobalOptions   & opt,
                       std::string const  & session_name,
                       vw::BBox2i const   & left_trans_crop_win,
                       vw::BBox2i         & right_trans_crop_win,
                       vw::Matrix<double> & left_local_mat,
                       vw::Matrix<double> & right_local_mat,
                       std::string        & left_aligned_file,
                       std::string        & right_aligned_file,
                       int                & min_disp,
                       int                & max_disp); 
  
  // Go from 1D disparity of images with affine epipolar alignment to the 2D
  // disparity by undoing the transforms that applied this alignment.
  void unalign_disparity(// Inputs
                         vw::ImageViewRef<float> disp_1d, 
                         vw::BBox2i const& left_crop_win, 
                         vw::BBox2i const& right_crop_win,
                         vw::math::Matrix<double> const& left_align_mat,
                         vw::math::Matrix<double> const& right_align_mat,
                         // Output
                         vw::ImageView<vw::PixelMask<vw::Vector2f>> & disp_2d);

  } // end namespace asp

#endif // __LOCAL_ALIGNMENT_H__
