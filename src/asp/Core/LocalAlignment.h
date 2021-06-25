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
                       bool                 write_nodata,
                       std::string        & left_aligned_file,
                       std::string        & right_aligned_file,
                       int                & min_disp,
                       int                & max_disp); 
  
  // Go from 1D disparity of images with affine epipolar alignment to the 2D
  // disparity by undoing the transforms that applied this alignment.
  void unalign_1d_disparity(// Inputs
                            vw::ImageViewRef<float> aligned_disp_1d, 
                            vw::BBox2i const& left_crop_win, 
                            vw::BBox2i const& right_crop_win,
                            vw::math::Matrix<double> const& left_align_mat,
                            vw::math::Matrix<double> const& right_align_mat,
                            // Output
                            vw::ImageView<vw::PixelMask<vw::Vector2f>> & unaligned_disp_2d);
  
  // Go from 2D disparity of images with affine epipolar alignment to the 2D
  // disparity by undoing the transforms that applied this alignment.
  void unalign_2d_disparity(// Inputs
                            vw::ImageView<vw::PixelMask<vw::Vector2f>> const& aligned_disp_2d,
                            vw::BBox2i const& left_crop_win, 
                            vw::BBox2i const& right_crop_win,
                            vw::math::Matrix<double> const& left_align_mat,
                            vw::math::Matrix<double> const& right_align_mat,
                            // Output
                            vw::ImageView<vw::PixelMask<vw::Vector2f>> & unaligned_disp_2d);
  
  // Read the list of external stereo programs (plugins) and extract
  // the path to each such plugin and its library dependencies.
  void parse_plugins_list(std::map<std::string, std::string> & plugins,
                          std::map<std::string, std::string> & plugin_libs);

  // Given a string like "mgm -O 8 -s vfit", separate the name,
  // which is the first word, from the options, which is the rest.
  void parse_stereo_alg_name_and_opts(std::string const& stereo_alg,
                                      std::string      & alg_name,
                                      std::string      & alg_opts);

  // Given an input string having algorithm options, like "-v 4", and
  // environmental variables, like "VAL=5", possibly with repetitions,
  // so VAL=5 and VAL=6 can both be present, separate the two kinds
  // and remove the repetitions by keeping the values later in the
  // string. Do not allow any input character except letters, numbers,
  // space, period, underscore, plus, minus, and equal signs, for
  // security purposes.
  void extract_opts_and_env_vars(std::string const& input_str,
                                 std::string & options,
                                 std::map<std::string, std::string> & option_map,
                                 std::string & env_vars,
                                 std::map<std::string, std::string> & env_vars_map);

  // Call the OpenCV BM or SGBM algorithm
  void call_opencv_bm_or_sgbm(std::string const& left_file,
                              std::string const& right_file,
                              std::string const& mode, // bm or a flavor of sgbm 
                              int block_size,
                              int min_disp,
                              int max_disp,
                              int prefilter_cap, 
                              int uniqueness_ratio, 
                              int speckle_size, 
                              int speckle_range, 
                              int disp12_diff, 
                              int texture_thresh,      // only for BM
                              int P1,                  // only for SGBM
                              int P2,                  // only for SGBM
                              ASPGlobalOptions const& opt,
                              std::string const& disparity_file,
                              // Output
                              vw::ImageViewRef<float> & out_disp);
  
} // end namespace asp

#endif // __LOCAL_ALIGNMENT_H__
