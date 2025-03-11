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


#ifndef __ASP_CORE_DISPARITY_PROCESSING_H__
#define __ASP_CORE_DISPARITY_PROCESSING_H__

#include <vw/Image/ImageView.h>
#include <vw/Image/ImageViewRef.h>
#include <vw/Camera/CameraModel.h>
#include <vw/Math/Transform.h>
#include <vw/Cartography/Datum.h>

namespace asp {

  typedef vw::ImageViewRef<vw::PixelMask<vw::Vector2f>> DispImageType;

  // Forward declarations
  class ASPGlobalOptions;

  /// Load the D_sub file in a consistent format.
  /// - Returns false if the file does not exist.
  bool load_D_sub(std::string const& d_sub_file,
                  vw::ImageView<vw::PixelMask<vw::Vector2f>> & sub_disp);

  
  // Load the low-res disparity and the scale needed to convert it to full-res
  void load_D_sub_and_scale(std::string                          const & out_prefix, 
                            std::string                          const & d_sub_file, 
                            vw::ImageView<vw::PixelMask<vw::Vector2f>> & sub_disp,
                            vw::Vector2                                & upsample_scale);
  
  // Filter D_sub. All alignment methods are supported.
  void filter_D_sub(ASPGlobalOptions const& opt,
                    vw::TransformPtr tx_left, vw::TransformPtr tx_right,
                    boost::shared_ptr<vw::camera::CameraModel> left_camera_model, 
                    boost::shared_ptr<vw::camera::CameraModel> right_camera_model,
                    vw::cartography::Datum const& datum,
                    std::string const& d_sub_file,
                    vw::Vector2 const& outlier_removal_params);
  
  // Filter D_sub by reducing its spread around the median. Return the count of
  // removed pixels.
  int dispSpreadFilter(vw::ImageView<vw::PixelMask<vw::Vector2f>> & sub_disp,
                      double max_disp_spread,
                      vw::Vector2 const& upsample_scale);
  
  // Filter D_sub by reducing its spread around the median. Read from disk,
  // filter, write back to disk.
  void dispSpreadFilterIO(ASPGlobalOptions const& opt, std::string const& d_sub_file,
                          double max_disp_spread);
  
  // Take a given disparity and make it between the original unaligned images
  void unalign_disparity(bool is_map_projected,
                         DispImageType    const& disparity,
                         vw::TransformPtr const& left_trans,
                         vw::TransformPtr const& right_trans,
                         ASPGlobalOptions const& opt,
                         std::string      const& disp_file);

  /// Bin the disparities, and from each bin get a disparity value.
  /// This will create a correspondence from the left to right image,
  /// which we save in the match format.
  /// When gen_triplets is true, and there are many overlapping images,
  /// try hard to have many IP with the property that each such IP is seen
  /// in more than two images. This helps with bundle adjustment.
  void compute_matches_from_disp(ASPGlobalOptions const& opt,
                                 DispImageType    const& disp,
                                 std::string const& left_raw_image,
                                 std::string const& right_raw_image, 
                                 vw::TransformPtr const& left_trans,
                                 vw::TransformPtr const& right_trans,
                                 std::string      const& match_file,
                                 int max_num_matches,
                                 bool gen_triplets, bool is_map_projected);
  
} // End namespace asp

#endif//__ASP_CORE_DISPARITY_PROCESSING_H__
