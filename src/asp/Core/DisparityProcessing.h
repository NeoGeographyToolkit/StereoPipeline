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

  typedef boost::shared_ptr<vw::Transform> TransPtr;
  typedef vw::ImageViewRef<vw::PixelMask<vw::Vector2f>> DispImageType;

  // Forward declarations
  class ASPGlobalOptions;

  // Filter D_sub. Must be called only for alignment method affineepipolar, homography,
  // and local_epipolar. For now this is not in use as a dataset where this would help
  // was not found. It was tested though.
  void filter_D_sub(ASPGlobalOptions const& opt,
                    boost::shared_ptr<vw::camera::CameraModel> left_camera_model, 
                    boost::shared_ptr<vw::camera::CameraModel> right_camera_model,
                    vw::cartography::Datum const& datum,
                    std::string const& d_sub_file,
                    vw::Vector2 const& outlier_removal_params);
  
  // A small function used in making a copy of the transform for
  // map-projected images with a sanity check.
  TransPtr make_transform_copy(TransPtr trans);
  
  // Take a given disparity and make it between the original unaligned images
  void unalign_disparity(bool is_map_projected,
                         DispImageType    const& disparity,
                         TransPtr         const& left_trans,
                         TransPtr         const& right_trans,
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
                                 TransPtr         const& left_trans,
                                 TransPtr         const& right_trans,
                                 std::string      const& match_file,
                                 int max_num_matches,
                                 bool gen_triplets);
  
} // End namespace asp

#endif//__ASP_CORE_DISPARITY_PROCESSING_H__
