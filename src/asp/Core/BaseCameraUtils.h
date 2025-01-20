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

// Low-level camera utilities

#ifndef __CORE_BASE_CAMERA_UTILS_H__
#define __CORE_BASE_CAMERA_UTILS_H__

#include <vw/Camera/CameraModel.h>

namespace asp {

// Create the adjusted camera file name from the original camera filename,
// unless it is empty, and then use the image file name.
// - Convert dir1/image1.cub to out-prefix-image1.adjust
std::string bundle_adjust_file_name(std::string const& prefix, 
                                    std::string const& input_img,
                                    std::string const& input_cam);

// Read an .adjust file
void read_adjustments(std::string const& filename,
                      vw::Vector3      & position_correction,
                      vw::Quat         & pose_correction,
                      vw::Vector2      & pixel_offset, 
                      double           & scale);

/// Write global adjustments
// TODO(oalexan1): This should be unified with analogous logic in VW
void write_adjustments(std::string const& filename,
                        vw::Vector3 const& position_correction,
                        vw::Quat    const& pose_correction);

// If we have adjusted camera models, load them. The adjustment may be in the
// rotation matrix, camera center, or pixel offset. Otherwise return unadjusted
// cameras.
vw::CamPtr load_adjusted_model(vw::CamPtr cam,
                               std::string const& image_file,
                               std::string const& camera_file,
                               std::string const& ba_prefix,
                               vw::Vector2 const& pixel_offset);

// If both left-image-crop-win and right-image-crop win are specified,
// we crop the images to these boxes, and hence the need to keep
// the upper-left corners of the crop windows to handle the cameras correctly.
vw::Vector2 camera_pixel_offset(bool isMapProjected,
                                std::string const& left_image_file,
                                std::string const& right_image_file,
                                std::string const& curr_image_file);


} //end namespace asp

#endif//__CORE_BASE_CAMERA_UTILS_H__
