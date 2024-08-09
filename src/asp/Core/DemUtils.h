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

#ifndef __CORE_DEM_UTILS_H__
#define __CORE_DEM_UTILS_H__

#include <vw/Image/ImageViewRef.h>
#include <vw/Image/PixelMask.h>
#include <vw/Camera/CameraModel.h>

#include <fstream>
#include <iostream>
#include <string>

namespace asp {

// Given an image pixel, trace a ray to the ground and find the intersection.
void queryPixel(std::string const& dem_file, vw::CamPtr camera_model,
                vw::Vector2 const& query_pixel);


} //end namespace asp

#endif//__CORE_DEM_UTILS_H__

