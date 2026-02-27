// __BEGIN_LICENSE__
//  Copyright (c) 2006-2026, United States Government as represented by the
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

// \file GuiConstants.h
// 
// Magic numbers used across stereo_gui, centralized in one place.

#ifndef __STEREO_GUI_CONSTANTS_H__
#define __STEREO_GUI_CONSTANTS_H__

namespace asp {

// Max pixels in top pyramid level for image loading
const int TOP_IMAGE_MAX_PIX = 1000 * 1000;

// Default lowest-resolution subimage size
const int LOWRES_SIZE_DEFAULT = 1000 * 1000;

// Lowest-resolution subimage size when in preview (delay) mode
const int LOWRES_SIZE_PREVIEW = 8000 * 8000;

// Pyramid subsample factor for image loading in load()
const int LOAD_SUBSAMPLE = 4;

// Default pyramid subsample factor for DiskImagePyramidMultiChannel
const int PYRAMID_SUBSAMPLE = 2;

// Max screen distance (pixels) for match point pick/delete
const double MATCH_POINT_DISTANCE_LIMIT = 70.0;

} // namespace asp

#endif // __STEREO_GUI_CONSTANTS_H__
