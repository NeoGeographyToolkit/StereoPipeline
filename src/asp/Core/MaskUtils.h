// __BEGIN_LICENSE__
//  Copyright (c) 2009-2025, United States Government as represented by the
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

/// \file MaskUtils.h
///
/// Utilities for operating on image masks.

#ifndef __ASP_CORE_MASK_UTILS_H__
#define __ASP_CORE_MASK_UTILS_H__

#include <vw/Image/ImageViewRef.h>
#include <vw/Image/PixelMask.h>
#include <vw/Core/FundamentalTypes.h>

namespace asp {

// Grow the valid region of a mask by 'radius' pixels. A pixel becomes valid if
// the input mask has a valid pixel within 'radius' (Manhattan distance) pixels.
// The image border is treated as valid, as in vw::grassfire with its default
// settings. The returned view is lazy: each tile is computed on demand from the
// input over a small collar, so it streams and does not hold the whole mask in
// memory. Used to add geolocation slack to the mapprojected overlap intersection
// (see --mapproj-geolocation-uncertainty). A radius of zero returns the input
// unchanged.
vw::ImageViewRef<vw::PixelMask<vw::uint8>>
expandMask(vw::ImageViewRef<vw::PixelMask<vw::uint8>> const& mask, int radius);

} // namespace asp

#endif // __ASP_CORE_MASK_UTILS_H__
