// __BEGIN_LICENSE__
//  Copyright (c) 2006-2025, United States Government as represented by the
//  Administrator of the National Aeronautics and Space Administration. All
//  rights reserved.
//
//  The NASA Vision Workbench is licensed under the Apache License,
//  Version 2.0 (the "License"); you may not use this file except in
//  compliance with the License. You may obtain a copy of the License at
//  http://www.apache.org/licenses/LICENSE-2.0
//
//  Unless required by applicable law or agreed to in writing, software
//  distributed under the License is distributed on an "AS IS" BASIS,
//  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
//  See the License for the specific language governing permissions and
//  limitations under the License.
// __END_LICENSE__

#ifndef __ASP_PC_ALIGN_SLOPE_ASPECT_H__
#define __ASP_PC_ALIGN_SLOPE_ASPECT_H__

#include <vw/Image/ImageView.h>
#include <vw/Image/PixelMask.h>
#include <vw/Cartography/GeoReference.h>
namespace vw {
namespace cartography {

// Calculate the DEM slope and aspect in degrees. Use the same logic as gdaldem. 
void calcSlopeAspect(vw::ImageView<vw::PixelMask<float>> const& dem, 
                     vw::cartography::GeoReference const& georef,
                     // Outputs
                     vw::ImageView<vw::PixelMask<float>> & slope,
                     vw::ImageView<vw::PixelMask<float>> & aspect);

}} // namespace vw::cartography

#endif // __ASP_PC_ALIGN_SLOPE_ASPECT_H__
