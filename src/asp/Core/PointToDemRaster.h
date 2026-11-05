// __BEGIN_LICENSE__
//  Copyright (c) 2009-2026, United States Government as represented by the
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

/// \file PointToDemRaster.h
/// Point cloud rasterization routines for point2dem

#ifndef __ASP_CORE_POINT_TO_DEM_RASTER_H__
#define __ASP_CORE_POINT_TO_DEM_RASTER_H__

#include <cstdint>

namespace vw {
  class GeoReference;
}

namespace asp {

class OrthoRasterizerView;
struct DemOptions;

// Rasterize a DEM, and optionally error images, orthoimages, stddev, etc.
void rasterize_cloud(asp::OrthoRasterizerView& rasterizer,
                     DemOptions& opt,
                     vw::GeoReference& georef,
                     std::int64_t * num_invalid_pixels);

} // namespace asp

#endif // __ASP_CORE_POINT_TO_DEM_RASTER_H__
