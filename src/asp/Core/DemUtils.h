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

// Forward declarations
namespace vw {
  namespace cartography {
    class GeoReference;
  }
  class GdalWriteOptions;
}

namespace asp {

// Given an image pixel, trace a ray to the ground and find the intersection.
void queryPixel(std::string const& dem_file, vw::CamPtr camera_model,
                vw::Vector2 const& query_pixel);


// Prepare a DEM file that encompasses a given image and with a given height,
// or reuse the one already available if agrees with what is intended.
void setupOrCheckDem(vw::GdalWriteOptions const& options,
                     vw::ImageViewRef<float> img,
                     vw::cartography::GeoReference const& image_georef,
                     std::string const& tag, 
                     std::string const& out_prefix, 
                     double dem_height,
                     // Outputs
                     std::string & dem_path);

// Given two DEMs, warp the second one to the first one's georef, and crop
// to a pixel box in first DEM's coordinates.
vw::ImageViewRef<vw::PixelMask<double>> 
  warpCrop(vw::ImageViewRef<double> dem2, 
           double dem2_nodata,
           vw::cartography::GeoReference const& dem2_georef,
           vw::cartography::GeoReference const& dem1_georef,
           vw::BBox2i const& dem1_crop_box,
           std::string const& interp_type);

// Estimate the GSD of a DEM in meters
void calcGsd(vw::ImageViewRef<double> const& dem,
             vw::cartography::GeoReference const& geo, double nodata_val,
             int sample_col_rate, int sample_row_rate,
             double & gridx, double & gridy);

} //end namespace asp

#endif//__CORE_DEM_UTILS_H__

