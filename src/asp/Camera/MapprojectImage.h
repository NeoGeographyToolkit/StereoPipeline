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

/// \file MapprojectImage.h
/// Algorithms specific to mapprojecting images.

#ifndef __MAPPROJECT_IMAGE_H__
#define __MAPPROJECT_IMAGE_H__

#include <vw/Cartography/GeoReference.h>
#include <vw/FileIO/GdalWriteOptions.h>
#include <vw/FileIO/DiskImageView.h>
#include <vw/Camera/CameraModel.h>

#include <string>

namespace asp {
  
struct MapprojOptions: vw::GdalWriteOptions {
  // Input
  std::string dem_file, image_file, camera_file, output_file, stereo_session,
    bundle_adjust_prefix, ref_map;
  bool isQuery, noGeoHeaderInfo, nearest_neighbor, parseOptions, aster_use_csm;
  bool multithreaded_model; // This is set based on the session type
  
  // Keep a copy of the model here to not have to pass it around separately
  boost::shared_ptr<vw::camera::CameraModel> camera_model;
  
  // Settings
  std::string target_srs_string, output_type, metadata;
  double nodata_value, tr, mpp, ppd, datum_offset;
  vw::BBox2 target_projwin, target_pixelwin;
  vw::Vector2 query_pixel;
};

// Project the image depending on image format.
void project_image(asp::MapprojOptions & opt, 
                   vw::cartography::GeoReference const& dem_georef,
                   vw::cartography::GeoReference const& target_georef, vw::cartography::GeoReference const& croppedGeoRef,
                   vw::Vector2i const& image_size, 
                   int virtual_image_width, int virtual_image_height,
                   vw::BBox2i const& croppedImageBB);

} // end namespace asp

#endif // __MAPPROJECT_IMAGE_H__
