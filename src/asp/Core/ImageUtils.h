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

/// \file ImageUtils.h
/// Utility functions for handling images

#ifndef __ASP_CORE_IMAGE_UTILS_H__
#define __ASP_CORE_IMAGE_UTILS_H__

#include <vw/Image/ImageViewRef.h>
#include <vw/Camera/CameraModel.h>
#include <vw/InterestPoint/InterestData.h>

namespace vw {
  namespace cartography {
    class GeoReference;
  }
}

namespace asp {

/// Load an input image, georef, and nodata value
void load_image(std::string const& image_file,
                vw::ImageViewRef<double> & image, double & nodata,
                bool & has_georef, vw::cartography::GeoReference & georef);

/// Create a DEM ready to use for interpolation
void create_interp_dem(std::string const& dem_file,
                        vw::cartography::GeoReference & dem_georef,
                        vw::ImageViewRef<vw::PixelMask<double>> & interp_dem);

/// Take an interest point from a map projected image and convert it
/// to the corresponding IP in the original non-map-projected image.
/// - Return false if the pixel could not be converted.
bool projected_ip_to_raw_ip(vw::ip::InterestPoint &P,
                            vw::ImageViewRef<vw::PixelMask<double>> const& interp_dem,
                            vw::CamPtr camera_model,
                            vw::cartography::GeoReference const& georef,
                            vw::cartography::GeoReference const& dem_georef);

// Read keywords that describe how the images were map-projected.
void read_mapproj_header(std::string const& map_file,
                         // Outputs
                         std::string & adj_key, std::string & img_file_key,
                         std::string & cam_type_key, std::string & cam_file_key, 
                         std::string & dem_file_key,
                         std::string & adj_prefix,
                         std::string & image_file, std::string & cam_type,
                         std::string & cam_file, std::string & dem_file);

// Compute the min, max, mean, and standard deviation of an image object and
// write them to a file. This is not a member function.
// the "tag" is only used to make the log messages more descriptive.
// If prefix and image_path is set, will cache the results to a file.
vw::Vector<vw::float32,6>
gather_stats(vw::ImageViewRef<vw::PixelMask<float>> image,
             std::string const& tag,
             std::string const& prefix,
             std::string const& image_path,
             bool force_reuse_cache = false);

} // end namespace asp

#endif//__ASP_CORE_IMAGE_UTILS_H__
