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

/// \file PdalUtils.h
///

// Utility functions that use PDAL

#ifndef __ASP_CORE_PDAL_UTILS_H__
#define __ASP_CORE_PDAL_UTILS_H__

#include <vw/Image/ImageViewRef.h>

// Forward declaration of georef
namespace vw { 
  namespace cartography {
    class GeoReference;
  }
}

#include <string>
namespace asp {

// Read the number of points in the LAS file
std::int64_t las_file_size(std::string const& las_file);

// Read the georef from the las file and return true if the georef exists
bool georef_from_las(std::string const& las_file,
                    vw::cartography::GeoReference & georef);

// Save a point cloud and triangulation error to the LAS format
void write_las(bool has_georef, vw::cartography::GeoReference const& georef,
               vw::ImageViewRef<vw::Vector3> point_image,
               vw::ImageViewRef<double> error_image,
               vw::ImageViewRef<float> intensity,
               vw::ImageViewRef<double> horizontal_stddev,
               vw::ImageViewRef<double> vertical_stddev,
               vw::Vector3 const& offset,  vw::Vector3 const& scale,
               bool compressed, bool save_triangulation_error,
               double max_valid_triangulation_error,
               std::string const& out_prefix);
  
void read_las();
  
} // End namespace asp

#endif
