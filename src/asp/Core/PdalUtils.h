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
#include <vw/Math/Vector.h>
#include <vw/Math/BBox.h>

#include <Eigen/Dense>

#include <string>

// Forward declaration of georef
namespace vw { 
  namespace cartography {
    class GeoReference;
  }
}

namespace pdal {
  class Reader;
  class Options;
}

namespace asp {

// Read the number of points in the LAS file. For COPC files, we usually
// care not for this, but for the number of points in a region.
std::int64_t las_file_size(std::string const& las_file);

// Read the georef from the las file and return true if the georef exists
bool georef_from_las(std::string const& las_file,
                    vw::cartography::GeoReference & georef);

// Check if a file is in the LAS COPC format
bool isCopc(std::string const& file);

// Save a point cloud and triangulation error to the LAS format.
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

// Try to load at least this many points from the LAS file. 
// TODO(oalexan1): This function should reduce the number of points
// if they are too many.
void load_las(std::string const& file_name,
              std::int64_t num_points_to_load,
              vw::BBox2 const& lonlat_box,
              vw::BBox2 const& copc_win, bool copc_read_all,
              bool calc_shift,
              vw::Vector3 & shift,
              vw::cartography::GeoReference const& geo,
              bool verbose, Eigen::MatrixXd & data);

// Apply a given transform to a LAS file and save it.
void apply_transform_to_las(std::string const& input_file,
                            std::string const& output_file,
                            vw::BBox2 const& copc_win, bool copc_read_all,
                            Eigen::MatrixXd const& T);

// Set up a reader for a LAS or COPC file 
void setupLasOrCopcReader(std::string const& in_file,
                          vw::BBox2 const& copc_win, bool copc_read_all,
                          boost::shared_ptr<pdal::Reader>& pdal_reader,
                          pdal::Options& read_options,
                          std::int64_t & num_total_points);

} // End namespace asp

#endif
