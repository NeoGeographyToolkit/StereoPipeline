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

/// \file PointCloudProcessing.h
///

// Process point clouds. This makes use of PDAL and ASP's csv logic.

#ifndef __ASP_CORE_POINT_CLOUD_PROCESSING_H__
#define __ASP_CORE_POINT_CLOUD_PROCESSING_H__

#include <string>
#include <vector>

namespace vw {
  
  class GdalWriteOptions;
  namespace cartography {
    class GeoReference;
  }
}
namespace asp {
  
  class CsvConv;
  
  // Builds a GeoReference from the first cloud having a georeference in the list
  bool georef_from_pc_files(std::vector<std::string> const& files,
			    vw::cartography::GeoReference & georef);


  /// Fetch a chunk of the las file of area TILE_LEN x TILE_LEN,
  /// split it into bins of spatially close points, and write
  /// it to disk as a tile in a vector tif image.
  void las_or_csv_to_tif(std::string const& in_file,
                         std::string const& out_prefix,
                         int num_rows, int block_size,
                         vw::GdalWriteOptions * opt,
                         vw::cartography::GeoReference const& csv_georef,
                         asp::CsvConv const& csv_conv,
                         std::vector<std::string> & out_files);

  
} // End namespace asp

#endif // __ASP_CORE_POINT_CLOUD_PROCESSING_H__
