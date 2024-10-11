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

/// \file DataLoader.cc
///

#include <asp/Core/DataLoader.h>
#include <asp/Core/EigenUtils.h>
#include <asp/Core/PointUtils.h>

using namespace vw;

namespace asp {

void load_csv_or_dem(std::string const& csv_format_str, std::string const& csv_srs_str,
                     std::string const& reference_terrain,
                     int max_num_reference_points,
                     vw::cartography::GeoReference & geo, // may change
                     // Outputs
                     std::vector<vw::Vector3> & ref_data) {
  
  // Wipe the output
  ref_data.clear();
  
  asp::CsvConv csv_conv;
  csv_conv.parse_csv_format(csv_format_str, csv_srs_str);
  csv_conv.parse_georef(geo);
  
  vw::BBox2 lonlat_box; // not used
  bool      calc_shift = false;
  Vector3   shift; // must be set to 0
  bool      is_lola_rdr_format;
  double    mean_longitude;
  bool      verbose = true;
  asp::DoubleMatrix data;
  
  // Read the reference terrain
  vw_out() << "Loading at most " << max_num_reference_points << " points from "
           << reference_terrain << std::endl;
  std::string file_type = asp::get_cloud_type(reference_terrain);
  if (file_type == "DEM") 
    asp::load_dem(reference_terrain,  
                  max_num_reference_points, lonlat_box,  
                  calc_shift, shift, verbose, data);
  
  else if (file_type == "CSV")
    asp::load_csv(reference_terrain,  max_num_reference_points,
                  lonlat_box, calc_shift, shift, geo,  
                    csv_conv, is_lola_rdr_format, mean_longitude, verbose,  
                  data);
  else
    vw_throw(ArgumentErr() << "Unsupported file: " << reference_terrain << " of type "
             << file_type << ".\n");

  int num_cols = data.cols();
  ref_data.clear();
  for (int data_col = 0; data_col < num_cols; data_col++) {
    vw::Vector3 reference_xyz;

    for (int row = 0; row < asp::DIM; row++)
      reference_xyz[row] = data(row, data_col);

    ref_data.push_back(reference_xyz);
  }
  
}
  
} // end namespace asp
