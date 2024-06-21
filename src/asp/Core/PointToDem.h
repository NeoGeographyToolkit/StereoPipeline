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

/// \file PointToDem.h
/// Utility functions for point2dem

#ifndef __ASP_CORE_POINT_TO_DEM_H__
#define __ASP_CORE_POINT_TO_DEM_H__

#include <vw/FileIO/GdalWriteOptions.h>
#include <asp/Core/OrthoRasterizer.h>

namespace vw {
  namespace cartography {
    class Datum;
    class GeoReference;
  }
}

namespace asp {

// This is a list of types the user can specify for output with a dedicated
// command line flag.
enum ProjectionType {
  SINUSOIDAL,
  MERCATOR,
  TRANSVERSEMERCATOR,
  ORTHOGRAPHIC,
  STEREOGRAPHIC,
  OSTEREOGRAPHIC,
  GNOMONIC,
  LAMBERTAZIMUTHAL,
  UTM,
  PLATECARREE // has no effect, this is the default
};

struct DemOptions: vw::GdalWriteOptions {
  // Input
  std::vector<std::string> pointcloud_files, texture_files;

  // Settings
  std::vector<double> dem_spacing;
  float       nodata_value;
  double      semi_major, semi_minor;
  std::string reference_spheroid, datum;
  double      phi_rot, omega_rot, kappa_rot;
  std::string rot_order;
  double      proj_lat, proj_lon, proj_scale, false_easting, false_northing;
  double      lon_offset, lat_offset, height_offset;
  size_t      utm_zone;
  ProjectionType projection;
  bool        has_alpha, do_normalize, do_ortho, do_error, propagate_errors, no_dem,
              scalar_error;
  double      rounding_error;
  std::string target_srs_string;
  vw::BBox2   target_projwin;
  int         fsaa, dem_hole_fill_len, ortho_hole_fill_len, ortho_hole_fill_extra_len;
  bool        remove_outliers_with_pct, use_tukey_outlier_removal;
  vw::Vector2 remove_outliers_params;
  double      max_valid_triangulation_error;
  vw::Vector2 median_filter_params;
  int         erode_len;
  std::string csv_format_str, csv_proj4_str, filter;
  double      search_radius_factor, sigma_factor, default_grid_size_multiplier;
  bool        use_surface_sampling;
  bool        has_las_or_csv_or_pcd, auto_proj_center;
  vw::Vector2i max_output_size;
  bool        input_is_projected;

  // Output
  std::string out_prefix, output_file_type;
  
  // Constructor
  DemOptions();
};

// Create an antialiased DEM. This is old code. Needs to be wiped at some point.
ImageViewRef<PixelGray<float>>
generate_fsaa_raster(asp::OrthoRasterizerView const& rasterizer, 
                     DemOptions const& opt);

void parse_input_clouds_textures(std::vector<std::string> const& files,
                                 DemOptions& opt);

// Convert any LAS or CSV files to ASP tif files. We do some binning
// to make the spatial data more localized, to improve performance.
// - We will later wipe these temporary tifs.
void chip_convert_to_tif(DemOptions& opt, vw::cartography::Datum const& datum,
                               std::vector<std::string> & tmp_tifs);

// Rasterize a DEM
void do_software_rasterization(asp::OrthoRasterizerView& rasterizer,
                               DemOptions& opt,
                               cartography::GeoReference& georef,
                               std::int64_t * num_invalid_pixels);

} // end namespace asp

#endif//__ASP_CORE_POINT_TO_DEM_H__
