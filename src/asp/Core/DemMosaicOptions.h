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

/// \file DemMosaicOptions.h
/// Options structure for dem_mosaic

#ifndef __ASP_CORE_DEM_MOSAIC_OPTIONS_H__
#define __ASP_CORE_DEM_MOSAIC_OPTIONS_H__

#include <vw/FileIO/GdalWriteOptions.h>
#include <vw/Math/BBox.h>

#include <string>
#include <vector>
#include <set>
#include <limits>

namespace asp {

struct DemMosaicOptions: vw::GdalWriteOptions {
  std::string dem_list, out_prefix, target_srs_string,
    output_type, tile_list_str, this_dem_as_reference, weight_list, dem_list_file;
  std::vector<std::string> dem_files, weight_files;
  double tr, geo_tile_size;
  bool   has_out_nodata, force_projwin;
  double out_nodata_value;
  int    tile_size, tile_index, erode_len, priority_blending_len,
         extra_crop_len, hole_fill_len, block_size, save_dem_weight,
         fill_num_passes;
  double weights_exp, weights_blur_sigma, dem_blur_sigma;
  double nodata_threshold, fill_search_radius, fill_power, fill_percent, min_weight;
  bool   first, last, min, max, block_max, mean, stddev, median, nmad,
    count, tap, gdal_tap, save_index_map, use_centerline_weights, first_dem_as_reference, 
    propagate_nodata, no_border_blend, invert_weights;
  std::set<int> tile_list;
  vw::BBox2 projwin;

  DemMosaicOptions(): tr(0), geo_tile_size(0), has_out_nodata(false), force_projwin(false), 
             tile_index(-1), erode_len(0), priority_blending_len(0), extra_crop_len(0),
             hole_fill_len(0), block_size(0), save_dem_weight(-1), 
             fill_search_radius(0), fill_power(0), fill_percent(0), fill_num_passes(0),
             weights_exp(0), weights_blur_sigma(0.0), dem_blur_sigma(0.0),
             nodata_threshold(std::numeric_limits<double>::quiet_NaN()),
             first(false), last(false), min(false), max(false), block_max(false),
             mean(false), stddev(false), median(false), nmad(false),
             count(false), save_index_map(false), tap(false), gdal_tap(false),
             use_centerline_weights(false), first_dem_as_reference(false), 
             projwin(vw::BBox2()),
             invert_weights(false) {}
};

// Return the number of no-blending options selected
inline int numNoBlendOptions(DemMosaicOptions const& opt) {
  return int(opt.first) + int(opt.last) + int(opt.min) + int(opt.max)
    + int(opt.mean) + int(opt.stddev) + int(opt.median)
    + int(opt.nmad) + int(opt.count) + int(opt.block_max);
}

} // end namespace asp

#endif //__ASP_CORE_DEM_MOSAIC_OPTIONS_H__
