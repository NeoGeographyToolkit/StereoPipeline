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

/// \file StereoTiling.h
///

#ifndef __ASP_CORE_STEREO_TILING_H__
#define __ASP_CORE_STEREO_TILING_H__

#include <string>
#include <vw/Math/BBox.h>
#include <vw/Math/Vector.h>

namespace asp {

// Produce the list of tiles for parallel_stereo. If D_sub is available, write
// only those tiles for which D_sub has valid values. Also save a shape file
// with the tiles and the tile index for each tile, to be read in QGIS for
// visualization.
void produceTiles(bool is_map_projected,
                  std::string const& output_prefix,
                  vw::Vector2 const& trans_left_image_size,
                  vw::Vector2i const& parallel_tile_size,
                  int sgm_collar_size);

// Produce the list of tiles covering the overlap region of two mapprojected images,
// for use with stereo_dist. Write as a text file and a shapefile for visualization.
void produceDistTileList(std::string const& in_file1,
                         std::string const& in_file2,
                         std::string const& output_prefix,
                         vw::Vector2i const& stereo_dist_tile_params);

// Handle the crop windows for distributed stereo mode. Here the left crop
// window is expanded by the collar size and the right crop window is
// auto-computed.
void handleDistCropWins(std::string const& left_image,
                        std::string const& right_image,
                        int collar_size,
                        // Outputs
                        vw::BBox2 & left_crop_win,
                        vw::BBox2 & right_crop_win);

} // end namespace asp

#endif // __ASP_CORE_STEREO_TILING_H__
