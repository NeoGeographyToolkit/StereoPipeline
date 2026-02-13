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

/// \file StereoTiling.cc
///

#include <asp/Core/StereoTiling.h>

namespace asp {

// Produce the list of tiles for parallel_stereo. If D_sub is available, write
// only those tiles for which D_sub has valid values. Also save a shape file
// with the tiles and the tile index for each tile, to be read in QGIS for
// visualization.
void produceTiles(ASPGlobalOptions const& opt,
                  std::string const& output_prefix, 
                  vw::Vector2 const& trans_left_image_size,
                  vw::Vector2i const& parallel_tile_size,
                  int sgm_collar_size) {

  if (trans_left_image_size == vw::Vector2(0, 0))
      vw_throw(ArgumentErr() << "Cannot produce tiles without a valid L.tif.\n");

  // This georeference has at least the datum
  vw::cartography::GeoReference georef;
  bool is_map_projected = opt.session->isMapProjected();
  // If is mapprojected, read the georef from L.tif, as that is what is used for tiling.
  // This should handle correctly --left-image-crop-win as well.
  if (is_map_projected) {
    std::string left_image_file = output_prefix + "-L.tif";
    bool has_georef = vw::cartography::read_georeference(georef, left_image_file);
    if (!has_georef)
      vw_throw(ArgumentErr() << "L.tif has no georeference, cannot produce tiles.\n");
  }
  
  // Will store here the tile structure for visualization. The API requires
  // a vector of dPoly.
  std::vector<vw::geometry::dPoly> polyVec(1);
  vw::geometry::dPoly & poly = polyVec[0]; // alias
  std::vector<int> tile_id_vec;
  std::string fieldId = "tile_id"; // Will be used on reading in stereo_gui as well

  // We check for valid D_sub only if seed_mode is not 0 and not part of a multiview
  // run, as that one is tricky to get right, given that each pair run has its own D_sub.
  bool have_D_sub = false;
  std::string d_sub_file = output_prefix + "-D_sub.tif";
  vw::ImageView<vw::PixelMask<vw::Vector2f>> sub_disp;
  vw::Vector2 up_scale(0, 0);
  bool is_multiview = stereo_settings().part_of_multiview_run;
  if (stereo_settings().seed_mode != 0 && !is_multiview) {
    have_D_sub = true; 
    try {
      asp::load_D_sub_and_scale(output_prefix, d_sub_file, sub_disp, up_scale);
    } catch (...) {
      // Keep on going if we cannot load D_sub. In that case we cannot exclude
      // the tiles with no data.
      have_D_sub = false;
    }
  }

  int tile_x = parallel_tile_size[0];
  int tile_y = parallel_tile_size[1];
  int tiles_nx = int(std::ceil(double(trans_left_image_size[0]) / tile_x));
  int tiles_ny = int(std::ceil(double(trans_left_image_size[1]) / tile_y));

  // Open the file for writing
  std::string dirList = output_prefix + "-dirList.txt";
  std::ofstream ofs(dirList.c_str()); 

  // Go over all tiles
  size_t tile_id = 0;
  for (int iy = 0; iy < tiles_ny; iy++) {
    for (int ix = 0; ix < tiles_nx; ix++) {
      
      // Adjust for the tiles at the boundary
      int curr_tile_x = tile_x;
      int curr_tile_y = tile_y;
      if (ix == tiles_nx - 1)
        curr_tile_x = std::max(int(trans_left_image_size[0]) - ix * tile_x, 0);
      if (iy == tiles_ny - 1)
        curr_tile_y = std::max(int(trans_left_image_size[1]) - iy * tile_y, 0);
        
      int beg_x = ix * tile_x;
      int beg_y = iy * tile_y;
      
      bool has_valid_vals = true;
      if (have_D_sub) {
        has_valid_vals = false;
        int min_sub_x = std::floor((beg_x - sgm_collar_size) / up_scale[0]);
        int min_sub_y = std::floor((beg_y - sgm_collar_size) / up_scale[1]);
        int max_sub_x = std::ceil((beg_x + curr_tile_x + sgm_collar_size) / up_scale[0]);
        int max_sub_y = std::ceil((beg_y + curr_tile_y + sgm_collar_size) / up_scale[1]);
        
        min_sub_x = std::max(min_sub_x, 0);
        min_sub_y = std::max(min_sub_y, 0);
        max_sub_x = std::min(max_sub_x, sub_disp.cols() - 1);
        max_sub_y = std::min(max_sub_y, sub_disp.rows() - 1);
      
        for (int y = min_sub_y; y <= max_sub_y; y++) {
          for (int x = min_sub_x; x <= max_sub_x; x++) {
            if (is_valid(sub_disp(x, y))) {
              has_valid_vals = true;
              break;
            }
          }
          if (has_valid_vals) 
            break;
        } 
      }
      
      if (has_valid_vals) {
        // Save the tile
        ofs << output_prefix << "-" << beg_x << "_" << beg_y << "_" 
            << curr_tile_x << "_" << curr_tile_y << "\n";
            
        // Append the tile to the shape file structure
        appendTileToPoly(beg_x, beg_y, curr_tile_x, curr_tile_y,
                         is_map_projected, georef, poly, tile_id_vec, tile_id);
        tile_id++;
      }
    }
  }
  ofs.close();

  // Save the shape file and qml file. Will save a georef only if the images are
  // mapprojected. In that case the shapefile can be overlaid on top of L.tif.
  std::string shapeFile = output_prefix + "-tiles.shp";
  vw::vw_out() << "Writing shape file: " << shapeFile << "\n";
  vw::geometry::write_shapefile(shapeFile, is_map_projected, georef, polyVec,
                                fieldId, tile_id_vec);
  std::string qmlFile = output_prefix + "-tiles.qml"; // must match shapefile name
  vw::vw_out() << "Writing qml file: " << qmlFile << "\n";
  vw::geometry::writeQml(qmlFile, fieldId);
}

} // end namespace asp
