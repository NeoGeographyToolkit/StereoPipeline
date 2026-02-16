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
#include <asp/Core/StereoSettings.h>
#include <asp/Core/DisparityProcessing.h>

#include <vw/Cartography/GeoTransform.h>
#include <vw/Cartography/GeoReferenceUtils.h>
#include <vw/Cartography/shapeFile.h>

#include <fstream>

namespace asp {

// Append a tile to a given set of polygons
void appendTileToPoly(int beg_x, int beg_y, int curr_tile_x, int curr_tile_y,
                      bool is_map_projected,
                      vw::cartography::GeoReference const& georef,
                      vw::geometry::dPoly & poly,
                      std::vector<int> & tile_id_vec,
                      size_t tile_id) {

  std::vector<double> x = {double(beg_x), double(beg_x + curr_tile_x),
                           double(beg_x + curr_tile_x), double(beg_x)};
  std::vector<double> y = {double(beg_y), double(beg_y),
                           double(beg_y + curr_tile_y), double(beg_y + curr_tile_y)};

  if (is_map_projected) {
    // If the images are mapprojected, overwrite x and y with projected coordinates
    std::vector<double> proj_x, proj_y;
    for (size_t i = 0; i < x.size(); i++) {
      vw::Vector2 pix_pt(x[i], y[i]);
      vw::Vector2 proj_pt = georef.pixel_to_point(pix_pt);
      proj_x.push_back(proj_pt[0]);
      proj_y.push_back(proj_pt[1]);
    }
    x = proj_x;
    y = proj_y;
  } else {
    // Only flip in y, to have the shapefiles agree with the images
    for (size_t i = 0; i < x.size(); i++)
      y[i] = -y[i];
  }

  // Follow the dPoly API
  bool isPolyClosed = true;
  std::string color = "green", layer = "";
  poly.appendPolygon(x.size(), vw::geometry::vecPtr(x), vw::geometry::vecPtr(y),
                     isPolyClosed, color, layer);

  // This will be needed for QGIS
  tile_id_vec.push_back(tile_id);
}

// Produce the list of tiles for parallel_stereo. If D_sub is available, write
// only those tiles for which D_sub has valid values. Also save a shape file
// with the tiles and the tile index for each tile, to be read in QGIS for
// visualization.
void produceTiles(bool is_map_projected,
                  std::string const& output_prefix,
                  vw::Vector2 const& trans_left_image_size,
                  vw::Vector2i const& parallel_tile_size,
                  int sgm_collar_size) {

  if (trans_left_image_size == vw::Vector2(0, 0))
      vw::vw_throw(vw::ArgumentErr() << "Cannot produce tiles without a valid L.tif.\n");

  // This georeference has at least the datum
  vw::cartography::GeoReference georef;
  // If is mapprojected, read the georef from L.tif, as that is what is used for tiling.
  // This should handle correctly --left-image-crop-win as well.
  if (is_map_projected) {
    std::string left_image_file = output_prefix + "-L.tif";
    bool has_georef = vw::cartography::read_georeference(georef, left_image_file);
    if (!has_georef)
      vw::vw_throw(vw::ArgumentErr() << "L.tif has no georeference, cannot produce tiles.\n");
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

// Produce the list of tiles covering the overlap region of two mapprojected images,
// for use with stereo_dist. Write as a text file. Write a shapefile for visualization.
void produceDistTileList(std::string const& in_file1,
                         std::string const& in_file2,
                         std::string const& output_prefix,
                         vw::Vector2i const& stereo_dist_tile_params) {

  int tile_size = stereo_dist_tile_params[0];
  int tile_padding = stereo_dist_tile_params[1];

  if (tile_size <= 0 || tile_padding <= 0)
    vw::vw_throw(vw::ArgumentErr() << "Tile size and padding must be positive.\n");

  // Read georefs from both images
  vw::cartography::GeoReference georef1, georef2;
  bool has_georef1 = vw::cartography::read_georeference(georef1, in_file1);
  bool has_georef2 = vw::cartography::read_georeference(georef2, in_file2);
  if (!has_georef1)
    vw::vw_throw(vw::ArgumentErr() << "Missing georeference in: " << in_file1 << "\n");
  if (!has_georef2)
    vw::vw_throw(vw::ArgumentErr() << "Missing georeference in: " << in_file2 << "\n");

  // Form bounding boxes in pixel coordinates
  vw::Vector2 dims1 = vw::file_image_size(in_file1);
  vw::Vector2 dims2 = vw::file_image_size(in_file2);
  vw::BBox2i bbox1_pix(0, 0, dims1[0], dims1[1]);
  vw::BBox2i bbox2_pix(0, 0, dims2[0], dims2[1]);

  // Form projected bounding box for image1 in its own projected coordinates
  vw::BBox2 bbox1_proj = georef1.pixel_to_point_bbox(bbox1_pix);

  // Form bounding box for image2 in image1's projected coordinates
  // GeoTransform converts from source (georef2) to destination (georef1)
  vw::cartography::GeoTransform geo_trans(georef2, georef1);
  vw::BBox2 bbox2_in_proj1 = geo_trans.pixel_to_point_bbox(bbox2_pix);

  // Find intersection in projected coordinates
  vw::BBox2 intersection_proj = bbox1_proj;
  intersection_proj.crop(bbox2_in_proj1);
  if (intersection_proj.empty())
    vw::vw_throw(vw::ArgumentErr() << "The two images do not overlap.\n");

  // Convert intersection to image1's pixel coordinates
  vw::BBox2i intersection_pix = georef1.point_to_pixel_bbox(intersection_proj);
  intersection_pix.expand(1);
  // Crop to image1's pixel box, just in case
  intersection_pix.crop(bbox1_pix);

  // Calculate number of tiles
  int width = intersection_pix.width();
  int height = intersection_pix.height();
  int tiles_nx = (int)std::ceil((double)width / tile_size);
  int tiles_ny = (int)std::ceil((double)height / tile_size);

  // Open output file
  std::string tile_list_file = output_prefix + "-distTileList.txt";
  std::ofstream ofs(tile_list_file.c_str());
  if (!ofs.good())
    vw::vw_throw(vw::ArgumentErr() << "Cannot open for writing: " << tile_list_file << "\n");

  vw::vw_out() << "Writing tile list: " << tile_list_file << "\n";
  vw::vw_out() << "Intersection region: " << intersection_pix << "\n";
  vw::vw_out() << "Tile size: " << tile_size << ", padding: " << tile_padding << "\n";

  // Prepare shapefile structure
  std::vector<vw::geometry::dPoly> polyVec(1);
  vw::geometry::dPoly& poly = polyVec[0];
  std::vector<int> tile_id_vec;
  std::string fieldId = "tile_id";

  // Generate tiles covering the intersection region
  int num_tiles = 0;
  for (int iy = 0; iy < tiles_ny; iy++) {
    for (int ix = 0; ix < tiles_nx; ix++) {

      // Tile position and size (without padding)
      int tile_x = intersection_pix.min().x() + ix * tile_size;
      int tile_y = intersection_pix.min().y() + iy * tile_size;
      int tile_w = std::min(tile_size, intersection_pix.max().x() - tile_x);
      int tile_h = std::min(tile_size, intersection_pix.max().y() - tile_y);

      // Write: tile_x tile_y tile_w tile_h tile_padding
      ofs << tile_x << " " << tile_y << " " << tile_w << " " << tile_h << " " << tile_padding << "\n";

      // Convert tile corners to projected coordinates for shapefile
      std::vector<double> px(4), py(4);
      vw::Vector2 pt;
      pt = georef1.pixel_to_point(vw::Vector2(tile_x, tile_y));
      px[0] = pt[0]; py[0] = pt[1];
      pt = georef1.pixel_to_point(vw::Vector2(tile_x + tile_w, tile_y));
      px[1] = pt[0]; py[1] = pt[1];
      pt = georef1.pixel_to_point(vw::Vector2(tile_x + tile_w, tile_y + tile_h));
      px[2] = pt[0]; py[2] = pt[1];
      pt = georef1.pixel_to_point(vw::Vector2(tile_x, tile_y + tile_h));
      px[3] = pt[0]; py[3] = pt[1];

      // Append polygon
      bool isPolyClosed = true;
      std::string color = "green", layer = "";
      poly.appendPolygon(px.size(), vw::geometry::vecPtr(px), vw::geometry::vecPtr(py),
                         isPolyClosed, color, layer);
      tile_id_vec.push_back(num_tiles);

      num_tiles++;
    }
  }

  ofs.close();
  vw::vw_out() << "Wrote " << num_tiles << " tiles.\n";

  // Write shapefile and qml file (for QGIS)
  std::string shapeFile = output_prefix + "-distTileList.shp";
  vw::vw_out() << "Writing shape file: " << shapeFile << "\n";
  bool is_map_projected = true; // These are mapprojected images
  vw::geometry::write_shapefile(shapeFile, is_map_projected, georef1, polyVec,
                                fieldId, tile_id_vec);
  std::string qmlFile = output_prefix + "-distTileList.qml";
  vw::vw_out() << "Writing qml file: " << qmlFile << "\n";
  vw::geometry::writeQml(qmlFile, fieldId);
}

// Handle the crop windows for distributed stereo mode. Here the left crop
// window is expanded by the collar size and the right crop window is
// auto-computed.
void handleDistCropWins(std::string const& left_image,
                        std::string const& right_image,
                        int collar_size,
                        // Outputs
                        vw::BBox2 & left_crop_win,
                        vw::BBox2 & right_crop_win) {

  // Sanity checks
  if (left_crop_win.width() <= 0 || left_crop_win.height() <= 0)
    vw::vw_throw(vw::ArgumentErr() << "In distributed stereo mode the left crop window "
              << "must have positive width and height.\n");
  if (collar_size <= 0)
    vw::vw_throw(vw::ArgumentErr() << "In distributed stereo mode --sgm-collar-size "
              << "must be positive.\n");
  if (right_crop_win != vw::BBox2(0, 0, 0, 0))
    vw::vw_throw(vw::ArgumentErr() << "In distributed stereo mode the right crop window "
              << "must not be set, as it will be auto-computed.\n");

  // Read georefs from both images
  vw::cartography::GeoReference left_georef, right_georef;
  bool has_left_georef = vw::cartography::read_georeference(left_georef, left_image);
  bool has_right_georef = vw::cartography::read_georeference(right_georef, right_image);
  if (!has_left_georef)
    vw::vw_throw(vw::ArgumentErr() << "Missing georeference in: " << left_image << "\n");
  if (!has_right_georef)
    vw::vw_throw(vw::ArgumentErr() << "Missing georeference in: " << right_image << "\n");

  // Get left image bbox
  vw::Vector2 left_dims = vw::file_image_size(left_image);
  vw::BBox2 left_bbox(0, 0, left_dims[0], left_dims[1]);
  // Expand left crop win by collar size
  left_crop_win.expand(collar_size);
  // Crop to left image bbox
  left_crop_win.crop(left_bbox);

  // Use geo transform to convert left crop win (pixels) to right image pixels
  // GeoTransform::forward_bbox converts pixel bbox from source to destination
  vw::cartography::GeoTransform geo_trans(left_georef, right_georef);
  right_crop_win = geo_trans.forward_bbox(left_crop_win);

  // Crop to right image bbox
  vw::Vector2 right_dims = vw::file_image_size(right_image);
  vw::BBox2 right_bbox(0, 0, right_dims[0], right_dims[1]);
  right_crop_win.crop(right_bbox);
}

} // end namespace asp
