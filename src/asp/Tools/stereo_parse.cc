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

/// \file stereo_parse.cc
///
/// This program is to allow python access to stereo settings.

#include <asp/Core/StereoSettings.h>
#include <asp/Core/AspProgramOptions.h>
#include <asp/Core/Macros.h>
#include <asp/Tools/stereo.h>
#include <asp/Sessions/StereoSession.h>
#include <asp/Sessions/StereoSessionFactory.h>
#include <asp/Core/DisparityProcessing.h>

#include <vw/Stereo/DisparityMap.h>
#include <vw/Cartography/GeoReferenceUtils.h>
#include <vw/Stereo/CorrelationView.h>
#include <vw/Cartography/shapeFile.h>

#include <xercesc/util/PlatformUtils.hpp>

using namespace vw;
using namespace asp;
using namespace std;
namespace fs = boost::filesystem;

// Find the tile at given location for a parallel_stereo run with local epipolar
// alignment.
void find_tile_at_loc(std::string const& tile_at_loc, ASPGlobalOptions const& opt) {

  if (opt.input_dem != "") 
    vw_throw(ArgumentErr() << "Option --tile-at-location does not work "
             << "with mapprojected images.\n");

  std::istringstream iss(tile_at_loc);
  double lon, lat, h;
  if (!(iss >> lon >> lat >> h))
    vw_throw(ArgumentErr() << "Could not parse --tile-at-location.\n");

  vw::cartography::GeoReference georef = opt.session->get_georef();

  vw::CamPtr left_camera_model, right_camera_model;
  opt.session->camera_models(left_camera_model, right_camera_model);
  
  Vector3 xyz = georef.datum().geodetic_to_cartesian(Vector3(lon, lat, h));
  Vector2 pix = left_camera_model->point_to_pixel(xyz);

  vw::TransformPtr tx_left = opt.session->tx_left();

  pix = tx_left->forward(pix);

  // Read the tiles
  std::string line;
  std::string dir_list = opt.out_prefix + "-dirList.txt";
  vw_out() << "Reading list of tiles: " << dir_list << "\n";
  std::ifstream ifs(dir_list);
  std::vector<BBox2i> boxes;
  bool success = false;
  while (ifs >> line) {
    std::string::size_type pos = line.find(opt.out_prefix);
    if (pos == std::string::npos) 
      vw_throw(ArgumentErr() << "Could not find the output prefix in " << dir_list << ".\n");

    std::string tile_name = line;
    line.replace(pos, opt.out_prefix.size() + 1, ""); // add 1 to replace the dash

    int start_x, start_y, wid_x, wid_y;
    int ans = sscanf(line.c_str(), "%d_%d_%d_%d", &start_x, &start_y, &wid_x, &wid_y);
    if (ans != 4) 
      vw_throw(ArgumentErr() << "Error parsing 4 numbers from string: " << line);

    BBox2i box(start_x, start_y, wid_x, wid_y);
    if (box.contains(pix)) {
      vw::vw_out() << "Tile with location: " << tile_name << "\n";
      success = true;
    }
  }

  if (!success)
    vw_out() << "No tile found at location.\n"; 
}

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
          
  // If mapproj, overwrite x and y with projected coordinates
  if (is_map_projected) {
    std::vector<double> proj_x, proj_y;
    for (size_t i = 0; i < x.size(); i++) {
      Vector2 pix_pt(x[i], y[i]);
      Vector2 proj_pt = georef.pixel_to_point(pix_pt);
      proj_x.push_back(proj_pt[0]);
      proj_y.push_back(proj_pt[1]);
    }
    x = proj_x;
    y = proj_y;
  }
  
  // Follow the dPoly API
  bool isPolyClosed = true;
  std::string color = "green", layer = "";
  poly.appendPolygon(x.size(), vw::geometry::vecPtr(x), vw::geometry::vecPtr(y),
                     isPolyClosed, color, layer);
  
  // This will be needed for QGIS
  tile_id_vec.push_back(tile_id);
}

// Produce the list of tiles. If D_sub is available, write only those tiles
// for which D_sub has valid values. Also save a shape file with the tiles
// and the tile index for each tile, to be read in QGIS for visualization.
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
  std::string fieldId = "tile_id"; 

  // We check for valid D_sub only if seed_mode is not 0 and not part of a multiview
  // run, as that one is tricky to get right, given that each pair run has its own D_sub.
  bool have_D_sub = false;
  std::string d_sub_file = output_prefix + "-D_sub.tif";
  vw::ImageView<vw::PixelMask<vw::Vector2f>> sub_disp;
  vw::Vector2 upsample_scale(0, 0);
  bool is_multiview = stereo_settings().part_of_multiview_run;
  if (stereo_settings().seed_mode != 0 && !is_multiview) {
    have_D_sub = true; 
    try {
      asp::load_D_sub_and_scale(output_prefix, d_sub_file, sub_disp, upsample_scale);
    } catch (...) {
      // Keep on going if we cannot load D_sub. In that case we cannot exclude
      // the tiles with no data.
      have_D_sub = false;
    }
  }

  int tile_x = parallel_tile_size[0];
  int tile_y = parallel_tile_size[1];
  int tiles_nx = int(ceil(double(trans_left_image_size[0]) / tile_x));
  int tiles_ny = int(ceil(double(trans_left_image_size[1]) / tile_y));

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
        int min_sub_x = floor((beg_x - sgm_collar_size) / upsample_scale[0]);
        int min_sub_y = floor((beg_y - sgm_collar_size) / upsample_scale[1]);
        int max_sub_x = ceil((beg_x + curr_tile_x + sgm_collar_size) / upsample_scale[0]);
        int max_sub_y = ceil((beg_y + curr_tile_y + sgm_collar_size) / upsample_scale[1]);
        
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
  std::string qmlFile = output_prefix + "-tiles.qml"; // must match shapefile name
  vw::vw_out() << "Writing shape file: " << shapeFile << "\n";
  vw::vw_out() << "Writing qml file: " << qmlFile << "\n";
  vw::geometry::write_shapefile(shapeFile, is_map_projected, georef, polyVec,
                                fieldId, tile_id_vec);
  vw::geometry::writeQml(qmlFile, fieldId);
}

int main(int argc, char* argv[]) {

  try {
    xercesc::XMLPlatformUtils::Initialize();
    stereo_register_sessions();

    bool verbose = true;
    vector<ASPGlobalOptions> opt_vec;
    string output_prefix;
    asp::parse_multiview(argc, argv, ParseDescription(),
                         verbose, output_prefix, opt_vec);
    if (opt_vec.empty())
      return 1;

    ASPGlobalOptions opt = opt_vec[0];

    if (!stereo_settings().tile_at_loc.empty()) {
      // Use this as a small utility in debugging parallel_stereo runs
      find_tile_at_loc(stereo_settings().tile_at_loc, opt);
      return 1;
    }
    
    vw_out() << "in_file1,"        << opt.in_file1        << "\n";
    vw_out() << "in_file2,"        << opt.in_file2        << "\n";
    vw_out() << "cam_file1,"       << opt.cam_file1       << "\n";
    vw_out() << "cam_file2,"       << opt.cam_file2       << "\n";
    vw_out() << "input_dem,"       << opt.input_dem       << "\n";
    vw_out() << "extra_argument1," << opt.extra_argument1 << "\n";
    vw_out() << "extra_argument2," << opt.extra_argument2 << "\n";
    vw_out() << "extra_argument3," << opt.extra_argument3 << "\n";

    vw_out() << "stereo_session,"   << opt.stereo_session << "\n";
    vw_out() << "stereo_default_filename," << opt.stereo_default_filename << "\n";
    vw_out() << "left_image_crop_win,"
             << stereo_settings().left_image_crop_win.min().x() << ","
             << stereo_settings().left_image_crop_win.min().y() << ","
             << stereo_settings().left_image_crop_win.width()   << ","
             << stereo_settings().left_image_crop_win.height()  << "\n";

    // This is different from opt.out_prefix for multiview
    vw_out() << "out_prefix," << output_prefix << "\n";

    Vector2i left_image_size  = file_image_size(opt.in_file1),
             right_image_size = file_image_size(opt.in_file2);
    vw_out() << "left_image_size,"  << left_image_size.x()  << "," 
             << left_image_size.y()  << "\n";
    vw_out() << "right_image_size," << right_image_size.x() << "," 
             << right_image_size.y() << "\n";

    std::string trans_left_image  = opt.out_prefix+"-L.tif";
    std::string trans_right_image = opt.out_prefix+"-R.tif";
    vw::vw_out() << "trans_left_image,"  << trans_left_image  << "\n";
    vw::vw_out() << "trans_right_image," << trans_right_image << "\n";
    
    // This is needed to create tiles in parallel_stereo
    vw::Vector2 trans_left_image_size(0, 0);
    if (fs::exists(trans_left_image))
      trans_left_image_size = file_image_size(trans_left_image);
    vw_out() << "trans_left_image_size," << trans_left_image_size.x() << "," 
             << trans_left_image_size.y() << "\n";

    // This separator needs to be in sync with parallel_stereo. It is needed to
    // extract from a line of text the portion after the separator.
    std::string sep = "--non-comma-separator--";
     
    cartography::GeoReference georef = opt.session->get_georef();
    vw_out() << "WKT" << sep << georef.get_wkt() << "\n";

    // Write the geotransform as a string as expected by GDAL's vrt xml format
    // TODO: Not sure if this will be useful as a member function in GeoReference.
    Matrix3x3 T = georef.transform();
    std::ostringstream os;
    os.precision(18);
    os << " "  << T(0, 2) << ",  " << T(0, 0) << ",  " << T(0, 1) << ",  "
       << " "  << T(1, 2) << ",  " << T(1, 0) << ",  " << T(1, 1);
    vw_out() << "GeoTransform" << sep << os.str() << "\n";

    // Some care is needed below. The transformed_window will be used
    // by parallel_stereo to parallelize stereo on a given user-specified
    // region. If both --left-image-crop-win and --right-image-crop-win
    // is specified, we already chopped the input images to these windows,
    // and created L.tif. Hence, we will work on the bd box on L.tif. However,
    // if just left-image-crop-win was set, we still use the full images,
    // but just the domain of computation is restricted. Hence we take user's
    // crop window, transform it to be in the L.tif coordinates, and use that one.
    bool crop_left = (stereo_settings().left_image_crop_win  != BBox2i(0, 0, 0, 0));
    BBox2i transformed_window;
    if (crop_left) {
      transformed_window.min() = Vector2(0, 0);
      transformed_window.max() = trans_left_image_size;
    }else{
      transformed_window = transformed_crop_win(opt);
    }
    vw_out() << "transformed_window," << transformed_window.min().x() << ","
             << transformed_window.min().y() << ","
             << transformed_window.width()   << ","
             << transformed_window.height()  << "\n";

    //vw_out() << "corr_tile_size," << ASPGlobalOptions::corr_tile_size() << "\n";
    vw_out() << "corr_tile_size," << stereo_settings().corr_tile_size_ovr << "\n";
    vw_out() << "rfne_tile_size," << ASPGlobalOptions::rfne_tile_size() << "\n";
    vw_out() << "tri_tile_size,"  << ASPGlobalOptions::tri_tile_size()  << "\n";
    vw_out() << "stereo_algorithm," << stereo_settings().stereo_algorithm << "\n";
    vw_out() << "alignment_method,"  << stereo_settings().alignment_method << "\n";
    vw_out() << "subpixel_mode," << stereo_settings().subpixel_mode << "\n";

    // Apart from default block matching, correlation will be done tiles
    // with padding, which is called here collar_size.
    vw::stereo::CorrelationAlgorithm stereo_alg
      = asp::stereo_alg_to_num(stereo_settings().stereo_algorithm);
    bool using_tiles = (stereo_alg > vw::stereo::VW_CORRELATION_BM ||
                        stereo_settings().alignment_method == "local_epipolar");
    
    int sgm_collar_size = 0;
    if (using_tiles)
      sgm_collar_size = stereo_settings().sgm_collar_size;
    vw_out() << "collar_size," << sgm_collar_size << "\n";

    vw_out() << "corr_memory_limit_mb," << stereo_settings().corr_memory_limit_mb << "\n";
    vw_out() << "save_lr_disp_diff," << stereo_settings().save_lr_disp_diff << "\n";
    vw_out() << "correlator_mode," << stereo_settings().correlator_mode << "\n";

    if (asp::stereo_settings().parallel_tile_size != vw::Vector2i(0, 0)) 
      produceTiles(opt, output_prefix, trans_left_image_size, 
                   asp::stereo_settings().parallel_tile_size, sgm_collar_size);

    // Attach a georeference to this disparity. 
    // TODO(oalexan1): Make this into a function
    std::string left_image_file = opt.out_prefix + "-L.tif";
    if (stereo_settings().attach_georeference_to_lowres_disparity &&
        fs::exists(left_image_file)) {

      cartography::GeoReference left_georef, left_sub_georef;
      bool   has_left_georef = read_georeference(left_georef, left_image_file);
      bool   has_nodata      = false;
      double output_nodata   = -32768.0;
      if (has_left_georef) {

        DiskImageView<float> left_image(left_image_file);
        for (int i = 0; i < 2; i++) {
          std::string d_sub_file = opt.out_prefix + "-D_sub.tif";
          if (i == 1) d_sub_file = opt.out_prefix + "-D_sub_spread.tif";
          if (!fs::exists(d_sub_file)) 
            continue;

          bool has_sub_georef = read_georeference(left_sub_georef, d_sub_file);
          if (has_sub_georef) {
            // If D_sub already has a georef, as with seed-mode 3, don't overwrite it.
            continue;
          }

          vw::ImageView<PixelMask<Vector2f>> d_sub;
          vw::read_image(d_sub, d_sub_file);
          // Account for scale.
          double left_scale = 0.5*(double(d_sub.cols())/left_image.cols() +
                                   double(d_sub.rows())/left_image.rows());
          left_sub_georef = resample(left_georef, left_scale);
          vw::cartography::block_write_gdal_image(d_sub_file, d_sub,
                                      has_left_georef, left_sub_georef,
                                      has_nodata, output_nodata,
                                      opt, TerminalProgressCallback("asp", "\t    D_sub: "));
        } // End i loop
      } // End has_left_georef
    } // End georef attach 

    xercesc::XMLPlatformUtils::Terminate();
  } ASP_STANDARD_CATCHES;

  return 0;
}
