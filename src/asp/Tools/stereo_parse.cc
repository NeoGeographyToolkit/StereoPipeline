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

#include <asp/Tools/stereo.h>
#include <vw/Stereo/DisparityMap.h>
#include <vw/Cartography/GeoReferenceUtils.h>
#include <vw/Stereo/CorrelationView.h>
#include <asp/Sessions/StereoSession.h>
#include <asp/Sessions/StereoSessionFactory.h>
#include <asp/Core/DisparityProcessing.h>
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
      std::cout << "Tile with location: " << tile_name << "\n";
      success = true;
    }
  }

  if (!success)
    vw_out() << "No tile found at location.\n"; 
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

    cartography::GeoReference georef = opt.session->get_georef();
    vw_out() << "WKT--non-comma-separator--" << georef.get_wkt() << "\n";

    // Write the geotransform as a string as expected by GDAL's vrt xml format
    // TODO: Not sure if this will be useful as a member function in GeoReference.
    Matrix3x3 T = georef.transform();
    std::ostringstream os;
    os.precision(18);
    os << " "  << T(0, 2) << ",  " << T(0, 0) << ",  " << T(0, 1) << ",  "
       << " "  << T(1, 2) << ",  " << T(1, 0) << ",  " << T(1, 1);
    vw_out() << "GeoTransform--non-comma-separator--" << os.str() << "\n";

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
    
    if (asp::stereo_settings().parallel_tile_size != vw::Vector2i(0, 0)) {
      // Produce the tiles for parallel_stereo. Skip the ones not having valid
      // disparity.
      // if (trans_left_image_size == vw::Vector2(0, 0))
      //   vw_throw(ArgumentErr() << "Cannot produce tiles without a valid L.tif.\n");

      std::string d_sub_file = opt.out_prefix + "-D_sub.tif";  
      vw::ImageView<vw::PixelMask<vw::Vector2f>> sub_disp;
      vw::Vector2 upsample_scale;
      asp::load_D_sub_and_scale(opt.out_prefix, d_sub_file, sub_disp, upsample_scale);
      std::cout << "--upsample_scale is " << upsample_scale << std::endl;
      
      std::cout << "--trans_left_image size is " << trans_left_image_size << std::endl;
      std::cout << "--sgm collar size is " << sgm_collar_size << std::endl;
      int tile_x = asp::stereo_settings().parallel_tile_size[0];
      int tile_y = asp::stereo_settings().parallel_tile_size[1];
      std::cout << "--tilew is " << tile_x << std::endl;
      std::cout << "--tileh is " << tile_y << std::endl;
      int tiles_nx = int(ceil(double(trans_left_image_size[0]) / tile_x));
      int tiles_ny = int(ceil(double(trans_left_image_size[1]) / tile_y));
      std::cout << "--tiles_nx is " << tiles_nx << std::endl;
      std::cout << "--tiles_ny is " << tiles_ny << std::endl;
      
      std::string dirList = opt.out_prefix + "-dirList.txt";
      // Open this for writing
      std::ofstream ofs(dirList.c_str()); 
      // Iterate in iy from 0 to tiles_ny - 1, and in ix from 0 to tiles_nx - 1.
      for (int iy = 0; iy < tiles_ny; iy++) {
        for (int ix = 0; ix < tiles_nx; ix++) {
          
          // Adjust for the tiles at the boundary
          int curr_tile_x = tile_x;
          int curr_tile_y = tile_y;
          if (ix == tiles_nx - 1)
            curr_tile_x = int(trans_left_image_size[0]) - ix * tile_x;
          if (iy == tiles_ny - 1)
            curr_tile_y = int(trans_left_image_size[1]) - iy * tile_y;
            
          int beg_x = ix * tile_x;
          int beg_y = iy * tile_y;
          
          int min_sub_x = floor((beg_x - sgm_collar_size) / upsample_scale[0]);
          int min_sub_y = floor((beg_y - sgm_collar_size) / upsample_scale[1]);
          int max_sub_x = ceil((beg_x + curr_tile_x + sgm_collar_size) / upsample_scale[0]);
          int max_sub_y = ceil((beg_y + curr_tile_y + sgm_collar_size) / upsample_scale[1]);
          
          min_sub_x = std::max(min_sub_x, 0);
          min_sub_y = std::max(min_sub_y, 0);
          max_sub_x = std::min(max_sub_x, sub_disp.cols() - 1);
          max_sub_y = std::min(max_sub_y, sub_disp.rows() - 1);
         
          bool has_valid_vals = false;
          for (int y = min_sub_y; y <= max_sub_y; y++) {
            for (int x = min_sub_x; x <= max_sub_x; x++) {
              if (is_valid(sub_disp(x, y))) {
                has_valid_vals = true;
                break;
              }
            }
            if (has_valid_vals) break;
          } 
          
          if (has_valid_vals)
            ofs << opt.out_prefix << "-" << beg_x << "_" << beg_y << "_" 
                << curr_tile_x << "_" << curr_tile_y << "\n";
        }
      }
    }

    // This block of code should be in its own executable but I am
    // reluctant to create one just for it. This functionality will be
    // invoked after low-res disparity is computed, whether done in
    // C++ or in Python. It will attach a georeference to this disparity.
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
          double left_scale = 0.5*( double(d_sub.cols())/left_image.cols() +
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
