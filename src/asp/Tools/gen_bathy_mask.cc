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


// Create a mask in a raw image by projecting into the camera points a DEM
// in which areas not needing bathymetry correction were set to no-data,
// which will show up as 0 when projected, while DEM pixels needing
// that will show up as 1 in the mask.

// TODO(oalexan1): It will be more efficient to just project points on the
// mask perimeter, then use gdal_rasterize, if it works without a georeference.

/// \file gen_bathy_mask.cc
///

#include <asp/Core/PointUtils.h>
#include <asp/Core/Macros.h>
#include <asp/Core/Common.h>
#include <asp/Core/StereoSettings.h>
#include <asp/Sessions/StereoSession.h>
#include <asp/Sessions/StereoSessionFactory.h>

#include <vw/Core/Stopwatch.h>
#include <vw/FileIO/DiskImageUtils.h>
#include <vw/Cartography/shapeFile.h>
#include <vw/Image/InpaintView.h>

#include <Eigen/Dense>

using namespace vw;
using namespace vw::cartography;
namespace po = boost::program_options;
namespace fs = boost::filesystem;

typedef boost::scoped_ptr<asp::StereoSession> SessionPtr;

struct Options : vw::cartography::GdalWriteOptions {
  std::string dem_file, image_file, camera_file, output_mask_file, stereo_session_string;
  int num_samples_per_dem_pixel, hole_fill_len, mask_padding;
  Options() {}
};

void handle_arguments(int argc, char *argv[], Options& opt) {

  po::options_description general_options("General Options");
  general_options.add_options()
    ("dem",   po::value(&opt.dem_file),
     "Specify the dem to use for the mask, it should have no-data values only where water is.")
    ("num-samples-per-dem-pixel",   po::value(&opt.num_samples_per_dem_pixel)->default_value(1),
     "How many samples to pick between two DEM pixels when projecting into the camera to find the mask corresponding to the DEM. The value 0 means pick only the pixel itself.")
    ("hole-fill-len",   po::value(&opt.hole_fill_len)->default_value(20),
     "Fill small holes in the mask of these dimensions.")
    ("mask-padding",   po::value(&opt.mask_padding)->default_value(100),
     "How many pixels to use to pad the mask in order to avoid boundary artifacts.")
    ("output-mask",   po::value(&opt.output_mask_file),
     "The name of the output mask.")
    ("session-type,t",   po::value(&opt.stereo_session_string)->default_value(""),
     "Select the stereo session type to use for processing. Options: nadirpinhole dg rpc. Usually the program can select this automatically by the file extension but needs to be made explicit for rpc.");
    
  general_options.add( vw::cartography::GdalWriteOptionsDescription(opt) );
  
  po::options_description positional("");
  positional.add_options()
    ("image",   po::value(&opt.image_file),   "Satellite image file")
    ("camera",  po::value(&opt.camera_file),  "Camera file");
  
  po::positional_options_description positional_desc;
  positional_desc.add("image",   1);
  positional_desc.add("camera",  1);
  
  std::string usage("--dem burned_dem.tif image.tif camera.xml [other options]");
  bool allow_unregistered = false;
  std::vector<std::string> unregistered;
  po::variables_map vm =
    asp::check_command_line(argc, argv, opt, general_options, general_options,
                            positional, positional_desc, usage,
                            allow_unregistered, unregistered);

  if (opt.dem_file == "" || opt.image_file == "" || opt.camera_file == "" ||
      opt.output_mask_file == "")
    vw_throw( ArgumentErr() << "Not all inputs were specified.\n" << usage << general_options );

  if (opt.num_samples_per_dem_pixel < 0) 
    vw_throw( ArgumentErr() << "Must specify a non-negative number of samples.\n" );

}

int main( int argc, char *argv[] ) {

  Options opt;
  try {
    handle_arguments(argc, argv, opt);

    std::cout << "image is " << opt.image_file << std::endl;
    std::cout << "camera is " << opt.camera_file << std::endl;
    std::cout << "dem is " << opt.dem_file << std::endl;
    
    // Read the DEM and its associated data
    // TODO(oalexan1): Think more about the interpolation method
    std::cout << "Reading the DEM: " << opt.dem_file << std::endl;
    vw::cartography::GeoReference dem_georef;
    if (!read_georeference(dem_georef, opt.dem_file))
      vw_throw( ArgumentErr() << "The input DEM has no georeference.\n" );
    double dem_nodata_val = -std::numeric_limits<float>::max(); // note we use a float nodata
    if (!vw::read_nodata_val(opt.dem_file, dem_nodata_val))
      vw_throw( ArgumentErr() << "Could not read the DEM nodata value.\n");
    std::cout << "Read DEM nodata value: " << dem_nodata_val << std::endl;
    DiskImageView<float> dem(opt.dem_file);
    std::cout << "The DEM width and height are: " << dem.cols() << ' ' << dem.rows() << std::endl;
    ImageViewRef< PixelMask<float> > masked_dem = create_mask(dem, dem_nodata_val);
    ImageViewRef< PixelMask<float> > interp_dem = interpolate(masked_dem,
                                                              BilinearInterpolation(),
                                                              ConstantEdgeExtension());
    
    // Load the camera
    std::string out_prefix;
    SessionPtr session(asp::StereoSessionFactory::create(opt.stereo_session_string, // may change
                                                         opt,
                                                         opt.image_file, opt.image_file,
                                                         opt.camera_file, opt.camera_file,
                                                         out_prefix));
    boost::shared_ptr<vw::camera::CameraModel> camera_model
      = session->camera_model(opt.image_file, opt.camera_file);

    // Sanity check, to be used after the camera is loaded
    if (opt.stereo_session_string != "dg" &&
        opt.stereo_session_string != "rpc" &&
        opt.stereo_session_string != "nadirpinhole") 
      vw_throw( ArgumentErr() << "Bathymetry correction only works with dg, rpc, "
                << "and nadirpinhole sessions. Got: "
                << opt.stereo_session_string << ".\n" );

    // Project the DEM into the camera image, and record where it gets projected
    // TODO(oalexan1): This code must be made multi-threaded
    std::cout << "Creating the mask" << std::endl;
    vw::TerminalProgressCallback mask_tpc("asp", "\t--> ");
    double inc_amount = 1.0 / double(dem.cols());
    mask_tpc.report_progress(0);

    // The input image
    DiskImageView<float> input_image(opt.image_file);
    
    std::set<std::pair<int, int> > camera_pixels;
    for (int col = 0; col < dem.cols(); col++) {
      for (int row = 0; row < dem.rows(); row++) {

        if (!is_valid(masked_dem(col, row)))
          continue;

        for (int sx = 0; sx <= opt.num_samples_per_dem_pixel; sx++) {
          for (int sy = 0; sy <= opt.num_samples_per_dem_pixel; sy++) {

            double x, y, h;
            if (sx == 0 && sy == 0) {
              // Use this pixel before we try to look between pixels,
              // where there may be nothing, depending on whether
              // neighboring pixels are valid
              x = col;
              y = row;
              h = dem(col, row);
            } else {
              x = col + double(sx)/(opt.num_samples_per_dem_pixel + 1.0);
              y = row + double(sy)/(opt.num_samples_per_dem_pixel + 1.0);
              PixelMask<float> H = interp_dem(x, y);
              if (!is_valid(H))
                continue;
              h = H.child();
            }
          
            Vector2 pix(x, y);
            Vector3 llh;
            subvector(llh, 0, 2) = dem_georef.pixel_to_lonlat(pix); // lon and lat
            llh[2] = h;
            Vector3 xyz = dem_georef.datum().geodetic_to_cartesian(llh);
            Vector2 cam_pix = camera_model->point_to_pixel(xyz);

            // Cast to int and remove pixels which project out of range
            int cam_pix_x = round(cam_pix[0]);
            int cam_pix_y = round(cam_pix[1]);
            if (cam_pix_x < 0 || cam_pix_x >= input_image.cols() ||  
                cam_pix_y < 0 || cam_pix_y >= input_image.rows()) 
              continue;
            camera_pixels.insert(std::make_pair(cam_pix_x, cam_pix_y));
          }
        }
      }
      mask_tpc.report_incremental_progress( inc_amount );
    }
    mask_tpc.report_finished();


    // TODO(oalexan1): We may run out of memory here
    
    // Find the extent of the pixels
    int min_col = input_image.cols(), min_row = input_image.rows(), max_col = 0, max_row = 0;
    for (auto it = camera_pixels.begin(); it != camera_pixels.end(); it++) {
      min_col = std::min(min_col, (*it).first);
      max_col = std::max(max_col, (*it).first);
      min_row = std::min(min_row, (*it).second);
      max_row = std::max(max_row, (*it).second);
    }
    
    // Leave some padding at the boundary
    min_col = std::max(0, min_col - opt.mask_padding);
    min_row = std::max(0, min_row - opt.mask_padding);
    max_col = std::min(input_image.cols() - 1, max_col + opt.mask_padding);
    max_row = std::min(input_image.rows() - 1, max_row + opt.mask_padding);

    ImageView< PixelMask<int> > Mask(max_col - min_col + 1, max_row - min_row + 1);
    for (int col = 0; col < Mask.cols(); col++) {
      for (int row = 0; row < Mask.rows(); row++) {
        Mask(col, row) = 0;
        Mask(col, row).invalidate();
      }
    }
    
    for (auto it = camera_pixels.begin(); it != camera_pixels.end(); it++) {
      int col = (*it).first;
      int row = (*it).second;

      // Adjust to the crop dimensions
      Mask(col - min_col, row - min_row) = 1; 
      Mask(col - min_col, row - min_row).validate();
      // For debugging can use input_image(col, row);
    }

    // Fill holes. Let the number of pixels to fill be the area of the
    // specified input fill length.
    ImageViewRef< PixelMask<int> > filledMask
      = vw::fill_holes_grass(Mask, opt.hole_fill_len * opt.hole_fill_len);

    // Save the mask channel (after filling holes the value of the filled mask is still
    // 0, but its mask channel now has 1).
    bool has_mask_georef = false, has_mask_nodata = false;
    int mask_nodata = -1;
    vw::cartography::GeoReference mask_georef;
    std::cout << "Writing: " << opt.output_mask_file << std::endl;
    TerminalProgressCallback tpc("asp", ": ");
    block_write_gdal_image(opt.output_mask_file, select_channel(filledMask, 1),
                           has_mask_georef, mask_georef,
                           has_mask_nodata, mask_nodata, opt, tpc);

    std::cout << "Crop win for stereo: " << min_col << ' ' << min_row << ' '
              << filledMask.cols() << ' ' << filledMask.rows() << std::endl;
    
    
  } ASP_STANDARD_CATCHES;
  
  return 0;
}
