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

// Create an RPC model from point pairs obtained by sampling a given lon-lat-height box
// or a given DEM. Imitate to some extent the DG WV camera
// model. Optionally save a TIF version of the input image restricted to the same box.
// The saved image and RPC file can be used for stereo with S2P, ASP, and SETSM
// some work needs to be done for these packages to give correct results off-Earth.

#include <asp/Sessions/StereoSessionFactory.h>
#include <asp/Core/Common.h>
#include <asp/Core/Macros.h>
#include <asp/Core/FileUtils.h>
#include <asp/Camera/RPCModelGen.h>
#include <asp/Core/PointUtils.h>
#include <asp/Camera/RpcUtils.h>

#include <vw/FileIO/DiskImageView.h>
#include <vw/Core/StringUtils.h>
#include <vw/Camera/PinholeModel.h>
#include <vw/Cartography/Datum.h>
#include <vw/Cartography/GeoReference.h>
#include <vw/Cartography/CameraBBox.h>
#include <vw/FileIO/FileUtils.h>

#include <limits>
#include <cstring>

#include <boost/filesystem.hpp>
#include <boost/algorithm/string.hpp>
#include <boost/program_options.hpp>

namespace fs = boost::filesystem;
namespace po = boost::program_options;

using namespace vw;
using namespace vw::camera;
using namespace std;
using namespace vw::cartography;

struct Options : public vw::GdalWriteOptions {
  double penalty_weight;
  string image_file, camera_file, output_rpc, stereo_session, bundle_adjust_prefix,
    datum_str, dem_file, target_srs_string;
  bool no_crop, skip_computing_rpc, save_tif, has_output_nodata;
  BBox2 lon_lat_range;
  BBox2i image_crop_box;
  Vector2 height_range;
  float input_nodata_value, output_nodata_value;
  double semi_major, semi_minor;
  double gsd;
  int num_samples;
  Datum datum;
  Options(): penalty_weight(-1.0), no_crop(false),
             skip_computing_rpc(false), save_tif(false), has_output_nodata(false),
             gsd(-1.0), num_samples(-1) {}
};

void handle_arguments(int argc, char *argv[], Options& opt) {
  po::options_description general_options("");
  float nan = std::numeric_limits<float>::quiet_NaN();
  general_options.add_options()
    ("datum", po::value(&opt.datum_str)->default_value(""),
     "Use this datum to interpret the heights. Options: WGS_1984, D_MOON (1,737,400 meters), D_MARS (3,396,190 meters), MOLA (3,396,000 meters), NAD83, WGS72, and NAD27. Also accepted: Earth (=WGS_1984), Mars (=D_MARS), Moon (=D_MOON).")
    ("semi-major-axis", po::value(&opt.semi_major)->default_value(0), "Explicitly set the datum semi-major axis in meters.")
    ("semi-minor-axis", po::value(&opt.semi_minor)->default_value(0), "Explicitly set the datum semi-minor axis in meters.")
    ("t_srs", po::value(&opt.target_srs_string)->default_value(""), "Specify a GDAL projection string instead of the datum (in WKT, GeoJSON, or PROJ format).")
    ("lon-lat-range", po::value(&opt.lon_lat_range)->default_value(BBox2(0,0,0,0), "0 0 0 0"),
     "The longitude-latitude range in which to compute the RPC model. Specify in the "
     "format: lon_min lat_min lon_max lat_max.")
    ("height-range", po::value(&opt.height_range)->default_value(Vector2(0,0),"0 0"),
     "Minimum and maximum heights above the datum in which to compute the RPC model.")
    ("dem-file", po::value(&opt.dem_file)->default_value(""),
     "Compute the longitude-latitude-height box in which to fit the RPC camera as the "
      "bounding box of the portion of this DEM that is seen by the input camera.")
    ("num-samples", po::value(&opt.num_samples)->default_value(40),
     "How many samples to use in each direction in the longitude-latitude-height range.")
    ("penalty-weight", po::value(&opt.penalty_weight)->default_value(0.03), // check here!
     "A higher penalty weight will result in smaller higher-order RPC coefficients.")
    ("save-tif-image", po::bool_switch(&opt.save_tif)->default_value(false),
     "Save a TIF version of the input image that approximately corresponds to the input longitude-latitude-height range and which can be used for stereo together with the RPC model.")
    ("input-nodata-value", po::value(&opt.input_nodata_value)->default_value(nan),
     "Set the image input nodata value.")
    ("output-nodata-value", po::value(&opt.output_nodata_value)->default_value(nan),
     "Set the image output nodata value. If not specified, the input nodata value will be used.")
    ("session-type,t", po::value(&opt.stereo_session),
     "Select the stereo session type to use for processing. Usually the program can select this automatically by the file extension, except for xml cameras. See the doc for options.")
    ("bundle-adjust-prefix", po::value(&opt.bundle_adjust_prefix),
     "Use the camera adjustment obtained by previously running bundle_adjust with this output prefix.")
    ("image-crop-box", po::value(&opt.image_crop_box)->default_value(BBox2i(0,0,0,0), "0 0 0 0"),
     "The output image and RPC model should not exceed this box, specified in input image pixels as minx miny widx widy.")
    ("no-crop", po::bool_switch(&opt.no_crop)->default_value(false),
      "Try to create an RPC model over the entire input image, even if the input longitude-latitude-height box covers just a small portion of it. Not recommended.")
    ("skip-computing-rpc", po::bool_switch(&opt.skip_computing_rpc)->default_value(false),
     "Skip computing the RPC model.")
    ("gsd", po::value(&opt.gsd)->default_value(-1),
     "Expected resolution on the ground, in meters. This is needed for SETSM.");

  general_options.add(vw::GdalWriteOptionsDescription(opt));

  po::options_description positional("");
  positional.add_options()
    ("camera-image", po::value(&opt.image_file))
    ("camera-model", po::value(&opt.camera_file))
    ("output-rpc" , po::value(&opt.output_rpc));

  po::positional_options_description positional_desc;
  positional_desc.add("camera-image",1);
  positional_desc.add("camera-model",1);
  positional_desc.add("output-rpc", 1);

  string usage("[options] <camera-image> <camera-model> <output-rpc>");
  bool allow_unregistered = false;
  std::vector<std::string> unregistered;
  po::variables_map vm =
    asp::check_command_line(argc, argv, opt, general_options, general_options,
                            positional, positional_desc, usage,
                            allow_unregistered, unregistered);

  if (opt.image_file.empty())
    vw_throw(ArgumentErr() << "Missing input image.\n" << usage << general_options);

  if (boost::iends_with(opt.image_file, ".cub") && opt.stereo_session == "")
    opt.stereo_session = "isis";

  // Need this to be able to load adjusted camera models. That will happen
  // in the stereo session.
  asp::stereo_settings().bundle_adjust_prefix = opt.bundle_adjust_prefix;

  // This is a bug fix. The user by mistake passed in an empty projection string.
  if (!vm["t_srs"].defaulted() && opt.target_srs_string.empty())
    vw_throw(ArgumentErr() 
             << "The value of --t_srs is empty. Then it must not be set at all.\n");

  // Swap min and max if need be
  if (opt.lon_lat_range.min().x() > opt.lon_lat_range.max().x())
    std::swap(opt.lon_lat_range.min().x(), opt.lon_lat_range.max().x());
  if (opt.lon_lat_range.min().y() > opt.lon_lat_range.max().y())
    std::swap(opt.lon_lat_range.min().y(), opt.lon_lat_range.max().y());

  // If we cannot read the data from a DEM, must specify a lot of things.
  if (opt.dem_file.empty()) {

    // See if the user specified the datum outside of the srs string
    bool have_user_datum = asp::read_user_datum(opt.semi_major, opt.semi_minor,
                                                opt.datum_str, opt.datum);
    // Set the srs string into georef.
    if (!opt.target_srs_string.empty()) {
      vw::cartography::GeoReference georef;
      asp::set_srs_string(opt.target_srs_string, have_user_datum, opt.datum, georef);
      opt.datum = georef.datum();
    }

    if (opt.datum_str.empty() && !have_user_datum && opt.target_srs_string.empty())
      vw_throw(ArgumentErr() << "Missing input datum. Must set one of: "
                << "--datum,  --t_srs, --semi-major-axis, and --semi-minor-axis, "
                << "--t_srs, or --dem-file.\n" << usage << general_options);

    if (opt.height_range[0] >= opt.height_range[1])
      vw_throw(ArgumentErr() << "Must specify a valid range of heights.\n"
                << usage << general_options);

    if (opt.lon_lat_range.empty())
      vw_throw(ArgumentErr() << "Must specify a valid range for longitude and latitude.\n"
                << usage << general_options);

    vw_out() << "Height range is " << opt.height_range[0] << ' ' << opt.height_range[1] 
             << " meters.\n";
    vw_out() << "Lon-lat range is " << opt.lon_lat_range.min() << ' ' 
             << opt.lon_lat_range.max() << " degrees.\n";
  }

  // If the DEM file is provided, the lon-lat and height ranges must not be set 
  if (!opt.dem_file.empty()) {
    if (opt.lon_lat_range != BBox2(0,0,0,0))
      vw_throw(ArgumentErr() << "Cannot specify both a DEM file and a lon-lat range.\n");
    if (opt.height_range != Vector2(0,0))
      vw_throw(ArgumentErr() << "Cannot specify both a DEM file and a height range.\n");
  }
  
  // Convert from width and height to min and max
  if (!opt.image_crop_box.empty()) {
    BBox2 b = opt.image_crop_box; // make a copy
    opt.image_crop_box = BBox2i(b.min().x(), b.min().y(), b.max().x(), b.max().y());
  }
  
  // There must be at least 2 samples in each dimension
  if (opt.num_samples < 2)
    vw_throw(ArgumentErr() << "Must have at least 2 samples in each dimension.\n");
}


// Add pixel and llh samples along the perimeter and diagonals of image_box.
// using the DEM.
void sample_dem_perim_diag(BBox2 const& image_box, 
                           vw::CamPtr cam,
                           ImageViewRef<PixelMask<float>> dem,
                           GeoReference const& dem_geo,
                           // Outputs (append to these)
                           std::vector<Vector3> & all_llh,
                           std::vector<Vector2> & all_pixels) {

  // Reduce the max by 1, as sample_float_box() assumes the max is inclusive
  BBox2 b = image_box;
  b.max() -= Vector2(1, 1);

  // Calc samples on box perimeter and diagonals
  int num_steps = 100;
  std::vector<vw::Vector2> points;
  vw::cartography::sample_float_box(b, points, num_steps);
  
  double height_guess = vw::cartography::demHeightGuess(dem);
  vw::Vector3 xyz_guess(0, 0, 0); // will be updated
  
  for (size_t j = 0; j < points.size(); j++) {
    vw::Vector2 pix = points[j];
    
    // Intersect the ray going from the given camera pixel with a DEM
    // Use xyz_guess as initial guess and overwrite it with the new value
    bool treat_nodata_as_zero = false;
    bool has_intersection = false;
    double max_abs_tol = 1e-14;
    double max_rel_tol = max_abs_tol;
    double dem_height_error_tol = 1e-3; // 1 mm
    int num_max_iter = 100;
    vw::Vector3 xyz;
    try {
      
      // Intersect with the DEM
      xyz = vw::cartography::
        camera_pixel_to_dem_xyz(cam->camera_center(pix), 
                                cam->pixel_to_vector(pix),
                                dem,
                                dem_geo, 
                                treat_nodata_as_zero,
                                has_intersection, 
                                dem_height_error_tol, 
                                max_abs_tol, max_rel_tol, 
                                num_max_iter, 
                                xyz_guess, 
                                height_guess);
    } catch (...) {
      continue;
    }

    if (!has_intersection || xyz == vw::Vector3())
      continue;

    // Update the guess for nex time, now that we have a valid intersection point
    xyz_guess = xyz;

    vw::Vector3 llh = dem_geo.datum().cartesian_to_geodetic(xyz);
      
    all_llh.push_back(llh);
    all_pixels.push_back(pix);
    
  } // end loop through points
  
  return;
}

// Compute the lon-lat box and height range from the DEM
void calc_llh_bbox_from_dem(Options & opt, vw::CamPtr cam,
                            vw::BBox2 const& image_box,
                            ImageViewRef<PixelMask<float>> input_img) {
  
  vw::vw_out() << "Estimating the lon-lat-height range from DEM: " << opt.dem_file << "\n";
  
  std::vector<Vector3> all_llh;
  std::vector<Vector2> all_pixels;
  
  float dem_nodata_val = -std::numeric_limits<float>::max(); 
  vw::read_nodata_val(opt.dem_file, dem_nodata_val);

  ImageViewRef<PixelMask<float>> dem 
    = create_mask(DiskImageView<float>(opt.dem_file), dem_nodata_val);

  GeoReference dem_geo;
  if (!read_georeference(dem_geo, opt.dem_file))
    vw_throw(ArgumentErr() << "Missing georef in DEM: " << opt.dem_file << ".\n");

  // Get the datum from the DEM
  opt.datum = dem_geo.datum();

  // If the DEM is too big, we need to skip points. About
  // 40,000 points should be good enough to determine 78 RPC
  // coefficients.
  double delta_col = std::max(1.0, dem.cols()/double(opt.num_samples));
  double delta_row = std::max(1.0, dem.rows()/double(opt.num_samples));
  
  vw::TerminalProgressCallback tpc("asp", "\t--> ");  
  double inc_amount = delta_col / dem.cols();
  
  tpc.report_progress(0);
  for (double dcol = 0; dcol < dem.cols(); dcol += delta_col) {
    for (double drow = 0; drow < dem.rows(); drow += delta_row) {
      int col = dcol, row = drow; // cast to int

      if (!is_valid(dem(col, row))) 
        continue;

      Vector2 pix(col, row);
      Vector2 lonlat = dem_geo.pixel_to_lonlat(pix);

      // Lon lat height
      Vector3 llh;
      llh[0] = lonlat[0]; llh[1] = lonlat[1]; llh[2] = dem(col, row).child();
      Vector3 xyz = opt.datum.geodetic_to_cartesian(llh);

      // Go back to llh. This is a bugfix for the 360 deg offset problem.
      llh = opt.datum.cartesian_to_geodetic(xyz);

      Vector2 cam_pix;
      try {
        // The point_to_pixel function can be capricious
        cam_pix = cam->point_to_pixel(xyz);
      } catch(...) {
        continue;
      }

      if (image_box.contains(cam_pix) && is_valid(input_img(cam_pix[0], cam_pix[1]))) {
        all_llh.push_back(llh);
        all_pixels.push_back(cam_pix);
      }

    }
    tpc.report_incremental_progress(inc_amount);
  } // end loop through DEM pixels
  tpc.report_finished();

  // Add pixel and llh samples along the perimeter and diagonals of image_box.
  // using the DEM.
  sample_dem_perim_diag(image_box, cam, dem, dem_geo, all_llh, all_pixels);

  // Based on these, find th lon-lat and height ranges
  opt.lon_lat_range = BBox2();
  double big = std::numeric_limits<double>::max();
  opt.height_range  = Vector2(big, -big);
  for (size_t i = 0; i < all_llh.size(); i++) {
    Vector3 llh = all_llh[i];
    opt.lon_lat_range.grow(Vector2(llh[0], llh[1]));
    opt.height_range[0] = std::min(opt.height_range[0], llh[2]);
    opt.height_range[1] = std::max(opt.height_range[1], llh[2]);
  }
  
  vw::vw_out() << "Computed lon-lat range: " << opt.lon_lat_range << "\n";
  vw::vw_out() << "Computed height range: " << opt.height_range << "\n";
  
  return;
}

int main(int argc, char *argv[]) {

  Options opt;
  try {

    handle_arguments(argc, argv, opt);
    
    bool allow_map_promote = false;
    asp::SessionPtr session(asp::StereoSessionFactory::create
                       (opt.stereo_session, // may change
                        opt,
                        opt.image_file, opt.image_file,
                        opt.camera_file, opt.camera_file,
                        opt.output_rpc,
                        opt.dem_file,
                        allow_map_promote));

    // If the session was passed in or guessed isis or rpc, adjust for the fact
    // that the isis .cub file also has camera info.
    if (opt.output_rpc.empty() &&
         ((session->name() == "isis"         ||
           session->name() == "isismapisis") ||
          session->name()  == "rpc")) {
      // The user did not provide an output file. Then the camera
      // information is contained within the image file and what is in
      // the camera file is actually the output file.
      opt.output_rpc  = opt.camera_file;
      opt.camera_file = opt.image_file;
    }

    if (opt.camera_file.empty())
      vw_throw(ArgumentErr() << "Missing input camera.\n");

    if (opt.output_rpc.empty())
      vw_throw(ArgumentErr() << "Missing output RPC file.\n");

    // Create the output directory
    vw::create_out_dir(opt.output_rpc);

    vw::CamPtr cam = session->camera_model(opt.image_file, opt.camera_file);

    // Get the input nodata value from the image file, unless the
    // user overwrites it.
    float val = std::numeric_limits<float>::quiet_NaN();
    bool has_input_nodata = vw::read_nodata_val(opt.image_file, val);
    if (has_input_nodata && boost::math::isnan(opt.input_nodata_value))
      opt.input_nodata_value = val;

    if (!boost::math::isnan(opt.input_nodata_value)) 
      vw_out() << "Using input nodata value: " << opt.input_nodata_value << "\n";
    else
      has_input_nodata = false;

    // If the output nodata value was not specified, use the input one
    if (boost::math::isnan(opt.output_nodata_value)) 
      opt.output_nodata_value = opt.input_nodata_value;

    if (!boost::math::isnan(opt.output_nodata_value)) {
      opt.has_output_nodata = true;
      vw_out() << "Using output nodata value: " << opt.output_nodata_value << "\n";
    }else{
      opt.has_output_nodata = false;
    }

    // Masked input image
    DiskImageView<float> disk_view(opt.image_file);
    ImageViewRef<PixelMask<float>> input_img
      = create_mask(disk_view, opt.input_nodata_value);

    // The bounding box
    BBox2 image_box = bounding_box(disk_view);
    if (!opt.image_crop_box.empty()) 
      image_box.crop(opt.image_crop_box);

    // Calc the lon-lat-height ranges based on the DEM rather than the user input    
    if (!opt.dem_file.empty())
      calc_llh_bbox_from_dem(opt, cam, image_box, input_img);

    // Put this here, after peeking inside the DEM with calc_llh_bbox_from_dem()
    vw_out() << "Datum: " << opt.datum << "\n";

    // Generate point pairs
    std::vector<Vector3> all_llh;
    std::vector<Vector2> all_pixels;
    asp::sample_llh_pix_bbox(opt.lon_lat_range, opt.height_range, opt.num_samples,
                             opt.datum, cam, image_box, 
                             all_llh, all_pixels); // outputs
    // Add points for pixels along the perimeter and diagonals of image_box. Constrain
    // by the ll box.
    asp::add_perimeter_diag_points(image_box, opt.datum, cam, opt.lon_lat_range, 
                                   opt.height_range,
                                   all_llh, all_pixels); // outputs

    // The pixel box
    BBox2 pixel_box;
    for (size_t i = 0; i < all_pixels.size(); i++) 
      pixel_box.grow(all_pixels[i]);

    // Below we assume pixel_box to be max-exclusive, so expand it by 1
    pixel_box.max() += Vector2(1, 1);
    
    // Find the range of lon-lat-heights
    BBox3 llh_box;
    for (size_t i = 0; i < all_llh.size(); i++) 
      llh_box.grow(all_llh[i]);

    // If cropping, adjust the pixels
    BBox2 crop_box;
    if (!opt.no_crop) {
      // Cast to int so that we can crop properly
      pixel_box.min() = floor(pixel_box.min());
      pixel_box.max() = ceil(pixel_box.max());
      pixel_box.crop(image_box);

      crop_box = pixel_box; // save it before we modify pixel_box

      // Shift all pixels by the crop corner, including the pixel box itself
      for (size_t i = 0; i < all_pixels.size(); i++) 
        all_pixels[i] -= pixel_box.min();

      // Need to first save the corner before subtracting it, otherwise get wrong result
      Vector2 shift = pixel_box.min(); 
      pixel_box -= shift;
    }

    if (opt.save_tif) {
      ImageViewRef<PixelMask<float>> output_img = input_img;
      if (!opt.no_crop) 
        output_img = crop(input_img, crop_box);

      std::string out_img_file = fs::path(opt.output_rpc).replace_extension("tif").string();
      vw_out() << "Writing: " << out_img_file << "\n";

      GeoReference img_geo;
      bool has_img_geo = false;
      vw::TerminalProgressCallback tpc("asp", "\t-->: ");
      vw::cartography::block_write_gdal_image(out_img_file,
                                              apply_mask(output_img, opt.output_nodata_value),
                                              has_img_geo, img_geo,
                                              opt.has_output_nodata,
                                              opt.output_nodata_value,
                                              opt, tpc);
    }

    if (opt.skip_computing_rpc) 
      return 0;

    Vector3 llh_scale  = (llh_box.max() - llh_box.min())/2.0; // half range
    Vector3 llh_offset = (llh_box.max() + llh_box.min())/2.0; // center point

    Vector2 pixel_scale  = (pixel_box.max() - pixel_box.min())/2.0; // half range 
    Vector2 pixel_offset = (pixel_box.max() + pixel_box.min())/2.0; // center point

    vw_out() << "Lon-lat-height box for the RPC approx: " << llh_box   << "\n";
    vw_out() << "Camera pixel box for the RPC approx (after crop): " << pixel_box << "\n";

    // Form the vectors of normalized llh and pixel values
    Vector<double> normalized_llh;
    Vector<double> normalized_pixels;
    asp::normalizeLlhPix(all_llh, all_pixels, llh_offset, llh_scale,
                         pixel_offset, pixel_scale,
                         normalized_llh, normalized_pixels); // outputs

    // Find the RPC coefficients
    asp::RPCModel::CoeffVec line_num, line_den, samp_num, samp_den;
    vw_out() << "Generating the RPC approximation using " << all_llh.size()
             << " point pairs.\n";
    bool refine_only = false;
    asp::gen_rpc(// Inputs
                 opt.penalty_weight, normalized_llh, normalized_pixels,
                 refine_only,
                 // Outputs
                 line_num, line_den, samp_num, samp_den);

    // Form and save the model
    vw::vw_out() << "Writing: " << opt.output_rpc << "\n";
    vw::cartography::Datum datum(opt.datum);
    asp::RPCModel rpc(datum, line_num, line_den, samp_num, samp_den,
                      pixel_offset, pixel_scale, llh_offset, llh_scale);
    rpc.saveXML(opt.output_rpc);

  } ASP_STANDARD_CATCHES;

  return 0;
}
