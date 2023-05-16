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

// Tool to create simulated satellite images and/or pinhole cameras for them.
// See the manual for details.

#include <asp/Core/Macros.h>
#include <asp/Core/Common.h>
#include <asp/Core/StereoSettings.h>
#include <asp/Sessions/StereoSession.h>

using namespace vw::cartography;
namespace po = boost::program_options;
namespace fs = boost::filesystem;

struct Options : vw::GdalWriteOptions {
  std::string dem_file, ortho_file, out_prefix;
  vw::Vector3 first, last; // dem pixel and height above dem datum
  int num_cameras;
  Options() {}
};

void handle_arguments(int argc, char *argv[], Options& opt) {

  po::options_description general_options("General options");
  general_options.add_options()
    ("dem", po::value(&opt.dem_file)->default_value(""), "Input DEM file.")
    ("ortho", po::value(&opt.ortho_file)->default_value(""), "Input ortho image file.")
    ("output-prefix,o", po::value(&opt.out_prefix), "Specify the output prefix. All the "
    "files that are saved will start with this prefix.")
    ("first", po::value(&opt.first)->default_value(vw::Vector3(), ""),
    "First camera position, specified as DEM pixel column and row, and height above "
    "the DEM datum.")
    ("last", po::value(&opt.last)->default_value(vw::Vector3(), ""),
    "Last camera position, specified as DEM pixel column and row, and height above "
    "the DEM datum.")
    ("num", po::value(&opt.num_cameras)->default_value(0),
    "Number of cameras to generate, including the first and last ones. Must be positive. "
    "The cameras are uniformly distributed along the straight edge from first to last (in "
    "projected coordinates).")
    ;
  general_options.add(vw::GdalWriteOptionsDescription(opt));

  po::options_description positional("");
  po::positional_options_description positional_desc;

  std::string usage("--dem <dem file> --ortho <ortho image file> "
                    "[other options]");

  bool allow_unregistered = false;
  std::vector<std::string> unregistered;
  po::variables_map vm =
    asp::check_command_line(argc, argv, opt, general_options, general_options,
                            positional, positional_desc, usage,
                            allow_unregistered, unregistered);
  if (opt.dem_file == "" || opt.ortho_file == "")
    vw::vw_throw(vw::ArgumentErr() << "Missing input DEM and/or ortho image.\n");
  if (opt.out_prefix == "")
    vw::vw_throw(vw::ArgumentErr() << "Missing output prefix.\n");

  if (opt.first == vw::Vector3() || opt.last == vw::Vector3())
    vw::vw_throw(vw::ArgumentErr() << "The first and last camera positions must be "
      "specified.\n");

  if (opt.num_cameras < 1)
    vw::vw_throw(vw::ArgumentErr() << "The number of cameras must be positive.\n");

  // Create the output directory based on the output prefix
  vw::create_out_dir(opt.out_prefix);

  return;
}

// A function that will read a geo-referenced image, its nodata value,
// and the georeference, and will return a PixelMasked image, the nodata
// value, and the georeference.
// TODO(oalexan1): May need to move this to a more general place.
void readGeorefImage(std::string const& image_file, 
  float & nodata_val, vw::cartography::GeoReference & georef,
  vw::ImageViewRef<vw::PixelMask<float>> & masked_image) {

  // Initial value, in case the image has no nodata field
  nodata_val = std::numeric_limits<float>::quiet_NaN();
  if (!vw::read_nodata_val(image_file, nodata_val))
        vw::vw_out() << "Warning: Could not read the nodata value for: "
                      << image_file << "\nUsing: " << nodata_val << ".\n";

    // Read the image
    vw::vw_out() << "Reading: " << image_file << std::endl;
    vw::DiskImageView<float> image(image_file);
    // Create the masked image
    masked_image = vw::create_mask(image, nodata_val);

    // Read the georeference, and throw an exception if it is missing
    bool has_georef = vw::cartography::read_georeference(georef, image_file);
    if (!has_georef)
      vw::vw_throw(vw::ArgumentErr() << "Missing georeference in: "
                                     << image_file << ".\n");
}

int main(int argc, char *argv[]) {

  Options opt;
  try {
    handle_arguments(argc, argv, opt);

    // Read the DEM
    vw::ImageViewRef<vw::PixelMask<float>> dem;
    float dem_nodata_val = -std::numeric_limits<float>::max(); // will change
    vw::cartography::GeoReference dem_georef;
    readGeorefImage(opt.dem_file, dem_nodata_val, dem_georef, dem);
    std::cout << "georef is " << dem_georef << std::endl;

    // Read the ortho image
    vw::ImageViewRef<vw::PixelMask<float>> ortho;
    float ortho_nodata_val = -std::numeric_limits<float>::max(); // will change
    vw::cartography::GeoReference ortho_georef;
    readGeorefImage(opt.ortho_file, ortho_nodata_val, ortho_georef, ortho);
    std::cout << "georef is " << ortho_georef << std::endl;

    // Print first and last camera positions
    std::cout.precision(17);
    std::cout << "First camera position: " << opt.first << std::endl;
    std::cout << "Last camera position:  " << opt.last << std::endl;

    // Convert the first and last positions to projected coordinates
    vw::Vector2 first_proj = dem_georef.pixel_to_point
        (vw::math::subvector(opt.first, 0, 2));
    vw::Vector2 last_proj  = dem_georef.pixel_to_point
        (vw::math::subvector(opt.last,  0, 2));
    std::cout << "First camera position in projected coordinates: " << first_proj << std::endl;
    std::cout << "Last camera position in projected coordinates:  " << last_proj << std::endl;

    std::cout << "Number of cameras: " << opt.num_cameras << std::endl;

  } ASP_STANDARD_CATCHES;

  return 0;
}
