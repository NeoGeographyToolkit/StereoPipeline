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


/// \file corr_eval.cc

// Evaluate the quality of produced correlation with several metrics.

#include <asp/Core/Macros.h>
#include <asp/Core/Common.h>
#include <asp/Core/StereoSettings.h>
//#include <vw/Stereo/NCC.h>

namespace po = boost::program_options;
namespace fs = boost::filesystem;

struct Options : vw::cartography::GdalWriteOptions {
  std::string left_image, right_image, disparity, output_image;
  vw::Vector2i kernel_size;
};

void handle_arguments(int argc, char *argv[], Options& opt) {
  
  po::options_description general_options("");
  general_options.add_options()
    ("kernel-size", po::value(&opt.kernel_size)->default_value(vw::Vector2i(21,21),"21 21"),
     "Kernel size used for normalized cross-correlation.");

  general_options.add(vw::cartography::GdalWriteOptionsDescription(opt));
  
  po::options_description positional("");
  positional.add_options()
    ("left-image", po::value(&opt.left_image))
    ("right-image", po::value(&opt.right_image))
    ("disparity", po::value(&opt.disparity))
    ("output-image", po::value(&opt.output_image));
  
  po::positional_options_description positional_desc;
  positional_desc.add("left-image", 1);
  positional_desc.add("right-image", 1);
  positional_desc.add("disparity", 1);
  positional_desc.add("output-image", 1);
  
  std::string usage("[options] <L.tif> <R.tif> <Disp.tif> <output.tif>");
  bool allow_unregistered = false;
  std::vector<std::string> unregistered;
  po::variables_map vm =
    asp::check_command_line(argc, argv, opt, general_options, general_options,
                             positional, positional_desc, usage,
                             allow_unregistered, unregistered);

  if (!vm.count("left-image")  ||
      !vm.count("right-image") ||
      !vm.count("disparity")   ||
      !vm.count("output-image"))
    vw::vw_throw(vw::ArgumentErr() << "Not all required arguments were specified.\n\n"
                 << usage << general_options);
  
  vw::create_out_dir(opt.output_image);
}

int main(int argc, char *argv[]) {

  using namespace vw;
  
  Options opt;
  try {
    handle_arguments(argc, argv, opt);

    DiskImageView<float> left(opt.left_image);
    DiskImageView<float> right(opt.right_image);
    DiskImageView<PixelMask<vw::Vector2f>> disp(opt.disparity);

    float left_nodata = -32768.0;
    if (!vw::read_nodata_val(opt.left_image, left_nodata))
      vw::vw_throw(vw::ArgumentErr() << "The left image lacks a nodata values.\n");
    
    float right_nodata = -32768.0;
    if (!vw::read_nodata_val(opt.right_image, right_nodata))
      vw::vw_throw(vw::ArgumentErr() << "The right image lacks a nodata values.\n");

    // Use bigger tiles on output, should be faster that way given that
    // we have to grab a big chunk of the right image for each tile anyway.
    opt.raster_tile_size = Vector2i(asp::ASPGlobalOptions::corr_tile_size(),
                                    asp::ASPGlobalOptions::corr_tile_size());

#if 0
    vw::cartography::GeoReference left_georef;
    bool has_left_georef = read_georeference(left_georef,  opt.left_image);
    bool has_nodata      = true;
    vw_out() << "Writing: " << opt.output_image << ".\n";
    vw::cartography::block_write_gdal_image
      (opt.output_image,
       apply_mask(vw::stereo::ncc(create_mask(left, left_nodata),
                                  create_mask(right, right_nodata),
                                  disp, opt.kernel_size), left_nodata),
       has_left_georef, left_georef,
       has_nodata, left_nodata, opt,
       TerminalProgressCallback("asp", "\t--> Correlation quality:"));
#endif
    
  } ASP_STANDARD_CATCHES;
  
  return 0;
}
