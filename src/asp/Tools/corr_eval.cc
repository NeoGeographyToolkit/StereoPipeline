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

// Evaluate the quality of produced correlation using several metrics.
// See CorrEval.h and this tool's manual for more info.

#include <vw/Stereo/PreFilter.h>
#include <vw/Stereo/CorrEval.h>
#include <asp/Core/Macros.h>
#include <asp/Core/Common.h>
#include <asp/Core/StereoSettings.h>

namespace po = boost::program_options;
namespace fs = boost::filesystem;

struct Options : vw::cartography::GdalWriteOptions {
  std::string left_image, right_image, disparity, output_prefix, metric;
  vw::Vector2i kernel_size;
  bool round_to_int;
  int prefilter_mode;
  float prefilter_kernel_width;
};

void handle_arguments(int argc, char *argv[], Options& opt) {
  
  po::options_description general_options("");
  general_options.add_options()
    ("kernel-size", po::value(&opt.kernel_size)->default_value(vw::Vector2i(21, 21),"21 21"),
     "The dimensions of image patches. These must be positive odd numbers.")
    ("metric", po::value(&opt.metric)->default_value("ncc"),
     "The metric to use to evaluate the quality of correlation. Options: ncc, stddev.")
    ("prefilter-mode", po::value(&opt.prefilter_mode)->default_value(0),
     "Prefilter mode. This is the same prefilter as in stereo correlation with "
     "the asp_bm method. Options: 0 (none), 1 (subtracted mean), 2 (LoG).")
    ("prefilter-kernel-width", po::value(&opt.prefilter_kernel_width)->default_value(1.5),
     "The diameter of the Gaussian convolution kernel "
     "for prefilter modes 1 and 2. A value of 1.5 works "
     "well for LoG and 25-30 is suggested for the subtracted mean.")
    ("round-to-int", po::bool_switch(&opt.round_to_int)->default_value(false),
     "Round the disparity to integer and skip interpolation when finding the right image patches. This make the program faster by a factor of about 2, without changing significantly the output image.");

  general_options.add(vw::cartography::GdalWriteOptionsDescription(opt));
  
  po::options_description positional("");
  positional.add_options()
    ("left-image", po::value(&opt.left_image))
    ("right-image", po::value(&opt.right_image))
    ("disparity", po::value(&opt.disparity))
    ("output-prefix", po::value(&opt.output_prefix));

  po::positional_options_description positional_desc;
  positional_desc.add("left-image", 1);
  positional_desc.add("right-image", 1);
  positional_desc.add("disparity", 1);
  positional_desc.add("output-prefix", 1);
  
  std::string usage("[options] <L.tif> <R.tif> <Disp.tif> <output.tif>");
  bool allow_unregistered = false;
  std::vector<std::string> unregistered;
  po::variables_map vm =
    asp::check_command_line(argc, argv, opt, general_options, general_options,
                             positional, positional_desc, usage,
                             allow_unregistered, unregistered);

  if (!vm.count("left-image")  || !vm.count("right-image") ||
      !vm.count("disparity")   || !vm.count("output-prefix"))
    vw::vw_throw(vw::ArgumentErr() << "Not all required arguments were specified.\n\n"
                 << usage << general_options);

  if (opt.metric != "ncc" && opt.metric != "stddev") 
    vw::vw_throw(vw::ArgumentErr() << "Invalid value provided for --metric.\n\n"
                 << usage << general_options);
  
  vw::create_out_dir(opt.output_prefix);
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
      vw::vw_throw(vw::ArgumentErr() << "The left aligned image lacks a nodata value.\n");
    
    float right_nodata = -32768.0;
    if (!vw::read_nodata_val(opt.right_image, right_nodata))
      vw::vw_throw(vw::ArgumentErr() << "The right aligned image lacks a nodata value.\n");

    // Use bigger tiles on output, should be faster that way given
    // that we have to grab a big chunk of the right image for each
    // tile.
    opt.raster_tile_size = Vector2i(asp::ASPGlobalOptions::corr_tile_size(),
                                    asp::ASPGlobalOptions::corr_tile_size());

    ImageViewRef<PixelMask<float>> masked_left  = create_mask(left, left_nodata);
    ImageViewRef<PixelMask<float>> masked_right = create_mask(right, right_nodata);
    
    if (opt.prefilter_mode > 0) {
      masked_left = vw::stereo::prefilter_image(masked_left,
                                                vw::stereo::PrefilterModeType(opt.prefilter_mode),
                                                opt.prefilter_kernel_width);
      masked_right = vw::stereo::prefilter_image(masked_right,
                                                vw::stereo::PrefilterModeType(opt.prefilter_mode),
                                                opt.prefilter_kernel_width);
    }
    
    vw::cartography::GeoReference left_georef;
    bool has_left_georef = read_georeference(left_georef, opt.left_image);
    bool has_nodata      = true;
    std::string output_image = opt.output_prefix + "-" + opt.metric + ".tif";
    vw_out() << "Writing: " << output_image << "\n";
    vw::cartography::block_write_gdal_image
      (output_image,
       apply_mask(vw::stereo::corr_eval(masked_left, masked_right,
                                        disp, opt.kernel_size, opt.metric, opt.round_to_int),
                  left_nodata),
       has_left_georef, left_georef,
       has_nodata, left_nodata, opt,
       TerminalProgressCallback("asp", "\t--> Correlation quality:"));
    
  } ASP_STANDARD_CATCHES;
  
  return 0;
}
