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

struct Options : vw::GdalWriteOptions {
  std::string left_image, right_image, disparity, output_prefix, metric;
  vw::Vector2i kernel_size;
  bool round_to_int;
  int prefilter_mode, sample_rate;
  float prefilter_kernel_width;
};

// Create the selected prefilter class and apply the filter to an image.
// Return an ImageRef, unlike prefilter_image() in VW, which returns
// an in-memory image, which can be huge.
namespace vw {
namespace stereo {
template <class ImageT>
ImageViewRef<typename ImageT::pixel_type> 
prefilter_image_ref(ImageViewBase<ImageT> const& image,
                    PrefilterModeType prefilter_mode,
                    float             prefilter_width) {
  
  if (prefilter_mode == PREFILTER_LOG){  // LOG
    stereo::LaplacianOfGaussian prefilter(prefilter_width);
    return prefilter.filter(image);
  }
  if (prefilter_mode == PREFILTER_MEANSUB){  // Subtracted mean
    stereo::SubtractedMean prefilter(prefilter_width);
    return prefilter.filter(image);
  }
  //Default: PREFILTER_NONE
  stereo::NullOperation prefilter;
  return prefilter.filter(image);
}
}}

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
    ("sample-rate", po::value(&opt.sample_rate)->default_value(1),
     "Compute the quality image only at one out of this many rows and columns, for speed. "
     "The output image size does not change. To shrink it, (say by 2x), run "
     "gdal_translate -r average -outsize 50% 50% in.tif out.tif.")
    ("round-to-int", po::bool_switch(&opt.round_to_int)->default_value(false),
     "Round the disparity to integer and skip interpolation when finding the right image patches. This make the program faster by a factor of about 2, without changing significantly the output image.");

  general_options.add(vw::GdalWriteOptionsDescription(opt));
  
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

  if (opt.sample_rate < 1)
    vw::vw_throw(vw::ArgumentErr() << "The value of --sample-rate must be positive.\n"
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
    vw::read_nodata_val(opt.left_image, left_nodata); // this may not succeed
    
    float right_nodata = -32768.0;
    vw::read_nodata_val(opt.right_image, right_nodata); // this may not succeed

    vw_out() << "Left and right image no-data values: " << left_nodata << ' '
             << right_nodata << "\n";
    
    // Use bigger tiles on output, should be faster that way given
    // that we have to grab a big chunk of the right image for each
    // tile.
    //opt.raster_tile_size = Vector2i(asp::ASPGlobalOptions::corr_tile_size(), // 1024
    //                                asp::ASPGlobalOptions::corr_tile_size());
    // These should result in less memory being used
    opt.raster_tile_size = Vector2i(asp::ASPGlobalOptions::rfne_tile_size(), // 256
                                    asp::ASPGlobalOptions::rfne_tile_size());

    ImageViewRef<PixelMask<float>> masked_left  = create_mask(left, left_nodata);
    ImageViewRef<PixelMask<float>> masked_right = create_mask(right, right_nodata);

    int m = opt.prefilter_mode; // to write less below
    if (m > 0) {
      masked_left = vw::stereo::prefilter_image_ref(masked_left,
                                                vw::stereo::PrefilterModeType(m),
                                                opt.prefilter_kernel_width);
      masked_right = vw::stereo::prefilter_image_ref(masked_right,
                                                vw::stereo::PrefilterModeType(m),
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
                                        disp, opt.kernel_size, opt.metric,
                                        opt.sample_rate, opt.round_to_int),
                  left_nodata),
       has_left_georef, left_georef,
       has_nodata, left_nodata, opt,
       TerminalProgressCallback("asp", "\t--> Correlation quality:"));
    
  } ASP_STANDARD_CATCHES;
  
  return 0;
}
