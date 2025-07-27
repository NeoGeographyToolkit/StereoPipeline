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

/// \file otsu_thresh.cc
///
/// Tool for finding the Otsu image threshold
/// https://en.wikipedia.org/wiki/Otsu%27s_method

#include <limits>

#include <vw/FileIO/DiskImageUtils.h>
#include <vw/Image/Statistics.h>

#include <asp/Core/AspProgramOptions.h>
#include <asp/Core/Macros.h>

namespace po = boost::program_options;

using namespace vw;

struct Options: vw::GdalWriteOptions {
  std::vector<std::string> image_files;
  bool has_nodata_value;
  std::int64_t num_samples, num_bins;
  double nodata_value;
  Options(): has_nodata_value(false), num_samples(-1),
             nodata_value(std::numeric_limits<double>::quiet_NaN()){}
};

void handle_arguments(int argc, char *argv[], Options& opt) {
  po::options_description general_options("");
  // Add the reverse option
  general_options.add(vw::GdalWriteOptionsDescription(opt));
  general_options.add_options()
    ("num-samples", po::value(&opt.num_samples)->default_value(-1),
     "The number of samples to pick from the image. If not specified, hence set to -1, "
     "the full image will be used.")
    ("num-bins", po::value(&opt.num_bins)->default_value(256),
     "Number of bins to use for the histogram. A larger value is "
     "suggested if the image has outlying pixel values.")
    ("nodata-value", po::value(&opt.nodata_value),
     "Use this nodata value instead of what is read from the file, if present.");
  
  po::options_description positional("");
  positional.add_options()
    ("image-files", po::value(&opt.image_files));
  
  po::positional_options_description positional_desc;
  positional_desc.add("image-files", -1);

  std::string usage("[options] <images>");
  bool allow_unregistered = false;
  std::vector<std::string> unregistered;
  po::variables_map vm =
    asp::check_command_line(argc, argv, opt, general_options, general_options,
                            positional, positional_desc, usage,
                            allow_unregistered, unregistered);
  
  opt.has_nodata_value  = vm.count("nodata-value");

  if (opt.image_files.empty())
    vw_throw(ArgumentErr() << "No input images were specified.\n" << usage << general_options);

  if (opt.num_bins <= 100) 
    vw_throw(ArgumentErr() << "At least 100 bins should be used, and ideally more.\n"
             << usage << general_options);
}

int main( int argc, char *argv[] ) {

  Options opt;
  
  try {

    // Find command line options
    handle_arguments(argc, argv, opt);

    for (size_t it = 0; it < opt.image_files.size(); it++) {
      std::string image_file = opt.image_files[it];

      std::cout << "Reading image: " << image_file << std::endl;
      DiskImageView<float> image(image_file);
      
      boost::shared_ptr<vw::DiskImageResource> rsrc(vw::DiskImageResourcePtr(image_file));
      
      // Check for nodata value in the file
      if (opt.has_nodata_value) {
        std::cout << "Using as nodata value: " << opt.nodata_value << ".\n";
      }else if (rsrc->has_nodata_read()) {
        opt.nodata_value = rsrc->nodata_read();
        opt.has_nodata_value = true;
        std::cout << "Extracted nodata value from file: " << opt.nodata_value << ".\n";
      } else {
        std::cout << "No nodata value present in the file." << std::endl;
      }

      // Use a 64 bit int to not run into an overflow
      std::int64_t num_rows = image.rows(), num_cols = image.cols();
      std::int64_t num_vals = num_rows * num_cols;

      if (num_vals <= 0) 
        vw_throw(ArgumentErr() << "Found empty image: " << image_file << "\n");
      
      double samp_ratio = 1.0;
      if (opt.num_samples > 0) 
        samp_ratio = sqrt(double(num_vals) / double(opt.num_samples));
      samp_ratio = std::max(samp_ratio, 1.0);

      std::int64_t num_sample_rows = round(num_rows / samp_ratio);
      num_sample_rows = std::max(std::int64_t(1), num_sample_rows);
      num_sample_rows = std::min(num_rows, num_sample_rows);
      
      std::int64_t num_sample_cols = round(num_cols / samp_ratio);
      num_sample_cols = std::max(std::int64_t(1), num_sample_cols);
      num_sample_cols = std::min(num_cols, num_sample_cols);
      
      std::cout << "Number of image rows and columns: "
                << num_rows << ", " << num_cols << "\n";
      std::cout << "Picking a uniform sample of dimensions "
                << num_sample_rows << ", " << num_sample_cols << "\n";
      std::cout << "Number of bins in the histogram: " << opt.num_bins << std::endl;
      std::cout << "It may take several minutes to find the answer.\n";

      // The mask creation can handle a NaN for the opt.nodata_value.
      double threshold = vw::otsu_threshold(create_mask(image, opt.nodata_value),
                                               num_sample_rows, num_sample_cols,
                                               opt.num_bins);


      // A different implementation, which gives the same result
      // if above the full image is sampled and 256 bins are used.
      //double threshold2 = vw::otsu_threshold(create_mask(image, opt.nodata_value));
      
      vw_out() << std::setprecision(16)
        << "Otsu threshold for image " << image_file << ": " << threshold << "\n";
    }
    
  } ASP_STANDARD_CATCHES;
  return 0;
}
