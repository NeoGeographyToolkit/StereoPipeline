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

// C++ implementation of the Nuth and Kaab alginment method from:
// https://github.com/dshean/demcoreg/tree/master

#include <asp/Core/Macros.h>
#include <asp/Core/Common.h>
#include <asp/Core/StereoSettings.h>

#include <vw/Core/Exception.h>
#include <vw/Core/Stopwatch.h>

#include <boost/program_options.hpp>

namespace po = boost::program_options;

struct Options: vw::GdalWriteOptions {
  std::string ref, src, out_prefix, res;
  int poly_order, max_iter;
  double tol, max_offset, max_dz;
  bool tiltcorr;
  vw::Vector2 slope_lim;
  Options() {}
};

void handle_arguments(int argc, char *argv[], Options& opt) {

  po::options_description general_options("General options");
  general_options.add_options()
    ("ref,r", po::value(&opt.ref)->default_value(""),
     "Reference DEM.")
    ("src,s", po::value(&opt.src)->default_value(""),
     "Source DEM to align to the reference.")
    ("output-prefix,o", po::value(&opt.out_prefix)->default_value(""), 
     "Output prefix for writing all produced files.")
    ("poly-order", po::value(&opt.poly_order)->default_value(1), 
     "Specify the order of the polynomial fit.")
    ("tol", po::value(&opt.tol)->default_value(0.1),
      "Stop when iterative translation magnitude is below this tolerance (meters).")
    ("max-offset", po::value(&opt.max_offset)->default_value(100.0),
     "Maximum expected horizontal translation magnitude (meters).")
    ("max-dz", po::value(&opt.max_dz)->default_value(100.0),
     "Maximum expected vertical offset in meters, used to filter outliers.")
    ("tiltcorr", po::bool_switch(&opt.tiltcorr)->default_value(false),
     "After a preliminary translation, fit a polynomial to residual elevation offsets "
     "and remove.")
    ("res", po::value(&opt.res)->default_value("mean"),
     "Regrid the input DEMs to this resolution given the resolutions of input datasets. "
     "Options: min, max, mean, common_scale_factor.")
    ("slope-lim", 
     po::value(&opt.slope_lim)->default_value(vw::Vector2(0.1, 40.0), "0.1, 40.0"),
     "Minimum and maximum surface slope limits to consider (degrees).")
    ("max-iter", po::value(&opt.max_iter)->default_value(30),
     "Maximum number of iterations, if tolerance is not reached.")
    ;
  general_options.add(vw::GdalWriteOptionsDescription(opt));

  po::options_description positional("");
  po::positional_options_description positional_desc;

  std::string usage("-r ref_dem.tif -s src_dem.tif -o output_prefix [options]\n");

  bool allow_unregistered = false;
  std::vector<std::string> unregistered;
  po::variables_map vm =
    asp::check_command_line(argc, argv, opt, general_options, general_options,
                            positional, positional_desc, usage,
                            allow_unregistered, unregistered);

  // On failure print the usage as well, for when the command is called with no
  // arguments.
  if (opt.out_prefix == "")
    vw::vw_throw(vw::ArgumentErr() << "The output prefix was not set.\n"
             << usage << general_options);

  // The ref and src DEMs must be provided.
  if (opt.ref == "" || opt.src == "")
    vw::vw_throw(vw::ArgumentErr() << "The reference and source DEMs must be provided.\n");

  // The poly order must be positive.
  if (opt.poly_order < 1)
    vw::vw_throw(vw::ArgumentErr() << "The polynomial order must be at least 1.\n");
  
  // The tol must be positive.
  if (opt.tol <= 0)
    vw::vw_throw(vw::ArgumentErr() << "The tolerance must be positive.\n");
  
  // Max offset must be positive. Same with max_dz.
  if (opt.max_offset <= 0.0 || opt.max_dz <= 0.0)
    vw::vw_throw(vw::ArgumentErr() 
                 << "The maximum horizontal and vertical offsets must be positive.\n");

  // Check that res is one of the allowed values.
  if (opt.res != "min" && opt.res != "max" && opt.res != "mean" &&
      opt.res != "common_scale_factor")
    vw::vw_throw(vw::ArgumentErr() << "Unknown value for --res: " << opt.res << ".\n");
  
  // Check that slope limits are positive and in the right order.
  if (opt.slope_lim[0] <= 0.0 || opt.slope_lim[1] <= 0.0 || 
      opt.slope_lim[0] >= opt.slope_lim[1])
    vw::vw_throw(vw::ArgumentErr() 
                 << "The slope limits must be positive and in increasing order.\n");
    
  // Create the output directory
  vw::create_out_dir(opt.out_prefix);

  // Turn on logging to file
  asp::log_to_file(argc, argv, "", opt.out_prefix);
}


void run_nuth(Options const& opt) {
  std::cout << "--now in run_nuth\n";
}

int main(int argc, char *argv[]) {

  Options opt;
  try {

    handle_arguments(argc, argv, opt);
    run_nuth(opt);

  } ASP_STANDARD_CATCHES;

  return 0;
}
