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

#include <asp/PcAlign/NuthAlignmentParse.h>
#include <asp/Core/AspProgramOptions.h>

#include <vw/Core/Exception.h>
#include <vw/Core/Log.h>
#include <vw/FileIO/DiskImageView.h>

#include <boost/program_options.hpp>
#include <omp.h>

#include <iostream>
#include <vector>
#include <cmath>
#include <thread>
#include <sstream>
#include <limits>

namespace po = boost::program_options;

namespace asp {

// Parse the arguments. Some were set by now directly into opt.
void handleNuthArgs(int argc, char *argv[],  NuthOptions& opt) {

  double nan = std::numeric_limits<double>::quiet_NaN();
  po::options_description general_options("General options");
  general_options.add_options()
    ("slope-lim", 
     po::value(&opt.slope_lim)->default_value(vw::Vector2(0.1, 40.0), "0.1, 40.0"),
     "Minimum and maximum surface slope limits to consider (degrees).")
    ("tol", po::value(&opt.tol)->default_value(0.01),
     "Stop when the addition to the computed translation at given iteration has magnitude "
     "below this tolerance (meters).")
    ("max-horizontal-offset", po::value(&opt.max_horiz_offset)->default_value(nan),
     "Maximum expected horizontal translation magnitude (meters). Used to filter outliers.")
    ("max-vertical-offset", po::value(&opt.max_vert_offset)->default_value(nan),
     "Maximum expected vertical translation (meters). Used to filter outliers.")
    ("num-inner-iter", po::value(&opt.inner_iter)->default_value(10),
      "Maximum number of iterations for the inner loop, when finding the best "
      "fit parameters for the current translation.")
    // The options below are not yet supported
    ("tiltcorr", po::bool_switch(&opt.tiltcorr)->default_value(false),
     "After a preliminary translation, fit a polynomial to residual elevation offsets "
     "and remove.")
    ("res", po::value(&opt.res)->default_value("mean"),
     "Regrid the input DEMs to this resolution given the resolutions of input datasets. "
     " NuthOptions: min, max, mean, common_scale_factor.")
    ("poly-order", po::value(&opt.poly_order)->default_value(1), 
     "Specify the order of the polynomial fit.")
    ;
  general_options.add(vw::GdalWriteOptionsDescription(opt));

  po::options_description positional("");
  po::positional_options_description positional_desc;
  std::string usage; // not used
   
  bool allow_unregistered = false;
  std::vector<std::string> unregistered;
  po::variables_map vm =
    asp::check_command_line(argc, argv, opt, general_options, general_options,
                            positional, positional_desc, usage,
                            allow_unregistered, unregistered);

  if (opt.out_prefix == "")
    vw::vw_throw(vw::ArgumentErr() << "The output prefix was not set.\n");

  // The ref and src DEMs must be provided
  if (opt.ref == "" || opt.src == "")
    vw::vw_throw(vw::ArgumentErr() << "The reference and source DEMs must be provided.\n");

  // The poly order must be positive
  if (opt.poly_order < 1)
    vw::vw_throw(vw::ArgumentErr() << "The polynomial order must be at least 1.\n");
  
  // The tol must be positive
  if (opt.tol <= 0.0)
    vw::vw_throw(vw::ArgumentErr() << "The tolerance must be positive.\n");

  // If horizontal and vertical offset are NaN (so, not set), use max_displacement
  if (std::isnan(opt.max_horiz_offset))
     opt.max_horiz_offset = opt.max_displacement;
  if (std::isnan(opt.max_vert_offset))
      opt.max_vert_offset = opt.max_displacement;
         
  // All these offsets must be positive
  if (opt.max_displacement <= 0.0)
    vw::vw_throw(vw::ArgumentErr() << "The maximum displacement must be positive.\n");
  if (opt.max_horiz_offset <= 0.0 || opt.max_vert_offset <= 0.0)
    vw::vw_throw(vw::ArgumentErr() 
                 << "The maximum horizontal and vertical offsets must be positive.\n");

  // Check that res is one of the allowed values
  if (opt.res != "min" && opt.res != "max" && opt.res != "mean" &&
      opt.res != "common_scale_factor")
    vw::vw_throw(vw::ArgumentErr() << "Unknown value for --res: " << opt.res << ".\n");
  
  // Check that slope limits are positive and in the right order
  if (opt.slope_lim[0] <= 0.0 || opt.slope_lim[1] <= 0.0 || 
      opt.slope_lim[0] >= opt.slope_lim[1])
    vw::vw_throw(vw::ArgumentErr() 
                 << "The slope limits must be positive and in increasing order.\n");

  if (opt.tiltcorr)
    vw::vw_throw(vw::NoImplErr() << "Tilt correction is not implemented yet.\n");
     
  // TODO(oalexan1): Sort out the number of threads. Now it comes from .vwrc if
  // not set.

  // Set the number of threads for OpenMP.  
  int processor_count = std::thread::hardware_concurrency();
  omp_set_dynamic(0);
  omp_set_num_threads(processor_count);
  vw::vw_out() << "Using " << processor_count << " threads with OpenMP.\n";
}

// Given a command-line string, form argc and argv. In addition to pointers,
// will also store the strings in argv_str, to ensure permanence.
void formArgcArgv(std::string const& cmd,
                  int & argc,
                  std::vector<char*> & argv,
                  std::vector<std::string> & argv_str) {

  // Break by space and read into argv
  argv_str.clear();
  argv_str.push_back("nuthAlignment"); // program name
  std::istringstream iss(cmd);
  std::string word;
  while (iss >> word) 
    argv_str.push_back(word);
  
  argc = argv_str.size();  
  argv.resize(argc);
  for (int i = 0; i < argc; i++)
    argv[i] = &argv_str[i][0];
  
  return;
}

} // end namespace asp
