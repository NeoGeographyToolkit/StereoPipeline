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


// std header
#include <stdlib.h>
#include <iostream>
#include <vector>

// Boost
#include <boost/program_options.hpp>
#include <boost/filesystem/operations.hpp>
namespace po = boost::program_options;
namespace fs = boost::filesystem;

// Vision Workbench
#include <vw/Math.h>
#include <vw/InterestPoint/InterestData.h>
#include <vw/InterestPoint/Matcher.h>
using namespace vw;

// Stereo Pipeline
#include <asp/ControlNetTK/Equalization.h>
#include <asp/Core/Macros.h>
#include <asp/Core/Common.h>
using namespace asp;

struct Options : public asp::BaseOptions {
  std::vector<std::string> match_files;
  size_t max_points, min_points;
};

void handle_arguments( int argc, char *argv[], Options& opt ) {
  po::options_description general_options("");
  general_options.add_options()
    ("max-pts,m",po::value(&opt.max_points)->default_value(100),"Max points a pair can have. If it execeeds we trim.")
    ("min-pts,n",po::value(&opt.min_points)->default_value(10),"Minimum points a pair is required to have. Delete if fails this.");
  general_options.add( asp::BaseOptionsDescription(opt) );

  po::options_description positional("");
  positional.add_options()
    ("input-files", po::value(&opt.match_files));

  po::positional_options_description positional_desc;
  positional_desc.add("input-files", -1);

  std::string usage("[options] <match-files> ...");
  po::variables_map vm =
    asp::check_command_line( argc, argv, opt, general_options, general_options,
                             positional, positional_desc, usage );

  if ( opt.match_files.empty() )
    vw_throw( ArgumentErr() << "Must specify at least one input file!\n\n"
              << usage << general_options );
}

int main( int argc, char* argv[] ) {

  Options opt;
  try {
    handle_arguments( argc, argv, opt );
    BOOST_FOREACH( std::string const& matchfile, opt.match_files ) {
      vw_out() << "Loading: " << matchfile << "\n";

      std::vector<ip::InterestPoint> ip1, ip2;
      ip::read_binary_match_file( matchfile, ip1, ip2 );
      vw_out() << "\t> Found " << ip1.size() << " matches.\n";

      if ( ip1.size() < opt.min_points ) {
        vw_out() << "Match failed to have enough pairs.\n";
        fs::remove( matchfile );
        continue;
      }

      vw_out() << "Performing equalization\n";
      cnettk::equalization( ip1, ip2, opt.max_points );

      // Finally write back out the reduced match file
      ip::write_binary_match_file( matchfile, ip1, ip2 );
      vw_out() << "Wrote back: " << matchfile << "\n";
    }
  } ASP_STANDARD_CATCHES;

  return 0;
}
