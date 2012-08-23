// __BEGIN_LICENSE__
//  Copyright (c) 2009-2012, United States Government as represented by the
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


/// \file rmax2cahvor.cc
///


/************************************************************************
 *     File: rmaxcahvor.cc
 *     Date: May 2007
 *       By: Todd Templeton
 *      For: NASA Ames Research Center, Intelligent Mechanisms Group
 * Function: Extract CAHVOR camera model from RMAX images
 *
 ************************************************************************/
#include <stdlib.h>
#include <iostream>

#include <boost/algorithm/string.hpp>
#include <boost/program_options.hpp>
namespace po = boost::program_options;

#include <vw/Camera/CAHVORModel.h>
using namespace vw;
using namespace vw::camera;

#include <asp/Sessions/RMAX/RMAX.h>

int main(int argc, char* argv[]) {
  std::string image_file;
  std::string adjustment_filename;

  po::options_description general_options("Options");
  general_options.add_options()
    ("adjustment-file,a", po::value<std::string>(&adjustment_filename), "Apply adjustments from file.\n")
    ("help,h", "Display this help message.")
    ("verbose", "Verbose output");

  po::options_description hidden_options("");
  hidden_options.add_options()
    ("input-file", po::value<std::string>(&image_file));

  po::options_description options("Allowed Options");
  options.add(general_options).add(hidden_options);

  po::positional_options_description p;
  p.add("input-file", -1);

  po::variables_map vm;
  po::store( po::command_line_parser( argc, argv ).options(options).positional(p).run(), vm );
  po::notify( vm );

  std::ostringstream usage;
  usage << "Usage: " << argv[0] << " [options] <rmax image filenames>..." << std::endl << std::endl;
  usage << general_options << std::endl;

  if( vm.count("help") ) {
    std::cout << usage.str() << std::endl;
    return 1;
  } else if ( vm.count("input-file") < 1 ) {
    std::cout << "No input files!\n" << usage.str() << std::endl;
    return 1;
  }


  std::string output;
  if( vm.count("adjustment-file") ) {
    ImageInfo info;
    read_image_info( image_file, info );

    std::ifstream f(adjustment_filename.c_str());
    Vector3 position_adjustment;
    Vector3 pose_adjustment;

    f >> position_adjustment[0] >> position_adjustment[1] >> position_adjustment[2];
    f >> pose_adjustment[0] >> pose_adjustment[1] >> pose_adjustment[2];
    std::cout << position_adjustment << "   " << pose_adjustment<< "\n";
    CAHVORModel cahvor = rmax_image_camera_model(info, position_adjustment, pose_adjustment);
    output = asp::prefix_from_filename(image_file) + ".rmax_adjust.cahvor";
    cahvor.write(output);
  } else {
    CAHVORModel cahvor = rmax_image_camera_model(argv[1]);
    output = prefix_from_filename(image_file) + ".cahvor";
    cahvor.write(output);
  }
}
