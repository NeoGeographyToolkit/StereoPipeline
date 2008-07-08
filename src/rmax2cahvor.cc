// __BEGIN_LICENSE__
// 
// Copyright (C) 2008 United States Government as represented by the
// Administrator of the National Aeronautics and Space Administration
// (NASA).  All Rights Reserved.
// 
// Copyright 2008 Carnegie Mellon University. All rights reserved.
// 
// This software is distributed under the NASA Open Source Agreement
// (NOSA), version 1.3.  The NOSA has been approved by the Open Source
// Initiative.  See the file COPYING at the top of the distribution
// directory tree for the complete NOSA document.
// 
// THE SUBJECT SOFTWARE IS PROVIDED "AS IS" WITHOUT ANY WARRANTY OF ANY
// KIND, EITHER EXPRESSED, IMPLIED, OR STATUTORY, INCLUDING, BUT NOT
// LIMITED TO, ANY WARRANTY THAT THE SUBJECT SOFTWARE WILL CONFORM TO
// SPECIFICATIONS, ANY IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR
// A PARTICULAR PURPOSE, OR FREEDOM FROM INFRINGEMENT, ANY WARRANTY THAT
// THE SUBJECT SOFTWARE WILL BE ERROR FREE, OR ANY WARRANTY THAT
// DOCUMENTATION, IF PROVIDED, WILL CONFORM TO THE SUBJECT SOFTWARE.
//
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

#include "RMAX/RMAX.h"

static std::string prefix_from_filename(std::string const& filename) {
  std::string result = filename;
  int index = result.rfind(".");
  if (index != -1) 
    result.erase(index, result.size());
  return result;
}

int main(int argc, char* argv[]) {
  std::string image_file;
  std::string adjustment_filename;

  po::options_description general_options("Options");
  general_options.add_options()
    ("adjustment-file,a", po::value<std::string>(&adjustment_filename), "Apply adjustments from file.\n")
    ("help", "Display this help message")
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
    std::cout << usage << std::endl;
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
    output = prefix_from_filename(image_file) + ".rmax_adjust.cahvor";
    cahvor.write(output);
  } else {
    CAHVORModel cahvor = rmax_image_camera_model(argv[1]);
    output = prefix_from_filename(image_file) + ".cahvor";
    cahvor.write(output);
  }
}
