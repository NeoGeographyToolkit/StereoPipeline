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

#include <vw/Camera/CAHVORModel.h>
using namespace vw;
using namespace vw::camera;

#include "RMAX/RMAX.h"

// Split filename into base and extension.
int split_filename(const std::string& filename, std::string& base, std::string& extension) {
  std::string::size_type dot = filename.find_last_of('.');
  if(dot == std::string::npos || dot <= 0)
    return -1;
  extension = filename.substr(dot);
  boost::to_lower(extension);
  base = filename.substr(0, dot);
  return 0;
}

int main(int argc, char* argv[]) {
  std::string base, extension, output;
  vw::camera::CAHVORModel cahvor;

  // If the command line wasn't properly formed or the user requested
  // help, we print a usage message.
  if(argc < 2 || split_filename(argv[1], base, extension) != 0 || extension != ".png") {
    std::cout << "USAGE: " << argv[0] << " input.png [output]" << std::endl;
  }

  // Construct output filename.
  if(argc >= 3)
    output = argv[2];
  else
    output = base + ".cahvor";

  // Read CAHVOR camera model from image.
  cahvor = rmax_image_camera_model(argv[1]);

  // Write CAHVOR camera model to file.
  cahvor.write(output);

  return 0;
}
