/************************************************************************
 *     File: stereodefault.cc
 *     Date: May 2007
 *       By: Todd Templeton
 *      For: NASA Ames Research Center, Intelligent Mechanisms Group 
 * Function: Canonicalize stereo default file, or create file based on defaults	
 *          				
 ************************************************************************/
#include <stdlib.h>
#include <iostream>

#include "StereoSettings.h"

int main(int argc, char* argv[]) {
  const char* infile = 0;
  const char* outfile = "stereo.default";

  // parse command-line arguments
  switch(argc) {
  case 1:
    break;
  case 2:
    outfile = argv[1];
    break;
  case 3:
    infile = argv[1];
    outfile = argv[2];
    break;
  default:
    fprintf(stderr, "USAGE: %s [infile] outfile\n", argv[0]);
    return 0;
  }

  // Initialize the stereo settings
  stereo_settings();
 
  if(infile) {
    // Read all of the options out of the stereo default file. 
    stereo_settings().read(stereo_default_filename);
  }

  // write stereo default file
  std::cout << "ERROR: MICHAEL NEEDS TO REIMPLEMENT WRITE_DEFAULT_FILE()!!!\n";
  //  write_default_file(&dft, &execute, outfile);

  return 0;
}
