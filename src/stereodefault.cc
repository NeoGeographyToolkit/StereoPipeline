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

#include "file_lib.h"

int main(int argc, char* argv[]) {
  DFT_F dft;       /* parameters read in stereo.default */
  TO_DO execute;   /* whether or not to execute specific parts of the program */
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

  // initialize structures
  init_dft_struct(&dft, &execute);

  if(infile) {
    // Read all of the options out of the stereo default file. 
    read_default_file(&dft, &execute, infile);
  }

  // write stereo default file
  write_default_file(&dft, &execute, outfile);

  return 0;
}
