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

/// \file stereodefault.cc
///

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
