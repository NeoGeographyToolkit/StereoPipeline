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

/// \file StereoSessionHRSC.cc
///

#include <boost/shared_ptr.hpp>

#include "HRSC/StereoSessionHRSC.h"

#include "HRSC/HRSC.h"
#include "stereo.h"

using namespace vw;
using namespace vw::camera;

/// Erases a file suffix if one exists and returns the base string
static std::string prefix_from_filename(std::string const& filename) {
  std::string result = filename;
  int index = result.rfind(".");
  if (index != -1) 
    result.erase(index, result.size());
  return result;
}

boost::shared_ptr<vw::camera::CameraModel> StereoSessionHRSC::camera_model(std::string image_file, 
                                                                           std::string camera_file) {
  // Build the input prefix path by removing the filename suffix
  std::string in_prefix = prefix_from_filename(image_file);
  
  // Initialize the HRSC metadata object
  HRSCImageMetadata hrsc_metadata(image_file);
  try {
    std::cout << "Loading HRSC Metadata.\n";
    hrsc_metadata.read_line_times(in_prefix + ".txt");
    hrsc_metadata.read_ephemeris_supplement(in_prefix + ".sup");

    // Reading EXTORI files disabled for now... re-enable here if you
    // need to use HRSC bundle adjustment files.
    //     if (m_extra_argument1.size() != 0) 
    //       hrsc_metadata1.read_extori_file(m_extra_argument1,m_extra_argument3);
  } catch (IOErr &e) {
    std::cout << "An error occurred when loading HRSC metadata:\n\t" << e.what();
    std::cout << "\nExiting.\n\n";
    exit(1);
  }

  return boost::shared_ptr<camera::CameraModel>(hrsc_metadata.camera_model());
}

