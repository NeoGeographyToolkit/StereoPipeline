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

/// \file StereoSessionMOC.cc
///

#include <boost/shared_ptr.hpp>

#include <asp/Sessions/MOC/StereoSessionMOC.h>
#include <asp/Sessions/MOC/Ephemeris.h>
#include <asp/Sessions/MOC/Metadata.h>
#include <asp/Sessions/MOC/MOLA.h>
#include <asp/SpiceIO/SpiceUtilities.h>

using namespace vw;
using namespace vw::camera;


boost::shared_ptr<vw::camera::CameraModel> StereoSessionMOC::camera_model(std::string image_file,
                                                                          std::string camera_file) {
  MOCImageMetadata moc_metadata(image_file);

  // If the spice kernels are available, try to use them directly to
  // read in MOC telemetry.
  try {
    load_moc_kernels();
  } catch (spice::SpiceErr &e) {
    vw_out(0) << "Warning: an error occurred when reading SPICE information.  Falling back to *.sup files\n";
  }

  // Read in the tabulated description entry
  std::string description_tab_filename = "description.tab"; // FIXME: Needs to get this setting from the command line.
  vw_out(0) << "Reading data from " << description_tab_filename << ".\n";
  moc_metadata.read_tabulated_description(description_tab_filename);
  moc_metadata.write_viz_site_frame(m_out_prefix);
  vw_out(0) << image_file << " TAB ET: " << moc_metadata.ephemeris_time() << "\n";

  try {
    vw_out(0) << "Reading MOC telemetry from supplementary ephemeris files.\n";
    moc_metadata.read_ephemeris_supplement(camera_file);
    vw_out(0) << camera_file << " ET: " << moc_metadata.ephemeris_time() << "\n";
  } catch (EphemerisErr &e) {
    vw_out(0) << "Failed to open the supplementary ephemeris file:\n\t";
    vw_out(0) << e.what() << "\n";
    vw_out(0) << "\tWarning: Proceeding without supplementary ephemeris information.\n";
  }

  // If the spice kernels are available, try to use them directly to
  // read in MOC telemetry.
  try {
    vw_out(0) << "Attempting to read MOC telemetry from SPICE kernels... \n";
    moc_metadata.read_spice_data();
    vw_out(0) << image_file << " SPICE ET: " << moc_metadata.ephemeris_time() << "\n";
  } catch (spice::SpiceErr &e) {
    vw_out(0) << "Warning: an error occurred when reading SPICE information.  Falling back to *.sup files\n";
  }

  return boost::shared_ptr<camera::CameraModel>(moc_metadata.camera_model());
}

