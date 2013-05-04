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


/// \file HRSC.cc
///

#include <asp/Sessions/HRSC/HRSC.h>
#include <asp/Sessions/MOC/Metadata.h>
#include <asp/Sessions/HRSC/ExtoriExtrinsics.h>

#include <fstream>
#include <vw/Core/Exception.h>
#include <vw/Math/EulerAngles.h>

using namespace vw::camera;

HRSCImageMetadata::HRSCImageMetadata(std::string const& filename) {
  m_filename = filename;

  m_line_times.clear();
  m_extori_ephem_times.clear();
}

/// Returns a newly allocated camera model object of the appropriate
/// type.  It is the responsibility of the user to later deallocate
/// this camera model object or to manage it using some sort of smart
/// pointer.
vw::camera::CameraModel* HRSCImageMetadata::camera_model() {

  // The HRSC frame of reference is defined as follows:
  //
  // +Z out of the front of the camera (nadir)
  // +Y perpindicular to the imaging lines in the direction of flight
  // +X parallel to the scanlines, but increasing X is decreasing u...
  vw::Vector3 pointing_vec(0,0,1);
  vw::Vector3 u_vec(-1,0,0);

  if (m_extori_ephem_times.size() > 0) {
    std::cout << "NOTE: Generating HRSC camera model based on EXTORI metadata...\n";

    std::cout << "rows(): " << rows() << "\n";
    std::cout << "t0: " << m_first_line_ephem_time << "\n";
    std::cout << "LT: " << m_line_times[0] << "\n";
    std::cout << "LT: " << m_line_times[m_line_times.size()-1] << "\n";
    std::cout << "LT s: " << m_line_times.size() << "\n";
    std::cout << "LT: " << m_extori_ephem_times[m_extori_ephem_times.size()-1] << "\n";
    std::cout << "QUAT0: " << m_extori_quat[0] << "\n\n";

    // If there is extori information, it takes precedence because it is
    // more accurate.
    return new LinescanModel<ExtoriPositionInterpolation,
      ExtoriPoseInterpolation>(rows(),
                               cols(),
                               int(m_start_sample/m_crosstrack_summing),
                               m_focal_length,
                               m_along_scan_pixel_size*m_downtrack_summing,
                               m_across_scan_pixel_size*m_crosstrack_summing,
                               m_line_times,
                               pointing_vec, u_vec,
                               ExtoriPositionInterpolation(m_extori_ephem_times, m_extori_ephem),
                               ExtoriPoseInterpolation(m_extori_ephem_times, m_extori_quat));
  } else {

    // Use the values that were obtained from the *.sup file to program
    // the camera model parameters.
    return new LinescanModel<Curve3DPositionInterpolation,
      SLERPPoseInterpolation>(rows(),
                              cols(),
                              int(m_start_sample/m_crosstrack_summing),
                              m_focal_length,
                              m_along_scan_pixel_size*m_downtrack_summing,
                              m_across_scan_pixel_size*m_crosstrack_summing,
                              m_line_times,
                              pointing_vec, u_vec,
                              Curve3DPositionInterpolation(m_ephem, m_t0_ephem, m_dt_ephem),
                              SLERPPoseInterpolation(m_quat, m_t0_quat, m_dt_quat));
  }
}

/// Read the line times from an HRSC metadata file
void HRSCImageMetadata::read_line_times(std::string const& filename) {

  std::ifstream infile(filename.c_str());
  double scanline, ephemeris_time, integration_time;

  if ( infile.is_open() ) {
    m_line_times.clear();
    while (infile >> scanline >> ephemeris_time >> integration_time ) {
      m_line_times.push_back(ephemeris_time);
    }
  } else {
    throw vw::IOErr() << "hrsc_line_integration_times: could not open file \"" << filename << "\"\n";
  }
  std::cout << filename << ": " << m_line_times.size() << " records.\n";

  m_first_line_ephem_time = m_line_times[0];
  for (unsigned i = 0; i < m_line_times.size(); ++i) {
    m_line_times[i] -= m_first_line_ephem_time;
  }
  infile.close();
}

// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
//                  HRSC SUP FILE MANIPULATION
// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
//
// Read a HRSC *.sup data file provided by Ross Beyer
// This file is used to initialize a linescan camera model.
void HRSCImageMetadata::read_ephemeris_supplement(std::string const& filename) {

  SupplementaryEphemerisParser parser(filename);

  try {

    // First, gather the basic intrinsic camera prameters
    //
    // Note that these values may be overridden by the values in the
    // description.tab file (though they should be identical values).
    //
    m_crosstrack_summing = parser.read_double("PIXEL_SUMMING");
    m_downtrack_summing = parser.read_double("PIXEL_SUMMING");
    //    double scan_duration = parser.read_double("SCAN_DURATION");

    double downloaded_lines = parser.read_double("DOWNLOADED_LINES");
    m_height_pixels = downloaded_lines/m_downtrack_summing;

    m_focal_length = parser.read_double("FOCAL_LENGTH") / 1000.0;
    m_across_scan_pixel_size = parser.read_double("ACROSS_SCAN_PIXEL_SIZE") / 1.0e6;
    m_along_scan_pixel_size = parser.read_double("ALONG_SCAN_PIXEL_SIZE") / 1.0e6;

    double downloaded_samples = parser.read_double("DOWNLOADED_SAMPLES");
    m_width_pixels = downloaded_samples/m_crosstrack_summing;

    // This is really the only piece of information we need from the ephemeris file.
    // (This does noet appear in the description.tab file)
    m_start_sample = parser.read_double("START_SAMPLE");

    /* Read the full ephemeris information into a Nx3 matrix*/
    double n_ephem = parser.read_double("N_EPHEM");
    m_t0_ephem = parser.read_double("T0_EPHEM");
    m_dt_ephem = parser.read_double("DT_EPHEM");
    m_ephem = parser.read_vector3s("EPHEM", (int)n_ephem, 3);
    m_ephem_rate = parser.read_vector3s("EPHEM_RATE", (int)n_ephem, 3);

    /* Next, read in the time serios of data regarding orientation */
    double n_quat = parser.read_double("NUM_QUAT");
    m_t0_quat = parser.read_double("T0_QUAT");
    m_dt_quat = parser.read_double("DT_QUAT");
    m_quat = parser.read_quaternions("QUATERNIONS", (int)n_quat, 4);

  } catch (EphemerisErr &e) {
    throw vw::IOErr() << "An error occured while parsing the ephemeris file.\n";
  }
}

// This is test code for comprehending the extori angles.  It should be deleted. -mbroxton

//   // This additional rotation takes us from the extori "spacecraft"
//   // frame to the more familiar spice HRSC instrument frame.
//   vw::Matrix<double,3,3> hack;
//   hack(0,0) = 0; hack(0,1) = 1.0; hack(0,2) = 0;
//   hack(1,0) = 1; hack(1,1) = 0.0; hack(1,2) = 0;
//   hack(2,0) = 0; hack(2,1) = 0; hack(2,2) = -1;

//   // Here we incorporate the rotation for individual HRSC scanlines.
//   vw::Matrix<double,3,3> scanline_phi, scanline_omega;
//   if (scanline == "S1") {
//     scanline_phi = rotation_z_axis(0.0205*M_PI/180.0);
//     scanline_omega = rotation_x_axis(18.9414*M_PI/180.0);
//   } else if (scanline == "S2") {
//     scanline_phi = rotation_z_axis(0.0270*M_PI/180.0);
//     scanline_omega = rotation_x_axis(-18.9351*M_PI/180.0);
//   } else {
//     throw vw::ArgumentErr() << "euler_to_quaternion(): unsupported scanline name.";
//   }
//   vw::Matrix<double,3,3> scanline_rotation = transpose(scanline_omega);
//   vw::Matrix<double,3,3> instrumenthead_rotation = transpose(rotation_x_axis(-0.3340*M_PI/180.0) * rotation_y_axis(0.0101*M_PI/180.0));

/// Read the line times from an HRSC metadata file
void HRSCImageMetadata::read_extori_file(std::string const& filename, std::string const& scanline) {

  if (m_line_times.size() == 0)
    throw vw::LogicErr() << "read_extori_file(): cannot read extori file until the line times file has been read.  please read that file first.\n";

  std::ifstream infile(filename.c_str());
  double sclk_time;
  vw::Vector3 position;
  double phi, omega, kappa;

  m_extori_ephem_times.clear();
  m_extori_ephem.clear();
  m_extori_quat.clear();

  if ( infile.is_open() ) {

    // read through the lines in the header
    char dummy[256];
    for (int i =0; i < 7; ++i) {
      infile.getline(dummy, 256);
    }

    // Build up the fixed rotation from "extori frame" to the frame for this HRSC scanline.
    vw::math::Quaternion<double> extori_to_mex_spacecraft = vw::math::euler_to_quaternion(M_PI, M_PI/2, 0, "XZX");
    vw::math::Quaternion<double> hrsc_head_to_hrsc_scanline;
    if (scanline == "S1") {
      hrsc_head_to_hrsc_scanline = vw::math::euler_to_quaternion(0.0205*M_PI/180.0, 18.9414*M_PI/180.0, 0, "ZXZ");
    } else if (scanline == "S2") {
      hrsc_head_to_hrsc_scanline = vw::math::euler_to_quaternion(0.0270*M_PI/180.0, -18.9351*M_PI/180.0, 0, "ZXZ");
    } else if (scanline == "P1") {
      hrsc_head_to_hrsc_scanline = vw::math::euler_to_quaternion(0.0457*M_PI/180.0, 12.7544*M_PI/180.0, 0, "ZXZ");
    } else if (scanline == "P2") {
      hrsc_head_to_hrsc_scanline = vw::math::euler_to_quaternion(0.0343*M_PI/180.0, -12.7481*M_PI/180.0, 0, "ZXZ");
    } else {
      throw vw::IOErr() << "read_extori_file(): unrecognized scanline ID.  You must specify the scanline id's after the extori filenames on the command line.";
    }
    vw::math::Quaternion<double> rotation_correction = hrsc_head_to_hrsc_scanline*extori_to_mex_spacecraft;

    // Read the actual data
    //
    // The extori file contains euler angles phi, omega, kappa, which
    // correspond to rotations to be applied about the y, x, and
    // z-axes respectively.
    //
    // NOTE!!: However, these angles are in units of gon.  There are
    // 400 gon in a circle, so we convert that here.
    while (infile >> sclk_time >> position(0) >> position(1) >> position(2) >> phi >> omega >> kappa ) {
      m_extori_ephem_times.push_back(sclk_time);
      m_extori_ephem.push_back(position);
      m_extori_quat.push_back(rotation_correction*vw::math::euler_to_quaternion(-phi*360/400*M_PI/180.0, -omega*360/400*M_PI/180.0, -kappa*360/400*M_PI/180.0, "YXZ"));
    }
  } else {
    throw vw::IOErr() << "read_extori_file(): could not open file \"" << filename << "\"\n";
  }

  // Some basic error checking
  if (m_extori_ephem.size() == 0) {
    throw vw::IOErr() << "read_extori_file(): there was a problem reading \"" << filename << "\"\n";
  } else {
    std::cout << filename << ": " << m_extori_ephem.size() << " records.\n";
  }

  // Subtract off the time for the first scanline.
  for (unsigned i = 0; i < m_extori_ephem_times.size(); ++i) {
    m_extori_ephem_times[i] -= m_first_line_ephem_time;

    //    std::cout << m_extori_ephem_times[i] << ":  " << m_extori_ephem[i] << "      " << m_extori_quat[i] << "    \n";
  }
  infile.close();
}
