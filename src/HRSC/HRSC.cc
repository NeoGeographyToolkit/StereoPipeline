#include "HRSC/HRSC.h"
#include "MOC/Metadata.h"
#include <fstream>
#include <vw/Core/Exception.h>

HRSCImageMetadata::HRSCImageMetadata(std::string const& filename) { 
  m_filename = filename; 
}

/// Build a camera model based on the available metadata
vw::camera::OrbitingPushbroomModel HRSCImageMetadata::camera_model() {

  // The HRSC frame of reference is defined as follows:
  //
  // +Z out of the front of the camera (nadir)
  // +Y perpindicular to the imaging lines in the direction of flight
  // +X parallel to the scanlines, but increasing X is decreasing u...
  vw::Vector3 pointing_vec(0,0,1);
  vw::Vector3 u_vec(-1,0,0);

  // Use the values that were obtained from the *.sup file to program
  // the camera model parameters.
  return vw::camera::OrbitingPushbroomModel( rows(), // number of lines
                                             cols(), // sampels per line
                                             int(m_start_sample/m_crosstrack_summing), // sample offset
                                             m_focal_length, 
                                             m_along_scan_pixel_size*m_downtrack_summing, 
                                             m_across_scan_pixel_size*m_crosstrack_summing,
                                             m_line_times,
                                             m_t0_quat, m_dt_quat,
                                             m_t0_ephem, m_dt_ephem,
                                             pointing_vec,
                                             u_vec,
                                             m_quat,   // camera poses
                                             m_ephem); // camera positions
}

/// Read the line times from an HRSC metadata file
void HRSCImageMetadata::read_line_times(std::string const& filename) {
  
  std::ifstream infile(filename.c_str());
  double scanline, sclk_time, integration_time;

  if ( infile.is_open() ) {
    m_line_times.clear();
    while (infile >> scanline >> sclk_time >> integration_time ) {
      m_line_times.push_back(sclk_time);
    }
  } else { 
    throw vw::IOErr() << "hrsc_line_integration_times: could not open file \"" << filename << "\"\n";
  }
  std::cout << filename << ": " << m_line_times.size() << " records.\n";

  double base_line_time = m_line_times[0];
  for (int i = 0; i < m_line_times.size(); ++i) {
    m_line_times[i] -= base_line_time;
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
    double scan_duration = parser.read_double("SCAN_DURATION");

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

