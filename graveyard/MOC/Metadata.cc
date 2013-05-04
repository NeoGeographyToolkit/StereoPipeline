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


/// \file Metadata.cc
///

/* ------------------------------------------------------------------
 * MOCMetadata.cc
 *
 * General functions for reading MOC ephemeris information provided by
 * Malin Space Science System.  This file contains funcitons for reading
 * the following two types of data file:
 *
 * 1. description.tab files that contain tabulated MOC ephemeris information.
 *    This information is essentially what would be available in the PDS header.
 *
 * 2. SUP files that contain detailed information about camera position, velocity,
 *    pose, focal length, pixel summing, and image size for a given MOC image.
 *    There is also a function for generating a VRML model that depicts the
 *    location of the MGS orbiting Mars at the position and pose corresponding
 *    to two MOC-NA image shots.
 */
#include <asp/Sessions/MOC/Metadata.h>
#include <asp/Sessions/MOC/Ephemeris.h>
#include <asp/SpiceIO/SpiceUtilities.h>
#include <asp/SpiceIO/TabulatedDataReader.h>

// VisionWorkbench
#include <vw/Core/Log.h>
#include <vw/Camera/OrbitingPushbroomModel.h>

// Boost
#include <boost/algorithm/string.hpp>

// STL
#include <vector>
#include <string>
#include <iostream>                        // for WriteVizSiteFrame
#include <fstream>                         // for WriteVizSiteFrame

using namespace std;
using namespace vw;
// Circumference of Mars in meters:
static const double kMarsCircumference = 21344.0e6; // km
static const double kMarsRadius = 3397.0e3;             // km, equatorial (polar is 3395)


/* ----------------------------------------------------------------------
 *               MOCImageMetadata Class Methods
 * ----------------------------------------------------------------------*/

// Parse the MOC image number from the filename
MOCImageMetadata::MOCImageMetadata(std::string const& filename) {
  m_filename = filename;
  m_focal_length = 3.437;
  m_along_scan_pixel_size = 13e-6;
  m_across_scan_pixel_size = 13e-6;
  m_start_sample = -9999;

  // Change to upper case
  std::string name = boost::to_upper_copy(filename);

  // Remove any file suffix
  std::vector<string> suffix_split;
  boost::split( suffix_split, name, boost::is_any_of(".") );

  // Erase all seperators
  std::vector<string> vec;
  boost::split( vec, suffix_split[0], boost::is_any_of("-/") );
  if (vec.size() == 2) {
    string first = vec[0];
    string last = vec[1];

    // Fix discrepancy between FHGA and FHA
    if (first == "FHGA")
      first = "FHA";

    // Insert extra 0 if needed
    if (first.length() < 3)
      first.insert(1,"0");

    while (last.length() < 5)
      last.insert(0,"0");

    m_moc_identifier = (first + "/" + last);
  } else if (vec.size() == 1 && vec[0].size() > 5){
    int len = vec[0].size();
    m_moc_identifier = vec[0].substr(0,len-5) + "/" + vec[0].substr(len-5, len-1);
  } else {
    // In this case we assume that the
    std::cout << "WARNING: failed to parse the filename \"" << filename << "\"."
              << "Data lookup in \n";
    m_moc_identifier = filename;
  }
}


/* - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
 *             GENERATE CAMERA MODEL FROM FREE PARAMETERS
 * - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - */
vw::camera::CameraModel* MOCImageMetadata::camera_model() {

  int start_sample;
  if (m_start_sample == -9999) {
    std::cout << "WARNING: SETTING START_SAMPLE TO -1024.\n";
    start_sample = int(-1024/m_crosstrack_summing);
  } else {
    start_sample = int(m_start_sample/m_crosstrack_summing);
  }

  // Use the values that were obtained from the *.sup file to program
  // the camera model parameters.
  return new vw::camera::OrbitingPushbroomModel( rows(), // number of lines
                                                 cols(), // sampels per line
                                                 start_sample,
                                                 m_focal_length,
                                                 m_along_scan_pixel_size*m_downtrack_summing,
                                                 m_across_scan_pixel_size*m_crosstrack_summing,
                                                 scan_duration() / rows(), // line integration time
                                                 m_t0_quat, m_dt_quat,
                                                 m_t0_ephem, m_dt_ephem,
                                                 Vector3(0,0,1),  // pointing_vect
                                                 Vector3(0,1,0),  // u_vec
                                                 m_quat,   // camera poses
                                                 m_ephem); // camera positions
}

// Load satellite telemetry directly using SPICE.
void MOCImageMetadata::read_spice_data() {
  m_t0_ephem = -1.25;   m_dt_ephem = 0.25;
  m_t0_quat = -1.25;    m_dt_quat = 0.25;

  // For debugging:
  //   cout << "Scan duration " << scan_duration() << "\n";
  //   cout << "Range [" << ephemeris_time + m_t0_ephem << ", " << ephemeris_time + scan_duration() - m_t0_ephem << ", " << m_dt_ephem << "] \n";

  MOC_state(ephemeris_time() + m_t0_ephem,
            ephemeris_time() + scan_duration() - m_t0_ephem,
            m_dt_ephem, m_ephem, m_ephem_rate, m_quat);
}

/* - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
 *              MOC DESCRIPTION.TAB FILE MANIPULATION
 *                    AND VIZ SITE FRAME I/O
 * - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - */

// --------------------------------------------------------
//                Ephemeris entry data
//            (one line of description.tab
// --------------------------------------------------------
/* Byte indices for fields in the MOC description.tab file */
enum { EPHEM_IMAGE_WIDTH = 6,           // pixels
       EPHEM_IMAGE_HEIGHT = 7,          // pixels
       EPHEM_CROSSTRACK_SUMMING = 8,    // unitless
       EPHEM_DOWNTRACK_SUMMING = 9,     // unitless
       EPHEM_ASPECT_RATIO = 11,         // unitless
       EPHEM_INCIDENCE_ANGLE = 12,      // degrees
       EPHEM_LONGITUDE = 15,            // degrees
       EPHEM_LATITUDE = 16,             // degrees
       EPHEM_SCLK_TIME = 27,            // system clock ticks
       EPHEM_ACTUAL_WIDTH = 34,         // kilometers
       EPHEM_ACTUAL_HEIGHT = 35,        // kilometers
       EPHEM_IMAGE_FLIPPED = 39,        // 'Y' or 'N'
       EPHEM_NORTH_ANGLE = 40,          // degrees
       EPHEM_SKEW_ANGLE = 49,           // degrees
       EPHEM_LINE_INTEGRATION_TIME = 29,// seconds
       EPHEM_ORBIT_NUMBER = 52          // orbits
};


void MOCImageMetadata::read_tabulated_description(std::string const& filename) {
  // Read in the information found in the description.tab file.
  try {
    TabulatedDataReader ephemeris_reader(filename, ",");
    vector<string> imagetab;
    cout << "\tFinding entries for " << m_moc_identifier << ".\n";
    if (!ephemeris_reader.find_line_with_text(m_moc_identifier, imagetab)) {
      cout << "Error: could not find an entry that matched " << m_moc_identifier << " in the description file " << filename << "\nExiting.\n\n";
      exit(0);
    }
    parse_ephemeris_entry(imagetab);
    ephemeris_reader.close();
  } catch (IOErr &e) {
    cout << "Failed to open the description file: " << filename << "\n\t";
    cout << e.what() << "\n";
    exit(0);
  }
}

void MOCImageMetadata::parse_ephemeris_entry(std::vector<std::string> ephemerisData)
{
  sscanf(ephemerisData[EPHEM_LONGITUDE].c_str(), "%lf", &m_longitude);          // Image center longitude
  sscanf(ephemerisData[EPHEM_LATITUDE].c_str(), "%lf", &m_latitude);              // Image center latitude
  sscanf(ephemerisData[EPHEM_IMAGE_WIDTH].c_str(), "%lf", &m_width_pixels);     // Image width in pixels
  sscanf(ephemerisData[EPHEM_IMAGE_HEIGHT].c_str(), "%lf", &m_height_pixels);   // Image height in pixels
  sscanf(ephemerisData[EPHEM_CROSSTRACK_SUMMING].c_str(), "%lf", &m_crosstrack_summing);
  sscanf(ephemerisData[EPHEM_DOWNTRACK_SUMMING].c_str(), "%lf", &m_downtrack_summing);
  sscanf(ephemerisData[EPHEM_ASPECT_RATIO].c_str(), "%lf", &m_aspect);
  sscanf(ephemerisData[EPHEM_ACTUAL_WIDTH].c_str(), "%lf", &m_width_meters); // Image width in kilometers
  sscanf(ephemerisData[EPHEM_ACTUAL_HEIGHT].c_str(), "%lf", &m_height_meters); // Image height in kilometers
  sscanf(ephemerisData[EPHEM_INCIDENCE_ANGLE].c_str(), "%lf", &m_angle);           // Incidence angle in degrees
  sscanf(ephemerisData[EPHEM_NORTH_ANGLE].c_str(), "%lf", &m_north_angle);
  sscanf(ephemerisData[EPHEM_SKEW_ANGLE].c_str(), "%lf", &m_skew_angle);
  sscanf(ephemerisData[EPHEM_LINE_INTEGRATION_TIME].c_str(), "%lf", &m_line_integration_time);  // Line integration time
  sscanf(ephemerisData[EPHEM_ORBIT_NUMBER].c_str(), "%d", &m_orbit_number);

  m_sclk_time = ephemerisData[EPHEM_SCLK_TIME];
  m_sclk_time.erase(0, 1);
  m_sclk_time.erase(m_sclk_time.size() - 1, 1);
  m_flip = (*(ephemerisData[EPHEM_IMAGE_FLIPPED].c_str()) == 'F');

  m_worldToLocal = GenWorldToLocalTransform(m_latitude, m_longitude, kMarsRadius);
  m_aspect = 1.0 / m_aspect;
  m_width_meters = m_width_meters * 1000.0;        // Image width in meters
  m_height_meters = m_height_meters * 1000.0;      // Image height in meters
  m_angle = m_angle * M_PI / 180;          // Incidence angle in radians
  m_north_angle = m_north_angle * M_PI / 180;
  m_skew_angle = m_skew_angle * M_PI / 180;
  m_line_integration_time /= 1000.0;              // Convert from ms to secs
}


// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
//                  MOC SUP FILE MANIPULATION
// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
//
// Read a MOC *.sup data file provided by Malin Space Science Systems
// This file is used to initialize a pushbroom camera model.
void MOCImageMetadata::read_ephemeris_supplement(std::string const& filename) {

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

    double focal_length = parser.read_double("FOCAL_LENGTH") / 1000.0;
    if (focal_length != m_focal_length) { std::cout << "WARNING: File focal length differs from standard MOC focal length ( " << focal_length << "\n"; }

    double across_scan_pixel_size = parser.read_double("ACROSS_SCAN_PIXEL_SIZE") / 1.0e6;
    double along_scan_pixel_size = parser.read_double("ALONG_SCAN_PIXEL_SIZE") / 1.0e6;
    if (across_scan_pixel_size != m_across_scan_pixel_size) { std::cout << "WARNING: File across scan pixel size differs from standard MOC focal length.\n"; }
    if (along_scan_pixel_size != m_along_scan_pixel_size) { std::cout << "WARNING: File along scan pixel size differs from standard MOC focal length.\n"; }

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

    /* For IRG generated SUP files, the quaternion already takes into account the minute
     * rotation between the MOC frame of reference and the MGS S/C frame of reference.
     *
     * However, MSSS quaternions do not (we think).
     *
     * The rotation matrix below could be used to correct the quaternions from MGS frame
     * to MOC-NA frame.
     *
     * This matrix is built from a set of Euler angles that were obtained from Malin
     * Space Science Systems.  Refer to docs/MGS_to_MOC_alignment.txt for a full explanation
     * of how this matrix is built.
     */
    //   Matrix<double> R_moc(3,3);
    //   R_moc(0,0) = 1.00000;   R_moc(0,1) = -0.0031;  R_moc(0,2) = 0.0002;
    //   R_moc(1,0) = 0.0031;    R_moc(1,1) = 1.0000;   R_moc(1,2) = 0.0012;
    //   R_moc(2,0) = -0.0002;   R_moc(2,1) = -0.0012;  R_moc(2,2) = 1.0000;

  } catch (EphemerisErr &e) {
    throw IOErr() << "An error occured while parsing the ephemeris file.\n";
  }
}

/* ----------------------------------------------------------------------
 *               VIZ Site Frame Related Methods
 * ----------------------------------------------------------------------*/
vw::Matrix<double> MOCImageMetadata::GenWorldToLocalTransform(double lat, double lon, double radius)
{
  double sinLat = sin(M_PI * lat / 180.0), cosLat = cos(M_PI * lat / 180.0);
  double xOrigin, yOrigin, zOrigin;
  vw::Vector3 localXAxis, localYAxis, localZAxis;

  // Longitude is defined positive west!
  lon = (360.0 - lon);
  // The following assumes latitude is measured from the equatorial
  // plane with north positive. This is different than normal
  // spherical coordinate conversion where the equivalent angle is
  // measured from the positive z axis.
  xOrigin = radius * cosLat * cos(M_PI * lon / 180.0);
  yOrigin = radius * cosLat * sin(M_PI * lon / 180.0);
  zOrigin = radius * sinLat;

  // Debugging:
  //   cout << "\tImage center: ("
  //        << xOrigin << "," << yOrigin << "," << zOrigin << ")" << endl;

  vw::Matrix<double> translation(4,4);
  translation.set_identity();
  translation[0][2] = xOrigin;
  translation[1][2] = yOrigin;
  translation[2][2] = zOrigin;

  vw::Matrix<double> rotation(4,4);
  rotation.set_identity();

  // Mars local level is a Z-down coordinate system
  localZAxis[0] = -xOrigin / radius;
  localZAxis[1] = -yOrigin / radius;
  localZAxis[2] = -zOrigin / radius;

  if (cosLat != 0.0)
  {
    // Z_local X Z_world  = Y_local
    localYAxis[0] =  localZAxis[1];
    localYAxis[1] = -localZAxis[0];
    localYAxis[2] =  0.0;
    localYAxis /= vw::math::norm_2(localYAxis); // Normalize

    // Y_local X Z_local = X_local
    localXAxis = vw::math::cross_prod(localYAxis, localZAxis);
    localXAxis /= vw::math::norm_2(localXAxis); // Normalize
  }
  else                                     // we're at one of the poles
  {
    if (zOrigin > 0.0)                     // north pole
      localXAxis[0] = -1.0;
    else                                   // south pole
      localXAxis[0] = 1.0;
    localXAxis[1] = 0.0; localXAxis[2] = 0.0;
    localYAxis[0] = 0.0; localYAxis[1] = 1.0; localYAxis[2] = 0.0;
  }

  // The 3x3 matrix consisting of the local level coordinate system
  // axes (in world coordinates) as rows gives the local-to-world
  // rotation:
  rotation[0][0] = localXAxis[0];
  rotation[0][1] = localXAxis[1];
  rotation[0][2] = localXAxis[2];

  rotation[1][0] = localYAxis[0];
  rotation[1][1] = localYAxis[1];
  rotation[1][2] = localYAxis[2];

  rotation[2][0] = localZAxis[0];
  rotation[2][1] = localZAxis[1];
  rotation[2][2] = localZAxis[2];

  // We do the translation followed by rotation:
  vw::Matrix<double> localToWorld = rotation * translation;
  return vw::math::inverse(localToWorld);
}

void MOCImageMetadata::write_viz_site_frame(std::string prefix)
{
  string fileName = prefix + string(".viz");
  std::ofstream vizFile(fileName.c_str());

  vw_out(0) << "MOCImage::WriteVizSiteFrame(): writing viz file..."
            << std::flush;

  vizFile << "#Inventor V2.1 ascii" << std::endl << std::endl;

  vizFile << "DEF +parent1 SoVizGroup" << std::endl;
  vizFile << "{" << std::endl;
  vizFile << "    fields [ SFEnum renderCaching, SFEnum boundingBoxCaching,"
          << std::endl;
  vizFile << "             SFEnum renderCulling, SFEnum pickCulling,"
          << std::endl;
  vizFile << "       SFString title, SFNode parent, SFNode transform,"
    " SFNode render" << std::endl;
  vizFile << "       ]" << std::endl;
  vizFile << "    renderCaching OFF" << std::endl;
  vizFile << "    boundingBoxCaching    OFF" << std::endl;
  vizFile << "    renderCulling ON" << std::endl;
  vizFile << "    pickCulling           ON" << std::endl;
  vizFile << "    title         \"" << m_moc_identifier << "\"" << std::endl;
  vizFile << "    parent                NULL" << std::endl;
  vizFile << "    transform DEF +transform1 MatrixTransform" << std::endl;
  vizFile << "                          {" << std::endl;
  vizFile << "                                  matrix" << std::endl;
  vizFile << "                                  "
          << m_worldToLocal[0][0] << " " << m_worldToLocal[0][1] << " "
          << m_worldToLocal[0][2] << " " << m_worldToLocal[0][3] << std::endl;
  vizFile << "                                  "
          << m_worldToLocal[1][0] << " " << m_worldToLocal[1][1] << " "
          << m_worldToLocal[1][2] << " " << m_worldToLocal[1][3] << std::endl;
  vizFile << "                                  "
          << m_worldToLocal[2][0] << " " << m_worldToLocal[2][1] << " "
          << m_worldToLocal[2][2] << " " << m_worldToLocal[2][3] << std::endl;
  vizFile << "                                  "
          << m_worldToLocal[3][0] << " " << m_worldToLocal[3][1] << " "
          << m_worldToLocal[3][2] << " " << m_worldToLocal[3][3] << std::endl;
  vizFile << "                          }" << std::endl;
  vizFile << "    render DEF +children1 Group" << std::endl;
  vizFile << "    {" << std::endl;
  vizFile << "    }" << std::endl;
  vizFile << "    USE +transform1" << std::endl;
  vizFile << "    USE +children1" << std::endl;
  vizFile << "}" << std::endl;

  vizFile.close();

  vw_out(0) << " done." << std::endl << std::endl;
}

