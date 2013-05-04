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


/// \file Metadata.h
///

#ifndef __MOC_METADATA_H__
#define __MOC_METADATA_H__

#include <vw/Core/Exception.h>

// Local exception to MOC, HRSC, MRO code
VW_DEFINE_EXCEPTION(EphemerisErr, vw::Exception);

#include <vw/Math/Matrix.h>
#include <vw/Math/Vector.h>
#include <vw/Math/Quaternion.h>
#include <vw/Camera/OrbitingPushbroomModel.h>

#include <asp/SpiceIO/SpiceUtilities.h>

#include <iostream>
#include <fstream>
#include <string>

#define MOLA_PEDR_EQUATORIAL_RADIUS 3396000.0
#define IAU2000_MARS_EQUATORIAL_RADIUS 3396190.0
#define IAU2000_MARS_POLAR_RADIUS 3376200.0

// Constants
#define SUP_EPHEMERIS_BUFFER_SIZE 2048


class MOCImageMetadata {

 public:
  MOCImageMetadata(std::string const& filename);

  void read_tabulated_description(std::string const& filename);
  void read_ephemeris_supplement(std::string const& filename);
  void read_spice_data();
  void write_viz_site_frame(std::string prefix);

  /// Returns a newly allocated camera model object of the appropriate
  /// type.  It is the responsibility of the user to later deallocate
  /// this camera model object or to manage it using some sort of smart
  /// pointer.
  vw::camera::CameraModel* camera_model();

  // Accessors
  double latitude() const { return m_latitude; }
  double longitude() const { return m_longitude; }
  int cols() const { return (int)m_width_pixels; }
  int rows() const { return (int)m_height_pixels; }
  double x_resolution_meters() const { return m_width_meters/this->cols(); }
  double y_resolution_meters() const { return m_height_meters/this->rows(); }
  double scan_duration() const { return m_line_integration_time * rows() * m_downtrack_summing; }
  double ephemeris_time() const { return spice::sclk_to_et(m_sclk_time, spice::NAIF_ID_MGS); }
  int orbit_number() const { return m_orbit_number; }
  std::string moc_identifier() const { return m_moc_identifier; }
  double crosstrack_summing() const { return m_crosstrack_summing; }
  double downtrack_summing() const { return m_crosstrack_summing; }
  double start_sample() const {return m_start_sample; }

private:

  // Constants
  double m_focal_length;
  double m_along_scan_pixel_size;
  double m_across_scan_pixel_size;

  // Parameters from description.tab file
  std::string m_filename;
  std::string m_moc_identifier;
  std::string m_sclk_time;
  double m_latitude, m_longitude;
  double m_downtrack_summing;
  double m_crosstrack_summing;
  double m_aspect;
  double m_angle;
  double m_width_meters;
  double m_height_meters;
  double m_width_pixels;
  double m_height_pixels;
  double m_north_angle;
  double m_skew_angle;
  bool m_flip;
  double m_res_x;
  double m_res_y;
  unsigned int m_orbit_number;
  double m_line_integration_time;

  // Parameterns supplied from supplementary ephemeris file
  double m_start_sample;
  std::vector<vw::Vector3> m_ephem;
  std::vector<vw::Vector3> m_ephem_rate;
  std::vector<vw::Quaternion<double> > m_quat;
  double m_t0_ephem;
  double m_dt_ephem;
  double m_t0_quat;
  double m_dt_quat;

  void parse_ephemeris_entry(std::vector<std::string> ephemerisData);
  vw::Matrix<double> GenWorldToLocalTransform(double lat, double lon, double radius);

  vw::Matrix<double> m_worldToLocal;
};

class SupplementaryEphemerisParser {
  std::ifstream m_data_file;

public:
  SupplementaryEphemerisParser(std::string const& filename) {
    m_data_file.open(filename.c_str());

    // Make sure the file was opened correctly
    if( !(m_data_file.is_open() ) ) {
      throw vw::IOErr() << "Could not open ephemeris file.";
    }
  }

  ~SupplementaryEphemerisParser() {
    if (m_data_file.is_open())
      m_data_file.close();
  }

  double read_double(const char* tag) {

    char buffer[SUP_EPHEMERIS_BUFFER_SIZE];
    char fieldName[SUP_EPHEMERIS_BUFFER_SIZE];
    char fieldValue[SUP_EPHEMERIS_BUFFER_SIZE];
    std::istringstream stringStream, tokenStream;
    double result;

    m_data_file.seekg (0, std::ios::beg);  // Rewind

    while (!m_data_file.eof()) {
      m_data_file.getline(buffer, SUP_EPHEMERIS_BUFFER_SIZE);

      // Attach a stream reader to the string
      stringStream.str(buffer);
      stringStream.clear();

      // Parse out the label and value from the line in the ephemeris file.
      stringStream >> std::ws;                         // Skip white space
      stringStream.getline(fieldName, SUP_EPHEMERIS_BUFFER_SIZE, '=');

      // Remove the trailing whitespace, if it exists.
      if (fieldName[strlen(fieldName) - 1] == ' ') {
        fieldName[strlen(fieldName) - 1] = fieldName[strlen(fieldName)];
      }

      stringStream.getline(fieldValue, SUP_EPHEMERIS_BUFFER_SIZE, '=');

      if (strcmp(fieldName, tag) == 0) {
        sscanf(fieldValue, "%lf", &result);
        return result;
      }
    }

    // If we get here, we failed to find the tag in question.
    throw EphemerisErr() << "Could not find the specified tag in the ephemeris file.";
  }

  vw::Matrix<double> read_matrix(const char* tag, int rows, int cols) {
    char buffer[SUP_EPHEMERIS_BUFFER_SIZE];
    char fieldName[SUP_EPHEMERIS_BUFFER_SIZE];
    char fieldValue[SUP_EPHEMERIS_BUFFER_SIZE];
    char token[SUP_EPHEMERIS_BUFFER_SIZE];
    std::istringstream stringStream, tokenStream;
    double val;

    m_data_file.seekg (0, std::ios::beg);  /* Rewind */

    while (!m_data_file.eof()) {
      m_data_file.getline(buffer, SUP_EPHEMERIS_BUFFER_SIZE);

      // Attach a stream reader to the string
      stringStream.str(buffer);
      stringStream.clear();

      // Parse out the label and value from the line in the ephemeris file.
      stringStream >> std::ws;                                              // Skip white space
      stringStream.getline(fieldName, SUP_EPHEMERIS_BUFFER_SIZE, '=');

      // Remove the trailing whitespace, if it exists.
      if (fieldName[strlen(fieldName) - 1] == ' ') {
        fieldName[strlen(fieldName) - 1] = fieldName[strlen(fieldName)];
      }


      // If we have found the field name that matches the tag we were searching for,
      // stop and read in a series of data values
      if (strcmp(fieldName, tag) == 0) {
        vw::Matrix<double> result(rows, cols);
        for (int i = 0; i < rows; i++) {
          if (m_data_file.eof()) {
            throw EphemerisErr() << "Reached end of file before the entire matrix was read in.";
          }

          // Read in the row of values
          m_data_file.getline(fieldValue, SUP_EPHEMERIS_BUFFER_SIZE);

          // Attach a stream reader to the line
          tokenStream.str(fieldValue);
          tokenStream.clear();
          tokenStream >> std::ws;                                              // Skip white space

          for (int j = 0; j < cols; j++) {
            tokenStream.getline(token, SUP_EPHEMERIS_BUFFER_SIZE, ' ');
            sscanf(token, "%lf", &val);
            result(i,j) = val;
          }

        }
        return result;
      }
    }
    // If we get here, we failed to find the tag in question.
    throw EphemerisErr() << "Could not find the tag in the ephemeris file.";
  }

  std::vector<vw::Vector3> read_vector3s(const char* tag, int rows, int cols) {
    char buffer[SUP_EPHEMERIS_BUFFER_SIZE];
    char fieldName[SUP_EPHEMERIS_BUFFER_SIZE];
    char fieldValue[SUP_EPHEMERIS_BUFFER_SIZE];
    char token[SUP_EPHEMERIS_BUFFER_SIZE];
    std::istringstream stringStream, tokenStream;
    double val;

    m_data_file.seekg (0, std::ios::beg);  /* Rewind */

    while (!m_data_file.eof()) {
      m_data_file.getline(buffer, SUP_EPHEMERIS_BUFFER_SIZE);

      // Attach a stream reader to the string
      stringStream.str(buffer);
      stringStream.clear();

      // Parse out the label and value from the line in the ephemeris file.
      stringStream >> std::ws;                                              // Skip white space
      stringStream.getline(fieldName, SUP_EPHEMERIS_BUFFER_SIZE, '=');

      // Remove the trailing whitespace, if it exists.
      if (fieldName[strlen(fieldName) - 1] == ' ') {
        fieldName[strlen(fieldName) - 1] = fieldName[strlen(fieldName)];
      }

      // If we have found the field name that matches the tag we were searching for,
      // stop and read in a series of data values
      if (strcmp(fieldName, tag) == 0) {
        std::vector<vw::Vector3> result(rows);
        for (int i = 0; i < rows; i++) {
          // Check to make sure we don't read past the end of the file
          if (m_data_file.eof())
            throw EphemerisErr() << "Reached end of file before the entire matrix was read in.";

          // Read in the row of values
          m_data_file.getline(fieldValue, SUP_EPHEMERIS_BUFFER_SIZE);

          // Attach a stream reader to the line
          tokenStream.str(fieldValue);
          tokenStream.clear();
          tokenStream >> std::ws;                                              // Skip white space

          vw::Vector3 temporary;
          for (int j = 0; j < cols; j++) {
            tokenStream.getline(token, SUP_EPHEMERIS_BUFFER_SIZE, ' ');
            sscanf(token, "%lf", &val);
            temporary(j) = val;
          }
          result[i] = temporary;

        }
        return result;
      }
    }
    // If we get here, we failed to find the tag in question.
    throw EphemerisErr() << "Could not find the tag in the ephemeris file.";
  }

  std::vector<vw::Quaternion<double> > read_quaternions(const char* tag, int rows, int cols) {
    char buffer[SUP_EPHEMERIS_BUFFER_SIZE];
    char fieldName[SUP_EPHEMERIS_BUFFER_SIZE];
    char fieldValue[SUP_EPHEMERIS_BUFFER_SIZE];
    char token[SUP_EPHEMERIS_BUFFER_SIZE];
    std::istringstream stringStream, tokenStream;
    double val;

    m_data_file.seekg (0, std::ios::beg);  /* Rewind */

    while (!m_data_file.eof()) {
      m_data_file.getline(buffer, SUP_EPHEMERIS_BUFFER_SIZE);

      // Attach a stream reader to the string
      stringStream.str(buffer);
      stringStream.clear();

      // Parse out the label and value from the line in the ephemeris file.
      stringStream >> std::ws;                                              // Skip white space
      stringStream.getline(fieldName, SUP_EPHEMERIS_BUFFER_SIZE, '=');

      // Remove the trailing whitespace, if it exists.
      if (fieldName[strlen(fieldName) - 1] == ' ') {
        fieldName[strlen(fieldName) - 1] = fieldName[strlen(fieldName)];
      }

      // If we have found the field name that matches the tag we were searching for,
      // stop and read in a series of data values
      if (strcmp(fieldName, tag) == 0) {
        std::vector<vw::Quaternion<double> > result(rows);
        for (int i = 0; i < rows; i++) {
          // Check to make sure we don't read past the end of the file
          if (m_data_file.eof())
            throw EphemerisErr() << "Reached end of file before the entire matrix was read in.";

          // Read in the row of values
          m_data_file.getline(fieldValue, SUP_EPHEMERIS_BUFFER_SIZE);

          // Attach a stream reader to the line
          tokenStream.str(fieldValue);
          tokenStream.clear();
          tokenStream >> std::ws;                                              // Skip white space

          vw::Quaternion<double> temporary;
          for (int j = 0; j < cols; j++) {
            tokenStream.getline(token, SUP_EPHEMERIS_BUFFER_SIZE, ' ');
            sscanf(token, "%lf", &val);
            temporary[j] = val;
          }
          result[i] = temporary;

        }
      return result;
      }
    }

    // If we get here, we failed to find the tag in question.
    throw EphemerisErr() << "Could not find the tag in the ephemeris file.";
  }
};

#endif // __MOC_METADATA_H__
