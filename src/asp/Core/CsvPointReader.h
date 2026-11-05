// __BEGIN_LICENSE__
//  Copyright (c) 2009-2026, United States Government as represented by the
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

/// \file CsvPointReader.h

// Data structure and utilities for converting CSV lines to Cartesian/geodetic
// points and vice-versa.

#ifndef __ASP_CORE_CSV_POINT_READER_H__
#define __ASP_CORE_CSV_POINT_READER_H__

#include <vw/Math/Vector.h>

#include <map>
#include <string>
#include <vector>
#include <list>

namespace vw {
  class GeoReference;
  class anno;
}

namespace asp {

// A Data structure which converts from CSV to Cartesian and vice-versa.
// TODO(oalexan1): There is no enforcement that the georeference has actually
// been populated before it gets used. Fixing this will be quite some work
// given the many places this is used.
class CsvConv {

public: // Definitions

  /// Any valid input file must contain one of these sets of fields
  /// - Note that the data is not stored in this order internally.
  enum CsvFormat {
    XYZ, HEIGHT_LAT_LON, LAT_LON_RADIUS_M,
    LAT_LON_RADIUS_KM, EASTING_HEIGHT_NORTHING, PIXEL_XYVAL
  };

  /// Object used to store the data parsed from a CSV line.
  struct CsvRecord {
    vw::Vector3 point_data;
    std::string file;
  };

public: // Functions

  /// Default Constructor, the object is not ready to use.
  CsvConv() : format(XYZ), utm_zone(-1), utm_north(false), num_fields(0) {}

  bool      is_configured() const { return csv_format_str != ""; }
  CsvFormat get_format   () const { return format; }

  /// Writes out a header string containing each of the extracted column names
  /// in the order they were specified.
  std::string write_header_string(std::string const delimiter = ", ") const;

  /// Initialize using a pair of CSV format strings
  void parse_csv_format(std::string const& csv_format_str,
                        std::string const& csv_srs,
                        int min_num_fields = 3);

  /// If the user passed in a csv file containing easting, northing, height
  /// above datum, and either a utm zone or a custom proj4 string,
  /// pass that info into the georeference for the purpose of converting
  /// later from easting and northing to lon and lat.
  bool parse_georef(vw::GeoReference & georef) const;

  /// Extract values we care about from a csv string and pack them in a vector.
  /// - This extracts the values in the same order they appear in the file.
  /// - Use one of the conversion functions to get coordinates from this vector.
  CsvRecord parse_csv_line(bool & is_first_line, bool & success,
                           std::string const& line) const;

  /// Reads an entire CSV file and stores a record for each line.
  /// - Intended for use with smaller files.
  size_t read_csv_file(std::string const    & file_path,
                       std::list<CsvRecord> & output_list) const;

  /// Reads an entire CSV file having polygons. Individual
  /// polygons are separated by a newline or some other unexpected text.
  size_t read_poly_file(std::string     const   & file_path,
                        std::list<CsvRecord>     & output_list,
                        std::vector<int>         & contiguous_blocks,
                        std::vector<std::string> & colors,
                        std::vector<vw::anno>    & annotations) const;

  /// Convert values read from a csv file using parse_csv_line (in
  /// the same order they appear in the file) to a Cartesian
  /// point. If return_point_height is true, and the csv point is
  /// not in xyz format, return instead the projected point and
  /// height above datum.
  vw::Vector3 csv_to_cartesian_or_point_height(CsvRecord const& csv,
                                               vw::GeoReference const& geo,
                                               bool return_point_height) const;

  /// Convert values read from a csv file using parse_csv_line to a Cartesian point.
  vw::Vector3 csv_to_cartesian(CsvRecord const& csv,
                               vw::GeoReference const& geo) const;

  /// Convert values read from a csv file using parse_csv_line to a lon/lat/height point.
  vw::Vector3 csv_to_geodetic(CsvRecord const& csv,
                              vw::GeoReference const& geo) const;

  /// Convert values read from a csv file using parse_csv_line to a lon/lat point.
  vw::Vector2 csv_to_lonlat(CsvRecord const& csv,
                            vw::GeoReference const& geo) const;

  /// Extracts the file name read from the csv line, or "" if it was not present.
  std::string file_from_csv(CsvRecord const& csv) const { return csv.file; }

  /// Re-order an xyz point so the values appear in the format and order they
  /// did in the csv file. This function does nothing about all the fields we
  /// ignored when we read the file.
  vw::Vector3 cartesian_to_csv(vw::Vector3 const& xyz,
                               vw::GeoReference const& geo,
                               double mean_longitude) const;

  /// Converts the point data vector from the order the fields appeared in the file
  /// to the order according to the "format" member variable.
  vw::Vector3 sort_parsed_vector3(CsvRecord const& v) const;

  /// Performs the reverse of sort_parsed_vector3, putting the values in the
  /// order that they originally appeared in the file (ignores the file
  /// field).
  vw::Vector3 unsort_vector3(vw::Vector3 const& v) const;

private: // Variables
  std::map<std::string, int> name2col; ///< Target names -> Column index in input csv
  std::map<int, std::string> col2name; ///< Target column in input csv -> Name
  std::map<int, int>         col2sort; ///< Which input columns went in which vector indices (numbers only)

  std::string csv_format_str;
  std::string csv_srs;
  CsvFormat   format;
  int         utm_zone;
  bool        utm_north;
  int         num_fields; ///< The number of elements to extract from each CSV line

  friend class CsvReader;

private: // Functions

  /// This function hard-codes the location in the parsed vector where each
  /// column type will go.
  static int get_sorted_index_for_name(std::string const& name);

}; // End class CsvConv

} // end namespace asp

#endif // __ASP_CORE_CSV_POINT_READER_H__
