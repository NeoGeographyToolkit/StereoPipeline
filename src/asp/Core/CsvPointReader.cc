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

/// \file CsvPointReader.cc

#include <asp/Core/CsvPointReader.h>
#include <asp/Core/PointUtils.h>

#include <vw/Core/Exception.h>
#include <vw/Core/Log.h>
#include <vw/Cartography/GeoReference.h>
#include <vw/Cartography/Datum.h>
#include <vw/Cartography/GeoReferenceUtils.h>
#include <vw/Geometry/geomUtils.h>
#include <vw/Math/Vector.h>

#include <boost/filesystem.hpp>
#include <boost/algorithm/string.hpp>

#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>
#include <map>
#include <list>

using namespace vw;

//------------------------------------------------------------------------------------------
// Class CsvConv functions

std::string asp::CsvConv::write_header_string(std::string const delimiter) const{
  std::stringstream s;
  std::map<int, std::string>::const_iterator it;
  for (it = this->col2name.begin(); it != this->col2name.end(); it++)
    s << it->second << delimiter;
  return s.str();
}

// This is a complete list of all supported column names, it must be kept up to date.
int asp::CsvConv::get_sorted_index_for_name(std::string const& name) {
  if (name == "file")               return 3; // The string goes in a different location
  if (name == "lon")                return 0;
  if (name == "lat")                return 1;
  if (name == "radius_m")           return 2;
  if (name == "radius_km")          return 2;
  if (name == "x")                  return 0;
  if (name == "y")                  return 1;
  if (name == "z")                  return 2;
  if (name == "pixel_x")            return 0;
  if (name == "pixel_y")            return 1;
  if (name == "pixel_val")          return 2;
  if (name == "easting")            return 0;
  if (name == "northing")           return 1;
  if (name == "height_above_datum") return 2;

  vw_throw(ArgumentErr() << "Unsupported column name: " << name);
}

// Parse the CSV format string and build the data structure which
// will enable to convert from CSV to Cartesian and vice-versa.
// The user specifies 3 fields that determine the coordinate.
// If min_num_fields 2, autocomplete the third value to 0.
void asp::CsvConv::parse_csv_format(std::string const& csv_format_str,
                                    std::string const& csv_srs,
                                    int min_num_fields) {

  // This guards against the case where the user specifies a proj string
  // only having the string D_MARS, with no spheroid or axes.
  std::string csv_srs_lc = boost::to_lower_copy(csv_srs); // lowercase
  if ((csv_srs_lc.find("d_moon") != std::string::npos ||
       csv_srs_lc.find("d_mars") != std::string::npos) &&
      csv_srs_lc.find("spheroid") == std::string::npos) {
    vw_throw(ArgumentErr() << "D_MOON and D_MARS are not official projection strings. "
                           << "The PROJ string is: " << csv_srs << ".\n");
  }

  *this = asp::CsvConv(); // Reset this object to the default state

  this->csv_format_str = csv_format_str; // Record inputs
  this->csv_srs  = csv_srs;

  std::string local = csv_format_str; // Make lowercase
  boost::algorithm::to_lower(local);

  if (local == "") // No format string passed in
    return;

  boost::replace_all(local, ":", " "); // Convert to space-delimited
  boost::replace_all(local, ",", " ");
  std::istringstream is(local);

  // The case of utm: "utm:23N 1:x 2:y 3:height_above_datum". Parse
  // the initial bit to get utm_zone and utm_north, leave the rest
  // alone.
  std::string str;
  is >> str;
  if (str == "utm") {
    is >> str;
    asp::parse_utm_str(str, this->utm_zone, this->utm_north);
  } else {
    // Go back to the original string
    is.clear();
    is.str(local);
  }

  int col;
  std::string name;
  while (is.good()) {
    // Grab the next two elements
    if (! (is >> col >> name))
      vw_throw(ArgumentErr() << "Could not parse: '" << csv_format_str << "'\n");

    // Convert to zero-based indexing and error check
    col--;
    if (col < 0 || this->col2name.count(col))
      vw_throw(ArgumentErr() << "Illegal column index in: '" << csv_format_str << "'\n");

    // Store in the lookup maps
    this->name2col[name] = col;
    this->col2name[col]  = name;
  }
  this->num_fields = this->name2col.size();
  const int MAX_NUM_FIELDS = 4; // Location and a file
  if ((this->num_fields < min_num_fields) || (this->num_fields > MAX_NUM_FIELDS))
    vw_throw(ArgumentErr() << "Invalid number of column indices in: '" << csv_format_str << "'\n");
  if (min_num_fields < 2)
    vw::vw_throw(vw::ArgumentErr() << "Expecting at least two fields in the csv format.\n");

  // Sort the names into a pre-specified order.
  std::vector<std::string> sorted_names(this->num_fields);
  for (auto it = this->name2col.begin(); it != this->name2col.end(); it++) {
    int index = get_sorted_index_for_name(it->first);
    sorted_names[index] = it->first;
    if (it->first != "file") // Only the point data goes into a vector, not the filename
      this->col2sort[it->second] = index;
  }

  // If only two fields are set, auto-complete the third. Values for that field
  // will be set to 0. This makes it convenient to use many of the functions
  // expecting 3 fields.
  if (this->num_fields == 2 && sorted_names.size() == 2) {
    if (sorted_names[0] == "x" && sorted_names[1] == "y")
      sorted_names.push_back("z");
    else if (sorted_names[0] == "lon" && sorted_names[1] == "lat")
      sorted_names.push_back("height_above_datum");
    else if (sorted_names[0] == "easting"  && sorted_names[1] == "northing")
      sorted_names.push_back("height_above_datum");
    else if (sorted_names[0] == "pixel_x" && sorted_names[1] == "pixel_y")
      sorted_names.push_back("pixel_val");
    else
      sorted_names.push_back("not_found"); // will trigger a failure below
  }

  // From the input strings, determine which set type applies to this file.
  if (sorted_names[0] == "x" &&
      sorted_names[1] == "y" &&
      sorted_names[2] == "z") {
    this->format = XYZ;
  } else if (sorted_names[0] == "lon" &&
            sorted_names[1] == "lat" &&
            sorted_names[2] == "radius_m") {
    this->format = LAT_LON_RADIUS_M;
  } else if (sorted_names[0] == "lon" &&
            sorted_names[1] == "lat" &&
            sorted_names[2] == "radius_km") {
    this->format = LAT_LON_RADIUS_KM;
  } else if (sorted_names[0] == "lon" &&
            sorted_names[1] == "lat" &&
            sorted_names[2] == "height_above_datum") {
    this->format = HEIGHT_LAT_LON;
  } else if (sorted_names[0] == "easting"  &&
            sorted_names[1] == "northing" &&
            sorted_names[2] == "height_above_datum") {
    this->format = EASTING_HEIGHT_NORTHING;
  } else if (sorted_names[0] == "pixel_x" &&
             sorted_names[1] == "pixel_y" &&
             sorted_names[2] == "pixel_val") {
    this->format = PIXEL_XYVAL;
  } else {
    vw_throw(ArgumentErr() << "Cannot understand the csv format string: "
                            << csv_format_str << ".\n");
  }
}

// If the user passed in a csv file containing easting, northing, height
// above datum, and either a utm zone or a custom PROJ string,
// pass that info into the georeference for the purpose of converting
// later from easting and northing to lon and lat.
// TODO(oalexan1): This is fragile. It can quietly default to WGS84.
bool asp::CsvConv::parse_georef(vw::GeoReference & georef) const {

  bool success = false;
  if (this->utm_zone >= 0) { // UTM case, for Earth only
    try{
      georef.set_UTM(this->utm_zone, this->utm_north);
      success = true;
    } catch (const std::exception& e) {
      vw_throw(ArgumentErr() << "Detected error: " << e.what()
                             << "\nPlease check if you are using an Earth datum.\n");
    }
  } else if (this->csv_srs != "") { // Not UTM, with PROJ string
    bool have_user_datum = false;
    Datum user_datum;
    vw::set_srs_string(this->csv_srs, have_user_datum, user_datum, georef);
    success = true;
  }

  if (this->format == EASTING_HEIGHT_NORTHING && !georef.is_projected())
      vw::vw_throw(vw::ArgumentErr() << "When a CSV file has Easting and Northing, "
                   "must set --csv-srs, and it must use projected coordinates, "
                   "not longitude-latitude.\n");

  return success;
}

asp::CsvConv::CsvRecord asp::CsvConv::parse_csv_line(bool & is_first_line, bool & success,
                                                     std::string const& line) const {
  // Parse a CSV file line in given format
  success = true;

  // Copy the input line into a temporary buffer
  const int bufSize = 2048;
  char temp[bufSize];
  strncpy(temp, line.c_str(), bufSize);

  std::string sep = asp::csv_separator();

  int col_index = -1; // The current column we are reading
  int num_floats_read = 0;
  int num_values_read = 0;

  CsvRecord values;

  // Quietly ignore empty lines, lines with spaces only, and lines starting with comments
  if (line.empty() || line[0] == '#' || hasSpacesOnly(line)) {
    success = false;
    is_first_line = false;
    return values;
  }

  char * ptr = temp;
  while (1) {

    col_index++; // Increment the column counter
    const char* token = strtok(ptr, sep.c_str());  // Split line on separator char
    ptr = NULL; // After the first call, strtok expects a null pointer as input.
    if (token == NULL) break; // no more tokens
    if (num_values_read >= this->num_fields) break; // read enough values

    // Check if this is one of the columns we need to read
    if (this->col2name.find(col_index) == this->col2name.end())
      continue;

    if (this->col2name.at(col_index) == "file") // This is a string input
      values.file = token;
    else {
      // Parse the floating point value from the token
      double val;
      int flag = sscanf(token, "%lg", &val);
      if (flag == 0) { // Handle parsing failure
        success = false;
        break;
      }
      values.point_data[num_floats_read] = val;
      num_floats_read++;
    }
    num_values_read++;

  } // End loop through columns

  // Check if enough values were read and for NaN values
  if (num_values_read != this->num_fields || values.point_data != values.point_data)
    success = false;

  if (!success) {
    if (!is_first_line) {
      // Not the header
      vw_out () << "Failed to read line: " << line << "\n";
    }
  }

  is_first_line = false;
  return values;
}

// Search for "color = red" and find "red". Return false on failure.
// Can handle uppercase strings, also "color=red" and "color red".
bool parse_color(std::string const& line, std::string & color) {

  color = ""; // reset the output
  if (line.empty() || line[0] == '#')
    return false;

  std::string line_lc = boost::to_lower_copy(line); // make lowercase

  size_t pos = line_lc.find("color");
  if (pos == std::string::npos)
    return false;

  pos += 5; // go past the color

  // Skip past spaces and equal sign
  while (pos < line_lc.size() && (line_lc[pos] < 'a' || line_lc[pos] > 'z'))
    pos++;

  if (pos >= line_lc.size())
    return false; // no color was set

  line_lc = line_lc.substr(pos);

  // Return the first token. This is useful if there are spaces and other things
  // afterward.
  std::istringstream iss(line_lc);
  std::string token;
  if (iss >> token) {
    color = token;
    return true;
  }

  return false;
}

size_t asp::CsvConv::read_csv_file(std::string const & file_path,
				   std::list<CsvRecord> & output_list) const {

  // Clear output object
  output_list.clear();

  // Open input file
  std::ifstream file(file_path.c_str());
  if (!file)
    vw_throw(vw::IOErr() << "Unable to open file \"" << file_path << "\"");

  // Read through all the lines of the input file, parse each line, and build the output list.
  bool success;
  bool first_line = true; // TODO(oalexan1): Wipe this variable.
  std::string line = "";

  while (std::getline(file, line, '\n')) {

    CsvRecord new_record = asp::CsvConv::parse_csv_line(first_line, success, line);
    if (success)
      output_list.push_back(new_record);

    first_line = false;
  }

  file.close();

  return output_list.size();
}

// Reads an entire CSV file having polygons. Individual
// polygons are separated by a newline or some other unexpected text.
size_t asp::CsvConv::read_poly_file(std::string    const & file_path,
                                    std::list<CsvRecord> & output_list,
                                    std::vector<int>         & contiguous_blocks,
                                    std::vector<std::string> & colors,
                                    std::vector<vw::anno> & annotations) const {

  // Clear output object
  output_list.clear();
  annotations.clear();
  
  std::string color = "green"; // some default
  if (colors.size() > 0) {
    color = colors[0]; // use as default color what is passed from outside
    colors.clear(); // so we can keep on pushing for each polygon
  }

  contiguous_blocks.clear();
  contiguous_blocks.push_back(0);

  // Open the input file
  std::ifstream file(file_path.c_str());
  if (!file)
    vw_throw(vw::IOErr() << "Unable to open file \"" << file_path << "\"");

  // Read through all the lines of the input file, parse each line, and build
  // the output list. We ignore failed lines and lines having colors, and use
  // them as separators between polygons.
  bool success = false;
  bool first_line = true; // TODO(oalexan1): Rename this to: quiet_on_failure.
  std::string line = "";
  vw::anno annotation;

  while (std::getline(file, line, '\n')) {
    
    // Parse colors
    std::string local_color;
    if (parse_color(line, local_color)) {
      color = local_color;
      while (colors.size() < contiguous_blocks.size())
        colors.push_back(color); // catch up on colors
    }

    // Search for annotations
    bool found_anno = false;
    if (line.find("anno ") == 0 && vw::searchForAnnotation(line, annotation)) {
      annotations.push_back(annotation);
      found_anno = true;
      success = false;
    }
    
    CsvRecord new_record;
    if (!found_anno) 
      new_record = asp::CsvConv::parse_csv_line(first_line, success, line);
    first_line = true; // Because for now the api changes this var. To fix at some point.

    if (success && !found_anno) {
      output_list.push_back(new_record);
      // This is a bugfix. If we are just starting a block, and there were some
      // invalid lines before, and some were colors, let the color of this
      // block be the last collected color
      if (contiguous_blocks.size() > 0 &&
        contiguous_blocks.back() == 0 && colors.size() == contiguous_blocks.size()) {
        colors.back() = color;
      }
      // add an element to the last block
      contiguous_blocks.back()++;
    } else if (!found_anno) {
      // An annotation does not force a new block
      if (contiguous_blocks.back() > 0) {
        contiguous_blocks.push_back(0); // Add a new block
        while (colors.size() < contiguous_blocks.size())
          colors.push_back(color); // catch up on colors
      }
    }
  }

  // This is needed in case no colors were found
  while (colors.size() < contiguous_blocks.size())
    colors.push_back(color);

  // Wipe all blocks of length 0. Likely there is only one at the end.
  std::vector<int> & v = contiguous_blocks; // alias
  v.erase(std::remove(v.begin(), v.end(), 0), v.end());
  if (colors.size() > contiguous_blocks.size())
    colors.resize(contiguous_blocks.size());

  file.close();

  return output_list.size();
}

vw::Vector3 asp::CsvConv::sort_parsed_vector3(CsvRecord const& csv) const {
  Vector3 ordered_csv;
  int count = 0;
  const int NUM_POINT_PARAMS = 3;
  for (auto it = this->col2sort.begin(); it != this->col2sort.end(); it++) {
    if (it->second < NUM_POINT_PARAMS) // Don't include elements past the first three
      ordered_csv[it->second] = csv.point_data[count];
    count++;
  }
  return ordered_csv;
}

vw::Vector3 asp::CsvConv::unsort_vector3(vw::Vector3 const& csv) const {
  Vector3 csv2;
  int count = 0;
  const int NUM_POINT_PARAMS = 3;
  for (std::map<int, int>::const_iterator it = this->col2sort.begin(); it != this->col2sort.end(); it++) {
    if (it->second < NUM_POINT_PARAMS) { // Don't include elements past the first three
      csv2[count] = csv[it->second];
      count++;
    }
  }
  return csv2;
}

// There is a lot of repeated code for the next three functions in order to
// improve the speed of parsing points by doing the minimum number of conversions.

// Return either xyz or a projected point. Note that the flag return_point_height
// is not necessarily respected.
Vector3 asp::CsvConv::csv_to_cartesian_or_point_height(CsvRecord const& csv,
                                                       GeoReference const& geo,
                                                       bool return_point_height) const{

  Vector3 ordered_csv = sort_parsed_vector3(csv);

  // If the format is XYZ, there is a good chance we don't even have a reference.
  // So we cannot return a point_height. We need things this way for the chipper,
  // but this is quite confusing.
  if (this->format == XYZ)
    return ordered_csv; // already as xyz

  // Convert from CSV to Cartesian. Later we may convert to point_height format,
  // which, due to the projection in the georeference, may not be the same
  // as the input CSV format. E.g., input CSV may be lon, lat, height,
  // but the output of this function may be easting, northing, height.

  Vector3 xyz;
  if (this->format == EASTING_HEIGHT_NORTHING) {

  Vector3 point_height = Vector3(ordered_csv[0], ordered_csv[1], ordered_csv[2]);
  Vector2 ll  = geo.point_to_lonlat(Vector2(point_height[0], point_height[1]));
  Vector3 llh = Vector3(ll[0], ll[1], point_height[2]); // now lon, lat, height
  xyz = geo.datum().geodetic_to_cartesian(llh);

  } else if (this->format == HEIGHT_LAT_LON) {

    xyz = geo.datum().geodetic_to_cartesian(ordered_csv);

  } else { // Handle asp::LAT_LON_RADIUS_M and asp::LAT_LON_RADIUS_KM

    if (this->format == LAT_LON_RADIUS_KM)
      ordered_csv[2] *= 1000.0; // now lon, lat, radius_m

    Vector3 tmp = ordered_csv; tmp[2] = 0; // now lon, lat, 0
    xyz = geo.datum().geodetic_to_cartesian(tmp);

    // Update the radius
    xyz = ordered_csv[2]*(xyz/norm_2(xyz));

  }

  if (return_point_height)
    return geo.geodetic_to_point(geo.datum().cartesian_to_geodetic(xyz));

  return xyz;
}

vw::Vector3 asp::CsvConv::csv_to_cartesian(CsvRecord const& csv,
                                           vw::GeoReference const& geo) const {
  Vector3 ordered_csv = sort_parsed_vector3(csv);

  Vector3 xyz;
  if (this->format == XYZ) {
    return ordered_csv; // already as xyz

  } else if (this->format == EASTING_HEIGHT_NORTHING) {
    Vector3 point_height = Vector3(ordered_csv[0], ordered_csv[1], ordered_csv[2]);
    Vector2 ll           = geo.point_to_lonlat(Vector2(point_height[0], point_height[1]));
    Vector3 llh          = Vector3(ll[0], ll[1], point_height[2]); // now lon, lat, height
    xyz = geo.datum().geodetic_to_cartesian(llh);

  } else if (this->format == HEIGHT_LAT_LON) {
    xyz = geo.datum().geodetic_to_cartesian(ordered_csv);

  } else { // Handle asp::LAT_LON_RADIUS_M and asp::LAT_LON_RADIUS_KM
    if (this->format == LAT_LON_RADIUS_KM)
      ordered_csv[2] *= 1000.0; // now lon, lat, radius_m

    Vector3 tmp = ordered_csv; tmp[2] = 0; // now lon, lat, 0
    xyz = geo.datum().geodetic_to_cartesian(tmp);

    // Update the radius
    xyz = ordered_csv[2]*(xyz/norm_2(xyz));
  }
  return xyz;
}

// Returns Vector3(lon, lat, height_above_datum)
vw::Vector3 asp::CsvConv::csv_to_geodetic(CsvRecord const& csv,
                                          vw::GeoReference const& geo) const {
  Vector3 ordered_csv = sort_parsed_vector3(csv);
  Vector3 llh;

  if (this->format == XYZ) {
    llh = geo.datum().cartesian_to_geodetic(ordered_csv);

  } else if (this->format == EASTING_HEIGHT_NORTHING) {
    Vector3 point_height = Vector3(ordered_csv[0], ordered_csv[1], ordered_csv[2]);
    Vector2 ll           = geo.point_to_lonlat(Vector2(point_height[0], point_height[1]));
    llh = Vector3(ll[0], ll[1], point_height[2]); // now lon, lat, height

  } else if (this->format == HEIGHT_LAT_LON) {
    return ordered_csv;

  } else { // Handle asp::LAT_LON_RADIUS_M and asp::LAT_LON_RADIUS_KM
    if (this->format == LAT_LON_RADIUS_KM)
      ordered_csv[2] *= 1000.0; // now lon, lat, radius_m

    Vector3 tmp = ordered_csv; tmp[2] = 0; // now lon, lat, 0
    Vector3 xyz = geo.datum().geodetic_to_cartesian(tmp);

    // Update the radius
    xyz = ordered_csv[2]*(xyz/norm_2(xyz));
    llh = geo.datum().cartesian_to_geodetic(xyz);
  }
  return llh;
}

vw::Vector2 asp::CsvConv::csv_to_lonlat(CsvRecord const& csv,
                                        vw::GeoReference const& geo) const {
  Vector3 ordered_csv = sort_parsed_vector3(csv);

  if (this->format == XYZ) {
    Vector3 llh = geo.datum().cartesian_to_geodetic(ordered_csv);
    return Vector2(llh[0], llh[1]);
  } else if (this->format == EASTING_HEIGHT_NORTHING) {
    return geo.point_to_lonlat(Vector2(ordered_csv[0], ordered_csv[1]));
  } else if (this->format == HEIGHT_LAT_LON) {
    return Vector2(ordered_csv[0], ordered_csv[1]);
  } else { // Handle asp::LAT_LON_RADIUS_M and asp::LAT_LON_RADIUS_KM
    return Vector2(ordered_csv[0], ordered_csv[1]);
  }

}

Vector3 asp::CsvConv::cartesian_to_csv(Vector3 const& xyz,
                                       GeoReference const& geo,
                                       double mean_longitude) const {
  Vector3 csv;
  if (this->format == XYZ) {
    csv = xyz; // order is x, y, z

  } else { // format != XYZ, convert to the csv format.

    // Must assert here that the datum was specified.

    Vector3 llh = geo.datum().cartesian_to_geodetic(xyz);   // lon-lat-height
    llh[0] += 360.0*round((mean_longitude - llh[0])/360.0); // 360 deg adjust

    if (this->format == EASTING_HEIGHT_NORTHING) {

      // go from lon, lat to easting, northing
      Vector2 en = geo.lonlat_to_point(Vector2(llh[0], llh[1]));
      csv = Vector3(en[0], en[1], llh[2]); // order is easting, northing, height

    } else if (this->format == HEIGHT_LAT_LON) {
      csv = llh;

    } else {
      // Handle asp::LAT_LON_RADIUS_M and asp::LAT_LON_RADIUS_KM

      llh[2] = norm_2(xyz); // order is lon, lat, radius_m

      if (this->format == LAT_LON_RADIUS_KM) {
        llh[2] /= 1000.0; // order is lon, lat, radius_km
      }
      csv = llh;
    }
  }
  // Now the csv vector contains the sorted values for our format

  // Now we have the csv fields, but they are in the order
  // corresponding to the sorted column names. Need to put them
  // in the same order as they were in the file originally.
  return unsort_vector3(csv);
}

