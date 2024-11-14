// __BEGIN_LICENSE__
//  Copyright (c) 2009-2024, United States Government as represented by the
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

/// \file PointUtils.cc
///

#include <asp/Core/Macros.h>
#include <asp/Core/Common.h>
#include <asp/Core/PointUtils.h>
#include <vw/Core/Stopwatch.h>

#include <boost/math/special_functions/fpclassify.hpp>
#include <boost/math/special_functions/next.hpp>

using namespace vw;
using namespace vw::cartography;

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
bool asp::CsvConv::parse_georef(vw::cartography::GeoReference & georef) const {

  bool success = false;
  if (this->utm_zone >= 0) { // UTM case
    try{
      georef.set_UTM(this->utm_zone, this->utm_north);
      success = true;
    } catch (const std::exception& e) {
      vw_throw(ArgumentErr() << "Detected error: " << e.what()
                             << "\nPlease check if you are using an Earth datum.\n");
    }
  } else if (this->csv_srs != "") { // Not UTM, with PROJ string
    bool have_user_datum = false, have_input_georef = false;
    Datum user_datum;
    asp::set_srs_string(this->csv_srs, have_user_datum, user_datum,
                        have_input_georef, georef);
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
    const char* token = strtok(ptr, sep.c_str());  // Split line on seperator char
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
                                    std::vector<std::string> & colors) const {

  // Clear output object
  output_list.clear();

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
  bool success;
  bool first_line = true; // TODO(oalexan1): Rename this to: quiet_on_failure.
  std::string line = "";

  while (std::getline(file, line, '\n')) {
    std::string local_color;
    if (parse_color(line, local_color)) {
      color = local_color;
      while (colors.size() < contiguous_blocks.size())
        colors.push_back(color); // catch up on colors
    }

    CsvRecord new_record = asp::CsvConv::parse_csv_line(first_line, success, line);
    first_line = true; // Because for now the api changes this var. To fix at some point.

    if (success) {
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
    } else {
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
                                           vw::cartography::GeoReference const& geo) const {
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
                                          vw::cartography::GeoReference const& geo) const {
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
                                        vw::cartography::GeoReference const& geo) const {
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

// End class CsvConv functions

bool asp::is_tif (std::string const& file) {
  std::string lfile = boost::to_lower_copy(file);
  return (boost::iends_with(lfile, ".tif")  || boost::iends_with(lfile, ".ntf"));
}

bool asp::is_las(std::string const& file) {
  std::string lfile = boost::to_lower_copy(file);
  return (boost::iends_with(lfile, ".las")  || boost::iends_with(lfile, ".laz"));
}

bool asp::is_csv(std::string const& file) {
  std::string lfile = boost::to_lower_copy(file);
  return (boost::iends_with(lfile, ".csv")  || boost::iends_with(lfile, ".txt"));
}

bool asp::is_pcd(std::string const& file) {
  std::string lfile = boost::to_lower_copy(file);
  return boost::iends_with(lfile, ".pcd");
}

bool asp::is_las_or_csv_or_pcd(std::string const& file) {
  return asp::is_las(file) || is_csv(file);
}

bool asp::read_user_datum(double semi_major, double semi_minor,
                          std::string const& reference_spheroid,
                          cartography::Datum& datum) {
  // Select a cartographic datum. There are several hard coded datums
  // that can be used here, or the user can specify their own.
  if (reference_spheroid != "") {
    datum.set_well_known_datum(reference_spheroid);
  } else if (semi_major == 6378137 &&
             std::abs(semi_minor - 6356752.3142451793) < 1e-6) {
    datum.set_well_known_datum("WGS84");
  } else if (semi_major > 0 && semi_minor > 0) {
    datum = cartography::Datum("User Specified Datum",
                               "User Specified Spheroid",
                               "Reference Meridian",
                               semi_major, semi_minor, 0.0);
  } else {
    return false;
  }
  vw_out() << "\t--> Re-referencing altitude values using datum: "
           << datum.name() << ".\n";
  vw_out() << "\t    Axes [" << datum.semi_major_axis() << " "
           << datum.semi_minor_axis() << "] meters.\n";
  return true;
}

void asp::parse_utm_str(std::string const& utm, int & zone, bool & north) {

  // Parse the string 58N

  // Initialize
  zone = -1; north = false;

  std::string a, b;
  for (int s = 0; s < (int)utm.size(); s++) {
    if (utm[s] >= '0' && utm[s] <= '9') {
      a += utm[s];
    } else {
      b = utm[s];
      break;
    }
  }

  if (a == "" || b == "")
    vw_throw(ArgumentErr() << "Could not parse UTM string: '" << utm << "'\n");

  zone = atoi(a.c_str());
  if (b == "n" || b == "N") {
    north = true;
  } else if (b == "s" || b == "S") {
    north = false;
  } else
    vw_throw(ArgumentErr() << "Could not parse UTM string: '" << utm << "'\n");
}

bool asp::hasSpacesOnly(std::string const& str) {
  bool only_spaces = true;
  for (size_t it = 0; it < str.size(); it++) {
    if (str[it] != ' ' && str[it] != '\n' && str[it] != '\t') {
      only_spaces = false;
      break;
    }
  }
  return only_spaces;
}

bool asp::is_valid_csv_line(std::string const& line) {
  // A valid line is not empty and does not start with '#' and does not have spaces only.

  bool only_spaces = hasSpacesOnly(line);

  return (!only_spaces) && (!line.empty()) && (line[0] != '#');
}

std::int64_t asp::csv_file_size(std::string const& file) {

  std::ifstream fh(file.c_str());
  if (!fh)
    vw_throw(vw::IOErr() << "Unable to open file \"" << file << "\"");

  std::int64_t num_total_points = 0;
  std::string line;
  while (getline(fh, line, '\n')) {
    if (!asp::is_valid_csv_line(line)) continue;
    num_total_points++;
  }

  return num_total_points;
}

// Peek at the first valid line in a file to find how many columns it has
int asp::fileNumCols(std::string const& file) {

  const int bufSize = 2048;
  char buffer[bufSize];

  std::string sep = asp::csv_separator();

  int num = 0;
  std::ifstream fh(file.c_str());

  std::string line;
  while (getline(fh, line, '\n')) {
    if (!asp::is_valid_csv_line(line)) continue;

    // Copy the input line into a buffer that can be modified
    strncpy(buffer, line.c_str(), bufSize);
    char * ptr = buffer;

    // Inspect the tokens
    while (1) {

      const char* token = strtok(ptr, sep.c_str());  // Split line on seperator char
      ptr = NULL; // After the first call, strtok expects a null pointer as input.

      if (token == NULL)
        break; // no more tokens

      // Parse the floating point value from the token
      double val;
      int flag = sscanf(token, "%lg", &val);
      if (flag == 0) // Handle parsing failure
        break;

      num++;
    }

    break; // done finding a good line
  }

  return num;
}

// Erases a file suffix if one exists and returns the base string
std::string asp::prefix_from_pointcloud_filename(std::string const& filename) {
  std::string result = filename;

  // First case: filenames that match <prefix>-PC.<suffix>
  int index = result.rfind("-PC.");
  if (index != -1) {
    result.erase(index, result.size());
    return result;
  }

  // Second case: filenames that match <prefix>.<suffix>
  index = result.rfind(".");
  if (index != -1) {
    result.erase(index, result.size());
    return result;
  }

  // No match
  return result;
}

// Compute bounding box of the given cloud. If is_geodetic is false,
// that means a cloud of raw xyz cartesian values, then Vector3()
// signifies no-data. If is_geodetic is true, no-data is suggested
// by having the z component of the point be NaN.
vw::BBox3 asp::pointcloud_bbox(vw::ImageViewRef<vw::Vector3> const& point_image,
                               bool is_geodetic) {

  vw::BBox3 result;
  vw::vw_out() << "Computing the point cloud bounding box.\n";
  vw::TerminalProgressCallback progress_bar("asp", "\t--> ");

  for (int row=0; row < point_image.rows(); ++row) {
    progress_bar.report_fractional_progress(row, point_image.rows());
    for (int col=0; col < point_image.cols(); ++col) {
      vw::Vector3 pt = point_image(col, row);
      if ((!is_geodetic && pt != vw::Vector3()) ||
           (is_geodetic  &&  !boost::math::isnan(pt.z())))
        result.grow(pt);
    }
  }
  progress_bar.report_finished();

  return result;
}

// Determine if we should be using a longitude range between
// [-180, 180] or [0,360]. The former is used, unless the latter
// results in a tighter range of longitudes, such as when crossing
// the international date line.
vw::BBox2 asp::estim_lonlat_box(vw::ImageViewRef<vw::Vector3> const& point_image,
                                vw::cartography::Datum const& datum) {

  Stopwatch sw;
  sw.start();

  // Do two attempts. Need this heuristic, because for small clouds, especially
  // created from CSV files, this can fail.
  vw::BBox2 ll_box, shifted_ll_box;
  for (int attempt = 0; attempt < 2; attempt++) {

    int32 subsample_amt = int32(norm_2(Vector2(point_image.cols(), point_image.rows()))/32.0);
    if (attempt == 1) {
      // For CSV, which can be small
      if (point_image.cols() <= ASP_POINT_CLOUD_TILE_LEN &&
          point_image.rows() <= ASP_POINT_CLOUD_TILE_LEN) {
        subsample_amt = 1;
      } else if (point_image.cols() <= 10 * ASP_POINT_CLOUD_TILE_LEN &&
                 point_image.rows() <= 10 * ASP_POINT_CLOUD_TILE_LEN) {
         // Still a rather small cloud
        subsample_amt = int32(norm_2(Vector2(point_image.cols(), point_image.rows()))/8.0);
      } else {
        // Very big clouds
        subsample_amt = int32(norm_2(Vector2(point_image.cols(), point_image.rows()))/16.0);
      }
    }

    if (subsample_amt < 1)
      subsample_amt = 1;

    ImageViewRef<Vector3> sub_image = subsample(point_image, subsample_amt);

    for (int32 col = 0; col < sub_image.cols(); col++) {
      for (int32 row = 0; row < sub_image.rows(); row++) {
        auto pix = sub_image(col, row);
        if (pix == Vector3())
          continue;

        // Longitude range is [-180, 180]
        vw::Vector3 llh = datum.cartesian_to_geodetic(pix);
        ll_box.grow(Vector2(llh[0], llh[1]));

        // Create the shifted box
        llh[0] += 360;
        if (llh[0] >= 360)
          llh[0] -= 360;
        shifted_ll_box.grow(Vector2(llh[0], llh[1]));
      }
    }

    // Stop at the first attempt if we have a non-empty box
    if (!ll_box.empty())
      break;
  }

  if (ll_box.empty()) {
    // Empty box. The use the [-180, 180] range.
    ll_box = vw::BBox2(-0.1, -0.1, 0.2, 0.2);
    return ll_box;
  }

  // See which has narrower range. Account for numerical error.
  if (ll_box.width() > shifted_ll_box.width() + 1e-10)
    ll_box = shifted_ll_box;

  sw.stop();
  vw_out(DebugMessage,"asp") << "Statistics time: " << sw.elapsed_seconds() << std::endl;

  return ll_box;
}

// Find the median longitude and latitude for a subset of the point cloud
void asp::median_lon_lat(vw::ImageViewRef<vw::Vector3> const& point_image,
                         vw::cartography::GeoReference const& georef,
                         double & lon, double & lat) {

  vw_out() << "Estimating the median longitude and latitude for the cloud.\n";

  // Initialize the outputs
  lon = 0.0; lat = 0.0;

  // Try to subsample with these amounts
  std::vector<double> sub = {32.0, 16.0};

  Stopwatch sw;
  sw.start();

  // Iterate over sub
  bool success = false;
  for (size_t s = 0; s < sub.size(); s++) {

    int32 subsample_amt = int32(norm_2(Vector2(point_image.cols(),
                                               point_image.rows()))/sub[s]);
    if (subsample_amt < 1)
      subsample_amt = 1;

    ImageViewRef<Vector3> sub_image = subsample(point_image, subsample_amt);

    // Accumulate valid values
    std::vector<double> lons, lats;
    for (int32 col = 0; col < sub_image.cols(); col++) {
      for (int32 row = 0; row < sub_image.rows(); row++) {
        Vector3 ecef = sub_image(col, row);
        if (ecef == Vector3())
          continue;
        Vector3 llh = georef.datum().cartesian_to_geodetic(ecef);
        lons.push_back(llh[0]);
        lats.push_back(llh[1]);
      }
    }
    if (lons.empty() || lats.empty())
      continue;

    lon = vw::math::destructive_median(lons);
    lat = vw::math::destructive_median(lats);
    success = true;
    break;
  }

  if (!success)
    vw_throw(ArgumentErr() << "Could not find a valid median longitude and latitude. "
             << "Check if your cloud is empty.\n");

  sw.stop();
  vw_out(DebugMessage,"asp") << "Median longitude and latitude elapsed time: "
                             << sw.elapsed_seconds() << std::endl;
}

/// Analyze a file name to determine the file type
std::string asp::get_cloud_type(std::string const& file_name) {

  if (asp::is_csv(file_name))
    return "CSV";
  if (asp::is_las(file_name))
    return "LAS";

  // Note that any tif, ntf, and cub file with one channel with georeference be
  // interpreted as a DEM.
  int nc = vw::get_num_channels(file_name);

  vw::cartography::GeoReference geo;
  bool has_georef = vw::cartography::read_georeference(geo, file_name);

  if (nc == 1 && has_georef)
    return "DEM";
  if (nc >= 3)
    return "PC";
  vw_throw(vw::ArgumentErr() << "File: " << file_name
                         << " is neither a point cloud nor a DEM.\n");
}

// Find the number of channels in the point clouds.
// If the point clouds have inconsistent number of channels,
// return the minimum of 3 and the minimum number of channels.
// This will be used to flag that we cannot reliable extract the
// error channels, which start at channel 4.
int asp::num_channels(std::vector<std::string> const& pc_files) {

  VW_ASSERT(pc_files.size() >= 1,
             ArgumentErr() << "Expecting at least one point cloud file.\n");

  int num_channels0 = get_num_channels(pc_files[0]);
  int min_num_channels = num_channels0;
  for (int i = 1; i < (int)pc_files.size(); i++) {
    int num_channels = get_num_channels(pc_files[i]);
    min_num_channels = std::min(min_num_channels, num_channels);
    if (num_channels != num_channels0)
      min_num_channels = std::min(min_num_channels, 3);
  }
  return min_num_channels;
}

// See if all the input point cloud files have stddev values
bool asp::has_stddev(std::vector<std::string> const& pc_files) {

  VW_ASSERT(pc_files.size() >= 1,
            ArgumentErr() << "Expecting at least one point cloud file.\n");

  bool has_sd = true;
  for (size_t i = 0; i < pc_files.size(); i++) {

    std::string val;
    boost::shared_ptr<vw::DiskImageResource> rsrc(new vw::DiskImageResourceGDAL(pc_files[i]));

    std::string adj_key = "BAND5";
    vw::cartography::read_header_string(*rsrc.get(), adj_key, val);
    if (val != "HorizontalStdDev")
      has_sd = false;

    adj_key = "BAND6";
    vw::cartography::read_header_string(*rsrc.get(), adj_key, val);
    if (val != "VerticalStdDev")
      has_sd = false;
  }

  if (has_sd && asp::num_channels(pc_files) < 6)
    has_sd = false;

  return has_sd;
}

// Get a handle to the error image given a set of point clouds with 4 or 6 bands
vw::ImageViewRef<double> asp::point_cloud_error_image
  (std::vector<std::string> const& pointcloud_files) {

  ImageViewRef<double> error_image;
  int num_channels = asp::num_channels(pointcloud_files);
  bool has_sd = asp::has_stddev(pointcloud_files);

  if (num_channels == 4 || (num_channels == 6 && has_sd)) {
    // The error is a scalar (4 channels or 6 channels but last two are stddev values)
    error_image = asp::error_norm<4>(pointcloud_files);
  } else if (num_channels == 6) {
    error_image = asp::error_norm<6>(pointcloud_files);
  } else {
    // Return an empty image
    ImageView<double> image;
    image.set_size(0, 0);
    error_image = image;
  }

  return error_image;
}
