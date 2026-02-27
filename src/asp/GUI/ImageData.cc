// __BEGIN_LICENSE__
//  Copyright (c) 2006-2026, United States Government as represented by the
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

// \file ImageData.cc
// 
// Implementation of the imageData class

#include <asp/GUI/ImageData.h>
#include <asp/GUI/GuiBase.h>
#include <asp/GUI/GuiConstants.h>
#include <asp/GUI/GuiGeom.h>
#include <asp/Core/StereoSettings.h>
#include <asp/Core/PointUtils.h>

#include <vw/Cartography/GeoReferenceUtils.h>
#include <vw/Cartography/shapeFile.h>
#include <vw/FileIO/FileTypes.h>

#include <boost/filesystem.hpp>
#include <boost/algorithm/string/trim.hpp>

#include <fstream>
#include <sstream>
#include <string>
#include <vector>

using namespace vw;
using namespace vw::geometry;

namespace asp {

namespace fs = boost::filesystem;

// Return true if the extension is .csv or .txt
// TODO(oalexan1): Move this to Common.cc
bool hasCsv(std::string const& fileName) {
  std::string ext = vw::get_extension(fileName);
  if (ext == ".csv" || ext == ".txt")
    return true;

  return false;
}

// Read datum from a csv file, such as a pointmap file saved by bundle_adjust.
bool read_datum_from_csv(std::string const& file, vw::cartography::Datum & datum) {

  int count = 0;
  std::ifstream fh(file);
    std::string line;
    while (getline(fh, line, '\n')) {

      // If the datum was not found early on, give up
      count++;
      if (count > 10)
        return false;

      if (vw::cartography::read_datum_from_str(line, datum))
        return true;

    }
  return false;
}

// Given one or more of --csv-format-str, --csv-srs, and datum, extract
// the needed metadata.
void read_csv_metadata(std::string              const& csv_file,
                       std::string              const& csv_format,
                       bool                            isPoly,
                       asp::CsvConv                  & csv_conv,
                       bool                          & has_pixel_vals,
                       bool                          & has_georef,
                       vw::cartography::GeoReference & georef) {

  if (!fs::exists(csv_file)) {
    popUp("Could not load file: " + csv_file);
    return;
  }

  // Each file has its own csv format. Will be overriden if specified by the user
  // via --csv-format-str.
  std::string local_csv_format_str;

  bool guessed_lola = (csv_file.find("RDR") != std::string::npos &&
                       csv_file.find("PointPerRow") != std::string::npos);

  if (csv_format != "") {
    local_csv_format_str = csv_format;
  } else {
    // For the pointmap and match_offsets files the csv format is known, read it from
    // the file if not specified the user.  Same for anchor_points files written by
    // jitter_solve.
    if (csv_file.find("pointmap") != std::string::npos       || // bundle_adjust
        csv_file.find("match_offsets") != std::string::npos  || // bundle_adjust
        csv_file.find("anchor_points") != std::string::npos  || // jitter_solve
        csv_file.find("ref_terrain") != std::string::npos    || // jitter_solve
        csv_file.find("beg_errors.csv") != std::string::npos || // pc_align
        csv_file.find("end_errors.csv") != std::string::npos)   // pc_align
      local_csv_format_str = "1:lon, 2:lat, 4:height_above_datum";

    // For the diff.csv files produced by geodiff the csv format is known, read it from
    // the file if not specified the user.
    if (csv_file.find("-diff.csv") != std::string::npos) // geodiff
      local_csv_format_str = "1:lon, 2:lat, 3:height_above_datum";

    // LOLA
    if (local_csv_format_str == "" && guessed_lola)
      local_csv_format_str = "2:lon, 3:lat, 4:radius_km";
  }

  // For polygons, can assume that first coordinate is x and second is y
  if (isPoly && local_csv_format_str.empty())
    local_csv_format_str = "1:x, 2:y";

  if (asp::stereo_settings().csv_srs != "")
    vw_out() << "Using projection: " << asp::stereo_settings().csv_srs << "\n";

  try {
    int min_num_fields = 3;
    if (isPoly)
      min_num_fields = 2; // only x and y coordinates may exist
    csv_conv.parse_csv_format(local_csv_format_str,
                              asp::stereo_settings().csv_srs,
                              min_num_fields);
  } catch (...) {
    // Give a more specific error message
    popUp("Could not parse the csv format. Check or specify --csv-format.\n");
    exit(0); // This is fatal, can't do anything without the format
  }

  // For the x, y, z format we will just plot pixel x, pixel y, and value (z).
  // No georeference can be used.
  asp::CsvConv::CsvFormat fmt = csv_conv.get_format();
  if (fmt == asp::CsvConv::XYZ || fmt == asp::CsvConv::PIXEL_XYVAL)
    has_georef = false;
  else
    has_georef = true;

  has_pixel_vals = (fmt == asp::CsvConv::PIXEL_XYVAL);

  if (local_csv_format_str == "") {
    popUp("The option --csv-format-str must be specified.");
    exit(0); // this is fatal
    return;
  }
  vw_out() << "Using CSV format: " << local_csv_format_str << "\n";

  // Handle the datum
  bool has_datum = false;

  // For a pointmap file, anchor points, or a -diff.csv file, read the
  // datum from the file. The --csv-datum option, if set, will
  // override this.
  bool known_csv = (csv_file.find("pointmap") != std::string::npos       ||
                    csv_file.find("anchor_points") != std::string::npos  ||
                    csv_file.find("match_offsets") != std::string::npos  ||
                    csv_file.find("beg_errors.csv") != std::string::npos ||
                    csv_file.find("end_errors.csv") != std::string::npos ||
                    csv_file.find("-diff.csv") != std::string::npos ||
                    asp::hasCsv(csv_file));

  if (known_csv) {
    vw::cartography::Datum datum;
    if (read_datum_from_csv(csv_file, datum)) {
      georef.set_datum(datum);
      has_datum = true;
    }
  }

  // Parse the datum and populate the georef
  csv_conv.parse_georef(georef);
  if (asp::stereo_settings().csv_datum != "") {
    vw::cartography::Datum datum(asp::stereo_settings().csv_datum);
    georef.set_datum(datum);
    has_datum = true;
  }

  if (!has_datum && guessed_lola) {
    vw::vw_out() << "Guessing the datum for LOLA.\n";
    vw::cartography::Datum datum("D_MOON");
    georef.set_datum(datum);
    has_datum = true;
  }

  if (has_georef) {
    if (!has_datum) {
      popUp("Must specify --csv-datum.");
      vw_throw(ArgumentErr() << "Missing --csv-datum.\n");
      return;
    }
  }

  if (has_datum)
    vw::vw_out() << "Using datum: " << georef.datum() << "\n";

  return;
}

void imageData::read(std::string const& name_in, vw::GdalWriteOptions const& opt,
                     DisplayMode display_mode,
                     std::map<std::string, std::string> const& properties,
                     bool delay_loading) {

  if (display_mode == REGULAR_VIEW)
    name = name_in;
  else if (display_mode == HILLSHADED_VIEW)
    hillshaded_name = name_in;
  else if (display_mode == THRESHOLDED_VIEW)
    thresholded_name = name_in;
  else if (display_mode == COLORIZED_VIEW)
    colorized_name = name_in;
  else
    vw::vw_throw(vw::ArgumentErr() << "Unknown display mode.\n");

  // TODO(oalexan1): There is no need to make the color a class member,
  // as it is already stored in individual polygons
  color = "default";
  style = "default";
  colormap = "binary-red-blue";
  colorbar = false;

  m_opt = opt;
  m_display_mode = display_mode;

  // Properties passed on the command line; they take precedence
  for (auto it = properties.begin(); it != properties.end(); it++) {
    if (it->first == "color")
      color = it->second; // copy the poly/line/points color
    if (it->first == "style")
      style = it->second; // copy the style (poly, line, points)
    if (it->first == "colormap")
      colormap = it->second; // copy the colormap style (e.g., binary-red-blue)
    if (it->first == "colorbar")
      colorbar = atof(it->second.c_str());
  }

  // These must be set once we know the name and the style
  m_isPoly = imageData::isPolyInternal(name, style);
  m_isCsv = imageData::isCsvInternal(name, style);

  // When there are many images, we may prefer to load them on demand
  if (!delay_loading)
    load();
}

// A few small local functions
namespace {

// Given a line of the form:
// # key value1 value2 and so on
// Read the key in one string, and the values in another string.
void readEntryVal(std::string line, std::string & key, std::string & val) {

  // Initialize the outputs
  key = "";
  val = "";

  // Skip empty lines or those not starting with a pound sign
  if (line.empty() || line[0] != '#')
    return;

  // Eliminate the pound sign
  line = line.substr(1, line.size()-1);

  std::istringstream is(line);
  is >> key;
  val = "";
  std::string token;
  while (is >> token) {
    val += token + " ";
  }
  boost::algorithm::trim(val);
}

// Open a file and extract the values for: WKT:, csv-format:, style:
void parseCsvHeader(std::string const& file, std::string & wkt,
                    std::string & csv_format, std::string & style) {

  wkt = "";
  csv_format = "";
  style = "";

  std::ifstream fh(file.c_str());

  // Read line by line. Stop when a line no longer starts with a pound sign.
  std::string line;
  while (getline(fh, line, '\n')) {

    // skip empty lines
    if (line.empty())
      continue;

    // Stop if the pound sign is not the first character
    if (line[0] != '#')
      break;

    std::string key, val;
    readEntryVal(line, key, val);

    if (key == "WKT:")
      wkt = val;
    if (key == "csv-format:")
      csv_format = val;
    if (key == "style:")
      style = val;
  }
}

} // end local namespace

// Load the image is not loaded so far
void imageData::load() {

  // Loaded data need not be reloaded
  if (m_display_mode == REGULAR_VIEW) {
    if (loaded_regular)
      return;
    vw_out() << "Reading: " << name << "\n";
    loaded_regular = true;
  } else if (m_display_mode == HILLSHADED_VIEW) {
    if (loaded_hillshaded)
      return;
    vw_out() << "Reading: " << hillshaded_name << "\n";
    loaded_hillshaded = true;
  } else if (m_display_mode == THRESHOLDED_VIEW) {
    if (loaded_thresholded)
      return;
    vw_out() << "Reading: " << thresholded_name << "\n";
    loaded_thresholded = true;
  } else if (m_display_mode == COLORIZED_VIEW) {
    if (loaded_colorized)
      return;
    vw_out() << "Reading: " << colorized_name << "\n";
    loaded_colorized = true;
  }

  std::string default_poly_color = "green"; // default, will be overwritten later

  if (vw::has_shp_extension(name)) {
    // Read a shape file
    std::string poly_color = default_poly_color;
    if (color != "default" && color != "")
      poly_color = color;
    std::string fieldId = "tile_id"; // For reading tile geom written by stereo_parse
    read_shapefile(name, poly_color, has_georef, georef, polyVec, fieldId);

    double xll = -1.0, yll = -1.0, xur = -1.0, yur = -1.0;
    shapefile_bdbox(polyVec,
                    xll, yll, xur, yur); // outputs

    image_bbox.min() = Vector2(xll, yll);
    image_bbox.max() = Vector2(xur, yur);

  } else if (asp::hasCsv(name)) {

    // Open a file and extract the values for: WKT:, csv-format:, style:
    std::string local_wkt, local_csv_format, local_style;
    parseCsvHeader(name, local_wkt, local_csv_format, local_style);

    // The style from the file overrides the style from the command line
    if (local_style != "")
      style = local_style;

    bool isPoly = (style == "poly" || style == "fpoly" || style == "line");
    asp::CsvConv csv_conv;
    bool has_pixel_vals = false; // may change later
    has_georef = true; // this may change later

    // Read poly or csv
    if (isPoly && local_style != "") {
      // The polygon file has all the data
      int numCols = 2; // only x and y coordinates may exist
      csv_conv.parse_csv_format(local_csv_format, local_wkt, numCols);
      has_georef = (local_wkt != "");
      if (has_georef)
        georef.set_wkt(local_wkt);
      has_pixel_vals = (csv_conv.get_format() == asp::CsvConv::PIXEL_XYVAL);
    } else {
      // Use options specified on the command line
      int numCols = asp::fileNumCols(name);
      read_csv_metadata(name, asp::stereo_settings().csv_format_str,
                        isPoly, csv_conv, has_pixel_vals, has_georef, georef);
    }

    std::vector<int> contiguous_blocks;
    std::vector<std::string> colors;
    std::vector<vw::geometry::anno> annotations;
    colors.push_back(default_poly_color); // to provide a default color for the reader

    // Read the file
    std::list<asp::CsvConv::CsvRecord> pos_records;
    if (!isPoly)
      csv_conv.read_csv_file(name, pos_records);
    else
      csv_conv.read_poly_file(name, pos_records, contiguous_blocks, colors, annotations);

    scattered_data.clear(); // note that this is a member variable
    vw::BBox3 bounds;
    for (auto iter = pos_records.begin(); iter != pos_records.end(); iter++) {
      Vector3 val = csv_conv.sort_parsed_vector3(*iter);

      // For pixel values the y axis will go down
      if (has_pixel_vals)
        val[1] *= -1.0;

      scattered_data.push_back(val);
      bounds.grow(val);
    }
    image_bbox.min() = subvector(bounds.min(), 0, 2);
    image_bbox.max() = subvector(bounds.max(), 0, 2);
    val_range[0] = bounds.min()[2];
    val_range[1] = bounds.max()[2];

    if (isPoly) {
      asp::formPoly(color, contiguous_blocks, colors, annotations, scattered_data, polyVec);
      scattered_data.clear(); // the data is now in the poly structure
    }

  }else{
    // Read an image
    int top_image_max_pix = TOP_IMAGE_MAX_PIX;
    int subsample = LOAD_SUBSAMPLE;
    has_georef = vw::cartography::read_georeference(georef, name);
    if (m_display_mode == REGULAR_VIEW) {
      img = DiskImagePyramidMultiChannel(name, m_opt, top_image_max_pix, subsample);
      image_bbox = BBox2(0, 0, img.cols(), img.rows());
    } else if (m_display_mode == HILLSHADED_VIEW) {
      hillshaded_img = DiskImagePyramidMultiChannel(hillshaded_name, m_opt,
                                                    top_image_max_pix, subsample);
      image_bbox = BBox2(0, 0, hillshaded_img.cols(), hillshaded_img.rows());
    } else if (m_display_mode == THRESHOLDED_VIEW) {
      thresholded_img = DiskImagePyramidMultiChannel(thresholded_name, m_opt,
                                                     top_image_max_pix, subsample);
      image_bbox = BBox2(0, 0, thresholded_img.cols(), thresholded_img.rows());
    } else if (m_display_mode == COLORIZED_VIEW) {
      colorized_img = DiskImagePyramidMultiChannel(colorized_name, m_opt,
                                                     top_image_max_pix, subsample);
      image_bbox = BBox2(0, 0, colorized_img.cols(), colorized_img.rows());
    }
  }
}

// Save the polygons to a plain text file. This must be in sync
// with the logic for reading it.
void imageData::writePoly(std::string const& polyFile) {

  // Put all the polygons into a single poly structure
  vw::geometry::dPoly poly;
  for (size_t polyIter = 0; polyIter < this->polyVec.size(); polyIter++)
    poly.appendPolygons(this->polyVec[polyIter]);

  bool has_geo = this->has_georef;
  vw::cartography::GeoReference const& geo = this->georef;

  vw_out() << "Writing: " << polyFile << "\n";

  std::ofstream out(polyFile.c_str());
  if (!out.is_open())
    vw::vw_throw(vw::IOErr() << "Could not open: " << polyFile << "\n");

  // Save to the file all properties that are needed on reading it back
  if (has_geo) {
    out << "# WKT: " << geo.get_wkt() << "\n";
    if (geo.is_projected())
      out << "# csv-format: 1:easting,2:northing\n";
    else
      out << "# csv-format: 1:lon,2:lat\n";
  } else {
    out << "# WKT:\n"; // no georef
    out << "# csv-format: 1:x,2:y\n";
  }
  out << "# style: poly\n";

  std::string defaultColor = "green"; // only to be used if individual colors are missing
  bool emptyLineAsSeparator = true; // Don't want to use a "NEXT" statement as separator
  poly.writePoly(out, defaultColor, emptyLineAsSeparator);
}

// The two functions below are very slow if used per pixel, so we cache their
// values in member variables. Never call these directly.
bool imageData::isPolyInternal(std::string const& name, std::string const& style) const {
  return (vw::has_shp_extension(name) ||
          (asp::hasCsv(name) &&
           (style == "poly" || style == "fpoly" || style == "line")));
}
bool imageData::isCsvInternal(std::string const& name, std::string const& style) const {
  return asp::hasCsv(name) && !imageData::isPolyInternal(name, style);
}

} // namespace asp
