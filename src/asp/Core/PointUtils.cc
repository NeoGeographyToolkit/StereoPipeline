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

#include <asp/Core/PointUtils.h>
#include <asp/Core/PointCloudRead.h>
#include <vw/Core/Stopwatch.h>
#include <vw/FileIO/DiskImageUtils.h>
#include <vw/Mosaic/ImageComposite.h>
#include <vw/Geometry/geomUtils.h>
#include <vw/Image/ImageView.h>
#include <vw/Image/Manipulation.h>

#include <cmath>
#include <cstdint>

using namespace vw;

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
                          vw::Datum& datum) {
  // Select a cartographic datum. There are several hard coded datums
  // that can be used here, or the user can specify their own.
  if (reference_spheroid != "") {
    datum.set_well_known_datum(reference_spheroid);
  } else if (semi_major == 6378137 &&
             std::abs(semi_minor - 6356752.3142451793) < 1e-6) {
    datum.set_well_known_datum("WGS84");
  } else if (semi_major > 0 && semi_minor > 0) {
    datum = vw::Datum("User Specified Datum",
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

// Chunk size (cols, rows) for reading a cloud in bulk blocks. Aligns with
// integer multiples of the file's native block size (block_w, block_h) so
// disk tiles are read whole, capped at a moderate size (~1024x1024, ~25 MB).
vw::Vector2i asp::cloud_chunk_size(int cols, int rows, int block_w, int block_h) {
  int const target = 1024;
  if (block_w <= 0) block_w = 256;
  if (block_h <= 0) block_h = 256;

  int chunk_w, chunk_h;

  if (block_w < cols) {
    int n = std::max(1, target / block_w);
    chunk_w = n * block_w;
  } else {
    chunk_w = std::min(cols, target);
  }

  if (block_h < rows) {
    int n = std::max(1, target / block_h);
    chunk_h = n * block_h;
  } else {
    chunk_h = std::min(rows, target);
  }

  chunk_w = std::max(1, std::min(chunk_w, cols));
  chunk_h = std::max(1, std::min(chunk_h, rows));
  return vw::Vector2i(chunk_w, chunk_h);
}

// Compute bounding box of the given cloud. If is_geodetic is false,
// that means a cloud of raw xyz cartesian values, then Vector3()
// signifies no-data. If is_geodetic is true, no-data is suggested
// by having the z component of the point be NaN.
vw::BBox3 asp::pointcloud_bbox(vw::ImageViewRef<vw::Vector3> const& point_image,
                               bool is_geodetic, int block_w, int block_h) {

  vw::BBox3 result;
  vw::vw_out() << "Computing the point cloud bounding box.\n";
  vw::TerminalProgressCallback progress_bar("asp", "\t--> ");

  int cols = point_image.cols(), rows = point_image.rows();
  vw::Vector2i chunk = asp::cloud_chunk_size(cols, rows, block_w, block_h);
  int chunk_w = chunk.x(), chunk_h = chunk.y();

  // Read block-aligned chunks in bulk (each tile once), rather than lazy
  // per-pixel access that re-decodes tiles and thrashes the cache.
  for (int row0 = 0; row0 < rows; row0 += chunk_h) {
    int h = std::min(chunk_h, rows - row0);
    progress_bar.report_fractional_progress(row0, rows);
    for (int col0 = 0; col0 < cols; col0 += chunk_w) {
      int w = std::min(chunk_w, cols - col0);
      vw::ImageView<vw::Vector3> block = vw::crop(point_image, vw::BBox2i(col0, row0, w, h));
      for (int r = 0; r < h; r++) {
        for (int c = 0; c < w; c++) {
          vw::Vector3 pt = block(c, r);
          if ((!is_geodetic && pt != vw::Vector3()) ||
               (is_geodetic  &&  !std::isnan(pt.z())))
            result.grow(pt);
        }
      }
    }
  }
  progress_bar.report_finished();

  return result;
}

// Strip height: maintained for backward compatibility.
int asp::cloud_strip_rows(int cols, int rows, int block_h) {
  return asp::cloud_chunk_size(cols, rows, 256, block_h).y();
}

// Compute per-axis subsample factors so very-wide point clouds (e.g. long
// thin TMC strips at ~1:37 aspect) don't end up with zero-row sub-images.
// For moderate aspect ratios (subsample_amt <= min_dim/4) this is a no-op
// and sub_x = sub_y = subsample_amt, preserving prior behavior.
void asp::setupSampleRate(vw::int32 cols, vw::int32 rows, vw::int32 subsample_amt,
                          vw::int32 & sub_x, vw::int32 & sub_y) {
  sub_x = subsample_amt;
  sub_y = subsample_amt;
  sub_x = std::min(sub_x, vw::int32(cols / 4));
  sub_x = std::max(sub_x, vw::int32(1));
  sub_y = std::min(sub_y, vw::int32(rows / 4));
  sub_y = std::max(sub_y, vw::int32(1));
}

// Determine if we should be using a longitude range between
// [-180, 180] or [0,360]. The former is used, unless the latter
// results in a tighter range of longitudes, such as when crossing
// the international date line.
vw::BBox2 asp::estim_lonlat_box(vw::ImageViewRef<vw::Vector3> const& point_image,
                                vw::Datum const& datum) {

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

    int32 sub_x = 0, sub_y = 0;
    asp::setupSampleRate(point_image.cols(), point_image.rows(),
                         subsample_amt, sub_x, sub_y);

    ImageViewRef<Vector3> sub_image = subsample(point_image, sub_x, sub_y);

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
                         vw::GeoReference const& georef,
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

    int32 sub_x = 0, sub_y = 0;
    asp::setupSampleRate(point_image.cols(), point_image.rows(),
                         subsample_amt, sub_x, sub_y);

    ImageViewRef<Vector3> sub_image = subsample(point_image, sub_x, sub_y);

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

    lon = vw::median_in_place(lons);
    lat = vw::median_in_place(lats);
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

  vw::GeoReference geo;
  bool has_georef = vw::read_georeference(geo, file_name);

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
    vw::read_header_string(*rsrc.get(), adj_key, val);
    if (val != "HorizontalStdDev")
      has_sd = false;

    adj_key = "BAND6";
    vw::read_header_string(*rsrc.get(), adj_key, val);
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

// Read a single image file as a point cloud or texture, depending on PixelT.
// For PixelGray<float>, just read the file directly. For vector types, use
// the pre-compiled read_asp_point_cloud wrappers to avoid template bloat.
namespace {
template<class PixelT>
typename boost::enable_if<boost::is_same<PixelT, PixelGray<float>>,
                          ImageViewRef<PixelT>>::type
read_point_cloud_compatible_file(std::string const& file) {
  return DiskImageView<PixelT>(file);
}
template<class PixelT>
typename boost::disable_if<boost::is_same<PixelT, PixelGray<float>>,
                           ImageViewRef<PixelT>>::type
read_point_cloud_compatible_file(std::string const& file) {
  constexpr int N = vw::VectorSize<PixelT>::value;
  if constexpr (N == 3)
    return asp::read_asp_point_cloud_3(file);
  else if constexpr (N == 4)
    return asp::read_asp_point_cloud_4(file);
  else if constexpr (N == 6)
    return asp::read_asp_point_cloud_6(file);
  else
    vw::vw_throw(vw::ArgumentErr() << "Unsupported number of channels: " << N << "\n");
}
} // end anonymous namespace

// Read given files and form an image composite.
template<class PixelT>
ImageViewRef<PixelT>
asp::form_point_cloud_composite(std::vector<std::string> const& files,
                                int spacing) {
  VW_ASSERT(files.size() >= 1, ArgumentErr() << "Expecting at least one file.\n");

  vw::ImageComposite<PixelT> composite_image;
  composite_image.set_draft_mode(true);

  for (int i = 0; i < (int)files.size(); i++) {
    ImageViewRef<PixelT> I = read_point_cloud_compatible_file<PixelT>(files[i]);

    // Stack images side by side, transposing wide ones
    if (files.size() > 1 && I.rows() < I.cols())
      I = transpose(I);

    int start = composite_image.cols();
    if (i > 0)
      start = spacing * (int)ceil(double(start) / spacing) + spacing;

    composite_image.insert(I, start, 0);
  }

  return composite_image;
}

// Explicit instantiations of form_point_cloud_composite
template ImageViewRef<PixelGray<float>>
asp::form_point_cloud_composite<PixelGray<float>>(std::vector<std::string> const&, int);
template ImageViewRef<Vector3>
asp::form_point_cloud_composite<Vector3>(std::vector<std::string> const&, int);
template ImageViewRef<Vector4>
asp::form_point_cloud_composite<Vector4>(std::vector<std::string> const&, int);
template ImageViewRef<Vector6>
asp::form_point_cloud_composite<Vector6>(std::vector<std::string> const&, int);

// Read the error channels from the point clouds, and take their norm
template<int num_ch>
ImageViewRef<double>
asp::error_norm(std::vector<std::string> const& pc_files) {
  VW_ASSERT(pc_files.size() >= 1, ArgumentErr() << "Expecting at least one file.\n");

  const int beg_ech = 3;
  const int num_ech = num_ch - beg_ech;
  ImageViewRef<Vector<double, num_ch>> point_disk_image
    = asp::form_point_cloud_composite<Vector<double, num_ch>>
      (pc_files, ASP_MAX_SUBBLOCK_SIZE);
  ImageViewRef<Vector<double, num_ech>> error_channels
    = select_channels<num_ech, num_ch, double>(point_disk_image, beg_ech);

  return per_pixel_filter(error_channels, asp::VectorNorm<Vector<double, num_ech>>());
}

// Explicit instantiations of error_norm
template ImageViewRef<double> asp::error_norm<4>(std::vector<std::string> const&);
template ImageViewRef<double> asp::error_norm<6>(std::vector<std::string> const&);
