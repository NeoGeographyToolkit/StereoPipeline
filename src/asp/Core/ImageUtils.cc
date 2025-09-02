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

#include <asp/Core/ImageUtils.h>
#include <asp/Core/FileUtils.h>

#include <vw/FileIO/DiskImageResourceGDAL.h>
#include <vw/Cartography/GeoReference.h>
#include <vw/FileIO/DiskImageUtils.h>
#include <vw/Image/Interpolation.h>

#include <boost/filesystem.hpp>

using namespace vw;

namespace fs = boost::filesystem;

namespace asp {

/// Load an input image, georef, and nodata value
void load_image(std::string const& image_file,
                vw::ImageViewRef<double> & image, double & nodata,
                bool & has_georef, vw::cartography::GeoReference & georef) {
  
  // Ensure the output variables are initialized
  nodata = -std::numeric_limits<double>::max();
  has_georef = false;

  image = vw::load_image_as_double(image_file);

  // Read nodata-value from disk
  DiskImageResourceGDAL in_rsrc(image_file);
  bool has_nodata = in_rsrc.has_nodata_read();
  if (has_nodata) {
    nodata = in_rsrc.nodata_read();
    //vw_out() << "Read no-data value for image " << image_file << ": " << nodata << ".\n";
  } else {
    nodata = vw::get_default_nodata(in_rsrc.channel_type());
  }
  
  has_georef = vw::cartography::read_georeference(georef, image_file);
}

/// Create a DEM ready to use for interpolation
void create_interp_dem(std::string const& dem_file,
                       vw::cartography::GeoReference & dem_georef,
                       ImageViewRef<PixelMask<double>> & interp_dem) {
  
  vw_out() << "Loading DEM: " << dem_file << std::endl;

  // Read the no-data
  double nodata_val = -std::numeric_limits<float>::max(); // note we use a float nodata
  if (vw::read_nodata_val(dem_file, nodata_val))
    vw_out() << "Found DEM nodata value: " << nodata_val << std::endl;

  // Create the interpolated DEM. Values out of bounds will be invalid.
  vw::PixelMask<double> invalid_val;
  invalid_val[0] = nodata_val;
  invalid_val.invalidate();
  ImageViewRef<PixelMask<double>> dem
    = create_mask(DiskImageView<double>(dem_file), nodata_val);
  interp_dem = interpolate(dem, BilinearInterpolation(), 
                           vw::ValueEdgeExtension<vw::PixelMask<float>>(invalid_val));

  // Read the georef. It must exist.
  bool is_good = vw::cartography::read_georeference(dem_georef, dem_file);
  if (!is_good) {
    vw_throw(ArgumentErr() << "Error: Cannot read a georeference from DEM: "
             << dem_file << ".\n");
  }
}

/// Take an interest point from a map projected image and convert it
/// to the corresponding IP in the original non-map-projected image.
/// - Return false if the pixel could not be converted.
bool projected_ip_to_raw_ip(vw::ip::InterestPoint &P,
                            vw::ImageViewRef<vw::PixelMask<double>> const& interp_dem,
                            vw::CamPtr camera_model,
                            vw::cartography::GeoReference const& georef,
                            vw::cartography::GeoReference const& dem_georef) {

  // Get IP coordinate in the DEM
  Vector2 pix(P.x, P.y);
  Vector2 ll      = georef.pixel_to_lonlat(pix);
  Vector2 dem_pix = dem_georef.lonlat_to_pixel(ll);
  if (!interp_dem.pixel_in_bounds(dem_pix))
    return false;
  // Load the elevation from the DEM
  PixelMask<double> dem_val = interp_dem(dem_pix[0], dem_pix[1]);
  if (!is_valid(dem_val))
    return false;
  Vector3 llh(ll[0], ll[1], dem_val.child());
  Vector3 xyz = dem_georef.datum().geodetic_to_cartesian(llh);

  // Project into the camera
  Vector2 cam_pix;
  try {
   cam_pix = camera_model->point_to_pixel(xyz);
  } catch(...) {
    return false; // Don't update the point.
  }
  P.x  = cam_pix.x();
  P.y  = cam_pix.y();
  P.ix = P.x;
  P.iy = P.y;
  return true;
}

// Read keywords that describe how the images were map-projected.
void read_mapproj_header(std::string const& map_file,
                         // Outputs
                         std::string & adj_key, std::string & img_file_key,
                         std::string & cam_type_key, std::string & cam_file_key, 
                         std::string & dem_file_key,
                         std::string & adj_prefix,
                         std::string & image_file, std::string & cam_type,
                         std::string & cam_file, std::string & dem_file) {

  boost::shared_ptr<vw::DiskImageResource> rsrc(new vw::DiskImageResourceGDAL(map_file));
  adj_key      = "BUNDLE_ADJUST_PREFIX"; 
  img_file_key = "INPUT_IMAGE_FILE";
  cam_type_key = "CAMERA_MODEL_TYPE",
  cam_file_key = "CAMERA_FILE";
  dem_file_key = "DEM_FILE"; 

  vw::cartography::read_header_string(*rsrc.get(), adj_key,      adj_prefix);
  vw::cartography::read_header_string(*rsrc.get(), img_file_key, image_file);
  vw::cartography::read_header_string(*rsrc.get(), cam_type_key, cam_type);
  vw::cartography::read_header_string(*rsrc.get(), cam_file_key, cam_file);
  vw::cartography::read_header_string(*rsrc.get(), dem_file_key, dem_file);
  
  // This is important. When writing, have to write something, so use NONE,
  // but on reading, if the string is NONE, make it empty.
  if (adj_prefix == "NONE") 
    adj_prefix = "";
} 

/// Function to apply a functor to each pixel of an input image.
/// Traverse the image row by row.
template <class ViewT, class FuncT>
void for_each_pixel_rowwise(const vw::ImageViewBase<ViewT> &view_, FuncT &func,
  vw::TerminalProgressCallback const& progress) {

  const ViewT& view = view_.impl();
  typedef typename ViewT::pixel_accessor pixel_accessor;
  pixel_accessor plane_acc = view.origin();

  for (int32 plane = view.planes(); plane; plane--) { // Loop through planes

    pixel_accessor row_acc = plane_acc;
    for (int32 row = 0; row<view.rows(); row++) { // Loop through rows
      progress.report_fractional_progress(row, view.rows());
      pixel_accessor col_acc = row_acc;
      for (int32 col = view.cols(); col; col--) { // Loop along the row
        func(*col_acc);  // Apply the functor to this pixel value
        col_acc.next_col();
      }
      row_acc.next_row();
    }
    plane_acc.next_plane();
  }
  progress.report_finished();
}

/// Function to apply a functor to each pixel of an input image.
/// Traverse the image column by column.
template <class ViewT, class FuncT>
void for_each_pixel_columnwise(const vw::ImageViewBase<ViewT> &view_, FuncT &func,
  vw::TerminalProgressCallback const& progress) {

  const ViewT& view = view_.impl();
  typedef typename ViewT::pixel_accessor pixel_accessor;
  pixel_accessor plane_acc = view.origin();

  for (int32 plane = view.planes(); plane; plane--) { // Loop through planes

    pixel_accessor col_acc = plane_acc;
    for (int32 col = 0; col < view.cols(); col++) { // Loop through cols
      progress.report_fractional_progress(col, view.cols());
      pixel_accessor row_acc = col_acc;
      for (int32 row = view.rows(); row; row--) { // Loop along cols
        func(*row_acc);  // Apply the functor to this pixel value
        row_acc.next_row();
      }
      col_acc.next_col();
    }
    plane_acc.next_plane();
  }
  progress.report_finished();

  return;
}

// Compute the min, max, mean, and standard deviation of an image object and
// write them to a log. This is not a member function.
// The "tag" is only used to make the log messages more descriptive.
// If prefix and image_path is set, will cache the results to a file.
// For efficiency, the image must be traversed either rowwise or columnwise,
// depending on how it is stored on disk.
// This makes use of the global variable: asp::stereo_settings().force_reuse_match_files.
// TODO(oalexan1): This function must take into account ISIS special
// pixels from StereoSessionIsis::preprocessing_hook(). Then, must eliminate
// that function in favor of a single preprocessing_hook() in the base class.
vw::Vector<vw::float32,6> 
gather_stats(vw::ImageViewRef<vw::PixelMask<float>> image,
             std::string const& tag,
             std::string const& prefix,
             std::string const& image_path,
             bool force_reuse_cache) {

  vw_out(InfoMessage) << "Computing statistics for " + tag << std::endl;

  Vector6f result;
  const bool use_cache = ((prefix != "") && (image_path != ""));
  std::string cache_path = "";
  if (use_cache) {
    if (image_path.find(prefix) == 0) {
      // If the image is, for example, run/run-L.tif,
      // then cache_path = run/run-L-stats.tif.
      cache_path =  fs::path(image_path).replace_extension("").string() + "-stats.tif";
    } else {
      // If the image is left_image.tif,
      // then cache_path = run/run-left_image.tif
      cache_path = prefix + '-' + fs::path(image_path).stem().string() + "-stats.tif";
    }
  }

  // Check if this stats file was computed after any image modifications.
  if ((use_cache && asp::first_is_newer(cache_path, image_path)) ||
      (force_reuse_cache && fs::exists(cache_path))) {
    vw_out(InfoMessage) << "\t--> Reading statistics from file " + cache_path << std::endl;
    Vector<float32> stats;
    read_vector(stats, cache_path); // Just fetch the stats from the file on disk.
    result = stats;

  } else { // Compute the results

    // Read the resource and determine the block structure on disk. Use a boost shared ptr.
    vw::Vector2i block_size;
    {
      boost::shared_ptr<DiskImageResource> rsrc (DiskImageResourcePtr(image_path));
      block_size  = rsrc->block_read_size();
    }
    // Print a warning that processing can be slow if any of the block size
    // coords are bigger than 5120 pixels.
    if (block_size[0] > 5120 || block_size[1] > 5120) {
      vw_out(WarningMessage) << "Image " << image_path
        << " has block sizes of dimensions " << block_size[0] << " x " << block_size[1]
        << " (as shown by gdalinfo). This can make processing slow. Consider converting "
        << "it to tile format, using the command:\n"
        << "gdal_translate -co TILED=yes -co BLOCKXSIZE=256 -co BLOCKYSIZE=256 "
        << "input.tif output.tif\n";
    }

    // Compute statistics at a reduced resolution
    const float TARGET_NUM_PIXELS = 1000000;
    float num_pixels = float(image.cols())*float(image.rows());
    int   stat_scale = int(ceil(sqrt(num_pixels / TARGET_NUM_PIXELS)));

    vw_out(InfoMessage) << "Using downsample scale: " << stat_scale << std::endl;

    ChannelAccumulator<vw::math::CDFAccumulator<float> > accumulator;
    vw::TerminalProgressCallback tp("asp","\t  stats:  ");
    if (block_size[0] >= block_size[1]) // Rows are long, so go row by row
     for_each_pixel_rowwise(subsample(edge_extend(image, ConstantEdgeExtension()),
                               stat_scale), accumulator, tp);
    else // Columns are long, so go column by column
     for_each_pixel_columnwise(subsample(edge_extend(image, ConstantEdgeExtension()),
                                  stat_scale), accumulator, tp);

    result[0] = accumulator.quantile(0); // Min
    result[1] = accumulator.quantile(1); // Max
    result[2] = accumulator.approximate_mean();
    result[3] = accumulator.approximate_stddev();
    result[4] = accumulator.quantile(0.02); // Percentile values
    result[5] = accumulator.quantile(0.98);

    // Cache the results to disk
    if (use_cache) {
      vw_out() << "\t    Writing stats file: " << cache_path << std::endl;
      Vector<float32> stats = result;  // cast
      write_vector(cache_path, stats);
    }

  } // Done computing the results

  vw_out(InfoMessage) << "\t    " << tag << ": [ lo: " << result[0] << " hi: " << result[1]
                      << " mean: " << result[2] << " std_dev: "  << result[3] << " ]\n";

  return result;
} // end function gather_stats

} // end namespace asp
