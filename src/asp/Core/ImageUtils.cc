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
#include <vw/FileIO/MatrixIO.h>

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

/// Create a masked DEM
void create_masked_dem(std::string const& dem_file,
                       vw::cartography::GeoReference & dem_georef,
                       vw::ImageViewRef<vw::PixelMask<double>> & masked_dem) {
  
  vw_out() << "Loading DEM: " << dem_file << std::endl;

  // Read the no-data
  double nodata_val = -std::numeric_limits<float>::max(); // note we use a float nodata
  if (vw::read_nodata_val(dem_file, nodata_val))
    vw_out() << "Found DEM nodata value: " << nodata_val << std::endl;

  // Create the interpolated DEM. Values out of bounds will be invalid.
  vw::PixelMask<double> invalid_val;
  invalid_val[0] = nodata_val;
  invalid_val.invalidate();
  masked_dem = create_mask(DiskImageView<double>(dem_file), nodata_val);

  // Read the georef. It must exist.
  bool is_good = vw::cartography::read_georeference(dem_georef, dem_file);
  if (!is_good) {
    vw_throw(ArgumentErr() << "Error: Cannot read a georeference from DEM: "
             << dem_file << ".\n");
  }
}

/// Create a DEM ready to use for interpolation
void create_interp_dem(std::string const& dem_file,
                       vw::cartography::GeoReference & dem_georef,
                       ImageViewRef<PixelMask<double>> & interp_dem) {
  
  vw::ImageViewRef<vw::PixelMask<double>> masked_dem;
  asp::create_masked_dem(dem_file, dem_georef, masked_dem);

  vw::PixelMask<double> invalid_val;
  interp_dem = interpolate(masked_dem, BilinearInterpolation(), 
                           vw::ValueEdgeExtension<vw::PixelMask<float>>(invalid_val));

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
// write them to a log. If prefix and image_path is set, will cache the results
// to a file. For efficiency, the image must be traversed either rowwise or
// columnwise, depending on how it is stored on disk.
// TODO(oalexan1): This function must take into account ISIS special
// pixels from StereoSessionIsis::preprocessing_hook(). Then, must eliminate
// that function in favor of a single preprocessing_hook() in the base class.
vw::Vector<vw::float32,6>
gather_stats(vw::ImageViewRef<vw::PixelMask<float>> image,
             std::string const& out_prefix,
             std::string const& image_path,
             bool force_reuse_cache,
             bool adjust_min_max_with_std) {
  
  vw_out(InfoMessage) << "Computing statistics for " + image_path << "\n";

  Vector6f result;
  const bool use_cache = ((out_prefix != "") && (image_path != ""));
  std::string stats_file = "";
  if (use_cache) {
    if (image_path.find(out_prefix) == 0) {
      // If the image is, for example, run/run-L.tif,
      // then stats_file = run/run-L-stats.tif.
      stats_file = fs::path(image_path).replace_extension("").string() + "-stats.tif";
    } else {
      // If the image is left_image.tif,
      // then stats_file = run/run-left_image-stats.tif
      stats_file = out_prefix + '-' + fs::path(image_path).stem().string() + "-stats.tif";
    }
  }

  // Check if this stats file was computed after any image modifications.
  if ((use_cache && asp::first_is_newer(stats_file, image_path)) ||
      (force_reuse_cache && fs::exists(stats_file))) {
    vw_out(InfoMessage) << "\t--> Reading statistics from file " + stats_file << std::endl;
    Vector<float32> stats;
    read_vector(stats, stats_file); // Just fetch the stats from the file on disk.
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

    // Adjust lo/hi to be within 2 standard deviations of the mean, but do not
    // exceed min and max. Thi is needed for ISIS which has special pixels. May
    // become the default eventually or all uses as there is no point in
    // normalizing to a range that is much wider than the actual pixel values.
    if (adjust_min_max_with_std) {
      vw_out(InfoMessage) << "\t--> Adjusting hi and lo to -+2 sigmas around mean.\n";
      float lo   = result[0];
      float hi   = result[1];
      float mean = result[2];
      float std  = result[3];
      if (lo < mean - 2*std)
        lo = mean - 2*std;
      if (hi > mean + 2*std)
        hi = mean + 2*std;
      result[0] = lo;
      result[1] = hi;
    }

    // Cache the results to disk
    if (use_cache) {
      vw_out() << "\t    Writing stats file: " << stats_file << std::endl;
      Vector<float32> stats = result;  // cast
      write_vector(stats_file, stats);
    }

  } // Done computing the results

  vw_out(InfoMessage) << "\t: [ lo: " << result[0] << " hi: " << result[1]
                      << " mean: " << result[2] << " std_dev: "  << result[3] << " ]\n";

  return result;
} // end function gather_stats

// Checks if the given image file has an 8-bit channels
bool hasByteChannels(const std::string& image_path) {

  boost::shared_ptr<vw::DiskImageResource> rsrc = vw::DiskImageResourcePtr(image_path);
  auto pixel_format = rsrc->pixel_format();
  auto channel_type = rsrc->channel_type();
  return (channel_type == vw::VW_CHANNEL_UINT8) && 
         (pixel_format == vw::VW_PIXEL_GRAY   ||
          pixel_format == vw::VW_PIXEL_RGB    ||
          pixel_format == vw::VW_PIXEL_RGBA   ||
          pixel_format == vw::VW_PIXEL_GRAYA);
}

// Expand a box by a given percentage (typically pct is between 0 and 100)
void expand_box_by_pct(vw::BBox2 & box, double pct) {
  
  // Check the pct is non-negative
  if (pct < 0.0) 
    vw_throw(ArgumentErr() << "Invalid percentage when expanding a box: " 
              << pct << ".\n");
    
  double factor = pct / 100.0;
  double half_extra_x = 0.5 * box.width()  * factor;
  double half_extra_y = 0.5 * box.height() * factor;
  box.min() -= Vector2(half_extra_x, half_extra_y);
  box.max() += Vector2(half_extra_x, half_extra_y);
}

} // end namespace asp
