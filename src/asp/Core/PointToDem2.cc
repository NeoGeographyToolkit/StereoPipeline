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

#include <asp/Core/PointToDem.h>
#include <asp/Core/PointUtils.h>
#include <asp/Core/GdalUtils.h>

#include <vw/Image/AntiAliasing.h>
#include <vw/Image/Filter.h>
#include <vw/Image/InpaintView.h>

#include <boost/filesystem.hpp>
namespace fs = boost::filesystem;

// Moved here some logic from PointToDem.cc to speed up compilation.

using namespace vw;

namespace asp {

// Write an image to disk while handling some common options.
template<class ImageT>
void save_image(DemOptions & opt, ImageT img, vw::cartography::GeoReference const& georef,
                int hole_fill_len, std::string const& imgName) {

  // When hole-filling is used, we need to look hole_fill_len beyond
  // the current block.  If the block size is 256, and hole fill len
  // is big, like 512 or 1024, we end up processing a huge block
  // only to save a small center block.  For that reason, save
  // temporarily with big blocks, and then re-save with small blocks.
  if (hole_fill_len > 512)
    vw_out(WarningMessage) << "Detected large hole-fill length. "
                            << "Memory usage and run-time may go up.\n";

  int block_size = nextpow2(2.0*hole_fill_len);
  block_size = std::max(256, block_size);

  // Append a tag if desired to compute the min, max, etc. Later on, in OrthoRasterizer
  // we do a full validation of opt.filter.
  std::string tag = "";
  if (opt.filter != "weighted_average")
    tag = "-" + opt.filter;

  std::string output_file = opt.out_prefix + tag + "-" + imgName
    + "." + opt.output_file_type;
  vw_out() << "Writing: " << output_file << "\n";
  TerminalProgressCallback tpc("asp", imgName + ": ");
  bool has_georef = true, has_nodata = true;
  if (opt.output_file_type == "tif")
    asp::save_with_temp_big_blocks(block_size, output_file, img,
                                    has_georef, georef,
                                    has_nodata, opt.nodata_value, opt, tpc);
  else
    vw::cartography::write_gdal_image(output_file, img, georef, opt, tpc);
} // End function save_image

// Round pixels in given image to multiple of given scale.
// Don't round nodata values.
template <class PixelT>
struct RoundImagePixelsSkipNoData: public vw::ReturnFixedType<PixelT> {

  double m_scale, m_nodata;

  RoundImagePixelsSkipNoData(double scale, double nodata):
  m_scale(scale), m_nodata(nodata) {}

  PixelT operator() (PixelT const& pt) const {

    // We will pass in m_scale = 0 if we don't want rounding to happen.
    if (m_scale <= 0)
      return pt;

    // Skip given pixel if any channels are nodata
    int num_channels = PixelNumChannels<PixelT>::value;
    typedef typename CompoundChannelType<PixelT>::type channel_type;
    for (int c = 0; c < num_channels; c++) {
      if ((double)compound_select_channel<channel_type const&>(pt,c) == m_nodata)
        return pt;
    }

    return PixelT(m_scale*round(channel_cast<double>(pt)/m_scale));
  }

};

// If the third component of a vector is NaN, mask that vector as invalid
template<class VectorT>
struct NaN2Mask: public ReturnFixedType<PixelMask<VectorT>> {
  NaN2Mask() {}
  PixelMask<VectorT> operator() (VectorT const& vec) const {
    if (boost::math::isnan(vec.z()))
      return PixelMask<VectorT>(); // invalid
    else
      return PixelMask<VectorT>(vec); // valid
  }
};

// Reverse the operation of NaN2Mask
template<class VectorT>
struct Mask2NaN: public ReturnFixedType<VectorT> {
  Mask2NaN() {}
  VectorT operator() (PixelMask<VectorT> const& pvec) const {
    if (!is_valid(pvec))
      return VectorT(0, 0, std::numeric_limits<typename VectorT::value_type>::quiet_NaN());
    else
      return pvec.child();
  }
};

// If the third component of a vector is NaN, assign to it the given no-data value
struct NaN2NoData: public ReturnFixedType<Vector3> {
  NaN2NoData(float nodata_val):m_nodata_val(nodata_val) {}
  float m_nodata_val;
  Vector3 operator() (Vector3 const& vec) const {
    if (boost::math::isnan(vec.z()))
      return Vector3(m_nodata_val, m_nodata_val, m_nodata_val); // invalid
    else
      return vec; // valid
  }
};

// Take a given point xyz and the error at that point. Convert the
// error to the NED (North-East-Down) coordinate system.
struct ErrorToNED: public ReturnFixedType<Vector3> {
  vw::cartography::GeoReference m_georef;
  ErrorToNED(vw::cartography::GeoReference const& georef):m_georef(georef) {}

  Vector3 operator() (Vector6 const& pt) const {

    Vector3 xyz = subvector(pt, 0, 3);
    if (xyz == Vector3()) return Vector3();

    Vector3   err     = subvector(pt, 3, 3);
    Vector3   llh     = m_georef.datum().cartesian_to_geodetic(xyz);
    Matrix3x3 M       = m_georef.datum().lonlat_to_ned_matrix(llh);
    Vector3   ned_err = inverse(M)*err;
    return ned_err;
  }
};
template <class ImageT>
UnaryPerPixelView<ImageT, ErrorToNED>
inline error_to_NED(ImageViewBase<ImageT> const& image,
                    vw::cartography::GeoReference const& georef) {
  return UnaryPerPixelView<ImageT, ErrorToNED>(image.impl(), ErrorToNED(georef));
}

template <class ImageT>
vw::UnaryPerPixelView<ImageT, RoundImagePixelsSkipNoData<typename ImageT::pixel_type>>
inline round_image_pixels_skip_nodata(vw::ImageViewBase<ImageT> const& image,
                                      double scale, double nodata) {
  return vw::UnaryPerPixelView<ImageT, RoundImagePixelsSkipNoData<typename ImageT::pixel_type>>
    (image.impl(), RoundImagePixelsSkipNoData<typename ImageT::pixel_type>(scale, nodata));
}

// A class for combining the three channels of errors and finding their absolute
// values.
class CombinedAbsView: public ImageViewBase<CombinedAbsView> {
  double m_nodata_value;
  ImageViewRef<PixelGray<float>> m_image1;
  ImageViewRef<PixelGray<float>> m_image2;
  ImageViewRef<PixelGray<float>> m_image3;

public:

  typedef Vector3f pixel_type;
  typedef const Vector3f result_type;
  typedef ProceduralPixelAccessor<CombinedAbsView> pixel_accessor;

  CombinedAbsView(double nodata_value,
                  ImageViewRef<PixelGray<float>> image1,
                  ImageViewRef<PixelGray<float>> image2,
                  ImageViewRef<PixelGray<float>> image3):
    m_nodata_value(nodata_value),
    m_image1(image1),
    m_image2(image2),
    m_image3(image3) {}

  inline int32 cols  () const { return m_image1.cols(); }
  inline int32 rows  () const { return m_image1.rows(); }
  inline int32 planes() const { return 1; }

  inline pixel_accessor origin() const { return pixel_accessor(*this); }

  inline result_type operator()(size_t i, size_t j, size_t p=0) const {

    Vector3f error(m_image1(i, j), m_image2(i, j), m_image3(i, j));

    if (error[0] == m_nodata_value || error[1] == m_nodata_value ||
        error[2] == m_nodata_value) {
      return Vector3f(m_nodata_value, m_nodata_value, m_nodata_value);
    }

    return Vector3f(std::abs(error[0]), std::abs(error[1]), std::abs(error[2]));
  }

  typedef CombinedAbsView prerasterize_type;
  inline prerasterize_type prerasterize(BBox2i const& bbox) const {
    return prerasterize_type(m_nodata_value,
                             m_image1.prerasterize(bbox),
                             m_image2.prerasterize(bbox),
                             m_image3.prerasterize(bbox));
  }
  template <class DestT>
  inline void rasterize(DestT const& dest, BBox2i const& bbox) const {
    vw::rasterize(prerasterize(bbox), dest, bbox);
  }
};
CombinedAbsView combine_abs_channels(double nodata_value,
                                     ImageViewRef<PixelGray<float>> image1,
                                     ImageViewRef<PixelGray<float>> image2,
                                     ImageViewRef<PixelGray<float>> image3) {
  VW_ASSERT(image1.cols() == image2.cols() &&
            image2.cols() == image3.cols() &&
            image1.rows() == image2.rows() &&
            image2.rows() == image3.rows(),
            ArgumentErr() << "Expecting the error channels to have the same size.");

  return CombinedAbsView(nodata_value, image1, image2, image3);
}

// Save the DEM
void save_dem(DemOptions & opt,
              vw::cartography::GeoReference const& georef,
              asp::OrthoRasterizerView& rasterizer,
              Vector2 const& tile_size,
              std::int64_t * num_invalid_pixels) {

  // The value stored in num_invalid_pixels will get updated as the DEM is being
  // written to disk. That because OrthoRasterizerView has a pointer to it.
  // This must be reset before each use.
  // TODO(oalexan1): This logic is confusing. Better have two member functions that
  // first reset and later get this number.
  *num_invalid_pixels = 0;

  // We use the existing texture channel, which is the height.
  Stopwatch sw2;
  sw2.start();
  ImageViewRef<PixelGray<float>> dem
    = asp::round_image_pixels_skip_nodata(rasterizer.impl(), opt.rounding_error,
                                          opt.nodata_value);

  int hole_fill_len = opt.dem_hole_fill_len;
  if (hole_fill_len > 0) {
    // Note that we first cache the tiles of the rasterized DEM, and
    // fill holes later. This greatly improves the performance.
    dem = apply_mask
      (vw::fill_holes_grass(create_mask
                              (block_cache(dem, tile_size, opt.num_threads),
                              opt.nodata_value),
                              hole_fill_len),
        opt.nodata_value);
  }

  // Stop the program if it is going to create too large a DEM, this will
  // cause a crash.
  Vector2i dem_size = bounding_box(dem).size();
  vw_out()<< "Creating output file that is " << dem_size << " px.\n";
  if ((dem_size[0] > opt.max_output_size[0]) || (dem_size[1] > opt.max_output_size[1]))
    vw_throw(ArgumentErr()
              << "Requested DEM size is too large, max allowed output size is "
              << opt.max_output_size << " pixels.\n");

  asp::save_image(opt, dem, georef, hole_fill_len, "DEM");
  sw2.stop();
  vw_out(DebugMessage,"asp") << "DEM render time: " << sw2.elapsed_seconds() << ".\n";

  // num_invalid_pixels was updated as the DEM was written.
  double num_invalid_pixelsD = *num_invalid_pixels;

  // Below we convert to double first and multiply later, to avoid
  // 32-bit integer overflow.
  double num_total_pixels = double(dem_size[0]) * double(dem_size[1]);

  double invalid_ratio = num_invalid_pixelsD / num_total_pixels;
  vw_out() << "Percentage of valid pixels: "
            << 100.0*(1.0 - invalid_ratio) << "%\n";

  // Wipe after use. This will reset the counter in OrthoRasterizerView.
  *num_invalid_pixels = 0;
}

// Save the intersection error
void save_intersection_error(DemOptions & opt,
                             bool has_stddev,
                             vw::cartography::GeoReference const& georef,
                             Vector2 const& tile_size,
                             asp::OrthoRasterizerView& rasterizer) {

  int hole_fill_len = 0;
  int num_channels = asp::num_channels(opt.pointcloud_files);

  if (num_channels == 4 || (num_channels == 6 && has_stddev) || opt.scalar_error) {
    // The error is a scalar (4 channels or 6 channels but last two are stddev),
    // or we want to find the norm of the error.
    ImageViewRef<double> error_channel = asp::point_cloud_error_image(opt.pointcloud_files);
    rasterizer.set_texture(error_channel);
    save_image(opt, asp::round_image_pixels_skip_nodata(rasterizer.impl(),
                                                        opt.rounding_error,
                                                        opt.nodata_value),
                georef, hole_fill_len, "IntersectionErr");
  } else if (num_channels == 6) {
    // The error is a 3D vector. Convert it to NED coordinate system, and rasterize it.
    ImageViewRef<Vector6> point_disk_image = asp::form_point_cloud_composite<Vector6>
      (opt.pointcloud_files, ASP_MAX_SUBBLOCK_SIZE);
    ImageViewRef<Vector3> ned_err = asp::error_to_NED(point_disk_image, georef);
    std::vector<ImageViewRef<PixelGray<float>>>  rasterized(3);
    for (int ch_index = 0; ch_index < 3; ch_index++) {
      ImageViewRef<double> ch = select_channel(ned_err, ch_index);
      rasterizer.set_texture(ch);
      rasterized[ch_index] =
        block_cache(rasterizer.impl(), tile_size, opt.num_threads);
    }
    auto err_vec = asp::combine_abs_channels(opt.nodata_value, rasterized[0],
                                            rasterized[1], rasterized[2]);
    save_image(opt, asp::round_image_pixels_skip_nodata(err_vec,
                              opt.rounding_error, opt.nodata_value),
                georef, hole_fill_len, "IntersectionErr");
  } else {
    // Note: We don't throw. We still would like to write the
    // DRG (later) even if we can't write the error image.
    vw_out() << "The point cloud files must have an equal number of channels which "
              << "must be 4 or 6 to be able to process the intersection error.\n";
  }

}

// Save horizontal and vertical stddev
void save_stddev(DemOptions & opt,
                 vw::cartography::GeoReference const& georef,
                 asp::OrthoRasterizerView& rasterizer) {

  int num_channels = asp::num_channels(opt.pointcloud_files);

  double rounding_error = 0.0;
  vw_out() << "Not rounding propagated errors (option: --rounding-error) to avoid "
            << "introducing step artifacts.\n";

  // Note: We don't throw here. We still would like to write the
  // DRG (later) even if we can't write the stddev.
  if (num_channels != 6) {
    vw_out() << "The input point cloud(s) must have 6 channels to be able to "
              << "grid the horizontal and vertical stddev.\n";
  } else {
    int hole_fill_len = 0;
    ImageViewRef<Vector6> point_disk_image = asp::form_point_cloud_composite<Vector6>
      (opt.pointcloud_files, ASP_MAX_SUBBLOCK_SIZE);

    ImageViewRef<double> horizontal_stddev_channel = select_channel(point_disk_image, 4);
    rasterizer.set_texture(horizontal_stddev_channel);
    save_image(opt, asp::round_image_pixels_skip_nodata(rasterizer.impl(),
                                                        rounding_error, // local value
                                                        opt.nodata_value),
                georef, hole_fill_len, "HorizontalStdDev");

    ImageViewRef<double> vertical_stddev_channel = select_channel(point_disk_image, 5);
    rasterizer.set_texture(vertical_stddev_channel);
    save_image(opt, asp::round_image_pixels_skip_nodata(rasterizer.impl(),
                                                        rounding_error, // local value
                                                        opt.nodata_value),
                georef, hole_fill_len, "VerticalStdDev");
  }

}

// Save the orthoimage. The texture comes form L.tif instead of the heights.
void save_ortho(DemOptions & opt,
                vw::cartography::GeoReference const& georef,
                asp::OrthoRasterizerView& rasterizer) {

  Stopwatch sw3;
  sw3.start();

  // Set the texture channel
  ImageViewRef<PixelGray<float>> texture
    = asp::form_point_cloud_composite<PixelGray<float>>
    (opt.texture_files, ASP_MAX_SUBBLOCK_SIZE);
  rasterizer.set_texture(texture);

  if (opt.ortho_hole_fill_len > 0) {

    // Need to convert the hole fill length from output image pixels
    // to input point cloud pixels as we will fill holes in the cloud
    // itself.
    int hole_fill_len = rasterizer.pc_hole_fill_len(opt.ortho_hole_fill_len);

    // Fetch a reference to the current point image
    ImageViewRef<Vector3> point_image = rasterizer.get_point_image();

    // Mask the NaNs
    ImageViewRef<PixelMask<Vector3>> point_image_mask
      = per_pixel_filter(point_image, asp::NaN2Mask<Vector3>());

    // If to grow the cloud a bit, to help hole-filling later. This should
    // not be large as it creates artifacts. The main work better
    // be done by grassfire later.
    if (opt.ortho_hole_fill_extra_len > 0) {
      int hole_fill_mode = 2;
      int hole_fill_num_smooth_iter = 3;
      int hole_fill_extra_len
        = rasterizer.pc_hole_fill_len(opt.ortho_hole_fill_extra_len);
      point_image_mask = vw::fill_holes(point_image_mask, hole_fill_mode,
                                        hole_fill_num_smooth_iter,
                                        hole_fill_extra_len);

      // Use big tiles, to reduce the overhead
      // of expanding each tile by hole size.
      int big_block_size = nextpow2(2.0*hole_fill_extra_len);
      big_block_size = std::max(256, big_block_size);

      // Cache each hole-filled point cloud tile as likely we will
      // need it again in the future when rasterizing a different
      // portion of the output ortho image.
      point_image_mask = block_cache(point_image_mask,
                                      vw::Vector2(big_block_size, big_block_size),
                                      opt.num_threads);
    }

    // Fill the holes
    point_image_mask =vw::fill_holes_grass(point_image_mask, hole_fill_len);

    // back to NaNs
    point_image = per_pixel_filter(point_image_mask, asp::Mask2NaN<Vector3>());

    // When filling holes, use big tiles, to reduce the overhead
    // of expanding each tile by hole size.
    int big_block_size = nextpow2(2.0*hole_fill_len);
    big_block_size = std::max(256, big_block_size);

    // Cache each hole-filled point cloud tile as likely we will
    // need it again in the future when rasterizing a different
    // portion of the output ortho image.
    point_image = block_cache(point_image,
                              vw::Vector2(big_block_size, big_block_size),
                              opt.num_threads);

    // Pass to the rasterizer the point image with the holes filled
    rasterizer.set_point_image(point_image);
  }

  asp::save_image(opt, rasterizer.impl(), georef,
                  0, // no need for a buffer here, as we cache hole-filled tiles
                  "DRG");
  sw3.stop();
  vw_out(DebugMessage,"asp") << "DRG render time: " << sw3.elapsed_seconds() << "\n";
}

// Rasterize a DEM, and perhaps the error image, orthoimage, stddev, etc.
// This may be called several times, with different grid sizes.
void rasterize_cloud(asp::OrthoRasterizerView& rasterizer,
                               DemOptions& opt,
                               vw::cartography::GeoReference& georef,
                               std::int64_t * num_invalid_pixels) {

  vw_out() << "\tStarting DEM rasterization\n";
  vw_out() << "\t--> DEM spacing: " <<     rasterizer.spacing() << " pt/px\n";
  vw_out() << "\t             or: " << 1.0/rasterizer.spacing() << " px/pt\n";

  // TODO: Maybe put a warning or check here if the size is too big

  // The affine transform. If the user specified --t_projwin, the transform
  // already incorporated that.
  georef.set_transform(rasterizer.geo_transform());

  // Fix have pixel offset required if pixel_interpretation is
  // PixelAsArea. We could have done that earlier, but it makes
  // the above easier to not think about it.
  if (georef.pixel_interpretation() == vw::cartography::GeoReference::PixelAsArea) {
    Matrix3x3 transform = georef.transform();
    transform(0,2) -= 0.5 * transform(0,0);
    transform(1,2) -= 0.5 * transform(1,1);
    georef.set_transform(transform);
  }

  // Do not round the DEM heights for small bodies
  if (georef.datum().semi_major_axis() <= asp::MIN_RADIUS_FOR_ROUNDING ||
      georef.datum().semi_minor_axis() <= asp::MIN_RADIUS_FOR_ROUNDING) {
    opt.rounding_error = 0.0;
  }

  Vector2 tile_size(vw_settings().default_tile_size(),
                    vw_settings().default_tile_size());

  // Write out the DEM. We've set the texture to be the height.
  // This must happen before we set the texture to something else.
  if (!opt.no_dem)
     save_dem(opt, georef, rasterizer, tile_size, num_invalid_pixels);

  // If the point cloud has propagated stddev affects how we write the error image.
  bool has_stddev = asp::has_stddev(opt.pointcloud_files);

  // Write triangulation error image if requested
  if (opt.do_error)
    save_intersection_error(opt, has_stddev, georef, tile_size, rasterizer);

  if (opt.propagate_errors && !has_stddev) {
    // Do not throw an error. Go on and save at least the intersection
    // error and orthoimage.
    vw_out() << "Cannot grid the horizontal and vertical stddev as the point "
             << "cloud file is not in the expected format.\n";
    opt.propagate_errors = false;
  }

  if (opt.propagate_errors)
    save_stddev(opt, georef, rasterizer);

  // Write out a normalized version of the DEM, if requested (for debugging).
  // Here the DEM is read back and written normalized to a new file.
  if (opt.do_normalize) {
    int hole_fill_len = 0;
    DiskImageView<PixelGray<float>> dem_image(opt.out_prefix + "-DEM." + opt.output_file_type);
    asp::save_image(opt, apply_mask(channel_cast<uint8>
                                    (normalize(create_mask(dem_image, opt.nodata_value),
                                               rasterizer.bounding_box().min().z(),
                                               rasterizer.bounding_box().max().z(),
                                               0, 255))),
                    georef, hole_fill_len, "DEM-normalized");
  }

  // Write DRG if the user requested and provided a texture file.
  // This must be at the end, as we may be messing with the point
  // image in irreversible ways.
  if (opt.do_ortho)
   save_ortho(opt, georef, rasterizer);

} // End rasterize_cloud

} // end namespace asp
