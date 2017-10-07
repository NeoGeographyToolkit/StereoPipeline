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

/// \file dem_mosaic.cc
///

// A tool to mosaic and blend DEMs, and output the mosaic as tiles.

// Note 1: In practice, the tool may be more efficient if the entire
// mosaic is written out as one single large image, rather than being
// broken up into tiles. To achieve that, just specify to the tool a
// very large tile size, and use 0 for the tile index in the command
// line options.

// Note 2: The tool can be high on memory usage, so processes for
// individual tiles may need to be run on separate machines.

#include <iostream>
#include <fstream>
#include <iomanip>
#include <string>
#include <vector>
#include <sys/types.h>
#include <sys/stat.h>
#include <unistd.h>
#include <time.h>
#include <limits>
#include <algorithm>

#include <vw/FileIO.h>
#include <vw/Image.h>
#include <vw/Cartography.h>
#include <vw/Math.h>
#include <vw/FileIO/DiskImageManager.h>
#include <vw/Image/InpaintView.h>
#include <asp/Core/Macros.h>
#include <asp/Core/Common.h>


#include <boost/math/special_functions/fpclassify.hpp>
#include <boost/math/special_functions/erf.hpp>
#include <boost/program_options.hpp>

#include <boost/filesystem/convenience.hpp>

using namespace std;
using namespace vw;
using namespace vw::cartography;
namespace po = boost::program_options;
namespace fs = boost::filesystem;

// This tool casts all input DEMs to float. The processing is done in double
// precision though. 
typedef float RealT;

// This is used for various tolerances
double g_tol = 1e-6;

// An S-shaped function. Value at 0 is 0. Value at M is M.
// Flat before 0 and after M. Higher value of L means
// more flatness at the ends, but higher growth
// in the middle.
double S_shape(double x, double M, double L){
  if (x <= 0) return 0;
  if (x >= M) return M;
  return 0.5*M*( 1 + boost::math::erf (0.5*sqrt(M_PI) * (2*x*L/M - L) ) );
}

// Function for highlighting spots of data
template<class PixelT>
class NotNoDataFunctor {
  typedef typename CompoundChannelType<PixelT>::type channel_type;
  channel_type m_nodata;
  typedef ChannelRange<channel_type> range_type;
public:
  NotNoDataFunctor( channel_type nodata ) : m_nodata(nodata) {}

  template <class Args> struct result {
    typedef channel_type type;
  };

  inline channel_type operator()( channel_type const& val ) const {
    return (val != m_nodata && !isnan(val))? range_type::max() : range_type::min();
  }
};

template <class ImageT, class NoDataT>
UnaryPerPixelView<ImageT,UnaryCompoundFunctor<NotNoDataFunctor<typename ImageT::pixel_type>, typename ImageT::pixel_type>  >
inline notnodata( ImageViewBase<ImageT> const& image, NoDataT nodata ) {
  typedef UnaryCompoundFunctor<NotNoDataFunctor<typename ImageT::pixel_type>, typename ImageT::pixel_type> func_type;
  func_type func( nodata );
  return UnaryPerPixelView<ImageT,func_type>( image.impl(), func );
}

// Set nodata pixels to 0 and valid data pixels to something big.
template<class PixelT>
struct BigOrZero: public ReturnFixedType<PixelT> {
  PixelT m_nodata;
  BigOrZero(PixelT nodata):m_nodata(nodata){}
  double operator() (PixelT const& pix) const {
    if (pix != m_nodata && !isnan(pix)) return 1e+8;
    return 0;
  }
};

template<class ImageT>
ImageView<double> compute_weights(ImageT const& img, bool use_centerline_weights){
  if (use_centerline_weights) {
    ImageView<double> result;
    weights_from_centerline(img, result);
    return result;
  }

  return grassfire(img);
}

void blur_weights(ImageView<double> & weights, double sigma){

  if (sigma <= 0)
    return;

  // Blur the weights. To try to make the weights not drop much at the
  // boundary, expand the weights with zero, blur, crop back to the
  // original region.

  // It is highly important to note that blurring can increase the weights
  // at the boundary, even with the extension done above. Erosion before
  // blurring does not help with that, as for weights with complicated
  // boundary erosion can wipe things in a non-uniform way leaving
  // huge holes. To get smooth weights, if really desired one should
  // use the weights-exponent option.

  int half_kernel = vw::compute_kernel_size(sigma)/2;
  int extra = half_kernel + 1; // to guarantee we stay zero at boundary

  int cols = weights.cols(), rows = weights.rows();

  ImageView<double> extra_wts(cols + 2*extra, rows + 2*extra);
  fill(extra_wts, 0);
  for (int col = 0; col < cols; col++) {
    for (int row = 0; row < rows; row++) {
      extra_wts(col + extra, row + extra) = weights(col, row);
    }
  }

  ImageView<double> blurred_wts = gaussian_filter(extra_wts, sigma);

  // Copy back.  The weights must not grow. In particular, where the
  // original weights were zero, the new weights must also be zero, as
  // at those points there is no DEM data.
  for (int col = 0; col < cols; col++) {
    for (int row = 0; row < rows; row++) {
      if (weights(col, row) > 0) {
        weights(col, row) = blurred_wts(col + extra, row + extra);
      }
      //weights(col, row) = std::min(weights(col, row), blurred_wts(col + extra, row + extra));
    }
  }

}

BBox2 custom_point_to_pixel_bbox(GeoReference const& georef, BBox2 const& ptbox){

  // Given the corners in the projected space, find the pixel corners.
  // If the biggest pixel value is say 42, the image must have 43
  // pixels.  And if the biggest pixel value is 42.1 or 42.9, the
  // image must have 44 pixels.  That's why below we use
  // ceil(pix_box.max() + Vector2(1, 1)).

  // This differs a bit from the point_to_pixel_bbox() function in
  // GeoReferenceBase.cc.

  // TODO: Maybe that function can be made to imitate this one. 

  BBox2 pix_box;
  Vector2 cr[] = {ptbox.min(), ptbox.max(),
		  Vector2(ptbox.min().x(), ptbox.max().y()),
		  Vector2(ptbox.max().x(), ptbox.min().y())};
  for (int icr = 0; icr < (int)(sizeof(cr)/sizeof(Vector2)); icr++)
    pix_box.grow( georef.point_to_pixel(cr[icr]) );
  
  // If the corner is actually very close to an integer number, we
  // assume it should in fact be integer but got moved a bit due to
  // numerical error. Then we set it to integer. This ensures that
  // when we mosaic a single DEM we get its corners to be the same as
  // the originals rather than moved by a slight offset.
  if (norm_2(pix_box.max() - round(pix_box.max())) < g_tol ) 
    pix_box.max() = round(pix_box.max());

  pix_box.max() = ceil( pix_box.max() + Vector2(1, 1));
  
  return pix_box;
}

GeoReference read_georef(std::string const& file){
  // Read a georef, and check for success
  GeoReference geo;
  bool is_good = read_georeference(geo, file);
  if (!is_good)
    vw_throw(ArgumentErr() << "No georeference found in " << file << ".\n");
  return geo;
}

std::string processed_proj4(std::string const& srs){
  // Apparently functionally identical proj4 strings can differ in
  // subtle ways, such as an extra space, etc. For that reason, must
  // parse and process any srs string before comparing it with another string.
  GeoReference georef;
  bool have_user_datum = false;
  Datum user_datum;
  asp::set_srs_string(srs, have_user_datum, user_datum, georef);
  return georef.overall_proj4_str();
}

struct Options : vw::cartography::GdalWriteOptions {
  string dem_list_file, out_prefix, target_srs_string, tile_list_str, this_dem_as_reference;
  vector<string> dem_files;
  double tr, geo_tile_size;
  bool   has_out_nodata;
  double out_nodata_value;
  int    tile_size, tile_index, erode_len, priority_blending_len, extra_crop_len, hole_fill_len, block_size, save_dem_weight;
  double  weights_exp, weights_blur_sigma, dem_blur_sigma;
  double nodata_threshold;
  bool   first, last, min, max, block_max, mean, stddev, median, count, save_index_map, use_centerline_weights, first_dem_as_reference;
  std::set<int> tile_list;
  BBox2 projwin;
  Options(): tr(0), geo_tile_size(0), has_out_nodata(false), tile_index(-1),
	     erode_len(0), priority_blending_len(0), extra_crop_len(0),
	     hole_fill_len(0), block_size(0), save_dem_weight(-1), 
	     weights_exp(0), weights_blur_sigma(0.0), dem_blur_sigma(0.0),
	     nodata_threshold(std::numeric_limits<double>::quiet_NaN()),
	     first(false), last(false), min(false), max(false), block_max(false),
	     mean(false), stddev(false), median(false), count(false), save_index_map(false),
	     use_centerline_weights(false), first_dem_as_reference(false), projwin(BBox2()) {}
};

/// Return the number of no-blending options selected.
int no_blend(Options const& opt){
  return int(opt.first) + int(opt.last) + int(opt.min) + int(opt.max)
    + int(opt.mean) + int(opt.stddev) + int(opt.median) + int(opt.count) + int(opt.block_max);
}

std::string tile_suffix(Options const& opt){
  std::string ans;
  if (opt.first    ) ans = "-first";
  if (opt.last     ) ans = "-last";
  if (opt.min      ) ans = "-min";
  if (opt.max      ) ans = "-max";
  if (opt.block_max) ans = "-block-max";
  if (opt.mean     ) ans = "-mean";
  if (opt.stddev   ) ans = "-stddev";
  if (opt.median   ) ans = "-median";
  if (opt.count    ) ans = "-count";
  if (opt.save_index_map)       ans += "-index-map";
  if (opt.save_dem_weight >= 0) ans += "-weight-dem-index-" + stringify(opt.save_dem_weight);

  return ans;
}

/// Class that does the actual image processing work
class DemMosaicView: public ImageViewBase<DemMosaicView>{
  int m_cols, m_rows, m_bias;
  Options                 const& m_opt;              // alias
  DiskImageManager<RealT>      & m_imgMgr;           // alias
  vector<GeoReference>    const& m_georefs;          // alias
  GeoReference                   m_out_georef;
  vector<double>          const& m_nodata_values;    // alias
  vector<BBox2i>          const& m_dem_pixel_bboxes; // alias
  long long int                & m_num_valid_pixels; // alias, to populate on output
  vw::Mutex                    & m_count_mutex;      // alias, a lock for m_num_valid_pixels

public:
  DemMosaicView(int cols, int rows, int bias,
                Options                const& opt,
                DiskImageManager<RealT>     & imgMgr,
                vector<GeoReference>   const& georefs,
                GeoReference           const& out_georef,
                vector<double>         const& nodata_values,
                vector<BBox2i>         const& dem_pixel_bboxes,
                long long int               & num_valid_pixels,
                vw::Mutex                   & count_mutex):
    m_cols(cols), m_rows(rows), m_bias(bias), m_opt(opt),
    m_imgMgr(imgMgr), m_georefs(georefs),
    m_out_georef(out_georef), m_nodata_values(nodata_values),
    m_dem_pixel_bboxes(dem_pixel_bboxes), m_num_valid_pixels(num_valid_pixels),
    m_count_mutex(count_mutex) {

    // How many valid pixels we will have
    m_num_valid_pixels = 0;
    
    if (imgMgr.size() != georefs.size()       ||
        imgMgr.size() != nodata_values.size() ||
        imgMgr.size() != dem_pixel_bboxes.size())
      vw_throw(ArgumentErr() << "Inputs expected to have the same size do not.\n");

    // Sanity check, see if datums differ, then the tool won't work
    const double out_major_axis = m_out_georef.datum().semi_major_axis();
    const double out_minor_axis = m_out_georef.datum().semi_minor_axis();
    for (int i = 0; i < (int)m_georefs.size(); i++) {
      double this_major_axis = m_georefs[i].datum().semi_major_axis();
      double this_minor_axis = m_georefs[i].datum().semi_minor_axis();
      if (std::abs(this_major_axis - out_major_axis) > 0.1 || 
          std::abs(this_minor_axis - out_minor_axis) > 0.1 ||
          m_georefs[i].datum().meridian_offset()
          != m_out_georef.datum().meridian_offset() ){
        vw_throw(NoImplErr() << "Mosaicking of DEMs with differing datum radii "
                 << " or meridian offsets is not implemented. Datums encountered:\n"
                 << m_georefs[i].datum() << "\n"
                 <<  m_out_georef.datum() << "\n");
      }
      if (m_georefs[i].datum().name() != m_out_georef.datum().name() &&
          this_major_axis == out_major_axis &&
          this_minor_axis == out_minor_axis &&
          m_georefs[i].datum().meridian_offset() == m_out_georef.datum().meridian_offset() ){
        vw_out(WarningMessage) << "Found DEMs with the same radii and meridian offsets, "
                               << "but different names: "
                               << m_georefs[i].datum().name() << " and "
                               << m_out_georef.datum().name() << "\n";
      }
    }
  }

  // Boilerplate
  typedef RealT      pixel_type;
  typedef pixel_type result_type;
  typedef ProceduralPixelAccessor<DemMosaicView> pixel_accessor;
  inline int cols  () const { return m_cols; }
  inline int rows  () const { return m_rows; }
  inline int planes() const { return 1; }
  inline pixel_accessor origin() const { return pixel_accessor( *this, 0, 0 ); }

  inline pixel_type operator()( double/*i*/, double/*j*/, int/*p*/ = 0 ) const {
    vw_throw(NoImplErr() << "DemMosaicView::operator()(...) is not implemented");
    return pixel_type();
  }

  typedef CropView<ImageView<pixel_type> > prerasterize_type;
  inline prerasterize_type prerasterize(BBox2i bbox) const {

    BBox2i orig_box = bbox;
    
    // When doing priority blending, we will do all the work in the
    // output pixels domain. Hence we need to take into account the
    // bias here rather than later.
    if (m_opt.priority_blending_len > 0)
      bbox.expand(m_bias + BilinearInterpolation::pixel_buffer + 1);

    // We will do all computations in double precision, regardless
    // of the precision of the inputs, for increased accuracy.
    // - The image data buffers are initialized here
    typedef PixelGrayA<double> DoubleGrayA;
    ImageView<double> tile   (bbox.width(), bbox.height()); // the output tile (in most cases)
    ImageView<double> weights(bbox.width(), bbox.height()); // accumulated weights (in most cases)
    fill( tile, m_opt.out_nodata_value );
    fill( weights, 0.0 );

    // True if we won't be doing any DEM blending.
    bool noblend = (no_blend(m_opt) > 0);

    // A vector of images the size of the output tile.
    // - Used for median and stddev calculation.
    std::vector< ImageView<double> > tile_vec, weight_vec;
    std::vector< std::string > dem_vec;
    if (m_opt.median) // Store each input separately
      tile_vec.reserve(m_imgMgr.size());
    if (m_opt.stddev) { // Need one working image
      tile_vec.push_back(ImageView<double>(bbox.width(), bbox.height()));
      // Each pixel starts at zero, nodata is handled later
      fill( tile_vec[0], 0.0 );
      fill( tile,        0.0 );
    }
    if (m_opt.priority_blending_len > 0) { // Store each weight separately
      tile_vec.reserve  (m_imgMgr.size());
      weight_vec.reserve(m_imgMgr.size());
    }

    // This will ensure that pixels from earlier images are
    // mostly used unmodified except being blended at the boundary.
    ImageView<double> weight_modifier;
    if (m_opt.priority_blending_len > 0) {
      weight_modifier = ImageView<double>(bbox.width(), bbox.height());
      fill(weight_modifier, std::numeric_limits<double>::max());
    }

    // For saving the weights
    std::vector<int> clip2dem_index;
    ImageView<double> saved_weight;
    if (m_opt.save_dem_weight >= 0) {
      saved_weight = ImageView<double>(bbox.width(), bbox.height());
      fill(saved_weight, 0.0);
    }

    // For saving the index map
    ImageView<double> index_map;
    if (m_opt.save_index_map) {
      index_map = ImageView<double>(bbox.width(), bbox.height());
      fill(index_map, m_opt.out_nodata_value);
    }

    ImageView<double> first_dem;
    
    // Loop through all input DEMs
    for (int dem_iter = 0; dem_iter < (int)m_imgMgr.size(); dem_iter++){

      // Load the information for this DEM
      GeoReference georef        = m_georefs         [dem_iter];
      BBox2i       dem_pixel_box = m_dem_pixel_bboxes[dem_iter];
      
      // The GeoTransform will hide the messy details of conversions
      // from pixels to points and lon-lat.
      GeoTransform geotrans(georef, m_out_georef, dem_pixel_box, bbox);

      // Get the tile bbox in the frame of the current input DEM
      BBox2 in_box = geotrans.reverse_bbox(bbox);

      // Grow to account for blending and erosion length, etc.  If
      // priority blending length was positive, we've already done
      // that.
      if (m_opt.priority_blending_len <= 0)
        in_box.expand(m_bias + BilinearInterpolation::pixel_buffer + 1);

      in_box.crop(dem_pixel_box);
      if (in_box.width() == 1 || in_box.height() == 1){
        // Grassfire likes to have width of at least 2
        in_box.expand(1);
        in_box.crop(dem_pixel_box);
      }
      if (in_box.width() <= 1 || in_box.height() <= 1)
        continue; // No overlap with this tile, skip to the next DEM.

      if (m_opt.median || m_opt.priority_blending_len > 0 || m_opt.block_max){
        // Must use a blank tile each time
        fill( tile, m_opt.out_nodata_value );
        fill( weights, 0.0 );
      }

      // Crop the disk dem to a 2-channel in-memory image. First
      // channel is the image pixels, second will be the weights.
      ImageViewRef<double     > disk_dem = pixel_cast<double>(m_imgMgr.get_handle(dem_iter, bbox));
      ImageView   <DoubleGrayA> dem      = crop(disk_dem, in_box);

      if (m_opt.first_dem_as_reference && dem_iter == 0) {
        // We need to keep the first DEM, to use it as ref
        // when merging in the blended DEM
        first_dem = crop(disk_dem, bbox);
      }
      
      std::string dem_name = m_imgMgr.get_file_name(dem_iter);
      
      // If the nodata_threshold is specified, all values no more than this
      // will be invalidated.
      double nodata_value = m_nodata_values[dem_iter];
      if (!boost::math::isnan(m_opt.nodata_threshold)) {
        nodata_value = m_opt.nodata_threshold;
        for (int col = 0; col < dem.cols(); col++) {
          for (int row = 0; row < dem.rows(); row++) {
            if (dem(col, row)[0] <= nodata_value) {
              dem(col, row)[0] = nodata_value;
            }
          }
        }
      }

      if (m_opt.first_dem_as_reference && dem_iter == 0) {
        // Convert to the output nodata value
        for (int col = 0; col < first_dem.cols(); col++) {
          for (int row = 0; row < first_dem.rows(); row++) {
            if (first_dem(col, row) == nodata_value) {
              first_dem(col, row) = m_opt.out_nodata_value;
            }
          }
        }
      }

      // Mark the handle to the image as not in use, though we still
      // keep that image file open, for increased performance, unless
      // their number becomes too large.
      m_imgMgr.release(dem_iter);
      
      if (dem_iter == 0 && m_opt.this_dem_as_reference != "") {
        // We won't actually use this DEM, we just do all in reference to it.
        continue;
      }
      
      // Compute linear weights
      ImageView<double> local_wts = grassfire(notnodata(select_channel(dem, 0), nodata_value));
      if (m_opt.use_centerline_weights) {
        // Erode based on grassfire weights, and then overwrite the grassfire
        // weights with centerline weights
        ImageView<DoubleGrayA> dem2 = copy(dem);
        for (int col = 0; col < dem2.cols(); col++) {
          for (int row = 0; row < dem2.rows(); row++) {
            if (local_wts(col, row) <= m_opt.erode_len) {
              dem2(col, row) = DoubleGrayA(nodata_value);
            }
          }
        }
        centerline_weights
                (create_mask_less_or_equal(select_channel(dem2, 0), nodata_value),
                 local_wts);
      }

      // If we don't limit the weights from above, we will have tiling artifacts,
      // as in different tiles the weights grow to different heights since
      // they are cropped to different regions. for priority blending length,
      // we'll do this process later, as the bbox is obtained differently in that case.
      if (m_opt.priority_blending_len <= 0) {
        for (int col = 0; col < local_wts.cols(); col++) {
          for (int row = 0; row < local_wts.rows(); row++) {
            local_wts(col, row) = std::min(local_wts(col, row), double(m_bias));
          }
        }
      }
      
      // Erode. We already did that if centerline weights are used.
      if (!m_opt.use_centerline_weights){
        int max_cutoff = max_pixel_value(local_wts);
        int min_cutoff = m_opt.erode_len;
        if (max_cutoff <= min_cutoff)
          max_cutoff = min_cutoff + 1; // precaution
        local_wts = clamp(local_wts - min_cutoff, 0.0, max_cutoff - min_cutoff);
      }
      
      // Blur the weights. If priority blending length is on, we'll do the blur later,
      // after weights from different DEMs are combined.
      if (m_opt.weights_blur_sigma > 0 && m_opt.priority_blending_len <= 0)
        blur_weights(local_wts, m_opt.weights_blur_sigma);

      // Raise to the power. Note that when priority blending length is positive, we
      // delay this process.
      if (m_opt.weights_exp != 1 && m_opt.priority_blending_len <= 0) {
        for (int col = 0; col < dem.cols(); col++){
          for (int row = 0; row < dem.rows(); row++){
            local_wts(col, row) = pow(local_wts(col, row), m_opt.weights_exp);
          }
        }
      }

#if 0
      // Dump the weights
      std::ostringstream os;
      os << "weights_" << dem_iter << ".tif";
      vw_out() << "Writing: " << os.str() << std::endl;
      bool has_georef = true, has_nodata = true;
      block_write_gdal_image(os.str(), local_wts,
			     has_georef, georef,
			     has_nodata, -100,
			     vw::cartography::GdalWriteOptions(),
			     TerminalProgressCallback("asp", ""));
#endif

      // Set the weights in the alpha channel
      for (int col = 0; col < dem.cols(); col++){
        for (int row = 0; row < dem.rows(); row++){
          dem(col, row).a() = local_wts(col, row);
        }
      }

      // Prepare the DEM for interpolation
      ImageViewRef<DoubleGrayA> interp_dem
        = interpolate(dem, BilinearInterpolation(), ConstantEdgeExtension());

      // Loop through each output pixel
      for (int c = 0; c < bbox.width(); c++){
        for (int r = 0; r < bbox.height(); r++){

          // Coordinates in the output mosaic
          Vector2 out_pix(c +  bbox.min().x(), r +  bbox.min().y());
          // Coordinate in this input DEM
          Vector2 in_pix = geotrans.reverse(out_pix);

          // Input DEM pixel relative to loaded bbox
          double x = in_pix[0] - in_box.min().x();
          double y = in_pix[1] - in_box.min().y();
          DoubleGrayA pval;

          int i0 = round(x),  // Round to nearest integer location
              j0 = round(y);
          if ((fabs(x-i0) < g_tol) && (fabs(y-j0) < g_tol) &&
              ((i0 >= 0) && (i0 <= dem.cols()-1) &&
               (j0 >= 0) && (j0 <= dem.rows()-1)) ){

            // A lot of care is needed here. We are at an integer
            // pixel, save for numerical error. Just borrow pixel's
            // value, and don't interpolate. Interpolation can result
            // in invalid pixels if the current pixel is valid but its
            // neighbors are not. It can also make it appear is if the
            // current indices are out of bounds while in fact they
            // are barely so.
            pval = dem(i0, j0);

          }else{ // We are not right on an integer pixel and we need to interpolate

            // Below must use x <= cols()-1 as x is double
            bool is_good = ((x >= 0) && (x <= dem.cols()-1) && // TODO: should be an image function!
		            (y >= 0) && (y <= dem.rows()-1));
            if (!is_good)
              continue; // Outside the loaded DEM bounds, skip to the next pixel

            // If we have weights of 0, that means there are invalid pixels, so skip this point.
            int i0 = (int)floor(x), j0 = (int)floor(y);
            int i1 = (int)ceil(x),  j1 = (int)ceil(y);
            if ((dem(i0, j0).a() <= 0) || (dem(i1, j0).a() <= 0) ||
	        (dem(i0, j1).a() <= 0) || (dem(i1, j1).a() <= 0))
              continue;

            pval = interp_dem(x, y); // Things checked out, do the interpolation.
          }
          // Seperate the value and alpha for this pixel.
          double val = pval.v();
          double wt  = pval.a();

          if (m_opt.priority_blending_len > 0) {
            // The priority blending, pixels from earlier DEMs at this location
            // are used unmodified unless close to that DEM boundary.
            wt = std::min(weight_modifier(c, r), wt);

            // Now ensure that the current DEM values will be used
            // unmodified unless close to the boundary for subsequent
            // DEMs. The weight w2 will be 0 well inside the DEM, and
            // increase towards the boundary.
            double wt2 = wt;
            wt2 = std::max(0.0, m_opt.priority_blending_len - wt2);
            weight_modifier(c, r) = std::min(weight_modifier(c, r), wt2);
          }

          if (wt <= 0)
            continue; // No need to continue if the weight is zero

          // Check if the current output value at this pixel is nodata
          bool is_nodata = ((tile(c, r) == m_opt.out_nodata_value));

          // Initialize the tile if not done already.
          // Init to zero not needed with some types.
          if (!m_opt.stddev && !m_opt.median && !m_opt.min && !m_opt.max &&
              m_opt.priority_blending_len <= 0){
            if ( is_nodata ){
              tile   (c, r) = 0;
              weights(c, r) = 0.0;
            }
          }

          // Update the output value according to the commanded mode
          if ( ( m_opt.first && is_nodata)                        ||
               m_opt.last                                         ||
               ( m_opt.min && ( val < tile(c, r) || is_nodata ) ) ||
               ( m_opt.max && ( val > tile(c, r) || is_nodata ) ) ||
               m_opt.median || m_opt.priority_blending_len > 0    ||
                     m_opt.block_max){
            // --> Conditions where we replace the current value
            tile   (c, r) = val;
            weights(c, r) = wt;

            // In these cases, the saved weight will be 1 or 0, since either
            // a given DEM gives it all, or nothing at all.
            if (m_opt.save_dem_weight >= 0 && (m_opt.first || m_opt.last ||
					              m_opt.min || m_opt.max))
              saved_weight(c, r) = (m_opt.save_dem_weight == dem_iter);

            // In these cases, the saved weight will be 1 or 0, since either
            // a given DEM gives it all, or nothing at all.
            if (m_opt.save_index_map && (m_opt.first || m_opt.last ||
				         m_opt.min || m_opt.max))
              index_map(c, r) = dem_iter;

          }else if (m_opt.mean){ // Mean --> Accumulate the value
            tile(c, r) += val;
            weights(c, r)++;

            if (m_opt.save_dem_weight == dem_iter)
              saved_weight(c, r) = 1;

          }else if (m_opt.count){ // Count --> Increment the value
            tile(c, r)++;
            weights(c, r) += wt;
          }else if (m_opt.stddev){ // Standard Deviation --> Keep running calculation
            weights(c, r) += 1.0;
            double curr_mean = tile_vec[0](c,r);
            double delta     = val - curr_mean;
            curr_mean     += delta / weights(c, r);
            double newVal = tile(c, r) + delta*(val - curr_mean);
            tile(c, r)    = newVal;
            tile_vec[0](c,r) = curr_mean;
          }else if (!noblend){ // Blending --> Weighted average
            tile(c, r) += wt*val;
            weights(c, r) += wt;
            if (m_opt.save_dem_weight == dem_iter)
              saved_weight(c, r) = wt;
          }

        } // End col loop
      } // End row loop

      // For the median option, keep a copy of the output tile for each input DEM!
      // Also do it for max per block.
      // - This will be memory intensive. 
      if (m_opt.median || m_opt.block_max) {
        tile_vec.push_back(copy(tile));
        dem_vec.push_back(dem_name);
      }
      
      // For priority blending, need also to keep all tiles, but also the weights
      if (m_opt.priority_blending_len > 0){
        tile_vec.push_back(copy(tile));
        weight_vec.push_back(copy(weights));
        clip2dem_index.push_back(dem_iter);
      }

    } // End iterating over DEMs

    // Divide by the weights in blend, mean
    if (!noblend || m_opt.mean){
      for (int c = 0; c < bbox.width(); c++){ // Iterate over all pixels!
        for (int r = 0; r < bbox.height(); r++){
          if ( weights(c, r) > 0 )
            tile(c, r) /= weights(c, r);

          if (m_opt.save_dem_weight >= 0 && weights(c, r) > 0)
            saved_weight(c, r) /= weights(c, r);

        } // End row loop
      } // End col loop
    } // End dividing case


    // Finish stddev calculations
    if (m_opt.stddev){
      for (int c = 0; c < bbox.width(); c++){ // Iterate over all pixels!
        for (int r = 0; r < bbox.height(); r++){

          if ( weights(c, r) > 1.0 ){
            tile(c, r) = sqrt( tile(c, r) / (weights(c, r) - 1.0) );
          } else { // Invalid pixel!
            tile(c, r) = m_opt.out_nodata_value;
          }
        } // End row loop
      } // End col loop
    } // End stddev case

    // For the median operation
    if (m_opt.median){
      // Init output pixels to nodata
      fill( tile, m_opt.out_nodata_value );
      vector<double> vals(tile_vec.size());
      // Iterate through all pixels
      for (int c = 0; c < bbox.width(); c++){
        for (int r = 0; r < bbox.height(); r++){
          // Compute the median for this pixel
          vals.clear();
          for (int i = 0; i < (int)tile_vec.size(); i++){
            ImageView<double> & tile_ref = tile_vec[i];
            if ( tile_ref(c, r) == m_opt.out_nodata_value )
              continue;
            vals.push_back(tile_ref(c, r));
          }
          if (!vals.empty()){
            tile(c, r) = math::destructive_median(vals);
          }
        }// End row loop
      } // End col loop
    } // End median case

    // For max per block, find the sum of values in each DEM
    if (m_opt.block_max) {
      fill( tile, m_opt.out_nodata_value );
      int num_tiles = tile_vec.size();
      if (tile_vec.size() != dem_vec.size()) 
        vw_throw(ArgumentErr() << "Book-keeping error.\n");
      std::vector<double> tile_sum(num_tiles, 0);
      for (int i = 0; i < num_tiles; i++) {
        for (int c = 0; c < tile_vec[i].cols(); c++) {
          for (int r = 0; r < tile_vec[i].rows(); r++) {
            if (tile_vec[i](c, r) != m_opt.out_nodata_value) {
              tile_sum[i] += tile_vec[i](c, r);
            }
          }
        }
        // The latter is useful later when inspecting the individual DEMs
        vw_out(DebugMessage,"asp") << "\n" << dem_vec[i] << " sum: " << tile_sum[i] << std::endl;
      }
      int max_index = std::distance(tile_sum.begin(),
                                    std::max_element(tile_sum.begin(), tile_sum.end()));
      if (max_index >= 0 && max_index < num_tiles) 
        tile = copy(tile_vec[max_index]);
    }
    
    // For priority blending length.
    if (m_opt.priority_blending_len > 0) {

      if (tile_vec.size() != weight_vec.size() || tile_vec.size() != clip2dem_index.size())
        vw_throw(ArgumentErr() << "There must be as many dem tiles as weight tiles.\n");

      // We will use the weights created so far only to burn holes in
      // the DEMs where we don't want blending. Then we will have to
      // recreate the weights. That because the current weights have
      // been interpolated from a different grid, and won't handle
      // erosion and bluring well.
      for (size_t clip_iter = 0; clip_iter < weight_vec.size(); clip_iter++) {
        for (int col = 0; col < weight_vec[clip_iter].cols(); col++){
          for (int row = 0; row < weight_vec[clip_iter].rows(); row++){
            if (weight_vec[clip_iter](col, row) <= 0)
              tile_vec[clip_iter](col, row) = m_opt.out_nodata_value;
          }
        }
        
        weight_vec[clip_iter] = grassfire(notnodata(tile_vec[clip_iter],
                                                      m_opt.out_nodata_value));
      }
      
      // Don't allow the weights to grow too fast, for uniqueness.
      for (size_t clip_iter = 0; clip_iter < weight_vec.size(); clip_iter++) {
        for (int col = 0; col < weight_vec[clip_iter].cols(); col++) {
          for (int row = 0; row < weight_vec[clip_iter].rows(); row++) {
            weight_vec[clip_iter](col, row)
              = std::min(weight_vec[clip_iter](col, row), double(m_bias));
          }
        }
      }

      // Blur the weights.
      for (size_t clip_iter = 0; clip_iter < weight_vec.size(); clip_iter++) {
        blur_weights(weight_vec[clip_iter], m_opt.weights_blur_sigma);
      }

      // Raise to power
      if (m_opt.weights_exp != 1) {
        for (size_t clip_iter = 0; clip_iter < weight_vec.size(); clip_iter++) {
          for (int col = 0; col < weight_vec[clip_iter].cols(); col++){
            for (int row = 0; row < weight_vec[clip_iter].rows(); row++){
              weight_vec[clip_iter](col, row)
                = pow(weight_vec[clip_iter](col, row), m_opt.weights_exp);
            }
          }
        }
      }

      // Now we are ready for blending
      fill( tile, m_opt.out_nodata_value );
      fill( weights, 0.0 );

      if (m_opt.save_dem_weight >= 0)
        fill(saved_weight, 0.0);

      for (size_t clip_iter = 0; clip_iter < weight_vec.size(); clip_iter++) {
        for (int col = 0; col < weight_vec[clip_iter].cols(); col++){
          for (int row = 0; row < weight_vec[clip_iter].rows(); row++){

            double wt = weight_vec[clip_iter](col, row);
            if (wt <= 0) continue; // nothing to do

            // Initialize the tile
            if (tile(col, row) == m_opt.out_nodata_value)
              tile(col, row) = 0;

            tile(col, row)    += wt*tile_vec[clip_iter](col, row);
            weights(col, row) += wt;

            if (clip2dem_index[clip_iter] == m_opt.save_dem_weight)
              saved_weight(col, row) = wt;
          }
        }
      }

      // Compute the weighted average
      for (int col = 0; col < tile.cols(); col++){
        for (int row = 0; row < weights.rows(); row++){
          if ( weights(col, row) > 0 )
            tile(col, row) /= weights(col, row);

          if (m_opt.save_dem_weight >= 0 && weights(col, row) > 0)
            saved_weight(col, row) /= weights(col, row);

        }
      }

#if 0
      for (size_t clip_iter = 0; clip_iter < weight_vec.size(); clip_iter++) {
	// Dump the modifier weights
	GeoReference crop_georef = crop(m_out_georef, bbox);
	std::ostringstream os;
	os << "tile_weight_" << clip_iter << ".tif";
	vw_out() << "Writing: " << os.str() << std::endl;
	bool has_georef = true, has_nodata = true;
	block_write_gdal_image(os.str(), weight_vec[clip_iter],
			       has_georef, crop_georef,
			       has_nodata, -100,
			       vw::cartography::GdalWriteOptions(),
			       TerminalProgressCallback("asp", ""));
      }
#endif

    } // end considering the priority blending length

    // Fill-in no-data values a bit and blur. If just the blurring is used,
    // it will choke on no-data values, leaving large holes around each,
    // hence the need to fill a little.
    if (m_opt.dem_blur_sigma > 0.0) {
      int kernel_size = vw::compute_kernel_size(m_opt.dem_blur_sigma);
      tile = apply_mask(gaussian_filter(fill_nodata_with_avg
					(create_mask(tile, m_opt.out_nodata_value),
					 kernel_size),
					m_opt.dem_blur_sigma),
			m_opt.out_nodata_value);
    }
    
    // Fill holes
    if (m_opt.hole_fill_len > 0){
      tile = apply_mask(vw::fill_holes_grass
			   (create_mask(tile, m_opt.out_nodata_value),
			    m_opt.hole_fill_len),
			m_opt.out_nodata_value);
    }

    // Save the weight instead
    if (m_opt.save_dem_weight >= 0)
      tile = saved_weight;

    // Save the index map instead
    if (m_opt.save_index_map)
      tile = index_map;


    // How many valid pixels are there in the tile
    long long int num_valid_in_tile = 0;
    for (int col = 0; col < tile.cols(); col++) {
      for (int row = 0; row < tile.rows(); row++) {
        Vector2 pix = Vector2(col, row) + bbox.min();
        if (!orig_box.contains(pix)) continue; // in case the box got expanded, ignore the padding
        if (tile(col, row) == m_opt.out_nodata_value) continue;
        num_valid_in_tile++;
      }
    }
    {
      // Lock and update the total number of valid pixels
      vw::Mutex::Lock lock(m_count_mutex);
      m_num_valid_pixels += num_valid_in_tile;
    }

    if (m_opt.first_dem_as_reference) {
      
      if (first_dem.cols() != tile.cols() || first_dem.rows() != tile.rows()) {
        vw_throw(ArgumentErr() << "Book-keeping error when blending into first DEM.\n");
      }

      // Wipe from the tile all values outside the perimeter of
      // first_dem. So we don't wipe values that happen to be
      // in the holes of first_dem.
      ImageView<double> local_wts;
      bool fill_holes = true;
      centerline_weights(create_mask(first_dem, m_opt.out_nodata_value), local_wts,
                         BBox2(), fill_holes);
      for (int col = 0; col < tile.cols(); col++) {
        for (int row = 0; row < tile.rows(); row++) {
          if (local_wts(col, row) == 0)
            tile(col, row) = m_opt.out_nodata_value;
        }
      }
    }
    
    // Return the tile we created with fake borders to make it look
    // the size of the entire output image. So far we operated
    // on doubles, here we cast to RealT.
    return prerasterize_type(pixel_cast<RealT>(tile),
			     -bbox.min().x(), -bbox.min().y(),
			     cols(), rows() );
  }

  template <class DestT>
  inline void rasterize(DestT const& dest, BBox2i bbox) const {
    vw::rasterize(prerasterize(bbox), dest, bbox);
  }
}; // End class DemMosaicView


/// Find the bounding box of all DEMs in the projected space.
/// - mosaic_bbox is the output bounding box in projected space
/// - dem_proj_bboxes and dem_pixel_bboxes are the locations of
///   each input DEM in the output DEM in projected and pixel coordinates.
void load_dem_bounding_boxes(Options       const& opt,
			     GeoReference  const& mosaic_georef,
			     BBox2              & mosaic_bbox, // Projected coordinates
			     std::vector<BBox2> & dem_proj_bboxes,
			     std::vector<BBox2i> & dem_pixel_bboxes) {

  vw_out() << "Determining the bounding boxes of the input DEMs.\n";

  // Initialize the outputs
  mosaic_bbox = BBox2();
  dem_proj_bboxes.clear();
  dem_pixel_bboxes.clear();
  
  TerminalProgressCallback tpc("", "\t--> ");
  tpc.report_progress(0);
  double inc_amount = 1.0 / double(opt.dem_files.size() );

  BBox2 first_dem_proj_box;
  
  // Loop through all DEMs
  for (int dem_iter = 0; dem_iter < (int)opt.dem_files.size(); dem_iter++){ 

    // Open a handle to this DEM file
    DiskImageResourceGDAL in_rsrc(opt.dem_files[dem_iter]);
    DiskImageView<RealT>  img(opt.dem_files[dem_iter]);
    GeoReference          georef = read_georef(opt.dem_files[dem_iter]);
    BBox2i                pixel_box = bounding_box(img);

    dem_pixel_bboxes.push_back(pixel_box);

    if (dem_iter == 0) 
      first_dem_proj_box = georef.bounding_box(img);
    
    bool has_lonat = (georef.proj4_str().find("+proj=longlat") != std::string::npos ||
                      mosaic_georef.proj4_str().find("+proj=longlat") != std::string::npos );
    
    // Compute bounding box of this DEM. The simple case is when all DEMs have
    // the same projection, and it is not longlat, as then we need to worry about
    // a 360 degree shift.
    if ( (!has_lonat) && mosaic_georef.overall_proj4_str() == georef.overall_proj4_str() ){
      BBox2 proj_box = georef.bounding_box(img);
      mosaic_bbox.grow(proj_box);
      dem_proj_bboxes.push_back(proj_box);
    }else{
      // Compute the bounding box of the current image in projected
      // coordinates of the mosaic. There is always a worry that the
      // lonlat of the mosaic so far and of the current DEM will be
      // offset by 360 degrees. Try to deal with that.
      BBox2 proj_box;
      BBox2 imgbox = bounding_box(img);
      BBox2 mosaic_pixel_box;
      
      // Get the bbox of current mosaic in pixels.
      if (dem_iter == 0) {
        // TODO: Not robust. How to estimate the pixel extent of the
        // first DEM in the mosaic? Taking into account that
        // the first DEM lonlat box and the mosaic lonlat box
        // may be offset by 360 degrees?
        mosaic_pixel_box = imgbox;
      }else{
        mosaic_pixel_box = mosaic_georef.point_to_pixel_bbox(mosaic_bbox);
      }
      
      GeoTransform geotrans(georef, mosaic_georef, imgbox, mosaic_pixel_box);
      proj_box = geotrans.pixel_to_point_bbox(imgbox);

      // Notify the user if one of the input DEMS is going to wrap around the 
      // left and right sides of their output image.
      if ((dem_iter > 0) && geotrans.check_bbox_wraparound()) {
        vw_out() << "WARNING: Longitude wraparound detected from input DEM "
                 << opt.dem_files[dem_iter] << " to the output georeference. "
                 << "This can result in an output DEM *much* larger than expected. "
                 << "Consider changing your output georeference options or your inputs.\n";
      }

      mosaic_bbox.grow(proj_box);
      dem_proj_bboxes.push_back(proj_box);
    } // End second case

    tpc.report_incremental_progress( inc_amount );
  } // End loop through DEM files
  tpc.report_finished();

  // If the first dem is used as reference, no matter what use its own box
  if (opt.first_dem_as_reference) 
    mosaic_bbox = first_dem_proj_box;
  
} // End function load_dem_bounding_boxes


void handle_arguments( int argc, char *argv[], Options& opt ) {

  po::options_description general_options("Options");
  general_options.add_options()
    ("dem-list-file,l", po::value<string>(&opt.dem_list_file),
	   "Text file listing the DEM files to mosaic, one per line.")
    ("output-prefix,o", po::value(&opt.out_prefix), "Specify the output prefix.")
    ("tile-size",       po::value<int>(&opt.tile_size)->default_value(1000000),
	   "The maximum size of output DEM tile files to write, in pixels.")
    ("tile-index",      po::value<int>(&opt.tile_index),
     "The index of the tile to save (starting from zero). When this program is invoked, it will print out how many tiles are there. Default: save all tiles.")
    ("tile-list",      po::value(&opt.tile_list_str)->default_value(""),
     "List of tile indices (in quotes) to save. A tile index starts from 0.")
    ("erode-length",    po::value<int>(&opt.erode_len)->default_value(0),
	   "Erode input DEMs by this many pixels at boundary before mosaicking them.")
    ("priority-blending-length", po::value<int>(&opt.priority_blending_len)->default_value(0),
	   "If positive, keep unmodified values from the earliest available DEM at the current location except a band this wide measured in pixels around its boundary where blending will happen.")
    ("hole-fill-length",   po::value(&opt.hole_fill_len)->default_value(0),
	   "Maximum dimensions of a hole in the output DEM to fill in, in pixels.")
    ("tr",              po::value(&opt.tr),
	   "Output DEM resolution in target georeferenced units per pixel. Default: use the same resolution as the first DEM to be mosaicked.")
    ("t_srs",           po::value(&opt.target_srs_string)->default_value(""),
	   "Specify the output projection (PROJ.4 string). Default: use the one from the first DEM to be mosaicked.")
    ("t_projwin",       po::value(&opt.projwin),
	   "Limit the mosaic to this region, with the corners given in georeferenced coordinates (xmin ymin xmax ymax). Max is exclusive.")
    ("first",   po::bool_switch(&opt.first)->default_value(false),
	   "Keep the first encountered DEM value (in the input order).")
    ("last",    po::bool_switch(&opt.last)->default_value(false),
	   "Keep the last encountered DEM value (in the input order).")
    ("min",     po::bool_switch(&opt.min)->default_value(false),
	   "Keep the smallest encountered DEM value.")
    ("max",     po::bool_switch(&opt.max)->default_value(false),
	   "Keep the largest encountered DEM value.")
    ("mean",    po::bool_switch(&opt.mean)->default_value(false),
	   "Find the mean DEM value.")
    ("stddev",    po::bool_switch(&opt.stddev)->default_value(false),
	   "Find the standard deviation of the DEM values.")
    ("median",  po::bool_switch(&opt.median)->default_value(false),
	   "Find the median DEM value (this can be memory-intensive, fewer threads are suggested).")
    ("count",   po::bool_switch(&opt.count)->default_value(false),
     "Each pixel is set to the number of valid DEM heights at that pixel.")
    ("block-max", po::bool_switch(&opt.block_max)->default_value(false),
     "For each block of size --block-size, keep the DEM with the largest sum of values in the block.")
    ("georef-tile-size",    po::value<double>(&opt.geo_tile_size),
	   "Set the tile size in georeferenced (projected) units (e.g., degrees or meters).")
    ("output-nodata-value", po::value<double>(&opt.out_nodata_value),
	   "No-data value to use on output. Default: use the one from the first DEM to be mosaicked.")
    ("weights-blur-sigma", po::value<double>(&opt.weights_blur_sigma)->default_value(5.0),
	   "The standard deviation of the Gaussian used to blur the weights. Higher value results in smoother weights and blending. Set to 0 to not use blurring.")
    ("weights-exponent",   po::value<double>(&opt.weights_exp)->default_value(2.0),
	   "The weights used to blend the DEMs should increase away from the boundary as a power with this exponent. Higher values will result in smoother but faster-growing weights.")
    ("use-centerline-weights",   po::bool_switch(&opt.use_centerline_weights)->default_value(false),
     "Compute weights based on a DEM centerline algorithm. Produces smoother weights if the input DEMs don't have holes or complicated boundary.")
    ("dem-blur-sigma", po::value<double>(&opt.dem_blur_sigma)->default_value(0.0),
     "Blur the final DEM using a Gaussian with this value of sigma. Default: No blur.")
    ("nodata-threshold", po::value(&opt.nodata_threshold)->default_value(std::numeric_limits<double>::quiet_NaN()),
     "Values no larger than this number will be interpreted as no-data.")
    ("extra-crop-length", po::value<int>(&opt.extra_crop_len)->default_value(200),
     "Crop the DEMs this far from the current tile (measured in pixels) before blending them (a small value may result in artifacts).")
    ("block-size",      po::value<int>(&opt.block_size)->default_value(0),
     "To be used with --max-per-block. A large value can result in increased memory usage.")
    ("save-dem-weight",      po::value<int>(&opt.save_dem_weight),
     "Save the weight image that tracks how much the input DEM with given index contributed to the output mosaic at each pixel (smallest index is 0).")
    ("first-dem-as-reference", po::bool_switch(&opt.first_dem_as_reference)->default_value(false),
     "The output DEM will have the same size, grid, and georeference as the first one, with the other DEMs blended within its perimeter.")
    ("this-dem-as-reference", po::value(&opt.this_dem_as_reference)->default_value(""),
     "The output DEM will have the same size, grid, and georeference as this one, but it will not be used in the mosaic.")
    ("save-index-map",   po::bool_switch(&opt.save_index_map)->default_value(false),
     "For each output pixel, save the index of the input DEM it came from (applicable only for --first, --last, --min, and --max). A text file with the index assigned to each input DEM is saved as well.")
    ("threads",             po::value<int>(&opt.num_threads)->default_value(4),
	   "Number of threads to use.")
    ("help,h", "Display this help message.");

  po::options_description positional("");
  po::positional_options_description positional_desc;

  std::string usage("[options] <dem files or -l dem_files_list.txt> -o output_file_prefix");
  bool allow_unregistered = true;
  std::vector<std::string> unregistered;
  po::variables_map vm =
    asp::check_command_line( argc, argv, opt, general_options, general_options,
			     positional, positional_desc, usage,
			     allow_unregistered, unregistered );

  // Error checking
  if (opt.out_prefix == "")
    vw_throw(ArgumentErr() << "No output prefix was specified.\n"
			   << usage << general_options );
  if (opt.num_threads == 0)
    vw_throw(ArgumentErr() << "The number of threads must be set and positive.\n"
			   << usage << general_options );
  if (opt.erode_len < 0)
    vw_throw(ArgumentErr() << "The erode length must not be negative.\n"
			   << usage << general_options );
  if (opt.extra_crop_len < 0)
    vw_throw(ArgumentErr() << "The blending length must not be negative.\n"
			   << usage << general_options );
  if (opt.hole_fill_len < 0)
    vw_throw(ArgumentErr() << "The hole fill length must not be negative.\n"
			   << usage << general_options );
  if (opt.tile_size <= 0)
    vw_throw(ArgumentErr() << "The size of a tile in pixels must be positive.\n"
			   << usage << general_options );

  if (opt.priority_blending_len < 0)
    vw_throw(ArgumentErr() << "The priority blending length must not be negative.\n"
			   << usage << general_options );

  // If priority blending is used, need to adjust extra_crop_len accordingly
  opt.extra_crop_len = std::max(opt.extra_crop_len, 3*opt.priority_blending_len);

  // Make sure no more than one of these options is enabled.
  int noblend = no_blend(opt);
  if (noblend > 1)
    vw_throw(ArgumentErr() << "At most one of the options --first, --last, "
	     << "--min, --max, -mean, --stddev, --median, --count can be specified.\n"
	     << usage << general_options );

  if (opt.geo_tile_size < 0)
    vw_throw(ArgumentErr() << "The size of a tile in georeferenced units must not be negative.\n"
			   << usage << general_options );

  if (noblend && opt.priority_blending_len > 0) {
    vw_throw(ArgumentErr()
	     << "Priority blending cannot happen if any of the statistics DEMs are computed.\n"
	     << usage << general_options );
  }

  if (opt.priority_blending_len > 0 && opt.weights_exp == 2) {
    vw_out() << "Increasing --weights-exponent to 3 for smoother blending.\n";
    opt.weights_exp = 3;
  }
  
  if (noblend && !opt.first && !opt.last && !opt.min && !opt.max && !opt.mean
      && opt.save_dem_weight >= 0) {
    vw_throw(ArgumentErr()
	     << "Cannot save the weights unless blending is on or one of "
	     << "--first, --last, --min, --max, --mean is invoked.\n"
	     << usage << general_options );
  }

  if (opt.save_index_map && !opt.first && !opt.last && !opt.min && !opt.max)
    vw_throw(ArgumentErr()
	     << "Cannot save an index map unless one of "
	     << "--first, --last, --min, --max is invoked.\n"
	     << usage << general_options );

  if (opt.save_dem_weight >= 0 && opt.save_index_map)
    vw_throw(ArgumentErr()
	     << "Cannot save both the index map and the DEM weights at the same time.\n"
	     << usage << general_options );

  // For compatibility with the GDAL tools, allow the min and max to be reversed.
  if (opt.projwin != BBox2()) {
    if (opt.projwin.min().x() > opt.projwin.max().x())
      std::swap(opt.projwin.min().x(), opt.projwin.max().x());
    if (opt.projwin.min().y() > opt.projwin.max().y())
      std::swap(opt.projwin.min().y(), opt.projwin.max().y());
  }
  
  if (opt.weights_blur_sigma < 0.0)
    vw_throw(ArgumentErr() << "The standard deviation used for blurring must be non-negative.\n"
			   << usage << general_options );

  if (opt.weights_exp <= 0)
    vw_throw(ArgumentErr() << "The weights exponent must be positive.\n"
			   << usage << general_options );

  // Read the DEMs
  if (opt.dem_list_file != ""){ // Get them from a list

    if (!unregistered.empty())
      vw_throw(ArgumentErr() << "The DEMs were specified via a list. "
			     << "There were however extraneous files or options passed in.\n"
			     << usage << general_options );

    ifstream is(opt.dem_list_file.c_str());
    string file;
    while (is >> file)
      opt.dem_files.push_back(file);
    if (opt.dem_files.empty())
      vw_throw(ArgumentErr() << "No DEM files to mosaic.\n");
    is.close();

  }else{  // Get them from the command line

    if (unregistered.empty())
      vw_throw(ArgumentErr() << "No input DEMs were specified.\n"
			     << usage << general_options );
    opt.dem_files = unregistered;
  }

  if (opt.this_dem_as_reference != "" && opt.first_dem_as_reference) {
    vw_throw(ArgumentErr() << "Cannot have both options --first-dem-as-reference "
             << "and --this-dem-as-reference.\n"
	     << usage << general_options );
  }

  // We will co-opt the logic of first_dem_as_reference but won't blend the reference DEM
  if (opt.this_dem_as_reference != "") {
    opt.first_dem_as_reference = true;
    opt.dem_files.insert(opt.dem_files.begin(), opt.this_dem_as_reference);
  }
  
  if (int(opt.dem_files.size()) <= opt.save_dem_weight) {
    vw_throw(ArgumentErr() << "Cannot save weights for given index as it is out of bounds.\n"
	     << usage << general_options );
  }

  // Create the output directory
  vw::create_out_dir(opt.out_prefix);

  // Turn on logging to file
  asp::log_to_file(argc, argv, "", opt.out_prefix);

  if (!vm.count("output-nodata-value")){
    // Set a default out_nodata_value, but remember that this is
    // set internally, not by the user.
    opt.has_out_nodata = false;
    opt.out_nodata_value = -numeric_limits<RealT>::max();
  }else
    opt.has_out_nodata = true;

  // Cast this to float. All our nodata are float.
  opt.nodata_threshold = RealT(opt.nodata_threshold);

  // Parse the list of tiles to save. First replace commas and semicolons by a space.
  std::replace(opt.tile_list_str.begin(), opt.tile_list_str.end(), ',', ' ');
  std::replace(opt.tile_list_str.begin(), opt.tile_list_str.end(), ';', ' ');
  opt.tile_list.clear();
  std::istringstream os(opt.tile_list_str);
  int val;
  while (os >> val){
    opt.tile_list.insert(val);
  }
  
} // End function handle_arguments

int main( int argc, char *argv[] ) {

  Options opt;
  try{

    handle_arguments( argc, argv, opt );

    // TODO: Fix here. If the DEM is double, read the nodata as double,
    // without casting to float. If it is float, cast to float.
    
    // Read nodata from first DEM, unless the user chooses to specify it.
    if (!opt.has_out_nodata){
      DiskImageResourceGDAL in_rsrc(opt.dem_files[0]);
      // Since the DEMs have float pixels, we must read the no-data as
      // float as well. (this is a bug fix). Yet we store it in a
      // double, as we will cast the DEM pixels to double as well.
      if (in_rsrc.has_nodata_read()) opt.out_nodata_value = RealT(in_rsrc.nodata_read());
    }

    // Watch for underflow, if mixing doubles and float. Particularly problematic
    // is when the nodata_value cannot be represented exactly as a float.
    if (opt.out_nodata_value < static_cast<double>(-numeric_limits<RealT>::max()) ||
        RealT(opt.out_nodata_value)  != double(opt.out_nodata_value) ) {
      vw_out() << "The no-data value cannot be represented exactly as a float. "
	       << "Changing it to the smallest float.\n";
      opt.out_nodata_value = static_cast<double>(-numeric_limits<RealT>::max());
    }

    vw_out() << "Using output no-data value: " << opt.out_nodata_value << endl;

    // Form the mosaic georef. The georef of the first DEM is used as
    // initial guess unless user wants to change the resolution and projection.
    if (opt.target_srs_string != "")
      opt.target_srs_string = processed_proj4(opt.target_srs_string);

    // By default the output georef is equal to the first input georef
    GeoReference mosaic_georef = read_georef(opt.dem_files[0]);

    if (opt.first_dem_as_reference) {
      if (opt.target_srs_string != "" || opt.tr > 0 || opt.projwin != BBox2()) 
        vw_throw(ArgumentErr()
                 << "Cannot change the projection, spacing, or output box, if the first DEM "
                 << "is to be used as reference.\n");
      if (opt.first || opt.last || opt.min || opt.max || opt.mean || opt.median || opt.stddev ||
          opt.priority_blending_len > 0 || opt.save_dem_weight >= 0 ||
          !boost::math::isnan(opt.nodata_threshold)) {
        vw_throw(ArgumentErr()
                 << "Cannot do anything except regular blending if the first DEM "
                 << "is to be used as reference.\n");
      }
    }
  
    double spacing = opt.tr;
    if (opt.target_srs_string != ""                                              &&
      opt.target_srs_string != processed_proj4(mosaic_georef.overall_proj4_str()) &&
      spacing <= 0 ){
        vw_throw(ArgumentErr()
           << "Changing the projection was requested. The output DEM "
           << "resolution must be specified via the --tr option.\n");
    }

    if (opt.target_srs_string != ""){
      // Set the srs string into georef.
      bool have_user_datum = false;
      Datum user_datum;
      asp::set_srs_string(opt.target_srs_string,
			  have_user_datum, user_datum, mosaic_georef);
    }

    // Steal the datum and its name from the input, if the output
    // datum name is unknown.
    if (mosaic_georef.datum().name() == "unknown"){
      GeoReference georef = read_georef(opt.dem_files[0]);
      if (mosaic_georef.datum().semi_major_axis() == georef.datum().semi_major_axis() &&
    	  mosaic_georef.datum().semi_minor_axis() == georef.datum().semi_minor_axis() ){
          vw_out() << "Using the datum: " << georef.datum() << std::endl;
          mosaic_georef.set_datum(georef.datum());
      }
    }

    // Use desired spacing if user-specified
    if (spacing > 0.0){
      // Get lonlat bounding box of the first DEM.
      DiskImageView<RealT> dem0(opt.dem_files[0]);
      BBox2 llbox0 = mosaic_georef.pixel_to_lonlat_bbox(bounding_box(dem0));
      
      // Reset transform with user provided spacing.
      Matrix<double,3,3> transform = mosaic_georef.transform();
      transform.set_identity();
      transform(0, 0) =  spacing;
      transform(1, 1) = -spacing;
      mosaic_georef.set_transform(transform);

      // Set the translation part of the transform so that the origin
      // maps to the lonlat box corner. This is still not fully
      // reliable, but better than nothing. We will adjust
      // mosaic_georef later on.
      Vector2 ul = mosaic_georef.lonlat_to_pixel(Vector2(llbox0.min().x(), llbox0.max().y()));
      mosaic_georef = crop(mosaic_georef, ul.x(), ul.y());

    }else // Update spacing variable from the current transform
      spacing = mosaic_georef.transform()(0, 0);

    // if the user specified the tile size in georeferenced units.
    if (opt.geo_tile_size > 0){
      opt.tile_size = (int)round(opt.geo_tile_size/spacing);
      vw_out() << "Tile size in pixels: " << opt.tile_size << "\n";
    }
    opt.tile_size = std::max(opt.tile_size, 1);

    // Load the bounding boxes from all of the DEMs
    BBox2 mosaic_bbox;
    vector<BBox2> dem_proj_bboxes;
    vector<BBox2i> dem_pixel_bboxes, loaded_dem_pixel_bboxes;
    load_dem_bounding_boxes(opt, mosaic_georef, mosaic_bbox,
                            dem_proj_bboxes, dem_pixel_bboxes);

    if (opt.projwin != BBox2()) {
      // If to create the mosaic only in a given region
      mosaic_bbox.crop(opt.projwin);
      // Crop the proj boxes as well
      for (int dem_iter = 0; dem_iter < (int)opt.dem_files.size(); dem_iter++)
        dem_proj_bboxes[dem_iter].crop(opt.projwin);
    }
    
    // First-guess pixel box
    BBox2 pixel_box = custom_point_to_pixel_bbox(mosaic_georef, mosaic_bbox);

    // Take care of numerical artifacts
    Vector2 beg_pix = pixel_box.min();
    if (norm_2(beg_pix - round(beg_pix)) < g_tol )
      beg_pix = round(beg_pix);
    mosaic_georef = crop(mosaic_georef, beg_pix[0], beg_pix[1]);

    // Image size
    pixel_box = custom_point_to_pixel_bbox(mosaic_georef, mosaic_bbox);
    Vector2 end_pix = pixel_box.max();
    int cols = (int)round(end_pix[0]); // end_pix is the last pix in the image
    int rows = (int)round(end_pix[1]);

    // Form the mosaic and write it to disk
    vw_out()<< "The size of the mosaic is " << cols << " x " << rows << " pixels.\n";
    vw_out()<< "The output georeference is\n" << mosaic_georef << std::endl;

    // This bias is very important. This is how much we should read from
    // the images beyond the current boundary to avoid tiling artifacts.
    int bias = opt.erode_len + opt.extra_crop_len + opt.hole_fill_len
      + 2*vw::compute_kernel_size(opt.weights_blur_sigma);

    // The next power of 2 >= 4*bias. We want to make the blocks big,
    // to reduce overhead from this bias, but not so big that it may
    // not fit in memory.
    int block_size = nextpow2(4.0*bias);
    block_size = std::max(block_size, 256); // don't make them too small though
    if (opt.block_size > 0)
      block_size = opt.block_size;
    
    int num_tiles_x = (int)ceil((double)cols/double(opt.tile_size));
    int num_tiles_y = (int)ceil((double)rows/double(opt.tile_size));
    if (num_tiles_x <= 0) num_tiles_x = 1;
    if (num_tiles_y <= 0) num_tiles_y = 1;
    int num_tiles = num_tiles_x*num_tiles_y;
    vw_out() << "Number of tiles: " << num_tiles_x << " x "
             << num_tiles_y << " = " << num_tiles << ".\n";
    
    if (opt.tile_index >= num_tiles){
      vw_out() << "Tile with index: " << opt.tile_index
	             << " is out of bounds." << std::endl;
      return 0;
    }

    // If to use a range
    if (!opt.tile_list.empty() && opt.tile_index >= 0) 
      vw_throw(ArgumentErr() << "Cannot specify both tile index and tile range.\n");

    // See if to save all tiles, or an individual tile.
    int start_tile = opt.tile_index, end_tile = opt.tile_index + 1;
    if (opt.tile_index < 0){
      start_tile = 0;
      end_tile = num_tiles;
    }

    // Compute the bounding box of each output tile
    std::vector<BBox2i> tile_pixel_bboxes;
    for (int tile_id = start_tile; tile_id < end_tile; tile_id++){

      int tile_index_y = tile_id / num_tiles_x;
      int tile_index_x = tile_id - tile_index_y*num_tiles_x;
      BBox2i tile_box(tile_index_x*opt.tile_size,
		      tile_index_y*opt.tile_size,
		      opt.tile_size, opt.tile_size);

      // Bounding box of this tile in pixels in the output image
      tile_box.crop(BBox2i(0, 0, cols, rows));

      tile_pixel_bboxes.push_back(tile_box);
    }

    // Store the no-data values, pointers to images, and georeferences (for speed).
    vw_out() << "Reading the input DEMs.\n";
    vector<double>          nodata_values;
    vector<GeoReference>    georefs;
    std::vector<string>     loaded_dems;
    DiskImageManager<RealT> imgMgr;

    BBox2i output_dem_box = BBox2i(0, 0, cols, rows); // output DEM box
    
    // Loop through all DEMs
    for (int dem_iter = 0; dem_iter < (int)opt.dem_files.size(); dem_iter++){

      // Get the DEM bounding box that we previously computed (output projected coords)
      BBox2 dem_bbox = dem_proj_bboxes[dem_iter];

      // Go through each of the tile bounding boxes and see they intersect this DEM
      bool use_this_dem = false;
      for (int tile_id = start_tile; tile_id < end_tile; tile_id++){

        if (!opt.tile_list.empty() && opt.tile_list.find(tile_id) == opt.tile_list.end()) 
          continue;
        
        // Get tile bbox in pixels, then convert it to projected coords.
        BBox2i tile_pixel_box = tile_pixel_bboxes[tile_id - start_tile];
        BBox2  tile_proj_box  = mosaic_georef.pixel_to_point_bbox(tile_pixel_box);

        if (tile_proj_box.intersects(dem_bbox)) {
          use_this_dem = true;
          break;
        }
      }
      if (use_this_dem == false)
        continue; // Skip to the next DEM if we don't need this one.

      // The GeoTransform will hide the messy details of conversions
      // from pixels to points and lon-lat.
      GeoReference georef  = read_georef(opt.dem_files[dem_iter]);
      BBox2i dem_pixel_box = dem_pixel_bboxes[dem_iter];
      GeoTransform geotrans(georef, mosaic_georef, dem_pixel_box, output_dem_box);

      // Get the current DEM bounding box in pixel units of the output mosaicked DEM
      BBox2 curr_box = geotrans.forward_bbox(dem_pixel_box);
      curr_box.crop(output_dem_box);

      // This is a fix for GDAL crashing when there are too many open
      // file handles. In such situation, just selectively close the
      // handles furthest from the current location.
      imgMgr.add_file_handle_not_thread_safe(opt.dem_files[dem_iter], curr_box);
      
      double curr_nodata_value = opt.out_nodata_value;
      try {
        // Get the nodata-value. Need a try block, in case we can't
        // open more handles.
        DiskImageResourceGDAL in_rsrc(opt.dem_files[dem_iter]);
        if ( in_rsrc.has_nodata_read() )
          curr_nodata_value = RealT(in_rsrc.nodata_read());
      }catch(std::exception const& e){
        // Try again
        imgMgr.freeup_handles_not_thread_safe();
        DiskImageResourceGDAL in_rsrc(opt.dem_files[dem_iter]);
        if ( in_rsrc.has_nodata_read() )
          curr_nodata_value = RealT(in_rsrc.nodata_read());
      }
      
      loaded_dems.push_back(opt.dem_files[dem_iter]);

      if (!boost::math::isnan(opt.nodata_threshold)) 
        curr_nodata_value = opt.nodata_threshold;
      
      // Add the info for this DEM to the appropriate vectors
      nodata_values.push_back(curr_nodata_value);
      georefs.push_back(georef);
      loaded_dem_pixel_bboxes.push_back(dem_pixel_box);
    } // End loop through DEM files

    // If there are 17 tiles, let them be tile-00, ..., tile-16.
    int num_digits = 1;
    int tens = 10;
    while (num_tiles - 1 >= tens){
      num_digits++;
      tens *= 10;
    }
    
    // Time to generate each of the output tiles
    for (int tile_id = start_tile; tile_id < end_tile; tile_id++){

      if (!opt.tile_list.empty() && opt.tile_list.find(tile_id) == opt.tile_list.end()) 
        continue;
      
      // Get the bounding box we previously computed
      BBox2i tile_box = tile_pixel_bboxes[tile_id - start_tile];

      ostringstream os;
      os << opt.out_prefix << "-tile-"
         << std::setfill('0') << std::setw(num_digits) << tile_id
         << tile_suffix(opt) << ".tif";
      std::string dem_tile = os.str();

      // Set up tile image and metadata
      long long int num_valid_pixels; // Will be populated when saving to disk
      vw::Mutex count_mutex; // to lock when updating num_valid_pixels

      ImageViewRef<RealT> out_dem
        = crop(DemMosaicView(cols, rows, bias, opt,
                             imgMgr, georefs,
                             mosaic_georef, nodata_values,
                             loaded_dem_pixel_bboxes,
                             num_valid_pixels, count_mutex),
               tile_box);
      GeoReference crop_georef = crop(mosaic_georef, tile_box.min().x(),
				      tile_box.min().y());

      // Raster the tile to disk
      vw_out() << "Writing: " << dem_tile << std::endl;
      TerminalProgressCallback tpc("asp", "\t--> ");
      asp::save_with_temp_big_blocks(block_size, dem_tile, out_dem, crop_georef,
				     opt.out_nodata_value, opt, tpc);

      vw_out() << "Number of valid (not no-data) pixels written: " << num_valid_pixels
               << "."<< std::endl;
      if (num_valid_pixels == 0) {
        vw_out() << "Removing tile with no valid pixels: " << dem_tile << std::endl;
        boost::filesystem::remove(dem_tile);
      }
      
    } // End loop through tiles

    // Write the name of each DEM file that was used together with its index
    if (opt.save_index_map) {
      std::string index_map = opt.out_prefix + "-index-map.txt";
      vw_out() << "Writing: " << index_map << std::endl;
      std::ofstream ih(index_map.c_str());
      for (int dem_iter = 0; dem_iter < (int)loaded_dems.size(); dem_iter++){
        ih << opt.dem_files[dem_iter] << ' ' << dem_iter << std::endl;
      }
    }

  } ASP_STANDARD_CATCHES;

  return 0;
}
