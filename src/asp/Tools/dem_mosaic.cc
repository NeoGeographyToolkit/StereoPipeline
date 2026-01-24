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

/// \file dem_mosaic.cc
///

// A program to merge several DEMs into one. The inputs are usually float. The
// processing happens in double precision.

#include <asp/Core/Macros.h>
#include <asp/Core/GdalUtils.h>
#include <asp/Core/DemMosaic.h>
#include <asp/Core/DemMosaicOptions.h>
#include <asp/Core/DemMosaicParse.h>
#include <asp/Core/FileUtils.h>

#include <vw/FileIO/DiskImageManager.h>
#include <vw/Image/InpaintView.h>
#include <vw/Image/Algorithms2.h>
#include <vw/Image/Filter.h>
#include <vw/Cartography/GeoTransform.h>
#include <vw/Image/NoDataAlg.h>
#include <vw/FileIO/FileUtils.h>

#include <cmath>
#include <iostream>
#include <fstream>
#include <iomanip>
#include <string>
#include <vector>
#include <sys/types.h>
#include <sys/stat.h>
#include <unistd.h>
#include <limits>
#include <algorithm>

using namespace vw; // TODO(oalexan1): Remove this namespace

// This is used for various tolerances
double g_tol = 1e-6;

std::string tile_suffix(asp::DemMosaicOptions const& opt) {
  std::string ans;
  if (opt.first) ans     = "-first";
  if (opt.last) ans      = "-last";
  if (opt.min) ans       = "-min";
  if (opt.max) ans       = "-max";
  if (opt.block_max) ans = "-block-max";
  if (opt.mean) ans      = "-mean";
  if (opt.stddev) ans    = "-stddev";
  if (opt.median) ans    = "-median";
  if (opt.nmad) ans      = "-nmad";
  if (opt.count) ans     = "-count";
  if (opt.save_index_map)       ans += "-index-map";
  if (opt.save_dem_weight >= 0) ans += "-weight-dem-index-" + stringify(opt.save_dem_weight);

  return ans;
}

// Initializations needed for various modes
void initializeTileVector(int num_images,
                          BBox2i const& bbox, asp::DemMosaicOptions const& opt,
                          // Outputs
                          ImageView<double> & tile,
                          std::vector<ImageView<double>>& tile_vec,
                          std::vector<ImageView<double>>& weight_vec) {

  // Wipe the output vectors
  tile_vec.clear();
  weight_vec.clear();

  if (opt.median || opt.nmad) // Store each input separately
    tile_vec.reserve(num_images);

  if (opt.stddev) { // Need one working image
    tile_vec.push_back(ImageView<double>(bbox.width(), bbox.height()));
    // Each pixel starts at zero, nodata is handled later
    fill(tile_vec[0], 0.0);
    fill(tile,        0.0);
  }

  if (opt.priority_blending_len > 0) { // Store each weight separately
    tile_vec.reserve  (num_images);
    weight_vec.reserve(num_images);
  }

  return;
}

// Use the weights created so far only to burn holes in
// the DEMs where we don't want blending. Then we will have to
// recreate the weights. That because the current weights have
// been interpolated from a different grid, and won't handle
// erosion and blur well.
void priorityBlend(double out_nodata_value,
                   double bias,
                   double weights_blur_sigma,
                   double weights_exp,
                   bool no_border_blend,
                   int save_dem_weight,
                   std::vector<int> const& clip2dem_index,
                   vw::cartography::GeoReference const& out_georef,
                   BBox2i const& bbox,
                   // Outputs
                   std::vector<ImageView<double>> & tile_vec,
                   std::vector<ImageView<double>> & weight_vec,
                   ImageView<double> & tile,
                   ImageView<double> & weights,
                   ImageView<double> & saved_weight) {

  if (tile_vec.size() != weight_vec.size() || tile_vec.size() != clip2dem_index.size())
    vw_throw(ArgumentErr() << "There must be as many dem tiles as weight tiles.\n");

  for (size_t clip_iter = 0; clip_iter < weight_vec.size(); clip_iter++) {
    for (int col = 0; col < weight_vec[clip_iter].cols(); col++) {
      for (int row = 0; row < weight_vec[clip_iter].rows(); row++) {
        if (weight_vec[clip_iter](col, row) <= 0)
          tile_vec[clip_iter](col, row) = out_nodata_value;
      }
    }

    weight_vec[clip_iter] = grassfire(notnodata(tile_vec[clip_iter],
                                                out_nodata_value),
                                      no_border_blend);
  }

  // Don't allow the weights to grow too fast, for uniqueness.
  for (size_t clip_iter = 0; clip_iter < weight_vec.size(); clip_iter++) {
    for (int col = 0; col < weight_vec[clip_iter].cols(); col++) {
      for (int row = 0; row < weight_vec[clip_iter].rows(); row++) {
        weight_vec[clip_iter](col, row)
          = std::min(weight_vec[clip_iter](col, row), double(bias));
      }
    }
  }

  // Blur the weights
  for (size_t clip_iter = 0; clip_iter < weight_vec.size(); clip_iter++)
    asp::blurWeights(weight_vec[clip_iter], weights_blur_sigma);

  // Raise to power
  if (weights_exp != 1) {
    for (size_t clip_iter = 0; clip_iter < weight_vec.size(); clip_iter++) {
      for (int col = 0; col < weight_vec[clip_iter].cols(); col++) {
        for (int row = 0; row < weight_vec[clip_iter].rows(); row++) {
          weight_vec[clip_iter](col, row)
            = pow(weight_vec[clip_iter](col, row), weights_exp);
        }
      }
    }
  }

  // Now we are ready for blending
  fill(tile, out_nodata_value);
  fill(weights, 0.0);

  if (save_dem_weight >= 0)
    fill(saved_weight, 0.0);

  for (size_t clip_iter = 0; clip_iter < weight_vec.size(); clip_iter++) {
    for (int col = 0; col < weight_vec[clip_iter].cols(); col++) {
      for (int row = 0; row < weight_vec[clip_iter].rows(); row++) {

        double wt = weight_vec[clip_iter](col, row);
        if (wt <= 0)
          continue; // nothing to do

        // Initialize the tile
        if (tile(col, row) == out_nodata_value)
          tile(col, row) = 0;

        tile(col, row)    += wt*tile_vec[clip_iter](col, row);
        weights(col, row) += wt;

        if (clip2dem_index[clip_iter] == save_dem_weight)
          saved_weight(col, row) = wt;
      }
    }
  }

  // Compute the weighted average
  for (int col = 0; col < tile.cols(); col++) {
    for (int row = 0; row < weights.rows(); row++) {
      if (weights(col, row) > 0)
        tile(col, row) /= weights(col, row);

      if (save_dem_weight >= 0 && weights(col, row) > 0)
        saved_weight(col, row) /= weights(col, row);

    }
  }

  // Dump the weights. Useful for debugging.
  bool save_weights = false;
  if (save_weights) {
    for (size_t clip_iter = 0; clip_iter < weight_vec.size(); clip_iter++) {
      vw::cartography::GeoReference crop_georef = crop(out_georef, bbox);
      std::ostringstream os;
      os << "tile_weight_" << clip_iter << ".tif";
      vw_out() << "\nWriting: " << os.str() << "\n";
      bool has_georef = true, has_nodata = true;
      vw::cartography::block_write_gdal_image(os.str(), weight_vec[clip_iter],
                  has_georef, crop_georef,
                  has_nodata, -100,
                  vw::GdalWriteOptions(),
                  TerminalProgressCallback("asp", ""));
    }
  }

  return;
}

// Process a DEM tile pixel by pixel. Take into account the various
// blending options. 
void processDemTile(asp::DemMosaicOptions const& opt,
                    BBox2i const& bbox,
                    Vector2 const& in_box_min,
                    ImageView<asp::DoubleGrayA> const& dem,
                    ImageViewRef<asp::DoubleGrayA> const& interp_dem,
                    vw::cartography::GeoTransform const& geotrans,
                    std::vector<ImageView<double>>& tile_vec,
                    bool noblend,
                    bool use_priority_blend,
                    int dem_iter,
                    double tol,
                    // Outputs
                    ImageView<double>& tile,
                    ImageView<double>& weights,
                    ImageView<double>& weight_modifier,
                    ImageView<double>& saved_weight,
                    ImageView<double>& index_map) {

  // Loop through each output pixel
  for (int c = 0; c < bbox.width(); c++) {
    for (int r = 0; r < bbox.height(); r++) {

      // Coordinates in the output mosaic
      vw::Vector2 out_pix(c + bbox.min().x(), r + bbox.min().y());
      // Coordinate in this input DEM
      vw::Vector2 in_pix = geotrans.reverse(out_pix);

      // Input DEM pixel relative to loaded bbox
      double x = in_pix[0] - in_box_min.x();
      double y = in_pix[1] - in_box_min.y();

      // Interpolate
      asp::DoubleGrayA pval = asp::interpDem(x, y, dem, interp_dem, tol, opt.propagate_nodata);

      // Separate the value and alpha for this pixel.
      double val = pval.v();
      double wt = pval.a();

      if (use_priority_blend) {
        // The priority blending, pixels from earlier DEMs at this location
        // are used unmodified unless close to that DEM boundary.
        wt = std::min(weight_modifier(c, r), wt);

        // Now ensure that the current DEM values will be used
        // unmodified unless close to the boundary for subsequent
        // DEMs. The weight w2 will be 0 well inside the DEM, and
        // increase towards the boundary.
        double wt2 = wt;
        wt2 = std::max(0.0, opt.priority_blending_len - wt2);
        weight_modifier(c, r) = std::min(weight_modifier(c, r), wt2);
      }

      // If point is in-bounds and nodata, make sure this point stays 
      //  at nodata even if other DEMs contain it.
      if (wt == 0 && opt.propagate_nodata) {
        tile(c, r) = 0;
        weights(c, r) = -1.0;
      }

      if (wt <= 0.0)
        continue; // No need to continue if the weight is zero

      // Check if the current output value at this pixel is nodata
      bool is_nodata = ((tile(c, r) == opt.out_nodata_value));

      // Initialize the tile if not done already.
      // Init to zero not needed with some types.
      if (!opt.stddev && !opt.median && !opt.nmad && !opt.min && !opt.max &&
          !use_priority_blend) {
        if (is_nodata) {
          tile(c, r) = 0;
          weights(c, r) = 0.0;
        }
      }

      // Update the output value according to the commanded mode
      if ((opt.first && is_nodata)                      ||
           opt.last                                     ||
           (opt.min && (val < tile(c, r) || is_nodata)) ||
           (opt.max && (val > tile(c, r) || is_nodata)) ||
           opt.median || opt.nmad ||
           use_priority_blend || opt.block_max) {
        // --> Conditions where we replace the current value
        tile(c, r) = val;
        weights(c, r) = wt;

        // In these cases, the saved weight will be 1 or 0, since either
        // a given DEM gives it all, or nothing at all.
        if (opt.save_dem_weight >= 0 && (opt.first || opt.last ||
                                         opt.min   || opt.max))
          saved_weight(c, r) = (opt.save_dem_weight == dem_iter);

        // In these cases, the saved weight will be 1 or 0, since either
        // a given DEM gives it all, or nothing at all.
        if (opt.save_index_map && (opt.first || opt.last ||
                                   opt.min   || opt.max))
          index_map(c, r) = dem_iter;

      } else if (opt.mean) { // Mean --> Accumulate the value
        tile(c, r) += val;
        weights(c, r)++;

        if (opt.save_dem_weight == dem_iter)
          saved_weight(c, r) = 1;

      } else if (opt.count) { // Increment the count
        tile(c, r)++;
        weights(c, r) += wt;
      } else if (opt.stddev) { // Standard deviation --> Keep running calculation
        weights(c, r) += 1.0;
        double curr_mean = tile_vec[0](c,r);
        double delta = val - curr_mean;
        curr_mean += delta / weights(c, r);
        double newVal = tile(c, r) + delta*(val - curr_mean);
        tile(c, r) = newVal;
        tile_vec[0](c,r) = curr_mean;
      } else if (!noblend) { // Blending --> Weighted average
        tile(c, r) += wt*val;
        weights(c, r) += wt;
        if (opt.save_dem_weight == dem_iter)
          saved_weight(c, r) = wt;
      }

    } // End col loop
  } // End row loop

  return;
}

void processMedianOrNmad(BBox2i const& bbox,
                         double out_nodata_value,
                         bool is_median,
                         bool save_index_map,
                         std::vector<int> const& clip2dem_index,
                         std::vector<ImageView<double>> const& tile_vec,
                         ImageView<double> & tile,
                         ImageView<double> & index_map) {

  // Init output pixels to nodata
  fill(tile, out_nodata_value);
  std::vector<double> vals, vals_all(tile_vec.size());
  // Iterate through all pixels
  for (int c = 0; c < bbox.width(); c++) {
    for (int r = 0; r < bbox.height(); r++) {
      // Compute the median for this pixel
      vals.clear();
      for (int i = 0; i < (int)tile_vec.size(); i++) {
        ImageView<double> const& tile_ref = tile_vec[i];
        double this_val = tile_ref(c, r);
        vals_all[i] = this_val; // Record the original order.
        if (this_val == out_nodata_value)
          continue;
        vals.push_back(this_val);
      }

      if (vals.empty())
        continue;
      if (is_median)
        tile(c, r) = math::destructive_median(vals);
      else
        tile(c, r) = math::destructive_nmad(vals);

      if (!save_index_map)
        continue;

      // Record the index of the image that is closest to the
      // median value.  Note that the median can average two
      // values, so the median value may not equal exactly any of
      // the input values.
      double min_dist = std::numeric_limits<double>::max();
      for (size_t m = 0; m < vals_all.size(); m++) {
        double dist = fabs(vals_all[m] - tile(c, r));
        if (dist < min_dist) {
          // Here we save the index not in the current array which
          // is m, but in the full list of DEMs, some of which are
          // likely skipped in this tile as they don't intersect
          // it.
          index_map(c, r) = clip2dem_index[m];
          min_dist = dist;
        }
      }

    }// End row loop
  } // End col loop

  return;
}

/// Class that does the actual image processing work
class DemMosaicView: public ImageViewBase<DemMosaicView> {
  int m_cols, m_rows, m_bias;
  asp::DemMosaicOptions  const& m_opt; // alias
  DiskImageManager<float> & m_imgMgr; // alias
  std::vector<vw::cartography::GeoReference> const& m_georefs; // alias
  vw::cartography::GeoReference m_out_georef;
  std::vector<double> const& m_nodata_values; // alias
  std::vector<vw::BBox2i> const& m_dem_pixel_bboxes; // alias
  long long int & m_num_valid_pixels; // alias, to populate on output
  vw::Mutex & m_count_mutex; // alias, a lock for m_num_valid_pixels

public:
  DemMosaicView(int cols, int rows, int bias,
                asp::DemMosaicOptions     const& opt,
                DiskImageManager<float>        & imgMgr,
                std::vector<vw::cartography::GeoReference> const& georefs,
                vw::cartography::GeoReference              const& out_georef,
                std::vector<double>       const& nodata_values,
                std::vector<BBox2i>       const& dem_pixel_bboxes,
                long long int                  & num_valid_pixels,
                vw::Mutex                      & count_mutex):
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
          != m_out_georef.datum().meridian_offset()) {
        vw_throw(NoImplErr() << "Mosaicking of DEMs with differing datum radii "
                 << " or meridian offsets is not implemented. Datums encountered:\n"
                 << m_georefs[i].datum() << "\n"
                 <<  m_out_georef.datum() << "\n");
      }
      if (m_georefs[i].datum().name() != m_out_georef.datum().name() &&
          this_major_axis == out_major_axis &&
          this_minor_axis == out_minor_axis &&
          m_georefs[i].datum().meridian_offset() == m_out_georef.datum().meridian_offset()) {
        vw_out(WarningMessage) << "Found DEMs with the same radii and meridian offsets, "
                               << "but different names: "
                               << m_georefs[i].datum().name() << " and "
                               << m_out_georef.datum().name() << "\n";
      }
    }
  }

  // Boilerplate
  typedef float      pixel_type;
  typedef pixel_type result_type;
  typedef ProceduralPixelAccessor<DemMosaicView> pixel_accessor;
  inline int cols  () const { return m_cols; }
  inline int rows  () const { return m_rows; }
  inline int planes() const { return 1; }
  inline pixel_accessor origin() const { return pixel_accessor(*this, 0, 0); }

  inline pixel_type operator()(double/*i*/, double/*j*/, int/*p*/ = 0) const {
    vw_throw(NoImplErr() << "DemMosaicView::operator()(...) is not implemented");
    return pixel_type();
  }

  typedef CropView<ImageView<pixel_type> > prerasterize_type;
  inline prerasterize_type prerasterize(BBox2i bbox) const {

    BBox2i orig_box = bbox;

    // Get a shorthand for this
    const bool use_priority_blend = (m_opt.priority_blending_len > 0);

    // When doing priority blending, we will do all the work in the
    // output pixels domain. Hence we need to take into account the
    // bias here rather than later.
    if (use_priority_blend)
      bbox.expand(m_bias + BilinearInterpolation::pixel_buffer + 1);

    // We will do all computations in double precision, regardless
    // of the precision of the inputs, for increased accuracy.
    // - The image data buffers are initialized here
    ImageView<double> tile   (bbox.width(), bbox.height()); // the output tile (in most cases)
    ImageView<double> weights(bbox.width(), bbox.height()); // weights (in most cases)
    fill(tile, m_opt.out_nodata_value);
    fill(weights, 0.0);

    // True if we won't be doing any DEM blending.
    bool noblend = (numNoBlendOptions(m_opt) > 0);

    // A vector of tiles, each of the size of the output tile, 
    // is for median, nmad, and stddev calculation.
    int num_images = m_imgMgr.size();
    std::vector<ImageView<double>> tile_vec, weight_vec;
    std::vector<std::string> dem_vec;
    initializeTileVector(num_images, bbox, m_opt, tile, tile_vec, weight_vec);

    // This will ensure that pixels from earlier images are
    // mostly used unmodified except being blended at the boundary.
    vw::ImageView<double> weight_modifier;
    if (use_priority_blend) {
      weight_modifier = vw::ImageView<double>(bbox.width(), bbox.height());
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

      // Sanity check: the output no-data value must not equal to
      // any of the indices in the map, as then the two cannot be
      // distinguished.
      for (int dem_iter = 0; dem_iter < (int)m_imgMgr.size(); dem_iter++) {
        if (dem_iter == m_opt.out_nodata_value)
          vw_throw(ArgumentErr() << "Cannot have the output no-data value equal to "
                   << m_opt.out_nodata_value
                   << " as this is one of the indices being saved in the index map.\n");
      }
    }

    ImageView<double> first_dem;

    // Loop through all input DEMs
    for (int dem_iter = 0; dem_iter < (int)m_imgMgr.size(); dem_iter++) {

      // Load the information for this DEM
      vw::cartography::GeoReference georef        = m_georefs         [dem_iter];
      BBox2i       dem_pixel_box = m_dem_pixel_bboxes[dem_iter];

      // The vw::cartography::GeoTransform will hide the messy details of conversions
      // from pixels to points and lon-lat.
      vw::cartography::GeoTransform geotrans(georef, m_out_georef, dem_pixel_box, bbox);

      // Get the tile bbox in the frame of the current input DEM
      BBox2 in_box = geotrans.reverse_bbox(bbox);

      // Grow to account for blending and erosion length, etc.  If
      // priority blending length was positive, we've already expanded 'bbox'.
      if (!use_priority_blend)
        in_box.expand(m_bias + BilinearInterpolation::pixel_buffer + 1);

      in_box.crop(dem_pixel_box);

      if (in_box.width() == 1 || in_box.height() == 1) {
        // Grassfire likes to have width of at least 2
        in_box.expand(1);
        in_box.crop(dem_pixel_box);
      }
      if (in_box.width() <= 1 || in_box.height() <= 1)
        continue; // No overlap with this tile, skip to the next DEM.

      if (m_opt.median || m_opt.nmad || use_priority_blend || m_opt.block_max) {
        // Must use a blank tile each time
        fill(tile, m_opt.out_nodata_value);
        fill(weights, 0.0);
      }

      // Crop the disk dem to a 2-channel in-memory image. First
      // channel is the image pixels, second will be the weights.
      ImageViewRef<double> disk_dem = pixel_cast<double>(m_imgMgr.get_handle(dem_iter, bbox));
      ImageView<asp::DoubleGrayA> dem    = crop(disk_dem, in_box);

      if (m_opt.first_dem_as_reference && dem_iter == 0) {
        // We need to keep the first DEM, to use it as ref
        // when merging in the blended DEM
        first_dem = crop(disk_dem, bbox);
      }

      std::string dem_name = m_imgMgr.get_file_name(dem_iter);

      // If the nodata_threshold is specified, all values no more than this
      // will be invalidated.
      // TODO(oalexan1): This must be a function
      double nodata_value = m_nodata_values[dem_iter];
      if (!std::isnan(m_opt.nodata_threshold)) {
        nodata_value = m_opt.nodata_threshold;
        for (int col = 0; col < dem.cols(); col++) {
          for (int row = 0; row < dem.rows(); row++) {
            if (dem(col, row)[0] <= nodata_value) {
              dem(col, row)[0] = nodata_value;
            }
          }
        }
      }

      // NaN values get set to no-data. This is a bugfix.
      for (int col = 0; col < dem.cols(); col++) {
        for (int row = 0; row < dem.rows(); row++) {
          if (std::isnan(dem(col, row)[0])) {
            dem(col, row)[0] = nodata_value;
          }
        }
      }

      if (m_opt.first_dem_as_reference && dem_iter == 0) {
        //TODO: Should be a function!
        // Convert to the output nodata value
        for (int col = 0; col < first_dem.cols(); col++) {
          for (int row = 0; row < first_dem.rows(); row++) {
            if (first_dem(col, row) == nodata_value || std::isnan(first_dem(col, row)))
              first_dem(col, row) = m_opt.out_nodata_value;
          }
        }
      }

      // Fill holes. This happens here, in the expanded tile, to
      // ensure we catch holes which are partially outside the tile
      // being processed.
      if (m_opt.hole_fill_len > 0)
        dem = apply_mask(vw::fill_holes_grass
                         (create_mask(select_channel(dem, 0), nodata_value),
                          m_opt.hole_fill_len),
                         nodata_value);

      // Fill nodata based on radius. There is a sanity check that ensures we don't
      // do both this and the hole filling above.
      if (m_opt.fill_search_radius > 0.0) {
        dem = apply_mask(fillNodataWithSearchRadius
        (create_mask(select_channel(dem, 0), nodata_value),
          m_opt.fill_search_radius, m_opt.fill_power, m_opt.fill_percent,
          m_opt.fill_num_passes),
          nodata_value);
      }

      // Fill-in no-data values a bit and blur. If just the blurring is used,
      // it will choke on no-data values, leaving large holes around each,
      // hence the need to fill a little.
      if (m_opt.dem_blur_sigma > 0.0) {
        int kernel_size = vw::compute_kernel_size(m_opt.dem_blur_sigma);
        dem = apply_mask(gaussian_filter(fill_nodata_with_avg
                                         (create_mask(select_channel(dem, 0), nodata_value),
                                          kernel_size),
                                         m_opt.dem_blur_sigma), nodata_value);
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
      ImageView<double> local_wts
       = grassfire(notnodata(select_channel(dem, 0), nodata_value),
                   m_opt.no_border_blend);

      if (m_opt.use_centerline_weights) {
        // Erode based on grassfire weights, and then overwrite the grassfire
        // weights with centerline weights
        ImageView<asp::DoubleGrayA> dem2 = copy(dem);
        for (int col = 0; col < dem2.cols(); col++) {
          for (int row = 0; row < dem2.rows(); row++) {
            if (local_wts(col, row) <= m_opt.erode_len) {
              dem2(col, row) = asp::DoubleGrayA(nodata_value);
            }
          }
        }
        ImageView<PixelMask<double>> mask_img =
          create_mask_less_or_equal(select_channel(dem2, 0), nodata_value);
        asp::centerlineWeightsWithHoles(mask_img, local_wts, m_bias, -1.0);

      } // End centerline weights case

      // If we don't limit the weights from above, we will have tiling artifacts,
      // as in different tiles the weights grow to different heights since
      // they are cropped to different regions. For priority blending length,
      // we'll do this process later, as the bbox is obtained differently in that case.
      // With centerline weights, that is handled before 1D weights are multiplied
      // to get the 2D weights.
      if (!use_priority_blend && !m_opt.use_centerline_weights) {
        for (int col = 0; col < local_wts.cols(); col++) {
          for (int row = 0; row < local_wts.rows(); row++) {
            local_wts(col, row) = std::min(local_wts(col, row), double(m_bias));
          }
        }
      }

      // Erode. We already did that if centerline weights are used.
      if (!m_opt.use_centerline_weights) {
        int max_cutoff = max_pixel_value(local_wts);
        int min_cutoff = m_opt.erode_len;
        if (max_cutoff <= min_cutoff)
          max_cutoff = min_cutoff + 1; // precaution
        local_wts = clamp(local_wts - min_cutoff, 0.0, max_cutoff - min_cutoff);
      }

      // Blur the weights. If priority blending length is on, we'll do the blur later,
      // after weights from different DEMs are combined.
      if (m_opt.weights_blur_sigma > 0 && !use_priority_blend)
        asp::blurWeights(local_wts, m_opt.weights_blur_sigma);

      // Raise to the power. Note that when priority blending length is positive, we
      // delay this process.
      // TODO(oalexan1): This must be a function
      if (m_opt.weights_exp != 1 && !use_priority_blend) {
        for (int col = 0; col < dem.cols(); col++) {
          for (int row = 0; row < dem.rows(); row++) {
            if (local_wts(col, row) > 0)
              local_wts(col, row) = pow(local_wts(col, row), m_opt.weights_exp);
          }
        }
      }

#if 0
      // Save the weights with georeference. Very useful for debugging
      // non-uniqueness issues across tiles.
      std::ostringstream os;
      os << "weights_" << dem_iter << "_" << bbox.min().x() << "_" << bbox.min().y()
         << ".tif";
      vw_out() << "Writing: " << os.str() << "\n";
      bool has_georef = true, has_nodata = true;
      vw::cartography::block_write_gdal_image(os.str(), local_wts,
                 has_georef, georef,
                 has_nodata, -100,
                 vw::GdalWriteOptions(),
                 TerminalProgressCallback("asp", ""));
#endif

      // Apply external weights
      if (!m_opt.weight_files.empty())
        asp::applyExternalWeights(m_opt.weight_files[dem_iter], m_opt.min_weight,
                                  m_opt.invert_weights, in_box, local_wts);

      // Set the weights in the alpha channel
      for (int col = 0; col < dem.cols(); col++) {
        for (int row = 0; row < dem.rows(); row++) {
          dem(col, row).a() = local_wts(col, row);
        }
      }

      // Prepare the DEM for interpolation
      ImageViewRef<asp::DoubleGrayA> interp_dem
        = interpolate(dem, BilinearInterpolation(), ConstantEdgeExtension());

      // Process the DEM tile pixel by pixel
      processDemTile(m_opt, bbox, in_box.min(), dem, interp_dem, geotrans,
                     tile_vec, noblend, use_priority_blend, dem_iter, g_tol,
                     // Outputs
                     tile, weights, weight_modifier, saved_weight, index_map);

      // For the median option, keep a copy of the output tile for each input DEM.
      // Also do it for max per block. This will be memory intensive. 
      if (m_opt.median || m_opt.nmad || m_opt.block_max) {
        tile_vec.push_back(copy(tile));
        dem_vec.push_back(dem_name);
      }

      // For priority blending, need also to keep all tiles, but also the weights
      if (use_priority_blend) {
        tile_vec.push_back(copy(tile));
        weight_vec.push_back(copy(weights));
      }

      if (use_priority_blend || m_opt.save_index_map)
        clip2dem_index.push_back(dem_iter);

    } // End iterating over DEMs

    // Divide by the weights in blend, mean
    if (!noblend || m_opt.mean) {
      for (int c = 0; c < bbox.width(); c++) { // Iterate over all pixels!
        for (int r = 0; r < bbox.height(); r++) {
          if (weights(c, r) > 0)
            tile(c, r) /= weights(c, r);
        } // End row loop
      } // End col loop
    } // End dividing case

    // Finish stddev calculations
    if (m_opt.stddev) {
      for (int c = 0; c < bbox.width(); c++) { // Iterate over all pixels!
        for (int r = 0; r < bbox.height(); r++) {

          if (weights(c, r) > 1.0) {
            tile(c, r) = sqrt(tile(c, r) / (weights(c, r) - 1.0));
          } else { // Invalid pixel
            tile(c, r) = m_opt.out_nodata_value;
          }
        } // End row loop
      } // End col loop
    } // End stddev case

    // Median and nmad operations
    if (m_opt.median || m_opt.nmad)
      processMedianOrNmad(bbox, m_opt.out_nodata_value, m_opt.median, m_opt.save_index_map,
                          clip2dem_index, tile_vec,
                          // Outputs
                          tile, index_map);

    // For max per block, find the sum of values in each DEM
    // TODO(oalexan1): This must be a function
    if (m_opt.block_max) {
      fill(tile, m_opt.out_nodata_value);
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
        // The whole purpose of --block-max is to print the sum of
        // pixels for each mapprojected image/DEM when doing SfS.
        // The documentation has a longer explanation.
        vw_out() << "\n" << bbox << " " << dem_vec[i]
                 << " pixel sum: " << tile_sum[i] << "\n";
      }
      int max_index = std::distance(tile_sum.begin(),
                                    std::max_element(tile_sum.begin(), tile_sum.end()));
      if (max_index >= 0 && max_index < num_tiles)
        tile = copy(tile_vec[max_index]);
    }

    // For priority blending length, use the weights created so far only to burn holes in
    // the DEMs where we don't want blending, then recreate the weights
    if (use_priority_blend)
      priorityBlend(m_opt.out_nodata_value, m_bias, m_opt.weights_blur_sigma,
                    m_opt.weights_exp, m_opt.no_border_blend, m_opt.save_dem_weight,
                    clip2dem_index, m_out_georef, bbox,
                    // Outputs
                    tile_vec, weight_vec, tile, weights, saved_weight);

    // Save the weight instead
    if (m_opt.save_dem_weight >= 0)
      tile = saved_weight;

    // Save the index map instead
    if (m_opt.save_index_map)
      tile = index_map;

    // How many valid pixels are there in the tile
    // TODO(oalexan1): This must be a function
    long long int num_valid_in_tile = 0; // use int64 to not overflow for large images
    for (int col = 0; col < tile.cols(); col++) {
      for (int row = 0; row < tile.rows(); row++) {
        vw::Vector2 pix = vw::Vector2(col, row) + bbox.min();
        if (!orig_box.contains(pix))
          continue; // in case the box got expanded, ignore the padding
        if (tile(col, row) == m_opt.out_nodata_value)
          continue;
        num_valid_in_tile++;
      }
    }
    {
      // Lock and update the total number of valid pixels
      vw::Mutex::Lock lock(m_count_mutex);
      m_num_valid_pixels += num_valid_in_tile;
    }

    if (m_opt.first_dem_as_reference) {

      // TODO(oalexan1): This must be a function
      if (first_dem.cols() != tile.cols() || first_dem.rows() != tile.rows())
        vw_throw(ArgumentErr() << "Book-keeping error when blending into first DEM.\n");

      // Wipe from the tile all values outside the perimeter of
      // first_dem. So we don't wipe values that happen to be
      // in the holes of first_dem.
      // TODO(oalexan1): How about using here the function centerlineWeightsWithHoles()?
      vw::ImageView<double> local_wts;
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
    // on doubles, here we cast to float.
    return prerasterize_type(pixel_cast<float>(tile),
                             -bbox.min().x(), -bbox.min().y(),
                             cols(), rows());
  } // end function prerasterize

  template <class DestT>
  inline void rasterize(DestT const& dest, BBox2i bbox) const {
    vw::rasterize(prerasterize(bbox), dest, bbox);
  }
}; // End class DemMosaicView

/// Find the bounding box of all DEMs in the projected space.
/// - mosaic_bbox is the output bounding box in projected space
/// - dem_proj_bboxes and dem_pixel_bboxes are the locations of
///   each input DEM in the output DEM in projected and pixel coordinates.
void load_dem_bounding_boxes(asp::DemMosaicOptions       const& opt,
                             vw::cartography::GeoReference  const& mosaic_georef,
                             BBox2              & mosaic_bbox, // Projected coordinates
                             std::vector<BBox2> & dem_proj_bboxes,
                             std::vector<BBox2i> & dem_pixel_bboxes) {

  vw_out() << "Determining the bounding boxes of the inputs.\n";

  // Initialize the outputs
  mosaic_bbox = BBox2();
  dem_proj_bboxes.clear();
  dem_pixel_bboxes.clear();

  TerminalProgressCallback tpc("", "\t--> ");
  tpc.report_progress(0);
  double inc_amount = 1.0 / double(opt.dem_files.size());

  BBox2 first_dem_proj_box;

  // Loop through all DEMs
  for (int dem_iter = 0; dem_iter < (int)opt.dem_files.size(); dem_iter++) {

    // Open a handle to this DEM file
    DiskImageResourceGDAL in_rsrc(opt.dem_files[dem_iter]);
    DiskImageView<float>  img(opt.dem_files[dem_iter]);
    vw::cartography::GeoReference          georef = asp::readGeorefOrThrow(opt.dem_files[dem_iter]);
    BBox2i                pixel_box = bounding_box(img);

    dem_pixel_bboxes.push_back(pixel_box);

    if (dem_iter == 0)
      first_dem_proj_box = georef.bounding_box(img);

    bool has_lonat = (georef.proj4_str().find("+proj=longlat") != std::string::npos ||
                      mosaic_georef.proj4_str().find("+proj=longlat") != std::string::npos);

    // Compute bounding box of this DEM. The simple case is when all DEMs have
    // the same projection, and it is not longlat, as then we need to worry about
    // a 360 degree shift.
    if ((!has_lonat) && mosaic_georef.get_wkt() == georef.get_wkt()) {
      BBox2 proj_box = georef.bounding_box(img);
      mosaic_bbox.grow(proj_box);
      dem_proj_bboxes.push_back(proj_box);
    } else {
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
      } else {
        mosaic_pixel_box = mosaic_georef.point_to_pixel_bbox(mosaic_bbox);
      }

      vw::cartography::GeoTransform geotrans(georef, mosaic_georef, imgbox, mosaic_pixel_box);
      proj_box = geotrans.pixel_to_point_bbox(imgbox);

      mosaic_bbox.grow(proj_box);
      dem_proj_bboxes.push_back(proj_box);
    } // End second case

    tpc.report_incremental_progress(inc_amount);
  } // End loop through DEM files
  tpc.report_finished();

  // If the first dem is used as reference, no matter what use its own box
  if (opt.first_dem_as_reference)
    mosaic_bbox = first_dem_proj_box;

} // End function load_dem_bounding_boxes

int main(int argc, char *argv[]) {

  asp::DemMosaicOptions opt;

  try {

    asp::handleDemMosaicArgs(argc, argv, opt);

    // TODO: Fix here. If the DEM is double, read the nodata as double,
    // without casting to float. If it is float, cast to float.

    // Read nodata from first DEM, unless the user chooses to specify it.
    if (!opt.has_out_nodata) {
      DiskImageResourceGDAL in_rsrc(opt.dem_files[0]);
      // Since the DEMs have float pixels, we must read the no-data as
      // float as well. (this is a bug fix). Yet we store it in a
      // double, as we will cast the DEM pixels to double as well.
      if (in_rsrc.has_nodata_read())
        opt.out_nodata_value = float(in_rsrc.nodata_read());
    }

    // Watch for underflow, if mixing doubles and float. Particularly problematic
    // is when the nodata_value cannot be represented exactly as a float.
    if (opt.out_nodata_value < static_cast<double>(-std::numeric_limits<float>::max()) ||
        float(opt.out_nodata_value)  != double(opt.out_nodata_value)) {
      vw_out() << "The no-data value cannot be represented exactly as a float. "
           << "Changing it to the smallest float.\n";
      opt.out_nodata_value = static_cast<double>(-std::numeric_limits<float>::max());
    }

    vw_out() << "Using output no-data value: " << opt.out_nodata_value << "\n";

    // Form the mosaic georef. The georef of the first DEM is used as
    // initial guess unless user wants to change the resolution and projection.

    // By default the output georef is equal to the first input georef
    vw::cartography::GeoReference mosaic_georef = asp::readGeorefOrThrow(opt.dem_files[0]);

    double spacing = opt.tr;
    if (opt.target_srs_string != "" && spacing <= 0) {
        vw_throw(ArgumentErr()
           << "Changing the projection was requested. The output DEM "
           << "resolution must be specified via the --tr option.\n");
    }

    if (opt.target_srs_string != "") {
      // Set the srs string into georef.
      bool have_user_datum = false;
      vw::cartography::Datum user_datum;
      vw::cartography::set_srs_string(opt.target_srs_string, have_user_datum,
                                      user_datum, mosaic_georef);
    }

    // Use desired spacing if user-specified
    if (spacing > 0.0) {
      // Get lonlat bounding box of the first DEM.
      DiskImageView<float> dem0(opt.dem_files[0]);
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
      vw::Vector2 ul = mosaic_georef.lonlat_to_pixel(Vector2(llbox0.min().x(),
                                                             llbox0.max().y()));
      mosaic_georef = crop(mosaic_georef, ul.x(), ul.y());
    } else {
      // Update spacing variable from the current transform
      spacing = mosaic_georef.transform()(0, 0);
    if (spacing <= 0)
      vw_throw(ArgumentErr() << "The output grid size must be positive.\n");
    }

    if (opt.gdal_tap) {
      // A first adjustment, will refine later
      auto T = mosaic_georef.transform();
      T(0, 2) = spacing * round(T(0, 2)/spacing);
      T(1, 2) = spacing * round(T(1, 2)/spacing);
      mosaic_georef.set_transform(T);
    }

    // if the user specified the tile size in georeferenced units.
    if (opt.geo_tile_size > 0) {
      opt.tile_size = (int)round(opt.geo_tile_size/spacing);
      vw_out() << "Tile size in pixels: " << opt.tile_size << "\n";
    }
    opt.tile_size = std::max(opt.tile_size, 1);

    // Load the bounding boxes from all of the DEMs
    vw::BBox2 mosaic_bbox;
    std::vector<BBox2> dem_proj_bboxes;
    std::vector<BBox2i> dem_pixel_bboxes, loaded_dem_pixel_bboxes;
    load_dem_bounding_boxes(opt, mosaic_georef, mosaic_bbox,
                            dem_proj_bboxes, dem_pixel_bboxes);

    if (opt.tap) {
      // Ensure that the grid is at integer multiples of grid size
      mosaic_bbox.min() = spacing * floor(mosaic_bbox.min() / spacing);
      mosaic_bbox.max() = spacing * ceil(mosaic_bbox.max()  / spacing);
      if (opt.projwin != BBox2()) {
        opt.projwin.min() = spacing * floor(opt.projwin.min() / spacing);
        opt.projwin.max() = spacing * ceil(opt.projwin.max()  / spacing);
      }
    }

    if (opt.gdal_tap) {
      // Conform to GDAL -tap. Ensure that the output bounds are at integer
      // multiples of grid size.
      mosaic_bbox.min() = spacing * round(mosaic_bbox.min() / spacing);
      mosaic_bbox.max() = spacing * round(mosaic_bbox.max()  / spacing);
      if (opt.projwin != BBox2()) {
        opt.projwin.min() = spacing * round(opt.projwin.min() / spacing);
        opt.projwin.max() = spacing * round(opt.projwin.max()  / spacing);
      }
    }

    if (opt.projwin != BBox2()) {
      // If to create the mosaic only in a given region
      if (!opt.gdal_tap)
        mosaic_bbox.crop(opt.projwin);
      else
        mosaic_bbox = opt.projwin; // GDAL --tap overrides the mosaic box

      // Crop the proj boxes as well
      for (int dem_iter = 0; dem_iter < (int)opt.dem_files.size(); dem_iter++)
        dem_proj_bboxes[dem_iter].crop(opt.projwin);
      if (opt.force_projwin)
        mosaic_bbox = opt.projwin;
    }

    if (opt.gdal_tap) {
       // Further adjustment of the georef. By now mosaic_bbox incorporates
       // --projwin if given. The corners of mosaic_bbox are multiples of spacing.
       auto T = mosaic_georef.transform();
       if (T(0, 0) <= 0)
         vw::vw_throw(vw::ArgumentErr() << "The output grid size in x must be positive.\n");
       if (T(1, 1) >= 0)
         vw::vw_throw(vw::ArgumentErr() << "The output grid size in y must be negative.\n");
       T(0, 2) = mosaic_bbox.min().x();
       T(1, 2) = mosaic_bbox.max().y();
       mosaic_georef.set_transform(T);
    }

    // First-guess pixel box
    BBox2 pixel_box = asp::pointToPixelBboxSnapped(mosaic_georef, mosaic_bbox, g_tol);

    // Take care of numerical artifacts
    vw::Vector2 beg_pix = pixel_box.min();
    if (norm_2(beg_pix - round(beg_pix)) < g_tol)
      beg_pix = round(beg_pix);
    mosaic_georef = crop(mosaic_georef, beg_pix[0], beg_pix[1]);

    // Image size
    pixel_box = asp::pointToPixelBboxSnapped(mosaic_georef, mosaic_bbox, g_tol);
    vw::Vector2 end_pix = pixel_box.max();
    int cols = (int)round(end_pix[0]); // end_pix is the last pix in the image
    int rows = (int)round(end_pix[1]);

    // Form the mosaic and write it to disk
    vw_out() << "Mosaic size: " << cols << " x " << rows << " pixels.\n";

    // This bias is very important. This is how much we should read from
    // the images beyond the current boundary to avoid tiling artifacts.
    // The +1 is to ensure extra pixels beyond the hole fill length.
    int bias = opt.erode_len + opt.extra_crop_len + opt.hole_fill_len
      + opt.fill_search_radius
      + 2*std::max(vw::compute_kernel_size(opt.weights_blur_sigma),
                   vw::compute_kernel_size(opt.dem_blur_sigma))
                   + 1;

    // If we just fill holes based on search radius, we do not need a large bias.
    // Filling with large search radius is slow as it is.
    if (opt.fill_search_radius > 0)
      bias = opt.fill_search_radius + 10;

    // The next power of 2 >= 4*bias. We want to make the blocks big,
    // to reduce overhead from this bias, but not so big that it may
    // not fit in memory.
    int block_size = nextpow2(4.0*bias);
    block_size = std::max(block_size, 256); // don't make them too small though
    if (opt.block_size > 0)
      block_size = opt.block_size;

    // The block size must be a multiple of 16
    if (block_size % 16 != 0)
      vw::vw_throw(vw::ArgumentErr() << "The block size must be a multiple of 16.\n");

    // See if to lump all mosaic in just a given file, rather than creating tiles.
    bool write_to_precise_file = (opt.out_prefix.size() >= 4 &&
                   opt.out_prefix.substr(opt.out_prefix.size()-4, 4) == ".tif");

    int num_tiles_x = (int)ceil((double)cols/double(opt.tile_size));
    int num_tiles_y = (int)ceil((double)rows/double(opt.tile_size));
    if (num_tiles_x <= 0) num_tiles_x = 1;
    if (num_tiles_y <= 0) num_tiles_y = 1;
    int num_tiles = num_tiles_x*num_tiles_y;
    vw_out() << "Number of tiles: " << num_tiles_x << " x "
             << num_tiles_y << " = " << num_tiles << ".\n";

    if (opt.tile_index >= num_tiles) {
      vw_out() << "Tile with index: " << opt.tile_index
                 << " is out of bounds." << "\n";
      return 0;
    }

    if (num_tiles > 1 && write_to_precise_file)
      vw_throw(ArgumentErr() << "Cannot fit all mosaic in the given output file name. "
           << "Hence specify an output prefix instead, and then multiple "
           << "tiles will be created.\n");

    // If to use a range
    if (!opt.tile_list.empty() && opt.tile_index >= 0)
      vw_throw(ArgumentErr() << "Cannot specify both tile index and tile range.\n");

    // See if to save all tiles, or an individual tile.
    int start_tile = opt.tile_index, end_tile = opt.tile_index + 1;
    if (opt.tile_index < 0) {
      start_tile = 0;
      end_tile = num_tiles;
    }

    // Compute the bounding box of each output tile
    std::vector<BBox2i> tile_pixel_bboxes;
    for (int tile_id = start_tile; tile_id < end_tile; tile_id++) {

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
    std::vector<double>       nodata_values;
    std::vector<vw::cartography::GeoReference> georefs;
    std::vector<std::string>  loaded_dems;
    DiskImageManager<float>   imgMgr;

    BBox2i output_dem_box = BBox2i(0, 0, cols, rows); // output DEM box

    // Loop through all DEMs
    for (int dem_iter = 0; dem_iter < (int)opt.dem_files.size(); dem_iter++) {

      // Get the DEM bounding box that we previously computed (output projected coords)
      BBox2 dem_bbox = dem_proj_bboxes[dem_iter];

      // Go through each of the tile bounding boxes and see they intersect this DEM
      bool use_this_dem = false;
      for (int tile_id = start_tile; tile_id < end_tile; tile_id++) {

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

      // The vw::cartography::GeoTransform will hide the messy details of conversions
      // from pixels to points and lon-lat.
      vw::cartography::GeoReference georef  = asp::readGeorefOrThrow(opt.dem_files[dem_iter]);
      BBox2i dem_pixel_box = dem_pixel_bboxes[dem_iter];
      vw::cartography::GeoTransform geotrans(georef, mosaic_georef, dem_pixel_box, output_dem_box);

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
        if (in_rsrc.has_nodata_read())
          curr_nodata_value = float(in_rsrc.nodata_read());
      } catch(std::exception const& e) {
        // Try again
        imgMgr.freeup_handles_not_thread_safe();
        DiskImageResourceGDAL in_rsrc(opt.dem_files[dem_iter]);
        if (in_rsrc.has_nodata_read())
          curr_nodata_value = float(in_rsrc.nodata_read());
      }

      loaded_dems.push_back(opt.dem_files[dem_iter]);

      if (!std::isnan(opt.nodata_threshold))
        curr_nodata_value = opt.nodata_threshold;

      // Add the info for this DEM to the appropriate vectors
      nodata_values.push_back(curr_nodata_value);
      georefs.push_back(georef);
      loaded_dem_pixel_bboxes.push_back(dem_pixel_box);
    } // End loop through DEM files

    // If there are 17 tiles, let them be tile-00, ..., tile-16.
    int num_digits = 1;
    int tens = 10;
    while (num_tiles - 1 >= tens) {
      num_digits++;
      tens *= 10;
    }

    // Time to generate each of the output tiles
    for (int tile_id = start_tile; tile_id < end_tile; tile_id++) {

      if (!opt.tile_list.empty() && opt.tile_list.find(tile_id) == opt.tile_list.end())
        continue;

      // Get the bounding box we previously computed
      vw::BBox2i tile_box = tile_pixel_bboxes[tile_id - start_tile];

      std::string dem_tile;
      if (!write_to_precise_file) {
        std::ostringstream os;
        os << opt.out_prefix << "-tile-"
            << std::setfill('0') << std::setw(num_digits) << tile_id
            << tile_suffix(opt) << ".tif";
        dem_tile = os.str();
      } else {
        dem_tile = opt.out_prefix; // the file name was set by user
      }

      // Set up tile image and metadata
      long long int num_valid_pixels; // Will be populated when saving to disk
      vw::Mutex count_mutex; // to lock when updating num_valid_pixels

      ImageViewRef<float> out_dem
        = crop(DemMosaicView(cols, rows, bias, opt,
                             imgMgr, georefs,
                             mosaic_georef, nodata_values,
                             loaded_dem_pixel_bboxes,
                             num_valid_pixels, count_mutex),
               tile_box);
      vw::cartography::GeoReference crop_georef = crop(mosaic_georef, tile_box.min().x(),
                      tile_box.min().y());
      // Update the lon-lat box given that we know the final georef and image size
      crop_georef.ll_box_from_pix_box(BBox2i(0, 0, cols, rows));

      // Raster the tile to disk. Optionally cast to int (may be
      // useful for mosaicking ortho images).
      vw_out() << "Writing: " << dem_tile << "\n";
      bool has_georef = true, has_nodata = true;
      TerminalProgressCallback tpc("asp", "\t--> ");
      if (opt.output_type == "Float32")
        asp::save_with_temp_big_blocks(block_size, dem_tile, out_dem,
                                       has_georef, crop_georef,
                                       has_nodata, opt.out_nodata_value, opt, tpc);
      else if (opt.output_type == "Byte")
        asp::save_with_temp_big_blocks(block_size, dem_tile,
                       per_pixel_filter(out_dem, RoundAndClamp<uint8, float>()),
                                       has_georef, crop_georef,
                                       has_nodata, vw::round_and_clamp<uint8>(opt.out_nodata_value),
                                       opt, tpc);
      else if (opt.output_type == "UInt16")
        asp::save_with_temp_big_blocks(block_size, dem_tile,
                       per_pixel_filter(out_dem, RoundAndClamp<uint16, float>()),
                                       has_georef, crop_georef,
                                       has_nodata, vw::round_and_clamp<uint16>(opt.out_nodata_value),
                                       opt, tpc);
      else if (opt.output_type == "Int16")
        asp::save_with_temp_big_blocks(block_size, dem_tile,
                       per_pixel_filter(out_dem, RoundAndClamp<int16, float>()),
                                       has_georef, crop_georef,
                                       has_nodata, vw::round_and_clamp<int16>(opt.out_nodata_value),
                                       opt, tpc);
      else if (opt.output_type == "UInt32")
        asp::save_with_temp_big_blocks(block_size, dem_tile,
                       per_pixel_filter(out_dem, RoundAndClamp<uint32, float>()),
                                       has_georef, crop_georef,
                                       has_nodata, vw::round_and_clamp<uint32>(opt.out_nodata_value),
                                       opt, tpc);
      else if (opt.output_type == "Int32")
        asp::save_with_temp_big_blocks(block_size, dem_tile,
                       per_pixel_filter(out_dem, RoundAndClamp<int32, float>()),
                                       has_georef, crop_georef,
                                       has_nodata, vw::round_and_clamp<int32>(opt.out_nodata_value),
                                       opt, tpc);
      else
        vw_throw(NoImplErr() << "Unsupported output type: " << opt.output_type << ".\n");

      vw_out() << "Number of valid (not no-data) pixels written: " << num_valid_pixels
               << ".\n";
      if (num_valid_pixels == 0) {
        vw_out() << "Removing tile with no valid pixels: " << dem_tile << "\n";
        boost::filesystem::remove(dem_tile);
      }

    } // End loop through tiles

    // Write the name of each DEM file that was used together with its index
    if (opt.save_index_map) {
      std::string index_map = opt.out_prefix + "-index-map.txt";
      vw_out() << "Writing: " << index_map << "\n";
      std::ofstream ih(index_map.c_str());
      for (int dem_iter = 0; dem_iter < (int)loaded_dems.size(); dem_iter++) {
        ih << opt.dem_files[dem_iter] << ' ' << dem_iter << "\n";
      }
    }

  } ASP_STANDARD_CATCHES;

  return 0;
}
