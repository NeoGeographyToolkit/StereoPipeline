// __BEGIN_LICENSE__
//  Copyright (c) 2009-2025, United States Government as represented by the
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

/// \file sfs_blend.cc
///

// A tool to take an SfS-produced DEM, and replace in the areas in
// permanent shadow this DEM with the original LOLA DEM, with a
// transition at the boundary.

// It uses the Euclidean distance to the boundary, which is better
// than the Manhattan distance employed by grassfire.

#include <asp/Core/Macros.h>
#include <asp/Core/AspProgramOptions.h>
#include <asp/Core/GdalUtils.h>
#include <asp/SfS/SfsImageProc.h>

#include <vw/FileIO/DiskImageManager.h>
#include <vw/Image/InpaintView.h>
#include <vw/Image/RoundAndClamp.h>
#include <vw/Image/Filter.h>
#include <vw/Cartography/GeoTransform.h>
#include <vw/FileIO/FileUtils.h>

#include <boost/math/special_functions/fpclassify.hpp>
#include <boost/math/special_functions/erf.hpp>
#include <boost/program_options.hpp>

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
#include <cmath>

using namespace vw;
using namespace vw::cartography;
namespace po = boost::program_options;
namespace fs = boost::filesystem;

GeoReference read_georef(std::string const& file) {
  // Read a georef, and check for success
  GeoReference geo;
  bool is_good = read_georeference(geo, file);
  if (!is_good)
    vw_throw(ArgumentErr() << "No georeference found in " << file << ".\n");

  // This is a bug fix. The georef pixel size in y must be negative
  // for the image to be oriented correctly.
  if (geo.transform()(1, 1) > 0)
    vw_throw(ArgumentErr() << "The georeference in " << file
              << " has a positive pixel size in y. "
              << "This is unexpected. Normally it is negative since the (0, 0) "
              << "pixel is in the upper-left. Check your DEM pixel size with "
              << "gdalinfo. Cannot continue.\n");

  return geo;
}

struct Options: vw::GdalWriteOptions {
  std::string sfs_dem, lola_dem, max_lit_image_mosaic, output_dem, output_weight,
    shadow_fill_mode;
  double image_threshold, weight_blur_sigma, lit_blend_length,
    shadow_blend_length, min_blend_size;
  std::vector<double> shadow_fill_params;
  double shadow_fill_max_diam, shadow_fill_ratio; // parsed from shadow_fill_params
  Options(): image_threshold(0.0), weight_blur_sigma(0.0), lit_blend_length(0.0),
             shadow_blend_length(0.0), min_blend_size(0.0),
             shadow_fill_max_diam(0.0), shadow_fill_ratio(0.0) {}
};

// Label connected shadow blobs with 4-connectivity. Here shadow(col, row) is
// non-zero in the shadow. On output, labels is 0 outside shadow and a positive
// blob id inside. This is a local operation on a small tile crop.
void labelShadowBlobs(ImageView<char> const& shadow,
                      // Outputs
                      ImageView<int> & labels, int & num_blobs) {
  int cols = shadow.cols(), rows = shadow.rows();
  labels.set_size(cols, rows);
  for (int col = 0; col < cols; col++) {
    for (int row = 0; row < rows; row++)
      labels(col, row) = 0;
  }
  num_blobs = 0;
  int dcol[4] = {1, -1, 0, 0};
  int drow[4] = {0, 0, 1, -1};
  std::vector<Vector2i> stack;
  for (int col = 0; col < cols; col++) {
    for (int row = 0; row < rows; row++) {
      if (!shadow(col, row) || labels(col, row) != 0)
        continue;
      num_blobs++;
      labels(col, row) = num_blobs;
      stack.clear();
      stack.push_back(Vector2i(col, row));
      while (!stack.empty()) {
        Vector2i p = stack.back();
        stack.pop_back();
        for (int k = 0; k < 4; k++) {
          int nc = p.x() + dcol[k], nr = p.y() + drow[k];
          if (nc < 0 || nc >= cols || nr < 0 || nr >= rows)
            continue;
          if (shadow(nc, nr) && labels(nc, nr) == 0) {
            labels(nc, nr) = num_blobs;
            stack.push_back(Vector2i(nc, nr));
          }
        }
      }
    }
  }
}

// Absorb interior specks. A non-shadow pixel that cannot be reached from the
// tile border through non-shadow pixels is an enclosed speck. Left in, it would
// become an interior boundary condition that spikes the Laplace solve, so mark
// it as shadow too.
void fillInteriorSpecks(ImageView<char> & shadow) {
  int cols = shadow.cols(), rows = shadow.rows();
  ImageView<char> reached(cols, rows);
  for (int col = 0; col < cols; col++) {
    for (int row = 0; row < rows; row++)
      reached(col, row) = 0;
  }
  int dcol[4] = {1, -1, 0, 0};
  int drow[4] = {0, 0, 1, -1};
  std::vector<Vector2i> stack;
  // Seed from all non-shadow border pixels.
  for (int col = 0; col < cols; col++) {
    for (int row = 0; row < rows; row++) {
      bool border = (col == 0 || row == 0 || col == cols - 1 || row == rows - 1);
      if (border && !shadow(col, row) && !reached(col, row)) {
        reached(col, row) = 1;
        stack.push_back(Vector2i(col, row));
      }
    }
  }
  while (!stack.empty()) {
    Vector2i p = stack.back();
    stack.pop_back();
    for (int k = 0; k < 4; k++) {
      int nc = p.x() + dcol[k], nr = p.y() + drow[k];
      if (nc < 0 || nc >= cols || nr < 0 || nr >= rows)
        continue;
      if (!shadow(nc, nr) && !reached(nc, nr)) {
        reached(nc, nr) = 1;
        stack.push_back(Vector2i(nc, nr));
      }
    }
  }
  for (int col = 0; col < cols; col++) {
    for (int row = 0; row < rows; row++) {
      if (!shadow(col, row) && !reached(col, row))
        shadow(col, row) = 1;
    }
  }
}

// Grow and smooth the shadow mask: dilate by dilate_r then erode by erode_r
// (4-connectivity). With dilate_r > erode_r this nets an outward growth, which
// absorbs the unreliable band of fabricated bright SfS at the shadow rim (the
// grazing-illumination garbage knobs) into the hole so they get the smooth grown
// floor instead of being kept as jagged spikes against the lowered floor. The
// closing part also swallows thin lit fingers.
void morphCloseShadow(ImageView<char> & shadow, int dilate_r, int erode_r) {
  if (dilate_r <= 0 && erode_r <= 0)
    return;
  int cols = shadow.cols(), rows = shadow.rows();
  int dcol[4] = {1, -1, 0, 0};
  int drow[4] = {0, 0, 1, -1};
  ImageView<char> tmp(cols, rows);
  // Dilate.
  for (int step = 0; step < dilate_r; step++) {
    for (int col = 0; col < cols; col++) {
      for (int row = 0; row < rows; row++) {
        char v = shadow(col, row);
        for (int k = 0; k < 4 && !v; k++) {
          int nc = col + dcol[k], nr = row + drow[k];
          if (nc >= 0 && nc < cols && nr >= 0 && nr < rows && shadow(nc, nr))
            v = 1;
        }
        tmp(col, row) = v;
      }
    }
    shadow = vw::copy(tmp);
  }
  // Erode, treating the image border as shadow so the closing does not eat the
  // shadow where a blob runs up against the tile edge.
  for (int step = 0; step < erode_r; step++) {
    for (int col = 0; col < cols; col++) {
      for (int row = 0; row < rows; row++) {
        char v = shadow(col, row);
        for (int k = 0; k < 4 && v; k++) {
          int nc = col + dcol[k], nr = row + drow[k];
          if (nc >= 0 && nc < cols && nr >= 0 && nr < rows && !shadow(nc, nr))
            v = 0;
        }
        tmp(col, row) = v;
      }
    }
    shadow = vw::copy(tmp);
  }
}

// Solve the discrete Poisson equation (Laplacian z = forcing) on the hole pixels
// via successive over-relaxation. The Dirichlet boundary value at a non-hole
// neighbor is read from bdry. The update is z = (sum of 4 neighbors - forcing)/4.
// For the Laplace membrane use forcing 0 and bdry = SfS; for the unit bowl use
// forcing 1 and bdry = 0. The result at hole pixels is written into z.
void solveSOR(ImageView<char> const& is_hole,
              ImageView<double> const& bdry,
              double forcing,
              std::vector<Vector2i> const& pix,
              // Output
              ImageView<double> & z) {
  int cols = is_hole.cols(), rows = is_hole.rows();
  int dcol[4] = {1, -1, 0, 0};
  int drow[4] = {0, 0, 1, -1};
  // Initialize hole pixels to the mean of their boundary (non-hole) neighbors,
  // falling back to 0, so the iteration starts near the answer.
  for (size_t i = 0; i < pix.size(); i++) {
    int col = pix[i].x(), row = pix[i].y();
    double sum = 0.0;
    int cnt = 0;
    for (int k = 0; k < 4; k++) {
      int nc = col + dcol[k], nr = row + drow[k];
      if (nc < 0 || nc >= cols || nr < 0 || nr >= rows)
        continue;
      if (!is_hole(nc, nr)) {
        sum += bdry(nc, nr);
        cnt++;
      }
    }
    z(col, row) = (cnt > 0) ? sum / cnt : 0.0;
  }
  double omega = 1.9;
  int max_iter = 5000;
  double tol = 1e-4;
  for (int iter = 0; iter < max_iter; iter++) {
    double max_change = 0.0;
    for (size_t i = 0; i < pix.size(); i++) {
      int col = pix[i].x(), row = pix[i].y();
      double sum = 0.0;
      for (int k = 0; k < 4; k++) {
        int nc = col + dcol[k], nr = row + drow[k];
        if (is_hole(nc, nr))
          sum += z(nc, nr);
        else
          sum += bdry(nc, nr);
      }
      double z_new = (sum - forcing) / 4.0;
      double z_sor = z(col, row) + omega * (z_new - z(col, row));
      max_change = std::max(max_change, std::abs(z_sor - z(col, row)));
      z(col, row) = z_sor;
    }
    if (max_change < tol)
      break;
  }
}



// The workhorse of this code, do the blending
class SfsBlendView: public ImageViewBase<SfsBlendView>{

  ImageViewRef<float> m_sfs_dem, m_lola_dem, m_image_mosaic;
  float m_sfs_nodata, m_lola_nodata, m_weight_nodata;
  int m_extra;
  bool m_save_weight;
  Options const& m_opt;

  typedef float PixelT;

public:
  SfsBlendView(ImageViewRef<float> sfs_dem, ImageViewRef<float> lola_dem,
               ImageViewRef<float> image_mosaic,
               float sfs_nodata, float lola_nodata, float weight_nodata, int extra,
               bool save_weight, Options const& opt):
    m_sfs_dem(sfs_dem), m_lola_dem(lola_dem), m_image_mosaic(image_mosaic),
    m_sfs_nodata(sfs_nodata), m_lola_nodata(lola_nodata),
    m_weight_nodata(weight_nodata), m_extra(extra),
    m_save_weight(save_weight), m_opt(opt) {}

  typedef PixelT pixel_type;
  typedef PixelT result_type;
  typedef ProceduralPixelAccessor<SfsBlendView> pixel_accessor;

  inline int32 cols() const { return m_sfs_dem.cols(); }
  inline int32 rows() const { return m_sfs_dem.rows(); }
  inline int32 planes() const { return 1; }

  inline pixel_accessor origin() const { return pixel_accessor(*this, 0, 0); }

  inline pixel_type operator()(double/*i*/, double/*j*/, int32/*p*/ = 0) const {
    vw_throw(NoImplErr() << "SfsBlendView::operator()(...) is not implemented");
    return pixel_type();
  }

  typedef CropView<ImageView<pixel_type>> prerasterize_type;
  inline prerasterize_type prerasterize(BBox2i const& bbox) const {
    
    BBox2i biased_box = bbox;
    biased_box.expand(m_extra);
    biased_box.crop(bounding_box(m_sfs_dem));

    // Make crops in memory (from references)
    ImageView<pixel_type> sfs_dem_crop = crop(m_sfs_dem, biased_box);
    ImageView<pixel_type> lola_dem_crop = crop(m_lola_dem, biased_box);
    ImageView<pixel_type> image_mosaic_crop = crop(m_image_mosaic, biased_box);

    // The mask of lit pixels
    ImageView<PixelMask<pixel_type>> mask
      = create_mask_less_or_equal(image_mosaic_crop, m_opt.image_threshold);

    // The mask of unlit pixels
    ImageView<PixelMask<pixel_type>> inv_mask = vw::copy(mask);
    for (int col = 0; col < inv_mask.cols(); col++) {
      for (int row = 0; row < inv_mask.rows(); row++) {
        if (is_valid(mask(col, row))) {
          inv_mask(col, row) = 0;
          inv_mask(col, row).invalidate();
        } else {
          inv_mask(col, row) = 1;
          inv_mask(col, row).validate();
        }
      }
    }

    // For the curved and lola-curved modes, clean the lit/shadow classification
    // before it drives both the blend weight and the grown floor: absorb interior
    // specks and close thin lit fingers of fabricated bright SfS at the shadow
    // edge, so those garbage spikes become part of the hole and get the smooth
    // floor instead of being kept at the lit SfS value. The legacy 'lola' mode is
    // left untouched.
    if (m_opt.shadow_fill_mode != "lola") {
      int cols = inv_mask.cols(), rows = inv_mask.rows();
      ImageView<char> shadow(cols, rows);
      for (int col = 0; col < cols; col++) {
        for (int row = 0; row < rows; row++)
          shadow(col, row) = is_valid(inv_mask(col, row)) ? 1 : 0;
      }
      fillInteriorSpecks(shadow);
      // Close thin lit fingers of fabricated bright SfS at the shadow edge.
      morphCloseShadow(shadow, 3, 3);
      fillInteriorSpecks(shadow);
      for (int col = 0; col < cols; col++) {
        for (int row = 0; row < rows; row++) {
          if (shadow(col, row)) {
            inv_mask(col, row) = 1;
            inv_mask(col, row).validate();
            mask(col, row) = 0;
            mask(col, row).invalidate();
          } else {
            inv_mask(col, row) = 0;
            inv_mask(col, row).invalidate();
            mask(col, row) = image_mosaic_crop(col, row);
            mask(col, row).validate();
          }
        }
      }
    }

    // The grassfire weight positive in the lit region, with zero at the light-shadow
    // boundary
    bool no_zero_at_border = true; // don't decrease the weights to zero at image border

    // Fill small holes that we don't blend in those, then compute the distance
    // to the remaining holes
    ImageView<pixel_type> lit_grass_dist
      = vw::grassfire(vw::copy(vw::fill_holes_grass(mask, m_opt.min_blend_size)),
                      no_zero_at_border);

    // The grassfire weights are positive in the shadow region, with
    // zero at the light-shadow boundary
    ImageView<pixel_type> shadow_grass_dist = vw::grassfire(inv_mask, no_zero_at_border);

    // Note that our boundary is in fact two pixel wide at the light-shadow
    // interface, given how lit_grass_dist and shadow_grass_dist are defined as
    // the negation of each other. The boundary is the set of pixels where both
    // of these are <= 1. Here must use double pixels as otherwise there's not
    // enough precision for the final weights (even though they are float).
    ImageView<double> dist_to_bd 
      = asp::calcClampedBdDist(lit_grass_dist, shadow_grass_dist,
                               m_opt.lit_blend_length, m_opt.shadow_blend_length);

    // Apply the blur
    if (m_opt.weight_blur_sigma > 0)
      dist_to_bd = vw::gaussian_filter(dist_to_bd, m_opt.weight_blur_sigma);

    // Build the fill DEM used inside the shadow. For the legacy 'lola' mode this
    // is the LOLA DEM. For the curved and lola-curved modes it is the SfS DEM
    // outside the shadow blobs, so the fill is continuous with the surrounding
    // terrain across the shadow boundary (no step, hence no seam), and inside
    // each fully-contained blob it is a smooth floor grown from the surrounding
    // SfS by solving the Laplace equation, curved down by the crater
    // depth/diameter ratio. For lola-curved the floor is capped by LOLA (the
    // lower of the two), so LOLA can never raise it. Big blobs (wider than the
    // tile margin or the max diameter) are left unfilled, so the blend there is
    // SfS to SfS (curved) or picks up LOLA only through the cap (lola-curved).
    ImageView<float> fill_crop;
    if (m_opt.shadow_fill_mode == "lola")
      fill_crop = vw::copy(lola_dem_crop);
    else
      fill_crop = vw::copy(sfs_dem_crop);
    if (m_opt.shadow_fill_mode != "lola") {
      int cols = sfs_dem_crop.cols(), rows = sfs_dem_crop.rows();

      // The lit/shadow classification was already cleaned above (specks absorbed
      // and thin lit fingers closed), so just read the shadow mask here.
      ImageView<char> shadow(cols, rows);
      for (int col = 0; col < cols; col++) {
        for (int row = 0; row < rows; row++)
          shadow(col, row) = is_valid(inv_mask(col, row)) ? 1 : 0;
      }

      // In the shadow default to LOLA (so big unfilled blobs hand off to LOLA);
      // the lit region keeps the SfS set above. Filled blobs are overwritten
      // below with the grown floor.
      for (int col = 0; col < cols; col++) {
        for (int row = 0; row < rows; row++) {
          if (shadow(col, row))
            fill_crop(col, row) = lola_dem_crop(col, row);
        }
      }

      ImageView<int> labels;
      int num_blobs = 0;
      labelShadowBlobs(shadow, labels, num_blobs);

      ImageView<double> sfs_d(cols, rows), zero_d(cols, rows),
        zlid(cols, rows), zbowl(cols, rows);
      for (int col = 0; col < cols; col++) {
        for (int row = 0; row < rows; row++) {
          sfs_d(col, row) = sfs_dem_crop(col, row);
          zero_d(col, row) = 0.0;
          zlid(col, row) = 0.0;
          zbowl(col, row) = 0.0;
        }
      }

      // Collect each blob's pixels, whether it touches the tile border, and its
      // inscribed radius (the max shadow grassfire distance in the blob).
      std::vector<std::vector<Vector2i>> blob_pix(num_blobs + 1);
      std::vector<bool> blob_border(num_blobs + 1, false);
      std::vector<double> blob_rad(num_blobs + 1, 0.0);
      for (int col = 0; col < cols; col++) {
        for (int row = 0; row < rows; row++) {
          int b = labels(col, row);
          if (b == 0)
            continue;
          blob_pix[b].push_back(Vector2i(col, row));
          if (col == 0 || row == 0 || col == cols - 1 || row == rows - 1)
            blob_border[b] = true;
          blob_rad[b] = std::max(blob_rad[b], (double)shadow_grass_dist(col, row));
        }
      }

      ImageView<char> is_hole(cols, rows);
      for (int col = 0; col < cols; col++) {
        for (int row = 0; row < rows; row++)
          is_hole(col, row) = 0;
      }

      for (int b = 1; b <= num_blobs; b++) {
        // Skip blobs that are not fully contained (bigger than the tile margin):
        // these are left as LOLA, which is the intended handoff for big craters.
        if (blob_border[b])
          continue;
        if ((double)blob_pix[b].size() < m_opt.min_blend_size)
          continue;

        for (size_t i = 0; i < blob_pix[b].size(); i++)
          is_hole(blob_pix[b][i].x(), blob_pix[b][i].y()) = 1;

        // Laplace membrane with the surrounding SfS as boundary.
        solveSOR(is_hole, sfs_d, 0.0, blob_pix[b], zlid);

        // Curve the floor down by the depth/diameter ratio, but only for blobs
        // up to the max diameter. The unit bowl (Poisson, zero boundary) is
        // scaled to hit the target center depth, and stays <= the membrane so it
        // cannot pop up.
        double diam = 2.0 * blob_rad[b];
        double depth = 0.0;
        if (diam <= m_opt.shadow_fill_max_diam)
          depth = m_opt.shadow_fill_ratio * diam;
        double scale = 0.0;
        if (depth > 0.0) {
          solveSOR(is_hole, zero_d, 1.0, blob_pix[b], zbowl);
          double min_b = 0.0;
          for (size_t i = 0; i < blob_pix[b].size(); i++)
            min_b = std::min(min_b, zbowl(blob_pix[b][i].x(), blob_pix[b][i].y()));
          if (min_b < -1e-6)
            scale = depth / (-min_b);
        }

        for (size_t i = 0; i < blob_pix[b].size(); i++) {
          int col = blob_pix[b][i].x(), row = blob_pix[b][i].y();
          double val = zlid(col, row) + scale * zbowl(col, row);
          if (m_opt.shadow_fill_mode == "lola-curved" &&
              lola_dem_crop(col, row) != m_lola_nodata)
            val = std::min(val, (double)lola_dem_crop(col, row));
          fill_crop(col, row) = val;
        }

        // Reset the hole mask for the next blob.
        for (size_t i = 0; i < blob_pix[b].size(); i++)
          is_hole(blob_pix[b][i].x(), blob_pix[b][i].y()) = 0;
      }
    }

    // Do the blending
    ImageView<float> blended_dem;
    blended_dem.set_size(sfs_dem_crop.cols(), sfs_dem_crop.rows());
    for (int col = 0; col < sfs_dem_crop.cols(); col++) {
      for (int row = 0; row < sfs_dem_crop.rows(); row++) {

        if (!m_save_weight)
          blended_dem(col, row) = m_sfs_nodata;
        else
          blended_dem(col, row) = m_weight_nodata;

        // The signed distance to the boundary is modified so that the smallest
        // value is 0, the largest is 1, and in a band close to the boundary it
        // transitions from 0 to 1. Must use here a double value, for precision.
        double weight = (dist_to_bd(col, row) + m_opt.shadow_blend_length) /
          (m_opt.shadow_blend_length + m_opt.lit_blend_length);

        // These are not strictly necessary but enforce them
        if (weight > 1.0)
          weight = 1.0;
        if (weight < 1e-7) // take into account the float error
          weight = 0.0;
        
        // Handle no-data values. These are not meant to happen, but do this just in case.
        if (sfs_dem_crop(col, row) == m_sfs_nodata)
          weight = 0.0; // Use LOLA

        if (fill_crop(col, row) == m_lola_nodata)
          continue;

        if (!m_save_weight)
          blended_dem(col, row)
            = weight * sfs_dem_crop(col, row) + (1.0 - weight) * fill_crop(col, row);
        else
          blended_dem(col, row) = weight;
      }
    }

    return prerasterize_type(blended_dem, -biased_box.min().x(), -biased_box.min().y(),
                             cols(), rows());
  }

  template <class DestT>
  inline void rasterize(DestT const& dest, BBox2i bbox) const {
    vw::rasterize(prerasterize(bbox), dest, bbox);
  }
};

void handle_arguments(int argc, char *argv[], Options& opt) {

  po::options_description general_options("Options");
  general_options.add_options()
    ("sfs-dem", po::value<std::string>(&opt.sfs_dem),
     "The SfS DEM to process.")
    ("lola-dem", po::value<std::string>(&opt.lola_dem),
     "The LOLA DEM to use to fill in the regions in permanent shadow.")
    ("max-lit-image-mosaic", po::value<std::string>(&opt.max_lit_image_mosaic),
     "The maximally lit image mosaic to use to determine the permanently shadowed regions.")
    ("image-threshold",  po::value<double>(&opt.image_threshold)->default_value(0.0),
     "The value separating permanently shadowed pixels from lit pixels in the maximally lit image mosaic.")
    ("lit-blend-length", po::value<double>(&opt.lit_blend_length)->default_value(0.0),
     "The length, in pixels, over which to blend the SfS and LOLA DEMs at the boundary of the permanently shadowed region towards the lit region.")
    ("shadow-blend-length", po::value<double>(&opt.shadow_blend_length)->default_value(0.0),
     "The length, in pixels, over which to blend the SfS and LOLA DEMs at the boundary of the permanently shadowed region towards the shadowed region.")
    ("weight-blur-sigma", po::value<double>(&opt.weight_blur_sigma)->default_value(0.0),
     "The standard deviation of the Gaussian used to blur the weight that performs the transition from the SfS to the LOLA DEM. A higher value results in a smoother transition (this does not smooth the DEMs). The extent of the blur is about 7 times this deviation, though it tapers fast to 0 before that. Set to 0 to not use this operation.")
    ("min-blend-size", po::value<double>(&opt.min_blend_size)->default_value(0.0),
     "Do not apply blending in shadowed areas of dimensions less than this, hence keeping there the SfS DEM.")
    ("shadow-fill-mode", po::value<std::string>(&opt.shadow_fill_mode)->default_value("lola"),
     "What to put in a permanently shadowed hole before blending. 'lola' uses the LOLA DEM (the default, legacy behavior). 'curved' grows a smooth floor from the surrounding SfS by solving the Laplace equation, curved gently down like a crater. 'lola-curved' uses the lower of that curved floor and LOLA, so LOLA can never raise the floor. See --shadow-fill-params.")
    ("shadow-fill-params", po::value<std::vector<double>>(&opt.shadow_fill_params)->multitoken(),
     "Two values used by the 'curved' and 'lola-curved' shadow fill modes: the maximum hole diameter (in pixels) that is curved, and the crater depth/diameter ratio. A hole up to this diameter gets a floor a fraction (the ratio) of its own width deep; larger holes are left flat (and handed off to LOLA in lola-curved mode). Ignored when --shadow-fill-mode is 'lola'. Suggested: 100 0.15.")
    ("output-dem", po::value(&opt.output_dem), "The blended output DEM to save.")
    ("output-weight", po::value(&opt.output_weight), "The weight showing the proportion of the SfS DEM in the blend with the LOLA DEM (1 is for purely SfS and 0 is for purely LOLA).");

  general_options.add(vw::GdalWriteOptionsDescription(opt));

  po::options_description positional("");
  po::positional_options_description positional_desc;

  std::string usage("[options]");
  bool allow_unregistered = false;
  std::vector<std::string> unregistered;
  po::variables_map vm =
    asp::check_command_line(argc, argv, opt, general_options, general_options,
                            positional, positional_desc, usage,
                            allow_unregistered, unregistered);

  // Error checking
  if (opt.sfs_dem == "" || opt.lola_dem == "" ||
      opt.max_lit_image_mosaic == "" || opt.output_dem == "" || opt.output_weight == "")
    vw_throw(ArgumentErr() << "Not all input or output files were specified.\n"
                           << usage << general_options);
  if (opt.lit_blend_length <= 0)
    vw_throw(ArgumentErr() << "The lit blending length must be positive.\n"
                           << usage << general_options);
  if (opt.shadow_blend_length <= 0)
    vw_throw(ArgumentErr() << "The shadow blending length must be positive.\n"
                           << usage << general_options);
  if (opt.image_threshold <= 0)
    vw_throw(ArgumentErr() << "The image threshold must be positive.\n"
                           << usage << general_options);

  // Validate the shadow fill mode and parse its parameters.
  if (opt.shadow_fill_mode != "lola" && opt.shadow_fill_mode != "curved" &&
      opt.shadow_fill_mode != "lola-curved")
    vw_throw(ArgumentErr() << "Unknown --shadow-fill-mode: " << opt.shadow_fill_mode
                           << ". Must be 'lola', 'curved', or 'lola-curved'.\n");
  opt.shadow_fill_max_diam = 0.0;
  opt.shadow_fill_ratio = 0.0;
  if (opt.shadow_fill_mode != "lola") {
    if (opt.shadow_fill_params.size() != 2)
      vw_throw(ArgumentErr() << "--shadow-fill-params expects two values: "
               << "the max hole diameter in pixels and the depth/diameter ratio.\n");
    opt.shadow_fill_max_diam = opt.shadow_fill_params[0];
    opt.shadow_fill_ratio = opt.shadow_fill_params[1];
    if (opt.shadow_fill_max_diam <= 0)
      vw_throw(ArgumentErr() << "The shadow fill max diameter must be positive.\n");
    if (opt.shadow_fill_ratio < 0 || opt.shadow_fill_ratio > 0.5)
      vw_throw(ArgumentErr() << "The shadow fill depth/diameter ratio must be "
               << "in [0, 0.5].\n");
  }

  // Create the output directory
  vw::create_out_dir(opt.output_dem);
} // End function handle_arguments

int main(int argc, char *argv[]) {

  Options opt;

  try{

    handle_arguments(argc, argv, opt);

    vw_out() << "Reading SfS DEM: " << opt.sfs_dem << std::endl;
    DiskImageView<float> sfs_dem(opt.sfs_dem);


    vw_out() << "Reading LOLA DEM: " << opt.lola_dem << std::endl;
    DiskImageView<float> lola_dem(opt.lola_dem);

    vw_out() << "Reading maximally-lit image mosaic: " << opt.max_lit_image_mosaic << std::endl;
    DiskImageView<float> image_mosaic(opt.max_lit_image_mosaic);

    std::string err_msg = " As a fallback solution, gdalwarp with bicubic "
                          "interpolation may be used to reconcile these.";

    if (sfs_dem.cols() != lola_dem.cols() || sfs_dem.rows() != lola_dem.rows())
      vw_throw(ArgumentErr() << "The SfS DEM and LOLA DEM must have the same "
               << "dimensions." << err_msg);

    if (sfs_dem.cols() != image_mosaic.cols() || sfs_dem.rows() != image_mosaic.rows())
      vw_throw(ArgumentErr() << "The SfS DEM and image mosaic must have the same "
               << "dimensions." << err_msg);

    GeoReference sfs_georef   = read_georef(opt.sfs_dem);
    GeoReference lola_georef  = read_georef(opt.lola_dem);
    GeoReference image_georef = read_georef(opt.max_lit_image_mosaic);
    if (sfs_georef.proj4_str() != lola_georef.proj4_str() ||
        sfs_georef.proj4_str() != image_georef.proj4_str())
      vw_throw(ArgumentErr() << "The SfS DEM, LOLA DEM, and image mosaic "
               << "must have the same PROJ4 string." << err_msg);

    // All these must be on the same grid, or else the blending will be wrong.
    // Allow some tolerance here as sometimes products created with different
    // tools can differ a bit. 1e-10 degrees is about 0.01 mm on Earth's surface.
    vw::Vector2 sfs_corner = sfs_georef.pixel_to_point(Vector2(0, 0));
    vw::Vector2 lola_corner = lola_georef.pixel_to_point(Vector2(0, 0));
    vw::Vector2 image_corner = image_georef.pixel_to_point(Vector2(0, 0));
    if (norm_2(sfs_corner - lola_corner) > 1e-10 || norm_2(sfs_corner - image_corner) > 1e-10)
      vw_throw(ArgumentErr() << "The SfS DEM, LOLA DEM, and image mosaic "
               << "must be on the same grid." << err_msg);

    float sfs_nodata = -1.0, lola_nodata = -1.0, image_nodata = -1.0;
    DiskImageResourceGDAL sfs_rsrc(opt.sfs_dem);
    if (sfs_rsrc.has_nodata_read())
      sfs_nodata = sfs_rsrc.nodata_read();
    else
      vw_throw(ArgumentErr() << "The SfS DEM does not have a no-data value.");
    DiskImageResourceGDAL lola_rsrc(opt.lola_dem);
    if (lola_rsrc.has_nodata_read())
      lola_nodata = lola_rsrc.nodata_read();
    else
      vw_throw(ArgumentErr() << "The LOLA DEM does not have a no-data value.");
    DiskImageResourceGDAL image_rsrc(opt.max_lit_image_mosaic);
    if (image_rsrc.has_nodata_read())
      image_nodata = image_rsrc.nodata_read();
    else
      vw_throw(ArgumentErr() << "The maximally-lit mosaic does not have a no-data value.");

    // When processing the DEM tile by tile, need to see further in
    // each tile because of blending and blurring
    int extra = 2*opt.lit_blend_length + 2*opt.shadow_blend_length + opt.min_blend_size;
    if (opt.weight_blur_sigma > 0)
      extra += vw::compute_kernel_size(opt.weight_blur_sigma);

    // A curved shadow blob is only filled if it fits fully inside the tile
    // margin, so the margin must be at least its radius (half the max diameter)
    // for the per-tile fill to be consistent across tiles.
    if (opt.shadow_fill_mode != "lola")
      extra = std::max(extra, (int)std::ceil(opt.shadow_fill_max_diam / 2.0) + 2);

    // Write bigger tiles to make the processing with the extra margin
    // more efficient.
    int block_size = 256 + 2 * extra;
    block_size = 16*std::ceil(block_size/16.0); // internal constraint

    vw_out() << "Writing: " << opt.output_dem << std::endl;
    bool has_georef = true, has_nodata = true;
    TerminalProgressCallback tpc("asp", ": ");
    float weight_nodata = -1.0;
    bool save_weight = false;
    asp::saveWithTempBigBlocks(block_size,
                               opt.output_dem,
                               SfsBlendView(sfs_dem, lola_dem, image_mosaic,
                                            sfs_nodata, lola_nodata,
                                            weight_nodata, extra,
                                            save_weight, opt),
                               has_georef, sfs_georef,
                               has_nodata, sfs_nodata, opt, tpc);

    // Write the weight. Have to rerun the same logic due to ASP's limitations,
    // it cannot write two large files at the same time.
    vw_out() << "Writing the blending weight: "
             << opt.output_weight << std::endl;
    save_weight = true;
    asp::saveWithTempBigBlocks(block_size,
                               opt.output_weight,
                               SfsBlendView(sfs_dem, lola_dem, image_mosaic,
                                            sfs_nodata, lola_nodata,
                                            weight_nodata, extra,
                                            save_weight, opt),
                               has_georef, sfs_georef,
                               has_nodata, weight_nodata, opt, tpc);


  } ASP_STANDARD_CATCHES;

  return 0;
}
