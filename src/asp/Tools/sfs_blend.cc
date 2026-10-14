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
    shadow_blend_length, min_blend_size, shadow_fill_max_diam;
  Options(): image_threshold(0.0), weight_blur_sigma(0.0), lit_blend_length(0.0),
             shadow_blend_length(0.0), min_blend_size(0.0),
             shadow_fill_max_diam(0.0) {}
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



// Estimate the local terrain curvature just outside a shadow blob, to be used
// as the Poisson forcing so the grown floor continues the surrounding shape:
// flat ground gives a flat fill, a bowl-shaped neighborhood gives a bowl. Sample
// the discrete Laplacian of the SfS (second derivative, spacing s) over a thin
// band of valid lit ground a few pixels outside the blob, and average it. Not a
// parameter. The sign convention matches solveSOR: a positive value is a bowl
// that dips down, so the caller clamps the result to be non-negative to make
// sure the fill can never dome up.
double estimateLocalCurvature(ImageView<double> const& sfs_d,
                              ImageView<char> const& shadow,
                              std::vector<Vector2i> const& blob_pix,
                              int cols, int rows) {
  // Window = blob bounding box expanded by a margin.
  int wc0 = cols, wc1 = 0, wr0 = rows, wr1 = 0;
  for (size_t i = 0; i < blob_pix.size(); i++) {
    wc0 = std::min(wc0, blob_pix[i].x()); wc1 = std::max(wc1, blob_pix[i].x());
    wr0 = std::min(wr0, blob_pix[i].y()); wr1 = std::max(wr1, blob_pix[i].y());
  }
  int M = 10;
  wc0 = std::max(0, wc0 - M); wc1 = std::min(cols - 1, wc1 + M);
  wr0 = std::max(0, wr0 - M); wr1 = std::min(rows - 1, wr1 + M);
  int ww = wc1 - wc0 + 1, wh = wr1 - wr0 + 1;

  // Local hole mask, then dilate by 2 and by 6 to carve a sample band [3, 6] px
  // of ground just outside the blob.
  ImageView<char> base(ww, wh);
  for (int c = 0; c < ww; c++) {
    for (int r = 0; r < wh; r++)
      base(c, r) = 0;
  }
  for (size_t i = 0; i < blob_pix.size(); i++)
    base(blob_pix[i].x() - wc0, blob_pix[i].y() - wr0) = 1;
  int dcol[4] = {1, -1, 0, 0};
  int drow[4] = {0, 0, 1, -1};
  ImageView<char> near2 = vw::copy(base), near6 = vw::copy(base), tmp(ww, wh);
  for (int pass = 0; pass < 6; pass++) {
    for (int c = 0; c < ww; c++) {
      for (int r = 0; r < wh; r++) {
        char v = near6(c, r);
        for (int k = 0; k < 4 && !v; k++) {
          int nc = c + dcol[k], nr = r + drow[k];
          if (nc >= 0 && nc < ww && nr >= 0 && nr < wh && near6(nc, nr))
            v = 1;
        }
        tmp(c, r) = v;
      }
    }
    near6 = vw::copy(tmp);
    if (pass == 1)
      near2 = vw::copy(tmp);
  }

  int s = 3;
  double sum = 0.0;
  int cnt = 0;
  for (int c = 0; c < ww; c++) {
    for (int r = 0; r < wh; r++) {
      if (!near6(c, r) || near2(c, r))
        continue;
      int gc = c + wc0, gr = r + wr0;
      if (shadow(gc, gr))
        continue;
      if (gc - s < 0 || gc + s >= cols || gr - s < 0 || gr + s >= rows)
        continue;
      if (shadow(gc - s, gr) || shadow(gc + s, gr) ||
          shadow(gc, gr - s) || shadow(gc, gr + s))
        continue;
      double lap = (sfs_d(gc + s, gr) + sfs_d(gc - s, gr) - 2.0 * sfs_d(gc, gr)) / (s * s)
                 + (sfs_d(gc, gr + s) + sfs_d(gc, gr - s) - 2.0 * sfs_d(gc, gr)) / (s * s);
      sum += lap;
      cnt++;
    }
  }
  return (cnt > 0) ? sum / cnt : 0.0;
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

    // For the grown-floor modes, clean the lit/shadow classification
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
    // is the LOLA DEM. For the grown-floor modes it is the SfS DEM outside the
    // shadow blobs, so the fill is continuous with the surrounding terrain
    // across the shadow boundary (no step, hence no seam), and inside each
    // fully-contained blob it is a smooth floor grown from the surrounding SfS
    // by solving the Laplace equation, curved down by the crater depth/diameter
    // ratio. That grown floor is then capped (the lower of it and the reference)
    // by SfS for sfs-curved, and by both SfS and LOLA for lola-curved. Big blobs
    // (wider than the tile margin or the max diameter) are left unfilled, so the
    // blend there is SfS to SfS, or picks up LOLA only in the 'curved' handoff.
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
      // below with the grown floor. For sfs-curved and lola-curved the shadow
      // keeps the SfS (the grown floor is only a ceiling that caps SfS pop-ups),
      // so do not default it to LOLA.
      if (m_opt.shadow_fill_mode != "sfs-curved" &&
          m_opt.shadow_fill_mode != "lola-curved") {
        for (int col = 0; col < cols; col++) {
          for (int row = 0; row < rows; row++) {
            if (shadow(col, row))
              fill_crop(col, row) = lola_dem_crop(col, row);
          }
        }
      }

      ImageView<int> labels;
      int num_blobs = 0;
      labelShadowBlobs(shadow, labels, num_blobs);

      ImageView<double> sfs_d(cols, rows), zfill(cols, rows);
      for (int col = 0; col < cols; col++) {
        for (int row = 0; row < rows; row++) {
          sfs_d(col, row) = sfs_dem_crop(col, row);
          zfill(col, row) = 0.0;
        }
      }

      // Collect each blob's pixels and whether it touches the tile border.
      std::vector<std::vector<Vector2i>> blob_pix(num_blobs + 1);
      std::vector<bool> blob_border(num_blobs + 1, false);
      for (int col = 0; col < cols; col++) {
        for (int row = 0; row < rows; row++) {
          int b = labels(col, row);
          if (b == 0)
            continue;
          blob_pix[b].push_back(Vector2i(col, row));
          if (col == 0 || row == 0 || col == cols - 1 || row == rows - 1)
            blob_border[b] = true;
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

        // Grow the floor by solving the Poisson equation with the surrounding
        // SfS as boundary and a forcing equal to the local terrain curvature just
        // outside the hole. This continues the neighborhood shape inward: flat
        // around gives a flat fill, a bowl around gives a bowl. Clamp the forcing
        // to be non-negative so the fill can only dip down or stay flat, never
        // dome up.
        double c_est = estimateLocalCurvature(sfs_d, shadow, blob_pix[b], cols, rows);
        double forcing = std::max(c_est, 0.0);
        solveSOR(is_hole, sfs_d, forcing, blob_pix[b], zfill);

        for (size_t i = 0; i < blob_pix[b].size(); i++) {
          int col = blob_pix[b][i].x(), row = blob_pix[b][i].y();
          double val = zfill(col, row);
          // Cap by SfS, and for lola-curved also by LOLA, so the floor is the
          // lower of the grown ceiling and the chosen references. sfs-curved
          // keeps the real SfS floor and clips only where SfS pops above the
          // ceiling. lola-curved additionally lets LOLA pull the floor down.
          if (m_opt.shadow_fill_mode == "sfs-curved") {
            val = std::min(val, (double)sfs_dem_crop(col, row));
          } else if (m_opt.shadow_fill_mode == "lola-curved") {
            val = std::min(val, (double)sfs_dem_crop(col, row));
            if (lola_dem_crop(col, row) != m_lola_nodata)
              val = std::min(val, (double)lola_dem_crop(col, row));
          }
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

    // Feathered boundary blur: soften the seam where the grown floor meets the
    // real terrain. Blur the blended DEM (sigma = weight-blur-sigma), then blend
    // the blurred and original with a weight that is 1 on the shadow/lit boundary
    // and falls to 0 a distance L out on each side (L = lit-blend-length in the
    // lit region, shadow-blend-length in the shadow). The weight is built from
    // the grassfire distance, which is integer-valued, so it is then smoothed
    // with a Gaussian: an unsmoothed weight steps every pixel and those steps,
    // times (blurred minus original), band the result into stripes. Smoothing the
    // weight (as the old sfs_blend smooths its transition weight) removes them.
    // Only for the grown-floor modes and the DEM (not the weight) pass.
    double lit_L = m_opt.lit_blend_length;
    double shadow_L = m_opt.shadow_blend_length;
    double blur_sigma = m_opt.weight_blur_sigma;
    if (!m_save_weight && m_opt.shadow_fill_mode != "lola" &&
        blur_sigma > 0 && (lit_L > 0 || shadow_L > 0)) {
      int cols = sfs_dem_crop.cols(), rows = sfs_dem_crop.rows();
      // Blur input: replace nodata with the SfS so the blur stays well defined.
      ImageView<double> binput(cols, rows), wimg(cols, rows);
      double pi = 3.141592653589793;
      for (int col = 0; col < cols; col++) {
        for (int row = 0; row < rows; row++) {
          bool nodata = (blended_dem(col, row) == m_sfs_nodata);
          binput(col, row) = nodata ? sfs_dem_crop(col, row) : blended_dem(col, row);
          double wv = 0.0;
          if (!nodata) {
            bool is_shadow = is_valid(inv_mask(col, row));
            double d = is_shadow ? (double)shadow_grass_dist(col, row)
                                 : (double)lit_grass_dist(col, row);
            double L = is_shadow ? shadow_L : lit_L;
            if (L > 0 && d < L)
              wv = 0.5 * (1.0 + std::cos(pi * d / L));
          }
          wimg(col, row) = wv;
        }
      }
      ImageView<double> blurred = vw::gaussian_filter(binput, blur_sigma);
      // Smooth the weight to kill the per-pixel banding from the integer distance.
      double wsigma = std::max(2.0, std::max(lit_L, shadow_L) / 4.0);
      ImageView<double> wsm = vw::gaussian_filter(wimg, wsigma);
      for (int col = 0; col < cols; col++) {
        for (int row = 0; row < rows; row++) {
          if (blended_dem(col, row) == m_sfs_nodata)
            continue;
          double w = wsm(col, row);
          if (w <= 1e-6)
            continue;
          if (w > 1.0)
            w = 1.0;
          blended_dem(col, row) = w * blurred(col, row)
                                + (1.0 - w) * blended_dem(col, row);
        }
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
    ("shadow-fill-mode", po::value<std::string>(&opt.shadow_fill_mode)->default_value("lola-curved"),
     "What to put in a permanently shadowed hole before blending. 'lola' uses the LOLA DEM (the legacy behavior). 'curved' grows a smooth floor from the surrounding SfS by solving the Laplace equation with a locally estimated curvature, clamped so the floor cannot rise above its surroundings. 'sfs-curved' keeps the SfS and lowers it only where it rises above the curved floor, so the real SfS floor and detail are preserved and only spurious bumps are clipped. 'lola-curved' is like 'sfs-curved' but also takes the lower of LOLA, so LOLA can pull the floor down where it is genuinely lower (the default). See --shadow-fill-max-diameter.")
    ("shadow-fill-max-diameter", po::value<double>(&opt.shadow_fill_max_diam)->default_value(0.0),
     "The maximum hole diameter, in pixels, to which the grown floor is applied in the 'curved', 'sfs-curved', and 'lola-curved' modes. Larger holes keep the LOLA fill. Ignored when --shadow-fill-mode is 'lola'. A value such as 300 is reasonable.")
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
  if (opt.shadow_blend_length < 0)
    vw_throw(ArgumentErr() << "The shadow blending length must be non-negative.\n"
                           << usage << general_options);
  if (opt.image_threshold <= 0)
    vw_throw(ArgumentErr() << "The image threshold must be positive.\n"
                           << usage << general_options);

  // Validate the shadow fill mode and parse its parameters.
  if (opt.shadow_fill_mode != "lola" && opt.shadow_fill_mode != "curved" &&
      opt.shadow_fill_mode != "sfs-curved" && opt.shadow_fill_mode != "lola-curved")
    vw_throw(ArgumentErr() << "Unknown --shadow-fill-mode: " << opt.shadow_fill_mode
                           << ". Must be 'lola', 'curved', 'sfs-curved', "
                           << "or 'lola-curved'.\n");
  if (opt.shadow_fill_mode != "lola" && opt.shadow_fill_max_diam <= 0)
    vw_throw(ArgumentErr() << "The --shadow-fill-max-diameter must be positive "
             << "for the grown-floor shadow fill modes.\n");

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
