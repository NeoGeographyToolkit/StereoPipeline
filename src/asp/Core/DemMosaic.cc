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

#include <asp/Core/DemMosaic.h>

#include <vw/FileIO/DiskImageView.h>
#include <vw/FileIO/DiskImageResourceGDAL.h>
#include <vw/Image/Filter.h>
#include <vw/Cartography/GeoReference.h>
#include <boost/math/special_functions/erf.hpp>

namespace asp {

void blurWeights(vw::ImageView<double> & weights, double sigma) {

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

  vw::ImageView<double> extra_wts(cols + 2*extra, rows + 2*extra);
  vw::fill(extra_wts, 0);
  for (int col = 0; col < cols; col++) {
    for (int row = 0; row < rows; row++) {
      if (weights(col,row) > 0)
        extra_wts(col + extra, row + extra) = weights(col, row);
      else
        extra_wts(col + extra, row + extra) = 0;
    }
  }

  vw::ImageView<double> blurred_wts = vw::gaussian_filter(extra_wts, sigma);

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

void applyExternalWeights(std::string const& weight_file,
                          double min_weight, bool invert_weights,
                          vw::BBox2i const& in_box,
                          vw::ImageView<double>& local_wts) {

  vw::DiskImageResourceGDAL rsrc(weight_file);
  vw::DiskImageView<double> disk_weight(rsrc);
  vw::ImageView<double> external_weight = vw::crop(disk_weight, in_box);

  // Must have the same size as the local weights
  if (local_wts.cols() != external_weight.cols() ||
      local_wts.rows() != external_weight.rows())
    vw::vw_throw(vw::ArgumentErr() << "The external weights must have the same size as the DEM.\n");

  // Read the no-data value. Any no-data weights will be ignored.
  double weight_nodata = -std::numeric_limits<double>::max();
  if (rsrc.has_nodata_read())
    weight_nodata = rsrc.nodata_read();

  // Limit from below
  if (min_weight > 0) {
    for (int col = 0; col < external_weight.cols(); col++) {
      for (int row = 0; row < external_weight.rows(); row++) {
        bool good = (external_weight(col, row) != weight_nodata);
        if (!good)
          continue;
        external_weight(col, row)
        = std::max(external_weight(col, row), min_weight);
      }
    }
  }

  // See if to invert the weight
  if (invert_weights) {
    for (int col = 0; col < external_weight.cols(); col++) {
      for (int row = 0; row < external_weight.rows(); row++) {
        bool good = (external_weight(col, row) != weight_nodata);
        if (!good)
          continue;
        if (external_weight(col, row) > 0)
        external_weight(col, row) = 1.0 / external_weight(col, row);
      }
    }
  }

  // Multiply the local weights by the external weights
  for (int col = 0; col < local_wts.cols(); col++) {
    for (int row = 0; row < local_wts.rows(); row++) {
        bool good = (external_weight(col, row) != weight_nodata);
        if (!good)
          continue;
      local_wts(col, row) *= external_weight(col, row);
    }
  }

  return;
}

// Compute a weight that is zero, then rises, plateaus, and then falls.
// - hCenterLine contains the center column at each row/col
// - hMaxDistArray contains the width of the column at each row/col
// TODO(oalexan1): Move this to VW, and see about overwriting 
// the other weight functions. The other one uses normalization, which is bad.
double computePlateauedWeights(vw::Vector2 const& pix, bool horizontal,
                               std::vector<double> const& centers,
                               std::vector<double> const& widths,
                               double max_weight_val) {

  int primary_axis = 0, secondary_axis = 1; // Vertical
  if (horizontal) {
    primary_axis   = 1;
    secondary_axis = 0;
  }

  // We round below, to avoid issues when we are within numerical value
  // to an integer value for row/col.
  // To do: Need to do interpolation here.

  int pos = (int)round(pix[primary_axis]); // The row or column
  if (pos < 0 || pos >= (int)widths.size() || pos >= (int)centers.size())
    return 0;

  double max_dist = widths[pos]/2.0; // Half column width
  double center   = centers[pos];
  double dist     = fabs(pix[secondary_axis]-center); // Pixel distance from center column

  if (max_dist <= 0 || dist < 0)
    return 0;

  // We want to make sure the weight is positive (even if small) at
  // the first/last valid pixel.
  double tol = 1e-8*max_dist;

  // The weight is not normalized. This is a bugfix. Normalized weights result
  // in higher weights in narrow regions, which is not what we want.
  double weight = std::max(0.0, max_dist - dist + tol);
  weight = std::min(max_weight_val, weight);

  return weight;
}

// Compute centerline weights using plateaued weight function (not normalized,
// unlike VW's version). Distinguishes interior holes from border pixels and
// assigns them different values for DEM mosaicking.
void centerlineWeightsWithHoles(vw::ImageView<vw::PixelMask<double>> const& img,
                                vw::ImageView<double> & weights,
                                double max_weight_val,
                                double hole_fill_value) {

  int numRows = img.rows();
  int numCols = img.cols();

  // Arrays to be returned out of this function
  std::vector<double> hCenterLine  (numRows, 0);
  std::vector<double> hMaxDistArray(numRows, 0);
  std::vector<double> vCenterLine  (numCols, 0);
  std::vector<double> vMaxDistArray(numCols, 0);

  std::vector<int> minValInRow(numRows, 0);
  std::vector<int> maxValInRow(numRows, 0);
  std::vector<int> minValInCol(numCols, 0);
  std::vector<int> maxValInCol(numCols, 0);

  for (int k = 0; k < numRows; k++) {
    minValInRow[k] = numCols;
    maxValInRow[k] = 0;
  }
  for (int col = 0; col < numCols; col++) {
    minValInCol[col] = numRows;
    maxValInCol[col] = 0;
  }

  // Note that we do just a single pass through the image to compute
  // both the horizontal and vertical min/max values.
  for (int row = 0 ; row < numRows; row++) {
    for (int col = 0; col < numCols; col++) {

      if (!is_valid(img(col,row))) continue;

      // Record the first and last valid column in each row
      if (col < minValInRow[row]) minValInRow[row] = col;
      if (col > maxValInRow[row]) maxValInRow[row] = col;

      // Record the first and last valid row in each column
      if (row < minValInCol[col]) minValInCol[col] = row;
      if (row > maxValInCol[col]) maxValInCol[col] = row;
    }
  }

  // For each row, record central column and the column width
  for (int row = 0; row < numRows; row++) {
    hCenterLine   [row] = (minValInRow[row] + maxValInRow[row])/2.0;
    hMaxDistArray [row] =  maxValInRow[row] - minValInRow[row];
    if (hMaxDistArray[row] < 0)
      hMaxDistArray[row]=0;
  }

  // For each row, record central column and the column width
  for (int col = 0 ; col < numCols; col++) {
    vCenterLine   [col] = (minValInCol[col] + maxValInCol[col])/2.0;
    vMaxDistArray [col] =  maxValInCol[col] - minValInCol[col];
    if (vMaxDistArray[col] < 0)
      vMaxDistArray[col]=0;
  }

  // Process the entire image
  vw::BBox2i output_bbox = vw::bounding_box(img);

  // Compute the weighting for each pixel in the image
  weights.set_size(output_bbox.width(), output_bbox.height());
  vw::fill(weights, 0);

  for (int row = output_bbox.min().y(); row < output_bbox.max().y(); row++) {
    for (int col = output_bbox.min().x(); col < output_bbox.max().x(); col++) {
      bool inner_row = ((row >= minValInCol[col]) && (row <= maxValInCol[col]));
      bool inner_col = ((col >= minValInRow[row]) && (col <= maxValInRow[row]));
      bool inner_pixel = inner_row && inner_col;
      vw::Vector2 pix(col, row);
      double new_weight = 0; // Invalid pixels usually get zero weight
      if (is_valid(img(col,row))) {
        double weight_h = computePlateauedWeights(pix, true,  hCenterLine, hMaxDistArray,
                                                   max_weight_val);
        double weight_v = computePlateauedWeights(pix, false, vCenterLine, vMaxDistArray,
                                                   max_weight_val);
        new_weight = weight_h*weight_v;
      } else { // Invalid pixel
        if (inner_pixel)
          new_weight = hole_fill_value;
        else // Border pixel
          new_weight = -1; // Border fill value
      }
      weights(col-output_bbox.min().x(), row-output_bbox.min().y()) = new_weight;
    }
  }
}

// Smooth step function using error function. Returns 0 at x=0, M at x=M,
// with smooth S-curve transition. Higher L gives flatter ends and steeper middle.
double erfSmoothStep(double x, double M, double L) {
  if (x <= 0) return 0;
  if (x >= M) return M;
  return 0.5*M*(1 + boost::math::erf (0.5*sqrt(M_PI) * (2*x*L/M - L)));
}

// Convert a projected-space bounding box to pixel coordinates with snapping.
// Transforms all 4 corners, rounds near-integer values (within tol) to integers,
// then floors/ceils to ensure integer pixel boundaries.
vw::BBox2 pointToPixelBboxSnapped(vw::cartography::GeoReference const& georef,
                                  vw::BBox2 const& point_bbox, double tol) {

  vw::BBox2 pix_box;
  vw::Vector2 corners[] = {point_bbox.min(), point_bbox.max(),
                           vw::Vector2(point_bbox.min().x(), point_bbox.max().y()),
                           vw::Vector2(point_bbox.max().x(), point_bbox.min().y())};
  for (int i = 0; i < 4; i++)
    pix_box.grow(georef.point_to_pixel(corners[i]));

  // Round values that are likely due to numerical error
  if (norm_2(pix_box.min() - round(pix_box.min())) < tol)
    pix_box.min() = round(pix_box.min());
  if (norm_2(pix_box.max() - round(pix_box.max())) < tol)
    pix_box.max() = round(pix_box.max());

  // Grow the box until the corners are integer
  pix_box.min() = floor(pix_box.min());
  pix_box.max() = ceil (pix_box.max());

  return pix_box;
}

// Read a georeference from a file and throw if not found.
vw::cartography::GeoReference readGeorefOrThrow(std::string const& file) {
  vw::cartography::GeoReference geo;
  bool is_good = vw::cartography::read_georeference(geo, file);
  if (!is_good)
    vw::vw_throw(vw::ArgumentErr() << "No georeference found in " << file << ".\n");
  return geo;
}

// A helper function to do interpolation. Will not interpolate when
// exactly on the grid.
DoubleGrayA interpDem(double x, double y,
                      vw::ImageView<DoubleGrayA> const& dem,
                      vw::ImageViewRef<DoubleGrayA> const& interp_dem,
                      double tol,
                      bool propagate_nodata) {

  DoubleGrayA pval;

  // Round to nearest integer location
  int i0 = round(x), j0 = round(y);
  if ((fabs(x-i0) < tol) && (fabs(y-j0) < tol) &&
      ((i0 >= 0) && (i0 <= dem.cols()-1) &&
        (j0 >= 0) && (j0 <= dem.rows()-1))) {

    // A lot of care is needed here. We are at an integer
    // pixel, save for numerical error. Just borrow pixel's
    // value, and don't interpolate. Interpolation can result
    // in invalid pixels if the current pixel is valid but its
    // neighbors are not. It can also make it appear is if the
    // current indices are out of bounds while in fact they
    // are barely so.
    pval = dem(i0, j0);

  } else { // We are not right on an integer pixel and we need to interpolate

    // Below must use x <= cols()-1 as x is double
    bool is_good = ((x >= 0) && (x <= dem.cols()-1) &&
        (y >= 0) && (y <= dem.rows()-1));
    if (!is_good) {
      pval.a() = 0; // Flag as nodata
      return pval; // Outside the loaded DEM bounds, skip to the next pixel
    }

    // If we have weights of 0, that means there are invalid pixels, so skip this point.
    int i0 = (int)floor(x), j0 = (int)floor(y);
    int i1 = (int)ceil(x),  j1 = (int)ceil(y);
    bool nodata = ((dem(i0, j0).a() == 0) || (dem(i1, j0).a() == 0) ||
                   (dem(i0, j1).a() == 0) || (dem(i1, j1).a() == 0));
    bool border = ((dem(i0, j0).a() <  0) || (dem(i1, j0).a() <  0) ||
                   (dem(i0, j1).a() <  0) || (dem(i1, j1).a() <  0));

    if (nodata || border) {
      pval.v() = 0;
      pval.a() = -1; // Flag as border

      if (propagate_nodata && !border)
        pval.a() = 0; // Flag as nodata

    } else
      pval = interp_dem(x, y); // Things checked out, do the interpolation.
  }

  return pval;
}

} // end namespace asp
