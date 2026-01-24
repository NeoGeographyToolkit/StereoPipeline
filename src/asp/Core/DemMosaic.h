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

/// \file DemMosaic.h
/// Utility functions for dem_mosaic

#ifndef __ASP_CORE_DEM_MOSAIC_H__
#define __ASP_CORE_DEM_MOSAIC_H__

#include <vw/Image/ImageView.h>
#include <vw/Image/ImageViewRef.h>
#include <vw/Image/PixelTypes.h>
#include <vw/Image/Manipulation.h>
#include <vw/Image/Algorithms.h>
#include <vw/Math/Vector.h>

namespace asp {

void blurWeights(vw::ImageView<double> & weights, double sigma);

void applyExternalWeights(std::string const& weight_file,
                          double min_weight, bool invert_weights,
                          vw::BBox2i const& in_box,
                          vw::ImageView<double>& local_wts);

double compute_plateaued_weights(vw::Vector2 const& pix, bool horizontal,
                                 std::vector<double> const& centers,
                                 std::vector<double> const& widths,
                                 double max_weight_val);

template<class ImageT>
void centerline_weights2(ImageT const& img, vw::ImageView<double> & weights,
                         double max_weight_val, double hole_fill_value=0, 
                         double border_fill_value=-1, vw::BBox2i roi=vw::BBox2i()) {

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

  vw::BBox2i output_bbox = roi;
  if (roi.empty())
    output_bbox = bounding_box(img);

  // Compute the weighting for each pixel in the image
  weights.set_size(output_bbox.width(), output_bbox.height());
  fill(weights, 0);
  
  for (int row = output_bbox.min().y(); row < output_bbox.max().y(); row++) {
    for (int col = output_bbox.min().x(); col < output_bbox.max().x(); col++) {
      bool inner_row = ((row >= minValInCol[col]) && (row <= maxValInCol[col]));
      bool inner_col = ((col >= minValInRow[row]) && (col <= maxValInRow[row]));
      bool inner_pixel = inner_row && inner_col;
      vw::Vector2 pix(col, row);
      double new_weight = 0; // Invalid pixels usually get zero weight
      if (is_valid(img(col,row))) {
        double weight_h = compute_plateaued_weights(pix, true,  hCenterLine, hMaxDistArray,
                                                    max_weight_val);
        double weight_v = compute_plateaued_weights(pix, false, vCenterLine, vMaxDistArray,
                                                    max_weight_val);
        new_weight = weight_h*weight_v;
      }
      else { // Invalid pixel
        if (inner_pixel)
          new_weight = hole_fill_value;
        else // Border pixel
          new_weight = border_fill_value;
      }
      weights(col-output_bbox.min().x(), row-output_bbox.min().y()) = new_weight;
    }
  }
}

double S_shape(double x, double M, double L);

typedef vw::PixelGrayA<double> DoubleGrayA;
DoubleGrayA interpDem(double x, double y, 
                     vw::ImageView<DoubleGrayA> const& dem,
                     vw::ImageViewRef<DoubleGrayA> const& interp_dem,
                     double tol,
                     bool propagate_nodata);

} // end namespace asp

#endif //__ASP_CORE_DEM_MOSAIC_H__
