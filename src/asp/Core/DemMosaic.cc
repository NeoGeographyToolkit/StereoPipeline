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

#include <asp/Core/DemMosaic.h>

#include <vw/FileIO/DiskImageView.h>
#include <vw/FileIO/DiskImageResourceGDAL.h>
#include <vw/Image/Filter.h>

namespace asp {

using namespace vw;

void blurWeights(ImageView<double> & weights, double sigma) {

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
      if (weights(col,row) > 0)
        extra_wts(col + extra, row + extra) = weights(col, row);
      else
        extra_wts(col + extra, row + extra) = 0;
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

void applyExternalWeights(std::string const& weight_file,
                          double min_weight, bool invert_weights,
                          vw::BBox2i const& in_box,
                          vw::ImageView<double>& local_wts) {

  DiskImageResourceGDAL rsrc(weight_file);
  DiskImageView<double> disk_weight(rsrc);
  ImageView<double> external_weight = crop(disk_weight, in_box);

  // Must have the same size as the local weights
  if (local_wts.cols() != external_weight.cols() ||
      local_wts.rows() != external_weight.rows())
    vw_throw(ArgumentErr() << "The external weights must have the same size as the DEM.\n");
      
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

} // end namespace asp
