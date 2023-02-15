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

/// \file SfsImageProc.cc
/// Image processing routines for SfS

#include <vw/Core/Log.h>
#include <vw/Camera/CameraModel.h>
#include <vw/Cartography/GeoReference.h>
#include <vw/FileIO/DiskImageView.h>
#include <asp/Core/SfsImageProc.h>

#include <string>

using namespace vw;


namespace asp {

void adjustBorderlineDataWeights(int cols, int rows,
                                 int blending_dist, double blending_power,
                                 std::vector<ImageView<double>> & ground_weights) {
  
  int num_images = ground_weights.size();
  
  ImageView<double> union_weight(cols, rows), boundary_weight(cols, rows);
  for (int col = 0; col < cols; col++) {
    for (int row = 0; row < rows; row++) {
      union_weight(col, row) = 0.0;
      boundary_weight(col, row) = 0.0;
    }
  }
  for (int image_iter = 0; image_iter < num_images; image_iter++) {
    auto & wt = ground_weights[image_iter]; // alias
    if (wt.cols() > 0 && wt.rows() > 0) {
      
      if (wt.cols() != cols || wt.rows() != rows) 
        vw::vw_throw(vw::ArgumentErr() << "The input DEM and computed extended weights must "
                     << "have the same dimensions.\n");
      
      // Prepare to create some weights smaller than min_wt. First, bump up
      // the existing weights. Normally they are already well above this.
      for (int col = 0; col < wt.cols(); col++) {
        for (int row = 0; row < wt.rows(); row++) {
          union_weight(col, row) = std::max(union_weight(col, row), wt(col, row));
        }
      }
    }
  }
  // TODO(oalexan1): Make the block below a small function which finds the unsigned
  // distance to boundary.
  double blending_dist_sq = blending_dist * blending_dist;
  int max_dist_int = ceil(blending_dist); // an int overestimate
      
  for (int col = 0; col < cols; col++) {
    for (int row = 0; row < rows; row++) {
          
      // Look for a boundary pixel, which is a pixel with zero weight but with neighbors
      // with positive weight
      if (union_weight(col, row) > 0)
        continue;
      bool is_bd_pix = false;
      for (int c = col - 1; c <= col + 1; c++) {
        for (int r = row - 1; r <= row + 1; r++) {
          if (c < 0 || c >= cols || r < 0 || r >= rows)
            continue;
          if (union_weight(c, r) > 0) {
            is_bd_pix = true;
            break; // found it
          }
        }
        if (is_bd_pix) 
          break; // found it
      }
      if (!is_bd_pix) 
        continue; // did not find it
          
      // Found the boundary pixel. Increase the weight in the
      // circular neighborhood.  It will still be below min_wt
      // and decay to 0 at the boundary of this neighborhood.
      for (int c = col - max_dist_int; c <= col + max_dist_int; c++) {
        for (int r = row - max_dist_int; r <= row + max_dist_int; r++) {
          if (c < 0 || c >= cols || r < 0 || r >= rows)
            continue;
              
          // Cast to double before multiplying to avoid integer overflow
          double dsq = double(c - col) * double(c - col) + 
            double(r - row) * double(r - row);
              
          // Too far 
          if (dsq >= blending_dist_sq) 
            continue;
              
          double d = sqrt(dsq);
          d = blending_dist - d; // get a cone pointing up, with base at height 0.
          d /= double(blending_dist); // make it between 0 and 1
          d = std::max(d, 0.0); // should not be necessary
          // Add its contribution
          boundary_weight(c, r) = std::max(boundary_weight(c, r), d);
        }
      }
    }
  }
  
  std::vector<ImageView<double>> avg_weights(num_images);
  for (int image_iter = 0; image_iter < num_images; image_iter++) {
    auto & ground_wt = ground_weights[image_iter]; // alias
    if (ground_wt.cols() > 0 && ground_wt.rows() > 0) {
      
      if (ground_wt.cols() != cols || ground_wt.rows() != rows) 
        vw::vw_throw(vw::ArgumentErr() << "The input DEM and computed extended weights must "
                     << "have the same dimensions.\n");
      
      avg_weights[image_iter].set_size(cols, rows);
      for (int col = 0; col < ground_wt.cols(); col++) {
        for (int row = 0; row < ground_wt.rows(); row++) {
          avg_weights[image_iter](col, row) = (ground_wt(col, row) > 0);
        }
      }
      
      double blending_dist_sq = blending_dist * blending_dist;
      int max_dist_int = ceil(blending_dist); // an int overestimate
      
      for (int col = 0; col < cols; col++) {
        for (int row = 0; row < rows; row++) {
          
          // Look for a boundary pixel, which is a pixel with zero weight but with neighbors
          // with positive weight
          if (ground_wt(col, row) > 0)
            continue;
          bool is_bd_pix = false;
          for (int c = col - 1; c <= col + 1; c++) {
            for (int r = row - 1; r <= row + 1; r++) {
              if (c < 0 || c >= cols || r < 0 || r >= rows)
                continue;
              if (ground_wt(c, r) > 0) {
                is_bd_pix = true;
                break; // found it
              }
            }
            if (is_bd_pix) 
                  break; // found it
          }
          if (!is_bd_pix) 
            continue; // did not find it
          
          // Found the boundary pixel. Increase the weight in the
          // circular neighborhood.  It will still be below 1
          // and decay to 0 at the boundary of this neighborhood.
          for (int c = col - max_dist_int; c <= col + max_dist_int; c++) {
            for (int r = row - max_dist_int; r <= row + max_dist_int; r++) {
              if (c < 0 || c >= cols || r < 0 || r >= rows)
                continue;
              
              // Cast to double before multiplying to avoid integer overflow
              double dsq = double(c - col) * double(c - col) + 
                double(r - row) * double(r - row);
              
              // Too far 
              if (dsq >= blending_dist_sq) 
                continue;
              
              double d = sqrt(dsq);
              d = blending_dist - d; // get a cone pointing up, with base at height 0.
              d /= double(blending_dist); // make it between 0 and 1
              d = std::max(d, 0.0); // should not be necessary
              // Add its contribution
              avg_weights[image_iter](c, r) = std::max(avg_weights[image_iter](c, r), d);
            }
          }
          
        }
      }
    }
  } // end iterating over images

  // Do the weighted average. First initialize to 0.
  for (int col = 0; col < cols; col++) {
    for (int row = 0; row < rows; row++) {
          
      // Find the sum at each pixel
      double sum = 0.0;
      for (int image_iter = 0; image_iter < num_images; image_iter++) {
        auto & avg_wt = avg_weights[image_iter]; // alias
        if (avg_wt.cols() > 0 && avg_wt.rows() > 0 && avg_wt(col, row) > 0) {
          sum += avg_wt(col, row);
        }
      }
      if (sum <= 0) 
        continue;
          
      // Divide by the sum at each pixel
      for (int image_iter = 0; image_iter < num_images; image_iter++) {
        auto & avg_wt = avg_weights[image_iter]; // alias
        if (avg_wt.cols() > 0 && avg_wt.rows() > 0 && avg_wt(col, row) > 0) {
          avg_wt(col, row) = avg_wt(col, row) / sum; // weighted average
          avg_wt(col, row) *= boundary_weight(col, row); // get a share of the bd weight
              
          // undo the power in the weight, add the new contribution, and put back the power
          double wt = pow(ground_weights[image_iter](col, row), 1.0/blending_power)
            + avg_wt(col, row);
          wt = std::min(wt, 1.0); // make sure the weight is no more than one
          avg_wt(col, row) = pow(wt, blending_power); // put back the power
        }
      }
    }
  }

  // Use the newly created weights
  for (int image_iter = 0; image_iter < num_images; image_iter++)
    ground_weights[image_iter] = copy(avg_weights[image_iter]);

  // TODO(oalexan1): Must make sure to make the images have non-negative but valid values
  // where the weights are positive and invalid values where they are zero.
  // TODO(oalexan1): Should these weights be updated as the terrain changes?
  
#if 0
  // Debug code
  bool has_georef = true, has_nodata = false;
  std::string union_weight_file = out_prefix + "-union_weight.tif";
  vw_out() << "Writing: " << union_weight_file << std::endl;
  block_write_gdal_image(union_weight_file,
                         union_weight,
                         has_georef, geos[0][0], has_nodata,
                         img_nodata_val, opt,
                         TerminalProgressCallback("asp", ": "));
      
  std::string boundary_weight_file = out_prefix + "-boundary_weight.tif";
  vw_out() << "Writing: " << boundary_weight_file << std::endl;
  block_write_gdal_image(boundary_weight_file,
                         boundary_weight,
                         has_georef, geos[0][0], has_nodata,
                         img_nodata_val, opt,
                         TerminalProgressCallback("asp", ": "));
      
  for (int image_iter = 0; image_iter < num_images; image_iter++) {

    if (skip_images[0].find(image_iter) != skip_images[0].end())
      continue;

    std::string out_camera_file
      = asp::bundle_adjust_file_name(out_prefix,
                                     input_images[image_iter],
                                     input_cameras[image_iter]);
    std::string local_prefix = fs::path(out_camera_file).replace_extension("").string();
    bool has_georef = true, has_nodata = false;
    std::string image_weight_file = local_prefix + "-image_weight.tif";
    vw_out() << "Writing: " << image_weight_file << std::endl;
    block_write_gdal_image(image_weight_file,
                           ground_weights[image_iter],
                           has_georef, geos[0][0], has_nodata,
                           img_nodata_val, opt,
                           TerminalProgressCallback("asp", ": "));
        
    std::string avg_weight_file = local_prefix + "-avg_weight.tif";
    vw_out() << "Writing: " << avg_weight_file << std::endl;
    block_write_gdal_image(avg_weight_file,
                           avg_weights[image_iter],
                           has_georef, geos[0][0], has_nodata,
                           img_nodata_val, opt,
                           TerminalProgressCallback("asp", ": "));
  }
#endif
  avg_weights.clear(); // not needed anymore

  return;
}

}
