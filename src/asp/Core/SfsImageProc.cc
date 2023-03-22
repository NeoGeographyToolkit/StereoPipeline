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

#include <asp/Core/SfsImageProc.h>

#include <vw/Core/Log.h>
#include <vw/Camera/CameraModel.h>
#include <vw/Cartography/GeoReference.h>
#include <vw/FileIO/DiskImageView.h>
#include <vw/FileIO/GdalWriteOptions.h>
#include <vw/Cartography/GeoReferenceUtils.h>
#include <asp/Core/BundleAdjustUtils.h>

#include <boost/filesystem.hpp>

#include <string>

namespace fs = boost::filesystem;
using namespace vw;


namespace asp {

// Given a set of images of same dimensions, find the per-pixel maximum.
void maxImage(int cols, int rows,
              std::set<int> const& skip_images,
              std::vector<vw::ImageView<double>> const& images,
              ImageView<double> & max_image) {
  
  int num_images = images.size();

  max_image.set_size(cols, rows);
  for (int col = 0; col < cols; col++) {
    for (int row = 0; row < rows; row++) {
      max_image(col, row) = 0.0;
    }
  }

  for (int image_iter = 0; image_iter < num_images; image_iter++) {

    if (skip_images.find(image_iter) != skip_images.end())
      continue;

    auto & img = images[image_iter]; // alias
    if (img.cols() <= 0 || img.rows() <= 0) 
      continue;
    
    if (img.cols() != cols || img.rows() != rows) 
      vw::vw_throw(vw::ArgumentErr() << "The input DEM and computed extended images "
                   << "must have the same dimensions.\n");
    
    for (int col = 0; col < img.cols(); col++) {
      for (int row = 0; row < img.rows(); row++) {
        max_image(col, row) = std::max(max_image(col, row), img(col, row));
      }
    }
  }

  return;
}

// Given an image with float pixels, find the pixels where the image
// value is non-positive but some of its neighbors have positive
// values. Create an image which has the value 1 at such pixels and
// whose values linearly decrease to 0 both in the direction of pixels
// with positive and non-positive input values. 
void boundaryWeight(int blending_dist, ImageView<double> const & image, // inputs
                    ImageView<double> & boundary_weight) { // output
  
  double blending_dist_sq = blending_dist * blending_dist;
  int max_dist_int = ceil(blending_dist); // an int overestimate

  // Initialize the output to 0
  int cols = image.cols(), rows = image.rows();
  boundary_weight.set_size(cols, rows);
  for (int col = 0; col < cols; col++) {
    for (int row = 0; row < rows; row++) {
      boundary_weight(col, row) = 0.0;
    }
  }

  for (int col = 0; col < cols; col++) {
    for (int row = 0; row < rows; row++) {
          
      // Look for a boundary pixel, which is a pixel with non-positive
      // value but with neighbors with positive value
      if (image(col, row) > 0)
        continue;
      
      bool is_bd_pix = false;
      for (int c = col - 1; c <= col + 1; c++) {
        for (int r = row - 1; r <= row + 1; r++) {
          if (c < 0 || c >= cols || r < 0 || r >= rows)
            continue;
          if (image(c, r) > 0) {
            is_bd_pix = true;
            break; // found it
          }
        }
        if (is_bd_pix) 
          break; // found it
      }
      if (!is_bd_pix) 
        continue; // did not find it
          
      // Found the boundary pixel. Increase the weight in the circular
      // neighborhood. It will decay to 0 at the boundary of this
      // neighborhood.
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

  return;
}
  
// Find the function which is 1 on the boundary of the max lit region
// and linearly decays to 0 away from it. Give portions of this to the
// image blending weights, in proportion to how relevant the images are
// likely to contribute. Hence, in the area where all data is
// borderline, we give more weight to the borderline data, because
// there is nothing else.  This improves the reconstruction.
// Note: Input image blending weights are 1 away from shadows and decay to 0
// at the shadow boundary. Output weights will decay then to 0 a bit
// deeper in the shadow area where there is no other data.
void adjustBorderlineDataWeights(int cols, int rows,
                                 int blending_dist, double blending_power,
                                 vw::GdalWriteOptions const& opt,
                                 vw::cartography::GeoReference const& geo,
                                 std::set<int> const& skip_images,
                                 std::string const& out_prefix, // for debug data
                                 std::vector<std::string> const& input_images, 
                                 std::vector<std::string> const& input_cameras, 
                                 std::vector<vw::ImageView<double>> & ground_weights) {
  
  int num_images = ground_weights.size();

  // Find the max per-pixel weight
  ImageView<double> max_weight;
  maxImage(cols, rows, skip_images, ground_weights,
           max_weight);

  // Find a weight which is 1 at the lit/unlit interface and decaying linearly
  // to 0.
  ImageView<double> boundary_weight;
  boundaryWeight(blending_dist, max_weight, // inputs
                 boundary_weight); // output

  // TODO(oalexan1): Factor this out.
  // For any input image, find the the weight which is 1 inside where
  // the image pixels are lit, and linearly decreases from 1 to 0 at
  // image boundary (outwardly, in the area of unlit pixels).
  std::vector<ImageView<double>> avg_weights(num_images);
  for (int image_iter = 0; image_iter < num_images; image_iter++) {

    if (skip_images.find(image_iter) != skip_images.end())
      continue;
    
    auto & ground_wt = ground_weights[image_iter]; // alias
    if (ground_wt.cols() > 0 && ground_wt.rows() > 0) {
      
      if (ground_wt.cols() != cols || ground_wt.rows() != rows) 
        vw::vw_throw(vw::ArgumentErr() << "The input DEM and computed extended "
                     << "weights must have the same dimensions.\n");
      
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
          
          // Look for a boundary pixel, which is a pixel with zero
          // weight but with neighbors with positive weight
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
        if (skip_images.find(image_iter) != skip_images.end())
          continue;
        auto & avg_wt = avg_weights[image_iter]; // alias
        if (avg_wt.cols() > 0 && avg_wt.rows() > 0 && avg_wt(col, row) > 0) {
          sum += avg_wt(col, row);
        }
      }
      if (sum <= 0) 
        continue;
          
      // Divide by the sum at each pixel
      for (int image_iter = 0; image_iter < num_images; image_iter++) {
        
        if (skip_images.find(image_iter) != skip_images.end())
          continue;
        
        auto & avg_wt = avg_weights[image_iter]; // alias
        if (avg_wt.cols() > 0 && avg_wt.rows() > 0 && avg_wt(col, row) > 0) {

          // This is the core of the logic. When only one image has lit pixels nearby,
          // ensure this weight is 1. When there's a lot of them, ensure the others
          // don't dilute this weight. But still ensure this weight is continuous.
          avg_wt(col, row) = avg_wt(col, row) * std::max(1.0, 1.0/sum); // weighted average

          // Adjust by a share of the bd weight. This way this correction will only
          // happen at the max-lit boundary and not other per-image boundaries.
          avg_wt(col, row) *= boundary_weight(col, row);
              
          // Undo the power in the weight being passed in, add the new
          // contribution, and put back the power.
          double wt = pow(ground_weights[image_iter](col, row), 1.0/blending_power)
            + avg_wt(col, row); // undo
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
  float img_nodata_val = false; // will not be used
  std::string max_weight_file = out_prefix + "-max_weight.tif";
  vw_out() << "Writing: " << max_weight_file << std::endl;
  vw::cartography::block_write_gdal_image(max_weight_file,
                         max_weight,
                         has_georef, geo, has_nodata,
                         img_nodata_val, opt,
                         TerminalProgressCallback("asp", ": "));
      
  std::string boundary_weight_file = out_prefix + "-boundary_weight.tif";
  vw_out() << "Writing: " << boundary_weight_file << std::endl;
  vw::cartography::block_write_gdal_image(boundary_weight_file,
                         boundary_weight,
                         has_georef, geo, has_nodata,
                         img_nodata_val, opt,
                         TerminalProgressCallback("asp", ": "));
      
  for (int image_iter = 0; image_iter < num_images; image_iter++) {

    if (skip_images.find(image_iter) != skip_images.end())
      continue;

    std::string out_camera_file
      = asp::bundle_adjust_file_name(out_prefix,
                                     input_images[image_iter],
                                     input_cameras[image_iter]);
    std::string local_prefix = fs::path(out_camera_file).replace_extension("").string();
    
    bool has_georef = true, has_nodata = false;
    std::string image_weight_file = local_prefix + "-image_weight.tif";
    vw_out() << "Writing: " << image_weight_file << std::endl;
    vw::cartography::block_write_gdal_image(image_weight_file,
                           ground_weights[image_iter],
                           has_georef, geo, has_nodata,
                           img_nodata_val, opt,
                           TerminalProgressCallback("asp", ": "));
        
    std::string avg_weight_file = local_prefix + "-avg_weight.tif";
    vw_out() << "Writing: " << avg_weight_file << std::endl;
    vw::cartography::block_write_gdal_image(avg_weight_file,
                           avg_weights[image_iter],
                           has_georef, geo, has_nodata,
                           img_nodata_val, opt,
                           TerminalProgressCallback("asp", ": "));
  }
#endif
  avg_weights.clear(); // not needed anymore

  return;
}

}
