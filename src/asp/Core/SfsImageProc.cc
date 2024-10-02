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
#include <vw/Image/Transform.h>
#include <vw/Image/InpaintView.h>

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

// Given an image with non-negative values, create another image
// which is 1 where the input image has positive values, and decays
// to 0 linearly beyond that.
void extendedWeight(int blending_dist, ImageView<double> const & image, // inputs         
                    ImageView<double> & extended_weight) { // output

  int cols = image.cols(), rows = image.rows();
  extended_weight.set_size(cols, rows);
  for (int col = 0; col < image.cols(); col++) {
    for (int row = 0; row < image.rows(); row++) {
      extended_weight(col, row) = (image(col, row) > 0);
    }
  }
  
  double blending_dist_sq = blending_dist * blending_dist;
  int max_dist_int = ceil(blending_dist); // an int overestimate
  
  for (int col = 0; col < cols; col++) {
    for (int row = 0; row < rows; row++) {
      
      // Look for a boundary pixel, which is a pixel with zero
          // weight but with neighbors with positive weight
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
          extended_weight(c, r) = std::max(extended_weight(c, r), d);
        }
      }
    }
  }

  return;
}

// Adjust the weights at the boundary of the max-lit region
// so they don't decay to 0, as there we need all the image data
// we get, and there's no concern that we'll create a seam
// with some other better data. Weights which are already 0 by then
// but are non-zero close to that interface get grown too, depending,
// on how close they get to that interface.
// recompute these weights as the DEM changes, which is an approximation.
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

  // Find a weight which is 1 at the max-lit/unlit interface and decaying linearly
  // to 0.
  ImageView<double> boundary_weight;
  boundaryWeight(blending_dist, max_weight, // inputs
                 boundary_weight); // output

  // Weights at the boundary of the max lit mosaic are not allowed to decay
  // all the way to 0 as they are not needed for blending there, and
  // just result in blur.
  for (int col = 0; col < cols; col++) {
    for (int row = 0; row < rows; row++) {

      if (max_weight(col, row) <= 0 || boundary_weight(col, row) <= 0)
        continue; // not in the region of interest
        
      for (int image_iter = 0; image_iter < num_images; image_iter++) {
        
        if (skip_images.find(image_iter) != skip_images.end())
          continue;
        
        // Undo the power in the weight being passed in
        double ground_wt = ground_weights[image_iter](col, row);
        if (ground_wt <= 0.0) 
          continue;
        
        double max_wt = max_weight(col, row);
        ground_wt = pow(ground_wt, 1.0/blending_power);
        max_wt = pow(max_wt, 1.0/blending_power);

        // The closer one is to the max-lit boundary, the more this
        // weight is important
        ground_wt = std::max(ground_wt, ground_wt / max_wt);
        ground_wt = std::min(ground_wt, 1.0); // not necessary
        
        // put back the power
        ground_wt = pow(ground_wt, blending_power);

        // Put back the weight
        ground_weights[image_iter](col, row) = ground_wt;
      }
    }
  }

  // TODO(oalexan1): Must make sure to make the images have
  // non-negative but valid values where the weights are positive and
  // invalid values where they are zero.

  bool save_debug_info = false;
  if (save_debug_info) {
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
      std::string ground_weight_file = local_prefix + "-ground_weight.tif";
      vw_out() << "Writing: " << ground_weight_file << std::endl;
      vw::cartography::block_write_gdal_image(ground_weight_file,
                                              ground_weights[image_iter],
                                              has_georef, geo, has_nodata,
                                              img_nodata_val, opt,
                                              TerminalProgressCallback("asp", ": "));
        
    }
  } // end saving debug info

  return;
}

using namespace vw;

// Find the blending weight for an image. This is 1 inside the lit area and away
// from any edges or shadows, then linearly decrease to 0 towards such edges,
// and then is raised to a power to change how it decays to 0.
// Do not make these decrease around holes smaller than min_blend_size,
// TODO(oalexan1): Grassfire weights use the Manhattan distance, which
// result in noisy weights. The Euclidean distance to boundary would work
// better.
vw::ImageView<double> blendingWeights(vw::ImageViewRef<vw::PixelMask<float>> const& img,
                                      double blending_dist,
                                      double blending_power,
                                      int min_blend_size) {

  //   if (img.cols() <= 2 || img.rows() <= 2) {
  //     // The image is too small to have good weights. grassfire crashes.
  //     ImageView<double> weights(img.cols(), img.rows());
  //     for (int col = 0; col < weights.cols(); col++) {
  //       for (int row = 0; row < weights.rows(); row++) {
  //     weights(col, row) = 0;
  //       }
  //     }
  //     return weights;
  //   }

  ImageView<double> weights;

  // The copying seems necessary at each stage to get the correct answer
  if (min_blend_size <= 0)
    weights = grassfire(img);
  else
    weights = vw::copy(grassfire(vw::copy(vw::fill_holes_grass(vw::copy(img), min_blend_size))));

  // We will later count on this precise logic.
  for (int col = 0; col < weights.cols(); col++) {
    for (int row = 0; row < weights.rows(); row++) {
      weights(col, row) = pow(std::min(weights(col, row)/blending_dist, 1.0),
                              blending_power);
    }
  }
  return weights;
}
  
// Find the points on a given DEM that are shadowed by other points of
// the DEM.  Start marching from the point on the DEM on a ray towards
// the sun in small increments, until hitting the maximum DEM height.
bool isInShadow(int col, int row, Vector3 const& sunPos,
                ImageView<double> const& dem, double max_dem_height,
                double gridx, double gridy,
                cartography::GeoReference const& geo){

  // Here bicubic interpolation won't work. It is easier to interpret
  // the DEM as piecewise-linear when dealing with rays intersecting
  // it.
  InterpolationView<EdgeExtensionView< ImageView<double>,
    ConstantEdgeExtension >, BilinearInterpolation>
    interp_dem = interpolate(dem, BilinearInterpolation(),
                             ConstantEdgeExtension());

  // The xyz position at the center grid point
  Vector2 dem_llh = geo.pixel_to_lonlat(Vector2(col, row));
  Vector3 dem_lonlat_height = Vector3(dem_llh(0), dem_llh(1), dem(col, row));
  Vector3 xyz = geo.datum().geodetic_to_cartesian(dem_lonlat_height);

  // Normalized direction from the view point
  Vector3 dir = sunPos - xyz;
  if (dir == Vector3())
    return false;
  dir = dir/norm_2(dir);

  // The projection of dir onto the tangent plane at xyz,
  // that is, the "horizontal" component at the current sphere surface.
  Vector3 dir2 = dir - dot_prod(dir, xyz)*xyz/dot_prod(xyz, xyz);

  // Ensure that we advance by at most half a grid point each time
  double delta = 0.5*std::min(gridx, gridy)/std::max(norm_2(dir2), 1e-16);

  // Go along the ray. Don't allow the loop to go forever.
  for (int i = 1; i < 10000000; i++) {
    Vector3 ray_P = xyz + i * delta * dir;
    Vector3 ray_llh = geo.datum().cartesian_to_geodetic(ray_P);
    if (ray_llh[2] > max_dem_height) {
      // We're above the highest terrain, no point in continuing
      return false;
    }

    // Compensate for any longitude 360 degree offset, e.g., 270 deg vs -90 deg
    ray_llh[0] += 360.0*round((dem_llh[0] - ray_llh[0])/360.0);

    Vector2 ray_pix = geo.lonlat_to_pixel(Vector2(ray_llh[0], ray_llh[1]));

    if (ray_pix[0] < 0 || ray_pix[0] > dem.cols() - 1 ||
        ray_pix[1] < 0 || ray_pix[1] > dem.rows() - 1 ) {
      return false; // got out of the DEM, no point continuing
    }

    // Dem height at the current point on the ray
    double dem_h = interp_dem(ray_pix[0], ray_pix[1]);

    if (ray_llh[2] < dem_h) {
      // The ray goes under the DEM, so we are in shadow.
      return true;
    }
  }

  return false;
}

void areInShadow(Vector3 const& sunPos, ImageView<double> const& dem,
                 double gridx, double gridy,
                 cartography::GeoReference const& geo,
                 ImageView<float> & shadow){

  // Find the max DEM height
  double max_dem_height = -std::numeric_limits<double>::max();
  for (int col = 0; col < dem.cols(); col++) {
    for (int row = 0; row < dem.rows(); row++) {
      if (dem(col, row) > max_dem_height) {
        max_dem_height = dem(col, row);
      }
    }
  }

  shadow.set_size(dem.cols(), dem.rows());
  for (int col = 0; col < dem.cols(); col++) {
    for (int row = 0; row < dem.rows(); row++) {
      shadow(col, row) = isInShadow(col, row, sunPos, dem,
                                    max_dem_height, gridx, gridy, geo);
    }
  }
}
  
} // end namespace asp
