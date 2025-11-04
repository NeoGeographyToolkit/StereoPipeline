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

/// \file SfsImageProc.cc
/// Image processing routines for SfS

#include <asp/SfS/SfsImageProc.h>
#include <asp/SfS/SfsOptions.h>
#include <asp/Core/BaseCameraUtils.h>

#include <vw/Core/Log.h>
#include <vw/Camera/CameraModel.h>
#include <vw/Cartography/GeoReference.h>
#include <vw/FileIO/DiskImageView.h>
#include <vw/FileIO/GdalWriteOptions.h>
#include <vw/Cartography/GeoReferenceUtils.h>
#include <vw/Image/Transform.h>
#include <vw/Image/InpaintView.h>
#include <vw/Image/Filter.h>
#include <vw/Image/NoDataAlg.h>
#include <boost/filesystem.hpp>

#include <string>

namespace fs = boost::filesystem;
using namespace vw;

namespace asp {

// Compute mean and standard deviation of two images. Do it where both are valid.
void calcJointStats(MaskedDblImgT const& I1,
                    MaskedDblImgT const& I2,
                    double & mean1, double & std1,
                    double & mean2, double & std2) {

  if (I1.cols() != I2.cols() || I1.rows() != I2.rows())
    vw_throw(ArgumentErr() << "Expecting two input images of same size.\n");

  mean1 = 0; std1 = 0;
  mean2 = 0; std2 = 0;

  double sum1 = 0.0, sum2 = 0.0, sum1_sq = 0.0, sum2_sq = 0.0, count = 0.0;
  for (int col = 0; col < I1.cols(); col++) {
    for (int row = 0; row < I1.rows(); row++) {

      if (!is_valid(I1(col, row)) || !is_valid(I2(col, row))) continue;

      count++;

      double val1 = I1(col, row); sum1 += val1; sum1_sq += val1*val1;
      double val2 = I2(col, row); sum2 += val2; sum2_sq += val2*val2;
    }
  }

  if (count > 0) {
    mean1 = sum1/count; std1 = sqrt(sum1_sq/count - mean1*mean1);
    mean2 = sum2/count; std2 = sqrt(sum2_sq/count - mean2*mean2);
  }

  return;
}

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

// Max image function, but with masked pixels
void maxImage(int cols, int rows,
              std::set<int> const& skip_images,
              std::vector<MaskedDblImgT> const& images,
              vw::ImageView<double> & max_image) {

  int num_images = images.size();

  max_image.set_size(cols, rows);
  for (int col = 0; col < cols; col++) {
    for (int row = 0; row < rows; row++) {

      // Initialize to 0
      max_image(col, row) = 0.0;

      // Iterate over images, except the skipped ones
      for (int image_iter = 0; image_iter < num_images; image_iter++) {

        if (skip_images.find(image_iter) != skip_images.end())
          continue;

        // Skip if meas intensity is invalid
        if (!is_valid(images[image_iter](col, row)))
          continue;

        double inten = images[image_iter](col, row).child();
        if (inten > max_image(col, row))
          max_image(col, row) = inten;
      }
    }
  }
}

double maxDemHeight(vw::ImageView<double> const& dem) {

  double max_dem_height = -std::numeric_limits<double>::max();
  for (int col = 0; col < dem.cols(); col++) {
    for (int row = 0; row < dem.rows(); row++) {
      if (dem(col, row) > max_dem_height) {
        max_dem_height = dem(col, row);
      }
    }
  }

  return max_dem_height;
}

// TODO(oalexan1): The albedo must have its own no-data value.
// Must check the albedo has everywhere valid values.
double meanAlbedo(vw::ImageView<double> const& dem,
                  vw::ImageView<double> const& albedo,
                  double dem_nodata_val) {

  double mean_albedo = 0.0, albedo_count = 0.0;
  for (int col = 0; col < dem.cols(); col++) {
    for (int row = 0; row < dem.rows(); row++) {
      if (dem(col, row) != dem_nodata_val) {
        mean_albedo += albedo(col, row);
        albedo_count += 1.0;
      }
    }
  }

  if (albedo_count > 0)
    mean_albedo /= albedo_count;
  else
    mean_albedo = 0.0; // Or some other sensible default

  return mean_albedo;
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

// Saves the ground weight images
void saveGroundWeights(std::set<int> const& skip_images,
                       std::string const& out_prefix,
                       std::vector<std::string> const& input_images,
                       std::vector<std::string> const& input_cameras,
                       std::vector<vw::ImageView<double>> const& ground_weights,
                       vw::cartography::GeoReference const& geo,
                       vw::GdalWriteOptions const& opt) {

  int num_images = input_images.size();
  float img_nodata_val = -std::numeric_limits<float>::max(); // part of api

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
}

// Adjust the weights at the boundary of the max-lit region
// so they don't decay to 0, as there we need all the image data
// we get, and there's no concern that we'll create a seam
// with some other better data. Weights which are already 0 by then
// but are non-zero close to that interface get grown too, depending,
// on how close they get to that interface.
// We do not recompute these weights as the DEM changes, which is an approximation.
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
           max_weight); // output

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
    float img_nodata_val = -std::numeric_limits<float>::max(); // part of api
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

    // Save the adjusted ground weights
    saveGroundWeights(skip_images, out_prefix, input_images,
                      input_cameras, ground_weights, geo, opt);
  } // end saving debug info

  return;
}

// Find the blending weight for an image. This is 1 inside the lit area and away
// from any edges or shadows, then linearly decrease to 0 towards such edges,
// and then is raised to a power to change how it decays to 0.
// Do not make these decrease around holes smaller than min_blend_size,
// TODO(oalexan1): Grassfire weights use the Manhattan distance, which
// result in noisy weights. The Euclidean distance to boundary would work
// better.
vw::ImageView<double> blendingWeights(MaskedImgRefT const& img,
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
                cartography::GeoReference const& geo) {

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
        ray_pix[1] < 0 || ray_pix[1] > dem.rows() - 1) {
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
                 ImageView<float> & shadow) {

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

// Prototype code to identify permanently shadowed areas
// and deepen the craters there. Needs to be integrated
// and tested with various shapes of the deepened crater.
void deepenCraters(std::string const& dem_file,
                   std::vector<std::string> const& image_files,
                   double sigma,
                   std::string const& max_img_file,
                   std::string const& grass_file,
                   std::string const& out_dem_file) {

  float dem_nodata_val = -std::numeric_limits<float>::max();
  if (vw::read_nodata_val(dem_file, dem_nodata_val))
    vw_out() << "Dem nodata: " << dem_nodata_val << "\n";

  ImageView<PixelMask<float>> dem (create_mask(DiskImageView<float>(dem_file), dem_nodata_val));
  vw::cartography::GeoReference georef;
  if (!read_georeference(georef, dem_file))
    vw_throw(ArgumentErr() << "The input DEM " << dem_file << " has no georeference.\n");

  // The maximum of all valid pixel values with no-data where there is no-valid data.
  ImageView<PixelMask<float>> max_img(dem.cols(), dem.rows());
  for (int col = 0; col < dem.cols(); col++) {
    for (int row = 0; row < dem.rows(); row++) {
      max_img(col, row) = dem_nodata_val;
      max_img(col, row).invalidate();
    }
  }

  for (int i = 1; i < image_files.size(); i++) {

    std::string img_file = image_files[i];
    float img_nodata_val = -std::numeric_limits<float>::max();
    if (vw::read_nodata_val(img_file, img_nodata_val)) {
      vw_out() << "Img nodata: " << img_nodata_val << std::endl;
    }

    ImageView<PixelMask<float>> img(create_mask(DiskImageView<float>(img_file),
                                                img_nodata_val));
    if (img.cols() != dem.cols() || img.rows() != dem.rows()) {
      vw_throw(ArgumentErr() << "Images and DEM must have same size.\n");
    }

    for (int col = 0; col < img.cols(); col++) {
      for (int row = 0; row < img.rows(); row++) {

        // Nothing to do if the current image has invalid data
        if (!is_valid(img(col, row)))
          continue;

        // If the output image is not valid yet, copy the current image's valid pixel
        if (!is_valid(max_img(col, row) && img(col, row).child() > 0)) {
          max_img(col, row) = img(col, row);
          continue;
        }

        // Now both the current image and the output image are valid
        if (img(col, row).child() > max_img(col, row).child() &&
            img(col, row).child() > 0) {
          max_img(col, row) = img(col, row);
        }

      }
    }
  }

  // At the boundary the intensity is always invalid, but that is due to
  // computational limitations. Make it valid if we can.
  // TODO: Test here that the image has at least 3 rows and 3 cols!
  for (int col = 0; col < max_img.cols(); col++) {
    for (int row = 0; row < max_img.rows(); row++) {
      if ((col == 0 || col == max_img.cols() - 1) ||
           (row == 0 || row == max_img.rows() - 1)) {
        int next_col = col, next_row = row;
        if (col == 0) next_col = 1;
        if (col == max_img.cols() - 1) next_col = max_img.cols() - 2;
        if (row == 0) next_row = 1;
        if (row == max_img.rows() - 1) next_row = max_img.rows() - 2;

        if (!is_valid(max_img(col, row)) && is_valid(max_img(next_col, next_row)))
          max_img(col, row) = max_img(next_col, next_row);
      }
    }
  }

  GdalWriteOptions opt;
  bool has_nodata = true, has_georef = true;
  TerminalProgressCallback tpc("", "\t--> ");

  vw_out() << "Writing: " << max_img_file << "\n";
  block_write_gdal_image(max_img_file, apply_mask(max_img, dem_nodata_val),
                         has_georef, georef,
                         has_nodata, dem_nodata_val,
                         opt, tpc);

  ImageView<double> grass = grassfire(notnodata(select_channel(max_img, 0), dem_nodata_val));

  // Scale as craters are shallow.
  // TODO: Need to think of a better algorithm!
  for (int col = 0; col < grass.cols(); col++) {
    for (int row = 0; row < grass.rows(); row++) {
      grass(col, row) *= 0.2;
    }
  }

  // Blur with a given sigma
  ImageView<double> blurred_grass;
  if (sigma > 0)
    blurred_grass = gaussian_filter(grass, sigma);
  else
    blurred_grass = copy(grass);

  vw_out() << "Writing: " << grass_file << "\n";

  bool grass_has_nodata = false;
  block_write_gdal_image(grass_file, blurred_grass,
                         has_georef, georef,
                         grass_has_nodata, dem_nodata_val,
                         opt, tpc);

  // Bias the DEM by that grassfire height deepening the craters
  for (int col = 0; col < dem.cols(); col++) {
    for (int row = 0; row < dem.rows(); row++) {
      if (is_valid(dem(col, row))) {
        dem(col, row).child() -= blurred_grass(col, row);
      }
    }
  }

  vw_out() << "Writing: " << out_dem_file << "\n";
  block_write_gdal_image(out_dem_file, apply_mask(dem, dem_nodata_val),
                         has_georef, georef,
                         has_nodata, dem_nodata_val,
                         opt, tpc);

}

// Sample large DEMs. Keep about num_samples row and column samples.
void calcSampleRates(vw::ImageViewRef<double> const& dem, int num_samples,
                     int & sample_col_rate, int & sample_row_rate) {

  if (num_samples <= 0)
    vw_throw(ArgumentErr() << "Expecting a positive number of samples.\n");

  sample_col_rate = std::max((int)round(dem.cols()/double(num_samples)), 1);
  sample_row_rate = std::max((int)round(dem.rows()/double(num_samples)), 1);
}

// Compute a full-resolution image by specific interpolation into a low-resolution
// one. The full-res image may not fit in memory, so we need to compute it in tiles.
// See computeReflectanceAndIntensity() for low-res vs full-res relationship.
SfsInterpView::SfsInterpView(int full_res_cols, int full_res_rows,
                             int sample_col_rate, int sample_row_rate,
                             vw::ImageView<float> const& lowres_img):
    m_full_res_cols(full_res_cols), m_full_res_rows(full_res_rows),
    m_sample_col_rate(sample_col_rate), m_sample_row_rate(sample_row_rate),
    m_lowres_img(lowres_img) {
  }

// Per-pixel operation not implemented
SfsInterpView::pixel_type
SfsInterpView::operator()(double/*i*/, double/*j*/, vw::int32/*p*/) const {
  vw::vw_throw(vw::NoImplErr() << "SfsInterpView::operator()(...) is not implemented");
  return SfsInterpView::pixel_type();
}

// Per-tile operation
SfsInterpView::prerasterize_type SfsInterpView::prerasterize(vw::BBox2i const& bbox) const {

  vw::InterpolationView<vw::EdgeExtensionView<vw::ImageView<float>,
    vw::ConstantEdgeExtension>, vw::BilinearInterpolation>
    interp_lowres_img
      = vw::interpolate(m_lowres_img,
                        vw::BilinearInterpolation(),
                        vw::ConstantEdgeExtension());

  vw::ImageView<result_type> tile(bbox.width(), bbox.height());
  for (int col = bbox.min().x(); col < bbox.max().x(); col++) {
    for (int row = bbox.min().y(); row < bbox.max().y(); row++) {
      double valx = (col - 1.0) / double(m_sample_col_rate) + 1.0;
      double valy = (row - 1.0) / double(m_sample_row_rate) + 1.0;
      tile(col - bbox.min().x(), row - bbox.min().y())
        = interp_lowres_img(valx, valy);
    }
  }

  return prerasterize_type(tile, -bbox.min().x(), -bbox.min().y(),
                           cols(), rows());
}

// Factor out the logic for updating masks and weights
void updateMasksWeights(SfsOptions const& opt,
                        int num_images,
                        std::vector<vw::BBox2i> const& crop_boxes,
                        std::vector<vw::ImageView<double>> & ground_weights,
                        float & img_nodata_val,
                        std::vector<MaskedImgRefT> & masked_images,
                        std::vector<vw::ImageView<double>> & blend_weights) {

  // Redo the image masks. Unlike before, the shadow threshold is set to 0
  // to allow shadow pixels. The weights will control how much of these
  // are actually used. This approach is better than a hard cutoff with the mask.
  for (int image_iter = 0; image_iter < num_images; image_iter++) {
    if (opt.skip_images.find(image_iter) != opt.skip_images.end())
      continue;

    std::string img_file = opt.input_images[image_iter];
    vw::read_nodata_val(img_file, img_nodata_val);
    float shadow_thresh = 0.0; // Note how the shadow thresh is now 0, unlike before
    // Make a copy in memory for faster access
    if (!crop_boxes[image_iter].empty()) {
      vw::ImageView<float> cropped_img =
        crop(DiskImageView<float>(img_file), crop_boxes[image_iter]);
      masked_images[image_iter]
        = create_pixel_range_mask2(cropped_img,
                                   std::max(img_nodata_val, shadow_thresh),
                                   opt.max_valid_image_vals_vec[image_iter]);

      // Overwrite the blending weights with ground weights
      blend_weights[image_iter] = copy(ground_weights[image_iter]);
    }
  }

  ground_weights.clear(); // not needed anymore
}

// Pixels in low-light are given less weight if at the same location
// there exist pixels in other images with stronger light. This if a fix
// for seams. Must be used only in clips known to have seams.
// TODO(oalexan1): This is work in progress but it is promising.
void handleLowLight(SfsOptions const& opt,
                    vw::ImageView<double> const& dem,
                    vw::cartography::GeoReference const& geo,
                    std::vector<MaskedDblImgT> const& meas_intensities,
                    std::vector<MaskedDblImgT> const& comp_intensities,
                    std::vector<vw::ImageView<double>> & blend_weights) {

  int num_images = meas_intensities.size();

  // Find the max meas image
  vw::ImageView<double> max_intensity;
  maxImage(dem.cols(), dem.rows(), opt.skip_images, meas_intensities,
            max_intensity); // output

  // Find the abs diff (error) between meas and comp inten
  std::vector<ImageView<double>> err(num_images);
  
  // Find median error  
  std::vector<double> errs;  
  for (int image_iter = 0; image_iter < num_images; image_iter++) {

    if (opt.skip_images.find(image_iter) != opt.skip_images.end())
      continue;

    for (int col = 0; col < dem.cols(); col++) {
      for (int row = 0; row < dem.rows(); row++) {

        // Find the meas and comp intensities
        auto meas_intensity = meas_intensities[image_iter](col, row);
        auto comp_intensity = comp_intensities[image_iter](col, row);

        // Set to zero if either intensity is invalid
        if (!is_valid(meas_intensity) || !is_valid(comp_intensity)) {
          continue;
        }

        double curr_err  
          = std::abs(meas_intensity.child() - comp_intensity.child());
        errs.push_back(curr_err);
      }
    }
  }
  double median_err = vw::math::destructive_median(errs);

  // Find the error images
  for (int image_iter = 0; image_iter < num_images; image_iter++) {

    if (opt.skip_images.find(image_iter) != opt.skip_images.end())
      continue;

    err[image_iter].set_size(dem.cols(), dem.rows());
    for (int col = 0; col < dem.cols(); col++) {
      for (int row = 0; row < dem.rows(); row++) {

        // Initalize these to 0, except for skipped images
        err[image_iter](col, row) = 0.0;

        // Find the meas and comp intensities
        auto meas_intensity = meas_intensities[image_iter](col, row);
        auto comp_intensity = comp_intensities[image_iter](col, row);

        // Set to zero if either intensity is invalid
        if (!is_valid(meas_intensity) || !is_valid(comp_intensity)) {
          err[image_iter](col, row) = 0.0;
          continue;
        }

        err[image_iter](col, row) 
          = std::abs(meas_intensity.child() - comp_intensity.child());
      }
    }
  }
  
  // Find the adjustment weights for low light
  std::vector<ImageView<double>> adj_weights(num_images);
  for (int image_iter = 0; image_iter < num_images; image_iter++) {

    if (opt.skip_images.find(image_iter) != opt.skip_images.end())
      continue;

    adj_weights[image_iter].set_size(dem.cols(), dem.rows());
    for (int col = 0; col < adj_weights[image_iter].cols(); col++) {
      for (int row = 0; row < adj_weights[image_iter].rows(); row++) {

        // Initalize these to 1, except for skipped images
        adj_weights[image_iter](col, row) = 1.0;

        // Find the curr intensity
        double curr_intensity = meas_intensities[image_iter](col, row).child();

        // Set to zero if meas intensity is invalid or below shadow threshold
        if (!is_valid(meas_intensities[image_iter](col, row)) ||
            curr_intensity < opt.shadow_threshold_vec[image_iter]) {
          adj_weights[image_iter](col, row) = 0.0;
          continue;
        }

        // Skip if meas intensity is above the low light threshold
        if (curr_intensity > opt.low_light_threshold)
          continue;

        // Skip if curr intensity equals max intensity, to respect
        // --adjust-borderline-data)
        if (curr_intensity == max_intensity(col, row))
          continue;
          
        // TODO(oalexan1): Should one make use of the intensity  
        // double img_ratio = (curr_intensity - th) / (opt.low_light_threshold - th);
        // // Must be non-negative here
        // img_ratio = std::max(0.0, img_ratio);
        
        double err_ratio = 1.0;
        double curr_err = err[image_iter](col, row);
        if (curr_err > 0.0 && curr_err >= median_err)
          err_ratio = median_err / curr_err;
        
        // TODO(oalexan1): The power must be a param. 4 works well.
        adj_weights[image_iter](col, row) = pow(err_ratio, 4.0);
      }
    }
  }
  
  // Blur the adjustment weights
  for (int image_iter = 0; image_iter < num_images; image_iter++) {
    if (opt.skip_images.find(image_iter) != opt.skip_images.end())
      continue;
    // TODO(oalexan1): Must put here a parameter for the sigma
    adj_weights[image_iter] = gaussian_filter(adj_weights[image_iter], 10.0);
  }  

  // TODO(oalexan1): Must again put back the original weight when the intensity equals
  // the max intensity, to respect --adjust-borderline-data.
  
  // Adjust the blending weights by multiplying them with the adj weights for low light.
  for (int image_iter = 0; image_iter < num_images; image_iter++) {
    if (opt.skip_images.find(image_iter) != opt.skip_images.end())
      continue;
    for (int col = 0; col < blend_weights[image_iter].cols(); col++) {
      for (int row = 0; row < blend_weights[image_iter].rows(); row++) {
        blend_weights[image_iter](col, row) *= adj_weights[image_iter](col, row);
      }
    }
  }

  // Saves the ground weight images
  if (false) // for debugging
    asp::saveGroundWeights(opt.skip_images, opt.out_prefix,
                           opt.input_images, opt.input_cameras,
                           //blend_weights, 
                           adj_weights,
                           geo, vw::GdalWriteOptions(opt));
} // end function handleLowLight

// This will adjust the weights to account for borderline pixels and low-light conditions.
// The blend weights and masked images are modified in place.
void handleBorderlineAndLowLight(SfsOptions & opt,
                                 int num_images,
                                 vw::ImageView<double> const& dem,
                                 vw::cartography::GeoReference const& geo,
                                 std::vector<BBox2i> const& crop_boxes,
                                 std::vector<MaskedDblImgT> const& meas_intensities,
                                 std::vector<MaskedDblImgT> const& comp_intensities,
                                 // Outputs
                                 float & img_nodata_val,
                                 std::vector<MaskedImgRefT> & masked_images,
                                 std::vector<vw::ImageView<double>> & blend_weights,
                                 bool & blend_weight_is_ground_weight,
                                 std::vector<ImageView<double>> & ground_weights) {

  // Use the ground weights from now on instead of in-camera blending weights.
  // Will overwrite the weights below.
  blend_weight_is_ground_weight = true;

  // TODO(oalexan1): These weights should be created before any calculation
  // of intensity. As of now, they kick after that, and before iterative
  // SfS. Must check the effect of that. Should result in minor changes to
  // exposure only.
  int cols = dem.cols(), rows = dem.rows();
  asp::adjustBorderlineDataWeights(cols, rows, opt.blending_dist, opt.blending_power,
                                    vw::GdalWriteOptions(opt), // slice
                                    geo,
                                    opt.skip_images,
                                    opt.out_prefix, // for debug data
                                    opt.input_images, opt.input_cameras,
                                    ground_weights); // output

  // Recreate the the masked images and overwrite the blending weights
  asp::updateMasksWeights(opt, num_images, crop_boxes, ground_weights,
                          img_nodata_val, masked_images, blend_weights);

  // Handle --low-light-threshold option
  if (opt.low_light_threshold > 0.0)
    handleLowLight(opt, dem, geo, meas_intensities, comp_intensities, blend_weights);

} // end function handleBorderlineAndLowLight

} // end namespace asp
