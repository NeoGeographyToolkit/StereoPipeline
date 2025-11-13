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

// \file SfsErrorEstim.cc
// SfS error estimation methods

#include <asp/SfS/SfsErrorEstim.h>
#include <asp/SfS/SfsModel.h>

#include <vw/Core/Log.h>
#include <vw/Core/ProgressCallback.h>
#include <vw/FileIO/FileUtils.h>

#include <string>
#include <map>

namespace asp {

// Use this struct to keep track of height errors.
HeightErrEstim::
HeightErrEstim(int num_cols, int num_rows, int num_height_samples_in,
               double max_height_error_in, double nodata_height_val_in,
               vw::ImageView<double> const* albedo_in) {

  num_height_samples = num_height_samples_in; // TODO(oalexan1): This must be a parameter
  max_height_error   = max_height_error_in;   // TODO(oalexan1): This must be a parameter
  nodata_height_val  = nodata_height_val_in;

  albedo = albedo_in;

  image_iter = 0; // will be modified later

  height_error_vec.set_size(num_cols, num_rows);
  for (int col = 0; col < num_cols; col++) {
    for (int row = 0; row < num_rows; row++) {
      height_error_vec(col, row)[0] = -max_height_error;
      height_error_vec(col, row)[1] =  max_height_error;
    }
  }

} // end constructor

// Given the normal (height) to the SfS DEM, find how different
// a height can be from this before the computed intensity
// due to that height is bigger than max_intensity_err.
void estimateHeightError(vw::ImageView<double> const& dem,
                         vw::cartography::GeoReference const& geo,
                         vw::Vector3 const& cameraPosition,
                         vw::Vector3 const& sunPosition,
                         asp::ReflParams const& refl_params,
                         const double * refl_coeffs,
                         double meas_intensity,
                         double max_intensity_err,
                         int col, int row,
                         double grid_x, double grid_y,
                         int image_iter,
                         asp::SfsOptions const& opt,
                         vw::ImageView<double> const& albedo,
                         asp::HeightErrEstim * heightErrEstim) {

  // Look at the neighbors
  int cols[] = {col - 1, col,     col,     col + 1};
  int rows[] = {row,     row - 1, row + 1, row};
  for (int it = 0; it < 4; it++) {

    int colx = cols[it], rowx = rows[it];

    // Can't be at edges as need to compute normals
    if (colx <= 0 || rowx <= 0 || colx >= dem.cols() - 1 || rowx >= dem.rows() - 1)
      continue;

    // Perturb the height down and up
    for (int sign = -1; sign <= 1; sign += 2) {
      for (int height_it = 0; height_it < heightErrEstim->num_height_samples; height_it++) {
        double dh = sign * heightErrEstim->max_height_error
          * double(height_it)/double(heightErrEstim->num_height_samples);

        if (sign == -1) {
          if (dh < heightErrEstim->height_error_vec(colx, rowx)[0]) {
            // We already determined dh can't go as low, so stop here
            break;
          }
        } else if (sign == 1) {
          if (dh > heightErrEstim->height_error_vec(colx, rowx)[1]) {
            break;
          }
        }

        // Determine where to add the dh. Recall that we compute the intensity
        // at (col, row), while perturbing the dem height at (colx, rowx)
        double left_dh = 0, center_dh = 0, right_dh = 0, bottom_dh = 0, top_dh = 0;
        if (colx == col - 1 && rowx == row) left_dh   = dh;
        else if (colx == col     && rowx == row) center_dh = dh; // won't be reached
        else if (colx == col + 1 && rowx == row) right_dh  = dh;
        else if (colx == col     && rowx == row + 1) bottom_dh = dh;
        else if (colx == col     && rowx == row - 1) top_dh    = dh;

        double left_h   = dem(col - 1, row)     + left_dh;
        double center_h = dem(col,     row)     + center_dh;
        double right_h  = dem(col + 1, row)     + right_dh;
        double bottom_h = dem(col,     row + 1) + bottom_dh;
        double top_h    = dem(col,     row - 1) + top_dh;

        vw::Vector3 xyz, normal;
        bool use_pq = false;
        double p = 0.0, q = 0.0;
        calcPointAndNormal(col, row, left_h, center_h, right_h, bottom_h, top_h,
                           use_pq, p, q, geo, grid_x, grid_y, xyz, normal);

        vw::PixelMask<double> reflectance
           = asp::calcReflectance(cameraPosition, normal, xyz, sunPosition,
                                  refl_params, refl_coeffs);
        reflectance.validate();
        double comp_intensity
          = asp::calcIntensity(albedo(col, row), reflectance,
                               opt.image_exposures_vec[image_iter],
                               opt.steepness_factor, &opt.image_haze_vec[image_iter][0],
                               opt.num_haze_coeffs);

        if (std::abs(comp_intensity - meas_intensity) > max_intensity_err) {
          // We exceeded the error budget, record the dh at which it happens
          if (sign == -1) {
            heightErrEstim->height_error_vec(colx, rowx)[0] = dh;
          } else if (sign == 1) {
            heightErrEstim->height_error_vec(colx, rowx)[1] = dh;
          }

          break;
        }

      }
    }
  }

  return;
} // end function estimateHeightError

// Estimate the height error at each DEM pixel based on estimates for each image
void combineHeightErrors(boost::shared_ptr<HeightErrEstim> const& heightErrEstim,
                         SfsOptions const& opt, vw::cartography::GeoReference const& geo) {

  // Find the height error from the range of heights
  vw::ImageView<float> height_error;
  height_error.set_size(heightErrEstim->height_error_vec.cols(),
                        heightErrEstim->height_error_vec.rows());
  for (int col = 0; col < height_error.cols(); col++) {
    for (int row = 0; row < height_error.rows(); row++) {
      height_error(col, row)
        = std::max(-heightErrEstim->height_error_vec(col, row)[0],
                    heightErrEstim->height_error_vec(col, row)[1]);

      // When we are stuck at the highest error that means we could not find it
      if (height_error(col, row) == heightErrEstim->max_height_error)
        height_error(col, row) = heightErrEstim->nodata_height_val;
    }
  }
  
  vw::TerminalProgressCallback tpc("asp", ": ");
  bool has_georef = true, has_nodata = true;
  std::string height_error_file = opt.out_prefix + "-height-error.tif";
  vw::vw_out() << "Writing: " << height_error_file << std::endl;
  vw::cartography::block_write_gdal_image(height_error_file,
                                          height_error,
                                          has_georef, geo,
                                          has_nodata, heightErrEstim->nodata_height_val,
                                          opt, tpc);
}

} // end namespace asp
