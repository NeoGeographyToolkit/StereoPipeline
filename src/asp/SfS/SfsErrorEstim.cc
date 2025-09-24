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
#include <asp/SfS/SfsReflectanceModel.h>

#include <string>
#include <map>

namespace asp {

// Use this struct to keep track of height errors.
HeightErrEstim::
HeightErrEstim(int num_cols, int num_rows, int num_height_samples_in,
               double max_height_error_in, double nodata_height_val_in,
               vw::ImageView<double> const* albedo_in,
               asp::SfsOptions const* opt_in) {

  num_height_samples = num_height_samples_in; // TODO(oalexan1): This must be a parameter
  max_height_error   = max_height_error_in;   // TODO(oalexan1): This must be a parameter
  nodata_height_val  = nodata_height_val_in;

  albedo = albedo_in;
  opt = opt_in;

  image_iter = 0; // will be modified later

  height_error_vec.set_size(num_cols, num_rows);
  for (int col = 0; col < num_cols; col++) {
    for (int row = 0; row < num_rows; row++) {
      height_error_vec(col, row)[0] = -max_height_error;
      height_error_vec(col, row)[1] =  max_height_error;
    }
  }

} // end constructor

// Use this struct to keep track of slope errors.
SlopeErrEstim::
SlopeErrEstim(int num_cols, int num_rows, int num_a_samples_in, int num_b_samples_in,
              vw::ImageView<double> const* albedo_in, asp::SfsOptions const* opt_in) {

  num_a_samples = num_a_samples_in;
  num_b_samples = num_b_samples_in;
  albedo = albedo_in;
  opt = opt_in;

  image_iter = 0; // will be modified later

  // The maximum possible deviation from the normal in degrees
  max_angle = 90.0;

  slope_errs.resize(num_cols);
  for (int col = 0; col < num_cols; col++) {
    slope_errs[col].resize(num_rows);
    for (int row = 0; row < num_rows; row++) {
      slope_errs[col][row].resize(num_b_samples, max_angle);
    }
  }
} // end constructor

// Given the normal (slope) to the SfS DEM, find how different
// a slope can be from this before the computed intensity
// due to that slope is bigger than max_intensity_err.
void estimateSlopeError(vw::Vector3 const& cameraPosition,
                        vw::Vector3 const& normal,
                        vw::Vector3 const& xyz,
                        vw::Vector3 const& sunPosition,
                        asp::ReflParams const& refl_params,
                        const double * refl_coeffs,
                        double meas_intensity,
                        double max_intensity_err,
                        int col, int row, int image_iter,
                        asp::SfsOptions const& opt,
                        vw::ImageView<double> const& albedo,
                        asp::SlopeErrEstim * slopeErrEstim) {

  // Find the angle u from the normal to the z axis, and the angle v
  // from the x axis to the projection of the normal in the xy plane.
  double u = acos(normal[2]);
  double v = 0.0;
  if (normal[0] != 0.0 || normal[1] != 0.0)
    v = atan2(normal[1], normal[0]);

  double cv = cos(v), sv = sin(v), cu = cos(u), su = sin(u);
  vw::Vector3 n(cv*su, sv*su, cu);

  // Sanity check, these angles should give us back the normal
  if (vw::math::norm_2(normal - n) > 1e-8)
    vw::vw_throw(vw::LogicErr() << "Book-keeping error in slope estimation.\n");

  // Find the rotation R that transforms the vector (0, 0, 1) to the normal
  vw::Matrix3x3 R1, R2, R;

  R1[0][0] = cv;  R1[0][1] = -sv; R1[0][2] = 0;
  R1[1][0] = sv;  R1[1][1] =  cv; R1[1][2] = 0;
  R1[2][0] = 0;   R1[2][1] =  0;  R1[2][2] = 1;

  R2[0][0] = cu;  R2[0][1] =  0;  R2[0][2] = su;
  R2[1][0] = 0;   R2[1][1] =  1;  R2[1][2] = 0;
  R2[2][0] = -su; R2[2][1] =  0;  R2[2][2] = cu;

  R = R1 * R2;

  // We must have R * n0 = n
  vw::Vector3 n0(0, 0, 1);
  if (vw::math::norm_2(R*n0 - n) > 1e-8)
    vw::vw_throw(vw::LogicErr() << "Book-keeping error in slope estimation.\n");

  int num_a_samples = slopeErrEstim->num_a_samples;
  int num_b_samples = slopeErrEstim->num_b_samples;

  int num_cols = slopeErrEstim->slope_errs.size();
  int num_rows = slopeErrEstim->slope_errs[0].size();
  int num_b_samples2 = slopeErrEstim->slope_errs[0][0].size();

  if (num_b_samples != num_b_samples2)
    vw::vw_throw(vw::LogicErr()
              << "Book-keeping failure in estimating the slope error!\n");

  // Sample the set of unit vectors w which make the angle 'a' with
  // the normal. For that, start with w having angle 'a' with the z
  // axis, and angle 'b' between the projection of w onto the xy plane
  // and the x axis. Then apply the rotation R to it which will make
  // the angle between w and the normal be 'a'. By varying 'b' we will
  // sample all such angles.
  double deg2rad = M_PI/180.0;
  for (int b_iter = 0; b_iter < num_b_samples; b_iter++) {

    double b = 360.0 * double(b_iter)/num_b_samples;
    double cb = cos(deg2rad * b), sb = sin(deg2rad * b);

    for (int a_iter = 0; a_iter < num_a_samples; a_iter++) {

      double a = 90.0 * double(a_iter)/num_a_samples;

      if (slopeErrEstim->slope_errs[col][row][b_iter] < a) {
        // We already determined that the slope error can't be as big as
        // a, so there is no point to explore bigger angles
        break;
      }

      double ca = cos(deg2rad * a), sa = sin(deg2rad * a);

      vw::Vector3 w(cb*sa, sb*sa, ca);
      w = R*w;

      // Compute here dot product from w to n. Should be cos(a) for all b.
      double prod = dot_prod(w, normal);
      if (std::abs(prod - ca) > 1e-8)
        vw::vw_throw(vw::LogicErr() << "Book-keeping error in slope estimation.\n");

      // Compute the reflectance with the given normal
      vw::PixelMask<double> reflectance
        = asp::calcReflectance(cameraPosition, w, xyz, sunPosition,
                               refl_params, refl_coeffs);
      reflectance.validate();

      double comp_intensity
        = asp::calcIntensity(albedo(col, row), reflectance,
                             opt.image_exposures_vec[image_iter],
                             opt.steepness_factor,
                             &opt.image_haze_vec[image_iter][0],
                             opt.num_haze_coeffs);

      if (std::abs(comp_intensity - meas_intensity) > max_intensity_err) {
        // We exceeded the error budget, hence this is an upper bound on the slope
        slopeErrEstim->slope_errs[col][row][b_iter] = a;
        break;
      }

    }
  }

  return;
} // end function estimateSlopeError

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

} // end namespace asp
