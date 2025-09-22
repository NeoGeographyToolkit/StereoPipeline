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

// \file SfsErrorEstim.h
// SfS error estimation methods

#ifndef __ASP_SFS_SFS_ERROR_ESTIM_H__
#define __ASP_SFS_SFS_ERROR_ESTIM_H__

#include <asp/SfS/SfsOptions.h>
#include <asp/SfS/SfsReflectanceModel.h>

#include <vw/Image/ImageView.h>

namespace asp {

// Use this struct to keep track of height errors.
struct HeightErrEstim {

  HeightErrEstim(int num_cols, int num_rows, int num_height_samples_in,
                 double max_height_error_in, double nodata_height_val_in,
                 vw::ImageView<double> * albedo_in,
                 asp::SfsOptions * opt_in);

  int num_height_samples;
  vw::ImageView<double> * albedo;
  asp::SfsOptions * opt;
  vw::ImageView<vw::Vector2> height_error_vec;
  int image_iter;
  double max_height_error;
  double nodata_height_val;
};

// Use this struct to keep track of slope errors.
struct SlopeErrEstim {

  SlopeErrEstim(int num_cols, int num_rows, int num_a_samples_in, int num_b_samples_in,
                vw::ImageView<double> * albedo_in, SfsOptions * opt_in);

  int num_a_samples, num_b_samples;
  vw::ImageView<double> * albedo;
  asp::SfsOptions * opt;
  std::vector<std::vector<std::vector<double>>> slope_errs;
  int image_iter;
  double max_angle;
};

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
                        asp::SfsOptions & opt,
                        vw::ImageView<double> & albedo,
                        asp::SlopeErrEstim * slopeErrEstim);

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
                         asp::SfsOptions & opt,
                         vw::ImageView<double> & albedo,
                         asp::HeightErrEstim * heightErrEstim);

} // end namespace asp

#endif // __ASP_SFS_SFS_ERROR_ESTIM_H__
