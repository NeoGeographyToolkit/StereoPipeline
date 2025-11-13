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

#include <vw/Image/ImageView.h>
#include <boost/shared_ptr.hpp>

namespace asp {

class ReflParams;

// Use this struct to keep track of height errors.
struct HeightErrEstim {

  HeightErrEstim(int num_cols, int num_rows, int num_height_samples_in,
                 double max_height_error_in, double nodata_height_val_in,
                 vw::ImageView<double> const* albedo_in);

  int num_height_samples;
  vw::ImageView<double> const* albedo;
  vw::ImageView<vw::Vector2> height_error_vec;
  int image_iter;
  double max_height_error;
  double nodata_height_val;
};

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
                         asp::HeightErrEstim * heightErrEstim);

// Estimate the height error at each DEM pixel based on estimates for each image
void combineHeightErrors(boost::shared_ptr<HeightErrEstim> const& heightErrEstim,
                         SfsOptions const& opt, vw::cartography::GeoReference const& geo);

} // end namespace asp

#endif // __ASP_SFS_SFS_ERROR_ESTIM_H__
