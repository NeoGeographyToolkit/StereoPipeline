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

// \file SfsModel.h
// Modeling reflectance, intensity, albedo for SfS

#ifndef __ASP_SFS_SFS_MODEL_H__
#define __ASP_SFS_SFS_MODEL_H__

#include <vw/Camera/CameraModel.h>
#include <vw/Image/MaskViews.h>
#include <vw/Image/ImageViewRef.h>
#include <vw/Math/Vector.h>

namespace vw { namespace cartography {
  class GeoReference;
}} // namespace vw::cartography

namespace asp {

// Maximum number of reflectance model and haze coefficients
const size_t g_num_model_coeffs = 16;
const size_t g_max_num_haze_coeffs = 6;

typedef vw::ImageViewRef<vw::PixelMask<float>> MaskedImgT;
typedef vw::ImageViewRef<double> DoubleImgT;

enum REFL_TYPE {NO_REFL = 0, LAMBERT, LUNAR_LAMBERT, HAPKE, ARBITRARY_MODEL, CHARON};

class HeightErrEstim;
class SlopeErrEstim;
class SfsOptions;

struct ReflParams {
  int reflectanceType;
  // Two parameters used in the formula for the Lunar-Lambertian
  // reflectance
  double phaseCoeffC1, phaseCoeffC2;
};

// Reflectance formula that is nonlinear if there is more than one haze coefficient
// (that is experimental).
double nonlinReflectance(double reflectance, double exposure,
                         double steepness_factor,
                         double const* haze, int num_haze_coeffs);

// Computes the ground reflectance with a desired reflectance model.
double calcReflectance(vw::Vector3 const& cameraPosition, vw::Vector3 const& normal,
                       vw::Vector3 const& xyz, vw::Vector3 const& sun_position,
                       ReflParams const& refl_params,
                       const double * refl_coeffs);

// Computed intensity:
// albedo * nonlinReflectance(reflectance_i, exposures[i], haze, num_haze_coeffs) + haze[0]
// Cost function:
// sum_i | I_i - comp_intensity_i|^2
double calcIntensity(double albedo, double reflectance, double exposure,
                     double steepness_factor, double const* haze, int num_haze_coeffs);

// Calc albedo given the intensity. See calcIntensity().
double calcAlbedo(double intensity, double reflectance, double exposure,
                  double steepness_factor, double const* haze, int num_haze_coeffs);

// Calculate current ECEF position and normal vector for a given DEM pixel.
// This is an auxiliary function needed to compute the reflectance.
void calcPointAndNormal(int col, int row,
                        double left_h, double center_h, double right_h,
                        double bottom_h, double top_h,
                        bool use_pq, double p, double q, // dem partial derivatives
                        vw::cartography::GeoReference const& geo,
                        double gridx, double gridy,
                        // Outputs
                        vw::Vector3 & xyz, vw::Vector3 & normal);

// Compute the reflectance and intensity at a single pixel. Compute the slope and/or
// height error estimation if the pointer is not NULL.
bool calcPixReflectanceInten(double left_h, double center_h, double right_h,
                             double bottom_h, double top_h,
                             bool use_pq, double p, double q, // dem partial derivatives
                             int col, int row,
                             vw::ImageView<double>         const& dem,
                             vw::cartography::GeoReference const& geo,
                             bool model_shadows,
                             double max_dem_height,
                             double gridx, double gridy,
                             vw::Vector3       const & sunPosition,
                             asp::ReflParams   const & refl_params,
                             vw::BBox2i        const & crop_box,
                             MaskedImgT        const & image,
                             DoubleImgT        const & blend_weight,
                             bool blend_weight_is_ground_weight,
                             vw::CamPtr camera,
                             vw::PixelMask<double>   & reflectance,
                             vw::PixelMask<double>   & intensity,
                             double                  & ground_weight,
                             double            const * refl_coeffs,
                             asp::SfsOptions   const & opt,
                             asp::SlopeErrEstim      * slopeErrEstim = NULL,
                             asp::HeightErrEstim     * heightErrEstim = NULL);

// The value stored in the output intensity(i, j) is the one at entry
// (i - 1) * sample_col_rate + 1, (j - 1) * sample_row_rate + 1
// in the full image. For i = 0 or j = 0 invalid values are stored.
void computeReflectanceAndIntensity(vw::ImageView<double> const& dem,
                                    vw::ImageView<vw::Vector2> const& pq,
                                    vw::cartography::GeoReference const& geo,
                                    bool model_shadows,
                                    double & max_dem_height, // alias
                                    double gridx, double gridy,
                                    int sample_col_rate, int sample_row_rate,
                                    vw::Vector3 const& sunPosition,
                                    asp::ReflParams const& refl_params,
                                    vw::BBox2i const& crop_box,
                                    asp::MaskedImgT const  & image,
                                    asp::DoubleImgT const  & blend_weight,
                                    bool blend_weight_is_ground_weight,
                                    vw::CamPtr camera,
                                    vw::ImageView<vw::PixelMask<double>> & reflectance,
                                    vw::ImageView<vw::PixelMask<double>> & intensity,
                                    vw::ImageView<double>                & ground_weight,
                                    const double                 * refl_coeffs,
                                    asp::SfsOptions        const & opt,
                                    asp::SlopeErrEstim  * slopeErrEstim = NULL,
                                    asp::HeightErrEstim * heightErrEstim = NULL);

// Initalize the reflectance parameters based on user input
void setupReflectance(asp::ReflParams & refl_params, asp::SfsOptions & opt);
  
} // end namespace asp

#endif // __ASP_SFS_SFS_MODEL_H__
