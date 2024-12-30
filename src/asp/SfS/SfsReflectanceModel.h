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

// \file SfsReflectanceModel.h
// The reflectance models used by SfS.

#ifndef __ASP_SFS_SFS_REFLECTANCE_MODEL_H__
#define __ASP_SFS_SFS_REFLECTANCE_MODEL_H__

#include <vw/Math/Vector.h>
namespace asp {

enum REFL_TYPE {NO_REFL = 0, LAMBERT, LUNAR_LAMBERT, HAPKE, ARBITRARY_MODEL, CHARON};

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

} // end namespace asp

#endif // __ASP_SFS_SFS_REFLECTANCE_MODEL_H__
