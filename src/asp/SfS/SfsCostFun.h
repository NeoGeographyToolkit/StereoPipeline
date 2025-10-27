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

// \file SfsCostFun.h
// Cost function logic for SfS

#ifndef __ASP_SFS_SFS_COST_FUN_H__
#define __ASP_SFS_SFS_COST_FUN_H__

#include <asp/SfS/SfsErrorEstim.h>
#include <asp/SfS/SfsModel.h>

#include <vw/Image/ImageView.h>
#include <vw/Image/PixelMask.h>
#include <vw/Camera/CameraModel.h>
#include <vw/Cartography/GeoReference.h>

#include <ceres/ceres.h>
#include <ceres/loss_function.h>

namespace asp {

class SfsOptions;

// A function to invoke at every iteration of ceres.
class SfsCallback: public ceres::IterationCallback {

public:

  // Constructor to initialize references to the necessary data
  SfsCallback(asp::SfsOptions const& opt, 
              vw::ImageView<double>& dem,
              vw::ImageView<vw::Vector2>& pq,
              vw::ImageView<double>& albedo, 
              vw::cartography::GeoReference const& geo, 
              asp::ReflParams const& refl_params, 
              std::vector<vw::Vector3> const& sunPosition,
              std::vector<vw::BBox2i> const& crop_boxes, 
              std::vector<asp::MaskedImgRefT> const& masked_images,
              std::vector<asp::DoubleImgT> const& blend_weights,
              bool blend_weight_is_ground_weight,
              std::vector<vw::CamPtr>& cameras, 
              double& dem_nodata_val, float& img_nodata_val,
              std::vector<double>& exposures, 
              std::vector<std::vector<double>>& haze,
              double& max_dem_height, double& gridx, double& gridy,
              std::vector<double> & refl_coeffs);

  virtual ceres::CallbackReturnType 
  operator()(const ceres::IterationSummary& summary) override;

  void set_final_iter(bool is_final_iter);

private:

  // Class members storing references to the data
  asp::SfsOptions const& opt;
  vw::ImageView<double>& dem;
  vw::ImageView<vw::Vector2>& pq;
  vw::ImageView<double>& albedo;
  vw::cartography::GeoReference const& geo;
  asp::ReflParams const& refl_params;
  std::vector<vw::Vector3> const& sunPosition;
  std::vector<vw::BBox2i> const& crop_boxes;
  std::vector<asp::MaskedImgRefT> const& masked_images;
  std::vector<asp::DoubleImgT> const& blend_weights;
  bool blend_weight_is_ground_weight;
  std::vector<vw::CamPtr>& cameras;
  double& dem_nodata_val;
  float& img_nodata_val;
  std::vector<double>& exposures;
  std::vector<std::vector<double>>& haze;
  double& max_dem_height;
  double& gridx;
  double& gridy;
  std::vector<double> refl_coeffs;
  int iter;
  bool final_iter;
};

// Form the SfS cost function
void sfsCostFun(// Fixed quantities
                double                                gridx,
                double                                gridy,
                double                                smoothness_weight,
                double                                max_dem_height,
                double                                dem_nodata_val,
                float                                 img_nodata_val,
                bool                                  blend_weight_is_ground_weight,
                vw::cartography::GeoReference const & geo,
                std::vector<vw::BBox2i>       const & crop_boxes,
                std::vector<MaskedImgRefT>       const & masked_images,
                std::vector<DoubleImgT>       const & blend_weights,
                asp::ReflParams               const & refl_params,
                std::vector<vw::Vector3>      const & sunPosition,
                vw::ImageView<double>         const & orig_dem,
                vw::ImageView<int>            const & lit_image_mask,
                vw::ImageView<double>         const & curvature_in_shadow_weight,
                // Variable quantities
                asp::SfsOptions                     & opt,
                vw::ImageView<double>               & dem,
                vw::ImageView<double>               & albedo,
                std::vector<vw::CamPtr>             & cameras,
                std::vector<double>                 & exposures,
                std::vector<std::vector<double>>    & haze,
                std::vector<double>                 & refl_coeffs,
                vw::ImageView<vw::Vector2>          & pq,  
                ceres::Problem                      & problem);

// Find the best-fit exposure and haze given the input sampled image and reflectance.
// Also find the sampled albedo along the way. The albedo will be optimized
// only if --float-albedo is on. Otherwise it will be kept at the nominal value.
// TODO(oalexan1): Move this to SfsCostFun.cc.
void estimExposureHazeAlbedo(SfsOptions & opt,
                             std::vector<MaskedImgRefT> const& masked_images,
                             std::vector<DoubleImgT> const& blend_weights,
                             bool blend_weight_is_ground_weight,
                             vw::ImageView<double> const& dem,
                             double mean_albedo,
                             vw::cartography::GeoReference const& geo,
                             std::vector<vw::CamPtr> const& cameras,
                             double & max_dem_height,
                             std::vector<vw::BBox2i> const& crop_boxes,
                             std::vector<vw::Vector3> const& sunPosition,
                             asp::ReflParams const& refl_params,
                             double gridx, double gridy);

} // end namespace asp

#endif // __ASP_SFS_SFS_COST_FUN_H__
