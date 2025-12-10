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

// \file SfsCostFun.cc
// Cost function logic for SfS

#include <asp/SfS/SfsCostFun.h>
#include <asp/SfS/SfsUtils.h>
#include <asp/Core/BaseCameraUtils.h>
#include <asp/SfS/SfsOptions.h>
#include <asp/SfS/SfsImageProc.h>

#include <asp/asp_config.h> // defines ASP_HAVE_PKG_ISIS
#if defined(ASP_HAVE_PKG_ISIS) && ASP_HAVE_PKG_ISIS == 1
#include <asp/IsisIO/IsisCameraModel.h>
#endif // ASP_HAVE_PKG_ISIS

#include <vw/Core/ProgressCallback.h>
#include <vw/Core/Thread.h>

#include <boost/filesystem/path.hpp>

namespace fs = boost::filesystem;

namespace asp {

// A callback to invoke at every iteration of ceres

// Constructor to initialize references to the necessary data
SfsCallback::
SfsCallback(asp::SfsOptions const& opt,
            vw::ImageView<double>& dem,
            vw::ImageView<vw::Vector2>& pq,
            vw::ImageView<double>& albedo,
            vw::cartography::GeoReference const& geo,
            asp::ReflParams const& refl_params,
            std::vector<vw::Vector3> const& sunPosition,
            std::vector<vw::BBox2i> const& crop_boxes,
            std::vector<asp::MaskedImgRefT> const& masked_images,
            std::vector<asp::DblImgT> const& blend_weights,
            bool blend_weight_is_ground_weight,
            std::vector<vw::CamPtr>& cameras,
            double& dem_nodata_val, float& img_nodata_val,
            std::vector<double>& exposures,
            std::vector<std::vector<double>>& haze,
            double& max_dem_height, double& gridx, double& gridy,
            std::vector<double> & refl_coeffs):
  opt(opt), dem(dem), pq(pq), albedo(albedo), geo(geo), refl_params(refl_params),
  sunPosition(sunPosition), crop_boxes(crop_boxes), masked_images(masked_images),
  blend_weights(blend_weights), blend_weight_is_ground_weight(blend_weight_is_ground_weight),
  cameras(cameras), dem_nodata_val(dem_nodata_val),
  img_nodata_val(img_nodata_val), exposures(exposures), haze(haze),
  max_dem_height(max_dem_height), gridx(gridx), gridy(gridy),
  refl_coeffs(refl_coeffs),
  iter(-1), final_iter(false) {}

ceres::CallbackReturnType
SfsCallback::operator()(const ceres::IterationSummary& summary) {

  iter++;

  vw::vw_out() << "Finished iteration: " << iter << "\n";

  if (!opt.save_sim_intensity_only && !opt.save_meas_intensity_only)
    asp::saveExposures(opt.out_prefix, opt.input_images, exposures);

  if (opt.num_haze_coeffs > 0 && !opt.save_sim_intensity_only && !opt.save_meas_intensity_only)
    asp::saveHaze(opt.out_prefix, opt.input_images, haze);

  std::string model_coeffs_file = asp::modelCoeffsFileName(opt.out_prefix);
  if (!opt.save_sim_intensity_only && !opt.save_meas_intensity_only) {
    // Not needed, usually
    // vw::vw_out() << "Writing: " << model_coeffs_file << "\n";
    // std::ofstream mcf(model_coeffs_file.c_str());
    // mcf.precision(17);
    // for (size_t coeff_iter = 0; coeff_iter < refl_coeffs.size(); coeff_iter++)
    //   mcf << refl_coeffs[coeff_iter] << " ";
    // mcf << "\n";
    // mcf.close();
  }

  std::ostringstream os;
  if (!final_iter)
    os << "-iter" << iter;
  else
    os << "-final";

  std::string iter_str = os.str();

  // The DEM with no-data where there are no valid image pixels
  vw::ImageView<double> dem_nodata;
  if (opt.save_dem_with_nodata) {
    dem_nodata = vw::ImageView<double>(dem.cols(), dem.rows());
    vw::fill(dem_nodata, dem_nodata_val);
  }

  bool has_georef = true, has_nodata = true;
  vw::TerminalProgressCallback tpc("asp", ": ");
  if ((!opt.save_sparingly || final_iter) && !opt.save_sim_intensity_only && 
      !opt.save_meas_intensity_only) {
    std::string out_dem_file = opt.out_prefix + "-DEM"
      + iter_str + ".tif";
    vw::vw_out() << "Writing: " << out_dem_file << "\n";
    vw::cartography::block_write_gdal_image(out_dem_file, dem, has_georef, geo,
                                            has_nodata, dem_nodata_val,
                                            opt, tpc);
  }

  if ((!opt.save_sparingly || (final_iter && opt.float_albedo)) &&
      !opt.save_sim_intensity_only && !opt.save_meas_intensity_only) {
    std::string out_albedo_file;
    // For the final albedo do not use comp-albedo, as there is only one.
    // Only for intermediate iterations we contrast the computed albedo vs 
    // the measured one.
    if (!final_iter)
     out_albedo_file = opt.out_prefix + "-comp-albedo" + iter_str + ".tif";
    else
     out_albedo_file = opt.out_prefix + "-albedo" + iter_str + ".tif";
    
    vw::vw_out() << "Writing: " << out_albedo_file << "\n";
    vw::cartography::block_write_gdal_image(out_albedo_file, albedo,
                                            has_georef, geo,
                                            has_nodata, dem_nodata_val,
                                            opt, tpc);
  }

  // Print reflectance and other things
  for (size_t image_iter = 0; image_iter < masked_images.size(); image_iter++) {

    if (opt.skip_images.find(image_iter) != opt.skip_images.end())
      continue;

    MaskedDblImgT reflectance, intensity, sim_intensity;
    vw::ImageView<double> ground_weight;

    std::string out_camera_file
      = asp::bundle_adjust_file_name(opt.out_prefix,
                                     opt.input_images[image_iter],
                                     opt.input_cameras[image_iter]);

    if (opt.save_sparingly && !opt.save_dem_with_nodata)
      continue; // don't write too many things

    // Manufacture an output prefix for the other data associated with this camera
    std::string iter_str2 = fs::path(out_camera_file).replace_extension("").string();
    iter_str2 += iter_str;

    // Compute reflectance and intensity with optimized DEM
    int sample_col_rate = 1, sample_row_rate = 1;
    bool show_progress = false;
    computeReflectanceAndIntensity(dem, pq, geo,
                                  opt.model_shadows, show_progress,
                                  max_dem_height,
                                  gridx, gridy,
                                  sample_col_rate, sample_row_rate,
                                  sunPosition[image_iter],
                                  refl_params,
                                  crop_boxes[image_iter],
                                  masked_images[image_iter],
                                  blend_weights[image_iter],
                                  blend_weight_is_ground_weight,
                                  cameras[image_iter],
                                  reflectance, intensity, ground_weight,
                                  &refl_coeffs[0], opt);

    // dem_nodata equals to dem if the image has valid pixels and no shadows
    if (opt.save_dem_with_nodata) {
      for (int col = 0; col < reflectance.cols(); col++) {
        for (int row = 0; row < reflectance.rows(); row++) {
          if (vw::is_valid(reflectance(col, row)))
            dem_nodata(col, row) = dem(col, row);
        }
      }
    }

    if (opt.save_sparingly)
      continue;

    // Find the simulated intensity
    sim_intensity.set_size(reflectance.cols(), reflectance.rows());
    for (int col = 0; col < sim_intensity.cols(); col++) {
      for (int row = 0; row < sim_intensity.rows(); row++) {
        sim_intensity(col, row) 
          = calcSimIntensity(albedo(col, row), reflectance(col, row),
                             exposures[image_iter], opt.steepness_factor,
                             &haze[image_iter][0], opt.num_haze_coeffs);
      }
    }

    std::string out_meas_intensity_file = iter_str2 + "-meas-intensity.tif";
    vw::vw_out() << "Writing: " << out_meas_intensity_file << "\n";
    vw::cartography::block_write_gdal_image(out_meas_intensity_file,
                                            vw::apply_mask(intensity, img_nodata_val),
                                            has_georef, geo, has_nodata, img_nodata_val,
                                            opt, tpc);

    std::string out_sim_intensity_file = iter_str2 + "-sim-intensity.tif";
    vw::vw_out() << "Writing: " << out_sim_intensity_file << "\n";
    vw::cartography::block_write_gdal_image(out_sim_intensity_file,
                                            vw::apply_mask(sim_intensity, img_nodata_val),
                                            has_georef, geo, has_nodata, img_nodata_val,
                                            opt, tpc);

    if (opt.save_sim_intensity_only || opt.save_meas_intensity_only)
      continue; // don't write too many things
    
    // Not needed usually  
    // std::string out_weight_file = iter_str2 + "-blending-weight.tif";
    // vw::vw_out() << "Writing: " << out_weight_file << "\n";
    // vw::cartography::block_write_gdal_image(out_weight_file, ground_weight,
    //                                         has_georef, geo, has_nodata, img_nodata_val,
    //                                         opt, tpc);

    std::string out_reflectance_file = iter_str2 + "-reflectance.tif";
    vw::vw_out() << "Writing: " << out_reflectance_file << "\n";
    vw::cartography::block_write_gdal_image(out_reflectance_file,
                                            vw::apply_mask(reflectance, img_nodata_val),
                                            has_georef, geo, has_nodata, img_nodata_val,
                                            opt, tpc);

    // Find the measured normalized albedo, after correcting for
    // reflectance. Do not save this for the final iteration. This is a debug product
    // that is enabled only if not saving sparingly.
    if (!final_iter) {
      vw::ImageView<double> measured_albedo;
      measured_albedo.set_size(reflectance.cols(), reflectance.rows());
      for (int col = 0; col < measured_albedo.cols(); col++) {
        for (int row = 0; row < measured_albedo.rows(); row++) {
          if (!vw::is_valid(reflectance(col, row)))
            measured_albedo(col, row) = 1;
          else
            measured_albedo(col, row)
              = calcAlbedo(intensity(col, row), reflectance(col, row),
                          exposures[image_iter], opt.steepness_factor,
                          &haze[image_iter][0], opt.num_haze_coeffs);
        }
      }
      std::string out_albedo_file = iter_str2 + "-meas-albedo.tif";
      vw::vw_out() << "Writing: " << out_albedo_file << "\n";
      vw::cartography::block_write_gdal_image(out_albedo_file, measured_albedo,
                                              has_georef, geo, has_nodata, 0, opt, tpc);
    }
    
    vw::vw_out() << "Exposure for image " << image_iter << ": "
             << exposures[image_iter] << "\n";

    if (opt.num_haze_coeffs > 0) {
      vw::vw_out() << "Haze for image " << image_iter << ":";
      for (size_t hiter = 0; hiter < haze[image_iter].size(); hiter++) {
        vw::vw_out() << " " << haze[image_iter][hiter];
      }
      vw::vw_out() << "\n";
    }
  } // end loop through images

  if (opt.save_dem_with_nodata && (!opt.save_sparingly || final_iter)) {
    std::string out_dem_nodata_file = opt.out_prefix + "-DEM-nodata"
      + iter_str + ".tif";
    vw::vw_out() << "Writing: " << out_dem_nodata_file << "\n";
    vw::TerminalProgressCallback tpc("asp", ": ");
    vw::cartography::block_write_gdal_image(out_dem_nodata_file, dem_nodata,
                                            has_georef, geo,
                                            has_nodata, dem_nodata_val, opt, tpc);
  }

  return ceres::SOLVER_CONTINUE;
} // end callback function

void SfsCallback::set_final_iter(bool is_final_iter) {
  final_iter = is_final_iter;
}

// See SmoothnessError() for the definitions of bottom, top, etc.
template <typename F, typename G>
inline bool
calc_intensity_residual(SfsOptions const& opt,
                        const F* const exposure, const F* const haze,
                        const G* const left, const G* const center, const G* const right,
                        const G* const bottom, const G* const top,
                        bool use_pq, const G* const pq, // dem partial derivatives
                        const G* const albedo,
                        const G* const refl_coeffs,
                        int col, int row,
                        vw::ImageView<double>         const & dem,            // alias
                        vw::cartography::GeoReference const & geo,            // alias
                        bool                              model_shadows,
                        double                            camera_position_step_size,
                        double                    const & max_dem_height, // alias
                        double                            gridx,
                        double                            gridy,
                        ReflParams              const & refl_params,  // alias
                        vw::Vector3               const & sunPosition,   // alias
                        vw::BBox2i                            crop_box,
                        MaskedImgRefT                const & image,          // alias
                        DblImgT                const & blend_weight,   // alias
                        bool                              blend_weight_is_ground_weight,
                        vw::CamPtr                const & camera,         // alias
                        F* residuals) {

  // Default residuals. Using here 0 rather than some big number tuned out to
  // work better than the alternative.
  residuals[0] = F(0.0);
  try {
    vw::PixelMask<double> reflectance(0), intensity(0);
    double ground_weight = 0;

    // Need to be careful not to access an array which does not exist
    G p = 0, q = 0;
    if (use_pq) {
      p = pq[0];
      q = pq[1];
    }

    bool success =
      calcPixReflectanceInten(left[0], center[0], right[0],
                              bottom[0], top[0],
                              use_pq, p, q,
                              col, row,  dem, geo,
                              model_shadows, max_dem_height,
                              gridx, gridy,
                              sunPosition,  refl_params, crop_box, image,
                              blend_weight, blend_weight_is_ground_weight,
                              camera, reflectance, intensity, ground_weight,
                              refl_coeffs, opt);

    if (success && vw::is_valid(intensity) && vw::is_valid(reflectance))
      residuals[0] = ground_weight * (intensity -
                      calcSimIntensity(albedo[0], reflectance.child(), exposure[0],
                                       opt.steepness_factor, haze, opt.num_haze_coeffs));

  } catch (...) {
    // To be able to handle robustly DEMs that extend beyond the camera,
    // always return true when we fail to project, but with zero residual.
    // This needs more study.
    residuals[0] = F(0.0);
    return true;
  }

  return true;
}

// Discrepancy between measured and computed intensity. See the formula above.
struct IntensityError {
  IntensityError(SfsOptions const& opt, int col, int row,
                 vw::ImageView<double> const& dem,
                 vw::cartography::GeoReference const& geo,
                 bool model_shadows,
                 double camera_position_step_size,
                 double const& max_dem_height, // note: this is an alias
                 double gridx, double gridy,
                 ReflParams const& refl_params,
                 vw::Vector3 const& sunPosition,
                 vw::BBox2i const& crop_box,
                 MaskedImgRefT const& image,
                 DblImgT const& blend_weight,
                 bool blend_weight_is_ground_weight,
                 vw::CamPtr const& camera):
    m_opt(opt),
    m_col(col), m_row(row), m_dem(dem), m_geo(geo),
    m_model_shadows(model_shadows),
    m_camera_position_step_size(camera_position_step_size),
    m_max_dem_height(max_dem_height),
    m_gridx(gridx), m_gridy(gridy),
    m_refl_params(refl_params),
    m_sunPosition(sunPosition),
    m_crop_box(crop_box),
    m_image(image), m_blend_weight(blend_weight),
    m_blend_weight_is_ground_weight(blend_weight_is_ground_weight),
    m_camera(camera) {}

  // See SmoothnessError() for the definitions of bottom, top, etc.
  template <typename F>
  bool operator()(const F* const exposure, const F* const haze,
                  const F* const left, const F* const center, const F* const right,
                  const F* const bottom, const F* const top,
                  const F* const albedo, const F* const refl_coeffs,
                  F* residuals) const {

    // For this error we do not use p and q, hence just use a placeholder.
    bool use_pq = false;
    const F * const pq = NULL;
    return calc_intensity_residual(m_opt, exposure, haze,
                                   left, center, right, bottom, top,
                                   use_pq, pq, albedo,
                                   refl_coeffs,
                                   m_col, m_row,
                                   m_dem,  // alias
                                   m_geo,  // alias
                                   m_model_shadows,
                                   m_camera_position_step_size,
                                   m_max_dem_height,  // alias
                                   m_gridx, m_gridy,
                                   m_refl_params,   // alias
                                   m_sunPosition,    // alias
                                   m_crop_box,
                                   m_image,           // alias
                                   m_blend_weight,    // alias
                                   m_blend_weight_is_ground_weight,
                                   m_camera,          // alias
                                   residuals);
  }

  // Factory to hide the construction of the CostFunction object from
  // the client code.
  static ceres::CostFunction* Create(SfsOptions const& opt, int col, int row,
                                     vw::ImageView<double> const& dem,
                                     vw::cartography::GeoReference const& geo,
                                     bool model_shadows,
                                     double camera_position_step_size,
                                     double const& max_dem_height, // alias
                                     double gridx, double gridy,
                                     ReflParams const& refl_params,
                                     vw::Vector3 const& sunPosition,
                                     vw::BBox2i const& crop_box,
                                     MaskedImgRefT const& image,
                                     DblImgT const& blend_weight,
                                     bool blend_weight_is_ground_weight,
                                     vw::CamPtr const& camera) {
    return (new ceres::NumericDiffCostFunction<IntensityError,
            ceres::CENTRAL, 1, 1, g_max_num_haze_coeffs, 1, 1, 1, 1, 1, 1, g_num_model_coeffs>
            (new IntensityError(opt, col, row, dem, geo,
                                model_shadows,
                                camera_position_step_size,
                                max_dem_height,
                                gridx, gridy,
                                refl_params, sunPosition,
                                crop_box, image, blend_weight, blend_weight_is_ground_weight,
                                camera)));
  }

  SfsOptions                         const& m_opt;
  int                                       m_col, m_row;
  vw::ImageView<double>             const & m_dem;            // alias
  vw::cartography::GeoReference     const & m_geo;            // alias
  bool                                      m_model_shadows;
  double                                    m_camera_position_step_size;
  double                            const & m_max_dem_height; // alias
  double                                    m_gridx, m_gridy;
  asp::ReflParams                   const & m_refl_params;    // alias
  vw::Vector3                       const & m_sunPosition;    // alias
  vw::BBox2i                                m_crop_box;
  MaskedImgRefT                        const & m_image;          // alias
  DblImgT                        const & m_blend_weight;   // alias
  bool                                      m_blend_weight_is_ground_weight;
  vw::CamPtr                        const & m_camera;         // alias
};

// A variation of the intensity error where only the DEM is floated. This
// produces the same results as IntensityError (when most quantities are passed
// as variables that are later fixed), but uses notably less memory.
struct IntensityErrorFloatDemOnly {
  IntensityErrorFloatDemOnly(SfsOptions const& opt, int col, int row,
                             vw::ImageView<double> const& dem,
                             double albedo,
                             double * refl_coeffs,
                             double * exposure,
                             double * haze,
                             vw::cartography::GeoReference const& geo,
                             bool model_shadows,
                             double camera_position_step_size,
                             double const& max_dem_height, // note: this is an alias
                             double gridx, double gridy,
                             ReflParams const& refl_params,
                             vw::Vector3 const& sunPosition,
                             vw::BBox2i const& crop_box,
                             MaskedImgRefT const& image,
                             DblImgT const& blend_weight,
                             bool blend_weight_is_ground_weight,
                             vw::CamPtr const& camera):
    m_opt(opt), m_col(col), m_row(row), m_dem(dem),
    m_albedo(albedo), m_refl_coeffs(refl_coeffs),
    m_exposure(exposure), m_haze(haze), m_geo(geo),
    m_model_shadows(model_shadows),
    m_camera_position_step_size(camera_position_step_size),
    m_max_dem_height(max_dem_height),
    m_gridx(gridx), m_gridy(gridy),
    m_refl_params(refl_params),
    m_sunPosition(sunPosition),
    m_crop_box(crop_box),
    m_image(image), m_blend_weight(blend_weight),
    m_blend_weight_is_ground_weight(blend_weight_is_ground_weight),
    m_camera(camera) {}

  // See SmoothnessError() for the definitions of bottom, top, etc.
  template <typename F>
  bool operator()(const F* const left,
                  const F* const center,
                  const F* const right,
                  const F* const bottom,
                  const F* const top,
                  F* residuals) const {

    // For this error we do not use p and q, hence just use a placeholder.
    bool use_pq = false;
    const F * const pq = NULL;

    return calc_intensity_residual(m_opt, m_exposure, m_haze,
                                   left, center, right, bottom, top,
                                   use_pq, pq,
                                   &m_albedo,
                                   m_refl_coeffs,
                                   m_col, m_row,
                                   m_dem,  // alias
                                   m_geo,  // alias
                                   m_model_shadows,
                                   m_camera_position_step_size,
                                   m_max_dem_height,  // alias
                                   m_gridx, m_gridy,
                                   m_refl_params,   // alias
                                   m_sunPosition,    // alias
                                   m_crop_box,
                                   m_image,           // alias
                                   m_blend_weight,    // alias
                                   m_blend_weight_is_ground_weight,
                                   m_camera,          // alias
                                   residuals);
  }

  // Factory to hide the construction of the CostFunction object from
  // the client code.
  static ceres::CostFunction* Create(SfsOptions const& opt, int col, int row,
                                     vw::ImageView<double> const& dem,
                                     double albedo,
                                     double * refl_coeffs,
                                     double * exposure,
                                     double * haze,
                                     vw::cartography::GeoReference const& geo,
                                     bool model_shadows,
                                     double camera_position_step_size,
                                     double const& max_dem_height, // alias
                                     double gridx, double gridy,
                                     ReflParams const& refl_params,
                                     vw::Vector3 const& sunPosition,
                                     vw::BBox2i const& crop_box,
                                     MaskedImgRefT const& image,
                                     DblImgT const& blend_weight,
                                     bool blend_weight_is_ground_weight,
                                     vw::CamPtr const& camera) {
    return (new ceres::NumericDiffCostFunction<IntensityErrorFloatDemOnly,
            ceres::CENTRAL, 1, 1, 1, 1, 1, 1>
            (new IntensityErrorFloatDemOnly(opt, col, row, dem,
                                            albedo, refl_coeffs,
                                            exposure, haze, geo,
                                            model_shadows,
                                            camera_position_step_size,
                                            max_dem_height,
                                            gridx, gridy,
                                            refl_params, sunPosition,
                                            crop_box, image, blend_weight,
                                            blend_weight_is_ground_weight,
                                            camera)));
  }

  SfsOptions                const & m_opt;
  int                               m_col, m_row;
  vw::ImageView<double>     const & m_dem;            // alias
  double                            m_albedo;
  double                          * m_refl_coeffs;
  double                          * m_exposure;
  double                          * m_haze;
  vw::cartography::GeoReference const & m_geo;            // alias
  bool                              m_model_shadows;
  double                            m_camera_position_step_size;
  double                    const & m_max_dem_height; // alias
  double                            m_gridx, m_gridy;
  asp::ReflParams           const & m_refl_params;  // alias
  vw::Vector3               const & m_sunPosition;   // alias
  vw::BBox2i                        m_crop_box;
  MaskedImgRefT             const & m_image;          // alias
  DblImgT                   const & m_blend_weight;   // alias
  bool                              m_blend_weight_is_ground_weight;
  vw::CamPtr                const & m_camera;         // alias
}; // end class IntensityErrorFloatDemOnly

// A variant of the intensity error when we float the partial derivatives
// in x and in y of the dem, which we call p and q.
// TODO(oalexan1): Validate again if this gives better results as compared to
// usual intensity error.
struct IntensityErrorPQ {
  IntensityErrorPQ(SfsOptions const& opt, int col, int row,
                   vw::ImageView<double> const& dem,
                   vw::cartography::GeoReference const& geo,
                   bool model_shadows,
                   double camera_position_step_size,
                   double const& max_dem_height, // note: this is an alias
                   double gridx, double gridy,
                   ReflParams const& refl_params,
                   vw::Vector3 const& sunPosition,
                   vw::BBox2i const& crop_box,
                   MaskedImgRefT const& image,
                   DblImgT const& blend_weight,
                   bool blend_weight_is_ground_weight,
                   vw::CamPtr const& camera):
    m_opt(opt), m_col(col), m_row(row), m_dem(dem), m_geo(geo),
    m_model_shadows(model_shadows),
    m_camera_position_step_size(camera_position_step_size),
    m_max_dem_height(max_dem_height),
    m_gridx(gridx), m_gridy(gridy),
    m_refl_params(refl_params),
    m_sunPosition(sunPosition),
    m_crop_box(crop_box),
    m_image(image), m_blend_weight(blend_weight),
    m_blend_weight_is_ground_weight(blend_weight_is_ground_weight),
    m_camera(camera) {}

  // See SmoothnessError() for the definitions of bottom, top, etc.
  template <typename F>
  bool operator()(const F* const exposure,
                  const F* const haze,
                  const F* const center_h,
                  const F* const pq,                 // array of length 2
                  const F* const albedo,
                  const F* const refl_coeffs,
                  F* residuals) const {

    bool use_pq = true;

    F v = 0;
    return calc_intensity_residual(m_opt, exposure, haze, &v, center_h, &v, &v, &v,
                                   use_pq, pq, albedo,
                                   refl_coeffs,
                                   m_col, m_row,
                                   m_dem,  // alias
                                   m_geo,  // alias
                                   m_model_shadows,
                                   m_camera_position_step_size,
                                   m_max_dem_height,  // alias
                                   m_gridx, m_gridy,
                                   m_refl_params,   // alias
                                   m_sunPosition,    // alias
                                   m_crop_box,
                                   m_image,           // alias
                                   m_blend_weight,    // alias
                                   m_blend_weight_is_ground_weight,
                                   m_camera,          // alias
                                   residuals);
  }

  // Factory to hide the construction of the CostFunction object from the client
  // code.
  static ceres::CostFunction* Create(SfsOptions const& opt, int col, int row,
                                     vw::ImageView<double> const& dem,
                                     vw::cartography::GeoReference const& geo,
                                     bool model_shadows,
                                     double camera_position_step_size,
                                     double const& max_dem_height, // alias
                                     double gridx, double gridy,
                                     ReflParams const& refl_params,
                                     vw::Vector3 const& sunPosition,
                                     vw::BBox2i const& crop_box,
                                     MaskedImgRefT const& image,
                                     DblImgT const& blend_weight,
                                     bool blend_weight_is_ground_weight,
                                     vw::CamPtr const& camera) {
    return (new ceres::NumericDiffCostFunction<IntensityErrorPQ,
            ceres::CENTRAL, 1, 1, g_max_num_haze_coeffs, 1, 2, 1, g_num_model_coeffs>
            (new IntensityErrorPQ(opt, col, row, dem, geo,
                                  model_shadows,
                                  camera_position_step_size,
                                  max_dem_height,
                                  gridx, gridy,
                                  refl_params, sunPosition,
                                  crop_box, image, blend_weight,
                                  blend_weight_is_ground_weight,
                                  camera)));
  }

  SfsOptions                    const & m_opt;
  int m_col, m_row;
  vw::ImageView<double>         const & m_dem;            // alias
  vw::cartography::GeoReference const & m_geo;            // alias
  bool                                  m_model_shadows;
  double                                m_camera_position_step_size;
  double                        const & m_max_dem_height; // alias
  double                                m_gridx, m_gridy;
  ReflParams                    const & m_refl_params;  // alias
  vw::Vector3                   const & m_sunPosition;   // alias
  vw::BBox2i                            m_crop_box;
  MaskedImgRefT                 const & m_image;          // alias
  DblImgT                       const & m_blend_weight;   // alias
  bool                                  m_blend_weight_is_ground_weight;
  vw::CamPtr                    const & m_camera;         // alias
};

// Discrepancy between measured and computed intensity. Assume fixed reflectance,
// as this is only an initial estimate. See calcSimIntensity() for the formula.
struct IntensityErrorFixedReflectance {
  IntensityErrorFixedReflectance(vw::PixelMask<float> const& intensity,
                                 vw::PixelMask<float> const& reflectance,
                                 int num_haze_coeffs,
                                 double steepness_factor):
    m_intensity(intensity), m_reflectance(reflectance),
    m_num_haze_coeffs(num_haze_coeffs), m_steepness_factor(steepness_factor) {}

  template <typename F>
  bool operator()(const F* const exposure, const F* const haze, const F* const albedo,
                  F* residuals) const {

    if (!vw::is_valid(m_intensity) || !vw::is_valid(m_reflectance)) {
      residuals[0] = 0;
      return true;
    }
    
    residuals[0] = calcSimIntensity(albedo[0], m_reflectance, exposure[0],
                                    m_steepness_factor, haze, m_num_haze_coeffs)
                 - m_intensity;

    return true;
  }

  // Factory to hide the construction of the CostFunction object from
  // the client code.
  static ceres::CostFunction* Create(vw::PixelMask<float> const& intensity,
                                     vw::PixelMask<float> const& reflectance,
                                     int num_haze_coeffs,
                                     double steepness_factor) {
    return (new ceres::NumericDiffCostFunction<IntensityErrorFixedReflectance,
            ceres::CENTRAL,
            // residual, exposure, haze, albedo
            1, 1, g_max_num_haze_coeffs, 1>
            (new IntensityErrorFixedReflectance(intensity, reflectance,
                                                num_haze_coeffs, steepness_factor)));
  }

  vw::PixelMask<float> m_intensity, m_reflectance;
  int m_num_haze_coeffs;
  double m_steepness_factor;
};

// The smoothness error is the sum of squares of
// the 4 second order partial derivatives, with a weight:
// error = smoothness_weight * (u_xx^2 + u_xy^2 + u_yx^2 + u_yy^2)

// We will use finite differences to compute these.
// Consider a grid point and its neighbors, 9 points in all.
//
// bl   = u(c-1, r+1)  bottom = u(c, r+1) br    = u(c+1,r+1)
// left = u(c-1, r)  center = u(c, r) right = u(c+1,r)
// tl   = u(c-1, r-1)  top    = u(c, r-1) tr    = u(c+1,r-1)
//
// See https://en.wikipedia.org/wiki/Finite_difference
// for the obtained formulas.
struct SmoothnessError {
  SmoothnessError(double smoothness_weight, double gridx, double gridy):
    m_smoothness_weight(smoothness_weight),
    m_gridx(gridx), m_gridy(gridy) {}

  template <typename T>
  bool operator()(const T* const bl,   const T* const bottom,    const T* const br,
                  const T* const left, const T* const center,    const T* const right,
                  const T* const tl,   const T* const top,       const T* const tr,
                  T* residuals) const {

    // Normalize by grid size seems to make the functional less
    // sensitive to the actual grid size used.
    residuals[0] = (left[0] + right[0] - 2*center[0])/m_gridx/m_gridx;   // u_xx
    residuals[1] = (br[0] + tl[0] - bl[0] - tr[0])/4.0/m_gridx/m_gridy; // u_xy
    residuals[2] = residuals[1];                                         // u_yx
    residuals[3] = (bottom[0] + top[0] - 2*center[0])/m_gridy/m_gridy;   // u_yy

    for (int i = 0; i < 4; i++)
      residuals[i] *= m_smoothness_weight;

    return true;
  }

  // Factory to hide the construction of the CostFunction object from
  // the client code.
  static ceres::CostFunction* Create(double smoothness_weight,
                                     double gridx, double gridy) {
    return (new ceres::NumericDiffCostFunction<SmoothnessError,
            ceres::CENTRAL, 4, 1, 1, 1, 1, 1, 1, 1, 1, 1>
            (new SmoothnessError(smoothness_weight, gridx, gridy)));
  }

  double m_smoothness_weight, m_gridx, m_gridy;
};

// The gradient error is the sum of squares of
// the first order partial derivatives, with a weight:
// error = gradient_weight * (u_x^2 + u_y^2)

// We will use finite differences to compute these. See
// SmoothnessError() for more details.
struct GradientError {
  GradientError(double gradient_weight, double gridx, double gridy):
    m_gradient_weight(gradient_weight), m_gridx(gridx), m_gridy(gridy) {}

  template <typename T>
  bool operator()(const T* const bottom, const T* const left, const T* const center,
                  const T* const right, const T* const top, T* residuals) const {

    // This results in a smoother solution than using centered differences
    residuals[0] = (right[0]  - center[0])/m_gridx; // u_x
    residuals[1] = (center[0] - left[0])/m_gridx; // u_x
    residuals[2] = (top[0]    - center[0])/m_gridy; // u_y
    residuals[3] = (center[0] - bottom[0])/m_gridy; // u_y

    for (int i = 0; i < 4; i++)
      residuals[i] *= m_gradient_weight;

    return true;
  }

  // Factory to hide the construction of the CostFunction object from
  // the client code.
  static ceres::CostFunction* Create(double gradient_weight,
                                     double gridx, double gridy) {
    return (new ceres::NumericDiffCostFunction<GradientError,
            ceres::CENTRAL, 4, 1, 1, 1, 1, 1>
            (new GradientError(gradient_weight, gridx, gridy)));
  }

  double m_gradient_weight, m_gridx, m_gridy;
};

// Try to make the DEM in shadow have positive curvature. The error term is
// (curvature_weight *(terrain_xx + terrain_xy - curvature))^2 in the shadow,
// and not used in lit areas.
struct CurvatureInShadowError {
  CurvatureInShadowError(double curvature_in_shadow, double curvature_in_shadow_weight,
                         double gridx, double gridy):
    m_curvature_in_shadow(curvature_in_shadow),
    m_curvature_in_shadow_weight(curvature_in_shadow_weight),
    m_gridx(gridx), m_gridy(gridy) {}

  template <typename T>
  bool operator()(const T* const bottom, const T* const left, const T* const center,
                  const T* const right, const T* const top, T* residuals) const {

    // Normalize by grid size seems to make the functional less
    // sensitive to the actual grid size used.
    double u_xx = (left[0] + right[0] - 2*center[0])/m_gridx/m_gridx;   // u_xx
    double u_yy = (bottom[0] + top[0] - 2*center[0])/m_gridy/m_gridy;   // u_yy

    residuals[0] = m_curvature_in_shadow_weight*(u_xx + u_yy - m_curvature_in_shadow);

    return true;
  }

  // Factory to hide the construction of the CostFunction object from
  // the client code.
  static ceres::CostFunction* Create(double curvature_in_shadow,
                                     double curvature_in_shadow_weight,
                                     double gridx, double gridy) {
    return (new ceres::NumericDiffCostFunction<CurvatureInShadowError,
            ceres::CENTRAL, 1, 1, 1, 1, 1, 1>
            (new CurvatureInShadowError(curvature_in_shadow, curvature_in_shadow_weight,
                                        gridx, gridy)));
  }

  double m_curvature_in_shadow, m_curvature_in_shadow_weight, m_gridx, m_gridy;
};

struct SmoothnessErrorPQ {
  SmoothnessErrorPQ(double smoothness_weight_pq, double gridx, double gridy):
    m_smoothness_weight_pq(smoothness_weight_pq),
    m_gridx(gridx), m_gridy(gridy) {}

  template <typename T>
  bool operator()(const T* const bottom_pq, const T* const left_pq, const T* const right_pq,
                  const T* const top_pq, T* residuals) const {

    // Normalize by grid size seems to make the functional less
    // sensitive to the actual grid size used.
    residuals[0] = (right_pq[0] - left_pq[0])/(2*m_gridx);   // p_x
    residuals[1] = (top_pq[0] - bottom_pq[0])/(2*m_gridy);   // p_y
    residuals[2] = (right_pq[1] - left_pq[1])/(2*m_gridx);   // q_x
    residuals[3] = (top_pq[1] - bottom_pq[1])/(2*m_gridy);   // q_y

    for (int i = 0; i < 4; i++)
      residuals[i] *= m_smoothness_weight_pq;

    return true;
  }

  // Factory to hide the construction of the CostFunction object from
  // the client code.
  static ceres::CostFunction* Create(double smoothness_weight_pq,
                                     double gridx, double gridy) {
    return (new ceres::NumericDiffCostFunction<SmoothnessErrorPQ,
            ceres::CENTRAL, 4, 2, 2, 2, 2>
            (new SmoothnessErrorPQ(smoothness_weight_pq, gridx, gridy)));
  }

  double m_smoothness_weight_pq, m_gridx, m_gridy;
};

// The integrability error is the discrepancy between the
// independently optimized gradients p and q, and the partial
// derivatives of the dem, here denoted by u.
// error = integrability_weight * ((u_x - p)^2 + (u_y - q)^2)
// See SmoothnessError for the notation below.
struct IntegrabilityError {
  IntegrabilityError(double integrability_weight, double gridx, double gridy):
    m_integrability_weight(integrability_weight),
    m_gridx(gridx), m_gridy(gridy) {}

  template <typename T>
  bool operator()(const T* const bottom, const T* const left, const T* const right,
                  const T* const top, const T* const pq,
                  T* residuals) const {

    residuals[0] = (right[0] - left[0])/(2*m_gridx) - pq[0];
    residuals[1] = (top[0] - bottom[0])/(2*m_gridy) - pq[1];

    for (int i = 0; i < 2; i++)
      residuals[i] *= m_integrability_weight;

    return true;
  }

  // Factory to hide the construction of the CostFunction object from
  // the client code.
  static ceres::CostFunction* Create(double integrability_weight,
                                     double gridx, double gridy) {
    return (new ceres::NumericDiffCostFunction<IntegrabilityError,
            ceres::CENTRAL, 2, 1, 1, 1, 1, 2>
            (new IntegrabilityError(integrability_weight, gridx, gridy)));
  }

  double m_integrability_weight, m_gridx, m_gridy;
};

// A cost function that will penalize deviating too much from the original DEM height.
struct HeightChangeError {
  HeightChangeError(double orig_height, double initial_dem_constraint_weight):
    m_orig_height(orig_height),
    m_initial_dem_constraint_weight(initial_dem_constraint_weight) {}

  template <typename T>
  bool operator()(const T* const center, T* residuals) const {
    residuals[0] = (center[0] - m_orig_height)*m_initial_dem_constraint_weight;
    return true;
  }

  // Factory to hide the construction of the CostFunction object from
  // the client code.
  static ceres::CostFunction* Create(double orig_height,
                                     double initial_dem_constraint_weight) {
    return (new ceres::NumericDiffCostFunction<HeightChangeError,
            ceres::CENTRAL, 1, 1>
            (new HeightChangeError(orig_height, initial_dem_constraint_weight)));
  }

  double m_orig_height, m_initial_dem_constraint_weight;
};

// A cost function that will penalize deviating too much from the initial albedo.
struct AlbedoChangeError {
  AlbedoChangeError(double initial_albedo, double albedo_constraint_weight):
    m_initial_albedo(initial_albedo), m_albedo_constraint_weight(albedo_constraint_weight) {}

  template <typename T>
  bool operator()(const T* const center, T* residuals) const {
    residuals[0] = (center[0] - m_initial_albedo)*m_albedo_constraint_weight;
    return true;
  }

  // Factory to hide the construction of the CostFunction object from
  // the client code.
  static ceres::CostFunction* Create(double initial_albedo,
                                     double albedo_constraint_weight) {
    return (new ceres::NumericDiffCostFunction<AlbedoChangeError, ceres::CENTRAL, 1, 1>
            (new AlbedoChangeError(initial_albedo, albedo_constraint_weight)));
  }

  double m_initial_albedo, m_albedo_constraint_weight;
};

// TODO(oalexan1): This should be modularized
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
                std::vector<MaskedImgRefT>    const & masked_images,
                std::vector<DblImgT>          const & blend_weights,
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
                ceres::Problem                      & problem) {

  int num_images = opt.input_images.size();

  // See if a given image is used in at least one clip or skipped in
  // all of them
  std::vector<bool> use_image(num_images, false);
  int num_used = 0;
  for (int image_iter = 0; image_iter < num_images; image_iter++) {
    if (opt.skip_images.find(image_iter) == opt.skip_images.end()) {
      use_image[image_iter] = true;
      num_used++;
    }
  }

  // We define p and q as the partial derivatives in x in y of the dem.
  // When using the integrability constraint, they are floated as variables
  // in their own right, while constrained to not go too far from the DEM.
  if (opt.integrability_weight > 0) {
    pq.set_size(dem.cols(), dem.rows());
    for (int col = 1; col < dem.cols()-1; col++) {
      for (int row = 1; row < dem.rows()-1; row++) {
        pq(col, row)[0] = (dem(col+1, row) - dem(col-1, row))/(2*gridx); // right - left
        pq(col, row)[1] = (dem(col, row-1) - dem(col, row+1))/(2*gridy); // top - bottom
      }
    }
  }

  // Use a simpler cost function if only the DEM is floated. This results in
  // notably reduced memory usage but no gain in speed.
  bool float_dem_only = true;
  if (opt.float_albedo || opt.float_exposure || opt.fix_dem || opt.float_reflectance_model ||
      opt.float_haze || opt.integrability_weight > 0) {
    float_dem_only = false;
  }

  // Add a residual block for every grid point not at the boundary
  int bd = 1;
  for (int col = bd; col < dem.cols() - bd; col++) {
    for (int row = bd; row < dem.rows() - bd; row++) {

      // Intensity error for each image
      for (int image_iter = 0; image_iter < num_images; image_iter++) {

        if (opt.skip_images.find(image_iter) != opt.skip_images.end())
          continue;

        ceres::LossFunction* loss_function_img = NULL;
        if (opt.robust_threshold > 0)
          loss_function_img = new ceres::CauchyLoss(opt.robust_threshold);

        if (float_dem_only) {
          ceres::CostFunction* cost_function_img =
            IntensityErrorFloatDemOnly::Create(opt, col, row,
                                               dem,
                                               albedo(col, row),
                                               &refl_coeffs[0],
                                               &exposures[image_iter], &haze[image_iter][0],
                                               geo,
                                               opt.model_shadows,
                                               opt.camera_position_step_size,
                                               max_dem_height,
                                               gridx, gridy,
                                               refl_params, sunPosition[image_iter],
                                               crop_boxes[image_iter],
                                               masked_images[image_iter],
                                               blend_weights[image_iter],
                                               blend_weight_is_ground_weight,
                                               cameras[image_iter]);
          problem.AddResidualBlock(cost_function_img, loss_function_img,
                                    &dem(col-1, row),  // left
                                    &dem(col, row),    // center
                                    &dem(col+1, row),  // right
                                    &dem(col, row+1),  // bottom
                                    &dem(col, row-1)); // top
        } else if (opt.integrability_weight == 0) {
          ceres::CostFunction* cost_function_img =
            IntensityError::Create(opt, col, row, dem, geo,
                                   opt.model_shadows,
                                   opt.camera_position_step_size,
                                   max_dem_height,
                                   gridx, gridy,
                                   refl_params, sunPosition[image_iter],
                                   crop_boxes[image_iter],
                                   masked_images[image_iter],
                                   blend_weights[image_iter],
                                   blend_weight_is_ground_weight,
                                   cameras[image_iter]);
          problem.AddResidualBlock(cost_function_img, loss_function_img,
                                   &exposures[image_iter],       // exposure
                                   &haze[image_iter][0],         // haze
                                   &dem(col-1, row),  // left
                                   &dem(col, row),    // center
                                   &dem(col+1, row),  // right
                                   &dem(col, row+1),  // bottom
                                   &dem(col, row-1),  // top
                                   &albedo(col, row), // albedo
                                   &refl_coeffs[0]);
        } else {
          // Use the integrability constraint
          ceres::CostFunction* cost_function_img =
            IntensityErrorPQ::Create(opt, col, row, dem, geo,
                                      opt.model_shadows,
                                      opt.camera_position_step_size,
                                      max_dem_height,
                                      gridx, gridy,
                                      refl_params, sunPosition[image_iter],
                                      crop_boxes[image_iter],
                                      masked_images[image_iter],
                                      blend_weights[image_iter],
                                      blend_weight_is_ground_weight,
                                      cameras[image_iter]);
          problem.AddResidualBlock(cost_function_img, loss_function_img,
                                    &exposures[image_iter],          // exposure
                                    &haze[image_iter][0],            // haze
                                    &dem(col, row),       // center
                                    &pq(col, row)[0],      // pq
                                    &albedo(col, row),    // albedo
                                    &refl_coeffs[0]);   // reflectance
        }

      } // end iterating over images

      // Smoothness penalty. We always add this, even if the smoothness weight is 0,
      // to make Ceres not complain about blocks not being set.
      ceres::LossFunction* loss_function_sm = NULL;
      ceres::CostFunction* cost_function_sm =
        SmoothnessError::Create(smoothness_weight, gridx, gridy);
      problem.AddResidualBlock(cost_function_sm, loss_function_sm,
                                &dem(col-1, row+1),  // bottom left
                                &dem(col, row+1),    // bottom
                                &dem(col+1, row+1),  // bottom right
                                &dem(col-1, row),    // left
                                &dem(col, row),      // center
                                &dem(col+1, row),    // right
                                &dem(col-1, row-1),  // top left
                                &dem(col, row-1),    // top
                                &dem(col+1, row-1)); // top right

      // Add curvature in shadow. Note that we use a per-pixel curvature_in_shadow_weight,
      // to gradually phase it in to avoid artifacts.
      if (opt.curvature_in_shadow_weight > 0.0 &&
          curvature_in_shadow_weight(col, row) > 0) {
        ceres::LossFunction* loss_function_cv = NULL;
        ceres::CostFunction* cost_function_cv =
          CurvatureInShadowError::Create(opt.curvature_in_shadow,
                                          curvature_in_shadow_weight(col, row),
                                          gridx, gridy);
        problem.AddResidualBlock(cost_function_cv, loss_function_cv,
                                  &dem(col,   row+1),  // bottom
                                  &dem(col-1, row),    // left
                                  &dem(col,   row),    // center
                                  &dem(col+1, row),    // right
                                  &dem(col,   row-1)); // top
      }

      // Add gradient weight
      if (opt.gradient_weight > 0.0) {
        ceres::LossFunction* loss_function_grad = NULL;
        ceres::CostFunction* cost_function_grad =
          GradientError::Create(opt.gradient_weight, gridx, gridy);
        problem.AddResidualBlock(cost_function_grad, loss_function_grad,
                                  &dem(col,   row+1),  // bottom
                                  &dem(col-1, row),    // left
                                  &dem(col,   row),    // center
                                  &dem(col+1, row),    // right
                                  &dem(col,   row-1)); // top
      }

      if (opt.integrability_weight > 0) {
        ceres::LossFunction* loss_function_int = NULL;
        ceres::CostFunction* cost_function_int =
          IntegrabilityError::Create(opt.integrability_weight, gridx, gridy);
        problem.AddResidualBlock(cost_function_int, loss_function_int,
                                  &dem(col,   row+1),   // bottom
                                  &dem(col-1, row),     // left
                                  &dem(col+1, row),     // right
                                  &dem(col,   row-1),   // top
                                  &pq  (col,   row)[0]); // pq

        if (opt.smoothness_weight_pq > 0) {
          ceres::LossFunction* loss_function_sm_pq = NULL;
          ceres::CostFunction* cost_function_sm_pq =
            SmoothnessErrorPQ::Create(opt.smoothness_weight_pq, gridx, gridy);
          problem.AddResidualBlock(cost_function_sm_pq, loss_function_sm_pq,
                                    &pq(col, row+1)[0],  // bottom
                                    &pq(col-1, row)[0],  // left
                                    &pq(col+1, row)[0],  // right
                                    &pq(col, row-1)[0]); // top
        }
      }

      // Deviation from prescribed height constraint
      if (opt.initial_dem_constraint_weight > 0) {
        ceres::LossFunction* loss_function_hc = NULL;
        ceres::CostFunction* cost_function_hc =
          HeightChangeError::Create(orig_dem(col, row),
                                    opt.initial_dem_constraint_weight);
        problem.AddResidualBlock(cost_function_hc, loss_function_hc,
                                  &dem(col, row));
      }

      // Deviation from prescribed albedo
      if (opt.float_albedo > 0 && opt.albedo_constraint_weight > 0) {
        ceres::LossFunction* loss_function_hc = NULL;
        if (opt.albedo_robust_threshold > 0)
          loss_function_hc = new ceres::CauchyLoss(opt.albedo_robust_threshold);
        ceres::CostFunction* cost_function_hc =
          AlbedoChangeError::Create(albedo(col, row), opt.albedo_constraint_weight);
        problem.AddResidualBlock(cost_function_hc, loss_function_hc, &albedo(col, row));
      } // end if float albedo

    } // end row iter
  } // end col iter

  // DEM at the boundary must be fixed
  for (int col = 0; col < dem.cols(); col++) {
    for (int row = 0; row < dem.rows(); row++) {
      if (col == 0 || col == dem.cols() - 1 || row == 0 || row == dem.rows() - 1) {
          problem.SetParameterBlockConstant(&dem(col, row));
      }
    }
  }

  if (opt.fix_dem) {
    for (int col = 0; col < dem.cols(); col++) {
      for (int row = 0; row < dem.rows(); row++)
        problem.SetParameterBlockConstant(&dem(col, row));
    }
  }

  if (opt.initial_dem_constraint_weight <= 0 && num_used <= 1) {

    if (opt.float_albedo && opt.albedo_constraint_weight <= 0) {
      vw::vw_out() << "No DEM or albedo constraint is used, and there is at most one "
                   << "usable image. Fixing the albedo.\n";

      opt.float_albedo = false;
    }

    // If there's just one image, don't float the exposure, as the
    // problem is under-determined. If we float the albedo, we will
    // implicitly float the exposure, hence keep the exposure itself
    // fixed.
    if (opt.float_exposure) {
      vw::vw_out() << "No DEM constraint is used, and there is at most one "
                << "usable image. Fixing the exposure.\n";
      opt.float_exposure = false;
    }
  }

  // If to float the albedo
  if (!float_dem_only) {
    for (int col = 1; col < dem.cols() - 1; col++) {
      for (int row = 1; row < dem.rows() - 1; row++) {
        if (!opt.float_albedo && num_used > 0)
          problem.SetParameterBlockConstant(&albedo(col, row));
      }
    }
  }

  if (!float_dem_only) {

    // If floating the DEM only, none of the below parameters are even added to the problem,
    // it does not make sense to check to keep them fixed or floating them.

    if (!opt.float_exposure) {
      for (int image_iter = 0; image_iter < num_images; image_iter++) {
        if (use_image[image_iter])
          problem.SetParameterBlockConstant(&exposures[image_iter]);
      }
    }
    if (!opt.float_haze) {
      for (int image_iter = 0; image_iter < num_images; image_iter++) {
        if (use_image[image_iter])
          problem.SetParameterBlockConstant(&haze[image_iter][0]);
      }
    }

    // If to float the reflectance model coefficients
    if (!opt.float_reflectance_model && num_used > 0)
      problem.SetParameterBlockConstant(&refl_coeffs[0]);
  }

} // end function formCostFun()

// Find the best-fit exposure and haze given the input sampled image and reflectance.
// Also find the sampled albedo along the way. The albedo will be optimized
// only if --float-albedo is on. Otherwise it will be kept at the nominal value.
void estimExposureHazeAlbedo(SfsOptions & opt,
                             std::vector<MaskedImgRefT> const& masked_images,
                             std::vector<DblImgT> const& blend_weights,
                             bool blend_weight_is_ground_weight,
                             vw::ImageView<double> const& dem,
                             double mean_albedo,
                             vw::cartography::GeoReference const& geo,
                             std::vector<vw::CamPtr> const& cameras,
                             double & max_dem_height,
                             std::vector<vw::BBox2i> const& crop_boxes,
                             std::vector<vw::Vector3> const& sunPosition,
                             asp::ReflParams const& refl_params,
                             double gridx, double gridy) {

  vw::vw_out() << "Estimating the exposure, haze, albedo.\n";
  if (!opt.float_albedo)
    vw::vw_out() << "The albedo is fixed.\n";

  // Sample large DEMs
  int sample_col_rate = 0, sample_row_rate = 0;
  asp::calcSampleRates(dem, opt.num_samples_for_estim, sample_col_rate, sample_row_rate);

  int num_images = opt.input_images.size();
  std::vector<double> local_exposures_vec(num_images, 0), local_haze_vec(num_images, 0);

  std::vector<MaskedDblImgT> reflectance(num_images), intensity(num_images);
  int num_sampled_cols = 0, num_sampled_rows = 0;
  for (int image_iter = 0; image_iter < num_images; image_iter++) {

    if (opt.skip_images.find(image_iter) != opt.skip_images.end())
      continue;

    vw::ImageView<double> ground_weight;
    vw::ImageView<vw::Vector2> pq; // no need for these just for initialization
    bool show_progress = false;
    computeReflectanceAndIntensity(dem, pq, geo,
                                   opt.model_shadows, show_progress, max_dem_height,
                                   gridx, gridy, sample_col_rate, sample_row_rate,
                                   sunPosition[image_iter],
                                   refl_params,
                                   crop_boxes[image_iter],
                                   masked_images[image_iter],
                                   blend_weights[image_iter],
                                   blend_weight_is_ground_weight,
                                   cameras[image_iter],
                                   reflectance[image_iter], intensity[image_iter],
                                   ground_weight,
                                   &opt.model_coeffs_vec[0], opt);

    num_sampled_cols = reflectance[image_iter].cols();
    num_sampled_rows = reflectance[image_iter].rows();
  }

  // Create the inital albedo image, of same size as sampled intensity
  vw::ImageView<double> albedo(num_sampled_cols, num_sampled_rows);
  for (int col = 0; col < albedo.cols(); col++) {
    for (int row = 0; row < albedo.rows(); row++) {
      albedo(col, row) = mean_albedo;
    }
  }
  
  // Create the image of valid pixels, so where albedo was solved for
  vw::ImageView<int> valid_mask(num_sampled_cols, num_sampled_rows);
  for (int col = 0; col < valid_mask.cols(); col++) {
    for (int row = 0; row < valid_mask.rows(); row++) {
      valid_mask(col, row) = 0;
    }
  }

  // Create the problem
  ceres::Problem problem;

  // Add a residual block for every pixel in each image
  for (int image_iter = 0; image_iter < num_images; image_iter++) {
    for (int col = 0; col < intensity[image_iter].cols(); col++) {
      for (int row = 0; row < intensity[image_iter].rows(); row++) {
        
        // Skip invalid pixels, such as at boundary. The albedo will be kept
        // to its initial value at these pixels.
        if (!vw::is_valid(intensity[image_iter](col, row)) ||
            !vw::is_valid(reflectance[image_iter](col, row)))
          continue;
        
        // Mark this albedo pixel as valid
        valid_mask(col, row) = 1;

        ceres::CostFunction* cost_function_img =
          IntensityErrorFixedReflectance::Create(intensity[image_iter](col, row),
                                                  reflectance[image_iter](col, row),
                                                  opt.num_haze_coeffs,
                                                  opt.steepness_factor);

        ceres::LossFunction* loss_function_img = NULL;
        if (opt.robust_threshold > 0)
          loss_function_img = new ceres::CauchyLoss(opt.robust_threshold);

        problem.AddResidualBlock(cost_function_img,
                                loss_function_img,
                                &opt.image_exposures_vec[image_iter],
                                &opt.image_haze_vec[image_iter][0],
                                &albedo(col, row));

        // If zero haze coefficients are used, fix the haze
        if (opt.num_haze_coeffs == 0)
          problem.SetParameterBlockConstant(&opt.image_haze_vec[image_iter][0]);

        if (!opt.float_albedo)
          problem.SetParameterBlockConstant(&albedo(col, row));
          
      } // end row
    } // end col
  } // end image_iter

  // Deviation from prescribed albedo at each albedo pixel
  if (opt.float_albedo > 0 && opt.albedo_constraint_weight > 0) {
    for (int col = 0; col < albedo.cols(); col++) {
      for (int row = 0; row < albedo.rows(); row++) {
        ceres::LossFunction* loss_function_hc = NULL;
        if (opt.albedo_robust_threshold > 0)
          loss_function_hc = new ceres::CauchyLoss(opt.albedo_robust_threshold);
        ceres::CostFunction* cost_function_hc =
          AlbedoChangeError::Create(albedo(col, row), opt.albedo_constraint_weight);
        problem.AddResidualBlock(cost_function_hc, loss_function_hc, &albedo(col, row));
      } // end row
    } // end col
  }

  // Ceres options
  ceres::Solver::Options options;
  options.gradient_tolerance = 1e-16;
  options.function_tolerance = 1e-16;
  options.max_num_iterations = opt.max_iterations;
  options.minimizer_progress_to_stdout = 1;
  options.num_threads = opt.num_threads;
  options.linear_solver_type = ceres::SPARSE_SCHUR;

  // Solve the problem
  ceres::Solver::Summary summary;
  ceres::Solve(options, &problem, &summary);
  vw::vw_out() << summary.FullReport() << "\n";

  if (opt.float_albedo) {
    
    // In-fill the albedo image to reduce boundary artifacts
    inFillImage3x3(albedo, valid_mask);
    
    // Up-sample the estimated albedo to full-res dimensions with bilinear
    // interpolation. This is not needed and not used if albedo is not floated.
    bool has_georef = true;
    bool has_nodata = true;
    double albedo_nodata_val = -1e+6; // large but reasonable
    vw::TerminalProgressCallback tpc("asp", ": ");
    std::string albedo_file = opt.out_prefix + "-albedo-estim.tif";
    vw::vw_out() << "Up-sampling the estimated albedo to input DEM dimensions.\n";
    vw::vw_out() << "Writing: " << albedo_file << "\n";
    // Note: This logic produces junk if the SfsInterpView is initialized on a
    // separate line and then passed to block_write_gdal_image(). Not clear why.
    vw::cartography::block_write_gdal_image(albedo_file, 
                           SfsInterpView(dem.cols(), dem.rows(), sample_col_rate,
                                         sample_row_rate, albedo),
                           has_georef, geo, has_nodata, albedo_nodata_val, opt, tpc);
  }

  // The haze and exposures will be saved outside this function.
  return;
}

// Compute the DEM variance for the given problem, and also the albedo variance,
// if --float-albedo is on.
bool calcSfsVariances(SfsOptions const& opt,
                       vw::ImageView<double> const& dem,
                       vw::ImageView<double> const& albedo,
                       ceres::Problem &problem,
                       ceres::Covariance &covariance) { // output
  
  vw::vw_out() << "Computing variances.\n";

  std::vector<const double*> parameter_blocks;
  for (int col = 0; col < dem.cols(); col++) {
    for (int row = 0; row < dem.rows(); row++) {
      if (problem.HasParameterBlock(&dem(col, row)) && 
          !problem.IsParameterBlockConstant(&dem(col, row))) {
        parameter_blocks.push_back(&dem(col, row));
      }
    }
  }
  if (opt.float_albedo) {
    for (int col = 0; col < albedo.cols(); col++) {
      for (int row = 0; row < albedo.rows(); row++) {
        if (problem.HasParameterBlock(&albedo(col, row)) && 
            !problem.IsParameterBlockConstant(&albedo(col, row))) {
          parameter_blocks.push_back(&albedo(col, row));
        }
      }
    }
  }
  
  if (!covariance.Compute(parameter_blocks, &problem)) {
    vw::vw_out(vw::WarningMessage) 
      << "The CERES solver failed to compute the variances. If --float-albedo is on, "
      << "consider disabling one or both of --float-haze and --float-exposures, "
      << "and perhaps increasing --albedo-constraint-weight, to make the problem better " 
      << "determined.\n";
    return false;
  }
  
  return true;
}

// A function to save the variance of a given parameter set
void saveSfsVariance(SfsOptions const& opt,
                     vw::ImageView<double> const& values,
                     std::string const& variance_file,
                     vw::cartography::GeoReference const& geo,
                     double nodata_val,
                     ceres::Problem &problem,
                     ceres::Covariance &covariance) {

  vw::ImageView<double> variance_image(values.cols(), values.rows());
  vw::fill(variance_image, nodata_val);
  for (int col = 0; col < values.cols(); col++) {
    for (int row = 0; row < values.rows(); row++) {
      if (problem.HasParameterBlock(&values(col, row)) && 
          !problem.IsParameterBlockConstant(&values(col, row))) {
        double var = 0;
        if (covariance.GetCovarianceBlock(&values(col, row), &values(col, row), &var))
          variance_image(col, row) = var;
      }
    }
  }

  vw::vw_out() << "Writing: " << variance_file << "\n";
  bool has_georef = true, has_nodata = true;
  vw::TerminalProgressCallback tpc("asp", ": ");
  vw::cartography::block_write_gdal_image(variance_file, variance_image,
                                          has_georef, geo,
                                          has_nodata, nodata_val,
                                          opt, tpc);
}

// Compute and save the variances
void calcSaveSfsVariances(SfsOptions const& opt,
                          vw::ImageView<double> const& dem,
                          vw::ImageView<double> const& albedo,
                          ceres::Problem &problem,
                          vw::cartography::GeoReference const& geo,
                          double dem_nodata_val) {

  ceres::Covariance::Options covariance_options;
  covariance_options.num_threads = opt.num_threads;
  ceres::Covariance covariance(covariance_options);
  if (calcSfsVariances(opt, dem, albedo, problem, covariance)) {
    // Save DEM variance
    std::string dem_variance_file = opt.out_prefix + "-DEM-variance.tif";
    saveSfsVariance(opt, dem, dem_variance_file, geo, dem_nodata_val,
                    problem, covariance);
    
    // Save albedo variance
    if (opt.float_albedo) {
      std::string albedo_variance_file = opt.out_prefix + "-albedo-variance.tif";
      saveSfsVariance(opt, albedo, albedo_variance_file, geo, dem_nodata_val,
                      problem, covariance);
    }
  }
}

} // end namespace asp
