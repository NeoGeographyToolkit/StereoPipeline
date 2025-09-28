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
            std::vector<asp::MaskedImgT> const& masked_images,
            std::vector<asp::DoubleImgT> const& blend_weights,
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
  
  using namespace vw;
  
  iter++;

  vw_out() << "Finished iteration: " << iter << std::endl;

  if (!opt.save_computed_intensity_only)
    asp::saveExposures(opt.out_prefix, opt.input_images, exposures);

  if (opt.num_haze_coeffs > 0 && !opt.save_computed_intensity_only)
    asp::saveHaze(opt.out_prefix, opt.input_images, haze);

  std::string model_coeffs_file = asp::modelCoeffsFileName(opt.out_prefix);
  if (!opt.save_computed_intensity_only) {
    vw_out() << "Writing: " << model_coeffs_file << std::endl;
    std::ofstream mcf(model_coeffs_file.c_str());
    mcf.precision(17);
    for (size_t coeff_iter = 0; coeff_iter < refl_coeffs.size(); coeff_iter++)
      mcf << refl_coeffs[coeff_iter] << " ";
    mcf << "\n";
    mcf.close();
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
    fill(dem_nodata, dem_nodata_val);
  }

  bool has_georef = true, has_nodata = true;
  TerminalProgressCallback tpc("asp", ": ");
  if ( (!opt.save_sparingly || final_iter) && !opt.save_computed_intensity_only ) {
    std::string out_dem_file = opt.out_prefix + "-DEM"
      + iter_str + ".tif";
    vw_out() << "Writing: " << out_dem_file << std::endl;
    block_write_gdal_image(out_dem_file, dem, has_georef, geo,
                          has_nodata, dem_nodata_val,
                          opt, tpc);
  }

  if ((!opt.save_sparingly || (final_iter && opt.float_albedo)) &&
      !opt.save_computed_intensity_only) {
    std::string out_albedo_file = opt.out_prefix + "-comp-albedo"
      + iter_str + ".tif";
    vw_out() << "Writing: " << out_albedo_file << std::endl;
    block_write_gdal_image(out_albedo_file, albedo, has_georef,
                          geo,
                          has_nodata, dem_nodata_val,
                          opt, tpc);
  }

  // Print reflectance and other things
  for (size_t image_iter = 0; image_iter < masked_images.size(); image_iter++) {

    if (opt.skip_images.find(image_iter) != opt.skip_images.end())
      continue;

    vw::ImageView<PixelMask<double>> reflectance, intensity, comp_intensity;
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
    computeReflectanceAndIntensity(dem, pq, geo,
                                  opt.model_shadows,
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
          if (is_valid(reflectance(col, row)))
            dem_nodata(col, row) = dem(col, row);
        }
      }
    }

    if (opt.save_sparingly)
      continue;

    // Find the computed intensity
    comp_intensity.set_size(reflectance.cols(), reflectance.rows());
    for (int col = 0; col < comp_intensity.cols(); col++) {
      for (int row = 0; row < comp_intensity.rows(); row++) {
        comp_intensity(col, row) = calcIntensity(albedo(col, row),
                                                 reflectance(col, row),
                                                 exposures[image_iter],
                                                 opt.steepness_factor,
                                                 &haze[image_iter][0],
                                                 opt.num_haze_coeffs);
      }
    }

    std::string out_meas_intensity_file = iter_str2 + "-meas-intensity.tif";
    vw_out() << "Writing: " << out_meas_intensity_file << std::endl;
    block_write_gdal_image(out_meas_intensity_file, apply_mask(intensity, img_nodata_val),
                          has_georef, geo, has_nodata, img_nodata_val, opt, tpc);

    std::string out_comp_intensity_file = iter_str2 + "-comp-intensity.tif";
    vw_out() << "Writing: " << out_comp_intensity_file << std::endl;
    block_write_gdal_image(out_comp_intensity_file,
                          apply_mask(comp_intensity, img_nodata_val),
                          has_georef, geo, has_nodata, img_nodata_val,
                          opt, tpc);

    if (opt.save_computed_intensity_only)
      continue; // don't write too many things

    std::string out_weight_file = iter_str2 + "-blending-weight.tif";
    vw_out() << "Writing: " << out_weight_file << std::endl;
    block_write_gdal_image(out_weight_file, ground_weight,
                          has_georef, geo, has_nodata, img_nodata_val, 
                          opt, tpc);

    std::string out_reflectance_file = iter_str2 + "-reflectance.tif";
    vw_out() << "Writing: " << out_reflectance_file << std::endl;
    block_write_gdal_image(out_reflectance_file,
                           apply_mask(reflectance, img_nodata_val),
                           has_georef, geo, has_nodata, img_nodata_val,
                           opt, tpc);

    // Find the measured normalized albedo, after correcting for
    // reflectance.
    vw::ImageView<double> measured_albedo;
    measured_albedo.set_size(reflectance.cols(), reflectance.rows());
    for (int col = 0; col < measured_albedo.cols(); col++) {
      for (int row = 0; row < measured_albedo.rows(); row++) {
        if (!is_valid(reflectance(col, row)))
          measured_albedo(col, row) = 1;
        else
          measured_albedo(col, row)
            = calcAlbedo(intensity(col, row), reflectance(col, row),
                         exposures[image_iter], opt.steepness_factor,
                         &haze[image_iter][0], opt.num_haze_coeffs);
      }
    }
    std::string out_albedo_file = iter_str2 + "-meas-albedo.tif";
    vw_out() << "Writing: " << out_albedo_file << std::endl;
    block_write_gdal_image(out_albedo_file, measured_albedo,
                           has_georef, geo, has_nodata, 0, opt, tpc);

    vw_out() << "Exposure for image " << image_iter << ": "
            << exposures[image_iter] << std::endl;

    if (opt.num_haze_coeffs > 0) {
      vw_out() << "Haze for image " << image_iter << ":";
      for (size_t hiter = 0; hiter < haze[image_iter].size(); hiter++) {
        vw_out() << " " << haze[image_iter][hiter];
      }
      vw_out() << std::endl;
    }
  } // end loop through images

  if (opt.save_dem_with_nodata && (!opt.save_sparingly || final_iter)) {
    std::string out_dem_nodata_file = opt.out_prefix + "-DEM-nodata"
      + iter_str + ".tif";
    vw_out() << "Writing: " << out_dem_nodata_file << std::endl;
    TerminalProgressCallback tpc("asp", ": ");
    block_write_gdal_image(out_dem_nodata_file, dem_nodata, has_georef, geo,
                           has_nodata, dem_nodata_val, opt, tpc);
  }

  return ceres::SOLVER_CONTINUE;
} // end callback function

void SfsCallback::set_final_iter(bool is_final_iter) {
  final_iter = is_final_iter;
}

} // end namespace asp
