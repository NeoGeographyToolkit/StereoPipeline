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

// Deal with outliers in image intensity.
// TODO: Ensure the output DEM is float. Check its no-data value.
// TODO: Must specify in the SfS doc that the Lunar-Lambertian model fails at poles
// TODO: Study the effect of using bicubic interpolation.
// TODO: Find a good automatic value for the smoothness weight.
// TODO: Check that we are within image boundaries when interpolating.
// TODO: Study the normal computation formula.

/// \file sfs.cc

// Turn off warnings from boost and other packages
#if defined(__GNUC__) || defined(__GNUG__)
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-local-typedefs"
#endif

#include <asp/SfS/SfsImageProc.h>
#include <asp/SfS/SfsUtils.h>
#include <asp/SfS/SfsOptions.h>
#include <asp/SfS/SfsArgs.h>
#include <asp/SfS/SfsCamera.h>
#include <asp/SfS/SfsModel.h>
#include <asp/SfS/SfsErrorEstim.h>
#include <asp/SfS/SfsCostFun.h>
#include <asp/Sessions/StereoSessionFactory.h>
#include <asp/Camera/CsmModel.h>
#include <asp/Core/Macros.h>
#include <asp/Core/DemUtils.h>
#include <asp/Core/BaseCameraUtils.h>
#include <asp/Core/StereoSettings.h>
#include <asp/Core/FileUtils.h>

#include <asp/asp_config.h> // defines ASP_HAVE_PKG_ISIS
#if defined(ASP_HAVE_PKG_ISIS) && ASP_HAVE_PKG_ISIS == 1
#include <asp/IsisIO/IsisCameraModel.h>
#endif // ASP_HAVE_PKG_ISIS

#include <vw/Image/MaskViews.h>
#include <vw/Image/AntiAliasing.h>
#include <vw/Cartography/GeoReferenceUtils.h>
#include <vw/FileIO/FileUtils.h>
#include <vw/FileIO/DiskImageView.h>
#include <vw/Core/Stopwatch.h>
#include <vw/Cartography/GeoTransform.h>

#include <ceres/ceres.h>
#include <ceres/loss_function.h>

#include <iostream>
#include <string>

#if defined(__GNUC__) || defined(__GNUG__)
#pragma GCC diagnostic pop
#undef LOCAL_GCC_VERSION
#endif

namespace fs = boost::filesystem;

using namespace vw::camera;
using namespace vw::cartography;
using namespace asp;

// Run sfs
void runSfs(// Fixed quantities
            int                                num_iterations,
            double                             gridx,
            double                             gridy,
            SfsOptions                       & opt,
            GeoReference               const & geo,
            double                             smoothness_weight,
            double                             max_dem_height,
            double                             dem_nodata_val,
            float                              img_nodata_val,
            std::vector<vw::BBox2i>    const & crop_boxes,
            std::vector<MaskedImgRefT> const & masked_images,
            std::vector<DblImgT>       const & blend_weights,
            bool                               blend_weight_is_ground_weight,
            asp::ReflParams            const & refl_params,
            std::vector<vw::Vector3>   const & sunPosition,
            vw::ImageView<double>      const & orig_dem,
            vw::ImageView<int>         const & lit_image_mask,
            vw::ImageView<double>      const & curvature_in_shadow_weight,
            // Variable quantities
            vw::ImageView<double>            & dem,
            vw::ImageView<double>            & albedo,
            std::vector<vw::CamPtr>          & cameras,
            std::vector<double>              & exposures,
            std::vector<std::vector<double>> & haze,
            std::vector<double>              & refl_coeffs) {

  // Set up the cost function 
  int num_images = opt.input_images.size();
  ceres::Problem problem;
  vw::ImageView<vw::Vector2> pq;
  asp::sfsCostFun(gridx, gridy, smoothness_weight, max_dem_height, dem_nodata_val,
                  img_nodata_val, blend_weight_is_ground_weight, geo,
                  crop_boxes, masked_images, blend_weights, refl_params, sunPosition,
                  orig_dem, lit_image_mask, curvature_in_shadow_weight, opt,
                  // Outputs
                  dem, albedo, cameras, exposures, haze, refl_coeffs, pq, problem);

  // Solver options and the callback
  ceres::Solver::Options options;
  options.gradient_tolerance = 1e-16;
  options.function_tolerance = 1e-16;
  options.max_num_iterations = num_iterations;
  options.minimizer_progress_to_stdout = 1;
  options.num_threads = opt.num_threads;
  options.linear_solver_type = ceres::SPARSE_SCHUR;
  SfsCallback callback(opt, dem, pq, albedo, geo, refl_params, sunPosition,
                       crop_boxes, masked_images, blend_weights,
                       blend_weight_is_ground_weight, cameras,
                       dem_nodata_val, img_nodata_val, exposures, haze,
                       max_dem_height, gridx, gridy, refl_coeffs);
  options.callbacks.push_back(&callback);
  options.update_state_every_iteration = true;

  // Solve the problem if asked to do iterations. Otherwise
  // just keep the DEM at the initial guess, while saving
  // all the output data as if iterations happened.
  ceres::Solver::Summary summary;
  if (options.max_num_iterations > 0)
    ceres::Solve(options, &problem, &summary);

  // Save the final results
  callback.set_final_iter(true);
  ceres::IterationSummary callback_summary;
  callback(callback_summary);

  vw::vw_out() << summary.FullReport() << "\n" << std::endl;
}

// Load cameras and sun positions
void loadCamerasSunPos(SfsOptions &opt,
                       vw::ImageView<double> const& dem,
                       double dem_nodata_val,
                       vw::cartography::GeoReference const& geo,
                       std::vector<vw::CamPtr> &cameras,
                       std::vector<vw::Vector3> &sunPosition) {

  // Initialize outputs
  int num_images = opt.input_images.size();
  cameras.resize(num_images);  
   sunPosition.resize(num_images, vw::Vector3());

  // Read from list or from angles
  if (opt.sun_positions_list != "")
    asp::readSunPositions(opt.sun_positions_list, opt.input_images,
                          dem, dem_nodata_val, geo, sunPosition);
  if (opt.sun_angles_list != "")
    asp::readSunAngles(opt.sun_angles_list, opt.input_images,
                        dem, dem_nodata_val, geo, sunPosition);

  // Read cameras and compute sun positions if not read from file
  for (int image_iter = 0; image_iter < num_images; image_iter++) {

    if (opt.skip_images.find(image_iter) != opt.skip_images.end())
      continue;

    asp::SessionPtr 
      session(asp::StereoSessionFactory::create(opt.stereo_session, // in-out
                                                opt,
                                                opt.input_images[image_iter],
                                                opt.input_images[image_iter],
                                                opt.input_cameras[image_iter],
                                                opt.input_cameras[image_iter],
                                                opt.out_prefix));
    cameras[image_iter] = session->camera_model(opt.input_images[image_iter],
                                                opt.input_cameras[image_iter]);

    // Read the sun position from the camera if it is was not read from the list
    if (sunPosition[image_iter] == vw::Vector3())
      sunPosition[image_iter] = asp::sunPositionFromCamera(cameras[image_iter]);

    // Sanity check
    if (sunPosition[image_iter] == vw::Vector3())
      vw::vw_throw(vw::ArgumentErr()
                << "Could not read sun positions from list or from camera model files.\n");

    // Compute the azimuth and elevation
    double azimuth = 0.0, elevation = 0.0;
    asp::sunAngles(dem, dem_nodata_val, geo, sunPosition[image_iter],
                    azimuth, elevation);

    // Print this. It will be used to organize the images by illumination
    // for bundle adjustment.
    // Since the sun position has very big values and we want to sort uniquely
    // the images by azimuth angle, use high precision below.
    vw::vw_out().precision(17);
    vw::vw_out() << "Sun position for: " << opt.input_images[image_iter] << " is "
                 << sunPosition[image_iter] << "\n";
    vw::vw_out() << "Sun azimuth and elevation for: "
                 << opt.input_images[image_iter] << " are " << azimuth
                 << " and " << elevation << " degrees.\n";
    vw::vw_out().precision(6); // Go back to usual precision
  }
    
} // end function loadCamerasSunPos

void calcApproxCamsCropBoxes(vw::ImageView<double> const& dem,
                             vw::cartography::GeoReference const& geo,
                             double dem_nodata_val,
                             // Outputs
                             SfsOptions &opt,
                             std::vector<vw::CamPtr> &cameras, 
                             std::vector<vw::BBox2i> &crop_boxes,
                             vw::Mutex &camera_mutex) {

  double max_approx_err = 0.0;
  int num_images = opt.input_images.size();
  
  for (int image_iter = 0; image_iter < num_images; image_iter++) {

    if (opt.skip_images.find(image_iter) != opt.skip_images.end()) continue;

    // Here we make a copy, since soon cameras[image_iter] will be overwritten
    vw::CamPtr exact_camera = cameras[image_iter];

    vw::vw_out() << "Creating an approximate camera model for "
                 << opt.input_images[image_iter] << "\n";
    vw::BBox2i img_bbox = crop_boxes[image_iter];
    vw::Stopwatch sw;
    sw.start();
    boost::shared_ptr<CameraModel> apcam;
    apcam.reset(new asp::ApproxCameraModel(exact_camera, img_bbox, dem, geo,
                                           dem_nodata_val, camera_mutex));
    cameras[image_iter] = apcam;

    sw.stop();
    vw::vw_out() << "Approximate model generation time: " << sw.elapsed_seconds() << " s.\n";

    // Cast the pointer back to ApproxCameraModel as we need that.
    asp::ApproxCameraModel* cam_ptr
      = dynamic_cast<asp::ApproxCameraModel*>(apcam.get());
    if (cam_ptr == NULL)
      vw::vw_throw(vw::ArgumentErr() << "Expecting an ApproxCameraModel.");

    bool model_is_valid = cam_ptr->model_is_valid();

    // Compared original and approximate models
    double max_curr_err = 0.0;

    if (model_is_valid) {
      for (int col = 0; col < dem.cols(); col++) {
        for (int row = 0; row < dem.rows(); row++) {
          vw::Vector2 ll = geo.pixel_to_lonlat(vw::Vector2(col, row));
          vw::Vector3 xyz = geo.datum().geodetic_to_cartesian
            (vw::Vector3(ll[0], ll[1], dem(col, row)));

          // Test how the exact and approximate models compare
          vw::Vector2 pix3 = exact_camera->point_to_pixel(xyz);
          vw::Vector2 pix4 = cameras[image_iter]->point_to_pixel(xyz);
          max_curr_err = std::max(max_curr_err, norm_2(pix3 - pix4));

          cam_ptr->crop_box().grow(pix3);
          cam_ptr->crop_box().grow(pix4);
        }
      }

      cam_ptr->crop_box().crop(img_bbox);
    } else {
      vw::vw_out() << "Invalid camera model.\n";
    }

    if (max_curr_err > 2.0 || !model_is_valid) {
      // This is a bugfix. When the DEM clip does not intersect the image,
      // the approx camera model has incorrect values.
      if (model_is_valid)
        vw::vw_out() << "Error of camera approximation is too big.\n";
      vw::vw_out() << "Skip image " << image_iter << "\n";
      opt.skip_images.insert(image_iter);
      cam_ptr->crop_box() = vw::BBox2();
      max_curr_err = 0.0;
    }

    max_approx_err = std::max(max_approx_err, max_curr_err);

    cam_ptr->crop_box().crop(img_bbox);
    vw::vw_out() << "Crop box dimensions: " << cam_ptr->crop_box() << std::endl;

    // Copy the crop box
    if (opt.crop_input_images)
      crop_boxes[image_iter].crop(cam_ptr->crop_box());

    // Skip images which result in empty crop boxes
    if (crop_boxes[image_iter].empty()) {
      opt.skip_images.insert(image_iter);
    }

  } // end iterating over images
  vw::vw_out() << "Max error of approximating cameras: " << max_approx_err << " pixels.\n";
  
} // end function calcApproxCamsCropBoxes

void calcCropBoxes(vw::ImageView<double> const& dem,
                   vw::cartography::GeoReference const& geo,
                   std::vector<vw::CamPtr> const& cameras,
                   // Outputs
                   SfsOptions &opt,
                   std::vector<vw::BBox2i> &crop_boxes) {

  int num_images = opt.input_images.size();
  for (int image_iter = 0; image_iter < num_images; image_iter++) {
    if (opt.skip_images.find(image_iter) != opt.skip_images.end()) continue;

    // Store the full image box, and initialize the crop box to an empty box
    vw::BBox2i img_bbox = crop_boxes[image_iter];
    crop_boxes[image_iter] = vw::BBox2i();

    for (int col = 0; col < dem.cols(); col++) {
      for (int row = 0; row < dem.rows(); row++) {
        vw::Vector2 ll = geo.pixel_to_lonlat(vw::Vector2(col, row));
        vw::Vector3 xyz = geo.datum().geodetic_to_cartesian
          (vw::Vector3(ll[0], ll[1], dem(col, row)));

        vw::Vector2 pix = cameras[image_iter]->point_to_pixel(xyz);
        crop_boxes[image_iter].grow(pix);
      }
    }

    // Double the box dimensions, just in case. Later the SfS heights
    // may change, and we may need to see beyond the given box.
    // TODO(oalexan1): This is likely excessive.
    double extraFactor = 0.5;
    double extrax = extraFactor * crop_boxes[image_iter].width();
    double extray = extraFactor * crop_boxes[image_iter].height();
    crop_boxes[image_iter].min() -= vw::Vector2(extrax, extray);
    crop_boxes[image_iter].max() += vw::Vector2(extrax, extray);

    // Crop to the bounding box of the image
    crop_boxes[image_iter].crop(img_bbox);

    //vw_out() << "Estimated crop box for image " << opt.input_images[image_iter] << "\n";

    if (crop_boxes[image_iter].empty())
      opt.skip_images.insert(image_iter);
  }

} // end function calcCropBoxes

// Load masked images and compute blending weights. These weights will be adjusted
// later.
void loadMaskedImagesCalcWeights(SfsOptions const& opt,
                                 std::vector<vw::BBox2i> const& crop_boxes,
                                 // Outputs
                                 std::vector<MaskedImgRefT> &masked_images,
                                 std::vector<vw::ImageView<double>> &blend_weights,
                                 float &img_nodata_val) {
    
  // Initialize outputs
  int num_images = opt.input_images.size();
  img_nodata_val = -std::numeric_limits<float>::max();
  masked_images.resize(num_images);
  blend_weights.resize(num_images);
  
  for (int image_iter = 0; image_iter < num_images; image_iter++) {

    if (opt.skip_images.find(image_iter) != opt.skip_images.end())
      continue;

    std::string img_file = opt.input_images[image_iter];
    vw::read_nodata_val(img_file, img_nodata_val);

    // Model the shadow threshold
    float shadow_thresh = opt.shadow_threshold_vec[image_iter];
    if (opt.crop_input_images) {
      // Make a copy in memory for faster access
      if (!crop_boxes[image_iter].empty()) {
        vw::ImageView<float> cropped_img =
          vw::crop(vw::DiskImageView<float>(img_file), crop_boxes[image_iter]);
        masked_images[image_iter]
          = vw::create_pixel_range_mask2(cropped_img,
                                      std::max(img_nodata_val, shadow_thresh),
                                      opt.max_valid_image_vals_vec[image_iter]);

        // Compute blending weights only when cropping the images. Otherwise
        // the weights are too huge.
        if (opt.blending_dist > 0)
          blend_weights[image_iter]
            = asp::blendingWeights(masked_images[image_iter],
                                   opt.blending_dist, opt.blending_power,
                                   opt.min_blend_size);
      }
    } else {
      masked_images[image_iter]
        = vw::create_pixel_range_mask2(vw::DiskImageView<float>(img_file),
                                       std::max(img_nodata_val, shadow_thresh),
                                       opt.max_valid_image_vals_vec[image_iter]);
    }
  }

} // end function loadMaskedImagesCalcWeights

// Heuristics for setting up the no-data value
double setupDemNodata(SfsOptions const& opt) {
  double dem_nodata_val = -1e+6;
  
  if (vw::read_nodata_val(opt.input_dem, dem_nodata_val)) {
    vw::vw_out() << "Found DEM nodata value: " << dem_nodata_val << std::endl;
    if (std::isnan(dem_nodata_val)) {
      dem_nodata_val = -1e+6; // bugfix for NaN
      vw::vw_out() << "Overwriting the nodata value with: " << dem_nodata_val << "\n";
    }
  }
  if (!boost::math::isnan(opt.nodata_val)) {
    dem_nodata_val = opt.nodata_val;
    vw::vw_out() << "Over-riding the DEM nodata value with: " << dem_nodata_val << "\n";
  }

  return dem_nodata_val;
}

// Read the georeference for the DEM and albedo
void loadGeoref(SfsOptions const& opt,
                vw::cartography::GeoReference &geo,
                vw::cartography::GeoReference &albedo_geo) {

  if (!vw::cartography::read_georeference(geo, opt.input_dem))
      vw::vw_throw(vw::ArgumentErr() << "The input DEM has no georeference.\n");
      
  if (!opt.input_albedo.empty()) {
    if (!vw::cartography::read_georeference(albedo_geo, opt.input_albedo))
      vw::vw_throw(vw::ArgumentErr() << "The input albedo has no georeference.\n");
  } else {
    // Ensure initialization
    albedo_geo = geo;
  }

  // This is a bug fix. The georef pixel size in y must be negative
  // for the DEM to be oriented correctly.
  if (geo.transform()(1, 1) > 0)
    vw::vw_throw(vw::ArgumentErr() << "The input DEM has a positive pixel size in y. "
              << "This is unexpected. Normally it is negative since the (0, 0) "
              << "pixel is in the upper-left. Check your DEM pixel size with "
              << "gdalinfo. Cannot continue.\n");
    
  // The albedo and georef must have same wkt
  if (geo.get_wkt() != albedo_geo.get_wkt())
    vw::vw_throw(vw::ArgumentErr()
                  << "The input DEM has a different georeference "
                  << "from the input albedo image.\n");
}



// If --ref-map is passed in, crop to its extent. This will create opt.crop_win.
void setupRefMap(vw::ImageViewRef<double> const& full_dem,
                 vw::cartography::GeoReference const& geo,
                 SfsOptions & opt) {

  if (opt.ref_map.empty())
    return;

  // Read the georeference from the reference map
  vw::cartography::GeoReference ref_map_georef;
  bool has_ref_map_georef = vw::cartography::read_georeference(ref_map_georef, opt.ref_map);
  if (!has_ref_map_georef)
    vw::vw_throw(vw::ArgumentErr() << "The image in --ref-map has no georeference.\n");
      
  // Find the bounding box of the reference map 
  vw::Vector2 ref_size = vw::file_image_size(opt.ref_map);
  vw::BBox2i ref_map_box(0, 0, ref_size.x(), ref_size.y());
    
  // Convert this to dem pixel coordinates via GeoTransform. If the georefs are
  // same, do not use forward_bbox, as that expands the box by a pixel
  std::string dem_wkt = geo.get_wkt();
  std::string ref_wkt = ref_map_georef.get_wkt();
  vw::cartography::GeoTransform ref2dem(ref_map_georef, geo);
  if (dem_wkt == ref_wkt) {
    vw::Vector2 beg = ref2dem.forward(vw::Vector2(0, 0));
    vw::Vector2 end = ref2dem.forward(vw::Vector2(ref_size.x(), ref_size.y()));
    ref_map_box.min() = round(beg);
    ref_map_box.max() = round(end);
  } else {
    ref_map_box = ref2dem.forward_bbox(ref_map_box);
  }

  // Crop to dem pixel coordinates
  ref_map_box.crop(vw::bounding_box(full_dem));
    
  // Assign to opt.crop_win. It is checked by now that opt.crop_win would be
  // empty otherwise.
  opt.crop_win = ref_map_box;

  // Must be non-empty
  if (opt.crop_win.empty())
    vw::vw_throw(vw::ArgumentErr() << "The --ref-map does not overlap with the input DEM.\n");
}

void prepareDemAndAlbedo(SfsOptions& opt,
                         vw::ImageView<double>& dem,
                         vw::ImageView<double>& albedo,
                         vw::cartography::GeoReference& geo,
                         double& dem_nodata_val) {

  // Manage no-data. Use here a value that is not overly large in magnitude,
  // and easy to represent as float.
  dem_nodata_val = setupDemNodata(opt);

  // Read the handle to the DEM. Here we don't load the DEM into memory yet. We
  // will later load into memory only a crop, if cropping is specified. This is
  // to save on memory.
  vw::ImageViewRef<double> full_dem = vw::DiskImageView<double>(opt.input_dem);
  vw::ImageViewRef<double> full_albedo;

  // Read the albedo
  if (!opt.input_albedo.empty()) {
    vw::vw_out() << "Reading albedo from: " << opt.input_albedo << "\n";
    full_albedo = vw::DiskImageView<double>(opt.input_albedo);
    // Must have the same size as dem
    if (full_albedo.cols() != full_dem.cols() || full_albedo.rows() != full_dem.rows())
      vw::vw_throw(vw::ArgumentErr()
                << "The input albedo must have the same dimensions as the DEM.\n");
  }

  // This must be done before the DEM is cropped. This stats is
  // queried from parallel_sfs.
  if (opt.query) {
    vw::vw_out() << "dem_cols, " << full_dem.cols() << "\n";
    vw::vw_out() << "dem_rows, " << full_dem.rows() << "\n";
  }

  // Read the georeference
  vw::cartography::GeoReference albedo_geo;
  loadGeoref(opt, geo, albedo_geo);

  // If --ref-map is passed in, need to crop to its extent. This modifies
  // opt.crop_win.
  setupRefMap(full_dem, geo, opt);
  
  // Adjust the crop win
  opt.crop_win.crop(vw::bounding_box(full_dem));

  // Crop the DEM and georef if requested to given box. Same for albedo.
  // In either case, read the needed portion fully in memory.
  if (!opt.crop_win.empty()) {
    dem = vw::crop(full_dem, opt.crop_win);
    geo = vw::cartography::crop(geo, opt.crop_win);
    if (!opt.input_albedo.empty()) {
      albedo = vw::crop(full_albedo, opt.crop_win);
      albedo_geo = vw::cartography::crop(albedo_geo, opt.crop_win);
    }
  } else {
    // No cropping
    dem = full_dem;
    if (!opt.input_albedo.empty())
      albedo = full_albedo;
  }

  // Initialize the albedo if not read from disk
  if (opt.input_albedo.empty()) {
    double initial_albedo = 1.0;
    albedo.set_size(dem.cols(), dem.rows());
    for (int col = 0; col < albedo.cols(); col++) {
      for (int row = 0; row < albedo.rows(); row++) {
        albedo(col, row) = initial_albedo;
      }
    }
    albedo_geo = geo;
  }

  // Albedo and DEM must have same dimensions and georef
  if (albedo.cols() != dem.cols() || albedo.rows() != dem.rows())
    vw::vw_throw(vw::ArgumentErr()
              << "The albedo image must have the same dimensions as the DEM.\n");
  if (geo.get_wkt() != albedo_geo.get_wkt())
    vw::vw_throw(vw::ArgumentErr()
                  << "The input DEM has a different georeference "
                  << "from the input albedo image.\n");
}

void sfsSanityChecks(asp::SfsOptions const& opt, 
                     vw::ImageView<double> const& dem, 
                     double dem_nodata_val) {

  // Refuse to run if there are no-data values or if the DEM is too small
  int min_dem_size = 5;
  for (int col = 0; col < dem.cols(); col++) {
    for (int row = 0; row < dem.rows(); row++) {
      if (dem(col, row) == dem_nodata_val ||
          std::isnan(dem(col, row))) {
        vw::vw_throw(vw::ArgumentErr()
                  << "Found a no-data or NaN pixel in the DEM. Cannot continue. "
                  << "The dem_mosaic tool can be used to fill in holes. Then "
                  << "crop and use a clip from this DEM having only valid data.");
      }
    }
  }
  if (dem.cols() < min_dem_size || dem.rows() < min_dem_size)
    vw::vw_throw(vw::ArgumentErr() << "The input DEM is too small.\n");

  // This check must happen before loading images but after we know the DEM size
  if ((dem.cols() > 500 || dem.rows() > 500) && !opt.compute_exposures_only &&
      !opt.estim_exposure_haze_albedo && !opt.save_sim_intensity_only &&
      !opt.save_meas_intensity_only)
    vw::vw_out(vw::WarningMessage) << "The input DEM is large and this program "
      << "may run out of memory. Use parallel_sfs instead, with small tiles.\n";
}

// Initialize the exposure as mean(intensity)/mean(reflectance)/albedo.
// Initialize the haze to 0. Skip images with zero exposure. If the user
// provided initial exposures and haze, use those, but still go through the
// motions to find the images to skip.
void estimExposuresHazeSkipVec(asp::SfsOptions const& opt,
                               vw::ImageView<double> const& dem,
                               vw::cartography::GeoReference const& geo,
                               double max_dem_height,
                               double gridx, double gridy,
                               std::vector<vw::Vector3>   const& sunPosition,
                               asp::ReflParams            const& refl_params,
                               std::vector<vw::BBox2i>    const& crop_boxes,
                               std::vector<MaskedImgRefT> const& masked_images,
                               std::vector<vw::ImageView<double>> const& blend_weights,
                               bool blend_weight_is_ground_weight,
                               std::vector<vw::CamPtr>    const& cameras,
                               double mean_albedo,
                               // Outputs
                               std::vector<double> & local_exposures_vec,
                               std::vector<double> & local_haze_vec,
                               std::set<int>       & skip_images) {

    // Initialize outputs
    int num_images = opt.input_images.size();
    local_exposures_vec.resize(num_images, 0);
    local_haze_vec.resize(num_images, 0);
    
    for (int image_iter = 0; image_iter < num_images; image_iter++) {

      if (skip_images.find(image_iter) != skip_images.end())
        continue;

      // Sample large DEMs. Keep about 200 row and column samples.
      int sample_col_rate = 0, sample_row_rate = 0;
      asp::calcSampleRates(dem, opt.num_samples_for_estim, sample_col_rate, sample_row_rate);
      
      MaskedDblImgT reflectance, intensity;
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
                                     reflectance, intensity, ground_weight,
                                     &opt.model_coeffs_vec[0], opt);
      asp::calcExposureHazeSkipImages(intensity, reflectance, mean_albedo,
                                      image_iter, opt.input_images,
                                      local_exposures_vec, local_haze_vec,
                                      skip_images);
    }
}

int main(int argc, char* argv[]) {

  vw::Stopwatch sw_total;
  sw_total.start();

  asp::SfsOptions opt;
  try {
    asp::handleSfsArgs(argc, argv, opt);

    // Set up model information
    ReflParams refl_params;
    setupReflectance(refl_params, opt);

    double dem_nodata_val = -1e+6; // Will change
    vw::ImageView<double> dem, albedo;
    vw::cartography::GeoReference geo;
    prepareDemAndAlbedo(opt, dem, albedo, geo, dem_nodata_val);

    // See if to use provided initial DEM height
    if (!boost::math::isnan(opt.init_dem_height)) {
      for (int col = 0; col < dem.cols(); col++) {
        for (int row = 0; row < dem.rows(); row++) {
          dem(col, row) = opt.init_dem_height;
        }
      }
    }

    // Read in the camera models and the sun positions
    std::vector<vw::CamPtr> cameras;
    std::vector<vw::Vector3> sunPosition;
    loadCamerasSunPos(opt, dem, dem_nodata_val, geo, cameras, sunPosition);

    // Stop here if all we wanted was some information
    if (opt.query)
      return 0;

    // Do the sanity checks after the query, as that one better not fail  
    sfsSanityChecks(opt, dem, dem_nodata_val);

    // This check must be here, after we find the session
    if (opt.stereo_session != "isis" && opt.use_approx_camera_models) {
      vw::vw_out() << "Computing approximate models works only with ISIS cameras. "
                   << "Ignoring this option.\n";
      opt.use_approx_camera_models = false;
    }

    if (opt.num_threads > 1 && opt.stereo_session == "isis" && 
        !opt.use_approx_camera_models) {
      vw::vw_out() << "Using exact ISIS camera models. Can run with only a single thread.\n";
      opt.num_threads = 1;
    }
    vw::vw_out() << "Using: " << opt.num_threads << " thread(s).\n";
    
    // We won't load the full images, just portions restricted to the area we we
    // will compute the DEM.
    int num_images = opt.input_images.size();    
    std::vector<vw::BBox2i> crop_boxes(num_images);

    // The crop box starts as the original image bounding box. We'll shrink it later.
    for (int image_iter = 0; image_iter < num_images; image_iter++) {
      std::string img_file = opt.input_images[image_iter];
      crop_boxes[image_iter] = vw::bounding_box(vw::DiskImageView<float>(img_file));
    }

    // Ensure that no two threads can access an ISIS camera at the same time.
    // Declare the lock here, as we want it to live until the end of the program.
    vw::Mutex camera_mutex;
    if (opt.use_approx_camera_models) // calc approx camera models and crop boxes
      calcApproxCamsCropBoxes(dem, geo, dem_nodata_val,
                              opt, cameras, crop_boxes, camera_mutex); // outputs
    else if (opt.crop_input_images) // calc crop boxes with the exact camera models
      calcCropBoxes(dem, geo, cameras, opt, crop_boxes);

    // Masked images and weights
    float img_nodata_val = -std::numeric_limits<float>::max(); // will change
    std::vector<MaskedImgRefT> masked_images;
    std::vector<vw::ImageView<double>> blend_weights;
    loadMaskedImagesCalcWeights(opt, crop_boxes,
                                masked_images, blend_weights, img_nodata_val); // outputs

    // Find the grid sizes in meters. Note that dem heights are in meters too,
    // so we treat both horizontal and vertical measurements in same units.
    // Sample large DEMs. Keep about 200 row and column samples.
    int sample_col_rate = 0, sample_row_rate = 0;
    asp::calcSampleRates(dem, opt.num_samples_for_estim, sample_col_rate, sample_row_rate);
    double gridx = 0.0, gridy = 0.0;
    asp::calcGsd(dem, geo, dem_nodata_val, sample_col_rate, sample_row_rate,
                 gridx, gridy); // outputs
    vw::vw_out() << "DEM grid in x and y in meters: " << gridx << ' ' << gridy << "\n";

    // Find the max DEM height
    double max_dem_height = asp::maxDemHeight(dem);

    // Find the mean albedo
    double mean_albedo = asp::meanAlbedo(dem, albedo, dem_nodata_val);

    // Initalize the exposure, haze, and skip images
    bool blend_weight_is_ground_weight = false; // will change later
    std::vector<double> local_exposures_vec, local_haze_vec;
    estimExposuresHazeSkipVec(opt, dem, geo, max_dem_height, gridx, gridy,
                              sunPosition, refl_params, crop_boxes, masked_images,
                              blend_weights, blend_weight_is_ground_weight,
                              cameras, mean_albedo,
                              // Outputs
                              local_exposures_vec, local_haze_vec,
                              opt.skip_images);
    
    // Only overwrite the exposures if we don't have them supplied
    if (opt.image_exposures_vec.empty())
      opt.image_exposures_vec = local_exposures_vec;

    // Initialize the haze as 0. If computed above, initialize its first coeff.
    if ((!opt.image_haze_vec.empty()) && (int)opt.image_haze_vec.size() != num_images)
      vw::vw_throw(vw::ArgumentErr() << "Expecting as many haze values as images.\n");
    if (opt.image_haze_vec.empty()) {
      for (int image_iter = 0; image_iter < num_images; image_iter++) {
        // Pad the haze vec with zeros
        std::vector<double> curr_image_haze_vec;
        while (curr_image_haze_vec.size() < g_max_num_haze_coeffs)
          curr_image_haze_vec.push_back(0);
        curr_image_haze_vec[0] = local_haze_vec[image_iter];
        opt.image_haze_vec.push_back(curr_image_haze_vec);
      }
    }

    // TODO(oalexan1): Check if --num-haze-coeffs is non-zero.
    // TODO(oalexan1): This should work even if albedo is not modeled.
    if (opt.estim_exposure_haze_albedo && (opt.float_albedo || opt.num_haze_coeffs > 0))
      estimExposureHazeAlbedo(opt, masked_images, blend_weights,
                              blend_weight_is_ground_weight,
                              dem, mean_albedo,
                              geo, cameras, max_dem_height,
                              crop_boxes, sunPosition,
                              refl_params, gridx, gridy);

    if (opt.compute_exposures_only || opt.estim_exposure_haze_albedo) {
      asp::saveExposures(opt.out_prefix, opt.input_images, opt.image_exposures_vec);
      // TODO(oalexan1): Think of this more
      if (opt.num_haze_coeffs > 0)
        asp::saveHaze(opt.out_prefix, opt.input_images, opt.image_haze_vec);
      return 0; // all done
    }

    // Need to compute the valid data image to be able to find the grid points always
    // in shadow, so when this image is zero.
    vw::ImageView<int> lit_image_mask;
    if (opt.curvature_in_shadow_weight > 0.0) {
      lit_image_mask.set_size(dem.cols(), dem.rows());
      for (int col = 0; col < lit_image_mask.cols(); col++) {
        for (int row = 0; row < lit_image_mask.rows(); row++) {
          lit_image_mask(col, row) = 0; // no valid points originally
        }
      }
    }

    // Compute and/or save the intensities, and/or estimate height errors,
    // and/or compute weights for borderline data.
    // Show progress when the DEM is big
    std::vector<vw::ImageView<double>> ground_weights(num_images);
    std::vector<MaskedDblImgT> meas_intensities(num_images);
    std::vector<MaskedDblImgT> sim_intensities(num_images);
    bool show_progress = (opt.save_sim_intensity_only || opt.save_meas_intensity_only);
    if (opt.save_sim_intensity_only || opt.save_meas_intensity_only ||
        opt.estimate_height_errors || opt.curvature_in_shadow_weight > 0.0 ||
        opt.allow_borderline_data || opt.low_light_threshold > 0.0)
      asp::calcIntenEstimHeights(opt, dem, albedo, geo, show_progress, max_dem_height,
                                 gridx, gridy, sunPosition, refl_params, crop_boxes,
                                 masked_images, blend_weights, blend_weight_is_ground_weight,
                                 cameras, img_nodata_val,
                                 // Outputs
                                 lit_image_mask, ground_weights,
                                 meas_intensities, sim_intensities);

    if (opt.save_sim_intensity_only || opt.save_meas_intensity_only ||
        opt.estimate_height_errors) {
      asp::saveExposures(opt.out_prefix, opt.input_images, opt.image_exposures_vec);
      return 0; // All done
    }

    // Print and make global the exposures and haze
    if (opt.num_haze_coeffs > 0) {
      for (size_t image_iter = 0; image_iter < opt.image_haze_vec.size(); image_iter++) {
        vw::vw_out() << "Image haze for " << opt.input_images[image_iter] << ':';
        for (size_t haze_it = 0; haze_it < opt.image_haze_vec[image_iter].size(); haze_it++)
          vw::vw_out() << " " << opt.image_haze_vec[image_iter][haze_it];
        vw::vw_out() << "\n";
      }
    }

    if (opt.allow_borderline_data)
      asp::handleBorderlineAndLowLight(opt, num_images, dem, geo, crop_boxes,
                                       meas_intensities, sim_intensities,
                                       img_nodata_val,
                                       // Outputs
                                       masked_images, blend_weights,
                                       blend_weight_is_ground_weight,
                                       ground_weights);
    
    // Calc the weight for option --curvature-in-shadow-weight
    vw::ImageView<double> curvature_in_shadow_weight;
    if (opt.curvature_in_shadow_weight > 0.0)
      asp::calcCurvatureInShadowWeight(opt, lit_image_mask, geo,
                                       curvature_in_shadow_weight); // output

    // For images that we don't use, wipe the cameras and all other
    // info, as those take up memory (the camera is a table).
    for (int image_iter = 0; image_iter < num_images; image_iter++) {
      if (opt.skip_images.find(image_iter) != opt.skip_images.end()) {
        masked_images[image_iter] = vw::ImageView<vw::PixelMask<float>>();
        blend_weights[image_iter] = vw::ImageView<double>();
        cameras[image_iter] = boost::shared_ptr<CameraModel>();
      }
    }

    // orig_dem will keep the input DEMs and won't change. Keep to the optimized
    // DEMs close to orig_dem. Make a deep copy below. Then run sfs.
    vw::ImageView<double> orig_dem = vw::copy(dem);
    runSfs(opt.max_iterations, gridx, gridy, opt, geo, opt.smoothness_weight,
           max_dem_height, dem_nodata_val, img_nodata_val,  crop_boxes, masked_images,
           blend_weights, blend_weight_is_ground_weight,
           refl_params, sunPosition, orig_dem,
           lit_image_mask, curvature_in_shadow_weight,
           // Variable quantities
           dem, albedo, cameras, opt.image_exposures_vec, opt.image_haze_vec,
           opt.model_coeffs_vec);

  } ASP_STANDARD_CATCHES;

  sw_total.stop();
  vw::vw_out() << "Total elapsed time: " << sw_total.elapsed_seconds() << " s.\n";

  return 0;
}
