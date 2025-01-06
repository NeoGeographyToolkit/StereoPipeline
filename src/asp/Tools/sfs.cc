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

// Deal with outliers in image intensity.
// TODO: Ensure the output DEM is float. Check its no-data value.
// TODO: Study more floating model coefficients.
// TODO: Study why using tabulated camera model and multiple resolutions does
// not work as well as it should.
// TODO: When using approx camera, we assume the DEM and image grids are very similar.
// TODO: Remove warning from the approx camera
// TODO: Make it possible to initialize a DEM from scratch.
// TODO: Must specify in the SfS doc that the Lunar-Lambertian model fails at poles
// TODO: If this code becomes multi-threaded, need to keep in mind
// that camera models are shared and modified, so
// this may cause problems.
// TODO: How to relax conditions at the boundary to improve the accuracy?
// TODO: Study if blurring the input images improves the fit.
// TODO: Add --orthoimage option, and make it clear where the final DEM is.
// Same for albedo. The other info should be printed only in debug mode.
// TODO: Study the effect of using bicubic interpolation.
// TODO: Study phaseCoeffC1, etc.
// TODO: Find a good automatic value for the smoothness weight.
// How to change the smoothness weight if the number of images changes?
// TODO: Investigate the sign of the normal.
// TODO: Check that we are within image boundaries when interpolating.
// TODO: Radiometric calibration of images.
// TODO: Add various kind of loss function.
// TODO: Study the normal computation formula.
// TODO: Move some code to Core.
// TODO: Clean up some of the classes, not all members are needed.
// TODO(oalexan1): Must implement initialization of exposure, haze, and also albedo
// if albedo is modeled, at low-res.
// TODO(oalexan1): Modularize. 
// TODO(oalexan1): Separate the optimization logic from image operations.
// TODO(oalexan1): Move image logic to SfsImageProc.cc. Add 
// SfS cost function file.
// TODO(oalexan1): Use OpenMP here in ComputeReflectanceAndIntensity.
// For isis without approx models need to ensure there is one thread.
// May need to differentiate between single process and multiple processes.

/// \file sfs.cc

// Turn off warnings from boost and other packages
#if defined(__GNUC__) || defined(__GNUG__)
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-local-typedefs"
#endif

#include <asp/Core/Macros.h>
#include <asp/Core/Common.h>
#include <asp/Sessions/StereoSessionFactory.h>
#include <asp/IsisIO/IsisCameraModel.h>
#include <asp/Camera/CsmModel.h>
#include <asp/Core/CameraUtils.h>
#include <asp/Core/StereoSettings.h>
#include <asp/SfS/SfsImageProc.h>
#include <asp/SfS/SfsUtils.h>
#include <asp/SfS/SfsCamera.h>
#include <asp/SfS/SfsReflectanceModel.h>

#include <vw/Image/MaskViews.h>
#include <vw/Image/AntiAliasing.h>
#include <vw/Image/InpaintView.h>
#include <vw/Image/DistanceFunction.h>
#include <vw/Cartography/GeoReferenceUtils.h>
#include <vw/Core/Stopwatch.h>
#include <vw/Core/CmdUtils.h>

#include <ceres/ceres.h>
#include <ceres/loss_function.h>

#include <iostream>
#include <stdexcept>
#include <string>
#include<sys/types.h>

#if defined(__GNUC__) || defined(__GNUG__)
#pragma GCC diagnostic pop
#undef LOCAL_GCC_VERSION
#endif

namespace po = boost::program_options;
namespace fs = boost::filesystem;

const size_t g_num_model_coeffs = 16;
const size_t g_max_num_haze_coeffs = 6; // see nonlinReflectance()

// If the blend weight is ground weight, rather than image weight,
// use it as it is, without projecting into camera and interpolating.
// It is a lot of work to pass this all over the place.
// TODO(oalexan1): Pass this properly. Use bool opt_allow_borderline_data,
// pass it along with bool model_shadows. Will need a testcase.
bool g_blend_weight_is_ground_weight = false;

using namespace vw;
using namespace vw::camera;
using namespace vw::cartography;

typedef ImageViewRef<PixelMask<float>> MaskedImgT;
typedef ImageViewRef<double> DoubleImgT;

using namespace asp;

// Get the memory usage for the given process. This is for debugging, not used
// in production code. It does not work on OSX.
void callTop() {

  std::ostringstream os;
  int pid = getpid();
  os << pid;
  
  std::string cmd = "top -b -n 1 | grep -i ' sfs' | grep -i '" + os.str() + "'";
  std::string ans = vw::exec_cmd(cmd.c_str());
  vw_out() << "Memory usage: " << cmd << " " << ans << "\n";
}

struct Options: public vw::GdalWriteOptions {
  std::string input_dem, image_list, camera_list, out_prefix, stereo_session, bundle_adjust_prefix, input_albedo;
  std::vector<std::string> input_images, input_cameras;
  std::string shadow_thresholds, custom_shadow_threshold_list, max_valid_image_vals, skip_images_str, image_exposures_prefix, model_coeffs_prefix, model_coeffs, image_haze_prefix, sun_positions_list, sun_angles_list;
  std::vector<float> shadow_threshold_vec, max_valid_image_vals_vec;
  std::vector<double> image_exposures_vec;
  std::vector<std::vector<double>> image_haze_vec;
  std::vector<double> model_coeffs_vec;
  std::set<int> skip_images;
  int max_iterations, reflectance_type, blending_dist, min_blend_size, num_haze_coeffs;
  bool float_albedo, float_exposure, model_shadows,
    save_computed_intensity_only, estimate_slope_errors, estimate_height_errors,
    compute_exposures_only, estimate_exposure_haze_albedo,
    save_dem_with_nodata, use_approx_camera_models, 
    crop_input_images, allow_borderline_data, fix_dem, float_reflectance_model, 
    query, save_sparingly, float_haze, read_exposures, read_haze, read_albedo;
    
  double smoothness_weight, steepness_factor, curvature_in_shadow,
    curvature_in_shadow_weight,
    lit_curvature_dist, shadow_curvature_dist, gradient_weight,
    blending_power, integrability_weight, smoothness_weight_pq, init_dem_height,
    nodata_val, initial_dem_constraint_weight, albedo_constraint_weight,
    albedo_robust_threshold, camera_position_step_size, unreliable_intensity_threshold, 
    robust_threshold, shadow_threshold;
  vw::BBox2 crop_win;
  vw::Vector2 height_error_params;
  
  Options(): max_iterations(0), reflectance_type(0),
            blending_dist(0), blending_power(2.0),
            min_blend_size(0), num_haze_coeffs(0),
            float_albedo(false), float_exposure(false), model_shadows(false), 
            save_computed_intensity_only(false),
            estimate_slope_errors(false),
            estimate_height_errors(false),
            compute_exposures_only(false),
            estimate_exposure_haze_albedo(false),
            save_dem_with_nodata(false),
            use_approx_camera_models(false),
            crop_input_images(false),
            allow_borderline_data(false), fix_dem(false),
            float_reflectance_model(false), query(false), 
            save_sparingly(false), float_haze(false),
            smoothness_weight(0), steepness_factor(1.0),
            curvature_in_shadow(0), curvature_in_shadow_weight(0.0),
            lit_curvature_dist(0.0), shadow_curvature_dist(0.0),
            gradient_weight(0.0), integrability_weight(0), smoothness_weight_pq(0),
            initial_dem_constraint_weight(0.0),
            albedo_constraint_weight(0.0), albedo_robust_threshold(0.0),
            camera_position_step_size(1.0), unreliable_intensity_threshold(0.0),
            crop_win(BBox2i(0, 0, 0, 0)){}
};

// Use this struct to keep track of height errors.
// TODO(oalexan1): Move to SfsErrorEstim.h
struct HeightErrEstim {

  HeightErrEstim(int num_cols, int num_rows, int num_height_samples_in,
                 double max_height_error_in, double nodata_height_val_in,
                 ImageView<double> * albedo_in,
                 Options * opt_in) {
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
  }
  
  int num_height_samples;
  ImageView<double> * albedo;
  Options * opt;
  ImageView<Vector2> height_error_vec;
  int image_iter;
  double max_height_error;
  double nodata_height_val;
};

// Use this struct to keep track of slope errors.
// TODO(oalexan1): Move to SfsErrorEstim.h
struct SlopeErrEstim {

  SlopeErrEstim(int num_cols, int num_rows, int num_a_samples_in, int num_b_samples_in,
                ImageView<double> * albedo_in, Options * opt_in) {
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
  }
  
  int num_a_samples, num_b_samples;
  ImageView<double> * albedo;
  Options * opt;
  std::vector<std::vector<std::vector<double>>> slope_errs;
  int image_iter;
  double max_angle;
};

// Given the normal (slope) to the SfS DEM, find how different
// a slope can be from this before the computed intensity
// due to that slope is bigger than max_intensity_err.
// TODO(oalexan1): Move to SfsErrorEstim.h
void estimateSlopeError(Vector3 const& cameraPosition,
                        Vector3 const& normal, Vector3 const& xyz,
                        vw::Vector3 const& sunPosition,
                        ReflParams const& refl_params,
                        const double * refl_coeffs,
                        double meas_intensity,
                        double max_intensity_err,
                        int col, int row, int image_iter,
                        Options & opt,
                        ImageView<double> & albedo,
                        SlopeErrEstim * slopeErrEstim){
  
  // Find the angle u from the normal to the z axis, and the angle v
  // from the x axis to the projection of the normal in the xy plane.
  double u = acos(normal[2]);
  double v = 0.0;
  if (normal[0] != 0.0 || normal[1] != 0.0) 
    v = atan2(normal[1], normal[0]);

  double cv = cos(v), sv = sin(v), cu = cos(u), su = sin(u);
  Vector3 n(cv*su, sv*su, cu);

  // Sanity check, these angles should give us back the normal
  if (norm_2(normal - n) > 1e-8) 
    vw_throw( LogicErr() << "Book-keeping error in slope estimation.\n" );
    
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
  Vector3 n0(0, 0, 1);
  if (norm_2(R*n0 - n) > 1e-8) 
    vw_throw( LogicErr() << "Book-keeping error in slope estimation.\n" );
  
  int num_a_samples = slopeErrEstim->num_a_samples;
  int num_b_samples = slopeErrEstim->num_b_samples;

  int num_cols = slopeErrEstim->slope_errs.size();
  int num_rows = slopeErrEstim->slope_errs[0].size();
  int num_b_samples2 = slopeErrEstim->slope_errs[0][0].size();

  if (num_b_samples != num_b_samples2)
    vw_throw( LogicErr()
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

      Vector3 w(cb*sa, sb*sa, ca);
      w = R*w;

      // Compute here dot product from w to n. Should be cos(a) for all b.
      double prod = dot_prod(w, normal);
      if (std::abs(prod - ca) > 1e-8)
        vw_throw( LogicErr() << "Book-keeping error in slope estimation.\n" );

      // Compute the reflectance with the given normal
      PixelMask<double> reflectance = calcReflectance(cameraPosition,
                                                      w, xyz, sunPosition,
                                                      refl_params, refl_coeffs);
      reflectance.validate();

      double comp_intensity = calcIntensity(albedo(col, row), 
                                            reflectance, 
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
}

// Given the normal (height) to the SfS DEM, find how different
// a height can be from this before the computed intensity
// due to that height is bigger than max_intensity_err.
// TODO(oalexan1): Move to SfsErrorEstim.h
void estimateHeightError(ImageView<double> const& dem,
                         vw::cartography::GeoReference const& geo,
                         Vector3 const& cameraPosition,
                         vw::Vector3 const& sunPosition,
                         ReflParams const& refl_params,
                         const double * refl_coeffs,
                         double meas_intensity,
                         double max_intensity_err,
                         int col, int row, 
                         double grid_x, double grid_y,
                         int image_iter,
                         Options & opt,
                         ImageView<double> & albedo,
                         HeightErrEstim * heightErrEstim){

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
        if      (colx == col - 1 && rowx == row    ) left_dh   = dh; 
        else if (colx == col     && rowx == row    ) center_dh = dh; // won't be reached
        else if (colx == col + 1 && rowx == row    ) right_dh  = dh; 
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
        
        PixelMask<double> reflectance = calcReflectance(cameraPosition,
                                                        normal, xyz, sunPosition,
                                                        refl_params, refl_coeffs);
        reflectance.validate();
        double comp_intensity = calcIntensity(albedo(col, row), 
                                              reflectance, 
                                              opt.image_exposures_vec[image_iter],
                                              opt.steepness_factor,
                                              &opt.image_haze_vec[image_iter][0], 
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
}

bool computeReflectanceAndIntensity(double left_h, double center_h, double right_h,
                                    double bottom_h, double top_h,
                                    bool use_pq, double p, double q, // dem partial derivatives
                                    int col, int row,
                                    ImageView<double>         const& dem,
                                    cartography::GeoReference const& geo,
                                    bool model_shadows,
                                    double max_dem_height,
                                    double gridx, double gridy,
                                    vw::Vector3  const & sunPosition,
                                    ReflParams const & refl_params,
                                    BBox2i       const & crop_box,
                                    MaskedImgT   const & image,
                                    DoubleImgT   const & blend_weight,
                                    CameraModel  const * camera,
                                    PixelMask<double>  & reflectance,
                                    PixelMask<double>  & intensity,
                                    double             & ground_weight,
                                    const double       * refl_coeffs,
                                    SlopeErrEstim      * slopeErrEstim = NULL,
                                    HeightErrEstim     * heightErrEstim = NULL) {

  // Set output values
  reflectance = 0.0; reflectance.invalidate();
  intensity   = 0.0; intensity.invalidate();
  ground_weight = 0.0;
  
  if (col >= dem.cols() - 1 || row >= dem.rows() - 1) return false;
  if (crop_box.empty()) return false;

  vw::Vector3 xyz, normal;
  calcPointAndNormal(col, row, left_h, center_h, right_h, bottom_h, top_h,
                     use_pq, p, q, geo, gridx, gridy, xyz, normal);

  // Update the camera position for the given pixel (camera position
  // is pixel-dependent for linescan cameras).
  Vector2 pix;
  Vector3 cameraPosition;
  try {
    pix = camera->point_to_pixel(xyz);
    
    // Need camera center only for Lunar Lambertian
    if (refl_params.reflectanceType != LAMBERT)
      cameraPosition = camera->camera_center(pix);
    
  } catch(...){
    reflectance = 0.0; reflectance.invalidate();
    intensity   = 0.0; intensity.invalidate();
    ground_weight = 0.0;
    return false;
  }
  
  reflectance = calcReflectance(cameraPosition,
                                normal, xyz, sunPosition,
                                refl_params, refl_coeffs);
  reflectance.validate();


  // Since our image is cropped
  pix -= crop_box.min();

  // Check for out of range
  if (pix[0] < 0 || pix[0] >= image.cols() - 1 || pix[1] < 0 || pix[1] >= image.rows() - 1) {
    reflectance = 0.0; reflectance.invalidate();
    intensity   = 0.0; intensity.invalidate();
    ground_weight = 0.0;
    return false;
  }

  InterpolationView<EdgeExtensionView<MaskedImgT, ConstantEdgeExtension>, BilinearInterpolation>
    interp_image = interpolate(image, BilinearInterpolation(),
                               ConstantEdgeExtension());
  intensity = interp_image(pix[0], pix[1]); // this interpolates

  if (g_blend_weight_is_ground_weight) {
    if (blend_weight.cols() != dem.cols() || blend_weight.rows() != dem.rows()) 
      vw::vw_throw(vw::ArgumentErr() 
                   << "Ground weight must have the same size as the DEM.\n");
    ground_weight = blend_weight(col, row);
  } else {
    InterpolationView<EdgeExtensionView<DoubleImgT, ConstantEdgeExtension>, BilinearInterpolation>
      interp_weight = interpolate(blend_weight, BilinearInterpolation(),
                                  ConstantEdgeExtension());
    if (blend_weight.cols() > 0 && blend_weight.rows() > 0) // The weight may not exist
      ground_weight = interp_weight(pix[0], pix[1]); // this interpolates
    else
      ground_weight = 1.0;
  }
  
  // Note that we allow negative reflectance for valid intensity. It will hopefully guide
  // the SfS solution the right way.
  if (!is_valid(intensity)) {
    reflectance = 0.0; reflectance.invalidate();
    intensity   = 0.0; intensity.invalidate();
    ground_weight = 0.0;
    return false;
  }

  if (model_shadows) {
    bool inShadow = asp::isInShadow(col, row, sunPosition,
                                    dem, max_dem_height, gridx, gridy,
                                    geo);

    if (inShadow) {
      // The reflectance is valid, it is just zero
      reflectance = 0;
      reflectance.validate();
    }
  }

  if (slopeErrEstim != NULL && is_valid(intensity) && is_valid(reflectance)) {
    
    int image_iter = slopeErrEstim->image_iter;
    Options & opt = *slopeErrEstim->opt; // alias
    ImageView<double> & albedo = *slopeErrEstim->albedo; // alias
    double comp_intensity = calcIntensity(albedo(col, row), 
                                          reflectance, 
                                          opt.image_exposures_vec[image_iter],
                                          opt.steepness_factor,
                                          &opt.image_haze_vec[image_iter][0], 
                                          opt.num_haze_coeffs);

    // We use twice the discrepancy between the computed and measured intensity
    // as a measure for how far is overall the computed intensity allowed
    // to diverge from the measured intensity
    double max_intensity_err = 2.0 * std::abs(intensity.child() - comp_intensity);
    estimateSlopeError(cameraPosition, normal, xyz, sunPosition,
                       refl_params, refl_coeffs, intensity.child(), 
                       max_intensity_err, col, row, image_iter,
                       opt, albedo, slopeErrEstim);
  }
  
  if (heightErrEstim != NULL && is_valid(intensity) && is_valid(reflectance)) {
    
    int image_iter = heightErrEstim->image_iter;
    Options & opt = *heightErrEstim->opt; // alias
    ImageView<double> & albedo = *heightErrEstim->albedo; // alias
    double comp_intensity = calcIntensity(albedo(col, row), 
                                          reflectance, 
                                          opt.image_exposures_vec[image_iter],
                                          opt.steepness_factor,
                                          &opt.image_haze_vec[image_iter][0], 
                                          opt.num_haze_coeffs);
    
    // We use twice the discrepancy between the computed and measured intensity
    // as a measure for how far is overall the computed intensity allowed
    // to diverge from the measured intensity
    double max_intensity_err = 2.0 * std::abs(intensity.child() - comp_intensity);

    estimateHeightError(dem, geo,  
                        cameraPosition, sunPosition,  refl_params,  
                        refl_coeffs, intensity.child(),  
                        max_intensity_err, 
                        col, row, gridx, gridy, 
                        image_iter, opt, albedo,  
                        heightErrEstim);
  }
  
  return true;
}

// TODO(oalexan1): Move to SfsReflectanceModel.h
void computeReflectanceAndIntensity(ImageView<double> const& dem,
                                    ImageView<Vector2> const& pq,
                                    cartography::GeoReference const& geo,
                                    bool model_shadows,
                                    double & max_dem_height, // alias
                                    double gridx, double gridy,
                                    int sample_col_rate, int sample_row_rate,
                                    vw::Vector3 const& sunPosition,
                                    ReflParams const& refl_params,
                                    BBox2i const& crop_box,
                                    MaskedImgT const  & image,
                                    DoubleImgT const  & blend_weight,
                                    CameraModel const * camera,
                                    ImageView<PixelMask<double>> & reflectance,
                                    ImageView<PixelMask<double>> & intensity,
                                    ImageView<double>            & ground_weight,
                                    const double   * refl_coeffs,
                                    SlopeErrEstim  * slopeErrEstim = NULL,
                                    HeightErrEstim * heightErrEstim = NULL) {
  
  // Update max_dem_height
  max_dem_height = -std::numeric_limits<double>::max();
  if (model_shadows) {
    for (int col = 0; col < dem.cols(); col += sample_col_rate) {
      for (int row = 0; row < dem.rows(); row += sample_row_rate) {
        if (dem(col, row) > max_dem_height) {
          max_dem_height = dem(col, row);
        }
      }
    }
    vw_out() << "Maximum DEM height: " << max_dem_height << std::endl;
  }
  
  // See how many samples we end up having further down. Must start counting
  // from 1, just as we do in that loop.
  int num_sample_cols = 0, num_sample_rows = 0;
  for (int col = 1; col < dem.cols() - 1; col += sample_col_rate) 
    num_sample_cols++;
  for (int row = 1; row < dem.rows() - 1; row += sample_row_rate)
    num_sample_rows++;
  
  // Add 2 for book-keeping purposes, to ensure that when the sampling rate
  // is 1, we get as many cols and rows as the DEM has.
  num_sample_cols += 2;
  num_sample_rows += 2;
  
  // Important sanity check
  if (sample_col_rate == 1 && num_sample_cols != dem.cols())
    vw_throw(LogicErr() << "Book-keeping error in computing reflectance and intensity.\n");
  if (sample_row_rate == 1 && num_sample_rows != dem.rows())
    vw_throw(LogicErr() << "Book-keeping error in computing reflectance and intensity.\n");
      
  // Init the reflectance and intensity as invalid. Do it at all grid
  // points, not just where we sample, to ensure that these quantities
  // are fully initialized.
  reflectance.set_size(num_sample_cols, num_sample_rows);
  intensity.set_size(num_sample_cols, num_sample_rows);
  ground_weight.set_size(num_sample_cols, num_sample_rows);
  for (int col = 0; col < num_sample_cols; col++) {
    for (int row = 0; row < num_sample_rows; row++) {
      reflectance(col, row).invalidate();
      intensity(col, row).invalidate();
      ground_weight(col, row) = 0.0;
    }
  }

  // Need to very carefully distinguish below between col and col_sample,
  // and between row and row_sample. These are same only if the sampling
  // rate is 1.
  bool use_pq = (pq.cols() > 0 && pq.rows() > 0);
  int col_sample = 0;
  for (int col = 1; col < dem.cols() - 1; col += sample_col_rate) {
    col_sample++;
    
    int row_sample = 0;
    for (int row = 1; row < dem.rows() - 1; row += sample_row_rate) {
      row_sample++;
    
      double pval = 0, qval = 0;
      if (use_pq) {
        pval = pq(col, row)[0];
        qval = pq(col, row)[1];
      }
      computeReflectanceAndIntensity(dem(col-1, row), dem(col, row), dem(col+1, row),
                                     dem(col, row+1), dem(col, row-1),
                                     use_pq, pval, qval,
                                     col, row, dem, geo,
                                     model_shadows, max_dem_height,
                                     gridx, gridy,
                                     sunPosition, refl_params,
                                     crop_box, image, blend_weight, camera,
                                     reflectance(col_sample, row_sample),
                                     intensity(col_sample, row_sample),
                                     ground_weight(col_sample, row_sample),
                                     refl_coeffs,
                                     slopeErrEstim,
                                     heightErrEstim);
    }
  }
  
  return;
}

// A function to invoke at every iteration of ceres.
class SfsCallback: public ceres::IterationCallback {
public:

// Constructor to initialize references to the necessary data
// TODO(oalexan1): Move to SfsCostFun.h
SfsCallback(Options const& opt, ImageView<double>& dem,  ImageView<Vector2>& pq,
            ImageView<double>& albedo, cartography::GeoReference const& geo, 
            ReflParams const& refl_params, std::vector<vw::Vector3> const& sunPosition,
            std::vector<BBox2i> const& crop_boxes, 
            std::vector<MaskedImgT> const& masked_images,
            std::vector<DoubleImgT> const& blend_weights,
            std::vector<vw::CamPtr>& cameras, 
            double& dem_nodata_val, float& img_nodata_val,
            std::vector<double>& exposures, std::vector<std::vector<double>>& haze,
            double& max_dem_height, double& gridx, double& gridy,
            std::vector<double> & refl_coeffs):
  opt(opt), dem(dem), pq(pq), albedo(albedo), geo(geo), refl_params(refl_params),
  sunPosition(sunPosition), crop_boxes(crop_boxes), masked_images(masked_images),
  blend_weights(blend_weights), cameras(cameras), dem_nodata_val(dem_nodata_val),
  img_nodata_val(img_nodata_val), exposures(exposures), haze(haze),
  max_dem_height(max_dem_height), gridx(gridx), gridy(gridy),
  refl_coeffs(refl_coeffs),
  iter(-1), final_iter(false) {}

  virtual ceres::CallbackReturnType operator()(const ceres::IterationSummary& 
                                               summary) override {
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
    for (size_t coeff_iter = 0; coeff_iter < g_num_model_coeffs; coeff_iter++) {
      mcf << refl_coeffs[coeff_iter] << " ";
    }
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
  ImageView<double> dem_nodata;
  if (opt.save_dem_with_nodata) {
    dem_nodata = ImageView<double>(dem.cols(), dem.rows());
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

    ImageView<PixelMask<double>> reflectance, intensity, comp_intensity;
    ImageView<double> ground_weight;

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
                                  cameras[image_iter].get(),
                                  reflectance, intensity, ground_weight,
                                  &refl_coeffs[0]); // Pass the address of the first element

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
    ImageView<double> measured_albedo;
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

void set_final_iter(bool is_final_iter) {
  final_iter = is_final_iter;
}
  
private:
  // Class members storing references to the data
  Options const& opt;
  ImageView<double>& dem;
  ImageView<Vector2>& pq;
  ImageView<double>& albedo;
  cartography::GeoReference const& geo;
  ReflParams const& refl_params;
  std::vector<vw::Vector3> const& sunPosition;
  std::vector<BBox2i> const& crop_boxes;
  std::vector<MaskedImgT> const& masked_images;
  std::vector<DoubleImgT> const& blend_weights;
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

// See SmoothnessError() for the definitions of bottom, top, etc.
// TODO(oalexan1): Move to SfsCostFun.h
template <typename F, typename G>
inline bool
calc_intensity_residual(Options const& opt, 
                        const F* const exposure, const F* const haze,
                        const G* const left, const G* const center, const G* const right,
                        const G* const bottom, const G* const top,
                        bool use_pq, const G* const pq, // dem partial derivatives
                        const G* const albedo,
                        const G* const refl_coeffs, 
                        int col, int row,
                        ImageView<double>         const & dem,            // alias
                        cartography::GeoReference const & geo,            // alias
                        bool                              model_shadows,
                        double                            camera_position_step_size,
                        double                    const & max_dem_height, // alias
                        double                            gridx,
                        double                            gridy,
                        ReflParams              const & refl_params,  // alias
                        vw::Vector3               const & sunPosition,   // alias
                        BBox2i                            crop_box,
                        MaskedImgT                const & image,          // alias
                        DoubleImgT                const & blend_weight,   // alias
                        vw::CamPtr                const & camera,         // alias
                        F* residuals) {
  
  // Default residuals. Using here 0 rather than some big number tuned out to
  // work better than the alternative.
  residuals[0] = F(0.0);
  try {
    PixelMask<double> reflectance(0), intensity(0);
    double ground_weight = 0;

    // Need to be careful not to access an array which does not exist
    G p = 0, q = 0;
    if (use_pq) {
      p = pq[0];
      q = pq[1];
    }
    
    bool success =
      computeReflectanceAndIntensity(left[0], center[0], right[0],
                                     bottom[0], top[0],
                                     use_pq, p, q,
                                     col, row,  dem, geo,
                                     model_shadows, max_dem_height,
                                     gridx, gridy,
                                     sunPosition,  refl_params,
                                     crop_box, image, blend_weight, camera.get(),
                                     reflectance, intensity, ground_weight, refl_coeffs);
      
    if (opt.unreliable_intensity_threshold > 0) {
      if (is_valid(intensity) && intensity.child() <= opt.unreliable_intensity_threshold &&
          intensity.child() >= 0) {
        ground_weight *=
          pow(intensity.child()/opt.unreliable_intensity_threshold, 2.0);
      }
    }
      
    if (success && is_valid(intensity) && is_valid(reflectance))
      residuals[0] = ground_weight * (intensity - 
                      calcIntensity(albedo[0], reflectance.child(), exposure[0],
                                    opt.steepness_factor,
                                    haze, opt.num_haze_coeffs));

  } catch (const camera::PointToPixelErr& e) {
    // To be able to handle robustly DEMs that extend beyond the camera,
    // always return true when we fail to project, but with zero residual.
    // This needs more study.
    residuals[0] = F(0.0);
    return true;
  }

  return true;
}

// Discrepancy between measured and computed intensity. See the formula above.
// TODO(oalexan1): Move to SfsCostFun.h
struct IntensityError {
  IntensityError(Options const& opt, int col, int row,
                 ImageView<double> const& dem,
                 cartography::GeoReference const& geo,
                 bool model_shadows,
                 double camera_position_step_size,
                 double const& max_dem_height, // note: this is an alias
                 double gridx, double gridy,
                 ReflParams const& refl_params,
                 vw::Vector3 const& sunPosition,
                 BBox2i const& crop_box,
                 MaskedImgT const& image,
                 DoubleImgT const& blend_weight,
                 boost::shared_ptr<CameraModel> const& camera):
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
                                   m_camera,          // alias
                                   residuals);
  }

  // Factory to hide the construction of the CostFunction object from
  // the client code.
  static ceres::CostFunction* Create(Options const& opt, int col, int row,
                                     ImageView<double> const& dem,
                                     vw::cartography::GeoReference const& geo,
                                     bool model_shadows,
                                     double camera_position_step_size,
                                     double const& max_dem_height, // alias
                                     double gridx, double gridy,
                                     ReflParams const& refl_params,
                                     vw::Vector3 const& sunPosition,
                                     BBox2i const& crop_box,
                                     MaskedImgT const& image,
                                     DoubleImgT const& blend_weight,
                                     boost::shared_ptr<CameraModel> const& camera) {
    return (new ceres::NumericDiffCostFunction<IntensityError,
            ceres::CENTRAL, 1, 1, g_max_num_haze_coeffs, 1, 1, 1, 1, 1, 1, g_num_model_coeffs>
            (new IntensityError(opt, col, row, dem, geo,
                                model_shadows,
                                camera_position_step_size,
                                max_dem_height,
                                gridx, gridy,
                                refl_params, sunPosition,
                                crop_box, image, blend_weight, camera)));
  }

  Options const& m_opt;
  int m_col, m_row;
  ImageView<double>                 const & m_dem;            // alias
  cartography::GeoReference         const & m_geo;            // alias
  bool                                      m_model_shadows;
  double                                    m_camera_position_step_size;
  double                            const & m_max_dem_height; // alias
  double                                    m_gridx, m_gridy;
  ReflParams                      const & m_refl_params;  // alias
  vw::Vector3                       const & m_sunPosition;   // alias
  BBox2i                                    m_crop_box;
  MaskedImgT                        const & m_image;          // alias
  DoubleImgT                        const & m_blend_weight;   // alias
  boost::shared_ptr<CameraModel>    const & m_camera;         // alias
};

// A variation of the intensity error where only the DEM is floated
// TODO(oalexan1): Wipe this as it did not improve speed, but test first. Unlikely it improves memory usage, but need to test. If useful, move to SfsCostFun.h
struct IntensityErrorFloatDemOnly {
  IntensityErrorFloatDemOnly(Options const& opt, int col, int row,
                             ImageView<double> const& dem,
                             double albedo,
                             double * refl_coeffs, 
                             double * exposure, 
                             double * haze, 
                             cartography::GeoReference const& geo,
                             bool model_shadows,
                             double camera_position_step_size,
                             double const& max_dem_height, // note: this is an alias
                             double gridx, double gridy,
                             ReflParams const& refl_params,
                             vw::Vector3 const& sunPosition,
                             BBox2i const& crop_box,
                             MaskedImgT const& image,
                             DoubleImgT const& blend_weight,
                             boost::shared_ptr<CameraModel> const& camera):
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
                                   m_camera,          // alias
                                   residuals);
  }

  // Factory to hide the construction of the CostFunction object from
  // the client code.
  static ceres::CostFunction* Create(Options const& opt, int col, int row,
                                     ImageView<double> const& dem,
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
                                     BBox2i const& crop_box,
                                     MaskedImgT const& image,
                                     DoubleImgT const& blend_weight,
                                     boost::shared_ptr<CameraModel> const& camera){
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
                                            crop_box, image, blend_weight, camera)));
  }

  Options                    const& m_opt;
  int                               m_col, m_row;
  ImageView<double>         const & m_dem;            // alias
  double                            m_albedo;
  double                          * m_refl_coeffs;
  double                          * m_exposure;
  double                          * m_haze;
  cartography::GeoReference const & m_geo;            // alias
  bool                              m_model_shadows;
  double                            m_camera_position_step_size;
  double                    const & m_max_dem_height; // alias
  double                            m_gridx, m_gridy;
  ReflParams              const & m_refl_params;  // alias
  vw::Vector3               const & m_sunPosition;   // alias
  BBox2i                            m_crop_box;
  MaskedImgT                const & m_image;          // alias
  DoubleImgT                const & m_blend_weight;   // alias
  vw::CamPtr                const & m_camera;         // alias
}; // end class IntensityErrorFloatDemOnly

// A variation of IntensityError where albedo, dem, and model params are fixed.
// TODO(oalexan1): Wipe this as it did not improve anything. 
// TODO(oalexan1): Check if improves memory usage at least. In either case,
// move to SfsCostFun.h.
struct IntensityErrorFixedMost {
  IntensityErrorFixedMost(Options const& opt, int col, int row,
                          ImageView<double> const& dem,
                          double albedo,
                          double * refl_coeffs, 
                          cartography::GeoReference const& geo,
                          bool model_shadows,
                          double camera_position_step_size,
                          double const& max_dem_height, // note: this is an alias
                          double gridx, double gridy,
                          ReflParams const& refl_params,
                          vw::Vector3 const& sunPosition,
                          BBox2i const& crop_box,
                          MaskedImgT const& image,
                          DoubleImgT const& blend_weight,
                          boost::shared_ptr<CameraModel> const& camera):
    m_opt(opt), m_col(col), m_row(row), m_dem(dem),
    m_albedo(albedo), m_refl_coeffs(refl_coeffs), 
    m_geo(geo),
    m_model_shadows(model_shadows),
    m_camera_position_step_size(camera_position_step_size),
    m_max_dem_height(max_dem_height),
    m_gridx(gridx), m_gridy(gridy),
    m_refl_params(refl_params),
    m_sunPosition(sunPosition),
    m_crop_box(crop_box),
    m_image(image), m_blend_weight(blend_weight),
    m_camera(camera) {}

  // See SmoothnessError() for the definitions of bottom, top, etc.
  template <typename F>
  bool operator()(const F* const exposure, const F* const haze, F* residuals) const {

    // For this error we do not use p and q, hence just use a placeholder.
    bool use_pq = false;
    const F * const pq = NULL;
    
    return calc_intensity_residual(m_opt, exposure, haze,
                                   &m_dem(m_col-1, m_row),            // left
                                   &m_dem(m_col, m_row),              // center
                                   &m_dem(m_col+1, m_row),            // right
                                   &m_dem(m_col, m_row+1),            // bottom
                                   &m_dem(m_col, m_row-1),            // top
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
                                   m_refl_params,  // alias
                                   m_sunPosition,  // alias
                                   m_crop_box,  
                                   m_image,  // alias
                                   m_blend_weight,  // alias
                                   m_camera,  // alias
                                   residuals);
  }

  // Factory to hide the construction of the CostFunction object from
  // the client code.
  static ceres::CostFunction* Create(Options const& opt, int col, int row,
                                     ImageView<double> const& dem,
                                     double albedo,
                                     double * refl_coeffs, 
                                     vw::cartography::GeoReference const& geo,
                                     bool model_shadows,
                                     double camera_position_step_size,
                                     double const& max_dem_height, // alias
                                     double gridx, double gridy,
                                     ReflParams const& refl_params,
                                     vw::Vector3 const& sunPosition,
                                     BBox2i const& crop_box,
                                     MaskedImgT const& image,
                                     DoubleImgT const& blend_weight,
                                     boost::shared_ptr<CameraModel> const& camera){
    return (new ceres::NumericDiffCostFunction<IntensityErrorFixedMost,
            ceres::CENTRAL, 1, 1, g_max_num_haze_coeffs>
            (new IntensityErrorFixedMost(opt, col, row, dem, albedo, 
                                         refl_coeffs,  geo,
                                         model_shadows,
                                         camera_position_step_size,
                                         max_dem_height,
                                         gridx, gridy,
                                         refl_params, sunPosition,
                                         crop_box, image, blend_weight, camera)));
  }

  Options                            const& m_opt;
  int                                       m_col, m_row;
  ImageView<double>                 const & m_dem;            // alias
  double                                    m_albedo;
  double                                  * m_refl_coeffs; 
  cartography::GeoReference         const & m_geo;            // alias
  bool                                      m_model_shadows;
  double                                    m_camera_position_step_size;
  double                            const & m_max_dem_height; // alias
  double                                    m_gridx, m_gridy;
  ReflParams                      const & m_refl_params;  // alias
  vw::Vector3                       const & m_sunPosition;   // alias
  BBox2i                                    m_crop_box;
  MaskedImgT                        const & m_image;          // alias
  DoubleImgT                        const & m_blend_weight;   // alias
  boost::shared_ptr<CameraModel>    const & m_camera;         // alias
};

// A variant of the intensity error when we float the partial derivatives
// in x and in y of the dem, which we call p and q.  
// TODO(oalexan1): Move to SfsCostFun.h
// TODO(oalexan1): Validate again if this gives better results as compared to
// usual intensity error.
struct IntensityErrorPQ {
  IntensityErrorPQ(Options const& opt, int col, int row,
                   ImageView<double> const& dem,
                   cartography::GeoReference const& geo,
                   bool model_shadows,
                   double camera_position_step_size,
                   double const& max_dem_height, // note: this is an alias
                   double gridx, double gridy,
                   ReflParams const& refl_params,
                   vw::Vector3 const& sunPosition,
                   BBox2i const& crop_box,
                   MaskedImgT const& image,
                   DoubleImgT const& blend_weight,
                   boost::shared_ptr<CameraModel> const& camera):
    m_opt(opt), m_col(col), m_row(row), m_dem(dem), m_geo(geo),
    m_model_shadows(model_shadows),
    m_camera_position_step_size(camera_position_step_size),
    m_max_dem_height(max_dem_height),
    m_gridx(gridx), m_gridy(gridy),
    m_refl_params(refl_params),
    m_sunPosition(sunPosition),
    m_crop_box(crop_box),
    m_image(image), m_blend_weight(blend_weight),
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
                                   m_camera,          // alias
                                   residuals);
  }

  // Factory to hide the construction of the CostFunction object from
  // the client code.
  static ceres::CostFunction* Create(Options const& opt, int col, int row,
                                     ImageView<double> const& dem,
                                     vw::cartography::GeoReference const& geo,
                                     bool model_shadows,
                                     double camera_position_step_size,
                                     double const& max_dem_height, // alias
                                     double gridx, double gridy,
                                     ReflParams const& refl_params,
                                     vw::Vector3 const& sunPosition,
                                     BBox2i const& crop_box,
                                     MaskedImgT const& image,
                                     DoubleImgT const& blend_weight,
                                     boost::shared_ptr<CameraModel> const& camera){
    return (new ceres::NumericDiffCostFunction<IntensityErrorPQ,
            ceres::CENTRAL, 1, 1, g_max_num_haze_coeffs, 1, 2, 1, g_num_model_coeffs>
            (new IntensityErrorPQ(opt, col, row, dem, geo,
                                  model_shadows,
                                  camera_position_step_size,
                                  max_dem_height,
                                  gridx, gridy,
                                  refl_params, sunPosition,
                                  crop_box, image, blend_weight, camera)));
  }

  Options                            const& m_opt;
  int m_col, m_row;
  ImageView<double>                 const & m_dem;            // alias
  cartography::GeoReference         const & m_geo;            // alias
  bool                                      m_model_shadows;
  double                                    m_camera_position_step_size;
  double                            const & m_max_dem_height; // alias
  double                                    m_gridx, m_gridy;
  ReflParams                      const & m_refl_params;  // alias
  vw::Vector3                       const & m_sunPosition;   // alias
  BBox2i                                    m_crop_box;
  MaskedImgT                        const & m_image;          // alias
  DoubleImgT                        const & m_blend_weight;   // alias
  boost::shared_ptr<CameraModel>    const & m_camera;         // alias
};

// Discrepancy between measured and computed intensity. Assume fixed reflectance,
// as this is only an initial estimate. See calcIntensity() for the formula.
// TODO(oalexan1): Move to SfsCostFun.h
struct IntensityErrorFixedReflectance {
  IntensityErrorFixedReflectance(PixelMask<float> const& intensity,
                                 PixelMask<float> const& reflectance,
                                 int num_haze_coeffs,
                                 double steepness_factor):
    m_intensity(intensity), m_reflectance(reflectance), 
    m_num_haze_coeffs(num_haze_coeffs), m_steepness_factor(steepness_factor) {}

  template <typename F>
  bool operator()(const F* const exposure,
                  const F* const haze,
                  const F* const albedo,
                  F* residuals) const {

    if (!is_valid(m_intensity) || !is_valid(m_reflectance)) {
      residuals[0] = 0;
      return true;
    }
    
    residuals[0] = calcIntensity(albedo[0], m_reflectance, exposure[0],
                                 m_steepness_factor, haze, m_num_haze_coeffs) 
                 - m_intensity;
    
    return true;
  }

  // Factory to hide the construction of the CostFunction object from
  // the client code.
  static ceres::CostFunction* Create(PixelMask<float> const& intensity,
                                     PixelMask<float> const& reflectance,
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
// error = smoothness_weight * ( u_xx^2 + u_xy^2 + u_yx^2 + u_yy^2 )

// We will use finite differences to compute these.
// Consider a grid point and its neighbors, 9 points in all.
//
// bl   = u(c-1, r+1)  bottom = u(c, r+1) br    = u(c+1,r+1)
// left = u(c-1, r  )  center = u(c, r  ) right = u(c+1,r  )
// tl   = u(c-1, r-1)  top    = u(c, r-1) tr    = u(c+1,r-1)
//
// See https://en.wikipedia.org/wiki/Finite_difference
// for the obtained formulas.
// TODO(oalexan1): Move to SfsCostFun.h
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
    residuals[1] = (br[0] + tl[0] - bl[0] - tr[0] )/4.0/m_gridx/m_gridy; // u_xy
    residuals[2] = residuals[1];                                         // u_yx
    residuals[3] = (bottom[0] + top[0] - 2*center[0])/m_gridy/m_gridy;   // u_yy
    
    for (int i = 0; i < 4; i++)
      residuals[i] *= m_smoothness_weight;

    return true;
  }

  // Factory to hide the construction of the CostFunction object from
  // the client code.
  static ceres::CostFunction* Create(double smoothness_weight,
                                     double gridx, double gridy){
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
// TODO(oalexan1): Move to SfsCostFun.h
struct GradientError {
  GradientError(double gradient_weight, double gridx, double gridy):
    m_gradient_weight(gradient_weight), m_gridx(gridx), m_gridy(gridy) {}

  template <typename T>
  bool operator()(const T* const bottom, const T* const left, const T* const center,
                  const T* const right, const T* const top, T* residuals) const {

    // This results in a smoother solution than using centered differences
    residuals[0] = (right[0]  - center[0])/m_gridx; // u_x
    residuals[1] = (center[0] - left[0]  )/m_gridx; // u_x
    residuals[2] = (top[0]    - center[0])/m_gridy; // u_y
    residuals[3] = (center[0] - bottom[0])/m_gridy; // u_y
    
    for (int i = 0; i < 4; i++)
      residuals[i] *= m_gradient_weight;

    return true;
  }

  // Factory to hide the construction of the CostFunction object from
  // the client code.
  static ceres::CostFunction* Create(double gradient_weight,
                                     double gridx, double gridy){
    return (new ceres::NumericDiffCostFunction<GradientError,
            ceres::CENTRAL, 4, 1, 1, 1, 1, 1>
            (new GradientError(gradient_weight, gridx, gridy)));
  }

  double m_gradient_weight, m_gridx, m_gridy;
};

// Try to make the DEM in shadow have positive curvature. The error term is
// (curvature_weight *(terrain_xx + terrain_xy - curvature))^2 in the shadow,
// and not used in lit areas.
// TODO(oalexan1): Move to SfsCostFun.h
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
                                     double gridx, double gridy){
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
                                     double gridx, double gridy){
    return (new ceres::NumericDiffCostFunction<SmoothnessErrorPQ,
            ceres::CENTRAL, 4, 2, 2, 2, 2>
            (new SmoothnessErrorPQ(smoothness_weight_pq, gridx, gridy)));
  }

  double m_smoothness_weight_pq, m_gridx, m_gridy;
};

// The integrability error is the discrepancy between the
// independently optimized gradients p and q, and the partial
// derivatives of the dem, here denoted by u.
// error = integrability_weight * ( (u_x - p)^2 + (u_y - q)^2 )
// See SmoothnessError for the notation below. 
// TODO(oalexan1): Move to SfsCostFun.h
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
                                     double gridx, double gridy){
    return (new ceres::NumericDiffCostFunction<IntegrabilityError,
            ceres::CENTRAL, 2, 1, 1, 1, 1, 2>
            (new IntegrabilityError(integrability_weight, gridx, gridy)));
  }

  double m_integrability_weight, m_gridx, m_gridy;
};


// A cost function that will penalize deviating too much from the original DEM height.
// TODO(oalexan1): Move to SfsCostFun.h
struct HeightChangeError {
  HeightChangeError(double orig_height, double initial_dem_constraint_weight):
    m_orig_height(orig_height), 
    m_initial_dem_constraint_weight(initial_dem_constraint_weight){}

  template <typename T>
  bool operator()(const T* const center, T* residuals) const {
    residuals[0] = (center[0] - m_orig_height)*m_initial_dem_constraint_weight;
    return true;
  }

  // Factory to hide the construction of the CostFunction object from
  // the client code.
  static ceres::CostFunction* Create(double orig_height,
                                     double initial_dem_constraint_weight){
    return (new ceres::NumericDiffCostFunction<HeightChangeError,
            ceres::CENTRAL, 1, 1>
            (new HeightChangeError(orig_height, initial_dem_constraint_weight)));
  }

  double m_orig_height, m_initial_dem_constraint_weight;
};

// A cost function that will penalize deviating too much from the initial albedo.
// TODO(oalexan1): Move to SfsCostFun.h
struct AlbedoChangeError {
  AlbedoChangeError(double initial_albedo, double albedo_constraint_weight):
    m_initial_albedo(initial_albedo), m_albedo_constraint_weight(albedo_constraint_weight){}

  template <typename T>
  bool operator()(const T* const center, T* residuals) const {
    residuals[0] = (center[0] - m_initial_albedo)*m_albedo_constraint_weight;
    return true;
  }

  // Factory to hide the construction of the CostFunction object from
  // the client code.
  static ceres::CostFunction* Create(double initial_albedo,
                                     double albedo_constraint_weight){
    return (new ceres::NumericDiffCostFunction<AlbedoChangeError, ceres::CENTRAL, 1, 1>
            (new AlbedoChangeError(initial_albedo, albedo_constraint_weight)));
  }

  double m_initial_albedo, m_albedo_constraint_weight;
};

void handle_arguments(int argc, char *argv[], Options& opt) {
  po::options_description general_options("");
  general_options.add_options()
    ("input-dem,i",  po::value(&opt.input_dem),
     "The input DEM to refine using SfS.")
    ("image-list", po::value(&opt.image_list)->default_value(""),
     "A file containing the list of images, when they are too many to specify on the command line. Use space or newline as separator. See also --camera-list and --mapprojected-data-list.")
    ("camera-list", po::value(&opt.camera_list)->default_value(""),
     "A file containing the list of cameras, when they are too many to specify on the "
     "command line. If the images have embedded camera information, such as for ISIS, this "
     "file must be empty but must be specified if --image-list is specified.")
    ("output-prefix,o", po::value(&opt.out_prefix),
     "Prefix for output filenames.")
    ("max-iterations,n", po::value(&opt.max_iterations)->default_value(10),
     "Set the maximum number of iterations. Normally 5-10 iterations is enough, even when convergence is not reached, as the solution usually improves quickly at first and only very fine refinements happen later.")
    ("reflectance-type", po::value(&opt.reflectance_type)->default_value(1),
     "Reflectance type: 0 = Lambertian, 1 = Lunar-Lambert, 2 = Hapke, 3 = Experimental extension of Lunar-Lambert, 4 = Charon model (a variation of Lunar-Lambert).")
    ("smoothness-weight", po::value(&opt.smoothness_weight)->default_value(0.04),
     "The weight given to the cost function term which consists of sums of squares of second-order derivatives. A larger value will result in a smoother solution with fewer artifacts. See also --gradient-weight.")
    ("initial-dem-constraint-weight", po::value(&opt.initial_dem_constraint_weight)->default_value(0),
     "A larger value will try harder to keep the SfS-optimized DEM closer to the initial guess DEM. A value between 0.0001 and 0.001 may work, unless your initial DEM is very unreliable.")
    ("bundle-adjust-prefix", po::value(&opt.bundle_adjust_prefix),
     "Use the camera adjustments obtained by previously running bundle_adjust with this output prefix.")
    ("input-albedo",  po::value(&opt.input_albedo),
     "The input albedo image, if known. Must have same dimensions as the input DEM. "
     "Otherwise it is initialized to 1. Can be refined with --float-albedo.")
    ("float-albedo",   po::bool_switch(&opt.float_albedo)->default_value(false)->implicit_value(true),
     "Float the albedo for each pixel. Will give incorrect results if only one image is present. The albedo is normalized, its nominal value is 1.")
    ("float-exposure",   po::bool_switch(&opt.float_exposure)->default_value(false)->implicit_value(true),
     "Float the exposure for each image. Will give incorrect results if only one image is present. It usually gives marginal results.")
    ("model-shadows",   po::bool_switch(&opt.model_shadows)->default_value(false)->implicit_value(true),
     "Model the fact that some points on the DEM are in the shadow (occluded from the Sun).")
    ("compute-exposures-only",   po::bool_switch(&opt.compute_exposures_only)->default_value(false)->implicit_value(true),
     "Quit after saving the exposures. This should be done once for a big DEM, before using these for small sub-clips without recomputing them.")
    ("save-computed-intensity-only",   po::bool_switch(&opt.save_computed_intensity_only)->default_value(false)->implicit_value(true),
     "Save the computed (simulated) image intensities for given DEM, images, cameras, and "
     "reflectance model, without refining the DEM. The measured intensities will be saved "
     "as well, for comparison. The image exposures will be computed along the way unless "
     "specified via --image-exposures-prefix, and will be saved in either case to <output "
     "prefix>-exposures.txt. Same for haze, if applicable.")
     ("estimate-exposure-haze-albedo", po::bool_switch(&opt.estimate_exposure_haze_albedo)->default_value(false)->implicit_value(true),
      "Estimate the exposure, haze, and albedo for each image. This is experimental.")
    ("estimate-slope-errors",   po::bool_switch(&opt.estimate_slope_errors)->default_value(false)->implicit_value(true),
     "Estimate the error for each slope (normal to the DEM). This is experimental.")
    ("estimate-height-errors",   po::bool_switch(&opt.estimate_height_errors)->default_value(false)->implicit_value(true),
     "Estimate the SfS DEM height uncertainty by finding the height perturbation (in meters) at each grid point which will make at least one of the simulated images at that point change by more than twice the discrepancy between the unperturbed simulated image and the measured image. The SfS DEM must be provided via the -i option. The number of iterations, blending parameters (--blending-dist, etc.), and smoothness weight are ignored. Results are not computed at image pixels in shadow. This produces <output prefix>-height-error.tif. No SfS DEM is computed.")
    ("height-error-params", po::value(&opt.height_error_params)->default_value(Vector2(5.0,1000.0), "5.0 1000"),
     "Specify the largest height deviation to examine (in meters), and how many samples to use from 0 to that height.")
    ("sun-positions", po::value(&opt.sun_positions_list)->default_value(""),
     "A file having on each line an image name and three values in double precision "
     "specifying the Sun position in meters in ECEF coordinates (origin is planet center). "
     "Use a space as separator. If not provided, these will be read from the camera files "
     "for ISIS and CSM models.")
    ("sun-angles", po::value(&opt.sun_angles_list)->default_value(""),
     "A file having on each line an image name and two values in double precision "
     "specifying the Sun azimuth and elevation in degrees, relative to the center point of "
     "the input DEM. Use a space as separator. This is an alternative to --sun-positions.")
    ("shadow-thresholds", po::value(&opt.shadow_thresholds)->default_value(""),
     "Optional shadow thresholds for the input images (a list of real values in quotes, one per image).")
    ("shadow-threshold", po::value(&opt.shadow_threshold)->default_value(-1),
     "A shadow threshold to apply to all images instead of using individual thresholds. (Must be positive.)")
    ("custom-shadow-threshold-list", po::value(&opt.custom_shadow_threshold_list)->default_value(""),
     "A list having one image and one shadow threshold per line. For the images specified here, override the shadow threshold supplied by other means with this value.")
    ("max-valid-image-vals", po::value(&opt.max_valid_image_vals)->default_value(""),
     "Optional values for the largest valid image value in each image (a list of real values in quotes, one per image).")
    ("robust-threshold", po::value(&opt.robust_threshold)->default_value(-1.0),
     "If positive, set the threshold for the robust measured-to-simulated intensity difference (using the Cauchy loss). Any difference much larger than this will be penalized. A good value may be 5% to 25% of the average image value or the same fraction of the computed image exposure values.")
    ("albedo-constraint-weight", po::value(&opt.albedo_constraint_weight)->default_value(0),
     "If floating the albedo, a larger value will try harder to keep the optimized albedo close to the nominal value of 1.")
    ("albedo-robust-threshold", po::value(&opt.albedo_robust_threshold)->default_value(0),
     "If floating the albedo and this threshold is positive, apply a Cauchy loss with this threshold to the product of the albedo difference and the albedo constraint weight.")
    ("unreliable-intensity-threshold", po::value(&opt.unreliable_intensity_threshold)->default_value(0.0),
     "Intensities lower than this will be considered unreliable and given less weight.")
    ("skip-images", po::value(&opt.skip_images_str)->default_value(""), "Skip images with these indices (indices start from 0).")
    ("save-dem-with-nodata",   po::bool_switch(&opt.save_dem_with_nodata)->default_value(false)->implicit_value(true),
     "Save a copy of the DEM while using a no-data value at a DEM grid point where all images show shadows. To be used if shadow thresholds are set.")
    ("use-approx-camera-models",   po::bool_switch(&opt.use_approx_camera_models)->default_value(false)->implicit_value(true),
     "Use approximate camera models for speed. Only with ISIS .cub cameras.")
    ("crop-input-images",   po::bool_switch(&opt.crop_input_images)->default_value(false)->implicit_value(true),
     "Crop the images to a region that was computed to be large enough, and keep them fully in memory, for speed.")
    ("blending-dist", po::value(&opt.blending_dist)->default_value(0),
     "Give less weight to image pixels close to no-data or boundary values. Enabled only when crop-input-images is true, for performance reasons. Blend over this many pixels.")
    ("blending-power", po::value(&opt.blending_power)->default_value(2.0),
     "A higher value will result in smoother blending.")
    ("min-blend-size", po::value(&opt.min_blend_size)->default_value(0),
     "Do not apply blending in shadowed areas of dimensions less than this.")
    ("allow-borderline-data",   po::bool_switch(&opt.allow_borderline_data)->default_value(false)->implicit_value(true),
     "At the border of the region where there are no lit pixels in any images, do not let the blending weights decay to 0. This noticeably improves the level of detail. The sfs_blend tool may need to be used to further tune this region.")
    ("steepness-factor", po::value(&opt.steepness_factor)->default_value(1.0),
     "Try to make the terrain steeper by this factor. This is not recommended in regular use.")
    ("curvature-in-shadow", po::value(&opt.curvature_in_shadow)->default_value(0.0),
     "Attempt to make the curvature of the DEM (the Laplacian) at points in shadow in all images equal to this value, which should make the DEM curve down.")
    ("curvature-in-shadow-weight", po::value(&opt.curvature_in_shadow_weight)->default_value(0.0),
     "The weight to give to the curvature in shadow constraint.")
    ("lit-curvature-dist", po::value(&opt.lit_curvature_dist)->default_value(0.0),
     "If using a curvature in shadow, start phasing it in this far from the shadow boundary in the lit region (in units of pixels).")
    ("shadow-curvature-dist", po::value(&opt.shadow_curvature_dist)->default_value(0.0),
     "If using a curvature in shadow, have it fully phased in this far from shadow boundary in the shadow region (in units of pixels).")
    ("image-exposures-prefix", po::value(&opt.image_exposures_prefix)->default_value(""),
     "Use this prefix to optionally read initial exposures (filename is <prefix>-exposures.txt).")
    ("model-coeffs-prefix", po::value(&opt.model_coeffs_prefix)->default_value(""),
     "Use this prefix to optionally read model coefficients from a file (filename is <prefix>-model_coeffs.txt).")
    ("model-coeffs", po::value(&opt.model_coeffs)->default_value(""),
     "Use the reflectance model coefficients specified as a list of numbers in quotes. Lunar-Lambertian: O, A, B, C, e.g., '1 -0.019 0.000242 -0.00000146'. Hapke: omega, b, c, B0, h, e.g., '0.68 0.17 0.62 0.52 0.52'. Charon: A, f(alpha), e.g., '0.7 0.63'.")
    ("num-haze-coeffs", po::value(&opt.num_haze_coeffs)->default_value(0),
     "Set this to 1 to model the problem as image = exposure * albedo * reflectance + "
     "haze, where haze is a single value for each image.")
    ("float-haze", 
     po::bool_switch(&opt.float_haze)->default_value(false)->implicit_value(true),
     "If specified, float the haze coefficients as part of the optimization, if "
     "--num-haze-coeffs is 1.")
    ("haze-prefix", po::value(&opt.image_haze_prefix)->default_value(""),
     "Use this prefix to read initial haze values (filename is <haze-prefix>-haze.txt). The file format is the same as what the tool writes itself, when triggered by the earlier options. If haze is modeled, it will be initially set to 0 unless read from such a file, and will be floated or not depending on whether --float-haze is on. The final haze values will be saved to <output prefix>-haze.txt.")
    ("init-dem-height", po::value(&opt.init_dem_height)->default_value(std::numeric_limits<double>::quiet_NaN()),
     "Use this value for initial DEM heights (measured in meters, relative to the datum). "
     "An input DEM still needs to be provided for georeference information.")
    ("crop-win", po::value(&opt.crop_win)->default_value(BBox2i(0, 0, 0, 0), "xoff yoff xsize ysize"),
     "Crop the input DEM to this region before continuing.")
    ("nodata-value", po::value(&opt.nodata_val)->default_value(std::numeric_limits<double>::quiet_NaN()),
     "Use this as the DEM no-data value, over-riding what is in the initial guess DEM.")
    ("fix-dem",   po::bool_switch(&opt.fix_dem)->default_value(false)->implicit_value(true),
     "Do not float the DEM at all. Useful when floating the model params.")
    ("read-exposures", po::bool_switch(&opt.read_exposures)->default_value(false)->implicit_value(true),
     "If specified, read the image exposures with the current output prefix. Useful with a "
     "repeat invocation.")
    ("read-haze", po::bool_switch(&opt.read_haze)->default_value(false)->implicit_value(true),
     "If specified, read the haze values with the current output prefix.")
    ("read-albedo", po::bool_switch(&opt.read_albedo)->default_value(false)->implicit_value(true),
     "If specified, read the computed albedo with the current output prefix.")
    ("float-reflectance-model",   po::bool_switch(&opt.float_reflectance_model)->default_value(false)->implicit_value(true),
     "Allow the coefficients of the reflectance model to float (not recommended).")
    ("integrability-constraint-weight", po::value(&opt.integrability_weight)->default_value(0.0),
     "Use the integrability constraint from Horn 1990 with this value of its weight.")
    ("smoothness-weight-pq", po::value(&opt.smoothness_weight_pq)->default_value(0.00),
     "Smoothness weight for p and q, when the integrability constraint "
     "is used. A larger value will result in a smoother solution "
     "(experimental).")
    ("query",   po::bool_switch(&opt.query)->default_value(false)->implicit_value(true),
     "Print some info and exit. Invoked from parallel_sfs.")
    ("session-type,t",   po::value(&opt.stereo_session)->default_value(""),
     "Select the stereo session type to use for processing. Usually the program can select this automatically by the file extension, except for xml cameras. See the doc for options.")
    ("gradient-weight", po::value(&opt.gradient_weight)->default_value(0.0),
     "The weight given to the cost function term which consists of sums "
     "of squares of first-order derivatives. A larger value will result "
     "in shallower slopes but less noise. This can be used in conjunction with "
     "--smoothness-weight. It is suggested to experiment with this "
     "with a value of 0.0001 - 0.01, while reducing the "
     "smoothness weight to a very small value.")
    ("save-sparingly",   po::bool_switch(&opt.save_sparingly)->default_value(false)->implicit_value(true),
     "Avoid saving any results except the adjustments and the DEM, as that's a lot of files.")
    ("camera-position-step-size", po::value(&opt.camera_position_step_size)->default_value(1.0),
     "Larger step size will result in more aggressiveness in varying the camera position if it is being floated (which may result in a better solution or in divergence).");

  general_options.add( vw::GdalWriteOptionsDescription(opt) );

  po::options_description positional("");
  positional.add_options()
    ("input-images", po::value(&opt.input_images));

  po::positional_options_description positional_desc;
  positional_desc.add("input-images", -1);

  std::string usage("-i <input DEM> -n <max iterations> -o <output prefix> <images> [other options]");
  bool allow_unregistered = false;
  std::vector<std::string> unregistered;
  po::variables_map vm =
    asp::check_command_line(argc, argv, opt, general_options, general_options,
                            positional, positional_desc, usage,
                            allow_unregistered, unregistered);


  // Sanity checks. Put this early, before separating images from cameras, as that
  // function can print a message not reflecting the true issue of missing the DEM.
  if (opt.input_dem.empty())
    vw_throw(ArgumentErr() << "Missing the input DEM.\n");

  // Separate the cameras from the images
  std::vector<std::string> inputs = opt.input_images;

  if (!opt.image_list.empty()) {
    // Read the images and cameras and put them in 'inputs' to be parsed later
    if (opt.camera_list.empty())
      vw_throw(ArgumentErr()
               << "The option --image-list must be invoked together with --camera-list.\n");
    if (!inputs.empty())
      vw_throw(ArgumentErr() << "The option --image-list was specified, but also "
               << "images or cameras on the command line.\n");
    asp::read_list(opt.image_list, inputs);
    std::vector<std::string> tmp;
    asp::read_list(opt.camera_list, tmp);
    for (size_t it = 0; it < tmp.size(); it++) 
      inputs.push_back(tmp[it]);
  }
  
  bool ensure_equal_sizes = true;
  asp::separate_images_from_cameras(inputs,
                                    opt.input_images, opt.input_cameras, // outputs
                                    ensure_equal_sizes); 
  
  if (opt.out_prefix.empty())
    vw_throw( ArgumentErr() << "Missing output prefix.\n"
              << usage << general_options );

  if (opt.max_iterations < 0)
    vw_throw( ArgumentErr() << "The number of iterations must be non-negative.\n"
              << usage << general_options );

  if (opt.input_images.empty())
    vw_throw( ArgumentErr() << "Missing input images.\n"
              << usage << general_options );

  if (opt.smoothness_weight < 0) 
    vw_throw(ArgumentErr() << "Expecting a non-negative smoothness weight.\n");

  if (opt.gradient_weight < 0) 
    vw_throw(ArgumentErr() << "Expecting a non-negative gradient weight.\n");

  if (opt.integrability_weight < 0) 
    vw_throw(ArgumentErr() << "Expecting a non-negative integrability weight.\n");

  if (opt.float_haze && opt.num_haze_coeffs == 0) 
    vw_throw(ArgumentErr() << "Haze cannot be floated unless there is at least one haze coefficient.\n");
  if (opt.image_haze_prefix != "" && opt.num_haze_coeffs == 0)
    vw_throw(ArgumentErr() << "Haze cannot be read unless there is at least one haze coefficient.\n");
  
  // There can be 0 or 1 haze coefficients. The modeling of more than one haze
  // coefficient needs to be looked into.
  if (opt.num_haze_coeffs < 0 || opt.num_haze_coeffs > g_max_num_haze_coeffs)
    vw_throw(ArgumentErr() << "Expecting up to " << g_max_num_haze_coeffs
             << " haze coefficients.\n");
    
  // Curvature in shadow params
  if (opt.curvature_in_shadow < 0.0 || opt.curvature_in_shadow_weight < 0.0) 
    vw_throw(ArgumentErr() << "Cannot have negative curvature in shadow or its weight.\n");
  if (opt.lit_curvature_dist < 0.0 || opt.shadow_curvature_dist < 0.0) 
    vw_throw(ArgumentErr() << "Cannot have negative curvature distances.\n");    
  if (opt.curvature_in_shadow > 0.0 &&
      opt.shadow_curvature_dist + opt.lit_curvature_dist <= 0.0)
    vw_throw(ArgumentErr() << "When modeling curvature in shadow, expecting a "
             << "positive value of shadow-curvature-dist or list-curvature-dist.\n");    

  if (opt.steepness_factor <= 0.0) 
    vw_throw(ArgumentErr() << "The steepness factor must be positive.\n");    

  if (opt.compute_exposures_only || opt.estimate_exposure_haze_albedo) {
    if (opt.use_approx_camera_models || opt.crop_input_images) {
      vw_out(WarningMessage) << "When computing exposures only, not using approximate "
                             << "camera models or cropping input images.\n";
      opt.use_approx_camera_models = false;
      opt.crop_input_images = false;
      opt.blending_dist = 0;
      opt.allow_borderline_data = false;
    }

    if (!opt.crop_win.empty()) {
      vw_throw(ArgumentErr() << "When computing exposures only, cannot crop the "
               << "input DEM as that will give wrong results. Use the full DEM.\n");
    }
  }
  
  if (opt.blending_dist > 0 && !opt.crop_input_images) 
    vw_throw(ArgumentErr() 
             << "A blending distance is only supported with --crop-input-images.\n");
  
  if (opt.allow_borderline_data && !opt.crop_input_images)
    vw_throw(ArgumentErr() << "Option --allow-borderline-data needs option "
             << "--crop-input-images.\n");

  if (opt.allow_borderline_data && opt.blending_dist <= 0) 
    vw::vw_throw(vw::ArgumentErr()
                 << "Option --allow-borderline-data needs a positive --blending-dist.\n");

  // Create the output directory
  vw::create_out_dir(opt.out_prefix);

  // Turn on logging to file
  asp::log_to_file(argc, argv, "", opt.out_prefix);

  // Parse the shadow thresholds
  std::istringstream ist(opt.shadow_thresholds);
  opt.shadow_threshold_vec.clear();
  float val;
  while (ist >> val)
    opt.shadow_threshold_vec.push_back(val);
  if (!opt.shadow_threshold_vec.empty() &&
      opt.shadow_threshold_vec.size() != opt.input_images.size())
    vw_throw(ArgumentErr()
             << "If specified, there must be as many shadow thresholds as images.\n");

  // See if to use opt.shadow_threshold.
  if (opt.shadow_threshold > 0) {
    if (!opt.shadow_threshold_vec.empty())
      vw_throw(ArgumentErr()
               << "Cannot specify both --shadow-threshold and --shadow-thresholds.\n");
    while (opt.shadow_threshold_vec.size() < opt.input_images.size())
      opt.shadow_threshold_vec.push_back(opt.shadow_threshold);
  }

  // Default thresholds are the smallest float.
  // Maybe it should be 0?
  if (opt.shadow_threshold_vec.empty()) {
    for (size_t i = 0; i < opt.input_images.size(); i++) {
      opt.shadow_threshold_vec.push_back(-std::numeric_limits<float>::max());
    }
  }

  // Override some shadow thresholds with custom versions
  if (!opt.custom_shadow_threshold_list.empty()) {
    std::map<std::string, double> custom_thresh;
    std::ifstream ifs(opt.custom_shadow_threshold_list);
    std::string image;
    double val;
    while (ifs >> image >> val) {
      custom_thresh[image] = val;
    }
    
    if (custom_thresh.empty()) 
      vw_throw(ArgumentErr() << "Could not read any data from: "
               << opt.custom_shadow_threshold_list << "\n");
    
    for (size_t it = 0; it < opt.input_images.size(); it++) {
      auto key = custom_thresh.find(opt.input_images[it]);
      if (key != custom_thresh.end()) {
        vw_out() << "Over-riding the shadow threshold for " << opt.input_images[it]
                 << " with: " << key->second << std::endl;
        opt.shadow_threshold_vec[it] = key->second;
      }
    }
  }
  
  // Parse max valid image vals
  std::istringstream ism(opt.max_valid_image_vals);
  opt.max_valid_image_vals_vec.clear();
  while (ism >> val)
    opt.max_valid_image_vals_vec.push_back(val);
  if (!opt.max_valid_image_vals_vec.empty() &&
      opt.max_valid_image_vals_vec.size() != opt.input_images.size())
    vw_throw(ArgumentErr()
             << "If specified, there must be as many max valid image vals as images.\n");

  if (opt.max_valid_image_vals_vec.empty()) {
    for (size_t i = 0; i < opt.input_images.size(); i++) {
      opt.max_valid_image_vals_vec.push_back(std::numeric_limits<float>::max());
    }
  }

  // --read-exposures defines where exposures are read from. Put this as a warning,
  // otherwise parallel_sfs won't run.
  if (opt.image_exposures_prefix != "" && opt.read_exposures && 
      opt.image_exposures_prefix != opt.out_prefix)
      vw::vw_out(vw::WarningMessage) << "Reading exposures with prefix: " 
        << opt.out_prefix << " rather than: " << opt.image_exposures_prefix << ".\n";
  if (opt.read_exposures)
    opt.image_exposures_prefix = opt.out_prefix;
  
  // Same for haze. This one parallel_sfs does not precompute. This may change.
  if (opt.image_haze_prefix != "" && opt.read_haze)
    vw_throw(ArgumentErr() 
             << "Cannot specify both --haze-prefix and --read-haze.\n");
  // If have opt.read_haze, use the current prefix
  if (opt.read_haze) 
    opt.image_haze_prefix = opt.out_prefix;
  
  // Same for albedo
  if (opt.input_albedo != "" && opt.read_albedo)
    vw_throw(ArgumentErr() 
             << "Cannot specify both --input-albedo and --read-albedo.\n");
  // If have opt.read_albedo, use the current prefix albedo
  if (opt.read_albedo)
    opt.input_albedo = opt.out_prefix + "-comp-albedo-final.tif";
          
  // Initial image exposures, if provided. First read them in a map,
  // as perhaps the initial exposures were created using more images
  // than what we have here. 
  // TODO(oalexan1): This must be a function.
  std::string exposure_file = asp::exposureFileName(opt.image_exposures_prefix);
  opt.image_exposures_vec.clear();
  std::map<std::string, double> img2exp;
  std::string name;
  double dval;
  std::ifstream ise(exposure_file.c_str());
  int exp_count = 0;
  while (ise >> name >> dval) {
    img2exp[name] = dval;
    exp_count++;
  }
  ise.close();
  if (exp_count > 0) {
    vw_out() << "Reading exposures from: " << exposure_file << std::endl;
    for (size_t i = 0; i < opt.input_images.size(); i++) {
      std::string img = opt.input_images[i];
      std::map<std::string, double>::iterator it = img2exp.find(img);
      if (it == img2exp.end()) {
        vw_throw(ArgumentErr()
                 << "Could not find the exposure for image: " << img << ".\n");
      }
      double exp_val = it->second;
      // vw_out() << "Exposure for " << img << ": " << exp_val << std::endl;
      opt.image_exposures_vec.push_back(exp_val);
    }
  }
  if (opt.image_exposures_prefix != "" && exp_count == 0) 
    vw_throw(ArgumentErr()
             << "Could not find the exposures file: " << exposure_file << ".\n");
  
  if (opt.steepness_factor != 1.0) 
    vw_out() << "Making the terrain artificially steeper by factor: " << opt.steepness_factor
             << ".\n";
  
  // Initial image haze, if provided. First read them in a map,
  // as perhaps the initial haze were created using more images
  // than what we have here. 
  // TODO(oalexan1): This must be a function.
  if (opt.num_haze_coeffs > 0) {
    std::string haze_file = asp::hazeFileName(opt.image_haze_prefix);
    opt.image_haze_vec.clear();
    std::map<std::string, std::vector<double>> img2haze;
    std::ifstream ish(haze_file.c_str());
    int haze_count = 0;
    while(1) {
      std::string line;
      std::getline(ish, line);
      std::istringstream hstream(line);
      if (! (hstream >> name) ) break;
      std::vector<double> haze_vec;
      while (hstream >> dval)
        haze_vec.push_back(dval);
      if (haze_vec.empty()) break;
      haze_count++;

      // Pad the haze vec
      while (haze_vec.size() < g_max_num_haze_coeffs) haze_vec.push_back(0);
      
      img2haze[name] = haze_vec;

      // All haze coefficients beyond the first num_haze_coeffs must
      // be zero, as that means we are reading results written with
      // different number of haze coeffs.
      for (size_t hiter = opt.num_haze_coeffs; hiter < g_max_num_haze_coeffs; hiter++) {
        if (haze_vec[hiter] != 0) 
          vw_throw(ArgumentErr() 
                   << "Found unexpected non-zero haze coefficient: " << haze_vec[hiter] << ".\n");
      }
    }
    ish.close();
    if (opt.image_haze_prefix != "" && haze_count == 0) 
      vw_throw(ArgumentErr()
               << "Could not find the haze file: " << haze_file << ".\n");
    
    if (haze_count > 0) {
      vw_out() << "Reading haze file: " << haze_file << std::endl;
      for (size_t i = 0; i < opt.input_images.size(); i++) {
        std::string img = opt.input_images[i];
        std::map< std::string, std::vector<double>>::iterator it = img2haze.find(img);
        if (it == img2haze.end()) {
          vw_throw(ArgumentErr()
                   << "Could not find the haze for image: " << img << ".\n");
        }
        std::vector<double> haze_vec = it->second;
        // vw_out() << "Haze for " << img << ":";
        //for (size_t hiter = 0; hiter < haze_vec.size(); hiter++) 
        //  vw_out() << " " << haze_vec[hiter];
        //vw_out() << "\n";
        opt.image_haze_vec.push_back(haze_vec);
      }
    }
  }
  
  // Initial model coeffs, if passed on the command line
  if (opt.model_coeffs != "") {
    vw_out() << "Parsing model coefficients: " << opt.model_coeffs << std::endl;
    std::istringstream is(opt.model_coeffs);
    double val;
    while (is >> val)
      opt.model_coeffs_vec.push_back(val);
  }

  // Initial model coefficients, if provided in the file
  if (opt.model_coeffs_prefix != "") {
    std::string model_coeffs_file = asp::modelCoeffsFileName(opt.model_coeffs_prefix);
    vw_out() << "Reading model coefficients from file: " << model_coeffs_file << std::endl;
    std::ifstream ism(model_coeffs_file.c_str());
    opt.model_coeffs_vec.clear();
    while (ism >> dval)
      opt.model_coeffs_vec.push_back(dval);
    ism.close();
    if ( opt.model_coeffs_vec.empty()) {
      vw_throw(ArgumentErr() << "Could not read model coefficients from: " << model_coeffs_file << ".\n");
    }
  }

  if (!opt.model_coeffs_vec.empty()) {
    // Pad with zeros if needed, as the Lunar Lambertian has 4 params, while Hapke has 5 of them.
    // the Charon one has 2.
    while (opt.model_coeffs_vec.size() < g_num_model_coeffs)
      opt.model_coeffs_vec.push_back(0);

    if (opt.model_coeffs_vec.size() != g_num_model_coeffs)
      vw_throw(ArgumentErr()
               << "If specified, there must be " << g_num_model_coeffs << " coefficients.\n");
  }

  // Sanity check
  if (opt.camera_position_step_size <= 0) {
    vw_throw(ArgumentErr() << "Expecting a positive value for camera-position-step-size.\n");
  }

  // Need this to be able to load adjusted camera models. That will happen
  // in the stereo session.
  asp::stereo_settings().bundle_adjust_prefix = opt.bundle_adjust_prefix;

  if (opt.input_images.size() <= 1 && opt.float_albedo && 
      opt.initial_dem_constraint_weight <= 0 && opt.albedo_constraint_weight <= 0.0)
    vw_throw(ArgumentErr()
             << "Floating the albedo is ill-posed for just one image without "
             << "the initial DEM constraint or the albedo constraint.\n");

  if (opt.input_images.size() <= 1 && opt.float_exposure && 
      opt.initial_dem_constraint_weight <= 0)
    vw_throw(ArgumentErr()
             << "Floating the exposure is ill-posed for just one image.\n");

  // Start with given images to skip. Later, for each dem clip, we may
  // skip more, if those images do not overlap with the clip.
  opt.skip_images.clear();
  if (opt.skip_images_str != "") {
    std::istringstream is(opt.skip_images_str);
    int val;
    while( is >> val)
      opt.skip_images.insert(val);
  }

  // estimate height errors and integrability constraint are mutually exclusive
  if (opt.estimate_height_errors && opt.integrability_weight > 0) 
    vw_throw(ArgumentErr() 
             << "Cannot estimate height errors when using the integrability constraint.\n");

  if (opt.estimate_slope_errors && opt.estimate_height_errors) 
    vw_throw(ArgumentErr() 
             << "Cannot estimate both slope and height error at the same time.");

  if (opt.estimate_height_errors && opt.model_shadows) 
    vw_throw( ArgumentErr() << "Cannot estimate height error when modeling shadows.");
  
  if (opt.save_computed_intensity_only || opt.estimate_slope_errors || 
      opt.estimate_height_errors) {
    
    // No iterations    
    opt.max_iterations = 0;

    // Need the exact cameras as they span the full DEM
    if (opt.use_approx_camera_models || opt.crop_input_images) {
      opt.use_approx_camera_models = false;
      opt.crop_input_images = false;
    }
    
    if (opt.num_haze_coeffs > 0 && opt.image_haze_vec.empty())
      vw_throw( ArgumentErr()
                << "Expecting the haze to be computed and passed in.\n" );
  }
  
  // Cannot have both sun positions and sun angles
  if (opt.sun_positions_list.size() > 0 && opt.sun_angles_list.size() > 0) 
    vw_throw(ArgumentErr() << "Cannot specify both sun positions and sun angles.\n");
} 

// Run sfs
void run_sfs(// Fixed quantities
             int                                num_iterations, 
             double                             gridx,
             double                             gridy,
             Options                          & opt,
             GeoReference               const & geo,
             double                             smoothness_weight,
             double                             max_dem_height,
             double                             dem_nodata_val,
             float                              img_nodata_val,
             std::vector<BBox2i>        const & crop_boxes,
             std::vector<MaskedImgT>    const & masked_images,
             std::vector<DoubleImgT>    const & blend_weights,
             ReflParams               const & refl_params,
             std::vector<vw::Vector3>   const & sunPosition,
             ImageView<double>          const & orig_dem,
             ImageView<int>             const & lit_image_mask,
             ImageView<double>          const & curvature_in_shadow_weight,
             // Variable quantities
             ImageView<double>                & dem,
             ImageView<double>                & albedo,
             std::vector<vw::CamPtr>          & cameras,
             std::vector<double>              & exposures,
             std::vector<std::vector<double>> & haze,
             std::vector<double>              & refl_coeffs) {

  int num_images = opt.input_images.size();
  ceres::Problem problem;
  
  // Sample large DEMs. Keep about 200 row and column samples.
  int num_samples = 200;
  int sample_col_rate = 0, sample_row_rate = 0;
  asp::calcSampleRates(dem, num_samples, sample_col_rate, sample_row_rate);
  
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
  ImageView<Vector2> pq;  
  if (opt.integrability_weight > 0) {
    pq.set_size(dem.cols(), dem.rows());

    for (int col = 1; col < dem.cols()-1; col++) {
      for (int row = 1; row < dem.rows()-1; row++) {
        // Note that the top value is dem(col, row-1) and the 
        //            bottom value is dem(col, row+1).
        pq(col, row)[0] // same as (right - left)/(2*gridx)
          = (dem(col+1, row) - dem(col-1, row))/(2*gridx);
        pq(col, row)[1] // same as (top - bottom)/(2*gridy)
          = (dem(col, row-1) - dem(col, row+1))/(2*gridy);
      }
    }
  }

  // Use a simpler cost function if only the DEM is floated. Not sure how much
  // of speedup that gives.
  bool float_dem_only = true;
  if (opt.float_albedo || opt.float_exposure || opt.fix_dem || opt.float_reflectance_model ||
      opt.float_haze || opt.integrability_weight > 0) {
    float_dem_only = false;
  }
  
  // To avoid a crash in Ceres when a param is fixed but not set
  std::set<int> use_albedo;
  
  // Add a residual block for every grid point not at the boundary
  int bd = 1;
  for (int col = bd; col < dem.cols()-bd; col++) {
    for (int row = bd; row < dem.rows()-bd; row++) {
      
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
      
      if (col > 0 && col < dem.cols() -1 && row > 0 && row < dem.rows() -1) {
        
        // Smoothness penalty. We always add this, even if the weight is 0,
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
        }
      }
      
    } // end row iter
  } // end col iter
  
  // DEM at the boundary must be fixed.
  for (int col = 0; col < dem.cols(); col++) {
    for (int row = 0; row < dem.rows(); row++) {
      if (col == 0 || col == dem.cols() - 1 ||
          row == 0 || row == dem.rows() - 1 ) {
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
      vw_out() << "No DEM or albedo constraint is used, and there is at most one "
                << "usable image. Fixing the albedo.\n";
      opt.float_albedo = false;
    }

    // If there's just one image, don't float the exposure, as the
    // problem is under-determined. If we float the albedo, we will
    // implicitly float the exposure, hence keep the exposure itself
    // fixed.
    if (opt.float_exposure) {
      vw_out() << "No DEM constraint is used, and there is at most one "
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
    
    if (!opt.float_exposure){
      for (int image_iter = 0; image_iter < num_images; image_iter++) {
        if (use_image[image_iter]) problem.SetParameterBlockConstant(&exposures[image_iter]);
      }
    }
    if (!opt.float_haze) {
      for (int image_iter = 0; image_iter < num_images; image_iter++) {
        if (use_image[image_iter]) problem.SetParameterBlockConstant(&haze[image_iter][0]);
      }
    }
    
    // If to float the reflectance model coefficients
    if (!opt.float_reflectance_model && num_used > 0)
      problem.SetParameterBlockConstant(&refl_coeffs[0]);
  }
  
  if (opt.num_threads > 1 && opt.stereo_session == "isis"  && !opt.use_approx_camera_models) {
    vw_out() << "Using exact ISIS camera models. Can run with only a single thread.\n";
    opt.num_threads = 1;
  }

  vw_out() << "Using: " << opt.num_threads << " thread(s).\n";

  ceres::Solver::Options options;
  options.gradient_tolerance = 1e-16;
  options.function_tolerance = 1e-16;
  options.max_num_iterations = num_iterations;
  options.minimizer_progress_to_stdout = 1;
  options.num_threads = opt.num_threads;
  options.linear_solver_type = ceres::SPARSE_SCHUR;

  SfsCallback callback(opt, dem, pq, albedo, geo, refl_params, sunPosition, 
                       crop_boxes, masked_images, blend_weights, cameras, 
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
  
  vw_out() << summary.FullReport() << "\n" << std::endl;
}

// TODO(oalexan1): Move this to SfsReflectance.cc
void setupReflectance(ReflParams & refl_params, Options & opt) {
  if (opt.reflectance_type == 0)
    refl_params.reflectanceType = LAMBERT;
  else if (opt.reflectance_type == 1)
    refl_params.reflectanceType = LUNAR_LAMBERT;
  else if (opt.reflectance_type == 2)
    refl_params.reflectanceType = HAPKE;
  else if (opt.reflectance_type == 3)
    refl_params.reflectanceType = ARBITRARY_MODEL;
  else if (opt.reflectance_type == 4)
    refl_params.reflectanceType = CHARON;
  else
    vw_throw( ArgumentErr() << "Expecting Lambertian or Lunar-Lambertian reflectance." );
  refl_params.phaseCoeffC1 = 0; 
  refl_params.phaseCoeffC2 = 0;
  
  // Default model coefficients, unless they were read already
  if (opt.model_coeffs_vec.empty()) {
    opt.model_coeffs_vec.resize(g_num_model_coeffs);
    if (refl_params.reflectanceType == LUNAR_LAMBERT ||
        refl_params.reflectanceType == ARBITRARY_MODEL ) {
      // Lunar lambertian or its crazy experimental generalization
      opt.model_coeffs_vec.resize(g_num_model_coeffs);
      opt.model_coeffs_vec[0] = 1;
      opt.model_coeffs_vec[1] = -0.019;
      opt.model_coeffs_vec[2] =  0.000242;   //0.242*1e-3;
      opt.model_coeffs_vec[3] = -0.00000146; //-1.46*1e-6;
      opt.model_coeffs_vec[4] = 1;
      opt.model_coeffs_vec[5] = 0;
      opt.model_coeffs_vec[6] = 0;
      opt.model_coeffs_vec[7] = 0;
      opt.model_coeffs_vec[8] = 1;
      opt.model_coeffs_vec[9] = -0.019;
      opt.model_coeffs_vec[10] =  0.000242;   //0.242*1e-3;
      opt.model_coeffs_vec[11] = -0.00000146; //-1.46*1e-6;
      opt.model_coeffs_vec[12] = 1;
      opt.model_coeffs_vec[13] = 0;
      opt.model_coeffs_vec[14] = 0;
      opt.model_coeffs_vec[15] = 0;
    }else if (refl_params.reflectanceType == HAPKE) {
      opt.model_coeffs_vec[0] = 0.68; // omega (also known as w)
      opt.model_coeffs_vec[1] = 0.17; // b
      opt.model_coeffs_vec[2] = 0.62; // c
      opt.model_coeffs_vec[3] = 0.52; // B0
      opt.model_coeffs_vec[4] = 0.52; // h
    }else if (refl_params.reflectanceType == CHARON) {
      opt.model_coeffs_vec.resize(g_num_model_coeffs);
      opt.model_coeffs_vec[0] = 0.7; // A
      opt.model_coeffs_vec[1] = 0.63; // f(alpha)
    }else if (refl_params.reflectanceType != LAMBERT) {
      vw_throw( ArgumentErr() << "The Hapke model coefficients were not set. "
                << "Use the --model-coeffs option." );
    }
  }
}

// Find the best-fit exposure and haze given the input sampled image and reflectance.
// Also find the sampled albedo along the way.
// TODO(oalexan1): Move this to SfsCostFun.cc.
void estimateExposureHazeAlbedo(Options & opt,
                                std::vector<MaskedImgT> const& masked_images,
                                std::vector<DoubleImgT> const& blend_weights,
                                ImageView<double> const& dem,
                                double mean_albedo,
                                double dem_nodata_val,
                                vw::cartography::GeoReference const& geo,
                                std::vector<vw::CamPtr> const& cameras,
                                double & max_dem_height,
                                std::vector<BBox2i> const& crop_boxes,
                                std::vector<vw::Vector3> const& sunPosition,
                                ReflParams const& refl_params,
                                double gridx, double gridy) {

  vw::vw_out() << "Estimating the exposure and haze.\n";

  int num_samples = 400; // TODO(oalexan1): Must be exposed
  
  int num_images = opt.input_images.size();
  std::vector<double> local_exposures_vec(num_images, 0), local_haze_vec(num_images, 0);
  
  std::vector<ImageView<PixelMask<double>>> reflectance(num_images), intensity(num_images);
  int num_sampled_cols = 0, num_sampled_rows = 0;
  for (int image_iter = 0; image_iter < num_images; image_iter++) {
    
    if (opt.skip_images.find(image_iter) != opt.skip_images.end()) 
      continue;
      
    // Sample large DEMs.
    int sample_col_rate = 0, sample_row_rate = 0;
    asp::calcSampleRates(dem, num_samples, sample_col_rate, sample_row_rate);

    ImageView<double> ground_weight;
    ImageView<Vector2> pq; // no need for these just for initialization
    computeReflectanceAndIntensity(dem, pq, geo,
                                   opt.model_shadows, max_dem_height,
                                   gridx, gridy, sample_col_rate, sample_row_rate,
                                   sunPosition[image_iter],
                                   refl_params,
                                   crop_boxes[image_iter],
                                   masked_images[image_iter],
                                   blend_weights[image_iter],
                                   cameras[image_iter].get(),
                                   reflectance[image_iter], intensity[image_iter],
                                   ground_weight,
                                   &opt.model_coeffs_vec[0]);
    
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
  
  // Create the problem
  ceres::Problem problem;
  
  for (int image_iter = 0; image_iter < num_images; image_iter++) {
    for (int col = 0; col < intensity[image_iter].cols(); col++) {
      for (int row = 0; row < intensity[image_iter].rows(); row++) {
         
       if (!is_valid(intensity[image_iter](col, row)) ||
            !is_valid(reflectance[image_iter](col, row)))
          continue;
        
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
        
        if (!opt.float_albedo)
          problem.SetParameterBlockConstant(&albedo(col, row));
       }
    }
  }
  
  // Options
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
  vw_out() << summary.FullReport() << "\n";
  
  if (opt.float_albedo) {
    // Save the low-res albedo. No georef is saved as that would need
    // resampling, which is tricky.
    bool has_georef = false; 
    bool has_nodata = true;
    vw::TerminalProgressCallback tpc("asp", ": ");
    std::string albedo_file = opt.out_prefix + "-lowres-albedo.tif";
    vw_out() << "Writing: " << albedo_file << "\n";
    block_write_gdal_image(albedo_file, albedo, has_georef, geo, has_nodata, dem_nodata_val,
                           opt, tpc); 
  }
    
  return;
}

// TODO(oalexan1): Modularize this long function
int main(int argc, char* argv[]) {

  Stopwatch sw_total;
  sw_total.start();
  
  Options opt;
  try {
    handle_arguments(argc, argv, opt);

    if (opt.compute_exposures_only && !opt.image_exposures_vec.empty()) {
      // TODO: This needs to be adjusted if haze is computed.
      vw_out() << "Exposures exist.";
      return 0;
    }

    // Set up model information
    ReflParams refl_params;
    setupReflectance(refl_params, opt);
    
    // Manage no-data
    double dem_nodata_val = -std::numeric_limits<float>::max(); // note we use a float nodata
    if (vw::read_nodata_val(opt.input_dem, dem_nodata_val)) {
      vw_out() << "Found DEM nodata value: " << dem_nodata_val << std::endl;
      if (std::isnan(dem_nodata_val)) {
        dem_nodata_val = -std::numeric_limits<float>::max(); // bugfix for NaN
        vw_out() << "Overwriting the nodata value with: " << dem_nodata_val << "\n";
      }
    }
    if (!boost::math::isnan(opt.nodata_val)) {
      dem_nodata_val = opt.nodata_val;
      vw_out() << "Over-riding the DEM nodata value with: " << dem_nodata_val << "\n";
    }
    
    // Read the handle to the DEM. Here we don't load the DEM into
    // memory yet. We will later load into memory only a crop,
    // if cropping is specified. This is to save on memory.
    ImageViewRef<double> dem_handle = DiskImageView<double>(opt.input_dem);

    // This must be done before the DEM is cropped. This stats is
    // queried from parallel_sfs.
    if (opt.query) {
      vw_out() << "dem_cols, " << dem_handle.cols() << "\n";
      vw_out() << "dem_rows, " << dem_handle.rows() << "\n";
    }

    // Adjust the crop win
    opt.crop_win.crop(bounding_box(dem_handle));
    
    // Read the georeference 
    vw::cartography::GeoReference geo;
    if (!read_georeference(geo, opt.input_dem))
        vw_throw(ArgumentErr() << "The input DEM has no georeference.\n");
      
    // This is a bug fix. The georef pixel size in y must be negative
    // for the DEM to be oriented correctly. 
    if (geo.transform()(1, 1) > 0)
      vw_throw(ArgumentErr() << "The input DEM has a positive pixel size in y. "
                << "This is unexpected. Normally it is negative since the (0, 0) "
                << "pixel is in the upper-left. Check your DEM pixel size with "
                << "gdalinfo. Cannot continue.\n");
      
    // Crop the DEM and georef if requested to given box.  The
    // cropped DEM (or uncropped if no cropping happens) is fully
    // loaded in memory.
    vw::ImageView<double> dem, orig_dem;
    if (!opt.crop_win.empty()) {
      dem = crop(dem_handle, opt.crop_win);
      geo = crop(geo, opt.crop_win);
    } else {
      dem = dem_handle; // load in memory
    }
  
    // This can be useful
    vw_out() << "DEM cols and rows: " << dem.cols()  << ' ' << dem.rows() << "\n";

    // See if to use provided initial DEM height    
    if (!boost::math::isnan(opt.init_dem_height)) {
      for (int col = 0; col < dem.cols(); col++) {
        for (int row = 0; row < dem.rows(); row++) {
          dem(col, row) = opt.init_dem_height;
        }
      }
    }
    
    // Read the sun positions from a list, if provided. Otherwise those
    // are read from the cameras, further down.
    int num_images = opt.input_images.size();
    std::vector<vw::Vector3> sunPosition(num_images, vw::Vector3());
    if (opt.sun_positions_list != "") 
      asp::readSunPositions(opt.sun_positions_list, opt.input_images,
                            dem, dem_nodata_val, geo, sunPosition);
    if (opt.sun_angles_list != "") 
      asp::readSunAngles(opt.sun_angles_list, opt.input_images,
                         dem, dem_nodata_val, geo, sunPosition);
      
    // Read in the camera models (and the sun positions, if not read from the list)
    // TODO(oalexan1): Put this in a function called readCameras())
    std::vector<vw::CamPtr> cameras(num_images);
    // TODO(oalexan1): First part can be replaced with calling load_cameras()
    for (int image_iter = 0; image_iter < num_images; image_iter++) {
    
      if (opt.skip_images.find(image_iter) != opt.skip_images.end()) 
        continue;
    
      asp::SessionPtr session(asp::StereoSessionFactory::create
                  (opt.stereo_session, // in-out
                  opt,
                  opt.input_images[image_iter],
                  opt.input_images[image_iter],
                  opt.input_cameras[image_iter],
                  opt.input_cameras[image_iter],
                  opt.out_prefix));

      //vw_out() << "Loading image and camera: " << opt.input_images[image_iter] << " "
      //          <<  opt.input_cameras[image_iter] << ".\n";
      cameras[image_iter] = session->camera_model(opt.input_images[image_iter],
                                                  opt.input_cameras[image_iter]);

      // TODO(oalexan1): Put this in a function called readSunPosition()
      // Read the sun position from the camera if it is was not read from the list
      if (sunPosition[image_iter] == Vector3())
        sunPosition[image_iter] = asp::sunPositionFromCamera(cameras[image_iter]);

      // Sanity check
      if (sunPosition[image_iter] == Vector3())
        vw_throw(ArgumentErr()
                  << "Could not read sun positions from list or from camera model files.\n");
        
      // Compute the azimuth and elevation
      double azimuth = 0.0, elevation = 0.0;
      asp::sunAngles(dem, dem_nodata_val, geo, sunPosition[image_iter],
                     azimuth, elevation);

      // Print this. It will be used to organize the images by illumination
      // for bundle adjustment.
      // Since the sun position has very big values and we want to sort uniquely
      // the images by azimuth angle, use high precision below.
      vw_out().precision(17);
      vw_out() << "Sun position for: " << opt.input_images[image_iter] << " is "
                << sunPosition[image_iter] << "\n";
      vw_out() << "Sun azimuth and elevation for: "
                << opt.input_images[image_iter] << " are " << azimuth
                << " and " << elevation << " degrees.\n";
      vw_out().precision(6); // Go back to usual precision
    }

    // Stop here if all we wanted was some information
    if (opt.query) 
      return 0;

    // Refuse to run if there are no-data values or if the DEM is too small
    int min_dem_size = 5;
    for (int col = 0; col < dem.cols(); col++) {
      for (int row = 0; row < dem.rows(); row++) {
        if (dem(col, row) == dem_nodata_val ||
            std::isnan(dem(col, row))) {
          vw_throw(ArgumentErr() 
                    << "Found a no-data or NaN pixel in the DEM. Cannot continue. "
                    << "The dem_mosaic tool can be used to fill in holes. Then "
                    << "crop and use a clip from this DEM having only valid data.");
        }
      }
    }
    if (dem.cols() < min_dem_size || dem.rows() < min_dem_size)
      vw_throw(ArgumentErr() << "The input DEM is too small.\n");

    // This check must be here, after we find the session
    if (opt.stereo_session != "isis" && opt.use_approx_camera_models) {
      vw_out() << "Computing approximate models works only with ISIS cameras. "
               << "Ignoring this option.\n";
      opt.use_approx_camera_models = false;
    }

    // Ensure our camera models are always adjustable. 
    // TODO(oalexan1): This is likely not needed anymore.
    for (int image_iter = 0; image_iter < num_images; image_iter++) {
      
      if (opt.skip_images.find(image_iter) != opt.skip_images.end())
        continue;

      CameraModel * icam
        = dynamic_cast<vw::camera::AdjustedCameraModel*>(cameras[image_iter].get());
      if (icam == NULL) {
        // Set a default identity adjustment
        Vector2 pixel_offset(0, 0);
        Vector3 translation(0, 0, 0);
        Quaternion<double> rotation = Quat(math::identity_matrix<3>());
        // For clarity, first make a copy of the object that we will overwrite.
        // This may not be necessary but looks safer this way.
        boost::shared_ptr<CameraModel> cam_ptr = cameras[image_iter];
        cameras[image_iter] = boost::shared_ptr<CameraModel>
          (new vw::camera::AdjustedCameraModel(cam_ptr, translation,
                                    rotation, pixel_offset));
      }
    }
    
    // We won't load the full images, just portions restricted
    // to the area we we will compute the DEM.
    std::vector<BBox2i> crop_boxes(num_images);
    
    // The crop box starts as the original image bounding box. We'll shrink it later.
    for (int image_iter = 0; image_iter < num_images; image_iter++) {
      std::string img_file = opt.input_images[image_iter];
      crop_boxes[image_iter] = vw::bounding_box(DiskImageView<float>(img_file));
    }
    
    // Ensure that no two threads can access an ISIS camera at the same time.
    // Declare the lock here, as we want it to live until the end of the program. 
    vw::Mutex camera_mutex;

    // If to use approximate camera models or to crop input images
    if (opt.use_approx_camera_models) {

      // TODO(oalexan1): This code needs to be modularized.
      double max_approx_err = 0.0;
    
      for (int image_iter = 0; image_iter < num_images; image_iter++){
      
        if (opt.skip_images.find(image_iter) != opt.skip_images.end()) continue;
    
        // Here we make a copy, since soon cameras[image_iter] will be overwritten
        vw::camera::AdjustedCameraModel exact_camera
          = *dynamic_cast<vw::camera::AdjustedCameraModel*>(cameras[image_iter].get());

        vw_out() << "Creating an approximate camera model for "
                << opt.input_images[image_iter] << "\n";
        BBox2i img_bbox = crop_boxes[image_iter];
        Stopwatch sw;
        sw.start();
        boost::shared_ptr<CameraModel> apcam;
        apcam.reset(new asp::ApproxCameraModel(exact_camera, img_bbox, dem, geo,
                                               dem_nodata_val, camera_mutex));
        cameras[image_iter] = apcam;
        
        sw.stop();
        vw_out() << "Approximate model generation time: " << sw.elapsed_seconds()
                << " s." << std::endl;
        
        // Cast the pointer back to ApproxCameraModel as we need that.
        asp::ApproxCameraModel* cam_ptr 
          = dynamic_cast<asp::ApproxCameraModel*>(apcam.get());
        if (cam_ptr == NULL) 
          vw_throw(ArgumentErr() << "Expecting an ApproxCameraModel.");

        bool model_is_valid = cam_ptr->model_is_valid();
        
        // Compared original and approximate models
        double max_curr_err = 0.0;

        if (model_is_valid) {
          for (int col = 0; col < dem.cols(); col++) {
            for (int row = 0; row < dem.rows(); row++) {
              Vector2 ll = geo.pixel_to_lonlat(Vector2(col, row));
              Vector3 xyz = geo.datum().geodetic_to_cartesian
                (Vector3(ll[0], ll[1], dem(col, row)));

              // Test how the exact and approximate models compare
              Vector2 pix3 = exact_camera.point_to_pixel(xyz);
              Vector2 pix4 = cameras[image_iter]->point_to_pixel(xyz);
              max_curr_err = std::max(max_curr_err, norm_2(pix3 - pix4));

              cam_ptr->crop_box().grow(pix3);
              cam_ptr->crop_box().grow(pix4);
            }
          }

          cam_ptr->crop_box().crop(img_bbox);
        } else {
          vw_out() << "Invalid camera model.\n";
        }
        
        if (max_curr_err > 2.0 || !model_is_valid) {
          // This is a bugfix. When the DEM clip does not intersect the image,
          // the approx camera model has incorrect values.
          if (model_is_valid)
            vw_out() << "Error of camera approximation is too big.\n";
          vw_out() << "Skip image " << image_iter << "\n";
          opt.skip_images.insert(image_iter);
          cam_ptr->crop_box() = BBox2();
          max_curr_err = 0.0;
        }

        max_approx_err = std::max(max_approx_err, max_curr_err);
      
        cam_ptr->crop_box().crop(img_bbox);
        vw_out() << "Crop box dimensions: " << cam_ptr->crop_box() << std::endl;
  
        // Copy the crop box
        if (opt.crop_input_images)
          crop_boxes[image_iter].crop(cam_ptr->crop_box());

        // Skip images which result in empty crop boxes
        if (crop_boxes[image_iter].empty()) {
          opt.skip_images.insert(image_iter);
        }
        
      } // end iterating over images
      vw_out() << "Max error of approximating cameras: " << max_approx_err << " pixels.\n";
      
      // end computing the approximate camera model
    } else if (opt.crop_input_images) {
      
      // We will arrive here if it is desired to crop the input images
      // without using an approximate model, such as for CSM.
      // Estimate the crop box by projecting the pixels in the exact
      // camera (with the adjustments applied, if present).
      
      for (int image_iter = 0; image_iter < num_images; image_iter++){
        if (opt.skip_images.find(image_iter)
            != opt.skip_images.end()) continue;

        // Store the full image box, and initialize the crop box to an empty box
        BBox2i img_bbox = crop_boxes[image_iter];
        crop_boxes[image_iter] = BBox2i();
        
        for (int col = 0; col < dem.cols(); col++) {
          for (int row = 0; row < dem.rows(); row++) {
            Vector2 ll = geo.pixel_to_lonlat(Vector2(col, row));
            Vector3 xyz = geo.datum().geodetic_to_cartesian
              (Vector3(ll[0], ll[1], dem(col, row)));
            
            Vector2 pix = cameras[image_iter]->point_to_pixel(xyz);
            crop_boxes[image_iter].grow(pix); 
          }
        }

        // Double the box dimensions, just in case. Later the SfS heights
        // may change, and we may need to see beyond the given box.
        // TODO(oalexan1): This is likely excessive.
        double extraFactor = 0.5;
        double extrax = extraFactor * crop_boxes[image_iter].width();
        double extray = extraFactor * crop_boxes[image_iter].height();
        crop_boxes[image_iter].min() -= Vector2(extrax, extray);
        crop_boxes[image_iter].max() += Vector2(extrax, extray);

        // Crop to the bounding box of the image
        crop_boxes[image_iter].crop(img_bbox);
          
        //vw_out() << "Estimated crop box for image " << opt.input_images[image_iter] << "\n";
        
        if (crop_boxes[image_iter].empty()) 
          opt.skip_images.insert(image_iter);
      }
    } // end the case of cropping the input images

    // Masked images and weights
    std::vector<MaskedImgT> masked_images(num_images);
    std::vector<DoubleImgT> blend_weights(num_images);
    
    float img_nodata_val = -std::numeric_limits<float>::max();
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
          ImageView<float> cropped_img = 
            crop(DiskImageView<float>(img_file), crop_boxes[image_iter]);
          masked_images[image_iter]
            = create_pixel_range_mask2(cropped_img,
                                        std::max(img_nodata_val, shadow_thresh),
                                        opt.max_valid_image_vals_vec[image_iter]);

          // Compute blending weights only when cropping the
          // images. Otherwise the weights are too huge.
          if (opt.blending_dist > 0)
            blend_weights[image_iter]
              = asp::blendingWeights(masked_images[image_iter],
                                     opt.blending_dist, opt.blending_power,
                                     opt.min_blend_size);
        }
      }else{
        masked_images[image_iter]
          = create_pixel_range_mask2(DiskImageView<float>(img_file),
                                     std::max(img_nodata_val, shadow_thresh),
                                     opt.max_valid_image_vals_vec[image_iter]);
      }
    }

    // Find the grid sizes in meters. Note that dem heights are in
    // meters too, so we treat both horizontal and vertical
    // measurements in same units.
    
    // Sample large DEMs. Keep about 200 row and column samples.
    int num_samples = 200;
    int sample_col_rate = 0, sample_row_rate = 0;
    asp::calcSampleRates(dem, num_samples, sample_col_rate, sample_row_rate);
    double gridx = 0.0, gridy = 0.0;
    asp::calcGsd(dem, geo, dem_nodata_val, sample_col_rate, sample_row_rate,
                 gridx, gridy); // outputs
    vw_out() << "DEM grid in x and y in meters: " << gridx << ' ' << gridy << "\n";

    // Find the max DEM height
    double max_dem_height = -std::numeric_limits<double>::max();
    if (opt.model_shadows) {
      for (int col = 0; col < dem.cols(); col++) {
        for (int row = 0; row < dem.rows(); row++) {
          if (dem(col, row) > max_dem_height) {
            max_dem_height = dem(col, row);
          }
        }
      }
    }
    
    // Compute or read the initial albedo.
    vw::ImageView<double> albedo;
    if (opt.input_albedo.empty()) {
      double initial_albedo = 1.0;
      albedo.set_size(dem.cols(), dem.rows());
      for (int col = 0; col < albedo.cols(); col++) {
        for (int row = 0; row < albedo.rows(); row++) {
          albedo(col, row) = initial_albedo;
        }
      }
    } else {
      // Read the input albedo
      vw::vw_out() << "Reading albedo from: " << opt.input_albedo << "\n";
      albedo = DiskImageView<double>(opt.input_albedo);
      // Must have the same size as dem
      if (albedo.cols() != dem.cols() || albedo.rows() != dem.rows())
        vw::vw_throw(ArgumentErr() 
                      << "Albedo image must have the same dimensions as the DEM.\n");
    }
    
    // Find the mean albedo
    double mean_albedo = 0.0, albedo_count = 0.0;
    for (int col = 0; col < dem.cols(); col++) {
      for (int row = 0; row < dem.rows(); row++) {
        if (dem(col, row) != dem_nodata_val) {
          mean_albedo += albedo(col, row);
          albedo_count += 1.0;
        }
      }
    }
    mean_albedo /= albedo_count;
    
    // Assume that haze is 0 to start with. Find the exposure as
    // mean(intensity)/mean(reflectance)/albedo. Use this to compute an
    // initial exposure and decide based on that which images to
    // skip. If the user provided initial exposures and haze, use those, but
    // still go through the motions to find the images to skip.
    // See the intensity formula in calcIntensity().
    // vw_out() << "Computing exposures.\n";
    std::vector<double> local_exposures_vec(num_images, 0), local_haze_vec(num_images, 0);
    for (int image_iter = 0; image_iter < num_images; image_iter++) {
      
      if (opt.skip_images.find(image_iter) != opt.skip_images.end()) 
        continue;
      
      // Sample large DEMs. Keep about 200 row and column samples.
      int num_samples = 200;
      int sample_col_rate = 0, sample_row_rate = 0;
      asp::calcSampleRates(dem, num_samples, sample_col_rate, sample_row_rate);

      ImageView<PixelMask<double>> reflectance, intensity;
      ImageView<double> ground_weight;
      ImageView<Vector2> pq; // no need for these just for initialization
      
      computeReflectanceAndIntensity(dem, pq, geo,
                                      opt.model_shadows, max_dem_height,
                                      gridx, gridy, sample_col_rate, sample_row_rate,
                                      sunPosition[image_iter],
                                      refl_params,
                                      crop_boxes[image_iter],
                                      masked_images[image_iter],
                                      blend_weights[image_iter],
                                      cameras[image_iter].get(),
                                      reflectance, intensity, ground_weight,
                                      &opt.model_coeffs_vec[0]);
      
      // TODO: Below is not the optimal way of finding the exposure!
      // Find it as the analytical minimum using calculus.
      double imgmean, imgstd, refmean, refstd;
      asp::calcJointStats(intensity, reflectance, imgmean, imgstd, refmean, refstd);
      double haze = 0.0;
  
      if (imgmean > 0 && refmean > 0) {
        double exposure = imgmean/refmean/mean_albedo;
        local_exposures_vec[image_iter] = exposure;
        local_haze_vec[image_iter] = haze;
        //vw_out() << "Local DEM estimated exposure for image " << image_iter << ": " 
        //          << exposure << "\n";
      } else {
        // Skip images with bad exposure. Apparently there is no good
        // imagery in the area.
        opt.skip_images.insert(image_iter);
        vw_out() << "Skip image with no data " << image_iter << " for this DEM.\n";
      }
    }
    
    // Only overwrite the exposures if we don't have them supplied
    if (opt.image_exposures_vec.empty()) {
      vw::vw_out() << "Using the input image exposures.\n";
      opt.image_exposures_vec = local_exposures_vec;
    }

    // Initialize the haze as 0. If computed above, initialize its first coeff.
    if ((!opt.image_haze_vec.empty()) && (int)opt.image_haze_vec.size() != num_images)
      vw_throw(ArgumentErr() << "Expecting as many haze values as images.\n");
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
    
    if (opt.estimate_exposure_haze_albedo) {
      // TODO(oalexan1): Check if --num-haze-coeffs is non-zero.
      // TODO(oalexan1): This should work even if albedo is not modeled.
      estimateExposureHazeAlbedo(opt, masked_images, blend_weights,
                                 dem, mean_albedo, dem_nodata_val,
                                 geo, cameras, max_dem_height,
                                 crop_boxes, sunPosition,
                                 refl_params, gridx, gridy);
    }
    
    for (size_t image_iter = 0; image_iter < opt.image_exposures_vec.size(); image_iter++) {
      vw_out() << "Image exposure for " << opt.input_images[image_iter] << ' '
               << opt.image_exposures_vec[image_iter] << std::endl;
    }

    if (opt.compute_exposures_only || opt.estimate_exposure_haze_albedo) {
      asp::saveExposures(opt.out_prefix, opt.input_images, opt.image_exposures_vec);
      // TODO(oalexan1): Think of this more
      if (opt.num_haze_coeffs > 0)
        asp::saveHaze(opt.out_prefix, opt.input_images, opt.image_haze_vec);
      // all done
      return 0;
    }

    // Need to compute the valid data image to be able to find the grid points always
    // in shadow, so when this image is zero.
    ImageView<int> lit_image_mask;
    if (opt.curvature_in_shadow_weight > 0.0) {
      lit_image_mask.set_size(dem.cols(), dem.rows());
      for (int col = 0; col < lit_image_mask.cols(); col++) {
        for (int row = 0; row < lit_image_mask.rows(); row++) {
          lit_image_mask(col, row) = 0; // no valid points originally
        }
      }
    }

    // If opt.allow_borderline_data is true, create for each image that will not be skipped
    // a weight matrix with dimensions equal to DEM dimensions, that will be used instead
    // of weights in the camera image space. These are balanced among each other and give more
    // weight to barely lit and unlit nearby pixels.
    std::vector<ImageView<double>> ground_weights(num_images);
    
    // Note that below we may use the exposures computed at the previous step
    // TODO(oalexan1): This block must be a function.
    if (opt.save_computed_intensity_only || opt.estimate_slope_errors ||
        opt.estimate_height_errors || opt.curvature_in_shadow_weight > 0.0 ||
        opt.allow_borderline_data) {
      // In this case simply save the computed and actual intensity, and for most of these quit
      ImageView<PixelMask<double>> reflectance, meas_intensity, comp_intensity;
      ImageView<double> ground_weight;
      ImageView<Vector2> pq; // no need for these just for initialization
      int sample_col_rate = 1, sample_row_rate = 1;

      boost::shared_ptr<SlopeErrEstim> slopeErrEstim = boost::shared_ptr<SlopeErrEstim>(NULL);
      if (opt.estimate_slope_errors) {
        int num_a_samples = 90; // Sample the 0 to 90 degree range with this many samples
        int num_b_samples = 360; // sample the 0 to 360 degree range with this many samples
        slopeErrEstim = boost::shared_ptr<SlopeErrEstim>
          (new SlopeErrEstim(dem.cols(), dem.rows(),
                             num_a_samples, num_b_samples, &albedo, &opt));
      }
      
      boost::shared_ptr<HeightErrEstim> heightErrEstim = boost::shared_ptr<HeightErrEstim>(NULL);
      if (opt.estimate_height_errors) {
        double max_height_error  = opt.height_error_params[0];
        int num_height_samples   = opt.height_error_params[1];
        vw_out() << "Maximum height error to examine: " << max_height_error << "\n";
        vw_out() << "Number of samples to use from 0 to that height: "
                 << num_height_samples << "\n";
          
        double nodata_height_val = -1.0;
        heightErrEstim = boost::shared_ptr<HeightErrEstim>
          (new HeightErrEstim(dem.cols(), dem.rows(),
                              num_height_samples, max_height_error, nodata_height_val,
                              &albedo, &opt));
      }
      
      for (int image_iter = 0; image_iter < num_images; image_iter++) {
        
        if (opt.estimate_slope_errors) 
          slopeErrEstim->image_iter = image_iter;
        if (opt.estimate_height_errors) 
          heightErrEstim->image_iter = image_iter;

        // Find the reflectance and measured intensity (and work towards
        // estimating the slopes if asked to).
        computeReflectanceAndIntensity(dem, pq, geo,
                                       opt.model_shadows, max_dem_height,
                                       gridx, gridy, sample_col_rate, sample_row_rate,
                                       sunPosition[image_iter],
                                       refl_params,
                                       crop_boxes[image_iter],
                                       masked_images[image_iter],
                                       blend_weights[image_iter],
                                       cameras[image_iter].get(),
                                       reflectance, meas_intensity, ground_weight,
                                       &opt.model_coeffs_vec[0],
                                       slopeErrEstim.get(), heightErrEstim.get());

        if (opt.skip_images.find(image_iter) == opt.skip_images.end() &&
            opt.allow_borderline_data) {
          // if not skipping, save the weight
          ground_weights[image_iter] = copy(ground_weight);
        }
        
        // Find the computed intensity.
        // TODO(oalexan1): Should one mark the no-data values rather than setting
        // them to 0? 
        comp_intensity.set_size(reflectance.cols(), reflectance.rows());
        for (int col = 0; col < comp_intensity.cols(); col++) {
          for (int row = 0; row < comp_intensity.rows(); row++) {
            comp_intensity(col, row) = calcIntensity(albedo(col, row), 
                                                      reflectance(col, row), 
                                                      opt.image_exposures_vec[image_iter],
                                                      opt.steepness_factor,
                                                      &opt.image_haze_vec[image_iter][0], 
                                                      opt.num_haze_coeffs);
          }
        }
        
        if (opt.curvature_in_shadow_weight > 0.0) {
          if (meas_intensity.cols() != lit_image_mask.cols() ||
              meas_intensity.rows() != lit_image_mask.rows()) {
            vw_throw(ArgumentErr()
                     << "Intensity image dimensions disagree with DEM clip dimensions.\n");
          }
          for (int col = 0; col < lit_image_mask.cols(); col++) {
            for (int row = 0; row < lit_image_mask.rows(); row++) {
              if (is_valid(meas_intensity(col, row))           || 
                  col == 0 || col == lit_image_mask.cols() - 1 ||
                  row == 0 || row == lit_image_mask.rows() - 1) {
                // Boundary pixels are declared lit. Otherwise they are
                // always unlit due to the peculiarities of how the intensity
                // is found at the boundary.
                lit_image_mask(col, row) = 1;
              }
            }
          }
        }
        
        if (opt.save_computed_intensity_only) {
          TerminalProgressCallback tpc("asp", ": ");
          bool has_georef = true, has_nodata = true;
          std::string out_camera_file
            = asp::bundle_adjust_file_name(opt.out_prefix,
                                           opt.input_images[image_iter],
                                           opt.input_cameras[image_iter]);
          std::string local_prefix = fs::path(out_camera_file).replace_extension("").string();
          std::string out_meas_intensity_file = local_prefix + "-meas-intensity.tif";
          vw_out() << "Writing: " << out_meas_intensity_file << std::endl;
          block_write_gdal_image(out_meas_intensity_file,
                                 apply_mask(meas_intensity, img_nodata_val),
                                 has_georef, geo, has_nodata,
                                 img_nodata_val, opt, tpc);
          
          std::string out_comp_intensity_file = local_prefix + "-comp-intensity.tif";
          vw_out() << "Writing: " << out_comp_intensity_file << std::endl;
          block_write_gdal_image(out_comp_intensity_file,
                                 apply_mask(comp_intensity, img_nodata_val),
                                 has_georef, geo, has_nodata, img_nodata_val,
                                 opt, tpc);
        }

      } // End iterating over images

      if (opt.estimate_slope_errors) {
        // Find the slope error as the maximum of slope errors in all directions
        // from the given slope.
        ImageView<float> slope_error;
        slope_error.set_size(reflectance.cols(), reflectance.rows());
        double nodata_slope_value = -1.0;
        for (int col = 0; col < slope_error.cols(); col++) {
          for (int row = 0; row < slope_error.rows(); row++) {
            slope_error(col, row) = nodata_slope_value;
            int num_samples = slopeErrEstim->slope_errs[col][row].size();
            for (int sample = 0; sample < num_samples; sample++) {
              slope_error(col, row)
                = std::max(double(slope_error(col, row)),
                           slopeErrEstim->slope_errs[col][row][sample]);
            }
          }
        }
        
        // Slope errors that are stuck at 90 degrees could not be estimated
        for (int col = 0; col < slope_error.cols(); col++) {
          for (int row = 0; row < slope_error.rows(); row++) {
            if (slope_error(col, row) == slopeErrEstim->max_angle)
              slope_error(col, row) = nodata_slope_value;
          }
        }
        
        TerminalProgressCallback tpc("asp", ": ");
        bool has_georef = true, has_nodata = true;
        std::string slope_error_file = opt.out_prefix + "-slope-error.tif";
        vw_out() << "Writing: " << slope_error_file << std::endl;
        block_write_gdal_image(slope_error_file,
                               slope_error, has_georef, geo, has_nodata,
                               nodata_slope_value, opt, tpc);
      }

      if (opt.estimate_height_errors) {
        // Find the height error from the range of heights
        ImageView<float> height_error;
        height_error.set_size(heightErrEstim->height_error_vec.cols(),
                              heightErrEstim->height_error_vec.rows());
        for (int col = 0; col < height_error.cols(); col++) {
          for (int row = 0; row < height_error.rows(); row++) {
            height_error(col, row)
              = std::max(-heightErrEstim->height_error_vec(col, row)[0],
                         heightErrEstim->height_error_vec(col, row)[1]);
            
            // When we are stuck at the highest error that means we could not
            // find it
            if (height_error(col, row) == heightErrEstim->max_height_error)
              height_error(col, row) = heightErrEstim->nodata_height_val;
          }
        }
        TerminalProgressCallback tpc("asp", ": ");
        bool has_georef = true, has_nodata = true;
        std::string height_error_file = opt.out_prefix + "-height-error.tif";
        vw_out() << "Writing: " << height_error_file << std::endl;
        block_write_gdal_image(height_error_file,
                               height_error,
                               has_georef, geo,
                               has_nodata, heightErrEstim->nodata_height_val,
                               opt, tpc);
      }
      
    } // End doing intensity computations and/or height and/or slope error estimations
      
    if (opt.save_computed_intensity_only || opt.estimate_slope_errors ||
        opt.estimate_height_errors) {
      asp::saveExposures(opt.out_prefix, opt.input_images, opt.image_exposures_vec);
      // All done
      return 0;
    }
    
    // Print and make global the exposures and haze
    if (opt.num_haze_coeffs > 0) {
      for (size_t image_iter = 0; image_iter < opt.image_haze_vec.size(); image_iter++) {
        vw_out() << "Image haze for " << opt.input_images[image_iter] << ':';
        for (size_t hiter = 0; hiter < opt.image_haze_vec[image_iter].size(); hiter++) {
          vw_out() << " " << opt.image_haze_vec[image_iter][hiter];
        }
        vw_out() << "\n";
      }
    }

    if (opt.allow_borderline_data) {
      int cols = dem.cols(), rows = dem.rows();
      asp::adjustBorderlineDataWeights(cols, rows, opt.blending_dist, opt.blending_power,
                                       vw::GdalWriteOptions(opt), // slice
                                       geo,
                                       opt.skip_images,
                                       opt.out_prefix, // for debug data
                                       opt.input_images, opt.input_cameras, 
                                       ground_weights); // output

      // Use the ground weights from now on instead of blending weights.
      // Will overwrite the weights below.
      g_blend_weight_is_ground_weight = true;

      // Redo the image masks. Unlike before, the shadow threshold is set to 0
      // to allow shadow pixels. The weights will control how much of these
      // are actually used. This approach is better than a hard cutoff with the mask.
      for (int image_iter = 0; image_iter < num_images; image_iter++) {
        if (opt.skip_images.find(image_iter) != opt.skip_images.end())
          continue;
        
        std::string img_file = opt.input_images[image_iter];
        vw::read_nodata_val(img_file, img_nodata_val);
        float shadow_thresh = 0.0; // Note how the shadow thresh is now 0, unlike before
        // Make a copy in memory for faster access
        if (!crop_boxes[image_iter].empty()) {
          ImageView<float> cropped_img = 
            crop(DiskImageView<float>(img_file), crop_boxes[image_iter]);
          masked_images[image_iter]
            = create_pixel_range_mask2(cropped_img,
                                        std::max(img_nodata_val, shadow_thresh),
                                        opt.max_valid_image_vals_vec[image_iter]);

          // Overwrite the blending weights with ground weights
          blend_weights[image_iter] = copy(ground_weights[image_iter]);
        }
      }

      ground_weights.clear(); // not needed anymore
    } // end allow borderline data
    
    ImageView<double> curvature_in_shadow_weight;
    if (opt.curvature_in_shadow_weight > 0.0) {
      TerminalProgressCallback tpc("asp", ": ");
      bool has_georef = true, has_nodata = false;
      double nodata_val = -1; // will not be used
      std::string lit_image_mask_file = opt.out_prefix + "-lit_image_mask.tif";
      vw_out() << "Writing: " << lit_image_mask_file << std::endl;
      block_write_gdal_image(lit_image_mask_file, lit_image_mask,
                             has_georef, geo, has_nodata, nodata_val, opt, tpc);

      // Form the curvature_in_shadow_weight image. It will start at 0
      // at distance opt.lit_curvature_dist from the shadow
      // boundary in the lit area, and then reach value
      // opt.curvature_in_shadow_weight when at distance
      // opt.shadow_curvature_dist from the boundary in the shadowed
      // area. This is done to avoid boundary artifacts.
      double max_dist = std::max(opt.lit_curvature_dist, opt.shadow_curvature_dist);
      vw::bounded_signed_dist<int>(vw::create_mask(lit_image_mask, 0), max_dist,
                                   curvature_in_shadow_weight);
      // Do further adjustments
      for (int col = 0; col < curvature_in_shadow_weight.cols(); col++) {
        for (int row = 0; row < curvature_in_shadow_weight.rows(); row++) {
          double val = curvature_in_shadow_weight(col, row);
          val = std::min(val, opt.lit_curvature_dist);
          val = std::max(val, -opt.shadow_curvature_dist);
          val = (opt.lit_curvature_dist - val) /
            (opt.lit_curvature_dist + opt.shadow_curvature_dist);
          curvature_in_shadow_weight(col, row) = val * opt.curvature_in_shadow_weight;
        }
      }
      
      std::string curvature_in_shadow_weight_file = opt.out_prefix
        + "-curvature_in_shadow_weight.tif";
      vw_out() << "Writing: " << curvature_in_shadow_weight_file << std::endl;
      block_write_gdal_image(curvature_in_shadow_weight_file, curvature_in_shadow_weight,
                             has_georef, geo, has_nodata, nodata_val, opt, tpc);
    }
    
    // For images that we don't use, wipe the cameras and all other
    // info, as those take up memory (the camera is a table). 
    for (int image_iter = 0; image_iter < num_images; image_iter++) {
      if (opt.skip_images.find(image_iter) != opt.skip_images.end()) {
        masked_images[image_iter] = ImageView<PixelMask<float>>();
        blend_weights[image_iter] = ImageView<double>();
        cameras[image_iter] = boost::shared_ptr<CameraModel>();
      }
    }
    
    // orig_dem will keep the input DEMs and won't change. Keep to the optimized
    // DEMs close to orig_dem. Make a deep copy below.
    orig_dem = copy(dem);
    
    run_sfs(// Fixed quantities
            opt.max_iterations, gridx, gridy, opt, geo, opt.smoothness_weight, 
            max_dem_height, dem_nodata_val, img_nodata_val,  crop_boxes, masked_images, 
            blend_weights, refl_params, sunPosition, orig_dem,
            lit_image_mask, curvature_in_shadow_weight,
            // Variable quantities
            dem, albedo, cameras, opt.image_exposures_vec, opt.image_haze_vec,
            opt.model_coeffs_vec);

  } ASP_STANDARD_CATCHES;
  
  sw_total.stop();
  vw_out() << "Total elapsed time: " << sw_total.elapsed_seconds() << " s.\n";
  
  return 0;
}
