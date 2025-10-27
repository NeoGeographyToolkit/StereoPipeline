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
// TODO(oalexan1): Use OpenMP in ComputeReflectanceAndIntensity.
// For isis without approx models need to ensure there is one thread.
// May need to differentiate between single process and multiple processes.

/// \file sfs.cc

// Turn off warnings from boost and other packages
#if defined(__GNUC__) || defined(__GNUG__)
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-local-typedefs"
#endif

#include <asp/SfS/SfsImageProc.h>
#include <asp/SfS/SfsUtils.h>
#include <asp/SfS/SfsOptions.h>
#include <asp/SfS/SfsCamera.h>
#include <asp/SfS/SfsModel.h>
#include <asp/SfS/SfsErrorEstim.h>
#include <asp/SfS/SfsCostFun.h>
#include <asp/Sessions/StereoSessionFactory.h>
#include <asp/Camera/CsmModel.h>
#include <asp/Core/Macros.h>
#include <asp/Core/AspProgramOptions.h>
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
#include <vw/Image/InpaintView.h>
#include <vw/Image/DistanceFunction.h>
#include <vw/Cartography/GeoReferenceUtils.h>
#include <vw/Core/Stopwatch.h>
#include <vw/FileIO/FileUtils.h>

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

using namespace vw;
using namespace vw::camera;
using namespace vw::cartography;
using namespace asp;

void handle_arguments(int argc, char *argv[], SfsOptions& opt) {
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
    ("save-computed-intensity-only",   po::bool_switch(&opt.save_computed_intensity_only)->default_value(false)->implicit_value(true),
     "Save the computed (simulated) image intensities for given DEM, images, cameras, and "
     "reflectance model, without refining the DEM. The measured intensities will be saved "
     "as well, for comparison. The image exposures will be computed along the way unless "
     "specified via --image-exposures-prefix, and will be saved in either case to <output "
     "prefix>-exposures.txt. Same for haze, if applicable.")
    ("estimate-exposure-haze-albedo",
      po::bool_switch(&opt.estim_exposure_haze_albedo)->default_value(false)->implicit_value(true),
     "Estimate the exposure for each image, the haze for each image (if "
     "--num-haze-coeffs is positive), and the global low-resolution albedo (if "
     "--float-albedo is on), then quit. This operation samples the input DEM "
     "based on --num-samples-for-estim. The produced estimated exposure, haze, "
     "and initial albedo are described in the doc.")
    ("compute-exposures-only", po::bool_switch(&opt.compute_exposures_only)->default_value(false)->implicit_value(true),
     "This older option is equivalent to --estimate-exposure-haze-albedo.")
    ("estimate-height-errors",   po::bool_switch(&opt.estimate_height_errors)->default_value(false)->implicit_value(true),
     "Estimate the SfS DEM height uncertainty by finding the height perturbation (in meters) at each grid point which will make at least one of the simulated images at that point change by more than twice the discrepancy between the unperturbed simulated image and the measured image. The SfS DEM must be provided via the -i option. The number of iterations, blending parameters (--blending-dist, etc.), and smoothness weight are ignored. Results are not computed at image pixels in shadow. This produces <output prefix>-height-error.tif. No SfS DEM is computed. See also: --height-error-params.")
    ("height-error-params", po::value(&opt.height_error_params)->default_value(Vector2(5.0, 100.0), "5.0 100"),
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
     "Optional shadow thresholds for the input images (a list of real values in quotes, one per image). See also --shadow-threshold.")
    ("shadow-threshold", po::value(&opt.shadow_threshold)->default_value(-1),
     "A shadow threshold to apply to all images. Must be positive. Areas that "
     "are in shadow in all images will result in a blurred version of the input DEM, "
     "influenced by the --smoothness-weight.")
    ("custom-shadow-threshold-list", po::value(&opt.custom_shadow_threshold_list)->default_value(""),
     "A list having one image and one shadow threshold per line. For the images specified here, override the shadow threshold supplied by other means with this value.")
    ("low-light-threshold", 
     po::value(&opt.low_light_threshold)->default_value(-1),
     "A threshold for lit but low-light pixels. If positive, pixels with intensity "
     "between this and the shadow threshold will be given less weight, if other images "
     "have higher intensity values at the same ground point. This helps fix seams.")
    ("max-valid-image-vals", po::value(&opt.max_valid_image_vals)->default_value(""),
     "Optional values for the largest valid image value in each image (a list of real values in quotes, one per image).")
    ("robust-threshold", po::value(&opt.robust_threshold)->default_value(-1.0),
     "If positive, set the threshold for the robust measured-to-simulated intensity difference (using the Cauchy loss). Any difference much larger than this will be penalized. A good value may be 5% to 25% of the average image value or the same fraction of the computed image exposure values.")
    ("albedo-constraint-weight", po::value(&opt.albedo_constraint_weight)->default_value(0),
     "If floating the albedo, a larger value will try harder to keep the optimized albedo "
     "close to the initial albedo. See also --input-albedo and --albedo-robust-threshold.")
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
     "Crop the images to a region that was computed to be large enough, and keep them fully in memory, for speed. This is the default in the latest builds.")
    ("blending-dist", po::value(&opt.blending_dist)->default_value(0),
     "Give less weight to image pixels close to no-data or boundary values. Enabled only when crop-input-images is true, for performance reasons. Blend over this many pixels.")
    ("blending-power", po::value(&opt.blending_power)->default_value(2.0),
     "A higher value will result in smoother blending.")
    ("min-blend-size", po::value(&opt.min_blend_size)->default_value(0),
     "Do not apply blending in shadowed areas for which both the width and height are less than this.")
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
    ("num-samples-for-estim", po::value(&opt.num_samples_for_estim)->default_value(200),
     "Number of samples to use for estimating the exposure, haze, and albedo. A large "
     "value will result in a more accurate estimate, but will take a lot more memory.")
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
    ("camera-position-step-size",
     po::value(&opt.camera_position_step_size)->default_value(1.0),
     "Larger step size will result in more aggressiveness in varying the camera position if it is being floated (which may result in a better solution or in divergence).");

  general_options.add(vw::GdalWriteOptionsDescription(opt));

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

  // Always crop the input images. This has been validated enough that it works well.
  // There is no reason to fully load images in memory except for computing exposures,
  // when this option will be disabled automatically.
  opt.crop_input_images = true;

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
    vw_throw(ArgumentErr() << "Missing output prefix.\n"
              << usage << general_options);

  if (opt.max_iterations < 0)
    vw_throw(ArgumentErr() << "The number of iterations must be non-negative.\n"
              << usage << general_options);

  if (opt.input_images.empty())
    vw_throw(ArgumentErr() << "Missing input images.\n"
              << usage << general_options);

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

  // The options --compute-exposures-only and --estimate-exposure-haze-albedo
  // are equivalent.
  if (opt.compute_exposures_only)
    opt.estim_exposure_haze_albedo = true;
  if (opt.estim_exposure_haze_albedo)
    opt.compute_exposures_only = true;

  if (opt.compute_exposures_only || opt.estim_exposure_haze_albedo) {
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
  
  // print the shadow thresh per image
  for (size_t i = 0; i < opt.input_images.size(); i++) {
    vw_out() << "Using shadow threshold for image " << i << " (" << opt.input_images[i]
             << "): " << opt.shadow_threshold_vec[i] << std::endl;
  }
  
  // If the low-light-threshold is specified, ensure it is above all shadow thresholds,
  // and that the shadow thresholds are positive.
  if (opt.low_light_threshold > 0) {
    for (size_t i = 0; i < opt.input_images.size(); i++) {
      if (opt.shadow_threshold_vec[i] < 0)
        vw_throw(ArgumentErr() << "When --low-light-threshold is set, all shadow thresholds "
                 << "must be set and non-negative.\n");
      if (opt.low_light_threshold <= opt.shadow_threshold_vec[i])
        vw_throw(ArgumentErr() << "The low-light-threshold must be larger than all "
                 << "shadow thresholds.\n");
    }
    if (!opt.allow_borderline_data)
      vw::vw_throw(vw::ArgumentErr()
        << "When using --low-light-threshold, must set --allow-borderline-data.\n");
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
    while (1) {
      std::string line;
      std::getline(ish, line);
      std::istringstream hstream(line);
      if (! (hstream >> name)) break;
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
    if (opt.model_coeffs_vec.empty()) {
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
    while (is >> val)
      opt.skip_images.insert(val);
  }

  // estimate height errors and integrability constraint are mutually exclusive
  if (opt.estimate_height_errors && opt.integrability_weight > 0)
    vw_throw(ArgumentErr()
       << "Cannot estimate height errors when using the integrability constraint.\n");

  if (opt.estimate_height_errors && opt.model_shadows)
    vw_throw(ArgumentErr() << "Cannot estimate height error when modeling shadows.");

  if (opt.save_computed_intensity_only || opt.estimate_height_errors) {

    // No iterations
    opt.max_iterations = 0;

    // Need the exact cameras as they span the full DEM
    if (opt.use_approx_camera_models || opt.crop_input_images) {
      opt.use_approx_camera_models = false;
      opt.crop_input_images = false;
    }

    if (opt.num_haze_coeffs > 0 && opt.image_haze_vec.empty())
      vw_throw(ArgumentErr()
                << "Expecting the haze to be computed and passed in.\n");
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
             SfsOptions                       & opt,
             GeoReference               const & geo,
             double                             smoothness_weight,
             double                             max_dem_height,
             double                             dem_nodata_val,
             float                              img_nodata_val,
             std::vector<BBox2i>        const & crop_boxes,
             std::vector<MaskedImgRefT>    const & masked_images,
             std::vector<DoubleImgT>    const & blend_weights,
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

  int num_images = opt.input_images.size();
  ceres::Problem problem;
  vw::ImageView<Vector2> pq;
  asp::sfsCostFun(gridx, gridy, smoothness_weight, max_dem_height, dem_nodata_val,
                  img_nodata_val, blend_weight_is_ground_weight, geo,
                  crop_boxes, masked_images, blend_weights, refl_params, sunPosition,
                  orig_dem, lit_image_mask, curvature_in_shadow_weight, opt,
                  // Outputs
                  dem, albedo, cameras, exposures, haze, refl_coeffs, pq, problem);

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

  vw_out() << summary.FullReport() << "\n" << std::endl;
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
    vw_out() << "Approximate model generation time: " << sw.elapsed_seconds() << " s.\n";

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
          Vector2 pix3 = exact_camera->point_to_pixel(xyz);
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
  
} // end function calcApproxCamsCropBoxes

void calcCropBoxes(vw::ImageView<double> const& dem,
                   vw::cartography::GeoReference const& geo,
                   std::vector<vw::CamPtr> const& cameras,
                   // Outputs
                   SfsOptions &opt,
                   std::vector<vw::BBox2i> &crop_boxes) {

  int num_images = opt.input_images.size();
  for (int image_iter = 0; image_iter < num_images; image_iter++) {
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
          crop(DiskImageView<float>(img_file), crop_boxes[image_iter]);
        masked_images[image_iter]
          = create_pixel_range_mask2(cropped_img,
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
        = create_pixel_range_mask2(DiskImageView<float>(img_file),
                                    std::max(img_nodata_val, shadow_thresh),
                                    opt.max_valid_image_vals_vec[image_iter]);
    }
  }

} // end function loadMaskedImagesCalcWeights

// Heuristics for setting up the no-data value
double setupDemNodata(SfsOptions const& opt) {
  double dem_nodata_val = -1e+6;
  
  if (vw::read_nodata_val(opt.input_dem, dem_nodata_val)) {
    vw_out() << "Found DEM nodata value: " << dem_nodata_val << std::endl;
    if (std::isnan(dem_nodata_val)) {
      dem_nodata_val = -1e+6; // bugfix for NaN
      vw_out() << "Overwriting the nodata value with: " << dem_nodata_val << "\n";
    }
  }
  if (!boost::math::isnan(opt.nodata_val)) {
    dem_nodata_val = opt.nodata_val;
    vw_out() << "Over-riding the DEM nodata value with: " << dem_nodata_val << "\n";
  }

  return dem_nodata_val;
}

// Read the georeference for the DEM and albedo
void loadGeoref(SfsOptions const& opt,
                vw::cartography::GeoReference &geo,
                vw::cartography::GeoReference &albedo_geo) {

  if (!read_georeference(geo, opt.input_dem))
      vw_throw(ArgumentErr() << "The input DEM has no georeference.\n");
      
  if (!opt.input_albedo.empty()) {
    if (!read_georeference(albedo_geo, opt.input_albedo))
      vw_throw(ArgumentErr() << "The input albedo has no georeference.\n");
  } else {
    // Ensure initialization
    albedo_geo = geo;
  }

  // This is a bug fix. The georef pixel size in y must be negative
  // for the DEM to be oriented correctly.
  if (geo.transform()(1, 1) > 0)
    vw_throw(ArgumentErr() << "The input DEM has a positive pixel size in y. "
              << "This is unexpected. Normally it is negative since the (0, 0) "
              << "pixel is in the upper-left. Check your DEM pixel size with "
              << "gdalinfo. Cannot continue.\n");
    
  // The albedo and georef must have same wkt
  if (geo.get_wkt() != albedo_geo.get_wkt())
    vw::vw_throw(vw::ArgumentErr()
                  << "The input DEM has a different georeference "
                  << "from the input albedo image.\n");
}

double findMaxDemHeight(bool model_shadows, vw::ImageView<double> const& dem) {

  double max_dem_height = -std::numeric_limits<double>::max();
  if (model_shadows) {
    for (int col = 0; col < dem.cols(); col++) {
      for (int row = 0; row < dem.rows(); row++) {
        if (dem(col, row) > max_dem_height) {
          max_dem_height = dem(col, row);
        }
      }
    }
  }
  
  return max_dem_height;
}
  
double findMeanAlbedo(vw::ImageView<double> const& dem,
                      vw::ImageView<double> const& albedo,
                      double dem_nodata_val) {
    
  double mean_albedo = 0.0, albedo_count = 0.0;
  for (int col = 0; col < dem.cols(); col++) {
    for (int row = 0; row < dem.rows(); row++) {
      if (dem(col, row) != dem_nodata_val) {
        mean_albedo += albedo(col, row);
        albedo_count += 1.0;
      }
    }
  }
  
  if (albedo_count > 0)
    mean_albedo /= albedo_count;
  else
    mean_albedo = 0.0; // Or some other sensible default
    
  return mean_albedo;
}

int main(int argc, char* argv[]) {

  Stopwatch sw_total;
  sw_total.start();

  SfsOptions opt;
  try {
    handle_arguments(argc, argv, opt);

    // Set up model information
    ReflParams refl_params;
    setupReflectance(refl_params, opt);

    // Manage no-data. Use here a value that is not overly large in magnitude,
    // and easy to represent as float.
    double dem_nodata_val = setupDemNodata(opt);

    // Read the handle to the DEM. Here we don't load the DEM into
    // memory yet. We will later load into memory only a crop,
    // if cropping is specified. This is to save on memory.
    vw::ImageViewRef<double> full_dem = DiskImageView<double>(opt.input_dem);
    vw::ImageViewRef<double> full_albedo;
    
    // Read the albedo
    if (!opt.input_albedo.empty()) {
      vw::vw_out() << "Reading albedo from: " << opt.input_albedo << "\n";
      full_albedo = DiskImageView<double>(opt.input_albedo);
      // Must have the same size as dem
      if (full_albedo.cols() != full_dem.cols() || full_albedo.rows() != full_dem.rows())
        vw::vw_throw(vw::ArgumentErr()
                 << "The input albedo must have the same dimensions as the DEM.\n");
    }

    // This must be done before the DEM is cropped. This stats is
    // queried from parallel_sfs.
    if (opt.query) {
      vw_out() << "dem_cols, " << full_dem.cols() << "\n";
      vw_out() << "dem_rows, " << full_dem.rows() << "\n";
    }

    // Read the georeference
    vw::cartography::GeoReference geo, albedo_geo;
    loadGeoref(opt, geo, albedo_geo);

    // Adjust the crop win
    opt.crop_win.crop(bounding_box(full_dem));

    // Crop the DEM and georef if requested to given box. Same for albedo.
    // In either case, read the needed portion fully in memory.
    vw::ImageView<double> dem, orig_dem, albedo;
    std::string full_dem_wkt = geo.get_wkt();
    if (!opt.crop_win.empty()) {
      dem = crop(full_dem, opt.crop_win);
      geo = crop(geo, opt.crop_win);
      if (!opt.input_albedo.empty()) {
        albedo = crop(full_albedo, opt.crop_win);
        albedo_geo = crop(albedo_geo, opt.crop_win);
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
      vw_throw(ArgumentErr()
               << "The albedo image must have the same dimensions as the DEM.\n");
    if (geo.get_wkt() != albedo_geo.get_wkt())
      vw::vw_throw(vw::ArgumentErr()
                    << "The input DEM has a different georeference "
                    << "from the input albedo image.\n");

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

    // Read in the camera models and the sun positions
    std::vector<vw::CamPtr> cameras;
    std::vector<vw::Vector3> sunPosition;
    loadCamerasSunPos(opt, dem, dem_nodata_val, geo, cameras, sunPosition);

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

    // This check must happen before loading images but after we know the DEM size
    if ((dem.cols() > 500 || dem.rows() > 500) && !opt.compute_exposures_only &&
        !opt.estim_exposure_haze_albedo && !opt.save_computed_intensity_only)
      vw::vw_out(vw::WarningMessage) << "The input DEM is large and this program "
        << "may run out of memory. Use parallel_sfs instead, with small tiles.\n";

    // This check must be here, after we find the session
    if (opt.stereo_session != "isis" && opt.use_approx_camera_models) {
      vw_out() << "Computing approximate models works only with ISIS cameras. "
               << "Ignoring this option.\n";
      opt.use_approx_camera_models = false;
    }

    // We won't load the full images, just portions restricted
    // to the area we we will compute the DEM.
    int num_images = opt.input_images.size();    
    std::vector<BBox2i> crop_boxes(num_images);

    // The crop box starts as the original image bounding box. We'll shrink it later.
    for (int image_iter = 0; image_iter < num_images; image_iter++) {
      std::string img_file = opt.input_images[image_iter];
      crop_boxes[image_iter] = vw::bounding_box(DiskImageView<float>(img_file));
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
    vw_out() << "DEM grid in x and y in meters: " << gridx << ' ' << gridy << "\n";

    // Find the max DEM height
    double max_dem_height = findMaxDemHeight(opt.model_shadows, dem);

    // Find the mean albedo
    double mean_albedo = findMeanAlbedo(dem, albedo, dem_nodata_val);

    // Declare two vectors for skipped and used images
    std::vector<std::string> skipped_images;
    std::vector<std::string> used_images;
    // reserve the proper size, cannot be more than the number of images
    skipped_images.reserve(num_images);
    used_images.reserve(num_images);
    bool blend_weight_is_ground_weight = false;

    // Assume that haze is 0 to start with. Find the exposure as
    // mean(intensity)/mean(reflectance)/albedo. Use this to compute an
    // initial exposure and decide based on that which images to
    // skip. If the user provided initial exposures and haze, use those, but
    // still go through the motions to find the images to skip.
    // See the intensity formula in calcIntensity().
    // vw_out() << "Computing exposures.\n";
    // TODO(oalexan1): Modularize this
    std::vector<double> local_exposures_vec(num_images, 0), local_haze_vec(num_images, 0);
    for (int image_iter = 0; image_iter < num_images; image_iter++) {

      if (opt.skip_images.find(image_iter) != opt.skip_images.end())
        continue;

      // Sample large DEMs. Keep about 200 row and column samples.
      int sample_col_rate = 0, sample_row_rate = 0;
      asp::calcSampleRates(dem, opt.num_samples_for_estim, sample_col_rate, sample_row_rate);

      vw::ImageView<PixelMask<double>> reflectance, intensity;
      vw::ImageView<double> ground_weight;
      vw::ImageView<Vector2> pq; // no need for these just for initialization

      computeReflectanceAndIntensity(dem, pq, geo,
                                     opt.model_shadows, max_dem_height,
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

      // TODO: Below is not the optimal way of finding the exposure!
      // Find it as the analytical minimum using calculus.
      double imgmean, imgstd, refmean, refstd;
      asp::calcJointStats(intensity, reflectance, imgmean, imgstd, refmean, refstd);
      double haze = 0.0;

      if (imgmean > 0 && refmean > 0) {
        double exposure = imgmean/refmean/mean_albedo;
        local_exposures_vec[image_iter] = exposure;
        local_haze_vec[image_iter] = haze;

        // append used image to used_images list
        used_images.push_back(opt.input_images[image_iter]);
        //vw_out() << "Local DEM estimated exposure for image " << image_iter << ": "
        //          << exposure << "\n";
      } else {
        // Skip images with bad exposure. Apparently there is no good
        // imagery in the area.
        opt.skip_images.insert(image_iter);
        // log out the skipped image path and the image_iter for it
        vw_out() << "Skipped image "
                 << image_iter
                 << ": "
                 << opt.input_images[image_iter]
                 << " with no data for this DEM.\n";
        // append skipped image to skipped_images list
        skipped_images.push_back(opt.input_images[image_iter]);
      }
    }
    // write out skipped and used images lists so long as they are not empty
    // TODO or always write out even if empty
    if (!used_images.empty())
      asp::saveUsedImages(opt.out_prefix, used_images);
    if (!skipped_images.empty())
      asp::saveSkippedImages(opt.out_prefix, skipped_images);

    // Only overwrite the exposures if we don't have them supplied
    if (opt.image_exposures_vec.empty())
      opt.image_exposures_vec = local_exposures_vec;

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

    // TODO(oalexan1): Check if --num-haze-coeffs is non-zero.
    // TODO(oalexan1): This should work even if albedo is not modeled.
    if (opt.estim_exposure_haze_albedo &&
        (opt.float_albedo || opt.num_haze_coeffs > 0))
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
      // all done
      return 0;
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

    // If opt.allow_borderline_data is true, create for each image that will not be skipped
    // a weight matrix with dimensions equal to DEM dimensions, that will be used instead
    // of weights in the camera image space. These are balanced among each other and give more
    // weight to barely lit and unlit nearby pixels.
    std::vector<ImageView<double>> ground_weights(num_images);
    std::vector<vw::ImageView<PixelMask<double>>> meas_intensities(num_images);

    // Note that below we may use the exposures computed at the previous step
    // TODO(oalexan1): This block must be a function.
    if (opt.save_computed_intensity_only || 
        opt.estimate_height_errors || opt.curvature_in_shadow_weight > 0.0 ||
        opt.allow_borderline_data) {
      // Save the computed and actual intensity, and for most of these quit
      vw::ImageView<PixelMask<double>> reflectance, meas_intensity, comp_intensity;
      vw::ImageView<double> ground_weight;
      vw::ImageView<Vector2> pq; // no need for these just for initialization
      int sample_col_rate = 1, sample_row_rate = 1;

      auto heightErrEstim = boost::shared_ptr<HeightErrEstim>(NULL);
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
                              &albedo));
      }

      for (int image_iter = 0; image_iter < num_images; image_iter++) {

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
                                       blend_weight_is_ground_weight,
                                       cameras[image_iter],
                                       reflectance, meas_intensity, ground_weight,
                                       &opt.model_coeffs_vec[0], opt,
                                       heightErrEstim.get());

        if (opt.skip_images.find(image_iter) == opt.skip_images.end() &&
            (opt.allow_borderline_data || opt.low_light_threshold > 0.0)) {
          // if not skipping, save the weight
          ground_weights[image_iter] = copy(ground_weight);
        }
        if (opt.skip_images.find(image_iter) == opt.skip_images.end() &&
            opt.low_light_threshold > 0.0) {
          // if not skipping, save the intensity
          meas_intensities[image_iter] = copy(meas_intensity);
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
                // Boundary pixels are declared lit. Otherwise they are always
                // unlit due to the peculiarities of how the intensity is found
                // at the boundary.
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

      if (opt.estimate_height_errors) {
        // Find the height error from the range of heights
        vw::ImageView<float> height_error;
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

    if (opt.save_computed_intensity_only || opt.estimate_height_errors) {
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
      // TODO(oalexan1): These weights should be created before any calculation
      // of intensity. As of now, they kick after that, and before iterative
      // SfS. Must check the effect of that. Should result in minor changes to
      // exposure only.
      int cols = dem.cols(), rows = dem.rows();
      asp::adjustBorderlineDataWeights(cols, rows, opt.blending_dist, opt.blending_power,
                                       vw::GdalWriteOptions(opt), // slice
                                       geo,
                                       opt.skip_images,
                                       opt.out_prefix, // for debug data
                                       opt.input_images, opt.input_cameras,
                                       ground_weights); // output

      // Use the ground weights from now on instead of in-camera blending weights.
      // Will overwrite the weights below.
      blend_weight_is_ground_weight = true;

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
          vw::ImageView<float> cropped_img =
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
     
      // std::cout << "--temp!\n";
      // asp::saveGroundWeights(opt.skip_images, opt.out_prefix, opt.input_images,
      //                 opt.input_cameras, blend_weights, geo, vw::GdalWriteOptions(opt));
      
    } // end allow borderline data

    vw::ImageView<double> curvature_in_shadow_weight;
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
        masked_images[image_iter] = vw::ImageView<PixelMask<float>>();
        blend_weights[image_iter] = vw::ImageView<double>();
        cameras[image_iter] = boost::shared_ptr<CameraModel>();
      }
    }

    // orig_dem will keep the input DEMs and won't change. Keep to the optimized
    // DEMs close to orig_dem. Make a deep copy below.
    orig_dem = copy(dem);

    run_sfs(// Fixed quantities
            opt.max_iterations, gridx, gridy, opt, geo, opt.smoothness_weight,
            max_dem_height, dem_nodata_val, img_nodata_val,  crop_boxes, masked_images,
            blend_weights, blend_weight_is_ground_weight,
            refl_params, sunPosition, orig_dem,
            lit_image_mask, curvature_in_shadow_weight,
            // Variable quantities
            dem, albedo, cameras, opt.image_exposures_vec, opt.image_haze_vec,
            opt.model_coeffs_vec);

  } ASP_STANDARD_CATCHES;

  sw_total.stop();
  vw_out() << "Total elapsed time: " << sw_total.elapsed_seconds() << " s.\n";

  return 0;
}
