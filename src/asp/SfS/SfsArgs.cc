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

#include <asp/SfS/SfsArgs.h>
#include <asp/Core/AspProgramOptions.h>
#include <asp/Core/AspLog.h>
#include <asp/Core/StereoSettings.h>
#include <asp/SfS/SfsUtils.h>
#include <asp/Core/FileUtils.h>
#include <asp/SfS/SfsModel.h>

#include <vw/FileIO/FileUtils.h>

#include <boost/program_options.hpp>
#include <boost/filesystem.hpp>

namespace po = boost::program_options;
namespace fs = boost::filesystem;

namespace asp {

void handleSfsArgs(int argc, char *argv[], SfsOptions& opt) {
  double nan = std::numeric_limits<double>::quiet_NaN();
  po::options_description general_options("");
  general_options.add_options()
  ("input-dem,i",
  po::value(&opt.input_dem),
   "The input DEM to refine using SfS.")
  ("image-list",
  po::value(&opt.image_list)->default_value(""),
   "A file containing the list of images, when they are too many to specify on the "
   "command line. Use space or newline as separator. See also --camera-list and "
   "--mapprojected-data-list.")
  ("camera-list",
  po::value(&opt.camera_list)->default_value(""),
   "A file containing the list of cameras, when they are too many to specify on the "
   "command line. If the images have embedded camera information, such as for ISIS, "
   "this file must be empty but must be specified if --image-list is specified.")
  ("output-prefix,o",
  po::value(&opt.out_prefix),
   "Prefix for output filenames.")
  ("max-iterations,n",
  po::value(&opt.max_iterations)->default_value(10),
   "Set the maximum number of iterations. Normally 5-10 iterations is enough, even "
   "when convergence is not reached, as the solution usually improves quickly at "
   "first and only very fine refinements happen later.")
  ("reflectance-type",
  po::value(&opt.reflectance_type)->default_value(1),
   "Reflectance type: 0 = Lambertian, 1 = Lunar-Lambert, 2 = Hapke, 3 = Experimental "
   "extension of Lunar-Lambert, 4 = Charon model (a variation of Lunar-Lambert).")
  ("smoothness-weight",
  po::value(&opt.smoothness_weight)->default_value(0.04),
   "The weight given to the cost function term which consists of sums of squares of "
   "second-order derivatives. A larger value will result in a smoother solution with "
   "fewer artifacts. See also --gradient-weight.")
  ("initial-dem-constraint-weight",
  po::value(&opt.initial_dem_constraint_weight)->default_value(0),
   "A larger value will try harder to keep the SfS-optimized DEM closer to the initial "
   "guess DEM. A value between 0.0001 and 0.001 may work, unless your initial DEM is "
   "very unreliable.")
  ("bundle-adjust-prefix",
  po::value(&opt.bundle_adjust_prefix),
   "Use the camera adjustments obtained by previously running bundle_adjust with this "
   "output prefix.")
  ("input-albedo",
  po::value(&opt.input_albedo),
   "The input albedo image, if known. Must have same dimensions as the input DEM. "
   "Otherwise it is initialized to 1. Can be refined with --float-albedo.")
  ("float-albedo",  
  po::bool_switch(&opt.float_albedo)->default_value(false)->implicit_value(true),
   "Float the albedo for each pixel. Will give incorrect results if only one image is "
   "present. The albedo is normalized, its nominal value is 1.")
  ("float-exposure",  
  po::bool_switch(&opt.float_exposure)->default_value(false)->implicit_value(true),
   "Float the exposure for each image. Will give incorrect results if only one image is "
   "present. It usually gives marginal results.")
  ("model-shadows",  
  po::bool_switch(&opt.model_shadows)->default_value(false)->implicit_value(true),
   "Model the fact that some points on the DEM are in the shadow (occluded from the Sun).")
  ("save-sim-intensity-only",  
  po::bool_switch(&opt.save_sim_intensity_only)->default_value(false)->implicit_value(true),
   "Save the simulated image intensities at each DEM pixel for the given DEM, images, "
   "cameras, and reflectance model, without refining the DEM. The output files are "
   "of the form <output prefix>-*-sim-intensity.tif. The image exposures "
   "will be computed along the way unless specified via --image-exposures-prefix, and "
   "will be saved in either case to <output prefix>-exposures.txt. Same for haze, if "
   "applicable. See also: --save-meas-intensity-only.")
  ("save-meas-intensity-only",
  po::bool_switch(&opt.save_meas_intensity_only)->default_value(false)->implicit_value(true),
    "Save the measured image intensities at each DEM pixel for the given DEM, "
    "images, and cameras, without refining the DEM. The output files are "
    "of the form <output prefix>-*-meas-intensity.tif. See also: "
    "--save-sim-intensity-only.")
  ("ref-map", po::value(&opt.ref_map)->default_value(""),
    "Save the simulated or measured intensity images to the extent given by this "
    "mapprojected image. For use with --save-sim-intensity-only and "
    "--save-meas-intensity-only.")
  ("estimate-exposure-haze-albedo",
  po::bool_switch(&opt.estim_exposure_haze_albedo)->default_value(false)->implicit_value(true),
   "Estimate the exposure for each image, the haze for each image (if --num-haze-coeffs "
   "is positive), and the global low-resolution albedo (if --float-albedo is on), then "
   "quit. This operation samples the input DEM based on --num-samples-for-estim. The "
   "produced estimated exposure, haze, and initial albedo are described in the doc. This "
   "is invoked automatically by parallel_sfs before running sfs proper, unless these "
   "quantities are provided as inputs.")
  ("compute-exposures-only",
  po::bool_switch(&opt.compute_exposures_only)->default_value(false)->implicit_value(true),
   "This older option is equivalent to --estimate-exposure-haze-albedo.")
  ("estimate-height-errors",  
  po::bool_switch(&opt.estimate_height_errors)->default_value(false)->implicit_value(true),
   "Estimate the SfS DEM height uncertainty by finding the height perturbation (in "
   "meters) at each grid point which will make at least one of the simulated images at "
   "that point change by more than twice the discrepancy between the unperturbed "
   "simulated image and the measured image. The SfS DEM must be provided via the -i "
   "option. The number of iterations, blending parameters (--blending-dist, etc.), and "
   "smoothness weight are ignored. Results are not computed at image pixels in shadow. "
   "This produces <output prefix>-height-error.tif. No SfS DEM is computed. See also: "
   "--height-error-params.")
  ("height-error-params", 
  po::value(&opt.height_error_params)->default_value(vw::Vector2(5.0, 100.0), "5.0 100"),
   "Specify the largest height deviation to examine (in meters), and how many samples to use from 0 to that height.")
  ("sun-positions",
  po::value(&opt.sun_positions_list)->default_value(""),
   "A file having on each line an image name and three values in double precision "
   "specifying the Sun position in meters in ECEF coordinates (origin is planet center). "
   "Use a space as separator. If not provided, these will be read from the camera files "
   "for ISIS and CSM models.")
  ("sun-angles",
  po::value(&opt.sun_angles_list)->default_value(""),
   "A file having on each line an image name and two values in double precision "
   "specifying the Sun azimuth and elevation in degrees, relative to the center point of "
   "the input DEM. Use a space as separator. This is an alternative to --sun-positions.")
  ("shadow-thresholds",
  po::value(&opt.shadow_thresholds)->default_value(""),
   "Optional shadow thresholds for the input images (a list of real values in quotes, "
   "one per image). See also --shadow-threshold.")
  ("shadow-threshold",
  po::value(&opt.shadow_threshold)->default_value(-1),
   "A shadow threshold to apply to all images. Must be positive. Areas that "
   "are in shadow in all images will result in a blurred version of the input DEM, "
   "influenced by the --smoothness-weight.")
  ("custom-shadow-threshold-list", 
  po::value(&opt.custom_shadow_threshold_list)->default_value(""),
   "A list having one image and one shadow threshold per line. For the images specified "
   "here, override the shadow threshold supplied by other means with this value.")
  ("low-light-threshold", 
  po::value(&opt.low_light_threshold)->default_value(-1.0),
   "A threshold for low-light pixels. If positive, pixels with intensity between "
   "this and the shadow threshold will be given less weight, if other images have higher "
   "intensity values at the same ground point. This helps fix seams. See also "
   "--low-light-weight-power and --low-light-blur-sigma.")
  ("low-light-weight-power", 
  po::value(&opt.low_light_weight_power)->default_value(4.0),
   "With the option --low-light-threshold, the weight of a low-light pixel is inversely "
   "proportional with the discrepancy between the simulated and observed pixel value, "
   "raised to this power.")
  ("low-light-blur-sigma", 
  po::value(&opt.low_light_blur_sigma)->default_value(3.0),
   "With the option --low-light-threshold, apply a Gaussian blur with this sigma to the "
   "low-light weight image, to make it continuous.")
  ("erode-seams",
  po::bool_switch(&opt.erode_seams)->default_value(false)->implicit_value(true),
   "Be more aggressive in removing seam artifacts, even if this results in erosion of valid "
   "terrain.")
  ("max-valid-image-vals",
  po::value(&opt.max_valid_image_vals)->default_value(""),
   "Optional values for the largest valid image value in each image (a list of real "
   "values in quotes, one per image).")
  ("robust-threshold",
  po::value(&opt.robust_threshold)->default_value(-1.0),
   "If positive, set the threshold for the robust measured-to-simulated intensity "
   "difference (using the Cauchy loss). Any difference much larger than this will be "
   "penalized. A good value may be 5% to 25% of the average image value or the same "
   "fraction of the computed image exposure values.")
  ("albedo-constraint-weight", 
  po::value(&opt.albedo_constraint_weight)->default_value(0),
   "If floating the albedo, a larger value will try harder to keep the optimized albedo "
   "close to the initial albedo. See also --input-albedo and --albedo-robust-threshold.")
  ("albedo-robust-threshold",
  po::value(&opt.albedo_robust_threshold)->default_value(0),
   "If floating the albedo and this threshold is positive, apply a Cauchy loss with this "
   "threshold to the product of the albedo difference and the albedo constraint weight.")
  ("skip-images",
  po::value(&opt.skip_images_str)->default_value(""), 
   "Skip images with these indices (indices start from 0).")
  ("save-dem-with-nodata",  
  po::bool_switch(&opt.save_dem_with_nodata)->default_value(false)->implicit_value(true),
   "Save a copy of the DEM while using a no-data value at a DEM grid point where all "
   "images show shadows. To be used if shadow thresholds are set.")
  ("use-approx-camera-models",  
  po::bool_switch(&opt.use_approx_camera_models)->default_value(false)->implicit_value(true),
   "Use approximate camera models for speed. Only with ISIS .cub cameras.")
  ("crop-input-images",  
  po::bool_switch(&opt.crop_input_images)->default_value(false)->implicit_value(true),
   "Crop the images to a region that was computed to be large enough, and keep them "
   "fully in memory, for speed. This is the default in the latest builds.")
  ("blending-dist",
  po::value(&opt.blending_dist)->default_value(0),
   "Give less weight to image pixels close to no-data or boundary values. Enabled only "
   "when crop-input-images is true, for performance reasons. Blend over this many pixels.")
  ("blending-power",
  po::value(&opt.blending_power)->default_value(2.0),
   "A higher value will result in smoother blending.")
  ("min-blend-size",
  po::value(&opt.min_blend_size)->default_value(0),
   "Do not apply blending in shadowed areas for which both the width and height are less "
   "than this.")
  ("allow-borderline-data",  
  po::bool_switch(&opt.allow_borderline_data)->default_value(false)->implicit_value(true),
   "At the border of the region where there are no lit pixels in any images, do not let "
   "the blending weights decay to 0. This noticeably improves the level of detail. The "
   "sfs_blend tool may need to be used to further tune this region.")
  ("steepness-factor",
  po::value(&opt.steepness_factor)->default_value(1.0),
   "Try to make the terrain steeper by this factor. This is not recommended in regular use.")
  ("curvature-in-shadow",
  po::value(&opt.curvature_in_shadow)->default_value(0.0),
   "Attempt to make the curvature of the DEM (the Laplacian) at points in shadow in all "
   "images equal to this value, which should make the DEM curve down.")
  ("curvature-in-shadow-weight", 
  po::value(&opt.curvature_in_shadow_weight)->default_value(0.0),
   "The weight to give to the curvature in shadow constraint.")
  ("lit-curvature-dist",
  po::value(&opt.lit_curvature_dist)->default_value(0.0),
   "If using a curvature in shadow, start phasing it in this far from the shadow boundary "
   "in the lit region (in units of pixels).")
  ("shadow-curvature-dist",
  po::value(&opt.shadow_curvature_dist)->default_value(0.0),
   "If using a curvature in shadow, have it fully phased in this far from shadow boundary "
   "in the shadow region (in units of pixels).")
  ("image-exposures-prefix",
  po::value(&opt.image_exposures_prefix)->default_value(""),
   "Use this prefix to optionally read initial exposures (filename is "
   "<prefix>-exposures.txt).")
  ("model-coeffs-prefix",
  po::value(&opt.model_coeffs_prefix)->default_value(""),
   "Use this prefix to optionally read model coefficients from a file (filename is <prefix>-model_coeffs.txt).")
  ("model-coeffs",
  po::value(&opt.model_coeffs)->default_value(""),
   "Use the reflectance model coefficients specified as a list of numbers in quotes. "
   "Lunar-Lambertian: O, A, B, C, e.g., '1 -0.019 0.000242 -0.00000146'. Hapke: omega, b, "
   "c, B0, h, e.g., '0.68 0.17 0.62 0.52 0.52'. Charon: A, f(alpha), e.g., '0.7 0.63'.")
  ("num-haze-coeffs",
  po::value(&opt.num_haze_coeffs)->default_value(0),
   "Set this to 1 to model the problem as image = exposure * albedo * reflectance + "
   "haze, where haze is a single value for each image.")
  ("float-haze",
  po::bool_switch(&opt.float_haze)->default_value(false)->implicit_value(true),
   "If specified, float the haze coefficients as part of the optimization, if "
   "--num-haze-coeffs is 1.")
  ("haze-prefix",
  po::value(&opt.image_haze_prefix)->default_value(""),
   "Use this prefix to read initial haze values (filename is <haze-prefix>-haze.txt). "
   "The file format is the same as what the tool writes itself, when triggered by the "
   "earlier options. If haze is modeled, it will be initially set to 0 unless read from "
   "such a file, and will be floated or not depending on whether --float-haze is on. "
   "The final haze values will be saved to <output prefix>-haze.txt.")
  ("num-samples-for-estim",
  po::value(&opt.num_samples_for_estim)->default_value(200),
   "Number of samples to use for estimating the exposure, haze, and albedo. A large "
   "value will result in a more accurate estimate, but will take a lot more memory.")
  ("init-dem-height",
  po::value(&opt.init_dem_height)->default_value(nan),
   "Use this value for initial DEM heights (measured in meters, relative to the datum). "
   "An input DEM still needs to be provided for georeference information.")
  ("crop-win", 
  po::value(&opt.crop_win)->default_value(vw::BBox2i(0, 0, 0, 0), "xoff yoff xsize ysize"),
   "Crop the input DEM to this region before continuing.")
  ("nodata-value",
  po::value(&opt.nodata_val)->default_value(nan),
   "Use this as the DEM no-data value, over-riding what is in the initial guess DEM.")
  ("fix-dem",  
  po::bool_switch(&opt.fix_dem)->default_value(false)->implicit_value(true),
   "Do not float the DEM at all. Useful when floating the model params.")
  ("read-exposures",
  po::bool_switch(&opt.read_exposures)->default_value(false)->implicit_value(true),
   "If specified, read the image exposures with the current output prefix. Useful with a "
   "repeat invocation.")
  ("read-haze",
  po::bool_switch(&opt.read_haze)->default_value(false)->implicit_value(true),
   "If specified, read the haze values with the current output prefix.")
  ("read-albedo",
  po::bool_switch(&opt.read_albedo)->default_value(false)->implicit_value(true),
   "If specified, read the computed albedo with the current output prefix.")
  ("float-reflectance-model",  
  po::bool_switch(&opt.float_reflectance_model)->default_value(false)->implicit_value(true),
   "Allow the coefficients of the reflectance model to float (not recommended).")
  ("integrability-constraint-weight", 
  po::value(&opt.integrability_weight)->default_value(0.0),
   "Use the integrability constraint from Horn 1990 with this value of its weight.")
  ("smoothness-weight-pq",
  po::value(&opt.smoothness_weight_pq)->default_value(0.00),
   "Smoothness weight for p and q, when the integrability constraint "
   "is used. A larger value will result in a smoother solution "
   "(experimental).")
  ("query",  
  po::bool_switch(&opt.query)->default_value(false)->implicit_value(true),
   "Print some info and exit. Invoked from parallel_sfs.")
  ("session-type,t",
  po::value(&opt.stereo_session)->default_value(""),
   "Select the stereo session type to use for processing. Usually the program can select "
   "this automatically by the file extension, except for xml cameras. See the doc for "
   "options.")
  ("gradient-weight", 
  po::value(&opt.gradient_weight)->default_value(0.0),
   "The weight given to the cost function term which consists of sums of squares of "
   "first-order derivatives. A larger value will result in shallower slopes but less "
   "noise. This can be used in conjunction with --smoothness-weight. It is suggested to "
   "experiment with this with a value of 0.0001 - 0.01, while reducing the smoothness "
   "weight to a very small value.")
  ("save-sparingly",  
  po::bool_switch(&opt.save_sparingly)->default_value(false)->implicit_value(true),
   "Avoid saving any results except the adjustments and the DEM, as that's a lot of files.")
  ("save-variances",
  po::bool_switch(&opt.save_variances)->default_value(false)->implicit_value(true),
   "Save the variance of the DEM for each pixel. If --float-albedo is on, also save the "
   "variance of the albedo. Note that computing the albedo variance can be ill-posed if "
   "--float-haze and/or --float-exposure is also on.")
  ("save-covariances",
  po::bool_switch(&opt.save_covariances)->default_value(false)->implicit_value(true),
   "In addition to saving the variance of the DEM (and albedo) at each pixel (as for "
   "--save-variances), also save the covariance between each DEM pixel and its four "
   "horizontal and vertical neighbors, and the same for the albedo if --float-albedo "
   "is on.")
  ("camera-position-step-size",
  po::value(&opt.camera_position_step_size)->default_value(1.0),
   "Larger step size will result in more aggressiveness in varying the camera position "
   "if it is being floated (which may result in a better solution or in divergence).");

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
    vw::vw_throw(vw::ArgumentErr() << "Missing the input DEM.\n");

  // Always crop the input images. This has been validated enough that it works well.
  // There is no reason to fully load images in memory except for computing exposures,
  // when this option will be disabled automatically.
  opt.crop_input_images = true;

  // Separate the cameras from the images
  std::vector<std::string> inputs = opt.input_images;

  if (!opt.image_list.empty()) {
    // Read the images and cameras and put them in 'inputs' to be parsed later
    if (opt.camera_list.empty())
      vw::vw_throw(vw::ArgumentErr()
               << "The option --image-list must be invoked together with --camera-list.\n");
    if (!inputs.empty())
      vw::vw_throw(vw::ArgumentErr() << "The option --image-list was specified, but also "
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
    vw::vw_throw(vw::ArgumentErr() << "Missing output prefix.\n"
              << usage << general_options);

  if (opt.max_iterations < 0)
    vw::vw_throw(vw::ArgumentErr() << "The number of iterations must be non-negative.\n"
              << usage << general_options);

  if (opt.input_images.empty())
    vw::vw_throw(vw::ArgumentErr() << "Missing input images.\n"
              << usage << general_options);

  if (opt.smoothness_weight < 0)
    vw::vw_throw(vw::ArgumentErr() << "Expecting a non-negative smoothness weight.\n");

  if (opt.gradient_weight < 0)
    vw::vw_throw(vw::ArgumentErr() << "Expecting a non-negative gradient weight.\n");

  if (opt.integrability_weight < 0)
    vw::vw_throw(vw::ArgumentErr() << "Expecting a non-negative integrability weight.\n");

  if (opt.float_haze && opt.num_haze_coeffs == 0)
    vw::vw_throw(vw::ArgumentErr() << "Haze cannot be floated unless there is at least one haze coefficient.\n");
  if (opt.image_haze_prefix != "" && opt.num_haze_coeffs == 0)
    vw::vw_throw(vw::ArgumentErr() << "Haze cannot be read unless there is at least one haze coefficient.\n");

  // There can be 0 or 1 haze coefficients. The modeling of more than one haze
  // coefficient needs to be looked into.
  if (opt.num_haze_coeffs < 0 || opt.num_haze_coeffs > g_max_num_haze_coeffs)
    vw::vw_throw(vw::ArgumentErr() << "Expecting up to " << g_max_num_haze_coeffs
             << " haze coefficients.\n");

  // Curvature in shadow params
  if (opt.curvature_in_shadow < 0.0 || opt.curvature_in_shadow_weight < 0.0)
    vw::vw_throw(vw::ArgumentErr() << "Cannot have negative curvature in shadow or its weight.\n");
  if (opt.lit_curvature_dist < 0.0 || opt.shadow_curvature_dist < 0.0)
    vw::vw_throw(vw::ArgumentErr() << "Cannot have negative curvature distances.\n");
  if (opt.curvature_in_shadow > 0.0 &&
      opt.shadow_curvature_dist + opt.lit_curvature_dist <= 0.0)
    vw::vw_throw(vw::ArgumentErr() << "When modeling curvature in shadow, expecting a "
             << "positive value of shadow-curvature-dist or list-curvature-dist.\n");

  if (opt.steepness_factor <= 0.0)
    vw::vw_throw(vw::ArgumentErr() << "The steepness factor must be positive.\n");

  // The options --compute-exposures-only and --estimate-exposure-haze-albedo
  // are equivalent.
  if (opt.compute_exposures_only)
    opt.estim_exposure_haze_albedo = true;
  if (opt.estim_exposure_haze_albedo)
    opt.compute_exposures_only = true;

  if (opt.compute_exposures_only || opt.estim_exposure_haze_albedo) {
    if (opt.use_approx_camera_models || opt.crop_input_images) {
      vw::vw_out(vw::WarningMessage) 
        << "When computing exposures only, not using approximate camera models or "
        << "cropping input images.\n";
      // Turn off various settings that are needed only when doing iterations  
      opt.use_approx_camera_models = false;
      opt.crop_input_images = false;
      opt.blending_dist = 0;
      opt.allow_borderline_data = false;
      opt.low_light_threshold = -1.0;
      opt.erode_seams = false;
    }

    if (!opt.crop_win.empty()) {
      vw::vw_throw(vw::ArgumentErr() << "When computing exposures only, cannot crop the "
               << "input DEM as that will give wrong results. Use the full DEM.\n");
    }
  }

  if (opt.blending_dist > 0 && !opt.crop_input_images)
    vw::vw_throw(vw::ArgumentErr()
             << "A blending distance is only supported with --crop-input-images.\n");

  if (opt.allow_borderline_data && !opt.crop_input_images)
    vw::vw_throw(vw::ArgumentErr() << "Option --allow-borderline-data needs option "
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
    vw::vw_throw(vw::ArgumentErr()
             << "If specified, there must be as many shadow thresholds as images.\n");

  // See if to use opt.shadow_threshold.
  if (opt.shadow_threshold > 0) {
    if (!opt.shadow_threshold_vec.empty())
      vw::vw_throw(vw::ArgumentErr()
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
    while (ifs >> image >> val)
      custom_thresh[image] = val;

    if (custom_thresh.empty())
      vw::vw_throw(vw::ArgumentErr() << "Could not read any data from: "
               << opt.custom_shadow_threshold_list << "\n");

    for (size_t it = 0; it < opt.input_images.size(); it++) {
      auto key = custom_thresh.find(opt.input_images[it]);
      if (key != custom_thresh.end()) {
        vw::vw_out() << "Over-riding the shadow threshold for " << opt.input_images[it]
                 << " with: " << key->second << std::endl;
        opt.shadow_threshold_vec[it] = key->second;
      }
    }
  }
  
  // If the low-light-threshold is specified, ensure it is above all shadow thresholds,
  // and that the shadow thresholds are positive.
  if (opt.low_light_threshold > 0) {
    for (size_t i = 0; i < opt.input_images.size(); i++) {
      if (opt.shadow_threshold_vec[i] < 0)
        vw::vw_throw(vw::ArgumentErr() << "When --low-light-threshold is set, all shadow thresholds "
                 << "must be set and non-negative.\n");
      if (opt.low_light_threshold <= opt.shadow_threshold_vec[i])
        vw::vw_throw(vw::ArgumentErr() << "The low-light-threshold must be larger than all "
                 << "shadow thresholds.\n");
    }
    if (!opt.allow_borderline_data)
      vw::vw_throw(vw::ArgumentErr()
        << "When using --low-light-threshold, must set --allow-borderline-data.\n");
  }
        
  if (opt.low_light_weight_power <= 0.0)
    vw::vw_throw(vw::ArgumentErr() << "Expecting a positive --low-light-weight-power.\n");
  if (opt.low_light_blur_sigma <= 0.0)
    vw::vw_throw(vw::ArgumentErr() << "Expecting a positive --low-light-blur-sigma.\n");
  if (opt.erode_seams && opt.low_light_threshold <= 0.0)
   vw::vw_throw(vw::ArgumentErr()
     << "When using --erode-seams, must set a positive --low-light-threshold.\n");
   
  // Parse max valid image vals
  std::istringstream ism(opt.max_valid_image_vals);
  opt.max_valid_image_vals_vec.clear();
  while (ism >> val)
    opt.max_valid_image_vals_vec.push_back(val);
  if (!opt.max_valid_image_vals_vec.empty() &&
      opt.max_valid_image_vals_vec.size() != opt.input_images.size())
    vw::vw_throw(vw::ArgumentErr()
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
    vw::vw_throw(vw::ArgumentErr()
             << "Cannot specify both --haze-prefix and --read-haze.\n");
  // If have opt.read_haze, use the current prefix
  if (opt.read_haze)
    opt.image_haze_prefix = opt.out_prefix;

  // Same for albedo
  if (opt.input_albedo != "" && opt.read_albedo)
    vw::vw_throw(vw::ArgumentErr()
             << "Cannot specify both --input-albedo and --read-albedo.\n");
  // If have opt.read_albedo, use the current prefix albedo
  if (opt.read_albedo)
    opt.input_albedo = opt.out_prefix + "-albedo-final.tif";

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
    vw::vw_out() << "Reading exposures from: " << exposure_file << std::endl;
    for (size_t i = 0; i < opt.input_images.size(); i++) {
      std::string img = opt.input_images[i];
      std::map<std::string, double>::iterator it = img2exp.find(img);
      if (it == img2exp.end()) {
        vw::vw_throw(vw::ArgumentErr()
                 << "Could not find the exposure for image: " << img << ".\n");
      }
      double exp_val = it->second;
      // vw::vw_out() << "Exposure for " << img << ": " << exp_val << std::endl;
      opt.image_exposures_vec.push_back(exp_val);
    }
  }
  if (opt.image_exposures_prefix != "" && exp_count == 0)
    vw::vw_throw(vw::ArgumentErr()
             << "Could not find the exposures file: " << exposure_file << ".\n");

  if (opt.steepness_factor != 1.0)
    vw::vw_out() << "Making the terrain artificially steeper by factor: " << opt.steepness_factor
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
          vw::vw_throw(vw::ArgumentErr()
                   << "Found unexpected non-zero haze coefficient: " << haze_vec[hiter] << ".\n");
      }
    }
    ish.close();
    if (opt.image_haze_prefix != "" && haze_count == 0)
      vw::vw_throw(vw::ArgumentErr()
               << "Could not find the haze file: " << haze_file << ".\n");

    if (haze_count > 0) {
      vw::vw_out() << "Reading haze file: " << haze_file << std::endl;
      for (size_t i = 0; i < opt.input_images.size(); i++) {
        std::string img = opt.input_images[i];
        std::map< std::string, std::vector<double>>::iterator it = img2haze.find(img);
        if (it == img2haze.end()) {
          vw::vw_throw(vw::ArgumentErr()
                   << "Could not find the haze for image: " << img << ".\n");
        }
        std::vector<double> haze_vec = it->second;
        // vw::vw_out() << "Haze for " << img << ":";
        //for (size_t hiter = 0; hiter < haze_vec.size(); hiter++)
        //  vw::vw_out() << " " << haze_vec[hiter];
        //vw::vw_out() << "\n";
        opt.image_haze_vec.push_back(haze_vec);
      }
    }
  }

  // Initial model coeffs, if passed on the command line
  if (opt.model_coeffs != "") {
    vw::vw_out() << "Parsing model coefficients: " << opt.model_coeffs << std::endl;
    std::istringstream is(opt.model_coeffs);
    double val;
    while (is >> val)
      opt.model_coeffs_vec.push_back(val);
  }

  // Initial model coefficients, if provided in the file
  if (opt.model_coeffs_prefix != "") {
    std::string model_coeffs_file = asp::modelCoeffsFileName(opt.model_coeffs_prefix);
    vw::vw_out() << "Reading model coefficients from file: " << model_coeffs_file << std::endl;
    std::ifstream ism(model_coeffs_file.c_str());
    opt.model_coeffs_vec.clear();
    while (ism >> dval)
      opt.model_coeffs_vec.push_back(dval);
    ism.close();
    if (opt.model_coeffs_vec.empty()) {
      vw::vw_throw(vw::ArgumentErr() << "Could not read model coefficients from: " << model_coeffs_file << ".\n");
    }
  }

  if (!opt.model_coeffs_vec.empty()) {
    // Pad with zeros if needed, as the Lunar Lambertian has 4 params, while Hapke has 5 of them.
    // the Charon one has 2.
    while (opt.model_coeffs_vec.size() < g_num_model_coeffs)
      opt.model_coeffs_vec.push_back(0);

    if (opt.model_coeffs_vec.size() != g_num_model_coeffs)
      vw::vw_throw(vw::ArgumentErr()
               << "If specified, there must be " << g_num_model_coeffs << " coefficients.\n");
  }

  // Sanity check
  if (opt.camera_position_step_size <= 0) {
    vw::vw_throw(vw::ArgumentErr() << "Expecting a positive value for camera-position-step-size.\n");
  }

  // Need this to be able to load adjusted camera models. That will happen
  // in the stereo session.
  asp::stereo_settings().bundle_adjust_prefix = opt.bundle_adjust_prefix;

  if (opt.input_images.size() <= 1 && opt.float_albedo &&
      opt.initial_dem_constraint_weight <= 0 && opt.albedo_constraint_weight <= 0.0)
    vw::vw_throw(vw::ArgumentErr()
             << "Floating the albedo is ill-posed for just one image without "
             << "the initial DEM constraint or the albedo constraint.\n");

  if (opt.input_images.size() <= 1 && opt.float_exposure &&
      opt.initial_dem_constraint_weight <= 0)
    vw::vw_throw(vw::ArgumentErr()
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
    vw::vw_throw(vw::ArgumentErr()
       << "Cannot estimate height errors when using the integrability constraint.\n");

  if (opt.estimate_height_errors && opt.model_shadows)
    vw::vw_throw(vw::ArgumentErr() << "Cannot estimate height error when modeling shadows.");

  if (opt.save_sim_intensity_only || opt.save_meas_intensity_only || 
      opt.estimate_height_errors) {

    // No iterations
    opt.max_iterations = 0;

    // Need the exact cameras as they span the full DEM
    if (opt.use_approx_camera_models || opt.crop_input_images) {
      opt.use_approx_camera_models = false;
      opt.crop_input_images = false;
    }

    if (opt.num_haze_coeffs > 0 && opt.image_haze_vec.empty())
      vw::vw_throw(vw::ArgumentErr()
                << "Expecting the haze to be computed and passed in.\n");
  }

  // Cannot have both sun positions and sun angles
  if (opt.sun_positions_list.size() > 0 && opt.sun_angles_list.size() > 0)
    vw::vw_throw(vw::ArgumentErr() << "Cannot specify both sun positions and sun angles.\n");
    
  if (!opt.ref_map.empty()) {
    // --ref-map is to be used only with --save-sim-intensity-only or
    // --save-meas-intensity-only
    if (!opt.save_sim_intensity_only && !opt.save_meas_intensity_only)
      vw::vw_throw(vw::ArgumentErr()
               << "--ref-map is to be used only with --save-sim-intensity-only or "
               << "--save-meas-intensity-only.\n");
      
     // This is also incompatible with non-empty crop win
     if (!opt.crop_win.empty())
      vw::vw_throw(vw::ArgumentErr()
               << "--ref-map is incompatible with --crop-win.\n");
      
  }

  // If --save-covariances is on, also turn on --save-variances
  if (opt.save_covariances)
    opt.save_variances = true;

  // If --save-variances is on and --float-albedo is on, warn if either --float-haze
  // or --float-exposures is also on, as this may make the albedo variance ill-posed.
  if (opt.save_variances && opt.float_albedo) {
    if (opt.float_haze || opt.float_exposure)
      vw::vw_out(vw::WarningMessage)
        << "Computing the albedo variance may be ill-posed when "
        << "floating haze or exposures.\n";
  }
  
} // end function handleSfsArgs
    
} // end namespace asp
