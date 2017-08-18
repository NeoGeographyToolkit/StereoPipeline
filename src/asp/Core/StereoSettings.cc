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


/// \file StereoSettings.h
///
#include <fstream>

#include <vw/Core/RunOnce.h>
#include <vw/Core/Log.h>

#include <asp/Core/Common.h>
#include <asp/Core/StereoSettings.h>

namespace po = boost::program_options;
using namespace vw;

// Create a single instance of the StereoSettings
// ---------------------------------------------------
namespace {
  vw::RunOnce stereo_settings_once = VW_RUNONCE_INIT;
  boost::shared_ptr<asp::StereoSettings> stereo_settings_ptr;
  void init_stereo_settings() {
    stereo_settings_ptr = boost::shared_ptr<asp::StereoSettings>(new asp::StereoSettings());
  }
}

namespace asp {

  StereoSettings& stereo_settings() {
    stereo_settings_once.run( init_stereo_settings );

    // Ensure StereoSettings are initialized
    if ( !(*stereo_settings_ptr).initialized_stereo_settings) {
      // This call must happen first, otherwise we get into infinite
      // recursion.
      stereo_settings_ptr->initialized_stereo_settings = true;

      // Initialize the members of StereoSettings().
      vw::cartography::GdalWriteOptions opt;
      stereo_settings_ptr->initialize(opt);
    }

    return *stereo_settings_ptr;
  }

  void StereoSettings::initialize(vw::cartography::GdalWriteOptions & opt){
    // This is a bug fix. Ensure that all members of this class as
    // well as opt itself are always initialized before using them,
    // whether in stereo, other ASP applications, or the unit
    // tests. What is going on below is the following. The call to
    // generate_config_file_options() specifies what the values of the
    // class members should be, then we parse and initialize them when
    // po::store is invoked. Note that we don't override any of the
    // options from the command line (that's done much later), hence
    // below we use empty command line options.

    po::options_description l_opts("");
    l_opts.add( asp::generate_config_file_options( opt ) );
    po::variables_map l_vm;
    try {
      int l_argc = 1; const char* l_argv[] = {""};
      po::store( po::command_line_parser( l_argc, l_argv ).options(l_opts).style( po::command_line_style::unix_style ).run(), l_vm );
      po::notify( l_vm );
    } catch (po::error const& e) {
      vw::vw_throw( vw::ArgumentErr() << "Error parsing input:\n"
                    << e.what() << "\n" << l_opts );
    }
  }

  StereoSettings::StereoSettings(){

    // This var will turn to true once all stereo settings were parsed
    // and all the members of this class are initialized.
    initialized_stereo_settings = false;

    // Must initialize this variable as it is used in mapproject
    // to get a camera pointer, and there we don't parse stereo.default
    disable_correct_velocity_aberration = false;

    double nan = std::numeric_limits<double>::quiet_NaN();
    nodata_value = nan;
  }

  // Define our options that are available
  // ----------------------------------------------------------
  PreProcessingDescription::PreProcessingDescription() : po::options_description("Preprocessing Options") {

    StereoSettings& global = stereo_settings();
    double nan = std::numeric_limits<double>::quiet_NaN();

    (*this).add_options()
      ("alignment-method",         po::value(&global.alignment_method)->default_value("affineepipolar"),
                     "Rough alignment for input images. [AffineEpipolar, Homography, Epipolar, None]")
      ("left-image-crop-win", po::value(&global.left_image_crop_win)->default_value(BBox2i(0, 0, 0, 0), "xoff yoff xsize ysize"),
                      "Do stereo in a subregion of the left image [default: use the entire image].")
      ("right-image-crop-win", po::value(&global.right_image_crop_win)->default_value(BBox2i(0, 0, 0, 0), "xoff yoff xsize ysize"),
                      "Do stereo in a subregion of the right image if specified together with left-image-crop-win [default: use the entire image].")
      ("force-use-entire-range",   po::bool_switch(&global.force_use_entire_range)->default_value(false)->implicit_value(true),
                     "Normalize images based on the global min and max values from both images. Don't use this option if you are using normalized cross correlation.")
      ("individually-normalize",   po::bool_switch(&global.individually_normalize)->default_value(false)->implicit_value(true),
                     "Individually normalize the input images between 0.0-1.0 using +- 2.5 sigmas about their mean values.")
      ("ip-per-tile",              po::value(&global.ip_per_tile)->default_value(0),
                     "How many interest points to detect in each 1024^2 image tile (default: automatic determination).")
      ("ip-detect-method",          po::value(&global.ip_matching_method)->default_value(0),
       "Interest point detection algorithm (0: Integral OBALoG (default), 1: OpenCV SIFT, 2: OpenCV ORB.")
      ("epipolar-threshold",       po::value(&global.epipolar_threshold)->default_value(-1),
                     "Maximum distance from the epipolar line to search for IP matches. Default: automatic calculation.")
      ("ip-inlier-factor",          po::value(&global.ip_inlier_factor)->default_value(1.0/15.0),
       " A higher factor will result in more interest points, but perhaps also more outliers.")
      ("ip-uniqueness-threshold",          po::value(&global.ip_uniqueness_thresh)->default_value(0.7),
       "Min percentage distance between closest and second closest IP descriptors, a larger value allows more IP matches.")
      ("num-obalog-scales",              po::value(&global.num_scales)->default_value(-1),
       "How many scales to use if detecting interest points with OBALoG. If not specified, 8 will be used. More can help for images with high frequency artifacts.")
      ("nodata-value",             po::value(&global.nodata_value)->default_value(nan),
       "Pixels with values less than or equal to this number are treated as no-data. This overrides the no-data values from input images.")
      ("nodata-pixel-percentage",  po::value(&global.nodata_pixel_percentage)->default_value(nan),
       "The percentage of (low-value) pixels treated as no-data (use a number between 0 and 100).")
      ("nodata-optimal-threshold-factor", po::value(&global.nodata_optimal_threshold_factor)->default_value(nan),
                     "Pixels with values less than this factor times the optimal Otsu threshold are treated as no-data. Suggested value: 0.1 to 0.2.")
      ("skip-rough-homography", po::bool_switch(&global.skip_rough_homography)->default_value(false)->implicit_value(true),
       "Skip the step of performing datum-based rough homography if it fails.")
      ("skip-image-normalization", po::bool_switch(&global.skip_image_normalization)->default_value(false)->implicit_value(true),
       "Skip the step of normalizing the values of input images and removing nodata-pixels. Create instead symbolic links to original images.")
      ("part-of-multiview-run", po::bool_switch(&global.part_of_multiview_run)->default_value(false)->implicit_value(true),
       "If the current run is part of a larger multiview run.")
      ("datum",                    po::value(&global.datum)->default_value("WGS_1984"),
       "Set the datum to use with RPC camera models. Options: WGS_1984, D_MOON (1,737,400 meters), D_MARS (3,396,190 meters), MOLA (3,396,000 meters), NAD83, WGS72, and NAD27. Also accepted: Earth (=WGS_1984), Mars (=D_MARS), Moon (=D_MOON).");
  }

  CorrelationDescription::CorrelationDescription() : po::options_description("Correlation Options") {
    StereoSettings& global = stereo_settings();
    (*this).add_options()
      ("prefilter-kernel-width", po::value(&global.slogW)->default_value(1.5),
                     "Sigma value for Gaussian kernel used in prefilter for correlator.")
      ("prefilter-mode",         po::value(&global.pre_filter_mode)->default_value(2),
                     "Preprocessing filter mode. [0 None, 1 Gaussian, 2 LoG, 3 Sign of LoG]")
      ("corr-seed-mode",         po::value(&global.seed_mode)->default_value(1),
                     "Correlation seed strategy. [0 None, 1 Use low-res disparity from stereo, 2 Use low-res disparity from provided DEM (see disparity-estimation-dem), 3 Use low-res disparity produced by sparse_disp (in development)]")
      ("corr-sub-seed-percent",  po::value(&global.seed_percent_pad)->default_value(0.25),
                     "Percent fudge factor for disparity seed's search range.")
      ("cost-mode",              po::value(&global.cost_mode)->default_value(2),
                     "Correlation cost metric. [0 Absolute, 1 Squared, 2 Normalized Cross Correlation, 3 Census Transform (SGM only), 4 Ternary Census Transform (SGM only)]")
      ("xcorr-threshold",        po::value(&global.xcorr_threshold)->default_value(2),
                     "L-R vs R-L agreement threshold in pixels.")
      ("min-xcorr-level",        po::value(&global.min_xcorr_level)->default_value(0),
                     "Minimum level to run xcorr check on (SGM only).")
      ("corr-kernel",            po::value(&global.corr_kernel)->default_value(Vector2i(21,21),"21 21"),
                    "Kernel size used for integer correlator.")
      ("corr-search",            po::value(&global.search_range)->default_value(BBox2i(0,0,0,0), "auto"),
                     "Disparity search range. Specify in format: hmin vmin hmax vmax.")
      ("corr-search-limit",      po::value(&global.search_range_limit)->default_value(BBox2i(0,0,0,0), "auto"),
                     "Limit on automatically computed disparity search range: hmin vmin hmax vmax.")
      ("elevation-limit",        po::value(&global.elevation_limit)->default_value(Vector2(0,0), "auto"),
       "Limit on expected elevation range: Specify as two values: min max.")
      // Note that we count later on the default for lon_lat_limit being BBox2(0,0,0,0).
      ("lon-lat-limit",     po::value(&global.lon_lat_limit)->default_value(BBox2(0,0,0,0), "auto"),
       "Limit the triangulated interest points to this longitude-latitude range. The format is: lon_min lat_min lon_max lat_max.")
      ("corr-max-levels",        po::value(&global.corr_max_levels)->default_value(5),
       "Max pyramid levels to process when using the integer correlator. (0 is just a single level).")
      // TODO: These parameters are used here, but are only set as filter options.
      //("rm-min-matches",      po::value(&global.rm_min_matches)->default_value(60),
      //                        "Minimum number of pixels to be matched to keep sample (for filter mode 2).")
      //("rm-threshold",        po::value(&global.rm_threshold)->default_value(3),
      //                        "Maximum distance between samples to be considered still matched (for filter mode 2).")
      ("rm-quantile-percentile",  po::value(&global.rm_quantile_percentile)->default_value(0.85),
                              "Filter out pixels in D_sub where disparity > multiple*quantile.")
      ("rm-quantile-multiple",    po::value(&global.rm_quantile_multiple)->default_value(-1),
                              "Filter out pixels in D_sub where disparity > multiple*quantile.  Set >0 to enable.")
      ("skip-low-res-disparity-comp", po::bool_switch(&global.skip_low_res_disparity_comp)->default_value(false)->implicit_value(true),
                     "Skip the low-resolution disparity computation. This option is used in parallel_stereo.")
      ("compute-low-res-disparity-only", po::bool_switch(&global.compute_low_res_disparity_only)->default_value(false)->implicit_value(true),
                     "Compute only the low-resolution disparity, skip the full-resolution disparity computation.")
      ("disparity-estimation-dem", po::value(&global.disparity_estimation_dem)->default_value(""),
                     "DEM to use in estimating the low-resolution disparity (when corr-seed-mode is 2).")
      ("disparity-estimation-dem-error", po::value(&global.disparity_estimation_dem_error)->default_value(0.0),
                     "Error (in meters) of the disparity estimation DEM.")
      ("use-local-homography",   po::bool_switch(&global.use_local_homography)->default_value(false)->implicit_value(true),
                     "Apply a local homography in each tile.")
      ("corr-timeout",           po::value(&global.corr_timeout)->default_value(900),
                     "Correlation timeout for a tile, in seconds.")
      ("stereo-algorithm",       po::value(&global.stereo_algorithm)->default_value(0),
                     "Stereo algorithm to use [0=local window, 1=SGM, 2=Smooth SGM].")
      ("corr-blob-filter",       po::value(&global.corr_blob_filter_area)->default_value(0),
                     "Filter blobs this size or less in correlation pyramid step.")
      ("corr-tile-size",         po::value(&global.corr_tile_size_ovr)->default_value(ASPGlobalOptions::corr_tile_size()),
                     "Override the default tile size used for processing.")
      ("sgm-collar-size",        po::value(&global.sgm_collar_size)->default_value(512),
                     "Extend SGM calculation to this distance to increase accuracy at tile borders.")
      ("sgm-search-buffer",        po::value(&global.sgm_search_buffer)->default_value(Vector2i(4,4),"4 4"),
                     "Search range expansion for SGM down stereo pyramid levels.  Smaller values are faster, but greater change of blunders.")
      ("corr-memory-limit-mb",     po::value(&global.corr_memory_limit_mb)->default_value(6*1024),
                     "Keep correlation memory usage (per tile) close to this limit.  Important for SGM/MGM.")
      ("stereo-debug",   po::bool_switch(&global.stereo_debug)->default_value(false)->implicit_value(true),
                     "Write stereo debug images and output.");

    po::options_description backwards_compat_options("Aliased backwards compatibility options");
    // Do not add default values here. They may override the values set
    // earlier for these variables.
    backwards_compat_options.add_options()
      ("h-kernel",   po::value(&global.corr_kernel[0]       ), "Correlation kernel width.")
      ("v-kernel",   po::value(&global.corr_kernel[1]       ), "Correlation kernel height.")
      ("h-corr-min", po::value(&global.search_range.min()[0]), "Correlation window size min x.")
      ("h-corr-max", po::value(&global.search_range.max()[0]), "Correlation window size max x.")
      ("v-corr-min", po::value(&global.search_range.min()[1]), "Correlation window size min y.")
      ("v-corr-max", po::value(&global.search_range.max()[1]), "Correlation window size max y.");
    (*this).add( backwards_compat_options );
  }

  SubpixelDescription::SubpixelDescription() : po::options_description("Subpixel Options") {
    StereoSettings& global = stereo_settings();
    (*this).add_options()
      ("subpixel-mode",       po::value(&global.subpixel_mode)->default_value(1),
  "Subpixel algorithm. [0 None, 1 Parabola, 2 Bayes EM, 3 Affine, 4 LK, 5 Bayes EM w/gamma, 6 SGM Linear, 7 SGM Poly4, 8 SGM Cos, 9 SGM Parabola 10 SGM None 11 SGM Blend]")
      ("subpixel-kernel",     po::value(&global.subpixel_kernel)->default_value(Vector2i(35,35), "35 35"),
                              "Kernel size used for subpixel method.")
      ("disable-h-subpixel",  po::bool_switch(&global.disable_h_subpixel)->default_value(false)->implicit_value(true),
                              "Disable calculation of subpixel in horizontal direction.")
      ("disable-v-subpixel",  po::bool_switch(&global.disable_v_subpixel)->default_value(false)->implicit_value(true),
                              "Disable calculation of subpixel in vertical direction.")
      ("subpixel-max-levels", po::value(&global.subpixel_max_levels)->default_value(2),
                              "Max pyramid levels to process when using the BayesEM refinement. (0 is just a single level).");

    po::options_description experimental_subpixel_options("Experimental Subpixel Options");
    experimental_subpixel_options.add_options()
      ("subpixel-em-iter",        po::value(&global.subpixel_em_iter)->default_value(15),
                                  "Maximum number of EM iterations for EMSubpixelCorrelator.")
      ("subpixel-affine-iter",    po::value(&global.subpixel_affine_iter)->default_value(5),
                                  "Maximum number of affine optimization iterations for EMSubpixelCorrelator.")
      ("subpixel-pyramid-levels", po::value(&global.subpixel_pyramid_levels)->default_value(3),
                                  "Number of pyramid levels for EMSubpixelCorrelator.");
    (*this).add( experimental_subpixel_options );

    po::options_description backwards_compat_options("Aliased backwards compatibility options");
    backwards_compat_options.add_options()
      ("subpixel-h-kernel", po::value(&global.subpixel_kernel[0]), "Subpixel kernel width.")
      ("subpixel-v-kernel", po::value(&global.subpixel_kernel[1]), "Subpixel kernel height.");
    (*this).add( backwards_compat_options );
  }

  FilteringDescription::FilteringDescription() : po::options_description("Filtering Options") {
    StereoSettings& global = stereo_settings();
    (*this).add_options()
      ("filter-mode",         po::value(&global.filter_mode)->default_value(1),
                              "Disparity filter mode. [0 None, 1 Use mean difference to neighbors (invalidates fewer pixels), 2 Use thresholds (invalidates more pixels)]")
      ("rm-half-kernel",      po::value(&global.rm_half_kernel)->default_value(Vector2i(5,5), "5 5"),
                              "Low confidence pixel removal kernel (half sized).")
      ("max-mean-diff",       po::value(&global.max_mean_diff)->default_value(3),
                              "Maximum difference between current pixel disparity and mean disparity of neighbors to still keep current disparity (for filter mode 1).")
      ("rm-min-matches",      po::value(&global.rm_min_matches)->default_value(60),
                              "Minimum number of pixels to be matched to keep sample (for filter mode 2).")
      ("rm-threshold",        po::value(&global.rm_threshold)->default_value(3),
                              "Maximum distance between samples to be considered still matched (for filter mode 2).")
      ("rm-cleanup-passes",   po::value(&global.rm_cleanup_passes)->default_value(1),
                              "Number of passes for cleanup during the post-processing phase.")
      ("enable-fill-holes",   po::bool_switch(&global.enable_fill_holes)->default_value(false)->implicit_value(true),
                              "Enable filling of holes in disparity using an inpainting method. Obsolete. It is suggested to use instead point2dem's analogous functionality.")
      ("disable-fill-holes",  po::bool_switch(&global.disable_fill_holes)->default_value(false)->implicit_value(true),
                              "Disable filling of holes using an inpainting method. Ignored and obsolete. To be removed in future versions of the software.")
      ("fill-holes-max-size", po::value(&global.fill_hole_max_size)->default_value(100000),
                              "Holes with no more pixels than this number should be filled in.")
      ("edge-buffer-size",    po::value(&global.mask_buffer_size)->default_value(-1),
                              "Size of region filtered off the image border.  If unset, equal to the subpixel kernel size.")
      ("erode-max-size",      po::value(&global.erode_max_size)->default_value(0),
                              "Isolated blobs with no more pixels than this number should be removed.")
      ("median-filter-size",  po::value(&global.median_filter_size)->default_value(0),
                              "Filter subpixel results with a median filter of this size. Can only be used with texture smoothing.")
      ("texture-smooth-size",  po::value(&global.disp_smooth_size)->default_value(0),
                               "Kernel size to perform texture aware disparity smoothing with. Can only be used with median smoothing.")
      ("texture-smooth-scale", po::value(&global.disp_smooth_texture)->default_value(0.15),
                               "Scaling factor for texture smoothing.  Larger is more smoothing.")
      ("mask-flatfield",      po::bool_switch(&global.mask_flatfield)->default_value(false)->implicit_value(true),
                              "Mask dust found on the sensor or film. (For use with Apollo Metric Cameras only!)");

    po::options_description backwards_compat_options("Aliased backwards compatibility options");
    // Do not add default values here. They may override the values set
    // earlier for these variables.
    backwards_compat_options.add_options()
      ("rm-h-half-kern", po::value(&global.rm_half_kernel[0]), "Filter kernel half width.")
      ("rm-v-half-kern", po::value(&global.rm_half_kernel[1]), "Filter kernel half height.");
    (*this).add( backwards_compat_options );
  }

  TriangulationDescription::TriangulationDescription() : po::options_description("Triangulation Options") {
    StereoSettings& global = stereo_settings();
    (*this).add_options()
      ("universe-center",                   po::value(&global.universe_center)->default_value("None"),
                                            "Center from which to measure the universe radius for the purpose of removing outliers. [Camera, Zero (planet center), None]")
      ("near-universe-radius",              po::value(&global.near_universe_radius)->default_value(0.0),
                                            "Radius of inner boundary of universe in meters (remove points with radius smaller than that).")
      ("far-universe-radius",               po::value(&global.far_universe_radius)->default_value(0.0),
                                            "Radius of outer boundary of universe in meters (remove points with radius larger than that).")
      ("min-triangulation-angle",           po::value(&global.min_triangulation_angle)->default_value(0.0),
                                            "The minimum angle, in degrees, at which rays must meet at a triangulated point to accept this point as valid. The internal default is somewhat less than 1 degree.")
      ("use-least-squares",                 po::bool_switch(&global.use_least_squares)->default_value(false)->implicit_value(true),
                                            "Use rigorous least squares triangulation process. This is slow for ISIS processes.")
      ("bundle-adjust-prefix", po::value(&global.bundle_adjust_prefix),
       "Use the camera adjustments obtained by previously running bundle_adjust with this output prefix.")
      ("image-lines-per-piecewise-adjustment", po::value(&global.image_lines_per_piecewise_adjustment)->default_value(0), "A positive value, e.g., 1000, will turn on using piecewise camera adjustments to help reduce jitter effects. Use one adjustment per this many image lines.")
      ("piecewise-adjustment-percentiles",     po::value(&global.piecewise_adjustment_percentiles)->default_value(Vector2(5, 95), "5 95"), "A narrower range will place the piecewise adjustments for jitter correction closer together and further from the first and last lines in the image.")
      ("piecewise-adjustment-interp-type", po::value(&global.piecewise_adjustment_interp_type)->default_value(1), "How to interpolate between adjustments. [1 Linear, 2 Using Gaussian weights]")
      ("num-matches-for-piecewise-adjustment", po::value(&global.num_matches_for_piecewise_adjustment)->default_value(90000), "How many matches among images to create based on the disparity for the purpose of solving for jitter using piecewise adjustment.")
      ("piecewise-adjustment-camera-weight", po::value(&global.piecewise_adjustment_camera_weight)->default_value(1.0), "The weight to use for the sum of squares of adjustments component of the cost function. Increasing this value will constrain the adjustments to be smaller.")
      ("point-cloud-rounding-error",        po::value(&global.point_cloud_rounding_error)->default_value(0.0),
                                            "How much to round the output point cloud values, in meters (more rounding means less precision but potentially smaller size on disk). The inverse of a power of 2 is suggested. Default: 1/2^10 for Earth and proportionally less for smaller bodies.")
      ("save-double-precision-point-cloud", po::bool_switch(&global.save_double_precision_point_cloud)->default_value(false)->implicit_value(true),
                                            "Save the final point cloud in double precision rather than bringing the points closer to origin and saving as float (marginally more precision at twice the storage).")
      ("compute-point-cloud-center-only",   po::bool_switch(&global.compute_point_cloud_center_only)->default_value(false)->implicit_value(true),
                                            "Only compute the center of triangulated point cloud and exit.")
      ("skip-point-cloud-center-comp", po::bool_switch(&global.skip_point_cloud_center_comp)->default_value(false)->implicit_value(true),
       "Skip the computation of the point cloud center. This option is used in parallel_stereo.")
      ("compute-error-vector",              po::bool_switch(&global.compute_error_vector)->default_value(false)->implicit_value(true),
                                            "Compute the triangulation error vector, not just its length.")
      ("compute-piecewise-adjustments-only", po::bool_switch(&global.compute_piecewise_adjustments_only)->default_value(false)->implicit_value(true),
       "Compute the piecewise adjustments as part of jitter correction, and then stop.")
      ("skip-computing-piecewise-adjustments", po::bool_switch(&global.skip_computing_piecewise_adjustments)->default_value(false)->implicit_value(true),
       "Skip computing the piecewise adjustments for jitter, they should have been done by now.")
      ;
  }

  GUIDescription::GUIDescription() : po::options_description("GUI Options") {
    StereoSettings& global = stereo_settings();
    (*this).add_options()
      ("grid-cols",         po::value(&global.grid_cols)->default_value(std::numeric_limits<int>::max()),
                            "Display images as tiles on a grid with this many columns. Default: Use one row.")
      ("window-size",       po::value(&global.window_size)->default_value(Vector2i(1200,800),"1200 800"),
                            "The width and height of the GUI window in pixels.")
      ("single-window,w",   po::bool_switch(&global.single_window)->default_value(false)->implicit_value(true),
                            "Show all images in the same window (with a dialog to choose among them) rather than next to each other.")
      ("use-georef,g",        po::bool_switch(&global.use_georef)->default_value(false)->implicit_value(true),
                            "Plot the images in the projected coordinate system given by the georeference of the images.")
      ("hillshade",         po::bool_switch(&global.hillshade)->default_value(false)->implicit_value(true),
                            "Interpret the input images as DEMs and hillshade them.")

      ("hillshade-azimuth", po::value(&global.hillshade_azimuth)->default_value(300),
                            "The azimuth value when showing hillshaded images.")
      ("hillshade-elevation", po::value(&global.hillshade_elevation)->default_value(20),
                            "The elevation value when showing hillshaded images.")
      ("view-matches",   po::bool_switch(&global.view_matches)->default_value(false)->implicit_value(true),
                            "Locate and display the interest point matches.")
      ("match-file", po::value(&global.match_file)->default_value(""),
       "Display this match file instead of looking one up based on existing conventions (implies --view-matches).")
      ("gcp-file", po::value(&global.gcp_file)->default_value(""),
       "Display the GCP pixel coordinates for this GCP file (implies --view-matches).")
      ("delete-temporary-files-on-exit",   po::bool_switch(&global.delete_temporary_files_on_exit)->default_value(false)->implicit_value(true),
       "Delete any subsampled and other files created by the GUI when exiting.")
      ("create-image-pyramids-only",   po::bool_switch(&global.create_image_pyramids_only)->default_value(false)->implicit_value(true),
       "Without starting the GUI, build multi-resolution pyramids for the inputs, to be able to load them fast later.")
      ;
  }

  DGDescription::DGDescription() : po::options_description("DG Options") {
    StereoSettings& global = stereo_settings();
    (*this).add_options()
      ("disable-correct-velocity-aberration", po::bool_switch(&global.disable_correct_velocity_aberration)->default_value(false)->implicit_value(true),
       "Apply the velocity aberration correction for Digital Globe cameras.");
  }

  UndocOptsDescription::UndocOptsDescription() : po::options_description("Undocumented Options") {
    StereoSettings& global = stereo_settings();
    (*this).add_options()
      ("trans-crop-win", po::value(&global.trans_crop_win)->default_value(BBox2i(0, 0, 0, 0), "xoff yoff xsize ysize"), "Left image crop window in respect to L.tif. This is an internal option. [default: use the entire image].")
      ("attach-georeference-to-lowres-disparity", po::bool_switch(&global.attach_georeference_to_lowres_disparity)->default_value(false)->implicit_value(true),
       "If input images are georeferenced, make D_sub and D_sub_spread georeferenced.");
  }

  po::options_description
  generate_config_file_options( vw::cartography::GdalWriteOptions& opt ) {
    po::options_description cfg_options;
    cfg_options.add( vw::cartography::GdalWriteOptionsDescription( opt ) );
    cfg_options.add( PreProcessingDescription() );
    cfg_options.add( CorrelationDescription()   );
    cfg_options.add( SubpixelDescription()      );
    cfg_options.add( FilteringDescription()     );
    cfg_options.add( TriangulationDescription() );
    cfg_options.add( GUIDescription()           );
    cfg_options.add( DGDescription()            );
    cfg_options.add( UndocOptsDescription()     );

    return cfg_options;
  }

  void StereoSettings::validate() {
    using namespace boost::algorithm;
    to_lower( alignment_method );
    trim( alignment_method );
    VW_ASSERT( alignment_method == "none"     || alignment_method == "homography" ||
               alignment_method == "epipolar" || alignment_method == "affineepipolar",
               ArgumentErr() << "\"" <<  alignment_method
               << "\" is not a valid option for ALIGNMENT_METHOD." );

    to_lower( universe_center );
    trim( universe_center );
    VW_ASSERT( universe_center == "camera" || universe_center == "zero" ||
               universe_center == "none",
               ArgumentErr() << "\"" << universe_center
               << "\" is not a valid option for UNIVERSE_CENTER." );
  }

  void StereoSettings::write_copy( int argc, char *argv[],
                                   std::string const& input_file,
                                   std::string const& output_file ) const {
    using namespace std;
    ifstream in( input_file.c_str() );
    ofstream out( output_file.c_str() );

    // Write some log information
    out << "# ASP Stereo Configuration Copy:" << endl;
    out << "# " << current_posix_time_string() << endl;
    out << "# > ";
    for ( int i = 0; i < argc; i++ ) {
      if ( i )
        out << " ";
      out << string(argv[i]);
    }
    out << endl << endl;

    out << in.rdbuf();
    in.close();
    out.close();
  }

  bool StereoSettings::is_search_defined() const {
    return !( search_range.min() == Vector2i() &&
              search_range.max() == Vector2i() );
  }

  bool asp_config_file_iterator::getline( std::string& s ) {
    std::string ws;

    if ( !std::getline( *is, ws, '\n') )
      return false;

    // Remove any comments that might be one the line
    size_t n = ws.find('#');
    if ( n != std::string::npos ) {
      ws = ws.substr(0,n);
      boost::trim(ws);
    }

    // Handle empty lines .. just pass them on through
    if ( ws.empty() ) {
      s = ws;
      return true;
    }

    // If there is not an equal sign, the first space is turned to
    // equal ... or it's just appended. Also, use lower case for the key.
    n = ws.find('=');
    if ( n  == std::string::npos ) {
      n = ws.find(' ');

      if ( n == std::string::npos ) {
        ws += "=";
        boost::to_lower(ws);
      } else {
        ws[n] = '=';
        std::string lowered_key = boost::to_lower_copy( ws.substr(0,n) );
        ws.replace( 0, n, lowered_key );
      }
    } else {
      std::string lowered_key = boost::to_lower_copy( ws.substr(0,n) );
      ws.replace( 0, n, lowered_key );
    }
    s = ws;
    return true;
  }

  asp_config_file_iterator::asp_config_file_iterator( std::basic_istream<char>& is,
                                                      const std::set<std::string>& allowed_options,
                                                      bool allow_unregistered ) :
    po::detail::common_config_file_iterator(allowed_options, allow_unregistered) {
    this->is.reset(&is, po::detail::null_deleter());
    get();
  }

  po::basic_parsed_options<char>
  parse_asp_config_file(std::basic_istream<char>& is,
                        const po::options_description& desc,
                        bool allow_unregistered ) {
    std::set<std::string> allowed_options;

    const std::vector<boost::shared_ptr<po::option_description> >& options = desc.options();
    for (size_t i = 0; i < options.size(); ++i) {
      const po::option_description& d = *options[i];

        if (d.long_name().empty())
          boost::throw_exception(po::error("long name required for config file"));

        allowed_options.insert(d.long_name());
      }

    // Parser return char strings
    po::parsed_options result(&desc);
    std::copy(asp_config_file_iterator(is, allowed_options, allow_unregistered),
              asp_config_file_iterator(),
              std::back_inserter(result.options));
    // Convert char strings into desired type.
    return po::basic_parsed_options<char>(result);
  }

  po::basic_parsed_options<char>
  parse_asp_config_file(bool print_warning,
                        std::string const& filename,
                        const po::options_description& desc,
                        bool allow_unregistered ) {
    std::basic_ifstream<char> strm(filename.c_str());
    if (!strm && print_warning) {
      vw_out(WarningMessage)
        << "Stereo file: " << filename << " could not be found. "
        << "Will use default settings and command line options only."
        << std::endl;
    }
    return parse_asp_config_file(strm, desc, allow_unregistered);
  }
} // end namespace asp
