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


/// \file StereoSettings.cc
///
#include <fstream>

#include <vw/Core/RunOnce.h>
#include <vw/Core/Log.h>

#include <asp/Core/Common.h>
#include <asp/Core/StereoSettings.h>
#include <asp/Core/PointUtils.h>

namespace po = boost::program_options;
using namespace vw;

const double g_nan_val = std::numeric_limits<double>::quiet_NaN();

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
      vw::GdalWriteOptions opt;
      stereo_settings_ptr->initialize(opt);
    }

    return *stereo_settings_ptr;
  }

  void StereoSettings::initialize(vw::GdalWriteOptions & opt){
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

  StereoSettings::StereoSettings() {

    // This var will turn to true once all stereo settings were parsed
    // and all the members of this class are initialized.
    initialized_stereo_settings = false;

    use_least_squares = false;
    
    default_corr_timeout = 900; // in seconds
    
    nodata_value = g_nan_val;
  }

  // Define our options that are available
  // ----------------------------------------------------------
  PreProcessingDescription::PreProcessingDescription() : po::options_description("Preprocessing options") {

    StereoSettings& global = stereo_settings();

    (*this).add_options()
      ("alignment-method",         po::value(&global.alignment_method)->default_value("affineepipolar"),
                     "Alignment for input images. [affineepipolar, local_epipolar, homography, epipolar, none]")
      ("left-image-crop-win", po::value(&global.left_image_crop_win)->default_value(BBox2i(0, 0, 0, 0), "xoff yoff xsize ysize"),
                      "Do stereo in a subregion of the left image [default: use the entire image].")
      ("right-image-crop-win", po::value(&global.right_image_crop_win)->default_value(BBox2i(0, 0, 0, 0), "xoff yoff xsize ysize"),
                      "Do stereo in a subregion of the right image if specified together with left-image-crop-win [default: use the entire image].")
      ("force-use-entire-range",   po::bool_switch(&global.force_use_entire_range)->default_value(false)->implicit_value(true),
                     "Normalize images based on the global min and max values from both images. Don't use this option if you are using normalized cross correlation.")
      ("individually-normalize",   po::bool_switch(&global.individually_normalize)->default_value(false)->implicit_value(true),
                     "Individually normalize the input images between 0.0-1.0 using +- 2.5 sigmas about their mean values.")
      ("ip-per-tile", po::value(&global.ip_per_tile)->default_value(0),
                     "How many interest points to detect in each 1024^2 image tile (default: automatic determination). This is before matching. Not all interest points will have a match. See also --matches-per-tile.")
      ("ip-per-image", po::value(&global.ip_per_image)->default_value(0),
                     "How many interest points to detect in each image, before matching (default: automatic determination). It is overridden by --ip-per-tile if provided.")
      ("matches-per-tile",  po::value(&global.matches_per_tile)->default_value(0),
         "How many interest point matches to compute in each image tile (of size "
         "normally 1024^2 pixels). Use a value of --ip-per-tile a few times larger "
         "than this. See also --matches-per-tile-params.")
      ("ip-detect-method",          po::value(&global.ip_matching_method)->default_value(0),
       "Interest point detection algorithm (0: Integral OBALoG (default), 1: OpenCV SIFT, 2: OpenCV ORB.")
      ("epipolar-threshold",       po::value(&global.epipolar_threshold)->default_value(-1),
                     "Maximum distance from the epipolar line to search for IP matches. Default: automatic calculation.")
      ("ip-edge-buffer",          po::value(&global.ip_edge_buffer_percent)->default_value(0),
       "Remove IP within this percentage from the outer edges of an image pair (integer percent).")
      ("normalize-ip-tiles", po::bool_switch(&global.ip_normalize_tiles)->default_value(false)->implicit_value(true),
       "Individually normalize tiles used for IP detection.")
      ("ip-inlier-factor",          po::value(&global.ip_inlier_factor)->default_value(1.0/15.0),
       "A higher factor will result in more interest points, but perhaps also more outliers, and a bigger search range.")
      ("ip-uniqueness-threshold",          po::value(&global.ip_uniqueness_thresh)->default_value(0.8),
       "Min percentage distance between closest and second closest IP descriptors, a larger value allows more IP matches.")
      ("ip-nodata-radius",          po::value(&global.ip_nodata_radius)->default_value(4),
       "Remove IP near nodata with this radius, in pixels.")
      ("ip-triangulation-max-error", po::value(&global.ip_triangulation_max_error)->default_value(-1),
       "When matching IP, filter out any pairs with a triangulation error higher than this.")
      ("ip-num-ransac-iterations", po::value(&global.ip_num_ransac_iterations)->default_value(1000),
       "How many RANSAC iterations to do in interest point matching.")
      ("disable-tri-ip-filter",     po::bool_switch(&global.disable_tri_filtering)->default_value(false)->implicit_value(true),
       "Turn off the tri-ip filtering step.")
      ("ip-debug-images",     po::bool_switch(&global.ip_debug_images)->default_value(false)->implicit_value(true),
                      "Write debug images to disk when detecting and matching interest points.")
      ("num-obalog-scales",              po::value(&global.num_scales)->default_value(-1),
       "How many scales to use if detecting interest points with OBALoG. If not specified, 8 will be used. More can help for images with high frequency artifacts.")
      ("nodata-value",             po::value(&global.nodata_value)->default_value(g_nan_val),
       "Pixels with values less than or equal to this number are treated as no-data. This overrides the no-data values from input images.")
      ("nodata-pixel-percentage",  po::value(&global.nodata_pixel_percentage)->default_value(g_nan_val),
       "The percentage of (low-value) pixels treated as no-data (use a number between 0 and 100).")
      ("stddev-mask-kernel",  po::value(&global.nodata_stddev_kernel)->default_value(-1),
       "Size of kernel to be used in standard deviation filtering of input images. Must be > 1 and odd to be enabled. To be used with --stddev-mask-thresh.")
      ("stddev-mask-thresh",  po::value(&global.nodata_stddev_thresh)->default_value(0.5),
       "Mask out pixels from input images where the local standard deviation score is less than this value. If set < 0, debug files (*stddev_filter_output.tif) will be written containing the filter output instead of masking out pixels.To be used with --stddev-mask-kernel.")
      ("skip-rough-homography", po::bool_switch(&global.skip_rough_homography)->default_value(false)->implicit_value(true),
       "Skip the step of performing datum-based rough homography if it fails.")
      ("no-datum", po::bool_switch(&global.no_datum)->default_value(false)->implicit_value(true),
       "Do not assume a reliable datum exists, such as for potato-shaped bodies.")
      ("skip-image-normalization", po::bool_switch(&global.skip_image_normalization)->default_value(false)->implicit_value(true),
       "Skip the step of normalizing the values of input images and removing nodata-pixels. Create instead symbolic links to original images. This is a speedup option for mapprojected input images with no alignment.")
      ("force-reuse-match-files", po::bool_switch(&global.force_reuse_match_files)->default_value(false)->implicit_value(true),
       "Force reusing the match files even if older than the images or cameras.")
      ("part-of-multiview-run", po::bool_switch(&global.part_of_multiview_run)->default_value(false)->implicit_value(true),
       "If the current run is part of a larger multiview run.")
      ("global-alignment-threshold",       po::value(&global.global_alignment_threshold)->default_value(10),
       "Maximum distance from inlier interest point matches to the epipolar line when calculating the global affine epipolar alignment.")
      ("local-alignment-threshold",       po::value(&global.local_alignment_threshold)->default_value(2),
       "Maximum distance from inlier interest point matches to the epipolar line when calculating the local affine epipolar alignment.")
      ("alignment-num-ransac-iterations", po::value(&global.alignment_num_ransac_iterations)->default_value(1000),
       "How many RANSAC iterations to use for global or local epipolar alignment.")
      ("outlier-removal-params",  po::value(&global.outlier_removal_params)->default_value(Vector2(95.0, 3.0), "pct factor"),
       "Outlier removal params (percentage and factor) to be used in filtering interest points and the disparity with the box-and-whisker algorithm. Set the percentage to 100 to turn this off.")
      ("matches-per-tile-params",  po::value(&global.matches_per_tile_params)->default_value(Vector2(1024, 1280), "1024 1280"),
       "To be used with --matches-per-tile. The first value is the image tile "
        "size for both images. A larger second value allows each right tile to "
        "further expand to this size, resulting in the tiles overlapping. This may be "
        "needed if the homography alignment between these images is not great, as "
        "this transform is used to pair up left and right image tiles.")
      ("disparity-range-expansion-percent", po::value(&global.disparity_range_expansion_percent)->default_value(20),
       "Expand the disparity range estimated from interest points by this percentage before computing the stereo correlation with local epipolar alignment.")
      ("datum",                    po::value(&global.datum)->default_value("WGS_1984"),
       "Set the datum. Used chiefly with RPC cameras. Options: WGS_1984, D_MOON (1,737,400 meters), D_MARS (3,396,190 meters), MOLA (3,396,000 meters), NAD83, WGS72, and NAD27. Also accepted: Earth (=WGS_1984), Mars (=D_MARS), Moon (=D_MOON).")
      ("match-files-prefix",  po::value(&global.match_files_prefix)->default_value(""),
       "Use the match file from this prefix. Normally contains match files "
       "created with bundle_adjust or parallel_stereo.")
      ("clean-match-files-prefix",  po::value(&global.clean_match_files_prefix)->default_value(""),
       "Use as input match file the *-clean.match file from this prefix (this had the outliers filtered out).")
      ("left-image-clip", po::value(&global.left_image_clip)->default_value(""),
       "If --left-image-crop-win is used, replaced the left image cropped to that window with this clip.")
      ("right-image-clip", po::value(&global.right_image_clip)->default_value(""),
       "If --right-image-crop-win is used, replaced the right image cropped to that window with this clip.")
      ("aster-use-csm", po::bool_switch(&global.aster_use_csm)->default_value(false)->implicit_value(true),
       "Use the CSM model with ASTER cameras (-t aster).")
      
      // For bathymetry correction
      ("left-bathy-mask", po::value(&global.left_bathy_mask),
       "Mask to use for the left image when doing bathymetry.")
      ("right-bathy-mask", po::value(&global.right_bathy_mask),
       "Mask to use for the right image when doing bathymetry.")
      ("bathy-plane", po::value(&global.bathy_plane),
       "The file storing the water plane used for bathymetry having the coefficients a, b, c, d with the plane being a*x + b*y + c*z + d = 0. Separate bathy planes can be used for the left and right images, to be passed in as 'left_plane.txt right_plane.txt'.")
      ("refraction-index", po::value(&global.refraction_index)->default_value(0),
       "The index of refraction of water to be used in bathymetry correction. (Must be specified and bigger than 1.)")
      ("output-cloud-type", po::value(&global.output_cloud_type)->default_value("all"),
       "When bathymetry correction is used, return only the triangulated cloud of points where the bathymetry correction was applied (option: 'bathy'), where it was not applied (option: 'topo'), or the full cloud (option: 'all'). The default is 'all'.");
  }

  CorrelationDescription::CorrelationDescription() : po::options_description("Correlation options") {
    StereoSettings& global = stereo_settings();
    (*this).add_options()
      ("prefilter-mode",         po::value(&global.pre_filter_mode)->default_value(2),
       "Filter used to prepare images before performing correlation. Used only with the "
       "asp_bm algorithm. Options: 0 (none), 1 (subtracted mean), 2 (LoG).")
      ("prefilter-kernel-width", po::value(&global.slogW)->default_value(1.5),
       "Sigma value for Gaussian kernel used with prefilter modes 1 and 2.")
      ("corr-seed-mode",         po::value(&global.seed_mode)->default_value(1),
                     "Correlation seed strategy. [0 None, 1 Use low-res disparity from stereo, 2 Use low-res disparity from provided DEM (see disparity-estimation-dem), 3 Use low-res disparity produced by sparse_disp (in development)]")
      ("min-num-ip",             po::value(&global.min_num_ip)->default_value(30),
                     "The minimum number of interest points which must be found to estimate the search range.")
      ("corr-sub-seed-percent",  po::value(&global.seed_percent_pad)->default_value(0.25),
                     "Expand the search range by this factor when computing the low-resolution disparity.")
      ("cost-mode",              po::value(&global.cost_mode)->default_value(2),
                     "Correlation cost metric. [0 Absolute, 1 Squared, 2 Normalized Cross Correlation, 3 Census Transform (SGM only), 4 Ternary Census Transform (SGM only)]")
      ("xcorr-threshold",        po::value(&global.xcorr_threshold)->default_value(2.0),
                     "L-R vs R-L agreement threshold in pixels.")
      ("min-xcorr-level",        po::value(&global.min_xcorr_level)->default_value(0),
                     "Minimum level to run xcorr check on (SGM only).")
      ("save-left-right-disparity-difference",   po::bool_switch(&global.save_lr_disp_diff)->default_value(false)->implicit_value(true),
       "Save the discrepancy between left-to-right and right-to-left disparities. See the doc for more info.")
      
      ("corr-kernel",            po::value(&global.corr_kernel)->default_value(Vector2i(21,21),"21 21"),
                    "Kernel size used for integer correlator.")
      ("corr-search",            po::value(&global.search_range)->default_value(BBox2(0,0,0,0), "auto"),
       "Disparity search range. Specify in format: hmin vmin hmax vmax.")
      ("max-disp-spread",      po::value(&global.max_disp_spread)->default_value(-1.0),
       "If positive, limit the spread of the disparity to this value (horizontally and vertically, centered at the median value). Do not specify together with --corr-search-limit.")

      ("corr-search-limit",      po::value(&global.corr_search_limit)->default_value(BBox2(0,0,0,0), "auto"),
                     "Limit the automatically computed disparity search range to these bounds, specified as: hmin vmin hmax vmax. See also --max-disp-spread.")
      ("ip-filter-using-dem",        po::value(&global.ip_filter_using_dem)->default_value(""),
       "Filter as outliers interest point matches whose triangulated height differs by more than given value from the height at the same location for the given DEM. All heights are in meters.  Specify as: '<dem file> <height diff>. Example: 'dem.tif 50.0'.")
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
                     "Skip the low-resolution disparity computation. This option is invoked from parallel_stereo.")
      ("compute-low-res-disparity-only", po::bool_switch(&global.compute_low_res_disparity_only)->default_value(false)->implicit_value(true),
                     "Compute only the low-resolution disparity, skip the full-resolution disparity computation.")
      ("disparity-estimation-dem", po::value(&global.disparity_estimation_dem)->default_value(""),
                     "DEM to use in estimating the low-resolution disparity (when corr-seed-mode is 2).")
      ("disparity-estimation-dem-error", po::value(&global.disparity_estimation_dem_error)->default_value(0.0),
                     "Error (in meters) of the disparity estimation DEM.")
      ("corr-timeout",           po::value(&global.corr_timeout)->default_value(global.default_corr_timeout),
                     "Correlation timeout for an image tile, in seconds.")
      ("stereo-algorithm",       po::value(&global.stereo_algorithm)->default_value("asp_bm"),
                     "Stereo algorithm to use. Options: asp_bm, asp_sgm, asp_mgm, asp_final_mgm, mgm (original author implementation), opencv_sgbm, libelas, msmw, msmw2, and opencv_bm.")
      ("corr-blob-filter",       po::value(&global.corr_blob_filter_area)->default_value(0),
                     "Filter blobs this size or less in correlation pyramid step.")
      ("corr-tile-size",         po::value(&global.corr_tile_size_ovr)->default_value(ASPGlobalOptions::corr_tile_size()),
                     "Override the default tile size used for processing.")
      ("sgm-collar-size",        po::value(&global.sgm_collar_size)->default_value(512),
                     "Extend SGM calculation to this distance to increase accuracy at tile borders.")
      ("sgm-search-buffer",        po::value(&global.sgm_search_buffer)->default_value(Vector2i(4,4),"4 4"),
                     "Search range expansion for SGM down stereo pyramid levels.  Smaller values are faster, but greater change of blunders.")
      ("corr-memory-limit-mb",     po::value(&global.corr_memory_limit_mb)->default_value(4*1024),
       "Keep correlation memory usage (per tile) close to this limit.  Important for SGM/MGM.")
      ("correlator-mode", po::bool_switch(&global.correlator_mode)->default_value(false)->implicit_value(true),
       "Function as an image correlator only (including with subpixel refinement). Assume no cameras, aligned input images, and stop before triangulation, so at filtered disparity.")

      ("stereo-debug",   po::bool_switch(&global.stereo_debug)->default_value(false)->implicit_value(true),
                     "Write stereo debug images and output.")
    ("local-alignment-debug",   po::bool_switch(&global.local_alignment_debug)->default_value(false)->implicit_value(true),
     "Save the results of more intermediate steps when doing local alignment.");

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

  SubpixelDescription::SubpixelDescription() : po::options_description("Subpixel options") {
    StereoSettings& global = stereo_settings();
    (*this).add_options()
      ("subpixel-mode",       po::value(&global.subpixel_mode)->default_value(1),
  "Subpixel algorithm. [0 None, 1 Parabola, 2 Bayes EM, 3 Affine, 4 Phase Correlation 5 LK, 6 Bayes EM w/gamma, 7 SGM None 8 SGM Linear, 9 SGM Poly4, 10 SGM Cos, 11 SGM Parabola 12 SGM Blend]")
      ("subpix-from-blend",   po::bool_switch(&global.subpix_from_blend)->default_value(false)->implicit_value(true),
                              "For the input to subpixel, use the -B.tif file instead of the -D.tif file.")
      ("subpixel-kernel",     po::value(&global.subpixel_kernel)->default_value(Vector2i(35,35), "35 35"),
                              "Kernel size used for subpixel method.")
      ("disable-h-subpixel",  po::bool_switch(&global.disable_h_subpixel)->default_value(false)->implicit_value(true),
                              "Disable calculation of subpixel in horizontal direction.")
      ("disable-v-subpixel",  po::bool_switch(&global.disable_v_subpixel)->default_value(false)->implicit_value(true),
                              "Disable calculation of subpixel in vertical direction.")
      ("subpixel-max-levels", po::value(&global.subpixel_max_levels)->default_value(2),
                              "Max pyramid levels to process when using the BayesEM refinement. (0 is just a single level).")
      ("phase-subpixel-accuracy", po::value(&global.phase_subpixel_accuracy)->default_value(20),
                              "Accuracy to use for mode 4 phase subpixel.  Resolution is 1/this.  Larger values take more time.");

    po::options_description experimental_subpixel_options("Experimental subpixel options");
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

  FilteringDescription::FilteringDescription() : po::options_description("Filtering options") {
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
      ("gotcha-disparity-refinement",   po::bool_switch(&global.gotcha_disparity_refinement)->default_value(false)->implicit_value(true),
                              "Turn on the experimental Gotcha disparity refinement. It refines and overwrites F.tif. See the option 'casp-go-param-file' for customizing its behavior.")
      ("casp-go-param-file", po::value(&global.casp_go_param_file)->default_value(""),
       "The parameter file to use with Gotcha (and in the future other CASP-GO functionality) when invoking the 'gotcha-disparity-refinement' option. The default is to use the file 'share/CASP-GO_params.xml' shipped with ASP.")
      ("mask-flatfield",      po::bool_switch(&global.mask_flatfield)->default_value(false)->implicit_value(true),
                              "Mask dust found on the sensor or film. (For use with Apollo Metric Cameras only.)");

    po::options_description backwards_compat_options("Aliased backwards compatibility options");
    // Do not add default values here. They may override the values set
    // earlier for these variables.
    backwards_compat_options.add_options()
      ("rm-h-half-kern", po::value(&global.rm_half_kernel[0]), "Filter kernel half width.")
      ("rm-v-half-kern", po::value(&global.rm_half_kernel[1]), "Filter kernel half height.");
    (*this).add( backwards_compat_options );
  }

  TriangulationDescription::TriangulationDescription() : po::options_description("Triangulation options") {
    StereoSettings& global = stereo_settings();
    (*this).add_options()
      ("universe-center",                   po::value(&global.universe_center)->default_value("None"),
                                            "Center from which to measure the universe radius for the purpose of removing outliers. [Camera, Zero (planet center), None]")
      ("near-universe-radius",              po::value(&global.near_universe_radius)->default_value(0.0),
                                            "Radius of inner boundary of universe in meters (remove points with radius smaller than that).")
      ("far-universe-radius",               po::value(&global.far_universe_radius)->default_value(0.0),
                                            "Radius of outer boundary of universe in meters (remove points with radius larger than that).")
      ("min-triangulation-angle",           po::value(&global.min_triangulation_angle)->default_value(-1.0),
                                            "The minimum angle, in degrees, at which rays must meet at a triangulated point to accept this point as valid. It must be positive. The internal default is somewhat less than 1 degree.")
    ("max-valid-triangulation-error", po::value(&global.max_valid_triangulation_error)->default_value(0),
            "If positive, points with triangulation error larger than this will be removed from the cloud. Measured in meters.")
      ("bundle-adjust-prefix", po::value(&global.bundle_adjust_prefix),
       "Use the camera adjustments obtained by previously running bundle_adjust with this output prefix.")
      ("propagate-errors",  po::bool_switch(&global.propagate_errors)->default_value(false)->implicit_value(true),
       "Propagate the errors from the input cameras to the triangulated point cloud.")
      ("horizontal-stddev", po::value(&global.horizontal_stddev)->default_value(Vector2(0, 0), "0 0"), "If positive, propagate these left and right camera horizontal ground plane stddev through triangulation. To be used with --propagate-errors.")
      
      ("position-covariance-factor", po::value(&global.position_covariance_factor)->default_value(1.0),
       "Multiply the satellite position covariances by this number before propagating them to the triangulated point cloud. Applicable only to Maxar(DigitalGlobe) linescan cameras.")
      ("orientation-covariance-factor", po::value(&global.orientation_covariance_factor)->default_value(1.0),
       "Multiply the satellite quaternion covariances by this number before propagating them to the triangulated point cloud. Applicable only to Maxar(DigitalGlobe) linescan cameras.")
      
      ("unalign-disparity",                 po::bool_switch(&global.unalign_disparity)->default_value(false)->implicit_value(true),
       "Take the computed disparity, and compute the disparity between unaligned images.")
      ("num-matches-from-disparity", po::value(&global.num_matches_from_disparity)->default_value(0), "Create a match file with this many points uniformly sampled from the stereo disparity. The matches are between original images (that is, before any alignment or map-projection). See also num-matches-from-disp-triplets.")
      ("num-matches-from-disp-triplets", po::value(&global.num_matches_from_disp_triplets)->default_value(0), "Create a match file with this many points uniformly sampled from the stereo disparity, while making sure that if there are more than two images, a set of ground features are represented by matches in at least three of them. The matches are between original images (that is, before any alignment or map-projection). The file name is <output prefix>-disp-<left image>__<right image>.match.")
      ("point-cloud-rounding-error",
       po::value(&global.point_cloud_rounding_error)->default_value(0.0),
       "How much to round the output point cloud values, in meters (more rounding means less precision but potentially smaller size on disk). The inverse of a power of 2 is suggested. Default: 1/2^10 for Earth and proportionally less for smaller bodies, unless error propagation happens, when it is set by default to 1e-8 meters, to avoid introducing step artifacts in these errors.")
      ("save-double-precision-point-cloud", po::bool_switch(&global.save_double_precision_point_cloud)->default_value(false)->implicit_value(true),
       "Save the final point cloud in double precision rather than bringing the points closer to origin and saving as float (marginally more precision at twice the storage).")
      
      ("compute-point-cloud-center-only",   po::bool_switch(&global.compute_point_cloud_center_only)->default_value(false)->implicit_value(true),
                                            "Only compute the center of triangulated point cloud and exit.")
      ("skip-point-cloud-center-comp", po::bool_switch(&global.skip_point_cloud_center_comp)->default_value(false)->implicit_value(true),
       "Skip the computation of the point cloud center. This option is invoked from parallel_stereo.")
      ("compute-error-vector",              po::bool_switch(&global.compute_error_vector)->default_value(false)->implicit_value(true),
                                            "Compute the triangulation error vector, not just its length.")
      // TODO(oalexan1): Wipe the least squares triangulation approach. Not used.
      ("use-least-squares",  po::bool_switch(&global.use_least_squares)->default_value(false)->implicit_value(true),
       "Use rigorous least squares triangulation. This is slow for ISIS processes.")      
      ;
  }

  GUIDescription::GUIDescription() : po::options_description("GUI options") {
    StereoSettings& global = stereo_settings();
    (*this).add_options()
      ("grid-cols",         po::value(&global.grid_cols)->default_value(std::numeric_limits<int>::max()),
                            "Display images as tiles on a grid with this many columns. Default: Use one row.")
      ("window-size",       po::value(&global.window_size)->default_value(Vector2i(1200,800),"1200 800"),
                            "The width and height of the GUI window in pixels.")
      ("single-window,w",   po::bool_switch(&global.single_window)->default_value(false)->implicit_value(true),
                            "Show all images in the same window (with a dialog to choose among them) rather than next to each other.")
      ("view-several-side-by-side",   po::bool_switch(&global.view_several_side_by_side)->default_value(false)->implicit_value(true),
                            "View several images side-by-side, with a dialog to choose which images to show.")
      ("use-georef,g",        po::bool_switch(&global.use_georef)->default_value(false)->implicit_value(true),
                            "Plot the images in the projected coordinate system given by the georeference of the images.")
      ("hillshade",         po::bool_switch(&global.hillshade)->default_value(false)->implicit_value(true),
                            "Interpret the input images as DEMs and hillshade them.")

      ("hillshade-azimuth", po::value(&global.hillshade_azimuth)->default_value(300),
                            "The azimuth value when showing hillshaded images.")
      ("hillshade-elevation", po::value(&global.hillshade_elevation)->default_value(20),
                            "The elevation value when showing hillshaded images.")
      ("lowest-resolution-subimage-num-pixels", po::value(&global.lowest_resolution_subimage_num_pixels)->default_value(-1),
       "When building a pyramid of lower-resolution versions of an image, the coarsest image will have no more than this many pixels. If not set, it will internally default to 1000 x 1000. This is increased to 10000 x 10000 when loading .nvm files or with the --preview option.")
      ("view-matches",   po::bool_switch(&global.view_matches)->default_value(false)->implicit_value(true),
                            "Locate and display the interest point matches for a stereo pair.")
      ("match-file", po::value(&global.match_file)->default_value(""),
       "Display this match file instead of looking one up based on existing conventions (implies --view-matches).")
      ("gcp-file", po::value(&global.gcp_file)->default_value(""),
       "Display the GCP pixel coordinates for this GCP file (implies --view-matches).")
      ("dem-file", po::value(&global.dem_file)->default_value(""),
       "Use this DEM when creating GCP from images.")
       ("hide-all",        po::bool_switch(&global.hide_all)->default_value(false)->implicit_value(true),
        "Start with all images turned off (if all images are in the same window, useful with a large number of images).")
      ("delete-temporary-files-on-exit",   po::bool_switch(&global.delete_temporary_files_on_exit)->default_value(false)->implicit_value(true),
       "Delete any subsampled and other files created by the GUI when exiting.")
      ("create-image-pyramids-only",   po::bool_switch(&global.create_image_pyramids_only)->default_value(false)->implicit_value(true),
       "Without starting the GUI, build multi-resolution pyramids for the inputs, to be able to load them fast later.")
      ("pairwise-matches",   po::bool_switch(&global.pairwise_matches)->default_value(false)->implicit_value(true), "Show images side-by-side. If just two of them are selected, load their corresponding match file, determined by the output prefix. Also accessible from the menu.")
      ("pairwise-clean-matches",   po::bool_switch(&global.pairwise_clean_matches)->default_value(false)->implicit_value(true), "Same as --pairwise-matches, but use *-clean.match files.")
      ("nvm", po::value(&global.nvm)->default_value(""),
       "Load this .nvm file having interest point matches. See also --nvm-no-shift. "
       "The rig_calibrator program can create such files. This option implies "
       "--pairwise-matches, --preview, and a larger value of " 
       "--lowest-resolution-subimage-num-pixels.")
      ("nvm-no-shift", po::bool_switch(&global.nvm_no_shift)->default_value(false)->implicit_value(true),
       "Assume that the image features in the input nvm file were saved without "
       "being shifted to be relative to the optical center of the camera.")
      ("isis-cnet", po::value(&global.isis_cnet)->default_value(""),
       "Load a control network having interest point matches from this binary file "
       "in the ISIS jigsaw format. See also --nvm.")
      ("zoom-proj-win", po::value(&global.zoom_proj_win)->default_value(BBox2(0,0,0,0), ""),
       "Zoom to this proj win on startup. It is assumed that the images are georeferenced. Also accessible from the View menu.")
      ("csv-format",     po::value(&global.csv_format_str)->default_value(""), asp::csv_opt_caption().c_str())
      ("csv-proj4",      po::value(&global.csv_proj4)->default_value(""), "The PROJ.4 string to use to interpret the entries in a CSV file. If not specified, try to use the --datum option.")
      ("csv-datum",      po::value(&global.csv_datum)->default_value(""), "The datum to use when plotting a CSV file.")
      ("preview",   po::bool_switch(&global.preview)->default_value(false)->implicit_value(true),
       "Load and display the images one at a time. The 'n' and 'p' keys can be used to cycle through them.")
      ("colorize",   po::bool_switch(&global.colorize)->default_value(false)->implicit_value(true),
       "Colorize input raster and CSV files (must set --min and --max).")
      ("min", po::value(&global.min)->default_value(g_nan_val),
       "Value corresponding to 'coldest' color in the color map, when using the --colorize option and plotting csv data. Also used to manually set the minimum value in grayscale images. If not set, use the dataset minimum for color images, and estimate the minimum for grayscale images.")
      ("max", po::value(&global.max)->default_value(g_nan_val),
       "Value corresponding to the 'hottest' color in the color map, when using the --colorize option and plotting csv data. Also used to manually set the maximum value in grayscale images. If not set, use the dataset maximum for color images, and estimate the maximum for grayscale images.")
      ("plot-point-radius", po::value(&global.plot_point_radius)->default_value(2),
       "When plotting points from CSV files, let each point be drawn as a filled ball with this radius, in pixels.")
      ("font-size", po::value(&global.font_size)->default_value(9),
       "Set the font size.")
      ("no-georef", 
        po::bool_switch(&global.no_georef)->default_value(false)->implicit_value(true),
       "Do not use georeference information when displaying the data, even when it exists.")
       ;
  }

  // Options for stereo_parse
  ParseDescription::ParseDescription() : po::options_description("stereo_parse options") {
    StereoSettings& global = stereo_settings();
    (*this).add_options()
      ("tile-at-location", po::value(&global.tile_at_loc)->default_value(""),
       "Find the tile in the current parallel_stereo run which generated the DEM portion having this lon-lat-height location. Specify as a string in quotes: 'lon lat height'. Use this option with stereo_parse and the rest of options used in parallel_stereo, including cameras, output prefix, etc. (except for those needed for tiling and parallelization). This does not work with mapprojected images.");
  }

  // Options for parallel_stereo. These are not used by the stereo
  // executables, but accept them quietly so that when stereo_gui or
  // stereo_parse is invoked with a parallel_stereo command it would
  // not fail. Later, if parallel_stereo is invoked from stereo_gui,
  // these will be passed on.
  ParallelDescription::ParallelDescription() : po::options_description("parallel_stereo options") {
    StereoSettings& global = stereo_settings();
    (*this).add_options()
      ("nodes-list", po::value(&global.nodes_list)->default_value(""),
       "The list of computing nodes, one per line. If not provided, run on the local machine.")
      ("ssh", po::value(&global.ssh)->default_value(""),
       "Specify the path to an alternate version of the ssh tool to use.")
      ("processes", po::value(&global.processes)->default_value(-1),
       "The number of processes to use per node.")
      ("threads-multiprocess", po::value(&global.threads_multi)->default_value(-1),
       "The number of threads to use per process when running multiple processes.")
      ("threads-singleprocess",po::value(&global.threads_single)->default_value(-1),
       "The number of threads to use when running a single process (stereo_pprc and stereo_fltr).")
      ("entry-point,e", po::value(&global.entry_point)->default_value(0),
       "Stereo Pipeline entry point (an integer from 0-5).")
      ("stop-point", po::value(&global.stop_point)->default_value(6),
       "Stereo Pipeline stop point (an integer from 1-6). Stop before this step.")
      ("job-size-w", po::value(&global.job_size_w)->default_value(2048),
       "Pixel width of input image tile for a single process.")
      ("job-size-h",           po::value(&global.job_size_h)->default_value(2048),
       "Pixel height of input image tile for a single process.")
      ("sparse-disp-options", po::value(&global.sparse_disp_options)->default_value(""),
       "Options to pass directly to sparse_disp. Use quotes around this string.")
      ("prev-run-prefix", po::value(&global.prev_run_prefix)->default_value(""),
       "Start at the triangulation stage while reusing the data from this prefix.")
      ("parallel-options", po::value(&global.parallel_options)->default_value(""),
       "Options to pass directly to GNU Parallel. Use quotes around this string.");
  }

  UndocOptsDescription::UndocOptsDescription() : po::options_description("Undocumented options") {
    StereoSettings& global = stereo_settings();
    (*this).add_options()
      ("trans-crop-win", po::value(&global.trans_crop_win)->default_value(BBox2i(0, 0, 0, 0), "xoff yoff xsize ysize"), "Left image crop window in respect to L.tif. This is an internal option. [default: use the entire image].")
      ("attach-georeference-to-lowres-disparity", po::bool_switch(&global.attach_georeference_to_lowres_disparity)->default_value(false)->implicit_value(true),
       "If input images are georeferenced, make D_sub and D_sub_spread georeferenced.");
  }

  // This handles options which are not in stereo_settings(), but
  // rather in 'opt'. So they are not config options set in
  // stereo.default but only command-line options.
  void addAspGlobalOptions(boost::program_options::options_description & descripton,
                           ASPGlobalOptions & opt) {
    descripton.add_options()
      ("session-type,t",      po::value(&opt.stereo_session),
       "Select the stereo session type to use for processing. "
       "Usually the program can select this automatically by the file extension, "
       "except for XML cameras. See the doc for options.")
      ("stereo-file,s", po::value(&opt.stereo_default_filename)->default_value("./stereo.default"),
       "Explicitly specify the stereo.default file to use.");
  }
  
  po::options_description
  generate_config_file_options(vw::GdalWriteOptions& opt) {
    po::options_description cfg_options;
    cfg_options.add(vw::GdalWriteOptionsDescription(opt));
    cfg_options.add(PreProcessingDescription());
    cfg_options.add(CorrelationDescription());
    cfg_options.add(SubpixelDescription());
    cfg_options.add(FilteringDescription());
    cfg_options.add(TriangulationDescription());
    cfg_options.add(GUIDescription());
    cfg_options.add(ParseDescription());
    cfg_options.add(ParallelDescription());
    cfg_options.add(UndocOptsDescription());

    return cfg_options;
  }

  void StereoSettings::validate() {
    using namespace boost::algorithm;
    
    to_lower(alignment_method);
    trim(alignment_method);
    VW_ASSERT(alignment_method == "none"     || alignment_method == "homography" ||
               alignment_method == "epipolar" || alignment_method == "affineepipolar" ||
               alignment_method == "local_epipolar",
               ArgumentErr() << "\"" <<  alignment_method
               << "\" is not a valid option for alignment-method.");

    to_lower(universe_center);
    trim(universe_center);
    VW_ASSERT(universe_center == "camera" || universe_center == "zero" ||
               universe_center == "none",
               ArgumentErr() << "\"" << universe_center
               << "\" is not a valid option for universe_center.");
  }

  void StereoSettings::write_copy(int argc, char *argv[],
                                   std::string const& input_file,
                                   std::string const& output_file) const {
    using namespace std;
    ifstream in(input_file.c_str());
    ofstream out(output_file.c_str());

    // Write some log information
    out << "# ASP stereo configuration copy:" << endl;
    out << "# " << current_posix_time_string() << endl;
    out << "# > ";
    for (int i = 0; i < argc; i++) {
      if (i)
        out << " ";
      out << string(argv[i]);
    }
    out << endl << endl;

    out << in.rdbuf();
    in.close();
    out.close();
  }

  bool StereoSettings::is_search_defined() const {
    return !(search_range.min() == Vector2() &&
              search_range.max() == Vector2());
  }

  bool asp_config_file_iterator::getline(std::string& s) {
    std::string ws;

    if (!std::getline(*is, ws, '\n'))
      return false;

    // Remove any comments that might be one the line
    size_t n = ws.find('#');
    if (n != std::string::npos)
      ws = ws.substr(0, n);

    // Wipe any whitespace on either end
    boost::trim(ws);

    // Handle empty lines. Just pass them on through.
    if (ws.empty()) {
      s = ws;
      return true;
    }

    // If there is not an equal sign, the first space is turned to
    // equal, or it is just appended. Also, use lowercase for the key.
    n = ws.find('=');
    if (n  == std::string::npos) {
      n = ws.find(' ');

      if (n == std::string::npos) {
        ws += "=";
        boost::to_lower(ws);
      } else {
        ws[n] = '=';
        std::string lowered_key = boost::to_lower_copy(ws.substr(0,n));
        ws.replace(0, n, lowered_key);
      }
    } else {
      std::string lowered_key = boost::to_lower_copy(ws.substr(0,n));
      ws.replace(0, n, lowered_key);
    }
    s = ws;
    return true;
  }

  asp_config_file_iterator::asp_config_file_iterator(std::basic_istream<char>& is,
                                                     const std::set<std::string>& allowed_options,
                                                     bool allow_unregistered):
    po::detail::common_config_file_iterator(allowed_options, allow_unregistered) {
    this->is.reset(&is, po::detail::null_deleter());
    get();
  }

  // Parse the ASP stereo config file, such as stereo.default, from an open handle.
  po::basic_parsed_options<char>
  parse_asp_config_file(std::basic_istream<char>& is, const po::options_description& desc,
                        bool allow_unregistered) {

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

  // Parse the ASP stereo config file, such as stereo.default.
  po::basic_parsed_options<char>
  parse_asp_config_file(bool print_warning, std::string const& filename,
                        const po::options_description& desc, bool allow_unregistered) {
    std::basic_ifstream<char> strm(filename.c_str());
    if (print_warning) {
      if (!strm)
        vw_out() << "Stereo file " << filename << " could not be found. "
                 << "Will use default settings and command line options only.\n";
      else
        vw_out() << "Using stereo file " << filename << ".\n";
    }

    return parse_asp_config_file(strm, desc, allow_unregistered);
  }
} // end namespace asp
