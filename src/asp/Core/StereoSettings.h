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


/// \file StereoSettings.h
///

#ifndef __ASP_CORE_STEREO_SETTINGS_H__
#define __ASP_CORE_STEREO_SETTINGS_H__

#include <boost/program_options.hpp>
#include <boost/program_options/detail/config_file.hpp>
#include <vw/FileIO/GdalWriteOptions.h>
#include <asp/Core/Common.h>

namespace asp {

  class StereoSession; // Forward declaration

  /// 'Global scoped' variables
  struct ASPGlobalOptions : vw::GdalWriteOptions {
    // Input
    std::string in_file1, in_file2, cam_file1, cam_file2, input_dem,
    extra_argument1, extra_argument2, extra_argument3;

    // Settings
    std::string stereo_session,
                stereo_default_filename;
    boost::shared_ptr<asp::StereoSession> session; // Used to extract cameras
    // Output
    std::string out_prefix;

    // Constants
    static int corr_tile_size() { return 1024; } // Tile size for correlation
    static int rfne_tile_size() { return 256;  } // Tile size for refinement
    static int tri_tile_size() { return 256;  } // Tile size for tri/point cloud
  };

  // Program options for each executable/step
  struct PreProcessingDescription: public boost::program_options::options_description {
    PreProcessingDescription();
  };
  struct CorrelationDescription: public boost::program_options::options_description {
    CorrelationDescription();
  };
  struct SubpixelDescription: public boost::program_options::options_description {
    SubpixelDescription();
  };
  struct FilteringDescription: public boost::program_options::options_description {
    FilteringDescription();
  };
  struct TriangulationDescription: public boost::program_options::options_description {
    TriangulationDescription();
  };
  struct GUIDescription: public boost::program_options::options_description {
    GUIDescription();
  };
  struct ParseDescription: public boost::program_options::options_description {
    ParseDescription();
  };
  struct ParallelDescription: public boost::program_options::options_description {
    ParallelDescription();
  };
  struct UndocOptsDescription: public boost::program_options::options_description {
    UndocOptsDescription();
  };

  boost::program_options::options_description
  generate_config_file_options(vw::GdalWriteOptions& opt);

  // This handles options which are not in stereo_settings(), but
  // rather in 'opt'. So they are not config options set in
  // stereo.default but only command-line options.
  void addAspGlobalOptions(boost::program_options::options_description & description,
                           ASPGlobalOptions & opt);

  /// Structure holding variables
  class StereoSettings {
  public:
    StereoSettings();
    void initialize(vw::GdalWriteOptions& opt);
    void validate();
    void write_copy(int argc, char *argv[],
                     std::string const& input_file,
                     std::string const& output_file) const;
    bool is_search_defined() const;

    // ----------------
    // Public variables
    // ----------------

    // Preprocessing options
    std::string alignment_method;           /// Valid options are: [Homography, Epipolar, None]

    // Note: Below we use BBox2 rather than BBox2i to not choke on float inputs.
    // Do stereo in given regions only.
    vw::BBox2 left_image_crop_win;
    vw::BBox2 right_image_crop_win;

    // Data for use with bathymetry
    std::string left_bathy_mask, right_bathy_mask;
    std::string bathy_plane, output_cloud_type;
    double refraction_index;

    bool   force_use_entire_range;          /// Use entire dynamic range of image
    bool   individually_normalize;          /// If > 1, normalize the images
                                            ///         individually with their
                                            ///         own hi's and lo's
    int   ip_per_tile;                      ///< How many ip to find in each 1024^2 tile
    int   ip_per_image;                     ///< How many ip to find in each image
    int   matches_per_tile;                 ///< How many ip matches to find in each 1024^2 tile
    int   ip_detect_method;               ///< Method used for matching interest points
                                            /// 0 = Zack's integral Obalog method
                                            /// 1 = OpenCV SIFT method
                                            /// 2 = OpenCV ORB method
    double epipolar_threshold;              /// Max distance from epipolar line to search for IP matches.
    double ip_inlier_factor;                /// General scaling factor for IP finding, a larger value allows more IPs to match.
    double ip_uniqueness_thresh;            /// Min percentage distance between closest and second closest IP descriptors.
    double ip_nodata_radius;                /// Remove IP near nodata with this radius, in pixels.
    double ip_triangulation_max_error;      ///< Remove IP matches with triangulation error higher than this.
    int    ip_num_ransac_iterations;        ///< How many ransac iterations to do in ip matching.
    bool   disable_tri_filtering;           ///< Turn of tri-ip filtering.

    int num_scales;                         /// How many scales to use if detecting interest points with OBALoG. If not specified, 8 will be used.
    int    ip_edge_buffer_percent;          ///< When detecting IP, throw out points within this many % of pixels
                                            ///  of the left/right edges of the images being matched.
    bool   ip_normalize_tiles;              ///< Individually normalize tiles for IP detection.
    bool   ip_debug_images;                 ///< Write debug interest point images.

    double nodata_value;                    ///< Pixels with values less than or equal to this number are treated as no-data.
                                            //  This overrides the nodata values from input images.
    double nodata_pixel_percentage;         ///< Percentage of low-value pixels treated as no-data
    double nodata_stddev_thresh;            ///
    int    nodata_stddev_kernel;            ///< Kernel size of the nadata stddev calculation
    bool   skip_rough_homography;           ///< Use this if datum-based rough homography fails.
    bool   no_datum;                        ///< Do not assume a reliable datum exists
    bool   skip_image_normalization;        ///< Skip the step of normalizing the values of input images and removing nodata-pixels. Create instead symbolic links to original images.
    bool   force_reuse_match_files;         ///< Force reusing the match files even if older than the images or cameras
    bool   part_of_multiview_run;           ///< If this run is part of a larger multiview run
    std::string datum;                      ///< The datum to use with RPC camera models
    std::string match_files_prefix, clean_match_files_prefix; // Load matches from here
    std::string left_image_clip, right_image_clip;
    double global_alignment_threshold;        /// Max distance from the epipolar line when doing global affine epipolar alignment
    double local_alignment_threshold;         /// Max distance from the epipolar line when doing local affine epipolar alignment
    int    alignment_num_ransac_iterations;   ///< How many ransac iterations to do in global or local epipolar alignment transform computation
    vw::Vector2 outlier_removal_params;
    vw::Vector2i matches_per_tile_params;
    int band;
    bool allow_different_mapproject_gsd;
    vw::Vector2 ortho_heights;
    std::string output_prefix_override; // override the output prefix with this 
    std::string flann_method; // The method to use for FLANN matching

    // This option will be the default in the future and then it will go away
    bool aster_use_csm; // Use the CSM camera model with ASTER images
    bool accept_provided_mapproj_dem;

    // Correlation options
    float slogW;                      ///< Preprocessing filter width
    vw::uint16 pre_filter_mode;       // 0 = None
                                      // 1 = Gaussian Blur
                                      // 2 = Log Filter

    vw::uint16  seed_mode;            // 0 = None, use global search for each tile
                                      // 1 = Use low-res disparity from stereo
                                      // 2 = Use low-res disparity from provided DEM
                                      //     (see disparity-estimation-dem)
                                      // 3 = Use low-res disparity produced by sparse_disp
                                      //     (in development)

    int   min_num_ip;                 ///< Minimum number of IP's needed for search range estimation.

    float seed_percent_pad;           ///< Pad amount towards the IP found
    float disparity_range_expansion_percent; ///< Expand the estimated disparity range by this percentage before computing the stereo correlation with local alignment

    vw::uint16 cost_mode;             // 0 = absolute difference
                                      // 1 = squared difference
                                      // 2 = normalized cross correlation
                                      // 3 = census transform
                                      // 3 = ternary census transform
    float        xcorr_threshold;     // L-R vs R-L agreement threshold in pixels
    int          min_xcorr_level;     // Min level to perform xcorr check at, if specified.
    bool         save_lr_disp_diff;   // Save the L-R and R-L disparity difference
    vw::Vector2i corr_kernel;         // Correlation kernel
    vw::BBox2    search_range;        // Correlation search range
    vw::BBox2    corr_search_limit;   // Correlation search range limit
    std::string  ip_filter_using_dem; // Filter using given DEM and height difference
    vw::Vector2  elevation_limit;     // Expected range of elevation to limit results to.
    vw::BBox2    lon_lat_limit;       // Limit the triangulated interest points to this lonlat range


    int corr_max_levels;     // Max pyramid levels to process. 0 hits only once.
    double max_disp_spread;    // Max disparity spread
    bool compute_low_res_disparity_only;      // Skip the full-resolution disparity computation
    bool skip_low_res_disparity_comp;
    std::string disparity_estimation_dem;     // DEM to use in estimating the low-resolution disparity
    double disparity_estimation_dem_error; // Error (in meters) of the disparity estimation DEM
    int disparity_estimation_sample_rate;
    int    corr_timeout;              // Correlation timeout for a tile, in seconds
    int default_corr_timeout;         // Will be used to adjust corr_timeout
    std::string stereo_algorithm;     // See StereoSettings.cc for the possible values.
    int    corr_blob_filter_area;     // Use blob filtering in pyramidal correlation
    int    corr_tile_size_ovr;        // Override the default tile size used for processing.
    int    sgm_collar_size;           // Extra tile padding used for SGM calculation.
    vw::Vector2i sgm_search_buffer;   // Search padding in SGM around previous pyramid level disparity value.
    size_t corr_memory_limit_mb;      // Correlation memory limit, only important for SGM/MGM.
    bool   correlator_mode;           // Use the correlation logic only (including subpixel rfne).
    bool   stereo_debug;              // Write stereo debug images and messages
    bool   local_alignment_debug;     // Debug local alignment

    // Subpixel options

    bool subpix_from_blend;           // Read from -B.tif instead of -D.tif

    vw::uint16 subpixel_mode;         // 0 = none
                                      // 1 = parabola fitting
                                      // 2 = affine, bayes weighting
                                      // 3 = affine
                                      // 4 = Lucas-Kanade
                                      // 5 = affine, bayes EM weighting
    vw::Vector2i subpixel_kernel;     // Subpixel correlation kernel
    bool disable_h_subpixel, disable_v_subpixel;
    vw::uint16 subpixel_max_levels;   // Max pyramid levels to process. 0 hits only once.
    vw::uint16 phase_subpixel_accuracy;  // Phase subpixel is accurate to 1/this pixels


    // Experimental Subpixel options (mode 3 only)
    int subpixel_em_iter;
    int subpixel_affine_iter;
    int subpixel_pyramid_levels;

    // Filtering options
    int filter_mode;                  // Which filter mode to use
    vw::Vector2i rm_half_kernel;      // Low confidence pixel removal kernel size
    int    max_mean_diff;             // Max mean diff between pixel and neighbors
    int    rm_min_matches;            // Min # of pxl to be matched to keep pxl
    double rm_threshold;              // rm_treshold < disp[n]-disp[m] reject pxl
    double rm_quantile_percentile;    // For quantile based filtering, reject low-res correlation
    double rm_quantile_multiple;      //   values >  multiple * quantile.
    int    rm_cleanup_passes;         // Number of times to perform cleanup
                                      // in the post-processing phase
    int  erode_max_size;              // Max island size in pixels that it'll remove
    bool enable_fill_holes;           // If to enable hole-filling
    bool disable_fill_holes;          // This obsolete parameter is ignored
    int  fill_hole_max_size;          // Maximum hole size in pixels that we'll attempt to fill
    int   edge_buffer_size;           // Size of region filtered out of image edges.
    int   median_filter_size;         // Filter subpixel results with median filter of this size
    int   disp_smooth_size;           // Adaptive disparity smoothing size
    double disp_smooth_texture;        // Adaptive disparity smoothing max texture value
    bool  gotcha_disparity_refinement;
    std::string casp_go_param_file;

    // Triangulation options
    std::string universe_center;      // Center for the radius clipping
    float  near_universe_radius;      // Radius of the universe in meters
    float  far_universe_radius;       // Radius of the universe in meters
    std::string bundle_adjust_prefix; // Use the camera adjustments obtained by previously running bundle_adjust with the output prefix specified here.

    // Pull this many matches from the stereo disparity
    int num_matches_from_disparity, num_matches_from_disp_triplets;

    // Error propagation options
    bool propagate_errors;
    vw::Vector2 horizontal_stddev;
    double position_covariance_factor, orientation_covariance_factor;

    bool compute_error_vector;              // Compute the triangulation error vector, not just its length

    double min_triangulation_angle;           // min angle for valid triangulation
    double max_valid_triangulation_error;
    bool   use_least_squares;                 // Use a more rigorous triangulation
    bool   save_double_precision_point_cloud; // Save final point cloud in double precision rather than bringing the points closer to origin and saving as float (marginally more precision at 2x the storage).
    double point_cloud_rounding_error;        // How much to round the output point cloud values
    bool   compute_point_cloud_center_only;   // Only compute the center of triangulated point cloud and exit.
    bool   skip_point_cloud_center_comp;
    bool   unalign_disparity;                 // Compute disparity between unaligned images
    bool enable_atmospheric_refraction_correction;
    bool enable_velocity_aberration_correction;

    // stereo_gui options
    int grid_cols;
    vw::Vector2i window_size; // The size of the GUI window
    bool single_window;
    bool use_georef;
    bool hillshade;
    int lowest_resolution_subimage_num_pixels;
    double hillshade_azimuth, hillshade_elevation, gcp_sigma;
    bool view_matches, view_several_side_by_side, colorize, preview;
    std::string match_file, gcp_file, dem_file, csv_datum, csv_format_str, csv_srs, nvm, isis_cnet;
    bool delete_temporary_files_on_exit;
    bool create_image_pyramids_only, hide_all, nvm_no_shift;
    bool pairwise_matches, pairwise_clean_matches, no_georef;
    std::vector<std::string> vwip_files;
    vw::BBox2 zoom_proj_win;
    double min, max;
    int plot_point_radius, font_size;

    // stereo_parse options
    std::string tile_at_loc;
    vw::Vector2i parallel_tile_size;

    // Options for parallel_stereo. These are not used, but accept
    // them quietly so that when stereo_gui or stereo_parse is invoked
    // with a parallel_stereo command it would not fail.
    std::string nodes_list, ssh, sparse_disp_options, parallel_options, prev_run_prefix;
    int threads_multi, threads_single, processes, entry_point, stop_point, job_size_h, job_size_w;

    // Undocumented options. We don't want these exposed to the user.
    vw::BBox2i trans_crop_win;        // Left image crop window in respect to L.tif.
    bool attach_georeference_to_lowres_disparity;

    // Internal variable, to ensure we always initialize this class before using it
    bool initialized_stereo_settings;
  };

  /// Return the singleton instance of the stereo setting structure.
  /// The stereo settings struct is created the first time this method
  /// is invoked.  You must *always* access the stereo settings through this function.
  StereoSettings& stereo_settings();

  /// Custom parsers for ASP's stereo.default files
  boost::program_options::basic_parsed_options<char>
  parse_asp_config_file(std::basic_istream<char>&,
                        const boost::program_options::options_description&,
                        bool allow_unregistered = false);

  boost::program_options::basic_parsed_options<char>
  parse_asp_config_file(bool print_warning,
                        std::string const&,
                        const boost::program_options::options_description&,
                        bool allow_unregistered = false);

}

#endif//__ASP_CORE_STEREO_SETTINGS_H__
