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

#ifndef __ASP_CORE_STEREO_SETTINGS_H__
#define __ASP_CORE_STEREO_SETTINGS_H__

#include <asp/Core/Common.h>
#include <boost/program_options.hpp>
#include <boost/program_options/detail/config_file.hpp>

namespace asp {

  class StereoSession; // Forward declaration

  // 'Global Scoped' Variables
  struct Options : asp::BaseOptions {
    // Input
    std::string in_file1, in_file2, cam_file1, cam_file2,
      input_dem, extra_argument1, extra_argument2, extra_argument3;

    // Settings
    std::string stereo_session_string, stereo_default_filename;
    boost::shared_ptr<asp::StereoSession> session; // Used to extract cameras
    // Note: Below we use BBox2 rather than BBox2i to not choke on float inputs.
    vw::BBox2 left_image_crop_win;                 // For stereo in a region
    // Output
    std::string out_prefix;
  };

  // Program Options for each executable/step
  struct PreProcessingDescription : public boost::program_options::options_description { PreProcessingDescription(); };
  struct CorrelationDescription   : public boost::program_options::options_description { CorrelationDescription  (); };
  struct SubpixelDescription      : public boost::program_options::options_description { SubpixelDescription     (); };
  struct FilteringDescription     : public boost::program_options::options_description { FilteringDescription    (); };
  struct TriangulationDescription : public boost::program_options::options_description { TriangulationDescription(); };
  struct DGDescription            : public boost::program_options::options_description { DGDescription           (); };
  struct UndocOptsDescription     : public boost::program_options::options_description { UndocOptsDescription    (); };

  boost::program_options::options_description
  generate_config_file_options( asp::BaseOptions& opt );

  // Structure holding variables
  class StereoSettings {
  public:
    StereoSettings();
    void validate();
    void write_copy( int argc, char *argv[],
                     std::string const& input_file,
                     std::string const& output_file ) const;
    bool is_search_defined() const;

    // ----------------
    // Public variables
    // ----------------

    // Preprocessing options
    std::string alignment_method;           // Valid options are: [Homography, Epipolar, None]
    bool   force_use_entire_range;          // Use entire dynamic range of image
    bool   individually_normalize;          // If > 1, normalize the images
                                            //         individually with their
                                            //         own hi's and lo's
    double nodata_value;                    // Pixels with values less than or equal to this number are treated as no-data.
                                            // This overrides the nodata values from input images.
    double nodata_pixel_percentage;         // Percentage of low-value pixels treated as no-data
    double nodata_optimal_threshold_factor; // Pixels with values less than this factor times the optimal Otsu threshold
                                            // are treated as no-data
    bool   skip_image_normalization;        // Skip the step of normalizing the values of input images and removing nodata-pixels. Create instead symbolic links to original images.
    
    // Correlation Options
    float slogW;                      // Preprocessing filter width
    vw::uint16 pre_filter_mode;       // 0 = None
                                      // 1 = Gaussian Blur
                                      // 2 = Log Filter
                                      // 3 = SLog Filter

    vw::uint16  seed_mode;            // 0 = None, use global search for each tile
                                      // 1 = Use low-res disparity from stereo
                                      // 2 = Use low-res disparity from provided DEM
                                      //     (see disparity-estimation-dem)
                                      // 3 = Use low-res disparity produced by sparse_disp
                                      //     (in development)

    float seed_percent_pad;           // Pad amound towards the IP found
    vw::uint16 cost_mode;             // 0 = absolute difference
                                      // 1 = squared difference
                                      // 2 = normalized cross correlation
    float        xcorr_threshold;     // L-R vs R-L agreement threshold in pixels
    vw::Vector2i corr_kernel;         // Correlation kernel
    vw::BBox2i   search_range;        // Correlation search range
    vw::uint16   corr_max_levels;     // Max pyramid levels to process. 0 hits only once.
    bool compute_low_res_disparity_only;      // Skip the full-resolution disparity computation
    std::string disparity_estimation_dem;     // DEM to use in estimating the low-resolution disparity
    double disparity_estimation_dem_error; // Error (in meters) of the disparity estimation DEM
    bool   use_local_homography;      // Apply a local homography in each tile
    int    corr_timeout;              // Correlation timeout for a tile, in seconds

    // Subpixel Options
    vw::uint16 subpixel_mode;         // 0 = none
                                      // 1 = parabola fitting
                                      // 2 = affine, bayes weighting
                                      // 3 = affine
                                      // 4 = Lucas-Kanade
                                      // 5 = affine, bayes EM weighting
    vw::Vector2i subpixel_kernel;     // Subpixel correlation kernel
    bool disable_h_subpixel, disable_v_subpixel;
    vw::uint16 subpixel_max_levels;   // Max pyramid levels to process. 0 hits only once.

    // Experimental Subpixel Options (mode 3 only)
    int subpixel_em_iter;
    int subpixel_affine_iter;
    int subpixel_pyramid_levels;

    // Filtering Options
    int filter_mode;                  // Which filter mode to use
    vw::Vector2i rm_half_kernel;      // Low confidence pixel removal kernel size
    int max_mean_diff;                // Max mean diff between pixel and neighbors
    int rm_min_matches;               // Min # of pxl to be matched to keep pxl
    int rm_threshold;                 // rm_treshold < disp[n]-disp[m] reject pxl
    int rm_cleanup_passes;            // Number of times to perform cleanup
                                      // in the post-processing phase
    int  erode_max_size;              // Max island size in pixels that it'll remove
    bool enable_fill_holes;           // If to enable hole-filling
    bool disable_fill_holes;          // This obsolete parameter is ignored
    int  fill_hole_max_size;          // Maximum hole size in pixels that we'll attempt
                                      // to fill
    bool mask_flatfield;              // Masks pixels in the input images that are less
                                      // than 0 (for use with Apollo Metric Camera)

    // Triangulation Options
    std::string universe_center;      // Center for the radius clipping
    float  near_universe_radius;      // Radius of the universe in meters
    float  far_universe_radius;       // Radius of the universe in meters
    float  max_valid_triangulation_error; // points with error > this are rm from cloud
    bool   use_least_squares;         // Use a more rigorous triangulation
    bool   save_double_precision_point_cloud; // Save final point cloud in double precision rather than bringing the points closer to origin and saving as float (marginally more precision at 2x the storage).
    double point_cloud_rounding_error;// How much to round the output point cloud values
    bool   compute_point_cloud_center_only; // Only compute the center of triangulated point cloud and exit.
    bool   compute_error_vector;      // Compute the triangulation error vector, not just its length

    // DG Options
    bool disable_correct_velocity_aberration;

    // Undocumented options
    vw::BBox2i trans_crop_win;        // Left image crop window in respect to L.tif.

  };

  /// Return the singleton instance of the stereo setting structure.
  /// The stereo settings struct is created the first time this method
  /// is invoked.  You must *always* access the stereo settings
  /// through this function.
  StereoSettings& stereo_settings();

  // Custom readers for Boost Program Options
  class asp_config_file_iterator : public boost::program_options::detail::common_config_file_iterator {
    boost::shared_ptr<std::basic_istream<char> > is;
  private:
    bool getline(std::string& s); // Used to precondition string before reading
  public:
    asp_config_file_iterator() {
      found_eof();
    }

    // Creates a config file parser for the specified stream.
    asp_config_file_iterator(std::basic_istream<char>& is,
                             const std::set<std::string>& allowed_options,
                             bool allow_unregistered = false);
  };

  // Custom Parsers for ASP's stereo.default files
  boost::program_options::basic_parsed_options<char>
  parse_asp_config_file( std::basic_istream<char>&,
                         const boost::program_options::options_description&,
                         bool allow_unregistered = false );

  boost::program_options::basic_parsed_options<char>
  parse_asp_config_file( std::string const&,
                         const boost::program_options::options_description&,
                         bool allow_unregistered = false );

}

#endif//__ASP_CORE_STEREO_SETTINGS_H__
