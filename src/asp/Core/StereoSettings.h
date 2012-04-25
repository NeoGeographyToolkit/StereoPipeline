// __BEGIN_LICENSE__
// Copyright (C) 2006-2011 United States Government as represented by
// the Administrator of the National Aeronautics and Space Administration.
// All Rights Reserved.
// __END_LICENSE__


/// \file StereoSettings.h
///

#include <boost/version.hpp>
#include <boost/program_options.hpp>
#include <iostream>

#include <vw/Math/Vector.h>
#include <vw/Math/BBox.h>

class StereoSettings {
  boost::program_options::options_description m_desc;
  boost::program_options::variables_map m_vm;

  void validate();

public:
  StereoSettings();
  void read(std::string const& filename);
  void copy_settings(std::string const& filename, std::string const& destination);
  bool is_search_defined() const;

  // ----------------
  // Public variables
  // ----------------

  // Preprocessing options
  std::string alignment_method; // Valid options are: [Homography, Epipolar, None]
  vw::uint16 individually_normalize; /* if > 1, normalize the images
                                        individually with their
                                        own hi's and low */
  vw::uint16 force_max_min;    // Use entire dynamic range of image..
  vw::uint16 pre_filter_mode;   /* 0 = None
                                   1 = Gaussian Blur
                                   2 = Log Filter
                                   3 = SLog Filter  */
  float slogW;                 // Preprocessing filter width

  // Correlation Options
  vw::uint16  seed_option;     /* 0 = User global search for each tile
                                  1 = Narrow search for each tile to low
                                      res disparity seed
                                  2 = Affine Transform and narrow search
                                      based on disparity seed */
  vw::Vector2i kernel;         // Correlation kernel
  vw::Vector2i subpixel_kernel;// Subpixel correlation kernel
  vw::BBox2i search_range;     // Correlation search range
  vw::uint16 do_h_subpixel;
  vw::uint16 do_v_subpixel;
  vw::uint16 subpixel_mode;    /* 0 = parabola fitting
                                  1 = affine, robust weighting
                                  2 = affine, bayes weighting
                                  3 = affine, bayes EM weighting */
  float xcorr_threshold;
  vw::uint16 cost_mode;        /* 0 = absolute difference
                                  1 = squared difference
                                  2 = normalized cross correlation */

  // EMSubpixelCorrelator Options (mode 3 only)
  int subpixel_affine_iter;
  int subpixel_em_iter;
  int subpixel_pyramid_levels;

  // Filtering Options
  int rm_h_half_kern;      /* low confidence pixel removal kernel size */
  int rm_v_half_kern;
  int rm_min_matches;      /* min # of pxl to be matched to keep pxl */
  int rm_threshold;        /* rm_treshold < disp[n]-disp[m] reject pxl */
  int rm_cleanup_passes;   /* number of times to perform cleanup
                              in the post-processing phase */
  int erode_max_size;      /* Max island size in pixels that it'll remove*/
  int fill_holes;
  int fill_hole_max_size;  /* Maximum hole size in pixels that we'll attempt
                              to fill */
  vw::uint16 mask_flatfield;/* Masks pixels in the input images that are less
                              than 0. (For use with apollo metric camera...) */

  // Triangulation Options
  std::string universe_center; /* center for the radius clipping   */
  float near_universe_radius;  /* radius of the universe in meters */
  float far_universe_radius;   /* radius of the universe in meters */
  vw::uint16 use_least_squares;/* use a more rigorous triangulation */

  // System Settings
  std::string cache_dir;    /* DiskCacheViews will use this directory */
  std::string tif_compress; // Options are [None, LZW, Deflate, Packbits]
};

/// Return the singleton instance of the stereo setting structure.
/// The stereo settings struct is created the first time this method
/// is invoked.  You should *always* access the stereo settings
/// through this function.
StereoSettings& stereo_settings();
