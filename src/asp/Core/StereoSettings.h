// __BEGIN_LICENSE__
// Copyright (C) 2006-2009 United States Government as represented by
// the Administrator of the National Aeronautics and Space Administration.
// All Rights Reserved.
// __END_LICENSE__


/// \file StereoSettings.h
///

#include <boost/version.hpp>
#include <boost/program_options.hpp>
#include <iostream>
namespace po = boost::program_options;

class StereoSettings {
  po::options_description m_desc;
  po::variables_map m_vm;

public:
  StereoSettings();
  void read(std::string const& filename);
  void copy_settings(std::string const& filename, std::string const& destination);
  bool is_search_defined( void ) {
    return h_corr_min != 0 || h_corr_max != 0 ||
           v_corr_min != 0 || v_corr_max != 0;
  }

  // ----------------
  // Public variables
  // ----------------

  // Preprocessing options
  int epipolar_alignment;         /* Align images using the
                                     epipolar constraints */
  int keypoint_alignment;         /* Align images using the
                                     keypoint alignment method */
  int keypoint_align_subsampling; /* if > 1, image sub-sampling factor
                                     for keypoint aligment (to speed
                                     things up) */
  int individually_normalize;     /* if > 1, normalize the images
                                     individually with their
                                     own hi's and low */
  int force_max_min;          // Use entire dynamic range of image..
  int pre_filter_mode;        /* 0 = None
                                 1 = Gaussian Blur
                                 2 = Log Filter
                                 3 = SLog Filter  */
  float slogW;                // Preprocessing filter width

  // Correlation Options
  int h_kern;              /* kernel width first pass */
  int v_kern;              /* kernel height first pass*/
  int subpixel_h_kern;     /* kernel width first pass */
  int subpixel_v_kern;     /* kernel height first pass*/
  int h_corr_max;          /* correlation window max x */
  int h_corr_min;          /* correlation window min x */
  int v_corr_max;          /* correlation window max y */
  int v_corr_min;          /* correlation window min y */
  int do_h_subpixel;       /* Both of these must on    */
  int do_v_subpixel;
  int subpixel_mode;       /* 0 = parabola fitting
                              1 = affine, robust weighting
                              2 = affine, bayes weighting
                              3 = affine, bayes EM weighting */
  float xcorr_threshold;
  float corrscore_rejection_threshold;
  int cost_blur;
  int cost_mode;

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
  int mask_flatfield;      /* Masks pixels in the input images that are less
                              than 0. (For use with apollo metric camera...) */

  // Triangulation Options
  float near_universe_radius;  /* radius of the universe in meters */
  float far_universe_radius;   /* radius of the universe in meters */

  // System Settings
  std::string cache_dir;   /* DiskCacheViews will use this directory */
};

/// Return the singleton instance of the stereo setting structure.
/// The stereo settings struct is created the first time this method
/// is invoked.  You should *always* access the stereo settings
/// through this function.
StereoSettings& stereo_settings();
