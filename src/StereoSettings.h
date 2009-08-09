// __BEGIN_LICENSE__
// 
// Copyright (C) 2008 United States Government as represented by the
// Administrator of the National Aeronautics and Space Administration
// (NASA).  All Rights Reserved.
// 
// Copyright 2008 Carnegie Mellon University. All rights reserved.
// 
// This software is distributed under the NASA Open Source Agreement
// (NOSA), version 1.3.  The NOSA has been approved by the Open Source
// Initiative.  See the file COPYING at the top of the distribution
// directory tree for the complete NOSA document.
// 
// THE SUBJECT SOFTWARE IS PROVIDED "AS IS" WITHOUT ANY WARRANTY OF ANY
// KIND, EITHER EXPRESSED, IMPLIED, OR STATUTORY, INCLUDING, BUT NOT
// LIMITED TO, ANY WARRANTY THAT THE SUBJECT SOFTWARE WILL CONFORM TO
// SPECIFICATIONS, ANY IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR
// A PARTICULAR PURPOSE, OR FREEDOM FROM INFRINGEMENT, ANY WARRANTY THAT
// THE SUBJECT SOFTWARE WILL BE ERROR FREE, OR ANY WARRANTY THAT
// DOCUMENTATION, IF PROVIDED, WILL CONFORM TO THE SUBJECT SOFTWARE.
//
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

  // ----------------
  // Public variables
  // ----------------

  // Preprocessing options
  int epipolar_alignment;     /* Align images using the epipolar constraints */
  int keypoint_alignment;     /* Align images using the keypoint alignment method */
  int keypoint_align_subsampling;	   // if > 1, image sub-sampling factor for keypoint aligment (to speed things up)
  int individually_normalize; // if > 1, normalize the images individually with their own hi's and low
  int force_max_min;          // Use entire dynamic range of image..

  // Correlation Options
  int slog;		/* perform an slog (relpace the emboss) */
  int log;		/* perform a log (laplacian of the gaussian blur) */
  float slogW;
  int h_kern;			/* kernel width first pass */
  int v_kern;			/* kernel height first pass*/
  int subpixel_h_kern;			/* kernel width first pass */
  int subpixel_v_kern;			/* kernel height first pass*/
  int h_corr_max;		/* correlation window max x */
  int h_corr_min;		/* correlation window min x */
  int v_corr_max;		/* correlation window max y */
  int v_corr_min;		/* correlation window min y */
  int do_h_subpixel;
  int do_v_subpixel;
  int subpixel_mode;       /* Use the affine adaptive subpixel correlator (slow!) */
  float xcorr_treshold;
  float corrscore_rejection_treshold;
  int cost_blur;
  int cost_mode;

  // Filtering Options
  int rm_h_half_kern;		/* low confidence pixel removal kernel size */
  int rm_v_half_kern;
  int rm_min_matches;		/* min # of pxl to be matched to keep pxl */
  int rm_treshold;		/* rm_treshold < disp[n]-disp[m] reject pxl */ 
  int rm_cleanup_passes;  /* number of times to perform cleanup in the post-processing phase */
  int fill_holes_NURBS;  
  int mask_flatfield;    // Masks pixels in the input images that are less than 0.  (For use with apollo metric camera...)  
  
  // Triangulation Options
  float near_universe_radius;  	/* radius of the universe in meters */
  float far_universe_radius;  	/* radius of the universe in meters */
};

/// Return the singleton instance of the stereo setting structure.
/// The stereo settings struct is created the first time this method
/// is invoked.  You should *always* access the stereo settings
/// through this function.
StereoSettings& stereo_settings();
