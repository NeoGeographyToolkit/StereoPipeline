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

#include "StereoSettings.h"

#include <vw/Core/Thread.h>
#include <fstream>

// ---------------------------------------------------
// Create a single instance of the StereoSettings
// ---------------------------------------------------
namespace {
  vw::RunOnce stereo_settings_once = VW_RUNONCE_INIT;
  boost::shared_ptr<StereoSettings> stereo_settings_ptr;
  void init_stereo_settings() {
    stereo_settings_ptr = boost::shared_ptr<StereoSettings>(new StereoSettings());
  }
}

StereoSettings& stereo_settings() {
  stereo_settings_once.run( init_stereo_settings );
  return *stereo_settings_ptr;
}

// ----------------------------------------------------------
// Utilities for parsing lines out of the stereo.default file
// ----------------------------------------------------------

// Determine whether the given character is a space character.
inline bool is_space_char(int c) {
  return (c == ' ' || c == '\t' || c == '\n' || c == '\r');
}

// Read from stream until (but not including) the next non-space character.
inline void ignorespace(std::istream& s) {
  int c;
  if(s.eof())
    return;
  while((c = s.get()) != EOF && is_space_char(c));
  if(c != EOF)
    s.unget();
}

// Read from stream until (but not including) the beginning of the next line.
inline void ignoreline(std::istream& s) {
  int c;
  if(s.eof())
    return;
  while((c = s.get()) != EOF && c != '\n');
}

// Read from stream until (but not including) the next space character. Ignores space characters at beginning.
inline void getword(std::istream& s, std::string& str) {
  int c;
  str.clear();
  ignorespace(s);
  if(s.eof())
    return;
  while((c = s.get()) != EOF && !is_space_char(c))
    str.push_back(c);
  if(c != EOF)
    s.unget();
}

//--------------------------------------------------
// StereoSettings Methods
//--------------------------------------------------
StereoSettings::StereoSettings() {

#define ASSOC_INT(X,Y,V,D)             m_desc.add_options()(X, po::value<int>(&(this->Y))->default_value(V), D)
#define ASSOC_FLOAT(X,Y,V,D)           m_desc.add_options()(X, po::value<float>(&(this->Y))->default_value(V), D)
#define ASSOC_DOUBLE(X,Y,V,D)          m_desc.add_options()(X, po::value<double>(&(this->Y))->default_value(V), D)

  // Preprocessing options
  ASSOC_INT("DO_EPIPOLAR_ALIGNMENT", epipolar_alignment, 1, "Align images using epipolar constraints");
  ASSOC_INT("DO_INTERESTPOINT_ALIGNMENT", keypoint_alignment, 0, "Align images using the keypoint alignment method");
  ASSOC_INT("INTERESTPOINT_ALIGNMENT_SUBSAMPLING", keypoint_align_subsampling, 1, "Image sub-sampling factor for keypoint alignment.");

  // Correlation Options
  ASSOC_INT("DO_SLOG", slog, 1, "perform an slog (relpace the emboss)");
  ASSOC_INT("DO_LOG", log, 0, "perform a log (laplacian of gaussian)");
  ASSOC_FLOAT("SLOG_KERNEL_WIDTH", slogW, 1.5, "SIGMA for the gaussian blure in LOG and SLOG");
  ASSOC_INT("H_KERNEL", h_kern, 25, "kernel width");
  ASSOC_INT("V_KERNEL", v_kern, 25, "kernel height");  
  ASSOC_INT("SUBPIXEL_H_KERNEL", subpixel_h_kern, 35, "subpixel kernel width");
  ASSOC_INT("SUBPIXEL_V_KERNEL", subpixel_v_kern, 35, "subpixel kernel height");  
  ASSOC_INT("H_CORR_MAX", h_corr_max, -100, "correlation window size max x");
  ASSOC_INT("H_CORR_MIN", h_corr_min, 100, "correlation window size min x");
  ASSOC_INT("V_CORR_MIN", v_corr_min, -10, "correlation window size min y");
  ASSOC_INT("V_CORR_MAX", v_corr_max, 10, "correlation window size max y");
  ASSOC_INT("DO_H_SUBPIXEL", do_h_subpixel, 1, "Do vertical subpixel interpolation.");
  ASSOC_INT("DO_V_SUBPIXEL", do_v_subpixel, 1, "Do horizontal subpixel interpolation.");
  ASSOC_INT("DO_AFFINE_SUBPIXEL", do_affine_subpixel, 0, "Use the affine adaptive subpixel correlator (slower, but more accurate)");
  ASSOC_FLOAT("XCORR_THRESHOLD", xcorr_treshold, 2.0, "");
  ASSOC_FLOAT("CORRSCORE_REJECTION_THRESHOLD", corrscore_rejection_treshold, 1.1, "");
  ASSOC_INT("COST_BLUR", cost_blur, 1, "");
  ASSOC_INT("COST_TYPE", cost_type, 0, "");

  // Filtering Options
  ASSOC_INT("RM_H_HALF_KERN", rm_h_half_kern, 5, "low conf pixel removal kernel half size");
  ASSOC_INT("RM_V_HALF_KERN", rm_v_half_kern, 5, "");
  ASSOC_INT("RM_MIN_MATCHES", rm_min_matches, 60, "min # of pxls to be matched to keep pxl");
  ASSOC_INT("RM_TRESHOLD", rm_treshold, 3, "rm_treshold > disp[n]-disp[m] pixels are not matching");
  ASSOC_INT("RM_CLEANUP_PASSES", rm_cleanup_passes, 1, "number of passes for cleanup during the post-processing phase");
  ASSOC_INT("FILL_HOLES_NURBS", fill_holes_NURBS, 1, "fill holes using Larry's NURBS code");

  // Triangulation Options
  ASSOC_FLOAT("NEAR_UNIVERSE_RADIUS", near_universe_radius, 0.0, "radius of inner boundary of universe [m]");
  ASSOC_FLOAT("FAR_UNIVERSE_RADIUS", far_universe_radius, 0.0, "radius of outer boundary of universe [m]");

#undef ASSOC_INT
#undef ASSOC_FLOAT
#undef ASSOC_DOUBLE

  int argc = 1;
  char* argv[1];
  argv[0] = "dummyprogname";
  po::store(po::parse_command_line(argc, argv, m_desc), m_vm);
  po::notify(m_vm);
}

void StereoSettings::read(std::string const& filename) {

  std::ifstream fp(filename.c_str());
  if(!fp) {
    std::cerr << "Error: cannot open stereo default file: " << filename << "\n";
    exit(EXIT_FAILURE);
  }

  std::cout << "*************************************************************\n";
  std::cout << "Reading Stereo Settings file: " << filename << "\n";

  std::string name, value, line;
  int c;

  while(!fp.eof()) {
    ignorespace(fp);
    if(!fp.eof() && (c = fp.peek()) != '#') {
      std::istringstream ss; //NOTE: cannot move this up with other variable declarations because then calling store(parse_config_file()) multiple times does not work as expected
      getword(fp, name);
      getword(fp, value);
      line = name.append(" = ").append(value);
      ss.str(line);
      try {
        po::store(po::parse_config_file(ss, m_desc), m_vm);
      } catch (boost::program_options::unknown_option &e) {
        std::cout << "\tWARNING --> Unknown stereo settings option: " << line << "\n";
      }
    }
    ignoreline(fp);
  }
  
  po::notify(m_vm);
  std::cout << "*************************************************************\n";
  fp.close();
}
