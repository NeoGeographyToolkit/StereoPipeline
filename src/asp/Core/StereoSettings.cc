// __BEGIN_LICENSE__
// Copyright (C) 2006-2010 United States Government as represented by
// the Administrator of the National Aeronautics and Space Administration.
// All Rights Reserved.
// __END_LICENSE__


/// \file StereoSettings.h
///
#include <asp/Core/StereoSettings.h>

#include <vw/Core/Thread.h>
#include <vw/Core/Log.h>
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
  while((c = s.get()) != EOF && is_space_char(c)){}
  if(c != EOF)
    s.unget();
}

// Read from stream until (but not including) the beginning of the next line.
inline void ignoreline(std::istream& s) {
  int c;
  if(s.eof())
    return;
  while((c = s.get()) != EOF && c != '\n'){}
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
#define ASSOC_STRING(X,Y,V,D)          m_desc.add_options()(X, po::value<std::string>(&(this->Y))->default_value(V), D)

  // ---------------------
  // Preprocessing options
  // ---------------------

  // Alignment
  ASSOC_INT("DO_INTERESTPOINT_ALIGNMENT", keypoint_alignment, 0, "Align images using the interest point alignment method");
  ASSOC_INT("INTERESTPOINT_ALIGNMENT_SUBSAMPLING", keypoint_align_subsampling, 1, "Image sub-sampling factor for keypoint alignment.");
  ASSOC_INT("DO_EPIPOLAR_ALIGNMENT", epipolar_alignment, 0, "Align images using epipolar constraints");

  // Image normalization
  ASSOC_INT("FORCE_USE_ENTIRE_RANGE", force_max_min, 0, "Use images entire values, otherwise compress image range to -+2.5 sigmas around mean.");
  ASSOC_INT("DO_INDIVIDUAL_NORMALIZATION", individually_normalize, 0, "Normalize each image individually before processsing.");

  // -------------------
  // Correlation Options
  // -------------------

  // Preproc filter
  ASSOC_INT("PREPROCESSING_FILTER_MODE", pre_filter_mode, 3, "selects the preprocessing filter");
  ASSOC_FLOAT("SLOG_KERNEL_WIDTH", slogW, 1.5, "SIGMA for the gaussian blure in LOG and SLOG");

  // Integer correlator
  ASSOC_INT("COST_MODE", cost_mode, 2, "0 - absolute different, 1 - squared difference, 2 - normalized cross correlation");
  ASSOC_FLOAT("XCORR_THRESHOLD", xcorr_threshold, 2.0, "");
  ASSOC_FLOAT("CORRSCORE_REJECTION_THRESHOLD", corrscore_rejection_threshold, 1.1, "");
  ASSOC_INT("COST_BLUR", cost_blur, 0, "Reduces the number of missing pixels by blurring the fitness landscape computed by the cost function.");
  ASSOC_INT("H_KERNEL", h_kern, 25, "kernel width");
  ASSOC_INT("V_KERNEL", v_kern, 25, "kernel height");

  ASSOC_INT("H_CORR_MAX", h_corr_max, 0, "correlation window size max x");
  ASSOC_INT("H_CORR_MIN", h_corr_min, 0, "correlation window size min x");
  ASSOC_INT("V_CORR_MAX", v_corr_max, 0, "correlation window size max y");
  ASSOC_INT("V_CORR_MIN", v_corr_min, 0, "correlation window size min y");

  ASSOC_INT("SUBPIXEL_MODE", subpixel_mode, 2, "0 - no subpixel, 1 - parabola, 2 - bayes EM");
  ASSOC_INT("SUBPIXEL_H_KERNEL", subpixel_h_kern, 35, "subpixel kernel width");
  ASSOC_INT("SUBPIXEL_V_KERNEL", subpixel_v_kern, 35, "subpixel kernel height");
  ASSOC_INT("DO_H_SUBPIXEL", do_h_subpixel, 1, "Do vertical subpixel interpolation.");
  ASSOC_INT("DO_V_SUBPIXEL", do_v_subpixel, 1, "Do horizontal subpixel interpolation.");

  // EMSubpixelCorrelator options
  ASSOC_INT("SUBPIXEL_EM_ITER", subpixel_em_iter, 15, "Maximum number of EM iterations for EMSubpixelCorrelator");
  ASSOC_INT("SUBPIXEL_AFFINE_ITER", subpixel_affine_iter, 5, "Maximum number of affine optimization iterations for EMSubpixelCorrelator");
  ASSOC_INT("SUBPIXEL_PYRAMID_LEVELS", subpixel_pyramid_levels, 3, "Number of pyramid levels for EMSubpixelCorrelator");

  // Filtering Options
  ASSOC_INT("RM_H_HALF_KERN", rm_h_half_kern, 5, "low conf pixel removal kernel half size");
  ASSOC_INT("RM_V_HALF_KERN", rm_v_half_kern, 5, "");
  ASSOC_INT("RM_MIN_MATCHES", rm_min_matches, 60, "min # of pxls to be matched to keep pxl");
  ASSOC_INT("RM_THRESHOLD", rm_threshold, 3, "rm_threshold > disp[n]-disp[m] pixels are not matching");
  ASSOC_INT("RM_CLEANUP_PASSES", rm_cleanup_passes, 1, "number of passes for cleanup during the post-processing phase");
  ASSOC_INT("ERODE_MAX_SIZE", erode_max_size, 1000, "max size of islands that should be removed");
  ASSOC_INT("FILL_HOLES", fill_holes, 1, "fill holes using an inpainting method");
  ASSOC_INT("FILL_HOLE_MAX_SIZE", fill_hole_max_size, 100000, "max size in pixels that should be filled");
  ASSOC_INT("MASK_FLATFIELD", mask_flatfield, 0, "mask pixels that are less than 0. (for use with apollo metric camera only!)");

  // Triangulation Options
  ASSOC_STRING("UNIVERSE_CENTER", universe_center, "NONE", "center for radius measurements [CAMERA, ZERO, NONE]");
  ASSOC_FLOAT("NEAR_UNIVERSE_RADIUS", near_universe_radius, 0.0, "radius of inner boundary of universe [m]");
  ASSOC_FLOAT("FAR_UNIVERSE_RADIUS", far_universe_radius, 0.0, "radius of outer boundary of universe [m]");

  // System Settings
  ASSOC_STRING("CACHE_DIR", cache_dir, "/tmp", "Change if can't write large files to /tmp (i.e. Super Computer)");

#undef ASSOC_INT
#undef ASSOC_FLOAT
#undef ASSOC_DOUBLE
#undef ASSOC_STRING

  int argc = 1;
  char* argv[1];
  argv[0] = (char*)malloc(2*sizeof(char));
  argv[0][0] = 'a';
  po::store(po::parse_command_line(argc, argv, m_desc), m_vm);
  po::notify(m_vm);
}

void StereoSettings::read(std::string const& filename) {

  std::ifstream fp(filename.c_str());
  if(!fp) {
    std::cerr << "Error: cannot open stereo default file: " << filename << "\n";
    exit(EXIT_FAILURE);
  }

  vw::vw_out() << "*************************************************************\n";
  vw::vw_out() << "Reading Stereo Settings file: " << filename << "\n";

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
        vw::vw_out() << "\tWARNING --> Unknown stereo settings option: " << line << "\n";
      }
    }
    ignoreline(fp);
  }

  po::notify(m_vm);
  vw::vw_out() << "*************************************************************\n";
  fp.close();
}

void StereoSettings::copy_settings(std::string const& filename, std::string const& destination) {
  std::ifstream in(filename.c_str());
  std::ofstream out(destination.c_str());
  out<<in.rdbuf(); // copy file
  in.close();
  out.close();
}
