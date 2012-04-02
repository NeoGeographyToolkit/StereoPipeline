// __BEGIN_LICENSE__
// Copyright (C) 2006-2011 United States Government as represented by
// the Administrator of the National Aeronautics and Space Administration.
// All Rights Reserved.
// __END_LICENSE__


/// \file stereo.cc
///

#ifndef __ASP_STEREO_H__
#define __ASP_STEREO_H__

#include <boost/algorithm/string.hpp>

#include <vw/Core.h>
#include <vw/Image.h>
#include <vw/Math.h>
#include <vw/FileIO.h>

#include <asp/Core/StereoSettings.h>
#include <asp/Core/MedianFilter.h>
#include <asp/Core/Macros.h>
#include <asp/Core/Common.h>
#include <asp/Sessions.h>

namespace po = boost::program_options;
namespace fs = boost::filesystem;

// Support for ISIS image files
#if defined(ASP_HAVE_PKG_ISISIO) && ASP_HAVE_PKG_ISISIO == 1
#include <asp/IsisIO/DiskImageResourceIsis.h>
#endif

// Boost headers
#include <boost/thread/xtime.hpp>
// Posix time is not fully supported in the version of Boost for RHEL
// Workstation 4
#ifdef __APPLE__
#include <boost/date_time/posix_time/posix_time.hpp>
#else
#include <ctime>
#endif

// The stereo pipeline has several stages, which are enumerated below.
enum { PREPROCESSING = 0,
       CORRELATION,
       REFINEMENT,
       FILTERING,
       POINT_CLOUD,
       WIRE_MESH,
       NUM_STAGES};

// 'Global Scoped' Variables
struct Options : asp::BaseOptions {
  // Input
  std::string in_file1, in_file2, cam_file1, cam_file2,
    extra_arg1, extra_arg2, extra_arg3, extra_arg4;

  // Settings
  std::string stereo_session_string, stereo_default_filename;
  boost::shared_ptr<asp::StereoSession> session;   // Used to extract cameras
  bool optimized_correlator, draft_mode;

  // Output
  std::string out_prefix, corr_debug_prefix;
};

// Allows FileIO to correctly read/write these pixel types
namespace vw {

  // Print time function
  std::string current_posix_time_string();

  // Parse input command line arguments
  void handle_arguments( int argc, char *argv[], Options& opt );

  // Register Session types
  void stereo_register_sessions();

} // end namespace vw

#endif//__ASP_STEREO_H__
