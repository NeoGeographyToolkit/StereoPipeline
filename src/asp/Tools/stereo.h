// __BEGIN_LICENSE__
//  Copyright (c) 2009-2012, United States Government as represented by the
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


/// \file stereo.h
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
    input_dem, extra_argument1, extra_argument2, extra_argument3;

  // Settings
  std::string stereo_session_string, stereo_default_filename;
  boost::shared_ptr<asp::StereoSession> session; // Used to extract cameras
  vw::BBox2i left_image_crop_win;                     // Used to do stereo in a region

  // Output
  std::string out_prefix;
};

// Allows FileIO to correctly read/write these pixel types
namespace asp {

  // Parse input command line arguments
  void handle_arguments( int argc, char *argv[], Options& opt,
                         boost::program_options::options_description const&
                         additional_options);

  // Based on arguments from the user, guesses the session type
  void guess_session_type(Options& opt);

  // Register Session types
  void stereo_register_sessions();

  // Checks for obvious user mistakes
  void user_safety_check(Options const& opt);

  // Pre-correlation, used in both stereo_corr and stereo_corr_sub
  void pre_correlation( Options const& opt );

  // Produces D_sub
  void produce_lowres_disparity( Options const& opt );

  // Approximate search range by looking at interest point match file,
  // used in both stereo_corr and stereo_corr_sub.
  vw::BBox2i approximate_search_range( std::string const& left_image,
                                       std::string const& right_image,
                                       std::string const& match_filename,
                                       float scale );

} // end namespace vw

#endif//__ASP_STEREO_H__
