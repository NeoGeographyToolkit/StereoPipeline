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

// Support for ISIS image files
#if defined(ASP_HAVE_PKG_ISISIO) && ASP_HAVE_PKG_ISISIO == 1
#include <asp/IsisIO/DiskImageResourceIsis.h>
#endif

// Boost headers
#include <boost/thread/xtime.hpp>
// Posix time is not fully supported in the version of Boost for RHEL Workstation 4
#ifdef __APPLE__
#include <boost/date_time/posix_time/posix_time.hpp>
#else
#include <ctime>
#endif

namespace po = boost::program_options;
namespace fs = boost::filesystem;

/// The stereo pipeline has several stages, which are enumerated below.
enum { PREPROCESSING = 0,
       CORRELATION,
       REFINEMENT,
       FILTERING,
       POINT_CLOUD,
       WIRE_MESH,
       NUM_STAGES};

// Allows FileIO to correctly read/write these pixel types
namespace asp {

  /// Transform the crop window to be in reference to L.tif
  vw::BBox2i transformed_crop_win(ASPGlobalOptions const& opt);

  /// Parse the command line options for multi-view stereo
  void parse_multiview(int argc, char* argv[],
                       boost::program_options::options_description const&
                       additional_options,
                       bool verbose,
                       std::string & output_prefix,
                       std::vector<ASPGlobalOptions> & opt_vec,
                       bool exit_early = false);

  /// Parse input command line arguments
  void handle_arguments( int argc, char *argv[], ASPGlobalOptions& opt,
                         boost::program_options::options_description const&
                         additional_options,
                         bool allow_unregistered,
                         std::vector<std::string> & unregistered,
                         std::string & usage, bool exit_early = false);

  /// Register DiskImageResource types that are not included in Vision Workbench.
  void stereo_register_sessions();

  /// Checks for obvious user mistakes
  /// - Throws if any incompatible settings are found.
  void user_safety_checks(ASPGlobalOptions const& opt);


  bool skip_image_normalization(ASPGlobalOptions const& opt);

} // end namespace vw

#endif//__ASP_STEREO_H__
