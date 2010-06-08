// __BEGIN_LICENSE__
// Copyright (C) 2006-2010 United States Government as represented by
// the Administrator of the National Aeronautics and Space Administration.
// All Rights Reserved.
// __END_LICENSE__


/// \file stereo.cc
///

#ifndef __ASP_STEREO_H__
#define __ASP_STEREO_H__

#include <boost/algorithm/string.hpp>
#include <boost/program_options.hpp>
#include <boost/filesystem/path.hpp>
#include <boost/filesystem/operations.hpp>
#include <boost/filesystem/convenience.hpp>
#include <boost/filesystem/fstream.hpp>

namespace po = boost::program_options;
namespace fs = boost::filesystem;

#include <vw/Core.h>
#include <vw/Image.h>
#include <vw/Math.h>
#include <vw/FileIO.h>
#include <vw/Camera/CameraModel.h>
#include <vw/Stereo.h>
#include <vw/Cartography.h>
#include <vw/Math/EulerAngles.h>
#include <vw/InterestPoint.h>

#include <asp/Sessions.h>
#include <asp/Core/StereoSettings.h>
#include <asp/Core/MedianFilter.h>
#include <asp/Core/BlobIndexThreaded.h>
#include <asp/Core/InpaintView.h>
#include <asp/Core/ErodeView.h>
#include <asp/Core/Macros.h>

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
struct Options {
  // Input
  std::string in_file1, in_file2, cam_file1, cam_file2,
    extra_arg1, extra_arg2, extra_arg3, extra_arg4;

  // Settings
  int entry_point, num_threads;
  std::string stereo_session_string, stereo_default_filename;
  boost::shared_ptr<StereoSession> session;        // Used to extract cameras
  vw::DiskImageResourceGDAL::Options gdal_options; // Repeated format options
  vw::Vector2i raster_tile_size;                   // Write tile size
  vw::BBox2i search_range;                         // Correlation search window
  bool optimized_correlator, draft_mode;

  // Output
  std::string out_prefix, corr_debug_prefix;
};

// Allows FileIO to correctly read/write these pixel types
namespace vw {

  template<> struct PixelFormatID<Vector3>   { static const PixelFormatEnum value = VW_PIXEL_GENERIC_3_CHANNEL; };

  // Duplicate matches for any given interest point probably indicate a
  // poor match, so we cull those out here.
  void remove_duplicates(std::vector<ip::InterestPoint> &ip1,
                         std::vector<ip::InterestPoint> &ip2) {
    std::vector<ip::InterestPoint> new_ip1, new_ip2;

    for (unsigned i = 0; i < ip1.size(); ++i) {
      bool bad_entry = false;
      for (unsigned j = 0; j < ip1.size(); ++j) {
        if (i != j &&
            ((ip1[i].x == ip1[j].x && ip1[i].y == ip1[j].y) ||
             (ip2[i].x == ip2[j].x && ip2[i].y == ip2[j].y)) ) {
          bad_entry = true;
        }
      }
      if (!bad_entry) {
        new_ip1.push_back(ip1[i]);
        new_ip2.push_back(ip2[i]);
      }
    }

    ip1 = new_ip1;
    ip2 = new_ip2;
  }

  // Posix time is not fully supported in the version of Boost for RHEL
  // Workstation 4
#ifndef __APPLE__
  inline std::string
    current_posix_time_string()
  {
    char time_string[2048];
    time_t t = time(0);
    struct tm* time_struct = localtime(&t);
    strftime(time_string, 2048, "%F %T", time_struct);
    return std::string(time_string);
  }
#else
  inline std::string
    current_posix_time_string()
  {
    std::ostringstream time_string_stream;
    time_string_stream << boost::posix_time::second_clock::local_time();
    return time_string_stream.str();
  }
#endif

} // end namespace vw

#endif//__ASP_STEREO_H__
