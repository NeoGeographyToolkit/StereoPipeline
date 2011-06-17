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
#include <vw/Stereo.h>

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
  vw::BBox2i search_range;                         // Correlation search window
  bool optimized_correlator, draft_mode;

  // Output
  std::string out_prefix, corr_debug_prefix;
};

// Allows FileIO to correctly read/write these pixel types
namespace vw {

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

  // Parse input command line arguments
  void handle_arguments( int argc, char *argv[], Options& opt ) {
    po::options_description general_options("");
    general_options.add_options()
      ("session-type,t", po::value(&opt.stereo_session_string), "Select the stereo session type to use for processing. [options: pinhole isis]")
      ("stereo-file,s", po::value(&opt.stereo_default_filename)->default_value("./stereo.default"), "Explicitly specify the stereo.default file to use. [default: ./stereo.default]")
      ("draft-mode", po::value(&opt.corr_debug_prefix),"Cause the pyramid correlator to save out debug imagery named with this prefix.")
      ("optimized-correlator", po::bool_switch(&opt.optimized_correlator)->default_value(false),
       "Use the optimized correlator instead of the pyramid correlator.");
    general_options.add( asp::BaseOptionsDescription(opt) );

    po::options_description positional("");
    positional.add_options()
      ("left-input-image", po::value(&opt.in_file1), "Left Input Image")
      ("right-input-image", po::value(&opt.in_file2), "Right Input Image")
      ("left-camera-model", po::value(&opt.cam_file1), "Left Camera Model File")
      ("right-camera-model", po::value(&opt.cam_file2), "Right Camera Model File")
      ("output-prefix", po::value(&opt.out_prefix), "Prefix for output filenames")
      ("extra_argument1", po::value(&opt.extra_arg1), "Extra Argument 1")
      ("extra_argument2", po::value(&opt.extra_arg2), "Extra Argument 2")
      ("extra_argument3", po::value(&opt.extra_arg3), "Extra Argument 3")
      ("extra_argument4", po::value(&opt.extra_arg4), "Extra Argument 4");

    po::positional_options_description positional_desc;
    positional_desc.add("left-input-image", 1);
    positional_desc.add("right-input-image", 1);
    positional_desc.add("left-camera-model", 1);
    positional_desc.add("right-camera-model", 1);
    positional_desc.add("output-prefix", 1);
    positional_desc.add("extra_argument1", 1);
    positional_desc.add("extra_argument2", 1);
    positional_desc.add("extra_argument3", 1);
    positional_desc.add("extra_argument4", 1);

    std::ostringstream usage;
    usage << "Usage: " << argv[0] << " [options] <Left_input_image> <Right_input_image> [Left_camera_file] [Right_camera_file] <output_file_prefix>\n"
          << "  Extensions are automaticaly added to the output files.\n"
          << "  Camera model arguments may be optional for some stereo session types (e.g. isis).\n"
          << "  Stereo parameters should be set in the stereo.default file.\n";

    po::variables_map vm =
      asp::check_command_line( argc, argv, opt, general_options,
                               positional, positional_desc, usage.str() );

    opt.draft_mode = vm.count("draft-mode");

    if (!vm.count("left-input-image") || !vm.count("right-input-image") ||
        !vm.count("left-camera-model") )
      vw_throw( ArgumentErr() << "Missing all of the correct input files.\n\n"
                << usage.str() << "\n" << general_options );

    // If the user hasn't specified a stereo session type, we take a
    // guess here based on the file suffixes.
    if (opt.stereo_session_string.empty()) {
      if ( asp::has_cam_extension( opt.cam_file1 ) &&
           asp::has_cam_extension( opt.cam_file2 ) ) {
        vw_out() << "\t--> Detected pinhole camera files. "
                 << "Executing pinhole stereo pipeline.\n";
        opt.stereo_session_string = "pinhole";
      } else if (boost::iends_with(opt.in_file1, ".cub") &&
                 boost::iends_with(opt.in_file2, ".cub")) {
        vw_out() << "\t--> Detected ISIS cube files. "
                 << "Executing ISIS stereo pipeline.\n";
        opt.stereo_session_string = "isis";
      } else {
        vw_throw( ArgumentErr() << "Could not determine stereo session type. "
                  << "Please set it explicitly.\n"
                  << "using the -t switch. Options include: [pinhole isis].\n" );
      }
    }

    // Some specialization here so that the user doesn't need to list
    // camera models on the command line for certain stereo session
    // types.  (e.g. isis).
    bool check_for_camera_models = true;
    if ( opt.stereo_session_string == "isis" ) {
      // Fix the ordering of the arguments if the user only supplies 3
      if (opt.out_prefix.empty())
        opt.out_prefix = opt.cam_file1;
      check_for_camera_models = false;
    }

    if ( check_for_camera_models &&
         ( opt.out_prefix.empty() || opt.cam_file2.empty() ) )
      vw_throw( ArgumentErr() << "\nMissing output-prefix or right camera model.\n" );

    fs::path out_prefix_path(opt.out_prefix);
    if (out_prefix_path.has_branch_path()) {
      if (!fs::is_directory(out_prefix_path.branch_path())) {
        vw_out() << "\nCreating output directory: "
                 << out_prefix_path.branch_path() << std::endl;
        fs::create_directory(out_prefix_path.branch_path());
      }
    }

    opt.session.reset( asp::StereoSession::create(opt.stereo_session_string) );
    opt.session->initialize(opt, opt.in_file1, opt.in_file2,
                            opt.cam_file1, opt.cam_file2,
                            opt.out_prefix, opt.extra_arg1, opt.extra_arg2,
                            opt.extra_arg3, opt.extra_arg4);

    // Finally read in the stereo settings
    stereo_settings().read(opt.stereo_default_filename);

    // Set search range from stereo.default file
    opt.search_range = BBox2i(Vector2i(stereo_settings().h_corr_min,
                                       stereo_settings().v_corr_min),
                              Vector2i(stereo_settings().h_corr_max,
                                       stereo_settings().v_corr_max));

    // The last thing we do before we get started is to copy the
    // stereo.default settings over into the results directory so that
    // we have a record of the most recent stereo.default that was used
    // with this data set.
    stereo_settings().copy_settings(opt.stereo_default_filename,
                                    opt.out_prefix + "-stereo.default");
  }

  // Register Session types
  void stereo_register_sessions() {

#if defined(ASP_HAVE_PKG_ISISIO) && ASP_HAVE_PKG_ISISIO == 1
    // Register the Isis file handler with the Vision Workbench
    // DiskImageResource system.
    DiskImageResource::register_file_type(".cub",
                                          DiskImageResourceIsis::type_static(),
                                          &DiskImageResourceIsis::construct_open,
                                          &DiskImageResourceIsis::construct_create);
#endif

    asp::StereoSession::register_session_type( "rmax", &asp::StereoSessionRmax::construct);
#if defined(ASP_HAVE_PKG_ISISIO) && ASP_HAVE_PKG_ISISIO == 1
    asp::StereoSession::register_session_type( "isis", &asp::StereoSessionIsis::construct);
#endif

  }

} // end namespace vw

#endif//__ASP_STEREO_H__
