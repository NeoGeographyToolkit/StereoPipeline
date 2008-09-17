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

/// \file stereo_gui.cc
///

// Qt
#include <QApplication>
#include <QWidget>

// Boost
#include <boost/program_options.hpp>
using namespace boost;
namespace po = boost::program_options;

// Standard Library
#include <iostream>

// VW
#include <vw/Core.h>
#include <vw/Image.h>
#include <vw/Math.h>
#include <vw/FileIO.h>
#include <vw/Stereo.h>
using namespace vw;
using namespace vw::math;
using namespace vw::camera;
using namespace vw::stereo;

#include "gui/MainWindow.h"
#include "StereoSettings.h"
#include "stereo.h"
#include "StereoSession.h"
#include "MRO/DiskImageResourceDDD.h"	   // support for Malin DDD image files
#include "HRSC/StereoSessionHRSC.h"
#include "MOC/StereoSessionMOC.h"
#include "apollo/StereoSessionApolloMetric.h"
#include "MRO/StereoSessionCTX.h"
#include "RMAX/StereoSessionRmax.h"

#if defined(ASP_HAVE_PKG_ISIS) && ASP_HAVE_PKG_ISIS == 1 
#include "Isis/DiskImageResourceIsis.h"
#include "Isis/StereoSessionIsis.h"
#endif

// Allows FileIO to correctly read/write these pixel types
namespace vw {
  template<> struct PixelFormatID<Vector3>   { static const PixelFormatEnum value = VW_PIXEL_GENERIC_3_CHANNEL; };
  template<> struct PixelFormatID<PixelDisparity<float> >   { static const PixelFormatEnum value = VW_PIXEL_GENERIC_3_CHANNEL; };
}

void print_usage(po::options_description const& visible_options) {
  std::cout << "\nUsage: stereo_gui [options] <Left_input_image> <Right_input_image> [Left_camera_file] [Right_camera_file] <output_file_prefix>\n"
            << "  Extensions are automaticaly added to the output files.\n"
            << "  Camera model arguments may be optional for some stereo session types (e.g. isis).\n"
            << "  Stereo parameters should be set in the stereo.default file.\n\n";
  std::cout << visible_options << std::endl;
}

int main(int argc, char *argv[]) {

  // The default file type are automatically registered the first time
  // a file is opened or created, however we want to override some of
  // the defaults, so we explicitly register them here before registering
  // our own FileIO driver code.
  DiskImageResource::register_default_file_types();

  // Register the DDD file handler with the Vision Workbench
  // DiskImageResource system.  DDD is the proprietary format used by
  // Malin Space Science Systems.
  DiskImageResource::register_file_type(".ddd",
                                        DiskImageResourceDDD::type_static(),
                                        &DiskImageResourceDDD::construct_open,
                                        &DiskImageResourceDDD::construct_create);
  
#if defined(ASP_HAVE_PKG_ISIS) && ASP_HAVE_PKG_ISIS == 1 
  // Register the Isis file handler with the Vision Workbench
  // DiskImageResource system.
  DiskImageResource::register_file_type(".cub",
                                        DiskImageResourceIsis::type_static(),
                                        &DiskImageResourceIsis::construct_open,
                                        &DiskImageResourceIsis::construct_create);
#endif 
  /*************************************/
  /* Parsing of command line arguments */
  /*************************************/

  // Boost has a nice command line parsing utility, which we use here
  // to specify the type, size, help string, etc, of the command line
  // arguments.
  int debug_level;
  unsigned cache_size;
  std::string stereo_session_string;
  std::string stereo_default_filename;
  std::string in_file1, in_file2, cam_file1, cam_file2, extra_arg1, extra_arg2, extra_arg3, extra_arg4;
  std::string out_prefix;

  po::options_description visible_options("Options");
  visible_options.add_options()
    ("help,h", "Display this help message")
    ("cache", po::value<unsigned>(&cache_size)->default_value(1800), "Cache size, in megabytes")
    ("session-type,t", po::value<std::string>(&stereo_session_string), "Select the stereo session type to use for processing. [options: pinhole isis]")
    ("stereo-file,s", po::value<std::string>(&stereo_default_filename)->default_value("./stereo.default"), "Explicitly specify the stereo.default file to use. [default: ./stereo.default]")
    ("debug-level,d", po::value<int>(&debug_level)->default_value(vw::DebugMessage-1), "Set the debugging output level. (0-50+)");

  po::options_description positional_options("Positional Options");
  positional_options.add_options()
    ("left-input-image", po::value<std::string>(&in_file1), "Left Input Image")
    ("right-input-image", po::value<std::string>(&in_file2), "Right Input Image")
    ("left-camera-model", po::value<std::string>(&cam_file1), "Left Camera Model File")
    ("right-camera-model", po::value<std::string>(&cam_file2), "Right Camera Model File")
    ("output-prefix", po::value<std::string>(&out_prefix), "Prefix for output filenames")
    ("extra_argument1", po::value<std::string>(&extra_arg1), "Extra Argument 1")
    ("extra_argument2", po::value<std::string>(&extra_arg2), "Extra Argument 2")
    ("extra_argument3", po::value<std::string>(&extra_arg3), "Extra Argument 3")
    ("extra_argument4", po::value<std::string>(&extra_arg4), "Extra Argument 4");
  po::positional_options_description positional_options_desc;
  positional_options_desc.add("left-input-image", 1);
  positional_options_desc.add("right-input-image", 1);
  positional_options_desc.add("left-camera-model", 1);
  positional_options_desc.add("right-camera-model", 1);
  positional_options_desc.add("output-prefix", 1);
  positional_options_desc.add("extra_argument1", 1);
  positional_options_desc.add("extra_argument2", 1);
  positional_options_desc.add("extra_argument3", 1);
  positional_options_desc.add("extra_argument4", 1);

  po::options_description all_options;
  all_options.add(visible_options).add(positional_options);

  po::variables_map vm;
  po::store( po::command_line_parser( argc, argv ).options(all_options).positional(positional_options_desc).run(), vm );
  po::notify( vm );

  // If the command line wasn't properly formed or the user requested
  // help, we print an usage message.
  if( vm.count("help") ) {
    print_usage(visible_options);
    exit(0);
  }
  
  // Read the stereo.conf file
  stereo_settings().read(stereo_default_filename);

  // Set the Vision Workbench debug level
  set_debug_level(debug_level);
  Cache::system_cache().resize( cache_size*1024*1024 ); // Set cache to 1Gb

  // Create a fresh stereo session and query it for the camera models.
  StereoSession::register_session_type( "hrsc", &StereoSessionHRSC::construct);
  StereoSession::register_session_type( "moc", &StereoSessionMOC::construct);
  StereoSession::register_session_type( "metric", &StereoSessionApolloMetric::construct);
  StereoSession::register_session_type( "ctx", &StereoSessionCTX::construct);
  StereoSession::register_session_type( "rmax", &StereoSessionRmax::construct);
#if defined(ASP_HAVE_PKG_ISIS) && ASP_HAVE_PKG_ISIS == 1 
  StereoSession::register_session_type( "isis", &StereoSessionIsis::construct);
#endif
  StereoSession* session = StereoSession::create(stereo_session_string);
  session->initialize(in_file1, in_file2, cam_file1, cam_file2, 
                      out_prefix, extra_arg1, extra_arg2, extra_arg3, extra_arg4);

  
  // Start up the Qt GUI
  QApplication app(argc, argv);
  MainWindow main_window(argc, argv);
  main_window.show();
  return app.exec(); 
}
