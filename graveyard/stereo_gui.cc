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
#include <QSplashScreen>

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
#include "gui/StereoGuiSession.h"
#include "StereoSettings.h"
#include "stereo.h"
#include "StereoSession.h"
#include "Isis/DiskImageResourceIsis.h"
#include "Isis/StereoSessionIsis.h"

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

  // Register the Isis file handler with the Vision Workbench
  // DiskImageResource system.
  DiskImageResource::register_file_type(".cub",
                                        DiskImageResourceIsis::type_static(),
                                        &DiskImageResourceIsis::construct_open,
                                        &DiskImageResourceIsis::construct_create);

  /*************************************/
  /* Parsing of command line arguments */
  /*************************************/

  // Boost has a nice command line parsing utility, which we use here
  // to specify the type, size, help string, etc, of the command line
  // arguments.
  int debug_level;
  unsigned cache_size;
  std::string stereo_default_filename;
  std::string left_input, right_input, output_prefix;

  po::options_description visible_options("Options");
  visible_options.add_options()
    ("help,h", "Display this help message")
    ("cache", po::value<unsigned>(&cache_size)->default_value(1800), "Cache size, in megabytes")
    ("stereo-file,s", po::value<std::string>(&stereo_default_filename)->default_value("./stereo.default"), "Explicitly specify the stereo.default file to use. [default: ./stereo.default]")
    ("debug-level,d", po::value<int>(&debug_level)->default_value(vw::DebugMessage-1), "Set the debugging output level. (0-50+)");

  po::options_description positional_options("Positional Options");
  positional_options.add_options()
    ("left-input-image", po::value<std::string>(&left_input), "Left Input Image")
    ("right-input-image", po::value<std::string>(&right_input), "Right Input Image")
    ("output-prefix", po::value<std::string>(&output_prefix), "Prefix for output filenames");
  po::positional_options_description positional_options_desc;
  positional_options_desc.add("left-input-image", 1);
  positional_options_desc.add("right-input-image", 1);
  positional_options_desc.add("output-prefix", 1);

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
  vw_system_cache().resize( cache_size*1024*1024 ); // Set cache to 1Gb

  // Create a fresh stereo session and query it for the camera models.
  StereoSession::register_session_type( "isis", &StereoSessionIsis::construct);
  stereo_gui_session().set_session(StereoSession::create("isis"));
  stereo_gui_session().session()->initialize(left_input, right_input,
                                             "not used", "not used",
                                             output_prefix,"","","","");
  stereo_gui_session().set_left_input_image(left_input);
  stereo_gui_session().set_right_input_image(right_input);
  stereo_gui_session().set_output_prefix(output_prefix);
  
  // Start up the Qt GUI
  QApplication app(argc, argv);

//   // Show the splash screen
//   QPixmap pixmap("/Users/mbroxton/Desktop/asp_gui_splash.png");
//   QSplashScreen splash(pixmap, Qt::WindowStaysOnTopHint);
//   splash.showMessage("v3.0", Qt::AlignRight | Qt::AlignBottom, Qt::white);
//   splash.show();
//   app.processEvents();

  MainWindow main_window;
  main_window.show();
  return app.exec(); 
}
