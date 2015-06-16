// __BEGIN_LICENSE__
//  Copyright (c) 2006-2013, United States Government as represented by the
//  Administrator of the National Aeronautics and Space Administration. All
//  rights reserved.
//
//  The NASA Vision Workbench is licensed under the Apache License,
//  Version 2.0 (the "License"); you may not use this file except in
//  compliance with the License. You may obtain a copy of the License at
//  http://www.apache.org/licenses/LICENSE-2.0
//
//  Unless required by applicable law or agreed to in writing, software
//  distributed under the License is distributed on an "AS IS" BASIS,
//  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
//  See the License for the specific language governing permissions and
//  limitations under the License.
// __END_LICENSE__


/// \file stereo_gui.cc
///
/// The Vision Workbench image viewer.
///

// Qt
#include <QApplication>
#include <QWidget>

// Boost
#include <boost/program_options.hpp>
using namespace boost;
using namespace std;
namespace po = boost::program_options;

// VW
#include <vw/Core.h>
#include <vw/Image.h>
#include <vw/Math.h>
#include <vw/FileIO.h>
using namespace vw;

#include <asp/gui/MainWindow.h>

// Allows FileIO to correctly read/write unusual pixel types
namespace vw {
  template<> struct PixelFormatID<Vector3>   { static const PixelFormatEnum value = VW_PIXEL_GENERIC_3_CHANNEL; };
}

void print_usage(po::options_description const& visible_options) {
  vw_out() << "\nUsage: stereo_gui [options] <image file> \n";
  vw_out() << "\nHere is a quick list of stereo_gui keybindings:\n\n"
           << "  drag mouse - moves the image around\n"
           << "  mousewheel - zooms in and out   <-- also works with two-finger-drag gestures on some laptops\n"
	   << "     + shift - zoom in and out much slower.\n"
	   << "  double-click and drag (two-finger-click and drag) - adjust gain\n"
           << "  +/-   -   Change requested transaction_id.\n"
           << "  e - turn on \"exact\" transaction_id matching (otherwise stereo_gui will show the tile with the largest t_id <= requested t_id)\n"
           << "  t - turn on visualization of tile boundaries\n"
           << "  1-4 - View R,G,B,Alpha channels separately\n"
           << "  0 - View all channels together as RGBA\n"
           << "  o - turn on 'offset' mode.  Drag mouse left and right to shift pixel intensities by offset\n"
           << "  g - turn on 'gain' mode.  Drag mouse left and right to multiply pixel intensities by gain\n"
           << "  v - turn on 'gamma' mode. Drag mouse left and right to scale pixels by gamma value\n"
           << "  i - toggle nearest neighbor/bilinear interpolation\n\n";
  vw_out() << visible_options << "<image files>" << std::endl;
}

int main(int argc, char *argv[]) {

  unsigned cache_size;
  float nodata_value;
  string geom;
  bool ignore_georef, hillshade;
  // Set up command line options
  po::options_description visible_options("Options");
  visible_options.add_options()
    ("help,h", "Display this help message")
    ("ignore-georef,g", po::bool_switch(&ignore_georef)->default_value(false)->implicit_value(true), "Do not read the georeference even when available.")
    ("hillshade,h", po::bool_switch(&hillshade)->default_value(false)->implicit_value(true), "Generate the hillshade of the current DEM.")
    ("nodata-value", po::value<float>(&nodata_value), "Choose a \"nodata\" value in the image to treat as transparent.")
    ("geometry", po::value<string>(&geom)->default_value("1200x800"), "Choose the window geometry (width and height).")
    ("cache", po::value<unsigned>(&cache_size)->default_value(1000), "Cache size, in megabytes");

  po::options_description all_options;
  all_options.add(visible_options);

  // Parse and store options
  po::variables_map vm;
  po::parsed_options parsed = po::command_line_parser( argc, argv ).options(all_options).allow_unregistered().run();
  po::store(parsed, vm );

  std::vector<std::string> images
    = po::collect_unrecognized(parsed.options,
                               po::include_positional);
  po::notify( vm );

  // If the command line wasn't properly formed or the user requested
  // help, we print an usage message.
  if( vm.count("help") || images.empty() ) {
    print_usage(visible_options);
    return 0;
  }

  // Set the Vision Workbench cache size
  vw_settings().set_system_cache_size( cache_size*1024*1024 );

  // Start up the Qt GUI
  QApplication app(argc, argv);
  vw::gui::MainWindow main_window(images, geom, ignore_georef, hillshade, argc, argv);
  main_window.show();
  try {
    app.exec();
  } catch (const vw::Exception& e) {
    vw_out() << "An unexpected error occurred: " << e.what() << "\nExiting\n\n";
  }

  return 0;
}
