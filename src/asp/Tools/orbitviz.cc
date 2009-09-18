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

/// \file orbitviz.cc
///

/************************************************************************
 *     File: orbitviz.cc
 ************************************************************************/
#include <boost/algorithm/string.hpp>
#include <boost/program_options.hpp>
#include <boost/filesystem/path.hpp>
#include <boost/filesystem/operations.hpp>
#include <boost/filesystem/convenience.hpp>
#include <boost/filesystem/fstream.hpp>
using namespace boost;
namespace po = boost::program_options;
namespace fs = boost::filesystem;

#include <vw/Core.h>
#include <vw/Image.h>
#include <vw/Math.h>
#include <vw/FileIO.h>
#include <vw/Camera.h>
#include <vw/Stereo.h>
#include <vw/Cartography.h>
using namespace vw;
using namespace vw::camera;
using namespace vw::stereo;
using namespace vw::cartography;

#include <asp/Sessions.h>
#include <asp/Core/DiskImageResourceDDD.h>

#if defined(ASP_HAVE_PKG_ISIS) && ASP_HAVE_PKG_ISIS == 1
#include <asp/IsisIO/DiskImageResourceIsis.h>
#endif

//***********************************************************************
// MAIN
//***********************************************************************

int main(int argc, char* argv[]) {

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

  // Register all stereo session types
#if defined(ASP_HAVE_PKG_SPICE) && ASP_HAVE_PKG_SPICE == 1
  StereoSession::register_session_type( "hrsc", &StereoSessionHRSC::construct);
  StereoSession::register_session_type( "moc", &StereoSessionMOC::construct);
  StereoSession::register_session_type( "ctx", &StereoSessionCTX::construct);
#endif
#if defined(ASP_HAVE_PKG_ISIS) && ASP_HAVE_PKG_ISIS == 1
  StereoSession::register_session_type( "isis", &StereoSessionIsis::construct);
#endif

  /*************************************/
  /* Parsing of command line arguments */
  /*************************************/

  // Boost has a nice command line parsing utility, which we use here
  // to specify the type, size, help string, etc, of the command line
  // arguments.
  std::string stereo_session_string;
  std::vector<std::string> input_files;
  std::string out_file;
  double scale;

  po::options_description visible_options("Options");
  visible_options.add_options()
    ("help,h", "Display this help message")
    ("output,o", po::value<std::string>(&out_file)->default_value("orbit.kml"), "The output kml file that will be written")
    ("scale,s", po::value<double>(&scale)->default_value(1.0), "Scale the size of the coordinate axes by this amount. Ex: To scale moon alt. measures up to earth size, use 3.66")
    ("session-type,t", "Select the stereo session type to use for processing. [default: pinhole]")
    ("use-simple-placemarks", "Draw simple icons at camera locations, instead of a coordinate model");

  po::options_description hidden_options("");
  hidden_options.add_options()
    ("input-files", po::value<std::vector<std::string> >(&input_files));

  po::options_description options("Allowed Options");
  options.add(visible_options).add(hidden_options);

  po::positional_options_description p;
  p.add("input-files", -1);

  po::variables_map vm;
  po::store( po::command_line_parser( argc, argv ).options(options).positional(p).run(), vm );
  po::notify( vm );

  // If the command line wasn't properly formed or the user requested
  // help, we print an usage message.
  std::ostringstream help;
  help << "\nUsage: " << argv[0] << " [options] <input image> <input camera model> <...and repeat...>\n\n";
  help << "Note: All cameras and their images must be of the same session\ntype. Must have at least 2 cameras and models. In the event of\nusing just straight cubes with positioning information, leave a\nplace holder for camera model.\n\n";
  help << visible_options << std::endl;

  // Checking to see if the user flubbed
  if( vm.count("help") ||
      input_files.size() < 2 ) {
    std::cout << help.str();
    return 1;
  }

  // Look up for session type based on file extensions
  if (stereo_session_string.size() == 0) {
    if ( boost::iends_with(input_files[1], ".cahvor") ||
         boost::iends_with(input_files[1], ".cahv") ||
         boost::iends_with(input_files[1], ".pin") ||
         boost::iends_with(input_files[1], ".tsai") ) {
      vw_out(0) << "\t--> Detected pinhole camera file\n";
      stereo_session_string = "pinhole";
    }
    else if (boost::iends_with(input_files[0], ".cub") ) {
      vw_out(0) << "\t--> Detected ISIS cube file\n";
      stereo_session_string = "isis";
    }
    else {
      vw_out(0) << "\n\n******************************************************************\n";
      vw_out(0) << "Could not determine stereo session type.   Please set it explicitly\n";
      vw_out(0) << "using the -t switch.\n";
      vw_out(0) << "******************************************************************\n\n";
      exit(0);
    }
  }

  // Additional Error checks on arguments
  bool cube_only_isis = false;
  if ( stereo_session_string != "isis" && input_files.size() < 4 ) {
    std::cout << help.str();
    return 1;
  } else if ( "isis" && boost::iends_with(input_files[1], ".cub") )
    cube_only_isis = true;

  StereoSession* session = StereoSession::create(stereo_session_string);

  // Data to be loaded
  unsigned no_cameras = cube_only_isis ? input_files.size() : input_files.size() / 2;
  std::cout << "Number of cameras: " << no_cameras << std::endl;
  std::vector<boost::shared_ptr<camera::CameraModel> > camera_models(no_cameras);
  std::vector<std::string> camera_names(no_cameras);

  // Copying file names
  for (unsigned load_i = 0, read_i = 0; load_i < no_cameras;
       load_i++) {
    camera_names[load_i] = input_files[read_i];
    read_i += cube_only_isis ? 1 : 2;
  }

  // Building Camera Models
  for (unsigned load_i = 0, read_i = 0; load_i < no_cameras;
       load_i++) {
    if (cube_only_isis) {
      camera_models[load_i] = session->camera_model( input_files[read_i],
                                                     input_files[read_i] );
      read_i++;
    } else {
      camera_models[load_i] = session->camera_model( input_files[read_i],
                                                     input_files[read_i+1] );
      read_i+=2;
    }
  }

  // Create the KML file.
  KMLFile kml( out_file, "orbitviz" );
  // Style listing
  if (vm.count("use-simple-placemarks")) {
    // Placemark Style
    kml.append_style( "plane", "", 1.2,
                      "http://maps.google.com/mapfiles/kml/shapes/airports.png");
    kml.append_style( "plane_highlight", "", 1.4,
                      "http://maps.google.com/mapfiles/kml/shapes/airports.png");
    kml.append_stylemap( "camera_placemark", "plane",
                         "plane_highlight" );
  }
  // Placemarks
  for ( unsigned i = 0; i < camera_models.size(); i++ )
    if (!vm.count("use-simple-placemarks"))
      kml.append_coordinate( camera_models[i]->camera_center(Vector2()),
                             camera_models[i]->camera_pose(Vector2()),
                             camera_names[i], "", scale );
    else {
      // Converting to lon lat radius
      cartography::XYZtoLonLatRadFunctor func;
      Vector3 lon_lat_alt = func(camera_models[i]->camera_center(Vector2()));
      kml.append_placemark( lon_lat_alt.x(), lon_lat_alt.y(),
                            camera_names[i], "", "camera_placemark",
                            lon_lat_alt.z()*scale - 6371e3, true );
    }
  kml.close_kml();
  exit(0);
}
