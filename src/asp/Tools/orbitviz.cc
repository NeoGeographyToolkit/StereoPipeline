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


/// \file orbitviz.cc
///

/************************************************************************
 *     File: orbitviz.cc
 ************************************************************************/
#include <vw/Core.h>
#include <vw/Math.h>
#include <vw/FileIO/KML.h>
#include <vw/Camera.h>
#include <vw/Cartography.h>

#include <asp/Core/Macros.h>
#include <asp/Sessions/StereoSessionFactory.h>

#if defined(ASP_HAVE_PKG_ISISIO) && ASP_HAVE_PKG_ISISIO == 1
#include <asp/IsisIO/IsisCameraModel.h>
#endif

#include <iomanip>

#include <boost/algorithm/string.hpp>
#include <boost/program_options.hpp>
#include <boost/filesystem/path.hpp>
#include <boost/filesystem/operations.hpp>
#include <boost/filesystem/convenience.hpp>
#include <boost/filesystem/fstream.hpp>
using namespace vw;
using namespace vw::camera;
using namespace vw::cartography;
namespace po = boost::program_options;
namespace fs = boost::filesystem;

struct Options : public asp::BaseOptions {
  Options() : loading_image_camera_order(true) {}
  // Input
  std::vector<std::string> input_files;
  std::string stereo_session_string, path_to_outside_model;

  // Settings
  bool loading_image_camera_order, write_csv;
  std::string datum;

  // Output
  std::string out_file;
};

// MAIN
//***********************************************************************
void handle_arguments( int argc, char *argv[], Options& opt ) {
  po::options_description general_options("");
  general_options.add_options()
    ("output,o", po::value(&opt.out_file)->default_value("orbit.kml"),
     "The output kml file that will be written")
    ("session-type,t", po::value(&opt.stereo_session_string),
     "Select the stereo session type to use for processing. [options: pinhole isis]")
    ("reference-spheroid,r", po::value(&opt.datum)->default_value("moon"),
     "Set a reference surface to a hard coded value (one of [moon, mars, wgs84].)")
    ("use-path-to-dae-model,u", po::value(&opt.path_to_outside_model),
     "Instead of using an icon to mark a camera, use a 3D model with extension .dae")
    ("write-csv", "write a csv file with the orbital the data.");
  general_options.add( asp::BaseOptionsDescription(opt) );

  po::options_description positional("");
  positional.add_options()
    ("input-files", po::value(&opt.input_files) );

  po::positional_options_description positional_desc;
  positional_desc.add("input-files", -1);

  std::string usage("[options] <input image> <input camera model> <...and repeat...>\nNote: All cameras and their images must be of the same session type. Camera models only can be used as input for stereo sessions pinhole and isis.");
  bool allow_unregistered = false;
  std::vector<std::string> unregistered;
  po::variables_map vm =
    asp::check_command_line( argc, argv, opt, general_options, general_options,
                             positional, positional_desc, usage,
                             allow_unregistered, unregistered  );

  // Determining if feed only camera model
  if ( opt.input_files.size() == 1 )
    opt.loading_image_camera_order = false;
  else if ( opt.input_files.size() > 1 ) {
    std::string first_extension =
      opt.input_files[0].substr( opt.input_files[0].size()-4,4 );
    if ( boost::iends_with(opt.input_files[1], first_extension ) )
      opt.loading_image_camera_order = false;
  }

  if ( opt.input_files.size() == 0 ||
       (opt.loading_image_camera_order && opt.input_files.size() < 2) )
    vw_throw( ArgumentErr() << usage << general_options );

  opt.write_csv = vm.count("write-csv");

  // Look up for session type based on file extensions
  if (opt.stereo_session_string.empty()) {
    int testing_i = 0;
    if ( opt.loading_image_camera_order )
      testing_i = 1;
    if ( boost::iends_with(opt.input_files[testing_i], ".cahvor") ||
         boost::iends_with(opt.input_files[testing_i], ".cahv") ||
         boost::iends_with(opt.input_files[testing_i], ".pin") ||
         boost::iends_with(opt.input_files[testing_i], ".tsai") ) {
      vw_out() << "\t--> Detected pinhole camera file\n";
      opt.stereo_session_string = "pinhole";
    } else if (boost::iends_with(opt.input_files[testing_i], ".cub") ) {
      vw_out() << "\t--> Detected ISIS cube file\n";
      opt.stereo_session_string = "isis";
    } else {
      vw_throw( ArgumentErr() << "\n\n******************************************************************\n"
                << "Could not determine stereo session type.   Please set it explicitly\n"
                << "using the -t switch.\n"
                << "******************************************************************\n\n" );
    }
  }

}

int main(int argc, char* argv[]) {

  Options opt;
  try {
    handle_arguments( argc, argv, opt );

    typedef boost::scoped_ptr<asp::StereoSession> SessionPtr;
    SessionPtr session( asp::StereoSessionFactory::create(opt.stereo_session_string, opt ) );

    // Data to be loaded
    unsigned no_cameras = opt.loading_image_camera_order ? opt.input_files.size()/2 : opt.input_files.size();
    vw_out() << "Number of cameras: " << no_cameras << std::endl;
    std::vector<std::string> camera_names(no_cameras);
    cartography::XYZtoLonLatRadFunctor conv_func;

    // Copying file names
    for (unsigned load_i = 0, read_i = 0; load_i < no_cameras;
         load_i++) {
      camera_names[load_i] = opt.input_files[read_i];
      read_i += opt.loading_image_camera_order ? 2 : 1;
    }

    // Create the KML file.
    KMLFile kml( opt.out_file, "orbitviz" );
    // Style listing
    if ( opt.path_to_outside_model.empty() ) {
      // Placemark Style
      kml.append_style( "plane", "", 1.2,
                        "http://maps.google.com/mapfiles/kml/shapes/airports.png");
      kml.append_style( "plane_highlight", "", 1.4,
                        "http://maps.google.com/mapfiles/kml/shapes/airports.png");
      kml.append_stylemap( "camera_placemark", "plane",
                           "plane_highlight" );
    }

    // Load up datum
    cartography::Datum datum;
    if ( opt.datum == "mars" ) {
      datum.set_well_known_datum("D_MARS");
    } else if ( opt.datum == "moon" ) {
      datum.set_well_known_datum("D_MOON");
    } else if ( opt.datum == "wgs84" ) {
      datum.set_well_known_datum("WGS84");
    } else {
      vw_out() << "Unknown spheriod request: " << opt.datum << "\n";
      vw_out() << "->  Defaulting to WGS84\n";
      datum.set_well_known_datum("WGS84");
    }

    std::ofstream csv_file("orbit_positions.csv");
    if ( !csv_file.is_open() )
      vw_throw( IOErr() << "Unable to open output file.\n" );

    // Building Camera Models and then writing to KML
    for (unsigned load_i = 0, read_i = 0; load_i < no_cameras;
         load_i++) {
      boost::shared_ptr<camera::CameraModel> current_camera;
      if (opt.loading_image_camera_order)
        current_camera = session->camera_model( opt.input_files[read_i],
                                                opt.input_files[read_i+1] );
      else
        current_camera = session->camera_model( opt.input_files[read_i],
                                                opt.input_files[read_i] );

      if ( opt.write_csv ) {
        csv_file << camera_names[load_i] << ", ";

#if defined(ASP_HAVE_PKG_ISISIO) && ASP_HAVE_PKG_ISISIO == 1
        boost::shared_ptr<IsisCameraModel> isis_cam =
          boost::dynamic_pointer_cast<IsisCameraModel>(current_camera);
        if ( isis_cam != NULL ) {
          csv_file << isis_cam->serial_number() << ", ";
        }
#endif

        Vector3 xyz = current_camera->camera_center(Vector2());
        csv_file << std::setprecision(12);
        csv_file << xyz[0] << ", "
                 << xyz[1] << ", " << xyz[2] << "\n";
      }

      // Adding Placemarks
      Vector3 lon_lat_alt = conv_func(current_camera->camera_center(Vector2()));
      lon_lat_alt[2] -= datum.radius(lon_lat_alt[0], lon_lat_alt[1]);

      if (!opt.path_to_outside_model.empty())
        kml.append_model( opt.path_to_outside_model,
                          lon_lat_alt.x(), lon_lat_alt.y(),
                          inverse(current_camera->camera_pose(Vector2())),
                          camera_names[load_i], "",
                          lon_lat_alt[2], 1 );
      else {
        kml.append_placemark( lon_lat_alt.x(), lon_lat_alt.y(),
                              camera_names[load_i], "", "camera_placemark",
                              lon_lat_alt[2], true );
      }

      // Increment
      read_i += opt.loading_image_camera_order ? 2 : 1;
    }

    csv_file.close();
    kml.close_kml();
  } ASP_STANDARD_CATCHES;

  return 0;
}
