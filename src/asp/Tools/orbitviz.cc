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
//#include <vw/FileIO/FileUtils.h>
#include <vw/Camera.h>
#include <vw/Cartography.h>
#include <vw/InterestPoint/InterestData.h>

#include <asp/Core/Macros.h>
#include <asp/Core/StereoSettings.h>
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
//namespace fs = boost::filesystem;

struct Options : public asp::BaseOptions {
  Options() : seperate_camera_files(true) {}
  // Input
  std::vector<std::string> input_files;
  std::string stereo_session_string, 
              path_to_outside_model, 
              bundle_adjust_prefix;

  // Settings
  bool seperate_camera_files, write_csv, load_camera_solve, hide_labels;
  std::string datum;
  double model_scale; ///< Size scaling applied to 3D models

  // Output
  std::string out_file;
};


// TODO: Update the function in VW
/// Returns the extension of a file.
std::string get_extension( std::string const& input, bool make_lower=true ) {
  boost::filesystem::path ipath( input );
  std::string ext = ipath.extension().string();
  if (make_lower)
    boost::algorithm::to_lower(ext);
  return ext;
}

/// Strip the directory out of a file path
std::string strip_directory( std::string const& input){
 boost::filesystem::path p(input); 
 return p.filename().string();
}

/// Strip the directory and extension out of a file path
std::string strip_directory_and_extension( std::string const& input){
 boost::filesystem::path p(input); 
 return p.stem().string();
}

// Would be nice to have this in a function
/// Populate a list of all files in a directory.
/// - Returns the number of files found.
/// - If an extension is passed in, files must match the extension.
size_t get_files_in_folder(std::string              const& folder,
                           std::vector<std::string>      & output,
                           std::string              const& ext="")
{
  output.clear();
  
  // Handle invalid inputs
  if(!boost::filesystem::exists(folder) || !boost::filesystem::is_directory(folder)) 
    return 0;

  boost::filesystem::directory_iterator it(folder);
  boost::filesystem::directory_iterator endit;

  if (ext != ""){ // Check the extension
    while(it != endit) {
        if(boost::filesystem::is_regular_file(*it) && it->path().extension() == ext) 
          output.push_back(it->path().filename().string());
        ++it;
    }
  }
  else{ // No extension check
    while(it != endit) {
        if(boost::filesystem::is_regular_file(*it)) 
          output.push_back(it->path().filename().string());
        ++it;
    }
  }
  return output.size();
}



// TODO: Eliminate bool input and move somewhere else.
/// Seperates a list of files into camera files and image files.
/// - The camera files may be the same as the image files.
size_t split_files_list(std::vector<std::string> const& input_files,
                        std::vector<std::string>      & image_files, 
                        std::vector<std::string>      & camera_files,
                        const bool seperate_camera_files) {

  size_t num_cameras = seperate_camera_files ? input_files.size()/2 : input_files.size();
  vw_out() << "Number of cameras: " << num_cameras << std::endl;
  
  // Split the list of image files and camera files
  image_files.resize (num_cameras);
  camera_files.resize(num_cameras);
  for (size_t i = 0, store=0; store < num_cameras; ++store) {
    image_files[store] = input_files[i];
    if (seperate_camera_files){ // Camera file comes after the image file
      camera_files[store] = input_files[i+1];
      i += 2;
    } else { // Camera file is the same as the image file
      camera_files[store] = camera_files[store];
      i += 1;
    }
  } // End loop through cameras
  
  return num_cameras;
}

/// Get a list of the files in the solver output folder
size_t get_files_from_solver_folder(std::string                 const& solver_folder,
                                      std::vector<std::string>       & image_files, 
                                      std::vector<std::string>       & camera_files,
                                      std::vector<std::vector<int> > & matched_cameras) {
  image_files.clear();
  camera_files.clear();
  matched_cameras.clear();
  std::vector<std::string> solver_files;
  get_files_in_folder(solver_folder, solver_files);
  
  // Identify the image extension
  std::string image_ext = "";
  for (size_t i=0; i<solver_files.size(); ++i) {
    if (asp::has_image_extension(solver_files[i])) {
      image_ext = get_extension(solver_files[i], false);
      break;
    }
  }
  vw_out() << "image_ext: " << image_ext << std::endl;
  if (image_ext == "")
    vw_throw( ArgumentErr() << "Failed to find any image files in the camera_solve directory: "
                            << solver_folder);
  
  // Grab a list of just the image files
  size_t num_images = get_files_in_folder(solver_folder, image_files, image_ext);
  vw_out() << "Number of cameras: " << num_images << std::endl;
  
  // Make a list of the camera model files
  camera_files.resize(num_images);
  for (size_t i=0; i<num_images; ++i)
    camera_files[i] = solver_folder + "/" + image_files[i] + ".final.tsai";
    
  // Search for IP match files
  std::vector<std::string> match_files;
  size_t num_matches = get_files_in_folder(solver_folder, match_files, ".match");
  matched_cameras.resize(num_images);
  
  // Ignore any match files where there are not many matches
  const size_t MIN_MATCHES = 30; // This is the default value, but it could be made an option.
  std::vector<ip::InterestPoint> ip1, ip2;
  for (size_t m=0; m<num_matches; ++m) {
    vw::ip::read_binary_match_file(solver_folder+ "/"+match_files[m], ip1, ip2);
    //std::cout << "Read " << ip1.size() << " matches from file " << match_files[m] << std::endl;
    if (ip1.size() < MIN_MATCHES)
      match_files[m] = "";
  }
  
  // Figure out which image pairs are matched
  std::string f1, f2;
  for (size_t i=0; i<num_images; ++i) {
    matched_cameras[i].clear(); // Start with empty list of matches
    f1 = strip_directory_and_extension(image_files[i]);
    //std::cout <<"f1 = " << f1 << std::endl;
    for (size_t j=0; j<num_images; ++j) {
      if (i == j) // No self-matching!
        continue;
      f2 = strip_directory_and_extension(image_files[j]);
      //std::cout <<"- f2 = " << f2 << std::endl;
      for (size_t m=0; m<num_matches; ++m) {
        // If both file names are contained in the match file name,
        //  make a match between image files i and j
        if ( (match_files[m].find(f1) != std::string::npos) && 
             (match_files[m].find(f2) != std::string::npos)   ) {
          //std::cout <<"= MATCH = " << match_files[m] << std::endl;
          matched_cameras[i].push_back(j);
          match_files[m] = ""; // Clear this name so we don't use the match twice
        }
      }
    }
  } // End nested loops for setting up file matches
    
  return num_images;
}



// MAIN
//***********************************************************************
void handle_arguments( int argc, char *argv[], Options& opt ) {
  po::options_description general_options("");
  general_options.add_options()
    ("output,o",                po::value(&opt.out_file)->default_value("orbit.kml"),
          "The output kml file that will be written")
    ("session-type,t",          po::value(&opt.stereo_session_string),
          "Select the stereo session type to use for processing. [options: pinhole isis]")
    ("load-camera-solve",       po::bool_switch(&opt.load_camera_solve)->default_value(false)->implicit_value(true),
          "Load the results from a run of the camera-solve tool. The only positional argument must be the path to the camera-solve output folder.")
    ("reference-spheroid,r",    po::value(&opt.datum)->default_value("moon"),
          "Set a reference surface to a hard coded value (one of [moon, mars, wgs84].)")
    ("use-path-to-dae-model,u", po::value(&opt.path_to_outside_model),
          "Instead of using an icon to mark a camera, use a 3D model with extension .dae")
    // The KML class applies a model scale of 3000 * this value.
    ("model-scale",             po::value(&opt.model_scale)->default_value(1.0/30.0),
          "Scale factor applied to 3D model size.")
    ("hide-labels",             po::bool_switch(&opt.hide_labels)->default_value(false)->implicit_value(true),
          "Hide image names unless the camera is highlighted.")
    ("write-csv", "write a csv file with the orbital the data.")
    ("bundle-adjust-prefix",    po::value(&opt.bundle_adjust_prefix),
          "Use the camera adjustment obtained by previously running bundle_adjust with this output prefix.");
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
    opt.seperate_camera_files = false;
  else if ( opt.input_files.size() > 1 ) {
    // If the files have different extensions, set opt.seperate_camera_files
    std::string first_extension = get_extension(opt.input_files[0], false);
    //std::cout << "first_extension = " << first_extension << std::endl;
    if ( boost::iends_with(opt.input_files[1], first_extension ) )
      opt.seperate_camera_files = false;
  }
  //std::cout << "opt.seperate_camera_files = " << opt.seperate_camera_files << std::endl;

  if ( opt.input_files.size() == 0 ||
       (opt.seperate_camera_files && opt.input_files.size() < 2) )
    vw_throw( ArgumentErr() << usage << general_options );

  opt.write_csv = vm.count("write-csv");

  // Need this to be able to load adjusted camera models. That will happen
  // in the stereo session.
  asp::stereo_settings().bundle_adjust_prefix = opt.bundle_adjust_prefix;

  // Look up for session type based on file extensions
  if (opt.stereo_session_string.empty()) {
    std::string type_containing_string = opt.input_files[0];
    if ( opt.seperate_camera_files ) // Look in camera model file instead of image file
      type_containing_string = opt.input_files[1];
    
    if ( boost::iends_with(type_containing_string, ".cahvor") ||
         boost::iends_with(type_containing_string, ".cahv"  ) ||
         boost::iends_with(type_containing_string, ".pin"   ) ||
         boost::iends_with(type_containing_string, ".tsai"  ) ) {
      vw_out() << "\t--> Detected pinhole camera file\n";
      opt.stereo_session_string = "pinhole";
    } else if (boost::iends_with(type_containing_string, ".cub") ) {
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


    // Get the list of image files and camera model files
    size_t num_cameras;
    std::vector<std::string> image_files, camera_files;
    std::vector<std::vector<int> > matched_cameras;
    if (!opt.load_camera_solve)
      num_cameras = split_files_list(opt.input_files, image_files, camera_files, 
                                     opt.seperate_camera_files);
    else
      num_cameras = get_files_from_solver_folder(opt.input_files[0], image_files, 
                                                 camera_files, matched_cameras);

    typedef boost::scoped_ptr<asp::StereoSession> SessionPtr;
    SessionPtr session( asp::StereoSessionFactory::create(opt.stereo_session_string, opt) );

    // Create the KML file.
    KMLFile kml( opt.out_file, "orbitviz" );
    // Style listing
    if ( opt.path_to_outside_model.empty() ) {
      // Placemark Style
      kml.append_style( "plane", "", 1.2,
                        "http://maps.google.com/mapfiles/kml/shapes/airports.png", 
                        opt.hide_labels);
      kml.append_style( "plane_highlight", "", 1.4,
                        "http://maps.google.com/mapfiles/kml/shapes/airports.png");
      kml.append_stylemap( "camera_placemark", "plane",
                           "plane_highlight" );
    }

    // TODO: There should be a datum parsing function!
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
    std::vector<Vector3> camera_positions(num_cameras);
    for (size_t i=0; i < num_cameras; i++) {
      boost::shared_ptr<camera::CameraModel> current_camera;
      current_camera = session->camera_model(image_files [i],
                                             camera_files[i]);

      if ( opt.write_csv ) {
        csv_file << image_files[i] << ", ";

        // Add the ISIS camera serial number if applicable
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
      } // End csv write condition

      // Compute and record the GDC coordinates
      Vector3 lon_lat_alt = datum.cartesian_to_geodetic(current_camera->camera_center(Vector2()));
      camera_positions[i] = lon_lat_alt;

      // Adding Placemarks
      std::string display_name = strip_directory(image_files[i]);
      if (!opt.path_to_outside_model.empty()) {
        kml.append_model( opt.path_to_outside_model,
                          lon_lat_alt.x(), lon_lat_alt.y(),
                          inverse(current_camera->camera_pose(Vector2())),
                          display_name, "",
                          lon_lat_alt[2], opt.model_scale );
      } else {
        kml.append_placemark( lon_lat_alt.x(), lon_lat_alt.y(),
                              display_name, "", "camera_placemark",
                              lon_lat_alt[2], true );
      }
      
    } // End loop through cameras

    // Draw lines between camera positions representing camera
    //  pairs with match files.
    const std::string style_id = "ip_match_style";
    kml.append_line_style(style_id, "FF00FF00", 1.0); // Green line with default size
    std::vector<Vector3> line_ends(2);
    for (size_t i=0; i<matched_cameras.size(); ++i) {
      line_ends[0] = camera_positions[i];
      for (size_t j=0; j<matched_cameras[i].size(); ++j) {
        int index = matched_cameras[i][j];
        line_ends[1] = camera_positions[index];
        kml.append_line(line_ends, "", style_id);
      }
    }

    csv_file.close();
    kml.close_kml();
  } ASP_STANDARD_CATCHES;

  return 0;
}
