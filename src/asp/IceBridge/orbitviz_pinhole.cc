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

// This is as scaled-down version of orbitviz that only operates on pinhole files

/************************************************************************
 *     File: orbitviz.cc
 ************************************************************************/
#include <vw/Core.h>
#include <vw/Math.h>
#include <vw/FileIO/KML.h>
#include <vw/FileIO/FileUtils.h>
#include <vw/Camera.h>
#include <vw/Cartography.h>


#include <iomanip>

#include <boost/algorithm/string.hpp>
#include <boost/program_options.hpp>
#include <boost/filesystem/path.hpp>
#include <boost/filesystem/operations.hpp>
#include <boost/filesystem/convenience.hpp>
#include <boost/filesystem/fstream.hpp>

#include <asp/Core/Common.h>


using namespace vw;
using namespace vw::camera;
using namespace vw::cartography;
namespace po = boost::program_options;
namespace fs = boost::filesystem;

struct Options : public vw::cartography::GdalWriteOptions {
  Options() {}
  // Input
  std::vector<std::string> input_files;
  std::string path_to_outside_model, input_list;

  // Settings
  bool write_csv, hide_labels;
  double model_scale; ///< Size scaling applied to 3D models

  // Output
  std::string out_file;
};

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

void handle_arguments( int argc, char *argv[], Options& opt ) {
  po::options_description general_options("");
  general_options.add_options()
    ("output,o",                po::value(&opt.out_file)->default_value("orbit.kml"),
          "The output kml file that will be written")
    ("input-list", po::value(&opt.input_list)->default_value(""),
          "File containing list of input files")
    ("use-path-to-dae-model,u", po::value(&opt.path_to_outside_model),
          "Instead of using an icon to mark a camera, use a 3D model with extension .dae")
    ("hide-labels",             po::bool_switch(&opt.hide_labels)->default_value(false)->implicit_value(true),
          "Hide image names unless the camera is highlighted.")
    // The KML class applies a model scale of 3000 * this value.
    ("model-scale",             po::value(&opt.model_scale)->default_value(1.0/30.0),
          "Scale factor applied to 3D model size.")
    ("write-csv", po::bool_switch(&opt.write_csv)->default_value(false),
     "Write a csv file with the orbital data.");
  general_options.add( vw::cartography::GdalWriteOptionsDescription(opt) );

  po::options_description positional("");
  positional.add_options()
    ("input-files", po::value(&opt.input_files) );

  po::positional_options_description positional_desc;
  positional_desc.add("input-files", -1);

  std::string usage("[options] <input cameras>\n");

  bool allow_unregistered = false;
  std::vector<std::string> unregistered;
  po::variables_map vm =
    asp::check_command_line( argc, argv, opt, general_options, general_options,
                             positional, positional_desc, usage,
                             allow_unregistered, unregistered );
 
 if (opt.input_list != "") {
   std::ifstream handle(opt.input_list.c_str());
   std::string line;
   size_t count = 0;
   while (getline(handle, line)){
     opt.input_files.push_back(line);
     ++count;
   }
   handle.close();
   vw_out() << "Read in " << count << " camera files from " << opt.input_list << std::endl;
 }
 
 if (opt.input_files.empty())
  vw_throw( ArgumentErr() << "No input files provided!\n" );
  
}

int main(int argc, char* argv[]) {

  Options opt;
  //try {
  handle_arguments( argc, argv, opt );

  size_t num_cameras = opt.input_files.size();

  // Prepare output directory
  vw::create_out_dir(opt.out_file);
  
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

  // Load up the datum
  cartography::Datum datum("WGS84");

  std::string csv_file = fs::path(opt.out_file).replace_extension("csv").string();
  std::ofstream csv_handle;
  if ( opt.write_csv ) {
    csv_handle.open(csv_file.c_str());
  
    if ( !csv_handle.is_open() )
      vw_throw( IOErr() << "Unable to open output file.\n" );
  }
  
  Vector2 camera_pixel(0, 0);

  // Building Camera Models and then writing to KML
  std::vector<Vector3> camera_positions(num_cameras);
  for (size_t i=0; i < num_cameras; i++) {
    // Load this input file
    PinholeModel current_camera(opt.input_files[i]);

    if ( opt.write_csv ) {
      csv_handle << opt.input_files[i] << ", ";

      Vector3 xyz = current_camera.camera_center(camera_pixel);
      csv_handle << std::setprecision(12);
      csv_handle << xyz[0] << ", "
                 << xyz[1] << ", " << xyz[2] << "\n";
    } // End csv write condition
    
    // Compute and record the GDC coordinates
    Vector3 lon_lat_alt = datum.cartesian_to_geodetic(
                              current_camera.camera_center(camera_pixel));
    camera_positions[i] = lon_lat_alt;

    // Adding Placemarks
    std::string display_name = strip_directory(opt.input_files[i]);
    if (!opt.path_to_outside_model.empty()) {
      kml.append_model( opt.path_to_outside_model,
                        lon_lat_alt.x(), lon_lat_alt.y(),
                        inverse(current_camera.camera_pose(camera_pixel)),
                        display_name, "",
                        lon_lat_alt[2], opt.model_scale );
    } else {
      kml.append_placemark( lon_lat_alt.x(), lon_lat_alt.y(),
                            display_name, "", "camera_placemark",
                            lon_lat_alt[2], true );
    }
    
  } // End loop through cameras

  // Put the Writing: messages here, so that they show up after all other info.
  vw_out() << "Writing: " << opt.out_file << std::endl; 
  kml.close_kml();

  if (opt.write_csv){
    vw_out() << "Writing: " << csv_file << std::endl;
    csv_handle.close();
  }
    
  //} ASP_STANDARD_CATCHES;

  return 0;
}
