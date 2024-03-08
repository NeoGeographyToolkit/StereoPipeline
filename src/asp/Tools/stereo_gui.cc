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

/// \file stereo_gui.cc
///

#include <QApplication>
#include <QWidget>

// Can't do much about warnings in boost except to hide them
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
#include <boost/core/null_deleter.hpp>
#include <boost/accumulators/accumulators.hpp>
#include <boost/accumulators/statistics.hpp>
#pragma GCC diagnostic pop

#include <vw/Stereo/PreFilter.h>
#include <vw/Stereo/CorrelationView.h>
#include <vw/Stereo/CostFunctions.h>
#include <vw/Stereo/DisparityMap.h>
#include <vw/Core/CmdUtils.h>
#include <asp/Tools/stereo.h>
#include <asp/Core/DemDisparity.h>
#include <asp/GUI/MainWindow.h>
#include <asp/GUI/GuiUtilities.h>
#include <xercesc/util/PlatformUtils.hpp>
#include <omp.h>
#include <thread>

using namespace vw;
using namespace vw::stereo;
using namespace asp;
namespace fs = boost::filesystem;

namespace asp{

  // Parse the input command line arguments. We'll use this function
  // only when we failed to parse the arguments under the assumption
  // that the tool was being invoked with the stereo interface. Hence
  // we'll fallback to this when the tool is used as an image viewer.
  void handle_arguments(int argc, char *argv[], ASPGlobalOptions& opt,
			std::vector<std::string> & input_files){

    // Options for which we will print the help message
    po::options_description general_options("");
    general_options.add(GUIDescription());

    // All options that we will parse
    po::options_description all_general_options("");

    // The values for these options are stored in 'opt'
    addAspGlobalOptions(all_general_options, opt);

    // The values for these are stored in stereo_settings()
    all_general_options.add(generate_config_file_options(opt));

    po::options_description positional_options("");
    po::positional_options_description positional_desc;
    positional_options.add_options()
      ("input-files", po::value< std::vector<std::string> >(), "Input files");
    positional_desc.add("input-files", -1);

    std::string usage = "[options] <images> <output_file_prefix>";
    bool allow_unregistered = false;
    std::vector<std::string> unregistered;
    po::variables_map vm = asp::check_command_line(argc, argv, opt, general_options,
                                                   all_general_options, positional_options,
                                                   positional_desc, usage,
                                                   allow_unregistered, unregistered);

    // Add the options to the usage
    std::ostringstream os;
    os << usage << general_options;
    usage = os.str();

    // Store the files
    if (vm.count("input-files") > 0) 
      input_files = vm["input-files"].as<std::vector<std::string>>();
    else
      input_files.clear(); // no input files, will be read from nvm later
    
    // Interpret the the last two coordinates of the crop win boxes as
    // width and height rather than max_x and max_y. 
    BBox2i bl = stereo_settings().left_image_crop_win;
    BBox2i br = stereo_settings().right_image_crop_win;
    stereo_settings().left_image_crop_win
      = BBox2i(bl.min().x(), bl.min().y(), bl.max().x(), bl.max().y());
    stereo_settings().right_image_crop_win
      = BBox2i(br.min().x(), br.min().y(), br.max().x(), br.max().y());

    // Adjust the zoom win dimensions
    auto & zoom_win = asp::stereo_settings().zoom_proj_win;
    if (zoom_win.min().x() > zoom_win.max().x())
      std::swap(zoom_win.min().x(), zoom_win.max().x());
    if (zoom_win.min().y() > zoom_win.max().y())
      std::swap(zoom_win.min().y(), zoom_win.max().y());
  }

  // Inherit from QApplication to be able to over-ride the notify() method
  // that throws exceptions.
  class StereoApplication: public QApplication {
  public:
    StereoApplication(int& argc, char** argv): QApplication(argc, argv) {}
    virtual bool notify(QObject *receiver, QEvent *e) {
      
      try {
        return QApplication::notify(receiver, e);
      } catch (std::exception& ex) {
        vw::gui::popUp(ex.what());
        return false;
      }
      
    }
  };

} // end namespace asp

// Given an input string as:
//
// stereo_gui --style line --color red --colormap-style inferno file1.txt --color green file2.txt
//
// extract each style, color, and colormap. Any of these apply for any following
// files, till modified by another such option. The default style and color
// are "default". Later, these will be either read from the files
// themselves, or otherwise set to "line" and "green". Then modify the
// args to remove these options, as the boost parser cannot parse
// repeated options.
// TODO(oalexan1): If the same file is repeated, the book-keeping will fail.
void preprocessArgs(int &argc, char** argv,
                    std::vector<std::map<std::string, std::string>> & properties) {

  std::string curr_style = "default", curr_color = "default", curr_colormap = "binary-red-blue",
    colorbar = "0";
  int out_it = 1;
  // One set of properties for each argument. That to make sure that a filename
  // can show up twice with different properties
  properties.resize(argc);
  for (int it = 1; it < argc; it++) { // skip program name, so start from 1

    // TODO(oalexan1): Add support for --no-colorize, and make this and --colorize
    // to be able to apply to all subsequent images.
    
    if (std::string(argv[it]) == "--style") {
      if (it == argc - 1)
        continue; // There is nothing else

      it++;
      curr_style = argv[it]; // copy the style value, and move past it
      continue;
    }

    if (std::string(argv[it]) == "--color") {
      if (it == argc - 1)
        continue; // There is nothing else

      it++;
      curr_color = argv[it]; // copy the color value, and move past it
      continue;
    }

    if (std::string(argv[it]) == "--colormap-style") {
      if (it == argc - 1)
        continue; // There is nothing else

      it++;
      curr_colormap = argv[it]; // copy the color value, and move past it
      continue;
    }

    // This is an option with no value
    if (std::string(argv[it]) == "--colorbar") {
      colorbar = "1";
      continue;
    }

    // This is an option with no value
    if (std::string(argv[it]) == "--no-colorbar") {
      colorbar = "0";
      continue;
    }

    // If this argument does not start with a dash, so is not an
    // option, assign to it the properties so far
    if (argv[it][0] != '-') {
      properties[it]["name"] = argv[it];
      properties[it]["style"] = curr_style;  
      properties[it]["color"] = curr_color;
      properties[it]["colormap"] = curr_colormap;
      properties[it]["colorbar"] = colorbar;
    }
    
    // Shift arguments left, which will wipe what we processed above
    argv[out_it] = argv[it]; // this copies pointer addresses
    out_it++; 
  }

  // Update the number of remaining arguments
  argc = out_it;

  return;
}

void readImageNames(std::vector<std::string> const& all_files,
                    std::vector<std::string> & images,
                    std::string & output_prefix) {

  output_prefix = "";
  images.clear();

  // Try to load each input file as a standalone image one at a time
  for (size_t i = 0; i < all_files.size(); i++) {
    std::string file = all_files[i];
    bool is_image = false;
    try {
      DiskImageView<float> tmp(file);
      is_image = true;
    } catch(std::exception & e){
      if (file.empty() || file[0] == '-') continue;
      if (get_extension(file) == ".match") {
        // Found a match file
        stereo_settings().match_file = file;
        is_image = false;
      } else if (get_extension(file) == ".vwip") {
        // Found a vwip file
        stereo_settings().vwip_files.push_back(file);
        is_image = false;
      } else if (get_extension(file) == ".gcp") {
        // Found a gcp file
        stereo_settings().gcp_file = file;
        is_image = false;
      } else if (get_extension(file) == ".nvm") {
        // Found an nvm file
        if (!stereo_settings().nvm.empty()) // sanity check
          vw_out() << "Multiple nvm files specified. Will load only: " << file << "\n";
        stereo_settings().nvm  = file;
        is_image = false;
      } else if (asp::has_shp_extension(file)) {
        // See if this is a shape file
        is_image = true; // will load it in the same struct as for images
      } else if (vw::gui::hasCsv(file)) {
        is_image = true; // will load it in the same struct as for images
      } else if (has_cam_extension(file)) {
        // We will get here for all cameras except .cub, which
        // is both an image and a camera and was picked up by
        // now. Don't print an error in this case as this is
        // expected to not be an image.
        is_image = false;
      } else {
        vw_out() << "Not a valid image: " << file << ". ";
        if (!fs::exists(file)) {
          vw_out() << "Using this as the output prefix.\n";
          output_prefix = file;
        } else {
          vw_out() << "\n";
        }
      }
    }
    if (is_image)
      images.push_back(file);
  }

  return;
}

int main(int argc, char** argv) {

  try {
    xercesc::XMLPlatformUtils::Initialize();
    stereo_register_sessions();

    // Extract style and color for scattered points, curves, and polygons
    std::vector<std::map<std::string, std::string>> properties;
    preprocessArgs(argc, argv, properties);

    bool verbose = false;
    ASPGlobalOptions opt;
    std::string output_prefix;
    std::vector<std::string> images;

    std::vector<std::string> all_files;
    stereo_settings().vwip_files.clear();
    handle_arguments(argc, argv, opt, all_files);

    readImageNames(all_files, images, output_prefix);

    if (stereo_settings().create_image_pyramids_only) {
      // Just create the image pyramids and exit. 
      for (size_t i = 0; i < images.size(); i++) {
        vw::gui::imageData img;
        img.read(images[i], opt);
        if (stereo_settings().hillshade) {
          // Create hillshaded images. 
          std::string hillshaded_file;
          bool have_gui = false; // so we don't use a pop up before the gui got started
          bool success = vw::gui::write_hillshade(opt,
                                                  have_gui,
                                                  stereo_settings().hillshade_azimuth,
                                                  stereo_settings().hillshade_elevation,
                                                  images[i],
                                                  // Output
                                                  hillshaded_file);
          if (success) // build the pyramids
            img.read(hillshaded_file, opt, vw::gui::HILLSHADED_VIEW);
        }
      }
      return 0;
    }

    // Create the application. Must be done before trying to read
    // images as that call uses pop-ups. Must not happen before
    // building image pyramids as that does not need a gui.
    asp::StereoApplication app(argc, argv);

    // Set the font size
    QFont f = app.font();
    f.setPointSize(asp::stereo_settings().font_size);
    app.setFont(f);
    
    vw::create_out_dir(output_prefix);

    // Use OpenMP to speed things up.
    // TODO(oalexan1): This should be enabled in more places.
    int processor_count = std::thread::hardware_concurrency();
    omp_set_dynamic(0);
    omp_set_num_threads(processor_count);
    
    // Start up the Qt GUI
    // TODO(oalexan1): The arguments below are accessible from the main window
    // from stereo_settings() directly, so there is no need to pass them this way.
    vw::gui::MainWindow main_window(opt, images, output_prefix,
                                    stereo_settings().grid_cols,
                                    stereo_settings().window_size,
                                    stereo_settings().single_window,
                                    stereo_settings().use_georef,
                                    properties,
                                    argc, argv);
    
    main_window.show();
    app.exec();
    
    xercesc::XMLPlatformUtils::Terminate();
  } ASP_STANDARD_CATCHES;
  
  return 0;
}
