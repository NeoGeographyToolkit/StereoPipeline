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

#include <vw/InterestPoint.h>
#include <boost/accumulators/accumulators.hpp>
#include <boost/accumulators/statistics.hpp>
#include <vw/Stereo/PreFilter.h>
#include <vw/Stereo/CorrelationView.h>
#include <vw/Stereo/CostFunctions.h>
#include <vw/Stereo/DisparityMap.h>
#include <asp/Tools/stereo.h>
#include <asp/Core/DemDisparity.h>
#include <asp/Core/LocalHomography.h>
#include <asp/GUI/MainWindow.h>
#include <asp/GUI/GuiUtilities.h>
#include <xercesc/util/PlatformUtils.hpp>

using namespace vw;
using namespace vw::stereo;
using namespace asp;
using namespace std;

// Allows FileIO to correctly read/write unusual pixel types
namespace vw {
  template<> struct PixelFormatID<Vector3>   { static const PixelFormatEnum value = VW_PIXEL_GENERIC_3_CHANNEL; };
}

namespace asp{

  // Parse the input command line arguments. We'll use this function
  // only when we failed to parse the arguments under the assumption
  // that the tool was being invoked with the stereo interface. Hence
  // we'll fallback to this when the tool is used as an image viewer.
  void handle_arguments( int argc, char *argv[], ASPGlobalOptions& opt,
                         std::vector<string> & input_files){

    po::options_description general_options("");
    general_options.add(GUIDescription());

    po::options_description positional_options("");
    po::positional_options_description positional_desc;
    positional_options.add_options()
      ("input-files", po::value< std::vector<std::string> >(), "Input files");
    positional_desc.add("input-files", -1);

    std::string usage = "[options] <images> [<cameras>] <output_file_prefix> [DEM]";
    bool allow_unregistered = false;
    std::vector<std::string> unregistered;
    po::variables_map vm = asp::check_command_line(argc, argv, opt, general_options,
                                                   general_options, positional_options,
                                                   positional_desc, usage,
                                                   allow_unregistered, unregistered);

    // Add the options to the usage
    std::ostringstream os;
    os << usage << general_options;
    usage = os.str();

    // Store the files
    if (vm.count("input-files") == 0)
      vw_throw(ArgumentErr() << "Missing input arguments.\n" << usage );
    input_files = vm["input-files"].as< std::vector<std::string> >();
  }

}

// Inherit from QApplication to be able to over-ride the notify() method
// that throws exceptions.
class StereoApplication: public QApplication {
public:
  StereoApplication(int& argc, char** argv): QApplication(argc, argv) {}
  virtual bool notify(QObject *receiver, QEvent *e) {

    try {
      return QApplication::notify(receiver, e);
    } catch ( std::exception& ex ) {
      vw::gui::popUp(ex.what());
      return false;
    }
 
  }
};

int main(int argc, char** argv) {

  try {
    xercesc::XMLPlatformUtils::Initialize();
    stereo_register_sessions();

    bool verbose = false;
    vector<ASPGlobalOptions> opt_vec;
    string output_prefix;
    std::vector<std::string> images;
    try {

      // For some reason, there is a crash with ISIS sometimes if
      // going through the full flow of parsing arguments. ISIS and Qt
      // mis-communicate.  Stop before loading any cameras. This
      // should be safe enough in GUI mode.
      bool exit_early = true;

      // See if we passed all the correct options for stereo
      asp::parse_multiview(argc, argv, asp::GUIDescription(),
                           verbose, output_prefix, opt_vec, exit_early);
      // Extract the images from the options, not the cameras though.
      for (size_t i = 0; i < opt_vec.size(); i++) {
        if (i == 0) images.push_back(opt_vec[i].in_file1);
        images.push_back(opt_vec[i].in_file2);
      }

    }catch (std::exception& e){

      // The tool was not invoked correctly using the stereo interface. Perhaps the
      // user only wants to see images?
      try {

        std::vector<std::string> all_images;
        opt_vec.resize(1);
        handle_arguments(argc, argv, opt_vec[0], all_images);

        // Try to load each input file as a standalone image one at a time
        for (size_t i = 0; i < all_images.size(); i++) {
          std::string image = all_images[i];
          bool is_image = false;
          try {
	    DiskImageView<float> tmp(image);
	    is_image = true;
    	  }catch(std::exception & e){
            if (!image.empty() && image[0] != '-') {
              if (asp::has_shp_extension(image)) {
		vw_out() << "Reading shapefile: " << image << std::endl;
		is_image = true;
	      }else{
		vw_out() << "Not a valid image: " << image << ".\n";
		if (!fs::exists(image)) {
		  vw_out() << "Using this as the output prefix.\n";
		  output_prefix = image;
		}
	      }
	    }
          }
          if (is_image)
            images.push_back(image);
        }
      }catch (std::exception& e2){
        vw_throw(ArgumentErr() << "Use either the stereo or the image viewer interface.\n"
                 << "stereo error: " << e.what() << "\n"
                 << "viewer error: " << e2.what() << "\n");
      }

      // Presumably the tool was invoked with no options. Just print the help message.
      if (images.empty())
        vw_throw(ArgumentErr() << e.what() << "\n");
    }

    if (stereo_settings().create_image_pyramids_only) {
      // Just create the image pyramids and exit.
      for (size_t i = 0; i < images.size(); i++) {
        vw::gui::imageData img;
        img.read(images[i], opt_vec[0], stereo_settings().use_georef);
      }
      return 0;
    }

    vw::create_out_dir(output_prefix);

    // Create the application. Must be done before trying to read
    // images as that call uses pop-ups.
    StereoApplication app(argc, argv);

    // Start up the Qt GUI
    vw::gui::MainWindow main_window(opt_vec[0],
                                    images, output_prefix,
                                    stereo_settings().grid_cols,
                                    stereo_settings().window_size,
                                    stereo_settings().single_window,
                                    stereo_settings().use_georef,
                                    stereo_settings().hillshade,
                                    stereo_settings().view_matches,
                                    stereo_settings().delete_temporary_files_on_exit,
                                    argc, argv);

    main_window.show();
    app.exec();
    
    xercesc::XMLPlatformUtils::Terminate();
  } ASP_STANDARD_CATCHES;

  return 0;
}
