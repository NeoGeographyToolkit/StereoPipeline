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
  void handle_arguments( int argc, char *argv[], Options& opt,
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

int main(int argc, char* argv[]) {

  try {

    stereo_register_sessions();

    bool verbose = false;
    vector<Options> opt_vec;
    string output_prefix;
    std::vector<std::string> images;
    try {
      // See if we passed all the correct options for stereo
      asp::parse_multiview(argc, argv, asp::GUIDescription(),
                           verbose, output_prefix, opt_vec);
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

        for (size_t i = 0; i < all_images.size(); i++) {
          std::string image = all_images[i];
          bool is_image = true;
          try { DiskImageView<float> tmp(image); }
          catch(...){
            if (!image.empty() && image[0] != '-') {
              vw_out() << "Not a valid image: " << image << ".\n";
	      if (!fs::exists(image)) {
		vw_out() << "Using this as the output prefix.\n";
		output_prefix = image;
	      }
            }
            is_image = false;
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

    vw::create_out_dir(output_prefix);

    // Start up the Qt GUI
    QApplication app(argc, argv);
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

  } ASP_STANDARD_CATCHES;

  return 0;
}
