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
    if (vm.count("input-files") == 0)
      vw_throw(ArgumentErr() << "Missing input arguments.\n" << usage );
    input_files = vm["input-files"].as< std::vector<std::string> >();

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

int main(int argc, char** argv) {

  try {
    xercesc::XMLPlatformUtils::Initialize();
    stereo_register_sessions();

    bool verbose = false;
    ASPGlobalOptions opt;
    std::string output_prefix;
    std::vector<std::string> images;

    std::vector<std::string> all_files;
    stereo_settings().vwip_files.clear();
    handle_arguments(argc, argv, opt, all_files);

    // Try to load each input file as a standalone image one at a time
    for (size_t i = 0; i < all_files.size(); i++) {
      std::string file = all_files[i];
      bool is_image = false;
      try {
        DiskImageView<float> tmp(file);
        is_image = true;
      }catch(std::exception & e){
        if (file.empty() || file[0] == '-') continue;
        if (get_extension(file) == ".match") {
          // Found a match file
          stereo_settings().match_file = file;
          is_image = false;
        }else if (get_extension(file) == ".vwip") {
          // Found a vwip file
          stereo_settings().vwip_files.push_back(file);
          is_image = false;
        }else if (asp::has_shp_extension(file)) {
          // See if this is a shape file
          is_image = true; // will load it in the same struct as for images
        }else if (has_cam_extension(file)) {
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
    
    // Presumably the tool was invoked with no options. Just print the help message.
    if (images.empty())
      vw_throw(ArgumentErr() << "Could not process the inputs.\n");
    
    if (stereo_settings().create_image_pyramids_only) {
      // Just create the image pyramids and exit
      for (size_t i = 0; i < images.size(); i++) {
        vw::gui::imageData img;
        img.read(images[i], opt);
        
        if (stereo_settings().hillshade) {
          // Create hillshaded images
          std::string hillshaded_file; 
          bool success = vw::gui::write_hillshade(opt,
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
    
    vw::create_out_dir(output_prefix);

    // Create the application. Must be done before trying to read
    // images as that call uses pop-ups.
    asp::StereoApplication app(argc, argv);
    
#if !__APPLE__
    // TODO(oalexan1): Figure out why clang cannot find OpenMP.
    // Set the number of threads
    // TODO(oalexan1): Figure out if this is better than
    // following the defaults
    int processor_count = std::thread::hardware_concurrency();
    omp_set_dynamic(0);
    omp_set_num_threads(processor_count);
#endif
    
    // Start up the Qt GUI
    // TODO(oalexan1): The arguments below are accessible from the main window
    // from stereo_settings() directly, so there is no need to pass them this way.
    vw::gui::MainWindow main_window(opt, images, output_prefix,
                                    stereo_settings().grid_cols,
                                    stereo_settings().window_size,
                                    stereo_settings().single_window,
                                    stereo_settings().use_georef,
                                    stereo_settings().hillshade,
                                    stereo_settings().delete_temporary_files_on_exit,
                                    argc, argv);
    
    main_window.show();
    app.exec();
    
    xercesc::XMLPlatformUtils::Terminate();
  } ASP_STANDARD_CATCHES;
  
  return 0;
}
