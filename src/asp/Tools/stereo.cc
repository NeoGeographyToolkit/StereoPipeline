// __BEGIN_LICENSE__
//  Copyright (c) 2009-2026, United States Government as represented by the
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

/// \file stereo.cc

#include <asp/Sessions/StereoSessionFactory.h>
#include <asp/Camera/CameraErrorPropagation.h>
#include <asp/Core/FileUtils.h>
#include <asp/Core/StereoSettings.h>
#include <asp/Core/Bathymetry.h>
#include <asp/Core/AspStringUtils.h>
#include <asp/Core/AspProgramOptions.h>
#include <asp/Core/StereoSettingsParse.h>
#include <asp/Core/IpMatchingAlgs.h>
#include <asp/Tools/stereo.h>
#include <asp/asp_config.h>

// Support for ISIS image files
#if defined(ASP_HAVE_PKG_ISIS) && ASP_HAVE_PKG_ISIS == 1
#include <asp/IsisIO/DiskImageResourceIsis.h>
#endif

#include <vw/Cartography/PointImageManipulation.h>
#include <vw/Stereo/StereoView.h>
#include <vw/Stereo/PreFilter.h>
#include <vw/Stereo/CorrelationView.h>
#include <vw/Stereo/CostFunctions.h>
#include <vw/Stereo/DisparityMap.h>
#include <vw/FileIO/MatrixIO.h>
#include <vw/FileIO/DiskImageUtils.h>
#include <vw/FileIO/FileTypes.h>
#include <vw/FileIO/FileUtils.h>
#include <vw/InterestPoint/MatcherIO.h>

// Can't do much about warnings in boost except to hide them
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
#include <boost/accumulators/accumulators.hpp>
#include <boost/accumulators/statistics.hpp>
#pragma GCC diagnostic pop

using namespace vw;
using namespace vw::cartography;

namespace asp {

// Transform the crop window to be in reference to L.tif. When --left-image-crop-win
// is set, different logic is used, which may need to be integrated here.
BBox2i transformed_crop_win(ASPGlobalOptions const& opt) {

  bool crop_left  = (stereo_settings().left_image_crop_win  != BBox2i(0, 0, 0, 0));
  if (crop_left)
    vw::vw_throw(ArgumentErr() << "Function transformed_crop_win() "
               << "should not be called with --left-image-crop-win.\n");

  BBox2i b;
  boost::shared_ptr<vw::DiskImageResource> rsrc =
          vw::DiskImageResourcePtr(opt.in_file1);
  DiskImageView<PixelGray<float>> left_image(rsrc);
  BBox2i full_box = bounding_box(left_image);
  
  if (fs::exists(opt.out_prefix+"-L.tif")) {
    DiskImageView<PixelGray<float>> L_img(opt.out_prefix+"-L.tif");
    b = bounding_box(L_img);
  } else {
    b = full_box; // To not have an empty box
  }

  return b;
}

// Set up options for stereo. This will be used in a couple of places.
void configStereoOpts(ASPGlobalOptions& opt,
                      po::options_description const& additional_options,
                      po::options_description& general_options,
                      po::options_description& all_general_options,
                      po::options_description& positional_options,
                      po::positional_options_description& positional_desc) {

  // Add options whose values are stored in ASPGlobalOptions rather than in stereo_settings()
  po::options_description general_options_sub("");
  addAspGlobalOptions(general_options_sub, opt);

  // Populate general_options (specific to the current tool)
  general_options.add(general_options_sub);
  general_options.add(additional_options);
  general_options.add(vw::GdalWriteOptionsDescription(opt));

  // Populate all_general_options (parsing fallback)
  all_general_options.add(general_options_sub);
  all_general_options.add(generate_config_file_options(opt));

  // Populate positional options
  positional_options.add_options()
    ("input-files", po::value<std::vector<std::string>>(), "Input files");
  
  positional_desc.add("input-files", -1);
}

// Parse the command line arguments. Extract in a vector all the options and
// their values, so everything apart from standalone files not associated with
// options. This approach does not get confused by a file showing up both as a
// value to an option and later as a standalone file. This is needed for later
// reconstructing the command line with subsets of the input files, such when as
// going from a multiview command to multiple pairwise commands.
void parseStereoOptsVals(int argc, char *argv[],
                         po::options_description const& additional_options,
                         std::vector<std::string> & opts_and_vals) {
  
  // Wipe the output
  opts_and_vals.clear();

  // We set up the option descriptions locally just to aid the parser
  // in distinguishing flags from positional files.
  po::options_description general_options("");
  po::options_description all_general_options("");
  po::options_description positional_options("");
  po::positional_options_description positional_desc;
  ASPGlobalOptions local_opt; 

  // Configure the descriptions using your helper function
  configStereoOpts(local_opt, additional_options, general_options, all_general_options, 
                   positional_options, positional_desc);

  // Parse the command line
  try {
    po::options_description all_options;
    all_options.add(all_general_options).add(positional_options);

    // Pass positional_desc to the parser so it knows which args are positional
    po::parsed_options parsed = 
      po::command_line_parser(argc, argv)
        .options(all_options)
        .positional(positional_desc)
        .style(po::command_line_style::unix_style)
        .run();
    
    // Populate opts_and_vals
    for (auto const& opt: parsed.options) {
      // Position_key is -1 for named options, >= 0 for positional args (files)
      if (opt.position_key == -1) { 
        opts_and_vals.insert(opts_and_vals.end(),
                             opt.original_tokens.begin(),
                             opt.original_tokens.end());
      }
    }

  } catch (po::error const& e) {
    // The main parser will catch errors, so this one can be silent.
  }
}

// Handle the arguments for the multiview case. It creates multiple pairwise
// invocations of stereo.
void handle_multiview(int argc, char* argv[],
                      int num_pairs,
                      std::vector<std::string> const& files,
                      std::vector<std::string> const& images,
                      std::vector<std::string> const& cameras,
                      std::string const& output_prefix,
                      std::string const& input_dem,
                      po::options_description const& additional_options,
                      bool verbose, bool exit_early,
                      // Outputs
                      std::string & usage,
                      std::vector<ASPGlobalOptions> & opt_vec) {

  //  Parse all the options and their values, stored in a vector, to be 
  // used later for pairwise stereo. It skips the standalone input files.
  std::vector<std::string> opts_vals; 
  parseStereoOptsVals(argc, argv, additional_options, opts_vals);
  
  // Must signal to the children runs that they are part of a multiview run
  std::string opt_str = "--part-of-multiview-run";
  auto it = find(opts_vals.begin(), opts_vals.end(), opt_str);
  if (it == opts_vals.end())
    opts_vals.push_back(opt_str);

  // Multiview is very picky about alignment method
  if (stereo_settings().alignment_method != "none" &&
      stereo_settings().alignment_method != "homography") {

    std::string new_alignment;
    if (input_dem == "")
      new_alignment = "homography";
    else
      new_alignment = "none";

    vw_out(WarningMessage)
      << "For multi-view stereo, only alignment method of 'none' or 'homography' "
      << "is supported. Changing alignment method from '"
      << stereo_settings().alignment_method << "' to '" << new_alignment << "'.\n";
    stereo_settings().alignment_method = new_alignment;

    // Set this for future pairwise runs as well
    std::string align_opt = "--alignment-method";
    auto it = std::find(opts_vals.begin(), opts_vals.end(), align_opt);
    if (it != opts_vals.end() && it + 1 != opts_vals.end()) {
      // Modify existing alignment
      *(it+1) = new_alignment;
    } else {
      // Set new alignment
      opts_vals.push_back(align_opt);
      opts_vals.push_back(new_alignment);
    }
  }

  // Generate the stereo command for each of the pairs made up of the first
  // image and each subsequent image, with corresponding cameras.
  opt_vec.resize(num_pairs);
  std::string prog_name = extract_prog_name(argv[0]);
  for (int p = 1; p <= num_pairs; p++) {

    std::vector<std::string> cmd;
    cmd.push_back(prog_name);

    // The command line options go first
    for (int t = 0; t < (int)opts_vals.size(); t++)
      cmd.push_back(opts_vals[t]);

    cmd.push_back(images[0]); // left image
    cmd.push_back(images[p]); // right image

    if (!cameras.empty()) {
      // Don't append the camera names if they are the same as the image names
      if ((images[0] != cameras[0]) && (images[p] != cameras[p])) {
        cmd.push_back(cameras[0]); // left camera
        cmd.push_back(cameras[p]); // right camera
      }
    }

    // Need to have a separate output prefix for each pair
    std::string local_prefix = output_prefix;
    std::ostringstream os;
    os << local_prefix << "-pair" << p << "/" << p;
    local_prefix = os.str();
    cmd.push_back(local_prefix);
    if (!input_dem.empty())
      cmd.push_back(input_dem);

    // Create a local argc and argv for the given stereo pair and parse them.
    int largc = cmd.size();
    std::vector<char*> largv;
    for (int t = 0; t < largc; t++)
      largv.push_back((char*)cmd[t].c_str());
    bool is_multiview = false; // single image and camera pair
    std::vector<std::string> local_files;
    bool override_out_prefix = false;
    handle_arguments(largc, &largv[0], opt_vec[p-1], additional_options,
                     is_multiview, override_out_prefix, local_files, usage, exit_early);

    if (verbose) {
      // Needed for stereo_parse
      int big = 10000; // helps keep things in order in the python script
      vw_out() << "multiview_command_" << big + p << ",";
      for (int t = 0; t < largc-1; t++)
        vw_out() << cmd[t] << ",";
      if (largc > 0)
        vw_out() << cmd[largc-1];
      vw_out() << "\n";
    }

  } // end loop through pairs

  // Sanity checks specific to multiview

  if (stereo_settings().propagate_errors)
    vw::vw_throw(vw::ArgumentErr() << "Error propagation is not "
                  << "implemented for more than two images.\n");

  if (prog_name != "stereo_parse" &&
      prog_name != "stereo_tri" && prog_name != "stereo_gui")
    vw_throw(ArgumentErr() << "The executable " << prog_name
              << " is not meant to be used directly with more than two images. Use "
              << "instead the stereo/parallel_stereo scripts with desired entry points.\n");

  // This must happen after StereoSession is initialized
  if (opt_vec[0].session->do_bathymetry())
    vw_throw(ArgumentErr() << "Bathymetry correction does not work with "
              << "multiview stereo.\n");

  return;
}

/// Parse the list of files specified as positional arguments on the command line
// The format is:  <N image paths> [N camera model paths] <output prefix> [input DEM path]
bool parse_multiview_cmd_files(bool override_out_prefix, 
                               std::vector<std::string> const &filesIn,
                               std::vector<std::string>       &image_paths,
                               std::vector<std::string>       &camera_paths,
                               std::string                    &prefix,
                               std::string                    &dem_path) {

  // Init outputs
  image_paths.clear();
  camera_paths.clear();
  prefix   = "";
  dem_path = "";

  // Find the input DEM, if any
  std::vector<std::string> files = filesIn; // Make a local copy to work with
  std::string input_dem;
  bool has_georef = false;
  try{ // Just try to load the last file path as a dem
    cartography::GeoReference georef;
    has_georef = read_georeference(georef, files.back());
  } catch(...) {}

  if (has_georef) { // I guess it worked
    dem_path = files.back();
    files.pop_back();
  } else { // We tried to load the prefix, there is no dem.
    dem_path = "";
  }
  if (files.size() < 3) {
    vw_throw(ArgumentErr() << "Expecting at least three inputs to stereo.\n");
    return false;
  }

  // Find the output prefix
  prefix = files.back(); // the dem, if present, was already popped off the back.
  files.pop_back();

  // parallel_stereo must be able to run with a tile output prefix. That program
  // does not understand all the stereo options, so, it delegates to the
  // underlying stereo executables to do this switch. Invoke this only
  // if is_multiview is true, to do the replacement of the prefix just once.
  // Do not do it later again, when the processing is per each stereo pair.
  if (asp::stereo_settings().output_prefix_override != "" && override_out_prefix)
    prefix = asp::stereo_settings().output_prefix_override;
  
  // An output prefix cannot be an image or a camera
  if (vw::has_image_extension(prefix) || vw::has_cam_extension(prefix) || prefix == "")
    vw_throw(ArgumentErr() << "Invalid output prefix: " << prefix << ".\n");

  // The output prefix must not exist as a file
  if (fs::exists(prefix))
      vw_out(WarningMessage)
        << "It appears that the output prefix exists as a file: "
        << prefix << ". Perhaps this was not intended.\n";

  // Now there are N images and possibly N camera paths
  bool ensure_equal_sizes = false;
  asp::separate_images_from_cameras(files, image_paths, camera_paths, ensure_equal_sizes);

  return true;
}

// If --trans-crop-win is in the input arguments, this is means that we are running
// stereo for a tile
bool is_tile_run(int argc, char* argv[]) {
  for (int s = 1; s < argc; s++) {
    if (std::string(argv[s]) == "--trans-crop-win")
      return true;
  }

  return false;
}

// Save some info that will be useful for peeking at a run
void save_run_info(ASPGlobalOptions const& opt,
                   std::vector<std::string> const& images,
                   std::vector<std::string> const& cameras,
                   std::string const& input_dem) {

  std::string info_file = opt.out_prefix + "-info.txt";
  std::ofstream ostr(info_file.c_str());
  if (!ostr.good())
    vw_throw(ArgumentErr() << "Failed to open: " << info_file << "\n");

  // Print the images
  ostr << "images: ";
  for (int i = 0; i < (int)images.size(); i++)
    ostr << images[i] << " ";
  ostr << "\n";

  // Print the cameras
  ostr << "cameras: ";
  for (int i = 0; i < (int)cameras.size(); i++)
    ostr << cameras[i] << " ";
  ostr << "\n";

  // Print the DEM
  ostr << "input_dem: " << input_dem << "\n";

  // Print the output prefix
  ostr << "output_prefix: " << opt.out_prefix << "\n";

  // Print the alignment method
  ostr << "alignment_method: " << stereo_settings().alignment_method << "\n";

  // Print the stereo session
  ostr << "stereo_session: " << opt.stereo_session << "\n";

  // Print left-image-crop-win
  auto l = stereo_settings().left_image_crop_win;
  ostr << "left_image_crop_win: " << l.min().x() << " " << l.min().y() << " "
      << l.width() << " " << l.height() << "\n";

  // Print right-image-crop-win
  auto r = stereo_settings().right_image_crop_win;
  ostr << "right_image_crop_win: " << r.min().x() << " " << r.min().y() << " "
      << r.width() << " " << r.height() << "\n";
}

// If a stereo program is invoked as:
// prog <images> <cameras> <output-prefix> [<input_dem>] <other options>
// with the number of images n >= 2, create n-1 individual
// ASPGlobalOptions entries, corresponding to n-1 stereo pairs between the
// first image and each of the subsequent images.
// TODO(oalexan1): The logic here is horrendous. The handle_arguments() function
// better be called just once, rather than four times, as below.
void parse_multiview(int argc, char* argv[],
                      boost::program_options::options_description const&
                      additional_options,
                      bool verbose,
                      std::string & output_prefix,
                      std::vector<ASPGlobalOptions> & opt_vec,
                      bool exit_early) {

  // First reset the outputs
  output_prefix.clear();
  opt_vec.clear();

  // Extract the images/cameras/output prefix, and perhaps the input DEM
  std::vector<std::string> files;
  bool is_multiview = true;
  ASPGlobalOptions opt;
  std::string usage;
  bool override_out_prefix = true;
  handle_arguments(argc, argv, opt, additional_options,
                   is_multiview, override_out_prefix, files, usage, exit_early);

  // Need this for the GUI, ensure that opt_vec is never empty, even on failures
  opt_vec.push_back(opt);

  if (files.size() < 3)
    vw_throw(ArgumentErr() << "Missing the input files and/or output prefix.\n");

  // Add note on the alignment method. If done in handle_arguments, it will be
  // printed twice.
  if (stereo_settings().correlator_mode)
    vw_out() << "Running in correlator mode. The alignment method is: "
              << stereo_settings().alignment_method << ".\n";

  // Extract all the positional elements
  std::vector<std::string> images, cameras;
  std::string input_dem;
  if (!parse_multiview_cmd_files(override_out_prefix, files, images, cameras, 
                                 output_prefix, input_dem))
    vw_throw(ArgumentErr() << "Missing the input files and/or output prefix.\n");

  int num_pairs = (int)images.size() - 1;
  if (num_pairs <= 0)
    vw_throw(ArgumentErr() << "Insufficient number of images provided.\n");

  // Needed for stereo_parse
  if (verbose)
    vw_out() << "num_stereo_pairs," << num_pairs << "\n";

  if (num_pairs == 1) {
    bool is_multiview = false, override_out_prefix = true;
    handle_arguments(argc, argv, opt_vec[0], additional_options,
                    is_multiview, override_out_prefix, files, usage, exit_early);
  } else {
    handle_multiview(argc, argv, num_pairs, files, images, cameras, output_prefix, input_dem,
                     additional_options, verbose, exit_early, usage,
                     opt_vec); // output
  }

  // For each stereo command not in a tile, print the run info
  if (!is_tile_run(argc, argv))
    save_run_info(opt_vec[0], images, cameras, input_dem);

  return;
}

// Parse data needed for error propagation
void setup_error_propagation(ASPGlobalOptions const& opt) {

  // A bugfix for the propagated errors not being saved with enough digits
  if (stereo_settings().point_cloud_rounding_error > 0) {
    vw_out(WarningMessage) << "Option --point-cloud-rounding-error is set to " <<
      stereo_settings().point_cloud_rounding_error << " meters. If too coarse, "
      "it may create artifacts in the propagated horizontal and vertical errors.\n";
  } else {
      stereo_settings().point_cloud_rounding_error = 1.0e-8;
      vw_out() << "Round triangulated points to "
                << stereo_settings().point_cloud_rounding_error << " meters. "
                << "(Option: --point-cloud-rounding-error.) "
                << "This is much finer rounding than usual, motivated by the "
                << "fact that the propagated errors vary slowly and will be "
                << "saved with step artifacts otherwise.\n";
  }

  vw::Vector2 & v = asp::stereo_settings().horizontal_stddev; // alias, will modify
  bool message_printed = false; // will print the message only once
  if (v[0] == 0 && v[1] == 0) {
    // This will not reload the cameras
    boost::shared_ptr<camera::CameraModel> camera_model1, camera_model2;
    opt.session->camera_models(camera_model1, camera_model2);
    v[0] = asp::horizontalStDevFromCamera(camera_model1, message_printed);
    v[1] = asp::horizontalStDevFromCamera(camera_model2, message_printed);
  }

  asp::horizontalStdDevCheck(v, opt.session->name());
}

// Parse input command line arguments
void handle_arguments(int argc, char *argv[], ASPGlobalOptions& opt,
                      boost::program_options::options_description const&
                      additional_options,
                      bool is_multiview, bool override_out_prefix, 
                      std::vector<std::string> & input_files,
                      std::string & usage, bool exit_early) {

  // Configure the stereo options
  po::options_description general_options("");
  po::options_description all_general_options("");
  po::options_description positional_options("");
  po::positional_options_description positional_desc;
  configStereoOpts(opt, additional_options, general_options, all_general_options, 
                   positional_options, positional_desc);
  
  usage =
   "[options] <images> [<cameras>] <output_file_prefix> [DEM]\n"
   "  Extensions are automatically added to the output files.\n"
   "  Camera model arguments may be optional for some stereo session types (e.g., isis).\n"
   "  Stereo parameters should be set in the stereo.default file.";
  bool allow_unregistered = false;
  std::vector<std::string> unregistered;
  po::variables_map vm = asp::check_command_line(argc, argv, opt, general_options,
                                                 all_general_options, positional_options,
                                                 positional_desc, usage,
                                                 allow_unregistered, unregistered);

  // Read the config file
  try {
    // The user can specify the positional input from the
    // stereo.default if they want to.
    po::options_description cfg_options;
    cfg_options.add(positional_options);
    cfg_options.add(generate_config_file_options(opt));

    // Append the options from the config file to vm. The Boost documentation
    // says that an option already stored will not be changed. So, the settings
    // in the config files will not overwrite the ones already set on the
    // command line.
    bool print_warnings = is_multiview; // print warnings just first time
    po::store(parse_asp_config_file(print_warnings,
                                    opt.stereo_default_filename,
                                    cfg_options), vm);
    po::notify(vm);
  } catch (po::error const& e) {
    vw::vw_throw(vw::ArgumentErr()
                  << "Error parsing configuration file:\n" << e.what() << "\n");
  }

  // This must happen early
  boost::to_lower(opt.stereo_session);
  asp::stereo_settings().validate();

  if (stereo_settings().correlator_mode) {
    // Images are assumed aligned, unless alignment is explicitly requested.
    if (vm["alignment-method"].defaulted())
      stereo_settings().alignment_method = "none";
    opt.stereo_session = "rpc";  // since inputs are images this seems simpler
    if (stereo_settings().propagate_errors)
      vw::vw_throw(vw::ArgumentErr() << "Cannot propagate errors in correlator mode.\n");
  }

  // Make sure that algorithm 0 is same as asp_bm, etc.
  boost::to_lower(stereo_settings().stereo_algorithm);
  if (stereo_settings().stereo_algorithm == "0")
    stereo_settings().stereo_algorithm = "asp_bm";
  else if (stereo_settings().stereo_algorithm == "1")
    stereo_settings().stereo_algorithm = "asp_sgm";
  else if (stereo_settings().stereo_algorithm == "2")
    stereo_settings().stereo_algorithm = "asp_mgm";
  else if (stereo_settings().stereo_algorithm == "3")
    stereo_settings().stereo_algorithm = "asp_final_mgm";

  // Add the options to the usage
  std::ostringstream os;
  os << usage << general_options;
  usage = os.str();

  // When called with no arguments, print the help message.
  if (vm.count("input-files") == 0)
    vw_throw(ArgumentErr() << "Missing input arguments.\n"
      << usage << general_options);
  input_files = vm["input-files"].as<std::vector<std::string>>();
  
  // For multiview, just store the files and return. Must happen after logging
  // starts, as logging for multiview is done in subdirectories. In multiview
  // mode, the logic further down will be later called for each pair.
  if (is_multiview)
    return;

  // Re-use the logic in parse_multiview_cmd_files, but just for two images/cameras.
  std::vector<std::string> images, cameras;
  if (!parse_multiview_cmd_files(override_out_prefix, input_files, // inputs
                                 images, cameras, opt.out_prefix, opt.input_dem)) // outputs
    vw_throw(ArgumentErr() << "Missing the input files and/or output prefix.\n");
  if (images.size() >= 1)
    opt.in_file1 = images[0];
  if (images.size() >= 2)
    opt.in_file2 = images[1];
  if (cameras.size() >= 1)
    opt.cam_file1 = cameras[0];
  if (cameras.size() >= 2)
    opt.cam_file2 = cameras[1];
  if (opt.in_file1.empty() || opt.in_file2.empty() || opt.out_prefix.empty())
    vw_throw(ArgumentErr() << "Missing the input files and/or output prefix.\n");

  // Create the output directory
  vw::create_out_dir(opt.out_prefix);

  // Turn on logging to file, except for stereo_parse, as that one is called
  // all the time.
  std::string prog_name = extract_prog_name(argv[0]);
  if (prog_name.find("stereo_parse") == std::string::npos)
    asp::log_to_file(argc, argv, opt.stereo_default_filename, opt.out_prefix);

  // There are two crop win boxes, in respect to original left
  // image, named left_image_crop_win, and in respect to the
  // transformed left image (L.tif), named trans_crop_win. We use
  // the second if available, otherwise we transform and use the
  // first. The box trans_crop_win is for internal use, invoked
  // from parallel_stereo.

  // Interpret the the last two coordinates of the crop win boxes as
  // width and height rather than max_x and max_y.
  BBox2i bl = stereo_settings().left_image_crop_win;
  BBox2i br = stereo_settings().right_image_crop_win;
  BBox2i bt = stereo_settings().trans_crop_win;
  stereo_settings().left_image_crop_win
    = BBox2i(bl.min().x(), bl.min().y(), bl.max().x(), bl.max().y());
  stereo_settings().right_image_crop_win
    = BBox2i(br.min().x(), br.min().y(), br.max().x(), br.max().y());
  stereo_settings().trans_crop_win
    = BBox2i(bt.min().x(), bt.min().y(), bt.max().x(), bt.max().y());

  int num_left_bands = vw::get_num_channels(opt.in_file1);
  int num_right_bands = vw::get_num_channels(opt.in_file2);

  // Ensure the crop windows are always contained in the images.
  boost::shared_ptr<vw::DiskImageResource> left_resource, right_resource;
  left_resource  = vw::DiskImageResourcePtr(opt.in_file1);
  right_resource = vw::DiskImageResourcePtr(opt.in_file2);

  // For multi-band images, this will only read the first band. This is enough
  // for now as we do only bounding box checks. During stereo preprocessing, the
  // images will be opened again and the correct band will be used.
  DiskImageView<float> left_image(left_resource);
  DiskImageView<float> right_image(right_resource);
  stereo_settings().left_image_crop_win.crop (bounding_box(left_image));
  stereo_settings().right_image_crop_win.crop(bounding_box(right_image));

  bool crop_left  = (stereo_settings().left_image_crop_win  != BBox2i(0, 0, 0, 0));
  bool crop_right = (stereo_settings().right_image_crop_win != BBox2i(0, 0, 0, 0));

  // If crops were specified, check that they are valid.
  if (crop_left && stereo_settings().left_image_crop_win.empty())
    vw_throw(ArgumentErr() << "Invalid left crop window specified!\n");
  if (crop_right && stereo_settings().right_image_crop_win.empty())
    vw_throw(ArgumentErr() << "Invalid right crop window specified!\n");

  // Make sure the trans_crop_win value is correct going forwards.
  if (!crop_left) {
    // The crop window after transforming the left image via
    // affine epipolar or homography alignment.
    if (stereo_settings().trans_crop_win == BBox2i(0, 0, 0, 0))
      stereo_settings().trans_crop_win = transformed_crop_win(opt);
      
    // Intersect with L.tif which is the transformed and processed left image.
    if (fs::exists(opt.out_prefix+"-L.tif")) {
      DiskImageView<PixelGray<float>> L_img(opt.out_prefix+"-L.tif");
      stereo_settings().trans_crop_win.crop(bounding_box(L_img));
    }
  } else {
    // If left_image_crop_win is specified, as can be see in
    // StereoSession::preprocessing_hook(), we actually
    // physically crop the image.  The trans_crop_win as passed
    // here from parallel_stereo will already be a tile in the
    // cropped image. So we just use it as it is. If it is not defined,
    // we set it to the entire cropped image.
    if (stereo_settings().trans_crop_win == BBox2i(0, 0, 0, 0)) {
      stereo_settings().trans_crop_win = bounding_box(left_image);
      if (fs::exists(opt.out_prefix+"-L.tif")) {
        DiskImageView<PixelGray<float>> L_img(opt.out_prefix+"-L.tif");
        stereo_settings().trans_crop_win = bounding_box(L_img);
      }
    }
  } // End crop checking case

  // If not using crop wins but the crop image exists, then things won't go well.
  if (!crop_left && !crop_right &&
      (fs::exists(opt.out_prefix+"-L-cropped.tif") ||
        fs::exists(opt.out_prefix+"-R-cropped.tif")))
    vw_throw(ArgumentErr() << "The current output prefix '" << opt.out_prefix
              << "' has an old run which used --left-image-crop-win, "
              << "but the current run does not. Results will be incorrect. "
              << "Use a new output prefix.");

  // TODO: May need to update this check for individual crop cases.
  // Sanity check. Don't run it if we have L-cropped.tif or R-cropped.tif,
  // in that case we have ran the gui before, and the sizes of the subimages
  // could be anything. We'll regenerate any of those anyway soon.
  if ((stereo_settings().trans_crop_win.width () <= 0 ||
        stereo_settings().trans_crop_win.height() <= 0) &&
      !fs::exists(opt.out_prefix+"-L-cropped.tif")     &&
      !fs::exists(opt.out_prefix+"-R-cropped.tif")) {
    vw_throw(ArgumentErr() << "Invalid region for doing stereo.\n\n");
  }

  // For time being the crop wins are not taken into account when
  // matches are produced from disparity, and the results are wrong.
  // Therefore, disable this.
  if ((crop_left || crop_right) &&
      (stereo_settings().num_matches_from_disparity > 0 ||
       stereo_settings().num_matches_from_disp_triplets > 0))
    vw_throw(ArgumentErr() << "Cannot use --num-matches-from-disparity or "
              << "--num-matches-from-disp-triplets with --left-image-crop-win or "
              << "--right-image-crop-win. The alternative is to manually crop "
              << "the left and right images while keeping the upper-left corner. "
              << "Otherwise the results would be incorrect.\n");

  // This does not work because tx_left() and tx_right() return the identity for
  // this alignment method. See StereoSessionPinhole::tx_left() for more
  // details.
  if (asp::stereo_settings().alignment_method == "epipolar" &&
      (stereo_settings().num_matches_from_disparity > 0 ||
       stereo_settings().num_matches_from_disp_triplets > 0))
    vw_throw(ArgumentErr() << "Cannot use --num-matches-from-disparity or "
              << "--num-matches-from-disp-triplets with epipolar alignment.\n");

  // Cannot have both matches from disparity and triplets
  if (stereo_settings().num_matches_from_disparity > 0 &&
      stereo_settings().num_matches_from_disp_triplets > 0)
    vw_throw(ArgumentErr() << "Cannot have both --num-matches-from-disparity and "
              << "--num-matches-from-disp-triplets.\n");

  // In the latest ASP always create triplets
  if (stereo_settings().num_matches_from_disparity > 0) {
    vw::vw_out(vw::WarningMessage)
      << "The option --num-matches-from-disparity is equivalent to "
      << "--num-matches-from-disp-triplets.\n";
    stereo_settings().num_matches_from_disp_triplets
      = stereo_settings().num_matches_from_disparity;
   stereo_settings().num_matches_from_disparity = 0;
  }

  // Ensure good order
  BBox2 & b = stereo_settings().lon_lat_limit; // alias
  if (b != BBox2(0,0,0,0)) {
    if (b.min().y() > b.max().y())
      std::swap(b.min().y(), b.max().y());
    if (b.min().x() > b.max().x())
      std::swap(b.min().x(), b.max().x());
  }

  if (!stereo_settings().match_files_prefix.empty() &&
      !stereo_settings().clean_match_files_prefix.empty())
    vw_throw(ArgumentErr() << "Cannot specify both --match-files-prefix and "
              << "--clean-match-files-prefix.\n");

  if (!stereo_settings().corr_search_limit.empty() && stereo_settings().max_disp_spread > 0)
    vw_throw(ArgumentErr() << "Cannot specify both --corr-search-limit and "
              << "--max-disp-spread.\n");

  // Verify that there is only one channel per input image
  if (asp::skip_image_normalization(opt) &&
    (num_left_bands > 1 || num_right_bands > 1))
    vw_throw(ArgumentErr()
             << "Error: Cannot skip image normalization if the input images "
             << "have more than one band (channel).\n\n");

  // Print a warning if more than one band exists and the band was not set.
  if (stereo_settings().band == -1 &&
      (num_left_bands > 1 || num_right_bands > 1)) {
    vw_out(WarningMessage) << "The input images have more than one band (channel), "
                            << "but the --band option was not set. Using band 1.\n";
    stereo_settings().band = 1;
  }

  // Having printed the warning, set the band to 1 if it was not set.
  if (stereo_settings().band == -1)
    stereo_settings().band = 1;

  // Sanity check
  if (stereo_settings().band <= 0 ||
      stereo_settings().band > num_left_bands ||
      stereo_settings().band > num_right_bands)
    vw_throw(ArgumentErr() << "The value of --band is out of range.\n");

  if ((stereo_settings().bundle_adjust_prefix != "") &&
      (stereo_settings().alignment_method == "epipolar"))
    vw_throw(ArgumentErr() << "Error: Epipolar alignment does not support using a "
              << "bundle adjust prefix.\n");

  // Replace normal default values with these when SGM is enabled.
  // - TODO: Move these somewhere easier to find!
  const int SGM_DEFAULT_SUBPIXEL_MODE        = 12; // Blend
  const int SGM_DEFAULT_COST_MODE            = 4;
  const int SGM_DEFAULT_KERNELSIZE           = 5;
  const int SGM_DEFAULT_RM_CLEANUP_PASSES    = 0;
  const int SGM_DEFAULT_MEDIAN_FILTER_SIZE   = 3;
  const int SGM_DEFAULT_TEXTURE_SMOOTH_SIZE  = 11;
  const double SGM_DEFAULT_TEXTURE_SMOOTH_SCALE = 0.13;

  // Increase the timeout for MGM, unless specified, as that one is slow.
  // Need some care here, to make sure that even if the parent function is called
  // twice, the increase happens just once.
  if (stereo_settings().stereo_algorithm == "mgm" &&
      stereo_settings().corr_timeout == stereo_settings().default_corr_timeout) {
      stereo_settings().corr_timeout = 10 * stereo_settings().default_corr_timeout;
    vw_out() << "For the original mgm algorithm increasing the --corr-timeout to: "
             << stereo_settings().corr_timeout << ".\n";
  }

  // TODO: Modify SGM tile sizes?

  vw::stereo::CorrelationAlgorithm stereo_alg
    = asp::stereo_alg_to_num(stereo_settings().stereo_algorithm);

  if (stereo_alg > vw::stereo::VW_CORRELATION_BM) {
    // If these parameters were not specified by the user, override
    // the normal default values.  Note that by setting
    // subpixel_mode to SGM_DEFAULT_SUBPIXEL_MODE, we will do no
    // further subpixel refinement than what all algorithms except
    // ASP's block matching are intrinsically capable of.  if the
    // user however explicitly specifies, for example,
    // --subpixel-mode 3, that one will be used later on.
    // TODO(oalexan1): Consider setting the default --subpixel-mode to 9
    // with non-BM algorithms, as it results in nicer results.

    // These are also useful with external algorithms, as the results are then
    // smoother.
    if (vm["rm-cleanup-passes"].defaulted())
      stereo_settings().rm_cleanup_passes = SGM_DEFAULT_RM_CLEANUP_PASSES;
    if (vm["median-filter-size"].defaulted())
      stereo_settings().median_filter_size = SGM_DEFAULT_MEDIAN_FILTER_SIZE;
    if (vm["texture-smooth-size"].defaulted())
      stereo_settings().disp_smooth_size = SGM_DEFAULT_TEXTURE_SMOOTH_SIZE;
    if (vm["texture-smooth-scale"].defaulted())
      stereo_settings().disp_smooth_texture = SGM_DEFAULT_TEXTURE_SMOOTH_SCALE;
    if (vm["subpixel-mode"].defaulted())
      stereo_settings().subpixel_mode = SGM_DEFAULT_SUBPIXEL_MODE;

    // This is for the case when settings are read from stereo.default. Print
    // some warnings.
    if (stereo_settings().rm_cleanup_passes != SGM_DEFAULT_RM_CLEANUP_PASSES)
      vw_out(WarningMessage) << "When using a stereo algorithm rather than asp_bm, "
                              << "the default suggested value for "
                              << "rm-cleanup-passes is "
                              << SGM_DEFAULT_RM_CLEANUP_PASSES << ". "
                              << "Got instead " << stereo_settings().rm_cleanup_passes
                              << ".\n";
    if (stereo_settings().median_filter_size != SGM_DEFAULT_MEDIAN_FILTER_SIZE)
      vw_out(WarningMessage) << "When using a stereo algorithm rather than asp_bm, "
                              << "the default suggested value for "
                              << "median-filter-size is "
                              << SGM_DEFAULT_MEDIAN_FILTER_SIZE << ". "
                              << "Got instead " << stereo_settings().median_filter_size
                              << ".\n";
    if (stereo_settings().disp_smooth_size != SGM_DEFAULT_TEXTURE_SMOOTH_SIZE)
      vw_out(WarningMessage) << "When using a stereo algorithm rather than asp_bm, "
                              << "the default suggested value for "
                              << "texture-smooth-size is "
                              << SGM_DEFAULT_TEXTURE_SMOOTH_SIZE << ". "
                              << "Got instead " << stereo_settings().disp_smooth_size
                              << ".\n";
    if (stereo_settings().disp_smooth_texture != SGM_DEFAULT_TEXTURE_SMOOTH_SCALE)
      vw_out(WarningMessage) << "When using a stereo algorithm rather than asp_bm, "
                              << "the default suggested value for "
                              << "texture-smooth-scale is "
                              << SGM_DEFAULT_TEXTURE_SMOOTH_SCALE << ". "
                              << "Got instead " << stereo_settings().disp_smooth_texture
                              << ".\n";
  }

  bool using_tiles = (stereo_alg > vw::stereo::VW_CORRELATION_BM ||
                      stereo_settings().alignment_method == "local_epipolar");

  // Settings for asp_mgm / asp_sgm. For external algorithms, the low-res
  // disparity in stereo_corr will be created with asp_mgm, so override
  // user's choice in that case.
  if (stereo_alg > vw::stereo::VW_CORRELATION_BM) {

    if (vm["cost-mode"].defaulted() || stereo_alg >= vw::stereo::VW_CORRELATION_OTHER)
      stereo_settings().cost_mode = SGM_DEFAULT_COST_MODE;
    if (vm["corr-kernel"].defaulted() || stereo_alg >= vw::stereo::VW_CORRELATION_OTHER)
      stereo_settings().corr_kernel
        = Vector2i(SGM_DEFAULT_KERNELSIZE, SGM_DEFAULT_KERNELSIZE);

    // This is a fix for the user setting cost-mode in stereo.default, when
    // it is not defaulted. Do not allow cost mode to be different than
    // 3 or 4 for asp_sgm / asp_mgm, as it produced junk.
    if (stereo_settings().cost_mode != 3 && stereo_settings().cost_mode != 4)
      vw_throw(ArgumentErr() << "When using the asp_sgm or asp_mgm "
              << "stereo algorithm, cost-mode must be 3 or 4. "
              << "Check your stereo.default or command-line options.\n");
    // Also do not allow corr-kernel to be outside of [3, 9]
    if (stereo_settings().corr_kernel[0] < 3 || stereo_settings().corr_kernel[0] > 9)
      vw_throw(ArgumentErr() << "For the asp_sgm / asp_mgm algorithm, "
        << "the corr kernel size must be between 3 and 9 (inclusive). "
        << "Check your stereo.default or command-line options.\n");
  }

  if (stereo_alg == vw::stereo::VW_CORRELATION_BM && stereo_settings().subpixel_mode > 6)
    vw::vw_throw(vw::ArgumentErr() << "Subpixel mode " << stereo_settings().subpixel_mode
              << " is not supported with block matching. Use mode <= 6.\n");

  if (!using_tiles) {
    // No need for a collar when we are not using tiles.
    stereo_settings().sgm_collar_size = 0;
  }

  if (stereo_alg >= vw::stereo::VW_CORRELATION_OTHER &&
      stereo_settings().alignment_method != "local_epipolar") {
    vw_throw(ArgumentErr() << "External stereo algorithms can be "
              << "used only with alignment method local_epipolar.\n");
  }

  if (exit_early)
    return;

  // The StereoSession call automatically determines the type of object to
  // create from the input parameters. In correlator mode there are no cameras,
  // so don't print the session.
  bool allow_map_promote = true;
  bool total_quiet = asp::stereo_settings().correlator_mode;
  opt.session.reset(asp::StereoSessionFactory::create(opt.stereo_session, // can change
                                                      opt,
                                                      opt.in_file1,   opt.in_file2,
                                                      opt.cam_file1,  opt.cam_file2,
                                                      opt.out_prefix, opt.input_dem,
                                                      allow_map_promote, total_quiet));

  // Load the cameras. They will be cached in the session.
  boost::shared_ptr<camera::CameraModel> camera_model1, camera_model2;
  opt.session->camera_models(camera_model1, camera_model2);

  // Run a set of checks to make sure the settings are compatible.
  user_safety_checks(opt);

  // This logic must happen after the cameras are loaded.
  if (stereo_settings().propagate_errors)
    setup_error_propagation(opt);

  // The last thing we do before we get started is to copy the
  // stereo.default settings over into the results directory so that
  // we have a record of the most recent stereo.default that was used
  // with this data set.
  asp::stereo_settings().write_copy(argc, argv,
                                    opt.stereo_default_filename,
                                    opt.out_prefix + "-stereo.default");
}

// Register Session types
void stereo_register_sessions() {
  // Register the Isis file handler with the Vision Workbench DiskImageResource system.
#if defined(ASP_HAVE_PKG_ISIS) && ASP_HAVE_PKG_ISIS == 1
  DiskImageResource::register_file_type(".cub",
                                        DiskImageResourceIsis::type_static(),
                                        &DiskImageResourceIsis::construct_open,
                                        &DiskImageResourceIsis::construct_create);
#endif
}

void user_safety_checks(ASPGlobalOptions const& opt) {

  // Error checking

  bool dem_provided = !opt.input_dem.empty();

  vw::Vector2 heights = asp::stereo_settings().ortho_heights;
  bool have_heights = (!std::isnan(heights[0]) && !std::isnan(heights[1]));
  if (have_heights && dem_provided)
    vw_throw(ArgumentErr()
             << "The option --ortho-heights expects no DEM as input argument.\n");

  // We will work as if the images were mapprojected
  if (have_heights)
    dem_provided = true;

  // Seed mode valid values
  if (stereo_settings().seed_mode > 3)
    vw_throw(ArgumentErr() << "Invalid value for --corr-seed-mode: "
              << stereo_settings().seed_mode << ".\n");

  if (stereo_settings().seed_mode == 2) {

    if (stereo_settings().disparity_estimation_dem_error <= 0.0)
      vw_throw(ArgumentErr()
        << "For --corr-seed-mode 2, the value of disparity-estimation-dem-error "
        << "must be positive.");

    if (stereo_settings().disparity_estimation_dem.empty())
      vw_throw(ArgumentErr()
        << "For --corr-seed-mode 2, must set --disparity-estimation-dem.\n");

    if (stereo_settings().alignment_method == "epipolar")
      vw_throw(ArgumentErr()
        << "For --corr-seed-mode 2, cannot use epipolar alignment.\n");
  }

  // Must use map-projected images if input DEM is provided
  GeoReference georef1, georef2;
  bool has_georef1 = vw::cartography::read_georeference(georef1, opt.in_file1);
  bool has_georef2 = vw::cartography::read_georeference(georef2, opt.in_file2);
  if (dem_provided && (!has_georef1 || !has_georef2)) {
    vw_throw(ArgumentErr() << "The images are not map-projected, "
              << "cannot use the provided DEM: " << opt.input_dem << "\n");
  }

  // If the images are map-projected, they need to use the same projection.
  if (dem_provided && georef1.get_wkt() != georef2.get_wkt())
    vw_throw(ArgumentErr() << "The left and right images must use the same projection.\n");

  // Must check here for same resolution. This is an endless source of bugs.
  if (dem_provided && !stereo_settings().allow_different_mapproject_gsd) {
    auto M1 = georef1.transform();
    auto M2 = georef2.transform();
    // The diagonal terms of these must be equal.
    double tol = 1e-10;
    if (std::abs(M1(0, 0) - M2(0, 0)) > tol || std::abs(M1(1, 1) - M2(1, 1)) > tol)
        vw::vw_throw(vw::ArgumentErr()
               << "The input mapprojected images must have the same ground resolution "
               << "for best results. This can be overriden with the option "
               << "--allow-different-mapproject-gsd, but is not recommended.\n");
  }

  // If the images are map-projected, we need an input DEM, as we use the ASP
  // flow with map-projected images.
  bool corr_only = stereo_settings().correlator_mode;
  if (has_georef1 && has_georef2 && !dem_provided && !corr_only) {

    // If we can identify the DEM these were map-projected from, that's a fatal
    // error.
    std::string l_dem_file, r_dem_file;
    std::string dem_file_key = "DEM_FILE";
    boost::shared_ptr<vw::DiskImageResource>
      l_rsrc(new vw::DiskImageResourceGDAL(opt.in_file1));
    vw::cartography::read_header_string(*l_rsrc.get(), dem_file_key, l_dem_file);
    boost::shared_ptr<vw::DiskImageResource>
      r_rsrc(new vw::DiskImageResourceGDAL(opt.in_file2));
    vw::cartography::read_header_string(*r_rsrc.get(), dem_file_key, r_dem_file);
    if (l_dem_file != "" || r_dem_file != "")
      vw_throw(ArgumentErr() << "The input images appear to be map-projected, "
                << "but no DEM was provided. Please provide a DEM.\n");

    // Otherwise, just print a warning. Maybe the user got these from somewhere else.
    vw_out(WarningMessage) << "It appears that the input images are "
                            << "map-projected. In that case a DEM needs to be "
                            << "provided for stereo to give correct results.\n";
  }

  // Check that if the user provided a dem that we are using a map projection method
  if (dem_provided && !opt.session->isMapProjected() && !corr_only) {
    vw_throw(ArgumentErr() << "Cannot use map-projected images with a session of type: "
                            << opt.session->name() << ".\n");
  }

  // No alignment must be set for map-projected images.
  if (stereo_settings().alignment_method != "none" && dem_provided) {
      stereo_settings().alignment_method  = "none";
    vw_out(WarningMessage) << "Changing the alignment method to 'none' "
                            << "as the images are map-projected." << "\n";
  }

  if (stereo_settings().corr_kernel[0]%2 == 0 ||
      stereo_settings().corr_kernel[1]%2 == 0) {
    vw_throw(ArgumentErr() << "The entries of corr-kernel must be odd numbers.\n");
  }

  if (stereo_settings().subpixel_kernel[0]%2 == 0 ||
      stereo_settings().subpixel_kernel[1]%2 == 0) {
    vw_throw(ArgumentErr() << "The entries of subpixel-kernel must be odd numbers.\n");
  }

  // Check SGM-related settings

  vw::stereo::CorrelationAlgorithm stereo_alg
    = asp::stereo_alg_to_num(stereo_settings().stereo_algorithm);

  // For external algorithms we will still use the MGM algorithm for low-res
  // disparity, so cost mode of 3 and 4 is fine unless for regular block matching.
  if (stereo_alg == vw::stereo::VW_CORRELATION_BM) {
    if (stereo_settings().cost_mode == 3 || stereo_settings().cost_mode == 4)
      vw_throw(ArgumentErr() << "Cannot use the census transform with "
                << "the ASP_BM block matching algorithm.\n");
  }

  if (stereo_settings().cost_mode > 4)
    vw_throw(ArgumentErr() << "Unknown value " << stereo_settings().cost_mode
              << " for cost-mode.\n");

  if (stereo_settings().min_triangulation_angle <= 0 &&
      stereo_settings().min_triangulation_angle != -1) {
    // This means the user modified it. Then it must be positive.
    vw_throw(ArgumentErr() << "The min triangulation angle must be positive.\n");
  }
  if (stereo_settings().min_triangulation_angle == -1) {
    // This means that the user did not set it. Set it to 0.
    // Deep inside StereoModel.cc it will be overwritten with some
    // positive value.
    // This is a bit awkward but is done so for backward compatibility.
    stereo_settings().min_triangulation_angle = 0;
  }

  if (opt.session->do_bathymetry())
    asp::bathyChecks(opt.session->name(), asp::stereo_settings()); 

  // Need the percentage to be more than 50 as we look at the range [100 - pct, pct].
  if (stereo_settings().outlier_removal_params[0] <= 50.0)
    vw_throw(ArgumentErr() 
             << "The --outlier-removal-params percentage must be more than 50.\n");
  if (stereo_settings().outlier_removal_params[1] <= 0.0)
    vw_throw(ArgumentErr() << "The --outlier-removal-params factor must be positive.\n");

  if (stereo_settings().save_lr_disp_diff) {

    if (stereo_settings().xcorr_threshold < 0.0)
      vw_throw(ArgumentErr() << "Must have a non-negative value of --xcorr-threshold "
                << "to be able to use --save-left-right-disparity-difference.\n");

    if (stereo_alg >= vw::stereo::VW_CORRELATION_OTHER)
      vw_throw(ArgumentErr() << "Can use --save-left-right-disparity-difference "
                << "only with stereo algorithms asp_bm, asp_sgm, asp_mgm, and "
                << "asp_final_mgm.\n");
  }

  if (stereo_settings().propagate_errors && stereo_settings().compute_error_vector)
    vw::vw_throw(vw::ArgumentErr() << "Cannot use option --error-vector for computing "
                  << "the triangulation error vector when propagating errors (covariances) "
                  << "from cameras, as those are stored instead in "
                  << "bands 5 and 6.\n");

  // Camera checks
  if (!stereo_settings().correlator_mode) {
    try {
      // Note. Cameras are loaded just once, and repeated invocation of camera_models()
      // will not reload them. Hence this check does not incur a performance hit
      // due to loading of the cameras.
      boost::shared_ptr<camera::CameraModel> camera_model1, camera_model2;
      opt.session->camera_models(camera_model1, camera_model2);

      Vector3 cam1_ctr = camera_model1->camera_center(Vector2());
      Vector3 cam2_ctr = camera_model2->camera_center(Vector2());
      Vector3 cam1_vec = camera_model1->pixel_to_vector(Vector2());
      Vector3 cam2_vec = camera_model2->pixel_to_vector(Vector2());
      // Do the cameras appear to be in the same location?
      if (norm_2(cam1_ctr - cam2_ctr) < 1e-3)
        vw_out(WarningMessage)
          << "Your cameras appear to be in the same location!\n"
          << "\tYou should double check your given camera\n"
          << "\tmodels as most likely stereo won't be able\n"
          << "\tto triangulate or perform epipolar rectification.\n";

      // Developer friendly help
      VW_OUT(DebugMessage,"asp") << "Camera 1 location: " << cam1_ctr << "\n"
                                  << "   in estimated Lon Lat Rad: "
                                  << cartography::xyz_to_lon_lat_radius_estimate(cam1_ctr) << "\n";
      VW_OUT(DebugMessage,"asp") << "Camera 2 location: " << cam2_ctr << "\n"
                                  << "   in estimated Lon Lat Rad: "
                                  << cartography::xyz_to_lon_lat_radius_estimate(cam2_ctr) << "\n";
      VW_OUT(DebugMessage,"asp") << "Camera 1 pointing dir: " << cam1_vec << "\n"
                                  << "      dot against pos: " << dot_prod(cam1_vec, cam1_ctr)
                                  << "\n";
      VW_OUT(DebugMessage,"asp") << "Camera 2 pointing dir: " << cam2_vec << "\n"
                                  << "      dot against pos: " << dot_prod(cam2_vec, cam2_ctr)
                                  << "\n";

      // For RPC cameras the camera center is not accurate, so don't print it.
      if (opt.stereo_session != "rpc" &&
          opt.stereo_session.find("rpcmap") == std::string::npos)
        vw_out() << "Distance between camera centers: "
                 << norm_2(cam1_ctr - cam2_ctr) << " meters.\n";

      // Can cameras triangulate to point at something in front of them?
      stereo::StereoModel model(camera_model1.get(), camera_model2.get());
      double error;
      Vector3 point = model(Vector2(), Vector2(), error);
      if (point != Vector3() && // triangulation succeeded
          ((dot_prod(cam1_vec, point - cam1_ctr) < 0) ||
           (dot_prod(cam2_vec, point - cam2_ctr) < 0))) {
        vw::vw_out(vw::WarningMessage)
          << "Your cameras appear to not to be pointing at the same location. "
          << "A test vector triangulated backwards through the camera models. "
          << "You should double-check your input cameras as most likely stereo "
          << "will not be able to triangulate.\n";
      }

    } catch (const std::exception& e) {
      // Don't throw an error here. There are legitimate reasons as to
      // why the first checks may fail. For example, the top left pixel
      // might not be valid on a map projected image. But notify the
      // user anyway.
      vw_out(DebugMessage,"asp") << e.what() << "\n";
    }
  } // end camera checks

} // End user_safety_checks

// See if user's request to skip image normalization can be
// satisfied.  This option is a speedup switch which is only meant
// to work with with mapprojected images. It is also not documented.
bool skip_image_normalization(ASPGlobalOptions const& opt) {

  if (!stereo_settings().skip_image_normalization)
    return false;

  bool crop_left  = (stereo_settings().left_image_crop_win  != BBox2i(0, 0, 0, 0));
  bool crop_right = (stereo_settings().right_image_crop_win != BBox2i(0, 0, 0, 0));

  // Respect user's choice for skipping the normalization of the input
  // images, if feasible.
  bool is_good = (!crop_left && !crop_right                    &&
                  stereo_settings().alignment_method == "none" &&
                  stereo_settings().cost_mode == 2             &&
                  vw::has_tif_or_ntf_extension(opt.in_file1)   &&
                  vw::has_tif_or_ntf_extension(opt.in_file2));

  if (!is_good)
    vw_throw(ArgumentErr()
              << "Cannot skip image normalization unless there is no alignment, "
              << "no use of --left-image-crop-win and --right-image-crop-win, "
              << "the option --cost-mode is set to 2, and the input images have "
              << ".tif or .ntf extension.");

  return is_good;
} // End function skip_image_normalization

// Convert, for example, 'asp_mgm' to '2'. For ASP algorithms we
// use the numbers 0 (BM), 1 (SGM), 2 (MGM), 3 (Final MGM).  For
// external algorithms will have to examine closer the algorithm
// string. This function has a Python analog in parallel_stereo.
vw::stereo::CorrelationAlgorithm stereo_alg_to_num(std::string alg) {

  // Make it lowercase first
  boost::to_lower(alg);

  // Sanity check
  if (alg == "")
    vw_throw(ArgumentErr() << "No stereo algorithm was specified.\n");

  if (alg.rfind("0", 0) == 0 || alg.rfind("asp_bm", 0) == 0)
    return vw::stereo::VW_CORRELATION_BM;

  if (alg.rfind("1", 0) == 0 || alg.rfind("asp_sgm", 0) == 0)
    return vw::stereo::VW_CORRELATION_SGM;

  if (alg.rfind("2", 0) == 0 || alg.rfind("asp_mgm", 0) == 0)
    return vw::stereo::VW_CORRELATION_MGM;

  if (alg.rfind("3", 0) == 0 || alg.rfind("asp_final_mgm", 0) == 0)
    return vw::stereo::VW_CORRELATION_FINAL_MGM;

  // Sanity check. Any numerical values except 0, 1, 2, 3 are not accepted.
  int num = atof(alg.c_str());
  if (num < 0 || num > 3)
    vw_throw(ArgumentErr() << "Unknown algorithm: " << alg << ".\n");

  // An external stereo algorithm
  return vw::stereo::VW_CORRELATION_OTHER;
}

// Find the median angle in degrees at which rays emanating from
// matching points meet
void estimate_convergence_angle(ASPGlobalOptions const& opt) {

  if (stereo_settings().correlator_mode)
    return; // No camera can be assumed, hence no convergence angle.

  // When having matches between L and R, need to do things a bit differently.
  bool have_aligned_matches = (stereo_settings().alignment_method == "none" ||
                               stereo_settings().alignment_method == "epipolar");

  std::string match_filename;
  if (have_aligned_matches)
    match_filename = vw::ip::match_filename(opt.out_prefix, "L.tif", "R.tif");
  else
    match_filename = asp::stereo_match_filename(opt.session->left_cropped_image(),
                                                opt.session->right_cropped_image(),
                                                opt.out_prefix);
  // The interest points must exist by now. But be tolerant of failure, as
  // this functionality is not critical.
  if (!fs::exists(match_filename)) {
    vw::vw_out(vw::WarningMessage)
      << "Cannot estimate the convergence angle, as cannot find the match file: "
      << match_filename << ".\n";
    return;
  }

  std::vector<ip::InterestPoint> left_ip, right_ip;
  ip::read_binary_match_file(match_filename, left_ip, right_ip);
  
  if (have_aligned_matches) {
    // Create the transforms ahead of time for clarity. When these are created
    // as part of unalign_ip() arguments, the creation order seems in reverse.
    auto left_tx = opt.session->tx_left();
    auto right_tx = opt.session->tx_right();

    // Unalign the interest point matches
    std::vector<vw::ip::InterestPoint> unaligned_left_ip, unaligned_right_ip;
    asp::unalign_ip(left_tx, right_tx, left_ip, right_ip,
                    unaligned_left_ip, unaligned_right_ip);
    left_ip  = unaligned_left_ip;
    right_ip = unaligned_right_ip;
  }

  std::vector<double> sorted_angles;
  boost::shared_ptr<camera::CameraModel> left_cam, right_cam;
  opt.session->camera_models(left_cam, right_cam);
  asp::convergence_angles(left_cam.get(), right_cam.get(), left_ip, right_ip, sorted_angles);

  if (sorted_angles.empty()) {
    vw_out(vw::WarningMessage) << "Could not compute the stereo convergence angle.\n";
    return;
  }

  int len = sorted_angles.size();
  vw_out() << "Convergence angle percentiles (in degrees) based on interest point matches:\n";
  vw_out() << "\t"
           << "25% " << sorted_angles[0.25*len] << ", "
           << "50% " << sorted_angles[0.50*len] << ", "
           << "75% " << sorted_angles[0.75*len] << ".\n";

   if (sorted_angles[0.50*len] < 5.0)
      vw_out(vw::WarningMessage)
        << "The stereo convergence angle is: " << sorted_angles[0.50*len] << " degrees. "
        << "This is quite low and may result in an empty or unreliable point cloud. "
        << "Reduce --min-triangulation-angle to triangulate with very small angles.\n";
}

} // end namespace asp

