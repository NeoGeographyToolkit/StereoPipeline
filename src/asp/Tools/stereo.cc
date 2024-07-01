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

/// \file stereo.cc

#include <asp/Tools/stereo.h>
#include <asp/Core/Bathymetry.h>
#include <asp/Sessions/StereoSessionFactory.h>
#include <asp/Core/AspStringUtils.h>
#include <asp/Camera/CameraErrorPropagation.h>

#include <vw/Cartography/PointImageManipulation.h>
#include <vw/Stereo/StereoView.h>
#include <vw/Stereo/PreFilter.h>
#include <vw/Stereo/CorrelationView.h>
#include <vw/Stereo/CostFunctions.h>
#include <vw/Stereo/DisparityMap.h>
#include <vw/FileIO/MatrixIO.h>

// Can't do much about warnings in boost except to hide them
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
#include <boost/accumulators/accumulators.hpp>
#include <boost/accumulators/statistics.hpp>
#pragma GCC diagnostic pop

using namespace vw;
using namespace vw::cartography;

namespace asp {

// Transform the crop window to be in reference to L.tif
BBox2i transformed_crop_win(ASPGlobalOptions const& opt){

  BBox2i b = stereo_settings().left_image_crop_win;
  boost::shared_ptr<vw::DiskImageResource> rsrc = 
          vw::DiskImageResourcePtr(opt.in_file1);
  DiskImageView<PixelGray<float>> left_image(rsrc);
  BBox2i full_box = bounding_box(left_image);
  if (b == BBox2i(0, 0, 0, 0)){

    // No box was provided. Use the full box.
    if ( fs::exists(opt.out_prefix+"-L.tif") ){
      DiskImageView<PixelGray<float>> L_img(opt.out_prefix+"-L.tif");
      b = bounding_box(L_img);
    }else{
      b = full_box; // To not have an empty box
    }

  }else{

    // Ensure that the region is inside the maximum theoretical region
    b.crop(full_box);

    if ( fs::exists(opt.out_prefix+"-align-L.exr") ){
      Matrix<double> align_left_matrix = math::identity_matrix<3>();
      read_matrix(align_left_matrix, opt.out_prefix + "-align-L.exr");
      b = HomographyTransform(align_left_matrix).forward_bbox(b);
    }

    if ( fs::exists(opt.out_prefix+"-L.tif") ){
      // Intersect with L.tif which is the transformed and processed left image
      DiskImageView<PixelGray<float>> L_img(opt.out_prefix+"-L.tif");
      b.crop(bounding_box(L_img));
    }

  }

  return b;
}

void parse_multiview(int argc, char* argv[],
                      boost::program_options::options_description const&
                      additional_options,
                      bool verbose,
                      std::string & output_prefix,
                      std::vector<ASPGlobalOptions> & opt_vec,
                      bool exit_early) {

  // If a stereo program is invoked as:

  // prog <images> <cameras> <output-prefix> [<input_dem>] <other options>

  // with the number of images n >= 2, create n-1 individual
  // ASPGlobalOptions entries, corresponding to n-1 stereo pairs between the
  // first image and each of the subsequent images.

  //vw_out() << "DEBUG - parse_multiview inputs:" << std::endl;
  //for (int i=0; i<argc; ++i) vw_out() << argv[i] << std::endl;

  // First reset the outputs
  output_prefix.clear();
  opt_vec.clear();

  // Extract the images/cameras/output prefix, and perhaps the input DEM
  std::vector<std::string> files;
  bool is_multiview = true;
  ASPGlobalOptions opt;
  std::string usage;
  handle_arguments(argc, argv, opt, additional_options,
                    is_multiview, files, usage, exit_early);

  // Need this for the GUI, ensure that opt_vec is never empty, even on failures
  opt_vec.push_back(opt);

  if (files.size() < 3)
    vw_throw(ArgumentErr() << "Missing all of the correct input files.\n\n" << usage);

  // Add note on the alignment method. If done in handle_arguments, it will be
  // printed twice.
  if (stereo_settings().correlator_mode) 
    vw_out() << "Running in correlator mode. The alignment method is: "
              << stereo_settings().alignment_method << ".\n";
  
  // If a file shows up more than once as input, that will confuse
  // the logic at the next step, so forbid that.
  std::map<std::string, int> vals;
  for (int s = 1; s < argc; s++)
    vals[argv[s]]++;
  for (int s = 0; s < (int)files.size(); s++){
    if (vals[files[s]] > 1) {
      vw_throw(ArgumentErr() << "The following input argument shows up more than "
                << "once and hence cannot be parsed correctly: "
                << files[s] << ".\n\n" << usage);
    }
  }

  // Store the options and their values (that is, not the input files).
  std::set<std::string> file_set;
  for (int s = 0; s < (int)files.size(); s++)
    file_set.insert(files[s]);
  std::vector<std::string> options;
  for (int s = 1; s < argc; s++){
    if (file_set.find(argv[s]) == file_set.end())
      options.push_back(argv[s]);
  }

  // Extract all the positional elements
  std::vector<std::string> images, cameras;
  std::string input_dem;
  if (!parse_multiview_cmd_files(files, images, cameras, output_prefix, input_dem))
    vw_throw(ArgumentErr() << "Missing all of the correct input files.\n\n" << usage);

  int num_pairs = (int)images.size() - 1;
  if (num_pairs <= 0)
    vw_throw(ArgumentErr() << "Insufficient number of images provided.\n");

  if (num_pairs > 1 && stereo_settings().propagate_errors) 
    vw::vw_throw(vw::ArgumentErr() << "Error propagation is not "
                  << "implemented for more than two images.\n");

  // Must signal to the children runs that they are part of a multiview run
  if (num_pairs > 1) {
    std::string opt_str = "--part-of-multiview-run";
    std::vector<std::string>::iterator it = find(options.begin(), options.end(),
                                                  opt_str);
    if (it == options.end())
      options.push_back(opt_str);
  }

  // Multiview is very picky about alignment method
  if ( (num_pairs > 1 || stereo_settings().part_of_multiview_run) &&
        stereo_settings().alignment_method != "none"               &&
        stereo_settings().alignment_method != "homography"){

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

    // Set this for future runs as well
    std::string align_opt = "--alignment-method";
    auto it = std::find(options.begin(), options.end(), align_opt);
    if (it != options.end() && it + 1 != options.end()){
      // Modify existing alignment
      *(it+1) = new_alignment;
    }else{
      // Set new alignment
      options.push_back(align_opt);
      options.push_back(new_alignment);
    }
    }

  // Needed for stereo_parse
  if (verbose)
    vw_out() << "num_stereo_pairs," << num_pairs << std::endl;

  std::string prog_name = extract_prog_name(argv[0]);

  // Generate the stereo command for each of the pairs made up of the first
  // image and each subsequent image, with corresponding cameras.
  opt_vec.resize(num_pairs);
  for (int p = 1; p <= num_pairs; p++){

    std::vector<std::string> cmd;
    cmd.push_back(prog_name);

    // The command line options go first
    for (int t = 0; t < (int)options.size(); t++)
      cmd.push_back(options[t]);

    cmd.push_back(images[0]); // left image
    cmd.push_back(images[p]); // right image

    if (!cameras.empty()){
      // Don't append the camera names if they are the same as the image names
      if ((images[0] != cameras[0]) && (images[p] != cameras[p])){
        cmd.push_back(cameras[0]); // left camera
        cmd.push_back(cameras[p]); // right camera
      }
    }

    std::string local_prefix = output_prefix;
    if (num_pairs > 1){
      // Need to have a separate output prefix for each pair
      std::ostringstream os;
      os << local_prefix << "-pair" << p << "/" << p;
      local_prefix = os.str();
    }
    cmd.push_back(local_prefix);

    if (!input_dem.empty())
      cmd.push_back(input_dem);

    // Create a local argc and argv for the given stereo pair and parse them.
    int largc = cmd.size();
    std::vector<char*> largv;
    for (int t = 0; t < largc; t++)
      largv.push_back((char*)cmd[t].c_str());
    ASPGlobalOptions opt;
    bool is_multiview = false;
    std::vector<std::string> files;
    handle_arguments(largc, &largv[0], opt, additional_options,
                      is_multiview, files, usage, exit_early);
    opt_vec[p-1] = opt;

    if (verbose) {
      // Needed for stereo_parse
      int big = 10000; // helps keep things in order in the python script
      vw_out() << "multiview_command_" << big + p << ",";
      for (int t = 0; t < largc-1; t++)
        vw_out() << cmd[t] << ",";
      if (largc > 0)
        vw_out() << cmd[largc-1];
      vw_out() << std::endl;
    }

  }

  if (num_pairs > 1 && prog_name != "stereo_parse" &&
      prog_name != "stereo_tri" && prog_name != "stereo_gui")
    vw_throw(ArgumentErr() << "The executable " << prog_name
              << " is not meant to be used directly with more than two images. "
              << "Use instead the stereo/parallel_stereo scripts with desired entry points.\n");

  // This must not happen earlier as StereoSession is not initialized yet
  if (num_pairs > 1 && opt.session->do_bathymetry()) 
    vw_throw(ArgumentErr() << "Bathymetry correction does not work with "
              << "multiview stereo.\n");
  
  if (opt.session->do_bathymetry() && stereo_settings().propagate_errors) 
    vw_throw(ArgumentErr() << "Error propagation is not implemented when "
              << "bathymetry is modeled.\n");
  
  if (stereo_settings().propagate_errors && 
      stereo_settings().compute_error_vector) 
    vw::vw_throw(vw::ArgumentErr() << "Cannot use option --error-vector for computing "
                  << "the triangulation error vector when propagating errors (covariances) "
                  << "from cameras, as those are stored instead in " 
                  << "bands 5 and 6.\n");
  
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
                      bool is_multiview, std::vector<std::string> & input_files,
                      std::string & usage, bool exit_early){

  // Add options whose values are stored in ASPGlobalOptions rather than in stereo_settings()
  po::options_description general_options_sub("");
  addAspGlobalOptions(general_options_sub, opt);

  // We distinguish between all_general_options, which is all the
  // options we must parse, even if we don't need some of them, and
  // general_options, which are the options specifically used by the
  // current tool, and for which we also print the help message.

  po::options_description general_options("");
  general_options.add(general_options_sub);
  general_options.add(additional_options);
  general_options.add(vw::GdalWriteOptionsDescription(opt));

  po::options_description all_general_options("");
  all_general_options.add(general_options_sub );
  all_general_options.add(generate_config_file_options(opt));

  po::options_description positional_options("");
  po::positional_options_description positional_desc;
  if (is_multiview){
    // The number of input files could be huge. Just store them in a vector,
    // we'll parse them in the caller.
    positional_options.add_options()
      ("input-files", po::value< std::vector<std::string>>(), "Input files");
    positional_desc.add("input-files", -1);
  }else{
    // Two-view, have left and right.
    positional_options.add_options()
      ("left-input-image",   po::value(&opt.in_file1),   "Left input image")
      ("right-input-image",  po::value(&opt.in_file2),   "Right input image")
      ("left-camera-model",  po::value(&opt.cam_file1),  "Left camera model file")
      ("right-camera-model", po::value(&opt.cam_file2),  "Right camera model file")
      ("output-prefix",      po::value(&opt.out_prefix), "Prefix for output filenames")
      ("input-dem",          po::value(&opt.input_dem),  "Input DEM");

    positional_desc.add("left-input-image",   1);
    positional_desc.add("right-input-image",  1);
    positional_desc.add("left-camera-model",  1);
    positional_desc.add("right-camera-model", 1);
    positional_desc.add("output-prefix",      1);
    positional_desc.add("input-dem",          1);
  }

  usage = "[options] <images> [<cameras>] <output_file_prefix> [DEM]\n  Extensions are automatically added to the output files.\n  Camera model arguments may be optional for some stereo session types (e.g., isis).\n  Stereo parameters should be set in the stereo.default file.";
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

    // Append the options from the config file. Do not overwrite the
    // options already set on the command line.
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

  // For multiview, just store the files and return
  if (is_multiview){
    if (vm.count("input-files") == 0)
      vw_throw(ArgumentErr() << "Missing input arguments.\n" << usage);
    input_files = vm["input-files"].as< std::vector<std::string>>();
    return;
  }

  // Re-use the logic in parse_multiview_cmd_files, but just for two images/cameras.
  std::vector<std::string> files;
  std::vector<std::string> images, cameras;
  if (!opt.in_file1.empty())   files.push_back(opt.in_file1);
  if (!opt.in_file2.empty())   files.push_back(opt.in_file2);
  if (!opt.cam_file1.empty())  files.push_back(opt.cam_file1);
  if (!opt.cam_file2.empty())  files.push_back(opt.cam_file2);
  if (!opt.out_prefix.empty()) files.push_back(opt.out_prefix);
  if (!opt.input_dem.empty())  files.push_back(opt.input_dem);
  if (!parse_multiview_cmd_files(files, // inputs
                                  images, cameras, opt.out_prefix, opt.input_dem)) // outputs
    vw_throw(ArgumentErr() << "Missing all of the correct input files.\n\n" << usage);

  opt.in_file1 = "";  if (images.size() >= 1)  opt.in_file1  = images[0];
  opt.in_file2 = "";  if (images.size() >= 2)  opt.in_file2  = images[1];
  opt.cam_file1 = ""; if (cameras.size() >= 1) opt.cam_file1 = cameras[0];
  opt.cam_file2 = ""; if (cameras.size() >= 2) opt.cam_file2 = cameras[1];

  if (opt.in_file1.empty() || opt.in_file2.empty() || opt.out_prefix.empty())
    vw_throw(ArgumentErr() << "Missing all of the correct input files.\n\n" << usage);

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

  // Ensure the crop windows are always contained in the images.
  boost::shared_ptr<vw::DiskImageResource> left_resource, right_resource;
  left_resource  = vw::DiskImageResourcePtr(opt.in_file1);
  right_resource = vw::DiskImageResourcePtr(opt.in_file2);
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
    if ( fs::exists(opt.out_prefix+"-L.tif") ){
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
      if ( fs::exists(opt.out_prefix+"-L.tif") ){
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
      !fs::exists(opt.out_prefix+"-R-cropped.tif") ){
    vw_throw(ArgumentErr() << "Invalid region for doing stereo.\n\n"
              << usage << general_options);
  }

  // For time being the crop wins are not taken into account when
  // matches are produced from disparity, and the results are wrong.
  // Therefore, disable this.
  if ((crop_left || crop_right) && 
      (stereo_settings().num_matches_from_disparity > 0 ||
       stereo_settings().num_matches_from_disp_triplets > 0))
    vw_throw(ArgumentErr() << "Cannot use --num-matches-from-disparity or "
              << "--num-matches-from-disp-triplets with --left-image-crop-win or "
              << "--right-image-crop-win.\n");

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
              << "--clean-match-files-prefix.\n\n" << usage << general_options);
  
  if (!stereo_settings().corr_search_limit.empty() && stereo_settings().max_disp_spread > 0)
    vw_throw(ArgumentErr() << "Cannot specify both --corr-search-limit and "
              << "--max-disp-spread.\n\n" << usage << general_options);

  // Verify that there is only one channel per input image
  if ( (left_resource->channels() > 1) || (right_resource->channels() > 1) )
    vw_throw(ArgumentErr() << "Error: Input images can only have a single channel.\n\n"
              << usage << general_options);

  if ((stereo_settings().bundle_adjust_prefix != "") &&
      (stereo_settings().alignment_method == "epipolar"))
    vw_throw(ArgumentErr() << "Error: Epipolar alignment does not support using a "
              << "bundle adjust prefix.\n\n" << usage << general_options);
  
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
    vw_out() << "For the original mgm algorithm increasing the --corr-timeout to: " <<
      stereo_settings().corr_timeout << ".\n";
  }

  if (stereo_settings().correlator_mode && !opt.input_dem.empty())
    vw_throw(ArgumentErr() << "Error: With --correlator-mode, use only two "
              << "input images and no reference DEM.\n");
    
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
  
  // The StereoSession call automatically determines the type of
  // object to create from the input parameters.
  opt.session.reset(asp::StereoSessionFactory::create(opt.stereo_session, // can change
                                                      opt,
                                                      opt.in_file1,   opt.in_file2,
                                                      opt.cam_file1,  opt.cam_file2,
                                                      opt.out_prefix, opt.input_dem));

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
  DiskImageResource::register_file_type(".cub",
                                        DiskImageResourceIsis::type_static(),
                                        &DiskImageResourceIsis::construct_open,
                                        &DiskImageResourceIsis::construct_create);
}

void user_safety_checks(ASPGlobalOptions const& opt) {

  // Error checking

  const bool dem_provided = !opt.input_dem.empty();

  // Seed mode valid values
  if (stereo_settings().seed_mode > 3){
    vw_throw(ArgumentErr() << "Invalid value for seed-mode: "
              << stereo_settings().seed_mode << ".\n");
  }

  // D_sub from DEM needs a positive disparity_estimation_dem_error
  if (stereo_settings().seed_mode == 2 &&
      stereo_settings().disparity_estimation_dem_error <= 0.0){
    vw_throw(ArgumentErr() 
              << "For seed-mode 2, the value of disparity-estimation-dem-error "
              << "must be positive.");
  }

  // D_sub from DEM needs a DEM
  if (stereo_settings().seed_mode == 2 &&
      stereo_settings().disparity_estimation_dem.empty()){
    vw_throw(ArgumentErr() << "For seed-mode 2, an input DEM must be provided.\n");
  }

  // D_sub from DEM does not work with map-projected images
  if (dem_provided && stereo_settings().seed_mode == 2)
    vw_throw(NoImplErr() << "Computation of low-resolution disparity from "
              << "DEM is not implemented for map-projected images.\n");

  // Must use map-projected images if input DEM is provided
  GeoReference georef1, georef2;
  bool has_georef1 = vw::cartography::read_georeference(georef1, opt.in_file1);
  bool has_georef2 = vw::cartography::read_georeference(georef2, opt.in_file2);
  if (dem_provided && (!has_georef1 || !has_georef2)){
    vw_throw(ArgumentErr() << "The images are not map-projected, "
              << "cannot use the provided DEM: " << opt.input_dem << "\n");
  }

  // If the images are map-projected, they need to use the same projection.
  if (dem_provided && georef1.get_wkt() != georef2.get_wkt())
    vw_throw(ArgumentErr() << "The left and right images must use the same projection.\n");

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
  if (dem_provided && !opt.session->uses_map_projected_inputs() && !corr_only) {
    vw_throw(ArgumentErr() << "Cannot use map-projected images with a session of type: "
                            << opt.session->name() << ".\n");
  }

  // No alignment must be set for map-projected images.
  if (stereo_settings().alignment_method != "none" && dem_provided) {
      stereo_settings().alignment_method  = "none";
    vw_out(WarningMessage) << "Changing the alignment method to 'none' "
                            << "as the images are map-projected." << std::endl;
  }

  if (stereo_settings().corr_kernel[0]%2 == 0 ||
      stereo_settings().corr_kernel[1]%2 == 0) {
    vw_throw(ArgumentErr() << "The entries of corr-kernel must be odd numbers.\n");
  }

  if (stereo_settings().subpixel_kernel[0]%2 == 0 ||
      stereo_settings().subpixel_kernel[1]%2 == 0   ){
    vw_throw(ArgumentErr() << "The entries of subpixel-kernel must be odd numbers.\n");
  }

  // Check SGM-related settings.

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

  if (opt.session->do_bathymetry()) {
    if (stereo_settings().refraction_index <= 1.0) 
      vw_throw(ArgumentErr() << "The water index of refraction to be used in "
                << "bathymetry correction must be bigger than 1.\n");

    if (stereo_settings().bathy_plane == "") 
      vw_throw(ArgumentErr() << "The value of --bathy-plane was unspecified.\n");

    // Sanity check reading the bathy plane
    std::vector<BathyPlaneSettings> bathy_plane_set;
    read_bathy_plane_set(stereo_settings().bathy_plane, bathy_plane_set);
    
    if (opt.session->name() != "dg" &&
        opt.session->name() != "rpc" &&
        opt.session->name() != "dgmaprpc" &&
        opt.session->name() != "rpcmaprpc" &&
        opt.session->name() != "nadirpinhole" &&
        opt.session->name() != "pinholemappinhole")
      vw_throw(ArgumentErr() << "Bathymetry correction only works with dg, rpc, and "
                << "nadirpinhole sessions, and mapprojected images for these. Got: "
                << opt.session->name() << ".\n");

    if (stereo_settings().alignment_method != "homography"     &&
        stereo_settings().alignment_method != "affineepipolar" &&
        stereo_settings().alignment_method != "local_epipolar" &&
        stereo_settings().alignment_method != "none") 
      vw_throw(ArgumentErr() << "Bathymetry correction only works with alignment methods "
                << "homography, affineepipolar, local_epipolar, and none.\n");
    
  }

  // Need the percentage to be more than 50 as we look at the range [100 - pct, pct].
  if (stereo_settings().outlier_removal_params[0] <= 50.0)
    vw_throw(ArgumentErr() << "The --outlier-removal-params percentage must be more than 50.\n");
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

  if (!std::isnan(stereo_settings().nodata_value) && stereo_settings().nodata_value < 0) 
     vw::vw_throw(vw::ArgumentErr() << "The value of nodata must be non-negative.\n");

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
      vw_out() << "Distance between camera centers in meters: "
                << norm_2(cam1_ctr - cam2_ctr) << ".\n";
      
      // Can cameras triangulate to point at something in front of them?
      stereo::StereoModel model(camera_model1.get(), camera_model2.get());
      double error;
      Vector3 point = model(Vector2(), Vector2(), error);
      if (point != Vector3() // triangulation succeeded
          && ((dot_prod(cam1_vec, point - cam1_ctr) < 0) ||
              (dot_prod(cam2_vec, point - cam2_ctr) < 0)   )
          ){
        vw_out(WarningMessage)
          << "Your cameras appear to not to be pointing at the same location! "
          << "A test vector triangulated backwards through "
          << "the camera models. You should double check "
          << "your input models as most likely stereo won't "
          << "be able to triangulate.\n";
      }
      
    } catch (const std::exception& e) {
      // Don't throw an error here. There are legitimate reasons as to
      // why the first checks may fail. For example, the top left pixel
      // might not be valid on a map projected image. But notify the
      // user anyway.
      vw_out(DebugMessage,"asp") << e.what() << std::endl;
    }
  } // end camera checks
  
} // End user_safety_checks

// See if user's request to skip image normalization can be
// satisfied.  This option is a speedup switch which is only meant
// to work with with mapprojected images. It is also not documented.
bool skip_image_normalization(ASPGlobalOptions const& opt){

  if (!stereo_settings().skip_image_normalization) 
    return false;
  
  bool crop_left  = (stereo_settings().left_image_crop_win  != BBox2i(0, 0, 0, 0));
  bool crop_right = (stereo_settings().right_image_crop_win != BBox2i(0, 0, 0, 0));

  // Respect user's choice for skipping the normalization of the input
  // images, if feasible.
  bool is_good = (!crop_left && !crop_right                    &&
                  stereo_settings().alignment_method == "none" &&
                  stereo_settings().cost_mode == 2             &&
                  has_tif_or_ntf_extension(opt.in_file1)       &&
                  has_tif_or_ntf_extension(opt.in_file2));

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

} // end namespace asp
