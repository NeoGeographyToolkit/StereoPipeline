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
///

#include <vw/Cartography.h>
#include <vw/Stereo/StereoView.h>
#include <vw/Stereo/PreFilter.h>
#include <vw/Stereo/CorrelationView.h>
#include <vw/Stereo/CostFunctions.h>
#include <vw/Stereo/DisparityMap.h>
#include <asp/Tools/stereo.h>
#include <asp/Camera/RPCModel.h>
#include <asp/Sessions/StereoSessionFactory.h>
#include <asp/Sessions/ResourceLoader.h>
#include <asp/Core/InterestPointMatching.h>

#include <boost/accumulators/accumulators.hpp>
#include <boost/accumulators/statistics.hpp>

using namespace vw;
using namespace vw::cartography;
using namespace std;

namespace asp {

  // Transform the crop window to be in reference to L.tif
  BBox2i transformed_crop_win(ASPGlobalOptions const& opt){

    BBox2i b = stereo_settings().left_image_crop_win;
    boost::shared_ptr<vw::DiskImageResource> rsrc = 
            load_disk_image_resource(opt.in_file1, opt.cam_file1);
    DiskImageView<PixelGray<float> > left_image(rsrc);
    BBox2i full_box = bounding_box(left_image);
    if (b == BBox2i(0, 0, 0, 0)){

      // No box was provided. Use the full box.
      if ( fs::exists(opt.out_prefix+"-L.tif") ){
        DiskImageView<PixelGray<float> > L_img(opt.out_prefix+"-L.tif");
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
        DiskImageView<PixelGray<float> > L_img(opt.out_prefix+"-L.tif");
        b.crop(bounding_box(L_img));
      }

    }

    return b;
  }

  void parse_multiview(int argc, char* argv[],
                       boost::program_options::options_description const&
                       additional_options,
                       bool verbose,
                       string & output_prefix,
                       vector<ASPGlobalOptions> & opt_vec){

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
    vector<string> files;
    bool is_multiview = true;
    ASPGlobalOptions opt;
    std::string usage;
    handle_arguments(argc, argv, opt, additional_options,
                     is_multiview, files, usage);

    // Need this for the GUI, ensure that opt_vec is never empty, even on failures
    opt_vec.push_back(opt);

    if (files.size() < 3)
      vw_throw( ArgumentErr() << "Missing all of the correct input files.\n\n" << usage );

    // If a file shows up more than once as input, that will confuse
    // the logic at the next step, so forbid that.
    map<string, int> vals;
    for (int s = 1; s < argc; s++)
      vals[argv[s]]++;
    for (int s = 0; s < (int)files.size(); s++){
      if (vals[files[s]] > 1){
        vw_throw( ArgumentErr() << "The following input argument shows up more than "
                  << "once and hence cannot be parsed correctly: "
                  << files[s] << ".\n\n" << usage );
      }
    }

    // Store the options and their values (that is, not the input files).
    set<string> file_set;
    for (int s = 0; s < (int)files.size(); s++)
      file_set.insert(files[s]);
    vector<string> options;
    for (int s = 1; s < argc; s++){
      if (file_set.find(argv[s]) == file_set.end())
        options.push_back(argv[s]);
    }

    // Extract all the positional elements
    vector<string> images, cameras;
    string input_dem;
    if (!parse_multiview_cmd_files(files, images, cameras, output_prefix, input_dem))
      vw_throw( ArgumentErr() << "Missing all of the correct input files.\n\n" << usage );

    if (fs::exists(output_prefix))
      vw_out(WarningMessage)
        << "It appears that the output prefix exists as a file: "
        << output_prefix << ". Perhaps this was not intended." << endl;

    // Verify that the images and cameras exist, otherwise GDAL prints funny messages later.
    for (int i = 0; i < (int)images.size(); i++){
      if (!fs::exists(images[i]))
        vw_throw(ArgumentErr() << "Cannot find the image file: " << images[i] << ".\n");
    }
    for (int i = 0; i < (int)cameras.size(); i++){
      if (!fs::exists(cameras[i]))
        vw_throw(ArgumentErr() << "Cannot find the camera file: " << cameras[i] << ".\n");
    }

    if (images.size() != cameras.size() && !cameras.empty())
      vw_throw(ArgumentErr() << "Expecting the number of images and cameras to agree.\n");

    int num_pairs = (int)images.size() - 1;
    if (num_pairs <= 0)
      vw_throw(ArgumentErr() << "Insufficient number of images provided.\n");

    // Must signal to the children runs that they are part of a multiview run
    if (num_pairs > 1){
      std::string opt_str = "--part-of-multiview-run";
      vector<string>::iterator it = find(options.begin(), options.end(), opt_str);
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
      vector<string>::iterator it = find(options.begin(), options.end(), align_opt);
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

      vector<string> cmd;
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

      string local_prefix = output_prefix;
      if (num_pairs > 1){
        // Need to have a separate output prefix for each pair
        ostringstream os;
        os << local_prefix << "-pair" << p << "/" << p;
        local_prefix = os.str();
      }
      cmd.push_back(local_prefix);

      if (!input_dem.empty())
        cmd.push_back(input_dem);

      // Create a local argc and argv for the given stereo pair and parse them.
      int largc = cmd.size();
      vector<char*> largv;
      for (int t = 0; t < largc; t++)
        largv.push_back((char*)cmd[t].c_str());
      ASPGlobalOptions opt;
      bool is_multiview = false;
      vector<string> files;
      handle_arguments( largc, &largv[0], opt, additional_options,
                        is_multiview, files, usage);
      opt_vec[p-1] = opt;

      if (verbose){
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
      vw_throw( ArgumentErr() << "The executable " << prog_name
                << " is not meant to be used directly with more than two images. "
                << "Use instead the stereo/parallel_stereo scripts with desired entry points.\n" );

  }

  // Parse input command line arguments
  void handle_arguments( int argc, char *argv[], ASPGlobalOptions& opt,
                         boost::program_options::options_description const&
                         additional_options,
                         bool is_multiview, vector<string> & input_files,
                         std::string & usage ){

    po::options_description general_options_sub("");
    general_options_sub.add_options()
      ("session-type,t",      po::value(&opt.stereo_session_string),
                              "Select the stereo session type to use for processing. [options: pinhole isis dg rpc spot5 aster pinholemappinhole isismapisis dgmaprpc rpcmaprpc astermaprpc spot5maprpc]")
      ("stereo-file,s",       po::value(&opt.stereo_default_filename)->default_value("./stereo.default"),
       "Explicitly specify the stereo.default file to use. [default: ./stereo.default]");


    // We distinguish between all_general_options, which is all the
    // options we must parse, even if we don't need some of them, and
    // general_options, which are the options specifically used by the
    // current tool, and for which we also print the help message.

    po::options_description general_options("");
    general_options.add(general_options_sub);
    general_options.add(additional_options);
    general_options.add(vw::cartography::GdalWriteOptionsDescription(opt));

    po::options_description all_general_options("");
    all_general_options.add(general_options_sub );
    all_general_options.add(generate_config_file_options(opt));

    po::options_description positional_options("");
    po::positional_options_description positional_desc;
    if (is_multiview){
      // The number of input files could be huge. Just store them in a vector,
      // we'll parse them in the caller.
      positional_options.add_options()
        ("input-files", po::value< std::vector<std::string> >(), "Input files");
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

    usage = "[options] <images> [<cameras>] <output_file_prefix> [DEM]\n  Extensions are automaticaly added to the output files.\n  Camera model arguments may be optional for some stereo session types (e.g., isis).\n  Stereo parameters should be set in the stereo.default file.";
    bool allow_unregistered = false;
    std::vector<std::string> unregistered;
    po::variables_map vm = asp::check_command_line(argc, argv, opt, general_options,
                                                   all_general_options, positional_options,
                                                   positional_desc, usage,
                                                   allow_unregistered, unregistered);

    // Read the config file
    try {
      po::options_description cfg_options;
      cfg_options.add(positional_options); // The user can specify the
                                             // positional input from the
                                             // stereo.default if they want
                                             // to.
      cfg_options.add(generate_config_file_options(opt));

      // Append the options from the config file. Do not overwrite the
      // options already set on the command line.
      bool print_warnings = is_multiview; // print warnings just first time
      po::store(parse_asp_config_file(print_warnings,
                                      opt.stereo_default_filename,
                                      cfg_options), vm);
      po::notify(vm);
    } catch (po::error const& e) {
      vw::vw_throw(vw::ArgumentErr() << "Error parsing configuration file:\n" << e.what() << "\n");
    }
    asp::stereo_settings().validate();

    // Add the options to the usage
    std::ostringstream os;
    os << usage << general_options;
    usage = os.str();

    // For multiview, just store the files and return
    if (is_multiview){
      if (vm.count("input-files") == 0)
        vw_throw(ArgumentErr() << "Missing input arguments.\n" << usage );
      input_files = vm["input-files"].as< std::vector<std::string> >();
      return;
    }

    if (opt.in_file1.empty() || opt.in_file2.empty() || opt.cam_file1.empty())
      vw_throw( ArgumentErr() << "Missing all of the correct input files.\n\n" << usage );

    // There are 3 valid methods of input into this application
    // 1.) <image1> <image2> <cam1> <cam2> <prefix> <dem>
    // 2.) <image1> <image2> <cam1> <cam2> <prefix>
    // 3.) <image1> <image2> <prefix>

    // Correcting for Case 3:
    bool this_is_case3 = false;
    if (opt.out_prefix.empty() && opt.cam_file2.empty()) {
      opt.out_prefix = opt.cam_file1;
      opt.cam_file1.clear();
      this_is_case3 = true;
    }

    // Error checking
    if (opt.out_prefix.empty())                  vw_throw(ArgumentErr() << "Missing output prefix"    );
    if (opt.in_file1.empty())                    vw_throw(ArgumentErr() << "Missing left input image" );
    if (opt.in_file2.empty())                    vw_throw(ArgumentErr() << "Missing right input image");
    if (!this_is_case3 && opt.cam_file1.empty()) vw_throw(ArgumentErr() << "Missing left camera file" );
    if (!this_is_case3 && opt.cam_file2.empty()) vw_throw(ArgumentErr() << "Missing right camera file");

    // Create the output directory
    vw::create_out_dir(opt.out_prefix);

    // Turn on logging to file
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
    stereo_settings().left_image_crop_win  = BBox2i(bl.min().x(), bl.min().y(), bl.max().x(), bl.max().y());
    stereo_settings().right_image_crop_win = BBox2i(br.min().x(), br.min().y(), br.max().x(), br.max().y());
    stereo_settings().trans_crop_win       = BBox2i(bt.min().x(), bt.min().y(), bt.max().x(), bt.max().y());

    // Ensure the crop windows are always contained in the images.
    boost::shared_ptr<vw::DiskImageResource> left_resource, right_resource;
    left_resource  = asp::load_disk_image_resource(opt.in_file1, opt.cam_file1);
    right_resource = asp::load_disk_image_resource(opt.in_file2, opt.cam_file2);
    DiskImageView<float> left_image(left_resource);
    DiskImageView<float> right_image(right_resource);
    stereo_settings().left_image_crop_win.crop(bounding_box(left_image));
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
        DiskImageView<PixelGray<float> > L_img(opt.out_prefix+"-L.tif");
        stereo_settings().trans_crop_win.crop(bounding_box(L_img));
      }
    }else{ 
      // If left_image_crop_win is specified, as can be see in
      // StereoSession::pre_preprocessing_hook(), we actually
      // physically crop the image.  The trans_crop_win as passed 
      // here from parallel_stereo will already be a tile in the 
      // cropped image. So we just use it as it is. If it is not defined, 
      // we set it to the entire cropped image.
      if (stereo_settings().trans_crop_win == BBox2i(0, 0, 0, 0)) {
        stereo_settings().trans_crop_win = bounding_box(left_image);
        if ( fs::exists(opt.out_prefix+"-L.tif") ){
          DiskImageView<PixelGray<float> > L_img(opt.out_prefix+"-L.tif");
          stereo_settings().trans_crop_win = bounding_box(L_img);
        }
      }
    } // End crop checking case

    // TODO: May need to update this check for individual crop cases.
    // Sanity check. Don't run it if we have L-cropped.tif or R-cropped.tif,
    // in that case we have ran the gui before, and the sizes of the subimages
    // could be anything. We'll regenerate any of those anyway soon.
    if ((stereo_settings().trans_crop_win.width () <= 0 ||
         stereo_settings().trans_crop_win.height() <= 0) &&
        !fs::exists(opt.out_prefix+"-L-cropped.tif")     &&
        !fs::exists(opt.out_prefix+"-R-cropped.tif") ){
      vw_throw(ArgumentErr() << "Invalid region for doing stereo.\n\n" << usage << general_options );
    }

    // Verify that there is only one channel per input image
    if ( (left_resource->channels() > 1) || (right_resource->channels() > 1) )
      vw_throw(ArgumentErr() << "Error: Input images can only have a single channel!\n\n" << usage << general_options );

    // The StereoSession call automatically determines the type of object to create from the input parameters.
    opt.session.reset(asp::StereoSessionFactory::create(opt.stereo_session_string, opt,// i/o
                                                        opt.in_file1,   opt.in_file2,
                                                        opt.cam_file1,  opt.cam_file2,
                                                        opt.out_prefix, opt.input_dem));
    // Run a set of checks to make sure the settings are compatible
    // - Since we already created the session, any errors are fatal.
    user_safety_checks(opt);

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

#if defined(ASP_HAVE_PKG_ISISIO) && ASP_HAVE_PKG_ISISIO == 1
    // Register the Isis file handler with the Vision Workbench DiskImageResource system.
    DiskImageResource::register_file_type(".cub",
                                          DiskImageResourceIsis::type_static(),
                                          &DiskImageResourceIsis::construct_open,
                                          &DiskImageResourceIsis::construct_create);
#endif
  }

  void user_safety_checks(ASPGlobalOptions const& opt){

    // Error checking

    const bool dem_provided = !opt.input_dem.empty();

    // Seed mode valid values
    if (stereo_settings().seed_mode > 3){
      vw_throw(ArgumentErr() << "Invalid value for seed-mode: " << stereo_settings().seed_mode << ".\n");
    }

    // Local homography needs D_sub
    if (stereo_settings().seed_mode == 0 &&
        stereo_settings().use_local_homography){
      vw_throw( ArgumentErr() << "Cannot use local homography without computing low-resolution disparity.\n");
    }

    // D_sub from DEM needs a positive disparity_estimation_dem_error
    if (stereo_settings().seed_mode == 2 &&
        stereo_settings().disparity_estimation_dem_error <= 0.0){
      vw_throw( ArgumentErr() << "For seed-mode 2, the value of disparity-estimation-dem-error must be positive." );
    }

    // D_sub from DEM needs a DEM
    if (stereo_settings().seed_mode == 2 &&
        stereo_settings().disparity_estimation_dem.empty()){
      vw_throw( ArgumentErr() << "For seed-mode 2, an input DEM must be provided.\n" );
    }

    // D_sub from DEM does not work with map-projected images
    if (dem_provided && stereo_settings().seed_mode == 2)
      vw_throw( NoImplErr() << "Computation of low-resolution disparity from "
                << "DEM is not implemented for map-projected images.\n");

    // Must use map-projected images if input DEM is provided
    GeoReference georef1, georef2;
    bool has_georef1 = asp::read_georeference_asp(georef1, opt.in_file1);
    bool has_georef2 = asp::read_georeference_asp(georef2, opt.in_file2);
    if (dem_provided && (!has_georef1 || !has_georef2)){
      vw_throw( ArgumentErr() << "The images are not map-projected, "
                << "cannot use the provided DEM: " << opt.input_dem << "\n");
    }

    // If the images are map-projected, they need to use the same projection.
    if (dem_provided &&
        georef1.overall_proj4_str() != georef2.overall_proj4_str()){
      vw_throw( ArgumentErr() << "The left and right images must use the same projection.\n");
    }

    //TODO: Clean up these conditional using some kind of enum system

    // If the images are map-projected, and the cameras are specified
    // separately from the images, we need an input DEM, as we use the
    // ASP flow with map-projected images.
    if (has_georef1 && has_georef2 && !dem_provided &&
        (opt.cam_file1 != opt.in_file1) && (opt.cam_file2 != opt.in_file2) &&
        !opt.cam_file1.empty() && !opt.cam_file2.empty() ) {
        
      vw_out() << "Georef 1: " << georef1 << std::endl;
      vw_out() << "Georef 2: " << georef1 << std::endl;
      
      vw_out(WarningMessage) << "It appears that the input images are "
                             << "map-projected. In that case a DEM needs to be "
                             << "provided for stereo to give correct results.\n";
    }

    // Check that if the user provided a dem that we are using a map projection method
    if (dem_provided && !opt.session->uses_map_projected_inputs()) {
      vw_throw(ArgumentErr() << "Cannot use map-projected images with a session of type: "
                             << opt.session->name() << ".\n");
    }

    // No alignment must be set for map-projected images.
    if (stereo_settings().alignment_method != "none" && dem_provided) {
        stereo_settings().alignment_method  = "none";
      vw_out(WarningMessage) << "Changing the alignment method to 'none' "
                             << "as the images are map-projected." << endl;
    }

    // Ensure that for dgmaprpc and rpcmaprpc sessions the images were
    // map-projected using -t rpc. For isismapisis it should have been
    // isis. Same for pinhole.
    if (dem_provided){

      string cam_tag = "CAMERA_MODEL_TYPE";
      string l_cam_type, r_cam_type;
      boost::shared_ptr<vw::DiskImageResource> l_rsrc(new vw::DiskImageResourceGDAL(opt.in_file1));
      vw::cartography::read_header_string(*l_rsrc.get(), cam_tag, l_cam_type);
      boost::shared_ptr<vw::DiskImageResource> r_rsrc(new vw::DiskImageResourceGDAL(opt.in_file2));
      vw::cartography::read_header_string(*r_rsrc.get(), cam_tag, r_cam_type);

      // Extract the 'rpc' from 'rpcmaprpc' and 'dgmaprc', and 'pinhole' from 'pinholemappinhole'
      std::string expected_cam_type;
      std::string sep = "map";
      std::size_t it = opt.session->name().find(sep);
      if (it != std::string::npos) {
        it += sep.size();
        expected_cam_type = opt.session->name().substr(it, opt.session->name().size());
      }

      if ((l_cam_type != "" && l_cam_type != expected_cam_type) ||
          (r_cam_type != "" && r_cam_type != expected_cam_type)   ){
        vw_throw(ArgumentErr() << "For session type "
                 << opt.session->name()
                 << ", the images should have been map-projected with "
                 << "the option -t \"" << expected_cam_type << "\". Instead, got: \""
                 << l_cam_type << "\" and \"" << r_cam_type << "\".\n");
      }

    } // End if dem_provided

    if (stereo_settings().corr_kernel[0]%2 == 0 ||
        stereo_settings().corr_kernel[1]%2 == 0   ){
      vw_throw(ArgumentErr() << "The entries of corr-kernel must be odd numbers.\n");
    }

    if (stereo_settings().subpixel_kernel[0]%2 == 0 ||
        stereo_settings().subpixel_kernel[1]%2 == 0   ){
      vw_throw(ArgumentErr() << "The entries of subpixel-kernel must be odd numbers.\n");
    }

    // Check some SGM related settings.
    if (stereo_settings().stereo_algorithm > vw::stereo::CORRELATION_WINDOW) {
      if (stereo_settings().subpixel_mode != 0)
        vw_out(WarningMessage) << "SGM mode detected, setting subpixel mode to the required value of zero.\n";
    }
    else { // Non-SGM mode
      if (stereo_settings().cost_mode == 3)
        vw_throw( ArgumentErr() << "Cannot use census transform without SGM!\n" );
      if (stereo_settings().cost_mode == 4)
        vw_throw( ArgumentErr() << "Cannot use ternary census transform without SGM!\n" );
    }
    if (stereo_settings().cost_mode > 4)
      vw_throw( ArgumentErr() << "Unknown value " << stereo_settings().cost_mode << " for cost-mode.\n" );

    // Camera checks
    bool force_throw = false;
    try {
      // TODO: Remove this extra camera load!
      //       - Some camera models take a long time to load and this causes us to load them twice!
      boost::shared_ptr<camera::CameraModel> camera_model1, camera_model2;
      opt.session->camera_models(camera_model1, camera_model2);

      Vector3 cam1_ctr = camera_model1->camera_center(Vector2());
      Vector3 cam2_ctr = camera_model2->camera_center(Vector2());
      Vector3 cam1_vec = camera_model1->pixel_to_vector(Vector2());
      Vector3 cam2_vec = camera_model2->pixel_to_vector(Vector2());
      // Do the cameras appear to be in the same location?
      if ( norm_2(cam1_ctr - cam2_ctr) < 1e-3 )
        vw_out(WarningMessage)
          << "Your cameras appear to be in the same location!\n"
          << "\tYou should double check your given camera\n"
          << "\tmodels as most likely stereo won't be able\n"
          << "\tto triangulate or perform epipolar rectification.\n";

      // Developer friendly help
      VW_OUT(DebugMessage,"asp") << "Camera 1 location: " << cam1_ctr << "\n"
             << "   in estimated Lon Lat Rad: " << cartography::xyz_to_lon_lat_radius_estimate(cam1_ctr) << "\n";
      VW_OUT(DebugMessage,"asp") << "Camera 2 location: " << cam2_ctr << "\n"
             << "   in estimated Lon Lat Rad: " << cartography::xyz_to_lon_lat_radius_estimate(cam2_ctr) << "\n";
      VW_OUT(DebugMessage,"asp") << "Camera 1 Pointing Dir: " << cam1_vec << "\n"
                                 << "      dot against pos: " << dot_prod(cam1_vec, cam1_ctr) << "\n";
      VW_OUT(DebugMessage,"asp") << "Camera 2 Pointing Dir: " << cam2_vec << "\n"
                                 << "      dot against pos: " << dot_prod(cam2_vec, cam2_ctr) << "\n";

      // Can cameras triangulate to point at something in front of them?
      stereo::StereoModel model(camera_model1.get(), camera_model2.get());
      double error;
      Vector3 point = model(Vector2(), Vector2(), error);
      if (point != Vector3() // triangulation succeeded
          && ((dot_prod(cam1_vec, point - cam1_ctr) < 0) ||
              (dot_prod(cam2_vec, point - cam2_ctr) < 0)   )
          ){
        vw_out(WarningMessage)
          << "Your cameras appear not to be pointing at the same location!\n"
          << "\tA test vector triangulated backwards through\n"
          << "\tthe camera models. You should double check\n"
          << "\tyour input models as most likely stereo won't\n"
          << "\tbe able to triangulate.\n";
      }

      // If later we perform piecewise adjustments, the cameras loaded
      // so far must not be adjusted. And we also can't just perform
      // stereo on cropped images, as we need the full disparity.
      if (stereo_settings().image_lines_per_piecewise_adjustment > 0) {

        force_throw = true;

        // This check must come first as it implies adjusted cameras
        if ( ( stereo_settings().left_image_crop_win  != BBox2i(0, 0, 0, 0)) &&
             ( stereo_settings().right_image_crop_win != BBox2i(0, 0, 0, 0) ) )
          vw_throw(ArgumentErr() << "Since we perform piecewise adjustments we "
                   << "need the full disparities, so --left-image-crop-win and  "
                   << "--right-image-crop-win cannot be used.\n");

        if (stereo_settings().piecewise_adjustment_interp_type != 1 &&
            stereo_settings().piecewise_adjustment_interp_type != 2)
          vw_throw(ArgumentErr() << "Interpolation type for piecewise "
                   << "adjustment can be only 1 or 2.\n");

      }

    } catch (const exception& e) {
      // Don't throw an error here. There are legitimate reasons as to
      // why the first checks may fail. For example, the top left pixel
      // might not be valid on a map projected image. But notify the
      // user anyway. Make an exception for the piecewise adjustment checks.
      if (!force_throw)
        vw_out(DebugMessage,"asp") << e.what() << endl;
      else
        vw_throw( ArgumentErr() << e.what() );
    }
  } // End user_safety_checks


// TODO: Move this histogram code!  Merge with image histogram code!


  /// Compute simple histogram from a vector of data
  void histogram( std::vector<double> const& values, int num_bins, double min_val, double max_val,
                  std::vector<double> &hist, std::vector<double> &bin_centers){
    
    VW_ASSERT(num_bins > 0, ArgumentErr() << "histogram: number of input bins must be positive");
    
    // TODO: Verify max/min values!
    
    // Populate the list of bin centers
    // The min and max vals represent the outer limits of the available bins.
    const double range     = max_val - min_val;
    const double bin_width = range / num_bins;
    bin_centers.resize(num_bins);
    for (int i=0; i<num_bins; ++i)
      bin_centers[i] = min_val + i*bin_width + bin_width/2.0;
    
    hist.assign(num_bins, 0.0);
    for (size_t i = 0; i < values.size(); ++i){
      double val = values[i];
      int bin = (int)round( (num_bins - 1) * ((val - min_val)/range) );
      
      // Saturate the bin assignment to prevent a memory exception.
      if (bin < 0)
        bin = 0;
      if (bin > num_bins-1)
        bin = num_bins-1;
        
      ++(hist[bin]);
    }
    return;
  }

  /// Returns the histogram bin index corresponding to the specified percentile
  inline size_t get_histogram_percentile(std::vector<double> const& hist, double percentile) {
  
    // Verify the input percentile is in the legal range
    if ((percentile < 0) || (percentile > 1.0)) {
      vw_throw(ArgumentErr() << "get_histogram_percentile: illegal percentile request: " << percentile << "\n");
    }

    // Get the total pixel count
    const size_t num_bins = hist.size();
    double num_pixels = 0;
    for (size_t i=0; i<num_bins; ++i)
      num_pixels += hist[i];

    // Now go through and find the requested percentile
    double running_percentile = 0;
    for (size_t i=0; i<num_bins; ++i) {
      double this_percent = hist[i] / num_pixels;
      running_percentile += this_percent;
      //std::cout << "p - " << running_percentile << std::endl;
      if (running_percentile >= percentile)
        return i;
    }
    vw_throw(LogicErr() << "get_histogram_percentile: Illegal histogram encountered!");
  }





  ///  Find interest points and grow them into a search range
  BBox2i
  approximate_search_range(std::string const& out_prefix,
                           std::string const& left_sub_file,
                           std::string const& right_sub_file,
                           float scale,
                           int inlier_threshold) {

    typedef PixelGray<float32> PixelT;
    vw_out() << "\t--> Using interest points to determine search window.\n";
    vector<ip::InterestPoint> matched_ip1, matched_ip2;
    float i_scale = 1.0/scale;

    string left_ip_file, right_ip_file;
    ip::ip_filenames(out_prefix, left_sub_file, right_sub_file, left_ip_file, right_ip_file);

    string match_filename = ip::match_filename(out_prefix, left_sub_file, right_sub_file);

    bool crop_applied = (stereo_settings().left_image_crop_win != BBox2i(0, 0, 0, 0));

    // Building / Loading Interest point data
    if (!fs::exists(match_filename) || crop_applied) {

      // No interest point operations have been performed before
      vw_out() << "\t    * Locating Interest Points\n";

      boost::shared_ptr<DiskImageResource>
        left_rsrc (DiskImageResource::open(left_sub_file )),
        right_rsrc(DiskImageResource::open(right_sub_file));

      // Read the no-data values written to disk previously when
      // the normalized left and right sub-images were created.
      float left_nodata_value  = numeric_limits<float>::quiet_NaN();
      float right_nodata_value = numeric_limits<float>::quiet_NaN();
      if (left_rsrc->has_nodata_read ()) left_nodata_value  = left_rsrc->nodata_read();
      if (right_rsrc->has_nodata_read()) right_nodata_value = right_rsrc->nodata_read();

      DiskImageView<PixelT> left_sub_image (left_rsrc );
      DiskImageView<PixelT> right_sub_image(right_rsrc);

      // This will write match_filename to disk.
      bool success = asp::homography_ip_matching(left_sub_image, right_sub_image,
                                                 stereo_settings().ip_per_tile,
                                                 match_filename,
                                                 inlier_threshold,
                                                 left_nodata_value, right_nodata_value);
      if (!success)
        vw_throw(ArgumentErr() << "Could not find interest points.\n");
    }

    vw_out() << "\t    * Using cached match file: " << match_filename << "\n";
    ip::read_binary_match_file(match_filename, matched_ip1, matched_ip2);

    // Find search window based on interest point matches

    // Record the disparities for each point pair
    const size_t num_ip = matched_ip1.size();
    std::vector<double> dx(num_ip), dy(num_ip);
    double min_dx, min_dy, max_dx, max_dy;
    min_dx = min_dy = std::numeric_limits<double>::max();
    max_dx = max_dy = std::numeric_limits<double>::min();
    for (size_t i = 0; i < num_ip; i++) {
      dx[i] = i_scale * (matched_ip2[i].x - matched_ip1[i].x);
      dy[i] = i_scale * (matched_ip2[i].y - matched_ip1[i].y);
      if (dx[i] < min_dx) min_dx = dx[i];  // Could be smarter about how this is done.
      if (dy[i] < min_dy) min_dy = dy[i];
      if (dx[i] > max_dx) max_dx = dx[i];
      if (dy[i] > max_dy) max_dy = dy[i];
    }
    
    // Compute histograms
    const int NUM_BINS = 2000; // Accuracy is important with scaled pixels
    std::vector<double> hist_x, centers_x, hist_y, centers_y;
    histogram(dx, NUM_BINS, min_dx, max_dx, hist_x, centers_x);
    histogram(dy, NUM_BINS, min_dy, max_dy, hist_y, centers_y);
    
    //printf("min x,y = %lf, %lf, max x,y = %lf, %lf\n", min_dx, min_dy, max_dx, max_dy);
    
    // Compute search ranges
    const double MAX_PERCENTILE = 0.95;
    const double MIN_PERCENTILE = 0.05;
    double search_scale = 2.0;
    size_t min_bin_x = get_histogram_percentile(hist_x, MIN_PERCENTILE);
    size_t min_bin_y = get_histogram_percentile(hist_y, MIN_PERCENTILE);
    size_t max_bin_x = get_histogram_percentile(hist_x, MAX_PERCENTILE);
    size_t max_bin_y = get_histogram_percentile(hist_y, MAX_PERCENTILE);
    Vector2 search_min(centers_x[min_bin_x],
                       centers_y[min_bin_y]);
    Vector2 search_max(centers_x[max_bin_x],
                       centers_y[max_bin_y]);
    //std::cout << "Unscaled search range = " << BBox2i(search_min, search_max) << std::endl;
    Vector2 search_center = (search_max + search_min) / 2.0;
    //std::cout << "search_center = " << search_center << std::endl;
    Vector2 d_min = search_min - search_center; // TODO: Make into a bbox function!
    Vector2 d_max = search_max - search_center;
    //std::cout << "d_min = " << d_min << std::endl;
    //std::cout << "d_max = " << d_max << std::endl;
    search_min = d_min*search_scale + search_center;
    search_max = d_max*search_scale + search_center;
    //std::cout << "Scaled search range = " << BBox2i(search_min, search_max) << std::endl;
    
/*
     // Debug code to print all the points
    for (size_t i = 0; i < matched_ip1.size(); i++) {
      Vector2f diff(i_scale * (matched_ip2[i].x - matched_ip1[i].x), 
                    i_scale * (matched_ip2[i].y - matched_ip1[i].y));
      //Vector2f diff(matched_ip2[i].x - matched_ip1[i].x, 
      //              matched_ip2[i].y - matched_ip1[i].y);
      vw_out(InfoMessage,"asp") << matched_ip1[i].x <<", "<<matched_ip1[i].y 
                << " <> " 
                << matched_ip2[i].x <<", "<<matched_ip2[i].y 
                 << " DIFF " << diff << " DIFF-M " << diff-mean << endl;
      //std::cout << diff[0] << ", " << diff[1] << endl;
    }
*/
    
    vw_out(InfoMessage,"asp") << "i_scale is : "       << i_scale << endl;
    BBox2i search_range(search_min, search_max);
    return search_range;
  } // End function approximate_search_range

  bool skip_image_normalization(ASPGlobalOptions const& opt ){

    bool crop_left  = ( stereo_settings().left_image_crop_win  != BBox2i(0, 0, 0, 0));
    bool crop_right = ( stereo_settings().right_image_crop_win != BBox2i(0, 0, 0, 0));

    // Respect user's choice for skipping the normalization of the input
    // images, if feasible.
    return(!crop_left && !crop_right                    &&
           stereo_settings().skip_image_normalization   &&
           stereo_settings().alignment_method == "none" &&
           stereo_settings().cost_mode == 2             &&
           has_tif_or_ntf_extension(opt.in_file1)       &&
           has_tif_or_ntf_extension(opt.in_file2));
  } // End function skip_image_normalization

} // end namespace asp
