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
#include <asp/Sessions/RPC/RPCModel.h>

#include <boost/accumulators/accumulators.hpp>
#include <boost/accumulators/statistics.hpp>

using namespace vw;
using namespace vw::cartography;
using namespace std;

namespace asp {

  // Transform the crop window to be in reference to L.tif
  BBox2i transformed_crop_win(Options const& opt){

    BBox2i b = opt.left_image_crop_win;
    DiskImageView<PixelGray<float> > left_image(opt.in_file1);
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
                       vector<Options> & opt_vec){

    // If a stereo program is invoked as:
  
    // prog <images> <cameras> <output-prefix> [<input_dem>] <other options>

    // with the number of images n >= 2, create n-1 individual
    // Options entries, corresponding to n-1 stereo pairs between the
    // first image and each of the subsequent images.

    // First reset the outputs
    output_prefix.clear();
    opt_vec.clear();

    // Extract the images/cameras/output prefix, and perhaps the input DEM
    vector<string> files;
    bool allow_unregistered = true;
    Options opt;
    std::string usage;
    handle_arguments(argc, argv, opt, additional_options,
                     allow_unregistered, files, usage);

    // Store the elements in argv which are not files or output prefix.
    // Note that argv[0] is the program name.
    set<string> file_set; 
    for (int s = 0; s < (int)files.size(); s++)
      file_set.insert(files[s]);
    vector<string> options;
    for (int s = 1; s < argc; s++){
      if (file_set.find(argv[s]) == file_set.end())
        options.push_back(argv[s]);
    }
  
    // Separate the cameras from the other files
    vector<string> cameras = asp::extract_cameras( files );

    if (files.size() < 3)
      vw_throw( ArgumentErr() << "Missing all of the correct input files.\n\n"
                << usage );

    // Find the input DEM, if any
    string input_dem;
    bool has_georef = false;
    try{
      cartography::GeoReference georef;
      has_georef = read_georeference( georef, files.back() );
    }catch(...){}
    if (has_georef){
      input_dem = files.back();
      files.pop_back();
    }else{
      input_dem = "";
    }

    // Find the output prefix
    output_prefix = files.back();
    files.pop_back();

    if (fs::exists(output_prefix))
      vw_out(WarningMessage) << "It appears that the output prefix "
                             << "exists as a file: " << output_prefix
                             << ". Perhaps this was not intended." << endl;
    
    if (files.size() != cameras.size() && !cameras.empty())
      vw_throw( ArgumentErr() << "Expecting the number of images and "
                << "cameras to agree.\n" );
    
    int num_pairs = (int)files.size() - 1;
    if (num_pairs <= 0)
      vw_throw( ArgumentErr() << "Insufficient number of images provided.\n" );

    // Must signal to the children runs that they are part of a multiview run
    if (num_pairs > 1){
      std::string opt_str = "--part-of-multiview-run";
      vector<string>::iterator it = find(options.begin(), options.end(), opt_str);
      if (it == options.end()) options.push_back(opt_str);
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
        << "For multi-view stereo, only alignment method of none or homography "
        << "is supported. Changing alignment method from "
        << stereo_settings().alignment_method << " to " << new_alignment << ".\n";
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
    for (int p = 1; p <= num_pairs; p++){
      
      vector<string> cmd;
      cmd.push_back(prog_name);

      // The command line options go first
      for (int t = 0; t < (int)options.size(); t++)
        cmd.push_back(options[t]);
    
      cmd.push_back(files[0]); // left image
      cmd.push_back(files[p]); // right image

      if (!cameras.empty()){
        cmd.push_back(cameras[0]); // left camera
        cmd.push_back(cameras[p]); // right camera
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

      // Create a local argc and argv for the given stereo pair and
      // parse them.
      int largc = cmd.size();
      vector<char*> largv;
      for (int t = 0; t < largc; t++)
        largv.push_back((char*)cmd[t].c_str());
      Options opt;
      bool allow_unregistered = false;
      vector<string> unregistered;
      handle_arguments( largc, &largv[0], opt, additional_options,
                        allow_unregistered, unregistered, usage);
      opt_vec.push_back(opt);

      if (verbose){
        // Needed for stereo_parse 
        int big = 10000; // helps keep things in order in the python script
        vw_out() << "multiview_command_" << big + p << ",";
        for (int t = 0; t < largc-1; t++) vw_out() << cmd[t] << ",";
        if (largc > 0)                    vw_out() << cmd[largc-1];
        vw_out() << std::endl;
      }
      
    }

    if (num_pairs > 1 && prog_name != "stereo_parse" && prog_name != "stereo_tri")
      vw_throw( ArgumentErr() << "The executable " << prog_name
                << " is not meant to be used directly with more than two images. "
                << "Use instead the stereo/parallel_stereo scripts with "
                << "desired entry points.\n" );
    
    
  }

  // Parse input command line arguments
  void handle_arguments( int argc, char *argv[], Options& opt,
                         boost::program_options::options_description const&
                         additional_options,
                         bool allow_unregistered, vector<string> & unregistered,
                         std::string & usage ){

    po::options_description general_options_sub("");
    general_options_sub.add_options()
      ("session-type,t", po::value(&opt.stereo_session_string), "Select the stereo session type to use for processing. [options: pinhole isis dg rpc]")
      ("stereo-file,s", po::value(&opt.stereo_default_filename)->default_value("./stereo.default"), "Explicitly specify the stereo.default file to use. [default: ./stereo.default]")
      ("left-image-crop-win", po::value(&opt.left_image_crop_win)->default_value(BBox2i(0, 0, 0, 0), "xoff yoff xsize ysize"), "Do stereo in a subregion of the left image [default: use the entire image].");

    // We distinguish between all_general_options, which is all the
    // options we must parse, even if we don't need some of them, and
    // general_options, which are the options specifically used by the
    // current tool, and for which we also print the help message.

    po::options_description general_options("");
    general_options.add ( general_options_sub );
    general_options.add( additional_options );
    general_options.add( asp::BaseOptionsDescription(opt) );

    po::options_description all_general_options("");
    all_general_options.add ( general_options_sub );
    all_general_options.add( generate_config_file_options( opt ) );

    po::options_description positional_options("");
    positional_options.add_options()
      ("left-input-image", po::value(&opt.in_file1), "Left Input Image")
      ("right-input-image", po::value(&opt.in_file2), "Right Input Image")
      ("left-camera-model", po::value(&opt.cam_file1), "Left Camera Model File")
      ("right-camera-model", po::value(&opt.cam_file2), "Right Camera Model File")
      ("output-prefix", po::value(&opt.out_prefix), "Prefix for output filenames")
      ("input-dem", po::value(&opt.input_dem), "Input DEM");

    po::positional_options_description positional_desc;
    positional_desc.add("left-input-image", 1);
    positional_desc.add("right-input-image", 1);
    positional_desc.add("left-camera-model", 1);
    positional_desc.add("right-camera-model", 1);
    positional_desc.add("output-prefix", 1);
    positional_desc.add("input-dem", 1);

    usage = "[options] <images> [<cameras>] <output_file_prefix> [DEM]\n  Extensions are automaticaly added to the output files.\n  Camera model arguments may be optional for some stereo session types (e.g., isis).\n  Stereo parameters should be set in the stereo.default file.";
    po::variables_map vm =
      asp::check_command_line( argc, argv, opt, general_options,
                               all_general_options, positional_options,
                               positional_desc, usage, 
                               allow_unregistered, unregistered );

    // Read the config file
    try {
      po::options_description cfg_options;
      cfg_options.add( positional_options ); // The user can specify the
                                             // positional input from the
                                             // stereo.default if they want
                                             // to.
      cfg_options.add( generate_config_file_options( opt ) );

      // Append the options from the config file. Do not overwrite the
      // options already set on the command line.
      po::store(parse_asp_config_file(allow_unregistered,
                                      opt.stereo_default_filename,
                                      cfg_options), vm);
      po::notify( vm );
    } catch ( po::error const& e ) {
      vw::vw_throw( vw::ArgumentErr() << "Error parsing configuration file:\n"
                    << e.what() << "\n" );
    }
    asp::stereo_settings().validate();

    // Add the options to the usage
    std::ostringstream os;
    os << usage << general_options;
    usage = os.str();
    
    // If we allow unregistered options, the logic below won't apply correctly.
    if (allow_unregistered) return;

    if ( opt.in_file1.empty() || opt.in_file2.empty() || opt.cam_file1.empty()  )
      vw_throw( ArgumentErr() << "Missing all of the correct input files.\n\n"
                << usage );
    
    /// There are 3 valid methods of input into this application
    /// 1.) <image1> <image2> <cam1> <cam2> <prefix> <dem>
    /// 2.) <image1> <image2> <cam1> <cam2> <prefix>
    /// 3.) <image1> <image2> <prefix>

    /// Correcting for Case 3:
    bool this_is_case3 = false;
    if ( opt.out_prefix.empty() && opt.cam_file2.empty() ) {
      opt.out_prefix = opt.cam_file1;
      opt.cam_file1.clear();
      this_is_case3 = true;
    }

    /// Error checking
    if ( opt.out_prefix.empty() )
      vw_throw( ArgumentErr() << "Missing output prefix" );
    if ( opt.in_file1.empty() )
      vw_throw( ArgumentErr() << "Missing left input image" );
    if ( opt.in_file2.empty() )
      vw_throw( ArgumentErr() << "Missing right input image" );
    if ( !this_is_case3 && opt.cam_file1.empty() )
      vw_throw( ArgumentErr() << "Missing left camera file" );
    if ( !this_is_case3 && opt.cam_file2.empty() )
      vw_throw( ArgumentErr() << "Missing right camera file" );

    // Create the output directory 
    asp::create_out_dir(opt.out_prefix);

    // Turn on logging to file
    asp::log_to_file(argc, argv, opt.stereo_default_filename, opt.out_prefix);
    
    // There are two crop win boxes, in respect to original left
    // image, named left_image_crop_win, and in respect to the
    // transformed left image (L.tif), named trans_crop_win. We use
    // the second if available, otherwise we transform and use the
    // first. The box trans_crop_win is for internal use.

    // Interpret the the last two coordinates of the crop win boxes as
    // width and height rather than max_x and max_y.
    BBox2i b = opt.left_image_crop_win;
    opt.left_image_crop_win = BBox2i(b.min().x(), b.min().y(), b.max().x(), b.max().y());
    b = stereo_settings().trans_crop_win;
    stereo_settings().trans_crop_win = BBox2i(b.min().x(), b.min().y(), b.max().x(), b.max().y());

    if (stereo_settings().trans_crop_win == BBox2i(0, 0, 0, 0)){
      stereo_settings().trans_crop_win = transformed_crop_win(opt);
    }

    if ( fs::exists(opt.out_prefix+"-L.tif") ){
      // Intersect with L.tif which is the transformed and processed left image
      DiskImageView<PixelGray<float> > L_img(opt.out_prefix+"-L.tif");
      stereo_settings().trans_crop_win.crop(bounding_box(L_img));
    }

    // Sanity check
    if (stereo_settings().trans_crop_win.width() <= 0  ||
        stereo_settings().trans_crop_win.height() <= 0 ){

      vw_throw( ArgumentErr() << "Invalid region for doing stereo.\n\n"
                << usage << general_options );
    }

    opt.session.reset( asp::StereoSession::create(opt.stereo_session_string,// i/o
                                                  opt, opt.in_file1,
                                                  opt.in_file2,
                                                  opt.cam_file1, opt.cam_file2,
                                                  opt.out_prefix,
                                                  opt.input_dem) );
    user_safety_checks(opt);

    // The last thing we do before we get started is to copy the
    // stereo.default settings over into the results directory so that
    // we have a record of the most recent stereo.default that was used
    // with this data set.
    asp::stereo_settings().write_copy( argc, argv,
                                       opt.stereo_default_filename,
                                       opt.out_prefix + "-stereo.default" );
  }

  // Register Session types
  void stereo_register_sessions() {

#if defined(ASP_HAVE_PKG_ISISIO) && ASP_HAVE_PKG_ISISIO == 1
    // Register the Isis file handler with the Vision Workbench
    // DiskImageResource system.
    DiskImageResource::register_file_type(".cub",
                                          DiskImageResourceIsis::type_static(),
                                          &DiskImageResourceIsis::construct_open,
                                          &DiskImageResourceIsis::construct_create);
#endif
  }

  void user_safety_checks(Options const& opt){
    
    // Error checking

    // Sanity check for max_valid_triangulation_error
    if ( stereo_settings().max_valid_triangulation_error <= 0 ){
      vw_throw( ArgumentErr() << "The maximum valid triangulation error "
                << "must be positive.\n" );
    }
    if ( stereo_settings().max_valid_triangulation_error > 0 ){
      vw_throw( ArgumentErr() << "The --max-valid-triangulation-error was moved "
                << "to point2dem. Alternatively, the point2dem --remove-outliers "
                << "option can be used for automatic detection of maximum "
                << "triangulation error.\n" );
    }
    
    // Seed mode valid values
    if ( stereo_settings().seed_mode > SPARSE_DISP ){
      vw_throw( ArgumentErr() << "Invalid value for seed-mode: "
                << stereo_settings().seed_mode << ".\n" );
    }

    // Local homography needs D_sub
    if ( stereo_settings().seed_mode == NO_SEED &&
         stereo_settings().corr_mode == LOCAL_HOMOGRAPHY ) {
      vw_throw( ArgumentErr() << "Cannot use local homography without "
                << "computing low-resolution disparity.\n");
    }

    // D_sub from DEM needs a positive disparity_estimation_dem_error
    if (stereo_settings().seed_mode == DEM_SEED &&
        stereo_settings().disparity_estimation_dem_error <= 0.0){
      vw_throw( ArgumentErr()
                << "For seed-mode 2, the value of disparity-estimation-dem-error"
                << " must be positive." );
    }
        
    // D_sub from DEM needs a DEM
    if (stereo_settings().seed_mode == DEM_SEED &&
        stereo_settings().disparity_estimation_dem.empty() ){
      vw_throw( ArgumentErr()
                << "For seed-mode 2, an input DEM must be provided.\n" );
    }

    // D_sub from DEM does not work with map-projected images
    if ( !opt.input_dem.empty() && stereo_settings().seed_mode == DEM_SEED )
      vw_throw( NoImplErr() << "Computation of low-resolution disparity from "
                << "DEM is not implemented for map-projected images.\n");

    // Must use map-projected images if input DEM is provided
    GeoReference georef1, georef2;
    bool has_georef1 = read_georeference( georef1, opt.in_file1 );
    bool has_georef2 = read_georeference( georef2, opt.in_file2 );
    if ( !opt.input_dem.empty() && (!has_georef1 || !has_georef2)){
      vw_throw( ArgumentErr() << "The images are not map-projected, "
                << "cannot use the provided DEM: " << opt.input_dem << "\n");
    }

    // If the images are map-projected, they need to use the same
    // projection.
    if ( !opt.input_dem.empty() &&
         georef1.overall_proj4_str() != georef2.overall_proj4_str() ){
      vw_throw( ArgumentErr() << "The left and right images "
                << "must use the same projection.\n");
    }
    
    // If images are map-projected, need an input DEM
    if ( (opt.session->name() == "dg" || opt.session->name() == "dgmaprpc" ) &&
         has_georef1 && has_georef2 && opt.input_dem.empty() ) {
      vw_out(WarningMessage) << "It appears that the input images are "
                             << "map-projected. In that case a DEM needs to be "
                             << "provided for stereo to give correct results.\n";
    }

    // We did not implement stereo using map-projected images with dem
    // on anything except "dg" and "dgmaprpc" sessions.
    if ( !opt.input_dem.empty() && opt.session->name() != "dg"
         && opt.session->name() != "dgmaprpc" ) {
      vw_throw( ArgumentErr() << "Cannot use map-projected images with "
                << "a session of type: " << opt.session->name() << ".\n");
    }

    // No alignment must be set for map-projected images.
    if ( stereo_settings().alignment_method != "none" && !opt.input_dem.empty() ) {
      stereo_settings().alignment_method = "none";
      vw_out(WarningMessage) << "Changing the alignment method to 'none' "
                             << "as the images are map-projected." << endl;
    }
    
    // Ensure that we are not accidentally doing stereo with
    // images map-projected with other camera model than 'rpc'.
    if (!opt.input_dem.empty()){
      
      string cam_tag = "CAMERA_MODEL_TYPE";
      string l_cam_type, r_cam_type;
      boost::shared_ptr<vw::DiskImageResource> l_rsrc
        ( new vw::DiskImageResourceGDAL(opt.in_file1) );
      vw::cartography::read_header_string(*l_rsrc.get(), cam_tag, l_cam_type);
      boost::shared_ptr<vw::DiskImageResource> r_rsrc
        ( new vw::DiskImageResourceGDAL(opt.in_file2) );
      vw::cartography::read_header_string(*r_rsrc.get(), cam_tag, r_cam_type);
        
      if ( (l_cam_type != "" && l_cam_type != "rpc")
           ||
           (r_cam_type != "" && r_cam_type != "rpc")             
           ){
        vw_throw( ArgumentErr()
                  << "The images were map-projected with another option "
                  << "than -t rpc.\n");
      }
    }
    
    if (stereo_settings().corr_kernel[0]%2 == 0 ||
        stereo_settings().corr_kernel[1]%2 == 0 ){
      vw_throw( ArgumentErr()
                << "The entries of corr-kernel must be odd numbers.\n");
    }
    
    if (stereo_settings().subpixel_kernel[0]%2 == 0 ||
        stereo_settings().subpixel_kernel[1]%2 == 0 ){
      vw_throw( ArgumentErr()
                << "The entries of subpixel-kernel must be odd numbers.\n");
    }
    
    // Camera checks
    try {
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
                                 << "   in Lon Lat Rad: " << cartography::xyz_to_lon_lat_radius(cam1_ctr) << "\n";
      VW_OUT(DebugMessage,"asp") << "Camera 2 location: " << cam2_ctr << "\n"
                                 << "   in Lon Lat Rad: " << cartography::xyz_to_lon_lat_radius(cam2_ctr) << "\n";
      VW_OUT(DebugMessage,"asp") << "Camera 1 Pointing Dir: " << cam1_vec << "\n"
                                 << "      dot against pos: " << dot_prod(cam1_vec, cam1_ctr) << "\n";
      VW_OUT(DebugMessage,"asp") << "Camera 2 Pointing Dir: " << cam2_vec << "\n"
                                 << "      dot against pos: " << dot_prod(cam2_vec, cam2_ctr) << "\n";

      // Can cameras triangulate to point at something in front of them?
      stereo::StereoModel model( camera_model1.get(), camera_model2.get() );
      double error;
      Vector3 point = model( Vector2(), Vector2(), error );
      if ( point != Vector3() // triangulation succeeded
           && (
               dot_prod( cam1_vec, point - cam1_ctr ) < 0
               ||
               dot_prod( cam2_vec, point - cam2_ctr ) < 0
               )
           ){
        vw_out(WarningMessage)
          << "Your cameras appear not to be pointing at the same location!\n"
          << "\tA test vector triangulated backwards through\n"
          << "\tthe camera models. You should double check\n"
          << "\tyour input models as most likely stereo won't\n"
          << "\tbe able to triangulate.\n";
      }
    } catch ( const exception& e ) {                
      // Don't throw an error here. There are legitimate reasons as to
      // why this check may fail. For example, the top left pixel
      // might not be valid on a map projected image. But notify the
      // user anyway.
      vw_out(DebugMessage,"asp") << e.what() << endl;
    }
        
  }

  // approximate search range
  //  Find interest points and grow them into a search range
  BBox2i
  approximate_search_range(string const& out_prefix,
                           string const& left_sub_file,
                           string const& right_sub_file,
                           float scale ) {

    typedef PixelGray<float32> PixelT;
    vw_out() << "\t--> Using interest points to determine search window.\n";
    vector<ip::InterestPoint> matched_ip1, matched_ip2;
    float i_scale = 1.0/scale;

    string left_ip_file, right_ip_file;
    ip::ip_filenames(out_prefix, left_sub_file, right_sub_file,
                     left_ip_file, right_ip_file);

    string match_filename
      = ip::match_filename(out_prefix, left_sub_file, right_sub_file);

    // Building / Loading Interest point data
    if ( fs::exists(match_filename) ) {

      vw_out() << "\t    * Using cached match file: " << match_filename << "\n";
      ip::read_binary_match_file(match_filename, matched_ip1, matched_ip2);

    } else {

      vector<ip::InterestPoint> ip1_copy, ip2_copy;

      if ( !fs::exists(left_ip_file) ||
           !fs::exists(right_ip_file) ) {

        // Worst case, no interest point operations have been performed before
        vw_out() << "\t    * Locating Interest Points\n";

        boost::shared_ptr<DiskImageResource>
          left_rsrc( DiskImageResource::open(left_sub_file) ),
          right_rsrc( DiskImageResource::open(right_sub_file) );

        // Read the no-data values written to disk previously when
        // the normalized left and right sub-images were created.
        float left_nodata_value = numeric_limits<float>::quiet_NaN();
        float right_nodata_value = numeric_limits<float>::quiet_NaN();
        if ( left_rsrc->has_nodata_read()  ) left_nodata_value  = left_rsrc->nodata_read();
        if ( right_rsrc->has_nodata_read() ) right_nodata_value = right_rsrc->nodata_read();

        DiskImageView<PixelT> left_sub_image ( left_rsrc );
        DiskImageView<PixelT> right_sub_image( right_rsrc );
        // Interest Point module detector code.
        float ipgain = 0.07;
        list<ip::InterestPoint> ip1, ip2;
        vw_out() << "\t    * Processing for Interest Points.\n";
        while ( ip1.size() < 1500 || ip2.size() < 1500 ) {
          ip1.clear(); ip2.clear();

          ip::OBALoGInterestOperator interest_operator( ipgain );
          ip::IntegralInterestPointDetector<ip::OBALoGInterestOperator> detector( interest_operator, 0 );

          if ( boost::math::isnan(left_nodata_value) )
            ip1 = detect_interest_points( left_sub_image, detector );
          else
            ip1 = detect_interest_points( apply_mask(create_mask_less_or_equal(left_sub_image,left_nodata_value)), detector );

          if ( boost::math::isnan(right_nodata_value) )
            ip2 = detect_interest_points( right_sub_image, detector );
          else
            ip2 = detect_interest_points( apply_mask(create_mask_less_or_equal(right_sub_image,right_nodata_value)), detector );

          if ( !boost::math::isnan(left_nodata_value) )
            remove_ip_near_nodata( left_sub_image, left_nodata_value, ip1 );

          if ( !boost::math::isnan(right_nodata_value) )
            remove_ip_near_nodata( right_sub_image, right_nodata_value, ip2 );

          ipgain *= 0.75;
          if ( ipgain < 1e-2 ) {
            vw_out() << "\t    * Unable to find desirable amount of Interest Points.\n";
            break;
          }
        }

        if ( ip1.size() < 8 || ip2.size() < 8 )
          vw_throw( InputErr() << "Unable to extract interest points from input images ["
                    << left_sub_file << "," << right_sub_file << "]! Unable to continue." );

        // Making sure we don't exceed 3000 points
        if ( ip1.size() > 3000 ) {
          ip1.sort(); ip1.resize(3000);
        }
        if ( ip2.size() > 3000 ) {
          ip2.sort(); ip2.resize(3000);
        }

        // Stripping out orientation. This allows for a better
        // possibility of interest point matches.
        //
        // This is no loss as images at this point are already aligned
        // since the dense correlator is not rotation-invariant.
        BOOST_FOREACH( ip::InterestPoint& ip, ip1 ) ip.orientation = 0;
        BOOST_FOREACH( ip::InterestPoint& ip, ip2 ) ip.orientation = 0;

        vw_out() << "\t    * Building descriptors..." << flush;
        ip::SGradDescriptorGenerator descriptor;
        if ( boost::math::isnan(left_nodata_value) )
          describe_interest_points( left_sub_image, descriptor, ip1 );
        else
          describe_interest_points( apply_mask(create_mask_less_or_equal(left_sub_image,left_nodata_value)), descriptor, ip1 );
        if ( boost::math::isnan(right_nodata_value) )
          describe_interest_points( right_sub_image, descriptor, ip2 );
        else
          describe_interest_points( apply_mask(create_mask_less_or_equal(right_sub_image,right_nodata_value)), descriptor, ip2 );

        vw_out() << "done.\n";

        // Writing out the results
        vw_out() << "\t    * Caching interest points: "
                 << left_ip_file << " & " << right_ip_file << endl;
        ip::write_binary_ip_file( left_ip_file, ip1 );
        ip::write_binary_ip_file( right_ip_file, ip2 );

      }

      vw_out() << "\t    * Using cached IPs.\n";
      ip1_copy = ip::read_binary_ip_file(left_ip_file);
      ip2_copy = ip::read_binary_ip_file(right_ip_file);

      vw_out() << "\t    * Matching interest points\n";
      ip::InterestPointMatcher<ip::L2NormMetric,ip::NullConstraint> matcher(0.6);

      matcher(ip1_copy, ip2_copy, matched_ip1, matched_ip2,
              TerminalProgressCallback( "asp", "\t    Matching: "));
      vw_out(InfoMessage) << "\t    " << matched_ip1.size() << " putative matches.\n";

      vw_out() << "\t    * Rejecting outliers using RANSAC.\n";
      ip::remove_duplicates(matched_ip1, matched_ip2);
      vector<Vector3>
        ransac_ip1 = ip::iplist_to_vectorlist(matched_ip1),
        ransac_ip2 = ip::iplist_to_vectorlist(matched_ip2);
      vector<size_t> indices;

      try {
        // Figure out the inlier threshold .. it should be about 3% of
        // the edge lengths. This is a bit of a magic number, but I'm
        // pulling from experience that an inlier threshold of 30
        // worked best for 1024^2 AMC imagery.
        float inlier_threshold =
          0.0075 * ( sum( file_image_size( left_sub_file ) ) +
                     sum( file_image_size( right_sub_file ) ) );

        math::RandomSampleConsensus<math::HomographyFittingFunctor,math::InterestPointErrorMetric>
          ransac( math::HomographyFittingFunctor(), math::InterestPointErrorMetric(), 100, inlier_threshold, ransac_ip1.size()/2, true );
        Matrix<double> trans = ransac( ransac_ip1, ransac_ip2 );
        vw_out(DebugMessage,"asp") << "\t    * Ransac Result: " << trans << endl;
        vw_out(DebugMessage,"asp") << "\t      inlier thresh: "
                                   << inlier_threshold << " px" << endl;
        indices = ransac.inlier_indices(trans, ransac_ip1, ransac_ip2 );
      } catch ( vw::math::RANSACErr const& e ) {
        vw_out() << "-------------------------------WARNING---------------------------------\n";
        vw_out() << "\t    RANSAC failed! Unable to auto detect search range.\n\n";
        vw_out() << "\t    Please proceed cautiously!\n";
        vw_out() << "-------------------------------WARNING---------------------------------\n";
        return BBox2i(-10,-10,20,20);
      }

      { // Keeping only inliers
        vector<ip::InterestPoint> inlier_ip1, inlier_ip2;
        for ( size_t i = 0; i < indices.size(); i++ ) {
          inlier_ip1.push_back( matched_ip1[indices[i]] );
          inlier_ip2.push_back( matched_ip2[indices[i]] );
        }
        matched_ip1 = inlier_ip1;
        matched_ip2 = inlier_ip2;
      }

      vw_out() << "\t    * Caching matches: " << match_filename << "\n";
      write_binary_match_file( match_filename, matched_ip1, matched_ip2);
    }

    // Find search window based on interest point matches
    namespace ba = boost::accumulators;
    ba::accumulator_set<float, ba::stats<ba::tag::variance> > acc_x, acc_y;
    for (size_t i = 0; i < matched_ip1.size(); i++) {
      acc_x(i_scale * ( matched_ip2[i].x - matched_ip1[i].x ));
      acc_y(i_scale * ( matched_ip2[i].y - matched_ip1[i].y ));
    }
    Vector2f mean( ba::mean(acc_x), ba::mean(acc_y) );
    vw_out(DebugMessage,"asp") << "Mean search is : " << mean << endl;
    Vector2f stddev( sqrt(ba::variance(acc_x)), sqrt(ba::variance(acc_y)) );
    BBox2i search_range( mean - 2.5*stddev,
                         mean + 2.5*stddev );
    return search_range;
  }

  bool is_tif_or_ntf(string const& file){
    return
      boost::iends_with(boost::to_lower_copy(file), ".tif")
      ||
      boost::iends_with(boost::to_lower_copy(file), ".ntf");
  }    
  
  bool skip_image_normalization(Options const& opt ){
    // Respect user's choice for skipping the normalization of the input
    // images, if feasible.
    return
      stereo_settings().skip_image_normalization                    && 
      stereo_settings().alignment_method == "none"                  &&
      stereo_settings().cost_mode == NCC                            &&
      is_tif_or_ntf(opt.in_file1)                                   && 
      is_tif_or_ntf(opt.in_file2);
  }

} // end namespace asp
