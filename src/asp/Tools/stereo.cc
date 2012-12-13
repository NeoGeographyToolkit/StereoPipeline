// __BEGIN_LICENSE__
//  Copyright (c) 2009-2012, United States Government as represented by the
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
#include <asp/Tools/stereo.h>
#include <asp/Sessions/RPC/RPCModel.h>

using namespace vw;

namespace asp {

  // Parse input command line arguments
  void handle_arguments( int argc, char *argv[], Options& opt,
                         boost::program_options::options_description const&
                         additional_options ) {

    // Print the command being run in debug mode.
    std::string run_cmd = "";
    for (int s = 0; s < argc; s++) run_cmd += std::string(argv[s]) + " ";
    VW_OUT(DebugMessage, "stereo") << "\n\n" << run_cmd << "\n";
    
    po::options_description general_options_sub("");
    general_options_sub.add_options()
      ("session-type,t", po::value(&opt.stereo_session_string), "Select the stereo session type to use for processing. [options: pinhole isis dg rpc]")
      ("stereo-file,s", po::value(&opt.stereo_default_filename)->default_value("./stereo.default"), "Explicitly specify the stereo.default file to use. [default: ./stereo.default]")
      ("left-image-crop-win", po::value(&opt.left_image_crop_win)->default_value(BBox2i(0, 0, 0, 0),""), "Do stereo in this region [xoff yoff xsize ysize] of the left image [default: use the entire image].");
      
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
      ("extra_argument1", po::value(&opt.extra_arg1), "Extra Argument 1")
      ("extra_argument2", po::value(&opt.extra_arg2), "Extra Argument 2")
      ("extra_argument3", po::value(&opt.extra_arg3), "Extra Argument 3")
      ("extra_argument4", po::value(&opt.extra_arg4), "Extra Argument 4");

    po::positional_options_description positional_desc;
    positional_desc.add("left-input-image", 1);
    positional_desc.add("right-input-image", 1);
    positional_desc.add("left-camera-model", 1);
    positional_desc.add("right-camera-model", 1);
    positional_desc.add("output-prefix", 1);
    positional_desc.add("extra_argument1", 1);
    positional_desc.add("extra_argument2", 1);
    positional_desc.add("extra_argument3", 1);
    positional_desc.add("extra_argument4", 1);

    std::string usage("[options] <Left_input_image> <Right_input_image> [Left_camera_file] [Right_camera_file] <output_file_prefix> [DEM]\n  Extensions are automaticaly added to the output files.\n  Camera model arguments may be optional for some stereo session types (e.g. isis).\n  Stereo parameters should be set in the stereo.default file.");
    po::variables_map vm =
      asp::check_command_line( argc, argv, opt, general_options, all_general_options,
                               positional_options, positional_desc, usage, false );

    if (!vm.count("left-input-image") || !vm.count("right-input-image") ||
        !vm.count("left-camera-model") )
      vw_throw( ArgumentErr() << "Missing all of the correct input files.\n\n"
                << usage << general_options );

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
      po::store(parse_asp_config_file(opt.stereo_default_filename,
                                      cfg_options), vm);
      po::notify( vm );
    } catch ( po::error const& e ) {
      vw::vw_throw( vw::ArgumentErr() << "Error parsing configuration file:\n"
                    << e.what() << "\n" );
    }
    asp::stereo_settings().validate();

    // If the user hasn't specified a stereo session type, we take a
    // guess here based on the file suffixes.
    if (opt.stereo_session_string.empty())
      guess_session_type(opt);

    // Some specialization here so that the user doesn't need to list
    // camera models on the command line for certain stereo session
    // types.  (e.g. isis).
    //
    // TODO: This modification of arguments should probably happen in
    // initialization and not be dependent on Stereo knowing what
    // session it is in.
    bool check_for_camera_models = true;
    if ( opt.stereo_session_string == "isis" ||
         opt.stereo_session_string == "rpc" ) {
      // Fix the ordering of the arguments if the user only supplies 3
      if (opt.out_prefix.empty()) {
        opt.out_prefix = opt.cam_file1;
        opt.cam_file1.clear();
      }
      check_for_camera_models = false;
    }

    if ( check_for_camera_models &&
         ( opt.out_prefix.empty() || opt.cam_file2.empty() ) )
      vw_throw( ArgumentErr() << "\nMissing output-prefix or right camera model.\n" );

    // Interpret the the last two coordinates of left_image_crop_win as
    // width and height rather than max_x and max_y
    BBox2i b = opt.left_image_crop_win;
    opt.left_image_crop_win = BBox2i(b.min().x(), b.min().y(), b.max().x(), b.max().y());
    // By default, we do stereo in the entire image
    DiskImageView<PixelGray<float> > left_image(opt.in_file1);
    BBox2i full_box = BBox2i(0, 0, left_image.cols(), left_image.rows());
    if (opt.left_image_crop_win == BBox2i(0, 0, 0, 0)){
      opt.left_image_crop_win = full_box;
    }
    // Ensure that the region is inside the maximum theoretical region
    opt.left_image_crop_win.crop(full_box);
    // Sanity check
    if (opt.left_image_crop_win.width() <= 0 || opt.left_image_crop_win.height() <= 0 ){
      vw_throw( ArgumentErr() << "Invalid region for doing stereo.\n\n"
                << usage << general_options );
    }
    
    fs::path out_prefix_path(opt.out_prefix);
    if (out_prefix_path.has_parent_path()) {
      if (!fs::is_directory(out_prefix_path.parent_path())) {
        vw_out() << "\nCreating output directory: "
                 << out_prefix_path.parent_path() << std::endl;
        fs::create_directory(out_prefix_path.parent_path());
      }
    }

    opt.session.reset( asp::StereoSession::create(opt.stereo_session_string) );
    opt.session->initialize(opt, opt.in_file1, opt.in_file2,
                            opt.cam_file1, opt.cam_file2,
                            opt.out_prefix, opt.extra_arg1, opt.extra_arg2,
                            opt.extra_arg3, opt.extra_arg4);

    user_safety_check(opt);

    // The last thing we do before we get started is to copy the
    // stereo.default settings over into the results directory so that
    // we have a record of the most recent stereo.default that was used
    // with this data set.
    asp::stereo_settings().write_copy( argc, argv,
                                       opt.stereo_default_filename,
                                       opt.out_prefix + "-stereo.default" );
  }

  void guess_session_type(Options& opt) {
    if ( asp::has_cam_extension( opt.cam_file1 ) &&
         asp::has_cam_extension( opt.cam_file2 ) ) {
      vw_out() << "\t--> Detected pinhole camera files. "
               << "Executing pinhole stereo pipeline.\n";
      opt.stereo_session_string = "pinhole";
      return;
    }
    if (boost::iends_with(boost::to_lower_copy(opt.in_file1), ".cub") &&
        boost::iends_with(boost::to_lower_copy(opt.in_file2), ".cub")) {
      vw_out() << "\t--> Detected ISIS cube files. "
               << "Executing ISIS stereo pipeline.\n";
      opt.stereo_session_string = "isis";
      return;
    }
    if (boost::iends_with(boost::to_lower_copy(opt.cam_file1), ".xml") &&
        boost::iends_with(boost::to_lower_copy(opt.cam_file2), ".xml")) {
      vw_out() << "\t--> Detected likely Digital Globe XML files. "
               << "Executing DG stereo pipeline.\n";
      opt.stereo_session_string = "dg";
      return;
    }
    try {
      asp::RPCModel left(opt.in_file1), right(opt.in_file2);
      vw_out() << "\t--> Detected RPC Model inside image files. "
               << "Executing RPC stereo pipeline.\n";
      opt.stereo_session_string = "rpc";
      return;
    } catch ( vw::NotFoundErr const& e ) {
      vw_out() << "Error thrown: " << e.what() << std::endl;
    } // If it throws, it wasn't RPC

    // If we get to this point. We couldn't guess the session type
    vw_throw( ArgumentErr() << "Could not determine stereo session type. "
              << "Please set it explicitly.\n"
              << "using the -t switch. Options include: [pinhole isis dg rpc].\n" );
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
    asp::StereoSession::register_session_type( "rpc",  &asp::StereoSessionRPC::construct);
    asp::StereoSession::register_session_type( "rmax", &asp::StereoSessionRmax::construct);
#if defined(ASP_HAVE_PKG_ISISIO) && ASP_HAVE_PKG_ISISIO == 1
    asp::StereoSession::register_session_type( "isis", &asp::StereoSessionIsis::construct);
#endif

  }

  void user_safety_check(Options const& opt){

    if (opt.stereo_session_string == "rpc"){
      // The user safety check does not make sense for RPC cameras as
      // they don't specify a camera center.
      // To do: May need to devise a check specific for RPC cameras.
      return;
    }

    //---------------------------------------------------------
    try {
      boost::shared_ptr<camera::CameraModel> camera_model1, camera_model2;
      opt.session->camera_models(camera_model1,camera_model2);

      // Do the cameras appear to be in the same location?
      if ( norm_2(camera_model1->camera_center(Vector2()) -
                  camera_model2->camera_center(Vector2())) < 1e-3 )
        vw_out(WarningMessage,"console")
          << "Your cameras appear to be in the same location!\n"
          << "\tYou should double check your given camera\n"
          << "\tmodels as most likely stereo won't be able\n"
          << "\tto triangulate or perform epipolar rectification.\n";

      // Developer friendly help
      VW_OUT(DebugMessage,"asp") << "Camera 1 location: " << camera_model1->camera_center(Vector2()) << "\n"
                                 << "   in Lon Lat Rad: " << cartography::xyz_to_lon_lat_radius(camera_model1->camera_center(Vector2())) << "\n";
      VW_OUT(DebugMessage,"asp") << "Camera 2 location: " << camera_model2->camera_center(Vector2()) << "\n"
                                 << "   in Lon Lat Rad: " << cartography::xyz_to_lon_lat_radius(camera_model2->camera_center(Vector2())) << "\n";
      VW_OUT(DebugMessage,"asp") << "Camera 1 Pointing Dir: " << camera_model1->pixel_to_vector(Vector2()) << "\n"
                                 << "      dot against pos: " << dot_prod(camera_model1->pixel_to_vector(Vector2()),
                                                                          camera_model1->camera_center(Vector2())) << "\n";
      VW_OUT(DebugMessage,"asp") << "Camera 2 Pointing Dir: " << camera_model2->pixel_to_vector(Vector2()) << "\n"
                                 << "      dot against pos: " << dot_prod(camera_model2->pixel_to_vector(Vector2()),
                                                                          camera_model2->camera_center(Vector2())) << "\n";

      // Can cameras triangulate to point at something in front of them?
      stereo::StereoModel model( camera_model1.get(), camera_model2.get() );
      double error;
      Vector3 point = model( Vector2(), Vector2(), error );
      if ( point != Vector3() // triangulation succeeded
           && (
               dot_prod( camera_model1->pixel_to_vector(Vector2()),
                         point - camera_model1->camera_center(Vector2()) ) < 0
               ||
               dot_prod( camera_model2->pixel_to_vector(Vector2()),
                         point - camera_model2->camera_center(Vector2()) ) < 0
               )
           ){
        vw_out(WarningMessage,"console")
          << "Your cameras appear not to be pointing at the same location!\n"
          << "\tA test vector triangulated backwards through\n"
          << "\tthe camera models. You should double check\n"
          << "\tyour input models as most likely stereo won't\n"
          << "\tbe able to triangulate.\n";
      }
    } catch ( camera::PixelToRayErr const& e ) {
    } catch ( camera::PointToPixelErr const& e ) {
      // Silent. Top Left pixel might not be valid on a map
      // projected image.
    }

    cartography::GeoReference georef;
    bool has_georef1 = read_georeference( georef, opt.in_file1 );
    bool has_georef2 = read_georeference( georef, opt.in_file2 );
    if (opt.stereo_session_string == "dg" && has_georef1 && has_georef2 && opt.extra_arg1 == "") {
      vw_out(WarningMessage) << "It appears that the input images are map-projected. In that case a DEM needs to be provided for stereo to give correct results.\n";
    }

  }
}
