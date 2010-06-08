// __BEGIN_LICENSE__
// Copyright (C) 2006-2010 United States Government as represented by
// the Administrator of the National Aeronautics and Space Administration.
// All Rights Reserved.
// __END_LICENSE__


/// \file stereo.cc
///
//#define USE_GRAPHICS

#include <asp/Tools/stereo.h>
#include <asp/Tools/stereo_preprocessing.h>
#include <asp/Tools/stereo_correlation.h>
#include <asp/Tools/stereo_refinement.h>
#include <asp/Tools/stereo_filtering.h>
#include <asp/Tools/stereo_triangulation.h>

using namespace vw;

namespace vw {
  template<> struct PixelFormatID<PixelMask<Vector<float, 5> > >   { static const PixelFormatEnum value = VW_PIXEL_GENERIC_6_CHANNEL; };
}

namespace vw {
  template<> struct PixelFormatID<Vector<float, 3> >   { static const PixelFormatEnum value = VW_PIXEL_GENERIC_3_CHANNEL; };
}

void handle_arguments( int argc, char *argv[], Options& opt ) {
  po::options_description general_options("");
  general_options.add_options()
    ("threads", po::value(&opt.num_threads)->default_value(0), "Select the number of processors (threads) to use.")
    ("session-type,t", po::value(&opt.stereo_session_string), "Select the stereo session type to use for processing. [options: pinhole isis]")
    ("stereo-file,s", po::value(&opt.stereo_default_filename)->default_value("./stereo.default"), "Explicitly specify the stereo.default file to use. [default: ./stereo.default]")
    ("entry-point,e", po::value(&opt.entry_point)->default_value(0), "Pipeline Entry Point (an integer from 1-4)")
    ("draft-mode", po::value(&opt.corr_debug_prefix), "Cause the pyramid correlator to save out debug imagery named with this prefix.")
    ("optimized-correlator", "Use the optimized correlator instead of the pyramid correlator.")
    ("help,h", "Display this help message");

  po::options_description positional("");
  positional.add_options()
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

  po::options_description all_options;
  all_options.add(general_options).add(positional);

  po::variables_map vm;
  try {
    po::store( po::command_line_parser( argc, argv ).options(all_options).positional(positional_desc).run(), vm );
    po::notify( vm );
  } catch (po::error &e) {
    vw_throw( ArgumentErr() << "Error parsing input:\n\t"
              << e.what() << general_options );
  }

  std::ostringstream usage;
  usage << "Usage: " << argv[0] << " [options] <Left_input_image> <Right_input_image> [Left_camera_file] [Right_camera_file] <output_file_prefix>\n"
        << "  Extensions are automaticaly added to the output files.\n"
        << "  Camera model arguments may be optional for some stereo session types (e.g. isis).\n"
        << "  Stereo parameters should be set in the stereo.default file.\n\n";

  opt.optimized_correlator = vm.count("optimized-correlator");
  opt.draft_mode = vm.count("draft-mode");

  if ( vm.count("help") )
    vw_throw( ArgumentErr() << usage.str() << general_options );
  if (!vm.count("left-input-image") || !vm.count("right-input-image") ||
      !vm.count("left-camera-model") )
    vw_throw( ArgumentErr() << "Missing all of the correct input files.\n"
              << usage << general_options );
}

int main(int argc, char* argv[]) {

  // The default file type are automatically registered the first time
  // a file is opened or created, however we want to override some of
  // the defaults, so we explicitly register them here before registering
  // our own FileIO driver code.
  DiskImageResource::register_default_file_types();

#if defined(ASP_HAVE_PKG_ISISIO) && ASP_HAVE_PKG_ISISIO == 1
  // Register the Isis file handler with the Vision Workbench
  // DiskImageResource system.
  DiskImageResource::register_file_type(".cub",
                                        DiskImageResourceIsis::type_static(),
                                        &DiskImageResourceIsis::construct_open,
                                        &DiskImageResourceIsis::construct_create);
#endif

  StereoSession::register_session_type( "rmax", &StereoSessionRmax::construct);
#if defined(ASP_HAVE_PKG_ISISIO) && ASP_HAVE_PKG_ISISIO == 1
  StereoSession::register_session_type( "isis", &StereoSessionIsis::construct);
#endif

  Options opt;
  try {
    handle_arguments( argc, argv, opt );

    stereo_settings().read(opt.stereo_default_filename);
    if ( opt.num_threads != 0 ) {
      vw_out() << "\t--> Setting number of processing threads to: "
               << opt.num_threads << std::endl;
      vw_settings().set_default_num_threads(opt.num_threads);
    }

    // Set search range from stereo.default file
    opt.search_range = BBox2i(Vector2i(stereo_settings().h_corr_min,
                                       stereo_settings().v_corr_min),
                              Vector2i(stereo_settings().h_corr_max,
                                       stereo_settings().v_corr_max));

    // If the user hasn't specified a stereo session type, we take a
    // guess here based on the file suffixes.
    if (opt.stereo_session_string.empty()) {
      if ( ( boost::iends_with(opt.cam_file1, ".cahvor") &&
             boost::iends_with(opt.cam_file2, ".cahvor") ) ||
           ( boost::iends_with(opt.cam_file1, ".cahv") &&
             boost::iends_with(opt.cam_file2, ".cahv") ) ||
           ( boost::iends_with(opt.cam_file1, ".pin") &&
             boost::iends_with(opt.cam_file2, ".pin") ) ||
           ( boost::iends_with(opt.cam_file1, ".tsai") &&
             boost::iends_with(opt.cam_file2, ".tsai") ) ||
           ( boost::iends_with(opt.cam_file1, ".cmod") &&
             boost::iends_with(opt.cam_file2, ".cmod") ) ) {
        vw_out() << "\t--> Detected pinhole camera files. "
                 << "Executing pinhole stereo pipeline.\n";
        opt.stereo_session_string = "pinhole";
      } else if (boost::iends_with(opt.in_file1, ".cub") &&
                 boost::iends_with(opt.in_file2, ".cub")) {
        vw_out() << "\t--> Detected ISIS cube files. "
                 << "Executing ISIS stereo pipeline.\n";
        opt.stereo_session_string = "isis";
      } else {
        vw_throw( ArgumentErr() << "Could not determine stereo session type. "
                  << "Please set it explicitly.\n"
                  << "using the -t switch. Options include: [pinhole isis].\n" );
      }
    }


    // Some specialization here so that the user doesn't need to list
    // camera models on the command line for certain stereo session
    // types.  (e.g. isis).
    bool check_for_camera_models = true;
    if ( opt.stereo_session_string == "isis" ) {
      // Fix the ordering of the arguments if the user only supplies 3
      if (opt.out_prefix.empty())
        opt.out_prefix = opt.cam_file1;
      check_for_camera_models = false;
    }

    if ( check_for_camera_models &&
         ( opt.out_prefix.empty() || opt.cam_file2.empty() ) )
      vw_throw( ArgumentErr() << "\nMissing output-prefix or right camera model.\n" );

    fs::path out_prefix_path(opt.out_prefix);
    if (out_prefix_path.has_branch_path()) {
      if (!fs::is_directory(out_prefix_path.branch_path())) {
        vw_out() << "\nCreating output directory: "
                 << out_prefix_path.branch_path() << std::endl;
        fs::create_directory(out_prefix_path.branch_path());
      }
    }

    opt.session = boost::shared_ptr<StereoSession>( StereoSession::create(opt.stereo_session_string) );
    opt.session->initialize(opt.in_file1, opt.in_file2, opt.cam_file1, opt.cam_file2,
                            opt.out_prefix, opt.extra_arg1, opt.extra_arg2,
                            opt.extra_arg3, opt.extra_arg4);

    // The last thing we do before we get started is to copy the
    // stereo.default settings over into the results directory so that
    // we have a record of the most recent stereo.default that was used
    // with this data set.
    stereo_settings().copy_settings(opt.stereo_default_filename,
                                    opt.out_prefix + "-stereo.default");

    // Common GDAL options
#if defined(VW_HAS_BIGTIFF) && VW_HAS_BIGTIFF == 1
    opt.gdal_options["COMPRESS"] = "LZW";
    opt.gdal_options["BIGTIFF"] = "NO";
#else
    opt.gdal_options["COMPRESS"] = "NONE";
    opt.gdal_options["BIGTIFF"] = "NO";
#endif

    opt.raster_tile_size = Vector2i(vw_settings().default_tile_size(),
                                    vw_settings().default_tile_size());

    // user safety check
    //---------------------------------------------------------
    {
      boost::shared_ptr<camera::CameraModel> camera_model1, camera_model2;
      opt.session->camera_models(camera_model1,camera_model2);

      // Do the camera's appear to be in the same location?
      if ( norm_2(camera_model1->camera_center(Vector2()) -
                  camera_model2->camera_center(Vector2())) < 1e-3 )
        vw_out(WarningMessage,"console")
          << "Your cameras appear to be in the same location!\n"
          << "\tYou should be double check your given camera\n"
          << "\tmodels as most likely stereo won't be able\n"
          << "\tto triangulate or perform epipolar rectification.\n";
    }

    // Internal Processes
    //---------------------------------------------------------
    if (opt.entry_point <= PREPROCESSING)
      stereo_preprocessing( opt );

    if (opt.entry_point <= CORRELATION )
      stereo_correlation( opt );

    if (opt.entry_point <= REFINEMENT )
      stereo_refinement( opt );

    if (opt.entry_point <= FILTERING)
      stereo_filtering( opt );

    if (opt.entry_point <= POINT_CLOUD)
      stereo_triangulation( opt );

    vw_out() << "\n[ " << current_posix_time_string() << " ] : FINISHED \n";

  } ASP_STANDARD_CATCHES;

  return 0;
}
