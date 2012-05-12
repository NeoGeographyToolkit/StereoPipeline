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


#include <asp/Tools/stereo.h>

using namespace vw;

// Print time function
std::string vw::current_posix_time_string() {
#ifndef __APPLE__
  char time_string[2048];
  time_t t = time(0);
  struct tm* time_struct = localtime(&t);
  strftime(time_string, 2048, "%F %T", time_struct);
  return std::string(time_string);
#else
  std::ostringstream time_string_stream;
  time_string_stream << boost::posix_time::second_clock::local_time();
  return time_string_stream.str();
#endif
}

// Parse input command line arguments
void vw::handle_arguments( int argc, char *argv[], Options& opt ) {
  po::options_description general_options("");
  general_options.add_options()
    ("session-type,t", po::value(&opt.stereo_session_string), "Select the stereo session type to use for processing. [options: pinhole isis]")
    ("stereo-file,s", po::value(&opt.stereo_default_filename)->default_value("./stereo.default"), "Explicitly specify the stereo.default file to use. [default: ./stereo.default]");
  general_options.add( asp::BaseOptionsDescription(opt) );

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

  std::string usage("[options] <Left_input_image> <Right_input_image> [Left_camera_file] [Right_camera_file] <output_file_prefix>\n  Extensions are automaticaly added to the output files.\n  Camera model arguments may be optional for some stereo session types (e.g. isis).\n  Stereo parameters should be set in the stereo.default file.");
  po::variables_map vm =
    asp::check_command_line( argc, argv, opt, general_options,
                             positional, positional_desc, usage );

  if (!vm.count("left-input-image") || !vm.count("right-input-image") ||
      !vm.count("left-camera-model") )
    vw_throw( ArgumentErr() << "Missing all of the correct input files.\n\n"
              << usage << general_options );

  // If the user hasn't specified a stereo session type, we take a
  // guess here based on the file suffixes.
  if (opt.stereo_session_string.empty()) {
    if ( asp::has_cam_extension( opt.cam_file1 ) &&
         asp::has_cam_extension( opt.cam_file2 ) ) {
      vw_out() << "\t--> Detected pinhole camera files. "
               << "Executing pinhole stereo pipeline.\n";
      opt.stereo_session_string = "pinhole";
    } else if (boost::iends_with(boost::to_lower_copy(opt.in_file1), ".cub") &&
               boost::iends_with(boost::to_lower_copy(opt.in_file2), ".cub")) {
      vw_out() << "\t--> Detected ISIS cube files. "
               << "Executing ISIS stereo pipeline.\n";
      opt.stereo_session_string = "isis";
    } else if (boost::iends_with(boost::to_lower_copy(opt.cam_file1), ".xml") &&
               boost::iends_with(boost::to_lower_copy(opt.cam_file2), ".xml")) {
      vw_out() << "\t--> Detected likely Digital Globe XML files. "
               << "Executing DG stereo pipeline.\n";
      opt.stereo_session_string = "dg";
    } else {
      vw_throw( ArgumentErr() << "Could not determine stereo session type. "
                << "Please set it explicitly.\n"
                << "using the -t switch. Options include: [pinhole isis dg].\n" );
    }
  }

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

  fs::path out_prefix_path(opt.out_prefix);
  if (out_prefix_path.has_branch_path()) {
    if (!fs::is_directory(out_prefix_path.branch_path())) {
      vw_out() << "\nCreating output directory: "
               << out_prefix_path.branch_path() << std::endl;
      fs::create_directory(out_prefix_path.branch_path());
    }
  }

  // Finally read in the stereo settings
  stereo_settings().read(opt.stereo_default_filename);

  // Pull out the TIFF compression option
  if ( stereo_settings().tif_compress == "lzw" ) {
    opt.gdal_options["COMPRESS"] = "LZW";
  } else if ( stereo_settings().tif_compress == "packbits" ) {
    opt.gdal_options["COMPRESS"] = "PACKBITS";
  } else if ( stereo_settings().tif_compress == "deflate" ) {
    opt.gdal_options["COMPRESS"] = "DEFLATE";
  } else {
    opt.gdal_options["COMPRESS"] = "NONE";
  }

  opt.session.reset( asp::StereoSession::create(opt.stereo_session_string) );
  opt.session->initialize(opt, opt.in_file1, opt.in_file2,
                          opt.cam_file1, opt.cam_file2,
                          opt.out_prefix, opt.extra_arg1, opt.extra_arg2,
                          opt.extra_arg3, opt.extra_arg4);

  // The last thing we do before we get started is to copy the
  // stereo.default settings over into the results directory so that
  // we have a record of the most recent stereo.default that was used
  // with this data set.
  stereo_settings().copy_settings(opt.stereo_default_filename,
                                  opt.out_prefix + "-stereo.default");
}

// Register Session types
void vw::stereo_register_sessions() {

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

