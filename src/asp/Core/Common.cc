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

#include <vw/Core/Log.h>
#include <vw/Core/System.h>
#include <vw/Math/BBox.h>
#include <vw/FileIO/DiskImageResource.h>
#include <asp/Core/Common.h>
#include <asp/Core/StereoSettings.h>

#include <map>
#include <sstream>
#include <string>
#include <vector>
#include <unistd.h>

#include <gdal_version.h>
#include <proj_api.h>

#include <boost/algorithm/string.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/filesystem/operations.hpp>
#include <boost/filesystem/path.hpp>
#include <boost/filesystem/path_traits.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/program_options.hpp>

#if defined(VW_HAVE_PKG_GDAL) && VW_HAVE_PKG_GDAL==1
#include "ogr_spatialref.h"
#endif

using namespace vw;
namespace po = boost::program_options;
namespace fs = boost::filesystem;

bool asp::has_cam_extension( std::string const& input ) {
  std::string ext = get_extension(input);
  if ( has_pinhole_extension(input) ||
      ext == ".cub" || ext == "xml"      )
    return true;
  return false;
}

bool asp::has_pinhole_extension( std::string const& input ) {
  std::string ext = get_extension(input);
  if ( ext == ".cahvor"  || ext == ".cahv"    ||
       ext == ".pin"     || ext == ".pinhole" ||
       ext == ".tsai"    || ext == ".cmod"    ||
       ext == ".cahvore")
    return true;
  return false;
}

bool asp::has_image_extension( std::string const& input ) {
  std::string ext = get_extension(input);
  if ( ext == ".tif"  || ext == ".tiff" || ext == ".ntf" ||
       ext == ".png"  || ext == ".jpeg" ||
       ext == ".jpg"  || ext == ".jp2"  ||
       ext == ".img"  || ext == ".cub"    )
    return true;
  return false;
}

bool asp::has_tif_or_ntf_extension(std::string const& input){
  std::string ext = get_extension(input);
  if ( ext == ".tif"  || ext == ".ntf")
    return true;
  return false;
}

bool asp::all_files_have_extension(std::vector<std::string> const& files, std::string const& ext){
  for (size_t i = 0; i < files.size(); i++){
    if ( ! boost::iends_with(boost::to_lower_copy(files[i]), ext) )
      return false;
  }
  return true;
}



std::vector<std::string>
asp::get_files_with_ext( std::vector<std::string>& files, std::string const& ext, bool prune_input_list ) {
  std::vector<std::string> match_files;
  std::vector<std::string>::iterator it = files.begin();
  while ( it != files.end() ) {
    if ( boost::iends_with(boost::to_lower_copy(*it), ext) ){ // Match
      match_files.push_back( *it );
      if (prune_input_list) // Clear match from the input list
        it = files.erase( it );
      else
        ++it;
    } else // No Match
      ++it;
  } // End loop through input list

  return match_files;
}



/// Parse the list of files specified as positional arguments on the command lin
bool asp::parse_multiview_cmd_files(std::vector<std::string> const &filesIn,
                                    std::vector<std::string>       &image_paths,
                                    std::vector<std::string>       &camera_paths,
                                    std::string                    &prefix,
                                    std::string                    &dem_path){
  // Init outputs
  image_paths.clear();
  camera_paths.clear();
  prefix   = "";
  dem_path = "";

  // The format is:  <N image paths> [N camera model paths] <output prefix> [input DEM path]

  // Find the input DEM, if any
  std::vector<std::string> files = filesIn; // Make a local copy to work with
  std::string input_dem;
  bool has_georef = false;
  try{ // Just try to load the last file path as a dem
    cartography::GeoReference georef;
    has_georef = read_georeference( georef, files.back() );
  }catch(...){}
  if (has_georef){ // I guess it worked!
    dem_path = files.back();
    files.pop_back();
  }else{ // We tried to load the prefix, there is no dem.
    dem_path = "";
  }

  if (files.size() < 3) // Check for the minimum number of files
    return false;

  // Find the output prefix
  prefix = files.back(); // Dem, if present, was already popped off the back.

  // An output prefix cannot be an image or a camera
  if (asp::has_image_extension(prefix) || asp::has_cam_extension(prefix)) {
    prefix = "";
    return false;
  }

  files.pop_back();

  // Now there are N images and possibly N camera paths
  // - Need to figure out of the camera paths are there!

  // If there is an odd number of files then this is easy
  bool has_camera_paths = (files.size() % 2 == 0);
  // Otherwise we need to do more checks
  if (has_camera_paths) {
    //int  num_image_files     = 0;
    //int  num_camera_files    = 0;
    bool multiple_extensions = false;

    for (size_t i=0; i<files.size(); ++i) {
      //if (has_image_extension(files[i]))
      //  num_image_files++;
      //if (has_camera_extension(files[i]))
      //  num_camera_files++;
      std::string ext = get_extension(files[i]);
      if (ext != get_extension(files[0]))
        multiple_extensions = true;
    }

    //if (num_image_files < files.size()) // Must be camera files
    //  break;
    if (!multiple_extensions)
      has_camera_paths = false; // Two types of files, some must be camera files.

  } // End check for camera file presence


  if (has_camera_paths) { // Move the cameras out of the files vector
    const size_t half = files.size()/2;
    camera_paths.resize(half);
    for (size_t i=0; i<half; ++i) {
      camera_paths[half-i-1] = files.back();
      files.pop_back();
    }
  }
  else // camera_paths = image_paths
    camera_paths = files;
  image_paths = files;

  return true;
}

// Convert dir1/image1.cub to out-prefix-image1.adjust
std::string asp::bundle_adjust_file_name(std::string const& prefix,
                                         std::string const& input_img,
                                         std::string const& input_cam){

  // Create the adjusted camera file name from the original camera filename,
  // unless it is empty, and then use the image file name.
  std::string file = input_cam;
  if (file == "")
    file = input_img;

  return prefix + "-" + fs::path(file).stem().string() + ".adjust";
}

// Print time function
std::string asp::current_posix_time_string() {
  return boost::posix_time::to_simple_string(boost::posix_time::second_clock::local_time());
}

// Unless user-specified, compute the rounding error for a given
// planet (a point on whose surface is given by 'shift'). Return an
// inverse power of 2, 1/2^10 for Earth and proportionally less for
// smaller bodies.
double asp::get_rounding_error(vw::Vector3 const& shift, double rounding_error){

  // Do nothing if the user specified it.
  if (rounding_error > 0.0) return rounding_error;

  double len = norm_2(shift);
  VW_ASSERT(len > 0,  vw::ArgumentErr()
            << "Expecting positive length in get_rounding_error().");
  rounding_error = 1.5e-10*len;
    rounding_error = pow(2.0, round(log(rounding_error)/log(2.0)) );
    return rounding_error;
}

// Run a system command and append the output to a given file
void asp::run_cmd_app_to_file(std::string cmd, std::string file){
  std::string full_cmd;
  full_cmd = "echo '" + cmd + "' >> " + file; // echo the command to run
  system(full_cmd.c_str());
  full_cmd = cmd + " >> " + file + " 2>&1";
  system(full_cmd.c_str());
  full_cmd = "echo '' >> " + file; // append a newline
  system(full_cmd.c_str());
}

std::string asp::extract_prog_name(std::string const& prog_str){

  // Get program name without path and leading 'lt-'.
  std::string prog_name = fs::basename(fs::path(prog_str));
  std::string pref = "lt-";
  size_t lp = pref.size();
  if (prog_name.size() >= lp && prog_name.substr(0, lp) == pref)
    prog_name = prog_name.substr(lp, prog_name.size() - lp);

  return prog_name;
}

void asp::log_to_file(int argc, char *argv[],
                      std::string stereo_default_filename,
                      std::string out_prefix){

  // Log some system info to a file, then copy the vw log
  // info to that file as well.

  if (out_prefix == "")
    vw::vw_throw( vw::ArgumentErr() << "Output prefix was not set.\n");

  // Create the output directory if not present
  vw::create_out_dir(out_prefix);

  std::string prog_name = extract_prog_name(argv[0]);

  // Create the log file and open it in write mode
  std::ostringstream os;
  int pid = getpid();
  os << out_prefix << "-log-" << prog_name << "-" << pid << ".txt";
  std::string log_file = os.str();
  vw_out() << "Writing log info to: " << log_file << std::endl;
  std::ofstream lg(log_file.c_str());

  // Write the program name and its arguments
  for (int s = 0; s < argc; s++) lg << std::string(argv[s]) + " ";
  lg << std::endl << std::endl;

  // Must ensure to close the file handle before further appending to
  // it below.
  lg.close();

  // System calls. Not all will succeed on all machines.
  asp::run_cmd_app_to_file("uname -a", log_file);
  asp::run_cmd_app_to_file("cat /proc/meminfo 2>/dev/null | grep MemTotal", log_file);
  asp::run_cmd_app_to_file("cat /proc/cpuinfo 2>/dev/null | tail -n 25", log_file);
  // The line below is for MacOSX
  asp::run_cmd_app_to_file("sysctl -a hw 2>/dev/null | grep -E \"ncpu|byteorder|memsize|cpufamily|cachesize|mmx|sse|machine|model\" | grep -v ipv6", log_file);
  if (stereo_default_filename != ""){
    std::string cmd = "cat " + stereo_default_filename + " 2>/dev/null";
    asp::run_cmd_app_to_file(cmd, log_file);
  }
  asp::run_cmd_app_to_file("cat ~/.vwrc 2>/dev/null", log_file);

  // Copy all the info going to the console to log_file as well,
  // except the progress bar.
  boost::shared_ptr<vw::LogInstance> current_log( new vw::LogInstance(log_file) );
  current_log->rule_set() = vw_log().console_log().rule_set();
  current_log->rule_set().add_rule(0, "*.progress");
  vw_log().add(current_log);
}

asp::BaseOptions::BaseOptions() {
#if defined(VW_HAS_BIGTIFF) && VW_HAS_BIGTIFF == 1
  gdal_options["COMPRESS"] = "LZW";
#else
  gdal_options["COMPRESS"] = "NONE";
  gdal_options["BIGTIFF"] = "NO";
#endif
  raster_tile_size =
    Vector2i(vw_settings().default_tile_size(),
             vw_settings().default_tile_size());
}

asp::BaseOptionsDescription::BaseOptionsDescription( asp::BaseOptions& opt ) {
  namespace po = boost::program_options;
  (*this).add_options()
    ("threads", po::value(&opt.num_threads)->default_value(0),
     "Select the number of processors (threads) to use.")
    ("no-bigtiff", "Tell GDAL to not create bigtiffs.")
    ("tif-compress", po::value(&opt.tif_compress)->default_value("LZW"),
     "TIFF Compression method. [None, LZW, Deflate, Packbits]")
    ("version,v", "Display the version of software.")
    ("help,h", "Display this help message.");
}

// User should only put the arguments to their application in the
// usage_comment argument. We'll finish filling in the repeated information.
po::variables_map
asp::check_command_line( int argc, char *argv[], BaseOptions& opt,
                         po::options_description const& public_options,
                         po::options_description const& all_public_options,
                         po::options_description const& positional_options,
                         po::positional_options_description const& positional_desc,
                         std::string & usage_comment,
                         bool allow_unregistered,
                         std::vector<std::string> & unregistered) {

  unregistered.clear();

  // Ensure that opt gets all needed fields from BaseOptionsDescription().
  // This is needed not only for stereo, but for all tools using BaseOptions.
  stereo_settings().initialize(opt);

  // Finish filling in the usage_comment.
  std::ostringstream ostr;
  ostr << "Usage: " << argv[0] << " " << usage_comment << "\n\n";
  ostr << "  [ASP " << ASP_VERSION << "]\n\n";
  usage_comment = ostr.str();

  // We distinguish between all_public_options, which is all the
  // options we must parse, even if we don't need some of them, and
  // public_options, which are the options specifically used by the
  // current tool, and for which we also print the help message.
  po::variables_map vm;
  try {
    po::options_description all_options;
    all_options.add(all_public_options).add(positional_options);

    if ( allow_unregistered ) {
      po::parsed_options parsed = po::command_line_parser( argc, argv ).options(all_options).allow_unregistered().style( po::command_line_style::unix_style ).run();
      unregistered = collect_unrecognized(parsed.options, po::include_positional);
      po::store( parsed, vm );
    } else {
      po::store( po::command_line_parser( argc, argv ).options(all_options).positional(positional_desc).style( po::command_line_style::unix_style ).run(), vm );
    }

    po::notify( vm );
  } catch (po::error const& e) {
    vw::vw_throw( vw::ArgumentErr() << "Error parsing input:\n"
                  << e.what() << "\n" << usage_comment << public_options );
  }

  // We really don't want to use BIGTIFF unless we have to. It's
  // hard to find viewers for bigtiff.
  if ( vm.count("no-bigtiff") ) {
    opt.gdal_options["BIGTIFF"] = "NO";
  } else {
    opt.gdal_options["BIGTIFF"] = "IF_SAFER";
  }

  if ( vm.count("help") )
    vw::vw_throw( vw::ArgumentErr() << usage_comment << public_options );

  if ( vm.count("version") ) {
    std::ostringstream ostr;
    ostr << ASP_PACKAGE_STRING  << "\n";
#if defined(ASP_COMMIT_ID)
    ostr << "  Build ID: " << ASP_COMMIT_ID << "\n";
#endif
    ostr << "\nBuilt against:\n  " << VW_PACKAGE_STRING << "\n";
#if defined(VW_COMMIT_ID)
    ostr << "    Build ID: " << VW_COMMIT_ID << "\n";
#endif
#if defined(ASP_HAVE_PKG_ISISIO) && ASP_HAVE_PKG_ISISIO == 1
    ostr << "  USGS ISIS " << ASP_ISIS_VERSION << "\n";
#endif
    ostr << "  Boost C++ Libraries " << ASP_BOOST_VERSION << "\n";
    ostr << "  GDAL " << GDAL_RELEASE_NAME << " | " << GDAL_RELEASE_DATE << "\n";
    ostr << "  Proj.4 " << PJ_VERSION << "\n";
    vw::vw_throw( vw::ArgumentErr() << ostr.str() );
  }

  // If the user did not set the number of threads, use what is set in
  // .vwrc.
  if (opt.num_threads <= 0)
    opt.num_threads = vw_settings().default_num_threads();

  // Print the message below just once per process.
  static bool verbose = true;
  if (verbose){
    vw::vw_out() << "\t--> Setting number of processing threads to: "
                 << opt.num_threads << std::endl;
    verbose = false;
  }

  // Here we ensure that opt.num_threads and default_num_threads()
  // are consistent among themselves.
  vw::vw_settings().set_default_num_threads(opt.num_threads);

  boost::algorithm::to_upper( opt.tif_compress );
  boost::algorithm::trim( opt.tif_compress );
  VW_ASSERT( opt.tif_compress == "NONE" || opt.tif_compress == "LZW" ||
             opt.tif_compress == "DEFLATE" || opt.tif_compress == "PACKBITS",
             ArgumentErr() << "\"" << opt.tif_compress
             << "\" is not a valid options for TIF_COMPRESS." );
  opt.gdal_options["COMPRESS"] = opt.tif_compress;

  return vm;
}


Vector2i asp::file_image_size( std::string const& input ) {
  boost::scoped_ptr<SrcImageResource>
    rsrc( DiskImageResource::open( input ) );
  Vector2i size( rsrc->cols(), rsrc->rows() );
  return size;
}

void asp::set_srs_string(std::string srs_string, bool have_user_datum,
                         vw::cartography::Datum const& user_datum,
                         vw::cartography::GeoReference & georef){

  // Set srs_string into given georef. Note that this may leave the
  // georef's affine transform inconsistent.

  // Use the target_srs_string
#if defined(VW_HAVE_PKG_GDAL) && VW_HAVE_PKG_GDAL==1

  // User convenience, convert 'IAU2000:' to 'DICT:IAU2000.wkt,'
  boost::replace_first(srs_string,
                       "IAU2000:","DICT:IAU2000.wkt,");

  // TODO: The line below is fishy. A better choice would be
  // srs_string = georef.overall_proj4_str() but this needs testing.
  if (srs_string == "")
    srs_string = "+proj=longlat";

  if ( have_user_datum )
    srs_string += " " + user_datum.proj4_str();

  OGRSpatialReference gdal_spatial_ref;
  if (gdal_spatial_ref.SetFromUserInput( srs_string.c_str() ))
    vw_throw( ArgumentErr() << "Failed to parse: \"" << srs_string << "\"." );
  char *wkt_str_tmp = NULL;
  gdal_spatial_ref.exportToWkt( &wkt_str_tmp );
  srs_string = wkt_str_tmp;
  OGRFree(wkt_str_tmp);
  georef.set_wkt(srs_string);

  // Re-apply the user's datum. The important values were already
  // there (major/minor axis), we're just re-applying to make sure
  // the name of the datum is there.
  if ( have_user_datum )
    georef.set_datum( user_datum );

#else
  vw_throw( NoImplErr() << "Target SRS option is not available without GDAL support. Please rebuild VW and ASP with GDAL." );
#endif

}


void asp::BitChecker::check_argument( vw::uint8 arg ) {
  // Turn on the arg'th bit in m_checksum
  m_checksum.set(arg);
}

asp::BitChecker::BitChecker( vw::uint8 num_arguments )  : m_checksum(0) {
  VW_ASSERT( num_arguments != 0,
             ArgumentErr() << "There must be at least one thing you read.\n");
  VW_ASSERT( num_arguments <= 32,
             ArgumentErr() << "You can only have up to 32 checks.\n" );

  // Turn on the first num_arguments bits in m_good
  m_good.reset();
  m_checksum.reset();
  for ( uint8 i=0; i<num_arguments; ++i )
    m_good.set(i);
}

bool asp::BitChecker::is_good() const {
  // Make sure all expected bits in m_checksum have been turned on.
  return (m_good == m_checksum);
}


namespace boost {
namespace program_options {

  // Custom value semantics, these explain how many tokens should be ingested.
  typed_2_value<vw::Vector2i>*
  value( vw::Vector2i* v ) {
    typed_2_value<vw::Vector2i>* r =
      new typed_2_value<vw::Vector2i>(v);
    return r;
  }

  typed_2_value<vw::Vector2>*
  value( vw::Vector2* v ) {
    typed_2_value<vw::Vector2>* r =
      new typed_2_value<vw::Vector2>(v);
    return r;
  }

  typed_4_value<vw::BBox2i>*
  value( vw::BBox2i* v ) {
    typed_4_value<vw::BBox2i>* r =
      new typed_4_value<vw::BBox2i>(v);
    return r;
  }

  typed_4_value<vw::BBox2>*
  value( vw::BBox2* v ) {
    typed_4_value<vw::BBox2>* r =
      new typed_4_value<vw::BBox2>(v);
    return r;
  }

  typed_6_value<vw::BBox3>*
  value( vw::BBox3* v ) {
    typed_6_value<vw::BBox3>* r =
      new typed_6_value<vw::BBox3>(v);
    return r;
  }

  // Custom validators which describe how text is turned into values

  // Validator for Vector2i
  template <>
  void validate( boost::any& v,
                 const std::vector<std::string>& values,
                 vw::Vector2i*, long ) {
    validators::check_first_occurrence(v);

    // Concatenate and then split again, so that the user can mix
    // comma and space delimited values.
    std::string joined = boost::algorithm::join(values, " ");
    std::vector<std::string> cvalues;
    boost::split(cvalues, joined, is_any_of(", "), boost::token_compress_on);

    if ( cvalues.size() != 2 )
      boost::throw_exception(invalid_syntax(invalid_syntax::missing_parameter));

    try {
      Vector2i output( boost::lexical_cast<int32>( cvalues[0] ),
                       boost::lexical_cast<int32>( cvalues[1] ) );
      v = output;
    } catch (boost::bad_lexical_cast const& e ) {
      boost::throw_exception(validation_error(validation_error::invalid_option_value));
    }
  }

  // Validator for Vector2
  template <>
  void validate( boost::any& v,
                 const std::vector<std::string>& values,
                 vw::Vector2*, long ) {
    validators::check_first_occurrence(v);

    // Concatenate and then split again, so that the user can mix
    // comma and space delimited values.
    std::string joined = boost::algorithm::join(values, " ");
    std::vector<std::string> cvalues;
    boost::split(cvalues, joined, is_any_of(", "), boost::token_compress_on);

    if ( cvalues.size() != 2 )
      boost::throw_exception(invalid_syntax(invalid_syntax::missing_parameter));

    try {
      Vector2 output( boost::lexical_cast<double>( cvalues[0] ),
                      boost::lexical_cast<double>( cvalues[1] ) );
      v = output;
    } catch (boost::bad_lexical_cast const& e ) {
      boost::throw_exception(validation_error(validation_error::invalid_option_value));
    }
  }

  // Validator for BBox2i
  template <>
  void validate( boost::any& v,
                 const std::vector<std::string>& values,
                 vw::BBox2i*, long ) {
    validators::check_first_occurrence(v);

    // Concatenate and then split again, so that the user can mix
    // comma and space delimited values.
    std::string joined = boost::algorithm::join(values, " ");
    std::vector<std::string> cvalues;
    boost::split(cvalues, joined, is_any_of(", "), boost::token_compress_on);

    if ( cvalues.size() != 4 )
      boost::throw_exception(invalid_syntax(invalid_syntax::missing_parameter));

    try {
      BBox2i output(Vector2i( boost::lexical_cast<int32>( cvalues[0] ),
                              boost::lexical_cast<int32>( cvalues[1] ) ),
                    Vector2i( boost::lexical_cast<int32>( cvalues[2] ),
                              boost::lexical_cast<int32>( cvalues[3] ) ) );
      v = output;
    } catch (boost::bad_lexical_cast const& e ) {
      boost::throw_exception(validation_error(validation_error::invalid_option_value));
    }
  }

  // Validator for BBox2
  template <>
  void validate( boost::any& v,
                 const std::vector<std::string>& values,
                 vw::BBox2*, long ) {
    validators::check_first_occurrence(v);

    // Concatenate and then split again, so that the user can mix
    // comma and space delimited values.
    std::string joined = boost::algorithm::join(values, " ");
    std::vector<std::string> cvalues;
    boost::split(cvalues, joined, is_any_of(", "), boost::token_compress_on);

    if ( cvalues.size() != 4 )
      boost::throw_exception(invalid_syntax(invalid_syntax::missing_parameter));

    try {
      BBox2 output(Vector2( boost::lexical_cast<double>( cvalues[0] ),
                            boost::lexical_cast<double>( cvalues[1] ) ),
                   Vector2( boost::lexical_cast<double>( cvalues[2] ),
                            boost::lexical_cast<double>( cvalues[3] ) ) );
      v = output;
    } catch (boost::bad_lexical_cast const& e ) {
      boost::throw_exception(validation_error(validation_error::invalid_option_value));
    }
  }

  // Validator for BBox3
  template <>
  void validate( boost::any& v,
                 const std::vector<std::string>& values,
                 vw::BBox3*, long ) {
    validators::check_first_occurrence(v);

    // Concatenate and then split again, so that the user can mix
    // comma and space delimited values.
    std::string joined = boost::algorithm::join(values, " ");
    std::vector<std::string> cvalues;
    boost::split(cvalues, joined, is_any_of(", "), boost::token_compress_on);

    if ( cvalues.size() != 6 )
      boost::throw_exception(invalid_syntax(invalid_syntax::missing_parameter));

    try {
      BBox3 output(Vector3( boost::lexical_cast<double>( cvalues[0] ),
                            boost::lexical_cast<double>( cvalues[1] ),
                            boost::lexical_cast<double>( cvalues[2] )
                            ),
                   Vector3( boost::lexical_cast<double>( cvalues[3] ),
                            boost::lexical_cast<double>( cvalues[4] ),
                            boost::lexical_cast<double>( cvalues[5] )
                            ) );
      v = output;
    } catch (boost::bad_lexical_cast const& e ) {
      boost::throw_exception(validation_error(validation_error::invalid_option_value));
    }
  }

}}
