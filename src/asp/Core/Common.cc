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

#include <map>
#include <sstream>
#include <string>
#include <vector>
#include <unistd.h>

#include <vw/config.h>
#include <asp/asp_config.h>
#include <gdal_version.h>
#include <proj_api.h>

#include <boost/algorithm/string.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/filesystem/operations.hpp>
#include <boost/filesystem/path.hpp>
#include <boost/filesystem/path_traits.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/program_options.hpp>

using namespace vw;
namespace po = boost::program_options;
namespace fs = boost::filesystem;

// Make the specified file to be relative to the specified directory.
fs::path asp::make_file_relative_to_dir(fs::path const file, fs::path const dir) {
  if (file.has_root_path()){
    if (file.root_path() != dir.root_path()) {
      return file;
    } else {
      return make_file_relative_to_dir(file.relative_path(), dir.relative_path());
    }
  } else {
    if (dir.has_root_path()) {
      fs::path file2 = fs::complete(file);
      return make_file_relative_to_dir(file2.relative_path(), dir.relative_path());
    } else {
      typedef fs::path::const_iterator path_iterator;
      path_iterator file_it = file.begin();
      path_iterator dir_it = dir.begin();
      while ( file_it != file.end() && dir_it != dir.end() ) {
        if (*file_it != *dir_it) break;
        ++file_it; ++dir_it;
      }
      fs::path result;
      for (; dir_it != dir.end(); ++dir_it) {
        result /= "..";
      }
      for (; file_it != file.end(); ++file_it) {
        result /= *file_it;
      }
      return result;
    }
  }
}

// Remove file name extension
std::string asp::prefix_from_filename(std::string const& filename) {
  std::string result = filename;
  int index = result.rfind(".");
  if (index != -1)
    result.erase(index, result.size());
  return result;
}

// Print time function
std::string asp::current_posix_time_string() {
  return boost::posix_time::to_simple_string(boost::posix_time::second_clock::local_time());
}

// If prefix is "dir/out", create directory "dir"
void asp::create_out_dir(std::string out_prefix){

  fs::path out_prefix_path(out_prefix);
  if (out_prefix_path.has_parent_path()) {
    if (!fs::is_directory(out_prefix_path.parent_path())) {
      vw_out() << "\nCreating output directory: "
               << out_prefix_path.parent_path() << std::endl;
      fs::create_directory(out_prefix_path.parent_path());
    }
  }

  return;
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

// Find how many channels/bands are in a given image
int asp::get_num_channels(std::string filename){
  boost::scoped_ptr<vw::SrcImageResource> src(vw::DiskImageResource::open(filename));
  int num_channels = src->channels();
  int num_planes   = src->planes();
  return num_channels*num_planes;
}

void asp::log_to_file(int argc, char *argv[],
                      std::string stereo_default_filename,
                      std::string out_prefix){

  // Log some system info to a file, then copy the vw log
  // info to that file as well.

  if (out_prefix == "")
    vw::vw_throw( vw::ArgumentErr() << "Output prefix was not set.\n");

  // Check that the output directory exists
  fs::path out_prefix_path(out_prefix);
  if (out_prefix_path.has_parent_path()) {
    if (!fs::is_directory(out_prefix_path.parent_path())) {
      vw::vw_throw( vw::ArgumentErr() << "Directory does not exist: "
                    << out_prefix_path.parent_path() << "\n");
    }
  }
  
  // Get program name without leading 'lt-'
  std::string prog_name = fs::basename(fs::path(std::string(argv[0])));
  std::string pref = "lt-";
  size_t lp = pref.size();
  if (prog_name.size() >= lp && prog_name.substr(0, lp) == pref)
    prog_name = prog_name.substr(lp, prog_name.size() - lp);
  
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
    ("cache-dir", po::value(&opt.cache_dir)->default_value("/tmp"),
     "Folder for temporary files. Change if directory is inaccessible to user such as on Pleiades.")
    ("version,v", "Display the version of software.")
    ("help,h", "Display this help message.");
}

// User should only put the arguments to their application in the
// usage_comment argument. We'll finish filling in the repeated
// information.
po::variables_map
asp::check_command_line( int argc, char *argv[], BaseOptions& opt,
                         po::options_description const& public_options,
                         po::options_description const& all_public_options,
                         po::options_description const& positional_options,
                         po::positional_options_description const& positional_desc,
                         std::string & usage_comment,
                         bool allow_unregistered ) {

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
      po::store( po::command_line_parser( argc, argv ).options(all_options).positional(positional_desc).allow_unregistered().style( po::command_line_style::unix_style ).run(), vm );
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
  if ( opt.num_threads != 0 ) {
    vw::vw_out() << "\t--> Setting number of processing threads to: "
                 << opt.num_threads << std::endl;
    vw::vw_settings().set_default_num_threads(opt.num_threads);
  }
  boost::algorithm::to_upper( opt.tif_compress );
  boost::algorithm::trim( opt.tif_compress );
  VW_ASSERT( opt.tif_compress == "NONE" || opt.tif_compress == "LZW" ||
             opt.tif_compress == "DEFLATE" || opt.tif_compress == "PACKBITS",
             ArgumentErr() << "\"" << opt.tif_compress
             << "\" is not a valid options for TIF_COMPRESS." );
  opt.gdal_options["COMPRESS"] = opt.tif_compress;

  return vm;
}

bool asp::has_cam_extension( std::string const& input ) {
  boost::filesystem::path ipath( input );
  std::string ext = ipath.extension().string();
  if ( ext == ".cahvor" || ext == ".cahv" ||
       ext == ".pin" || ext == ".pinhole" ||
       ext == ".tsai" || ext == ".cmod" ||
       ext == ".cahvore" )
    return true;
  return false;
}

Vector2i asp::file_image_size( std::string const& input ) {
  boost::scoped_ptr<SrcImageResource>
    rsrc( DiskImageResource::open( input ) );
  Vector2i size( rsrc->cols(), rsrc->rows() );
  return size;
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
