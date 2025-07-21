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

#include <asp/Core/Common.h>
#include <asp/Core/StereoSettings.h>
#include <asp/Core/EnvUtils.h>
#include <asp/Core/FileUtils.h>

#include <asp/asp_date_config.h>

#include <vw/Core/Log.h>
#include <vw/Core/System.h>
#include <vw/Math/BBox.h>
#include <vw/FileIO/DiskImageResource.h>
#include <vw/FileIO/FileUtils.h>
#include <vw/FileIO/FileTypes.h>

#include <boost/algorithm/string.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/filesystem/operations.hpp>
#include <boost/filesystem/path.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/program_options.hpp>
#include <boost/algorithm/string/replace.hpp>

// TODO(oalexan1): Move this to VW in the cartography module.
#include <gdal_version.h>
#if defined(VW_HAVE_PKG_GDAL) && VW_HAVE_PKG_GDAL==1
#include "ogr_spatialref.h"
#endif

#include <map>
#include <sstream>
#include <string>
#include <vector>
#include <unistd.h>

using namespace vw;
namespace po = boost::program_options;
namespace fs = boost::filesystem;

/// Parse 'VAR1=VAL1 VAR2=VAL2' into a map. Note that we append to the map,
/// so it may have some items there beforehand.
void asp::parse_append_metadata(std::string const& metadata,
                                std::map<std::string, std::string> & keywords){
  
  std::istringstream is(metadata);
  std::string meta, var, val;
  while (is >> meta){
    boost::replace_all(meta, "=", " ");  // replace equal with space
    std::istringstream is2(meta);
    if (!(is2 >> var >> val)) 
      vw_throw(ArgumentErr() << "Could not parse: " << meta << "\n");
    keywords[var] = val;
  }
}

// Print time function
std::string asp::current_posix_time_string() {
  return boost::posix_time::to_simple_string(boost::posix_time::second_clock::local_time());
}

// Run a system command and append the output to a given file
void asp::run_cmd_app_to_file(std::string cmd, std::string file){
  std::string full_cmd;
  full_cmd = "echo '" + cmd + "' >> " + file; // echo the command to run
  int code = system(full_cmd.c_str());
  full_cmd = cmd + " >> " + file + " 2>&1";
  code = system(full_cmd.c_str());
  full_cmd = "echo '' >> " + file; // append a newline
  code = system(full_cmd.c_str());
}

std::string asp::extract_prog_name(std::string const& prog_str){

  // Get program name without path and leading 'lt-'.
  std::string prog_name = fs::path(prog_str).stem().string();
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
    vw::vw_throw(vw::ArgumentErr() << "Output prefix was not set.\n");

  // Create the output directory if not present
  vw::create_out_dir(out_prefix);

  std::string prog_name = extract_prog_name(argv[0]);

  // Create the log file and open it in write mode
  std::ostringstream os;
  int pid = getpid();
  std::string timestamp = 
      boost::posix_time::to_iso_string(boost::posix_time::second_clock::local_time());
  std::string clean_timestamp = timestamp.substr(4, 9); // Trim off the year
  clean_timestamp.replace(4, 1, 1, '-'); // Replace T with -
  clean_timestamp.insert (2, 1,    '-'); // Insert - between month and day
  os << out_prefix << "-log-" << prog_name << "-" 
     << clean_timestamp << "-" << pid << ".txt";
  std::string log_file = os.str();
  vw_out() << "Writing log: " << log_file << std::endl;
  std::ofstream lg(log_file.c_str());

  // Write the version
  lg << "ASP " << ASP_VERSION << "\n";

#if defined(ASP_COMMIT_ID)
    lg << "Build ID: " << ASP_COMMIT_ID << "\n";
#endif
#if defined(ASP_BUILD_DATE)
    lg << "Build date: " << ASP_BUILD_DATE << "\n";
#endif

    lg << "\n"; // leave some separation
    
    // Write the program name and its arguments
    for (int s = 0; s < argc; s++) {
      std::string token = std::string(argv[s]);
      // Skip adding empty spaces
      if (token == " ")
        continue;
      // Use quotes if there are spaces
      if (token.find(" ") != std::string::npos || token.find("\t") != std::string::npos) 
        token = '"' + token + '"';
      lg << token + " ";
    }
    
  lg << std::endl << std::endl;

  // Must ensure to close the file handle before further appending to
  // it below.
  lg.close();

  // System calls. Not all will succeed on all machines.
  asp::run_cmd_app_to_file("uname -a", log_file);
  if (fs::exists("/proc/meminfo")) 
    asp::run_cmd_app_to_file("cat /proc/meminfo 2>/dev/null | grep MemTotal", log_file);
  if (fs::exists("/proc/cpuinfo")) 
    asp::run_cmd_app_to_file("cat /proc/cpuinfo 2>/dev/null | tail -n 25", log_file);
  
  // The line below is for MacOSX
  asp::run_cmd_app_to_file("sysctl -a hw 2>/dev/null | grep -E \"ncpu|byteorder|memsize|cpufamily|cachesize|mmx|sse|machine|model\" | grep -v ipv6", log_file);
  if (stereo_default_filename != "" && fs::exists(stereo_default_filename)) {
    std::string cmd = "cat " + stereo_default_filename + " 2>/dev/null";
    asp::run_cmd_app_to_file(cmd, log_file);
  }

  // Save the current .vwrc
  char * home_ptr = getenv("HOME");
  if (home_ptr) {
    std::string home = home_ptr;
    std::string vwrc = home + "/.vwrc";
    if (fs::exists(vwrc)) {
      std::string cmd = "cat " + vwrc + " 2>/dev/null";
      asp::run_cmd_app_to_file(cmd, log_file);
    }
  }
  
  // Copy all the info going to the console to log_file as well,
  // except the progress bar.
  boost::shared_ptr<vw::LogInstance> current_log(new vw::LogInstance(log_file));
  current_log->rule_set() = vw_log().console_log().rule_set();
  current_log->rule_set().add_rule(0, "*.progress");
  vw_log().add(current_log);
}

// User should only put the arguments to their application in the
// usage_comment argument. We'll finish filling in the repeated information.
po::variables_map
asp::check_command_line(int argc, char *argv[], vw::GdalWriteOptions& opt,
                        po::options_description const& public_options,
                        po::options_description const& all_public_options,
                        po::options_description const& positional_options,
                        po::positional_options_description const& positional_desc,
                        std::string & usage_comment,
                        bool allow_unregistered,
                        std::vector<std::string> & unregistered) {

  unregistered.clear();

  // Ensure that opt gets all needed fields from vw::GdalWriteOptionsDescription().
  // This is needed not only for stereo, but for all tools using vw::GdalWriteOptions.
  stereo_settings().initialize(opt);

  // Finish filling in the usage_comment.
  std::ostringstream ostr;
  ostr << "Usage: " << argv[0] << " " << usage_comment << "\n\n";
  ostr << "  [ASP " << ASP_VERSION << "]\n";
#if defined(ASP_BUILD_DATE)
  ostr << "  Build date: " << ASP_BUILD_DATE << "\n";
#endif
  ostr << "\n";
  
  usage_comment = ostr.str();

  // Set a handful of of env vars for ISIS, GDAL, PROJ
  asp::set_asp_env_vars();
  
  // We distinguish between all_public_options, which is all the
  // options we must parse, even if we don't need some of them, and
  // public_options, which are the options specifically used by the
  // current tool, and for which we also print the help message.
  po::variables_map vm;
  try {
    po::options_description all_options;
    all_options.add(all_public_options).add(positional_options);

    if (allow_unregistered) {
      po::parsed_options parsed = 
        po::command_line_parser(argc, argv).options(all_options).allow_unregistered()
         .style(po::command_line_style::unix_style).run();
      unregistered = collect_unrecognized(parsed.options, po::include_positional);
      po::store(parsed, vm);
    } else {
      po::parsed_options parsed = 
        po::command_line_parser(argc, argv).options(all_options).positional(positional_desc)
         .style(po::command_line_style::unix_style).run();
      po::store(parsed, vm);
    }

    po::notify(vm);
  } catch (po::error const& e) {
    vw::vw_throw(vw::ArgumentErr() << "Error parsing input:\n"
                  << e.what() << "\n" << usage_comment << public_options);
  }

  // We really don't want to use BIGTIFF unless we have to. It's
  // hard to find viewers for bigtiff.
  if (vm.count("no-bigtiff")) {
    opt.gdal_options["BIGTIFF"] = "NO";
  } else {
    opt.gdal_options["BIGTIFF"] = "IF_SAFER";
  }

  if (vm.count("help")) {
    vw::vw_out() << usage_comment << public_options << "\n";
    exit(0);
  }

  if (vm.count("version")) {
    std::ostringstream ostr;
    ostr << ASP_PACKAGE_STRING  << "\n";
#if defined(ASP_COMMIT_ID)
    ostr << "  Build ID: " << ASP_COMMIT_ID << "\n";
#endif
#if defined(ASP_BUILD_DATE)
    ostr << "  Build date: " << ASP_BUILD_DATE << "\n";
#endif
    ostr << "\nBuilt against:\n  " << VW_PACKAGE_STRING << "\n";
#if defined(VW_COMMIT_ID)
    ostr << "    Build ID: " << VW_COMMIT_ID << "\n";
#endif
#if defined(ASP_HAVE_PKG_ISIS) && ASP_HAVE_PKG_ISIS == 1
    ostr << "  USGS ISIS " << ASP_ISIS_VERSION << "\n";
#endif
    ostr << "  Boost C++ Libraries " << ASP_BOOST_VERSION << "\n";
    ostr << "  GDAL " << GDAL_RELEASE_NAME << " | " << GDAL_RELEASE_DATE << "\n";
    vw::vw_out() << ostr.str() << "\n";
    exit(0);
  }

  opt.setVwSettingsFromOpt();
  
  return vm;
}

// TODO(oalexan1): Move this to VW in the cartography module
void asp::set_srs_string(std::string srs_string, bool have_user_datum,
                         vw::cartography::Datum const& user_datum,
                         vw::cartography::GeoReference & georef) {

  // When an EPSG code is provided, store the name so that
  //  it shows up when the GeoReference object is written
  //  out to disk.
  if (srs_string.find("EPSG") != std::string::npos)
    georef.set_projcs_name(srs_string);

  // Set srs_string into given georef. Note that this may leave the
  // georef's affine transform inconsistent.

  // TODO: The line below needs more thought
  if (srs_string == "")
    srs_string = "+proj=longlat";

  // TODO(oalexan1): Use below datum.get_wkt().
  // But then cannot concatenate the wkt. Need to have a way
  // of reconciling the two wkt. May need to first call 
  // set_datum, and then set_wkt. The latter will 
  // try to reconcile the wkt with the prior datum.
  // But this needs testing, especially in cases when 
  // srs_string lacks the datum info.
  if (have_user_datum)
    srs_string += " " + user_datum.proj4_str();
  
  // TODO(oalexan1): Rename set_wkt to set_srs.
  // TODO(oalexan1): Must wipe all this and call directly georef.set_wkt.
  // TODO(oalexan1): Deal with datum name!
  vw::cartography::GeoReference input_georef = georef;
  OGRSpatialReference gdal_spatial_ref;
  if (gdal_spatial_ref.SetFromUserInput(srs_string.c_str()) != OGRERR_NONE)
    vw::vw_throw(vw::ArgumentErr() << "Failed to parse: \"" << srs_string << "\".");
  char *wkt_str_tmp = NULL;
  gdal_spatial_ref.exportToWkt(&wkt_str_tmp);
  srs_string = wkt_str_tmp;
  CPLFree(wkt_str_tmp);
  georef.set_wkt(srs_string);
}

namespace boost {
namespace program_options {

  // Custom value semantics, these explain how many tokens should be ingested.
  
  // 2 values
  typed_2_value<vw::Vector2i>*
  value(vw::Vector2i* v) {
    typed_2_value<vw::Vector2i>* r =
      new typed_2_value<vw::Vector2i>(v);
    return r;
  }
  typed_2_value<vw::Vector2>*
  value(vw::Vector2* v) {
    typed_2_value<vw::Vector2>* r =
      new typed_2_value<vw::Vector2>(v);
    return r;
  }

  // 3 values
  typed_3_value<vw::Vector3i>*
  value(vw::Vector3i* v) {
    typed_3_value<vw::Vector3i>* r =
      new typed_3_value<vw::Vector3i>(v);
    return r;
  }
  typed_3_value<vw::Vector3>*
  value(vw::Vector3* v) {
    typed_3_value<vw::Vector3>* r =
      new typed_3_value<vw::Vector3>(v);
    return r;
  }

  // 4 values
  typed_4_value<vw::BBox2i>*
  value(vw::BBox2i* v) {
    typed_4_value<vw::BBox2i>* r =
      new typed_4_value<vw::BBox2i>(v);
    return r;
  }
  typed_4_value<vw::BBox2>*
  value(vw::BBox2* v) {
    typed_4_value<vw::BBox2>* r =
      new typed_4_value<vw::BBox2>(v);
    return r;
  }

  // 6 values
  typed_6_value<vw::BBox3>*
  value(vw::BBox3* v) {
    typed_6_value<vw::BBox3>* r =
      new typed_6_value<vw::BBox3>(v);
    return r;
  }

  // Custom validators which describe how text is turned into values

  // Validator for Vector2i
  template <>
  void validate(boost::any& v,
                 const std::vector<std::string>& values,
                 vw::Vector2i*, long) {
    validators::check_first_occurrence(v);

    // Concatenate and then split again, so that the user can mix
    // comma and space delimited values.
    std::string joined = boost::algorithm::join(values, " ");
    std::vector<std::string> cvalues;
    boost::split(cvalues, joined, is_any_of(", "), boost::token_compress_on);

    if (cvalues.size() != 2)
      boost::throw_exception(invalid_syntax(invalid_syntax::missing_parameter));

    try {
      Vector2i output(boost::lexical_cast<int32>(cvalues[0]),
                       boost::lexical_cast<int32>(cvalues[1]));
      v = output;
    } catch (boost::bad_lexical_cast const& e) {
      boost::throw_exception(validation_error(validation_error::invalid_option_value));
    }
  }

  // Validator for Vector2
  template <>
  void validate(boost::any& v,
                 const std::vector<std::string>& values,
                 vw::Vector2*, long) {
    validators::check_first_occurrence(v);

    // Concatenate and then split again, so that the user can mix
    // comma and space delimited values.
    std::string joined = boost::algorithm::join(values, " ");
    std::vector<std::string> cvalues;
    boost::split(cvalues, joined, is_any_of(", "), boost::token_compress_on);

    if (cvalues.size() != 2)
      boost::throw_exception(invalid_syntax(invalid_syntax::missing_parameter));

    try {
      Vector2 output(boost::lexical_cast<double>(cvalues[0]),
                      boost::lexical_cast<double>(cvalues[1]));
      v = output;
    } catch (boost::bad_lexical_cast const& e) {
      boost::throw_exception(validation_error(validation_error::invalid_option_value));
    }
  }

  // Validator for Vector3i
  template <>
  void validate(boost::any& v,
                 const std::vector<std::string>& values,
                 vw::Vector3i*, long) {
    validators::check_first_occurrence(v);

    // Concatenate and then split again, so that the user can mix
    // comma and space delimited values.
    std::string joined = boost::algorithm::join(values, " ");
    std::vector<std::string> cvalues;
    boost::split(cvalues, joined, is_any_of(", "), boost::token_compress_on);

    if (cvalues.size() != 3)
      boost::throw_exception(invalid_syntax(invalid_syntax::missing_parameter));

    try {
      Vector3i output(boost::lexical_cast<int32>(cvalues[0]),
                      boost::lexical_cast<int32>(cvalues[1]),
                      boost::lexical_cast<int32>(cvalues[2]));
      v = output;
    } catch (boost::bad_lexical_cast const& e) {
      boost::throw_exception(validation_error(validation_error::invalid_option_value));
    }
  }

  // Validator for Vector3
  template <>
  void validate(boost::any& v,
                const std::vector<std::string>& values,
                vw::Vector3*, long) {
    validators::check_first_occurrence(v);

    // Concatenate and then split again, so that the user can mix
    // comma and space delimited values.
    std::string joined = boost::algorithm::join(values, " ");
    std::vector<std::string> cvalues;
    boost::split(cvalues, joined, is_any_of(", "), boost::token_compress_on);

    if (cvalues.size() != 3)
      boost::throw_exception(invalid_syntax(invalid_syntax::missing_parameter));

    try {
      Vector3 output(boost::lexical_cast<double>(cvalues[0]),
                     boost::lexical_cast<double>(cvalues[1]),
                     boost::lexical_cast<double>(cvalues[2]));

      v = output;
    } catch (boost::bad_lexical_cast const& e) {
      boost::throw_exception(validation_error(validation_error::invalid_option_value));
    }
  }

  // Validator for BBox2i
  template <>
  void validate(boost::any& v,
                 const std::vector<std::string>& values,
                 vw::BBox2i*, long) {
    validators::check_first_occurrence(v);

    // Concatenate and then split again, so that the user can mix
    // comma and space delimited values.
    std::string joined = boost::algorithm::join(values, " ");
    std::vector<std::string> cvalues;
    boost::split(cvalues, joined, is_any_of(", "), boost::token_compress_on);

    if (cvalues.size() != 4)
      boost::throw_exception(invalid_syntax(invalid_syntax::missing_parameter));

    try {
      BBox2i output(Vector2i(boost::lexical_cast<int32>(cvalues[0]),
                              boost::lexical_cast<int32>(cvalues[1])),
                    Vector2i(boost::lexical_cast<int32>(cvalues[2]),
                              boost::lexical_cast<int32>(cvalues[3])));
      v = output;
    } catch (boost::bad_lexical_cast const& e) {
      boost::throw_exception(validation_error(validation_error::invalid_option_value));
    }
  }

  // Validator for BBox2
  template <>
  void validate(boost::any& v,
                 const std::vector<std::string>& values,
                 vw::BBox2*, long) {
    validators::check_first_occurrence(v);

    // Concatenate and then split again, so that the user can mix
    // comma and space delimited values.
    std::string joined = boost::algorithm::join(values, " ");
    std::vector<std::string> cvalues;
    boost::split(cvalues, joined, is_any_of(", "), boost::token_compress_on);

    if (cvalues.size() != 4)
      boost::throw_exception(invalid_syntax(invalid_syntax::missing_parameter));

    try {
      BBox2 output(Vector2(boost::lexical_cast<double>(cvalues[0]),
                            boost::lexical_cast<double>(cvalues[1])),
                   Vector2(boost::lexical_cast<double>(cvalues[2]),
                            boost::lexical_cast<double>(cvalues[3])));
      v = output;
    } catch (boost::bad_lexical_cast const& e) {
      boost::throw_exception(validation_error(validation_error::invalid_option_value));
    }
  }

  // Validator for BBox3
  template <>
  void validate(boost::any& v,
                 const std::vector<std::string>& values,
                 vw::BBox3*, long) {
    validators::check_first_occurrence(v);

    // Concatenate and then split again, so that the user can mix
    // comma and space delimited values.
    std::string joined = boost::algorithm::join(values, " ");
    std::vector<std::string> cvalues;
    boost::split(cvalues, joined, is_any_of(", "), boost::token_compress_on);

    if (cvalues.size() != 6)
      boost::throw_exception(invalid_syntax(invalid_syntax::missing_parameter));

    try {
      BBox3 output(Vector3(boost::lexical_cast<double>(cvalues[0]),
                            boost::lexical_cast<double>(cvalues[1]),
                            boost::lexical_cast<double>(cvalues[2])),
                   Vector3(boost::lexical_cast<double>(cvalues[3]),
                            boost::lexical_cast<double>(cvalues[4]),
                            boost::lexical_cast<double>(cvalues[5])));
      v = output;
    } catch (boost::bad_lexical_cast const& e) {
      boost::throw_exception(validation_error(validation_error::invalid_option_value));
    }
  }

}}
