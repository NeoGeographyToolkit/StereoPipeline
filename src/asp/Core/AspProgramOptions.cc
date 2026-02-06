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

#include <asp/Core/StereoSettings.h>
#include <asp/Core/AspProgramOptions.h>
#include <asp/Core/AspLog.h>
#include <asp/Core/EnvUtils.h>
#include <asp/Core/FileUtils.h>
#include <asp/asp_date_config.h>
#include <asp/asp_config.h>

#include <vw/vw_date_config.h>
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

#include <map>
#include <sstream>
#include <string>
#include <vector>
#include <unistd.h>

using namespace vw;
namespace po = boost::program_options;
namespace fs = boost::filesystem;

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
  asp::stereo_settings().initialize(opt);

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
