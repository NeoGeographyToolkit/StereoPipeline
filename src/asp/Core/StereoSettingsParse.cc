// __BEGIN_LICENSE__
//  Copyright (c) 2009-2025, United States Government as represented by the
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

/// \file StereoSettingsParse.cc
/// 

#include <asp/Core/StereoSettingsParse.h>

#include <vw/Core/Log.h>

#include <boost/program_options.hpp>
#include <boost/algorithm/string.hpp>

#include <fstream>
#include <set>

namespace po = boost::program_options;

namespace asp {

/// Custom readers for Boost program options
class asp_config_file_iterator: public boost::program_options::detail::common_config_file_iterator {
  boost::shared_ptr<std::basic_istream<char> > is;
private:
  bool getline(std::string& s); // Used to precondition string before reading
public:
  asp_config_file_iterator() {
    found_eof();
  }

  // Creates a config file parser for the specified stream.
  asp_config_file_iterator(std::basic_istream<char>& is,
                            const std::set<std::string>& allowed_options,
                            bool allow_unregistered = false);
};

bool asp_config_file_iterator::getline(std::string& s) {
  std::string ws;

  if (!std::getline(*is, ws, '\n'))
    return false;

  // Remove any comments that might be one the line
  size_t n = ws.find('#');
  if (n != std::string::npos)
    ws = ws.substr(0, n);

  // Wipe any whitespace on either end
  boost::trim(ws);

  // Handle empty lines. Just pass them on through.
  if (ws.empty()) {
    s = ws;
    return true;
  }

  // If there is not an equal sign, the first space is turned to
  // equal, or it is just appended. Also, use lowercase for the key.
  n = ws.find('=');
  if (n  == std::string::npos) {
    n = ws.find(' ');

    if (n == std::string::npos) {
      ws += "=";
      boost::to_lower(ws);
    } else {
      ws[n] = '=';
      std::string lowered_key = boost::to_lower_copy(ws.substr(0,n));
      ws.replace(0, n, lowered_key);
    }
  } else {
    std::string lowered_key = boost::to_lower_copy(ws.substr(0,n));
    ws.replace(0, n, lowered_key);
  }
  s = ws;
  return true;
}

asp_config_file_iterator::asp_config_file_iterator(
  std::basic_istream<char>& is,
  const std::set<std::string>& allowed_options,
  bool allow_unregistered):
    po::detail::common_config_file_iterator(allowed_options, allow_unregistered) {

  this->is.reset(&is, po::detail::null_deleter());
  get();
}

// Parse the ASP stereo config file, such as stereo.default, from an open handle.
po::basic_parsed_options<char>
parse_asp_config_file(std::basic_istream<char>& is, const po::options_description& desc,
                      bool allow_unregistered) {

  std::set<std::string> allowed_options;
  auto & options = desc.options();
  for (size_t i = 0; i < options.size(); ++i) {
    const po::option_description& d = *options[i];

    if (d.long_name().empty())
      boost::throw_exception(po::error("long name required for config file"));

    allowed_options.insert(d.long_name());
  }

  // Parser return char strings
  po::parsed_options parsed(&desc);
  std::copy(asp_config_file_iterator(is, allowed_options, allow_unregistered),
            asp_config_file_iterator(),
            std::back_inserter(parsed.options));

  // Convert char strings into desired type
  return po::basic_parsed_options<char>(parsed);
}

// Parse the ASP stereo config file, such as stereo.default.
po::basic_parsed_options<char>
parse_asp_config_file(bool print_warning, std::string const& filename,
                      const po::options_description& desc, bool allow_unregistered) {

  std::basic_ifstream<char> strm(filename.c_str());
  if (print_warning) {
    if (!strm)
      vw::vw_out() << "Stereo file " << filename << " could not be found. "
                << "Will use default settings and command line options only.\n";
    else
      vw::vw_out() << "Loading stereo file: " << filename << "\n";
  }

  return parse_asp_config_file(strm, desc, allow_unregistered);
}

} // end namespace asp
