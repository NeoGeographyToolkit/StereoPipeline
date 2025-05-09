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


/// \file Common.h
///

#ifndef __ASP_CORE_COMMON_H__
#define __ASP_CORE_COMMON_H__

// These header files must be here as they define a lot of macros
// used in many tools, such as ASP_HAVE_PKG_ISIS.
#include <vw/config.h> // must come before asp_config.h, defines VW_BOOST_VERSION
#include <asp/asp_config.h>

// TODO(oalexan1): Break up Common.h into more manageable pieces.
// Image-writing code should be in VW. Low-level string utils
// should be moved to AspStringUtils.h. 

#include <vw/Core/StringUtils.h>
#include <vw/Image/ImageIO.h>
#include <vw/FileIO/DiskImageResourceGDAL.h>
#include <vw/FileIO/DiskImageView.h>
#include <vw/Math/Vector.h>
#include <vw/FileIO/FileUtils.h>
#include <vw/Image/ImageViewRef.h>
#include <vw/Cartography/GeoReference.h>
#include <vw/Cartography/GeoReferenceUtils.h>

#include <boost/program_options.hpp>
#include <boost/filesystem/path.hpp>
#include <boost/shared_ptr.hpp>

#include <map>
#include <string>
#include <bitset>

namespace vw {
  struct GdalWriteOptions;
}

namespace asp {

  class ASPGlobalOptions; // forward declaration
  
  /// Given a list of images/cameras and/or lists of such things, put the images
  /// and the cameras in separate vectors.
  void separate_images_from_cameras(std::vector<std::string> const& inputs,
				    std::vector<std::string>      & images,
				    std::vector<std::string>      & cameras,
				    bool ensure_equal_sizes);
  
  /// Parse 'VAR1=VAL1 VAR2=VAL2' into a map. Note that we append to the map,
  /// so it may have some items there beforehand.
  void parse_append_metadata(std::string const& metadata,
                             std::map<std::string, std::string> & keywords);
  
  /// Print time function
  std::string current_posix_time_string();

  /// Run a system command and append the output to a given file
  void run_cmd_app_to_file(std::string cmd, std::string file);

  /// Get program name without path and leading 'lt-'.
  std::string extract_prog_name(std::string const& prog_str);

  /// Write logs to a file
  void log_to_file(int argc, char *argv[],
                   std::string stereo_default_filename,
                   std::string output_prefix);

  /// Write a vector of strings from a file, one per line.
  void write_list(std::string const& file, std::vector<std::string> const & list);

  /// Read a vector of strings from a file, with spaces and newlines acting as separators.
  /// Throw an exception if the list is empty.
  void read_list(std::string const& file, std::vector<std::string> & list);
  
  /// Read a vector of doubles from a file  
  void read_vec(std::string const& filename, std::vector<double> & vals);

  /// Read the target name (planet name) from the plain text portion of an ISIS cub file
  std::string read_target_name(std::string const& filename);

  boost::program_options::variables_map
  check_command_line(int argc, char *argv[], vw::GdalWriteOptions& opt,
                     boost::program_options::options_description const& public_options,
                     boost::program_options::options_description const& all_public_options,
                     boost::program_options::options_description const& positional_options,
                     boost::program_options::positional_options_description
                     const& positional_desc,
                     std::string & usage_comment,
                     bool allow_unregistered, std::vector<std::string> & unregistered);

  /// Load multiple user options into a georef object.
  /// - This call supports more srs_string options than are possible
  ///   by loading a proj4 string into a GeoReference object, including
  ///   EPSG codes and URLs like http://spatialreference.org/ref/iau2000/49900/
  void set_srs_string(std::string srs_string, bool have_user_datum,
                      vw::cartography::Datum const& user_datum,
                      vw::cartography::GeoReference & georef);

  //---------------------------------------------------------------------------

} // end namespace asp

// Custom Boost Program Options validators for VW/ASP types
namespace boost {
namespace program_options {

  // Custom value semantics, these explain how many tokens should be ingested.

  // 2 params
  template <class T, class charT = char>
  class typed_2_value : public typed_value<T,charT> {
  public:
    typed_2_value(T* store_to) : typed_value<T,charT>(store_to) {
      this->multitoken();
    }

    unsigned min_tokens() const { return 2; }
    unsigned max_tokens() const { return 2; }
  };

  // 3 params
  template <class T, class charT = char>
  class typed_3_value : public typed_value<T,charT> {
  public:
    typed_3_value(T* store_to) : typed_value<T,charT>(store_to) {
      this->multitoken();
    }

    unsigned min_tokens() const { return 3; }
    unsigned max_tokens() const { return 3; }
  };

  // 4 params
  template <class T, class charT = char>
  class typed_4_value : public typed_value<T,charT> {
  public:
    typed_4_value(T* store_to) : typed_value<T,charT>(store_to) {
      this->multitoken();
    }

    unsigned min_tokens() const { return 4; }
    unsigned max_tokens() const { return 4; }
  };

  // 6 params  
  template <class T, class charT = char>
  class typed_6_value : public typed_value<T,charT> {
  public:
    typed_6_value(T* store_to) : typed_value<T,charT>(store_to) {
      this->multitoken();
    }

    unsigned min_tokens() const { return 6; }
    unsigned max_tokens() const { return 6; }
  };

  typed_2_value<vw::Vector2i>* value(vw::Vector2i* v);
  typed_2_value<vw::Vector2>*  value(vw::Vector2* v);
  typed_3_value<vw::Vector3i>* value(vw::Vector3i* v);
  typed_3_value<vw::Vector3>*  value(vw::Vector3* v);
  typed_4_value<vw::BBox2i>*   value(vw::BBox2i* v);
  typed_4_value<vw::BBox2>*    value(vw::BBox2* v);
  typed_6_value<vw::BBox3>*    value(vw::BBox3* v);

  // Custom validators which describe how text is turned into a value
  // 2 params
  template <>
  void validate(boost::any& v,
                 const std::vector<std::string>& values,
                 vw::Vector2i*, long);
  template <>
  void validate(boost::any& v,
                 const std::vector<std::string>& values,
                 vw::Vector2*, long);
  // 3 params
  template <>
  void validate(boost::any& v,
                 const std::vector<std::string>& values,
                 vw::Vector3i*, long);
  template <>
  void validate(boost::any& v,
                 const std::vector<std::string>& values,
                 vw::Vector3*, long);
  // 4 params
  template <>
  void validate(boost::any& v,
                 const std::vector<std::string>& values,
                 vw::BBox2i*, long);
  template <>
  void validate(boost::any& v,
                 const std::vector<std::string>& values,
                 vw::BBox2*, long);
  // 6 params
  template <>
  void validate(boost::any& v,
                 const std::vector<std::string>& values,
                 vw::BBox3*, long);

} // end namespace program_options
} // end namespace boost

#endif//__ASP_CORE_COMMON_H__
