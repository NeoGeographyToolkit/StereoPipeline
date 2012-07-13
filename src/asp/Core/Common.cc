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

#include <boost/date_time/posix_time/posix_time.hpp>

#include <vw/Core/Thread.h>
#include <vw/Core/Log.h>
#include <asp/Core/Common.h>

#include <vw/config.h>
#include <asp/asp_config.h>
#include <gdal_version.h>
#include <proj_api.h>

using namespace vw;
namespace po = boost::program_options;

// Print time function
std::string asp::current_posix_time_string() {
  return boost::posix_time::to_simple_string(boost::posix_time::second_clock::local_time());
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
    ("help,h", "Display this help message");
}

// User should only put the arguments to their application in the
// usage_comment argument. We'll finish filling in the repeated
// information;
po::variables_map
asp::check_command_line( int argc, char *argv[], BaseOptions& opt,
                         po::options_description const& public_options,
                         po::options_description const& hidden_options,
                         po::positional_options_description const& positional,
                         std::string & usage_comment ) {
  // Finish filling in the usage_comment;
  std::ostringstream ostr;
  ostr << "Usage: " << argv[0] << " " << usage_comment << "\n\n";
  ostr << "  [ASP " << ASP_VERSION << "]\n\n";
  usage_comment = ostr.str();

  po::variables_map vm;
  try {
    po::options_description all_options;
    all_options.add(public_options).add(hidden_options);
    po::store( po::command_line_parser( argc, argv ).options(all_options).positional(positional).allow_unregistered().run(), vm );
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

  template <>
  void validate( boost::any& v,
                 const std::vector<std::string>& values,
                 vw::Vector2i*, long ) {
    validators::check_first_occurrence(v);
    if ( values.size() != 1)
      boost::throw_exception(validation_error(validation_error::invalid_option_value));

    std::string key = boost::trim_copy(values[0]);
    std::vector<std::string> cvalues;
    boost::split( cvalues, key, is_any_of(", "),
                  boost::token_compress_on );

    try {
      Vector2i output( boost::lexical_cast<int32>( cvalues[0] ),
                       boost::lexical_cast<int32>( cvalues[1] ) );
      v = output;
    } catch (boost::bad_lexical_cast const& e ) {
      boost::throw_exception(validation_error(validation_error::invalid_option_value));
    }
  }
  template <>
  void validate( boost::any& v,
                 const std::vector<std::string>& values,
                 vw::BBox2i*, long ) {
    validators::check_first_occurrence(v);
    if ( values.size() != 1 )
      boost::throw_exception(validation_error(validation_error::invalid_option_value));

    std::string key = boost::trim_copy(values[0]);
    std::vector<std::string> cvalues;
    boost::split( cvalues, key, is_any_of(", "),
                  boost::token_compress_on );

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
  template <>
  void validate( boost::any& v,
                 const std::vector<std::string>& values,
                 vw::BBox2*, long ) {
    validators::check_first_occurrence(v);
    if ( values.size() != 1 )
      boost::throw_exception(validation_error(validation_error::invalid_option_value));

    std::string key = boost::trim_copy(values[0]);
    std::vector<std::string> cvalues;
    boost::split( cvalues, key, is_any_of(", "),
                  boost::token_compress_on );

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

}}
