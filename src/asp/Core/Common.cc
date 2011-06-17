// __BEGIN_LICENSE__
// Copyright (C) 2006-2011 United States Government as represented by
// the Administrator of the National Aeronautics and Space Administration.
// All Rights Reserved.
// __END_LICENSE__

#include <asp/Core/Common.h>

using namespace vw;
namespace po = boost::program_options;

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
    ("help,h", "Display this help message");
}

po::variables_map
asp::check_command_line( int argc, char *argv[], BaseOptions& opt,
                         po::options_description const& public_options,
                         po::options_description const& hidden_options,
                         po::positional_options_description const& positional,
                         std::string const& help ) {
  po::variables_map vm;
  try {
    po::options_description all_options;
    all_options.add(public_options).add(hidden_options);
    po::store( po::command_line_parser( argc, argv ).options(all_options).positional(positional).run(), vm );
    po::notify( vm );
  } catch (po::error const& e) {
    vw::vw_throw( vw::ArgumentErr() << "Error parsing input:\n"
                  << e.what() << "\n" << help << "\n" << public_options );
  }
  // We really don't want to use BIGTIFF unless we have to. It's
  // hard to find viewers for bigtiff.
  if ( vm.count("no-bigtiff") ) {
    opt.gdal_options["BIGTIFF"] = "NO";
  } else {
    opt.gdal_options["BIGTIFF"] = "IF_SAFER";
  }
  if ( vm.count("help") )
    vw::vw_throw( vw::ArgumentErr() << help << "\n" << public_options );
  if ( opt.num_threads != 0 ) {
    vw::vw_out() << "\t--> Setting number of processing threads to: "
                 << opt.num_threads << std::endl;
    vw::vw_settings().set_default_num_threads(opt.num_threads);
  }

  return vm;
}

bool asp::has_cam_extension( std::string const& input ) {
  boost::filesystem::path ipath( input );
  std::string ext = ipath.extension();
  if ( ext == ".cahvor" || ext == ".cahv" ||
       ext == ".pin" || ext == ".pinhole" ||
       ext == ".tsai" || ext == ".cmod" ||
       ext == ".cahvore" )
    return true;
  return false;
}
