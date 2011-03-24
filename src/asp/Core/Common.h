// __BEGIN_LICENSE__
// Copyright (C) 2006-2010 United States Government as represented by
// the Administrator of the National Aeronautics and Space Administration.
// All Rights Reserved.
// __END_LICENSE__

/// \file Common.h
///

#ifndef __ASP_CORE_COMMON_H__
#define __ASP_CORE_COMMON_H__

#include <boost/program_options.hpp>
#include <boost/filesystem/path.hpp>
#include <boost/shared_ptr.hpp>
#include <vw/Image/ImageIO.h>
#include <vw/FileIO/DiskImageResourceGDAL.h>
#include <vw/Math/Vector.h>

namespace asp {

  struct BaseOptions {
    vw::DiskImageResourceGDAL::Options gdal_options;
    vw::Vector2i raster_tile_size;
    vw::uint32 num_threads;

    BaseOptions() {
#if defined(VW_HAS_BIGTIFF) && VW_HAS_BIGTIFF == 1
      gdal_options["COMPRESS"] = "LZW";
#else
      gdal_options["COMPRESS"] = "NONE";
      gdal_options["BIGTIFF"] = "NO";
#endif
      raster_tile_size =
        vw::Vector2i(vw::vw_settings().default_tile_size(),
                     vw::vw_settings().default_tile_size());
    }
  };

  struct BaseOptionsDescription : public boost::program_options::options_description {
    BaseOptionsDescription( BaseOptions& opt) {
      namespace po = boost::program_options;
      (*this).add_options()
        ("threads", po::value(&opt.num_threads)->default_value(0),
         "Select the number of processors (threads) to use.")
        ("no-bigtiff", "Tell GDAL to not create bigtiffs.")
        ("help,h", "Display this help message");
    }
  };

  boost::program_options::variables_map
  check_command_line( int argc, char *argv[], BaseOptions& opt,
                      boost::program_options::options_description const& public_options,
                      boost::program_options::options_description const& hidden_options,
                      boost::program_options::positional_options_description const& positional,
                      std::string const& help ) {
    namespace po = boost::program_options;
    po::variables_map vm;
    try {
      po::options_description all_options;
      all_options.add(public_options).add(hidden_options);
      po::store( po::command_line_parser( argc, argv ).options(all_options).positional(positional).run(), vm );
      po::notify( vm );
    } catch (po::error &e) {
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

  bool has_cam_extension( std::string input ) {
    boost::filesystem::path ipath( input );
    std::string ext = ipath.extension();
    if ( ext == ".cahvor" || ext == ".cahv" ||
         ext == ".pin" || ext == ".pinhole" ||
         ext == ".tsai" || ext == ".cmod" )
      return true;
    return false;
  }

  template <class ImageT>
  vw::DiskImageResourceGDAL*
  build_gdal_rsrc( const std::string &filename,
                   vw::ImageViewBase<ImageT> const& image,
                   BaseOptions const& opt ) {
    return new vw::DiskImageResourceGDAL(filename, image.impl().format(), opt.raster_tile_size, opt.gdal_options);
  }

  template <class ImageT>
  void block_write_gdal_image( const std::string &filename,
                               vw::ImageViewBase<ImageT> const& image,
                               BaseOptions const& opt,
                               vw::ProgressCallback const& progress_callback = vw::ProgressCallback::dummy_instance() ) {
    vw::DiskImageResourceGDAL* rsrc = build_gdal_rsrc( filename, image, opt );
    vw::block_write_image( *rsrc, image.impl(), progress_callback );
    delete rsrc;
  }

  template <class ImageT>
  void write_gdal_image( const std::string &filename,
                         vw::ImageViewBase<ImageT> const& image,
                         BaseOptions const& opt,
                         vw::ProgressCallback const& progress_callback = vw::ProgressCallback::dummy_instance() ) {
    vw::DiskImageResourceGDAL* rsrc = build_gdal_rsrc( filename, image, opt );
    vw::write_image( *rsrc, image.impl(), progress_callback );
    delete rsrc;
  }

}

#endif//__ASP_CORE_COMMON_H__
