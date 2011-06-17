// __BEGIN_LICENSE__
// Copyright (C) 2006-2011 United States Government as represented by
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
#include <vw/Cartography/GeoReference.h>

namespace asp {

  // Standard Options
  struct BaseOptions {
    vw::DiskImageResourceGDAL::Options gdal_options;
    vw::Vector2i raster_tile_size;
    vw::uint32 num_threads;

    BaseOptions();
  };

  // An object to let Program Options know about our standard options
  struct BaseOptionsDescription : public boost::program_options::options_description {
    BaseOptionsDescription( BaseOptions& opt);
  };

  boost::program_options::variables_map
  check_command_line( int argc, char *argv[], BaseOptions& opt,
                      boost::program_options::options_description const& public_options,
                      boost::program_options::options_description const& hidden_options,
                      boost::program_options::positional_options_description const& positional,
                      std::string const& help );

  bool has_cam_extension( std::string const& input );

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
    boost::scoped_ptr<vw::DiskImageResourceGDAL> rsrc( build_gdal_rsrc( filename, image, opt ) );
    vw::block_write_image( *rsrc, image.impl(), progress_callback );
  }

  template <class ImageT>
  void block_write_gdal_image( const std::string &filename,
                               vw::ImageViewBase<ImageT> const& image,
                               vw::cartography::GeoReference const& georef,
                               BaseOptions const& opt,
                               vw::ProgressCallback const& progress_callback = vw::ProgressCallback::dummy_instance() ) {
    boost::scoped_ptr<vw::DiskImageResourceGDAL> rsrc( build_gdal_rsrc( filename, image, opt ) );
    vw::cartography::write_georeference(*rsrc, georef);
    vw::block_write_image( *rsrc, image.impl(), progress_callback );
  }

  template <class ImageT>
  void write_gdal_image( const std::string &filename,
                         vw::ImageViewBase<ImageT> const& image,
                         BaseOptions const& opt,
                         vw::ProgressCallback const& progress_callback = vw::ProgressCallback::dummy_instance() ) {
    boost::scoped_ptr<vw::DiskImageResourceGDAL> rsrc( build_gdal_rsrc( filename, image, opt ) );
    vw::write_image( *rsrc, image.impl(), progress_callback );
  }

  template <class ImageT>
  void write_gdal_georeferenced_image( const std::string &filename,
                                       vw::ImageViewBase<ImageT> const& image,
                                       vw::cartography::GeoReference const& georef,
                                       BaseOptions const& opt,
                                       vw::ProgressCallback const& progress_callback = vw::ProgressCallback::dummy_instance() ) {
    boost::scoped_ptr<vw::DiskImageResourceGDAL> rsrc( build_gdal_rsrc( filename, image, opt ) );
    vw::cartography::write_georeference(*rsrc, georef);
    vw::write_image( *rsrc, image.impl(), progress_callback );
  }

  template <class ImageT>
  void write_gdal_image( const std::string &filename,
                         vw::ImageViewBase<ImageT> const& image,
                         vw::cartography::GeoReference const& georef,
                         BaseOptions const& opt,
                         vw::ProgressCallback const& progress_callback = vw::ProgressCallback::dummy_instance() ) {
    write_gdal_georeferenced_image( filename, image.impl(), georef, opt,
                                    progress_callback );
  }

}

#endif//__ASP_CORE_COMMON_H__
