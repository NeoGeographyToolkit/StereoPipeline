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

  // Print time function
  std::string current_posix_time_string();

  // Standard Options
  struct BaseOptions {
    vw::DiskImageResourceGDAL::Options gdal_options;
    vw::Vector2i raster_tile_size;
    vw::uint32 num_threads;
    std::string cache_dir;
    std::string tif_compress;

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
                      std::string & usage_comment );

  bool has_cam_extension( std::string const& input );

  vw::Vector2i file_image_size( std::string const& input );

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

// Custom Boost Program Options validators for VW/ASP types
namespace boost {
namespace program_options {

  template <>
  void validate( boost::any& v,
                 const std::vector<std::string>& values,
                 vw::Vector2i*, long );
  template <>
  void validate( boost::any& v,
                 const std::vector<std::string>& values,
                 vw::BBox2i*, long );

  template <>
  void validate( boost::any& v,
                 const std::vector<std::string>& values,
                 vw::BBox2*, long );
}}


#endif//__ASP_CORE_COMMON_H__
