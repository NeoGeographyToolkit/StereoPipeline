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

#include <vw/config.h> // must come before asp_config.h, defines VW_BOOST_VERSION
#include <asp/asp_config.h>

#include <boost/program_options.hpp>
#include <boost/filesystem/path.hpp>
#include <boost/shared_ptr.hpp>
#include <vw/Image/ImageIO.h>
#include <vw/FileIO/DiskImageResourceGDAL.h>
#include <vw/FileIO/DiskImageView.h>
#include <vw/FileIO/DiskImageUtils.h>
#include <vw/Math/Vector.h>
#include <vw/FileIO/FileUtils.h>
#include <vw/Image/ImageViewRef.h>
#include <vw/Cartography/GeoReference.h>
#include <map>
#include <string>

namespace asp {

  /// Pack a Vector into a string
  template <class VecT>
  std::string vec_to_str(VecT const& vec);

  /// Extract a string into a Vector of given size.
  template<class VecT>
  VecT str_to_vec(std::string const& str);

  /// Returns true if the file has an extension which can contain a camera model
  bool has_cam_extension( std::string const& input );

  /// Returns true if the file has an extension which can contain a pinhole camera model
  bool has_pinhole_extension( std::string const& input );

  /// Returns true if the file has an extension which can contain an image
  bool has_image_extension( std::string const& input );

  /// Returns true if the file has an extension which is tif or ntf
  bool has_tif_or_ntf_extension( std::string const& input );

  /// Parse the list of files specified as positional arguments on the command lin
  bool parse_multiview_cmd_files(std::vector<std::string> const &filesIn,
                                 std::vector<std::string>       &image_paths,
                                 std::vector<std::string>       &camera_paths,
                                 std::string                    &prefix,
                                 std::string                    &dem_path);


  std::string bundle_adjust_file_name(std::string const& prefix, std::string const& input_img,
                                      std::string const& input_cam);

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

  /// Standard Options
  struct BaseOptions {
    vw::DiskImageResourceGDAL::Options gdal_options;
    vw::Vector2i raster_tile_size;
    vw::int32    num_threads;
    std::string  cache_dir;
    std::string  tif_compress;
    static int   corr_tile_size() { return 1024; } // Tile size for correlation
    static int   rfne_tile_size() { return 256;  } // Tile size for refinement
    static int   tri_tile_size()  { return 256;  } // Tile size for tri/point cloud
    BaseOptions();
  };

  /// An object to let Program Options know about our standard options
  struct BaseOptionsDescription : public boost::program_options::options_description {
    BaseOptionsDescription( BaseOptions& opt);
  };

  boost::program_options::variables_map
  check_command_line( int argc, char *argv[], BaseOptions& opt,
                      boost::program_options::options_description const& public_options,
                      boost::program_options::options_description const& all_public_options,
                      boost::program_options::options_description const& positional_options,
                      boost::program_options::positional_options_description const& positional_desc,
                      std::string & usage_comment,
                      bool allow_unregistered, std::vector<std::string> & unregistered);



  vw::Vector2i file_image_size( std::string const& input );

  void set_srs_string(std::string srs_string, bool have_user_datum,
                      vw::cartography::Datum const& user_datum,
                      vw::cartography::GeoReference & georef);

  //---------------------------------------------------------------------------
  // Functions for writing GDAL images - multithreaded.

  /// String we use in ASP written point cloud files to indicate that an offset
  ///  has been subtracted out from the points.
  // Note: We use this constant in the python code as well
  const std::string ASP_POINT_OFFSET_TAG_STR = "POINT_OFFSET";

  template <class ImageT>
  vw::DiskImageResourceGDAL*
  build_gdal_rsrc( const std::string &filename,
                   vw::ImageViewBase<ImageT> const& image,
                   BaseOptions const& opt ) {
    return new vw::DiskImageResourceGDAL(filename, image.impl().format(),
                                         opt.raster_tile_size, opt.gdal_options);
  }

  /// Multi-threaded block write image with, if available, nodata, georef, and
  /// keywords to geoheader.
  template <class ImageT>
  void block_write_gdal_image(const std::string &filename,
                              vw::ImageViewBase<ImageT> const& image,
                              bool has_georef,
                              vw::cartography::GeoReference const& georef,
                              bool has_nodata, double nodata,
                              BaseOptions const& opt,
                              vw::ProgressCallback const& progress_callback =
                              vw::ProgressCallback::dummy_instance(),
                              std::map<std::string, std::string> const& keywords =
                              std::map<std::string, std::string>() );

  /// Block write image without georef and nodata.
  template <class ImageT>
  void block_write_gdal_image(const std::string &filename,
                              vw::ImageViewBase<ImageT> const& image,
                              BaseOptions const& opt,
                              vw::ProgressCallback const& progress_callback =
                              vw::ProgressCallback::dummy_instance(),
                              std::map<std::string, std::string> const& keywords =
                              std::map<std::string, std::string>() );

  /// Block write image with nodata.
  template <class ImageT>
  void block_write_gdal_image(const std::string &filename,
                              vw::ImageViewBase<ImageT> const& image,
                              double nodata,
                              BaseOptions const& opt,
                              vw::ProgressCallback const& progress_callback =
                              vw::ProgressCallback::dummy_instance(),
                              std::map<std::string, std::string> const& keywords =
                              std::map<std::string, std::string>() );

  //---------------------------------------------------------------------------
  // Functions for writing GDAL images - single threaded.

  /// Single-threaded write image with, if available, nodata, georef, and
  /// keywords to geoheader.
  template <class ImageT>
  void write_gdal_image(const std::string &filename,
                        vw::ImageViewBase<ImageT> const& image,
                        bool has_georef,
                        vw::cartography::GeoReference const& georef,
                        bool has_nodata, double nodata,
                        BaseOptions const& opt,
                        vw::ProgressCallback const& progress_callback
                        = vw::ProgressCallback::dummy_instance(),
                        std::map<std::string, std::string> const& keywords
                        = std::map<std::string, std::string>() );


  /// Single-threaded write image with georef and keywords to geoheader.
  template <class ImageT>
  void write_gdal_image(const std::string &filename,
                        vw::ImageViewBase<ImageT> const& image,
                        vw::cartography::GeoReference const& georef,
                        BaseOptions const& opt,
                        vw::ProgressCallback const& progress_callback
                        = vw::ProgressCallback::dummy_instance(),
                        std::map<std::string, std::string> const& keywords
                        = std::map<std::string, std::string>() );

  /// Single-threaded write image with georef, nodata, and keywords to geoheader.
  template <class ImageT>
  void write_gdal_image(const std::string &filename,
                        vw::ImageViewBase<ImageT> const& image,
                        vw::cartography::GeoReference const& georef,
                        double nodata,
                        BaseOptions const& opt,
                        vw::ProgressCallback const& progress_callback
                        = vw::ProgressCallback::dummy_instance(),
                        std::map<std::string, std::string> const& keywords
                        = std::map<std::string, std::string>() );

  /// Single-threaded write image.
  template <class ImageT>
  void write_gdal_image(const std::string &filename,
                        vw::ImageViewBase<ImageT> const& image,
                        BaseOptions const& opt,
                        vw::ProgressCallback const& progress_callback
                        = vw::ProgressCallback::dummy_instance(),
                        std::map<std::string, std::string> const& keywords
                        = std::map<std::string, std::string>() );


  //---------------------------------------------------------------------------

  // Specialized functions for reading/writing images with a shift.
  // The shift is meant to bring the pixel values closer to origin,
  // with goal of saving the pixels as float instead of double.

  /// Subtract a given shift from first 3 components of given vector image.
  /// Skip pixels for which the first 3 components are (0, 0, 0).
  template <class VecT>
  struct SubtractShift: public vw::ReturnFixedType<VecT> {
    vw::Vector3 m_shift;
    SubtractShift(vw::Vector3 const& shift):m_shift(shift){}
    VecT operator() (VecT const& pt) const {
      VecT lpt = pt;
      int len = std::min(3, (int)lpt.size());
      if (subvector(lpt, 0, len) != subvector(vw::Vector3(), 0, len))
        subvector(lpt, 0, len) -= subvector(m_shift, 0, len);
      return lpt;
    }
  };
  template <class ImageT>
  vw::UnaryPerPixelView<ImageT, SubtractShift<typename ImageT::pixel_type> >
  inline subtract_shift( vw::ImageViewBase<ImageT> const& image,
                         vw::Vector3 const& shift ) {
    return vw::UnaryPerPixelView<ImageT, SubtractShift<typename ImageT::pixel_type> >
      ( image.impl(), SubtractShift<typename ImageT::pixel_type>(shift) );
  }

  /// Round pixels in given image to multiple of given rounding_error.
  template <class VecT>
  struct RoundImagePixels: public vw::ReturnFixedType<VecT> {
    double m_rounding_error;
    RoundImagePixels(double rounding_error):m_rounding_error(rounding_error){
      VW_ASSERT( m_rounding_error > 0.0,
                 vw::ArgumentErr() << "Rounding error must be positive.");
    }
    VecT operator() (VecT const& pt) const {
      return m_rounding_error*round(pt/m_rounding_error);
    }
  };
  template <class ImageT>
  vw::UnaryPerPixelView<ImageT, RoundImagePixels<typename ImageT::pixel_type> >
  inline round_image_pixels( vw::ImageViewBase<ImageT> const& image,
                         double rounding_error ) {
    return vw::UnaryPerPixelView<ImageT, RoundImagePixels<typename ImageT::pixel_type> >
      ( image.impl(), RoundImagePixels<typename ImageT::pixel_type>(rounding_error) );
  }


  /// To help with compression, round to about 1mm, but
  /// use for rounding a number with few digits in binary.
  const double APPROX_ONE_MM = 1.0/1024.0;

  /// Don't round pixels in point2dem for bodies of radius smaller than
  /// this in meters. Do it though in stereo_tri, see get_rounding_error().
  const double MIN_RADIUS_FOR_ROUNDING = 1e+6; // 1000 km

  /// Unless user-specified, compute the rounding error for a given
  /// planet (a point on whose surface is given by 'shift'). Return an
  /// inverse power of 2, 1/2^10 for Earth and proportionally less for smaller bodies.
  double get_rounding_error(vw::Vector3 const& shift, double rounding_error);

  /// Block write image while subtracting a given value from all pixels
  /// and casting the result to float, while rounding to nearest mm.
  template <class ImageT>
  void block_write_approx_gdal_image(const std::string &filename,
                                     vw::Vector3 const& shift,
                                     double rounding_error,
                                     vw::ImageViewBase<ImageT> const& image,
                                     bool has_georef,
                                     vw::cartography::GeoReference const& georef,
                                     bool has_nodata, double nodata,
                                     BaseOptions const& opt,
                                     vw::ProgressCallback const& progress_callback
                                     = vw::ProgressCallback::dummy_instance(),
                                     std::map<std::string, std::string> const& keywords =
                                     std::map<std::string, std::string>() );


  /// Single-threaded write image while subtracting a given value from
  /// all pixels and casting the result to float.
  template <class ImageT>
  void write_approx_gdal_image(const std::string &filename,
                               vw::Vector3 const& shift,
                               double rounding_error,
                               vw::ImageViewBase<ImageT> const& image,
                               bool has_georef,
                               vw::cartography::GeoReference const& georef,
                               bool has_nodata, double nodata,
                               BaseOptions const& opt,
                               vw::ProgressCallback const& progress_callback
                               = vw::ProgressCallback::dummy_instance(),
                               std::map<std::string, std::string> const& keywords =
                               std::map<std::string, std::string>() );


  /// Often times, we'd like to save an image to disk by using big
  /// blocks, for performance reasons, then re-write it with desired blocks.
  template <class ImageT>
  void save_with_temp_big_blocks(int big_block_size,
                                 const std::string &filename,
                                 vw::ImageViewBase<ImageT> const& img,
                                 vw::cartography::GeoReference const& georef,
                                 double nodata,
                                 BaseOptions & opt,
                                 vw::ProgressCallback const& tpc);

} // end namespace asp

// Custom Boost Program Options validators for VW/ASP types
namespace boost {
namespace program_options {

  // Custom value semantics, these explain how many tokens should be ingested.

  template <class T, class charT = char>
  class typed_2_value : public typed_value<T,charT> {
  public:
    typed_2_value(T* store_to) : typed_value<T,charT>(store_to) {
      this->multitoken();
    }

    unsigned min_tokens() const { return 2; }
    unsigned max_tokens() const { return 2; }
  };

  template <class T, class charT = char>
  class typed_4_value : public typed_value<T,charT> {
  public:
    typed_4_value(T* store_to) : typed_value<T,charT>(store_to) {
      this->multitoken();
    }

    unsigned min_tokens() const { return 4; }
    unsigned max_tokens() const { return 4; }
  };

  template <class T, class charT = char>
  class typed_6_value : public typed_value<T,charT> {
  public:
    typed_6_value(T* store_to) : typed_value<T,charT>(store_to) {
      this->multitoken();
    }

    unsigned min_tokens() const { return 6; }
    unsigned max_tokens() const { return 6; }
  };

  typed_2_value<vw::Vector2i>* value( vw::Vector2i* v );
  typed_2_value<vw::Vector2>*  value( vw::Vector2* v );
  typed_4_value<vw::BBox2i>*   value( vw::BBox2i* v );
  typed_4_value<vw::BBox2>*    value( vw::BBox2* v );
  typed_6_value<vw::BBox3>*    value( vw::BBox3* v );

  // Custom validators which describe how text is turned into a value
  template <>
  void validate( boost::any& v,
                 const std::vector<std::string>& values,
                 vw::Vector2i*, long );

  template <>
  void validate( boost::any& v,
                 const std::vector<std::string>& values,
                 vw::Vector2*, long );

  template <>
  void validate( boost::any& v,
                 const std::vector<std::string>& values,
                 vw::BBox2i*, long );

  template <>
  void validate( boost::any& v,
                 const std::vector<std::string>& values,
                 vw::BBox2*, long );

  template <>
  void validate( boost::any& v,
                 const std::vector<std::string>& values,
                 vw::BBox3*, long );

} // end namespace program_options
} // end nampspace boost


#include <asp/Core/Common.tcc>

#endif//__ASP_CORE_COMMON_H__
