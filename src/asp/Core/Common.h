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

#include <boost/program_options.hpp>
#include <boost/filesystem/path.hpp>
#include <boost/shared_ptr.hpp>
#include <vw/Image/ImageIO.h>
#include <vw/FileIO/DiskImageResourceGDAL.h>
#include <vw/FileIO/DiskImageView.h>
#include <vw/Math/Vector.h>
#include <vw/Image/ImageViewRef.h>
#include <vw/Cartography/GeoReference.h>
#include <map>
#include <string>

namespace asp {

  // Pack a Vector into a string
  template <class VecT>
  std::string vec_to_str(VecT const& vec){

    std::ostringstream oss;
    oss.precision(16);
    for (int i = 0; i < (int)vec.size(); i++)
      oss << vec[i] << " ";

    return oss.str();
  }

  // Extract a string into a Vector of given size.
  template<class VecT>
  VecT str_to_vec(std::string const& str){

    VecT vec;
    std::istringstream iss(str);
    for (int i = 0; i < (int)vec.size(); i++){
      if (! (iss >> vec[i]) )
        vw::vw_throw( vw::ArgumentErr() << "Could not extract xyz point from: "
                      << str << "\n");
    }
    return vec;
  }

  boost::filesystem::path make_file_relative_to_dir
  (boost::filesystem::path const file, boost::filesystem::path const dir);
  
  // Remove file name extension
  std::string prefix_from_filename(std::string const& filename);

  // Print time function
  std::string current_posix_time_string();

  // If prefix is "dir/out", create directory "dir"
  void create_out_dir(std::string out_prefix);

  // Imageview operator that extracts the first m channels
  // starting at channel k of an image with n channels.
  template <int k, int m, int n>
  struct SelectPoints : public vw::ReturnFixedType< vw::Vector<double, m> > {
    vw::Vector<double, m> operator() (vw::Vector<double, n> const& pt) const {
      return subvector(pt,k,m);
    }
  };

  template <int k, int m, int n>
  vw::UnaryPerPixelView<vw::DiskImageView< vw::Vector<double, n> >,
                        SelectPoints<k, m, n> >
  inline select_points( vw::ImageViewBase<vw::DiskImageView< vw::Vector<double, n> > > const& image ) {
    return vw::UnaryPerPixelView<vw::DiskImageView< vw::Vector<double, n> >,
      SelectPoints<k, m, n> >( image.impl(), SelectPoints<k, m, n>() );
  }

  // Find how many channels/bands are in a given image
  int get_num_channels(std::string filename);

  // Run a system command and append the output to a given file
  void run_cmd_app_to_file(std::string cmd, std::string file);
  
  // Write logs to a file
  void log_to_file(int argc, char *argv[],
                   std::string stereo_default_filename,
                   std::string output_prefix);
  
  // Standard Options
  struct BaseOptions {
    vw::DiskImageResourceGDAL::Options gdal_options;
    vw::Vector2i raster_tile_size;
    vw::uint32 num_threads;
    std::string cache_dir;
    std::string tif_compress;
    static int corr_tile_size() { return 1024; } // Tile size for correlation
    static int rfne_tile_size() { return 256;  } // Tile size for refinement
    static int tri_tile_size()  { return 256;  } // Tile size for tri/point cloud
    BaseOptions();
  };

  // An object to let Program Options know about our standard options
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
                      bool allow_unregistered = false );

  bool has_cam_extension( std::string const& input );

  vw::Vector2i file_image_size( std::string const& input );

  template <class ImageT>
  vw::DiskImageResourceGDAL*
  build_gdal_rsrc( const std::string &filename,
                   vw::ImageViewBase<ImageT> const& image,
                   BaseOptions const& opt ) {
    return new vw::DiskImageResourceGDAL(filename, image.impl().format(), opt.raster_tile_size, opt.gdal_options);
  }

  // Block write image.
  template <class ImageT>
  void block_write_gdal_image( const std::string &filename,
                               vw::ImageViewBase<ImageT> const& image,
                               BaseOptions const& opt,
                               vw::ProgressCallback const& progress_callback = vw::ProgressCallback::dummy_instance() ) {
    boost::scoped_ptr<vw::DiskImageResourceGDAL> rsrc( build_gdal_rsrc( filename, image, opt ) );
    vw::block_write_image( *rsrc, image.impl(), progress_callback );
  }

  // Block write image with georef and keywords to geoheader.
  template <class ImageT>
  void block_write_gdal_image( const std::string &filename,
                               vw::ImageViewBase<ImageT> const& image,
                               vw::cartography::GeoReference const& georef,
                               BaseOptions const& opt,
                               vw::ProgressCallback const& progress_callback = vw::ProgressCallback::dummy_instance(),
                               std::map<std::string, std::string> keywords =
                               std::map<std::string, std::string>()
                               ) {
    boost::scoped_ptr<vw::DiskImageResourceGDAL> rsrc( build_gdal_rsrc( filename, image, opt ) );
    for (std::map<std::string, std::string>::iterator i = keywords.begin();
         i != keywords.end(); i++){
      vw::cartography::write_header_string(*rsrc, i->first, i->second);
    }    
    vw::cartography::write_georeference(*rsrc, georef);
    vw::block_write_image( *rsrc, image.impl(), progress_callback );
  }

  // Block write image with nodata.
  template <class ImageT, class NoDataT>
  void block_write_gdal_image( const std::string &filename,
                               vw::ImageViewBase<ImageT> const& image,
                               NoDataT nodata,
                               BaseOptions const& opt,
                               vw::ProgressCallback const& progress_callback = vw::ProgressCallback::dummy_instance() ) {
    boost::scoped_ptr<vw::DiskImageResourceGDAL> rsrc( build_gdal_rsrc( filename, image, opt ) );
    rsrc->set_nodata_write(nodata);
    vw::block_write_image( *rsrc, image.impl(), progress_callback );
  }

  // Block write image with nodata, georef, and keywords to geoheader.
  template <class ImageT, class NoDataT>
  void block_write_gdal_image( const std::string &filename,
                               vw::ImageViewBase<ImageT> const& image,
                               vw::cartography::GeoReference const& georef,
                               NoDataT nodata,
                               BaseOptions const& opt,
                               vw::ProgressCallback const& progress_callback =
                               vw::ProgressCallback::dummy_instance(),
                               std::map<std::string, std::string> keywords =
                               std::map<std::string, std::string>()
                               ) {
    boost::scoped_ptr<vw::DiskImageResourceGDAL> rsrc( build_gdal_rsrc( filename, image, opt ) );
    rsrc->set_nodata_write(nodata);

    for (std::map<std::string, std::string>::iterator i = keywords.begin();
         i != keywords.end(); i++){
      vw::cartography::write_header_string(*rsrc, i->first, i->second);
    }    
    
    vw::cartography::write_georeference(*rsrc, georef);
    vw::block_write_image( *rsrc, image.impl(), progress_callback );
  }

  // Single-threaded write functions.

  // Write image with georef and keywords to geoheader.
  template <class ImageT>
  void write_gdal_georeferenced_image( const std::string &filename,
                                       vw::ImageViewBase<ImageT> const& image,
                                       vw::cartography::GeoReference const& georef,
                                       BaseOptions const& opt,
                                       vw::ProgressCallback const& progress_callback = vw::ProgressCallback::dummy_instance(),
                                       std::map<std::string, std::string> keywords = std::map<std::string, std::string>()
                                       ) {
    boost::scoped_ptr<vw::DiskImageResourceGDAL> rsrc( build_gdal_rsrc( filename, image, opt ) );
    for (std::map<std::string, std::string>::iterator i = keywords.begin();
         i != keywords.end(); i++){
      vw::cartography::write_header_string(*rsrc, i->first, i->second);
    }    
    vw::cartography::write_georeference(*rsrc, georef);
    vw::write_image( *rsrc, image.impl(), progress_callback );
  }

  // Write image with georef, nodata, and keywords to geoheader.
  template <class ImageT, class NoDataT>
  void write_gdal_georeferenced_image( const std::string &filename,
                                       vw::ImageViewBase<ImageT> const& image,
                                       vw::cartography::GeoReference const& georef,
                                       NoDataT nodata,
                                       BaseOptions const& opt,
                                       vw::ProgressCallback const& progress_callback = vw::ProgressCallback::dummy_instance(),
                                       std::map<std::string, std::string> keywords = std::map<std::string, std::string>()
                                       ) {
    boost::scoped_ptr<vw::DiskImageResourceGDAL> rsrc( build_gdal_rsrc( filename, image, opt ) );
    rsrc->set_nodata_write(nodata);
    for (std::map<std::string, std::string>::iterator i = keywords.begin();
         i != keywords.end(); i++){
      vw::cartography::write_header_string(*rsrc, i->first, i->second);
    }    
    vw::cartography::write_georeference(*rsrc, georef);
    vw::write_image( *rsrc, image.impl(), progress_callback );
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
  void write_gdal_image( const std::string &filename,
                         vw::ImageViewBase<ImageT> const& image,
                         vw::cartography::GeoReference const& georef,
                         BaseOptions const& opt,
                         vw::ProgressCallback const& progress_callback = vw::ProgressCallback::dummy_instance() ) {
    write_gdal_georeferenced_image( filename, image.impl(), georef, opt,
                                    progress_callback );
  }

  // Specialized functions for reading/writing images with a shift.
  // The shift is meant to bring the pixel values closer to origin,
  // with goal of saving the pixels as float instead of double.

  // Subtract a given shift from first 3 components of given vector image.
  // Skip pixels for which the first 3 components are (0, 0, 0).
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

  // Round pixels in given image to multiple of given rounding_error.
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

  // Note: We use this constant in the python code as well
  const std::string POINT_OFFSET = "POINT_OFFSET";

  // Given an image with n channels, return the first m channels.
  // We must have 1 <= m <= n <= 6.
  // If the image was written by subtracting a shift, put that shift
  // back.
  template<int m>
  vw::ImageViewRef< vw::Vector<double, m> > read_cloud(std::string filename){
    
    int max_n = 6;
    int n = get_num_channels(filename);

    vw::Vector3 shift;
    std::string shift_str;
    boost::shared_ptr<vw::DiskImageResource> rsrc ( new vw::DiskImageResourceGDAL(filename) );
    if (vw::cartography::read_header_string(*rsrc.get(), POINT_OFFSET, shift_str)){
      shift = str_to_vec<vw::Vector3>(shift_str);
    }

    VW_ASSERT( 1 <= m,
               vw::ArgumentErr() << "Attempting to read " << m
               << " channel(s) from an image.");
    VW_ASSERT( m <= n,
               vw::ArgumentErr() << "Attempting to read up to channel " << m
               << " from an image with " << n << " channel(s).");
    VW_ASSERT( n <= max_n,
               vw::NoImplErr() << "Reading from images with more than "
               << max_n << " channels is not implemented.");

    vw::ImageViewRef< vw::Vector<double, m> > out_image;
    if      (n == 1) out_image = select_points<0, m, 1>(vw::DiskImageView< vw::Vector<double, 1> >(filename));
    else if (n == 2) out_image = select_points<0, m, 2>(vw::DiskImageView< vw::Vector<double, 2> >(filename));
    else if (n == 3) out_image = select_points<0, m, 3>(vw::DiskImageView< vw::Vector<double, 3> >(filename));
    else if (n == 4) out_image = select_points<0, m, 4>(vw::DiskImageView< vw::Vector<double, 4> >(filename));
    else if (n == 5) out_image = select_points<0, m, 5>(vw::DiskImageView< vw::Vector<double, 5> >(filename));
    else if (n == 6) out_image = select_points<0, m, 6>(vw::DiskImageView< vw::Vector<double, 6> >(filename));

    // Subtract the point cloud shift from the several first channels
    out_image = subtract_shift(out_image, -shift);
    
    return out_image;
  }

  // To help with compression, round to about 1mm, but
  // use for rounding a number with few digits in binary.
  const double APPROX_ONE_MM = 1.0/1024.0;

  // Block write image while subtracting a given value from all pixels
  // and casting the result to float, while rounding to nearest mm.
  template <class ImageT>
  void block_write_approx_gdal_image( const std::string &filename,
                                      vw::Vector3 const& shift,
                                      double rounding_error,
                                      vw::ImageViewBase<ImageT> const& image,
                                      BaseOptions const& opt,
                                      vw::ProgressCallback const& progress_callback
                                      = vw::ProgressCallback::dummy_instance() ) {


    if (shift != vw::Vector3()){
      boost::scoped_ptr<vw::DiskImageResourceGDAL>
        rsrc( build_gdal_rsrc( filename,
                               vw::channel_cast<float>(image.impl()),
                               opt));
      vw::cartography::write_header_string(*rsrc, POINT_OFFSET, vec_to_str(shift));
      vw::block_write_image( *rsrc,
                             vw::channel_cast<float>
                             (round_image_pixels(subtract_shift(image.impl(),
                                                                shift),
                                                 rounding_error)),
                             progress_callback );
    }else{
      boost::scoped_ptr<vw::DiskImageResourceGDAL>
        rsrc( build_gdal_rsrc( filename, image, opt ) );
      vw::block_write_image( *rsrc, image.impl(), progress_callback );
    }

  }

  // Write image using a single thread while subtracting a given value
  // from all pixels and casting the result to float.
  template <class ImageT>
  void write_approx_gdal_image( const std::string &filename,
                                vw::Vector3 const& shift,
                                double rounding_error,
                                vw::ImageViewBase<ImageT> const& image,
                                BaseOptions const& opt,
                                vw::ProgressCallback const& progress_callback
                                = vw::ProgressCallback::dummy_instance() ) {


    if (shift != vw::Vector3()){
      boost::scoped_ptr<vw::DiskImageResourceGDAL>
        rsrc( build_gdal_rsrc( filename,
                               vw::channel_cast<float>(image.impl()),
                               opt ) );
      vw::cartography::write_header_string(*rsrc, POINT_OFFSET, vec_to_str(shift));
      vw::write_image( *rsrc,
                       vw::channel_cast<float>
                       (round_image_pixels(subtract_shift(image.impl(),
                                                          shift),
                                           rounding_error)),
                       progress_callback );
    }else{
      boost::scoped_ptr<vw::DiskImageResourceGDAL>
        rsrc( build_gdal_rsrc( filename, image, opt ) );
      vw::write_image( *rsrc, image.impl(), progress_callback );
    }
  }

} // namespace asp

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

}}


#endif//__ASP_CORE_COMMON_H__
