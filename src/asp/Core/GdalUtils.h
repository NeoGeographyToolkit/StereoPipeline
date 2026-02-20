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

/// \file GdalUtils.h

// Utilities for writing images

#ifndef __ASP_CORE_GDAL_UTILS_H__
#define __ASP_CORE_GDAL_UTILS_H__

#include <vw/Cartography/GeoReferenceUtils.h>
#include <vw/FileIO/DiskImageUtils.h>
#include <vw/Core/StringUtils.h>

namespace asp {

/// String we use in ASP written point cloud files to indicate that an offset
///  has been subtracted out from the points.
// Note: We use this constant in the python code as well
const std::string ASP_POINT_OFFSET_TAG_STR = "POINT_OFFSET";

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
vw::UnaryPerPixelView<ImageT, SubtractShift<typename ImageT::pixel_type>>
inline subtract_shift(vw::ImageViewBase<ImageT> const& image,
                      vw::Vector3 const& shift) {
  return vw::UnaryPerPixelView<ImageT, SubtractShift<typename ImageT::pixel_type>>
    (image.impl(), SubtractShift<typename ImageT::pixel_type>(shift));
}

// Compile-intensive via read_channels (12 instantiations).
// Prefer the wrappers in PointCloudRead.h.
template<int m>
vw::ImageViewRef<vw::Vector<double, m>> read_asp_point_cloud(std::string const& filename) {

  vw::Vector3 shift;
  std::string shift_str;
  boost::shared_ptr<vw::DiskImageResource> rsrc
    (new vw::DiskImageResourceGDAL(filename));
  bool success = vw::cartography::read_header_string(*rsrc.get(),
                                                     asp::ASP_POINT_OFFSET_TAG_STR,
                                                     shift_str);
  if (success)
    shift = vw::str_to_vec<vw::Vector3>(shift_str);

  // Read the first m channels
  vw::ImageViewRef<vw::Vector<double, m>> out_image 
    = vw::read_channels<m, double>(filename, 0);

  // Add the shift back to the first several channels.
  if (shift != vw::Vector3())
    out_image = subtract_shift(out_image, -shift);

  return out_image;
}

/// Don't round pixels in point2dem for bodies of radius smaller than
/// this in meters. Do it though in stereo_tri, see get_rounding_error().
const double MIN_RADIUS_FOR_ROUNDING = 1e+6; // 1000 km

/// Unless user-specified, compute the rounding error for a given
/// planet (a point on whose surface is given by 'shift'). Return an
/// inverse power of 2, 1/2^10 for Earth and proportionally less for smaller bodies.
double get_rounding_error(vw::Vector3 const& shift, double rounding_error);

/// Round pixels in given image to multiple of given rounding_error.
template <class VecT>
struct RoundImagePixels: public vw::ReturnFixedType<VecT> {
  double m_rounding_error;
  RoundImagePixels(double rounding_error):m_rounding_error(rounding_error){
    VW_ASSERT(m_rounding_error > 0.0,
                vw::ArgumentErr() << "Rounding error must be positive.");
  }
  VecT operator() (VecT const& pt) const {
    return m_rounding_error*round(pt/m_rounding_error);
  }
};
template <class ImageT>
vw::UnaryPerPixelView<ImageT, RoundImagePixels<typename ImageT::pixel_type> >
inline round_image_pixels(vw::ImageViewBase<ImageT> const& image,
                        double rounding_error) {
  return vw::UnaryPerPixelView<ImageT, RoundImagePixels<typename ImageT::pixel_type> >
    (image.impl(), RoundImagePixels<typename ImageT::pixel_type>(rounding_error));
}

// Save with temporarily big blocks if this helps with efficiency, such as if 
// the blocks need internal padding taken into account.
template <class ImageT>
void saveWithTempBigBlocks(int bigBlockSize,
                           const std::string &filename,
                           vw::ImageViewBase<ImageT> const& img,
                           bool hasGeoref,
                           vw::cartography::GeoReference const& georef,
                           bool hasNodata, double nodata,
                           vw::GdalWriteOptions const& opt,
                           vw::ProgressCallback const& tpc) {

  // Check if block size needs changing
  vw::Vector2 origBlockSize = opt.raster_tile_size;
  vw::Vector2 newBlockSize = vw::Vector2(bigBlockSize, bigBlockSize);
  
  // If block size matches, just write once with original options
  if (newBlockSize == origBlockSize) {
    block_write_gdal_image(filename, img, hasGeoref, georef, hasNodata, nodata,
                           opt, tpc);
    return;
  }
  
  // Block size differs - write temp with big blocks, then rewrite with correct blocks
  vw::GdalWriteOptions localOpt = opt;
  localOpt.raster_tile_size = newBlockSize;
  localOpt.cog = false;  // Disable COG for temporary write
  block_write_gdal_image(filename, img, hasGeoref, georef, hasNodata, nodata,
                         localOpt, tpc);

  std::string tmpFile
    = boost::filesystem::path(filename).replace_extension(".tmp.tif").string();
  boost::filesystem::rename(filename, tmpFile);
  vw::DiskImageView<typename ImageT::pixel_type> tmpImg(tmpFile);
  localOpt.raster_tile_size = origBlockSize;
  localOpt.cog = opt.cog;  // Restore COG setting for final write
  vw::vw_out() << "Re-writing with blocks of size: "
               << localOpt.raster_tile_size[0] << " x "
               << localOpt.raster_tile_size[1] << ".\n";
  vw::cartography::block_write_gdal_image(filename, tmpImg, hasGeoref, georef,
                                          hasNodata, nodata, localOpt, tpc);
  boost::filesystem::remove(tmpFile);
  return;
}

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
                                   vw::GdalWriteOptions const& opt,
                                   vw::ProgressCallback const& progress_callback
                                    = vw::ProgressCallback::dummy_instance(),
                                   std::map<std::string, std::string> const& keywords =
                                   std::map<std::string, std::string>()) {

  if (norm_2(shift) > 0) {

    // Add the point shift to keywords
    std::map<std::string, std::string> local_keywords = keywords;
    local_keywords[ASP_POINT_OFFSET_TAG_STR] = vw::vec_to_str(shift);

    block_write_gdal_image(filename,
                            vw::channel_cast<float>
                            (round_image_pixels(subtract_shift(image.impl(), shift),
                                                get_rounding_error(shift, rounding_error))),
                            has_georef, georef, has_nodata, nodata,
                            opt, progress_callback, local_keywords);

  }else{
    block_write_gdal_image(filename, image, has_georef, georef,
                            has_nodata, nodata, opt,
                            progress_callback, keywords);
  }

}

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
                              vw::GdalWriteOptions const& opt,
                              vw::ProgressCallback const& progress_callback
                              = vw::ProgressCallback::dummy_instance(),
                              std::map<std::string, std::string> const& keywords =
                              std::map<std::string, std::string>()) {

  if (norm_2(shift) > 0){
    // Add the point shift to keywords
    std::map<std::string, std::string> local_keywords = keywords;
    local_keywords[ASP_POINT_OFFSET_TAG_STR] = vw::vec_to_str(shift);
    write_gdal_image(filename,
                      vw::channel_cast<float>
                      (round_image_pixels(subtract_shift(image.impl(), shift),
                                          get_rounding_error(shift, rounding_error))),
                      has_georef, georef, has_nodata, nodata,
                      opt, progress_callback, local_keywords);
  }else{
    write_gdal_image(filename, image, has_georef, georef,
                      has_nodata, nodata, opt, progress_callback, keywords);
  }
}

} // End namespace asp

#endif//__ASP_CORE_GDAL_UTILS_H__
