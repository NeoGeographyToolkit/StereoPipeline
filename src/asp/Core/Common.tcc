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


/// \file Common.tcc
///

namespace asp {

  // Specialized functions for reading/writing images with a shift.
  // The shift is meant to bring the pixel values closer to origin,
  // with goal of saving the pixels as float instead of double.


  // Block write image while subtracting a given value from all pixels
  // and casting the result to float, while rounding to nearest mm.
  template <class ImageT>
  void block_write_approx_gdal_image(const std::string &filename,
                                     vw::Vector3 const& shift,
                                     double rounding_error,
                                     vw::ImageViewBase<ImageT> const& image,
                                     bool has_georef,
                                     vw::cartography::GeoReference const& georef,
                                     bool has_nodata, double nodata,
                                     vw::cartography::GdalWriteOptions const& opt,
                                     vw::ProgressCallback const& progress_callback,
                                     std::map<std::string, std::string> const& keywords) {


    if (norm_2(shift) > 0){

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

  // Single-threaded write image while subtracting a given value from
  // all pixels and casting the result to float.
  template <class ImageT>
  void write_approx_gdal_image(const std::string &filename,
                               vw::Vector3 const& shift,
                               double rounding_error,
                               vw::ImageViewBase<ImageT> const& image,
                               bool has_georef,
                               vw::cartography::GeoReference const& georef,
                               bool has_nodata, double nodata,
                               vw::cartography::GdalWriteOptions const& opt,
                               vw::ProgressCallback const& progress_callback,
                               std::map<std::string, std::string> const& keywords){

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

  // Often times, we'd like to save an image to disk by using big
  // blocks, for performance reasons, then re-write it with desired blocks.
  template <class ImageT>
  void save_with_temp_big_blocks(int big_block_size,
                                 const std::string &filename,
                                 vw::ImageViewBase<ImageT> const& img,
                                 vw::cartography::GeoReference const& georef,
                                 double nodata,
                                 vw::cartography::GdalWriteOptions & opt,
                                 vw::ProgressCallback const& tpc){

    vw::Vector2 orig_block_size = opt.raster_tile_size;
    opt.raster_tile_size = vw::Vector2(big_block_size, big_block_size);
    bool has_georef = true;
    bool has_nodata = true;
    block_write_gdal_image(filename, img, has_georef, georef, has_nodata, nodata, opt, tpc);

    if (opt.raster_tile_size != orig_block_size){
      std::string tmp_file
        = boost::filesystem::path(filename).replace_extension(".tmp.tif").string();
      boost::filesystem::rename(filename, tmp_file);
      vw::DiskImageView<typename ImageT::pixel_type> tmp_img(tmp_file);
      opt.raster_tile_size = orig_block_size;
      vw::vw_out() << "Re-writing with blocks of size: "
                   << opt.raster_tile_size[0] << " x " << opt.raster_tile_size[1] << ".\n";
      vw::cartography::block_write_gdal_image(filename, tmp_img, has_georef, georef,
                                  has_nodata, nodata, opt, tpc);
      boost::filesystem::remove(tmp_file);
    }
    return;
  }

} // namespace asp
