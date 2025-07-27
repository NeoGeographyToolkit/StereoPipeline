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

/// \file MapprojectImage.cc
/// Algorithms specific to mapprojecting images.
// Much templated logic here that is very slow to compile.
// It is best to avoid adding more code here.

#include <asp/Camera/MapprojectImage.h>
#include <asp/Core/StereoSettings.h>
#include <asp/Core/AspStringUtils.h>

#include <vw/Cartography/PointImageManipulation.h>
#include <vw/Cartography/Map2CamTrans.h>
#include <vw/Cartography/PointImageManipulation.h>
#include <vw/Cartography/GeoReferenceUtils.h>
#include <vw/Image/Filter.h>
#include <vw/Image/Algorithms2.h>
#include <vw/FileIO/FileUtils.h>

#include <boost/filesystem.hpp>

namespace fs = boost::filesystem;

namespace asp {

using namespace vw;
using namespace vw::cartography;

template <class ImageT>
void write_parallel_cond(std::string              const& filename,
                         ImageViewBase<ImageT>    const& image,
                         GeoReference             const& georef,
                         bool has_nodata, double nodata_val,
                         asp::MapprojOptions      const& opt,
                         TerminalProgressCallback const& tpc) {

  // Write names of the bundle adjust prefix, input image file, camera file,    
  // dem, and session type. Those will be used in StereoSession to load the
  // mapprojected image.
  // There is no difference between pinhole and nadirpinhole when it comes
  // to how mapprojection happens, that becomes important only in stereo.
  std::string session_type = opt.stereo_session;
  if (session_type == "isismapisis")
    session_type = "isis";
  if (session_type == "rpcmaprpc")
    session_type = "rpc";
  if (session_type == "pinholemappinhole" || session_type == "nadirpinhole")
    session_type = "pinhole";
  
  // Save some keywords that we will check later when using the mapprojected file

  std::map<std::string, std::string> keywords;
  if (!opt.noGeoHeaderInfo) {
    std::string prefix = asp::stereo_settings().bundle_adjust_prefix;;
    if (prefix == "") prefix = "NONE"; // to save the field, need to make it non-empty
    keywords["BUNDLE_ADJUST_PREFIX" ] = prefix;
    keywords["CAMERA_MODEL_TYPE" ]    = session_type;
    keywords["INPUT_IMAGE_FILE" ]     = opt.image_file;
    keywords["CAMERA_FILE" ]          = opt.camera_file;

    // Save the camera adjustment. That is an important record
    // for how the image got mapprojected and is good to keep.
    Vector3 t(0, 0, 0);
    vw::Quaternion<double> q(1, 0, 0, 0);
    vw::camera::AdjustedCameraModel * adj_cam
      = dynamic_cast<vw::camera::AdjustedCameraModel*>(opt.camera_model.get());
    if (adj_cam != NULL) {
      q = adj_cam->rotation();
      t = adj_cam->translation();
    }
    
    std::ostringstream osq;
    osq.precision(17);
    osq << q.w() << "," << q.x() << "," << q.y() << "," << q.z();
    keywords["ADJUSTMENT_QUATERNION"] = osq.str();
    
    std::ostringstream ost;
    ost.precision(17);
    ost << t.x() << "," << t.y() << "," << t.z();
    keywords["ADJUSTMENT_TRANSLATION"] = ost.str();

    keywords["DEM_FILE"] = opt.dem_file;
    
    // Parse keywords from the --mo option.
    asp::parse_append_metadata(opt.metadata, keywords);
  }
  
  bool has_georef = true;

  // ISIS is not thread safe so we must switch out based on what the session is.
  vw_out() << "Writing: " << filename << "\n";
  if (opt.multithreaded_model) {
    vw::cartography::block_write_gdal_image(filename, image.impl(), has_georef, georef,
                                has_nodata, nodata_val, opt, tpc, keywords);
  } else {
    vw::cartography::write_gdal_image(filename, image.impl(), has_georef, georef,
                          has_nodata, nodata_val, opt, tpc, keywords);
  }

}

// If the output type is some type of int, round and clamp to this
// type, both the pixels and the nodata-value. Else write as it is.
template <class ImageT>
void write_parallel_type(std::string              const& filename,
                         ImageT                   const& image,
                         GeoReference             const& georef,
                         bool has_nodata, double nodata_val,
                         asp::MapprojOptions      const& opt,
                         TerminalProgressCallback const& tpc) {

  typedef typename ImageT::pixel_type InputType;

  if (opt.output_type == "Float32") 
    write_parallel_cond(filename, image, georef, has_nodata, nodata_val, opt, tpc);
  else if (opt.output_type == "Byte") 
    write_parallel_cond(filename,
			per_pixel_filter(image, RoundAndClamp<uint8, InputType>()),
			georef, has_nodata,
			vw::round_and_clamp<uint8>(nodata_val),
			opt, tpc);
  else if (opt.output_type == "UInt16") 
    write_parallel_cond(filename,
			per_pixel_filter(image, RoundAndClamp<uint16, InputType>()),
                        georef, has_nodata,
			vw::round_and_clamp<uint16>(nodata_val),
			opt, tpc);
  else if (opt.output_type == "Int16") 
    write_parallel_cond(filename,
			per_pixel_filter(image, RoundAndClamp<int16, InputType>()),
                        georef, has_nodata,
			vw::round_and_clamp<int16>(nodata_val),
			opt, tpc);
  else if (opt.output_type == "UInt32") 
    write_parallel_cond(filename,
			per_pixel_filter(image, RoundAndClamp<uint32, InputType>()),
                        georef, has_nodata,
			vw::round_and_clamp<uint32>(nodata_val),
			opt, tpc);
  else if (opt.output_type == "Int32") 
    write_parallel_cond(filename,
			per_pixel_filter(image, RoundAndClamp<int32, InputType>()),
                        georef, has_nodata,
			vw::round_and_clamp<int32>(nodata_val),
			opt, tpc);
  else
    vw_throw( NoImplErr() << "Unsupported output type: " << opt.output_type << ".\n" );
}

/// Mapproject the image with a nodata value.  Used for single channel images.
template <class ImagePixelT, class Map2CamTransT>
void project_image_nodata(asp::MapprojOptions & opt,
                          GeoReference  const& croppedGeoRef,
                          Vector2i      const& virtual_image_size,
                          BBox2i        const& croppedImageBB,
                          Map2CamTransT const& transform) {

    typedef PixelMask<ImagePixelT> ImageMaskPixelT;

    // Create handle to input image to be projected on to the map
    boost::shared_ptr<DiskImageResource> img_rsrc = 
          vw::DiskImageResourcePtr(opt.image_file);   

    // Update the nodata value from the input file if it is present.
    if (img_rsrc->has_nodata_read()) 
      opt.nodata_value = img_rsrc->nodata_read();

    bool            has_img_nodata = true;
    ImageMaskPixelT nodata_mask = ImageMaskPixelT(); // invalid value for a PixelMask

    // TODO: This is a lot of code duplication, is there a better way?
    if (opt.nearest_neighbor) {
      write_parallel_type
        ( // Write to the output file
        opt.output_file,
        crop( // Apply crop (only happens if --t_pixelwin was specified)
              apply_mask
              ( // Handle nodata
              transform_nodata( // Apply the output from Map2CamTrans
                                create_mask(DiskImageView<ImagePixelT>(img_rsrc),
                                            opt.nodata_value), // Handle nodata
                                transform,
                                virtual_image_size[0],
                                virtual_image_size[1],
                                ValueEdgeExtension<ImageMaskPixelT>(nodata_mask),
                                NearestPixelInterpolation(), nodata_mask
                                ),
              opt.nodata_value
              ),
              croppedImageBB
              ),
        croppedGeoRef, has_img_nodata, opt.nodata_value, opt,
        TerminalProgressCallback("","")
        );
    } else {
      write_parallel_type
        ( // Write to the output file
        opt.output_file,
        crop( // Apply crop (only happens if --t_pixelwin was specified)
              apply_mask
              ( // Handle nodata
              transform_nodata( // Apply the output from Map2CamTrans
                                create_mask(DiskImageView<ImagePixelT>(img_rsrc),
                                            opt.nodata_value), // Handle nodata
                                transform,
                                virtual_image_size[0],
                                virtual_image_size[1],
                                ValueEdgeExtension<ImageMaskPixelT>(nodata_mask),
                                BicubicInterpolation(), nodata_mask
                                ),
              opt.nodata_value
              ),
              croppedImageBB
              ),
        croppedGeoRef, has_img_nodata, opt.nodata_value, opt,
        TerminalProgressCallback("","")
        );
    }

}

/// Map project the image with an alpha channel.  Used for multi-channel images.
template <class ImagePixelT, class Map2CamTransT>
void project_image_alpha(asp::MapprojOptions & opt,
                         GeoReference const& croppedGeoRef,
                         Vector2i     const& virtual_image_size,
                         BBox2i       const& croppedImageBB,
                         boost::shared_ptr<camera::CameraModel> const& camera_model,
                         Map2CamTransT const& transform) {
  
    // Create handle to input image to be projected on to the map
    boost::shared_ptr<DiskImageResource> img_rsrc = 
          vw::DiskImageResourcePtr(opt.image_file);   

    const bool        has_img_nodata    = false;
    const ImagePixelT transparent_pixel = ImagePixelT();

    // TODO: Is it possible to reduce code duplication?
    if (opt.nearest_neighbor) {
      write_parallel_type
        ( // Write to the output file
        opt.output_file,
        crop( // Apply crop (only happens if --t_pixelwin was specified)
              // Transparent pixels are inserted for nodata
              transform_nodata( // Apply the output from Map2CamTrans
                                DiskImageView<ImagePixelT>(img_rsrc),
                                transform,
                                virtual_image_size[0],
                                virtual_image_size[1],
                                ConstantEdgeExtension(),
                                NearestPixelInterpolation(), transparent_pixel),
              croppedImageBB),
        croppedGeoRef, has_img_nodata, opt.nodata_value, opt,
        TerminalProgressCallback("","")
        );
    } else {
      write_parallel_type
        ( // Write to the output file
        opt.output_file,
        crop( // Apply crop (only happens if --t_pixelwin was specified)
              // Transparent pixels are inserted for nodata
              transform_nodata( // Apply the output from Map2CamTrans
                                DiskImageView<ImagePixelT>(img_rsrc),
                                transform,
                                virtual_image_size[0],
                                virtual_image_size[1],
                                ConstantEdgeExtension(),
                                BicubicInterpolation(), transparent_pixel),
              croppedImageBB),
        croppedGeoRef, has_img_nodata, opt.nodata_value, opt,
        TerminalProgressCallback("",""));
    }

}

// The two "pick" functions below select between the Map2CamTrans and Datum2CamTrans
// transform classes which will be passed to the image projection function.
// - TODO: Is there a good reason for the transform classes to be CRTP instead of virtual?

template <class ImagePixelT>
void project_image_nodata_pick_transform(asp::MapprojOptions & opt,
                          GeoReference const& dem_georef,
                          GeoReference const& target_georef,
                          GeoReference const& croppedGeoRef,
                          Vector2i     const& image_size,
                          Vector2i     const& virtual_image_size,
                          BBox2i       const& croppedImageBB,
                          boost::shared_ptr<camera::CameraModel> const& camera_model) {
  const bool        call_from_mapproject = true;
  if (fs::path(opt.dem_file).extension() != "") {
    // A DEM file was provided
    return project_image_nodata<ImagePixelT>(opt, croppedGeoRef,
                                             virtual_image_size, croppedImageBB,
                                             Map2CamTrans(// Converts coordinates in DEM
                                                          // georeference to camera pixels
                                                          camera_model.get(), target_georef,
                                                          dem_georef, opt.dem_file, image_size,
                                                          call_from_mapproject,
                                                          opt.nearest_neighbor));
  } else {
    // A constant datum elevation was provided
    return project_image_nodata<ImagePixelT>(opt, croppedGeoRef,
                                             virtual_image_size, croppedImageBB,
                                             Datum2CamTrans
                                             (// Converts coordinates in DEM
                                              // georeference to camera pixels
                                              camera_model.get(), target_georef,
                                              dem_georef, opt.datum_offset, image_size,
                                              call_from_mapproject,
                                              opt.nearest_neighbor));
  }
}

template <class ImagePixelT>
void project_image_alpha_pick_transform(asp::MapprojOptions & opt,
                                        GeoReference const& dem_georef,
                                        GeoReference const& target_georef,
                                        GeoReference const& croppedGeoRef,
                                        Vector2i     const& image_size,
                                        Vector2i     const& virtual_image_size,
                                        BBox2i       const& croppedImageBB,
                                        boost::shared_ptr<camera::CameraModel> const&
                                        camera_model) {
  
  const bool call_from_mapproject = true;
  if (fs::path(opt.dem_file).extension() != "") {
    // A DEM file was provided
    return project_image_alpha<ImagePixelT>(opt, croppedGeoRef,
                                            virtual_image_size, croppedImageBB, camera_model, 
                                            Map2CamTrans(// Converts coordinates in DEM
                                                         // georeference to camera pixels
                                                         camera_model.get(), target_georef,
                                                         dem_georef, opt.dem_file, image_size,
                                                         call_from_mapproject,
                                                         opt.nearest_neighbor));
  } else {
    // A constant datum elevation was provided
    return project_image_alpha<ImagePixelT>(opt, croppedGeoRef,
                                            virtual_image_size, croppedImageBB, camera_model, 
                                            Datum2CamTrans(// Converts coordinates in DEM
                                                           // georeference to camera pixels
                                                           camera_model.get(), target_georef,
                                                           dem_georef, opt.datum_offset, image_size,
                                                           call_from_mapproject,
                                                           opt.nearest_neighbor));
  }
}

// Project the image depending on image format.
void project_image(asp::MapprojOptions & opt, GeoReference const& dem_georef,
                   GeoReference const& target_georef, GeoReference const& croppedGeoRef,
                   Vector2i const& image_size, 
                   int virtual_image_width, int virtual_image_height,
                   BBox2i const& croppedImageBB) {

  // Prepare output directory
  vw::create_out_dir(opt.output_file);
  
  // Determine the pixel type of the input image
  boost::shared_ptr<DiskImageResource> image_rsrc = vw::DiskImageResourcePtr(opt.image_file);
  ImageFormat image_fmt = image_rsrc->format();
  const int num_input_channels = num_channels(image_fmt.pixel_format);

  // Redirect to the correctly typed function to perform the actual map projection.
  // - Must correspond to the type of the input image.
  if (image_fmt.pixel_format == VW_PIXEL_RGB) {

    // We can't just use float for everything or the output will be cast
    //  into the -1 to 1 range which is probably not desired.
    // - Always use an alpha channel with RGB images.
    switch(image_fmt.channel_type) {
    case VW_CHANNEL_UINT8:
      project_image_alpha_pick_transform<PixelRGBA<uint8>>(opt, dem_georef, target_georef,
                                                            croppedGeoRef, image_size, 
                                                            Vector2i(virtual_image_width,
                                                                    virtual_image_height),
                                                            croppedImageBB, opt.camera_model);
      break;
    case VW_CHANNEL_INT16:
      project_image_alpha_pick_transform<PixelRGBA<int16>>(opt, dem_georef, target_georef,
                                                            croppedGeoRef, image_size, 
                                                            Vector2i(virtual_image_width,
                                                                    virtual_image_height),
                                                            croppedImageBB, opt.camera_model);
      break;
    case VW_CHANNEL_UINT16:
      project_image_alpha_pick_transform<PixelRGBA<uint16>>(opt, dem_georef, target_georef,
                                                            croppedGeoRef, image_size, 
                                                            Vector2i(virtual_image_width,
                                                                      virtual_image_height),
                                                            croppedImageBB, opt.camera_model);
      break;
    default:
      project_image_alpha_pick_transform<PixelRGBA<float32>>(opt, dem_georef, target_georef,
                                                              croppedGeoRef, image_size, 
                                                              Vector2i(virtual_image_width,
                                                                      virtual_image_height),
                                                              croppedImageBB, opt.camera_model);
      break;
    };
    
  } else {
    // If the input image is not RGB, only single channel images are supported.
    if (num_input_channels != 1 || image_fmt.planes != 1)
      //vw_throw( ArgumentErr() << "Input images must be single channel or RGB!\n" );
      vw_out() << "Detected multi-band image. Only the first band will be used. The pixels will be interpreted as float.\n";
    // This will cast to float but will not rescale the pixel values.
    project_image_nodata_pick_transform<float>(opt, dem_georef, target_georef, croppedGeoRef,
                                                image_size, 
                          Vector2i(virtual_image_width, virtual_image_height),
                          croppedImageBB, opt.camera_model);
  } 
  // Done map projecting
}

} // end namespace asp
