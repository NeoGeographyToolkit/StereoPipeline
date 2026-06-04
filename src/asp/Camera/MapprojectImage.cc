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
#include <asp/IsisIO/IsisSpecialPixels.h>

#include <vw/Cartography/PointImageManipulation.h>
#include <vw/Cartography/Map2CamTrans.h>
#include <vw/Cartography/PointImageManipulation.h>
#include <vw/Cartography/GeoReferenceUtils.h>
#include <vw/Image/Filter.h>
#include <vw/Image/ImageChannels.h>
#include <vw/Image/RoundAndClamp.h>
#include <vw/FileIO/FileUtils.h>

#include <boost/filesystem.hpp>

namespace fs = boost::filesystem;

namespace asp {

using namespace vw;
using namespace vw::cartography;

// Detect occlusion as a sign test on dot_prod(xyz/|xyz|, ray) where ray =
// pixel_to_vector(point_to_pixel(xyz)). For a visible point the camera ray
// points roughly inward, dot is negative. A small grazing margin is used in
// OcclusionCam so DEM noise near the limb does not produce
// ghost-survivor pixels.
static constexpr double OCCLUSION_GRAZING_DEG = 1.0;
static constexpr double OCCLUSION_DOT_THRESH  = -0.01745240643728351;

OcclusionCam::OcclusionCam(vw::CamPtr inner): m_inner(inner) {}

vw::Vector2 OcclusionCam::point_to_pixel(vw::Vector3 const& point) const {
  vw::Vector2 pix = m_inner->point_to_pixel(point);
  double n = vw::math::norm_2(point);
  if (n > 0.0) {
    vw::Vector3 ray = m_inner->pixel_to_vector(pix);
    if (vw::math::dot_prod(point / n, ray) > OCCLUSION_DOT_THRESH)
      vw::vw_throw(vw::camera::PointToPixelErr() << "Back-of-body occluded.");
  }
  return pix;
}

vw::Vector3 OcclusionCam::pixel_to_vector(vw::Vector2 const& pix) const {
  return m_inner->pixel_to_vector(pix);
}

vw::Vector3 OcclusionCam::camera_center(vw::Vector2 const& pix) const {
  return m_inner->camera_center(pix);
}

vw::Quat OcclusionCam::camera_pose(vw::Vector2 const& pix) const {
  return m_inner->camera_pose(pix);
}

std::string OcclusionCam::type() const {
  return m_inner->type();
}

// Estimate occlusion with some threshold for grazing angle
bool isOccluded(vw::Vector2 const& proj_pt,
                vw::ImageViewRef<MapprojDemPixel> const& dem,
                vw::cartography::GeoReference const& dem_georef,
                vw::cartography::GeoReference const& target_georef,
                vw::CamPtr const& camera_model) {

  Vector2 lonlat  = target_georef.point_to_lonlat(proj_pt);
  Vector2 dem_pix = dem_georef.lonlat_to_pixel(lonlat);

  if (dem_pix.x() < 0 || dem_pix.y() < 0 ||
      dem_pix.x() > dem.cols() - 1 || dem_pix.y() > dem.rows() - 1)
    return false; // outside DEM extent

  auto interp_dem = interpolate(dem); // bilinear, constant edge extension
  MapprojDemPixel h = interp_dem(dem_pix.x(), dem_pix.y());
  if (!is_valid(h))
    return false; // DEM nodata at this point

  vw::Vector3 llh = Vector3(lonlat[0], lonlat[1], h.child());
  vw::Vector3 xyz = dem_georef.datum().geodetic_to_cartesian(llh);
  if (xyz == Vector3())
    return false;

  vw::Vector2 cam_pix;
  try {
    cam_pix = camera_model->point_to_pixel(xyz);
  } catch (...) {
    return false;
  }

  Vector3 ray;
  try {
    ray = camera_model->pixel_to_vector(cam_pix);
  } catch (...) {
    return false;
  }

  double n = norm_2(xyz);
  if (n == 0.0)
    return false;
  return dot_prod(xyz / n, ray) > OCCLUSION_DOT_THRESH;
}

bool anyCornerOccluded(vw::BBox2 const& cam_box,
                       vw::ImageViewRef<MapprojDemPixel> const& dem,
                       vw::cartography::GeoReference const& dem_georef,
                       vw::cartography::GeoReference const& target_georef,
                       vw::CamPtr const& camera_model) {
  if (cam_box.empty())
    return false;
  Vector2 corners[4] = {
    cam_box.min(),
    Vector2(cam_box.max().x(), cam_box.min().y()),
    cam_box.max(),
    Vector2(cam_box.min().x(), cam_box.max().y())
  };
  for (int i = 0; i < 4; i++) {
    if (isOccluded(corners[i], dem, dem_georef, target_georef, camera_model))
      return true;
  }
  return false;
}

// Estimate the camera bbox taking into account occlusion by planetary body.
vw::BBox2 occlusionEstim(vw::BBox2 const& cam_box,
                         vw::ImageViewRef<MapprojDemPixel> const& dem,
                         vw::cartography::GeoReference const& dem_georef,
                         vw::cartography::GeoReference const& target_georef,
                         vw::CamPtr const& camera_model) {

  if (cam_box.empty())
    return cam_box;

  if (!anyCornerOccluded(cam_box, dem, dem_georef, target_georef, camera_model))
    return cam_box;

  vw_out() << "At least one corner of the camera bounding box is back-of-body "
           << "occluded. Sampling edges to estimate an unoccluded sub-box.\n";

  const int N = 100;
  BBox2 box_sans_occl;
  double xmin = cam_box.min().x();
  double ymin = cam_box.min().y();
  double xmax = cam_box.max().x();
  double ymax = cam_box.max().y();

  for (int i = 0; i < N; i++) {
    double t = double(i) / double(N - 1);
    double x = xmin + t * (xmax - xmin);
    double y = ymin + t * (ymax - ymin);
    Vector2 pts[4] = {
      Vector2(x, ymin), Vector2(x, ymax),  // bottom and top edges
      Vector2(xmin, y), Vector2(xmax, y)   // left and right edges
    };
    for (int k = 0; k < 4; k++) {
      if (!isOccluded(pts[k], dem, dem_georef, target_georef, camera_model))
        box_sans_occl.grow(pts[k]);
    }
  }

  if (box_sans_occl.empty())
    return cam_box; // Fall back to original box

  return box_sans_occl;
}

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
    
    // Parse keywords from the --mo option (it may be repeated, GDAL-style).
    for (size_t i = 0; i < opt.metadata.size(); i++)
      asp::parse_append_metadata(opt.metadata[i], keywords);
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

// Like write_parallel_type, but for compound pixel types (e.g., PixelRGBA<float32>).
// Uses channel_cast_round_and_clamp to convert each channel independently,
// preserving the multi-channel pixel structure.
template <class ImageT>
void write_parallel_type_multichannel(std::string              const& filename,
                                      ImageT                   const& image,
                                      GeoReference             const& georef,
                                      bool has_nodata, double nodata_val,
                                      asp::MapprojOptions      const& opt,
                                      TerminalProgressCallback const& tpc) {

  if (opt.output_type == "Float32")
    write_parallel_cond(filename, image, georef, has_nodata, nodata_val, opt, tpc);
  else if (opt.output_type == "Byte")
    write_parallel_cond(filename, channel_cast_round_and_clamp<uint8>(image),
                        georef, has_nodata,
                        vw::round_and_clamp<uint8>(nodata_val), opt, tpc);
  else if (opt.output_type == "UInt16")
    write_parallel_cond(filename, channel_cast_round_and_clamp<uint16>(image),
                        georef, has_nodata,
                        vw::round_and_clamp<uint16>(nodata_val), opt, tpc);
  else if (opt.output_type == "Int16")
    write_parallel_cond(filename, channel_cast_round_and_clamp<int16>(image),
                        georef, has_nodata,
                        vw::round_and_clamp<int16>(nodata_val), opt, tpc);
  else if (opt.output_type == "UInt32")
    write_parallel_cond(filename, channel_cast_round_and_clamp<uint32>(image),
                        georef, has_nodata,
                        vw::round_and_clamp<uint32>(nodata_val), opt, tpc);
  else if (opt.output_type == "Int32")
    write_parallel_cond(filename, channel_cast_round_and_clamp<int32>(image),
                        georef, has_nodata,
                        vw::round_and_clamp<int32>(nodata_val), opt, tpc);
  else
    vw_throw(NoImplErr() << "Unsupported output type: " << opt.output_type << ".\n");
}

/// Mapproject the image with a nodata value. Used for single channel images.
/// Input is always read as float (no template on pixel type needed since the
/// only caller uses float).
template <class Map2CamTransT>
void project_image_nodata(asp::MapprojOptions & opt,
                          GeoReference  const& out_georef,
                          Vector2i      const& out_size,
                          BBox2i        const& out_bbox,
                          Map2CamTransT const& transform) {

    typedef PixelMask<float> ImageMaskPixelT;

    // Create handle to input image to be projected on to the map
    boost::shared_ptr<DiskImageResource> img_rsrc =
          vw::DiskImageResourcePtr(opt.image_file);

    // Disable rescaling so integer pixels (uint8, uint16, etc.) are not
    // normalized to [0,1] when read as float. In practice, rescaling does
    // not appear to happen in the GDAL read path even without this call,
    // but set it explicitly as a safeguard.
    img_rsrc->set_rescale(false);

    // Update the nodata value from the input file if it is present.
    if (img_rsrc->has_nodata_read())
      opt.nodata_value = img_rsrc->nodata_read();

    // Read as single-plane float. Multi-band images (e.g., THEMIS IR) are
    // exposed as multiple planes by VW's GDAL reader; only the first band
    // is used in this path (the RGB path handles multi-channel images).
    ImageViewRef<float> disk_img = DiskImageView<float>(img_rsrc);
    if (disk_img.planes() > 1)
      disk_img = select_plane(disk_img, 0);

    // Create masked image from input
    ImageViewRef<ImageMaskPixelT> masked_input
      = create_mask(disk_img, opt.nodata_value);

    // For ISIS .cub files, also mask special pixels (LIS, LRS, HIS, HRS)
    // that are not covered by the single nodata value
    if (fs::path(opt.image_file).extension() == ".cub")
      asp::adjustIsisImage(opt.image_file, opt.nodata_value, masked_input);

    bool            has_img_nodata = true;
    ImageMaskPixelT nodata_mask = ImageMaskPixelT(); // invalid value for a PixelMask

    if (opt.nearest_neighbor) {
      write_parallel_type
        (opt.output_file,
        crop(apply_mask
              (transform_nodata(masked_input,
                                transform,
                                out_size[0],
                                out_size[1],
                                ValueEdgeExtension<ImageMaskPixelT>(nodata_mask),
                                NearestPixelInterpolation(), nodata_mask),
              opt.nodata_value),
              out_bbox),
        out_georef, has_img_nodata, opt.nodata_value, opt,
        TerminalProgressCallback("",""));
    } else {
      write_parallel_type
        (opt.output_file,
        crop(apply_mask
              (transform_nodata(masked_input,
                                transform,
                                out_size[0],
                                out_size[1],
                                ValueEdgeExtension<ImageMaskPixelT>(nodata_mask),
                                BicubicInterpolation(), nodata_mask),
              opt.nodata_value),
              out_bbox),
        out_georef, has_img_nodata, opt.nodata_value, opt,
        TerminalProgressCallback("",""));
    }
}

/// Map project the image with an alpha channel. Used for multi-channel images.
/// The input image is read as its native type, then channel_cast to float32
/// (preserving values, e.g., uint8 255 becomes 255.0f). Processing happens
/// entirely in float32. On write, channel_cast_round_and_clamp converts back
/// to the original integer type when appropriate.
template <class Map2CamTransT>
void project_image_alpha(asp::MapprojOptions & opt,
                         ImageViewRef<PixelRGBA<float32>> const& input_image,
                         GeoReference const& out_georef,
                         Vector2i     const& out_size,
                         BBox2i       const& out_bbox,
                         Map2CamTransT const& transform) {

    const bool has_img_nodata = false;
    const PixelRGBA<float32> transparent_pixel = PixelRGBA<float32>();

    if (opt.nearest_neighbor) {
      write_parallel_type_multichannel
        (opt.output_file,
        crop(transform_nodata(input_image,
                              transform,
                              out_size[0],
                              out_size[1],
                              ConstantEdgeExtension(),
                              NearestPixelInterpolation(), transparent_pixel),
              out_bbox),
        out_georef, has_img_nodata, opt.nodata_value, opt,
        TerminalProgressCallback("",""));
    } else {
      write_parallel_type_multichannel
        (opt.output_file,
        crop(transform_nodata(input_image,
                              transform,
                              out_size[0],
                              out_size[1],
                              ConstantEdgeExtension(),
                              BicubicInterpolation(), transparent_pixel),
              out_bbox),
        out_georef, has_img_nodata, opt.nodata_value, opt,
        TerminalProgressCallback("",""));
    }
}

// The two "pick" functions below select between the Map2CamTrans and Datum2CamTrans
// transform classes which will be passed to the image projection function.
// - TODO: Is there a good reason for the transform classes to be CRTP instead of virtual?

void project_image_nodata_pick_transform(asp::MapprojOptions & opt,
                          GeoReference const& dem_georef,
                          GeoReference const& target_georef,
                          GeoReference const& out_georef,
                          Vector2i     const& input_size,
                          Vector2i     const& out_size,
                          BBox2i       const& out_bbox,
                          vw::CamPtr const& camera_model) {
  const bool call_from_mapproject = true;
  if (fs::path(opt.dem_file).extension() != "") {
    // A DEM file was provided
    return project_image_nodata(opt, out_georef,
                                out_size, out_bbox,
                                Map2CamTrans(camera_model.get(), target_georef,
                                             dem_georef, opt.dem_file, input_size,
                                             call_from_mapproject,
                                             opt.nearest_neighbor));
  } else {
    // A constant datum elevation was provided
    return project_image_nodata(opt, out_georef,
                                out_size, out_bbox,
                                Datum2CamTrans(camera_model.get(), target_georef,
                                               dem_georef, opt.datum_offset,
                                               input_size, call_from_mapproject,
                                               opt.nearest_neighbor));
  }
}

void project_image_alpha_pick_transform(asp::MapprojOptions & opt,
                                        ImageViewRef<PixelRGBA<float32>> const& input_image,
                                        GeoReference const& dem_georef,
                                        GeoReference const& target_georef,
                                        GeoReference const& out_georef,
                                        Vector2i     const& input_size,
                                        Vector2i     const& out_size,
                                        BBox2i       const& out_bbox,
                                        vw::CamPtr const&
                                        camera_model) {

  const bool call_from_mapproject = true;
  if (fs::path(opt.dem_file).extension() != "") {
    // A DEM file was provided
    return project_image_alpha(opt, input_image, out_georef,
                               out_size, out_bbox,
                               Map2CamTrans(camera_model.get(), target_georef,
                                            dem_georef, opt.dem_file, input_size,
                                            call_from_mapproject,
                                            opt.nearest_neighbor));
  } else {
    // A constant datum elevation was provided
    return project_image_alpha(opt, input_image, out_georef,
                               out_size, out_bbox,
                               Datum2CamTrans(camera_model.get(), target_georef,
                                              dem_georef, opt.datum_offset,
                                              input_size, call_from_mapproject,
                                              opt.nearest_neighbor));
  }
}

// Project the image depending on image format.
void project_image(asp::MapprojOptions & opt,
                   GeoReference const& dem_georef,
                   GeoReference const& target_georef,
                   GeoReference const& out_georef,
                   Vector2i const& input_size,
                   Vector2i const& out_size,
                   BBox2i const& out_bbox) {

  // Prepare output directory
  vw::create_out_dir(opt.output_file);

  // Determine the pixel type of the input image
  boost::shared_ptr<DiskImageResource> image_rsrc = vw::DiskImageResourcePtr(opt.image_file);
  ImageFormat image_fmt = image_rsrc->format();
  const int num_input_channels = num_channels(image_fmt.pixel_format);

  // Redirect to the correctly typed function to perform the actual map projection.
  if (image_fmt.pixel_format == VW_PIXEL_RGB) {

    // RGB processing strategy: read as native type, channel_cast to float32
    // (preserving values: uint8 255 becomes 255.0f, not 1.0f), process entirely
    // in float32, then write_parallel_type applies RoundAndClamp to convert back
    // to the original integer type. This avoids heavy template instantiation for
    // each input channel type - only the lightweight read+cast is type-specific.
    ImageViewRef<PixelRGBA<float32>> float_image;
    switch (image_fmt.channel_type) {
    case VW_CHANNEL_UINT8:
      float_image = channel_cast<float32>(DiskImageView<PixelRGBA<uint8>>(image_rsrc));
      opt.output_type = "Byte";
      break;
    case VW_CHANNEL_INT16:
      float_image = channel_cast<float32>(DiskImageView<PixelRGBA<int16>>(image_rsrc));
      opt.output_type = "Int16";
      break;
    case VW_CHANNEL_UINT16:
      float_image = channel_cast<float32>(DiskImageView<PixelRGBA<uint16>>(image_rsrc));
      opt.output_type = "UInt16";
      break;
    default:
      float_image = DiskImageView<PixelRGBA<float32>>(image_rsrc);
      opt.output_type = "Float32";
      break;
    }

    project_image_alpha_pick_transform(opt, float_image, dem_georef,
                                       target_georef, out_georef, input_size,
                                       out_size, out_bbox, opt.camera_model);

  } else {
    // If the input image is not RGB, only single channel images are supported.
    if (num_input_channels != 1 || image_fmt.planes != 1)
      //vw_throw( ArgumentErr() << "Input images must be single channel or RGB!\n" );
      vw_out() << "Detected multi-band image. Only the first band will be used. The pixels will be interpreted as float.\n";
    // Read as float with rescaling disabled, so integer values are preserved.
    project_image_nodata_pick_transform(opt, dem_georef, target_georef, out_georef,
                                        input_size, out_size, out_bbox,
                                        opt.camera_model);
  }
  // Done map projecting
}

} // end namespace asp
