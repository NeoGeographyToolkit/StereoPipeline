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

/// \file MapprojectImage.h
/// Algorithms specific to mapprojecting images.

#ifndef __MAPPROJECT_IMAGE_H__
#define __MAPPROJECT_IMAGE_H__

#include <vw/Cartography/GeoReference.h>
#include <vw/FileIO/GdalWriteOptions.h>
#include <vw/FileIO/DiskImageView.h>
#include <vw/Camera/CameraModel.h>
#include <vw/Image/ImageViewRef.h>
#include <vw/Image/PixelMask.h>

#include <string>

namespace asp {
  
struct MapprojOptions: vw::GdalWriteOptions {
  // Input
  std::string dem_file, image_file, camera_file, output_file, stereo_session,
    bundle_adjust_prefix, ref_map;
  bool query_projection, write_wkt, noGeoHeaderInfo, nearest_neighbor, parseOptions, gdal_tap;
  bool model_occlusion; // Per-pixel back-of-body visibility check
  bool multithreaded_model; // This is set based on the session type
  
  // Keep a copy of the model here to not have to pass it around separately
  vw::CamPtr camera_model;
  
  // Settings
  std::string target_srs_string, output_type, metadata;
  double nodata_value, tr, mpp, ppd, datum_offset;
  vw::BBox2 target_projwin, target_pixelwin;
  vw::Vector2 query_pixel;
};

// Project the image depending on image format.
void project_image(asp::MapprojOptions & opt,
                   vw::cartography::GeoReference const& dem_georef,
                   vw::cartography::GeoReference const& target_georef, vw::cartography::GeoReference const& croppedGeoRef,
                   vw::Vector2i const& image_size,
                   int virtual_image_width, int virtual_image_height,
                   vw::BBox2i const& croppedImageBB);

// DEM pixel type used by mapproject's visibility helpers. Matches the local
// typedef in mapproject_single.cc so callers can pass the same view.
typedef vw::PixelMask<float> MapprojDemPixel;

// CameraModel wrapper that throws PointToPixelErr from point_to_pixel when
// the ground point is back-of-body occluded (the camera ray landing at the
// would-be pixel points outward from the body where the ground point sits).
// Map2CamTrans (in VW) already turns that exception into a nodata pixel in
// the output, so wrapping the camera is all that is needed to opt into
// per-pixel occlusion rejection. Costs an extra pixel_to_vector + dot
// product per output pixel that the camera was going to project.
class OcclusionCam: public vw::camera::CameraModel {
  vw::CamPtr m_inner;
public:
  explicit OcclusionCam(vw::CamPtr inner);

  vw::Vector2 point_to_pixel (vw::Vector3 const& point) const override;
  vw::Vector3 pixel_to_vector(vw::Vector2 const& pix)  const override;
  vw::Vector3 camera_center  (vw::Vector2 const& pix)  const override;
  vw::Quat    camera_pose    (vw::Vector2 const& pix)  const override;
  std::string type           ()                        const override;
};

// Test whether a projected (x, y) point is occluded by the body when seen
// by the camera. Looks up the DEM height (bilinear) at proj_pt, forms an
// ECEF ground point, projects through the camera, back-projects the
// resulting pixel, then checks whether the ray's direction points outward
// from the body where the ground point sits. Returns false when the DEM
// has no data, when projection fails, or when the ground point is
// visible. Used by anyCornerOccluded() and occlusionEstim().
bool isOccluded(vw::Vector2 const& proj_pt,
                vw::ImageViewRef<MapprojDemPixel> const& dem,
                vw::cartography::GeoReference const& dem_georef,
                vw::cartography::GeoReference const& target_georef,
                vw::CamPtr const& camera_model);

// True if any of the four corners of cam_box is back-of-body occluded.
// The query-projection phase emits this as model_occlusion,1/0 in its
// structured output so the parallel mapproject wrapper can opt every
// per-tile invocation into per-pixel occlusion rejection.
bool anyCornerOccluded(vw::BBox2 const& cam_box,
                       vw::ImageViewRef<MapprojDemPixel> const& dem,
                       vw::cartography::GeoReference const& dem_georef,
                       vw::cartography::GeoReference const& target_georef,
                       vw::CamPtr const& camera_model);

// Estimate the camera bbox taking into account occlusion by planetary body.
vw::BBox2 occlusionEstim(vw::BBox2 const& cam_box,
                         vw::ImageViewRef<MapprojDemPixel> const& dem,
                         vw::cartography::GeoReference const& dem_georef,
                         vw::cartography::GeoReference const& target_georef,
                         vw::CamPtr const& camera_model);

} // end namespace asp

#endif // __MAPPROJECT_IMAGE_H__
