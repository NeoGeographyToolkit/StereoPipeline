// __BEGIN_LICENSE__
//  Copyright (c) 2009-2025, United States Government as represented by the
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

// \file SfsCamera.h
// A camera approximation used by SfS around a small DEM region

#ifndef __ASP_SFS_SFS_CAMERA_H__
#define __ASP_SFS_SFS_CAMERA_H__

#include <vw/Image/ImageView.h>
#include <vw/Image/PixelMask.h>
#include <vw/Camera/CameraModel.h>
#include <vw/Core/Thread.h>
#include <vw/Cartography/GeoReference.h>

namespace asp {

class ApproxCameraModel: public vw::camera::CameraModel {
private:
  mutable vw::Vector3 m_mean_dir;  // mean vector from camera to ground
  vw::cartography::GeoReference m_geo;
  double m_mean_ht;
  mutable vw::ImageView<vw::PixelMask<vw::Vector3>> m_pixel_to_vec_mat;
  mutable vw::ImageView<vw::PixelMask<vw::Vector2>> m_point_to_pix_mat;
  double m_approx_table_gridx, m_approx_table_gridy;
  vw::Mutex& m_camera_mutex;
  vw::Vector2 m_uncompValue;
  mutable int m_begX, m_endX, m_begY, m_endY;
  mutable int m_count;
  vw::BBox2i m_img_bbox;
  mutable vw::BBox2 m_point_box, m_crop_box;
  bool m_model_is_valid;
  vw::camera::AdjustedCameraModel m_exact_camera;

  void comp_entries_in_table() const;

public:

  ApproxCameraModel(vw::camera::AdjustedCameraModel const& exact_camera,
                    vw::BBox2i img_bbox,
                    vw::ImageView<double> const& dem,
                    vw::cartography::GeoReference const& geo,
                    double nodata_val,
                    vw::Mutex& camera_mutex);

  virtual ~ApproxCameraModel() {}

  virtual vw::Vector2 point_to_pixel(vw::Vector3 const& xyz) const override;
  virtual vw::Vector3 pixel_to_vector(vw::Vector2 const& pix) const override;
  virtual vw::Vector3 camera_center(vw::Vector2 const& pix) const override;
  virtual vw::Quat camera_pose(vw::Vector2 const& pix) const override;
  virtual std::string type() const override;

  vw::BBox2& crop_box();
  bool model_is_valid();
  vw::camera::AdjustedCameraModel exact_camera() const;
};

} // end namespace asp

#endif // __ASP_SFS_SFS_CAMERA_H__
