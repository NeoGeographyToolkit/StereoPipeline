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

// \file SfsCamera.cc
// A camera approximation used by SfS around a small DEM region. It works
// by tabulation.

#include <asp/SfS/SfsCamera.h>
#include <vw/Image/Interpolation.h>

namespace asp {

using namespace vw;

ApproxCameraModel::ApproxCameraModel(vw::CamPtr const& exact_camera,
                                     BBox2i img_bbox,
                                     ImageView<double> const& dem,
                                     vw::cartography::GeoReference const& geo,
                                     double nodata_val,
                                     vw::Mutex & camera_mutex):
    m_geo(geo),
    m_camera_mutex(camera_mutex),
    m_exact_camera(exact_camera),
    m_img_bbox(img_bbox),
    m_model_is_valid(true) {

  int big = 1e+8;
  m_uncompValue = Vector2(-big, -big);

  // Compute the mean DEM height. We expect all DEM entries to be valid.
  m_mean_ht = 0;
  double num = 0.0;
  for (int col = 0; col < dem.cols(); col++) {
    for (int row = 0; row < dem.rows(); row++) {
      if (dem(col, row) == nodata_val)
        vw_throw(ArgumentErr()
                  << "ApproxCameraModel: Expecting a DEM without nodata values.\n");
      m_mean_ht += dem(col, row);
      num += 1.0;
    }
  }
  if (num > 0) m_mean_ht /= num;

  // The area we're supposed to work around
  m_point_box = m_geo.pixel_to_point_bbox(bounding_box(dem));
  double wx = m_point_box.width(), wy = m_point_box.height();
  m_approx_table_gridx = wx/std::max(dem.cols(), 1);
  m_approx_table_gridy = wy/std::max(dem.rows(), 1);

  if (m_approx_table_gridx == 0 || m_approx_table_gridy == 0) {
    vw_throw(ArgumentErr()
              << "ApproxCameraModel: Expecting a positive grid size.\n");
  }

  // Expand the box, as later the DEM will change.
  double extra = 0.5;
  m_point_box.min().x() -= extra*wx; m_point_box.max().x() += extra*wx;
  m_point_box.min().y() -= extra*wy; m_point_box.max().y() += extra*wy;
  wx = m_point_box.width();
  wy = m_point_box.height();

  vw_out() << "Approximation proj box: " << m_point_box << std::endl;

  // We will tabulate the point_to_pixel function at a multiple of
  // the grid, and we'll use interpolation for anything in
  // between.
  //m_approx_table_gridx /= 2.0; m_approx_table_gridy /= 2.0; // fine
  m_approx_table_gridx *= 2.0; m_approx_table_gridy *= 2.0; // Coarse. Good enough.

  int numx = wx/m_approx_table_gridx;
  int numy = wy/m_approx_table_gridy;

  vw_out() << "Lookup table dimensions: " << numx << ' ' << numy << std::endl;

  m_begX = 0; m_endX = numx-1;
  m_begY = 0; m_endY = numy-1;

  // Mark all values as uncomputed and invalid
  m_pixel_to_vec_mat.set_size(numx, numy);
  m_point_to_pix_mat.set_size(numx, numy);
  for (int x = 0; x < numx; x++) {
    for (int y = 0; y < numy; y++) {
      m_point_to_pix_mat(x, y) = m_uncompValue;
      m_point_to_pix_mat(x, y).invalidate();
    }
  }

  // Fill in the table. Find along the way the mean direction from
  // the camera to the ground. Invalid values will be masked.
  m_count = 0;
  m_mean_dir = Vector3();
  comp_entries_in_table();
  m_mean_dir /= std::max(1, m_count);
  m_mean_dir = m_mean_dir/norm_2(m_mean_dir);

  m_crop_box.crop(m_img_bbox);

  return;
} // End constructor

void ApproxCameraModel::comp_entries_in_table() const {
  for (int x = m_begX; x <= m_endX; x++) {
    for (int y = m_begY; y <= m_endY; y++) {

      // This will be useful when we invoke this function repeatedly
      if (m_point_to_pix_mat(x, y).child() != m_uncompValue) {
        continue;
      }

      Vector2 pt(m_point_box.min().x() + x*m_approx_table_gridx,
                  m_point_box.min().y() + y*m_approx_table_gridy);
      Vector2 lonlat = m_geo.point_to_lonlat(pt);
      Vector3 xyz = m_geo.datum().geodetic_to_cartesian
        (Vector3(lonlat[0], lonlat[1], m_mean_ht));
      bool success = true;
      Vector2 pix;
      Vector3 vec;
      try {
        pix = m_exact_camera->point_to_pixel(xyz);
        //if (true || m_img_bbox.contains(pix))  // Need to think more here
        vec = m_exact_camera->pixel_to_vector(pix);
        //else
        // success = false;

      } catch(...) {
        success = false;
      }
      if (success) {
        m_pixel_to_vec_mat(x, y) = vec;
        m_point_to_pix_mat(x, y) = pix;
        m_pixel_to_vec_mat(x, y).validate();
        m_point_to_pix_mat(x, y).validate();
        m_mean_dir += vec; // only when the point projects inside the camera?
        if (m_img_bbox.contains(pix))
          m_crop_box.grow(pix);
        m_count++;
      } else {
        m_pixel_to_vec_mat(x, y).invalidate();
        m_point_to_pix_mat(x, y).invalidate();
      }
    }
  }

   return;
}

// We have tabulated point_to_pixel at the mean dem height.
// Look-up point_to_pixel for the current point by first
// intersecting the ray from the current point to the camera
// with the datum at that height. We don't know that ray,
// so we iterate to find it.
vw::Vector2 ApproxCameraModel::point_to_pixel(Vector3 const& xyz) const {

  // TODO: What happens if we use bicubic interpolation?
  InterpolationView<EdgeExtensionView<ImageView<PixelMask<Vector3>>, ConstantEdgeExtension>, BilinearInterpolation> pixel_to_vec_interp
    = interpolate(m_pixel_to_vec_mat, BilinearInterpolation(),
                  ConstantEdgeExtension());

  InterpolationView<EdgeExtensionView<ImageView<PixelMask<Vector2>>, ConstantEdgeExtension>, BilinearInterpolation> point_to_pix_interp
    = interpolate(m_point_to_pix_mat, BilinearInterpolation(),
                  ConstantEdgeExtension());

  Vector3 dir = m_mean_dir;
  Vector2 pix;
  double major_radius = m_geo.datum().semi_major_axis() + m_mean_ht;
  double minor_radius = m_geo.datum().semi_minor_axis() + m_mean_ht;
  for (size_t i = 0; i < 10; i++) {

    Vector3 S = xyz - 1.1*major_radius*dir; // push the point outside the sphere
    if (norm_2(S) <= major_radius) // point is inside the sphere
      return m_exact_camera->point_to_pixel(xyz);

    Vector3 datum_pt
      = vw::cartography::datum_intersection(major_radius, minor_radius, S, dir);
    Vector3 llh = m_geo.datum().cartesian_to_geodetic(datum_pt);
    Vector2 pt = m_geo.lonlat_to_point(subvector(llh, 0, 2));

    // Indices
    double x = (pt.x() - m_point_box.min().x())/m_approx_table_gridx;
    double y = (pt.y() - m_point_box.min().y())/m_approx_table_gridy;

    bool out_of_range = (x < m_begX || x >= m_endX-1 ||
                         y < m_begY || y >= m_endY-1);

    // If out of range, return the exact result. This should be very slow.
    // The hope is that it will be very rare.
    if (out_of_range)
      return m_exact_camera->point_to_pixel(xyz);

    PixelMask<Vector3> masked_dir = pixel_to_vec_interp(x, y);
    PixelMask<Vector2> masked_pix = point_to_pix_interp(x, y);
    if (is_valid(masked_dir) && is_valid(masked_pix)) {
      dir = masked_dir.child();
      pix = masked_pix.child();
    } else {
      return m_exact_camera->point_to_pixel(xyz);
    }
  }

  return pix;
}

std::string ApproxCameraModel::type() const{
  return "ApproxSfSCamera";
}

// This is used rarely. Return the exact camera vector.
Vector3 ApproxCameraModel::pixel_to_vector(Vector2 const& pix) const {
  vw::Mutex::Lock lock(m_camera_mutex);
  return m_exact_camera->pixel_to_vector(pix);
}

// Return the exact camera center
Vector3 ApproxCameraModel::camera_center(Vector2 const& pix) const {
  vw::Mutex::Lock lock(m_camera_mutex);
  return m_exact_camera->camera_center(pix);
}

// Return the exact camera pose
Quat ApproxCameraModel::camera_pose(Vector2 const& pix) const {
  vw::Mutex::Lock lock(m_camera_mutex);
  return m_exact_camera->camera_pose(pix);
}

// The range of pixels in the image we are actually expected to use.
// Note that the function returns an alias, so that we can modify the
// crop box from outside.
BBox2 & ApproxCameraModel::crop_box() {
  m_crop_box.crop(m_img_bbox);
  return m_crop_box;
}

bool ApproxCameraModel::model_is_valid() {
  return m_model_is_valid;
}

vw::CamPtr ApproxCameraModel::exact_camera() const {
  return m_exact_camera;
}

} // end namespace asp
