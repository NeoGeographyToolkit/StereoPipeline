// __BEGIN_LICENSE__
//  Copyright (c) 2006-2013, United States Government as represented by the
//  Administrator of the National Aeronautics and Space Administration. All
//  rights reserved.
//
//  The NGT platform is licensed under the Apache License, Version 2.0 (the
//  "License"); you may not use this file except in compliance with the
//  License. You may obtain a copy of the License at
//  https://www.apache.org/licenses/LICENSE-2.0
//
//  Unless required by applicable law or agreed to in writing, software
//  distributed under the License is distributed on an "AS IS" BASIS,
//  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
//  See the License for the specific language governing permissions and
//  limitations under the License.
// __END_LICENSE__

/// \file WidgetBase.cc

#include <asp/GUI/WidgetBase.h>
#include <vw/Math/Statistics.h>

namespace vw { namespace cartography {
  class GeoTransform;
}}

namespace vw { namespace gui {

WidgetBase::WidgetBase(int beg_image_id, int end_image_id,
                       int base_image_id,
                       asp::AppData & data,
                       bool use_georef,
                       std::vector<imageData> & images,
                       std::vector<vw::cartography::GeoTransform> & world2image_trans,
                       std::vector<vw::cartography::GeoTransform> & image2world_trans):
    m_beg_image_id(beg_image_id),
    m_end_image_id(end_image_id),
    m_data(data),
    m_base_image_id(base_image_id), 
    m_use_georef(use_georef),
    m_images(images),
    m_world2image_trans(world2image_trans),
    m_image2world_trans(image2world_trans), 
    m_world_box(BBox2()), m_border_factor(0.95) {
}

// Convert from world coordinates to projected coordinates in given geospatial
// projection
vw::Vector2 WidgetBase::world2proj(vw::Vector2 P, int imageIndex) const {
  if (!m_use_georef)
      return flip_in_y(P);
  return m_world2image_trans[imageIndex].point_to_point(flip_in_y(P)); 
}

// The reverse of world2proj
vw::Vector2 WidgetBase::proj2world(vw::Vector2 P, int imageIndex) const {
  if (!m_use_georef)
    return flip_in_y(P);
  return flip_in_y(m_image2world_trans[imageIndex].point_to_point(P));
}

// Find the min and max values, ignoring outliers. We look only 
// at the last component of each point, as that has the intensity,
// while the previous two have the position.
// Keep this here as it is used only for plotting in widget code.
void findRobustBounds(std::vector<vw::Vector3> const& scattered_data,
  double & min_val, double & max_val) {

  std::vector<double> vals;
  for (size_t pt_it = 0; pt_it < scattered_data.size(); pt_it++)
    vals.push_back(scattered_data[pt_it][2]);

  double beg_inlier = -1, end_inlier = -1, pct_fraction = 0.25, factor = 3.0;
  vw::math::find_outlier_brackets(vals, pct_fraction, factor, beg_inlier, end_inlier);
  min_val = end_inlier;
  max_val = beg_inlier;

  for (size_t it = 0; it < vals.size(); it++) {
    if (vals[it] < beg_inlier || vals[it] > end_inlier) 
      continue;
    min_val = std::min(min_val, vals[it]);
    max_val = std::max(max_val, vals[it]);
  }

  return;
}

// Convert a position in the world coordinate system to a pixel
// position as seen on screen (the screen origin is the
// visible upper-left corner of the widget).
Vector2 WidgetBase::world2screen(Vector2 const& p) const {

  double x = m_window_width*((p.x() - m_current_view.min().x())
                              / m_current_view.width());
  double y = m_window_height*((p.y() - m_current_view.min().y())
                              / m_current_view.height());

  // Create an empty border margin, to make it easier to zoom
  // by allowing the zoom window to slightly exceed the visible image
  // area (that inability was such a nuisance).
  x = m_border_factor*(x - m_window_width /2.0) + m_window_width /2.0;
  y = m_border_factor*(y - m_window_height/2.0) + m_window_height/2.0;

  return vw::Vector2(x, y);
}

// Convert a pixel on the screen to world coordinates.
// See world2image() for the definition.
Vector2 WidgetBase::screen2world(Vector2 const& p) const {

  // First undo the empty border margin
  double x = p.x(), y = p.y();
  x = (x - m_window_width /2.0)/m_border_factor + m_window_width /2.0;
  y = (y - m_window_height/2.0)/m_border_factor + m_window_height/2.0;

  // Scale to world coordinates
  x = m_current_view.min().x() + (m_current_view.width () * x / m_window_width);
  y = m_current_view.min().y() + (m_current_view.height() * y / m_window_height);

  return vw::Vector2(x, y);
}

BBox2 WidgetBase::screen2world(BBox2 const& R) const {
  if (R.empty()) return R;
  Vector2 A = screen2world(R.min());
  Vector2 B = screen2world(R.max());
  return BBox2(A, B);
}

BBox2 WidgetBase::world2screen(BBox2 const& R) const {
  if (R.empty()) return R;
  Vector2 A = world2screen(R.min());
  Vector2 B = world2screen(R.max());
  return BBox2(A, B);
}

// TODO(oalexan1): Rename world2image to world2pixel.
// Then also have world2projpt, for both a point and a box.

// TODO(oalexan1): See if world2image can be used instead of world2projpt,
// and same for image2world and proj2world.

// If we use georef, the world is in projected point units of the
// first image, with y replaced with -y, to keep the y axis downward,
// for consistency with how images are plotted.  Convert a world box
// to a pixel box for the given image.
Vector2 WidgetBase::world2image(Vector2 const& P, int imageIndex) const{
  bool poly_or_xyz = (m_images[imageIndex].m_isPoly || m_images[imageIndex].m_isCsv);

  if (poly_or_xyz) {
    // Poly or points. There is no pixel concept in that case.
    if (!m_use_georef)
      return flip_in_y(P);
    return m_world2image_trans[imageIndex].point_to_point(flip_in_y(P));
  }

  // Image
  if (!m_use_georef)
    return P;
  return m_world2image_trans[imageIndex].point_to_pixel(flip_in_y(P));
}

BBox2 WidgetBase::world2image(BBox2 const& R, int imageIndex) const {

  bool poly_or_xyz = (m_images[imageIndex].m_isPoly || m_images[imageIndex].m_isCsv);

  if (R.empty())
    return R;
  if (m_images.empty())
    return R;

  if (poly_or_xyz) {
    // Poly or points. There is no pixel concept in that case.
    if (!m_use_georef)
      return flip_in_y(R);
    return m_world2image_trans[imageIndex].point_to_point_bbox(flip_in_y(R));
  }

  // Image
  if (!m_use_georef)
    return R;
  return m_world2image_trans[imageIndex].point_to_pixel_bbox(flip_in_y(R));
}

// The reverse of world2image()
Vector2 WidgetBase::image2world(Vector2 const& P, int imageIndex) const {

  bool poly_or_xyz = (m_images[imageIndex].m_isPoly || m_images[imageIndex].m_isCsv);

  if (poly_or_xyz) {
    if (!m_use_georef)
      return flip_in_y(P);

    return flip_in_y(m_image2world_trans[imageIndex].point_to_point(P));
  }

  if (!m_use_georef)
    return P;
  return flip_in_y(m_image2world_trans[imageIndex].pixel_to_point(P));
}

// The reverse of world2image()
BBox2 WidgetBase::image2world(BBox2 const& R, int imageIndex) const {

  if (R.empty()) return R;
  if (m_images.empty()) return R;

  bool poly_or_xyz = (m_images[imageIndex].m_isPoly || m_images[imageIndex].m_isCsv);

  // Consider the case when the current layer is a polygon.
  // TODO(oalexan1): What if a layer has both an image and a polygon?

  if (poly_or_xyz) {
    if (!m_use_georef)
      return flip_in_y(R);
    return flip_in_y(m_image2world_trans[imageIndex].point_to_point_bbox(R));
  }

  if (!m_use_georef)
    return R;
  return flip_in_y(m_image2world_trans[imageIndex].pixel_to_point_bbox(R));
}

}} // namespace vw::gui
