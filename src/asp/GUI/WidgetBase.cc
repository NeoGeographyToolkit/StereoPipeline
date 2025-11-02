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

namespace asp {

WidgetBase::WidgetBase(int beg_image_id, int end_image_id, int base_image_id,
                       asp::AppData & data):
    m_beg_image_id(beg_image_id),
    m_end_image_id(end_image_id),
    app_data(data),
    m_base_image_id(base_image_id), 
    m_world_box(vw::BBox2()), m_border_factor(0.95) {
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
vw::Vector2 WidgetBase::world2screen(vw::Vector2 const& p) const {

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
// See app_data.world2image_trans() for the definition.
vw::Vector2 WidgetBase::screen2world(vw::Vector2 const& p) const {

  // First undo the empty border margin
  double x = p.x(), y = p.y();
  x = (x - m_window_width /2.0)/m_border_factor + m_window_width /2.0;
  y = (y - m_window_height/2.0)/m_border_factor + m_window_height/2.0;

  // Scale to world coordinates
  x = m_current_view.min().x() + (m_current_view.width () * x / m_window_width);
  y = m_current_view.min().y() + (m_current_view.height() * y / m_window_height);

  return vw::Vector2(x, y);
}

vw::BBox2 WidgetBase::screen2world(vw::BBox2 const& R) const {
  if (R.empty()) return R;
  vw::Vector2 A = screen2world(R.min());
  vw::Vector2 B = screen2world(R.max());
  return vw::BBox2(A, B);
}

vw::BBox2 WidgetBase::world2screen(vw::BBox2 const& R) const {
  if (R.empty()) return R;
  vw::Vector2 A = world2screen(R.min());
  vw::Vector2 B = world2screen(R.max());
  return vw::BBox2(A, B);
}

} // namespace asp
