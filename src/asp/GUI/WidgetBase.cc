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

namespace vw { namespace gui {

WidgetBase::WidgetBase(int beg_image_id, int end_image_id,
             int base_image_id,
             bool use_georef,
             std::vector<imageData> & images):
    m_beg_image_id(beg_image_id),
    m_end_image_id(end_image_id),
    m_base_image_id(base_image_id), 
    m_use_georef(use_georef),
    m_images(images) {

    int num_images = m_images.size();
    m_world2image_geotransforms.resize(num_images);
    m_image2world_geotransforms.resize(num_images);
}
  
// Convert from world coordinates to projected coordinates in given geospatial
// projection
vw::Vector2 WidgetBase::world2projpoint(vw::Vector2 P, int imageIndex) const {
  if (!m_use_georef)
      return flip_in_y(P);
  return m_world2image_geotransforms[imageIndex].point_to_point(flip_in_y(P)); 
}

// The reverse of world2projpoint
vw::Vector2 WidgetBase::projpoint2world(vw::Vector2 P, int imageIndex) const {
  if (!m_use_georef)
    return flip_in_y(P);
  return flip_in_y(m_image2world_geotransforms[imageIndex].point_to_point(P));
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

}} // namespace vw::gui
