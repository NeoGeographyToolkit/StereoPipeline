// __BEGIN_LICENSE__
//  Copyright (c) 2006-2013, United States Government as represented by the
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

/// \file WidgetBase.h
///
/// This has logic that is used both in MainWidget and in the ColorAxes classes.
///
#ifndef __STEREO_GUI_WIDGET_BASE_H__
#define __STEREO_GUI_WIDGET_BASE_H__

#include <QObject> // to avoid errors about boost and Qobject

// ASP
#include <asp/GUI/GuiUtilities.h>
#include <asp/GUI/AppData.h>

// Vision Workbench
#include <vw/Math/BBox.h>
#include <vw/Math/Vector.h>
#include <vw/Cartography/GeoReference.h>
#include <vw/Cartography/GeoTransform.h>

#include <string>
#include <vector>
#include <list>
#include <set>

namespace asp {

class WidgetBase {

public:

  // Constructors/Destructor
  WidgetBase(int beg_image_id, int end_image_id,
             int base_image_id,
             asp::AppData & data,
             bool use_georef,
             std::vector<imageData> & images,
             std::vector<vw::cartography::GeoTransform> & world2image,
             std::vector<vw::cartography::GeoTransform> & image2world);
  
  virtual ~WidgetBase(){}

  // The box which contains fully all images in the current widget,
  // in world coordinates.
  vw::BBox2 m_world_box;

  // The box in world coordinates which has the current view and
  // last view.  This is normally smaller than m_world_box.
  vw::BBox2 m_current_view, m_last_view;

  // Dimensions and stats
  int m_window_width;  // the width  of the plotting window in screen pixels
  int m_window_height; // the height of the plotting window in screen pixels

  // Shrink the image to be shown on screen by this factor
  // (typically 0.90 to 0.95) to create an extra empty margin at a widget's
  // border, to make it easier to zoom.
  double m_border_factor;

  int m_beg_image_id;  // The id of the first image among images in this widget
  int m_end_image_id;  // The id of the image past the last image among images

  // The index of the image on top of which the rest are overlaid.
  // We will render in this image's pixel or projected domain. This
  // only becomes important if using georeference, and the images
  // have different projections.
  int m_base_image_id;

  // A reference to the shared application data
  asp::AppData & app_data;

  // Convert from world coordinates to projected coordinates in given geospatial
  // projection, and vice versa
  vw::Vector2 world2proj(vw::Vector2 const P, int imageIndex) const;
  vw::Vector2 proj2world(vw::Vector2 const P, int imageIndex) const;

  vw::Vector2 world2screen(vw::Vector2 const&  p) const;
  vw::Vector2 screen2world(vw::Vector2 const&  p) const;
  vw::Vector2 world2image(vw::Vector2 const& P, int imageIndex) const;
  vw::Vector2 image2world(vw::Vector2 const& P, int imageIndex) const;

  vw::BBox2 world2screen(vw::BBox2 const& R) const;
  vw::BBox2 screen2world(vw::BBox2 const& R) const;
  vw::BBox2 world2image(vw::BBox2 const& R, int imageIndex) const;
  vw::BBox2 image2world(vw::BBox2 const& R, int imageIndex) const;

}; // End class WidgetBase

// Find the min and max values, ignoring outliers. We look only 
// at the last component of each point, as that has the intensity,
// while the previous two have the position.
// Keep this here as it is used only for plotting in widget code
void findRobustBounds(std::vector<vw::Vector3> const& scattered_data,
                      double & min_val, double & max_val);

} // namespace asp

#endif  // __STEREO_GUI_WIDGET_BASE_H__
