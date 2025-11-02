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


/// \file ColorAxes.h
///
/// A widget showing colorized data with a colormap and axes
///

#ifndef __STEREO_GUI_COLOR_AXES_H__
#define __STEREO_GUI_COLOR_AXES_H__

#include <QWidget>
#include <qwt_plot.h>
#include <qwt_plot_spectrogram.h>

#include <asp/GUI/WidgetBase.h>

class QwtPlotZoomer;
class QContextMenuEvent;
class QMenu;
class QAction;
class QContextMenuEvent;

namespace asp {

class imageData;
class ColorAxesPlotter;
  
class ColorAxes: public QwtPlot, public WidgetBase {
  Q_OBJECT
  
public:
  ColorAxes(QWidget * parent, int beg_image_id, int end_image_id, int base_image_id,
            asp::AppData & data,
            bool use_georef,
            std::vector<imageData> & images,
            std::vector<vw::cartography::GeoTransform> & world2image,
            std::vector<vw::cartography::GeoTransform> & image2world);

  virtual void mousePressEvent(QMouseEvent *e);
  virtual void resizeEvent(QResizeEvent *e);
  void sizeToFit();

public slots:

  // Se the min and max intensity values for the data to be plotted
  void setMinMaxIntensity(); 

private:
  ColorAxesPlotter *m_plotter;
  QwtPlotZoomer* m_zoomer;

  // Spatial extent of the data to be plotted
  double m_min_x, m_min_y, m_max_x, m_max_y;

  // Context menu
  QMenu  * m_ContextMenu;
  void contextMenuEvent(QContextMenuEvent *event);
  
  QAction* m_setMinMaxIntensityAction;
};

} // namespace asp
  
#endif  // __STEREO_GUI_COLOR_AXES_H__
