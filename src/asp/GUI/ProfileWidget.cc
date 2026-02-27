// __BEGIN_LICENSE__
//  Copyright (c) 2006-2024, United States Government as represented by the
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

// \file ProfileWidget.cc
// 
// Member functions of MainWidget that have to do with profing.

#include <asp/GUI/MainWidget.h>
#include <asp/GUI/GuiGeom.h>

#include <QtGui>
#include <QtWidgets>

namespace asp {
  
// We assume the user picked n points in the image.
// Draw n-1 segments in between them. Plot the obtained profile.
void MainWidget::plotProfile(std::vector<imageData> const& images,
                              // indices in the image to profile
                              std::vector<double> const& profileX,
                              std::vector<double> const& profileY) {

  if (images.empty()) return; // nothing to do

  // Create the profile window
  if (m_profile.plot == NULL)
    m_profile.plot = new ProfilePlotter(this);

  int imgInd = m_beg_image_id; // just one image is present
  double nodata_val = images[imgInd].img().get_nodata_val();

  m_profile.valsX.clear(); m_profile.valsY.clear();
  int count = 0;

  int num_pts = profileX.size();
  for (int pt_iter = 0; pt_iter < num_pts; pt_iter++) {

    // Nothing to do if we are at the last point, unless
    // there is only one point.
    if (num_pts > 1 && pt_iter == num_pts - 1) continue;

   vw::Vector2 pt(profileX[pt_iter], profileY[pt_iter]);
   vw::Vector2 begP = app_data.world2image_trans(pt, imgInd);

    vw::Vector2 endP;
    if (num_pts == 1) {
      endP = begP; // only one point is present
    } else {
      vw::Vector2 pt(profileX[pt_iter+1], profileY[pt_iter+1]);
      endP = app_data.world2image_trans(pt, imgInd);
    }

    int begX = begP.x(),   begY = begP.y();
    int endX = endP.x(),   endY = endP.y();
    int seg_len = std::abs(begX - endX) + std::abs(begY - endY);
    if (seg_len == 0) seg_len = 1; // ensure it is never empty
    for (int p = 0; p <= seg_len; p++) {
      double t = double(p)/seg_len;
      int x = round(begX + t*(endX - begX));
      int y = round(begY + t*(endY - begY));
      bool is_in = (x >= 0 && x <= images[imgInd].img().cols()-1 &&
                    y >= 0 && y <= images[imgInd].img().rows()-1);
      if (!is_in)
        continue;

      double pixel_val = images[imgInd].img().get_value_as_double(x, y);

      // TODO: Deal with this NAN
      if (pixel_val == nodata_val)
        pixel_val = std::numeric_limits<double>::quiet_NaN();
      m_profile.valsX.push_back(count);
      m_profile.valsY.push_back(pixel_val);
      count++;
    }

  }

  if (num_pts == 1) {
    // Just one point, really
    m_profile.valsX.resize(1);
    m_profile.valsY.resize(1);
  }

  // Wipe whatever was there before
  m_profile.plot->detachItems();

  QwtPlotCurve * curve = new QwtPlotCurve("1D Profile");
  m_profile.plot->setFixedWidth(300);
  m_profile.plot->setWindowTitle("1D Profile");

  if (!m_profile.valsX.empty()) {

    double min_x = *std::min_element(m_profile.valsX.begin(), m_profile.valsX.end());
    double max_x = *std::max_element(m_profile.valsX.begin(), m_profile.valsX.end());
    double min_y = *std::min_element(m_profile.valsY.begin(), m_profile.valsY.end());
    double max_y = *std::max_element(m_profile.valsY.begin(), m_profile.valsY.end());

    // Ensure the window is always valid
    double small = 0.1;
    if (min_x == max_x) {
      min_x -= small; max_x += small;
    }
    if (min_y == max_y) {
      min_y -= small; max_y += small;
    }

    // Plot a point as a fat dot
    if (num_pts == 1) {
      curve->setStyle(QwtPlotCurve::Dots);
    }

    curve->setData(new QwtCPointerData<double>(&m_profile.valsX[0], &m_profile.valsY[0], m_profile.valsX.size()));
    curve->setPen(* new QPen(Qt::red));
    curve->attach(m_profile.plot);

    double delta = 0.1;  // expand a bit right to see more x and y labels
    double widx = max_x - min_x;
    double widy = max_y - min_y;
    m_profile.plot->setAxisScale(QwtPlot::xBottom, min_x - delta*widx, max_x + delta*widx);
    m_profile.plot->setAxisScale(QwtPlot::yLeft,   min_y - delta*widy, max_y + delta*widy);
  }

  // Finally, refresh the plot
  m_profile.plot->replot();
  m_profile.plot->show();
}

void MainWidget::setProfileMode(bool profile_mode) {
  m_profile.mode = profile_mode;

  if (!m_profile.mode) {
    // Clean up any profiling related info
    m_profile.x.clear();
    m_profile.y.clear();

    // Close the window.
    if (m_profile.plot != NULL) {
      m_profile.plot->close();
      m_profile.plot->deleteLater();
      delete m_profile.plot;
      m_profile.plot = NULL;
    }

    // Call back to the main window and tell it to uncheck the profile
    // mode checkbox.
    emit uncheckProfileModeCheckbox();
    return;
  } else {

    bool refresh = true;
    setPolyEditMode(false, refresh);

    // Load the data if not loaded already
    for (size_t it = 0; it < app_data.images.size(); it++)
      app_data.images[it].load();

    // Show the profile window
    MainWidget::plotProfile(app_data.images, m_profile.x, m_profile.y);
  }

  refreshPixmap();
}
  
} // end namespace asp
