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

/// \file ColorAxes.cc

#include <qwt_color_map.h>
#include <qwt_plot_spectrogram.h>
#include <qwt_scale_widget.h>
#include <qwt_scale_draw.h>
#include <qwt_plot_zoomer.h>
#include <qwt_plot_panner.h>
#include <qwt_plot_layout.h>
#include <qwt_plot_renderer.h>
#include <qwt_plot_canvas.h>

#include <asp/GUI/ColorAxes.h>

class MyZoomer: public QwtPlotZoomer {
public:
  MyZoomer(QWidget *canvas): QwtPlotZoomer(dynamic_cast<QwtPlotCanvas *>(canvas)) {
    setTrackerMode(AlwaysOff);
  }
  
};

class SpectrogramData: public QwtRasterData {
public:
    SpectrogramData() {
        setInterval(Qt::XAxis, QwtInterval(-1.5, 1.5));
        setInterval(Qt::YAxis, QwtInterval(-1.5, 1.5));
        setInterval(Qt::ZAxis, QwtInterval(0.0, 10.0));
    }

    virtual double value(double x, double y) const {
        const double c = 0.842;

        const double v1 = x * x + (y - c) * (y + c);
        const double v2 = x * (y + c) + x * (y + c);

        return 1.0 / (v1 * v1 + v2 * v2);
    }
};

class ColorMap: public QwtLinearColorMap {
public:
    ColorMap():
      QwtLinearColorMap(Qt::darkCyan, Qt::red) {
      addColorStop(0.1, Qt::cyan);
      addColorStop(0.6, Qt::green);
      addColorStop(0.95, Qt::yellow);
    }
};

ColorAxes::ColorAxes(QWidget *parent): QwtPlot(parent) {
  m_spectrogram = new QwtPlotSpectrogram();
  m_spectrogram->setRenderThreadCount(0); // use system specific thread count
  
  m_spectrogram->setColorMap(new ColorMap());
  m_spectrogram->setCachePolicy(QwtPlotRasterItem::PaintCache);
  
  m_spectrogram->setData(new SpectrogramData());
  m_spectrogram->attach(this);
  
  const QwtInterval zInterval = m_spectrogram->data()->interval(Qt::ZAxis);
  // A color bar on the right axis
  QwtScaleWidget *rightAxis = axisWidget(QwtPlot::yRight);
  rightAxis->setTitle("Intensity");
  rightAxis->setColorBarEnabled(true);
  rightAxis->setColorMap(zInterval, new ColorMap());
  
  setAxisScale(QwtPlot::yRight, zInterval.minValue(), zInterval.maxValue());
  enableAxis(QwtPlot::yRight);
  
  plotLayout()->setAlignCanvasToScales(true);

  // Show it
  m_spectrogram->setDisplayMode(QwtPlotSpectrogram::ImageMode, true);
  
  replot();
  
  // LeftButton for the zooming
  // MidButton for the panning
  // RightButton: zoom out by 1
  // Ctrl+RighButton: zoom out to full size
  
  QwtPlotZoomer* zoomer = new MyZoomer(canvas());
  zoomer->setMousePattern(QwtEventPattern::MouseSelect2,
                          Qt::RightButton, Qt::ControlModifier);
  zoomer->setMousePattern(QwtEventPattern::MouseSelect3,
                          Qt::RightButton);
  
  QwtPlotPanner *panner = new QwtPlotPanner(canvas());
  panner->setAxisEnabled(QwtPlot::yRight, false);
  panner->setMouseButton(Qt::MidButton);
  
  // Avoid jumping when labels with more/less digits
  // appear/disappear when scrolling vertically
  
  const QFontMetrics fm(axisWidget(QwtPlot::yLeft)->font());
  QwtScaleDraw *sd = axisScaleDraw(QwtPlot::yLeft);
  sd->setMinimumExtent(fm.width("100.00"));
  
  const QColor c(Qt::darkBlue);
  zoomer->setRubberBandPen(c);
  zoomer->setTrackerPen(c);
}

