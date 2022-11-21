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

// TODO(oalexan1): Reconcile this with MainWidget.cc so that different
// images and curves can be overlayed while using a colormap and axes.

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
#include <asp/GUI/GuiUtilities.h>
#include <asp/GUI/GuiBase.h>
#include <asp/Core/StereoSettings.h>
#include <vw/Image/Colormap.h> // colormaps supported by ASP

namespace vw { namespace gui { 
class MyZoomer: public QwtPlotZoomer {
public:
  MyZoomer(QWidget *canvas): QwtPlotZoomer(dynamic_cast<QwtPlotCanvas *>(canvas)) {
    setTrackerMode(AlwaysOff);
  }
  
};

class SpectrogramData: public QwtRasterData {
public:
  SpectrogramData(imageData & image): m_image(image) {
    // TODO(oalexan1): Need to handle georeferences.
    // TODO(oalexan1): Temporary. Need to write an analog of formQimage(),
    // to get a floating point image at given resolution level.
    vw::mosaic::DiskImagePyramid<double> & img = image.img.m_img_ch1_double;

    // TODO(oalexan1): This is temporary. Need to make out-of-range regions transparent.
    m_nodata_val = img.get_nodata_val();
    ImageViewRef<PixelMask<double>> full_img = create_mask(img.bottom(), m_nodata_val);
    m_interp_image = interpolate(full_img, BilinearInterpolation(), ZeroEdgeExtension());

    // TODO(oalexan1): This is temporary
    m_image_copy = copy(full_img);
    
    // TODO(oalexan1): How about removing a small percentile from ends
    // TODO(oalexan1): Handle no-data properly by using transparent pixels.

    m_min_val = asp::stereo_settings().min;
    m_max_val = asp::stereo_settings().max;
    if (std::isnan(m_min_val) || std::isnan(m_max_val)) {

      // Compute min and max if not specified by the user
      m_min_val = std::numeric_limits<double>::max();
      m_max_val = -m_min_val;
      for (int col = 0; col < full_img.cols(); col++) {
        for (int row = 0; row < full_img.rows(); row++) {
          
          auto val = full_img(col, row); // alias
          if (!is_valid(val)) 
            continue;
        
          m_min_val = std::min(m_min_val, val.child());
          m_max_val = std::max(m_max_val, val.child());
        }
      }
    }
    
    setInterval(Qt::XAxis, QwtInterval(0, full_img.cols() - 1));
    setInterval(Qt::YAxis, QwtInterval(0, full_img.rows() - 1));
    setInterval(Qt::ZAxis, QwtInterval(m_min_val, m_max_val));
  }
  
  virtual double value(double x, double y) const {
    //auto val = m_interp_image(x, y);
    x = round(x);
    y = round(y);
    if (x < 0 || y < 0 || x > m_image_copy.cols() - 1 || y > m_image_copy.rows() - 1)
      return m_min_val;
    
    auto val = m_image_copy(x, y);

    if (!is_valid(val)) 
      return m_min_val;     // TODO(oalexan1): Make transparent

    return val.child();
  }

private:
  imageData & m_image;
  double m_min_val, m_max_val, m_nodata_val;
  ImageViewRef<PixelMask<double>> m_interp_image;
  ImageView<PixelMask<double>> m_image_copy; // TODO(oalexan1): This needs to be removed
};

class ColorMap: public QwtLinearColorMap {
public:
  ColorMap():
    QwtLinearColorMap(Qt::darkCyan, Qt::red) {
    addColorStop(0.1, Qt::cyan);
    addColorStop(0.6, Qt::green);
    addColorStop(0.95, Qt::yellow);
  }

  ColorMap(std::map<float, vw::cm::Vector3u> const& lut_map) {

    if (lut_map.empty() || lut_map.begin()->first != 0.0 || lut_map.rbegin()->first != 1.0)
      popUp("First colormap stop must be at 0.0 and last at 1.0.");
    
    for (auto it = lut_map.begin(); it != lut_map.end(); it++) {
      auto const& c = it->second; // c has 3 indices between 0 and 255
      addColorStop(it->first, QColor(c[0], c[1], c[2]));
    }
  }

};

ColorAxes::ColorAxes(QWidget *parent, imageData & image):
  QwtPlot(parent), m_image(image) {
  m_spectrogram = new QwtPlotSpectrogram();
  m_spectrogram->setRenderThreadCount(0); // use system specific thread count

  // Parse and set the colormap
  std::map<float, vw::cm::Vector3u> lut_map;
  try {
    vw::cm::parse_color_style(m_image.colormap, lut_map);
  } catch (...) {
    popUp("Unknown colormap style: " + m_image.colormap);
    m_image.colormap = "binary-red-blue";
    vw::cm::parse_color_style(m_image.colormap, lut_map);
  }
  ColorMap * colormap = new ColorMap(lut_map);
  m_spectrogram->setColorMap(colormap);
  m_spectrogram->setCachePolicy(QwtPlotRasterItem::PaintCache);
  
  m_spectrogram->setData(new SpectrogramData(m_image));
  m_spectrogram->attach(this);
  
  // A color bar on the right axis. Must repeat the same
  // colormap as earlier.
  QwtScaleWidget *rightAxis = axisWidget(QwtPlot::yRight);
  QwtInterval zInterval = m_spectrogram->data()->interval(Qt::ZAxis);
  //rightAxis->setTitle("Intensity");
  rightAxis->setColorBarEnabled(true);
  rightAxis->setColorMap(zInterval, colormap);
  rightAxis->setColorBarWidth(30);

  // TODO(oalexan1): Disable auto-scaling but ensure the colorbar is
  // next to the plots rather than leaving a large gap
  
  setAxisScale(QwtPlot::yRight, zInterval.minValue(), zInterval.maxValue());
  enableAxis(QwtPlot::yRight);
  setAxisAutoScale(QwtPlot::yRight, true);

  QwtScaleWidget * bottomAxis = axisWidget(QwtPlot::xBottom);
  setAxisAutoScale(QwtPlot::xBottom, true);
    
  // TODO(oalexan1): What does this do?
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

}} // end namespace vw::gui

