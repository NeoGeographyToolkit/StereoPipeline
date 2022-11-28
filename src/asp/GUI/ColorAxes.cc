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

// Small auxiliary function
inline QColor rgb2color(vw::cm::Vector3u const& c) {
  return QColor(c[0], c[1], c[2]);
}

// Colormap based on a lookup table (lut)  
class LutColormap: public QwtLinearColorMap {
public:

  // Default constructor; will not be used
  LutColormap() {} 

  // Custom constructor
  LutColormap(std::map<float, vw::cm::Vector3u> const& lut_map) {

    // Sanity check: the first and last color keys must be 0 and 1.
    if (lut_map.empty() || lut_map.begin()->first != 0.0 || lut_map.rbegin()->first != 1.0)
      popUp("First colormap stop must be at 0.0 and last at 1.0.");

    // Must replace the automatically initialized endpoints
    setColorInterval(rgb2color(lut_map.begin()->second), rgb2color(lut_map.rbegin()->second));
    
    for (auto it = lut_map.begin(); it != lut_map.end(); it++) {
      
      if (it->first == 0.0 || it->first == 1.0) 
        continue; // endpoints already added
      
      auto const& c = it->second; // c has 3 indices between 0 and 255
      addColorStop(it->first, rgb2color(it->second));
    }
  }
  
};
  
class ColorAxesZoomer: public QwtPlotZoomer {
public:
  ColorAxesZoomer(QWidget *canvas): QwtPlotZoomer(dynamic_cast<QwtPlotCanvas *>(canvas)) {
    setTrackerMode(AlwaysOff);
  }
  
};

// Manages the image to display for the ColorAxes widget 
struct ColorAxesData: public QwtRasterData {
public:

  ColorAxesData(imageData & image): m_image(image) {
    // TODO(oalexan1): Need to handle georeferences.

    vw::mosaic::DiskImagePyramid<double> & img = image.img.m_img_ch1_double;
    
    if (img.planes() != 1) {
      // This will be caught before we get here, but is good to have for extra
      // robustness.
      popUp("Only images with one channel can be colorized.");
      return;
    }
    
    m_nodata_val = img.get_nodata_val();
    m_sub_scale = -1;  // This must be set properly before being used
    
    // TODO(oalexan1): How about removing a small percentile of intensity from ends?
    // TODO(oalexan1): Handle no-data properly by using transparent pixels.
    m_min_val = asp::stereo_settings().min;
    m_max_val = asp::stereo_settings().max;
    if (std::isnan(m_min_val) || std::isnan(m_max_val)) {

      // Get the lowest-resolution image version from the pyramid
      ImageView<double> lowres_img = m_image.img.m_img_ch1_double.pyramid().back();

      // Compute min and max if not specified by the user
      // TODO(oalexan1): Make this into a function
      m_min_val = std::numeric_limits<double>::max();
      m_max_val = -m_min_val;
      for (int col = 0; col < lowres_img.cols(); col++) {
        for (int row = 0; row < lowres_img.rows(); row++) {
          
          double val = lowres_img(col, row);
          if (val == m_nodata_val) 
            continue;
        
          m_min_val = std::min(m_min_val, val);
          m_max_val = std::max(m_max_val, val);
        }
      }
    }
    setInterval(Qt::XAxis, QwtInterval(0, img.cols() - 1));
    setInterval(Qt::YAxis, QwtInterval(0, img.rows() - 1));
    setInterval(Qt::ZAxis, QwtInterval(m_min_val, m_max_val));
  }
  
  virtual double value(double x, double y) const {
    
    // Instead of returning m_image(x, y), we will return
    // m_sub_image(x/m_sub_scale - m_beg_x, y/m_sub_scale - m_beg_y).
    if (m_sub_scale <= 0) 
      vw::vw_throw(vw::ArgumentErr() << "Programmer error. Not ready yet to render the image.\n");
    
    vw::mosaic::DiskImagePyramid<double> const& img = m_image.img.m_img_ch1_double;

    // Return pixels at the appropriate level of resolution
    x = round(x/m_sub_scale) - m_beg_x;
    y = round(y/m_sub_scale) - m_beg_y;

    if (x < 0 || y < 0 ||
        x > m_sub_image.cols() - 1 ||
        y > m_sub_image.rows() - 1)
      return m_min_val;
    
    double val = m_sub_image(x, y);
    
    if (val == m_nodata_val || val != val) 
      return m_min_val; // TODO(oalexan1): Make transparent

    return val;
  }

  imageData & m_image;
  double m_min_val, m_max_val, m_nodata_val;

  // Can get away by using a lower-res image version at this sub scale.
  int m_level, m_beg_x, m_beg_y;
  double m_sub_scale;
  ImageView<double> m_sub_image;
};

// Manages the plotting of the data at the correct resolution level
// Sources: https://qwt.sourceforge.io/qwt__plot__spectrogram_8cpp_source.html
//          https://qwt.sourceforge.io/qwt__plot__spectrogram_8h_source.html
class ColorAxesPlotter: public QwtPlotSpectrogram {
public:
  ColorAxesPlotter(ColorAxesData * data, const QString& title = QString()):
    m_data(data), QwtPlotSpectrogram(title) {}
  
 virtual void draw(QPainter * painter,
                   const QwtScaleMap & xMap,
                   const QwtScaleMap & yMap,
                   const QRectF      & canvasRect) const {
   
   QwtPlotSpectrogram::draw(painter, xMap, yMap, canvasRect);
 }

  virtual QImage renderImage(const QwtScaleMap & xMap,
                             const QwtScaleMap & yMap,
                             const QRectF & area, 
                             const QSize & imageSize) const {

    // Based on size of the canvas, determine the appropriate level of
    // the multi-resolution pyramid and the clip from it that will be
    // needed. Hence, do not provide the image at a much finer
    // resolution than what is displayed or read from disk more than
    // necessary.

    std::cout << "---noxw in render image " << std::endl;
    //std::cout << "area " << area << std::endl;
    std::cout << "--area x, y, wid ht " << area.x() << " " << area.y()
              << ' ' << area.width() << " " << area.height() << std::endl;
    std::cout << "render image size " << imageSize.width() << ' ' << imageSize.height() << std::endl;

    double tx0 = xMap.invTransform(0);
    double ty0 = yMap.invTransform(0);
    double tx1 = xMap.invTransform(imageSize.width());
    double ty1 = yMap.invTransform(imageSize.height());

    int beg_x = floor(std::min(tx0, tx1)), end_x = ceil(std::max(tx0, tx1));
    int beg_y = floor(std::min(ty0, ty1)), end_y = ceil(std::max(ty0, ty1));

    double sub_scale = std::min((end_x - beg_x) / imageSize.width(),
                            (end_y - beg_y) / imageSize.height());
      
    std::cout << "--x range from disk image ratio " << tx0 << ' ' << tx1
              << ' ' << std::abs(tx0 - tx1) / imageSize.width() << std::endl;
    std::cout << "--y range from disk image ratio " << ty0 << ' ' << ty1
              << ' ' << std::abs(ty0 - ty1) / imageSize.height() << std::endl;

    std::cout << "--sub_scale " << sub_scale << std::endl;
    m_data->m_level = m_data->m_image.img.m_img_ch1_double.pyramidLevel(sub_scale);
    std::cout << "--level is " << m_data->m_level << std::endl;

    m_data->m_sub_scale = round(pow(2.0, m_data->m_level));
    std::cout << "--scale " << m_data->m_sub_scale << std::endl;

    beg_x = floor(beg_x/m_data->m_sub_scale); end_x = ceil(end_x/m_data->m_sub_scale);
    beg_y = floor(beg_y/m_data->m_sub_scale); end_y = ceil(end_y/m_data->m_sub_scale);

    vw::BBox2i box;
    box.min() = Vector2i(beg_x, beg_y);
    box.max() = Vector2i(end_x + 1, end_y + 1); // because max is exclusive

    std::cout << "uncropped box " << box << std::endl;
    box.crop(vw::bounding_box(m_data->m_image.img.m_img_ch1_double.pyramid()[m_data->m_level]));

    std::cout << "--cropped box " << box << std::endl;

    // Instead of returning image(x, y), we will return sub_image(x/scale + beg_x, y/scale + beg_y).
    m_data->m_sub_image = crop(m_data->m_image.img.m_img_ch1_double.pyramid()[m_data->m_level], box);

    m_data->m_beg_x = box.min().x();
    m_data->m_beg_y = box.min().y();
    
    return QwtPlotSpectrogram::renderImage(xMap, yMap, area, imageSize);
  }
  
  ColorAxesData * m_data;  
};
  
ColorAxes::ColorAxes(QWidget *parent, imageData & image):
  QwtPlot(parent), m_image(image) {

  ColorAxesData * data = new ColorAxesData(m_image);

  // Have to pass the data twice, because the second such statement
  // does not know about the precise implementation and extra
  // functions of this class. But it is the second statement
  // which will manage the memory.
  m_plotter = new ColorAxesPlotter(data);
  m_plotter->setData(data);

  // Use system specific thread count
  m_plotter->setRenderThreadCount(0);

  // Parse and set the colormap
  std::map<float, vw::cm::Vector3u> lut_map;
  try {
    vw::cm::parse_color_style(m_image.colormap, lut_map);
  } catch (...) {
    popUp("Unknown colormap style: " + m_image.colormap);
    m_image.colormap = "binary-red-blue";
    vw::cm::parse_color_style(m_image.colormap, lut_map);
  }
  m_plotter->setColorMap(new LutColormap(lut_map));
  m_plotter->setCachePolicy(QwtPlotRasterItem::PaintCache);
  
  m_plotter->attach(this);
  
  // A color bar on the right axis. Must repeat the same
  // colormap as earlier.
  QwtScaleWidget *rightAxis = axisWidget(QwtPlot::yRight);
  QwtInterval zInterval = m_plotter->data()->interval(Qt::ZAxis);
  //rightAxis->setTitle("Intensity");
  rightAxis->setColorBarEnabled(true);
  rightAxis->setColorMap(zInterval, new LutColormap(lut_map));
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

  // Show it in image mode, not contour mode
  m_plotter->setDisplayMode(QwtPlotSpectrogram::ImageMode, true);
  
  replot();
  
  // LeftButton for the zooming
  // MidButton for the panning
  // RightButton: zoom out by 1
  // Ctrl+RighButton: zoom out to full size
  
  QwtPlotZoomer* zoomer = new ColorAxesZoomer(canvas());
  zoomer->setMousePattern(QwtEventPattern::MouseSelect2,
                          Qt::RightButton, Qt::ControlModifier);
  zoomer->setMousePattern(QwtEventPattern::MouseSelect3,
                          Qt::RightButton);
  
  const QColor c(Qt::darkBlue);
  zoomer->setRubberBandPen(c);
  zoomer->setTrackerPen(c);

  QwtPlotPanner *panner = new QwtPlotPanner(canvas());
  panner->setAxisEnabled(QwtPlot::yRight, false);
  panner->setMouseButton(Qt::MidButton);
  
  // Avoid jumping when labels with more/less digits
  // appear/disappear when scrolling vertically
  
  const QFontMetrics fm(axisWidget(QwtPlot::yLeft)->font());
  QwtScaleDraw *sd = axisScaleDraw(QwtPlot::yLeft);
  sd->setMinimumExtent(fm.width("100.00"));
  
}

}} // end namespace vw::gui

