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
#include <qwt_plot_rescaler.h>
#include <qwt_point_data.h>
#include <qwt_interval.h>
#include <qwt_matrix_raster_data.h>
#include <qwt_scale_map.h>
#include <QResizeEvent>
#include <QMouseEvent>
#include <QPen>

#include <asp/GUI/ColorAxes.h>
#include <asp/GUI/GuiUtilities.h>
#include <asp/GUI/GuiBase.h>
#include <asp/Core/StereoSettings.h>
#include <vw/Image/Colormap.h> // colormaps supported by ASP

namespace vw { namespace gui {

std::string QRectFToStr(QRectF const& box) {
  std::ostringstream os;
  os << "minx, miny, width, height " << box.left() << ' ' << box.top() << ' '
     << box.width() << ' ' << box.height();
  return os.str();
}

// TODO(oalexan1): Integrate this with MainWidget::expand_box_to_keep_aspect_ratio()
QRectF expand_box_to_aspect_ratio(QRectF const& in_box, double aspect_ratio) {
    
  QRectF out_box = in_box;
  
  if (out_box.isEmpty())
    return QRectF(0, 0, 1, aspect_ratio); // if it came to worst

  if (in_box.width() / in_box.height() < aspect_ratio) {
    // Width needs to grow
    double new_width = in_box.height() * aspect_ratio;
    double delta = (new_width - in_box.width())/2.0;
    out_box.setLeft(in_box.left() - delta); 
    out_box.setRight(in_box.right() + delta); 
  }else if (in_box.width() / in_box.height() > aspect_ratio) {
    // Height needs to grow
    double new_height = in_box.width() / aspect_ratio;
    double delta = (new_height - in_box.height())/2.0;
    out_box.setTop(in_box.top() - delta); 
    out_box.setBottom(in_box.bottom() + delta); 
  }

  return out_box;
}
  
  
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
ColorAxesZoomer(QWidget *canvas, double aspect_ratio):
  QwtPlotZoomer(dynamic_cast<QwtPlotCanvas *>(canvas)), m_aspect_ratio(aspect_ratio) {
  setTrackerMode(AlwaysOff);
}
  
virtual void zoom(const QRectF& rect) {
  // Need this to maintain the aspect ratio when zooming
  QRectF grown_rect = expand_box_to_aspect_ratio(rect, m_aspect_ratio);
  QwtPlotZoomer::zoom(grown_rect);
}

virtual void zoom(int offset) {
  QRectF box = zoomRect();
  QwtPlotZoomer::zoom(offset);
}

private:
  double m_aspect_ratio;
}; // end class ColorAxesZoomer
  
// Manages the image to display for the ColorAxes widget 
class ColorAxesData: public QwtMatrixRasterData {

private:
  
  void calcLowResMinMax(double& min_val, double& max_val) const {
    
    // TODO(oalexan1): How about removing a small percentile of intensity from ends?

    // Get the lowest-resolution image version from the pyramid
    ImageView<double> lowres_img = m_image.img.m_img_ch1_double.pyramid().back();

    min_val = std::numeric_limits<double>::max();
    max_val = -min_val;
    for (int col = 0; col < lowres_img.cols(); col++) {
      for (int row = 0; row < lowres_img.rows(); row++) {
        
        double val = lowres_img(col, row);
        if (val == m_nodata_val) 
          continue;
        
        min_val = std::min(min_val, val);
        max_val = std::max(max_val, val);
      }
    }

    if (min_val > max_val) {
      // If the image turned out to be empty
      min_val = m_nodata_val;
      max_val = m_nodata_val;
    }
    
    Vector2 approx_bounds = m_image.img.m_img_ch1_double.get_approx_bounds();
    // The approx_bounds are computed on the lowest resolution level
    // of the pyramid and are likely exaggerated, but were computed
    // with outlier removal.  Use them to adjust the existing bounds
    // which may have outliers.
    // TODO(oalexan1): Integrate this with formQimage logic.
    if (approx_bounds[0] < approx_bounds[1]) {
      min_val = std::max(min_val, approx_bounds[0]);
      max_val = std::min(max_val, approx_bounds[1]);
    }
    
    return;
  }
  
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

    // Some initializations
    m_nodata_val = img.get_nodata_val();
    m_sub_scale = -1;  // This must be set properly before being used
    m_beg_x = 0;
    m_beg_y = 0;
    
    // TODO(oalexan1): Handle no-data properly by using transparent pixels.
    m_min_val = asp::stereo_settings().min;
    m_max_val = asp::stereo_settings().max;
    if (std::isnan(m_min_val) || std::isnan(m_max_val)) // if the user did not set these
      calcLowResMinMax(m_min_val, m_max_val);

    QwtMatrixRasterData::setInterval(Qt::XAxis, QwtInterval(0, img.cols() - 1));
    QwtMatrixRasterData::setInterval(Qt::YAxis, QwtInterval(0, img.rows() - 1));
    QwtMatrixRasterData::setInterval(Qt::ZAxis, QwtInterval(m_min_val, m_max_val));
  }

  // Given that we plan to render a portion of an image on disk within these
  // bounds, and resulting in a given image size, read from disk a clip with resolution
  // and extent just enough for the job, or a little higher res and bigger.
  void prepareClip(double x0, double y0, double x1, double y1, QSize const& imageSize) {

    // Note that y0 and y1 are normally flipped
    int beg_x = floor(std::min(x0, x1)), end_x = ceil(std::max(x0, x1));
    int beg_y = floor(std::min(y0, y1)), end_y = ceil(std::max(y0, y1));

    // if in doubt, go with lower sub_scale, so higher resolution.
    double sub_scale = std::min((end_x - beg_x) / imageSize.width(),
                            (end_y - beg_y) / imageSize.height());
      
    m_level = m_image.img.m_img_ch1_double.pyramidLevel(sub_scale);
    m_sub_scale = round(pow(2.0, m_level));

    beg_x = floor(beg_x/m_sub_scale); end_x = ceil(end_x/m_sub_scale);
    beg_y = floor(beg_y/m_sub_scale); end_y = ceil(end_y/m_sub_scale);

    vw::BBox2i box;
    box.min() = Vector2i(beg_x, beg_y);
    box.max() = Vector2i(end_x + 1, end_y + 1); // because max is exclusive
    box.crop(vw::bounding_box(m_image.img.m_img_ch1_double.pyramid()[m_level]));

    // Instead of returning image(x, y), we will return
    // sub_image(x/scale + beg_x, y/scale + beg_y).
    m_sub_image = crop(m_image.img.m_img_ch1_double.pyramid()[m_level], box);

    m_beg_x = box.min().x();
    m_beg_y = box.min().y();
    
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
        x > m_sub_image.cols() - 1 || y > m_sub_image.rows() - 1)
      return m_min_val;
    
    double val = m_sub_image(x, y);
    
    if (val == m_nodata_val || val != val) 
      return m_min_val; // TODO(oalexan1): Make transparent

    return val;
  }
  
private:
  
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

   // canvasRect is the region, in screen pixel units, where the image
   // will go. If the image is narrow, it may not fill fully the
   // canvas. If the labels on the axes take up more space when
   // zooming, the image region will be affected. So, it is not
   // fixed once and for all.
   // std::cout << "--canvasRect " << canvasRect.left() << ' ' << canvasRect.top() << ' ' << canvasRect.width() << ' ' << canvasRect.height() << std::endl;
   
   QwtPlotSpectrogram::draw(painter, xMap, yMap, canvasRect);
 }

virtual QImage renderImage(const QwtScaleMap & xMap,
                           const QwtScaleMap & yMap,
                           const QRectF & area,
                           const QSize & imageSize) const {

  // 'area' is in world units, not in pixel units
  // imageSize has the dimensions, in pixels, of the canvas portion having the image
  
  // Based on size of the rendered image, determine the appropriate level of
  // resolution and extent to read from disk. This greatly helps with
  // reducing memory usage and latency.
  m_data->prepareClip(xMap.invTransform(0),
                      yMap.invTransform(0),
                      xMap.invTransform(imageSize.width()),
                      yMap.invTransform(imageSize.height()),
                      imageSize);
  
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
  
  // A color bar on the right axis. Must create a new colormap object
  // with the same data, to avoid a crash if using the same one.
  QwtScaleWidget *rightAxis = axisWidget(QwtPlot::yRight);
  QwtInterval zInterval = m_plotter->data()->interval(Qt::ZAxis);
  //rightAxis->setTitle("Intensity");
  rightAxis->setColorBarEnabled(true);
  rightAxis->setColorMap(zInterval, new LutColormap(lut_map));
  rightAxis->setColorBarWidth(30);

  m_plotter->setCachePolicy(QwtPlotRasterItem::PaintCache);
  m_plotter->attach(this);

  // TODO(oalexan1): Make equal aspect ratio work
  setAxisScale(QwtPlot::yRight, zInterval.minValue(), zInterval.maxValue());

  // These two scales will be over-ridden later
  setAxisScale(QwtPlot::yLeft, m_image.img.rows(), 0); // y axis goes down
  setAxisScale(QwtPlot::xBottom, 0, m_image.img.cols());
  
  enableAxis(QwtPlot::yRight);
  setAxisAutoScale(QwtPlot::yRight, false);
  setAxisAutoScale(QwtPlot::xBottom, false);
  setAxisAutoScale(QwtPlot::yLeft, false);
  plotLayout()->setAlignCanvasToScales(true);

  // Show it in image mode, not contour mode
  m_plotter->setDisplayMode(QwtPlotSpectrogram::ImageMode, true);

  // An attempt to find the true canvas dimensions.
  // https://qwt-interest.narkive.com/cquDtLQ5/qwtplot-s-initial-canvas-size
  // This is not enough. Only in renderImage() we will truly know the image size.
  plotLayout()->setCanvasMargin(0);
  plotLayout()->setAlignCanvasToScales(true);
  
  replot();
}

bool first_resize_event = true;

void ColorAxes::mousePressEvent(QMouseEvent *e) {
  QwtPlot::mousePressEvent(e);
}
  
void ColorAxes::resizeEvent(QResizeEvent *e) {

  // Try to ensure equal aspect ratio. For this, must know the paint canvas
  // size. Compute this based on intended window size.
  // https://qwt-interest.narkive.com/cquDtLQ5/qwtplot-s-initial-canvas-size
  QRectF resizedWin(frameWidth(), frameWidth(),
                    e->size().width() - frameWidth(),
                    e->size().height() - frameWidth());
  plotLayout()->activate(this, resizedWin);
  auto rect = plotLayout()->canvasRect();
  
  // Note: This is not purely the image area, as it bigger
  // by a few pixels than the area having the image later, but it is good
  // enough given that its size is on the order of 1000 pixels.
  // The true image area is known only in renderImage(), so much later.
  // The scales should be set up before that.
  double aspect_ratio = double(rect.width()) / double(rect.height());
  QRectF in_box(0, 0, m_image.img.cols(), m_image.img.rows()); 
  QRectF box = expand_box_to_aspect_ratio(in_box, aspect_ratio);
  
  // Adjust the scales accordingly
  setAxisScale(QwtPlot::yLeft, box.height(), 0); // y axis goes down
  setAxisScale(QwtPlot::xBottom, 0, box.width());

  // Set up the zooming and panning after we know the aspect ratio.
  // This makes zooming out work correctly.
  
  // LeftButton for the zooming
  // MidButton for the panning
  // RightButton: zoom out by 1
  // Ctrl+RighButton: zoom out to full size
  
  QwtPlotZoomer* zoomer = new ColorAxesZoomer(canvas(), aspect_ratio);
  zoomer->setMousePattern(QwtEventPattern::MouseSelect2,
                          Qt::RightButton, Qt::ControlModifier);
  zoomer->setMousePattern(QwtEventPattern::MouseSelect3,
                          Qt::RightButton);
  
  const QColor c(Qt::darkBlue);
  zoomer->setRubberBandPen(QPen(c));
  zoomer->setTrackerPen(QPen(c));

  QwtPlotPanner *panner = new QwtPlotPanner(canvas());
  panner->setAxisEnabled(QwtPlot::yRight, false);
  panner->setMouseButton(Qt::MidButton);
  
  // Avoid jumping when labels with more/less digits
  // appear/disappear when scrolling vertically
  
  const QFontMetrics fm(axisWidget(QwtPlot::yLeft)->font());
  QwtScaleDraw *sd = axisScaleDraw(QwtPlot::yLeft);
  sd->setMinimumExtent(fm.width("100.00"));

  // Call the parent resize function to continue the work
  QwtPlot::resizeEvent(e);
}
  
}} // end namespace vw::gui

