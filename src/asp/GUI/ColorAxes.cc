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
#include <QPainter>

#include <asp/GUI/ColorAxes.h>
#include <asp/GUI/GuiUtilities.h>
#include <asp/GUI/GuiBase.h>
#include <asp/Core/StereoSettings.h>
#include <vw/Image/Colormap.h> // colormaps supported by ASP
#include <vw/Core/Stopwatch.h>

namespace vw { namespace gui {

std::string QRectFToStr(QRectF const& box) {
  std::ostringstream os;
  os << "minx, miny, width, height " << box.left() << ' ' << box.top() << ' '
     << box.width() << ' ' << box.height();
  return os.str();
}

const double g_adjust_tol = 1e-3;

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

// The background will be set to black. This will be used in a few places
vw::cm::Vector3u bgColor(0, 0, 0);

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

    // To be able to deal with no-data, set the starting color to the
    // background. We will make sure to use it later only for no-data.
    setColorInterval(rgb2color(bgColor), rgb2color(lut_map.rbegin()->second));
    for (auto it = lut_map.begin(); it != lut_map.end(); it++) {
      double index = it->first;
      // Shift the starting color a little forward to not overwrite the bg. 
      // This is related to the logic in adjustMinVal(), and the number
      // here must be smaller than what is there to ensure the bg color
      // is not used for any valid values.
      if (index == 0)
        index += 0.5 * g_adjust_tol; 
      
      if (index == 0.0 || index == 1.0) 
        continue; // endpoints already added
      auto const& c = it->second; // c has 3 indices between 0 and 255
      addColorStop(index, rgb2color(it->second));
    }

    return;
  }
};

// Find spatial bounds. When the data is scattered, expand the bounds by 2% on each side
// to help with plotting later. Return a box with positive area even if there is no data.
void findSpatialBounds(imageData const& image, 
  // Outputs
  double & min_x, double & min_y, double & max_x, double & max_y) {
  
  bool poly_or_xyz = (image.m_isPoly || image.m_isCsv);
  if (poly_or_xyz) {

    if (image.scattered_data.empty()) {
      // If there is no data, better return some valid box
      min_x = 0.0;
      min_y = 0.0;
      max_x = 1.0;
      max_y = 1.0;
      return;
    }

    min_x = image.image_bbox.min().x();
    min_y = image.image_bbox.min().y();
    max_x = image.image_bbox.max().x();
    max_y = image.image_bbox.max().y();

    // Expand the bounds
    double small = 0.02;
    double dx = (max_x - min_x) * small;
    double dy = (max_y - min_y) * small;
    if (dx <= 0.0)
      dx = 1.0;
    if (dy <= 0.0)
      dy = 1.0;
    min_x -= dx; max_x += dx;
    min_y -= dy; max_y += dy;

  } else {

    vw::mosaic::DiskImagePyramid<double> const& img = image.img.m_img_ch1_double;
    min_x = 0;
    min_y = 0;
    max_x = std::max(img.cols() - 1, 1);
    max_y = std::max(img.rows() - 1, 1);
  }

  return;
}

// Make min_val smaller than any actual value, too to be able
// to use it for no-data and show it as black
double adjustMinVal(double min_val, double max_val) {
  double len = max_val - min_val;
  if (len <= 0.0)
    len = 1.0;
  
  return min_val - g_adjust_tol * len;
}

// Find min and max value based on a low-res image version
void calcLowResMinMax(imageData const& image, double nodata_val,
                      double& min_val, double& max_val) {
  
  // TODO(oalexan1): How about removing a small percentile of intensity from ends?

  // Get the lowest-resolution image version from the pyramid
  bool poly_or_xyz = (image.m_isPoly || image.m_isCsv);
  if (poly_or_xyz) // will not get here
    vw_throw(ArgumentErr() << "Expecting an image, not scattered points.\n");
  
  ImageView<double> lowres_img = image.img.m_img_ch1_double.pyramid().back();
  min_val = std::numeric_limits<double>::max();
  max_val = -min_val;
  for (int col = 0; col < lowres_img.cols(); col++) {
    for (int row = 0; row < lowres_img.rows(); row++) {
      
      double val = lowres_img(col, row);
      if (val == nodata_val) 
        continue;
      
      min_val = std::min(min_val, val);
      max_val = std::max(max_val, val);
    }
  }

  if (min_val > max_val) {
    // If the image turned out to be empty
    min_val = nodata_val;
    max_val = nodata_val;
  }
  
  Vector2 approx_bounds = image.img.m_img_ch1_double.approx_bounds();
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

public:

ColorAxesData(imageData & image, double min_x, double min_y, double max_x, double max_y):
  m_image(image), m_min_x(min_x), m_min_y(min_y), m_max_x(max_x), m_max_y(max_y) {

    // TODO(oalexan1): Need to handle georeferences.

    m_sub_scale = -1;  // This must be set properly before being used
    m_beg_x = 0;
    m_beg_y = 0;
 
    // TODO(oalexan1): Handle no-data properly by using transparent pixels.
    m_min_val = asp::stereo_settings().min;
    m_max_val = asp::stereo_settings().max;

    // Find the min and max values, and the no-data value
    bool poly_or_xyz = (m_image.m_isPoly || m_image.m_isCsv);
    if (poly_or_xyz) {
      m_nodata_val = -std::numeric_limits<double>::max();
      if (std::isnan(m_min_val) || std::isnan(m_max_val)) 
        findRobustBounds(m_image.scattered_data, m_min_val, m_max_val);
    } else {
      auto const& img = m_image.img.m_img_ch1_double;
      if (img.planes() != 1)
        vw::vw_throw(vw::ArgumentErr() 
          << "Only images with one channel can be colorized.\n"); // should not be reached

      m_nodata_val = img.get_nodata_val();
      if (std::isnan(m_min_val) || std::isnan(m_max_val)) // if the user did not set these
        calcLowResMinMax(m_image, m_nodata_val, m_min_val, m_max_val);
    }

    QwtMatrixRasterData::setInterval(Qt::XAxis, QwtInterval(m_min_x, m_max_x));
    QwtMatrixRasterData::setInterval(Qt::YAxis, QwtInterval(m_min_y, m_max_y));

    // Make the minimum value a little smaller than any actual value, too to be
    // able to use it for no-data and show it as black. The black color won't
    // really show even though it is the first color, because very close to it
    // we start using actual colors. See LutColormap().
    m_nodata_plot_val = adjustMinVal(m_min_val, m_max_val);
    QwtMatrixRasterData::setInterval(Qt::ZAxis, QwtInterval(m_nodata_plot_val, m_max_val));

    return;
  }

  // Given that we plan to render a portion of an image on disk within these
  // bounds, and resulting in a given image size, read from disk a clip with resolution
  // and extent just enough for the job, or a little higher res and bigger.
  void prepareClip(double x0, double y0, double x1, double y1, QSize const& imageSize) {

    // Ensure initialization
    m_level = 0;
    m_sub_scale = 1;
    m_beg_x = 0;
    m_beg_y = 0;

    bool poly_or_xyz = (m_image.m_isPoly || m_image.m_isCsv);
    if (poly_or_xyz) 
      return; // nothing to do

    // Have an image.
    
    // Note that y0 and y1 are normally flipped
    int beg_x = floor(std::min(x0, x1)), end_x = ceil(std::max(x0, x1));
    int beg_y = floor(std::min(y0, y1)), end_y = ceil(std::max(y0, y1));

    // if in doubt, go with lower sub_scale, so higher resolution.
    double sub_scale_x = (end_x - beg_x) / imageSize.width();
    double sub_scale_y = (end_y - beg_y) / imageSize.height();
    double sub_scale = sqrt(sub_scale_x * sub_scale_y);

    // Make the image a little blurrier. But this will run faster. Same logic
    // is used in MainWidget.cc
    sub_scale *= 1.3; 

    m_level =  m_image.img.m_img_ch1_double.pyramidLevel(sub_scale);
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
    
    // Instead of returning  m_image(x, y), we will return
    // m_sub_image(x/m_sub_scale - m_beg_x, y/m_sub_scale - m_beg_y).
    if (m_sub_scale <= 0) 
      vw::vw_throw(vw::ArgumentErr() << "Programmer error. Not ready yet to render the image.\n");
    
    bool poly_or_xyz = (m_image.m_isPoly || m_image.m_isCsv);
    if (poly_or_xyz)
      return m_nodata_plot_val; // TODO(oalexan1): Make transparent

    // Return pixels at the appropriate level of resolution
    x = round(x/m_sub_scale) - m_beg_x;
    y = round(y/m_sub_scale) - m_beg_y;

    if (x < 0 || y < 0 || x > m_sub_image.cols() - 1 || y > m_sub_image.rows() - 1)
      return m_nodata_plot_val;
    
    double val = m_sub_image(x, y);
    
    if (val == m_nodata_val || val != val) 
      return m_nodata_plot_val; // TODO(oalexan1): Make transparent, for now it is black

    // Ensure this is no smaller than m_min_val, as otherwise it will be shown
    // as black background.
    return std::max(m_min_val, val);
  }

public:
  imageData & m_image;
  double m_min_x, m_min_y, m_max_x, m_max_y; // spatial extent
  double m_min_val, m_max_val, m_nodata_val, m_nodata_plot_val; // values 

private:
  
  // Can get away by using a lower-res image version at this sub scale.
  int m_level, m_beg_x, m_beg_y;
  double m_sub_scale;
  ImageView<double> m_sub_image;
};

// Draw irregular xyz data to be plotted at (x, y) location with z giving
// the intensity. May be colorized.
// TODO(oalexan1): Integrate with function in main widget.
void drawScatteredData(imageData         const & image, 
                       QwtScaleMap       const & xMap,
                       QwtScaleMap       const & yMap,
                       QRectF            const & canvasRect, 
                       QwtInterval       const & I, 
                       QwtLinearColorMap const * cmap,
                       double min_val,  double max_val, 
                       double nodata_val, 
                       double nodata_plot_val,
                       QPainter          * painter) {
  
  int r = asp::stereo_settings().plot_point_radius;

  double den = max_val - nodata_plot_val;
  if (den <= 0.0)
    den = 1.0;

  for (size_t pt_it = 0; pt_it < image.scattered_data.size(); pt_it++) {
    auto const& P = image.scattered_data[pt_it];

    // Find the location on the screen. Adjust for the canvas offset.
    QPointF q;
    q.setX(xMap.transform(P.x()) + canvasRect.left());
    q.setY(yMap.transform(P.y()) + canvasRect.top());
  
    // Find the scaled intensity
    double val = P.z();
    if (val != val || val == nodata_val) 
      val = nodata_plot_val;
    else if (val < min_val)
      val = min_val;
    else if (val > max_val)
      val = max_val;

    // Note how we scale using nodata_plot_val, not min_val. 
    // That because we want to use the black color for no-data,
    // corresponding to s = 0, so any valid value will have s > 0,
    // because nodata_plot_val < min_val.
    double s = (val - nodata_plot_val) / den;

    // Find and draw the color 
    QColor c = cmap->rgb(I, s);
    painter->setPen(c);
    painter->setBrush(c);
    painter->drawEllipse(q, r, r); 
  }

  return;
}
  
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
    // canvasRect is the region, in screen pixel units, where the image will go.
    // If the image is narrow, it may not fill fully the canvas. If the labels
    // on the axes take up more space when zooming, the image region will be
    // affected. So, canvasRect is not fixed, even if the total
    // region allocated to the canvas and axes is fixed.
    // xMap and yMap are used to convert from world units to screen.

    //vw::Stopwatch sw2;
    //sw2.start();
    bool poly_or_xyz = (m_data->m_image.m_isPoly || m_data->m_image.m_isCsv);   
    if (!poly_or_xyz) {
      // Draw the image
     QwtPlotSpectrogram::draw(painter, xMap, yMap, canvasRect);
    } else {
      // Just draw the background
      painter->fillRect(canvasRect, rgb2color(bgColor));
    }
    //sw2.stop();
    //std::cout << "drawing took " << sw2.elapsed_seconds() << " s\n";

    QwtLinearColorMap const * cmap = (QwtLinearColorMap*)this->colorMap();
    QwtInterval I(0.0, 1.0); // QwtLinearColorMap does not remember its interval
    drawScatteredData(m_data->m_image, xMap, yMap, canvasRect, I, cmap, 
        m_data->m_min_val, m_data->m_max_val,  
        m_data->m_nodata_val, m_data->m_nodata_plot_val,
        painter);

    return;
  }

  // This will be called by draw().
  // The rectangle in 'area' is in world units, not in pixel units. imageSize
  // has the dimensions, in pixels, of the canvas portion having the image.
  virtual QImage renderImage(const QwtScaleMap & xMap,
                            const QwtScaleMap  & yMap,
                            const QRectF       & area,
                            const QSize        & imageSize) const {
  
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

ColorAxes::ColorAxes(QWidget *parent, 
  int beg_image_id, int end_image_id, int base_image_id, bool use_georef,
  std::vector<imageData> & images):
    QwtPlot(parent), WidgetBase(beg_image_id, end_image_id, base_image_id, use_georef, images) {

  int num_images = m_images.size();

  // TODO(oalexan1): Integrate this with MainWidget.cc
  // Set data per image. 
  for (int i = 0; i < num_images; i++) {

    bool in_range = (m_beg_image_id <= i && i < m_end_image_id);
    if (!in_range) 
      continue;

    m_images[i].load();
    
    if (m_use_georef && !m_images[i].has_georef) {
      popUp("No georeference present in: " + m_images[i].name + ".");
      vw_throw(ArgumentErr() << "Missing georeference.\n");
    }

    // Make sure we set these up before the image2world call below!
    if (m_use_georef) {
      m_world2image_geotransforms[i]
        = vw::cartography::GeoTransform(m_images[m_base_image_id].georef, 
                                        m_images[i].georef);
      m_image2world_geotransforms[i]
        = vw::cartography::GeoTransform(m_images[i].georef, 
                                        m_images[m_base_image_id].georef);
    }
  }

  findSpatialBounds(m_images[m_beg_image_id], m_min_x, m_min_y, m_max_x, m_max_y);
  ColorAxesData * data = new ColorAxesData(m_images[m_beg_image_id], m_min_x, m_min_y, 
                                           m_max_x, m_max_y);

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
    vw::cm::parse_color_style(m_images[m_beg_image_id].colormap, lut_map);
  } catch (...) {
    popUp("Unknown colormap style: " +  m_images[m_beg_image_id].colormap);
    m_images[m_beg_image_id].colormap = "binary-red-blue"; // fallback to this
    vw::cm::parse_color_style(m_images[m_beg_image_id].colormap, lut_map);
  }
  m_plotter->setColorMap(new LutColormap(lut_map));
  
  // Set the axes and the color bar. The axes will be adjusted later
  // when we know the aspect ratio.
  QwtScaleWidget *rightAxis = axisWidget(QwtPlot::yRight);
  QwtInterval zInterval = m_plotter->data()->interval(Qt::ZAxis);
  rightAxis->setColorBarEnabled(true);
  rightAxis->setColorMap(zInterval, new LutColormap(lut_map));
  rightAxis->setColorBarWidth(30);
  m_plotter->setCachePolicy(QwtPlotRasterItem::PaintCache);
  m_plotter->attach(this);
  setAxisScale(QwtPlot::xBottom, m_min_x, m_max_x);
  setAxisScale(QwtPlot::yRight, zInterval.minValue(), zInterval.maxValue());
  bool poly_or_xyz = (m_images[m_beg_image_id].m_isPoly || m_images[m_beg_image_id].m_isCsv);
  if (!poly_or_xyz)
    setAxisScale(QwtPlot::yLeft, m_max_y, m_min_y); // y axis goes down
  else
    setAxisScale(QwtPlot::yLeft, m_min_y, m_max_y); // y axis goes up
  enableAxis(QwtPlot::yRight);
  setAxisAutoScale(QwtPlot::yRight, false);
  setAxisAutoScale(QwtPlot::xBottom, false);
  setAxisAutoScale(QwtPlot::yLeft, false);
  plotLayout()->setAlignCanvasToScales(true);

  // Use image mode, not contour mode
  m_plotter->setDisplayMode(QwtPlotSpectrogram::ImageMode, true);

  // An attempt to find the true canvas dimensions.
  // https://qwt-interest.narkive.com/cquDtLQ5/qwtplot-s-initial-canvas-size
  // This is not enough. Only in renderImage() we will truly know the image size.
  plotLayout()->setCanvasMargin(0);
  plotLayout()->setAlignCanvasToScales(true);
  
  replot();
}

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
  
  // Note: This is bigger by a few pixels than the area having the image later,
  // but it is good enough given that its size is on the order of 1000 pixels.
  // The true image area is known only in renderImage(), so much later. The
  // scales should be set up before that.
  double aspect_ratio = double(rect.width()) / double(rect.height());
  QRectF in_box(m_min_x, m_min_y, m_max_x - m_min_x, m_max_y - m_min_y);
  QRectF box = expand_box_to_aspect_ratio(in_box, aspect_ratio);

  // Adjust the scales accordingly
  bool poly_or_xyz = (m_images[m_beg_image_id].m_isPoly || m_images[m_beg_image_id].m_isCsv);
  setAxisScale(QwtPlot::xBottom, box.left(), box.right());
  if (!poly_or_xyz)
    setAxisScale(QwtPlot::yLeft, box.bottom(), box.top()); // y axis goes down
  else
    setAxisScale(QwtPlot::yLeft, box.top(), box.bottom()); // y axis goes up

  // Set up the zooming and panning after we know the aspect ratio.
  // This makes zooming out work correctly.
  
  // LeftButton for the zooming
  // MidButton for the panning
  // RightButton: zoom out by 1
  // Ctrl+RightButton: zoom out to full size
  
  m_zoomer = new ColorAxesZoomer(canvas(), aspect_ratio);
  m_zoomer->setMousePattern(QwtEventPattern::MouseSelect2,
                          Qt::RightButton, Qt::ControlModifier);
  m_zoomer->setMousePattern(QwtEventPattern::MouseSelect3,
                          Qt::RightButton);
  
  const QColor c(Qt::darkBlue);
  m_zoomer->setRubberBandPen(QPen(c));
  m_zoomer->setTrackerPen(QPen(c));

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
  
// Zoom to full extent
void ColorAxes::sizeToFit() {
  m_zoomer->zoom(m_zoomer->zoomBase());
}

}} // end namespace vw::gui

