// __BEGIN_LICENSE__
//  Copyright (c) 2006-2013, United States Government as represented by the
//  Administrator of the National Aeronautics and Space Administration. All
//  rights reserved.
//
//  The NASA Vision Workbench is licensed under the Apache License,
//  Version 2.0 (the "License"); you may not use this file except in
//  compliance with the License. You may obtain a copy of the License at
//  http://www.apache.org/licenses/LICENSE-2.0
//
//  Unless required by applicable law or agreed to in writing, software
//  distributed under the License is distributed on an "AS IS" BASIS,
//  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
//  See the License for the specific language governing permissions and
//  limitations under the License.
// __END_LICENSE__


/// \file MainWidget.h
///
/// A widget showing an image.
///
#ifndef __STEREO_GUI__MAIN_WIDGET_H__
#define __STEREO_GUI_MAIN_WIDGET_H__

#include <string>
#include <vector>
#include <list>
#include <set>

#include <boost/filesystem/path.hpp>
#include <boost/filesystem.hpp>

// Qt
#include <QWidget>
#include <QPoint>

// Vision Workbench
#include <vw/Core/Thread.h>
#include <vw/Core/Log.h>
#include <vw/Image/ImageResource.h>
#include <vw/Image/ImageViewRef.h>
#include <vw/Image/ImageView.h>
#include <vw/Image/Manipulation.h>
#include <vw/Image/Statistics.h>
#include <vw/FileIO/DiskImageResource.h>
#include <vw/FileIO/DiskImageView.h>
#include <vw/Math/BBox.h>
#include <vw/Math/Vector.h>
#include <vw/Cartography/GeoReference.h>

// ASP
#include <asp/Core/Common.h>
#include <asp/Core/AntiAliasing.h>

class QMouseEvent;
class QWheelEvent;
class QPoint;
class QResizeEvent;
class QTableWidget;

namespace vw { namespace gui {


// A class to manage very large images and their subsampled versions
// in a pyramid. The most recently accessed tiles are cached in memory.
template <class PixelT>
class DiskImagePyramid : public ImageViewBase<DiskImagePyramid<PixelT> > {

public:
  typedef typename DiskImageView<PixelT>::pixel_type pixel_type;
  typedef typename DiskImageView<PixelT>::result_type result_type;
  typedef typename DiskImageView<PixelT>::pixel_accessor pixel_accessor;

  // Constructor
  DiskImagePyramid(std::string const& img_file = "",
                   int top_image_max_pix = 1000*1000,
                   int subsample = 2
                   ): m_subsample(subsample),
                      m_top_image_max_pix(top_image_max_pix){

    if (img_file == "")
      return;

    m_pyramid.push_back(vw::DiskImageView<PixelT>(img_file));
    m_pyramid_files.push_back(img_file);
    m_scales.push_back(1);

    if (subsample < 2) {
      vw_throw( ArgumentErr() << "Must subsample by a factor of at least 2.\n");
    }

    if (top_image_max_pix < 4) {
      vw_throw( ArgumentErr() << "The image at the top of the pyramid must "
                << "be at least 2x2 in size.\n");
    }

    int level = 0;
    int scale = 1;
    while (double(m_pyramid[level].cols())*double(m_pyramid[level].rows()) >
           m_top_image_max_pix ){

      // The name of the file at the current scale
      boost::filesystem::path p(img_file);
      std::string stem = p.stem().string();
      std::ostringstream os;
      scale *= subsample;
      os << stem << "_sub" << scale << ".tif";
      std::string curr_file = os.str();

      if (level == 0) {
        vw_out() << "Detected large image: " << img_file  << "." << std::endl;
        vw_out() << "Will construct an image pyramid on disk."  << std::endl;
      }

      // Get the nodata value
      double nodata_val;
      if (!asp::read_nodata_val(img_file, nodata_val)){
        nodata_val = -FLT_MAX;
      }

      // Resample the image at the current pyramid level.
      // TODO: resample_aa is a hacky thingy. Need to understand
      // what is a good way of resampling.
      ImageViewRef< PixelMask<PixelT> > masked
        = create_mask(m_pyramid[level], nodata_val);
      double sub_scale = 1.0/subsample;
      int tile_size = 256;
      int sub_threads = 1;
      ImageViewRef< PixelMask<PixelT> > resampled
        = block_rasterize
        (asp::cache_tile_aware_render(asp::resample_aa(masked, sub_scale),
                                      Vector2i(tile_size,tile_size) * sub_scale),
         tile_size, sub_threads);
      ImageViewRef<PixelT> unmasked = apply_mask(resampled, nodata_val);

      // If the file exists, and has the right size, don't write it again
      bool will_write = true;
      if (boost::filesystem::exists(curr_file)) {
        DiskImageView<float> tmp(curr_file);
        if (tmp.cols() == unmasked.cols() && tmp.rows() == unmasked.rows()) {
          will_write = false;
        }
      }

      if (will_write) {
        TerminalProgressCallback tpc("asp", ": ");
        asp::BaseOptions opt;
        vw_out() << "Writing: " << curr_file << std::endl;
        asp::block_write_gdal_image(curr_file, unmasked, nodata_val, opt, tpc);
      }else{
        vw_out() << "Using existing subsampled image: " << curr_file << std::endl;
      }

      // Note that m_pyramid contains a handle to DiskImageView.
      // DiskImageView's implementation will make it possible
      // cache in memory the most recently used tiles of all
      // the images in the pyramid.
      m_pyramid_files.push_back(curr_file);
      m_pyramid.push_back(vw::DiskImageView<PixelT>(curr_file));
      m_scales.push_back(scale);

      level++;
    }
  }

  // Given a region (at full resolution) and a scale factor,
  // return the portion of the image in the region, subsampled
  // by a factor no more than the input scale factor.
  // Also return the precise subsample factor used and the
  // region at that scale level.
  void getImageClip(double scale_in, vw::BBox2i region_in,
                    ImageView<PixelT> & clip_out, double & scale_out,
                    vw::BBox2i & region_out) {

    if (m_pyramid.empty())
      vw_throw( ArgumentErr() << "Uninitialized image pyramid.\n");

    // Find the right pyramid level to use
    int level = 0;
    while (1) {
      if (level+1 >= (int)m_scales.size()) break; // last level
      if (m_scales[level+1] > scale_in)  break; // too coarse
      level++;
    }

    vw_out() << "Reading: " << m_pyramid_files[level] << std::endl;

    region_in.crop(bounding_box(m_pyramid[0]));
    scale_out = m_scales[level];
    region_out = region_in/scale_out;
    region_out.crop(bounding_box(m_pyramid[level]));
    clip_out = crop(m_pyramid[level], region_out);
  }

  ~DiskImagePyramid() {}

  int32 cols() const { return m_pyramid[0].cols(); }
  int32 rows() const { return m_pyramid[0].rows(); }
  int32 planes() const { return m_pyramid[0].planes(); }

  pixel_accessor origin() const { return m_pyramid[0].origin(); }
  result_type operator()( int32 x, int32 y, int32 p = 0 ) const { return m_pyramid[0](x,y,p); }

  typedef typename DiskImageView<PixelT>::prerasterize_type prerasterize_type;
  prerasterize_type prerasterize( BBox2i const& bbox ) const { return m_pyramid[0].prerasterize( bbox ); }
  template <class DestT> void rasterize( DestT const& dest, BBox2i const& bbox ) const { m_pyramid[0].rasterize( dest, bbox ); }

  ImageViewRef<PixelT> bottom() { return m_pyramid[0]; }

private:
  // The subsample factor to go to the next level of the pyramid
  // (must be >= 2).
  int m_subsample;

  // The maxiumum number of pixels in the coarsest level of
  // the pyramid (keep on downsampling until getting to this number
  // or under it).
  int m_top_image_max_pix;

  //  The pyramid. Largest images come earlier.
  std::vector< vw::ImageViewRef<PixelT> > m_pyramid;

  // The files (stored on disk) containing the images in the pyramid.
  std::vector<std::string> m_pyramid_files;

  std::vector<int> m_scales;
};

  // A class to keep all data associated with an image file
  struct imageData{
    std::string name;
    bool has_georef;
    vw::cartography::GeoReference georef;
    BBox2 bbox;
    DiskImagePyramid<float> img;
    double nodata_val;
    void read(std::string const& image, bool ignore_georef,
              bool hillshade);
  };

  vw::Vector2 QPoint2Vec(QPoint const& qpt);
  QPoint Vec2QPoint(vw::Vector2 const& V);

  // A simple class for keeping track of crosshair locations and colors.
  class PointList {
    std::list<vw::Vector2> m_points;
    vw::Vector3 m_color;
  public:
    PointList(vw::Vector3 const& color) : m_color(color) {}
    PointList(std::list<vw::Vector2> const& points, vw::Vector3 const& color) :
      m_color(color) {
      this->push_back(points);
    }

    std::list<vw::Vector2> const& points() const { return m_points; }
    vw::Vector3 color() const { return m_color; }

    void push_back(vw::Vector2 pt) { m_points.push_back(pt); }
    void push_back(std::list<vw::Vector2> pts) {
      std::list<vw::Vector2>::iterator iter  = pts.begin();
      while (iter != pts.end()) {
        m_points.push_back(*iter);
        ++iter;
      }
    }
  };

  /// Class to create a file list on the left side of the window
  class chooseFilesDlg: public QWidget{
    Q_OBJECT

  public:
    chooseFilesDlg(QWidget * parent);
    ~chooseFilesDlg();
    void chooseFiles(const std::vector<imageData> & images);

    QTableWidget * getFilesTable(){ return m_filesTable; }
    static QString selectFilesTag(){ return ""; }
  private:
    QTableWidget * m_filesTable;
                                                      private slots:
  };

  class MainWidget : public QWidget {
    Q_OBJECT

  public:

    // Constructors/Destructor
    MainWidget(QWidget *parent, std::vector<std::string> const& images,
               chooseFilesDlg * chooseFiles, bool ignore_georef,
               bool hillshade);
    virtual ~MainWidget();

    QRect get_crop_win();

    // Set a default size for this widget.  This is usually overridden
    // by parent views.
    virtual QSize sizeHint () const { return QSize(500,500); }

    // Image Manipulation Methods
    void zoom(double scale);
  public slots:
    void size_to_fit();

    void showFilesChosenByUser();

    void set_nodata_value(double nodata_value) {
      m_nodata_value = nodata_value;
      m_use_nodata = 1;
    }

  protected:

    // Setup
    bool eventFilter (QObject *obj, QEvent *E);
    void resizeEvent(QResizeEvent*);

    // Event handlers
    void paintEvent(QPaintEvent *event);
    void mousePressEvent(QMouseEvent *event);
    void mouseReleaseEvent(QMouseEvent *event);
    void mouseMoveEvent(QMouseEvent *event);
    void mouseDoubleClickEvent(QMouseEvent *event);
    void wheelEvent(QWheelEvent *event);
    void enterEvent(QEvent *event);
    void leaveEvent(QEvent *event);
    void keyPressEvent(QKeyEvent *event);

  private:
    // Drawing is driven by QPaintEvents, which call out to drawImage()
    void drawImage(QPainter* paint);
    vw::Vector2 world2screen(vw::Vector2 const& p);
    vw::Vector2 screen2world(vw::Vector2 const& pix);
    QRect world2screen(QRect const& R);
    QRect screen2world(QRect const& R);
    vw::BBox2 expand_box_to_keep_aspect_ratio(BBox2 const& box);
    void updateCurrentMousePosition();

    bool  m_firstPaintEvent;
    QRect m_emptyRubberBand;
    QRect m_rubberBand, m_stereoCropWin;

    // Use double buffering: draw to a pixmap first, refresh it only
    // if really necessary, and display it when paintEvent is called.
    QPixmap m_pixmap;

    bool m_bilinear_filter;
    bool m_use_colormap;

    std::vector<imageData> m_images;
    BBox2 m_images_box;

    PixelRGBA<float> m_last_pixel_sample;

    // Adjustment mode
    enum AdjustmentMode { NoAdjustment,
                          TransformAdjustment, GainAdjustment,
                          OffsetAdjustment, GammaAdjustment };
    AdjustmentMode m_adjust_mode;

    // Mouse position
    vw::Vector2 m_curr_pixel_pos, m_curr_world_pos;
    QPoint m_last_viewport_min;

    // Dimensions and stats
    int m_window_width;  // the width  of the plotting window in screen pixels
    int m_window_height; // the height of the plotting window in screen pixels
    vw::float32 m_image_min;
    vw::float32 m_image_max;
    vw::float32 m_nodata_value;
    int m_use_nodata;

    // Image Parameters
    vw::BBox2 m_current_view;
    double m_gain, m_last_gain;
    double m_offset, m_last_offset;
    double m_gamma, m_last_gamma;

    enum DisplayChannel { DisplayRGBA = 0, DisplayR, DisplayG, DisplayB, DisplayA };
    int m_display_channel;
    int m_colorize_display;
    int m_hillshade_display;

    // Choose which files to hide/show in the GUI
    chooseFilesDlg  *     m_chooseFilesDlg;
    std::set<std::string> m_filesToHide;

    int m_mousePrsX,  m_mousePrsY, m_mouseRelX,  m_mouseRelY;

    void updateRubberBand(QRect & R);
    void refreshPixmap();
  };

}} // namespace vw::gui

#endif  // __STEREO_GUI_MAIN_WIDGET_H__
