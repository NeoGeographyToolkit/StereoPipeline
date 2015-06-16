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


/// \file stereo_gui_MainWidget.h
///
/// The Vision Workbench image viewer.
///
#ifndef __STEREO_GUI__MAIN_WIDGET_H__
#define __STEREO_GUI_MAIN_WIDGET_H__

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

// STL
#include <string>
#include <vector>
#include <list>
#include <set>

class QMouseEvent;
class QWheelEvent;
class QPoint;
class QResizeEvent;
class QTableWidget;

namespace vw {
namespace gui {

  // A class to keep all data associated with an image file
  struct imageData{
    std::string name;
    bool has_georef;
    vw::cartography::GeoReference georef;
    BBox2 bbox;
    ImageView<float> img;
    double nodata_val;
    double min_val, max_val;
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
    vw::Vector2 world2pixel(vw::Vector2 const& p);
    vw::Vector2 pixel2world(vw::Vector2 const& pix);
    QRect pixel2world(QRect const& R);
    QRect world2pixel(QRect const& R);
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
