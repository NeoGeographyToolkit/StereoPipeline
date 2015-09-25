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


/// \file MainWidget.h
///
/// A widget showing an image.
///
#ifndef __STEREO_GUI_MAIN_WIDGET_H__
#define __STEREO_GUI_MAIN_WIDGET_H__

#include <string>
#include <vector>
#include <list>
#include <set>

#include <boost/filesystem/path.hpp>
#include <boost/filesystem.hpp>
#include <boost/mpl/or.hpp>

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
#include <vw/Image/AntiAliasing.h>
#include <vw/FileIO/DiskImageResource.h>
#include <vw/FileIO/DiskImageView.h>
#include <vw/FileIO/DiskImageUtils.h>
#include <vw/Math/BBox.h>
#include <vw/Math/Vector.h>
#include <vw/Cartography/GeoReference.h>
#include <vw/InterestPoint/InterestData.h>

// ASP
#include <asp/Core/Common.h>
#include <asp/GUI/GuiUtilities.h>

class QMouseEvent;
class QWheelEvent;
class QPoint;
class QResizeEvent;
class QTableWidget;
class QContextMenuEvent;
class QMenu;


namespace vw { namespace gui {

  namespace fs = boost::filesystem;

   /// This class handles user interaction with the a single image pane.
  class MainWidget : public QWidget {
    Q_OBJECT

  public:

    // Constructors/Destructor
    MainWidget(QWidget *parent,
               asp::BaseOptions const& opt,
               int image_id,
               std::string& output_prefix,
               std::vector<std::string> const& image_files,
               std::vector<std::vector<ip::InterestPoint> > & matches,
               chooseFilesDlg * chooseFiles, bool use_georef,
               bool hillshade, bool view_matches);
    virtual ~MainWidget();

    bool get_crop_win(QRect & win);

    // Set a default size for this widget.  This is usually overridden
    // by parent views.
    virtual QSize sizeHint () const { return QSize(500,500); }

    // Image Manipulation Methods
    void zoom(double scale);
    void viewMatches(bool hide);

    void setShadowThreshMode(bool turnOn) { m_shadow_thresh_calc_mode = turnOn;}

    signals:
    void refreshAllMatches();

              public slots:
              void sizeToFit();
    void showFilesChosenByUser(int rowClicked, int columnClicked);
    void viewUnthreshImages();
    void viewThreshImages();
    void viewHillshadedImages(bool hillshade_mode);

    void addMatchPoint();    ///< Add a new interest point (from right click menu)
    void deleteMatchPoint(); ///< Delete an interest point (from right click menu)

  protected:

    // Setup
    bool eventFilter (QObject *obj, QEvent *E);
    void resizeEvent(QResizeEvent*);

    // Event handlers
    void paintEvent           (QPaintEvent *event);
    void mousePressEvent      (QMouseEvent *event);
    void mouseReleaseEvent    (QMouseEvent *event);
    void mouseMoveEvent       (QMouseEvent *event);
    void mouseDoubleClickEvent(QMouseEvent *event);
    void wheelEvent           (QWheelEvent *event);
    void enterEvent           (QEvent *event);
    void leaveEvent           (QEvent *event);
    void keyPressEvent        (QKeyEvent *event);
    void contextMenuEvent     (QContextMenuEvent *event);

  private:

    asp::BaseOptions m_opt;

    /// Handle to parent GUI panel used to select which of the multiple "owned"
    ///  images should be currently displayed.
    /// - Null if there is only one image.
    chooseFilesDlg  *     m_chooseFilesDlg;
    std::set<std::string> m_filesToHide; ///< Files that are currently not being displayed.
    std::vector<int> m_filesOrder;       ///< The order the images are drawn in.

    const int m_image_id; ///< An ID number assigned to this widget when it is created
    std::string & m_output_prefix; // alias
    std::vector<std::string> m_image_files;

    /// A set of matching interest points for each image.
    /// - Note that this is an alias wrapping an object passed in through the constructor.
    std::vector<std::vector<vw::ip::InterestPoint> > & m_matches;

    bool m_use_georef;

    bool  m_firstPaintEvent;
    QRect m_emptyRubberBand;
    QRect m_rubberBand;
    BBox2 m_stereoCropWin;

    // if we are selecting a crop win to do stereo in
    bool m_cropWinMode;

    // Use double buffering: draw to a pixmap first, refresh it only
    // if really necessary, and display it when paintEvent is called.
    QPixmap m_pixmap;

    bool m_bilinear_filter;
    bool m_use_colormap;

    std::vector<imageData> m_images;
    BBox2 m_images_box;

    // Adjustment mode
    enum AdjustmentMode { NoAdjustment,
                          TransformAdjustment, GainAdjustment,
                          OffsetAdjustment, GammaAdjustment };
    AdjustmentMode m_adjust_mode;

    // Mouse position
    vw::Vector2 m_curr_pixel_pos, m_curr_world_pos;

    // Dimensions and stats
    int m_window_width;  // the width  of the plotting window in screen pixels
    int m_window_height; // the height of the plotting window in screen pixels

    // Image Parameters
    vw::BBox2 m_current_view, m_last_view;
    double m_gain,   m_last_gain;
    double m_offset, m_last_offset;
    double m_gamma,  m_last_gamma;

    enum DisplayChannel { DisplayRGBA = 0, DisplayR, DisplayG, DisplayB, DisplayA };
    int m_display_channel;
    int m_colorize_display;

    // Mouse press  position
    int m_mousePrsX,  m_mousePrsY;

    // Right-click context menu
    QMenu  * m_ContextMenu;
    QAction* m_addMatchPoint;
    QAction* m_deleteMatchPoint;

    double m_shadow_thresh;
    bool   m_shadow_thresh_calc_mode;
    bool   m_shadow_thresh_view_mode;
    std::vector<imageData> m_shadow_thresh_images;

    bool m_hillshade_mode;
    std::vector<imageData> m_hillshaded_images;

    bool m_viewMatches; ///< Control if IP's are drawn

    // Drawing is driven by QPaintEvent, which calls out to drawImage()
    void drawImage(QPainter* paint);
    /// Add all the interest points to the provided canvas
    /// - Called internally by drawImage
    void drawInterestPoints(QPainter* paint, std::list<BBox2i> const& valid_regions);

    vw::Vector2 world2screen(vw::Vector2 const& p);
    vw::Vector2 screen2world(vw::Vector2 const& pix);
    BBox2       world2screen(BBox2 const& R);
    BBox2       screen2world(BBox2 const& R);
    Vector2     world2image(Vector2 const& P, int imageIndex);
    BBox2       world2image(BBox2 const& R, int imageIndex);
    BBox2       image2world(BBox2 const& R, int imageIndex);
    vw::BBox2   expand_box_to_keep_aspect_ratio(vw::BBox2 const& box);

    void updateCurrentMousePosition();
    void updateRubberBand(QRect & R);
    void refreshPixmap();
    void genHillshadedImages();
  };


}} // namespace vw::gui

#endif  // __STEREO_GUI_MAIN_WIDGET_H__
