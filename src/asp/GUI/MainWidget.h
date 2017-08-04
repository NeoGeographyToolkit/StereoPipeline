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

// Qwt
#include <qwt_plot.h>
#include <qwt_plot_curve.h>
#include <qwt_point_data.h>
#include <qwt_series_data.h>

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
#include <vw/Cartography/GeoTransform.h>
#include <vw/InterestPoint/InterestData.h>
#include <vw/Geometry/dPoly.h>

// ASP
#include <asp/Core/Common.h>
#include <asp/GUI/GuiUtilities.h>

class QMouseEvent;
class QWheelEvent;
class QPoint;
class QResizeEvent;
class QTableWidget;
class QTableWidgetItem;
class QContextMenuEvent;
class QMenu;
class QStylePainter;

namespace vw { namespace gui {

  namespace fs = boost::filesystem;

   /// This class handles user interaction with the a single image pane.
  class MainWidget : public QWidget {
    Q_OBJECT

  public:

    // Constructors/Destructor
    MainWidget(QWidget *parent,
               vw::cartography::GdalWriteOptions const& opt,
               int image_id,
               std::string& output_prefix,
               std::vector<std::string> const& image_files,
               std::string const& base_image_file,
               std::vector<std::vector<ip::InterestPoint> > & matches,
               chooseFilesDlg * chooseFiles, bool use_georef,
               std::vector<bool> const& hillshade, bool view_matches, bool zoom_all_to_same_region,
	       bool & allowMultipleSelections // alias
	       );
    virtual ~MainWidget();

    bool get_crop_win(QRect & win);

    // Set a default size for this widget.  This is usually overridden
    // by parent views.
    virtual QSize sizeHint () const { return QSize(500,500); }

    // Image Manipulation Methods
    void zoom(double scale);
    void viewMatches(bool hide);

    void setShadowThreshMode(bool turnOn) { m_shadow_thresh_calc_mode = turnOn;}
    void plotProfile(std::vector<imageData> const& images,
		     std::vector<double> const& profileX, 
		     std::vector<double> const& profileY);
    
    void drawOneVertex(int x0, int y0, QColor color, int lineWidth,
                       int drawVertIndex, QPainter &paint);
    
    void plotDPoly(bool plotPoints, bool plotEdges,
                   bool plotFilled, bool showIndices, int lineWidth,
                   int drawVertIndex, // 0 is a good choice here
                   QColor const& color,
                   QPainter &paint,
                   vw::geometry::dPoly currPoly // Make a local copy on purpose
                   );
    
    //void plotProfilePolyLine(QStylePainter & paint,
    void plotProfilePolyLine(QPainter & paint,
                             std::vector<double> const& profileX, 
                             std::vector<double> const& profileY);

    std::set<int> & indicesWithAction() { return m_indicesWithAction; }

    void setThreshold(double thresh); ///< Set the shadow threshold 
    double getThreshold();            ///< Get the shadow threshold

    void setZoomAllToSameRegion(bool zoom_all_to_same_region);
    vw::BBox2 current_view();
    void zoomToRegion (vw::BBox2 const& region);
    bool hillshadeMode() const;
    std::vector<bool> hillshadeModeVec() const {return m_hillshade_mode;}
    void setHillshadeMode(bool hillshade_mode);
    BBox2 firstImagePixelBox() const;
    BBox2 firstImageWorldBox(vw::BBox2 const& image_box) const;
    void setWorldBox(vw::BBox2 const& box);

    signals:
    void turnOnViewMatchesSignal();
    void turnOffViewMatchesSignal();
    void removeImageAndRefreshSignal();
    void uncheckProfileModeCheckbox();
    void uncheckPolyEditModeCheckbox();
    void zoomAllToSameRegionSignal(int);
    
public slots:
    void sizeToFit();
    void showFilesChosenByUser(int rowClicked, int columnClicked);
    void zoomToImageInTableCell(int rowClicked, int columnClicked);
    void toggleAllOnOff();
    void customMenuRequested(QPoint pos);
    void viewUnthreshImages();
    void viewThreshImages();
    void viewHillshadedImages(bool hillshade_mode);

    void addMatchPoint();           ///< Add a new interest point (from right click menu)
    void deleteMatchPoint();        ///< Delete an interest point (from right click menu)
    void setThreshold();            ///< Set change shadow threshold (from right click menu)
    void setHillshadeParams();      ///< Set the azimuth and elevation for hillshaded images.
    void toggleHillshade();         ///< Turn on/off hillshading per image (from right click menu)
    void refreshHillshade();        ///< We modified m_hillshade_mode. Update the display.
    void bringImageOnTopSlot();     ///< Show this image on top of other images.
    void pushImageToBottomSlot();   ///< Show all other images on top of this
    void zoomToImage();             ///< Zoom to have this image in full view.
    void deleteImage();             ///< Delete an image from the gui and refresh
    void allowMultipleSelections(); ///< Allow the user to select multiple regions
    void deleteSelection();         ///< Delete an area selected with the mouse at current point
    void hideImagesNotInRegion();   ///< Hide images not intersecting a given region 
    void saveVectorLayer();         ///< Delete current vector layer
    void setProfileMode(bool profile_mode); ///< Turn on and off the 1D profile tool
    void setPolyEditMode(bool polyEditMode); ///< Turn on and off the vector layer drawing
    void deleteVertex();            ///< Delete a vertex from a vector layer
    void deleteVertices();          ///< Delete poly vertices in selected region
    void insertVertex();            ///< Insert an intermediate vertex at right-click
    void mergePolys();              ///< Merge existing polygons
    void saveScreenshot();          ///< Save a screenshot of the current imagery

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

    class ProfilePlotter : public QwtPlot {
    public:
      ProfilePlotter(MainWidget * parent): QwtPlot(NULL),
					   m_parent(parent) {}
      ~ProfilePlotter() {}
      
    private:
      void closeEvent(QCloseEvent *){
        // Signal to the parent that the window got closed.
	// Turn off profiling.
        bool profile_mode = false;
        m_parent->setProfileMode(profile_mode);
      }
      
      MainWidget * m_parent;    
    };
    
    vw::cartography::GdalWriteOptions m_opt;

    /// Handle to parent GUI panel used to select which of the multiple "owned"
    ///  images should be currently displayed.
    /// - Null if there is only one image.
    chooseFilesDlg  *     m_chooseFilesDlg;
    std::set<std::string> m_filesToHide; ///< Files that are currently not being displayed.
    std::vector<int> m_filesOrder;       ///< The order the images are drawn in.

    const int m_image_id; ///< An ID number assigned to this widget when it is created
    std::string & m_output_prefix; // alias
    std::vector<std::string> m_image_files;
    std::vector<bool> m_hillshade_mode;
    double m_hillshade_azimuth, m_hillshade_elevation;
    
    /// A set of matching interest points for each image.
    /// - Note that this is an alias wrapping an object passed in through the constructor.
    std::vector<std::vector<vw::ip::InterestPoint> > & m_matches;

    bool m_use_georef;

    bool  m_firstPaintEvent;
    QRect m_emptyRubberBand;
    QRect m_rubberBand;
    BBox2 m_stereoCropWin;

    std::vector<BBox2> m_selectionRectangles;
    
    // If we are selecting a crop win to do stereo in
    bool m_cropWinMode;
    
    // If we are in the midst of drawing a profile
    bool m_profileMode;
    std::vector<double> m_profileX, m_profileY; // indices in the image to profile
    std::vector<double> m_valsX, m_valsY;    // index and pixel value
    ProfilePlotter * m_profilePlot;          // the profile window

    // Use double buffering: draw to a pixmap first, refresh it only
    // if really necessary, and display it when paintEvent is called.
    QPixmap m_pixmap;

    bool m_bilinear_filter;
    bool m_use_colormap;

    std::vector<imageData> m_images;

    // We will render in this image's pixel or projected domain.
    // This only becomes important if using georeference, and the images
    // have different projections.
    imageData m_base_image; 

    BBox2 m_world_box;
    
    std::vector<vw::cartography::GeoTransform> m_world2image_geotransforms;
    std::vector<vw::cartography::GeoTransform> m_image2world_geotransforms;
    
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

    // Shrink the image to be shown on screen by this factor
    // (typically 0.90 to 0.95) to create an extra empty margin at a widget's
    // border, to make it easier to zoom.
    double m_border_factor;

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
    QAction* m_toggleHillshade;
    QAction* m_setThreshold;
    QAction* m_setHillshadeParams;
    QAction* m_saveScreenshot;
    QAction* m_toggleHillshadeFromTable;
    QAction* m_zoomToImageFromTable;
    QAction* m_bringImageOnTopFromTable;
    QAction* m_pushImageToBottomFromTable;
    QAction* m_deleteImage;
    QAction* m_allowMultipleSelections_action;
    QAction* m_deleteSelection;
    QAction* m_hideImagesNotInRegion;
    QAction* m_saveVectorLayer;
    QAction* m_deleteVertex;
    QAction* m_deleteVertices;
    QAction* m_insertVertex;
    QAction* m_moveVertex;
    QAction* m_showIndices;
    QAction* m_mergePolys;
    QAction* m_showPolysFilled;
    
    double m_shadow_thresh;
    bool   m_shadow_thresh_calc_mode;
    bool   m_shadow_thresh_view_mode;
    std::vector<imageData> m_shadow_thresh_images;

    std::vector<imageData> m_hillshaded_images;
    std::set<int> m_indicesWithAction;
    
    bool m_view_matches; ///< Control if IP's are drawn

    bool m_zoom_all_to_same_region; // if all widgets are forced to zoom to same region
    bool & m_allowMultipleSelections; // alias, this is controlled from MainWindow for all widgets
    bool m_can_emit_zoom_all_signal; 

    // Drawing is driven by QPaintEvent, which calls out to drawImage()
    void drawImage(QPainter* paint);
    /// Add all the interest points to the provided canvas
    /// - Called internally by drawImage
    void drawInterestPoints(QPainter* paint, std::list<BBox2i> const& valid_regions);

    vw::Vector2 world2screen(vw::Vector2 const& p) const;
    vw::Vector2 screen2world(vw::Vector2 const& pix) const;
    BBox2       world2screen(BBox2 const& R) const;
    BBox2       screen2world(BBox2 const& R) const;
    Vector2     world2image(Vector2 const& P, int imageIndex) const;
    Vector2     image2world(Vector2 const& P, int imageIndex) const;
    BBox2       world2image(BBox2 const& R, int imageIndex) const;
    BBox2       image2world(BBox2 const& R, int imageIndex) const;
    vw::Vector2 world2projpoint(vw::Vector2 P, int imageIndex) const;
    vw::Vector2 projpoint2world(vw::Vector2 P, int imageIndex) const;
    vw::BBox2   expand_box_to_keep_aspect_ratio(vw::BBox2 const& box);

    void updateCurrentMousePosition();
    void updateRubberBand(QRect & R);
    void refreshPixmap();
    void maybeGenHillshade();
    void showImage(std::string const& image_name);
    void bringImageOnTop(int image_index);
    void pushImageToBottom(int image_index);

    // For polygon drawing
    bool m_polyEditMode;
    std::vector<vw::geometry::dPoly> m_polyVec;
    int m_polyVecIndex; // which of the current images owns the poly vector layer
    vw::Vector2 m_startPix; // The first poly vertex being drawn in world coords
    std::vector<double> m_currPolyX, m_currPolyY;
    int m_editPolyVecIndex, m_editIndexInCurrPoly, m_editVertIndexInCurrPoly; 
    
    // Points closer than this are in some situations considered equal
    int m_pixelTol;

    QColor m_backgroundColor;
    
    double pixelToWorldDist(double pd);
    void appendToPolyVec(const vw::geometry::dPoly & P);
    void addPolyVert(double px, double py);
    
  };
  
}} // namespace vw::gui

#endif  // __STEREO_GUI_MAIN_WIDGET_H__
