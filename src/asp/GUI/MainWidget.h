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
#include <boost/shared_ptr.hpp>
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

  class chooseFilesDlg;
  
  namespace fs = boost::filesystem;

   /// This class handles user interaction with the a single image pane.
  class MainWidget : public QWidget {
    Q_OBJECT

  public:

    // Constructors/Destructor
    MainWidget(QWidget *parent,
               vw::GdalWriteOptions const& opt,
               int beg_image_id, int end_image_id, int base_image_id,
               std::vector<imageData> & images, // will be aliased
               std::string & output_prefix,     // will be aliased
               MatchList & matches,
               pairwiseMatchList & pairwiseMatches,
               pairwiseMatchList & pairwiseCleanMatches,
               int & editMatchPointVecIndex,
               chooseFilesDlg * chooseFiles, bool use_georef,
               bool zoom_all_to_same_region,
               bool & allowMultipleSelections // alias
              );
    virtual ~MainWidget();

    bool get_crop_win(QRect & win);

    // Set a default size for this widget.  This is usually overridden
    // by parent views.
    virtual QSize sizeHint () const { return QSize(500,500); }

    // Image Manipulation Methods
    void zoom       (double scale);
    void viewMatches();
    void setEditingMatches(bool editingMatches) { m_editingMatches = editingMatches; }
    bool getEditingMatches() const { return m_editingMatches; }
    void setThreshMode(bool turnOn) { m_thresh_calc_mode = turnOn; }
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

    void   setThreshold(double thresh); ///< Set the image threshold 
    double getThreshold();            ///< Get the image threshold

    void   setLineWidth(int lineWidth); ///< Set the line width for polygons
    int  getLineWidth(); ///< Get the line width for polygons
    
    void   setPolyColor(std::string const& polyColor); ///< Set the color of polygons
    std::string getPolyColor(); ///< Get the color of polygons
    
    void  setZoomAllToSameRegion(bool zoom_all_to_same_region);
    vw::BBox2 current_view();
    void  zoomToRegion (vw::BBox2 const& region);
    void  setHillshadeMode(bool hillshade_mode);
    BBox2 firstImagePixelBox() const;
    BBox2 firstImageWorldBox(vw::BBox2 const& image_box) const;
    void setWorldBox(vw::BBox2 const& world_box);
    vw::BBox2 worldBox() const;
    
    void setCropWin(vw::BBox2 const& stereoCropWin) {
      m_stereoCropWin = stereoCropWin;
    }

signals:
    void toggleViewMatchesSignal    ();
    void updateMatchesSignal        (); // this one will do less work and keep the zoom level
    void uncheckProfileModeCheckbox ();
    void uncheckPolyEditModeCheckbox();
    void zoomAllToSameRegionSignal  (int);

public slots:
    void sizeToFit();
    void showFilesChosenByUser (int rowClicked, int columnClicked);
    void zoomToImageInTableCell(int rowClicked, int columnClicked);
    void viewNextImage();
    void viewPrevImage();
    void toggleAllOnOff();
    void customMenuRequested(QPoint pos);
    void viewUnthreshImages();
    void viewThreshImages  (bool refresh_pixmap);
    void viewHillshadedImages(bool hillshade_mode);

    void addMatchPoint          (); ///< Add a new interest point (from right click menu)
    void deleteMatchPoint       (); ///< Delete an interest point (from right click menu)
    void setThreshold           (); ///< Set change image threshold (from right click menu)
    void setHillshadeParams     (); ///< Set the azimuth and elevation for hillshaded images.
    void toggleHillshadeImageRightClick(); ///< Turn on/off hillshading on right-click on image
    void toggleHillshadeFromImageList(); ///< Toggle hillshade by right-click on image list
    void refreshHillshade       (); ///< Update the display if the state of hillshading changed.
    void bringImageOnTopSlot    (); ///< Show this image on top of other images.
    void pushImageToBottomSlot  (); ///< Show all other images on top of this
    void zoomToImage            (); ///< Zoom to have this image in full view.
    void changePolyColor        (); ///< Change the color of given set of polygons
    void allowMultipleSelections(); ///< Allow the user to select multiple regions
    void deleteSelection        (); ///< Delete an area selected with the mouse at current point
    void hideImagesNotInRegion  (); ///< Hide images not intersecting a given region 
    void saveVectorLayer        (); ///< Save polygons in current layer as shapefile
    bool contourImage           (); ///< Contour an image at a specified threshold
    void setProfileMode (bool profile_mode); ///< Turn on and off the 1D profile tool
    void setPolyEditMode(bool polyEditMode, bool refresh); ///< Turn on and off the vector layer drawing
    void deleteVertex           (); ///< Delete a vertex from a vector layer
    void deleteVertices         (); ///< Delete poly vertices in selected region
    void insertVertex           (); ///< Insert an intermediate vertex at right-click
    void mergePolys             (); ///< Merge existing polygons
    void saveScreenshot         (); ///< Save a screenshot of the current imagery

  protected:

    // Setup
    bool eventFilter(QObject *obj, QEvent *E);
    void resizeEvent(QResizeEvent*);

    // Event handlers
    void paintEvent           (QPaintEvent *event);
    void mousePressEvent      (QMouseEvent *event);
    void mouseReleaseEvent    (QMouseEvent *event);
    void mouseMoveEvent       (QMouseEvent *event);
    void mouseDoubleClickEvent(QMouseEvent *event);
    void wheelEvent           (QWheelEvent *event);
    void enterEvent           (QEvent      *event);
    void leaveEvent           (QEvent      *event);
    void keyPressEvent        (QKeyEvent   *event);
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

    vw::GdalWriteOptions m_opt;

    /// Handle to parent GUI panel used to select which of the multiple "owned"
    ///  images should be currently displayed.
    /// - Null if there is only one image.
    chooseFilesDlg  *     m_chooseFiles;
    std::vector<int>      m_filesOrder;     ///< The order the images are drawn in.

    int m_beg_image_id;  // The id of the first image among m_images in this widget
    int m_end_image_id;  // The id of the image past the last image among m_images in this widget

    // The index of the image on top of which the rest are overlaid.
    // We will render in this image's pixel or projected domain. This
    // only becomes important if using georeference, and the images
    // have different projections.
    int m_base_image_id;
    
    // Note that this is an alias. We would like to be able to modify
    // in this widget the states of all images in m_images (such as
    // the flag noting if hillshading is on) which would persist after
    // the widgets themselves are gone when the display layout
    // changes. The images actually drawn in this widget have indices
    // in [m_beg_image_id, m_end_image_id) in m_images.
    std::vector<imageData> & m_images;

    std::string & m_output_prefix; // alias
    double m_hillshade_azimuth, m_hillshade_elevation;

    /// Structure to keep track of all interest point matches.
    /// - Note that these are aliass wrapping an object passed in through the constructor.
    MatchList & m_matchlist;
    pairwiseMatchList & m_pairwiseMatches;
    pairwiseMatchList & m_pairwiseCleanMatches;
    int       &m_editMatchPointVecIndex; /// Point being edited
    bool      m_editingMatches;          /// If we are in the middle of editing match points
    
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

    std::string m_polyColor;
    std::map<int, std::string> m_perImagePolyColor;
    int m_lineWidth;

    // The box which contains fully all images in the current widget,
    // in world coordinates.
    BBox2 m_world_box;
    
    // The box in world coordinates which has the current view and
    // last view.  This is normally smaller than m_world_box.
    vw::BBox2 m_current_view, m_last_view;

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

    // Mouse press  position
    int m_mousePrsX,  m_mousePrsY;

    // Right-click context menu
    QMenu  * m_ContextMenu;
    QAction* m_addMatchPoint;
    QAction* m_deleteMatchPoint;
    QAction* m_moveMatchPoint;
    QAction* m_toggleHillshadeImageRightClick;
    QAction* m_setThreshold;
    QAction* m_setHillshadeParams;
    QAction* m_saveScreenshot;
    QAction* m_toggleHillshadeFromImageList;
    QAction* m_zoomToImageFromTable;
    QAction* m_bringImageOnTopFromTable;
    QAction* m_pushImageToBottomFromTable;
    QAction* m_changePolyColor;
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
    
    double m_thresh;
    bool   m_thresh_calc_mode;

    std::set<int> m_indicesWithAction;
    
    bool   m_zoom_all_to_same_region; // if all widgets are forced to zoom to same region
    bool & m_allowMultipleSelections; // alias, this is controlled from MainWindow for all widgets
    bool   m_can_emit_zoom_all_signal; 

    // Drawing is driven by QPaintEvent, which calls out to drawImage()
    void drawImage(QPainter* paint);
    /// Add all the interest points to the provided canvas
    /// - Called internally by paintEvent()
    void drawInterestPoints(QPainter* paint);

    // Draw irregular xyz data to be plotted at (x, y) location with z giving
    // the intensity. May be colorized.
    void drawScatteredData(QPainter* paint, int image_index);
    
    Vector2 world2screen   (Vector2 const  p  ) const;
    Vector2 screen2world   (Vector2 const  pix) const;
    BBox2   world2screen   (BBox2   const& R  ) const;
    BBox2   screen2world   (BBox2   const& R  ) const;
    Vector2 world2image    (Vector2 const  P, int imageIndex) const;
    Vector2 image2world    (Vector2 const  P, int imageIndex) const;
    BBox2   world2image    (BBox2   const& R, int imageIndex) const;
  public:
    // Need this public
    BBox2   image2world    (BBox2   const& R, int imageIndex) const;
  private:
    Vector2 world2projpoint(Vector2 const  P, int imageIndex) const;
    Vector2 projpoint2world(Vector2 const  P, int imageIndex) const;
    BBox2   expand_box_to_keep_aspect_ratio(vw::BBox2 const& box);

    // Find the closest point in a given set of imageData structures to a given point
    // in world coordinates.
    void findClosestPolyVertex(// inputs
                               double x0, double y0,
                               std::vector<imageData> const& imageData,
                               // outputs
                               int & clipIndex, 
                               int & polyVecIndex,
                               int & polyIndexInCurrPoly,
                               int & vertIndexInCurrPoly,
                               double & minX, double & minY,
                               double & minDist);
    
    // Find the closest edge in a given set of imageData structures to a given point.
    void findClosestPolyEdge(// inputs
                             double x0, double y0,
                             std::vector<imageData> const& imageData,
                             // outputs
                             int & clipIndex,
                             int & polyVecIndex,
                             int & polyIndexInCurrPoly,
                             int & vertIndexInCurrPoly,
                             double & minX, double & minY,
                             double & minDist);
    
    // Merge some polygons and save them in imageData[outIndex]
    void mergePolys(std::vector<imageData> & imageData, int outIndex);
    
    void updateCurrentMousePosition();
    void updateRubberBand(QRect & R);
    void refreshPixmap();
    void maybeGenHillshade();
    void showImage        (std::string const& image_name);
    void bringImageOnTop  (int image_index);
    void pushImageToBottom(int image_index);

    void viewOtherImage(int delta);
    void updateFilesToHide();
    
    // For polygon drawing
    bool        m_polyEditMode;
    int         m_polyLayerIndex; // which of the current images owns the poly vector layer
    vw::Vector2 m_startPix; // The first poly vertex being drawn in world coords
    std::vector<double> m_currPolyX, m_currPolyY;
    int         m_editPolyVecIndex, m_editIndexInCurrPoly, m_editVertIndexInCurrPoly; 
    
    // Points closer than this are in some situations considered equal
    int m_pixelTol;

    QColor m_backgroundColor;
    
    double pixelToWorldDist(double pd);
    void   appendToPolyVec (vw::geometry::dPoly const& P);
    void   addPolyVert     (double px, double py);
    
  };
  
}} // namespace vw::gui

#endif  // __STEREO_GUI_MAIN_WIDGET_H__
