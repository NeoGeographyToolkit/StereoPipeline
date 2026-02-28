// __BEGIN_LICENSE__
//  Copyright (c) 2006-2026, United States Government as represented by the
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

// \file MainWindow.cc
// 
// The stereo_gui main window class.
// 
#include <QtGui>
#include <QtWidgets>
#include <asp/GUI/MainWindow.h>
#include <asp/GUI/AppData.h>
#include <asp/GUI/GuiConstants.h>
#include <asp/GUI/GuiArgs.h>
#include <asp/GUI/MainWidget.h>
#include <asp/Core/StereoSettings.h>
#include <asp/GUI/ChooseFilesDlg.h>
#include <asp/Core/GCP.h>
#include <asp/Core/Nvm.h>
#include <asp/Core/IpMatchingAlgs.h>
#include <asp/Camera/BundleAdjustIsis.h>

#include <vw/vw_config.h>
#include <vw/Core/CmdUtils.h>
#include <vw/Image/MaskViews.h>
#include <vw/FileIO/DiskImageView.h>
#include <vw/FileIO/DiskImageResource.h>
#include <vw/Image/Statistics.h>
#include <vw/Image/PixelMask.h>
#include <vw/InterestPoint/InterestPoint.h>
#include <vw/InterestPoint/MatcherIO.h>
#include <vw/FileIO/FileTypes.h>

#include <boost/filesystem/path.hpp>

#include <vw/Image/Colormap.h>

#include <qwt_color_map.h>
#include <qwt_scale_widget.h>
#include <qwt_scale_engine.h>
#include <qwt_scale_draw.h>
#include <qwt_interval.h>

#include <sstream>

using namespace asp;
using namespace vw;

namespace fs = boost::filesystem;

// Need this class to manage what happens when keys are pressed while
// the ChooseFilesDlg table is in focus. Do not let it accept key
// strokes, which just end up editing the table entries, but rather
// pass them to the main program event filter.
class chooseFilesFilterDelegate: public QStyledItemDelegate {
public:
  chooseFilesFilterDelegate(QObject *filter, QObject *parent = 0):
    QStyledItemDelegate(parent), filter(filter) {}

  virtual QWidget *createEditor(QWidget *parent,
                                const QStyleOptionViewItem &option,
                                const QModelIndex &index) const {
    QWidget *editor = QStyledItemDelegate::createEditor(parent, option, index);
    editor->installEventFilter(filter);
    return editor;
  }
  
private:
  QObject *filter;
};

bool MainWindow::sanityChecks(int num_images) {
  
  if (num_images <= 0) {
    popUp("No valid images to display.");
    return false;
  }

  if (stereo_settings().dem_file == "" && stereo_settings().gcp_file != "" && 
      !fs::exists(stereo_settings().gcp_file)) {
    popUp("The GCP file does not exist. If desired to create it, please specify --dem-file.");
    return false;
  }
  
  if (stereo_settings().gcp_sigma <= 0) {
    popUp("The GCP sigma must be positive.");
    return false;
  }
  
  // The dem file must exist if not empty. Must also have a georef.
  if (stereo_settings().dem_file != "") {
    try {
      DiskImageView<float> dem(stereo_settings().dem_file);
    } catch (...) {
      popUp("The DEM file does not exist or is not a valid image file.");
      return false;
    }
    vw::cartography::GeoReference georef;
    bool has_georef = vw::cartography::read_georeference(georef, stereo_settings().dem_file);
    if (!has_georef) {
      popUp("The DEM file does not have a georeference. Please provide one.");
      return false;
    }
  }
                    
  // This is for the workflow of creating GCP from a DEM and images
  bool gcp_exists = !stereo_settings().gcp_file.empty() && 
                    fs::exists(stereo_settings().gcp_file);
  if (gcp_exists && stereo_settings().dem_file != "") {
    popUp("A DEM file and GCP file were specified. Then, the GCP file passed in "
          "must not exist, since it is supposed to be created based on the DEM.");
    return false;
  }
  
  if (int(!stereo_settings().match_file.empty()) +
      int(!stereo_settings().vwip_files.empty()) +
      int(gcp_exists) > 1) {
    // We make an exception for the non-existing GCP file, per the above
    popUp("Cannot load at the same time more than one of: matches, GCP, or .vwip files.");
    return false;
  }

  m_saved_gcp_and_ip = true;
  if (MainWindow::creatingGcp()) 
    m_saved_gcp_and_ip = false; // will need to save GCP and IP files
   
  // If a gcp file was passed in, we will interpret those as matches
  if (stereo_settings().gcp_file != "")
    asp::stereo_settings().view_matches = true;

  // If .vwip files were passed in, we will interpret those as matches.
  if (!stereo_settings().vwip_files.empty()) {
    asp::stereo_settings().view_matches = true;
    if (app_data.image_files.size() != stereo_settings().vwip_files.size()) {
      popUp("There must be as many .vwip files as images.");
      return false;
    }    
  }
  
  // If a match file was explicitly specified, use it.
  if (stereo_settings().match_file != "") {
    if (app_data.image_files.size() != 2) {
      popUp("The --match-file option only works with two valid input images.");
      return false;
    }
    asp::stereo_settings().view_matches = true;
  }

  int num =
    int(asp::stereo_settings().view_matches) +
    int(asp::stereo_settings().pairwise_matches) +
    int(asp::stereo_settings().pairwise_clean_matches);
  if (num > 1) {
    popUp("Conflicting options for viewing match files were specified.");
    return false;
  }

  // Cannot show matches when viewing as georeferenced images
  if (num > 0)
    app_data.use_georef = false;

  if (num > 0 && !stereo_settings().zoom_proj_win.empty()) {
    popUp("Cannot zoom to proj win when showing matches.");
    return false;
  }
  
  // Need at least two images to view matches, but make an exception for vwip files
  if (num_images <= 1 && stereo_settings().vwip_files.empty() &&
      (asp::stereo_settings().view_matches ||
       asp::stereo_settings().pairwise_matches ||
       asp::stereo_settings().pairwise_clean_matches)
      && stereo_settings().gcp_file.empty()) {
    popUp("Cannot view matches if there is at most one image.");
    return false;
  }

  if (asp::stereo_settings().plot_point_radius <= 0) {
    popUp("The value --plot-point-radius must be positive.");
    return false;
  }

  if (asp::stereo_settings().preview && sideBySideWithDialog()) {
    popUp("Cannot have both the preview mode and side-by-side images.");
    return false;
  }
  
  if (asp::stereo_settings().no_georef && asp::stereo_settings().use_georef) {
    popUp("Cannot have both the no-georef and use-georef options.");
    return false;
  }

  return true;
}

MainWindow::MainWindow(vw::GdalWriteOptions const& opt,
                       std::vector<std::string> const& images,
                       std::string& output_prefix,
                       int grid_cols,
                       vw::Vector2i const& window_size,
                       bool single_window,
                       bool use_georef,
                       std::vector<std::map<std::string, std::string>> const& properties,
                       int argc,  char ** argv):
  m_opt(opt),
  m_output_prefix(output_prefix), m_widRatio(0.3), m_chooseFiles(NULL),
  m_grid_cols(grid_cols),
  m_allowMultipleSelections(false),
  m_argc(argc), m_argv(argv),
  m_show_two_images_when_side_by_side_with_dialog(true),
  m_cursor_count(0),
  m_saved_gcp_and_ip(true),
  m_match_mgr(app_data) { // Initialize m_match_mgr with app_data
  // Window size
  resize(window_size[0], window_size[1]);

  // Window title
  std::string window_title = "Stereo GUI";
  this->setWindowTitle(window_title.c_str());

  // The images and other data
  std::vector<std::string> local_images = images; // may change later
  std::vector<Eigen::Affine3d> world_to_cam;
  std::map<std::string, Eigen::Vector2d> optical_offsets;

  // When loading an NVM file, will assume we want to inspect pairwise
  // matches. Also set the images from the NVM file.  It is assumed
  // that the interest points to be loaded are not shifted relative to
  // the optical center. Analogous logic happens for ISIS cnet files.
  if (!asp::stereo_settings().nvm.empty() && !asp::stereo_settings().isis_cnet.empty()) {
    popUp("Cannot load both an ISIS cnet and an NVM file.");
    exit(1);
  }
  if (!asp::stereo_settings().nvm.empty()) {
    asp::stereo_settings().pairwise_matches = true;
    vw_out() << "Reading NVM file: " << asp::stereo_settings().nvm << "\n";
    try {
       asp::readNvmAsCnet(asp::stereo_settings().nvm, std::vector<std::string>(),
                          asp::stereo_settings().nvm_no_shift,
                          m_match_mgr.m_cnet, world_to_cam, optical_offsets);
    } catch (const std::exception& e) {
      popUp(e.what());
      exit(1);
    }
    if (!local_images.empty())
      popUp("Will ignore the images passed in and will use the nvm file only.");
    local_images = m_match_mgr.m_cnet.get_image_list(); // overwrite local_images
  }
  if (!asp::stereo_settings().isis_cnet.empty()) {
    asp::stereo_settings().pairwise_matches = true;
    vw::vw_out() << "Reading ISIS control network: " 
                 << asp::stereo_settings().isis_cnet << "\n";
    
    try {
      asp::IsisCnetData isisCnetData; // Part of the API, unused here
      asp::loadIsisCnet(asp::stereo_settings().isis_cnet, local_images,
                        m_match_mgr.m_cnet, isisCnetData); // outputs
    } catch (const std::exception& e) {
      popUp(e.what());
      exit(1);
    }
  }

  // Collect only the valid images
  asp::filterImages(local_images);
  
  // Set the default lowest resolution subimage size. Use a shorthand below.
  // Must happen before images are loaded.
  int & lowres_size = asp::stereo_settings().lowest_resolution_subimage_num_pixels;
  bool delay = asp::stereo_settings().preview;
  if (lowres_size <= 0) {
    if (delay) {
      // To avoid creating many small subimages. But then the displaying is slow.
      lowres_size = LOWRES_SIZE_PREVIEW;
    } else {
      lowres_size = LOWRES_SIZE_DEFAULT;
    }
  }
  
  // All the data is stored and shared via with object
  app_data = asp::AppData(opt, use_georef, properties, local_images);
  
  // Ensure the inputs are reasonable
  int num_images = app_data.images.size();
  if (!MainWindow::sanityChecks(num_images))
    forceQuit();

  // For being able to choose which files to show/hide
  m_chooseFiles = new ChooseFilesDlg(this);
  m_chooseFiles->chooseFiles(app_data.image_files);
  // See note at chooseFilesFilterDelegate
  m_chooseFiles->getFilesTable()->setItemDelegate(new chooseFilesFilterDelegate(this));
  m_chooseFiles->getFilesTable()->installEventFilter(this);
  
  QObject::connect(m_chooseFiles->getFilesTable(), SIGNAL(cellClicked(int, int)),
                   this, SLOT(perhapsCreateLayout(int, int)));

  QObject::connect(m_chooseFiles->getFilesTable()->horizontalHeader(),
                   SIGNAL(sectionClicked(int)), this, SLOT(hideShowAll_windowVersion()));

  m_match_mgr.init(num_images);

  // By default, show the images in one row
  if (m_grid_cols <= 0)
    m_grid_cols = std::numeric_limits<int>::max();

  // If no custom view mode, this is the default
  m_view_type = VIEW_SIDE_BY_SIDE;

  // Preview mode normally implies a single window.
  if (asp::stereo_settings().preview && !sideBySideWithDialog()) 
    single_window = true;
  
  if (m_grid_cols > 0 && m_grid_cols < int(num_images) && !sideBySideWithDialog())
    m_view_type = VIEW_AS_TILES_ON_GRID;
  
  if (single_window && !sideBySideWithDialog())
    m_view_type = VIEW_IN_SINGLE_WINDOW;

  m_view_type_old = m_view_type; // initialize this
  
  // Set up the basic layout of the window and its menus
  m_win_menu_mgr.init(this);

  // Must happen after menus are created
  createLayout();
}

// Wrap a widget in a horizontal layout with a colorbar on the right.
// Returns the wrapper widget, or the original widget if bounds are invalid.
// The colormap style is taken from the first image in the range.
QWidget* createColorbarLayout(QWidget* widget,
                              double min_val, double max_val,
                              std::string const& colormap_style) {

  if (min_val >= max_val)
    return widget;

  // Parse the colormap
  std::map<float, vw::Vector3u> lut_map;
  vw::parseColorStyle(colormap_style, lut_map);

  // Build a QwtLinearColorMap from the LUT
  auto firstC = lut_map.begin()->second;
  auto lastC = lut_map.rbegin()->second;
  QwtLinearColorMap *cmap =
    new QwtLinearColorMap(QColor(firstC[0], firstC[1], firstC[2]),
                          QColor(lastC[0], lastC[1], lastC[2]));
  for (auto it = lut_map.begin(); it != lut_map.end(); it++) {
    if (it->first == 0.0 || it->first == 1.0)
      continue;
    auto const& c = it->second;
    cmap->addColorStop(it->first, QColor(c[0], c[1], c[2]));
  }

  // Create the colorbar widget
  QwtScaleWidget *colorbar =
    new QwtScaleWidget(QwtScaleDraw::RightScale);
  QwtInterval interval(min_val, max_val);
  colorbar->setColorBarEnabled(true);
  colorbar->setColorBarWidth(30);
  colorbar->setColorMap(interval, cmap);
  QwtLinearScaleEngine engine;
  colorbar->setScaleDiv(
    engine.divideScale(min_val, max_val, 8, 5));

  // Wrap in a horizontal layout
  QWidget *wrapper = new QWidget();
  QHBoxLayout *hbox = new QHBoxLayout(wrapper);
  hbox->setContentsMargins(0, 0, 0, 0);
  hbox->setSpacing(0);
  hbox->addWidget(widget, 1);
  hbox->addWidget(colorbar, 0);

  return wrapper;
}

// Create a new central widget. Qt is smart enough to de-allocate
// the previous widget and all of its children.
void MainWindow::createLayout() {

  MainWindow::updateViewMenuEntries();

  // We must cleanup the previous profiles before wiping the existing
  // widgets. There must be a better way of doing it.
  for (size_t wit = 0; wit < m_widgets.size(); wit++) {
    bool profile_mode = false;
    if (m_widgets[wit]) 
      m_widgets[wit]->setProfileMode(profile_mode);
  }

  QWidget * centralWidget = new QWidget(this);
  setCentralWidget(centralWidget);

  QSplitter * splitter = new QSplitter(centralWidget);

  // Wipe the widgets from the array. Qt will automatically delete
  // the widgets when the time is right.
  m_widgets.clear();

  // Note that the menus persist even when the layout changes

  // Show all images if switching to side-by-side view without a dialog
  if ((asp::stereo_settings().zoom_all_to_same_region ||
       m_view_type == VIEW_SIDE_BY_SIDE ||
       m_view_type == VIEW_AS_TILES_ON_GRID) && !sideBySideWithDialog())
    m_chooseFiles->showAllImages();
  
  // Set up the dialog for choosing files
  m_chooseFiles->setMaximumSize(int(m_widRatio*size().width()), size().height());

  if (asp::stereo_settings().preview && !sideBySideWithDialog()) {
    // Show only the first image, unless it was decided somewhere else
    // another one image is to be shown.
    if (m_chooseFiles && m_chooseFiles->numShown() != 1) 
      m_chooseFiles->setNumImagesToShow(1);
  }
  
  // When showing matches or other ways of showing images side-by-side,
  // automatically change the layout
  if (sideBySideWithDialog() || asp::stereo_settings().view_matches) {
    m_view_type = VIEW_SIDE_BY_SIDE;
    if (sideBySideWithDialog() && m_show_two_images_when_side_by_side_with_dialog) {
      // This must be triggered only when the tool starts or when switched
      // into sideBySideWithDialog() mode from the menu. Later the use
      // can change the number of images shown.
      m_chooseFiles->setNumImagesToShow(2);
      m_show_two_images_when_side_by_side_with_dialog = false;
    }
  }

  splitter->addWidget(m_chooseFiles);

  bool do_colorize = false;
  for (size_t i = 0; i < app_data.images.size(); i++)
    do_colorize = do_colorize || app_data.images[i].colorize ||
                  app_data.images[i].colorbar;
  bool delay = !asp::stereo_settings().nvm.empty()       ||
               !asp::stereo_settings().isis_cnet.empty() ||
                asp::stereo_settings().preview;
  if (delay && do_colorize) {
    popUp("Cannot colorize images when using the preview mode or when "
          "loading an NVM or ISIS cnet file.");
    // This is hard to get right
    do_colorize = false;
  }
  if (do_colorize && m_view_type == VIEW_IN_SINGLE_WINDOW) {
     if (app_data.images.size() > 1) // for one image do this quietly
      popUp("Colorized images can only be shown side-by-side.");
    m_view_type = VIEW_SIDE_BY_SIDE;
  }

  // See if to show it. In a side-by-side view it is normally not needed
  size_t num_images = app_data.images.size();
  bool showChooseFiles = ((m_view_type == VIEW_IN_SINGLE_WINDOW || sideBySideWithDialog()) &&
                          num_images > 1);
  m_chooseFiles->setVisible(showChooseFiles);

  if (m_view_type == VIEW_IN_SINGLE_WINDOW) {
    // Pass all images to a single MainWidget object. No colorizing in this mode.
    int beg_image_id = 0, end_image_id = app_data.images.size();
    MainWidget * widget = new MainWidget(centralWidget,
                                m_opt,
                                beg_image_id, end_image_id, BASE_IMAGE_ID, app_data, 
                                m_output_prefix,
                                m_match_mgr,
                                m_chooseFiles,
                                m_allowMultipleSelections);
    // Tell the widget if the poly edit mode and hillshade mode is on or not
    bool refresh = false; // Do not refresh prematurely
    widget->setPolyEditMode(m_win_menu_mgr.m_polyEditMode_action->isChecked(), refresh);
    m_widgets.push_back(widget);
  } else {
    // Each MainWidget object gets passed a single image
    for (size_t i = 0; i < app_data.images.size(); i++) {

      // Do not create hidden widgets, that really slows down the display when there
      // are many of them, but just a handful are needed.
      bool isHidden = (sideBySideWithDialog() && m_chooseFiles &&
                       m_chooseFiles->isHidden(app_data.images[i].name));
      if (isHidden) 
        continue;

      int beg_image_id = i, end_image_id = i + 1;

      MainWidget * widget
        = new MainWidget(centralWidget,
                         m_opt,
                         beg_image_id, end_image_id,
                         BASE_IMAGE_ID, app_data,
                         m_output_prefix,
                         m_match_mgr,
                         m_chooseFiles,
                         m_allowMultipleSelections);
      m_widgets.push_back(widget);
    }
  }

  // Once all widgets zoom to same proj win, turn this off
  asp::stereo_settings().zoom_proj_win = BBox2();

  // Put the images in a grid
  int num_widgets = m_widgets.size();
  QGridLayout *grid = new QGridLayout(centralWidget);

  // By default, show the images side-by-side
  int grid_cols = std::numeric_limits<int>::max();
  if (m_view_type == VIEW_AS_TILES_ON_GRID) 
    grid_cols = m_grid_cols;
  
  for (int i = 0; i < num_widgets; i++) {

    if (m_widgets[i] == NULL)
      continue;
      
    // Add the current widget
    int row = i / grid_cols;
    int col = i % grid_cols;

    // When --colorbar is set, wrap MainWidget + colorbar in a container
    QWidget* wid = m_widgets[i];
    int begIdx = m_widgets[i] ? m_widgets[i]->m_beg_image_id : 0;
    int endIdx = m_widgets[i] ? m_widgets[i]->m_end_image_id : 0;
    if (m_widgets[i] && app_data.images[begIdx].colorbar) {
      vw::Vector2 bounds = calcJointBounds(app_data.images,
                                           begIdx, endIdx);
      wid = createColorbarLayout(m_widgets[i],
                                 bounds[0], bounds[1],
                                 app_data.images[begIdx].colormap);
    }
    grid->addWidget(wid, row, col);

    if (!m_widgets[i])
      continue; // Only MainWidget type can be used below

    // Intercept this widget's request to view (or refresh) the matches in all
    // the widgets, not just this one's.
    connect(m_widgets[i], SIGNAL(toggleViewMatchesSignal()),
            this, SLOT(toggleViewMatches()));
    connect(m_widgets[i], SIGNAL(updateMatchesSignal()),
            this, SLOT(viewMatches()));
    connect(m_widgets[i], SIGNAL(uncheckProfileModeCheckbox()),
            this, SLOT(uncheckProfileModeCheckbox()));
    connect(m_widgets[i], SIGNAL(uncheckPolyEditModeCheckbox()),
            this, SLOT(uncheckPolyEditModeCheckbox()));
    connect(m_widgets[i], SIGNAL(zoomAllToSameRegionSignal(int)),
            this, SLOT(zoomAllToSameRegionAction(int)));
    connect(m_widgets[i], SIGNAL(recreateLayoutSignal()),
            this, SLOT(createLayout()));
  }

  QWidget *container = new QWidget(centralWidget);
  container->setLayout(grid);
  splitter->addWidget(container);

  // Set new layout
  QGridLayout *layout = new QGridLayout(centralWidget);
  layout->addWidget(splitter, 0, 0);
  centralWidget->setLayout(layout);

  if (asp::stereo_settings().view_matches)
    MainWindow::viewMatches();

  if (asp::stereo_settings().pairwise_matches || asp::stereo_settings().pairwise_clean_matches) 
    MainWindow::viewPairwiseMatchesOrCleanMatches();

  double nodata_value = stereo_settings().nodata_value;
  if (!std::isnan(nodata_value)) {
    for (size_t i = 0; i < m_widgets.size(); i++) {
      if (!m_widgets[i])
        continue;
      m_widgets[i]->setThreshold(nodata_value);
      bool refresh_pixmap = false; // Prepare everything but don't redraw yet
      m_widgets[i]->viewThreshImages(refresh_pixmap);
    }
  }

  // Refresh the menu checkboxes
  auto& wm = m_win_menu_mgr;
  wm.m_viewSingleWindow_action
    ->setChecked(m_view_type == VIEW_IN_SINGLE_WINDOW);
  wm.m_viewAllSideBySide_action
    ->setChecked(m_view_type == VIEW_SIDE_BY_SIDE &&
                 !asp::stereo_settings().view_several_side_by_side);
  wm.m_viewSeveralSideBySide_action
    ->setChecked(asp::stereo_settings().view_several_side_by_side);
  wm.m_viewAsTiles_action
    ->setChecked(m_view_type == VIEW_AS_TILES_ON_GRID);
  MainWindow::updateDisplayModeMenuEntries();
  wm.m_viewGeoreferencedImages_action
    ->setChecked(app_data.use_georef);
  wm.m_overlayGeoreferencedImages_action
    ->setChecked(app_data.use_georef &&
                 (m_view_type == VIEW_IN_SINGLE_WINDOW));
  wm.m_zoomAllToSameRegion_action
    ->setChecked(asp::stereo_settings().zoom_all_to_same_region);

  if (m_widgets.size() == 2                   &&
      num_images == 2                         &&
      asp::stereo_settings().left_image_crop_win  != BBox2() &&
      asp::stereo_settings().right_image_crop_win != BBox2()) {
    // Draw crop windows passed as arguments
    if (m_widgets[0])
      m_widgets[0]->setCropWin(stereo_settings().left_image_crop_win);
    if (m_widgets[1])
      m_widgets[1]->setCropWin(stereo_settings().right_image_crop_win);
    // Do this just once, on startup
    asp::stereo_settings().left_image_crop_win  = BBox2();
    asp::stereo_settings().right_image_crop_win = BBox2();
  }
  
  if (asp::stereo_settings().zoom_all_to_same_region) {
    // See where the earlier viewable region ended up in the current world
    // coordinate system. A lot of things could have changed since then.
    BBox2 world_box;
    for (size_t i = 0; i < m_widgets.size(); i++) {
      if (m_widgets[i]) {
        vw::BBox2 region = m_widgets[i]->worldBox();
        world_box.grow(region);
      }
    }
    // Let this be the world box in all images. Later, during resizeEvent(), the
    // sizeToFit() function will be called which will use this box.
    for (size_t i = 0; i < m_widgets.size(); i++) {
      if (m_widgets[i])
        m_widgets[i]->setWorldBox(world_box);
    }
  }
  
  return;
}

// In previewOrSideBySideWithDialog mode, checking/unchecking images has to result
// in redisplay of selected ones.
void MainWindow::perhapsCreateLayout(int row, int col) {

  if (m_chooseFiles && asp::stereo_settings().preview) {
    // If clicked on a cell, must show this image, and hide the rest
    int rows = m_chooseFiles->getFilesTable()->rowCount();
    for (int row_it = 0; row_it < rows; row_it++) {
      if (row_it == row) 
        m_chooseFiles->unhide(row_it);
      else
        m_chooseFiles->hide(row_it);
    }
  }
    
  if (previewOrSideBySideWithDialog())
    createLayout();
}

// When in sideBySideWithDialog mode, hide or show all images. Otherwise
// this function does nothing, and hideShowAll_widgetVersion() gets called.
void MainWindow::hideShowAll_windowVersion() {
  if (!sideBySideWithDialog()) {
    // The function hideShowAll_widgetVersion will be called, which
    // does not need to recreate the layout.
    return;
  }
  
  if (m_chooseFiles) 
    m_chooseFiles->hideShowAll();
  
  // This ensures we don't override how many images are shown
  m_show_two_images_when_side_by_side_with_dialog = false;
  createLayout();
}

void MainWindow::resizeEvent(QResizeEvent *) {
  if (m_chooseFiles)
    m_chooseFiles->setMaximumSize(int(m_widRatio*size().width()), size().height());
}

void MainWindow::closeEvent(QCloseEvent *) {
  forceQuit();
}

bool MainWindow::editingMatches() const {
  // See if in the middle of editing of matches
  bool editing_matches = false;
  for (size_t i = 0; i < m_widgets.size(); i++) {
    if (m_widgets[i] != NULL && m_widgets[i]->getEditingMatches()) 
      editing_matches = true;
  }
  return editing_matches;
}

// We can create GCP when --gcp-file and --dem-file got specified,
// and the GCP file does not exist yet.
bool MainWindow::creatingGcp() const {
 return (stereo_settings().dem_file != "" && 
         stereo_settings().gcp_file != "" && 
         !fs::exists(asp::stereo_settings().gcp_file));
}

void MainWindow::forceQuit() {

  // See if in the middle of creating GCP
  bool force_quit = false;
  if (!m_saved_gcp_and_ip) {
    QMessageBox::StandardButton reply;
    reply = QMessageBox::question(this, "Quit",
                                  "GCP and IP matches were not saved. Quit?",
                                  QMessageBox::Yes|QMessageBox::No);
    if (reply != QMessageBox::Yes) 
      return;

    // If we want to quit without saving GCP, will not pop up the dialog again
    force_quit = true;
  }
  
  // See if in the middle of editing of matches with unsaved matches
  if (!force_quit && MainWindow::editingMatches()) {
    QMessageBox::StandardButton reply;
    reply = QMessageBox::question(this, "Quit",
                                  "Interest point matches were not saved. Quit?",
                                  QMessageBox::Yes|QMessageBox::No);
    if (reply != QMessageBox::Yes)
      return;
  }
  
  if (asp::stereo_settings().delete_temporary_files_on_exit) {
    std::set<std::string> & tmp_files = asp::temporary_files().files;
    for (auto it = tmp_files.begin(); it != tmp_files.end() ; it++) {
      std::string file = *it;
      if (fs::exists(file)) {
        vw_out() << "Deleting: " << file << std::endl;
        fs::remove(file);
      }
    }
  }

  exit(0); // A fix for an older buggy version of Qt
}

// Zoom in/out of each image so that it fits fully within its allocated display area.
// If we zoom all images to same region, zoom all to the union of display areas.
void MainWindow::sizeToFit() {

  if (asp::stereo_settings().zoom_all_to_same_region) {

    BBox2 big_region; 
    for (size_t i = 0; i < m_widgets.size(); i++) {
      if (!m_widgets[i])
        continue;

      vw::BBox2 region = m_widgets[i]->worldBox();
      big_region.grow(region);
    }

    for (size_t i = 0; i < m_widgets.size(); i++) {
      if (!m_widgets[i])
        continue;
      m_widgets[i]->zoomToRegion(big_region);
    }

  }else{
    // Full view for each individual image
    for (size_t i = 0; i < m_widgets.size(); i++) {
      if (m_widgets[i])
        m_widgets[i]->sizeToFit();
    }
  }

}

void MainWindow::zoomIn() {
  for (size_t i = 0; i < m_widgets.size(); i++)
    if (m_widgets[i])
      m_widgets[i]->zoom(1.0/0.75);
}

void MainWindow::zoomOut() {
  for (size_t i = 0; i < m_widgets.size(); i++)
    if (m_widgets[i])
      m_widgets[i]->zoom(0.75);
}

void MainWindow::viewSingleWindow() {

  bool single_window = m_win_menu_mgr.m_viewSingleWindow_action->isChecked();

  if (single_window) {

    if (m_view_type != VIEW_IN_SINGLE_WINDOW)
      m_view_type_old = m_view_type; // back this up
    
    m_view_type = VIEW_IN_SINGLE_WINDOW;

    // Since we will view all in single window, can't select images with matches
    asp::stereo_settings().view_matches = false;
    setNoSideBySideWithDialog();
    MainWindow::updateViewMenuEntries();

    // Turn off zooming all images to same region if all are in the same window
    if (asp::stereo_settings().zoom_all_to_same_region) {
      asp::stereo_settings().zoom_all_to_same_region = false;
      setZoomAllToSameRegionAux(asp::stereo_settings().zoom_all_to_same_region);
    }

  }else{
    if (m_view_type_old != VIEW_IN_SINGLE_WINDOW)
      m_view_type = m_view_type_old; // restore this
    else{
      // nothing to restore to. Default to side by side.
      m_view_type = VIEW_SIDE_BY_SIDE;
    }
  }

  createLayout();
}

void MainWindow::viewAllSideBySide() {
  m_view_type = VIEW_SIDE_BY_SIDE;
  asp::stereo_settings().preview = false;
  setNoSideBySideWithDialog();
  createLayout();
}

void MainWindow::viewSeveralSideBySide() {
  m_view_type = VIEW_SIDE_BY_SIDE;
  asp::stereo_settings().preview = false;
  asp::stereo_settings().view_several_side_by_side = true;
  m_show_two_images_when_side_by_side_with_dialog = true; // start with 2 
  createLayout();
}

void MainWindow::viewAsTiles() {

  if (!m_win_menu_mgr.m_viewAsTiles_action->isChecked()) {
    if (m_view_type_old != VIEW_AS_TILES_ON_GRID)
      m_view_type = m_view_type_old; // restore this
    else
      m_view_type = VIEW_SIDE_BY_SIDE;

    asp::stereo_settings().preview = false;
    setNoSideBySideWithDialog();
    
    createLayout();
    return;
  }
  
  std::string gridColsStr;
  bool ans = getStringFromGui(this,
                              "Number of columns in the grid",
                              "Number of columns in the grid",
                              "",
                              gridColsStr);
  if (!ans)
    return;

  m_grid_cols     = std::max(atoi(gridColsStr.c_str()), 1);
  m_view_type_old = m_view_type; // back this up
  m_view_type     = VIEW_AS_TILES_ON_GRID;

  // When viewing as tile we cannot show matches
  asp::stereo_settings().view_matches = false;
  asp::stereo_settings().preview = false;
  setNoSideBySideWithDialog();
  MainWindow::updateViewMenuEntries();
  
  createLayout();
}

void MainWindow::zoomToProjWin() {
  std::string projWinStr;
  bool ans = getStringFromGui(this,
                              "Enter proj win (4 values)",
                              "Enter proj win (4 values)",
                              "",
                              projWinStr);
  if (!ans)
    return;

  std::istringstream is(projWinStr);
  double a, b, c, d;
  if (!(is >> a >> b >> c >> d)) {
    popUp("Four float values expected.");
    return;
  }

  if (!m_widgets[BASE_IMAGE_ID]) {
    popUp("Unexpected missing widget.");
    return;
  }

  if (!app_data.use_georef) {
    popUp("Turn on viewing georeferenced images to zoom to given proj win.");
    return;
  }
  
  // This takes care of the fact that the order of corners can be reversed
  BBox2 proj_win;
  proj_win.grow(Vector2(a, b));
  proj_win.grow(Vector2(c, d));

  BBox2 image_box;
  if (app_data.images[BASE_IMAGE_ID].isPolyOrCsv())
    image_box = proj_win;
  else
    image_box = app_data.images[BASE_IMAGE_ID].georef.point_to_pixel_bbox(proj_win);

  BBox2 world_box = app_data.image2world_trans(image_box, BASE_IMAGE_ID);
  for (size_t i = 0; i < m_widgets.size(); i++) {
    if (!m_widgets[i])
      continue;
    m_widgets[i]->zoomToRegion(world_box);
  }
  
}

// Update the checkboxes for the matches menu entries based on stereo_settings()
// values.
void MainWindow::updateViewMenuEntries() {
  auto& wm = m_win_menu_mgr;
  wm.m_viewMatches_action
    ->setChecked(asp::stereo_settings().view_matches);
  wm.m_viewPairwiseCleanMatches_action
    ->setChecked(asp::stereo_settings().pairwise_clean_matches);
  wm.m_viewPairwiseMatches_action
    ->setChecked(asp::stereo_settings().pairwise_matches);
  wm.m_viewSeveralSideBySide_action
    ->setChecked(asp::stereo_settings().view_several_side_by_side);
}

// Update checkboxes for viewing thresholded and hillshaded images
void MainWindow::updateDisplayModeMenuEntries() {
  auto& wm = m_win_menu_mgr;
  wm.m_viewThreshImages_action
    ->setChecked(app_data.display_mode == THRESHOLDED_VIEW);
  wm.m_viewHillshadedImages_action
    ->setChecked(app_data.display_mode == HILLSHADED_VIEW);
}

void MainWindow::viewMatchesFromMenu() {
  // Record user's intent
  asp::stereo_settings().view_matches = m_win_menu_mgr.m_viewMatches_action->isChecked();
  
  toggleViewMatches();
}

// The value of asp::stereo_settings().view_matches must be set before
// calling this. It will be invoked as result of user clicking on the
// menu or adding/deleting match points.
void MainWindow::toggleViewMatches() {
  asp::stereo_settings().pairwise_matches = false;
  asp::stereo_settings().pairwise_clean_matches = false;
  asp::stereo_settings().preview = false;
  MainWindow::updateViewMenuEntries();
  m_show_two_images_when_side_by_side_with_dialog = false;
  m_chooseFiles->showAllImages();
  MainWindow::createLayout(); // This will call viewMatches() after a GUI reorg
}

// Show or hide matches depending on the value of m_viewMatches. We
// assume first ip in first image manages first ip in all other
// images. We allow ip without matches in other images, that can be
// useful, but not before saving, when their numbers must agree for
// all images.
void MainWindow::viewMatches() {
  
  // Record user's intent
  asp::stereo_settings().view_matches = m_win_menu_mgr.m_viewMatches_action->isChecked();
  asp::stereo_settings().preview = false;
  
  // Turn off the other ways of viewing matches
  if (asp::stereo_settings().view_matches) {
    asp::stereo_settings().pairwise_matches = false;
    asp::stereo_settings().pairwise_clean_matches = false;
    asp::stereo_settings().preview = false;
    MainWindow::updateViewMenuEntries();
  }
  
  // If started editing matches do not load them from disk
  if (MainWindow::editingMatches())
    m_match_mgr.m_matches_exist = true;

  // TODO(oalexan1): Improve match loading when done this way, it is
  // rather ad hoc.  Maybe just switch to pairwise matches each time
  // there's more than two images.
  
  // We will load the matches just once, as we later will add/delete
  // matches manually.
  if (!m_match_mgr.m_matches_exist && asp::stereo_settings().view_matches) {

    // We will try to load matches
    m_match_mgr.m_matches_exist = true;
    
    size_t num_images = app_data.image_files.size();
    bool gcp_exists = !stereo_settings().gcp_file.empty() && 
                      fs::exists(stereo_settings().gcp_file);

    if (gcp_exists) {
      
      // Load GCP. If the GCP file was specified but does not exist, we will
      // try to load the matches instead, which later will be used to make the
      // gcp.
      try {
        m_match_mgr.m_matchlist.loadPointsFromGCPs(stereo_settings().gcp_file, app_data.image_files);
      } catch (std::exception const& e) {
        popUp(e.what());
        return;
      }

    } else if (!stereo_settings().vwip_files.empty()) {
      // Try to read matches from vwip files
      m_match_mgr.m_matchlist.loadPointsFromVwip(stereo_settings().vwip_files, app_data.image_files);
    } else {
        
      // If no match file was specified by now, ask the user for the output prefix.
      if (stereo_settings().match_file == "") {
        if (!supplyOutputPrefixIfNeeded(this, m_output_prefix))
          return;
      }
      
      // Load matches
      std::vector<std::string> matchFiles;
      std::vector<size_t> leftIndices;
      bool matchfiles_found = false;
      asp::populateMatchFiles(app_data.image_files, m_output_prefix,
                              stereo_settings().match_file,
                              stereo_settings().matches_as_txt,
                              matchFiles, leftIndices, matchfiles_found);      
      if (matchfiles_found)
        m_match_mgr.m_matchlist.loadPointsFromMatchFiles(matchFiles, leftIndices,
                                                         stereo_settings().matches_as_txt);
    }

  } // End case where we tried to load the matches

  // Set all the matches to be visible.
  for (size_t i = 0; i < m_widgets.size(); i++) {
    if (m_widgets[i])
      m_widgets[i]->viewMatches();
  }
  
  return;
}

void MainWindow::viewPairwiseMatchesSlot() {
  // Record user's intent
  asp::stereo_settings().pairwise_matches
    = m_win_menu_mgr.m_viewPairwiseMatches_action->isChecked();

  if (asp::stereo_settings().pairwise_matches)
    m_show_two_images_when_side_by_side_with_dialog = true;
  
  // Turn off the other ways of viewing matches
  asp::stereo_settings().view_matches = false;
  asp::stereo_settings().pairwise_clean_matches = false;
  MainWindow::updateViewMenuEntries();

  // Must always recreate the layout as this option can totally change the interface
  createLayout();
}

void MainWindow::viewPairwiseCleanMatchesSlot() {
  // Record user's intent
  asp::stereo_settings().pairwise_clean_matches
    = m_win_menu_mgr.m_viewPairwiseCleanMatches_action->isChecked();

  if (asp::stereo_settings().pairwise_clean_matches)
    m_show_two_images_when_side_by_side_with_dialog = true;
  
  // Turn off the other ways of viewing matches
  asp::stereo_settings().view_matches = false;
  asp::stereo_settings().pairwise_matches = false;
  MainWindow::updateViewMenuEntries();

  // Must always recreate the layout as this option can totally change the interface
  createLayout();
}

// These two modes will be handled together as they are very
// similar.
void MainWindow::viewPairwiseMatchesOrCleanMatches() {

  MainWindow::updateViewMenuEntries();

  if (!asp::stereo_settings().pairwise_matches &&
      !asp::stereo_settings().pairwise_clean_matches) {
    return;
  }

  if (app_data.use_georef) {
    popUp("To view matches, turn off viewing the images as georeferenced.");
    asp::stereo_settings().pairwise_matches = false;
    asp::stereo_settings().pairwise_clean_matches = false;
    MainWindow::updateViewMenuEntries();
    createLayout();
    return;
  }
  
  if (asp::stereo_settings().pairwise_matches && 
      asp::stereo_settings().pairwise_clean_matches) {
    popUp("Cannot show both pairwise matches and pairwise clean matches at the same time.");
    asp::stereo_settings().pairwise_matches = false;
    asp::stereo_settings().pairwise_clean_matches = false;
    MainWindow::updateViewMenuEntries();
    createLayout();
    return;
  }

  if (m_output_prefix == "" && 
      stereo_settings().nvm.empty() && 
      stereo_settings().isis_cnet.empty()) {
    popUp("Cannot show pairwise (clean) matches, as the output prefix was not set.");
    asp::stereo_settings().pairwise_matches = false;
    asp::stereo_settings().pairwise_clean_matches = false;
    MainWindow::updateViewMenuEntries();
    createLayout();
    return;
  }

  // See which images are seen
  std::vector<int> seen_indices;
  for (size_t it = 0; it < app_data.images.size(); it++) {
    if (!m_chooseFiles->isHidden(app_data.images[it].name)) {
      seen_indices.push_back(it);
    }
  }
  // Only show matches if precisely two images are currently displayed
  if (seen_indices.size() != 2) {
    // Ensure no stray matches from before are shown
    m_match_mgr.m_pairwiseMatches.ip_to_show.clear();
    m_match_mgr.m_pairwiseCleanMatches.ip_to_show.clear();
    return;
  }
  
  int left_index = seen_indices[0], right_index = seen_indices[1];
  m_match_mgr.loadPairwiseMatches(left_index, right_index, m_output_prefix);
  
  // Call viewMatches() in each widget. That widgets already knows about m_match_mgr.
  // The controls for what kind of matches to show are in asp::stereo_settings().
  for (size_t i = 0; i < m_widgets.size(); i++) {
    if (m_widgets[i])
      m_widgets[i]->viewMatches();
  }
  
}

void MainWindow::saveMatches() {

  if (stereo_settings().match_file == "") {
    if (!supplyOutputPrefixIfNeeded(this, m_output_prefix))
      return;
  }

  try {
    m_match_mgr.m_matchlist.savePointsToDisk(m_output_prefix, app_data.image_files,
                                             stereo_settings().match_file,
                                             stereo_settings().matches_as_txt);
  } catch (std::exception const& e) {
    popUp(e.what());
    return;
  }
    
  m_match_mgr.m_matches_exist = true;

  // If creating GCP, and matches are not saved yet, that means GCP were not saved 
  // either
  if (MainWindow::creatingGcp() && MainWindow::editingMatches())
    m_saved_gcp_and_ip = false;

  // matches got saved, no more editing for now
  for (size_t i = 0; i < m_widgets.size(); i++) {
    if (m_widgets[i] != NULL) 
      m_widgets[i]->setEditingMatches(false);
  }
}

void MainWindow::writeGroundControlPoints() {

  if (!m_match_mgr.m_matchlist.allPointsValid()) {
    popUp("Cannot save GCP, at least one point is missing or not valid.");
    return;
  }
  
  m_match_mgr.m_matches_exist = true;

  const size_t num_images = app_data.image_files.size();
  const size_t num_ips    = m_match_mgr.m_matchlist.getNumPoints();
  // Don't record pixels from the last image, which is used for reference
  const size_t num_images_to_save = num_images - 1; 

  vw_out() << "Saving GCP with " << num_images << " images and " << num_ips << " ips.\n";

  if (num_images != m_match_mgr.m_matchlist.getNumImages())
    return popUp("Cannot save matches. Image and match vectors are unequal!");
  if (num_ips < 1)
    return popUp("Cannot save matches. No matches have been created.");

  // Prompt the user for a DEM path unless specified via --dem-file.
  if (stereo_settings().dem_file == "") {
    try {
      stereo_settings().dem_file = 
        QFileDialog::getOpenFileName(0,
                                     "Select DEM to use for point elevations",
                                     m_output_prefix.c_str()).toStdString();
    } catch(...) {
      return popUp("Error selecting the DEM path.");
    }
  }

  if (stereo_settings().dem_file == "") 
    return; // Likely the user did not choose a file, so cancel the save.
  
  // Have a DEM file, need to save GCP, and did not save it yet
  m_saved_gcp_and_ip = false;
  
  // Prompt the user for the desired output path unless specified in --gcp-file.
  if (stereo_settings().gcp_file == "") {
    try {
      std::string default_path = m_output_prefix + "/ground_control_points.gcp";
      if (m_output_prefix.empty()) // Default to current dir if prefix not set
	      default_path = "ground_control_points.gcp";
      stereo_settings().gcp_file = QFileDialog::getSaveFileName(0,
					       "Select a path to save the GCP file to",
					       default_path.c_str()).toStdString();
    }catch(...) {
      return popUp("Error selecting the output path.");
    }
  }

  // Must save the matches. This is helpful in case
  // want to resume creating GCP.
  MainWindow::saveMatches();

  try {
    asp::genWriteGcp(app_data.image_files, stereo_settings().gcp_file, 
                     stereo_settings().dem_file, m_match_mgr.m_matchlist, 
                     asp::stereo_settings().gcp_sigma);
    m_saved_gcp_and_ip = true;
  } catch (std::exception const& e) {
    popUp(e.what());
    return;
  }
  
}

void MainWindow::addDelMatches() {
  popUp("Right-click on images to add/delete interest point matches.");
  return;
}

void MainWindow::run_stereo_or_parallel_stereo(std::string const& cmd) {

  if (m_widgets.size() < 2) {
    // Note: We allow three images, as the third may be the DEM
    QMessageBox::about(this, tr("Error"),
                       tr("Need to have two images side-by-side to run stereo."));
    return;
  }

  // Output prefix must be non-empty
  if (m_output_prefix.empty()) {
    QMessageBox::about(this, tr("Error"),
                       tr("The output prefix must be passed in, together with the images, "
                          "cameras, and stereo options, to be able to run stereo."));
    return;
  }
  
  // There is no need for a pop-up on failure here, as there will be one
  // in get_crop_win() if the crop windows are not set.
  QRect left_win, right_win;
  if ((!m_widgets[0] || !m_widgets[0]->get_crop_win(left_win)) ||
      (!m_widgets[1] || !m_widgets[1]->get_crop_win(right_win))) 
    return;

  int left_x  = left_win.x();
  int left_y  = left_win.y();
  int left_wx = left_win.width();
  int left_wy = left_win.height();

  int right_x = right_win.x();
  int right_y = right_win.y();
  int right_wx = right_win.width();
  int right_wy = right_win.height();

  std::string run_cmd = vw::program_path(cmd, m_argv[0]);

  // Wipe pre-existing left-image-crop-win and right-image-crop-win
  asp::rmOptionVals(m_argc, m_argv, "--left-image-crop-win", 4);
  asp::rmOptionVals(m_argc, m_argv, "--right-image-crop-win", 4);

  // Wipe the stereo_gui --window-size option and some others that are no
  // use for stereo.
  asp::rmOptionVals(m_argc, m_argv, "--window-size", 2);
  asp::rmOptionVals(m_argc, m_argv, "--font-size", 1);
  asp::rmOptionVals(m_argc, m_argv, "--lowest-resolution-subimage-num-pixels", 1);

  // Form the command to run
  for (int i = 1; i < m_argc; i++) {
    std::string token = std::string(m_argv[i]);
    // Skip adding empty spaces we may have introduced with asp::rmOptionVals().
    if (token == " ")
      continue;
    // Use quotes if there are spaces
    if (token.find(" ") != std::string::npos || token.find("\t") != std::string::npos) 
      token = '"' + token + '"';
    
    run_cmd += " " + token;
  }
  
  // Add crop win options
  std::ostringstream os;
  os << " --left-image-crop-win " << left_x << " " << left_y << " "
     << left_wx << " " << left_wy;
  os << " --right-image-crop-win " << right_x << " " << right_y << " "
     << right_wx << " " << right_wy;
  run_cmd += os.str();

  // Run the command
  vw_out() << "Running: " << run_cmd << std::endl;
  system(run_cmd.c_str());
  QMessageBox::about(this, tr("stereo_gui"), tr("Done running stereo"));
}

void MainWindow::save_screenshot() {
  QMessageBox::about(this, tr("Info"), tr("To save a screenshot, right-click on an image."));
  return;
}

void MainWindow::select_region() {
  QMessageBox::about(this, tr("Info"), tr("Use Control-Left Mouse to select a region. Its bounds will be printed in a terminal. Stereo can be run on selected regions."));
  return;
}

void MainWindow::change_cursor() {
  m_cursor_count = (m_cursor_count + 1) % 3;
  if (m_cursor_count == 0) {
    setCursor(Qt::PointingHandCursor);
  } else if (m_cursor_count == 1) {
    setCursor(Qt::UpArrowCursor);
  }  else{
    setCursor(Qt::ArrowCursor);
  }
}

void MainWindow::run_stereo() {
  MainWindow::run_stereo_or_parallel_stereo("stereo");
}

void MainWindow::run_parallel_stereo() {
  MainWindow::run_stereo_or_parallel_stereo("parallel_stereo");
}

// Toggle on or of the tool for detecting a threshold in images
void MainWindow::thresholdCalc() {
  bool on = m_win_menu_mgr.m_thresholdCalc_action->isChecked();
  for (size_t i = 0; i < m_widgets.size(); i++) {
    if (m_widgets[i])
      m_widgets[i]->setThreshMode(on);
  }
}

void MainWindow::viewThreshImages() {
  if (m_win_menu_mgr.m_viewThreshImages_action->isChecked())
    app_data.display_mode = THRESHOLDED_VIEW;
  else
    app_data.display_mode = REGULAR_VIEW;
  
  MainWindow::updateDisplayModeMenuEntries();

  for (size_t i = 0; i < m_widgets.size(); i++) {
    if (!m_widgets[i])
      continue;
    bool refresh_pixmap = true;
    if (app_data.display_mode == THRESHOLDED_VIEW) 
      m_widgets[i]->viewThreshImages(refresh_pixmap);
    else
      m_widgets[i]->viewUnthreshImages();
  }
}

void MainWindow::contourImages() {
  
  for (size_t i = 0; i < m_widgets.size(); i++) {
    if (!m_widgets[i] ||
        m_widgets[i]->getThreshold() == -std::numeric_limits<double>::max()) {
      popUp("Set the threshold via the Threshold menu before finding the contour.");
      return;
    }
  }
  
  for (size_t i = 0; i < m_widgets.size(); i++) {
    // If we fail at one of the contouring operations, presumably a
    // pop-up will be shown, and then it is not worth continuing,
    // which may just result in more pop-ups.
    if (!m_widgets[i]->contourImage()) 
      return;
  }
}

void MainWindow::saveVectorLayerAsShapeFile() {
  if (m_widgets.size() > 1) {
    popUp("More than one pane exists. Use the right-click menu of the desired pane instead.");
    return;
  }

  if (m_widgets.size() == 1 && m_widgets[0]) 
    m_widgets[0]->saveVectorLayerAsShapeFile();
}

void MainWindow::saveVectorLayerAsTextFile() {
  if (m_widgets.size() > 1) {
    popUp("More than one pane exists. Use the right-click menu of the desired pane instead.");
    return;
  }

  if (m_widgets.size() == 1 && m_widgets[0]) 
    m_widgets[0]->saveVectorLayerAsTextFile();
}

void MainWindow::thresholdGetSet() {

  std::ostringstream oss;
  oss.precision(18);
  
  for (size_t i = 0; i < m_widgets.size(); i++) {
    if (m_widgets[i])
      oss << m_widgets[i]->getThreshold() << " ";
  }
  std::string thresh = oss.str();
  bool ans = getStringFromGui(this,
                              "Image thresholds",
                              "Image thresholds",
                              thresh, thresh);
  if (!ans)
    return;

  std::istringstream iss(thresh.c_str());
  std::vector<double> thresholds;
  double val;
  while (iss >> val)
    thresholds.push_back(val);

  if (thresholds.size() != m_widgets.size()) {
    popUp("There must be as many thresholds as images.");
    return;
  }
  
  for (size_t i = 0; i < m_widgets.size(); i++) {
    if (m_widgets[i])
      m_widgets[i]->setThreshold(thresholds[i]);
  }
  
  // Turn off viewing thresholded images when the threshold is set. The user can
  // turn this back on, and then the thresholded image will be recomputed and
  // displayed.
  app_data.display_mode = REGULAR_VIEW;
  MainWindow::updateDisplayModeMenuEntries();
  MainWindow::viewThreshImages();
}

void MainWindow::setLineWidth() {

  std::ostringstream oss;
  for (size_t i = 0; i < m_widgets.size(); i++) {
    if (m_widgets[i]) {
      oss << m_widgets[i]->getLineWidth();
      // All widgets will have the same line width
      break;
    }
  }
  
  std::string lineWidthStr = oss.str();
  bool ans = getStringFromGui(this,
                              "Polygonal line width",
                              "Polygonal line width",
                              lineWidthStr, lineWidthStr);
  if (!ans)
    return;

  int lineWidth = atoi(lineWidthStr.c_str());

  if (lineWidth <= 0) {
    popUp("The line width must be a positive integer.");
    return;
  }
  
  for (size_t i = 0; i < m_widgets.size(); i++) {
    if (m_widgets[i])
      m_widgets[i]->setLineWidth(lineWidth);
  }
  
}

void MainWindow::setPolyColor() {

  std::string polyColor;
  for (size_t i = 0; i < m_widgets.size(); i++) {
    if (m_widgets[i]) {
      polyColor = m_widgets[i]->getPolyColor();
      // All widgets will have the same poly color
      break;
    }
  }
  
  bool ans = getStringFromGui(this,
                              "Polygonal line color",
                              "Polygonal line color",
                              polyColor, polyColor);
  if (!ans)
    return;

  if (polyColor == "") {
    popUp("The polygonal line color must be set.");
    return;
  }
  
  for (size_t i = 0; i < m_widgets.size(); i++) {
    if (m_widgets[i])
      m_widgets[i]->setPolyColor(polyColor);
  }
  
}

void MainWindow::viewHillshadedImages() {
  bool hillshade = m_win_menu_mgr.m_viewHillshadedImages_action->isChecked();
  app_data.display_mode = hillshade ? HILLSHADED_VIEW : REGULAR_VIEW;
  MainWindow::updateDisplayModeMenuEntries();
  
  for (size_t i = 0; i < m_widgets.size(); i++) {
    if (m_widgets[i])
      m_widgets[i]->viewHillshadedImages(app_data.display_mode == HILLSHADED_VIEW);
  }
}

// Pass to the widget the desire to zoom all images to the same region
// or its cancellation
void MainWindow::setZoomAllToSameRegionAux(bool do_zoom) {
  m_win_menu_mgr.m_zoomAllToSameRegion_action->setChecked(do_zoom);
}

// Pass to the widget the desire to zoom all images to the same region
// or its cancellation
void MainWindow::setZoomAllToSameRegion() {
  
  asp::stereo_settings().zoom_all_to_same_region
    = m_win_menu_mgr.m_zoomAllToSameRegion_action->isChecked();
  setZoomAllToSameRegionAux(asp::stereo_settings().zoom_all_to_same_region);

  if (!asp::stereo_settings().zoom_all_to_same_region)
    return; // nothing to do

  // If zooming to same region, windows better not be on top of each other
  if (m_view_type == VIEW_IN_SINGLE_WINDOW) {
    if (m_view_type_old != VIEW_IN_SINGLE_WINDOW)
      m_view_type = m_view_type_old; // restore this
    else
      m_view_type = VIEW_SIDE_BY_SIDE;
  }

  // If all images are georeferenced, it makes perfect sense to turn
  // on georeferencing when zooming. The user can later turn it off
  // from the menu if so desired.
  bool has_georef = true;
  for (size_t i = 0; i < app_data.image_files.size(); i++) {
    has_georef = has_georef && app_data.images[i].has_georef;
  }

  if (has_georef && !app_data.use_georef) {
    app_data.use_georef = true;
    m_win_menu_mgr.m_viewGeoreferencedImages_action->setChecked(app_data.use_georef);
    viewGeoreferencedImages(); // This will invoke createLayout() too.
  }else{
    createLayout();
  }

}

// Zoom all widgets to the same region which comes from widget with given id.
// There is no need to re-create any layouts now, as nothing changes
// except the fact that we zoom.
void MainWindow::zoomAllToSameRegionAction(int widget_id) {
  int num_widgets = m_widgets.size();
  if (widget_id < 0 || widget_id >= num_widgets) {
    popUp("Invalid widget id.");
    return;
  }

  if (!m_widgets[widget_id]) return;

  vw::BBox2 region = m_widgets[widget_id]->current_view();
  for (size_t i = 0; i < m_widgets.size(); i++) {
    if (m_widgets[i]) {
      m_widgets[i]->zoomToRegion(region);
    }
  }
  
}

// View next or previous image
void MainWindow::viewOtherImage(int delta) {
  
  if (!m_chooseFiles) {
    popUp("The file chooser is not on.");
    return;
  }

  if (m_view_type != VIEW_IN_SINGLE_WINDOW && !previewOrSideBySideWithDialog()) {
    popUp("Viewing the next/prev image requires that all images be in the same window or side-by-side with a dialog.");
    return;
  }

  m_chooseFiles->viewOtherImage(delta);

  if (m_view_type == VIEW_IN_SINGLE_WINDOW && !previewOrSideBySideWithDialog() && 
      m_widgets.size() == 1 && m_widgets[0])
    m_widgets[0]->refreshPixmap();
  else if (previewOrSideBySideWithDialog())
    createLayout();
  
  return;
}

void MainWindow::viewNextImage() {
  MainWindow::viewOtherImage(1); 
}

void MainWindow::viewPrevImage() {
  MainWindow::viewOtherImage(-1); 
}

void MainWindow::uncheckProfileModeCheckbox() {
  m_win_menu_mgr.m_profileMode_action->setChecked(false);
  return;
}

void MainWindow::profileMode() {
  bool profile_mode = m_win_menu_mgr.m_profileMode_action->isChecked();
  if (profile_mode && m_widgets.size() != 1) {
    popUp("A profile can be shown only when a single image is present.");
    uncheckProfileModeCheckbox();
    return;
  }
  
  for (size_t i = 0; i < m_widgets.size(); i++) {
    if (m_widgets[i])
      m_widgets[i]->setProfileMode(profile_mode);
  }
}

void MainWindow::uncheckPolyEditModeCheckbox() {
  m_win_menu_mgr.m_polyEditMode_action->setChecked(false);
  return;
}

void MainWindow::polyEditMode() {
  bool polyEditMode = m_win_menu_mgr.m_polyEditMode_action->isChecked();

  if (polyEditMode) {
    // Turn on vector layer editing
    
    if (!app_data.use_georef) {
      bool has_georef = true;
      for (size_t i = 0; i < app_data.images.size(); i++)
        has_georef = has_georef && app_data.images[i].has_georef;

      if (has_georef) {
        app_data.use_georef = true;
        // If georeference information exists, draw polygons in that mode,
        // and any newly-created polygons will inherit the georeference.
        popUp("To edit polygons, the data will be overlaid in one window using georeferences.");
        app_data.use_georef = true;
        m_win_menu_mgr.m_viewGeoreferencedImages_action->setChecked(app_data.use_georef);
        m_win_menu_mgr.m_overlayGeoreferencedImages_action
          ->setChecked(app_data.use_georef);
        overlayGeoreferencedImages();
        return;
      }
    }
  }
  
  // We arrive here if no GUI overhaul happens. Simply notify the
  // widgets to turn on or off the editing of polygons.
  bool refresh = true;
  for (size_t i = 0; i < m_widgets.size(); i++) {
    if (m_widgets[i])
      m_widgets[i]->setPolyEditMode(polyEditMode, refresh);
  }

  return;
}

void MainWindow::viewGeoreferencedImages() {
  app_data.use_georef = m_win_menu_mgr.m_viewGeoreferencedImages_action->isChecked();
  if (app_data.use_georef) {

    // Will show in single window with georef. Must first check if all images have georef.
    for (size_t i = 0; i < app_data.image_files.size(); i++) {
      if (!app_data.images[i].has_georef) {
        popUp("Cannot view georeferenced images, as there is no georeference in: "
              + app_data.image_files[i]);
        app_data.use_georef = false;
        m_win_menu_mgr.m_viewGeoreferencedImages_action->setChecked(app_data.use_georef);
        m_win_menu_mgr.m_overlayGeoreferencedImages_action
          ->setChecked(app_data.use_georef);
        return;
      }
    }
  }

  createLayout();
}

void MainWindow::overlayGeoreferencedImages() {
  app_data.use_georef = m_win_menu_mgr.m_overlayGeoreferencedImages_action->isChecked();

  if (app_data.use_georef) {

    // Will show in single window with georef. Must first check if all images have georef.
    for (size_t i = 0; i < app_data.image_files.size(); i++) {
      if (!app_data.images[i].has_georef) {
        popUp("Cannot overlay, as there is no georeference in: " + app_data.image_files[i]);
        app_data.use_georef = false;
        m_win_menu_mgr.m_viewGeoreferencedImages_action->setChecked(app_data.use_georef);
        m_win_menu_mgr.m_overlayGeoreferencedImages_action
          ->setChecked(app_data.use_georef);
        return;
      }
    }

    if (m_view_type != VIEW_IN_SINGLE_WINDOW) m_view_type_old = m_view_type; // back this up
    m_view_type = VIEW_IN_SINGLE_WINDOW;

    // Turn off zooming all images to same region if all are in the same window
    if (asp::stereo_settings().zoom_all_to_same_region) {
      asp::stereo_settings().zoom_all_to_same_region = false;
      setZoomAllToSameRegionAux(asp::stereo_settings().zoom_all_to_same_region);
    }
    
  }else{
    if (m_view_type_old != VIEW_IN_SINGLE_WINDOW) m_view_type = m_view_type_old; // restore this
    else m_view_type = VIEW_SIDE_BY_SIDE; 
  }

  createLayout();
}

void MainWindow::about() {
  std::ostringstream about_text;
  about_text << "<h3>stereo_gui</h3>"
             << "<p>NASA Ames Research Center. "
             << "See the manual for documentation.</p>";
  QMessageBox::about(this, tr("About stereo_gui"),
                     tr(about_text.str().c_str()));

}

bool MainWindow::eventFilter(QObject *obj, QEvent *event) {

#if 0
  // Logic which for now does nothing but may be useful one day
  if (event->type() == QEvent::KeyPress) {
    QKeyEvent *key_event = static_cast<QKeyEvent*>(event);
    if (int(key_event->key()) == 67) {
      // do something
    }
  }
#endif
  
  return QMainWindow::eventFilter(obj, event);
}

void MainWindow::keyPressEvent(QKeyEvent *event) {
  // Redirect to some widget. This is a fix for panning not working
  // unless one clicks on the image first.
  for (size_t wit = 0; wit < m_widgets.size(); wit++) {
    if (m_widgets[wit]) {
      m_widgets[wit]->keyPressEvent(event);
      break;
    }
  }
  
}
