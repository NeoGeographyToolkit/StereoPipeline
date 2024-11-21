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

/// \file MainWindow.cc
///
/// The stereo_gui main window class.
///
#include <QtGui>
#include <QtWidgets>
#include <asp/GUI/MainWindow.h>
#include <asp/GUI/MainWidget.h>
#include <asp/Core/StereoSettings.h>
#include <asp/GUI/chooseFilesDlg.h>
#include <asp/GUI/ColorAxes.h>
#include <asp/Core/GCP.h>
#include <asp/Rig/nvm.h>
#include <asp/Core/IpMatchingAlgs.h>
#include <asp/Camera/BundleAdjustIsis.h>

#include <vw/config.h>
#include <vw/Core/CmdUtils.h>
#include <vw/Image/MaskViews.h>
#include <vw/FileIO/DiskImageView.h>
#include <vw/FileIO/DiskImageResource.h>
#include <vw/Image/Statistics.h>
#include <vw/Image/PixelMask.h>
#include <vw/InterestPoint/InterestData.h>
#include <vw/InterestPoint/Matcher.h>

#include <boost/filesystem/path.hpp>

#include <sstream>

using namespace asp;
using namespace vw::gui;

namespace po = boost::program_options;
namespace fs = boost::filesystem;

// See MainWidget.h for what this id does
int BASE_IMAGE_ID = 0;

namespace asp {

// TODO(oalexan1): Move to utils
void rm_option_and_vals(int argc, char ** argv, std::string const& opt, int num_vals) {
  // Wipe say --left-image-crop-win 0 0 100 100, that is, an option
  // with 4 values.
  for (int i = 0; i < argc; i++) {
    if (std::string(argv[i]) != opt)
      continue;
    
    for (int j = i; j < i + num_vals + 1; j++) {
      if (j >= argc) break;
      // To avoid problems with empty strings, set the string to space instead
      if (strlen(argv[j]) > 0) {
        argv[j][0] = ' ';
        argv[j][1] = '\0';
      }
    }
  }
}

// TODO(oalexan1): Move to utils.
// Given a vector of properties, with each property having a
// potentially non-unique image name as an attribute and other
// attributes as well, and a list of images, find the index of the
// property for that image. Such logic is necessary because the same
// image may show up twice, but with different properties each
// time. So, the first occurrence of an image is matched to the first
// occurrence of a property with that name, and so on. Also, there may
// be properties for entities which are no longer in the list of
// images.
void lookupPropertyIndices(std::vector<std::map<std::string, std::string>> const&
                           properties,
                           std::vector<std::string> const& images,
                           std::vector<int> & propertyIndices) {

  propertyIndices.clear();

  size_t start_p = 0;
  for (size_t i = 0; i < images.size(); i++) {

    for (size_t p_it = start_p; p_it < properties.size(); p_it++) {
      auto key_ptr = properties[p_it].find("name");
      if (key_ptr == properties[p_it].end())
        continue;
      if (key_ptr->second == images[i]) {
        start_p = p_it; // Found the right index for the property for the current file
        break; 
      }
    }

    if (start_p >= properties.size()) {
      if (properties.empty()) 
        vw::vw_throw(vw::ArgumentErr() << "No image properties were found.\n");

      // For the nvm case, to not go out of bounds
      start_p = properties.size() - 1; 
    }
    propertyIndices.push_back(start_p);

    start_p++; // next time start the search after the entry just identified
  }
  
}

} // end namespace asp

// Need this class to manage what happens when keys are pressed while
// the chooseFilesDlg table is in focus. Do not let it accept key
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

// Will return non-NULL if the input pointer is to a MainWidget object.
MainWidget* mw(QWidget * wid) {
  return dynamic_cast<MainWidget*>(wid);
}

// Same function for the ColorAxes widget
ColorAxes* ca(QWidget * wid) {
  return dynamic_cast<ColorAxes*>(wid);
}

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
    if (m_image_files.size() != stereo_settings().vwip_files.size()) {
      popUp("There must be as many .vwip files as images.");
      return false;
    }    
  }
  
  // If a match file was explicitly specified, use it.
  if (stereo_settings().match_file != "") {
    if (m_image_files.size() != 2) {
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
    m_use_georef = false;

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
                       std::vector<std::map<std::string, std::string>> & properties,
                       int argc,  char ** argv):
  m_opt(opt),
  m_output_prefix(output_prefix), m_widRatio(0.3), m_chooseFiles(NULL),
  m_grid_cols(grid_cols),
  m_use_georef(use_georef),
  m_allowMultipleSelections(false), m_matches_exist(false),
  m_argc(argc), m_argv(argv),
  m_show_two_images_when_side_by_side_with_dialog(true), 
  m_cursor_count(0), m_cnet("ASP_control_network"),
  m_saved_gcp_and_ip(true) {

  m_display_mode = asp::stereo_settings().hillshade ? HILLSHADED_VIEW : REGULAR_VIEW;

  if (!stereo_settings().zoom_proj_win.empty())
    m_use_georef = true;
  
  // Window size
  resize(window_size[0], window_size[1]);

  // Window title
  std::string window_title = "Stereo GUI";
  this->setWindowTitle(window_title.c_str());

  // The images and other data
  std::vector<std::string> local_images = images;
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
       rig::readNvmAsCnet(asp::stereo_settings().nvm, std::vector<std::string>(),
                          asp::stereo_settings().nvm_no_shift,
                          m_cnet, world_to_cam, optical_offsets);
    } catch (const std::exception& e) {
      popUp(e.what());
      exit(1);
    }
    if (!local_images.empty())
      popUp("Will ignore the images passed in and will use the nvm file only.");
    local_images = m_cnet.get_image_list(); // overwrite local_images
  }
  if (!asp::stereo_settings().isis_cnet.empty()) {
    asp::stereo_settings().pairwise_matches = true;
    vw::vw_out() << "Reading ISIS control network: " 
                 << asp::stereo_settings().isis_cnet << "\n";
    
    try {
      asp::IsisCnetData isisCnetData; // Part of the API, unused here
      asp::loadIsisCnet(asp::stereo_settings().isis_cnet, local_images,
                        m_cnet, isisCnetData); // outputs
    } catch (const std::exception& e) {
      popUp(e.what());
      exit(1);
    }
  }
  
  // Collect only the valid images
  m_image_files.clear();
  for (size_t i = 0; i < local_images.size(); i++) {
    bool is_image = true;
    try {
      vw::DiskImageView<double> img(local_images[i]);
    } catch(...) {
      is_image = false;
    }
    
    // Accept shape files and csv files alongside images
    if (!is_image &&
        !asp::has_shp_extension(local_images[i]) &&
        !vw::gui::hasCsv(local_images[i]))
      continue;

    m_image_files.push_back(local_images[i]);
  }

  if (m_image_files.empty()) {
    popUp("No input images.");
    exit(1);
  }

  // Set the default lowest resolution subimage size. Use a shorthand below.
  // Must happen before images are loaded.
  int & lowres_size = asp::stereo_settings().lowest_resolution_subimage_num_pixels;
  bool delay = asp::stereo_settings().preview;
  // !asp::stereo_settings().nvm.empty() ||
  // !asp::stereo_settings().isis_cnet.empty() ||
               
  if (lowres_size <= 0) {
    if (delay) {
      // to avoid creating many small subimages. But then the displaying is slow.
      lowres_size = 10000 * 10000; 
    } else {
      lowres_size = 1000 * 1000;
    }
  }
  
  m_images.resize(m_image_files.size());
  std::vector<int> propertyIndices;
  asp::lookupPropertyIndices(properties, m_image_files, propertyIndices);

  // TODO(oalexan1): How will the preview mode play along with georeferences?
  bool has_georef = true;
  for (size_t i = 0; i < m_image_files.size(); i++) {
    m_images[i].read(m_image_files[i], m_opt, REGULAR_VIEW,
                     properties[propertyIndices[i]],
                     delay);
    
    // Above we read the image in regular mode. If plan to display hillshade,
    // for now set the flag for that, and the hillshaded image will be created
    // and set later. (Something more straightforward could be done.)
    m_images[i].m_display_mode = m_display_mode;
    has_georef = has_georef && m_images[i].has_georef;
  }

  // Use georef if all images have it. This may be turned off later if it is desired
  // to show matches.
  if (has_georef)
    m_use_georef = true;

  // It is tricky to set up a layout for georeferenced images if they are loaded
  // one or a few at a time.
  if (delay) 
    m_use_georef = false;
  
  // If the user explicitly asked to not use georef, do not use it on startup
  if (stereo_settings().no_georef) {
    m_use_georef = false; 
    // Further control of georef is from the gui menu
    asp::stereo_settings().no_georef = false; 
  }

  // Ensure the inputs are reasonable
  if (!MainWindow::sanityChecks(m_image_files.size()))
    forceQuit();

  // For being able to choose which files to show/hide
  m_chooseFiles = new chooseFilesDlg(this);
  m_chooseFiles->chooseFiles(m_image_files);
  // See note at chooseFilesFilterDelegate
  m_chooseFiles->getFilesTable()->setItemDelegate(new chooseFilesFilterDelegate(this));
  m_chooseFiles->getFilesTable()->installEventFilter(this);
  
  QObject::connect(m_chooseFiles->getFilesTable(), SIGNAL(cellClicked(int, int)),
                   this, SLOT(perhapsCreateLayout(int, int)));

  QObject::connect(m_chooseFiles->getFilesTable()->horizontalHeader(),
                   SIGNAL(sectionClicked(int)), this, SLOT(hideShowAll_windowVersion()));

  // For editing match points
  m_editMatchPointVecIndex = -1;
  m_matchlist.resize(m_image_files.size());

  // By default, show the images in one row
  if (m_grid_cols <= 0)
    m_grid_cols = std::numeric_limits<int>::max();

  // If no custom view mode, this is the default
  m_view_type = VIEW_SIDE_BY_SIDE;

  // Preview mode normally implies a single window.
  if (asp::stereo_settings().preview && !sideBySideWithDialog()) 
    single_window = true;
  
  if (m_grid_cols > 0 && m_grid_cols < int(m_image_files.size()) &&
      !sideBySideWithDialog())
    m_view_type = VIEW_AS_TILES_ON_GRID;
  
  if (single_window && !sideBySideWithDialog())
    m_view_type = VIEW_IN_SINGLE_WINDOW;

  m_view_type_old = m_view_type; // initialize this
  
  // Set up the basic layout of the window and its menus
  createMenus();

  // Must happen after menus are created
  createLayout();
}

// Create a new central widget. Qt is smart enough to de-allocate
// the previous widget and all of its children.
void MainWindow::createLayout() {

  MainWindow::updateViewMenuEntries();

  // We must cleanup the previous profiles before wiping the existing
  // widgets. There must be a better way of doing it.
  for (size_t wit = 0; wit < m_widgets.size(); wit++) {
    bool profile_mode = false;
    if (mw(m_widgets[wit])) 
      mw(m_widgets[wit])->setProfileMode(profile_mode);
  }

  QWidget * centralWidget = new QWidget(this);
  setCentralWidget(centralWidget);

  QSplitter * splitter = new QSplitter(centralWidget);

  // Wipe the widgets from the array. Qt will automatically delete
  // the widgets when the time is right.
  m_widgets.clear();

  // Note that the menus persist even when the layout changes

  // Show all images if switching to side-by-side view without a dialog
  bool zoom_all_to_same_region = m_zoomAllToSameRegion_action->isChecked();
  if ((zoom_all_to_same_region          ||
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
  for (size_t i = 0; i < m_images.size(); i++) 
    do_colorize = do_colorize || m_images[i].colorbar;
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
     if (m_images.size() > 1) // for one image do this quietly
      popUp("Colorized images can only be shown side-by-side.");
    m_view_type = VIEW_SIDE_BY_SIDE;
  }

  // See if to show it. In a side-by-side view it is normally not needed. 
  bool showChooseFiles
    = ((m_view_type == VIEW_IN_SINGLE_WINDOW || sideBySideWithDialog()) &&
       m_image_files.size() > 1);
  m_chooseFiles->setVisible(showChooseFiles);

  if (m_view_type == VIEW_IN_SINGLE_WINDOW) {
    // Pass all images to a single MainWidget object. No colorizing in this mode.
    int beg_image_id = 0, end_image_id = m_images.size();
    MainWidget * widget = new MainWidget(centralWidget,
                                m_opt,
                                beg_image_id, end_image_id, BASE_IMAGE_ID,
                                m_images, 
                                m_output_prefix,
                                m_matchlist,
                                m_pairwiseMatches, m_pairwiseCleanMatches,
                                m_editMatchPointVecIndex,
                                m_chooseFiles,
                                m_use_georef,
                                zoom_all_to_same_region,
                                m_allowMultipleSelections);
    // Tell the widget if the poly edit mode and hillshade mode is on or not
    bool refresh = false; // Do not refresh prematurely
    widget->setPolyEditMode(m_polyEditMode_action->isChecked(), refresh);
    m_widgets.push_back((QWidget*)widget);
  } else {
    // Each MainWidget object gets passed a single image
    for (size_t i = 0; i < m_images.size(); i++) {

      // Do not create hidden widgets, that really slows down the display when there
      // are many of them, but just a handful are needed.
      bool isHidden = (sideBySideWithDialog() && m_chooseFiles &&
                       m_chooseFiles->isHidden(m_images[i].name));
      if (isHidden) 
        continue;

      QWidget * widget = NULL;
      int beg_image_id = i, end_image_id = i + 1;
      if (!m_images[i].colorbar ||
          previewOrSideBySideWithDialog() || m_images[i].img.planes() > 1) {
        // regular plot
        widget = new MainWidget(centralWidget,
                                m_opt,
                                beg_image_id, end_image_id, BASE_IMAGE_ID, 
                                m_images, 
                                m_output_prefix,
                                m_matchlist, m_pairwiseMatches, m_pairwiseCleanMatches,
                                m_editMatchPointVecIndex,
                                m_chooseFiles,
                                m_use_georef, 
                                zoom_all_to_same_region,
                                m_allowMultipleSelections);
      } else{
        // Qwt plot with axes and colorbar. Hard to use the same API
        // as earlier.
        // TODO(oalexan1): Must integrate the two approaches.
        widget = new ColorAxes(this, 
                               beg_image_id, end_image_id, BASE_IMAGE_ID, 
                               m_use_georef, m_images);
      }
      m_widgets.push_back(widget);
    }
  }

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
    grid->addWidget(m_widgets[i], row, col);
    
    if (!mw(m_widgets[i]))
      continue; // Only MainWidget type can be used below

    // Intercept this widget's request to view (or refresh) the matches in all
    // the widgets, not just this one's.
    connect(mw(m_widgets[i]), SIGNAL(toggleViewMatchesSignal()),
            this, SLOT(toggleViewMatches()));
    connect(mw(m_widgets[i]), SIGNAL(updateMatchesSignal()),
            this, SLOT(viewMatches()));
    connect(mw(m_widgets[i]), SIGNAL(uncheckProfileModeCheckbox()),
            this, SLOT(uncheckProfileModeCheckbox()));
    connect(mw(m_widgets[i]), SIGNAL(uncheckPolyEditModeCheckbox()),
            this, SLOT(uncheckPolyEditModeCheckbox()));
    connect(mw(m_widgets[i]), SIGNAL(zoomAllToSameRegionSignal(int)),
            this, SLOT(zoomAllToSameRegionAction(int)));
    connect(mw(m_widgets[i]), SIGNAL(recreateLayoutSignal()),
            this, SLOT(createLayout()));
  }

  QWidget *container = new QWidget(centralWidget);
  container->setLayout(grid);
  splitter->addWidget(container);

  // Set new layout
  QGridLayout *layout = new QGridLayout(centralWidget);
  layout->addWidget (splitter, 0, 0, 0);
  centralWidget->setLayout(layout);

  if (asp::stereo_settings().view_matches)
    MainWindow::viewMatches();

  if (asp::stereo_settings().pairwise_matches || asp::stereo_settings().pairwise_clean_matches) 
    MainWindow::viewPairwiseMatchesOrCleanMatches();

  double nodata_value = stereo_settings().nodata_value;
  if (!std::isnan(nodata_value)) {
    for (size_t i = 0; i < m_widgets.size(); i++) {
      if (!mw(m_widgets[i]))
        continue;
      mw(m_widgets[i])->setThreshold(nodata_value);
      bool refresh_pixmap = false; // Prepare everything but don't redraw yet
      mw(m_widgets[i])->viewThreshImages(refresh_pixmap);
    }
  }

  // Refresh the menu checkboxes
  m_viewSingleWindow_action->setChecked(m_view_type == VIEW_IN_SINGLE_WINDOW);
  m_viewAllSideBySide_action->setChecked(m_view_type == VIEW_SIDE_BY_SIDE && !asp::stereo_settings().view_several_side_by_side);
  m_viewSeveralSideBySide_action->setChecked(asp::stereo_settings().view_several_side_by_side);
  m_viewAsTiles_action->setChecked(m_view_type == VIEW_AS_TILES_ON_GRID);
  MainWindow::updateDisplayModeMenuEntries();
  m_viewGeoreferencedImages_action->setChecked(m_use_georef);
  m_overlayGeoreferencedImages_action->setChecked(m_use_georef &&
                                                  (m_view_type == VIEW_IN_SINGLE_WINDOW));

  if (m_widgets.size() == 2                             &&
      m_image_files.size() == 2                         &&
      stereo_settings().left_image_crop_win  != BBox2() &&
      stereo_settings().right_image_crop_win != BBox2()) {
    // Draw crop windows passed as arguments
    if (mw(m_widgets[0]))
      mw(m_widgets[0])->setCropWin(stereo_settings().left_image_crop_win);
    if (mw(m_widgets[1]))
      mw(m_widgets[1])->setCropWin(stereo_settings().right_image_crop_win);
    // Do this just once, on startup
    stereo_settings().left_image_crop_win  = BBox2();
    stereo_settings().right_image_crop_win = BBox2();
  }

  return;
}

void MainWindow::createMenus() {

  QMenuBar* menu = menuBar();

  // Exit or Quit
  m_exit_action = new QAction(tr("Exit"), this);
  m_exit_action->setShortcut(tr("Q"));
  m_exit_action->setStatusTip(tr("Exit the application"));
  connect(m_exit_action, SIGNAL(triggered()), this, SLOT(forceQuit()));

  // Save screenshot
  m_save_screenshot_action = new QAction(tr("Save screenshot"), this);
  m_save_screenshot_action->setStatusTip(tr("Save screenshot"));
  connect(m_save_screenshot_action, SIGNAL(triggered()), this, SLOT(save_screenshot()));

  // Select region
  m_select_region_action = new QAction(tr("Select region"), this);
  m_select_region_action->setStatusTip(tr("Select rectangular region"));
  connect(m_select_region_action, SIGNAL(triggered()), this, SLOT(select_region()));
  
  // Change cursor shape (a workaround for Qt not setting the cursor correctly in vnc)
  m_change_cursor_action = new QAction(tr("Change cursor shape"), this);
  m_change_cursor_action->setStatusTip(tr("Change cursor shape"));
  connect(m_change_cursor_action, SIGNAL(triggered()), this, SLOT(change_cursor()));
  m_change_cursor_action->setShortcut(tr("C"));

  // Run parallel_stereo
  m_run_parallel_stereo_action = new QAction(tr("Run parallel_stereo"), this);
  m_run_parallel_stereo_action->setStatusTip(tr("Run parallel_stereo on selected clips"));
  connect(m_run_parallel_stereo_action, SIGNAL(triggered()), this, SLOT(run_parallel_stereo()));
  m_run_parallel_stereo_action->setShortcut(tr("R"));

  // Run stereo
  m_run_stereo_action = new QAction(tr("Run stereo"), this);
  m_run_stereo_action->setStatusTip(tr("Run stereo on selected clips"));
  connect(m_run_stereo_action, SIGNAL(triggered()), this, SLOT(run_stereo()));

  // Zoom to full view
  m_sizeToFit_action = new QAction(tr("Zoom to full view"), this);
  m_sizeToFit_action->setStatusTip(tr("Change the view to encompass the images"));
  connect(m_sizeToFit_action, SIGNAL(triggered()), this, SLOT(sizeToFit()));
  m_sizeToFit_action->setShortcut(tr("F"));

  m_viewSingleWindow_action = new QAction(tr("Single window"), this);
  m_viewSingleWindow_action->setStatusTip(tr("View images in a single window"));
  m_viewSingleWindow_action->setCheckable(true);
  m_viewSingleWindow_action->setChecked(m_view_type == VIEW_IN_SINGLE_WINDOW);
  m_viewSingleWindow_action->setShortcut(tr("W"));
  connect(m_viewSingleWindow_action, SIGNAL(triggered()), this, SLOT(viewSingleWindow()));

  m_viewAllSideBySide_action = new QAction(tr("All side-by-side"), this);
  m_viewAllSideBySide_action->setStatusTip(tr("View all images side-by-side"));
  m_viewAllSideBySide_action->setCheckable(true);
  m_viewAllSideBySide_action->setChecked(m_view_type == VIEW_SIDE_BY_SIDE && !sideBySideWithDialog());
  m_viewAllSideBySide_action->setShortcut(tr("S"));
  connect(m_viewAllSideBySide_action, SIGNAL(triggered()), this, SLOT(viewAllSideBySide()));

  m_viewSeveralSideBySide_action = new QAction(tr("Several side-by-side"), this);
  m_viewSeveralSideBySide_action->setStatusTip(tr("View several images side-by-side"));
  m_viewSeveralSideBySide_action->setCheckable(true);
  m_viewSeveralSideBySide_action->setChecked(asp::stereo_settings().view_several_side_by_side);
  connect(m_viewSeveralSideBySide_action, SIGNAL(triggered()),
          this, SLOT(viewSeveralSideBySide()));

  m_viewAsTiles_action = new QAction(tr("As tiles on grid"), this);
  m_viewAsTiles_action->setStatusTip(tr("View images as tiles on grid"));
  m_viewAsTiles_action->setCheckable(true);
  m_viewAsTiles_action->setChecked(m_view_type == VIEW_AS_TILES_ON_GRID);
  m_viewAsTiles_action->setShortcut(tr("T"));
  connect(m_viewAsTiles_action, SIGNAL(triggered()), this, SLOT(viewAsTiles()));

  // View hillshaded images
  m_viewHillshadedImages_action = new QAction(tr("Hillshaded images"), this);
  m_viewHillshadedImages_action->setStatusTip(tr("View hillshaded images"));
  m_viewHillshadedImages_action->setCheckable(true);
  m_viewHillshadedImages_action->setChecked(m_display_mode == HILLSHADED_VIEW);
  m_viewHillshadedImages_action->setShortcut(tr("H"));
  connect(m_viewHillshadedImages_action, SIGNAL(triggered()),
          this, SLOT(viewHillshadedImages()));

  // View as georeferenced
  m_viewGeoreferencedImages_action = new QAction(tr("View as georeferenced images"), this);
  m_viewGeoreferencedImages_action->setStatusTip(tr("View as georeferenced images"));
  m_viewGeoreferencedImages_action->setCheckable(true);
  m_viewGeoreferencedImages_action->setChecked(m_use_georef);
  m_viewGeoreferencedImages_action->setShortcut(tr("G"));
  connect(m_viewGeoreferencedImages_action, SIGNAL(triggered()),
          this, SLOT(viewGeoreferencedImages()));
  
  // View overlayed georeferenced images
  m_overlayGeoreferencedImages_action = new QAction(tr("Overlay georeferenced images"), this);
  m_overlayGeoreferencedImages_action->setStatusTip(tr("Overlay georeferenced images"));
  m_overlayGeoreferencedImages_action->setCheckable(true);
  m_overlayGeoreferencedImages_action->setChecked(m_use_georef &&
                                                  (m_view_type == VIEW_IN_SINGLE_WINDOW));
  m_overlayGeoreferencedImages_action->setShortcut(tr("O"));
  connect(m_overlayGeoreferencedImages_action, SIGNAL(triggered()),
          this, SLOT(overlayGeoreferencedImages()));

  // Zoom all images to same region
  m_zoomAllToSameRegion_action = new QAction(tr("Zoom all images to same region"), this);
  m_zoomAllToSameRegion_action->setStatusTip(tr("Zoom all images to same region"));
  m_zoomAllToSameRegion_action->setCheckable(true);
  m_zoomAllToSameRegion_action->setChecked(false);
  m_zoomAllToSameRegion_action->setShortcut(tr("Z"));
  connect(m_zoomAllToSameRegion_action, SIGNAL(triggered()),
          this, SLOT(setZoomAllToSameRegion()));

  // View next image
  m_viewNextImage_action = new QAction(tr("View next image"), this);
  m_viewNextImage_action->setStatusTip(tr("View next image"));
  m_viewNextImage_action->setCheckable(false);
  m_viewNextImage_action->setShortcut(tr("N"));
  connect(m_viewNextImage_action, SIGNAL(triggered()),
          this, SLOT(viewNextImage()));

  // View prev image
  m_viewPrevImage_action = new QAction(tr("View previous image"), this);
  m_viewPrevImage_action->setStatusTip(tr("View previous image"));
  m_viewPrevImage_action->setCheckable(false);
  m_viewPrevImage_action->setShortcut(tr("P"));
  connect(m_viewPrevImage_action, SIGNAL(triggered()),
          this, SLOT(viewPrevImage()));

  m_zoomToProjWin_action = new QAction(tr("Zoom to proj win"), this);
  m_zoomToProjWin_action->setStatusTip(tr("Zoom to proj win"));
  m_zoomToProjWin_action->setCheckable(false);
  connect(m_zoomToProjWin_action, SIGNAL(triggered()), this, SLOT(zoomToProjWin()));
  
  // IP matches
  m_viewMatches_action = new QAction(tr("View IP matches"), this);
  m_viewMatches_action->setStatusTip(tr("View IP matches"));
  m_viewMatches_action->setCheckable(true);
  m_viewMatches_action->setChecked(asp::stereo_settings().view_matches);
  connect(m_viewMatches_action, SIGNAL(triggered()), this, SLOT(viewMatchesFromMenu()));

  m_viewPairwiseMatches_action = new QAction(tr("View pairwise IP matches"), this);
  m_viewPairwiseMatches_action->setStatusTip(tr("View pairwise IP matches"));
  m_viewPairwiseMatches_action->setCheckable(true);
  m_viewPairwiseMatches_action->setChecked(asp::stereo_settings().pairwise_matches);
  connect(m_viewPairwiseMatches_action, SIGNAL(triggered()), this, SLOT(viewPairwiseMatchesSlot()));

  m_viewPairwiseCleanMatches_action = new QAction(tr("View pairwise clean IP matches"), this);
  m_viewPairwiseCleanMatches_action->setStatusTip(tr("View pairwise clean IP matches"));
  m_viewPairwiseCleanMatches_action->setCheckable(true);
  m_viewPairwiseCleanMatches_action->setChecked(asp::stereo_settings().pairwise_clean_matches);
  connect(m_viewPairwiseCleanMatches_action, SIGNAL(triggered()),
          this, SLOT(viewPairwiseCleanMatchesSlot()));

  m_addDelMatches_action = new QAction(tr("Add/delete IP matches"), this);
  m_addDelMatches_action->setStatusTip(tr("Add/delete interest point matches"));
  connect(m_addDelMatches_action, SIGNAL(triggered()), this, SLOT(addDelMatches()));

  m_saveMatches_action = new QAction(tr("Save IP matches"), this);
  m_saveMatches_action->setStatusTip(tr("Save interest point matches"));
  connect(m_saveMatches_action, SIGNAL(triggered()), this, SLOT(saveMatches()));

  m_writeGcp_action = new QAction(tr("Save GCP and IP matches"), this);
  m_writeGcp_action->setStatusTip(tr("Save interest point matches as GCP for bundle_adjust"));
  connect(m_writeGcp_action, SIGNAL(triggered()), this, SLOT(writeGroundControlPoints()));

  // Threshold calculation by clicking on pixels and setting the threshold
  // as the largest determined pixel value
  m_thresholdCalc_action = new QAction(tr("Threshold detection"), this);
  m_thresholdCalc_action->setStatusTip(tr("Threshold detection"));
  m_thresholdCalc_action->setCheckable(true);
  connect(m_thresholdCalc_action, SIGNAL(triggered()), this, SLOT(thresholdCalc()));

  // Thresholded image visualization
  m_viewThreshImages_action = new QAction(tr("View thresholded images"), this);
  m_viewThreshImages_action->setStatusTip(tr("View thresholded images"));
  m_viewThreshImages_action->setCheckable(true);
  m_viewThreshImages_action->setChecked(m_display_mode == THRESHOLDED_VIEW);
  connect(m_viewThreshImages_action, SIGNAL(triggered()), this, SLOT(viewThreshImages()));

  // View/set image threshold
  m_thresholdGetSet_action = new QAction(tr("View/set thresholds"), this);
  m_thresholdGetSet_action->setStatusTip(tr("View/set thresholds"));
  connect(m_thresholdGetSet_action, SIGNAL(triggered()), this, SLOT(thresholdGetSet()));

  // 1D profile mode
  m_profileMode_action = new QAction(tr("1D profile mode"), this);
  m_profileMode_action->setStatusTip(tr("Profile mode"));
  m_profileMode_action->setCheckable(true);
  m_profileMode_action->setChecked(false);
  connect(m_profileMode_action, SIGNAL(triggered()), this, SLOT(profileMode()));

  // Polygon edit mode
  m_polyEditMode_action = new QAction(tr("Polygon edit mode"), this);
  m_polyEditMode_action->setStatusTip(tr("Polygon edit mode"));
  m_polyEditMode_action->setCheckable(true);
  m_polyEditMode_action->setChecked(false);
  connect(m_polyEditMode_action, SIGNAL(triggered()), this, SLOT(polyEditMode()));

  // Set line width
  m_setLineWidth_action = new QAction(tr("Set line width"), this);
  m_setLineWidth_action->setStatusTip(tr("Set line width"));
  connect(m_setLineWidth_action, SIGNAL(triggered()), this, SLOT(setLineWidth()));

  // Set color of polygons
  m_setPolyColor_action = new QAction(tr("Set color of polygons"), this);
  m_setPolyColor_action->setStatusTip(tr("Set color of polygons"));
  connect(m_setPolyColor_action, SIGNAL(triggered()), this, SLOT(setPolyColor()));

  // Contour image
  m_contourImages_action = new QAction(tr("Find contour at threshold"), this);
  m_contourImages_action->setStatusTip(tr("Find contour at threshold"));
  connect(m_contourImages_action, SIGNAL(triggered()), this, SLOT(contourImages()));
  
  // Save vector layer as shape file
  m_saveVectorLayerAsShapeFile_action = new QAction(tr("Save vector layer as shapefile"), this);
  m_saveVectorLayerAsShapeFile_action->setStatusTip(tr("Save vector layer as shapefile"));
  connect(m_saveVectorLayerAsShapeFile_action, SIGNAL(triggered()), this,
          SLOT(saveVectorLayerAsShapeFile()));

  // Save vector layer as text file
  m_saveVectorLayerAsTextFile_action = new QAction(tr("Save vector layer as text file"), this);
  m_saveVectorLayerAsTextFile_action->setStatusTip(tr("Save vector layer as text file"));
  connect(m_saveVectorLayerAsTextFile_action, SIGNAL(triggered()), this,
          SLOT(saveVectorLayerAsTextFile()));

  // The About box
  m_about_action = new QAction(tr("About stereo_gui"), this);
  m_about_action->setStatusTip(tr("Show the stereo_gui about box"));
  connect(m_about_action, SIGNAL(triggered()), this, SLOT(about()));

  // File menu
  m_file_menu = menu->addMenu(tr("&File"));
  m_file_menu->addAction(m_save_screenshot_action);
  m_file_menu->addAction(m_select_region_action);
  m_file_menu->addAction(m_change_cursor_action);
  m_file_menu->addAction(m_exit_action);

  // Run menu
  m_file_menu = menu->addMenu(tr("&Run"));
  m_file_menu->addAction(m_run_parallel_stereo_action);
  m_file_menu->addAction(m_run_stereo_action);

  // View menu
  m_view_menu = menu->addMenu(tr("&View"));
  m_view_menu->addAction(m_sizeToFit_action);
  m_view_menu->addAction(m_viewSingleWindow_action);
  m_view_menu->addAction(m_viewAllSideBySide_action);
  m_view_menu->addAction(m_viewSeveralSideBySide_action);
  m_view_menu->addAction(m_viewAsTiles_action);
  m_view_menu->addAction(m_viewHillshadedImages_action);
  m_view_menu->addAction(m_viewGeoreferencedImages_action);
  m_view_menu->addAction(m_overlayGeoreferencedImages_action);
  m_view_menu->addAction(m_zoomAllToSameRegion_action);
  m_view_menu->addAction(m_viewNextImage_action);
  m_view_menu->addAction(m_viewPrevImage_action);
  m_view_menu->addAction(m_zoomToProjWin_action);

  // Matches menu
  m_matches_menu = menu->addMenu(tr("&IP matches"));
  m_matches_menu->addAction(m_viewMatches_action);
  m_matches_menu->addAction(m_viewPairwiseMatches_action);
  m_matches_menu->addAction(m_viewPairwiseCleanMatches_action);
  m_matches_menu->addAction(m_addDelMatches_action);
  m_matches_menu->addAction(m_saveMatches_action);
  m_matches_menu->addAction(m_writeGcp_action);

  // Threshold menu
  m_threshold_menu = menu->addMenu(tr("&Threshold"));
  m_threshold_menu->addAction(m_thresholdCalc_action);
  m_threshold_menu->addAction(m_viewThreshImages_action);
  m_threshold_menu->addAction(m_thresholdGetSet_action);

  // Profile menu
  m_profile_menu = menu->addMenu(tr("Profile"));
  m_profile_menu->addAction(m_profileMode_action);

  // Vector layer menu
  m_vector_layer_menu = menu->addMenu(tr("Vector layer"));
  m_vector_layer_menu->addAction(m_polyEditMode_action);
  m_vector_layer_menu->addAction(m_setLineWidth_action);
  m_vector_layer_menu->addAction(m_setPolyColor_action);
  m_vector_layer_menu->addAction(m_contourImages_action);
  m_vector_layer_menu->addAction(m_saveVectorLayerAsShapeFile_action);
  m_vector_layer_menu->addAction(m_saveVectorLayerAsTextFile_action);

  // Help menu
  m_help_menu = menu->addMenu(tr("&Help"));
  m_help_menu->addAction(m_about_action);
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
    if (mw(m_widgets[i]) != NULL && mw(m_widgets[i])->getEditingMatches()) 
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
    std::set<std::string> & tmp_files = vw::gui::temporary_files().files;
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

  bool zoom_all_to_same_region = m_zoomAllToSameRegion_action->isChecked();
  if (zoom_all_to_same_region) {

    BBox2 big_region; 
    for (size_t i = 0; i < m_widgets.size(); i++) {
      if (!mw(m_widgets[i]))
        continue;

      vw::BBox2 region = mw(m_widgets[i])->worldBox();
      big_region.grow(region);
    }

    for (size_t i = 0; i < m_widgets.size(); i++) {
      if (!mw(m_widgets[i]))
        continue;
      mw(m_widgets[i])->zoomToRegion(big_region);
    }

  }else{
    // Full view for each individual image
    for (size_t i = 0; i < m_widgets.size(); i++) {
      if (mw(m_widgets[i])) // for MainWidget
        mw(m_widgets[i])->sizeToFit();
      else if (ca(m_widgets[i])) // for ColorAxes widget
       ca(m_widgets[i])->sizeToFit();
    }
  }

}

void MainWindow::viewSingleWindow() {

  bool single_window = m_viewSingleWindow_action->isChecked();

  if (single_window) {

    if (m_view_type != VIEW_IN_SINGLE_WINDOW)
      m_view_type_old = m_view_type; // back this up
    
    m_view_type = VIEW_IN_SINGLE_WINDOW;

    // Since we will view all in single window, can't select images with matches
    asp::stereo_settings().view_matches = false;
    setNoSideBySideWithDialog();
    MainWindow::updateViewMenuEntries();

    // Turn off zooming all images to same region if all are in the same window
    bool zoom_all_to_same_region = m_zoomAllToSameRegion_action->isChecked();
    if (zoom_all_to_same_region) {
      zoom_all_to_same_region = false;
      setZoomAllToSameRegionAux(zoom_all_to_same_region);
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

  if (!m_viewAsTiles_action->isChecked()) {
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

  if (!mw(m_widgets[BASE_IMAGE_ID])) {
    popUp("Unexpected missing widget.");
    return;
  }

  if (!m_use_georef) {
    popUp("Turn on viewing georeferenced images to zoom to given proj win.");
    return;
  }
  
  // This takes care of the fact that the order of corners can be reversed
  BBox2 proj_win;
  proj_win.grow(Vector2(a, b));
  proj_win.grow(Vector2(c, d));

  BBox2 image_box;
  if (m_images[BASE_IMAGE_ID].m_isPoly || m_images[BASE_IMAGE_ID].m_isCsv)
    image_box = proj_win;
  else
    image_box = m_images[BASE_IMAGE_ID].georef.point_to_pixel_bbox(proj_win);

  BBox2 world_box = mw(m_widgets[BASE_IMAGE_ID])->image2world(image_box,
                                                              BASE_IMAGE_ID);
  for (size_t i = 0; i < m_widgets.size(); i++) {
    if (!mw(m_widgets[i]))
      continue;
    mw(m_widgets[i])->zoomToRegion(world_box);
  }
  
}

// Update the checkboxes for the matches menu entries based on stereo_settings()
// values.
void MainWindow::updateViewMenuEntries() {
  m_viewMatches_action->setChecked(asp::stereo_settings().view_matches);
  m_viewPairwiseCleanMatches_action->setChecked(asp::stereo_settings().pairwise_clean_matches);
  m_viewPairwiseMatches_action->setChecked(asp::stereo_settings().pairwise_matches);
  m_viewSeveralSideBySide_action->setChecked(asp::stereo_settings().view_several_side_by_side);
}

// Update checkboxes for viewing thresholded and hillshaded images
void MainWindow::updateDisplayModeMenuEntries() {
  m_viewThreshImages_action->setChecked(m_display_mode == THRESHOLDED_VIEW);
  m_viewHillshadedImages_action->setChecked(m_display_mode == HILLSHADED_VIEW);
}

void MainWindow::viewMatchesFromMenu() {
  // Record user's intent
  asp::stereo_settings().view_matches = m_viewMatches_action->isChecked();
  
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
  asp::stereo_settings().view_matches = m_viewMatches_action->isChecked();
  asp::stereo_settings().preview = false;
  
  // Turn off the other ways of viewing matches
  if (asp::stereo_settings().view_matches) {
    asp::stereo_settings().pairwise_matches = false;
    asp::stereo_settings().pairwise_clean_matches = false;
    MainWindow::updateViewMenuEntries();

    if (m_use_georef) {
      popUp("To view matches, turn off viewing the images as georeferenced.");
      asp::stereo_settings().view_matches = false;
      MainWindow::updateViewMenuEntries();
      MainWindow::createLayout();
      return;
    }
  }
  
  // If started editing matches do not load them from disk
  if (MainWindow::editingMatches())
    m_matches_exist = true;

  // TODO(oalexan1): Improve match loading when done this way, it is
  // rather ad hoc.  Maybe just switch to pairwise matches each time
  // there's more than two images.
  
  // We will load the matches just once, as we later will add/delete
  // matches manually.
  if (!m_matches_exist && asp::stereo_settings().view_matches) {

    // We will try to load matches
    m_matches_exist = true;
    
    const size_t num_images = m_image_files.size();
    bool gcp_exists = !stereo_settings().gcp_file.empty() && 
                      fs::exists(stereo_settings().gcp_file);

    if (gcp_exists) {
      
      // Load GCP. If the GCP file was specified but does not exist, we will
      // try to load the matches instead, which later will be used to make the
      // gcp.
      try {
        m_matchlist.loadPointsFromGCPs(stereo_settings().gcp_file, m_image_files);
      } catch (std::exception const& e) {
        popUp(e.what());
        return;
      }

    } else if (!stereo_settings().vwip_files.empty()) {
      // Try to read matches from vwip files
      m_matchlist.loadPointsFromVwip(stereo_settings().vwip_files, m_image_files);
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
      asp::populateMatchFiles(m_image_files, m_output_prefix, stereo_settings().match_file,
                              matchFiles, leftIndices, matchfiles_found);      
      if (matchfiles_found) 
        m_matchlist.loadPointsFromMatchFiles(matchFiles, leftIndices);
    }

  } // End case where we tried to load the matches

  // Set all the matches to be visible.
  for (size_t i = 0; i < m_widgets.size(); i++) {
    if (mw(m_widgets[i]))
      mw(m_widgets[i])->viewMatches();
  }
  
  return;
}

void MainWindow::viewPairwiseMatchesSlot() {
  // Record user's intent
  asp::stereo_settings().pairwise_matches = m_viewPairwiseMatches_action->isChecked();

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
    = m_viewPairwiseCleanMatches_action->isChecked();

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

  if (m_use_georef) {
    popUp("To view matches, turn off viewing the images as georeferenced.");
    asp::stereo_settings().pairwise_matches = false;
    asp::stereo_settings().pairwise_clean_matches = false;
    MainWindow::updateViewMenuEntries();
    createLayout();
    return;
  }
  
  if (asp::stereo_settings().pairwise_matches && asp::stereo_settings().pairwise_clean_matches) {
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
  for (size_t it = 0; it < m_images.size(); it++) {
    if (!m_chooseFiles->isHidden(m_images[it].name)) {
      seen_indices.push_back(it);
    }
  }
  // Only show matches if precisely two images are currently displayed
  if (seen_indices.size() != 2) {
    // Ensure no stray matches from before are shown
    m_pairwiseMatches.ip_to_show.clear();
    m_pairwiseCleanMatches.ip_to_show.clear();
    return;
  }
  
  int left_index = seen_indices[0], right_index = seen_indices[1];
  auto index_pair = std::make_pair(left_index, right_index);
  
  // Get the pointer to the right structure
  pairwiseMatchList * pairwiseMatches = NULL;
  std::string match_file;

  // Read matches or clean matches, unless read by now, for which we check
  // if pairwiseMatches->match_files[index_pair] is initialized.
  // Read either matches from first to second image, or vice versa.
  // First consider the case of loading from nvm.
  if (asp::stereo_settings().pairwise_matches) {
    
    pairwiseMatches = &m_pairwiseMatches;
    if (!asp::stereo_settings().nvm.empty() ||
        !asp::stereo_settings().isis_cnet.empty()) {
      // Load from nvm or isis cnet
      match_file = "placeholder.match";
      pairwiseMatches->match_files[index_pair] = match_file; // flag it as loaded

      // Where we want these loaded 
      auto & left_ip = pairwiseMatches->matches[index_pair].first;   // alias
      auto & right_ip = pairwiseMatches->matches[index_pair].second; // alias

      if (left_ip.empty() && right_ip.empty())
        asp::matchesForPair(m_cnet, left_index, right_index, left_ip, right_ip);
      
    } else if (pairwiseMatches->match_files.find(index_pair) ==
               pairwiseMatches->match_files.end()) {
      // Load pairwise matches
      match_file = vw::ip::match_filename(m_output_prefix, m_images[left_index].name,
                                          m_images[right_index].name);
    }
  } else {
    // Load pairwise clean matches
    pairwiseMatches = &m_pairwiseCleanMatches;
    if (pairwiseMatches->match_files.find(index_pair) == 
        pairwiseMatches->match_files.end()) {
      match_file = vw::ip::clean_match_filename(m_output_prefix, m_images[left_index].name,
                                                m_images[right_index].name);
    }
  }
  
  // Ensure the ip per image are always empty but initialized. This will ensure that
  // later in MainWidget::viewMatches() we plot the intended matches.
  pairwiseMatches->ip_to_show.clear();
  pairwiseMatches->ip_to_show.resize(m_images.size());
  
  // Where we want these loaded
  auto & left_ip = pairwiseMatches->matches[index_pair].first; // alias
  auto & right_ip = pairwiseMatches->matches[index_pair].second; // alias
  
  // If the file was not loaded before, load it. Note that matches from an nvm file
  // are loaded by now.
  if (pairwiseMatches->match_files.find(index_pair) == pairwiseMatches->match_files.end()) {
    // Flag it as loaded
    pairwiseMatches->match_files[index_pair] = match_file;
    try {
      // Load it
      vw_out() << "Loading match file: " << match_file << std::endl;
      ip::read_binary_match_file(match_file, left_ip, right_ip);
    } catch(...) {
      // Having this pop-up for a large number of images is annoying
      vw_out() << "Cannot find the match file with given images and output prefix.\n";
      return;
    }
  }
  
  // These will be read when interest points are drawn
  pairwiseMatches->ip_to_show[left_index] = left_ip;
  pairwiseMatches->ip_to_show[right_index] = right_ip;
  
  // Call viewMatches() in each widget. There things will be sorted out
  // based on stereo_settings().
  for (size_t i = 0; i < m_widgets.size(); i++) {
    if (mw(m_widgets[i]))
      mw(m_widgets[i])->viewMatches();
  }
  
}

void MainWindow::saveMatches() {

  if (stereo_settings().match_file == "") {
    if (!supplyOutputPrefixIfNeeded(this, m_output_prefix))
      return;
  }

  try {
    m_matchlist.savePointsToDisk(m_output_prefix, m_image_files,
                                 stereo_settings().match_file);
  } catch (std::exception const& e) {
    popUp(e.what());
    return;
  }
    
  m_matches_exist = true;

  // If creating GCP, and matches are not saved yet, that means GCP were not saved 
  // either
  if (MainWindow::creatingGcp() && MainWindow::editingMatches())
    m_saved_gcp_and_ip = false;

  // matches got saved, no more editing for now
  for (size_t i = 0; i < m_widgets.size(); i++) {
    if (mw(m_widgets[i]) != NULL) 
      mw(m_widgets[i])->setEditingMatches(false);
  }
}

void MainWindow::writeGroundControlPoints() {

  if (!m_matchlist.allPointsValid()) {
    popUp("Cannot save GCP, at least one point is missing or not valid.");
    return;
  }
  
  m_matches_exist = true;

  const size_t num_images = m_image_files.size();
  const size_t num_ips    = m_matchlist.getNumPoints();
  // Don't record pixels from the last image, which is used for reference
  const size_t num_images_to_save = num_images - 1; 

  vw_out() << "Saving GCP with " << num_images << " images and " << num_ips << " ips.\n";

  if (num_images != m_matchlist.getNumImages())
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
    asp::writeGCP(m_image_files,  
                  stereo_settings().gcp_file,  
                  stereo_settings().dem_file,
                  m_matchlist);
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
  if ((!mw(m_widgets[0]) || !mw(m_widgets[0])->get_crop_win(left_win)) ||
      (!mw(m_widgets[1]) || !mw(m_widgets[1])->get_crop_win(right_win))) 
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
  asp::rm_option_and_vals(m_argc, m_argv, "--left-image-crop-win", 4);
  asp::rm_option_and_vals(m_argc, m_argv, "--right-image-crop-win", 4);

  // Wipe the stereo_gui --window-size option and some others that are no
  // use for stereo.
  asp::rm_option_and_vals(m_argc, m_argv, "--window-size", 2);
  asp::rm_option_and_vals(m_argc, m_argv, "--font-size", 1);
  asp::rm_option_and_vals(m_argc, m_argv, "--lowest-resolution-subimage-num-pixels", 1);

  // Form the command to run
  for (int i = 1; i < m_argc; i++) {
    std::string token = std::string(m_argv[i]);
    // Skip adding empty spaces we may have introduced with asp::rm_option_and_vals().
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
  bool on = m_thresholdCalc_action->isChecked();
  for (size_t i = 0; i < m_widgets.size(); i++) {
    if (mw(m_widgets[i]))
      mw(m_widgets[i])->setThreshMode(on);
  }
}

void MainWindow::viewThreshImages() {
  if (m_viewThreshImages_action->isChecked())
    m_display_mode = THRESHOLDED_VIEW;
  else
    m_display_mode = REGULAR_VIEW;
  
  MainWindow::updateDisplayModeMenuEntries();

  for (size_t i = 0; i < m_widgets.size(); i++) {
    if (!mw(m_widgets[i]))
      continue;
    bool refresh_pixmap = true;
    if (m_display_mode == THRESHOLDED_VIEW) 
      mw(m_widgets[i])->viewThreshImages(refresh_pixmap);
    else
      mw(m_widgets[i])->viewUnthreshImages();
  }
}

void MainWindow::contourImages() {
  
  for (size_t i = 0; i < m_widgets.size(); i++) {
    if (!mw(m_widgets[i]) ||
        mw(m_widgets[i])->getThreshold() == -std::numeric_limits<double>::max()) {
      popUp("Set the threshold via the Threshold menu before finding the contour.");
      return;
    }
  }
  
  for (size_t i = 0; i < m_widgets.size(); i++) {
    // If we fail at one of the contouring operations, presumably a
    // pop-up will be shown, and then it is not worth continuing,
    // which may just result in more pop-ups.
    if (!mw(m_widgets[i])->contourImage()) 
      return;
  }
}

void MainWindow::saveVectorLayerAsShapeFile() {
  if (m_widgets.size() > 1) {
    popUp("More than one pane exists. Use the right-click menu of the desired pane instead.");
    return;
  }

  if (m_widgets.size() == 1 && mw(m_widgets[0])) 
    mw(m_widgets[0])->saveVectorLayerAsShapeFile();
}

void MainWindow::saveVectorLayerAsTextFile() {
  if (m_widgets.size() > 1) {
    popUp("More than one pane exists. Use the right-click menu of the desired pane instead.");
    return;
  }

  if (m_widgets.size() == 1 && mw(m_widgets[0])) 
    mw(m_widgets[0])->saveVectorLayerAsTextFile();
}

void MainWindow::thresholdGetSet() {

  std::ostringstream oss;
  oss.precision(18);
  
  for (size_t i = 0; i < m_widgets.size(); i++) {
    if (mw(m_widgets[i]))
      oss << mw(m_widgets[i])->getThreshold() << " ";
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
    if (mw(m_widgets[i]))
      mw(m_widgets[i])->setThreshold(thresholds[i]);
  }
  
  // Turn off viewing thresholded images when the threshold is set. The user can
  // turn this back on, and then the thresholded image will be recomputed and
  // displayed.
  m_display_mode = REGULAR_VIEW;
  MainWindow::updateDisplayModeMenuEntries();
  MainWindow::viewThreshImages();
}

void MainWindow::setLineWidth() {

  std::ostringstream oss;
  for (size_t i = 0; i < m_widgets.size(); i++) {
    if (mw(m_widgets[i])) {
      oss << mw(m_widgets[i])->getLineWidth();
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
    if (mw(m_widgets[i]))
      mw(m_widgets[i])->setLineWidth(lineWidth);
  }
  
}

void MainWindow::setPolyColor() {

  std::string polyColor;
  for (size_t i = 0; i < m_widgets.size(); i++) {
    if (mw(m_widgets[i])) {
      polyColor = mw(m_widgets[i])->getPolyColor();
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
    if (mw(m_widgets[i]))
      mw(m_widgets[i])->setPolyColor(polyColor);
  }
  
}

void MainWindow::viewHillshadedImages() {
  m_display_mode = m_viewHillshadedImages_action->isChecked() ? HILLSHADED_VIEW : REGULAR_VIEW;
  MainWindow::updateDisplayModeMenuEntries();
  
  for (size_t i = 0; i < m_widgets.size(); i++) {
    if (mw(m_widgets[i]))
      mw(m_widgets[i])->viewHillshadedImages(m_display_mode == HILLSHADED_VIEW);
  }
}

// Pass to the widget the desire to zoom all images to the same region
// or its cancellation
void MainWindow::setZoomAllToSameRegionAux(bool do_zoom) {

  m_zoomAllToSameRegion_action->setChecked(do_zoom);
  for (size_t i = 0; i < m_widgets.size(); i++) {
    if (mw(m_widgets[i]))
      mw(m_widgets[i])->setZoomAllToSameRegion(do_zoom);
  }
}

// Pass to the widget the desire to zoom all images to the same region
// or its cancellation
void MainWindow::setZoomAllToSameRegion() {
  
  bool zoom_all_to_same_region = m_zoomAllToSameRegion_action->isChecked();
  setZoomAllToSameRegionAux(zoom_all_to_same_region);

  if (zoom_all_to_same_region) {

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
    for (size_t i = 0; i < m_image_files.size(); i++) {
      has_georef = has_georef && m_images[i].has_georef;
    }

    if (has_georef && !m_use_georef) {
      m_use_georef = true;
      m_viewGeoreferencedImages_action->setChecked(m_use_georef);
      viewGeoreferencedImages(); // This will invoke createLayout() too.
    }else{
      createLayout();
    }

    // See where the earlier viewable region ended up in the current world
    // coordinate system. A lot of things could have changed since then.
    BBox2 world_box;
    for (size_t i = 0; i < m_widgets.size(); i++) {
      if (mw(m_widgets[i])) {
        vw::BBox2 region = mw(m_widgets[i])->worldBox();
        world_box.grow(region);
      }
    }
    
    // Now let this be the world box in all images. Later, during resizeEvent(),
    // the sizeToFit() function will be called which will use this box.
    for (size_t i = 0; i < m_widgets.size(); i++) {
      if (mw(m_widgets[i]))
        mw(m_widgets[i])->setWorldBox(world_box);
    }
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

  if (!mw(m_widgets[widget_id])) return;

  vw::BBox2 region = mw(m_widgets[widget_id])->current_view();
  for (size_t i = 0; i < m_widgets.size(); i++) {
    if (mw(m_widgets[i])) {
      mw(m_widgets[i])->zoomToRegion(region);
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
      m_widgets.size() == 1 && mw(m_widgets[0]))
    mw(m_widgets[0])->sizeToFit();
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
  m_profileMode_action->setChecked(false);
  return;
}

void MainWindow::profileMode() {
  bool profile_mode = m_profileMode_action->isChecked();
  if (profile_mode && m_widgets.size() != 1) {
    popUp("A profile can be shown only when a single image is present.");
    uncheckProfileModeCheckbox();
    return;
  }
  
  for (size_t i = 0; i < m_widgets.size(); i++) {
    if (mw(m_widgets[i]))
      mw(m_widgets[i])->setProfileMode(profile_mode);
  }
}

void MainWindow::uncheckPolyEditModeCheckbox() {
  m_polyEditMode_action->setChecked(false);
  return;
}

void MainWindow::polyEditMode() {
  bool polyEditMode = m_polyEditMode_action->isChecked();

  if (polyEditMode) {
    // Turn on vector layer editing
    
    if (!m_use_georef) {
      bool has_georef = true;
      for (size_t i = 0; i < m_images.size(); i++)
        has_georef = has_georef && m_images[i].has_georef;

      if (has_georef) {
        m_use_georef = true;
        // If georeference information exists, draw polygons in that mode,
        // and any newly-created polygons will inherit the georeference.
        popUp("To edit polygons, the data will be overlayed in one window using georeferences.");
        m_use_georef = true;
        m_viewGeoreferencedImages_action->setChecked(m_use_georef);
        m_overlayGeoreferencedImages_action->setChecked(m_use_georef);
        overlayGeoreferencedImages();
        return;
      }
    }
  }
  
  // We arrive here if no GUI overhaul happens. Simply notify the
  // widgets to turn on or off the editing of polygons.
  bool refresh = true;
  for (size_t i = 0; i < m_widgets.size(); i++) {
    if (mw(m_widgets[i]))
      mw(m_widgets[i])->setPolyEditMode(polyEditMode, refresh);
  }

  return;
}

void MainWindow::viewGeoreferencedImages() {
  m_use_georef = m_viewGeoreferencedImages_action->isChecked();
  if (m_use_georef) {

    // Will show in single window with georef. Must first check if all images have georef.
    for (size_t i = 0; i < m_image_files.size(); i++) {
      if (!m_images[i].has_georef) {
        popUp("Cannot view georeferenced images, as there is no georeference in: "
              + m_image_files[i]);
        m_use_georef = false;
        m_viewGeoreferencedImages_action->setChecked(m_use_georef);
        m_overlayGeoreferencedImages_action->setChecked(m_use_georef);
        return;
      }
    }
  }

  createLayout();
}

void MainWindow::overlayGeoreferencedImages() {
  m_use_georef = m_overlayGeoreferencedImages_action->isChecked();

  if (m_use_georef) {

    // Will show in single window with georef. Must first check if all images have georef.
    for (size_t i = 0; i < m_image_files.size(); i++) {
      if (!m_images[i].has_georef) {
        popUp("Cannot overlay, as there is no georeference in: " + m_image_files[i]);
        m_use_georef = false;
        m_viewGeoreferencedImages_action->setChecked(m_use_georef);
        m_overlayGeoreferencedImages_action->setChecked(m_use_georef);
        return;
      }
    }

    if (m_view_type != VIEW_IN_SINGLE_WINDOW) m_view_type_old = m_view_type; // back this up
    m_view_type = VIEW_IN_SINGLE_WINDOW;

    // Turn off zooming all images to same region if all are in the same window
    bool zoom_all_to_same_region = m_zoomAllToSameRegion_action->isChecked();
    if (zoom_all_to_same_region) {
      zoom_all_to_same_region = false;
      setZoomAllToSameRegionAux(zoom_all_to_same_region);
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
    if (mw(m_widgets[wit])) {
      mw(m_widgets[wit])->keyPressEvent(event);
      break;
    }
  }
  
}
