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
#include <asp/GUI/MainWindow.h>
#include <asp/GUI/MainWidget.h>
#include <asp/Core/StereoSettings.h>
using namespace asp;
using namespace vw::gui;

#include <vw/config.h>
#include <vw/Image/MaskViews.h>
#include <vw/FileIO/DiskImageView.h>
#include <vw/FileIO/DiskImageResource.h>
#include <vw/Image/Statistics.h>
#include <vw/Image/PixelMask.h>
#include <vw/InterestPoint/InterestData.h>
#include <vw/InterestPoint/Matcher.h>
#include <boost/filesystem/path.hpp>

#include <sstream>
namespace po = boost::program_options;
namespace fs = boost::filesystem;

namespace vw { namespace gui {

  void rm_option_and_vals(int argc, char ** argv, std::string const& opt, int num_vals){
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
}}

MainWindow::MainWindow(asp::BaseOptions const& opt,
                       std::vector<std::string> const& images,
                       std::string& output_prefix,
                       int grid_cols,
                       vw::Vector2i const& window_size,
                       bool single_window,
                       bool use_georef, bool hillshade,
                       bool view_matches,
                       bool delete_temporary_files_on_exit,
                       int argc,  char ** argv) :
  m_opt(opt),
  m_output_prefix(output_prefix), m_widRatio(0.3), m_chooseFiles(NULL),
  m_grid_cols(grid_cols),
  m_use_georef(use_georef), m_hillshade(hillshade), m_viewMatches(view_matches),
  m_delete_temporary_files_on_exit(delete_temporary_files_on_exit),
  m_argc(argc), m_argv(argv) {

  // Window size
  resize(window_size[0], window_size[1]);

  // Window title
  std::string window_title = "Stereo GUI";
  this->setWindowTitle(window_title.c_str());

  // Collect only the valid images
  m_image_paths.clear();
  for (size_t i = 0; i < images.size(); i++) {
    bool is_image = true;
    try {
      DiskImageView<double> img(images[i]);
    }catch(...){
      is_image = false;
    }
    if (!is_image) continue;
    m_image_paths.push_back(images[i]);
  }

  if (m_image_paths.empty()) {
    popUp("No valid images to display.");
    return;
  }

  // If a match file was explicitly specified, use it.
  if (stereo_settings().match_file != ""){
    if (m_image_paths.size() != 2){
      popUp("The --match-file option only works with two valid input images.");
      m_viewMatches = false;
      stereo_settings().match_file = "";
    }else{
      m_viewMatches = true;
      m_match_file = stereo_settings().match_file;
    }
  }

  m_matches_were_loaded = false;
  m_matches.clear();
  m_matches.resize(m_image_paths.size());

  m_view_type = VIEW_SIDE_BY_SIDE;
  if (m_grid_cols > 1) {
    m_view_type = VIEW_AS_TILES_ON_GRID;
  }
  if (single_window) {
    m_view_type = VIEW_IN_SINGLE_WINDOW;
  }
  m_view_type_old = m_view_type;

    // Set up the basic layout of the window and its menus.
  createMenus();

  createLayout();
}


void MainWindow::createLayout() {

  // Create a new central widget. Qt is smart enough to de-allocate
  // the previous widget and all of its children.

  QWidget * centralWidget = new QWidget(this);
  setCentralWidget(centralWidget);

  QSplitter * splitter = new QSplitter(centralWidget);

  // By default, show the images in one row
  if (m_grid_cols <= 0)
    m_grid_cols = std::numeric_limits<int>::max();

  // Wipe the widgets from the array. Qt will automatically delete
  // the widgets when the time is right.
  m_widgets.clear();

  if (m_view_type == VIEW_IN_SINGLE_WINDOW) {

    // Put all images in a single window, with a dialog for choosing images if
    // there's more than one image.

    if (m_image_paths.size() > 1) {
      m_chooseFiles = new chooseFilesDlg(this);
      m_chooseFiles->setMaximumSize(int(m_widRatio*size().width()), size().height());
      splitter->addWidget(m_chooseFiles);
    }

    // Pass all images to a single MainWidget object
    MainWidget * widget = new MainWidget(centralWidget,
                                         m_opt,
                                         0, m_output_prefix,
                                         m_image_paths, m_matches,
                                         m_chooseFiles,
                                         m_use_georef, m_hillshade, m_viewMatches);
    m_widgets.push_back(widget);

  } else{

    // Each MainWidget object gets passed a single image
    for (size_t i = 0; i < m_image_paths.size(); i++) {
      std::vector<std::string> local_images;
      local_images.push_back(m_image_paths[i]);
      m_chooseFiles = NULL;
      MainWidget * widget = new MainWidget(centralWidget,
                                           m_opt,
                                           i, m_output_prefix,
                                           local_images, m_matches,
                                           m_chooseFiles,
                                           m_use_georef, m_hillshade, m_viewMatches);
      m_widgets.push_back(widget);
    }
  }

  // Put the images in a grid
  int num_widgets = m_widgets.size();

  QGridLayout *grid = new QGridLayout(centralWidget);
  for (int i = 0; i < num_widgets; i++) {
    // Add the current widget
    int row = i / m_grid_cols;
    int col = i % m_grid_cols;
    grid->addWidget(m_widgets[i], row, col);

    // Intercept this widget's request to view (or refresh) the matches in all
    // the widgets, not just this one's.
    connect(m_widgets[i], SIGNAL(refreshAllMatches()), this, SLOT(viewExistingMatches()));
  }
  QWidget *container = new QWidget(centralWidget);
  container->setLayout(grid);
  splitter->addWidget(container);

  // Set new layout
  QGridLayout *layout = new QGridLayout(centralWidget);
  layout->addWidget (splitter, 0, 0, 0);
  centralWidget->setLayout(layout);

  // This code must be here in order to load the matches if needed
  viewMatches();

  // Refresh if this menu should be checked
  m_viewOverlayedImages_action->setChecked(m_use_georef && (m_view_type == VIEW_IN_SINGLE_WINDOW));
}

void MainWindow::createMenus() {

  QMenuBar* menu = menuBar();

  // Exit or Quit
  m_exit_action = new QAction(tr("Exit"), this);
  m_exit_action->setShortcut(tr("Q"));
  m_exit_action->setStatusTip(tr("Exit the application"));
  connect(m_exit_action, SIGNAL(triggered()), this, SLOT(forceQuit()));

  // Run stereo
  m_run_stereo_action = new QAction(tr("Run stereo"), this);
  m_run_stereo_action->setStatusTip(tr("Run stereo on selected clips."));
  connect(m_run_stereo_action, SIGNAL(triggered()), this, SLOT(run_stereo()));
  m_run_stereo_action->setShortcut(tr("R"));

  // Run parallel_stereo
  m_run_parallel_stereo_action = new QAction(tr("Run parallel_stereo"), this);
  m_run_parallel_stereo_action->setStatusTip(tr("Run parallel_stereo on selected clips."));
  connect(m_run_parallel_stereo_action, SIGNAL(triggered()), this, SLOT(run_parallel_stereo()));

  // Size to fit
  m_sizeToFit_action = new QAction(tr("Size to fit"), this);
  m_sizeToFit_action->setStatusTip(tr("Change the view to encompass the images."));
  connect(m_sizeToFit_action, SIGNAL(triggered()), this, SLOT(sizeToFit()));
  m_sizeToFit_action->setShortcut(tr("F"));

  m_viewSingleWindow_action = new QAction(tr("Single window"), this);
  m_viewSingleWindow_action->setStatusTip(tr("View images in a single window."));
  connect(m_viewSingleWindow_action, SIGNAL(triggered()), this, SLOT(viewSingleWindow()));

  m_viewSideBySide_action = new QAction(tr("Side-by-side"), this);
  m_viewSideBySide_action->setStatusTip(tr("View images side-by-side."));
  connect(m_viewSideBySide_action, SIGNAL(triggered()), this, SLOT(viewSideBySide()));

  m_viewAsTiles_action = new QAction(tr("As tiles on grid"), this);
  m_viewAsTiles_action->setStatusTip(tr("View images as tiles on grid."));
  connect(m_viewAsTiles_action, SIGNAL(triggered()), this, SLOT(viewAsTiles()));

  // View hillshaded images
  m_viewHillshadedImages_action = new QAction(tr("Hillshaded images"), this);
  m_viewHillshadedImages_action->setStatusTip(tr("View hillshaded images."));
  m_viewHillshadedImages_action->setCheckable(true);
  m_viewHillshadedImages_action->setChecked(m_hillshade);
  connect(m_viewHillshadedImages_action, SIGNAL(triggered()), this, SLOT(viewHillshadedImages()));

  // View overlayed
  m_viewOverlayedImages_action = new QAction(tr("Overlay georeferenced images"), this);
  m_viewOverlayedImages_action->setStatusTip(tr("Overlay georeferenced images."));
  m_viewOverlayedImages_action->setCheckable(true);
  m_viewOverlayedImages_action->setChecked(m_use_georef && (m_view_type == VIEW_IN_SINGLE_WINDOW));
  connect(m_viewOverlayedImages_action, SIGNAL(triggered()), this, SLOT(viewOverlayedImages()));

  m_viewMatches_action = new QAction(tr("View IP matches"), this);
  m_viewMatches_action->setStatusTip(tr("View IP matches."));
  m_viewMatches_action->setCheckable(true);
  m_viewMatches_action->setChecked(m_viewMatches);
  connect(m_viewMatches_action, SIGNAL(triggered()), this, SLOT(viewMatches()));

  m_addDelMatches_action = new QAction(tr("Add/delete IP matches"), this);
  m_addDelMatches_action->setStatusTip(tr("Add/delete interest point matches."));
  connect(m_addDelMatches_action, SIGNAL(triggered()), this, SLOT(addDelMatches()));

  m_saveMatches_action = new QAction(tr("Save IP matches"), this);
  m_saveMatches_action->setStatusTip(tr("Save interest point matches."));
  connect(m_saveMatches_action, SIGNAL(triggered()), this, SLOT(saveMatches()));

  m_writeGcp_action = new QAction(tr("Write GCP file"), this);
  m_writeGcp_action->setStatusTip(tr("Save interest point matches in a GCP format for bundle_adjust."));
  connect(m_writeGcp_action, SIGNAL(triggered()), this, SLOT(writeGroundControlPoints()));

  // Shadow threshold calculation
  m_shadowCalc_action = new QAction(tr("Shadow threshold detection"), this);
  m_shadowCalc_action->setStatusTip(tr("Shadow threshold detection."));
  m_shadowCalc_action->setCheckable(true);
  connect(m_shadowCalc_action, SIGNAL(triggered()), this, SLOT(shadowThresholdCalc()));

  // Shadow threshold visualization
  m_viewThreshImages_action = new QAction(tr("View shadow-thresholded images"), this);
  m_viewThreshImages_action->setStatusTip(tr("View shadow-thresholded images."));
  connect(m_viewThreshImages_action, SIGNAL(triggered()), this, SLOT(viewThreshImages()));

  // Shadow threshold visualization
  m_viewUnthreshImages_action = new QAction(tr("View un-thresholded images"), this);
  m_viewUnthreshImages_action->setStatusTip(tr("View un-thresholded images."));
  connect(m_viewUnthreshImages_action, SIGNAL(triggered()), this, SLOT(viewUnthreshImages()));

  // The About box
  m_about_action = new QAction(tr("About stereo_gui"), this);
  m_about_action->setStatusTip(tr("Show the stereo_gui about box."));
  connect(m_about_action, SIGNAL(triggered()), this, SLOT(about()));

  // File menu
  m_file_menu = menu->addMenu(tr("&File"));
  m_file_menu->addAction(m_exit_action);

  // Run menu
  m_file_menu = menu->addMenu(tr("&Run"));
  m_file_menu->addAction(m_run_stereo_action);
  m_file_menu->addAction(m_run_parallel_stereo_action);

  // View menu
  m_view_menu = menu->addMenu(tr("&View"));
  m_view_menu->addAction(m_sizeToFit_action);
  m_view_menu->addAction(m_viewSingleWindow_action);
  m_view_menu->addAction(m_viewSideBySide_action);
  m_view_menu->addAction(m_viewAsTiles_action);
  m_view_menu->addAction(m_viewHillshadedImages_action);
  m_view_menu->addAction(m_viewOverlayedImages_action);

  // Matches menu
  m_matches_menu = menu->addMenu(tr("&IP matches"));
  m_matches_menu->addAction(m_viewMatches_action);
  m_matches_menu->addAction(m_addDelMatches_action);
  m_matches_menu->addAction(m_saveMatches_action);
  m_matches_menu->addAction(m_writeGcp_action);

  // Threshold menu
  m_threshold_menu = menu->addMenu(tr("&Threshold"));
  m_threshold_menu->addAction(m_shadowCalc_action);
  m_threshold_menu->addAction(m_viewThreshImages_action);
  m_threshold_menu->addAction(m_viewUnthreshImages_action);

  // Help menu
  m_help_menu = menu->addMenu(tr("&Help"));
  m_help_menu->addAction(m_about_action);
}

void MainWindow::resizeEvent(QResizeEvent *){
  if (m_chooseFiles)
    m_chooseFiles->setMaximumSize(int(m_widRatio*size().width()), size().height());
}

void MainWindow::closeEvent(QCloseEvent *){
  forceQuit();
}

void MainWindow::forceQuit(){

  if (m_delete_temporary_files_on_exit) {
    std::set<std::string> & tmp_files = vw::gui::temporary_files().files;
    for (std::set<std::string>::iterator it = tmp_files.begin();
         it != tmp_files.end() ; it++) {
      std::string file = *it;
      if (fs::exists(file)){
        vw_out() << "Deleting: " << file << std::endl;
        fs::remove(file);
      }
    }
  }

  exit(0); // A fix for an older buggy version of Qt
}

void MainWindow::sizeToFit(){
  for (size_t i = 0; i < m_widgets.size(); i++) {
    if (m_widgets[i])
      m_widgets[i]->sizeToFit();
  }
}

void MainWindow::viewSingleWindow(){
  m_grid_cols = std::numeric_limits<int>::max();
  m_view_type = VIEW_IN_SINGLE_WINDOW;
  createLayout();
}

void MainWindow::viewSideBySide(){
  m_grid_cols = std::numeric_limits<int>::max();
  m_view_type = VIEW_SIDE_BY_SIDE;
  createLayout();
}

void MainWindow::viewAsTiles(){

  std::string gridColsStr;
  bool ans = getStringFromGui(this,
                              "Number of columns in the grid",
                              "Number of columns in the grid",
                              "",
                              gridColsStr);
  if (!ans)
    return;

  m_grid_cols = atoi(gridColsStr.c_str());
  m_view_type = VIEW_AS_TILES_ON_GRID;
  createLayout();
}

// This function will be invoked after matches got deleted. Since we had matches,
// we must set m_matches_were_loaded to true.
void MainWindow::viewExistingMatches(){
  m_matches_were_loaded = true;
  MainWindow::viewMatches();
}

// Show or hide matches depending on the value of m_viewMatches.
void MainWindow::viewMatches(){

  m_viewMatches = m_viewMatches_action->isChecked();

  // We will load the matches just once, as we later will add/delete matches manually
  if (!m_matches_were_loaded && (!m_matches.empty()) && m_matches[0].empty() && m_viewMatches) {

    // If no match file was specified by now, ask the user for the output prefix.
    if (m_match_file == "")
      if (!supplyOutputPrefixIfNeeded(this, m_output_prefix)) return;

    m_matches_were_loaded = true;
    m_matches.clear();
    m_matches.resize(m_image_paths.size());

    // All images must have the same number of matches to be able to view them
    int num_matches = -1;

    for (int i = 0; i < int(m_image_paths.size()); i++) {
      for (int j = int(i+1); j < int(m_image_paths.size()); j++) {

        // If the match file was not specified, look it up.
        std::string match_file = m_match_file;
        if (match_file == "")
          match_file = vw::ip::match_filename(m_output_prefix, m_image_paths[i], m_image_paths[j]);

        // Look for the match file in the default location, and if it
        // does not appear prompt the user or a path.
        try {
          ip::read_binary_match_file(match_file, m_matches[i], m_matches[j]);
        }catch(...){
          try {
            match_file = fileDialog("Manually select the match file...", m_output_prefix);

            // If we have just two images, save this match file as the
            // default. For more than two, it gets complicated.
            if (m_image_paths.size() == 2)
              m_match_file = match_file;

          }catch(...){
            popUp("Manually selected file failed to load. Cannot view matches.");
            return;
          }
        }

        vw_out() << "Loading " << match_file << std::endl;
        try {
          ip::read_binary_match_file(match_file, m_matches[i], m_matches[j]);

          if (num_matches < 0)
            num_matches = m_matches[i].size();

          if (num_matches != int(m_matches[i].size()) ||
              num_matches != int(m_matches[j].size()) ) {
            m_matches.clear();
            m_matches.resize(m_image_paths.size());
            popUp(std::string("All image must have the same number of matching ")
                  + "interest point to be able to display them.");
            return;
          }

        }catch(...){
          m_matches.clear();
          m_matches.resize(m_image_paths.size());
          popUp("Could not read matches file: " + match_file);
          return;
        }
      }
    }

    if (m_matches.empty() || m_matches[0].empty()) {
      popUp("Could not load any matches.");
      return;
    }
  }

  for (size_t i = 0; i < m_widgets.size(); i++) {
    if (m_widgets[i]) m_widgets[i]->viewMatches(m_viewMatches);
  }

  if (m_viewMatches)
    m_matches_were_loaded = true;
}

void MainWindow::saveMatches(){

  if (m_match_file == "")
    if (!supplyOutputPrefixIfNeeded(this, m_output_prefix)) return;

  // Sanity checks
  if (m_image_paths.size() != m_matches.size()) {
    popUp("The number of sets of interest points does not agree with the number of images.");
    return;
  }
  for (int i = 0; i < int(m_matches.size()); i++) {
    if (m_matches[0].size() != m_matches[i].size()) {
      popUp("Cannot save matches. Must have the same number of matches in each image.");
      return;
    }
  }

  for (int i = 0; i < int(m_image_paths.size()); i++) {
    for (int j = int(i+1); j < int(m_image_paths.size()); j++) {

      std::string match_file = m_match_file;
      if (match_file == "")
        match_file = vw::ip::match_filename(m_output_prefix, m_image_paths[i], m_image_paths[j]);
      try {
        vw_out() << "Writing: " << match_file << std::endl;
        ip::write_binary_match_file(match_file, m_matches[i], m_matches[j]);
      }catch(...){
        popUp("Failed to save match file: " + match_file);
      }
    }
  }

  m_matches_were_loaded = true;
}


void MainWindow::writeGroundControlPoints() {

  // Make sure the IP matches are ready
  for (size_t i = 0; i < m_matches.size(); i++) {
    if (m_matches[0].size() != m_matches[i].size()) {
      return popUp("Cannot save matches. Must have the same number of matches in each image.");
    }
  }
  const size_t num_ips    = m_matches[0].size();
  const size_t num_images = m_image_paths.size();
  const size_t num_images_to_save = num_images - 1; // Don't record pixels from the last image.
  if (num_images != m_matches.size()) {
    return popUp("Cannot save matches. Image and match vectors are unequal!");
  }

  // Prompt the user for a DEM path
  std::string dem_path = "";
  try {
    dem_path = QFileDialog::getOpenFileName(0,
                                      "Select DEM to use for point elevations",
                                      m_output_prefix.c_str()).toStdString();
  }catch(...){
    return popUp("Error selecting the dem path.");
  }

  // Load a georeference to use for the GCPs from the last image
  cartography::GeoReference georef_image, georef_dem;
  const size_t GEOREF_INDEX = m_image_paths.size() - 1;
  const std::string georef_image_path = m_image_paths[GEOREF_INDEX];
  bool has_georef = vw::cartography::read_georeference(georef_image, georef_image_path);
  if (!has_georef) {
    return popUp("Error: Could not load a valid georeference to use for ground control points in file: "
                 + georef_image_path);
  }
  vw_out() << "Loaded georef from file " << georef_image_path << std::endl;

  // Init the DEM to use for height interpolation
  boost::shared_ptr<DiskImageResource> dem_rsrc(DiskImageResource::open(dem_path));
  DiskImageView<float> dem_disk_image(dem_path);
  vw::ImageViewRef<PixelMask<float> > raw_dem;
  float nodata_val = -std::numeric_limits<float>::max();
  if (dem_rsrc->has_nodata_read()){
    nodata_val = dem_rsrc->nodata_read();
    raw_dem    = create_mask_less_or_equal(dem_disk_image, nodata_val);
  }else{
    raw_dem = pixel_cast<PixelMask<float> >(dem_disk_image);
  }
  PixelMask<float> fill_val;
  fill_val[0] = -99999;
  fill_val.invalidate();
  vw::ImageViewRef<PixelMask<float> > interp_dem = interpolate(raw_dem,
                                                      BilinearInterpolation(),
                                                      ValueEdgeExtension<PixelMask<float> >(fill_val));
  // Load the georef from the DEM
  has_georef = vw::cartography::read_georeference(georef_dem, dem_path);
  if (!has_georef) {
    return popUp("Error: Could not load a valid georeference from dem file: " + dem_path);
  }
  vw_out() << "Loaded georef from dem file " << dem_path << std::endl;

  // Prompt the user for the desired output path
  std::string save_path = "";
  try {
    std::string default_path = m_output_prefix + "/ground_control_points.gcp";
    save_path = QFileDialog::getSaveFileName(0,
                                      "Select a path to save the GCP file to",
                                      default_path.c_str()).toStdString();
  }catch(...){
    return popUp("Error selecting the output path.");
  }
  BBox2 image_bb = bounding_box(interp_dem);

  std::ofstream output_handle(save_path.c_str());
  size_t num_pts_skipped = 0, num_pts_used = 0;
  for (size_t p = 0; p < num_ips; p++) { // Loop through IPs

    // Compute the GDC coordinate of the point
    ip::InterestPoint ip = m_matches[GEOREF_INDEX][p];
    Vector2 lonlat    = georef_image.pixel_to_lonlat(Vector2(ip.x, ip.y));
    Vector2 dem_pixel = georef_dem.lonlat_to_pixel(lonlat);
    PixelMask<float> mask_height = interp_dem(dem_pixel[0], dem_pixel[1])[0];

    // We make a seperate bounding box check because the ValueEdgeExtension
    //  functionality may not work properly!
    if ( (!image_bb.contains(dem_pixel)) || (!is_valid(mask_height)) ) {
      vw_out() << "Warning: Skipped IP # " << p << " because it does not fall on the DEM.\n";
      ++num_pts_skipped;
      continue; // Skip locations which do not fall on the DEM
    }
    vw_out() << "Writing out location: " << mask_height << std::endl;

    // Write the per-point information
    output_handle << num_pts_used; // The ground control point ID
    output_handle << ", " << lonlat[1] << ", " << lonlat[0] << ", " << mask_height[0]; // Lat, lon, height
    output_handle << ", " << 1 << ", " << 1 << ", " << 1; // Sigma values

    // Write the per-image information
    for (size_t i = 0; i < num_images_to_save; i++) {
      // Add this IP to the current line
      ip::InterestPoint ip = m_matches[i][p];
      output_handle << ", " << m_image_paths[i];
      output_handle << ", " << ip.x << ", " << ip.y; // IP location in image
      output_handle << ", " << 1 << ", " << 1; // Sigma values
    } // End loop through IP sets
    output_handle << std::endl; // Finish the line
    ++num_pts_used;
  } // End loop through IPs

  output_handle.close();
  popUp("Finished writing file " + save_path);
}


void MainWindow::addDelMatches(){
  popUp("Right-click on images to add/delete interest point matches.");
  return;
}

void MainWindow::run_stereo_or_parallel_stereo(std::string const& cmd){

  if (m_widgets.size() != 2) {
    QMessageBox::about(this, tr("Error"), tr("Need to have two images side-by-side to run stereo."));
    return;
  }

  QRect left_win, right_win;
  if (!m_widgets[0]->get_crop_win(left_win))
    return;
  if (!m_widgets[1]->get_crop_win(right_win))
    return;

  int left_x = left_win.x();
  int left_y = left_win.y();
  int left_wx = left_win.width();
  int left_wy = left_win.height();

  int right_x = right_win.x();
  int right_y = right_win.y();
  int right_wx = right_win.width();
  int right_wy = right_win.height();

  // Directory in which the tool is located, and replace libexec with
  // bin.
  std::string dir_name = fs::path(m_argv[0]).parent_path().parent_path().string();

  // Command for packaged ASP.
  std::string run_cmd = dir_name + "/bin/" + cmd;

  // Command for dev ASP. The executables are in Tools.
  if (!fs::exists(run_cmd)) {
    run_cmd = dir_name + "/Tools/" + cmd;
    if (!fs::exists(run_cmd)) { // because we have Tools/.libs/stereo
      dir_name = fs::path(m_argv[0]).parent_path().parent_path().parent_path().string();
      run_cmd = dir_name + "/Tools/" + cmd;
    }
  }

  // Wipe pre-existing left-image-crop-win and right-image-crop-win
  rm_option_and_vals(m_argc, m_argv, "--left-image-crop-win", 4);
  rm_option_and_vals(m_argc, m_argv, "--right-image-crop-win", 4);

  // Add the options
  for (int i = 1; i < m_argc; i++) {
    if (std::string(m_argv[i]) != " ") {
      // Skip adding empty spaces we may have introduced with rm_option_and_vals().
      run_cmd += " " + std::string(m_argv[i]);
    }
  }
  std::ostringstream os;
  os << " --left-image-crop-win " << left_x << " " << left_y << " "
     << left_wx << " " << left_wy;
  os << " --right-image-crop-win " << right_x << " " << right_y << " "
     << right_wx << " " << right_wy;
  run_cmd += os.str();
  vw_out() << "Running: " << run_cmd << std::endl;
  system(run_cmd.c_str());
  QMessageBox::about(this, tr("stereo_gui"), tr("Done running stereo"));

}

void MainWindow::run_stereo(){
  MainWindow::run_stereo_or_parallel_stereo("stereo");
}

void MainWindow::run_parallel_stereo(){
  MainWindow::run_stereo_or_parallel_stereo("parallel_stereo");
}

// Toggle on or of the tool for detecting the shadow threshold in images
void MainWindow::shadowThresholdCalc() {
  bool on = m_shadowCalc_action->isChecked();
  for (size_t i = 0; i < m_widgets.size(); i++) {
    if (m_widgets[i])
      m_widgets[i]->setShadowThreshMode(on);
  }

}

void MainWindow::viewThreshImages() {
  for (size_t i = 0; i < m_widgets.size(); i++) {
    if (m_widgets[i]) {
      m_widgets[i]->viewThreshImages();
    }
  }
}

void MainWindow::viewUnthreshImages() {
  for (size_t i = 0; i < m_widgets.size(); i++) {
    if (m_widgets[i]) {
      m_widgets[i]->viewUnthreshImages();
    }
  }
}

void MainWindow::viewHillshadedImages() {
  m_hillshade = m_viewHillshadedImages_action->isChecked();
  for (size_t i = 0; i < m_widgets.size(); i++) {
    if (m_widgets[i]) {
      m_widgets[i]->viewHillshadedImages(m_hillshade);
    }
  }
}

void MainWindow::viewOverlayedImages() {
  m_use_georef = m_viewOverlayedImages_action->isChecked();
  if (m_use_georef) {

    // Will show in single window with georef. Must first check if all images have georef.
    for (size_t i = 0; i < m_image_paths.size(); i++) {
      cartography::GeoReference georef;
      bool has_georef = vw::cartography::read_georeference(georef, m_image_paths[i]);
      if (!has_georef) {
        popUp("Cannot overlay, as there is no georeference in: " + m_image_paths[i]);
        return;
      }
    }

    m_view_type_old = m_view_type; // back this up
    m_view_type = VIEW_IN_SINGLE_WINDOW;
  }else{
    m_view_type = m_view_type_old; // restore this
  }

  createLayout();
}

void MainWindow::about() {
  std::ostringstream about_text;
  about_text << "<h3>stereo_gui</h3>"
             << "<p>Copyright &copy; 2015 NASA Ames Research Center. See the manual for documentation.</p>";
  QMessageBox::about(this, tr("About stereo_gui"),
                     tr(about_text.str().c_str()));

}

void MainWindow::keyPressEvent(QKeyEvent *event) {

  std::ostringstream s;

  switch (event->key()) {
  case Qt::Key_Escape:  // Quit
    close();
    break;
  }
}
