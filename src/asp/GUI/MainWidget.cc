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


/// \file MainWidget.cc
///
///
/// TODO: Test with empty images and images having just one pixel.

#include <string>
#include <vector>
#include <QtGui>
#include <QContextMenuEvent>

#include <vw/Image.h>
#include <vw/FileIO.h>
#include <vw/Math/EulerAngles.h>
#include <vw/Image/Algorithms.h>
#include <vw/Core/RunOnce.h>
#include <vw/Cartography/GeoReferenceUtils.h>
#include <vw/Cartography/GeoTransform.h>
#include <asp/GUI/MainWidget.h>

using namespace vw;
using namespace vw::gui;
using namespace vw::cartography;
using namespace std;

namespace vw { namespace gui {

  // --------------------------------------------------------------
  //               MainWidget Public Methods
  // --------------------------------------------------------------

  // Convert a position in the world coordinate system to a pixel
  // position as seen on screen (the screen origin is the
  // visible upper-left corner of the widget).
  vw::Vector2 MainWidget::world2screen(vw::Vector2 const& p) const{

    double x = m_window_width*((p.x() - m_current_view.min().x())
                               /m_current_view.width());
    double y = m_window_height*((p.y() - m_current_view.min().y())
                                /m_current_view.height());

    // Create an empty border margin, to make it easier to zoom
    // by allowing the zoom window to slightly exceed the visible image
    // area (that inability was such a nuisance).
    x = m_border_factor*(x - m_window_width/2.0) + m_window_width/2.0;
    y = m_border_factor*(y - m_window_height/2.0) + m_window_height/2.0;

    return vw::Vector2(x, y);
  }

  // Convert a pixel on the screen to world coordinates.
  // See world2image() for the definition.
  vw::Vector2 MainWidget::screen2world(vw::Vector2 const& p) const{

    // First undo the empty border margin
    double x = p.x(), y = p.y();
    x = (x - m_window_width/2.0)/m_border_factor + m_window_width/2.0;
    y = (y - m_window_height/2.0)/m_border_factor + m_window_height/2.0;

    // Scale to world coordinates
    x = m_current_view.min().x()
      + m_current_view.width() * x / m_window_width;
    y = m_current_view.min().y()
      + m_current_view.height() * y / m_window_height;

    return vw::Vector2(x, y);
  }

  BBox2 MainWidget::screen2world(BBox2 const& R) const{
    if (R.empty()) return R;
    Vector2 A = screen2world(R.min());
    Vector2 B = screen2world(R.max());
    return BBox2(A, B);
  }

  BBox2 MainWidget::world2screen(BBox2 const& R) const {
    if (R.empty()) return R;
    Vector2 A = world2screen(R.min());
    Vector2 B = world2screen(R.max());
    return BBox2(A, B);
  }

  // If we use georef, the world is in projected point units of the
  // first image, with y replaced with -y, to keep the y axis downward,
  // for consistency with how images are plotted.  Convert a world box
  // to a pixel box for the given image.
  Vector2 MainWidget::world2image(Vector2 const& P, int imageIndex) const{
    if (!m_use_georef)
      return P;
    return m_world2image_geotransforms[imageIndex].point_to_pixel(flip_in_y(P));
  }

  BBox2 MainWidget::world2image(BBox2 const& R, int imageIndex) const{

    if (R.empty()) 
      return R;
    if (m_images.empty()) 
      return R;
    if (!m_use_georef)
      return R;

    BBox2 pixel_box = m_world2image_geotransforms[imageIndex].point_to_pixel_bbox(flip_in_y(R));
    return pixel_box;
  }

  // The reverse of world2image()
  BBox2 MainWidget::image2world(BBox2 const& R, int imageIndex) const{

    if (R.empty()) return R;
    if (m_images.empty()) return R;

    if (!m_use_georef)
      return R;

    BBox2 B = flip_in_y(m_image2world_geotransforms[imageIndex].pixel_to_point_bbox(R));
    return B;
  }

  MainWidget::MainWidget(QWidget *parent,
                         vw::cartography::GdalWriteOptions const& opt,
                         int image_id,
                         std::string & output_prefix,
                         std::vector<std::string> const& image_files,
                         std::string const& base_image_file,
                         std::vector<std::vector<vw::ip::InterestPoint> > & matches,
                         chooseFilesDlg * chooseFiles,
                         bool use_georef, bool hillshade, bool view_matches,
                         bool zoom_all_to_same_region, bool & allowMultipleSelections)
    : QWidget(parent), m_opt(opt), m_chooseFilesDlg(chooseFiles),
      m_image_id(image_id), m_output_prefix(output_prefix),
      m_image_files(image_files), m_matches(matches),  m_use_georef(use_georef),
      m_view_matches(view_matches), m_zoom_all_to_same_region(zoom_all_to_same_region),
      m_allowMultipleSelections(allowMultipleSelections), m_can_emit_zoom_all_signal(false){
    
    // Each image can be hillshaded independently of the other ones
    MainWidget::setHillshadeMode(hillshade);
    
    installEventFilter(this);

    m_firstPaintEvent = true;
    m_emptyRubberBand = QRect(0, 0, 0, 0);
    m_rubberBand      = m_emptyRubberBand;
    m_cropWinMode     = false;
    m_profileMode     = false;
    m_profilePlot     = NULL;
    m_world_box       = BBox2();
    
    m_mousePrsX = 0; m_mousePrsY = 0;

    m_border_factor = 0.95;

    // Set some reasonable defaults
    m_bilinear_filter = true;
    m_use_colormap = false;
    m_adjust_mode = NoAdjustment;
    m_display_channel = DisplayRGBA;
    m_colorize_display = false;

    // Set up shader parameters
    m_gain   = 1.0;
    m_offset = 0.0;
    m_gamma  = 1.0;

    // Set mouse tracking
    this->setMouseTracking(true);

    // Set the size policy that the widget can grow or shrink and still
    // be useful.
    this->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
    this->setFocusPolicy(Qt::ClickFocus);

    // Read the images. Find the box that will contain all of them.
    // If we use georef, that box is in projected point units
    // of the first image.
    // Also set up the image GeoReference transforms for each image
    // in both directions.
    int num_images = image_files.size();
    m_images.resize(num_images);
    m_filesOrder.resize(num_images);
    m_world2image_geotransforms.resize(num_images);
    m_image2world_geotransforms.resize(num_images);
    for (int i = 0; i < num_images; i++){
      m_images[i].read(image_files[i], m_opt, m_use_georef);

      // Read the base image, if different from the current image
      if (i == 0){
        if (image_files[i] == base_image_file) {
          m_base_image = m_images[i];
        }else{
          m_base_image.read(base_image_file, m_opt, m_use_georef);
        }
      }
      
      // Make sure we set these up before the image2world call below!
      m_world2image_geotransforms[i] = GeoTransform(m_base_image.georef, m_images[i].georef);
      m_image2world_geotransforms[i] = GeoTransform(m_images[i].georef, m_base_image.georef);
      
      m_filesOrder[i] = i; // start by keeping the order of files being read
      BBox2 B = MainWidget::image2world(m_images[i].image_bbox, i);
      m_world_box.grow(B);
    }

    m_shadow_thresh = -std::numeric_limits<double>::max();
    m_shadow_thresh_calc_mode = false;
    m_shadow_thresh_view_mode = false;

    // To do: Warn the user if some images have georef
    // while others don't.

    // Choose which files to hide/show in the GUI
    if (m_chooseFilesDlg){
      QObject::connect(m_chooseFilesDlg->getFilesTable(),
                       SIGNAL(cellClicked(int, int)),
                       this,
                       SLOT(showFilesChosenByUser(int, int)));
      m_chooseFilesDlg->chooseFiles(m_images);

      // When the user clicks on the table header on top to toggle all on/off
      connect(m_chooseFilesDlg->getFilesTable()->horizontalHeader(),
	      SIGNAL(sectionClicked(int)), this, SLOT(toggleAllOnOff()));
      
      m_chooseFilesDlg->getFilesTable()->setContextMenuPolicy(Qt::CustomContextMenu);
      connect(m_chooseFilesDlg->getFilesTable(), SIGNAL(customContextMenuRequested(QPoint)),
	      this, SLOT(customMenuRequested(QPoint)));
    }

    // Right-click context menu
    m_ContextMenu = new QMenu();
    //setContextMenuPolicy(Qt::CustomContextMenu);
    m_addMatchPoint    = m_ContextMenu->addAction("Add match point");
    m_deleteMatchPoint = m_ContextMenu->addAction("Delete match point");
    m_toggleHillshade  = m_ContextMenu->addAction("Toggle hillshaded display");
    m_saveScreenshot   = m_ContextMenu->addAction("Save screenshot");
    m_setThreshold     = m_ContextMenu->addAction("View/set shadow threshold");
    m_allowMultipleSelections_action = m_ContextMenu->addAction("Allow multiple selected regions");
    m_allowMultipleSelections_action->setCheckable(true);
    m_allowMultipleSelections_action->setChecked(m_allowMultipleSelections);
    
    connect(m_addMatchPoint,    SIGNAL(triggered()), this, SLOT(addMatchPoint()));
    connect(m_deleteMatchPoint, SIGNAL(triggered()), this, SLOT(deleteMatchPoint()));
    connect(m_toggleHillshade,  SIGNAL(triggered()), this, SLOT(toggleHillshade()));
    connect(m_setThreshold,     SIGNAL(triggered()), this, SLOT(setThreshold()));
    connect(m_saveScreenshot,   SIGNAL(triggered()), this, SLOT(saveScreenshot()));
    connect(m_allowMultipleSelections_action, SIGNAL(triggered()), this,
            SLOT(allowMultipleSelections()));

    MainWidget::maybeGenHillshade();

  } // End constructor


  MainWidget::~MainWidget() {
  }

  bool MainWidget::eventFilter(QObject *obj, QEvent *E){
    return QWidget::eventFilter(obj, E);
  }

  // What will happen when the user right-clicks on the table
  // listing the files.
  void MainWidget::customMenuRequested(QPoint pos){

    // Process user's choice from m_chooseFilesDlg.
    if (!m_chooseFilesDlg)
      return;

    QTableWidget * filesTable = m_chooseFilesDlg->getFilesTable();

    QModelIndex tablePos=filesTable->indexAt(pos);
    int imageIndex = tablePos.row();

    m_indicesWithAction.clear();
    m_indicesWithAction.insert(imageIndex);
    
    QMenu *menu=new QMenu(this);

    m_toggleHillshadeFromTable = menu->addAction("Toggle hillshade display");
    connect(m_toggleHillshadeFromTable, SIGNAL(triggered()), this, SLOT(refreshHillshade()));
    
    m_zoomToImageFromTable = menu->addAction("Zoom to image");
    connect(m_zoomToImageFromTable, SIGNAL(triggered()), this, SLOT(zoomToImage()));

    m_deleteImage = menu->addAction("Delete image");
    connect(m_deleteImage, SIGNAL(triggered()), this, SLOT(deleteImage()));

    menu->exec(filesTable->mapToGlobal(pos));
  }
  
  void MainWidget::showFilesChosenByUser(int rowClicked, int columnClicked){

    // Process user's choice from m_chooseFilesDlg.
    if (!m_chooseFilesDlg)
      return;

    m_filesToHide.clear();
    QTableWidget * filesTable = m_chooseFilesDlg->getFilesTable();
    int rows = filesTable->rowCount();

    // Make list of all the unchecked files
    for (int rowIter = 0; rowIter < rows; rowIter++){
      QTableWidgetItem *item = filesTable->item(rowIter, 0);
      if (item->checkState() != Qt::Checked){
        string fileName = (filesTable->item(rowIter, 1)->data(0)).toString().toStdString();
        m_filesToHide.insert(fileName);
      }
    }

    // If we just checked a certain image, it will be shown on top of the other ones.
    QTableWidgetItem *item = filesTable->item(rowClicked, 0);
    if (item->checkState() == Qt::Checked){
      putImageOnTop(rowClicked);
    }

    refreshPixmap();

    return;
  }

  void MainWidget::toggleAllOnOff(){
    
    // Process user's choice from m_chooseFilesDlg.
    if (!m_chooseFilesDlg)
      return;

    QTableWidget * filesTable = m_chooseFilesDlg->getFilesTable();
    int rows = filesTable->rowCount();

    // See if all files are hidden
    bool allOff = true;
    for (int rowIter = 0; rowIter < rows; rowIter++){
      QTableWidgetItem *item = filesTable->item(rowIter, 0);
      if (item->checkState() == Qt::Checked){
	allOff = false;
      }
    }

    // If all files are hidden, we will show all. Else hide all.
    m_filesToHide.clear();
    for (int rowIter = 0; rowIter < rows; rowIter++){
      QTableWidgetItem *item = filesTable->item(rowIter, 0);
      string fileName = (filesTable->item(rowIter, 1)->data(0)).toString().toStdString();

      if (allOff){
	item->setCheckState(Qt::Checked);
      }else{
	item->setCheckState(Qt::Unchecked);
	m_filesToHide.insert(fileName);
      }
    }

    if (!allOff) {
      // Now all files are hidden, per above. So reset the order, so that when
      // we show them, they are in the original order.
      int num_images = m_images.size();
      m_filesOrder.resize(num_images);
      for (int i = 0; i < num_images; i++)
        m_filesOrder[i] = i;
    }

    refreshPixmap();
  }
    
  BBox2 MainWidget::expand_box_to_keep_aspect_ratio(BBox2 const& box) {
    
    BBox2 in_box = box;
    if (in_box.empty()) in_box = BBox2(0, 0, 1, 1); // if it came to worst

    BBox2 out_box = in_box;
    double aspect = double(m_window_width) / m_window_height;
    if (in_box.width() / in_box.height() < aspect) {
      // Width needs to grow
      double new_width = in_box.height() * aspect;
      double delta = (new_width - in_box.width())/2.0;
      out_box.min().x() -= delta; out_box.max().x() += delta;
    }else if (in_box.width() / in_box.height() > aspect) {
      // Height needs to grow
      double new_height = in_box.width() / aspect;
      double delta = (new_height - in_box.height())/2.0;
      out_box.min().y() -= delta; out_box.max().y() += delta;
    }
    return out_box;
  }

  // Over-ride the world box. Useful when zooming all images to same region.
  void MainWidget::setWorldBox(BBox2 const& box){
    m_world_box = box;
  }
  
  // Zoom to show each image fully, unless one messed up with
  // m_world_box, as it happens when zooming all images to same region.
  void MainWidget::sizeToFit() {

    m_current_view = expand_box_to_keep_aspect_ratio(m_world_box);
    
    // If this is the first time we draw the image, so right when
    // we started, invoke update() which will invoke paintEvent().
    // That one will not only call refreshPixmap() but will
    // also mark that it did so. This is a bit confusing, but it is
    // necessary since otherwise Qt will first call this function,
    // invoking refreshPixmap(), then will call update() one more time
    // invoking needlessly refreshPixmap() again, which is expensive.
    if (m_firstPaintEvent){
      update();
    }else {
      refreshPixmap();
    }
  }

  void MainWidget::viewUnthreshImages(){
    m_shadow_thresh_view_mode = false;
    MainWidget::setHillshadeMode(false);
    refreshPixmap();
  }

  // The region that is currently viewable, in the first image pixel domain
  BBox2 MainWidget::firstImagePixelBox() const{
    
    if (m_images.size() == 0) {
      // Must never happen
      vw_out() << "Did not expect no images!";
      vw_throw(ArgumentErr() << "Did not expect no images.\n");
    }
    return MainWidget::world2image(m_current_view, 0);
  }

  // The current image box in world coordinates
  BBox2 MainWidget::firstImageWorldBox(BBox2 const& image_box) const{
    if (m_images.size() == 0) {
      // Must never happen
      vw_out() << "Did not expect no images!";
      vw_throw(ArgumentErr() << "Did not expect no images.\n");
    }
    return MainWidget::image2world(image_box, 0);
  }
  
  void MainWidget::viewThreshImages(){
    m_shadow_thresh_view_mode = true;
    MainWidget::setHillshadeMode(false);

    if (m_images.size() != 1) {
      popUp("Must have just one image in each window to be able to view thresholded images.");
      m_shadow_thresh_view_mode = false;

      refreshPixmap();
      return;
    }

    int num_images = m_images.size();
    m_shadow_thresh_images.clear(); // wipe the old copy
    m_shadow_thresh_images.resize(num_images);

    // Create the thresholded images and save them to disk. We have to do it each
    // time as perhaps the shadow threshold changed.
    for (int image_iter = 0; image_iter < num_images; image_iter++) {
      std::string input_file = m_image_files[image_iter];

      double nodata_val = -std::numeric_limits<double>::max();
      vw::read_nodata_val(input_file, nodata_val);
      nodata_val = std::max(nodata_val, m_shadow_thresh);

      int num_channels = m_images[image_iter].img.planes();
      if (num_channels != 1) {
        popUp("Thresholding makes sense only for single-channel images.");
        m_shadow_thresh_view_mode = false;
        return;
      }

      ImageViewRef<double> thresh_image
        = apply_mask(create_mask_less_or_equal(DiskImageView<double>(input_file),
                                               nodata_val), nodata_val);

      std::string suffix = "_thresh.tif";
      bool has_georef = false;
      bool has_nodata = true;
      vw::cartography::GeoReference georef;
      std::string output_file
        = write_in_orig_or_curr_dir(m_opt,
                                    thresh_image, input_file, suffix,
                                    has_georef,  georef,
                                    has_nodata, nodata_val);

      // Read it back right away
      m_shadow_thresh_images[image_iter].read(output_file, m_opt, m_use_georef);
      temporary_files().files.insert(output_file);
    }

    refreshPixmap();
  }

  void MainWidget::maybeGenHillshade(){

    int num_images = m_images.size();
    m_hillshaded_images.clear(); // wipe the old copy
    m_hillshaded_images.resize(num_images);

    // Create the hillshaded images and save them to disk. We have to do
    // it each time as perhaps the hillshade parameters changed.
    for (int image_iter = 0; image_iter < num_images; image_iter++) {

      if (!m_hillshade_mode[image_iter]) continue;

      if (!m_images[image_iter].has_georef) {
        popUp("Hill-shading requires georeferenced images.");
        m_hillshade_mode[image_iter] = false;
        return;
      }

      std::string input_file = m_image_files[image_iter];
      int num_channels = m_images[image_iter].img.planes();
      if (num_channels != 1) {
        popUp("Hill-shading makes sense only for single-channel images.");
        m_hillshade_mode[image_iter] = false;
        return;
      }

      // Save the hillshaded images to disk
      std::string hillshaded_file;
      bool success = write_hillshade(m_opt, input_file, hillshaded_file);
      if (!success) {
        m_hillshade_mode[image_iter] = false;
        return;
      }

      m_hillshaded_images[image_iter].read(hillshaded_file, m_opt, m_use_georef);
      temporary_files().files.insert(hillshaded_file);
    }
  }

  // Delete an image from the list
  void MainWidget::deleteImage(){
    emit removeImageAndRefreshSignal();
  }

  // Allow the user to select multiple windows. 
  void MainWidget::allowMultipleSelections(){
    m_allowMultipleSelections = !m_allowMultipleSelections;
    m_allowMultipleSelections_action->setChecked(m_allowMultipleSelections);
    if (!m_allowMultipleSelections) {
      m_selectionRectangles.clear();
      refreshPixmap();
    }
  }

  void MainWidget::refreshHillshade(){

    for (std::set<int>::iterator it = m_indicesWithAction.begin();
	 it != m_indicesWithAction.end(); it++) {
      m_hillshade_mode[*it] = !m_hillshade_mode[*it];

      // We will assume if the user wants to see the hillshade
      // status of this image change, he'll also want it on top.
      putImageOnTop(*it); 
    }

    m_shadow_thresh_calc_mode = false;
    m_shadow_thresh_view_mode = false;
    MainWidget::maybeGenHillshade();

    m_indicesWithAction.clear();
    refreshPixmap();
  }

  void MainWidget::zoomToImage(){

    for (std::set<int>::iterator it = m_indicesWithAction.begin();
	 it != m_indicesWithAction.end(); it++) {

      // We will assume if the user wants to zoom to this image,
      // it should be on top.
      putImageOnTop(*it); 

      m_current_view = expand_box_to_keep_aspect_ratio
        (MainWidget::image2world(m_images[*it].image_bbox, *it));
    }
    
    m_indicesWithAction.clear();

    refreshPixmap();
  }

  void MainWidget::viewHillshadedImages(bool hillshade_mode){
    MainWidget::setHillshadeMode(hillshade_mode);
    refreshHillshade();
  }

  void MainWidget::toggleHillshade(){
    m_hillshade_mode.resize(m_image_files.size());
    for (size_t image_iter = 0; image_iter < m_hillshade_mode.size(); image_iter++) 
      m_hillshade_mode[image_iter] = !m_hillshade_mode[image_iter];

    refreshHillshade();
  }

  bool MainWidget::hillshadeMode() const {
    if (m_hillshade_mode.empty()) return false;

    // If we have to return just one value, one image not being
    // hillshaded will imply that the value is false
    for (size_t it = 0; it < m_hillshade_mode.size(); it++) {
      if (!m_hillshade_mode[it]) return false;
    }
    return true;
  }

  // Each image can be hillshaded independently of the others
  void MainWidget::setHillshadeMode(bool hillshade_mode){
    m_hillshade_mode.resize(m_image_files.size());
    for (size_t image_iter = 0; image_iter < m_hillshade_mode.size(); image_iter++) 
      m_hillshade_mode[image_iter] = hillshade_mode;
  }
  
  // The image with the given index will be on top when shown.
  void MainWidget::putImageOnTop(int image_index){
    std::vector<int>::iterator it = std::find(m_filesOrder.begin(), m_filesOrder.end(), image_index);
    if (it != m_filesOrder.end()){
      m_filesOrder.erase(it);
      m_filesOrder.push_back(image_index); // show last, so on top
    }

    // An image on top better be viewable
    string fileName = m_images[image_index].name;
    std::set<std::string>::iterator it2 = m_filesToHide.find(fileName);
    if (it2 != m_filesToHide.end()){
      m_filesToHide.erase(it2);

      // Then turn on the checkbox in the table
      QTableWidget * filesTable = m_chooseFilesDlg->getFilesTable();
      int rows = filesTable->rowCount();
      
      for (int rowIter = 0; rowIter < rows; rowIter++){
        QTableWidgetItem *item = filesTable->item(rowIter, 0);
        string fileName2 = (filesTable->item(rowIter, 1)->data(0)).toString().toStdString();
        if (fileName == fileName2) {
          item->setCheckState(Qt::Checked);
        }
      }
    }
    
  }
    
  // Convert the crop window to original pixel coordinates from
  // pixel coordinates on the screen.
  // TODO: Make screen2world() do it, to take an input a QRect (or BBox2)
  // and return as output the converted box.
  bool MainWidget::get_crop_win(QRect & win) {

    if (m_images.size() != 1) {
      popUp("Must have just one image in each window to be able to select regions for stereo.");
      m_cropWinMode = false;
      m_rubberBand = m_emptyRubberBand;
      m_stereoCropWin = BBox2();
      refreshPixmap();
      return false;
    }

    if (m_stereoCropWin.empty()) {
      popUp("No valid region for stereo is present.");
      return false;
    }

    win = bbox2qrect(world2image(m_stereoCropWin, 0));
    return true;
  }

  void MainWidget::zoom(double scale) {

    updateCurrentMousePosition();
    scale = std::max(1e-8, scale);
    BBox2 current_view = (m_current_view - m_curr_world_pos) / scale
      + m_curr_world_pos;

    if (!current_view.empty()){
      // Check to make sure we haven't hit our zoom limits...
      m_current_view = current_view;
      m_can_emit_zoom_all_signal = true;
      refreshPixmap();
    }
  }

  void MainWidget::resizeEvent(QResizeEvent*){
    QRect v       = this->geometry();
    m_window_width  = std::max(v.width(), 1);
    m_window_height = std::max(v.height(), 1);
    sizeToFit();
    return;
  }

  // --------------------------------------------------------------
  //             MainWidget Private Methods
  // --------------------------------------------------------------

  void MainWidget::drawImage(QPainter* paint) {

    // Sometimes we arrive here prematurely, before the window geometry was
    // determined. Then, there is nothing to do.
    if (m_current_view.empty()) return;

    std::list<BBox2i> screen_box_list; // List of regions the images are drawn in
    // Loop through input images
    // - These images get drawn in the same
    for (int j = 0; j < (int)m_images.size(); j++){

      int i = m_filesOrder[j];

      // Don't show files the user wants hidden
      string fileName = m_images[i].name;
      if (m_filesToHide.find(fileName) != m_filesToHide.end()) continue;

      // The portion of the image in the current view. 
      BBox2 world_box = m_current_view; 
      BBox2 B = MainWidget::image2world(m_images[i].image_bbox, i);
      world_box.crop(B);

      // This is a bugfix for the case when the world boxes 
      // of images do not overlap.
      if (world_box.empty()) continue;

      // See where it fits on the screen
      BBox2i screen_box;
      screen_box.grow(floor(world2screen(world_box.min())));
      screen_box.grow(ceil(world2screen(world_box.max())));

      // Ensure the screen box is never empty
      if (screen_box.min().x() >= screen_box.max().x())
        screen_box.max().x() = screen_box.min().x() + 1;
      if (screen_box.min().y() >= screen_box.max().y())
        screen_box.max().y() = screen_box.min().y() + 1;
      screen_box_list.push_back(screen_box);

      // Go from world coordinates to pixels in the second image.
      BBox2 image_box = MainWidget::world2image(world_box, i);

      // Grow a bit to integer, as otherwise we get strange results
      // if zooming too close.
      image_box.min() = floor(image_box.min());
      image_box.max() = ceil(image_box.max());

      QImage qimg;
      // Since the image portion contained in image_box could be huge,
      // but the screen area small, render a sub-sampled version of
      // the image for speed.
      // Convert to double before multiplication, to avoid overflow
      // when multiplying large integers.
      double scale = sqrt((1.0*image_box.width()) * image_box.height())/
        std::max(1.0, sqrt((1.0*screen_box.width()) * screen_box.height()));
      double scale_out;
      BBox2i region_out;
      bool   highlight_nodata = m_shadow_thresh_view_mode;
      if (m_shadow_thresh_view_mode){
        m_shadow_thresh_images[i].img.get_image_clip(scale, image_box,
                                                     highlight_nodata,
                                                     qimg, scale_out, region_out);
      }else if (m_hillshade_mode[i]){
        m_hillshaded_images[i].img.get_image_clip(scale, image_box,
                                                  highlight_nodata,
                                                  qimg, scale_out, region_out);
      }else{
        // Original images
        m_images[i].img.get_image_clip(scale, image_box,
                                       highlight_nodata,
                                       qimg, scale_out, region_out);
      }

      // Draw on image screen
      if (!m_use_georef){
        // This is a regular image, no georeference, just pass it to the QT painter
        QRect rect(screen_box.min().x(), screen_box.min().y(),
                   screen_box.width(), screen_box.height());
        paint->drawImage (rect, qimg);
      }else{
        // We fetched a bunch of pixels at some scale.
        // Need to place them on the screen at given projected position.
        // - To do that we will fill up this QImage object with interpolated data, then paint it.
        QImage qimg2 = QImage(screen_box.width(), screen_box.height(),
                              QImage::Format_ARGB32_Premultiplied);

        // Initialize all pixels to transparent
        for (int col = 0; col < qimg2.width(); col++) {
          for (int row = 0; row < qimg2.height(); row++) {
            qimg2.setPixel(col, row,  QColor(0, 0, 0, 0).rgba());
          }
        }

        // Loop through pixels
        for (int x = screen_box.min().x(); x < screen_box.max().x(); x++){
          for (int y = screen_box.min().y(); y < screen_box.max().y(); y++){

            // Convert from a pixel as seen on screen to the world coordinate system.
            Vector2 world_pt = screen2world(Vector2(x, y));

            // p is in pixel coordinates of m_images[i]
            Vector2 p;
            try {
              p = MainWidget::world2image(world_pt, i);
              bool is_in = (p[0] >= 0 && p[0] <= m_images[i].img.cols()-1 &&
                            p[1] >= 0 && p[1] <= m_images[i].img.rows()-1 );
              if (!is_in) continue; // out of range
            }catch ( const std::exception & e ) {
              continue;
            }
                        
            // Convert to scaled image pixels and snap to integer value
            p = round(p/scale_out);

            if (!region_out.contains(p)) continue; // out of range again

            int px = p.x() - region_out.min().x();
            int py = p.y() - region_out.min().y();
            if (px < 0 || py < 0 || px >= qimg.width() || py >= qimg.height() ){
              vw_out() << "Book-keeping failure!";
              vw_throw(ArgumentErr() << "Book-keeping failure.\n");
            }
	    qimg2.setPixel(x-screen_box.min().x(), // Fill the temp QImage object
                           y-screen_box.min().y(),
                           qimg.pixel(px, py));
          }
        } // End loop through pixels

        // Send the temp QImage object to the painter
        QRect rect(screen_box.min().x(), screen_box.min().y(),
                   screen_box.width(), screen_box.height());
        paint->drawImage (rect, qimg2);
      }

    } // End loop through input images

    // Call another function to handle drawing the interest points
    if ((static_cast<size_t>(m_image_id) < m_matches.size()) && m_view_matches) {
      drawInterestPoints(paint, screen_box_list);
    }

    return;
  } // End function drawImage()


  void MainWidget::drawInterestPoints(QPainter* paint, std::list<BBox2i> const& valid_regions) {

    QColor ipColor          = QColor("red"  ); // Hard coded interest point color
    QColor ipHighlightColor = QColor("green"); // Used to highlight a point being selected

    // Convert the input rects to QRect format
    std::list<QRect> qrect_list;
    for (std::list<BBox2i>::const_iterator i=valid_regions.begin(); i!=valid_regions.end(); ++i)
      qrect_list.push_back(QRect(i->min().x(), i->min().y(),
                                 i->width(),   i->height()));

    paint->setPen(ipColor);
    paint->setBrush(Qt::NoBrush);

    std::vector<vw::ip::InterestPoint> & ip = m_matches[m_image_id]; // IP's for this image

    if (m_images.size() != 1 && !ip.empty()) {
      // Must refresh the matches in all the images, not just this one
      emit turnOffViewMatchesSignal();
      return;
    }

    // If this point is currently being edited by the user, highlight it.
    // - Here we check to see if it has not been placed in all images yet.
    bool highlight_last = false;
    for (size_t h = 0; h < m_matches.size(); h++) {
      if (ip.size() > m_matches[h].size())
        highlight_last = true;
    }

    // For each IP...
    for (size_t ip_iter = 0; ip_iter < ip.size(); ip_iter++) {
      // Generate the pixel coord of the point
      double x = ip[ip_iter].x;
      double y = ip[ip_iter].y;
      Vector2 P = world2screen(Vector2(x, y));
      QPoint Q(P.x(), P.y());

      // Skip the point if none of the valid regions contain it
      bool safe = false;
      for (std::list<QRect>::const_iterator i=qrect_list.begin(); i!=qrect_list.end(); ++i) {
        if (i->contains(Q)) { // Verify the conversion worked
          safe = true;
          break;
        }
      }
      if (!safe)
        continue;

      if (highlight_last && (ip_iter == ip.size()-1)) // Highlighting the last point
        paint->setPen(ipHighlightColor);

      paint->drawEllipse(Q, 2, 2); // Draw the point!
    } // End loop through points
  } // End function drawInterestPoints


  void MainWidget::updateCurrentMousePosition() {
    m_curr_world_pos = screen2world(m_curr_pixel_pos);
  }

  void MainWidget::setZoomAllToSameRegion(bool zoom_all_to_same_region){
    m_zoom_all_to_same_region = zoom_all_to_same_region;
  }

  vw::BBox2 MainWidget::current_view(){
    return m_current_view;
  }

  void MainWidget::zoomToRegion(vw::BBox2 const& region){
    if (region.empty()) {
      popUp("Cannot zoom to empty region.");
      return;
    }
    m_current_view = expand_box_to_keep_aspect_ratio(region);
    MainWidget::refreshPixmap();
  }

  // --------------------------------------------------------------
  //             MainWidget Event Handlers
  // --------------------------------------------------------------

  void MainWidget::refreshPixmap(){

    // This is an expensive function. It will completely redraw
    // what is on the screen. For that reason, don't draw directly on
    // the screen, but rather into m_pixmap, which we use as a cache.

    // If just tiny redrawings are necessary, such as updating the
    // rubberband, simply pull the view from this cache,
    // and update the rubberband on top of it. This technique
    // is a well-known design pattern in Qt.

    if (m_zoom_all_to_same_region && m_can_emit_zoom_all_signal){
      m_can_emit_zoom_all_signal = false;
      emit zoomAllToSameRegionSignal(m_image_id);

      // Now we call the parent, which will set the zoom window,
      // and call back here for all widgets.
      return;
    }

    m_pixmap = QPixmap(size());
    m_pixmap.fill(this, 0, 0);

    QPainter paint(&m_pixmap);
    paint.initFrom(this);

    //QFont F;
    //F.setPointSize(m_prefs.fontSize);
    //F.setStyleStrategy(QFont::NoAntialias);
    //paint.setFont(F);
    MainWidget::drawImage(&paint);

    // Invokes MainWidget::PaintEvent().
    update();

    return;
  }

  void MainWidget::paintEvent(QPaintEvent * /* event */) {

    if (m_firstPaintEvent){
      // This will be called the very first time the display is
      // initialized. We will paint into the pixmap, and
      // then display the pixmap on the screen.
      m_firstPaintEvent = false;
      refreshPixmap();
    }

    // Note that we draw from the cached pixmap, instead of redrawing
    // the image from scratch.
    QPainter paint(this);
    paint.drawPixmap(0, 0, m_pixmap);

    QColor rubberBandColor = QColor("yellow");
    QColor cropWinColor = QColor("red");

    // We will color the rubberband in the crop win color if we are
    // in crop win mode.
    if (m_cropWinMode)
      paint.setPen(cropWinColor);
    else
      paint.setPen(rubberBandColor);

    // Draw the rubberband. We adjust by subtracting 1 from right and
    // bottom corner below to be consistent with updateRubberBand(), as
    // rect.bottom() is rect.top() + rect.height()-1.
    paint.drawRect(m_rubberBand.normalized().adjusted(0, 0, -1, -1));

    // Draw the stereo crop window.  Note that the stereo crop window
    // may exist independently of whether the rubber band exists.
    if (!m_stereoCropWin.empty()) {
      QRect R = bbox2qrect(world2screen(m_stereoCropWin));
      paint.setPen(cropWinColor);
      paint.drawRect(R.normalized().adjusted(0, 0, -1, -1));
    }

    // If we allow multiple selection windows
    for (size_t win = 0; win < m_selectionRectangles.size(); win++) {
      QRect R = bbox2qrect(world2screen(m_selectionRectangles[win]));
      paint.setPen(cropWinColor);
      paint.drawRect(R.normalized().adjusted(0, 0, -1, -1));
    }
    
    // Plot the polygonal line which we are profiling
    plotProfilePolyLine(paint, m_profileX, m_profileY);
  }

  // Call paintEvent() on the edges of the rubberband
  void MainWidget::updateRubberBand(QRect & R){
    QRect rect = R.normalized();
    if (rect.width() > 0 || rect.height() > 0) {
      update(rect.left(),  rect.top(),    rect.width(), 1             );
      update(rect.left(),  rect.top(),    1,            rect.height() );
      update(rect.left(),  rect.bottom(), rect.width(), 1             );
      update(rect.right(), rect.top(),    1,            rect.height() );
    }
    return;
  }

  void MainWidget::mousePressEvent(QMouseEvent *event) {

    // for rubberband
    m_mousePrsX  = event->pos().x();
    m_mousePrsY  = event->pos().y();

    m_rubberBand = m_emptyRubberBand;

    m_curr_pixel_pos = QPoint2Vec(QPoint(m_mousePrsX, m_mousePrsY));
    m_last_gain      = m_gain;     // Store this so the user can do linear
    m_last_offset    = m_offset; // and nonlinear steps.
    m_last_gamma     = m_gamma;
    updateCurrentMousePosition();

    // Need this for panning
    m_last_view = m_current_view;
  }

  void MainWidget::mouseMoveEvent(QMouseEvent *event) {

    if (event->modifiers() & Qt::AltModifier) {
#if 0
      // Diff variables are just the movement of the mouse normalized to
      // 0.0-1.0;
      double x_diff = double(event->x() - m_curr_pixel_pos.x()) / m_window_width;
      double y_diff = double(event->y() - m_curr_pixel_pos.y()) / m_window_height;
      double width = m_current_view.width();
      double height = m_current_view.height();

      // TODO: Support other adjustment modes
      m_adjust_mode = TransformAdjustment;

      std::ostringstream s;
      switch (m_adjust_mode) {
      case NoAdjustment:
        break;
      case TransformAdjustment:
        // This code does not work
        m_current_view.min().x() = m_last_view.min().x() + x_diff;
        m_current_view.max().x() = m_last_view.max().x() + x_diff;
        m_current_view.min().y() = m_last_view.min().y() + y_diff;
        m_current_view.max().y() = m_last_view.max().y() + y_diff;
        refreshPixmap();
        break;

      case GainAdjustment:
        m_gain = m_last_gain * pow(2.0,x_diff);
        s << "Gain: " << (log(m_gain)/log(2)) << " f-stops\n";
        break;

      case OffsetAdjustment:
        m_offset = m_last_offset +
          (pow(100,fabs(x_diff))-1.0)*(x_diff > 0 ? 0.1 : -0.1);
        s << "Offset: " << m_offset << "\n";
        break;

      case GammaAdjustment:
        m_gamma = m_last_gamma * pow(2.0,x_diff);
        s << "Gamma: " << m_gamma << "\n";
        break;
      }
#endif

    } else if (event->buttons() & Qt::LeftButton) {

      if (event->modifiers() & Qt::ControlModifier)
        m_cropWinMode = true;

      QPoint Q = event->pos();
      int x = Q.x(), y = Q.y();

      // Standard Qt rubberband trick. This is highly confusing.  The
      // explanation for what is going on is the following.  We need to
      // wipe the old rubberband, and draw a new one.  Hence just the
      // perimeters of these two rectangles need to be re-painted,
      // nothing else changes. The first updateRubberBand() call below
      // schedules that the perimeter of the current rubberband be
      // repainted, but the actual repainting, and this is the key, WILL
      // HAPPEN LATER! Then we change m_rubberBand to the new value,
      // then we schedule the repaint event on the new rubberband.
      // Continued below.
      updateRubberBand(m_rubberBand);
      m_rubberBand = QRect( min(m_mousePrsX, x), min(m_mousePrsY, y),
                            abs(x - m_mousePrsX), abs(y - m_mousePrsY) );
      updateRubberBand(m_rubberBand);
      // Only now, a single call to MainWidget::PaintEvent() happens,
      // even though it appears from above that two calls could happen
      // since we requested two updates. This call updates the perimeter
      // of the old rubberband, in effect wiping it, since the region
      // occupied by the old rubberband is scheduled to be repainted,
      // but the rubberband itself is already changed.  It also updates
      // the perimeter of the new rubberband, and as can be seen in
      // MainWidget::PaintEvent() the effect is to draw the rubberband.

      if (m_cropWinMode && !m_allowMultipleSelections) {
        // If there is on screen already a crop window, wipe it, as
        // we are now in the process of creating a new one.
        QRect R = bbox2qrect(world2screen(m_stereoCropWin));
        updateRubberBand(R);
        m_stereoCropWin = BBox2();
        R = bbox2qrect(world2screen(m_stereoCropWin));
        updateRubberBand(R);
      }

    }

    updateCurrentMousePosition();
  }
  
  // We assume the user picked n points in the image.
  // Draw n-1 segments in between them. Plot the obtained profile.
  void MainWidget::plotProfile(std::vector<imageData> const& images,
			       // indices in the image to profile
			       std::vector<double> const& profileX, 
			       std::vector<double> const& profileY){

    if (images.empty()) return; // nothing to do

    // Create the profile window
    if (m_profilePlot == NULL) 
      m_profilePlot = new ProfilePlotter(this);

    int imgInd = 0; // just one image is present
    double nodata_val = images[imgInd].img.get_nodata_val();
    
    m_valsX.clear(); m_valsY.clear();
    int count = 0;
    
    int num_pts = profileX.size();
    for (int pt_iter = 0; pt_iter < num_pts; pt_iter++) {

      // Nothing to do if we are at the last point, unless
      // there is only one point.
      if (num_pts > 1 && pt_iter == num_pts - 1) continue;
	
      Vector2 begP = MainWidget::world2image(Vector2(profileX[pt_iter], profileY[pt_iter]),
                                             imgInd);

      Vector2 endP;
      if (num_pts == 1) {
	endP = begP; // only one point is present
      }else{
	endP = MainWidget::world2image(Vector2(profileX[pt_iter+1], profileY[pt_iter+1]),
				       imgInd);
      }
      
      int begX = begP.x(),   begY = begP.y();
      int endX = endP.x(),   endY = endP.y();
      int seg_len = std::abs(begX - endX) + std::abs(begY - endY);
      if (seg_len == 0) seg_len = 1; // ensure it is never empty
      for (int p = 0; p <= seg_len; p++) {
	double t = double(p)/seg_len;
	int x = round( begX + t*(endX - begX) );
	int y = round( begY + t*(endY - begY) );
	bool is_in = (x >= 0 && x <= images[imgInd].img.cols()-1 &&
		      y >= 0 && y <= images[imgInd].img.rows()-1 );
	if (!is_in) continue;

        double pixel_val = images[imgInd].img.get_value_as_double(x, y);

	// TODO: Deal with this NAN
	if (pixel_val == nodata_val)
          pixel_val = std::numeric_limits<double>::quiet_NaN();
        m_valsX.push_back(count);
        m_valsY.push_back(pixel_val);
        count++;
      }
      
    }

    if (num_pts == 1) {
      // Just one point, really
      m_valsX.resize(1);
      m_valsY.resize(1);
    }
    
    // Wipe whatever was there before
    m_profilePlot->detachItems(); 
    
    QwtPlotCurve * curve = new QwtPlotCurve("1D Profile");
    m_profilePlot->setFixedWidth(300);
    m_profilePlot->setWindowTitle("1D Profile");

    if (!m_valsX.empty()) {
      
      double min_x = *std::min_element(m_valsX.begin(), m_valsX.end());
      double max_x = *std::max_element(m_valsX.begin(), m_valsX.end());
      double min_y = *std::min_element(m_valsY.begin(), m_valsY.end());
      double max_y = *std::max_element(m_valsY.begin(), m_valsY.end());

      // Ensure the window is always valid
      double small = 0.1;
      if (min_x == max_x) {
	min_x -= small; max_x += small;
      }
      if (min_y == max_y) {
	min_y -= small; max_y += small;
      }

      // Plot a point as a fat dot
      if (num_pts == 1)  {
	curve->setStyle(QwtPlotCurve::Dots);
      }
      
      curve->setData(new QwtCPointerData(&m_valsX[0], &m_valsY[0], m_valsX.size()));
      curve->setPen(* new QPen(Qt::red));
      curve->attach(m_profilePlot);
      
      double delta = 0.1;  // expand a bit right to see more x and y labels
      double widx = max_x - min_x;
      double widy = max_y - min_y;
      m_profilePlot->setAxisScale(QwtPlot::xBottom, min_x - delta*widx, max_x + delta*widx);
      m_profilePlot->setAxisScale(QwtPlot::yLeft,   min_y - delta*widy, max_y + delta*widy);
    }

    // Finally, refresh the plot
    m_profilePlot->replot();
    m_profilePlot->show();
  }
  
  void MainWidget::toggleProfileMode(bool profile_mode){
    m_profileMode = profile_mode;

    if (!m_profileMode) {
      // Clean up any profiling related info
      m_profileX.clear();
      m_profileY.clear();

      // Close the window. 
      if (m_profilePlot != NULL) {
	m_profilePlot->close();
	m_profilePlot->deleteLater();
	delete m_profilePlot;
	m_profilePlot = NULL;
      }

      // Call back to the main window and tell it to uncheck the profile
      // mode checkbox.
      emit uncheckProfileModeCheckbox();
      return;
    }else{
      // Show the profile window
      MainWidget::plotProfile(m_images, m_profileX, m_profileY);
    }

    refreshPixmap();
  }

  // Go to the pixel locations on screen, and draw the polygonal line.
  // This is robust to zooming in the middle of profiling.
  void MainWidget::plotProfilePolyLine(QPainter & paint,
                                       std::vector<double> const& profileX, 
                                       std::vector<double> const& profileY){

    if (profileX.empty()) return;

    //QPainter paint(&m_pixmap);
    //paint.initFrom(this);
    
//     if (!m_profileMode) {
//       QPoint Q(mouse_rel_pos.x(), mouse_rel_pos.y());
//       paint.setPen(QColor("red"));
//       paint.drawEllipse(Q, 2, 2); // Draw the point, and make it a little large
//     }
    
    paint.setPen(QColor("red"));
    std::vector<QPoint> profilePixels;
    for (size_t it = 0; it < profileX.size(); it++) {
      Vector2 P = world2screen(Vector2(profileX[it], profileY[it]));
      QPoint Q(P.x(), P.y());
      paint.drawEllipse(Q, 2, 2); // Draw the point, and make it a little large
      profilePixels.push_back(Q);
    }
    paint.drawPolyline(&profilePixels[0], profilePixels.size());
  }
  
  void MainWidget::mouseReleaseEvent ( QMouseEvent *event ){

    QPoint mouse_rel_pos = event->pos();

    if (m_images.empty()) return;

    int tol = 3; // pixels

    // If the mouse was released close to where it was pressed
    if (std::abs(m_mousePrsX - mouse_rel_pos.x()) < tol &&
	std::abs(m_mousePrsY - mouse_rel_pos.y()) < tol ) {

      if (!m_shadow_thresh_calc_mode){

        Vector2 p = screen2world(Vector2(mouse_rel_pos.x(), mouse_rel_pos.y()));
        
        QPainter paint(&m_pixmap);
        paint.initFrom(this);

        if (!m_profileMode) {
          QPoint Q(mouse_rel_pos.x(), mouse_rel_pos.y());
          paint.setPen(QColor("red"));
          paint.drawEllipse(Q, 2, 2); // Draw the point, and make it a little large
        }
        
        bool can_profile = m_profileMode;
        
        // Print pixel coordinates and image value.
	for (size_t it = 0; it < m_images.size(); it++) {
          
	  std::string val = "none";
	  Vector2 q = world2image(p, it);
          int col = floor(q[0]), row = floor(q[1]);
          
	  if (col >= 0 && row >= 0 && col < m_images[it].img.cols() &&
	      row < m_images[it].img.rows() ) {
	    val = m_images[it].img.get_value_as_str(col, row);
	  }

	  vw_out() << "Pixel and value for " << m_image_files[it] << ": "
		   << col << ' ' << row << ' ' << val << std::endl;
	  update();

          if (m_profileMode) {

            // Sanity checks
            if (m_images.size() != 1) {
              popUp("A profile can be shown only when a single image is present.");
              can_profile = false;
            }
            int num_channels = m_images[it].img.planes();
            if (num_channels != 1) {
              popUp("A profile can be shown only when the image has a single channel.");
              can_profile = false;
            }
            
            if (!can_profile) {
              MainWidget::toggleProfileMode(can_profile);
	      return;
            }
            
	  }

	} // end iterating over images

	if (can_profile) {
	  // Save the current point the user clicked onto in the
	  // world coordinate system.
	  m_profileX.push_back(p.x());
	  m_profileY.push_back(p.y());
	  
	  // PaintEvent() will be called, which will call
	  // plotProfilePolyLine() to show the polygonal line.
          
	  // Now show the profile.
	  MainWidget::plotProfile(m_images, m_profileX, m_profileY);
	}
        
      }else{
	// Shadow threshold mode. If we released the mouse where we
	// pressed it, that means we want the current point to be
	// marked as shadow.
	if (m_images.size() != 1) {
	  popUp("Must have just one image in each window to do shadow threshold detection.");
	  m_shadow_thresh_calc_mode = false;
	  refreshPixmap();
	  return;
	}
	
	if (m_images[0].img.planes() != 1) {
	  popUp("Thresholding makes sense only for single-channel images.");
	  m_shadow_thresh_calc_mode = false;
	  return;
	}
	
	if (m_use_georef) {
	  popUp("Thresholding is not supported when using georeference information to show images.");
	  m_shadow_thresh_calc_mode = false;
	  return;
	}
	
	Vector2 p = screen2world(Vector2(mouse_rel_pos.x(), mouse_rel_pos.y()));
        Vector2 q = world2image(p, 0);

	int col = round(q[0]), row = round(q[1]);
	vw_out() << "Clicked on pixel: " << col << ' ' << row << std::endl;
	
	if (col >= 0 && row >= 0 && col < m_images[0].img.cols() &&
	    row < m_images[0].img.rows() ) {
	  double val = m_images[0].img.get_value_as_double(col, row);
	  m_shadow_thresh = std::max(m_shadow_thresh, val);
	}
	vw_out() << "Shadow threshold for "
		 << m_image_files[0]
		 << ": " << m_shadow_thresh << std::endl;
	return;
      }
    }
    
    if( (event->buttons() & Qt::LeftButton) &&
        (event->modifiers() & Qt::ControlModifier) ){
      m_cropWinMode = true;
    }

    if (event->buttons() & Qt::RightButton) {
      if (std::abs(mouse_rel_pos.x() - m_mousePrsX) < tol &&
          std::abs(mouse_rel_pos.y() - m_mousePrsY) < tol
          ){
        // If the mouse was released too close to where it was clicked,
        // do nothing.
        return;
      }

      // Drag the image along the mouse movement
      m_current_view -= (screen2world(QPoint2Vec(event->pos())) -
                         screen2world(QPoint2Vec(QPoint(m_mousePrsX, m_mousePrsY))));

      refreshPixmap(); // will call paintEvent()

    } else if (m_cropWinMode){

      // If now we allow multiple selected regions, but we did not allow at
      // the time the crop win was formed, save the crop win before it
      // will be overwritten.
      if (m_allowMultipleSelections && !m_stereoCropWin.empty()) {
        if (m_selectionRectangles.empty() || m_selectionRectangles.back() != m_stereoCropWin) {
          m_selectionRectangles.push_back(m_stereoCropWin);
        }
      }
      
      // User selects the region to use for stereo.  Convert it to world
      // coordinates, and round to integer.  If we use georeferences,
      // the crop win is in projected units for the first image,
      // so we must convert to pixels.
      m_stereoCropWin = screen2world(qrect2bbox(m_rubberBand));
      
      if (m_allowMultipleSelections && !m_stereoCropWin.empty()) 
        m_selectionRectangles.push_back(m_stereoCropWin);
      
      for (size_t i = 0; i < m_images.size(); i++) {

        BBox2i image_box = world2image(m_stereoCropWin, i);
        vw_out().precision(8);
        vw_out() << "Crop src win for  "
                 << m_image_files[i]
                 << ": "
                 << image_box.min().x() << ' '
                 << image_box.min().y() << ' '
                 << image_box.width()   << ' '
                 << image_box.height()  << std::endl;
        if (m_images[i].has_georef){
          Vector2 proj_min, proj_max;
          // Convert pixels to projected coordinates
          BBox2 point_box = m_images[i].georef.pixel_to_point_bbox(image_box);
          proj_min = point_box.min();
          proj_max = point_box.max();
          // Below we flip in y to make gdal happy
          vw_out() << "Crop proj win for "
                   << m_image_files[i] << ": "
                   << proj_min.x() << ' ' << proj_max.y() << ' '
                   << proj_max.x() << ' ' << proj_min.y() << std::endl;

          Vector2 lonlat_min, lonlat_max;
          BBox2 lonlat_box = m_images[i].georef.pixel_to_lonlat_bbox(image_box);
          lonlat_min = lonlat_box.min();
          lonlat_max = lonlat_box.max();
          // Again, miny and maxy are flipped on purpose
          vw_out() << "lonlat win for    "
                   << m_image_files[i] << ": "
                   << lonlat_min.x() << ' ' << lonlat_max.y() << ' '
                   << lonlat_max.x() << ' ' << lonlat_min.y() << std::endl;
        }
      }

      // Wipe the rubberband, no longer needed.
      updateRubberBand(m_rubberBand);
      m_rubberBand = m_emptyRubberBand;
      updateRubberBand(m_rubberBand);

      // Draw the crop window. This may not be precisely the rubberband
      // since there is some loss of precision in conversion from
      // Qrect to BBox2 and back. Note actually that we are not drawing
      // here, we are scheduling this area to be updated, the drawing
      // has to happen (with precisely this formula) in PaintEvent().
      QRect R = bbox2qrect(world2screen(m_stereoCropWin));
      updateRubberBand(R);

    } else if (Qt::LeftButton) {

      // Zoom

      // Wipe the rubberband
      updateRubberBand(m_rubberBand);
      m_rubberBand = m_emptyRubberBand;
      updateRubberBand(m_rubberBand);

      int mouseRelX = event->pos().x();
      int mouseRelY = event->pos().y();

      m_can_emit_zoom_all_signal = true; 
        
      if (mouseRelX > m_mousePrsX && mouseRelY > m_mousePrsY) {

        // Dragging the mouse from upper-left to lower-right zooms in

        // The window selected with the mouse in world coordinates
        Vector2 A = screen2world(Vector2(m_mousePrsX, m_mousePrsY));
        Vector2 B = screen2world(Vector2(mouseRelX, mouseRelY));
        BBox2 view = BBox2(A, B);

        // Zoom to this window. Don't zoom so much that the view box
        // ends up having size 0 to numerical precision.
        if (!view.empty())
          m_current_view = expand_box_to_keep_aspect_ratio(view);

        // Must redraw the entire image
        refreshPixmap();

      } else if (mouseRelX < m_mousePrsX && mouseRelY < m_mousePrsY) {
        // Dragging the mouse in reverse zooms out
        double scale = 0.8;
        zoom(scale);
      }

    }

    // At this stage the user is supposed to release the control key, so
    // we are no longer in crop win mode, even if we were so far.
    m_cropWinMode = false;

    return;
  }


  void MainWidget::mouseDoubleClickEvent(QMouseEvent *event) {
    m_curr_pixel_pos = QPoint2Vec(event->pos());
    updateCurrentMousePosition();
  }

  void MainWidget::wheelEvent(QWheelEvent *event) {
    int num_degrees = event->delta();
    double num_ticks = double(num_degrees) / 360;

    // 2.0 chosen arbitrarily here as a reasonable scale factor giving good
    // sensitivity of the mousewheel. Shift zooms 50 times slower.
    double scale_factor = 2;
    if (event->modifiers() & Qt::ShiftModifier)
      scale_factor *= 50;

    double mag = fabs(num_ticks/scale_factor);
    double scale = 1;
    if (num_ticks > 0)
      scale = 1+mag;
    else if (num_ticks < 0)
      scale = 1-mag;

    zoom(scale);

    m_curr_pixel_pos = QPoint2Vec(event->pos());
    updateCurrentMousePosition();
  }


  void MainWidget::enterEvent(QEvent */*event*/) {
  }

  void MainWidget::leaveEvent(QEvent */*event*/) {
  }

  void MainWidget::keyPressEvent(QKeyEvent *event) {

    std::ostringstream s;

    // Save these before we modify the box
    double width  = m_current_view.width();
    double height = m_current_view.height();
      
    double factor = 0.2;  // We will pan by moving by 20%.
    switch (event->key()) {

      // Pan
    case Qt::Key_Left: // Pan left
      m_current_view.min().x() -= width*factor;
      m_current_view.max().x() -= width*factor;
      m_can_emit_zoom_all_signal = true;
      refreshPixmap();
      break;
    case Qt::Key_Right: // Pan right
      m_current_view.min().x() += width*factor;
      m_current_view.max().x() += width*factor;
      m_can_emit_zoom_all_signal = true;
      refreshPixmap();
      break;
    case Qt::Key_Up: // Pan up
      m_current_view.min().y() -= height*factor;
      m_current_view.max().y() -= height*factor;
      m_can_emit_zoom_all_signal = true;
      refreshPixmap();
      break;
    case Qt::Key_Down: // Pan down
      m_current_view.min().y() += height*factor;
      m_current_view.max().y() += height*factor;
      m_can_emit_zoom_all_signal = true;
      refreshPixmap();
      break;

      // Zoom out
    case Qt::Key_Minus:
    case Qt::Key_Underscore:
      zoom(0.75);
      break;

      // Zoom in
    case Qt::Key_Plus:
    case Qt::Key_Equal:
      zoom(1.0/0.75);
      break;

#if 0
    case Qt::Key_I:  // Toggle bilinear/nearest neighbor interp
      m_bilinear_filter = !m_bilinear_filter;
      break;
    case Qt::Key_C:  // Activate colormap
      m_use_colormap = !m_use_colormap;
      break;
    case Qt::Key_H:  // Activate hillshade
    case Qt::Key_G:  // Gain adjustment mode
      if (m_adjust_mode == GainAdjustment) {
        m_adjust_mode = TransformAdjustment;
      } else {
        m_adjust_mode = GainAdjustment;
        s << "Gain: " << log(m_gain)/log(2) << " f-stops\n";
      }
      break;
    case Qt::Key_O:  // Offset adjustment mode
      if (m_adjust_mode == OffsetAdjustment) {
        m_adjust_mode = TransformAdjustment;
      } else {
        m_adjust_mode = OffsetAdjustment;
        s << "Offset: " << m_offset;
      }
      break;
    case Qt::Key_V:  // Gamma adjustment mode
      if (m_adjust_mode == GammaAdjustment) {
        m_adjust_mode = TransformAdjustment;
      } else {
        m_adjust_mode = GammaAdjustment;
        s << "Gamma: " << m_gamma;
      }
      break;
    case Qt::Key_1:  // Display red channel only
      m_display_channel = DisplayR;
      break;
    case Qt::Key_2:  // Display green channel only
      m_display_channel = DisplayG;
      break;
    case Qt::Key_3:  // Display blue channel only
      m_display_channel = DisplayB;
      break;
    case Qt::Key_4:  // Display alpha channel only
      m_display_channel = DisplayA;
      break;
    case Qt::Key_0:  // Display all color channels
      m_display_channel = DisplayRGBA;
      break;
#endif

    default:
      QWidget::keyPressEvent(event);
    }
  }

  void MainWidget::contextMenuEvent(QContextMenuEvent *event){

    int x = event->x(), y = event->y();
    m_mousePrsX = x;
    m_mousePrsY = y;

    // Refresh this from the variable, before popping up the menu
    m_allowMultipleSelections_action->setChecked(m_allowMultipleSelections);

    m_ContextMenu->popup(mapToGlobal(QPoint(x,y)));
    return;
  }

  void MainWidget::viewMatches(bool view_matches){
    // Complain if there are multiple images and matches was turned on
    if ((m_images.size() != 1) && view_matches) {
      emit turnOffViewMatchesSignal();
      return;
    }

    m_view_matches = view_matches;
    refreshPixmap();
  }

  void MainWidget::addMatchPoint(){

    if (m_image_id >= (int)m_matches.size()) {
      popUp("Number of existing matches is corrupted. Cannot add matches.");
      return;
    }

    if (m_images.size() != 1) {
      emit turnOffViewMatchesSignal();
      return;
    }

    // We will start with an interest point in the left-most image,
    // and add matches to it in the other images.
    // At any time, an image to the left must have no fewer ip than
    // images on the right. Upon saving, all images must
    // have the same number of interest points.
    size_t curr_pts = m_matches[m_image_id].size(); // # Pts from current image
    bool is_good = true;
    for (int i = 0; i < m_image_id; i++) { // Look through lower-id images
      if (m_matches[i].size() < curr_pts+1) {
        is_good = false;
      }
    }
    // Check all higher-id images, they should have the same # Pts as this one.
    for (int i = m_image_id+1; i < (int)m_matches.size(); i++) {
      if (m_matches[i].size() > curr_pts) {
        is_good = false;
      }
    }

    if (!is_good) {
      popUp(std::string("Add matches by adding a point in the left-most ")
            + "image and corresponding matches in the other images left to right. "
            + "Cannot add this match.");
      return;
    }

    // Convert mouse coords to world coords, then add a new IP to the list for this image.
    Vector2 P = screen2world(Vector2(m_mousePrsX, m_mousePrsY));
    ip::InterestPoint ip;
    ip.x = P.x();
    ip.y = P.y();
    m_matches[m_image_id].push_back(ip);

    bool view_matches = true;
    viewMatches(view_matches);

    // Must refresh the matches in all the images, not just this one
    emit turnOnViewMatchesSignal();
  }

  // We cannot delete match points unless all images have the same number of them.
  void MainWidget::deleteMatchPoint(){

    if (m_matches.empty() || m_matches[0].empty()){
      popUp("No matches to delete.");
      return;
    }

    // Sanity checks
    for (int i = 0; i < int(m_matches.size()); i++) {
      if (m_matches[0].size() != m_matches[i].size()) {
        popUp("Cannot delete matches. Must have the same number of matches in each image.");
        return;
      }
    }
    if (m_image_id >= (int)m_matches.size()) {
      popUp("Number of existing matches is corrupted. Cannot delete matches.");
      return;
    }

    if (m_images.size() != 1) {
      popUp("Must have just one image in each window to delete matches.");
      return;
    }

    // Delete the closest match to this point.
    Vector2 P = screen2world(Vector2(m_mousePrsX, m_mousePrsY));
    double min_dist = std::numeric_limits<double>::max();
    int min_index = -1;
    std::vector<vw::ip::InterestPoint> & ip = m_matches[m_image_id]; // alias
    for (size_t ip_iter = 0; ip_iter < ip.size(); ip_iter++) {
      Vector2 Q(ip[ip_iter].x, ip[ip_iter].y);
      double curr_dist = norm_2(Q-P);
      if (curr_dist < min_dist) {
        min_dist = curr_dist;
        min_index = ip_iter;
      }
    }
    if (min_index >= 0) {
      for (size_t vec_iter = 0; vec_iter < m_matches.size(); vec_iter++) {
        m_matches[vec_iter].erase(m_matches[vec_iter].begin() + min_index);
      }
    }

    // Must refresh the matches in all the images, not just this one
    emit turnOnViewMatchesSignal();
  }

  // Show the current shadow threshold, and allow the user to change it.
  void MainWidget::setThreshold(){

    std::ostringstream oss;
    oss.precision(18);
    oss << m_shadow_thresh;
    std::string shadowThresh = oss.str();
    bool ans = getStringFromGui(this,
				"Shadow threshold",
				"Shadow threshold",
				shadowThresh,
				shadowThresh);
    if (!ans)
      return;

    double thresh = atof(shadowThresh.c_str());
    MainWidget::setThreshold(thresh);
  }

  void MainWidget::setThreshold(double thresh){
    if (m_images.size() != 1) {
      popUp("Must have just one image in each window to set the shadow threshold.");
      return;
    }

    m_shadow_thresh = thresh;
    vw_out() << "Shadow threshold for " << m_image_files[0]
	     << ": " << m_shadow_thresh << std::endl;
  }

  double MainWidget::getThreshold(){
    return m_shadow_thresh;
  }

  // Save the current view to a file
  void MainWidget::saveScreenshot(){

    QString fileName = QFileDialog::getSaveFileName(this,
                                                    tr("Save screenshot"), "./screenshot.bmp",
                                                    tr("(*.bmp *.xpm)"));

    if (fileName.toStdString() == "") 
      return;

    QImageWriter writer(fileName);
    if(!writer.write(m_pixmap.toImage())){
      popUp(writer.errorString().toStdString());
    }
    
  }
  
}} // namespace vw::gui
