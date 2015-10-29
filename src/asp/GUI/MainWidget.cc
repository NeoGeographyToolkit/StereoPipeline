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
#include <asp/GUI/MainWidget.h>

using namespace vw;
using namespace vw::gui;
using namespace std;

namespace vw { namespace gui {

  // --------------------------------------------------------------
  //               MainWidget Public Methods
  // --------------------------------------------------------------

  // Convert a position in the world coordinate system to a pixel
  // position as seen on screen (the screen origin is the
  // visible upper-left corner of the widget).
  vw::Vector2 MainWidget::world2screen(vw::Vector2 const& p){
    double x = m_window_width*((p.x() - m_current_view.min().x())
                               /m_current_view.width());
    double y = m_window_height*((p.y() - m_current_view.min().y())
                                /m_current_view.height());
    return vw::Vector2(x, y);
  }

  // Convert a pixel on the screen to world coordinates.
  // See world2image() for the definition.
  vw::Vector2 MainWidget::screen2world(vw::Vector2 const& p){
    double x = m_current_view.min().x()
      + m_current_view.width() * double(p.x()) / m_window_width;
    double y = m_current_view.min().y()
      + m_current_view.height() * double(p.y()) / m_window_height;
    return vw::Vector2(x, y);
  }

  BBox2 MainWidget::screen2world(BBox2 const& R) {
    if (R.empty()) return R;
    Vector2 A = screen2world(R.min());
    Vector2 B = screen2world(R.max());
    return BBox2(A, B);
  }

  BBox2 MainWidget::world2screen(BBox2 const& R) {
    if (R.empty()) return R;
    Vector2 A = world2screen(R.min());
    Vector2 B = world2screen(R.max());
    return BBox2(A, B);
  }

  // If we use georef, the world is in projected point units of the
  // first image, with y replaced with -y, to keep the y axis downward,
  // for consistency with how images are plotted.  Convert a world box
  // to a pixel box for the given image.
  Vector2 MainWidget::world2image(Vector2 const& P, int imageIndex) {
    return point_to_pixel(flip_in_y(P), -m_images[imageIndex].m_lon_offset,
                          m_images[imageIndex].georef, m_images[0].georef);
  }

  BBox2 MainWidget::world2image(BBox2 const& R, int imageIndex) {

    if (R.empty()) return R;
    if (m_images.empty()) return R;

    if (!m_use_georef)
      return R;

    BBox2 pixel_box = point_to_pixel_bbox(flip_in_y(R), -m_images[imageIndex].m_lon_offset,
                                          m_images[imageIndex].georef, m_images[0].georef);

    return pixel_box;
  }

  // The reverse of world2image()
  BBox2 MainWidget::image2world(BBox2 const& R, int imageIndex) {

    if (R.empty()) return R;
    if (m_images.empty()) return R;

    if (!m_use_georef)
      return R;

    BBox2 B = flip_in_y(forward_pixel_to_point_bbox(R, -m_images[imageIndex].m_lon_offset,
                                                    m_images[imageIndex].georef,
                                                    m_images[0].georef));

    return B;
  }

  MainWidget::MainWidget(QWidget *parent,
                         asp::BaseOptions const& opt,
                         int image_id,
                         std::string & output_prefix,
                         std::vector<std::string> const& image_files,
                         std::vector<std::vector<vw::ip::InterestPoint> > & matches,
                         chooseFilesDlg * chooseFiles,
                         bool use_georef, bool hillshade, bool view_matches)
    : QWidget(parent), m_opt(opt), m_chooseFilesDlg(chooseFiles),
      m_image_id(image_id), m_output_prefix(output_prefix),
      m_image_files(image_files), m_matches(matches),  m_use_georef(use_georef),
      m_hillshade_mode(hillshade), m_viewMatches(view_matches){

    installEventFilter(this);

    m_firstPaintEvent = true;
    m_emptyRubberBand = QRect(0, 0, 0, 0);
    m_rubberBand      = m_emptyRubberBand;
    m_cropWinMode     = false;

    m_mousePrsX = 0; m_mousePrsY = 0;

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
    // If we use georef, that pox is in projected point units
    // of the first image.
    int num_images = image_files.size();
    m_images.resize(num_images);
    m_filesOrder.resize(num_images);
    for (int i = 0; i < num_images; i++){
      m_images[i].read(image_files[i], m_opt, m_use_georef);
      m_filesOrder[i] = i; // start by keeping the order of files being read
      if (!m_use_georef){
        m_images_box.grow(m_images[i].image_bbox);
      }else{
        // Compensate for the fact some images show pixels at -90
        // degrees while others at 270 degrees.
        double midi = (m_images[i].lonlat_bbox.min().x() + m_images[i].lonlat_bbox.max().x())/2.0;
        double mid0 = (m_images[0].lonlat_bbox.min().x() + m_images[0].lonlat_bbox.max().x())/2.0;
        m_images[i].m_lon_offset = 360.0*round((midi-mid0)/360.0);
        m_images[i].lonlat_bbox -= Vector2(m_images[i].m_lon_offset, 0);

        // Convert from pixels in image i to world coordinates.
        BBox2 B = MainWidget::image2world(m_images[i].image_bbox, i);
        m_images_box.grow(B);
      }
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
                       SLOT(showFilesChosenByUser(int, int))
                       );
      m_chooseFilesDlg->chooseFiles(m_images);
    }

    // Right-click context menu
    m_ContextMenu = new QMenu();
    //setContextMenuPolicy(Qt::CustomContextMenu);
    m_addMatchPoint    = m_ContextMenu->addAction("Add match point");
    m_deleteMatchPoint = m_ContextMenu->addAction("Delete match point");
    connect(m_addMatchPoint,    SIGNAL(triggered()),this,SLOT(addMatchPoint()));
    connect(m_deleteMatchPoint, SIGNAL(triggered()),this,SLOT(deleteMatchPoint()));

    if (m_hillshade_mode)
      MainWidget::genHillshadedImages();
  }


  MainWidget::~MainWidget() {
  }

  bool MainWidget::eventFilter(QObject *obj, QEvent *E){
    return QWidget::eventFilter(obj, E);
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
      std::vector<int>::iterator it = std::find(m_filesOrder.begin(), m_filesOrder.end(), rowClicked);
      if (it != m_filesOrder.end()){
        m_filesOrder.erase(it);
        m_filesOrder.push_back(rowClicked); // show last, so on top
      }
    }

    refreshPixmap();

    return;
  }

  BBox2 MainWidget::expand_box_to_keep_aspect_ratio(BBox2 const& box) {
    BBox2 out_box = box;
    double aspect = double(m_window_width) / m_window_height;
    if (box.width() / box.height() < aspect) {
      // Width needs to grow
      double new_width = box.height() * aspect;
      double delta = (new_width - box.width())/2.0;
      out_box.min().x() -= delta; out_box.max().x() += delta;
    }else if (box.width() / box.height() > aspect) {
      // Height needs to grow
      double new_height = box.width() / aspect;
      double delta = (new_height - box.height())/2.0;
      out_box.min().y() -= delta; out_box.max().y() += delta;
    }
    return out_box;
  }

  void MainWidget::sizeToFit() {

    m_current_view = expand_box_to_keep_aspect_ratio(m_images_box);

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
    m_hillshade_mode = false;
    refreshPixmap();
  }

  void MainWidget::viewThreshImages(){
    m_shadow_thresh_view_mode = true;
    m_hillshade_mode = false;

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

      int num_channels = get_num_channels(input_file);
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

  void MainWidget::genHillshadedImages(){

    int num_images = m_images.size();
    m_hillshaded_images.clear(); // wipe the old copy
    m_hillshaded_images.resize(num_images);

    // Create the hillshaded images and save them to disk. We have to do
    // it each time as perhaps the hillshade parameters changed.
    for (int image_iter = 0; image_iter < num_images; image_iter++) {

      if (!m_images[image_iter].has_georef) {
        popUp("Hill-shading requires georeferenced images.");
        m_hillshade_mode = false;
        return;
      }

      std::string input_file = m_image_files[image_iter];
      int num_channels = get_num_channels(input_file);
      if (num_channels != 1) {
        popUp("Hill-shading makes sense only for single-channel images.");
        m_hillshade_mode = false;
        return;
      }

      // Save the hillshaded images to disk
      std::string hillshaded_file;
      bool success = write_hillshade(m_opt, input_file, hillshaded_file);
      if (!success) {
        m_hillshade_mode = false;
        return;
      }

      m_hillshaded_images[image_iter].read(hillshaded_file, m_opt, m_use_georef);
      temporary_files().files.insert(hillshaded_file);
    }
  }

  void MainWidget::viewHillshadedImages(bool hillshade_mode){
    m_hillshade_mode = hillshade_mode;
    m_shadow_thresh_calc_mode = false;
    m_shadow_thresh_view_mode = false;

    if (m_hillshade_mode)
      MainWidget::genHillshadedImages();

    refreshPixmap();
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
      refreshPixmap();
    }
  }

  void MainWidget::resizeEvent(QResizeEvent*){
    QRect v       = this->geometry();
    m_window_width = v.width();
    m_window_height = v.height();
    sizeToFit();
    return;
  }


  // --------------------------------------------------------------
  //             MainWidget Private Methods
  // --------------------------------------------------------------

  void MainWidget::drawImage(QPainter* paint) {

    std::list<BBox2i> screen_box_list; // List of regions the images are drawn in

    // Loop through input images
    // - These images get drawn in the same
    for (int j = 0; j < (int)m_images.size(); j++){

      int i = m_filesOrder[j];

      // Don't show files the user wants hidden
      string fileName = m_images[i].name;
      if (m_filesToHide.find(fileName) != m_filesToHide.end()) continue;

      // The current view. If we use georef, the world coordinates are
      // described in world2image().
      BBox2 world_box = m_current_view;
      if (!m_use_georef)
        world_box.crop(m_images[i].image_bbox);
      else{
        // Convert from pixels in image i to projected points in image 0.
        BBox2 B = MainWidget::image2world(m_images[i].image_bbox, i);
        world_box.crop(B);
      }

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
      BBox2i image_box = MainWidget::world2image(world_box, i);

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
        m_shadow_thresh_images[i].img.getImageClip(scale, image_box,
                                                   highlight_nodata,
                                                   qimg, scale_out, region_out);
      }else if (m_hillshade_mode){
        m_hillshaded_images[i].img.getImageClip(scale, image_box,
                                                highlight_nodata,
                                                qimg, scale_out, region_out);
      }else{
        // Original images
        m_images[i].img.getImageClip(scale, image_box,
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
        QImage qimg2 = QImage(screen_box.width(), screen_box.height(), QImage::Format_RGB888);

        // Initialize all pixels to black
        for (int col = 0; col < qimg2.width(); col++) {
          for (int row = 0; row < qimg2.height(); row++) {
            qimg2.setPixel(col, row, qRgb(0, 0, 0));
          }
        }

        // Loop through pixels
        for (int x = screen_box.min().x(); x < screen_box.max().x(); x++){
          for (int y = screen_box.min().y(); y < screen_box.max().y(); y++){

            // Convert from a pixel as seen on screen to the world coordinate system.
            Vector2 world_pt = screen2world(Vector2(x, y));

            // p is in pixel coordinates of m_images[i]
            Vector2 p = MainWidget::world2image(world_pt, i);
            bool is_in = (p[0] >= 0 && p[0] <= m_images[i].img.cols()-1 &&
                          p[1] >= 0 && p[1] <= m_images[i].img.rows()-1 );
            if (!is_in)
              continue; // out of range

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
    if ((static_cast<size_t>(m_image_id) < m_matches.size()) && m_viewMatches) {
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
      popUp("Must have just one image in each window to view matches.");
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
    QStylePainter paint(this);
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
    m_last_gain = m_gain;     // Store this so the user can do linear
    m_last_offset = m_offset; // and nonlinear steps.
    m_last_gamma = m_gamma;
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

      if (m_cropWinMode) {
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

  void MainWidget::mouseReleaseEvent ( QMouseEvent *event ){

    QPoint mouse_rel_pos = event->pos();

    if (m_images.empty()) return;

    // If we are in shadow threshold detection mode, and we released
    // the mouse where we pressed it, that means we want the current
    // point to be marked as shadow.
    int tol = 3; // pixels
    if (m_shadow_thresh_calc_mode &&
        std::abs(m_mousePrsX - mouse_rel_pos.x()) < tol &&
        std::abs(m_mousePrsY - mouse_rel_pos.y()) < tol ) {

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

      int col = round(p[0]), row = round(p[1]);
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

      // User selects the region to use for stereo.  Convert it to world
      // coordinates, and round to integer.  If we use georeferences,
      // the crop win is in projected units for the first image,
      // so we must convert to pixels.
      m_stereoCropWin = screen2world(qrect2bbox(m_rubberBand));

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

    double factor = 0.2;  // We will pan by moving by 20%.
    switch (event->key()) {

      // Pan
    case Qt::Key_Left: // Pan left
      m_current_view.min().x() -= m_current_view.width()*factor;
      m_current_view.max().x() -= m_current_view.width()*factor;
      refreshPixmap();
      break;
    case Qt::Key_Right: // Pan right
      m_current_view.min().x() += m_current_view.width()*factor;
      m_current_view.max().x() += m_current_view.width()*factor;
      refreshPixmap();
      break;
    case Qt::Key_Up: // Pan up
      m_current_view.min().y() -= m_current_view.height()*factor;
      m_current_view.max().y() -= m_current_view.height()*factor;
      refreshPixmap();
      break;
    case Qt::Key_Down: // Pan down
      m_current_view.min().y() += m_current_view.height()*factor;
      m_current_view.max().y() += m_current_view.height()*factor;
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
    m_ContextMenu->popup(mapToGlobal(QPoint(x,y)));
    return;
  }

  void MainWidget::viewMatches(bool view_matches){
    // Complain if there are multiple images and matches was turned on
    if ((m_images.size() != 1) && view_matches) {
      popUp("Must have just one image in each window to view matches.");

      refreshPixmap();
      return;
    }

    m_viewMatches = view_matches;
    refreshPixmap();
  }

  void MainWidget::addMatchPoint(){

    if (m_image_id >= (int)m_matches.size()) {
      popUp("Number of existing matches is corrupted. Cannot add matches.");
      return;
    }

    if (m_images.size() != 1) {
      popUp("Must have just one image in each window to add matches.");
      return;
    }

    // We will start with an interest point in the left-most image,
    // and add matches to it in the other images.
    // - Must add the point in each image from left to right, one point at a time.
    size_t curr_pts = m_matches[m_image_id].size(); // # Pts from current image
    bool is_good = true;
    for (int i = 0; i < m_image_id; i++) { // Look through lower-id images
      if (m_matches[i].size() != curr_pts+1) { // They should all have one more IP
        is_good = false;
      }
    }
    // Check all higher-id images, they should have the same # Pts as this one.
    for (int i = m_image_id+1; i < (int)m_matches.size(); i++) {
      if (m_matches[i].size() != curr_pts) {
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
  }

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
    emit refreshAllMatches();
  }

}} // namespace vw::gui
