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


/// \file GuiUtilities.cc
///
///

#include <string>
#include <vector>
#include <QtGui>
#include <QContextMenuEvent>

#include <vw/Image.h>
#include <vw/FileIO.h>
#include <vw/Math/EulerAngles.h>
#include <vw/Image/Algorithms.h>
#include <vw/Cartography/GeoTransform.h>
#include <vw/tools/hillshade.h>
#include <vw/Core/RunOnce.h>
#include <asp/GUI/GuiUtilities.h>

using namespace vw;
using namespace vw::gui;
using namespace std;

namespace vw { namespace gui {

vw::RunOnce temporary_files_once = VW_RUNONCE_INIT;
boost::shared_ptr<TemporaryFiles> temporary_files_ptr;
void init_temporary_files() {
  temporary_files_ptr = boost::shared_ptr<TemporaryFiles>(new TemporaryFiles());
}

TemporaryFiles& temporary_files() {
  temporary_files_once.run( init_temporary_files );
  return *temporary_files_ptr;
}


// Get a filename by simply replacing current extension with given suffix
std::string filename_from_suffix1(std::string const& input_file,
                                  std::string const& suffix){
  std::string prefix = vw::prefix_from_filename(input_file);
  std::string output_file = prefix + suffix;
  return output_file;
}

// Get a filename by  replacing current extension with given suffix,
// and making the file be in the current directory.
std::string filename_from_suffix2(std::string const& input_file,
                                  std::string const& suffix){
  std::string prefix = vw::prefix_from_filename(input_file);
  boost::filesystem::path p(input_file);
  prefix = p.stem().string();
  std::string output_file = prefix + suffix;
  return output_file;
}


bool overwrite_if_no_good(std::string const& input_file,
                          std::string const& output_file,
                          int cols, int rows){

  fs::path input_path(input_file);
  std::time_t input_time = fs::last_write_time(input_path);

  bool overwrite = true;
  if (fs::exists(output_file)) {
    try{
      DiskImageView<double> curr(output_file);
      fs::path output_path(output_file);
      std::time_t output_time = fs::last_write_time(output_path);
      if (output_time < input_time){
        overwrite = true; // too old
        return overwrite;
      }

      if (cols >= 0 && rows >= 0) {
        if (curr.cols() == cols && curr.rows() == rows){
          overwrite = false;
        }else{
          overwrite = true;
        }
      }else{
        overwrite = false;
      }

    }catch(...){
      overwrite = true;
    }
  }

  return overwrite;
}

void popUp(std::string msg){
  QMessageBox msgBox;
  msgBox.setText(msg.c_str());
  msgBox.exec();
  return;
}

bool getStringFromGui(QWidget * parent,
		      std::string title, std::string description,
		      std::string inputStr,
		      std::string & outputStr){ // output
  outputStr = "";

  bool ok = false;
  QString text = QInputDialog::getText(parent, title.c_str(), description.c_str(),
				       QLineEdit::Normal, inputStr.c_str(),
				       &ok);

  if (ok) outputStr = text.toStdString();

  return ok;
}

bool supplyOutputPrefixIfNeeded(QWidget * parent, std::string & output_prefix){

  if (output_prefix != "") return true;

  bool ans = getStringFromGui(parent,
			      "Enter the output prefix to use for the interest point match file.",
			      "Enter the output prefix to use for the interest point match file.",
			      "",
				output_prefix);

  if (ans)
    vw::create_out_dir(output_prefix);

  return ans;
}

std::string fileDialog(std::string title, std::string start_folder){

  std::string fileName = QFileDialog::getOpenFileName(0,
                                      title.c_str(),
                                      start_folder.c_str()).toStdString();

  return fileName;
}

QRect bbox2qrect(BBox2 const& B){
  // Need some care here, an empty BBox2 can have its corners
  // as the largest double, which can cause overflow.
  if (B.empty()) return QRect();
  return QRect(round(B.min().x()), round(B.min().y()),
               round(B.width()), round(B.height()));
}

bool write_hillshade(asp::BaseOptions const& opt,
                     std::string const& input_file,
                     std::string & output_file) {

  // Sanity check: Must have a georeference
  cartography::GeoReference georef;
  bool has_georef = vw::cartography::read_georeference(georef, input_file);
  if (!has_georef) {
    popUp("No georeference present in: " + input_file + ".");
    return false;
  }

  // TODO: Expose these to the user
  int elevation = 20;
  int azimuth   = 300;

  double scale       = 0.0;
  double blur_sigma  = std::numeric_limits<double>::quiet_NaN();
  double nodata_val  = std::numeric_limits<double>::quiet_NaN();
  vw::read_nodata_val(input_file, nodata_val);
  std::string suffix = "_hillshade.tif";

  output_file = filename_from_suffix1(input_file, suffix);
  try {
    DiskImageView<float> input(input_file);
    try{
      bool will_write = overwrite_if_no_good(input_file, output_file,
                                             input.cols(), input.rows());
      if (will_write){
        vw_out() << "Writing: " << output_file << std::endl;
        vw::do_multitype_hillshade(input_file, output_file, azimuth, elevation, scale,
                                   nodata_val, blur_sigma);
      }
    }catch(...){
      // Failed to write, presumably because we have no write access.
      // Write the file in the current dir.
      vw_out() << "Failed to write: " << output_file << "\n";
      output_file = filename_from_suffix2(input_file, suffix);
      bool will_write = overwrite_if_no_good(input_file, output_file,
                                             input.cols(), input.rows());
      if (will_write){
        vw_out() << "Writing: " << output_file << std::endl;
        vw::do_multitype_hillshade(input_file,  output_file, azimuth, elevation, scale,
                                   nodata_val, blur_sigma);
      }
    }
  } catch ( const Exception& e ) {
    popUp(e.what());
    return false;
  }

  return true;
}

Vector2 point_to_pixel(Vector2 const& proj_pt2,
                       double lon_offset,
                       cartography::GeoReference const& georef1,
                       cartography::GeoReference const& georef2){
  Vector2 out_pt;
  Vector2 L(lon_offset, 0);
  return georef1.lonlat_to_pixel(georef2.point_to_lonlat(proj_pt2) - L);
}

// TODO: Carefully integrate this into GeoTransform. An example can be found in GeoReference.cc.
// The reverse of pixel_to_point_bbox. Given georef2 and a box in
// projected coordinates of this georef, convert it to a pixel box with georef1.
BBox2 point_to_pixel_bbox(BBox2 point_box2,
                          double lon_offset,
                          cartography::GeoReference const& georef1,
                          cartography::GeoReference const& georef2){

  // Ensure we don't get incorrect results for empty boxes with
  // strange corners.
  if (point_box2.empty())
    return point_box2;

  BBox2 out_box;

  double minx = point_box2.min().x(), maxx = point_box2.max().x();
  double miny = point_box2.min().y(), maxy = point_box2.max().y();

  // Compensate by the fact that lon1 and lon2 could be off
  // by 360 degrees.
  Vector2 L(lon_offset, 0);

  // At the poles this won't be enough, more thought is needed.
  int num = 100;
  for (int i = 0; i <= num; i++) {
    double r = double(i)/num;

    // left edge
    Vector2 P2 = Vector2(minx, miny + r*(maxy-miny));
    out_box.grow(point_to_pixel(P2, lon_offset, georef1, georef2));

    // right edge
    P2 = Vector2(maxx, miny + r*(maxy-miny));
    out_box.grow(point_to_pixel(P2, lon_offset, georef1, georef2));

    // bottom edge
    P2 = Vector2(minx + r*(maxx-minx), miny);
    out_box.grow(point_to_pixel(P2, lon_offset, georef1, georef2));

    // top edge
    P2 = Vector2(minx + r*(maxx-minx), maxy);
    out_box.grow(point_to_pixel(P2, lon_offset, georef1, georef2));

    // diag1
    P2 = Vector2(minx + r*(maxx-minx), miny + r*(maxy-miny));
    out_box.grow(point_to_pixel(P2, lon_offset, georef1, georef2));

    // diag2
    P2 = Vector2(maxx - r*(maxx-minx), miny + r*(maxy-miny));
    out_box.grow(point_to_pixel(P2, lon_offset, georef1, georef2));
  }

  return grow_bbox_to_int(out_box);
}

// Given an image with georef1 and a portion of its pixels in
// pixel_box1, find the bounding box of pixel_box1 in projected
// point units for georef2.
BBox2 forward_pixel_to_point_bbox(BBox2 pixel_box1,
                                  double lon_offset,
                                  cartography::GeoReference const& georef1,
                                  cartography::GeoReference const& georef2){

  cartography::GeoTransform T(georef1, georef2);
  T.set_offset(Vector2(lon_offset, 0));
  return T.forward_pixel_to_point_bbox(pixel_box1);
}


void imageData::read(std::string const& name_in, asp::BaseOptions const& opt,
                     bool use_georef){
  m_opt = opt;
  name = name_in;

  m_lon_offset = 0; // will be adjusted later

  int top_image_max_pix = 1000*1000;
  int subsample = 4;
  img = DiskImagePyramidMultiChannel(name, m_opt, top_image_max_pix, subsample);

  has_georef = vw::cartography::read_georeference(georef, name);

  if (use_georef && !has_georef){
    popUp("No georeference present in: " + name + ".");
    vw_throw(ArgumentErr() << "Missing georeference.\n");
  }

  image_bbox = BBox2(0, 0, img.cols(), img.rows());
  if (use_georef && has_georef)
    lonlat_bbox = georef.pixel_to_lonlat_bbox(image_bbox);
}

vw::Vector2 QPoint2Vec(QPoint const& qpt) {
  return vw::Vector2(qpt.x(), qpt.y());
}

QPoint Vec2QPoint(vw::Vector2 const& V) {
  return QPoint(round(V.x()), round(V.y()));
}

// Allow the user to choose which files to hide/show in the GUI.
// User's choice will be processed by MainWidget::showFilesChosenByUser().
chooseFilesDlg::chooseFilesDlg(QWidget * parent):
  QWidget(parent){

  setWindowModality(Qt::ApplicationModal);

  int spacing = 0;

  QVBoxLayout * vBoxLayout = new QVBoxLayout(this);
  vBoxLayout->setSpacing(spacing);
  vBoxLayout->setAlignment(Qt::AlignLeft);

  // The layout having the file names. It will be filled in
  // dynamically later.
  m_filesTable = new QTableWidget();

  m_filesTable->horizontalHeader()->hide();
  m_filesTable->verticalHeader()->hide();

  vBoxLayout->addWidget(m_filesTable);

  return;
}

chooseFilesDlg::~chooseFilesDlg(){}

void chooseFilesDlg::chooseFiles(const std::vector<imageData> & images){

  // See the top of this file for documentation.

  int numFiles = images.size();
  int numCols = 2;
  m_filesTable->setRowCount(numFiles);
  m_filesTable->setColumnCount(numCols);

  for (int fileIter = 0; fileIter < numFiles; fileIter++){

    QTableWidgetItem *item;
    item = new QTableWidgetItem(1);
    item->data(Qt::CheckStateRole);
    item->setCheckState(Qt::Checked);
    m_filesTable->setItem(fileIter, 0, item);

    string fileName = images[fileIter].name;
    item = new QTableWidgetItem(fileName.c_str());
    item->setFlags(Qt::NoItemFlags);
    item->setForeground(QColor::fromRgb(0,0,0));
    m_filesTable->setItem(fileIter, numCols - 1, item);

  }

  QStringList rowNamesList;
  for (int fileIter = 0; fileIter < numFiles; fileIter++) rowNamesList << "";
  m_filesTable->setVerticalHeaderLabels(rowNamesList);

  QStringList colNamesList;
  for (int colIter = 0; colIter < numCols; colIter++) colNamesList << "";
  m_filesTable->setHorizontalHeaderLabels(colNamesList);
  QTableWidgetItem * hs = m_filesTable->horizontalHeaderItem(0);
  hs->setBackground(QBrush(QColor("lightgray")));

  m_filesTable->setSelectionMode(QTableWidget::ExtendedSelection);
  string style = string("QTableWidget::indicator:unchecked ")
    + "{background-color:white; border: 1px solid black;}; " +
    "selection-background-color: rgba(128, 128, 128, 40);";

  m_filesTable->setSelectionMode(QTableWidget::NoSelection);

  m_filesTable->setStyleSheet(style.c_str());
  m_filesTable->resizeColumnsToContents();
  m_filesTable->resizeRowsToContents();

  // The processing of user's choice happens in MainWidget::showFilesChosenByUser()

  return;
}


DiskImagePyramidMultiChannel::DiskImagePyramidMultiChannel(std::string const& base_file,
                             asp::BaseOptions const& opt,
                             int top_image_max_pix,
                             int subsample):m_opt(opt),
                                                m_num_channels(0),
                                                m_rows(0), m_cols(0),
                                                m_type(UNINIT){
  if (base_file == "") return;

  m_num_channels = get_num_channels(base_file);
  if (m_num_channels == 1) {
    // Single channel image with float pixels.
    m_img_ch1_double = DiskImagePyramid<double>(base_file, m_opt);
    m_rows = m_img_ch1_double.rows();
    m_cols = m_img_ch1_double.cols();
    m_type = CH1_DOUBLE;
  }else if (m_num_channels == 2){
    // uint8 image with an alpha channel. Ignore the alpha channel.
    m_img_ch1_uint8 = DiskImagePyramid< Vector<vw::uint8, 1> >(base_file, m_opt);
    m_num_channels = 1; // we read only 1 channel
    m_rows = m_img_ch1_uint8.rows();
    m_cols = m_img_ch1_uint8.cols();
    m_type = CH1_UINT8;
  } else if (m_num_channels == 3 || m_num_channels == 4) {
    // RGB image with three uint8 channels and perhaps an
    // alpha channel which we ignore.
    m_img_ch3_uint8 = DiskImagePyramid< Vector<vw::uint8, 3> >(base_file, m_opt);
    m_num_channels = 3; // we read only 3 channels
    m_rows = m_img_ch3_uint8.rows();
    m_cols = m_img_ch3_uint8.cols();
    m_type = CH3_UINT8;
  }else{
    vw_throw( ArgumentErr() << "Unsupported image with "
              << m_num_channels << " bands.\n");
  }
}

void DiskImagePyramidMultiChannel::getImageClip(double scale_in, vw::BBox2i region_in,
                  bool highlight_nodata,
                  QImage & qimg, double & scale_out, vw::BBox2i & region_out) {

  bool scale_pixels = (m_type == CH1_DOUBLE);

  if (m_type == CH1_DOUBLE) {
    m_img_ch1_double.getImageClip(scale_in, region_in,
                                  highlight_nodata, scale_pixels, qimg,
                                  scale_out, region_out);
  } else if (m_type == CH1_UINT8) {
    m_img_ch1_uint8.getImageClip(scale_in, region_in,
                                 highlight_nodata, scale_pixels, qimg,
                                 scale_out, region_out);
  } else if (m_type == CH3_UINT8) {
    m_img_ch3_uint8.getImageClip(scale_in, region_in,
                                 highlight_nodata, scale_pixels, qimg,
                                 scale_out, region_out);
  }else{
    vw_throw( ArgumentErr() << "Unsupported image with "
              << m_num_channels << " bands\n");
  }
}

double DiskImagePyramidMultiChannel::get_value_as_double( int32 x, int32 y) const {
  if (m_type == CH1_DOUBLE) {
    return m_img_ch1_double.bottom()(x, y, 0);
  }else if (m_type == CH1_UINT8){
    return m_img_ch1_uint8.bottom()(x, y, 0)[0];
  }else{
    vw_throw( ArgumentErr() << "Unsupported image with " << m_num_channels << " bands\n");
  }
  return 0;
}



void PointList::push_back(std::list<vw::Vector2> pts) {
  std::list<vw::Vector2>::iterator iter  = pts.begin();
  while (iter != pts.end()) {
    m_points.push_back(*iter);
    ++iter;
  }
}

}} // namespace vw::gui
