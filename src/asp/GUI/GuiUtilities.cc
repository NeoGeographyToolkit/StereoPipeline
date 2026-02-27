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

// TODO(oalexan1): This is very slow to compile. Need to figure out
// why. Could be the bundle adjustment logic. Or contouring, or
// shapefile logic, or DiskImagePyramid read from GuiUtilities.h.
// These may need to be moved to separate files.

#include <asp/GUI/GuiUtilities.h>
#include <asp/GUI/GuiGeom.h>
#include <asp/Core/StereoSettings.h>
#include <asp/Core/PointUtils.h>

#include <vw/Image/Algorithms.h>
#include <vw/Cartography/GeoTransform.h>
#include <vw/Cartography/Hillshade.h>
#include <vw/Core/RunOnce.h>
#include <vw/BundleAdjustment/ControlNetworkLoader.h>
#include <vw/InterestPoint/MatcherIO.h>
#include <vw/Geometry/dPoly.h>
#include <vw/Cartography/shapeFile.h>
#include <vw/Core/Stopwatch.h>
#include <vw/FileIO/FileTypes.h>

#include <QPolygon>
#include <QtGui>
#include <QtWidgets>
#include <ogrsf_frmts.h>

// For contours
#include <opencv2/imgproc.hpp>

#include <string>
#include <vector>

using namespace vw;
using namespace vw::geometry;

namespace asp {

bool isPolyZeroDim(const QPolygon & pa){
  
  int numPts = pa.size();
  for (int s = 1; s < numPts; s++){
    if (pa[0] != pa[s]) return false;
  }
  
  return true;
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

  std::string caption = "Enter the output prefix to use for the interest point match file.";
  bool ans = getStringFromGui(parent, caption, caption, "", output_prefix);

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
  if (B.empty()) 
    return QRect();
  return QRect(round(B.min().x()), round(B.min().y()),
               round(B.width()), round(B.height()));
}

bool write_hillshade(vw::GdalWriteOptions const& opt,
                     bool have_gui,
                     double azimuth, double elevation,
                     std::string const& input_file,
                     std::string      & output_file) {

  // Sanity check: Must have a georeference
  cartography::GeoReference georef;
  bool has_georef = vw::cartography::read_georeference(georef, input_file);
  if (!has_georef) {
    popUp("No georeference present in: " + input_file + ".");
    return false;
  }

  double scale       = 0.0;
  double blur_sigma  = std::numeric_limits<double>::quiet_NaN();
  double nodata_val  = std::numeric_limits<double>::quiet_NaN();
  vw::read_nodata_val(input_file, nodata_val);
  std::ostringstream oss;
  oss << "_hillshade_a" << azimuth << "_e" << elevation << ".tif"; 
  std::string suffix = oss.str();

  output_file = vw::mosaic::filename_from_suffix1(input_file, suffix);
  bool align_light_to_georef = false;
  try {
    DiskImageView<float> input(input_file);
    // TODO(oalexan1): Factor out repeated logic below.
    try {
      bool will_write = vw::mosaic::overwrite_if_no_good(input_file, output_file,
                                             input.cols(), input.rows());
      if (will_write) {
        vw_out() << "Writing: " << output_file << "\n";
        vw::cartography::do_multitype_hillshade(input_file, output_file, azimuth, elevation, scale,
                                                nodata_val, blur_sigma, align_light_to_georef, opt);
      }
    } catch(...) {
      // Failed to write, presumably because we have no write access.
      // Write the file in the current dir.
      vw_out() << "Failed to write: " << output_file << "\n";
      output_file = vw::mosaic::filename_from_suffix2(input_file, suffix);
      bool will_write = vw::mosaic::overwrite_if_no_good(input_file, output_file,
                                             input.cols(), input.rows());
      if (will_write) {
        vw_out() << "Writing: " << output_file << "\n";
        vw::cartography::do_multitype_hillshade(input_file,  output_file,
                                                azimuth, elevation, scale,
                                                nodata_val, blur_sigma,
                                                align_light_to_georef, opt);
      }
    }
  } catch (const Exception& e) {
    if (!have_gui) 
      vw_out() << e.what() << "\n";
    else
      popUp(e.what());
    return false;
  }
  
  return true;
}

// TODO(oalexan1): The 0.5 bias may be the wrong thing to do. Need to test
// more overlaying an image which is above threshold in a rectangular region
// and below it outside that region
void contour_image(DiskImagePyramidMultiChannel const& img,
                   vw::cartography::GeoReference const & georef,
                   double threshold,
                   std::vector<vw::geometry::dPoly> & polyVec) {
  
  std::vector<std::vector<cv::Point>> contours;
  std::vector<cv::Vec4i> hierarchy;

  // Create the OpenCV matrix. We will have issues for huge images.
  cv::Mat cv_img = cv::Mat::zeros(img.cols(), img.rows(), CV_8UC1);
  
  // Form the binary image. Values above threshold become 1, and less
  // than equal to the threshold become 0.
  long long int num_pixels_above_thresh = 0;
  for (int col = 0; col < img.cols(); col++) {
    for (int row = 0; row < img.rows(); row++) {
      uchar val = (std::max(img.get_value_as_double(col, row), threshold) - threshold > 0);
      cv_img.at<uchar>(col, row) = val;
      if (val > 0) 
        num_pixels_above_thresh++;
    }    
  }

  // Add the contour to the list of polygons to display
  polyVec.clear();
  polyVec.resize(1);
  vw::geometry::dPoly & poly = polyVec[0]; // alias

  if (num_pixels_above_thresh == 0) 
    return; // Return early, nothing to do
  
  // Find the contour
  cv::findContours(cv_img, contours, hierarchy, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE);

  // Copy the polygon for export
  for (size_t k = 0; k < contours.size(); k++) {

    if (contours[k].empty()) 
      continue;

    // Copy from float to double
    std::vector<double> xv(contours[k].size()), yv(contours[k].size());
    for (size_t vIter = 0; vIter < contours[k].size(); vIter++) {

      // We would like the contour to go through the center of the
      // pixels not through their upper-left corners. Hence add 0.5.
      double bias = 0.5;
      
      // Note how we flip x and y, because in our GUI the first
      // coordinate is the column.
      Vector2 S(contours[k][vIter].y + bias, contours[k][vIter].x + bias);

      // The GUI expects the contours to be in georeferenced coordinates
      S = georef.pixel_to_point(S);
      
      xv[vIter] = S.x();
      yv[vIter] = S.y();
    }
    
    bool isPolyClosed = true;
    std::string color = "green";
    std::string layer = "0";
    poly.appendPolygon(contours[k].size(), &xv[0], &yv[0],  
                       isPolyClosed, color, layer);
  }
}

// This will tweak the georeference so that point_to_pixel() is the identity.
bool read_georef_from_shapefile(vw::cartography::GeoReference & georef,
                                std::string const& file){

  if (!vw::has_shp_extension(file))
    vw_throw(ArgumentErr() << "Expecting a shapefile as input, got: " << file << ".\n");
  
  bool has_georef = false; // will change
  std::vector<vw::geometry::dPoly> polyVec;
  std::string poly_color;
  read_shapefile(file, poly_color, has_georef, georef, polyVec);
  
  return has_georef;
}
  
bool read_georef_from_image_or_shapefile(vw::cartography::GeoReference & georef,
                                         std::string const& file){

  if (vw::has_shp_extension(file)) 
    return read_georef_from_shapefile(georef, file);
  
  return vw::cartography::read_georeference(georef, file);
}


vw::Vector2 QPoint2Vec(QPoint const& qpt) {
  return vw::Vector2(qpt.x(), qpt.y());
}

vw::Vector2 QPointF2Vec(QPointF const& qpt) {
  return vw::Vector2(qpt.x(), qpt.y());
}

QPoint Vec2QPoint(vw::Vector2 const& V) {
  return QPoint(round(V.x()), round(V.y()));
}

void PointList::push_back(std::list<vw::Vector2> pts) {
  std::list<vw::Vector2>::iterator iter  = pts.begin();
  while (iter != pts.end()) {
    m_points.push_back(*iter);
    ++iter;
  }
}

// See if we are in the mode where the images are displayed side-by-side with a
// dialog to choose which ones to display.
bool sideBySideWithDialog() {
  return (asp::stereo_settings().pairwise_matches       ||
          asp::stereo_settings().pairwise_clean_matches ||
          asp::stereo_settings().view_several_side_by_side);
}

bool previewOrSideBySideWithDialog() {
  return asp::stereo_settings().preview || sideBySideWithDialog();
}
  
// Turn off any such side-by-side logic
void setNoSideBySideWithDialog() {
  asp::stereo_settings().pairwise_matches          = false;
  asp::stereo_settings().pairwise_clean_matches    = false;
  asp::stereo_settings().view_several_side_by_side = false;
}

  
} // namespace asp
