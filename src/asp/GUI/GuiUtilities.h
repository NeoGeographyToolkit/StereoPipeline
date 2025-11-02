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

/// \file GuiUtilities.h
///
/// Low-level GUI logic
///
#ifndef __STEREO_GUI_GUI_UTILITIES_H__
#define __STEREO_GUI_GUI_UTILITIES_H__

// ASP
#include <asp/GUI/DiskImagePyramidMultiChannel.h>

// Vision Workbench
#include <vw/Core/Thread.h>
#include <vw/Core/Log.h>
#include <vw/Image/ImageViewRef.h>
#include <vw/Image/ImageView.h>
#include <vw/FileIO/DiskImageView.h>
#include <vw/FileIO/DiskImageUtils.h>
#include <vw/Math/BBox.h>
#include <vw/Math/Vector.h>
#include <vw/Cartography/GeoReference.h>
#include <vw/Cartography/GeoReferenceUtils.h>
#include <vw/InterestPoint/InterestData.h>
#include <vw/Geometry/dPoly.h>
#include <vw/Image/AntiAliasing.h>

#include <boost/filesystem/path.hpp>
#include <boost/filesystem.hpp>
#include <boost/foreach.hpp> // Must be before Qt headers
#include <boost/mpl/or.hpp>
#include <omp.h>

#include <QWidget>
#include <QPoint>

#include <string>
#include <vector>
#include <list>
#include <set>

class QMouseEvent;
class QWheelEvent;
class QPoint;
class QResizeEvent;
class QTableWidget;
class QContextMenuEvent;
class QMenu;
class QPolygon;

namespace asp {

  enum DisplayMode {REGULAR_VIEW, HILLSHADED_VIEW, COLORIZED_VIEW,
                    HILLSHADE_COLORIZED_VIEW,
                    THRESHOLDED_VIEW};
  
  // TODO(oalexan1): Remove this def out of this header file
  namespace fs = boost::filesystem;

  bool isPolyZeroDim(const QPolygon & pa);

  bool getStringFromGui(QWidget * parent,
			std::string title, std::string description,
			std::string inputStr,
			std::string & outputStr);

  bool supplyOutputPrefixIfNeeded(QWidget * parent, std::string & output_prefix);

  /// Pop-up a window to have the user select a file
  std::string fileDialog(std::string title, std::string start_folder="");

  // Flip a point and a box in y
  inline vw::Vector2 flip_in_y(vw::Vector2 const& P){
    return vw::Vector2(P.x(), -P.y());
  }
  inline vw::BBox2 flip_in_y(vw::BBox2 const& B){
    vw::BBox2 R = B;
    R.min().y() = -B.max().y();
    R.max().y() = -B.min().y();
    return R;
  }

  // Return true if the extension is .csv or .txt
  bool hasCsv(std::string const& fileName);
  
  /// A class to keep all data associated with an image file
  class imageData {
  public:
    std::string      name, hillshaded_name, thresholded_name, colorized_name;
    vw::GdalWriteOptions m_opt;
    bool             has_georef;
    vw::cartography::GeoReference georef;
    vw::BBox2        image_bbox;
    vw::Vector2      val_range;
    bool             loaded_regular, loaded_hillshaded,
      loaded_thresholded, loaded_colorized; // if the image was loaded
    // There are several display modes. The one being shown is
    // determined by m_display_mode. Store the corresponding
    // image in one of the structures below
    DisplayMode m_display_mode;
    DiskImagePyramidMultiChannel img;
    DiskImagePyramidMultiChannel hillshaded_img;
    DiskImagePyramidMultiChannel thresholded_img;
    DiskImagePyramidMultiChannel colorized_img;
    
    std::vector<vw::geometry::dPoly> polyVec; // a shapefile
    std::string color; // poly color
    std::string style; // plotting style
    std::string colormap; // colormap style
    bool colorbar; // if a given image must be colorized
    
    // Scattered data to be plotted at (x, y) location with z giving
    // the intensity. May be colorized.
    std::vector<vw::Vector3> scattered_data;
    
    imageData(): m_display_mode(REGULAR_VIEW), has_georef(false),
                 loaded_regular(false), loaded_hillshaded(false),
                 loaded_thresholded(false), loaded_colorized(false),
                 m_isPoly(false), m_isCsv(false), colorbar(false) {}
    
    /// Read an image from disk into img and set the other variables.
    void read(std::string const& image, vw::GdalWriteOptions const& opt,
              DisplayMode display_mode = REGULAR_VIEW,
              std::map<std::string, std::string> const& properties =
              std::map<std::string, std::string>(), bool delay_loading = false);

    // The actual loading happens here
    void load();
    
    // Save the polygons to a plain text file
    void writePoly(std::string const& polyFile);

    bool m_isPoly, m_isCsv;  

private:
    // These are very slow if used per pixel, so we cache their results in
    // member variables. Never call these directly.
    bool isPolyInternal(std::string const& name, std::string const& style) const;
    bool isCsvInternal(std::string const& name, std::string const& style) const;

  };

  /// Convert a QRect object to a BBox2 object.
  inline vw::BBox2 qrect2bbox(QRect const& R){
    return vw::BBox2(vw::Vector2(R.left(), R.top()), vw::Vector2(R.right(), R.bottom()));
  }

  /// Convert a BBox2 object to a QRect object.
  QRect bbox2qrect(vw::BBox2 const& B);

  /// Save a hillshaded file
  bool write_hillshade(vw::GdalWriteOptions const& opt,
                       bool have_gui,
                       double azimuth, double elevation,
                       std::string const& input_file,
                       std::string      & output_file);

  // Given an image, and an input file name, modify the filename using
  // a prefix. Write the image to that filename. If that fails, create
  // instead the filename in the current directory. Return the name
  // of the output file.
  template<class PixelT>
  std::string write_in_orig_or_curr_dir(vw::GdalWriteOptions const& opt,
                                        vw::ImageViewRef<PixelT> & image,
                                        std::string const& input_file,
                                        std::string const& suffix,
                                        bool has_georef,
                                        vw::cartography::GeoReference const & georef,
                                        bool has_nodata,
                                        double nodata_val);


  // Find the closest edge in a given vector of polygons to a given point.
  void findClosestPolyEdge(// inputs
			   double x0, double y0,
			   std::vector<vw::geometry::dPoly> const& polyVec,
			   // outputs
			   int & polyVecIndex,
			   int & polyIndexInCurrPoly,
			   int & vertIndexInCurrPoly,
			   double & minX, double & minY,
			   double & minDist);
  
  // This will tweak the georeference so that point_to_pixel() is the identity.
  bool read_georef_from_shapefile(vw::cartography::GeoReference & georef,
				  std::string const& file);
  
  bool read_georef_from_image_or_shapefile(vw::cartography::GeoReference & georef,
					   std::string const& file);

  // Create contours from given image
  void contour_image(DiskImagePyramidMultiChannel const& img,
                     vw::cartography::GeoReference const & georef,
                     double threshold,
                     std::vector<vw::geometry::dPoly> & polyVec);
  
  // QT conversion functions
  vw::Vector2 QPoint2Vec(QPoint      const& qpt);
  vw::Vector2 QPointF2Vec(QPointF    const& qpt);
  QPoint      Vec2QPoint(vw::Vector2 const& V  );

  /// A simple class for keeping track of crosshair locations and colors.
  /// - Contains a list of points and the color applied to them.
  class PointList {
    std::list<vw::Vector2> m_points;
    vw::Vector3 m_color;
  public:
    PointList(vw::Vector3 const& color) : m_color(color) {}
    PointList(std::list<vw::Vector2> const& points, vw::Vector3 const& color) :
      m_color(color) {
      this->push_back(points);
    }

    std::list<vw::Vector2> const& points() const { return m_points; }
    vw::Vector3 color() const { return m_color; }

    void push_back(vw::Vector2 pt) { m_points.push_back(pt); }
    void push_back(std::list<vw::Vector2> pts);
  };

  // For each pairs if image indices, store the match file and interest points.
  // Load these on-demand, and once loaded, keep them in memory.
  struct pairwiseMatchList {
    std::map<std::pair<int, int>, std::string> match_files;
    std::map<std::pair<int, int>, std::pair<std::vector<vw::ip::InterestPoint>, std::vector<vw::ip::InterestPoint>>> matches;

    // We will copy here only the ip that need to be shown for the moment
    std::vector<std::vector<vw::ip::InterestPoint>> ip_to_show;
  };

template<class PixelT>
std::string write_in_orig_or_curr_dir(vw::GdalWriteOptions const& opt,
                                      vw::ImageViewRef<PixelT> & image,
                                      std::string const& input_file,
                                      std::string const& suffix,
                                      bool has_georef,
                                      vw::cartography::GeoReference const & georef,
                                      bool has_nodata,
                                      double nodata_val) {

  std::string output_file = vw::mosaic::filename_from_suffix1(input_file, suffix);
  vw::TerminalProgressCallback tpc("asp", ": ");
  vw::vw_out() << "Writing: " << output_file << std::endl;
  try {
    vw::cartography::block_write_gdal_image(output_file, image, has_georef, georef,
                                has_nodata, nodata_val, opt, tpc);
  } catch(...) {
    // Failed to write, presumably because we have no write access.
    // Write the file in the current dir.
    vw::vw_out() << "Failed to write: " << output_file << "\n";
    output_file = vw::mosaic::filename_from_suffix2(input_file, suffix);
    vw::vw_out() << "Writing: " << output_file << std::endl;
    vw::cartography::block_write_gdal_image(output_file, image, has_georef, georef,
                                has_nodata, nodata_val, opt, tpc);
  }
  return output_file;
}

// See if we are in the mode where the images are displayed side-by-side with a
// dialog to choose which ones to display.
bool sideBySideWithDialog();
bool previewOrSideBySideWithDialog();
void setNoSideBySideWithDialog(); // turn off such logic

} // namespace asp

#endif  // __STEREO_GUI_GUI_UTILITIES_H__
