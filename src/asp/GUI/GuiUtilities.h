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

// TODO(oalexan1): Reorder the headers, with low-level ones at the bottom.
// But then compilation fails.
#include <boost/filesystem/path.hpp>
#include <boost/filesystem.hpp>
#include <boost/mpl/or.hpp>
#if !__APPLE__
#include <omp.h>
#endif

// Qt
#include <QWidget>
#include <QPoint>

// ASP
#include <asp/Core/Common.h>
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

namespace vw { namespace gui {

  enum DisplayMode {REGULAR_VIEW, HILLSHADED_VIEW, COLORIZED_VIEW, HILLSHADE_COLORIZED_VIEW,
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
  inline Vector2 flip_in_y(Vector2 const& P){
    return Vector2(P.x(), -P.y());
  }
  inline BBox2 flip_in_y(BBox2 const& B){
    BBox2 R = B;
    R.min().y() = -B.max().y();
    R.max().y() = -B.min().y();
    return R;
  }

  // Return true if the extension is .csv or .txt
  bool hasCsv(std::string const& fileName);
  
  /// A class to keep all data associated with an image file
  struct imageData{
    std::string      name, hillshaded_name, thresholded_name, colorized_name;
    vw::GdalWriteOptions m_opt;
    bool             has_georef;
    vw::cartography::GeoReference georef;
    vw::BBox2        image_bbox;
    vw::Vector2      val_range;
    
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
    
    // Scattered data to be plotted at (x, y) location with z giving
    // the intensity. May be colorized.
    std::vector<vw::Vector3> scattered_data;
    
    imageData(): m_display_mode(REGULAR_VIEW) {}
    
    /// Load an image from disk into img and set the other variables.
    void read(std::string const& image, vw::GdalWriteOptions const& opt,
              DisplayMode display_mode = REGULAR_VIEW,
              std::map<std::string, std::string> const& properties =
              std::map<std::string, std::string>());

    bool isPoly() const { return asp::has_shp_extension(name); }
    bool isCsv()  const { return vw::gui::hasCsv(name); }
  };

  /// Convert a QRect object to a BBox2 object.
  inline BBox2 qrect2bbox(QRect const& R){
    return BBox2(Vector2(R.left(), R.top()), Vector2(R.right(), R.bottom()));
  }

  /// Convert a BBox2 object to a QRect object.
  QRect bbox2qrect(BBox2 const& B);

  /// Save a hillshaded file
  bool write_hillshade(vw::GdalWriteOptions const& opt,
                       double azimuth, double elevation,
                       std::string const& input_file,
                       std::string      & output_file);

  // Given an image, and an input file name, modify the filename using
  // a prefix. Write the image to that filename. If that fails, create
  // instead the filename in the current directory. Return the name
  // of the output file.
  template<class PixelT>
  std::string write_in_orig_or_curr_dir(vw::GdalWriteOptions const& opt,
                                        ImageViewRef<PixelT> & image,
                                        std::string const& input_file,
                                        std::string const& suffix,
                                        bool has_georef,
                                        vw::cartography::GeoReference const & georef,
                                        bool has_nodata,
                                        double nodata_val);

  // Find the closest point in a given vector of polygons to a given point.
  void findClosestPolyVertex(// inputs
			     double x0, double y0,
			     std::vector<vw::geometry::dPoly> const& polyVec,
			     // outputs
			     int & polyVecIndex,
			     int & polyIndexInCurrPoly,
			     int & vertIndexInCurrPoly,
			     double & minX, double & minY,
			     double & minDist);

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

  /// Helper class to keep track of all the matching interest points
  /// - Each image must have the same number of interest points
  ///   but in some situations some of the points can be flagged as invalid.
  class MatchList {
  public:
    /// Clear all exsting points and set up for a new image count.
    void resize(size_t num_images);

    /// Add a single point to the list.
    /// - Points must be added in order, low index to high index.
    /// - Returns false if the point could not be added.
    bool addPoint(size_t image, vw::ip::InterestPoint const &pt, bool valid=true);

    /// Return the number of images.
    size_t getNumImages() const;

    /// Return the number of points (usually but not always the same in each image).
    size_t getNumPoints(size_t image=0) const;

    /// Get a handle to a specific IP.
    vw::ip::InterestPoint const& getPoint(size_t image, size_t point) const;

    /// Returns the x/y coordinate of a point.
    vw::Vector2 getPointCoord(size_t image, size_t point) const;

    /// Return true if the point exists (valid or invalid)
    bool pointExists(size_t image, size_t point) const;

    /// Return true if a point is valid.
    bool isPointValid(size_t image, size_t point) const;

    /// Set the validity of a point.
    void setPointValid(size_t image, size_t point, bool newValue=true);

    /// Change the position of an interest point.
    void setPointPosition(size_t image, size_t point, float x, float y);

    /// Return the index of the nearest match point to the given pixel.
    /// - Returns -1 if no match was found.
    /// - If distLimit is set, return -1 if best match distance is over the limit.
    int findNearestMatchPoint(size_t image, vw::Vector2 P, double distLimit) const;

    /// Delete all IP for an image.
    void deletePointsForImage(size_t image);

    /// Delete the same IP in each image.
    bool deletePointAcrossImages(size_t point);

    /// Returns true if all points are valid and none are missing.
    bool allPointsValid() const;

    /// Try to load all of the points from match files on disk.
    /// - The match files correspond to files 0 < i < N and leftIndices
    ///   contains the index of the other file they match to (0 or i-1).
    /// - Any points that cannot be loaded will be flagged as invalid.
    /// - Return the number of points loaded, or -1 for failure.
    bool loadPointsFromMatchFiles(std::vector<std::string> const& matchFiles,
                                  std::vector<size_t     > const& leftIndices);

    /// Try to load the interest points from a GCP file.
    bool loadPointsFromGCPs(std::string const gcpPath,
                            std::vector<std::string> const& imageNames);

    /// Try to load the interest points from vwip files.
    bool loadPointsFromVwip(std::vector<std::string> const& vwipFiles,
                            std::vector<std::string> const& imageNames);
    
    /// Write all points out using a given prefix.
    bool savePointsToDisk(std::string const& prefix,
                          std::vector<std::string> const& imageNames,
                          std::string const& match_file="") const;

  private:

    /// Throw an exception if the specified point does not exist.
    void throwIfNoPoint(size_t image, size_t point) const;

    /// Set all IP for the image as valid.
    void setIpValid(size_t image);

    /// A set of interest points for each input image
    /// - There is always one set of matched interest points shared among all images.
    /// - The only way the counts can differ is if the user is in the process of manually
    ///   adding an interest point to the images.
    /// - The length of the outer vector is equal to the number of MainWidget objects
    std::vector<std::vector<vw::ip::InterestPoint>> m_matches;
    /// Stay synced with m_matches, set to false if that match is not 
    std::vector<std::vector<bool>> m_valid_matches;

  }; // End class MatchList

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
                                      ImageViewRef<PixelT> & image,
                                      std::string const& input_file,
                                      std::string const& suffix,
                                      bool has_georef,
                                      vw::cartography::GeoReference const & georef,
                                      bool has_nodata,
                                      double nodata_val) {

  std::string output_file = vw::mosaic::filename_from_suffix1(input_file, suffix);
  TerminalProgressCallback tpc("asp", ": ");
  vw_out() << "Writing: " << output_file << std::endl;
  try{
    vw::cartography::block_write_gdal_image(output_file, image, has_georef, georef,
                                has_nodata, nodata_val, opt, tpc);
  }catch(...){
    // Failed to write, presumably because we have no write access.
    // Write the file in the current dir.
    vw_out() << "Failed to write: " << output_file << "\n";
    output_file = vw::mosaic::filename_from_suffix2(input_file, suffix);
    vw_out() << "Writing: " << output_file << std::endl;
    vw::cartography::block_write_gdal_image(output_file, image, has_georef, georef,
                                has_nodata, nodata_val, opt, tpc);
  }
  return output_file;
}

// See if we are in the mode where the images are displayed side-by-side with a
// dialog to choose which ones to display.
bool sideBySideWithDialog();
void setNoSideBySideWithDialog(); // turn off such logic

// A class to return a color for each value in [0, 1].
struct Colormap {

  // Initialize the colors
  Colormap();
  
  // Get the color. The value t must be in [0, 1].
  vw::Vector3 operator()(double t);
  
  // Find the colors by interpolating in this table
  vw::ImageViewRef<vw::Vector3> interp_colors;
  
}; // end class Colormap
  
}} // namespace vw::gui

#endif  // __STEREO_GUI_GUI_UTILITIES_H__
