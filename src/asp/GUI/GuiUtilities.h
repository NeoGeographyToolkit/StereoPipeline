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

#include <string>
#include <vector>
#include <list>
#include <set>

#include <boost/filesystem/path.hpp>
#include <boost/filesystem.hpp>
#include <boost/mpl/or.hpp>
#if !__APPLE__
#include <omp.h>
#endif

// Qt
#include <QWidget>
#include <QPoint>

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
#include <vw/Mosaic/DiskImagePyramid.h>
#include <vw/Geometry/dPoly.h>
#include <vw/Image/AntiAliasing.h>

// ASP
#include <asp/Core/Common.h>

class QMouseEvent;
class QWheelEvent;
class QPoint;
class QResizeEvent;
class QTableWidget;
class QContextMenuEvent;
class QMenu;
class QPolygon;


namespace vw { namespace gui {

  // The kinds of images we support
  enum ImgType {UNINIT, CH1_DOUBLE, CH2_UINT8, CH3_UINT8, CH4_UINT8};

  enum DisplayMode {REGULAR_VIEW, HILLSHADED_VIEW, COLORMAP_VIEW, HILLSHADE_COLORMAP_VIEW, THRESHOLDED_VIEW};
  
  // TODO(oalexan1): Remove this def out of this header file
  namespace fs = boost::filesystem;

  bool isPolyZeroDim(const QPolygon & pa);
  
  /// A global structure to hold all the temporary files we have created
  struct TemporaryFiles {
    std::set<std::string> files;
  };
  /// Access the global list of temporary files
  TemporaryFiles& temporary_files();

  // Pop-up a window with given message
  void popUp(std::string msg);

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

  // Form a QImage to show on screen. For scalar images, we scale them
  // and handle the nodata val. For two channel images, interpret the
  // second channel as mask. If there are 3 or more channels,
  // interpret those as RGB.
  template<class PixelT>
  typename boost::enable_if<boost::is_same<PixelT,double>, void>::type
  formQimage(bool highlight_nodata, bool scale_pixels, double nodata_val,
	     vw::Vector2 const& bounds,
             ImageView<PixelT> const& clip, QImage & qimg);

  template<class PixelT>
  typename boost::enable_if<boost::is_same<PixelT, vw::Vector<vw::uint8, 2> >, void>::type
  formQimage(bool highlight_nodata, bool scale_pixels, double nodata_val,
	     vw::Vector2 const& bounds, ImageView<PixelT> const& clip,
             QImage & qimg);

  template<class PixelT>
  typename boost::disable_if<boost::mpl::or_< boost::is_same<PixelT,double>,
                                              boost::is_same<PixelT, vw::Vector<vw::uint8, 2> > >,
                             void >::type
  formQimage(bool highlight_nodata,
             bool scale_pixels, double nodata_val,
	     vw::Vector2 const& bounds,
	     ImageView<PixelT> const& clip, QImage & qimg);
  
  // An image class that supports 1 to 3 channels.  We use
  // DiskImagePyramid<double> to be able to use some of the
  // pre-defined member functions for an image class. This class
  // is not a perfect solution, but there seem to be no easy way
  // in ASP to handle images with variable numbers of channels.
  // TODO: Add the case when multi-channel images also have float or double pixels
  struct DiskImagePyramidMultiChannel{
    vw::GdalWriteOptions m_opt;
    vw::mosaic::DiskImagePyramid<double>               m_img_ch1_double;
    vw::mosaic::DiskImagePyramid<Vector<vw::uint8, 2>> m_img_ch2_uint8;
    vw::mosaic::DiskImagePyramid<Vector<vw::uint8, 3>> m_img_ch3_uint8;
    vw::mosaic::DiskImagePyramid<Vector<vw::uint8, 4>> m_img_ch4_uint8;
    int m_num_channels;
    int m_rows, m_cols;
    ImgType m_type; // keeps track of which of the above images we use

    // Constructor
    DiskImagePyramidMultiChannel(std::string const& image_file = "",
                                 vw::GdalWriteOptions const&
                                 opt = vw::GdalWriteOptions(),
                                 int top_image_max_pix = 1000*1000,
                                 int subsample = 2);

    // This function will return a QImage to be shown on screen.
    // How we create it, depends on the type of image we want to display.
    void get_image_clip(double scale_in, vw::BBox2i region_in,
                      bool highlight_nodata,
                      QImage & qimg, double & scale_out, vw::BBox2i & region_out) const;
    double get_nodata_val() const;
    
    int32 cols  () const { return m_cols;  }
    int32 rows  () const { return m_rows;  }
    int32 planes() const { return m_num_channels; }

    /// Return the element at this location (at the lowest level) cast to double.
    /// - Only works for single channel pyramids!
    double get_value_as_double( int32 x, int32 y) const;

    // Return value as string
    std::string get_value_as_str( int32 x, int32 y) const;
  };

  // Return true if the extension is .csv or .txt
  bool hasXyzData(std::string const& fileName);
  
  /// A class to keep all data associated with an image file
  struct imageData{
    std::string      name, hillshaded_name; // TODO(oalexan1): Think more here
    vw::GdalWriteOptions m_opt;
    bool             has_georef;
    vw::cartography::GeoReference georef;
    BBox2            image_bbox;

    // There are several display modes. The one being shown is
    // determined by m_display_mode. Store the corresponding
    // image in one of the structures below
    DisplayMode m_display_mode;
    DiskImagePyramidMultiChannel img;
    DiskImagePyramidMultiChannel hillshaded_img;
    DiskImagePyramidMultiChannel thresholded_img;
    DiskImagePyramidMultiChannel colorized_img;
    DiskImagePyramidMultiChannel color_thresholded_img;
    
    std::vector<vw::geometry::dPoly> polyVec; // a shapefile

    // Irregular xyz data to be plotted at (x, y) location with z giving
    // the intensity. May be colorized.
    std::vector<vw::Vector3> xyz_data;
    
    imageData(): m_display_mode(REGULAR_VIEW) {}
    
    /// Load an image from disk into img and set the other variables.
    void read(std::string const& image, vw::GdalWriteOptions const& opt,
              int display_mode = REGULAR_VIEW);

    bool isPoly() const { return asp::has_shp_extension(name); }
    bool isXyz()  const { return vw::gui::hasXyzData(name); }
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
			     double & minDist
			     );

  // Find the closest edge in a given vector of polygons to a given point.
  void findClosestPolyEdge(// inputs
			   double x0, double y0,
			   std::vector<vw::geometry::dPoly> const& polyVec,
			   // outputs
			   int & polyVecIndex,
			   int & polyIndexInCurrPoly,
			   int & vertIndexInCurrPoly,
			   double & minX, double & minY,
			   double & minDist
			   );
  
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

  /// Class to create a file list on the left side of the window
  class chooseFilesDlg: public QWidget{
    Q_OBJECT

  public:
    chooseFilesDlg(QWidget * parent);
    ~chooseFilesDlg();
    void chooseFiles(const std::vector<imageData> & images);

    QTableWidget * getFilesTable(){ return m_filesTable; }
    static QString selectFilesTag(){ return ""; }

    // Check if the given image is hidden (not shown) based on the table checkbox  
    bool isHidden(std::string const& image) const;
    // Hide the given image  
    void hide(std::string const& image);
    // Show the given image  
    void unhide(std::string const& image);

    // Show only first two images; this is the best default for pairwise stereo
    void showTwoImages();
    
    // Show all images
    void showAllImages();
    
  private:
    int imageRow(std::string const& image) const;
    QTableWidget * m_filesTable;
    void keyPressEvent(QKeyEvent *event);
    std::map<std::string, int> image_to_row;
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
    std::vector<std::vector<vw::ip::InterestPoint> > m_matches;
    /// Stay synced with m_matches, set to false if that match is not 
    std::vector<std::vector<bool> > m_valid_matches;

  }; // End class MatchList

  // For each pairs if image indices, store the match file and interest points.
  // Load these on-demand, and once loaded, keep them in memory.
  struct pairwiseMatchList {
    std::map<std::pair<int, int>, std::string> match_files;
    std::map<std::pair<int, int>, std::pair<std::vector<vw::ip::InterestPoint>, std::vector<vw::ip::InterestPoint>>> matches;

    // We will copy here only the ip that need to be shown for the moment
    std::vector<std::vector<vw::ip::InterestPoint>> ip_to_show;
  };
//====================================================================================
//====================================================================================
// Function definitions

template<class PixelT>
typename boost::enable_if<boost::is_same<PixelT,double>, void>::type
formQimage(bool highlight_nodata, bool scale_pixels, double nodata_val,
	   vw::Vector2 const& bounds,
           ImageView<PixelT> const& clip, QImage & qimg){

  double min_val = std::numeric_limits<double>::max();
  double max_val = -std::numeric_limits<double>::max();
  if (scale_pixels) {
    // No multi-threading here since we modify shared values
    for (int col = 0; col < clip.cols(); col++){
      for (int row = 0; row < clip.rows(); row++){
        if (clip(col, row) <= nodata_val) continue;
        if (clip(col, row) < min_val) min_val = clip(col, row);
        if (clip(col, row) > max_val) max_val = clip(col, row);
      }
    }
    
    // The input bounds are computed on the lowest resolution level of the pyramid 
    //  with: vw::math::find_outlier_brackets(vals, 0.25, 4.0, b, e);
    //  but enforcing them here likely causes more problems than it solves.
    //// These bounds may contain outliers, so correct for that
    //if (bounds[0] != bounds[1]) {
    //  min_val = std::max(min_val, bounds[0]);
    //  max_val = std::min(max_val, bounds[1]);
    //}
    
    // A safety measure
    if (min_val >= max_val)
      max_val = min_val + 1.0;
  }

  qimg = QImage(clip.cols(), clip.rows(), QImage::Format_ARGB32_Premultiplied);
#pragma omp parallel for
  for (int col = 0; col < clip.cols(); col++){
    for (int row = 0; row < clip.rows(); row++){

      double v = clip(col, row);
      if (scale_pixels) 
        v = round(255*(std::max(v, min_val) - min_val)/(max_val-min_val));
     
      v = std::min(std::max(0.0, v), 255.0);
      
      if (clip(col, row) <= nodata_val || std::isnan(clip(col, row)) ){
        
        if (!highlight_nodata){
          // transparent
          qimg.setPixel(col, row, Qt::transparent);
        }else{
         // highlight in red
          qimg.setPixel(col, row, qRgb(255, 0, 0));
        }
        
      }else{
        // opaque
        qimg.setPixel(col, row, QColor(v, v, v, 255).rgba());
      }
    }
  }
}
  
template<class PixelT>
typename boost::enable_if<boost::is_same<PixelT, vw::Vector<vw::uint8, 2> >, void>::type
formQimage(bool highlight_nodata, bool scale_pixels, double nodata_val,
	   vw::Vector2 const& bounds,
           ImageView<PixelT> const& clip, QImage & qimg){

  qimg = QImage(clip.cols(), clip.rows(), QImage::Format_ARGB32_Premultiplied);
#pragma omp parallel for
  for (int col = 0; col < clip.cols(); col++){
    for (int row = 0; row < clip.rows(); row++){
      Vector<vw::uint8, 2> v = clip(col, row);
      if ( v[1] > 0 && v == v){ // need the latter for NaN
        // opaque grayscale
        qimg.setPixel(col, row, QColor(v[0], v[0], v[0], 255).rgba());
      }else{
        // transparent
        qimg.setPixel(col, row, QColor(0, 0, 0, 0).rgba());
      }
    }
  }
}

template<class PixelT>
typename boost::disable_if<boost::mpl::or_< boost::is_same<PixelT,double>,
                                            boost::is_same<PixelT, vw::Vector<vw::uint8, 2> > >, void >::type
formQimage(bool highlight_nodata, bool scale_pixels, double nodata_val,
	   vw::Vector2 const& bounds,
           ImageView<PixelT> const& clip, QImage & qimg){

  qimg = QImage(clip.cols(), clip.rows(), QImage::Format_ARGB32_Premultiplied);
#pragma omp parallel for
  for (int col = 0; col < clip.cols(); col++){
    for (int row = 0; row < clip.rows(); row++){
      PixelT v = clip(col, row);
      if (v != v) // NaN
	qimg.setPixel(col, row, QColor(0, 0, 0, 0).rgba()); // transparent
      else if (v.size() == 3)
        qimg.setPixel(col, row, QColor(v[0], v[1], v[2], 255).rgba()); // color
      else if (v.size() > 3)
        qimg.setPixel(col, row, QColor(v[0], v[1], v[2], 255*(v[3] > 0) ).rgba()); // transp or color

      else
        qimg.setPixel(col, row, QColor(v[0], v[0], v[0], 255).rgba()); // grayscale
    }
  }
}

template<class PixelT>
std::string write_in_orig_or_curr_dir(vw::GdalWriteOptions const& opt,
                                      ImageViewRef<PixelT> & image,
                                      std::string const& input_file,
                                      std::string const& suffix,
                                      bool has_georef,
                                      vw::cartography::GeoReference const & georef,
                                      bool has_nodata,
                                      double nodata_val){

  TerminalProgressCallback tpc("asp", ": ");

  std::string output_file = vw::mosaic::filename_from_suffix1(input_file, suffix);

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
  
}} // namespace vw::gui

#endif  // __STEREO_GUI_GUI_UTILITIES_H__
