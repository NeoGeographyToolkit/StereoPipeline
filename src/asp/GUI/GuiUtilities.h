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
/// A widget showing an image.
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

// Qt
#include <QWidget>
#include <QPoint>

// Vision Workbench
#include <vw/Core/Thread.h>
#include <vw/Core/Log.h>
#include <vw/Image/ImageResource.h>
#include <vw/Image/ImageViewRef.h>
#include <vw/Image/ImageView.h>
#include <vw/Image/Manipulation.h>
#include <vw/Image/Statistics.h>
#include <vw/FileIO/DiskImageResource.h>
#include <vw/FileIO/DiskImageView.h>
#include <vw/FileIO/DiskImageUtils.h>
#include <vw/Math/BBox.h>
#include <vw/Math/Vector.h>
#include <vw/Cartography/GeoReference.h>
#include <vw/Cartography/GeoReferenceUtils.h>
#include <vw/InterestPoint/InterestData.h>
#include <vw/Mosaic/DiskImagePyramid.h>
#include <vw/Geometry/dPoly.h>

// ASP
#include <asp/Core/Common.h>
#include <vw/Image/AntiAliasing.h>

class QMouseEvent;
class QWheelEvent;
class QPoint;
class QResizeEvent;
class QTableWidget;
class QContextMenuEvent;
class QMenu;
class QPolygon;

namespace vw {
  // To be able to write multi-channel images
  template<> struct PixelFormatID< Vector<vw::uint8, 1> >  { static const PixelFormatEnum value = VW_PIXEL_GENERIC_1_CHANNEL; };
  template<> struct PixelFormatID< Vector<vw::uint8, 2> >  { static const PixelFormatEnum value = VW_PIXEL_GENERIC_2_CHANNEL; };
  template<> struct PixelFormatID< Vector<vw::uint8, 3> >  { static const PixelFormatEnum value = VW_PIXEL_GENERIC_3_CHANNEL; };
  template<> struct PixelFormatID< Vector<vw::uint8, 4> >  { static const PixelFormatEnum value = VW_PIXEL_GENERIC_4_CHANNEL; };
}

namespace vw { namespace gui {

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

  // The kinds of images we support
  enum ImgType {UNINIT, CH1_DOUBLE, CH2_UINT8, CH3_UINT8, CH4_UINT8};

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
                                              boost::is_same<PixelT, vw::Vector<vw::uint8, 2> > >, void >::type
  formQimage(bool highlight_nodata,
             bool scale_pixels, double nodata_val,
	     vw::Vector2 const& bounds,
	     ImageView<PixelT> const& clip, QImage & qimg);
  
  /// Convert a QRect object to a BBox2 object.
  inline BBox2 qrect2bbox(QRect const& R){
    return BBox2( Vector2(R.left(), R.top()), Vector2(R.right(), R.bottom()) );
  }

  /// Convert a BBox2 object to a QRect object.
  QRect bbox2qrect(BBox2 const& B);

  /// Save a hillshaded file
  bool write_hillshade(vw::cartography::GdalWriteOptions const& opt,
                       double azimuth, double elevation,
                       std::string const& input_file,
                       std::string      & output_file);

  // Given an image, and an input file name, modify the filename using
  // a prefix. Write the image to that filename. If that fails, create
  // instead the filename in the current directory. Return the name
  // of the output file.
  template<class PixelT>
  std::string write_in_orig_or_curr_dir(vw::cartography::GdalWriteOptions const& opt,
                                        ImageViewRef<PixelT> & image,
                                        std::string const& input_file,
                                        std::string const& suffix,
                                        bool has_georef,
                                        vw::cartography::GeoReference const & georef,
                                        bool has_nodata,
                                        double nodata_val);

  // Shape file (vector layer) functions

  void read_shapefile(std::string const& file,
		      std::string const& poly_color,
		      bool & has_geo, 
		      vw::cartography::GeoReference & geo,
		      std::vector<vw::geometry::dPoly> & polyVec);
  
  void write_shapefile(std::string const& file,
		       bool has_geo,
		       vw::cartography::GeoReference const& geo, 
		       std::vector<vw::geometry::dPoly> const& polyVec);
  
  void shapefile_bdbox(const std::vector<vw::geometry::dPoly> & polyVec,
		       // outputs
		       double & xll, double & yll,
		       double & xur, double & yur);
  
  void mergePolys(std::vector<vw::geometry::dPoly> & polyVec);
  
  // This will tweak the georeference so that point_to_pixel() is the identity.
  bool read_georef_from_shapefile(vw::cartography::GeoReference & georef,
				  std::string const& file);
  
  bool read_georef_from_image_or_shapefile(vw::cartography::GeoReference & georef,
					   std::string const& file);
  
  // Find the closest point in a given vector of polygons to a given point.
  void findClosestPolyVertex(// inputs
			     double x0, double y0,
			     const std::vector<vw::geometry::dPoly> & polyVec,
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
			   const std::vector<vw::geometry::dPoly> & polyVec,
			   // outputs
			   int & polyVecIndex,
			   int & polyIndexInCurrPoly,
			   int & vertIndexInCurrPoly,
			   double & minX, double & minY,
			   double & minDist
			   );
  
  // An image class that supports 1 to 3 channels.  We use
  // DiskImagePyramid<double> to be able to use some of the
  // pre-defined member functions for an image class. This class
  // is not a perfect solution, but there seem to be no easy way
  // in ASP to handle images with variable numbers of channels.
  // TODO: Add the case when multi-channel images also have float or double pixels
  struct DiskImagePyramidMultiChannel{
    vw::cartography::GdalWriteOptions m_opt;
    vw::mosaic::DiskImagePyramid< double               > m_img_ch1_double;
    vw::mosaic::DiskImagePyramid< Vector<vw::uint8, 2> > m_img_ch2_uint8;
    vw::mosaic::DiskImagePyramid< Vector<vw::uint8, 3> > m_img_ch3_uint8;
    vw::mosaic::DiskImagePyramid< Vector<vw::uint8, 4> > m_img_ch4_uint8;
    int m_num_channels;
    int m_rows, m_cols;
    ImgType m_type; // keeps track of which of the above images we use

    // Constructor
    DiskImagePyramidMultiChannel(std::string const& base_file = "",
                                 vw::cartography::GdalWriteOptions const& opt = vw::cartography::GdalWriteOptions(),
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

  /// A class to keep all data associated with an image file
  struct imageData{
    std::string      name;
    vw::cartography::GdalWriteOptions m_opt;
    bool             has_georef;
    vw::cartography::GeoReference georef;
    BBox2            image_bbox;
    DiskImagePyramidMultiChannel img;
    std::vector<vw::geometry::dPoly> polyVec; // a shapefile
    
    /// Load an image from disk into img and set the other variables.
    void read(std::string const& image,
	      vw::cartography::GdalWriteOptions const& opt,
	      bool use_georef);

    bool isPoly() const { return asp::has_shp_extension(name); }
  };

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

  private:
    QTableWidget * m_filesTable;
    
    private slots:
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
    for (int col = 0; col < clip.cols(); col++){
      for (int row = 0; row < clip.rows(); row++){
        if (clip(col, row) <= nodata_val) continue;
        if (clip(col, row) < min_val) min_val = clip(col, row);
        if (clip(col, row) > max_val) max_val = clip(col, row);
      }
    }

    // These bounds may contain outliers, so correct for that
    min_val = std::max(min_val, bounds[0]);
    max_val = std::min(max_val, bounds[1]);

    // A safety measure
    if (min_val >= max_val)
      max_val = min_val + 1.0;
  }

  qimg = QImage(clip.cols(), clip.rows(), QImage::Format_ARGB32_Premultiplied);
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
std::string write_in_orig_or_curr_dir(vw::cartography::GdalWriteOptions const& opt,
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

}} // namespace vw::gui

#endif  // __STEREO_GUI_GUI_UTILITIES_H__
