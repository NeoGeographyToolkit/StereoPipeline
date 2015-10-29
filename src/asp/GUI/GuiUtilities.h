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
#include <vw/InterestPoint/InterestData.h>

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

namespace vw {
  // To be able to write Vector3 images
  template<> struct PixelFormatID< Vector<vw::uint8, 1> >  { static const PixelFormatEnum value = VW_PIXEL_GENERIC_1_CHANNEL; };
  template<> struct PixelFormatID< Vector<vw::uint8, 2> >  { static const PixelFormatEnum value = VW_PIXEL_GENERIC_2_CHANNEL; };
  template<> struct PixelFormatID< Vector<vw::uint8, 3> >  { static const PixelFormatEnum value = VW_PIXEL_GENERIC_3_CHANNEL; };
  template<> struct PixelFormatID< Vector<vw::uint8, 4> >  { static const PixelFormatEnum value = VW_PIXEL_GENERIC_4_CHANNEL; };
}

namespace vw { namespace gui {

  namespace fs = boost::filesystem;

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
  enum ImgType {UNINIT, CH1_DOUBLE, CH1_UINT8, CH3_UINT8};

  // Gets called for PixelT == double
  template<class PixelT>
  typename boost::enable_if< boost::mpl::or_< boost::is_same<PixelT,double>, boost::is_same<PixelT,vw::uint8> >, ImageViewRef< PixelMask<PixelT> > >::type
  create_custom_mask(ImageViewRef<PixelT> & img, double nodata_val){
    return create_mask_less_or_equal(img, nodata_val);
  }
  // Gets called for PixelT == Vector<u8, 1> and Vector<u8, 3>
  template<class PixelT>
  typename boost::disable_if< boost::mpl::or_< boost::is_same<PixelT,double>, boost::is_same<PixelT,vw::uint8> >, ImageViewRef< PixelMask<PixelT> > >::type
  create_custom_mask(ImageViewRef<PixelT> & img, double nodata_val){
    PixelT mask_pixel;
    mask_pixel.set_all(nodata_val);
    return create_mask(img, mask_pixel);
  }

  /// Single-channel images are read into Image<float>, while
  /// multi-channel in Image<Vector>, and we skip reading extra channels.
  template<class PixelT>
  typename boost::enable_if<boost::is_same<PixelT,double>, ImageViewRef<PixelT> >::type
  custom_read(std::string const& file){
    return DiskImageView<PixelT>(file);
  }
  template<class PixelT>
  typename boost::disable_if<boost::is_same<PixelT,double>, ImageViewRef<PixelT> >::type
  custom_read(std::string const& file){
    return vw::read_channels<vw::math::VectorSize<PixelT>::value, typename PixelT::value_type>(file, 0);
  }


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
  // and handle the nodata val.  For other types, we use the pixels as
  // they are.  if there are 3 or more channels, interpret those as
  // RGB, otherwise create a grayscale image.
  template<class PixelT>
  typename boost::enable_if<boost::mpl::or_< boost::is_same<PixelT,double>, boost::is_same<PixelT,vw::uint8> >, void >::type
  formQimage(bool highlight_nodata, bool scale_pixels, double nodata_val,
             ImageView<PixelT> const& clip,
             QImage & qimg);
  template<class PixelT>
  typename boost::disable_if<boost::mpl::or_< boost::is_same<PixelT,double>, boost::is_same<PixelT,vw::uint8> >, void >::type
  formQimage(bool highlight_nodata, bool scale_pixels, double nodata_val,
             ImageView<PixelT> const& clip, QImage & qimg);


  inline BBox2 qrect2bbox(QRect const& R){
    return BBox2( Vector2(R.left(), R.top()), Vector2(R.right(), R.bottom()) );
  }

  QRect bbox2qrect(BBox2 const& B);

  bool write_hillshade(asp::BaseOptions const& opt,
                       std::string const& input_file,
                       std::string & output_file);

  // Given georef2 and a point in projected coordinates with this
  // georef, convert it to pixel coordinates for georef1.
  Vector2 point_to_pixel(Vector2 const& proj_pt2,
                         double lon_offset,
                         cartography::GeoReference const& georef1,
                         cartography::GeoReference const& georef2);

  // The reverse of pixel_to_point_bbox. Given georef2 and a box in
  // projected coordinates of this georef, convert it to a pixel box with georef1.
  BBox2 point_to_pixel_bbox(BBox2 point_box2,
                            double lon_offset,
                            cartography::GeoReference const& georef1,
                            cartography::GeoReference const& georef2);

  // TODO: Move these pixel_to_point_bbox and point_to_pixel_bbox functions
  // to a lower-level location.

  // Given an image with georef1 and a portion of its pixels in
  // pixel_box1, find the bounding box of pixel_box1 in projected
  // point units for georef2.
  BBox2 forward_pixel_to_point_bbox(BBox2 pixel_box1,
                                    double lon_offset,
                                    cartography::GeoReference const& georef1,
                                    cartography::GeoReference const& georef2);


  // TODO: Move to a general strings file

  /// Get a filename by simply replacing current extension with given suffix
  std::string filename_from_suffix1(std::string const& input_file,
                                    std::string const& suffix);

  /// Get a filename by  replacing current extension with given suffix,
  /// and making the file be in the current directory.
  std::string filename_from_suffix2(std::string const& input_file,
                                    std::string const& suffix);

  // Given an image, and an input file name, modify the filename using
  // a prefix. Write the image to that filename. If that fails, create
  // instead the filename in the current directory. Return the name
  // of the output file.
  template<class PixelT>
  std::string write_in_orig_or_curr_dir(asp::BaseOptions const& opt,
                                        ImageViewRef<PixelT> & image,
                                        std::string const& input_file,
                                        std::string const& suffix,
                                        bool has_georef,
                                        vw::cartography::GeoReference const & georef,
                                        bool has_nodata,
                                        double nodata_val);

  /// If output_file exists, is not older than input_file,
  /// and has given numbers of rows and columns, don't overwrite it.
  bool overwrite_if_no_good(std::string const& input_file,
                            std::string const& output_file,
                            int cols = -1, int rows = -1);

  // TODO: Move this class!
  /// A class to manage very large images and their subsampled versions
  /// in a pyramid. The most recently accessed tiles are cached in memory.
  /// - Caching is handled by use of the DiskImageView class.
  /// - Constructing this class creates a temporary file on disk for each level of the pyramid.
  template <class PixelT>
  class DiskImagePyramid {

  public:
    typedef typename DiskImageView<PixelT>::pixel_type pixel_type;

    // Constructor. Note that we use NaN as nodata if not available,
    // that has the effect of not accidentally setting some pixels to nodata.
    DiskImagePyramid(std::string const& base_file = "",
                     asp::BaseOptions const& opt = asp::BaseOptions(),
                     int top_image_max_pix = 1000*1000,
                     int subsample = 2);

    // Given a region (at full resolution) and a scale factor, compute
    // the portion of the image in the region, subsampled by a factor no
    // more than the input scale factor. Convert it to QImage for
    // display, so in this function we hide the details of how channels
    // combined to create the QImage. Also return the precise subsample
    // factor used and the region at that scale level.
    void getImageClip(double scale_in, vw::BBox2i region_in,
                      bool highlight_nodata, bool scale_pixels,
                      QImage & qimg, double & scale_out, vw::BBox2i & region_out);

    ~DiskImagePyramid() {}

    // These all describe the highest resolution pyramid layer
    int32 cols  () const { return m_pyramid[0].cols(); }
    int32 rows  () const { return m_pyramid[0].rows(); }
    int32 planes() const { return m_pyramid[0].planes(); }

    /// Return the highest resolution pyramid layer
    ImageViewRef<PixelT>        bottom()       { return m_pyramid[0]; }
    ImageViewRef<PixelT> const& bottom() const { return m_pyramid[0]; }

  private:

    asp::BaseOptions m_opt;

    // The subsample factor to go to the next level of the pyramid (must be >= 2).
    int m_subsample;

    // The maxiumum number of pixels in the coarsest level of the pyramid
    // (keep on downsampling until getting to this number or under it).
    int m_top_image_max_pix;

    //  The pyramid. Largest images come earlier.
    std::vector< vw::ImageViewRef<PixelT> > m_pyramid;

    // The files (stored on disk) containing the images in the pyramid.
    std::vector<std::string> m_pyramid_files;

    // We may wipe these at the end
    std::vector<std::string> m_cached_files;

    double m_nodata_val;
    std::vector<int> m_scales;
  };

  // An image class that supports 1 to 3 channels.  We inherit from
  // DiskImagePyramid<double> to be able to use some of the
  // pre-defined member functions for an image class. This class
  // is not a perfect solution, but there seem to be no easy way
  // in ASP to handle images with variable numbers of channels.
  struct DiskImagePyramidMultiChannel{
    asp::BaseOptions m_opt;
    DiskImagePyramid< double               > m_img_ch1_double;
    DiskImagePyramid< Vector<vw::uint8, 1> > m_img_ch1_uint8;
    DiskImagePyramid< Vector<vw::uint8, 3> > m_img_ch3_uint8;
    int m_num_channels;
    int m_rows, m_cols;
    ImgType m_type; // keeps track of which of the above images we use

    // Constructor
    DiskImagePyramidMultiChannel(std::string const& base_file = "",
                                 asp::BaseOptions const& opt = asp::BaseOptions(),
                                 int top_image_max_pix = 1000*1000,
                                 int subsample = 2);

    // This function will return a QImage to be shown on screen.
    // How we create it, depends on the type of image we want to display.
    void getImageClip(double scale_in, vw::BBox2i region_in,
                      bool highlight_nodata,
                      QImage & qimg, double & scale_out, vw::BBox2i & region_out);

    int32 cols  () const { return m_cols;  }
    int32 rows  () const { return m_rows;  }
    int32 planes() const { return m_num_channels; }


    /// Return the element at this location (at the lowest level) cast to double.
    /// - Only works for single channel pyramids!
    double get_value_as_double( int32 x, int32 y) const;

  };

  /// A class to keep all data associated with an image file
  struct imageData{
    std::string      name;
    asp::BaseOptions m_opt;
    bool             has_georef;
    vw::cartography::GeoReference georef;
    BBox2            image_bbox;
    BBox2            lonlat_bbox;
    DiskImagePyramidMultiChannel img;
    double m_lon_offset; // to compensate for -90 deg equalling 270 deg

    /// Load an image from disk into img and set the other variables.
    void read(std::string const& image, asp::BaseOptions const& opt, bool use_georef);
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
typename boost::enable_if<boost::mpl::or_< boost::is_same<PixelT,double>, boost::is_same<PixelT,vw::uint8> >, void >::type
formQimage(bool highlight_nodata, bool scale_pixels, double nodata_val,
           ImageView<PixelT> const& clip,
           QImage & qimg){

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
    if (min_val >= max_val)
      max_val = min_val + 1.0;
  }

  qimg = QImage(clip.cols(), clip.rows(), QImage::Format_RGB888);
  for (int col = 0; col < clip.cols(); col++){
    for (int row = 0; row < clip.rows(); row++){
      double val = clip(col, row);
      if (scale_pixels)
        val = round(255*(std::max(val, min_val) - min_val)/(max_val-min_val));
      if (!highlight_nodata || clip(col, row) > nodata_val)
        qimg.setPixel(col, row, qRgb(val, val, val));
      else
        qimg.setPixel(col, row, qRgb(255, 0, 0)); // highlight in red
    }
  }
}
template<class PixelT>
typename boost::disable_if<boost::mpl::or_< boost::is_same<PixelT,double>, boost::is_same<PixelT,vw::uint8> >, void >::type
formQimage(bool highlight_nodata, bool scale_pixels, double nodata_val,
           ImageView<PixelT> const& clip, QImage & qimg){

  qimg = QImage(clip.cols(), clip.rows(), QImage::Format_RGB888);
  for (int col = 0; col < clip.cols(); col++){
    for (int row = 0; row < clip.rows(); row++){
      PixelT v = clip(col, row);
      if (v.size() >= 3)
        qimg.setPixel(col, row, qRgb(v[0], v[1], v[2])); // color
      else
        qimg.setPixel(col, row, qRgb(v[0], v[0], v[0])); // grayscale
    }
  }
}

template<class PixelT>
std::string write_in_orig_or_curr_dir(asp::BaseOptions const& opt,
                                      ImageViewRef<PixelT> & image,
                                      std::string const& input_file,
                                      std::string const& suffix,
                                      bool has_georef,
                                      vw::cartography::GeoReference const & georef,
                                      bool has_nodata,
                                      double nodata_val){

  TerminalProgressCallback tpc("asp", ": ");

  std::string output_file = filename_from_suffix1(input_file, suffix);

  vw_out() << "Writing: " << output_file << std::endl;
  try{
    asp::block_write_gdal_image(output_file, image, has_georef, georef,
                                has_nodata, nodata_val, opt, tpc);
  }catch(...){
    // Failed to write, presumably because we have no write access.
    // Write the file in the current dir.
    vw_out() << "Failed to write: " << output_file << "\n";
    output_file = filename_from_suffix2(input_file, suffix);
    vw_out() << "Writing: " << output_file << std::endl;
    asp::block_write_gdal_image(output_file, image, has_georef, georef,
                                has_nodata, nodata_val, opt, tpc);
  }
  return output_file;
}


template <class PixelT>
DiskImagePyramid<PixelT>::DiskImagePyramid(std::string const& base_file,
                 asp::BaseOptions const& opt,
                 int top_image_max_pix,
                 int subsample
                 ): m_opt(opt), m_subsample(subsample),
                    m_top_image_max_pix(top_image_max_pix),
                    m_nodata_val(std::numeric_limits<double>::quiet_NaN()) {

  if (base_file == "")
    return;

  if (subsample < 2) {
    vw_throw( ArgumentErr() << "Must subsample by a factor of at least 2.\n");
  }

  if (top_image_max_pix < 4) {
    vw_throw( ArgumentErr() << "The image at the top of the pyramid must be at least 2x2 in size.\n");
  }

  m_pyramid.push_back(custom_read<PixelT>(base_file));

  m_pyramid_files.push_back(base_file);
  m_scales.push_back(1);

  // Get the nodata value, if present
  bool has_nodata = vw::read_nodata_val(base_file, m_nodata_val);

  cartography::GeoReference georef;
  bool has_georef = vw::cartography::read_georeference(georef, base_file);

  // Keep making more pyramid levels until they are small enough
  int level = 0;
  int scale = 1;
  while (double(m_pyramid[level].cols())*double(m_pyramid[level].rows()) > m_top_image_max_pix ){

    // The name of the file at the current scale
    std::ostringstream os;
    scale *= subsample;
    os <<  "_sub" << scale << ".tif";
    std::string suffix = os.str();

    if (level == 0) {
      vw_out() << "Detected large image: " << base_file  << "." << std::endl;
      vw_out() << "Will construct an image pyramid on disk."  << std::endl;
    }

    ImageViewRef< PixelMask<PixelT> > masked = create_custom_mask(m_pyramid[level], m_nodata_val);
    double sub_scale   = 1.0/subsample;
    int    tile_size   = 256;
    int    sub_threads = 1;

    // Resample the image at the current pyramid level.
    // TODO: resample_aa is a hacky thingy. Need to understand
    // what is a good way of resampling.
    // Note that below we cast the channels to double for resampling,
    // then cast back to current pixel type for saving.
    PixelT nodata_pixel;
    set_all(nodata_pixel, m_nodata_val);
    ImageViewRef<PixelT> unmasked
        = block_rasterize(
            vw::cache_tile_aware_render(
              pixel_cast<PixelT>(
                apply_mask(
                  vw::resample_aa(
                    channel_cast<double>(masked),
                    sub_scale
                  ),
                  nodata_pixel
                )
              ),
              Vector2i(tile_size,tile_size) * sub_scale
            ),
            Vector2i(tile_size,tile_size), sub_threads
          );

    // Write the current image.
    vw::cartography::GeoReference sub_georef;
    if (has_georef)
      sub_georef = resample(georef, sub_scale);

    // If the file exists, and has the right size, and is not too old,
    // don't write it again
    std::string curr_file = filename_from_suffix1(base_file, suffix);
    bool will_write = overwrite_if_no_good(base_file, curr_file,
                                           unmasked.cols(), unmasked.rows());
    try {
      if (will_write) {
        TerminalProgressCallback tpc("asp", ": ");
        vw_out() << "Writing: " << curr_file << std::endl;
        try{
          asp::block_write_gdal_image(curr_file, unmasked, has_georef, sub_georef,
                                      has_nodata, m_nodata_val, opt, tpc);
        }catch(...){
          vw_out() << "Failed to write: " << curr_file << "\n";
          curr_file = filename_from_suffix2(base_file, suffix);
          will_write = overwrite_if_no_good(base_file, curr_file,
                                            unmasked.cols(), unmasked.rows());
          if (will_write) {
            vw_out() << "Writing: " << curr_file << std::endl;
            asp::block_write_gdal_image(curr_file, unmasked, has_georef, sub_georef,
                                        has_nodata, m_nodata_val, opt, tpc);
          }
        }
      }

    } catch ( const Exception& e ) {
      popUp(e.what());
      return;
    }

    if (!will_write)
      vw_out() << "Using existing subsampled image: " << curr_file << std::endl;

    // Note that m_pyramid contains a handle to DiskImageView.
    // DiskImageView's implementation will make it possible to
    // cache in memory the most recently used tiles of all
    // the images in the pyramid.
    m_pyramid_files.push_back(curr_file);
    temporary_files().files.insert(curr_file);
    m_pyramid.push_back(vw::DiskImageView<PixelT>(curr_file));
    m_scales.push_back(scale);

    level++;
  } // End level creation loop

}

template <class PixelT>
void DiskImagePyramid<PixelT>::getImageClip(double scale_in, vw::BBox2i region_in,
                  bool highlight_nodata, bool scale_pixels,
                  QImage & qimg, double & scale_out, vw::BBox2i & region_out) {

  if (m_pyramid.empty())
    vw_throw( ArgumentErr() << "Uninitialized image pyramid.\n");

  // Find the right pyramid level to use
  int level = 0;
  while (1) {
    if (level+1 >= (int)m_scales.size()) break; // last level
    if (m_scales[level+1] > scale_in)  break; // too coarse
    level++;
  }

  //vw_out() << "Reading: " << m_pyramid_files[level] << std::endl;

  region_in.crop(bounding_box(m_pyramid[0]));
  scale_out = m_scales[level];
  region_out = region_in/scale_out;
  region_out.crop(bounding_box(m_pyramid[level]));

  ImageView<PixelT> clip = crop(m_pyramid[level], region_out);
  formQimage(highlight_nodata, scale_pixels, m_nodata_val, clip, qimg);
}






}} // namespace vw::gui

#endif  // __STEREO_GUI_GUI_UTILITIES_H__
