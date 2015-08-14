// __BEGIN_LICENSE__
//  Copyright (c) 2006-2013, United States Government as represented by the
//  Administrator of the National Aeronautics and Space Administration. All
//  rights reserved.
//
//  The NASA Vision Workbench is licensed under the Apache License,
//  Version 2.0 (the "License"); you may not use this file except in
//  compliance with the License. You may obtain a copy of the License at
//  http://www.apache.org/licenses/LICENSE-2.0
//
//  Unless required by applicable law or agreed to in writing, software
//  distributed under the License is distributed on an "AS IS" BASIS,
//  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
//  See the License for the specific language governing permissions and
//  limitations under the License.
// __END_LICENSE__


/// \file MainWidget.h
///
/// A widget showing an image.
///
#ifndef __STEREO_GUI__MAIN_WIDGET_H__
#define __STEREO_GUI_MAIN_WIDGET_H__

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

  // Pop-up a window with given message
  void popUp(std::string msg);

  // The kinds of images we support
  enum ImgType {UNINIT, CH1_DOUBLE, CH1_UINT8, CH3_UINT8};

  // For multi-channel images we cannot use create_mask_less_or_equal.
  template<class PixelT>

  typename boost::enable_if< boost::mpl::or_< boost::is_same<PixelT,double>, boost::is_same<PixelT,vw::uint8> >, ImageViewRef< PixelMask<PixelT> > >::type
 create_custom_mask(ImageViewRef<PixelT> & img, double nodata_val){
    return create_mask_less_or_equal(img, nodata_val);
  }
  template<class PixelT>
  typename boost::disable_if< boost::mpl::or_< boost::is_same<PixelT,double>, boost::is_same<PixelT,vw::uint8> >, ImageViewRef< PixelMask<PixelT> > >::type
  create_custom_mask(ImageViewRef<PixelT> & img, double nodata_val){
    return create_mask(img, nodata_val);
  }

  // Single-channel images are read into Image<float>, while
  // multi-channel in Image<Vector>, and we skip reading
  // extra channels.
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

  // Form a qimage to show on screen. For scalar images, we scale them
  // and handle the nodata val.  For other types, we use the pixels as
  // they are.  if there are 3 or more channels, interpret those as
  // RGB, otherwise create a grayscale image.
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

// A class to manage very large images and their subsampled versions
// in a pyramid. The most recently accessed tiles are cached in memory.
template <class PixelT>
class DiskImagePyramid : public ImageViewBase<DiskImagePyramid<PixelT> > {

public:
  typedef typename DiskImageView<PixelT>::pixel_type pixel_type;
  typedef typename DiskImageView<PixelT>::result_type result_type;
  typedef typename DiskImageView<PixelT>::pixel_accessor pixel_accessor;

  // Constructor. Note that we use NaN as nodata if not available,
  // that has the effect of not accidentally setting some pixels
  // to nodata.
  DiskImagePyramid(std::string const& base_file = "",
                   int top_image_max_pix = 1000*1000,
                   int subsample = 2
                   ): m_subsample(subsample),
                      m_top_image_max_pix(top_image_max_pix),
                      m_nodata_val(std::numeric_limits<double>::quiet_NaN()) {

    if (base_file == "")
      return;

    if (subsample < 2) {
      vw_throw( ArgumentErr() << "Must subsample by a factor of at least 2.\n");
    }

    if (top_image_max_pix < 4) {
      vw_throw( ArgumentErr() << "The image at the top of the pyramid must "
                << "be at least 2x2 in size.\n");
    }

    m_pyramid.push_back(custom_read<PixelT>(base_file));

    m_pyramid_files.push_back(base_file);
    m_scales.push_back(1);

    // Get the nodata value, if present
    bool has_nodata = vw::read_nodata_val(base_file, m_nodata_val);
    std::string prefix = asp::prefix_from_filename(base_file);
    fs::path base_path(base_file);
    std::time_t base_time = fs::last_write_time(base_path);

    int level = 0;
    int scale = 1;
    while (double(m_pyramid[level].cols())*double(m_pyramid[level].rows()) >
           m_top_image_max_pix ){

      // The name of the file at the current scale
      std::ostringstream os;
      scale *= subsample;
      os <<  "_sub" << scale << ".tif";
      std::string suffix = os.str();
      std::string curr_file = prefix + suffix;

      if (level == 0) {
        vw_out() << "Detected large image: " << base_file  << "." << std::endl;
        vw_out() << "Will construct an image pyramid on disk."  << std::endl;
      }

      ImageViewRef< PixelMask<PixelT> > masked
        = create_custom_mask(m_pyramid[level], m_nodata_val);
      double sub_scale = 1.0/subsample;
      int tile_size = 256;
      int sub_threads = 1;

      // Resample the image at the current pyramid level.
      // TODO: resample_aa is a hacky thingy. Need to understand
      // what is a good way of resampling.
      // Note that below we cast the channels to double for resampling,
      // then cast back to current pixel type for saving.
      ImageViewRef<PixelT> unmasked
        = block_rasterize
        (vw::cache_tile_aware_render
         (pixel_cast<PixelT>
          (apply_mask
           (vw::resample_aa
            (channel_cast<double>(masked), sub_scale),
            m_nodata_val)),
          Vector2i(tile_size,tile_size) * sub_scale),
         tile_size, sub_threads);

      // If the file exists, and has the right size, and is not too old,
      // don't write it again
      bool will_write = true;
      if (fs::exists(curr_file)) {
        try{
          DiskImageView<double> curr(curr_file);
          fs::path curr_path(curr_file);
          std::time_t curr_file_time = fs::last_write_time(curr_path);
          if (curr.cols() == unmasked.cols() && curr.rows() == unmasked.rows() &&
              curr_file_time > base_time) {
            will_write = false;
          }
        }catch(...){
          will_write = false;
        }
      }

      if (will_write) {
        TerminalProgressCallback tpc("asp", ": ");
        bool has_georef = false;
        vw::cartography::GeoReference georef;
        asp::BaseOptions opt;
        vw_out() << "Writing: " << curr_file << std::endl;
        try{
          asp::block_write_gdal_image(curr_file, unmasked, has_georef, georef,
                                      has_nodata, m_nodata_val, opt, tpc);
        }catch(...){
          // Failed to write, presumably because we have no write access.
          // Write the file in the current dir.
          vw_out() << "Failed to write: " << curr_file << "\n";
          boost::filesystem::path p(base_file);
          prefix = base_path.stem().string();
          curr_file = prefix + suffix;
          vw_out() << "Writing: " << curr_file << std::endl;
          asp::block_write_gdal_image(curr_file, unmasked, has_georef, georef,
                                      has_nodata, m_nodata_val, opt, tpc);

        }
      }else{
        vw_out() << "Using existing subsampled image: " << curr_file << std::endl;
      }

      // Note that m_pyramid contains a handle to DiskImageView.
      // DiskImageView's implementation will make it possible
      // cache in memory the most recently used tiles of all
      // the images in the pyramid.
      m_pyramid_files.push_back(curr_file);
      m_pyramid.push_back(vw::DiskImageView<PixelT>(curr_file));
      m_scales.push_back(scale);

      level++;
    }
  }

  // Given a region (at full resolution) and a scale factor, compute
  // the portion of the image in the region, subsampled by a factor no
  // more than the input scale factor. Convert it to QImage for
  // display, so in this function we hide the details of how channels
  // combined to create the QImage. Also return the precise subsample
  // factor used and the region at that scale level.
  void getImageClip(double scale_in, vw::BBox2i region_in,
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

  ~DiskImagePyramid() {}

  int32 cols() const { return m_pyramid[0].cols(); }
  int32 rows() const { return m_pyramid[0].rows(); }
  int32 planes() const { return m_pyramid[0].planes(); }

  pixel_accessor origin() const { return m_pyramid[0].origin(); }
  result_type operator()( int32 x, int32 y, int32 p = 0 ) const { return m_pyramid[0](x,y,p); }

  typedef typename DiskImageView<PixelT>::prerasterize_type prerasterize_type;
  prerasterize_type prerasterize( BBox2i const& bbox ) const { return m_pyramid[0].prerasterize( bbox ); }
  template <class DestT> void rasterize( DestT const& dest, BBox2i const& bbox ) const { m_pyramid[0].rasterize( dest, bbox ); }

  ImageViewRef<PixelT> bottom() { return m_pyramid[0]; }

private:
  // The subsample factor to go to the next level of the pyramid
  // (must be >= 2).
  int m_subsample;

  // The maxiumum number of pixels in the coarsest level of
  // the pyramid (keep on downsampling until getting to this number
  // or under it).
  int m_top_image_max_pix;

  //  The pyramid. Largest images come earlier.
  std::vector< vw::ImageViewRef<PixelT> > m_pyramid;

  // The files (stored on disk) containing the images in the pyramid.
  std::vector<std::string> m_pyramid_files;

  double m_nodata_val;
  std::vector<int> m_scales;
};

  // An image class that supports 1 to 3 channels.  We inherit from
  // DiskImagePyramid<double> to be able to use some of the
  // pre-defined member functions for an image class. This class
  // is not a perfect solution, but there seem to be no easy way
  // in ASP to handle images with variable numbers of channels.
  struct DiskImagePyramidMultiChannel: public DiskImagePyramid<double> {
    DiskImagePyramid<double>  m_img_ch1_double;
    DiskImagePyramid< Vector<vw::uint8, 1> > m_img_ch1_uint8;
    DiskImagePyramid< Vector<vw::uint8, 3> > m_img_ch3_uint8;
    int m_num_channels;
    int m_rows, m_cols;
    ImgType m_type; // keeps track of which of the above images we use

    // Constructor
    DiskImagePyramidMultiChannel(std::string const& base_file = "",
                                 int top_image_max_pix = 1000*1000,
                                 int subsample = 2):m_num_channels(0),
                                                    m_rows(0), m_cols(0),
                                                    m_type(UNINIT){
      if (base_file == "") return;

      m_num_channels = get_num_channels(base_file);
      if (m_num_channels == 1) {
        // Single channel image with float pixels.
        m_img_ch1_double = DiskImagePyramid<double>(base_file);
        m_rows = m_img_ch1_double.rows();
        m_cols = m_img_ch1_double.cols();
        m_type = CH1_DOUBLE;
      }else if (m_num_channels == 2){
        // uint8 image with an alpha channel. Ignore the alpha channel.
        m_img_ch1_uint8 = DiskImagePyramid< Vector<vw::uint8, 1> >(base_file);
        m_num_channels = 1; // we read only 1 channel
        m_rows = m_img_ch1_uint8.rows();
        m_cols = m_img_ch1_uint8.cols();
        m_type = CH1_UINT8;
      } else if (m_num_channels == 3 || m_num_channels == 4) {
        // RGB image with three uint8 channels and perhaps an
        // alpha channel which we ignore.
        m_img_ch3_uint8 = DiskImagePyramid< Vector<vw::uint8, 3> >(base_file);
        m_num_channels = 3; // we read only 3 channels
        m_rows = m_img_ch3_uint8.rows();
        m_cols = m_img_ch3_uint8.cols();
        m_type = CH3_UINT8;
      }else{
        vw_throw( ArgumentErr() << "Unsupported image with "
                  << m_num_channels << " bands.\n");
      }
    }

    // This function will return a QImage to be shown on screen.
    // How we create it, depends on the type of image we want
    // to display.
    void getImageClip(double scale_in, vw::BBox2i region_in,
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
    int32 cols()   const { return m_cols;  }
    int32 rows()   const { return m_rows;  }
    int32 planes() const { return m_num_channels; }

    // This operator will quietly return just the first channel.
    // The getImageClip() function is what needs to be used
    // generally.
    double operator()( int32 x, int32 y, int32 p = 0 ) const {
      if (m_type == CH1_DOUBLE) {
        return m_img_ch1_double(x, y, p);
      }else if (m_type == CH1_UINT8){
        return m_img_ch1_uint8(x, y, p)[0];
      }else{
        vw_throw( ArgumentErr() << "Unsupported image with "
                  << m_num_channels << " bands\n");
      }
      return 0;
    }
  };

  // A class to keep all data associated with an image file
  struct imageData{
    std::string name;
    bool has_georef;
    vw::cartography::GeoReference georef;
    BBox2 bbox; // The pixel bbox or lonlat bbox if georef is present
    DiskImagePyramidMultiChannel img;
    void read(std::string const& image, bool use_georef, bool hillshade);
    double m_lon_offset; // to compensate for -90 deg equalling 270 deg
  };

  vw::Vector2 QPoint2Vec(QPoint const& qpt);
  QPoint Vec2QPoint(vw::Vector2 const& V);

  // A simple class for keeping track of crosshair locations and colors.
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
    void push_back(std::list<vw::Vector2> pts) {
      std::list<vw::Vector2>::iterator iter  = pts.begin();
      while (iter != pts.end()) {
        m_points.push_back(*iter);
        ++iter;
      }
    }
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

  class MainWidget : public QWidget {
    Q_OBJECT

  public:

    // Constructors/Destructor
    MainWidget(QWidget *parent,
               int image_id,
               std::string const& output_prefix,
               std::vector<std::string> const& image_files,
               std::vector<std::vector<ip::InterestPoint> > & matches,
               chooseFilesDlg * chooseFiles, bool use_georef,
               bool hillshade);
    virtual ~MainWidget();

    bool get_crop_win(QRect & win);

    // Set a default size for this widget.  This is usually overridden
    // by parent views.
    virtual QSize sizeHint () const { return QSize(500,500); }

    // Image Manipulation Methods
    void zoom(double scale);
    void viewMatches(bool hide);

    void setShadowThreshMode(bool turnOn) { m_shadow_thresh_calc_mode = turnOn;}

  signals:
    void refreshAllMatches();

  public slots:
    void sizeToFit();
    void showFilesChosenByUser(int rowClicked, int columnClicked);
    void viewUnthreshImages();
    void viewThreshImages();
    void viewHillshadeImages();
    void addMatchPoint();
    void deleteMatchPoint();

  protected:

    // Setup
    bool eventFilter (QObject *obj, QEvent *E);
    void resizeEvent(QResizeEvent*);

    // Event handlers
    void paintEvent(QPaintEvent *event);
    void mousePressEvent(QMouseEvent *event);
    void mouseReleaseEvent(QMouseEvent *event);
    void mouseMoveEvent(QMouseEvent *event);
    void mouseDoubleClickEvent(QMouseEvent *event);
    void wheelEvent(QWheelEvent *event);
    void enterEvent(QEvent *event);
    void leaveEvent(QEvent *event);
    void keyPressEvent(QKeyEvent *event);
    void contextMenuEvent(QContextMenuEvent *event);

  private:

    // Choose which files to hide/show in the GUI
    chooseFilesDlg  *     m_chooseFilesDlg;
    std::set<std::string> m_filesToHide;
    std::vector<int> m_filesOrder;

    int m_image_id;
    std::string m_output_prefix;
    std::vector<std::string> m_image_files;

    // Note that this is an alias
    std::vector<std::vector<vw::ip::InterestPoint> > & m_matches;
    bool m_hideMatches;

    bool m_use_georef;
    bool m_hillshade;

    bool  m_firstPaintEvent;
    QRect m_emptyRubberBand;
    QRect m_rubberBand;
    BBox2 m_stereoCropWin;

    // if we are selecting a crop win to do stereo in
    bool m_cropWinMode;

    // Use double buffering: draw to a pixmap first, refresh it only
    // if really necessary, and display it when paintEvent is called.
    QPixmap m_pixmap;

    bool m_bilinear_filter;
    bool m_use_colormap;

    std::vector<imageData> m_images;
    BBox2 m_images_box;

    // Adjustment mode
    enum AdjustmentMode { NoAdjustment,
                          TransformAdjustment, GainAdjustment,
                          OffsetAdjustment, GammaAdjustment };
    AdjustmentMode m_adjust_mode;

    // Mouse position
    vw::Vector2 m_curr_pixel_pos, m_curr_world_pos;

    // Dimensions and stats
    int m_window_width;  // the width  of the plotting window in screen pixels
    int m_window_height; // the height of the plotting window in screen pixels

    // Image Parameters
    vw::BBox2 m_current_view, m_last_view;
    double m_gain, m_last_gain;
    double m_offset, m_last_offset;
    double m_gamma, m_last_gamma;

    enum DisplayChannel { DisplayRGBA = 0, DisplayR, DisplayG, DisplayB, DisplayA };
    int m_display_channel;
    int m_colorize_display;

    // Mouse press  position
    int m_mousePrsX,  m_mousePrsY;

    // Right-click context menu
    QMenu * m_ContextMenu;
    QAction* m_addMatchPoint;
    QAction* m_deleteMatchPoint;

    double m_shadow_thresh;
    bool m_shadow_thresh_calc_mode;
    bool m_shadow_thresh_view_mode;
    std::vector<imageData> m_shadow_thresh_images;

    bool m_hillshade_mode;
    std::vector<imageData> m_hillshaded_images;


    // Drawing is driven by QPaintEvent, which calls out to drawImage()
    void drawImage(QPainter* paint);
    vw::Vector2 world2screen(vw::Vector2 const& p);
    vw::Vector2 screen2world(vw::Vector2 const& pix);
    BBox2 world2screen(BBox2 const& R);
    BBox2 screen2world(BBox2 const& R);
    BBox2i world2image(BBox2 const& R, int imageIndex);
    vw::BBox2 expand_box_to_keep_aspect_ratio(vw::BBox2 const& box);
    void updateCurrentMousePosition();
    void updateRubberBand(QRect & R);
    void refreshPixmap();
  };

}} // namespace vw::gui

#endif  // __STEREO_GUI_MAIN_WIDGET_H__
