// __BEGIN_LICENSE__
//  Copyright (c) 2006-2024, United States Government as represented by the
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

#include <asp/GUI/DiskImagePyramidMultiChannel.h>
#include <asp/Core/StereoSettings.h>
#include <vw/Core/Stopwatch.h>
#include <QtWidgets>

#include <string>
#include <vector>

using namespace vw;
using namespace vw::gui;

namespace vw { namespace gui {

vw::RunOnce temporary_files_once = VW_RUNONCE_INIT;
boost::shared_ptr<TemporaryFiles> temporary_files_ptr;
void init_temporary_files() {
  temporary_files_ptr = boost::shared_ptr<TemporaryFiles>(new TemporaryFiles());
}

TemporaryFiles& temporary_files() {
  temporary_files_once.run(init_temporary_files);
  return *temporary_files_ptr;
}

// Form a QImage to show on screen. For scalar images, we scale them
// and handle the nodata val. For two channel images, interpret the
// second channel as mask. If there are 3 or more channels,
// interpret those as RGB.

template<class PixelT>
typename boost::enable_if<boost::is_same<PixelT,double>, void>::type
formQimage(bool highlight_nodata, bool scale_pixels, double nodata_val,
           vw::Vector2 const& approx_bounds,
           ImageView<PixelT> const& clip, QImage & qimg) {

  double min_val = approx_bounds[0];
  double max_val = approx_bounds[1];

  qimg = QImage(clip.cols(), clip.rows(), QImage::Format_ARGB32_Premultiplied);

  #pragma omp parallel for
  for (int col = 0; col < clip.cols(); col++) {
    for (int row = 0; row < clip.rows(); row++) {

      double v = clip(col, row);
      if (scale_pixels)
        v = round(255*(std::max(v, min_val) - min_val)/(max_val-min_val));

      v = std::min(std::max(0.0, v), 255.0);

      if (clip(col, row) == nodata_val || std::isnan(clip(col, row))) {

        if (!highlight_nodata) {
          // transparent
          qimg.setPixel(col, row, QColor(0, 0, 0, 0).rgba());
        } else {
          // highlight in red
          qimg.setPixel(col, row, qRgb(255, 0, 0));
          std::cout << "--highlighting nodata " << col << ' ' << row << std::endl;
        }

      } else {
        // opaque
        qimg.setPixel(col, row, QColor(v, v, v, 255).rgba());
      }
    }
  }

  return;
}

template<class PixelT>
typename boost::enable_if<boost::is_same<PixelT, vw::Vector<vw::uint8, 2>>, void>::type
formQimage(bool highlight_nodata, bool scale_pixels, double nodata_val,
           vw::Vector2 const& approx_bounds,
           ImageView<PixelT> const& clip, QImage & qimg) {

  qimg = QImage(clip.cols(), clip.rows(), QImage::Format_ARGB32_Premultiplied);
  #pragma omp parallel for
  for (int col = 0; col < clip.cols(); col++) {
    for (int row = 0; row < clip.rows(); row++) {
      Vector<vw::uint8, 2> v = clip(col, row);
      if (v[1] > 0 && v == v) { // need the latter for NaN
        // opaque grayscale
        qimg.setPixel(col, row, QColor(v[0], v[0], v[0], 255).rgba());
      } else {
        // transparent
        qimg.setPixel(col, row, QColor(0, 0, 0, 0).rgba());
      }
    }
  }
}

template<class PixelT>
typename boost::disable_if<boost::mpl::or_<boost::is_same<PixelT,double>,
                                            boost::is_same<PixelT, vw::Vector<vw::uint8, 2>>>, void>::type
formQimage(bool highlight_nodata, bool scale_pixels, double nodata_val,
           vw::Vector2 const& approx_bounds,
           ImageView<PixelT> const& clip, QImage & qimg) {

  std::cout << "qidm 2\n";
  qimg = QImage(clip.cols(), clip.rows(), QImage::Format_ARGB32_Premultiplied);

  #pragma omp parallel for
  for (int col = 0; col < clip.cols(); col++) {
    for (int row = 0; row < clip.rows(); row++) {
      PixelT v = clip(col, row);
      if (v != v) // NaN, set to transparent
        qimg.setPixel(col, row, QColor(0, 0, 0, 0).rgba());
      else if (v.size() == 3) // color
        qimg.setPixel(col, row, QColor(v[0], v[1], v[2], 255).rgba());
      else if (v.size() > 3) // color or transparent
        qimg.setPixel(col, row, QColor(v[0], v[1], v[2], 255*(v[3] > 0)).rgba());
      else // grayscale
        qimg.setPixel(col, row, QColor(v[0], v[0], v[0], 255).rgba());
    }
  }
}

DiskImagePyramidMultiChannel::
DiskImagePyramidMultiChannel(std::string const& image_file,
                             vw::GdalWriteOptions const& opt,
                             int top_image_max_pix, int subsample):
  m_opt(opt), m_num_channels(0), m_rows(0), m_cols(0), m_type(UNINIT) {

  if (image_file == "")
    return;

  int & lowres_size = asp::stereo_settings().lowest_resolution_subimage_num_pixels; // alias
  if (lowres_size <= 0) // bug fix, longer term need to improve the workflow
    lowres_size = 1000 * 1000;

  boost::shared_ptr<DiskImageResource> image_rsrc
    = vw::DiskImageResourcePtr(image_file);
  ImageFormat image_fmt = image_rsrc->format();

  // Redirect to the correctly typed function to perform the actual map projection.
  // - Must correspond to the type of the input image.
  // Instantiate the correct DiskImagePyramid then record information including
  //  the list of temporary files it created.
  try {
    m_num_channels = get_num_channels(image_file);

    if (m_num_channels > 1 && image_fmt.channel_type != VW_CHANNEL_UINT8) {
      vw_out() << "File " << image_file << " has more than one band, and the "
               << "bands are not unsigned int. Reading only the first band in "
               << "double precision.\n";
    }

    if (m_num_channels == 1 || image_fmt.channel_type != VW_CHANNEL_UINT8) {
      // Single channel image with float pixels.

      m_img_ch1_double =
        vw::mosaic::DiskImagePyramid<double>(image_file, m_opt, lowres_size);
      m_rows = m_img_ch1_double.rows();
      m_cols = m_img_ch1_double.cols();
      m_type = CH1_DOUBLE;
      temporary_files().files.insert(m_img_ch1_double.get_temporary_files().begin(),
                                     m_img_ch1_double.get_temporary_files().end());
    } else if (m_num_channels == 2) {
      // uint8 image with an alpha channel.
      m_img_ch2_uint8 = vw::mosaic::DiskImagePyramid<Vector<vw::uint8, 2>>
        (image_file, m_opt, lowres_size);
      m_num_channels = 2; // we read only 1 channel
      m_rows = m_img_ch2_uint8.rows();
      m_cols = m_img_ch2_uint8.cols();
      m_type = CH2_UINT8;
      temporary_files().files.insert(m_img_ch2_uint8.get_temporary_files().begin(),
                                     m_img_ch2_uint8.get_temporary_files().end());
    } else if (m_num_channels == 3) {
      // RGB image with three uint8 channels.
      m_img_ch3_uint8 = vw::mosaic::DiskImagePyramid<Vector<vw::uint8, 3>>
        (image_file, m_opt, lowres_size);
      m_num_channels = 3;
      m_rows = m_img_ch3_uint8.rows();
      m_cols = m_img_ch3_uint8.cols();
      m_type = CH3_UINT8;
      temporary_files().files.insert(m_img_ch3_uint8.get_temporary_files().begin(),
                                     m_img_ch3_uint8.get_temporary_files().end());
    } else if (m_num_channels == 4) {
      // RGB image with three uint8 channels and an alpha channel
      m_img_ch4_uint8 = vw::mosaic::DiskImagePyramid<Vector<vw::uint8, 4>>
        (image_file, m_opt, lowres_size);
      m_num_channels = 4;
      m_rows = m_img_ch4_uint8.rows();
      m_cols = m_img_ch4_uint8.cols();
      m_type = CH4_UINT8;
      temporary_files().files.insert(m_img_ch4_uint8.get_temporary_files().begin(),
                                     m_img_ch4_uint8.get_temporary_files().end());
    } else {
      vw_throw(ArgumentErr() << "Unsupported image with " << m_num_channels
               << " bands.\n");
    }
  } catch (const Exception& e) {
    // Do not invoke popUp() here, since this function can be called without a GUI.
      vw_throw(ArgumentErr() << e.what());
      return;
  }
}

double DiskImagePyramidMultiChannel::get_nodata_val() const {

  // Extract the clip, then convert it from VW format to QImage format.
  if (m_type == CH1_DOUBLE) {
    return m_img_ch1_double.get_nodata_val();
  } else if (m_type == CH2_UINT8) {
    return m_img_ch2_uint8.get_nodata_val();
  } else if (m_type == CH3_UINT8) {
    return m_img_ch3_uint8.get_nodata_val();
  } else if (m_type == CH4_UINT8) {
    return m_img_ch4_uint8.get_nodata_val();
  } else {
    vw_throw(ArgumentErr() << "Unsupported image with " << m_num_channels << " bands\n");
  }
}

void DiskImagePyramidMultiChannel::get_image_clip(double scale_in, vw::BBox2i region_in,
                  bool highlight_nodata,
                  QImage & qimg, double & scale_out, vw::BBox2i & region_out) const{

  bool scale_pixels = (m_type == CH1_DOUBLE);
  vw::Vector2 approx_bounds;

  // Extract the clip, then convert it from VW format to QImage format.
  if (m_type == CH1_DOUBLE) {

    //Stopwatch sw0;
    //sw0.start();
    if (asp::stereo_settings().min < asp::stereo_settings().max) {
      // If the min and max are set, not NaN, and first is less than the second
      approx_bounds = vw::Vector2(asp::stereo_settings().min, asp::stereo_settings().max);
    } else {
      // Normally these are auto-estimated rather well, except for images with
      // most data being very small, like in shadow.
      approx_bounds = m_img_ch1_double.approx_bounds();
    }

    // Ensure the bounds are always distinct
    if (approx_bounds[0] >= approx_bounds[1] &&
        approx_bounds[1] > -std::numeric_limits<double>::max())
      approx_bounds[0] = approx_bounds[1] - 1.0;

    //sw0.stop();
    //vw_out() << "Render time sw0 (seconds): " << sw0.elapsed_seconds() << std::endl;

    ImageView<double> clip;
    //Stopwatch sw1;
    //sw1.start();
    m_img_ch1_double.get_image_clip(scale_in, region_in, clip, scale_out, region_out);
    //sw1.stop();
    //vw_out() << "Render time sw1 (seconds): " << sw1.elapsed_seconds() << std::endl;

    //Stopwatch sw2;
    //sw2.start();
    formQimage(highlight_nodata, scale_pixels, m_img_ch1_double.get_nodata_val(),
               approx_bounds, clip, qimg);
    //sw2.stop();
    //vw_out() << "Render time sw2 (seconds): " << sw2.elapsed_seconds() << std::endl;
  } else if (m_type == CH2_UINT8) {

    ImageView<Vector<vw::uint8, 2>> clip;
    //Stopwatch sw4;
    //sw4.start();
    m_img_ch2_uint8.get_image_clip(scale_in, region_in, clip,
                                 scale_out, region_out);
    //sw4.stop();
    //vw_out() << "Render time sw4 (seconds): " << sw4.elapsed_seconds() << std::endl;

    //Stopwatch sw5;
    //sw5.start();
    formQimage(highlight_nodata, scale_pixels, m_img_ch2_uint8.get_nodata_val(),
               approx_bounds, clip, qimg);
    //sw5.stop();
    //vw_out() << "Render time sw5 (seconds): " << sw5.elapsed_seconds() << std::endl;

  } else if (m_type == CH3_UINT8) {
    ImageView<Vector<vw::uint8, 3>> clip;
    //Stopwatch sw6;
    //sw6.start();
    m_img_ch3_uint8.get_image_clip(scale_in, region_in, clip,
                                 scale_out, region_out);
    //sw6.stop();
    //vw_out() << "Render time sw6 (seconds): " << sw6.elapsed_seconds() << std::endl;

    //Stopwatch sw7;
    //sw7.start();
    formQimage(highlight_nodata, scale_pixels, m_img_ch3_uint8.get_nodata_val(),
               approx_bounds, clip, qimg);
    //sw7.stop();
    //vw_out() << "Render time sw7 (seconds): " << sw7.elapsed_seconds() << std::endl;

  } else if (m_type == CH4_UINT8) {
    //Stopwatch sw8;
    //sw8.start();
    ImageView<Vector<vw::uint8, 4>> clip;
    m_img_ch4_uint8.get_image_clip(scale_in, region_in, clip,
          scale_out, region_out);
    //sw8.stop();
    //vw_out() << "Render time sw8 (seconds): " << sw8.elapsed_seconds() << std::endl;

    //Stopwatch sw9;
    //sw9.start();
    formQimage(highlight_nodata, scale_pixels, m_img_ch4_uint8.get_nodata_val(),
               approx_bounds, clip, qimg);
    //sw9.stop();
    //vw_out() << "Render time sw9 (seconds): " << sw9.elapsed_seconds() << std::endl;
  } else {
    vw_throw(ArgumentErr() << "Unsupported image with " << m_num_channels << " bands\n");
  }
}

std::string DiskImagePyramidMultiChannel::get_value_as_str(int32 x, int32 y) const {

  // Below we cast from Vector<uint8> to Vector<double>, as the former
  // refuses to print well.
  std::ostringstream os;
  if (m_type == CH1_DOUBLE) {
    os << m_img_ch1_double.bottom()(x, y, 0);
  } else if (m_type == CH2_UINT8) {
    os << Vector2(m_img_ch2_uint8.bottom()(x, y, 0));
  } else if (m_type == CH3_UINT8) {
    os << Vector3(m_img_ch3_uint8.bottom()(x, y, 0));
  } else if (m_type == CH4_UINT8) {
    os << Vector4(m_img_ch4_uint8.bottom()(x, y, 0));
  } else {
    vw_throw(ArgumentErr() << "Unsupported image with " << m_num_channels << " bands\n");
  }

  return os.str();
}

double DiskImagePyramidMultiChannel::get_value_as_double(int32 x, int32 y) const {
  if (m_type == CH1_DOUBLE) {
    return m_img_ch1_double.bottom()(x, y, 0);
  } else if (m_type == CH2_UINT8) {
    return m_img_ch2_uint8.bottom()(x, y, 0)[0];
  } else {
    vw_throw(ArgumentErr() << "Unsupported image with " << m_num_channels << " bands\n");
  }
  return 0;
}

}} // namespace vw::gui
