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


/// \file DiskImagePyramidMultiChannel.h
///
/// Functions for managing multi-channel images and converting
/// them to QImage.
///
#ifndef __STEREO_GUI_DISK_IMAGE_PYRAMID_MULTICHANNEL_H__
#define __STEREO_GUI_DISK_IMAGE_PYRAMID_MULTICHANNEL_H__

#include <asp/GUI/GuiBase.h>

#include <boost/filesystem/path.hpp>
#include <boost/filesystem.hpp>
#include <boost/mpl/or.hpp>
#include <omp.h>

// Qt
#include <QWidget>

// Vision Workbench
#include <vw/Core/Thread.h>
#include <vw/Core/Log.h>
#include <vw/Image/ImageViewRef.h>
#include <vw/Image/ImageView.h>
#include <vw/FileIO/DiskImageView.h>
#include <vw/FileIO/DiskImageUtils.h>
#include <vw/Math/BBox.h>
#include <vw/Math/Vector.h>
#include <vw/Mosaic/DiskImagePyramid.h>
#include <vw/Core/RunOnce.h>

#include <string>
#include <vector>
#include <list>
#include <set>

namespace vw { namespace gui {

  // The kinds of images we support
  enum ImgType {UNINIT, CH1_DOUBLE, CH2_UINT8, CH3_UINT8, CH4_UINT8};

  /// A global structure to hold all the temporary files we have created
  struct TemporaryFiles {
    std::set<std::string> files;
  };
  /// Access the global list of temporary files
  TemporaryFiles& temporary_files();
  
  // Form a QImage to show on screen. For scalar images, we scale them
  // and handle the nodata val. For two channel images, interpret the
  // second channel as mask. If there are 3 or more channels,
  // interpret those as RGB.
  
  template<class PixelT>
  typename boost::enable_if<boost::is_same<PixelT,double>, void>::type
  formQimage(bool highlight_nodata, bool scale_pixels, double nodata_val,
             vw::Vector2 const& approx_bounds,
             ImageView<PixelT> const& clip, QImage & qimg){

    double min_val = std::numeric_limits<double>::max();
    double max_val = -std::numeric_limits<double>::max();
    if (scale_pixels) {
      // No multi-threading here since we modify shared values
      for (int col = 0; col < clip.cols(); col++){
        for (int row = 0; row < clip.rows(); row++){
          if (clip(col, row) == nodata_val) continue;
          if (clip(col, row) < min_val) min_val = clip(col, row);
          if (clip(col, row) > max_val) max_val = clip(col, row);
        }
      }
    
      // The approx_bounds are computed on the lowest resolution level
      // of the pyramid and are likely exaggerated, but were computed
      // with outlier removal.  Use them to adjust the existing bounds
      // which may have outliers.
      if (approx_bounds[0] < approx_bounds[1]) {
        min_val = std::max(min_val, approx_bounds[0]);
        max_val = std::min(max_val, approx_bounds[1]);
      }
    
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
      
        if (clip(col, row) == nodata_val || std::isnan(clip(col, row)) ){
        
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
  typename boost::enable_if<boost::is_same<PixelT, vw::Vector<vw::uint8, 2>>, void>::type
  formQimage(bool highlight_nodata, bool scale_pixels, double nodata_val,
             vw::Vector2 const& approx_bounds,
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
  typename boost::disable_if<boost::mpl::or_<boost::is_same<PixelT,double>,
                                             boost::is_same<PixelT, vw::Vector<vw::uint8, 2>>>, void>::type
  formQimage(bool highlight_nodata, bool scale_pixels, double nodata_val,
             vw::Vector2 const& approx_bounds,
             ImageView<PixelT> const& clip, QImage & qimg){

    qimg = QImage(clip.cols(), clip.rows(), QImage::Format_ARGB32_Premultiplied);
#pragma omp parallel for
    for (int col = 0; col < clip.cols(); col++){
      for (int row = 0; row < clip.rows(); row++){
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

  // An image class that supports 1 to 3 channels.  We use
  // DiskImagePyramid<double> to be able to use some of the
  // pre-defined member functions for an image class. This class
  // is not a perfect solution, but there seem to be no easy way
  // in ASP to handle images with variable numbers of channels.
  // TODO: Add the case when multi-channel images also have float or double pixels
  struct DiskImagePyramidMultiChannel {
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
    void get_image_clip(double scale_in, vw::BBox2i region_in, bool highlight_nodata,
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
  
}} // namespace vw::gui

#endif  // __STEREO_GUI_DISK_IMAGE_PYRAMID_MULTICHANNEL_H__
