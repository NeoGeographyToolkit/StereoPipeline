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

// Form a QImage to show on screen. For scalar images, we scale them
// and handle the nodata val. For two channel images, interpret the
// second channel as mask. If there are 3 or more channels,
// interpret those as RGB.
  
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
        if (clip(col, row) == nodata_val) continue;
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
typename boost::disable_if<boost::mpl::or_<boost::is_same<PixelT,double>,
                                           boost::is_same<PixelT, vw::Vector<vw::uint8, 2>>>, void>::type
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
        qimg.setPixel(col, row, QColor(v[0], v[1], v[2], 255*(v[3] > 0)).rgba()); // color or transp

      else
        qimg.setPixel(col, row, QColor(v[0], v[0], v[0], 255).rgba()); // grayscale
    }
  }
}

}} // namespace vw::gui

#endif  // __STEREO_GUI_DISK_IMAGE_PYRAMID_MULTICHANNEL_H__
