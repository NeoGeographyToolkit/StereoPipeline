// __BEGIN_LICENSE__
//  Copyright (c) 2009-2013, United States Government as represented by the
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


/// \file MedianFilter.h
///

#ifndef __MEDIAN_FILTER_H__
#define __MEDIAN_FILTER_H__

#define CALC_PIXEL_NUM_VALS 256

#include <vw/Core/Functors.h>
#include <vw/Core/FundamentalTypes.h>
#include <vw/Math/Vector.h>
#include <vw/Math/BBox.h>
#include <vw/Image/ImageView.h>
#include <vw/Image/EdgeExtension.h>
#include <vw/Image/PerPixelAccessorViews.h>

namespace vw {

  uint8 find_median_in_histogram(Vector<int, CALC_PIXEL_NUM_VALS> histogram,
                                 int kernSize);

  template<class ImageT>
  ImageView<typename ImageT::pixel_type> fast_median_filter(ImageViewBase<ImageT> const& img,  int kernSize) {
    typedef typename ImageT::pixel_type PixelT;

    ImageView<uint8> src = pixel_cast_rescale<uint8>(img.impl());
    ImageView<uint8> result(src.cols(), src.rows());

    Vector<int, CALC_PIXEL_NUM_VALS> histogram;

    // Seed histogram
    for (int x = 0; x < kernSize; x++) {
      for (int y = 0; y < kernSize; y++) {
        histogram(src(x, y))++;
      }
    }

    bool goingRight = true;
    int x = 0, y = 0;
    //while (y <=src.rows() - kernSize) { //this was a bug
    while (y < src.rows() - kernSize) {

      result(x + kernSize / 2, y + kernSize / 2) = find_median_in_histogram(histogram, kernSize);

      if (goingRight) {
        if (x < src.cols() - kernSize) {
          for (int i = 0; i < kernSize; i++) {
            histogram(src(x, y + i))--;
            histogram(src(x + kernSize, y + i))++;
          }
          x++;

        }
        else {
          // Reached the right edge
          for (int i = 0; i < kernSize; i++) {
            histogram(src(x + i, y))--;
            histogram(src(x + i, y + kernSize))++;
          }
          goingRight = false;
          y++;
        }
      }
      else {
        if (x > 0) {
          for (int i = 0; i < kernSize; i++) {
            histogram(src(x - 1, y + i))++;
            histogram(src(x + kernSize - 1, y + i))--;
          }
          x--;
        }
        else {
          // Reached the left edge
          for (int i = 0; i < kernSize; i++) {
            histogram(src(x + i, y))--;
            histogram(src(x + i, y + kernSize))++;
          }
          goingRight = true;
          y++;
        }
      }
    }

    return pixel_cast_rescale<PixelT>(result);
  }

  template<class PixelT>
  class MedianFilterFunctor:public ReturnFixedType<PixelT>
  {
    int m_kernel_width;
    int m_kernel_height;

  public:
    MedianFilterFunctor(int kernel_width, int kernel_height):
      m_kernel_width(kernel_width),
      m_kernel_height(kernel_height){}

    BBox2i work_area() const{ return BBox2i(Vector2i(-m_kernel_width/2, -m_kernel_height/2),
                                            Vector2i(m_kernel_width/2, m_kernel_height/2));  }

    template<class PixelAccessorT>
    typename PixelAccessorT::pixel_type operator()(PixelAccessorT const& acc) const{

      Vector<int, CALC_PIXEL_NUM_VALS> histogram;

      // Seed histogram
      for (int x = 0; x < m_kernel_width; x++) {
        for (int y = 0; y < m_kernel_height; y++) {
          histogram(*acc)++;
        }
      }
      *acc = find_median_in_histogram(histogram, m_kernel_width);

      return *acc;
    }
  };

  template<class ViewT>
  UnaryPerPixelAccessorView<EdgeExtensionView<ViewT, ZeroEdgeExtension>, MedianFilterFunctor<typename ViewT::pixel_type> >
  my_median_filter(ImageViewBase<ViewT> const& input, int kernel_width, int kernel_height)
  {
    return UnaryPerPixelAccessorView<EdgeExtensionView<ViewT, ZeroEdgeExtension>, MedianFilterFunctor<typename ViewT::pixel_type> >(edge_extend(input, ZeroEdgeExtension()), MedianFilterFunctor<typename ViewT::pixel_type> (kernel_width, kernel_height));
  }

}

#endif // __MEDIAN_FILTER_H__
