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


#ifndef __VW_KYLE_STEREO__
#define __VW_KYLE_STEREO__

#include <vw/Image.h>
#include <vw/Stereo.h>

namespace vw {
  namespace stereo {
    enum PreprocFilterType {
      NO_PREPROC_FILTER,
      LOG_PREPROC_FILTER,
      SLOG_PREPROC_FILTER,
      GAUSSIAN_PREPROC_FILTER,
      BOX_PREPROC_FILTER
    };

    template <class ImageT>
    ImageViewRef<typename ImageT::pixel_type> calc_preproc_filter(ImageViewBase<ImageT> const& image, PreprocFilterType preproc_filter, double size = 0) {
      switch (preproc_filter) {
        case NO_PREPROC_FILTER:
          return image.impl();
          break;
        case LOG_PREPROC_FILTER:
          return laplacian_filter(gaussian_filter(image.impl(), size));
          break;
        case SLOG_PREPROC_FILTER:
          return threshold(laplacian_filter(gaussian_filter(image.impl(), size)), 0.0);
          break;
        case GAUSSIAN_PREPROC_FILTER:
          return gaussian_filter(image.impl(), size);
          break;
        case BOX_PREPROC_FILTER:
          return box_filter(image.impl(), Vector2i(int(size), int(size)));
          break;
        default:
          VW_ASSERT(0, ArgumentErr() << "Unrecognized Preprocessing Filter");
          return ImageViewRef<typename ImageT::pixel_type>();
      }
    }
  }
}

namespace vw {
  struct AbsDifferenceFunctor : BinaryReturnTemplateType<DifferenceType> {
    template <class Arg1T, class Arg2T>
    typename result<AbsDifferenceFunctor(Arg1T, Arg2T)>::type
    inline operator()(Arg1T const& arg1, Arg2T const& arg2) const {
      if (arg1 > arg2)
        return arg1 - arg2;
      else
        return arg2 - arg1;
    }
  };

  template <class Image1T, class Image2T>
  inline BinaryPerPixelView<Image1T, Image2T, AbsDifferenceFunctor> abs_difference(ImageViewBase<Image1T> const& image1, ImageViewBase<Image2T> const& image2) {
    return BinaryPerPixelView<Image1T, Image2T, AbsDifferenceFunctor>(image1.impl(), image2.impl(), AbsDifferenceFunctor());
  }

  struct SqDifferenceFunctor : BinaryReturnTemplateType<DifferenceType> {
    template <class Arg1T, class Arg2T>
    typename result<SqDifferenceFunctor(Arg1T, Arg2T)>::type
    inline operator()(Arg1T const& arg1, Arg2T const& arg2) const {
      return (arg1 - arg2) * (arg1 - arg2);
    }
  };

  template <class Image1T, class Image2T>
  inline BinaryPerPixelView<Image1T, Image2T, SqDifferenceFunctor> sq_difference(ImageViewBase<Image1T> const& image1, ImageViewBase<Image2T> const& image2) {
    return BinaryPerPixelView<Image1T, Image2T, SqDifferenceFunctor>(image1.impl(), image2.impl(), SqDifferenceFunctor());
  }

  template<class ImageT>
  ImageView<typename ImageT::pixel_type> box_filter(ImageViewBase<ImageT> const& img,  Vector2i const& kernSize) {
    typedef typename ImageT::pixel_type PixelT;

    ImageView<PixelT> src = img.impl();
    
    ImageView<PixelT> result(src.cols(), src.rows());
    
    Vector<PixelT> cSum(src.cols());

    // Seed the column sum buffer
    for (int x = 0; x < src.cols(); x++) {
      cSum(x) = 0;
      for (int ky = 0; ky < kernSize.y(); ky++) {
        cSum(x) += src(x, ky);
      }
    }
    
    for (int y = 0; y < src.rows() - kernSize.y(); y++) {
      // Seed the row sum
      PixelT rsum = 0;
      for (int i = 0; i < kernSize.x(); i++) {
        rsum += cSum(i);
      }

      for (int x = 0; x < src.cols() - kernSize.x(); x++) {
        result(x + kernSize.x() / 2, y + kernSize.y() / 2) = rsum;
        // Update the row sum
        rsum += cSum(x + kernSize.x()) - cSum(x);
      }

      // Update the column sum
      for (int i = 0; i < src.cols(); i++) {
        cSum(i) += src(i, y + kernSize.y()) - src(i, y);
      }
    }
   
    return result / (kernSize.x() * kernSize.y()); 
  }
}

namespace vw {
  namespace stereo {

    template <class PixelT>
    class CostFunction {
      public:
        virtual ImageView<PixelT> calculate(BBox2i const& left_bbox, Vector2i const& offset) const = 0;
        virtual int cols() const = 0;
        virtual int rows() const = 0;
        virtual int sample_size() const = 0; // What is the side length of the square of surrounding pixels needed to calculate the cost for a single pixel?
        virtual ~CostFunction() {}
    };

    template <class PixelT>
    class AbsDifferenceCost : public CostFunction<PixelT> {
      private:
        int m_kern_size;
        ImageView <PixelT> m_left, m_right;
      public:
        template <class ImageT>
        AbsDifferenceCost(ImageViewBase<ImageT> const& left, ImageViewBase<ImageT> const& right, int kern_size) : CostFunction<PixelT>(), 
          m_kern_size(kern_size),
          m_left(copy(left.impl())),
          m_right(copy(right.impl())) {
          VW_ASSERT(m_left.cols() == m_right.cols(), ArgumentErr() << "Left and right images not the same width");
          VW_ASSERT(m_left.rows() == m_right.rows(), ArgumentErr() << "Left and right images not the same height");
        }

        ImageView<PixelT> calculate(BBox2i const& left_bbox, Vector2i const& offset) const {
          CropView<ImageView<PixelT> > left_window(m_left, left_bbox);
          CropView<ImageView<PixelT> > right_window(m_right, left_bbox + offset);
          
          return box_filter(abs_difference(left_window, right_window), Vector2i(m_kern_size, m_kern_size));
        }

        int cols() const { return m_left.cols(); }
        int rows() const { return m_left.rows(); }
        int sample_size() const { return m_kern_size; }
    };

    template <class PixelT>
    class SqDifferenceCost : public CostFunction<PixelT> {
      private:
        int m_kern_size;
        ImageView <PixelT> m_left, m_right;
      public:
        template <class ImageT>
        SqDifferenceCost(ImageViewBase<ImageT> const& left, ImageViewBase<ImageT> const& right, int kern_size) : CostFunction<PixelT>(), 
          m_kern_size(kern_size),
          m_left(copy(left.impl())),
          m_right(copy(right.impl())) {
          VW_ASSERT(m_left.cols() == m_right.cols(), ArgumentErr() << "Left and right images not the same width");
          VW_ASSERT(m_left.rows() == m_right.rows(), ArgumentErr() << "Left and right images not the same height");
        }

        ImageView<PixelT> calculate(BBox2i const& left_bbox, Vector2i const& offset) const {
          CropView<ImageView<PixelT> > left_window(m_left, left_bbox);
          CropView<ImageView<PixelT> > right_window(m_right, left_bbox + offset);
          
          return box_filter(sq_difference(left_window, right_window), Vector2i(m_kern_size, m_kern_size));
        }

        int cols() const { return m_left.cols(); }
        int rows() const { return m_left.rows(); }
        int sample_size() const { return m_kern_size; }
    };

    template <class PixelT>
    class NormXCorrCost : public CostFunction<PixelT> {
      private:
        int m_kern_size;
        
        ImageView<PixelT> m_left;
        ImageView<PixelT> m_left_mean;
        ImageView<PixelT> m_left_variance;
  
        ImageView<PixelT> m_right;
        ImageView<PixelT> m_right_mean;
        ImageView<PixelT> m_right_variance;

      public:
        template <class ImageT>
        NormXCorrCost(ImageViewBase<ImageT> const& left, ImageViewBase<ImageT> const& right, int kern_size) : CostFunction<PixelT>(), 
          m_kern_size(kern_size), 
          m_left(copy(left.impl())), 
          m_right(copy(right.impl())) {
          VW_ASSERT(m_left.cols() == m_right.cols(), ArgumentErr() << "Left and right images not the same width");
          VW_ASSERT(m_left.rows() == m_right.rows(), ArgumentErr() << "Left and right images not the same height");

          vw::ImageView<PixelT> left_mean_sq = box_filter(m_left * m_left, Vector2i(m_kern_size, m_kern_size));
          vw::ImageView<PixelT> right_mean_sq = box_filter(m_right * m_right, Vector2i(m_kern_size, m_kern_size));
          
          m_left_mean = box_filter(m_left, Vector2i(m_kern_size, m_kern_size));
          m_left_variance = left_mean_sq - m_left_mean * m_left_mean;

          m_right_mean = box_filter(m_right, Vector2i(m_kern_size, m_kern_size));
          m_right_variance = right_mean_sq - m_right_mean * m_right_mean;
        }
        
        ImageView<PixelT> calculate(BBox2i const& left_bbox, Vector2i const& offset) const {
          CropView<ImageView<PixelT> > left_window(m_left, left_bbox);
          CropView<ImageView<PixelT> > left_mean_window(m_left_mean, left_bbox);
          CropView<ImageView<PixelT> > left_variance_window(m_left_variance, left_bbox);

          CropView<ImageView<PixelT> > right_window(m_right, left_bbox + offset);
          CropView<ImageView<PixelT> > right_mean_window(m_right_mean, left_bbox + offset);
          CropView<ImageView<PixelT> > right_variance_window(m_right_variance, left_bbox + offset);

          ImageView<PixelT> left_right_mean = box_filter(left_window * right_window, Vector2i(m_kern_size, m_kern_size));
         
          return -(pow(left_right_mean - left_mean_window * right_mean_window, 2) / left_variance_window / right_variance_window);
        }

        int cols() const { return m_left.cols(); }
        int rows() const { return m_left.rows(); }
        int sample_size() const { return m_kern_size; }
    };

    template <class PixelT>
    class BlurCost : public CostFunction<PixelT> {
      private:
        boost::shared_ptr<CostFunction<PixelT> > m_base_cost;
        int m_blur_size;
      public:
        BlurCost(boost::shared_ptr<CostFunction<PixelT> > base_cost, int blur_size) : CostFunction<PixelT>(), m_base_cost(base_cost), m_blur_size(blur_size) {}

        ImageView<PixelT> calculate(BBox2i const& left_bbox, Vector2i const& offset) const {
          return box_filter(m_base_cost->calculate(left_bbox, offset), Vector2i(m_blur_size, m_blur_size));
        }
        
        int cols() const { return m_base_cost->cols(); }
        int rows() const { return m_base_cost->rows(); }
        int sample_size() const { 
            return m_blur_size + m_base_cost->sample_size();
        }
    };

    template <class ScoreT>
    struct DisparityScore {
      ScoreT best, worst;
      int32 hdisp, vdisp;
      
      DisparityScore() {
        best = ScalarTypeLimits<ScoreT>::highest();
        worst = ScalarTypeLimits<ScoreT>::lowest();
        hdisp = vdisp = 0;
      }
    };

    template <class PixelT>
    ImageView<PixelDisparity<int32> > kylecorrelate(boost::shared_ptr<CostFunction<PixelT> > cost_function,
                                                    BBox2i const& search_window,
                                                    ProgressCallback const& progress = ProgressCallback::dummy_instance()
                                                    ) {

      int width = cost_function->cols();
      int height = cost_function->rows();

      ImageView<DisparityScore<PixelT> > result_buf(width, height);
      ImageView<PixelT> cost_buf(width, height);

      // Init the result_buf
      for (int x = 0; x < width; x++) {
        for (int y = 0; y < height; y++) {
          result_buf(x, y) = DisparityScore<PixelT>();
        }
      }

      int currIteration = 0;
      int totalIterations = (search_window.width() + 1) * (search_window.height() + 1);

      for (int dx = search_window.min().x(); dx <= search_window.max().x(); dx++) {
        for (int dy = search_window.min().y(); dy <= search_window.max().y(); dy++) {
          BBox2i corr_window((dx < 0) ? (-dx) : 0,
                             (dy < 0) ? (-dy) : 0,
                             width - abs(dx),
                             height - abs(dy));

          CropView<ImageView<DisparityScore<PixelT> > > result_buf_window(result_buf, corr_window);
          CropView<ImageView<PixelT> > cost_buf_window(cost_buf, corr_window);

          // Calculate cost function
          cost_buf_window = cost_function->calculate(corr_window, Vector2i(dx, dy));

          for (int x = 0; x < corr_window.width(); x++) {
            for (int y = 0; y < corr_window.height(); y++) {
              if (cost_buf_window(x, y) < result_buf_window(x, y).best) {
                result_buf_window(x, y).best = cost_buf_window(x, y);
                result_buf_window(x, y).hdisp = dx;
                result_buf_window(x, y).vdisp = dy;
              }
              if (cost_buf_window(x, y) > result_buf_window(x, y).worst) {
                result_buf_window(x, y).worst = cost_buf_window(x, y);
              }
            }
          }

          progress.report_fractional_progress(++currIteration, totalIterations);
          progress.abort_if_requested();
        }
      }

      // convert from the local result buffer to the return format
      ImageView<PixelDisparity<int32> > result(width, height);

      for (int x = 0; x < width; x++) {
        for (int y = 0; y < height; y++) {
          if (result_buf(x, y).best == ScalarTypeLimits<PixelT>::highest()) {
            result(x, y) = PixelDisparity<int32>();
          }
          else {
            result(x, y) = PixelDisparity<int32>(result_buf(x, y).hdisp, result_buf(x, y).vdisp);
          }
        }
      }

      progress.report_finished();

      return result; 
    }

    template <class PixelT>
    ImageView<PixelT> correlate_pixel(boost::shared_ptr<CostFunction<PixelT> > cost_function, BBox2i const& search_window, Vector2i const& pixel) {

      ImageView<PixelT> result(search_window.width() + 1, search_window.height() + 1);    
      
      BBox2i corr_window(pixel.x() - cost_function->sample_size() / 2,
                         pixel.y() - cost_function->sample_size() / 2,
                         cost_function->sample_size() + 1,
                         cost_function->sample_size() + 1);

      
      for (int dx = search_window.min().x(); dx <= search_window.max().x(); dx++) {
        for (int dy = search_window.min().y(); dy <= search_window.max().y(); dy++) {
          int resultX = dx - search_window.min().x();
          int resultY = dy - search_window.min().y();
          result(resultX, resultY) = cost_function->calculate(corr_window, Vector2i(dx, dy))(cost_function->sample_size() / 2, cost_function->sample_size() / 2);
        }
      }

      return result;
    }

  }
}

#endif
