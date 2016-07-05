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

#ifndef __ASP_MAPPING_NEW_CORRELATION_H__
#define __ASP_MAPPING_NEW_CORRELATION_H__

#include <vw/Core/Exception.h>
#include <vw/Core/Stopwatch.h>
#include <vw/Core/Thread.h>
#include <vw/Core/Debugging.h>
#include <vw/Math/BBox.h>
#include <vw/Image/Algorithms.h>
#include <vw/Image/AlgorithmFunctions.h>
#include <vw/Image/PerPixelAccessorViews.h>
#include <vw/Image/BlockRasterize.h>
#include <vw/Stereo/StereoModel.h>
#include <vw/Stereo/Correlation.h>
#include <vw/Stereo/CorrelationView.h>
#include <vw/Stereo/Correlate.h>
#include <vw/Stereo/DisparityMap.h>
#include <vw/Stereo/PreFilter.h>

#include <asp/Core/StereoSettings.h>
#include <vw/Image/BlobIndex.h>
#include <vw/Image/ErodeView.h>


#include <vw/FileIO.h>
#include <ctime>
#include <boost/foreach.hpp>
#include <boost/shared_ptr.hpp>
/*
// DEBUG!!!!!!!!!
#include "opencv2/core/core.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/calib3d/calib3d.hpp"
*/

/**
  Testbed for new stereo algorithm development!

  TODO: 
  - Clean up existing code
  - Add some diagnostic tools for looking at individual pixel results etc.
  - See what is going wrong with the existing correlation.
  - Start testing out new stuff!

*/


namespace vw {
namespace stereo {

  /// An image view for performing pyramid image correlation (Faster than CorrelationView).
  template <class Image1T, class Image2T, class Mask1T, class Mask2T>
  class NewCorrelationView : public ImageViewBase<NewCorrelationView<Image1T,Image2T, Mask1T, Mask2T> > {

    Image1T          m_left_image;
    Image2T          m_right_image;
    Mask1T           m_left_mask;
    Mask2T           m_right_mask;
    boost::shared_ptr<vw::stereo::StereoModel> m_stereo_model;
    
    // These two variables pick a prefilter which is applied to each pyramid level
    vw::uint16 m_prefilter_mode; ///< 0 = None, 1 = Gaussian Blur, 2 = Log Filter
    float m_prefilter_width;     ///< Preprocessing filter width
    BBox2i           m_search_region;
    Vector2i         m_kernel_size;
    stereo::CostFunctionType m_cost_type;
    int              m_corr_timeout;
    // How long it takes to do one corr op with given kernel and cost function
    double m_seconds_per_op;
    float  m_consistency_threshold; // < 0 = means don't do a consistency check
    int32  m_max_level_by_search;

    /// Downsample a mask by two.
    /// - If at least two mask pixels in a 2x2 region are on, the output pixel is on.
    struct SubsampleMaskByTwoFunc : public ReturnFixedType<uint8> {
      BBox2i work_area() const { return BBox2i(0,0,2,2); }

      template <class PixelAccessorT>
      typename boost::remove_reference<typename PixelAccessorT::pixel_type>::type
      operator()( PixelAccessorT acc ) const {

        typedef typename PixelAccessorT::pixel_type PixelT;

        uint8 count = 0;
        if ( *acc ) count++;
        acc.next_col();
        if ( *acc ) count++;
        acc.advance(-1,1);
        if ( *acc ) count++;
        acc.next_col();
        if ( *acc ) count++;
        if ( count > 1 )
          return PixelT(ScalarTypeLimits<PixelT>::highest());
        return PixelT();
      }
    }; // End struct SubsampleMaskByTwoFunc

    template <class ViewT>
    SubsampleView<UnaryPerPixelAccessorView<EdgeExtensionView<ViewT,ZeroEdgeExtension>, SubsampleMaskByTwoFunc> >
    subsample_mask_by_two( ImageViewBase<ViewT> const& input ) const {
      return subsample(per_pixel_accessor_filter(input.impl(), SubsampleMaskByTwoFunc()),2);
    }

    /// Create the image pyramids needed by the prerasterize function.
    /// - Most of this function is spent figuring out the correct ROIs to use.
    bool build_image_pyramids(vw::BBox2i const& bbox, int32 const max_pyramid_levels,
                              std::vector<ImageView<typename Image1T::pixel_type> > & left_pyramid,
                              std::vector<ImageView<typename Image2T::pixel_type> > & right_pyramid,
                              std::vector<ImageView<typename Mask1T::pixel_type > > & left_mask_pyramid,
                              std::vector<ImageView<typename Mask2T::pixel_type > > & right_mask_pyramid) const;

    /// Applies the specified prefilter to a pair of images
    void prefilter_images(ImageView<typename Image1T::pixel_type> &left_image,
                          ImageView<typename Image2T::pixel_type> &right_image) const;


  public:
    typedef PixelMask<Vector2i> pixel_type;
    typedef PixelMask<Vector2i> result_type;
    typedef ProceduralPixelAccessor<NewCorrelationView> pixel_accessor;


    NewCorrelationView( ImageViewBase<Image1T> const& left,
                        ImageViewBase<Image2T> const& right,
                        ImageViewBase<Mask1T > const& left_mask,
                        ImageViewBase<Mask2T > const& right_mask,
                        boost::shared_ptr<vw::stereo::StereoModel> const& stereo_model,
                        vw::uint16 prefilter_mode, float prefilter_width,
                        BBox2i const& search_region, Vector2i const& kernel_size,
                        stereo::CostFunctionType cost_type,
                        int corr_timeout, double seconds_per_op,
                        float consistency_threshold,
                        int32 max_pyramid_levels) :
      m_left_image(left.impl()),     m_right_image(right.impl()),
      m_left_mask(left_mask.impl()), m_right_mask(right_mask.impl()),
      m_stereo_model(stereo_model),
      m_prefilter_mode(prefilter_mode), m_prefilter_width(prefilter_width),
      m_search_region(search_region), m_kernel_size(kernel_size),
      m_cost_type(cost_type),
      m_corr_timeout(corr_timeout), m_seconds_per_op(seconds_per_op),
      m_consistency_threshold(consistency_threshold){
      
      // Calculating max pyramid levels according to the supplied search region.
      int32 largest_search = max( search_region.size() );
      m_max_level_by_search = std::floor(std::log(float(largest_search))/std::log(2.0f)) - 1;
      if ( m_max_level_by_search > max_pyramid_levels )
        m_max_level_by_search = max_pyramid_levels;
      if ( m_max_level_by_search < 0 )
        m_max_level_by_search = 0;
    }

    // Standard required ImageView interfaces
    inline int32 cols  () const { return m_left_image.cols(); }
    inline int32 rows  () const { return m_left_image.rows(); }
    inline int32 planes() const { return 1; }

    inline pixel_accessor origin() const { return pixel_accessor( *this, 0, 0 ); }
    inline pixel_type operator()( int32 /*i*/, int32 /*j*/, int32 /*p*/ = 0) const {
      vw_throw( NoImplErr() << "NewCorrelationView::operator()(....) has not been implemented." );
      return pixel_type();
    }

    /// Block rasterization section that does actual work
    typedef CropView<ImageView<pixel_type> > prerasterize_type;
    inline prerasterize_type prerasterize(BBox2i const& bbox) const;

    template <class DestT>
    inline void rasterize(DestT const& dest, BBox2i const& bbox) const {
      vw::rasterize(prerasterize(bbox), dest, bbox);
    }
    
  }; // End class NewCorrelationView

  template <class Image1T, class Image2T, class Mask1T, class Mask2T>
  NewCorrelationView<Image1T,Image2T,Mask1T,Mask2T>
  new_correlate( ImageViewBase<Image1T> const& left,
                 ImageViewBase<Image2T> const& right,
                 ImageViewBase<Mask1T > const& left_mask,
                 ImageViewBase<Mask2T > const& right_mask,
                 boost::shared_ptr<vw::stereo::StereoModel> const& stereo_model,
                 vw::uint16 prefilter_mode, float prefilter_width,
                 BBox2i const& search_region, Vector2i const& kernel_size,
                 stereo::CostFunctionType cost_type,
                 int corr_timeout, double seconds_per_op,
                 float consistency_threshold,
                 int32 max_pyramid_levels) {
    typedef NewCorrelationView<Image1T,Image2T,Mask1T,Mask2T> result_type;
    return result_type( left.impl(),      right.impl(), 
                        left_mask.impl(), right_mask.impl(),
                        stereo_model,
                        prefilter_mode, prefilter_width,
                        search_region,
                        kernel_size, cost_type,
                        corr_timeout, seconds_per_op,
                        consistency_threshold, max_pyramid_levels );
  }

} // namespace stereo
} // namespace vw

#include <asp/Tools/NewCorrelation.tcc>


#endif //__ASP_STEREO_MAPPING_NEW_CORRELATION_H__
