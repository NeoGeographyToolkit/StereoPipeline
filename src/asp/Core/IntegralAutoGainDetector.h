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


#ifndef __ASP_CORE_INTEGRAL_AUTO_GAIN_DETECTOR_H__
#define __ASP_CORE_INTEGRAL_AUTO_GAIN_DETECTOR_H__

#include <vw/InterestPoint/InterestData.h>
#include <vw/InterestPoint/IntegralDetector.h>
#include <vw/InterestPoint/IntegralInterestOperator.h>
#include <boost/foreach.hpp>
#include <vw/Image/Statistics.h>

namespace asp {

  class IntegralAutoGainDetector : public vw::ip::InterestDetectorBase<IntegralAutoGainDetector >,
                                   vw::ip::IntegralInterestPointDetector<vw::ip::OBALoGInterestOperator> {
  public:
    // Clear ambiguity of which impl to use. Scope doesn't work.
    using vw::ip::InterestDetectorBase<IntegralAutoGainDetector>::impl;
    using vw::ip::InterestDetectorBase<IntegralAutoGainDetector>::operator();

    IntegralAutoGainDetector( size_t max_points = 200 )
      : vw::ip::IntegralInterestPointDetector<vw::ip::OBALoGInterestOperator>( vw::ip::OBALoGInterestOperator(0), IP_DEFAULT_SCALES, max_points ) {}

    /// Detect Interest Points in the source image.
    template <class ViewT>
    vw::ip::InterestPointList process_image(vw::ImageViewBase<ViewT> const& image ) const {
      using namespace vw;
      typedef ImageView<typename PixelChannelType<typename ViewT::pixel_type>::type> ImageT;
      typedef ip::ImageInterestData<ImageT,ip::OBALoGInterestOperator> DataT;
      Timer total("\t\tTotal elapsed time", DebugMessage, "interest_point");

      // The input image is a lazy view. We'll rasterize so we're not
      // hitting the cache all of the image.
      ImageT original_image = image.impl();

      // The ImageInterestData structure doesn't really apply to
      // OBALoG. We don't need access to the original image after
      // we've made the integral image. To avoid excessive copying,
      // we're making an empty image to feed that structure.
      ImageT empty_image;

      // Producing Integral Image
      ImageT integral_image;
      {
        vw_out(DebugMessage, "interest_point") << "\tCreating Integral Image ...";
        Timer t("done, elapsed time", DebugMessage, "interest_point");
        integral_image = ip::IntegralImage( original_image );
      }

      // Creating Scales
      std::deque<DataT> interest_data;
      interest_data.push_back( DataT(empty_image, integral_image) );
      interest_data.push_back( DataT(empty_image, integral_image) );

      // Priming scales
      vw::ip::InterestPointList new_points;
      {
        vw_out(DebugMessage, "interest_point") << "\tScale 0 ... ";
        Timer t("done, elapsed time", DebugMessage, "interest_point");
        m_interest( interest_data[0], 0 );
      }
      {
        vw_out(DebugMessage, "interest_point") << "\tScale 1 ... ";
        Timer t("done, elapsed time", DebugMessage, "interest_point");
        m_interest( interest_data[1], 1 );
      }

      // Finally processing scales
      for ( int scale = 2; scale < m_scales; scale++ ) {

        interest_data.push_back( DataT(empty_image, integral_image) );
        {
          vw_out(DebugMessage, "interest_point") << "\tScale " << scale << " ... ";
          Timer t("done, elapsed time", DebugMessage, "interest_point");
          m_interest( interest_data[2], scale );
        }

        ip::InterestPointList scale_points;

        // Detecting interest points in middle
        int32 cols = original_image.cols() - 2;
        int32 rows = original_image.rows() - 2;
        typedef typename DataT::interest_type::pixel_accessor AccessT;

        AccessT l_row = interest_data[0].interest().origin();
        AccessT m_row = interest_data[1].interest().origin();
        AccessT h_row = interest_data[2].interest().origin();
        l_row.advance(1,1); m_row.advance(1,1); h_row.advance(1,1);
        for ( int32 r=0; r < rows; r++ ) {
          AccessT l_col = l_row;
          AccessT m_col = m_row;
          AccessT h_col = h_row;
          for ( int32 c=0; c < cols; c++ ) {
            if ( is_extrema( l_col, m_col, h_col ) ) {
              scale_points.push_back(ip::InterestPoint(c+1,r+1,
                                                       m_interest.float_scale(scale-1),
                                                       *m_col) );
            }
            l_col.next_col();
            m_col.next_col();
            h_col.next_col();
          }
          l_row.next_row();
          m_row.next_row();
          h_row.next_row();
        }

        VW_OUT(DebugMessage, "interest_point") << "\tPrior to thresholding there was: "
                                               << scale_points.size() << "\n";

        // Remove all interest points in the bottom 0.1% of our interest point range
        float imin, imax;
        min_max_pixel_values( interest_data[1].interest(), imin, imax );
        float threshold_lvl = imin + 0.001 * ( imax - imin );
        VW_OUT(DebugMessage, "interest_point") << "\tInterest threshold for scale: " << threshold_lvl << "\n";

        // Thresholding (in OBALOG this also does Harris)
        threshold(scale_points, interest_data[1], scale-1, threshold_lvl);

        VW_OUT(DebugMessage, "interest_point") << "\tAfter thresholding there was: "
                                               << scale_points.size() << "\n";

        // Appending to the greater set
        new_points.insert(new_points.end(),
                          scale_points.begin(),
                          scale_points.end());

        // Deleting lowest
        interest_data.pop_front();
      }

      // Are all points good?
      if ( m_max_points < int(new_points.size()) && m_max_points > 0 ) {
        VW_OUT(DebugMessage, "interest_point") << "\tCulling ...\n";
        Timer t("elapsed time", DebugMessage, "interest_point");

        int original_num_points = new_points.size();

        // Sort the interest of the points and pull out the top amount that the user wants
        new_points.sort();
        VW_OUT(DebugMessage, "interest_point") << "\t     Best IP : " << new_points.front().interest << std::endl;
        VW_OUT(DebugMessage, "interest_point") << "\t     Worst IP: " << new_points.back().interest << std::endl;
        new_points.resize( m_max_points );

        VW_OUT(DebugMessage, "interest_point") << "\t     (removed " << original_num_points - new_points.size() << " interest points, " << new_points.size() << " remaining.)\n";
      } else {
        VW_OUT(DebugMessage, "interest_point") << "\t     Not culling anything.\n";
      }

      return new_points;
    }

  protected:

    template <class DataT>
    inline void threshold( vw::ip::InterestPointList& points,
                           DataT const& img_data,
                           int const& scale, float threshold_lvl ) const {
      vw::ip::InterestPointList::iterator pos = points.begin();
      while (pos != points.end()) {
        if ( pos->interest < threshold_lvl ||
             !m_interest.threshold(*pos,
                                   img_data, scale) ) {
          pos = points.erase(pos);
        } else {
          pos++;
        }
      }
    }
  };

}

#endif//__ASP_CORE_INTEGRAL_AUTO_GAIN_DETECTOR_H__
