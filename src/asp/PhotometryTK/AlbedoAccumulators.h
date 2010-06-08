// __BEGIN_LICENSE__
// Copyright (C) 2006-2010 United States Government as represented by
// the Administrator of the National Aeronautics and Space Administration.
// All Rights Reserved.
// __END_LICENSE__


#ifndef __ASP_PHO_ALBEDO_ACCUMULATORS_H__
#define __ASP_PHO_ALBEDO_ACCUMULATORS_H__

#include <asp/PhotometryTK/ImageAccumulators.h>
#include <vw/Image/ImageView.h>
#include <vw/Image/Algorithms.h>

namespace asp {
namespace pho {

  template <class PixelT>
  class AlbedoInitAccumulator : public ImageAccumulatorBase<AlbedoInitAccumulator<PixelT> > {
    typedef typename vw::CompoundChannelCast<PixelT, double>::type accum_pixel_type;
    typedef typename vw::PixelChannelType<PixelT>::type channel_type;
    vw::ImageView<accum_pixel_type> m_sum;
    vw::ImageView<double> m_sum_weight;
    void reset( void ) {
      vw::fill(m_sum, accum_pixel_type() );
      vw::fill(m_sum_weight, 0 );
    }
  public:
    AlbedoInitAccumulator( vw::int32 const& cols, vw::int32 const& rows ) :
    m_sum(cols,rows), m_sum_weight(cols,rows) {
      this->reset();
    }

    // Accumulate
    template <class IViewT, class RViewT>
    void operator()( vw::ImageViewBase<IViewT> const& image,
                     vw::ImageViewBase<RViewT> const& reflectance,
                     double const& t ) {
      m_sum_weight += select_channel(image.impl(),1);
      m_sum += vw::channel_cast<double>(select_channel(image.impl(),0))*
        vw::channel_cast<double>(select_channel(image.impl(),1)) / ( t*reflectance.impl() );
    }

    // Result
    typedef vw::ImageView<PixelT> result_type;
    result_type result(void) {
      // Normalize
      result_type r = m_sum/m_sum_weight;
      // Apply alpha
      select_channel(r,1) = vw::channel_cast_rescale<channel_type>(threshold(m_sum_weight,0));
      this->reset();
      return r;
    }
  };

  template <class PixelT>
  class AlbedoInitNRAccumulator : public ImageAccumulatorBase<AlbedoInitNRAccumulator<PixelT> > {
    typedef typename vw::CompoundChannelCast<PixelT, double>::type accum_pixel_type;
    typedef typename vw::PixelChannelType<PixelT>::type channel_type;
    vw::ImageView<accum_pixel_type> m_sum;
    vw::ImageView<double> m_sum_weight;
    void reset( void ) {
      vw::fill(m_sum, accum_pixel_type() );
      vw::fill(m_sum_weight, 0 );
    }
  public:
    AlbedoInitNRAccumulator( vw::int32 const& cols, vw::int32 const& rows ) :
    m_sum(cols,rows), m_sum_weight(cols,rows) {
      this->reset();
    }

    // Accumulate
    template <class IViewT>
    void operator()( vw::ImageViewBase<IViewT> const& image,
                     double const& t ) {
      m_sum_weight += select_channel(image.impl(),1);
      m_sum += vw::channel_cast<double>(select_channel(image.impl(),0)) *
        vw::channel_cast<double>(select_channel(image.impl(),1)) / t;
    }

    // Result
    typedef vw::ImageView<PixelT> result_type;
    result_type result(void) {
      result_type r = m_sum/m_sum_weight;
      select_channel(r,1) = vw::channel_cast_rescale<channel_type>(threshold(m_sum_weight,0));
      this->reset();
      return r;
    }
  };

  template <class PixelT>
  class AlbedoDeltaAccumulator : public ImageAccumulatorBase<AlbedoDeltaAccumulator<PixelT> > {
    typedef typename vw::PixelChannelType<PixelT>::type channel_type;
    vw::ImageView<double> m_nominator, m_denominator;
    vw::ImageView<double> m_t_grad, m_t_error; // temporaries to avoid
                                               // excessive memory allocations.
    vw::ImageView<double> m_sum_weight;
    void reset( void ) {
      vw::fill(m_nominator, 0 );
      vw::fill(m_denominator, 0 );
      vw::fill(m_sum_weight, 0 );
    }
  public:
    AlbedoDeltaAccumulator( vw::int32 const& cols, vw::int32 const& rows ) :
    m_nominator(cols,rows), m_denominator(cols,rows),
      m_t_grad(cols,rows), m_t_error(cols,rows), m_sum_weight(cols,rows) {
      this->reset();
    }

    // Accumulate
    template <class IViewT, class AViewT, class RViewT>
    void operator()( vw::ImageViewBase<IViewT> const& image,
                     vw::ImageViewBase<AViewT> const& previous_albedo,
                     vw::ImageViewBase<RViewT> const& reflectance,
                     double const& t ) {
      // Intermediates
      m_t_grad = t * vw::channel_cast<double>(reflectance);
      m_t_error = vw::channel_cast<double>(select_channel(image,0)) -
        t * vw::channel_cast<double>(select_channel(previous_albedo,0)) * vw::channel_cast<double>(reflectance);

      // Update accumulators
      m_nominator += m_t_grad * m_t_error *
        vw::channel_cast<double>(select_channel(image,1));
      m_denominator += m_t_grad * m_t_grad *
        vw::channel_cast<double>(select_channel(image,1));

      m_sum_weight += select_channel(image,1);
    }

    // Result
    typedef vw::ImageView<PixelT> result_type;
    result_type result(void) {
      result_type r(m_t_error.cols(), m_t_error.rows());
      select_channel(r,0) = m_nominator / m_denominator;
      select_channel(r,1) = vw::channel_cast_rescale<channel_type>(threshold(m_sum_weight,0));
      this->reset();
      return r;
    }
  };

  template <class PixelT>
  class AlbedoDeltaNRAccumulator : public ImageAccumulatorBase<AlbedoDeltaNRAccumulator<PixelT> > {
    typedef typename vw::PixelChannelType<PixelT>::type channel_type;
    vw::ImageView<double> m_nominator, m_denominator;
    vw::ImageView<double> m_t_error; // temporary to avoid
                                     // excessive memory allocations.
    vw::ImageView<double> m_sum_weight;
    void reset( void ) {
      vw::fill(m_nominator, 0 );
      vw::fill(m_denominator, 0 );
      vw::fill(m_sum_weight, 0 );
    }
  public:
    AlbedoDeltaNRAccumulator( vw::int32 const& cols, vw::int32 const& rows ) :
    m_nominator( cols, rows ), m_denominator( cols, rows ),
      m_t_error( cols, rows ), m_sum_weight( cols, rows ) {
      this->reset(); // OCD
    }

    // Accumulate
    template <class IViewT, class AViewT>
    void operator()( vw::ImageViewBase<IViewT> const& image,
                     vw::ImageViewBase<AViewT> const& previous_albedo,
                     double const& t ) {
      // Intermediate
      m_t_error = vw::channel_cast<double>(select_channel(image,0)) -
        t*vw::channel_cast<double>(select_channel(previous_albedo,0));

      // Update accumulators
      m_nominator += t * m_t_error *
        vw::channel_cast<double>(select_channel(image,1));
      m_denominator += t * t *
        vw::channel_cast<double>(select_channel(image,1));

      m_sum_weight += select_channel(image,1);
    }

    // Result
    typedef vw::ImageView<PixelT> result_type;
    result_type result(void) {
      result_type r(m_t_error.cols(), m_t_error.rows());
      select_channel(r,0) = m_nominator / m_denominator;
      select_channel(r,1) = vw::channel_cast_rescale<channel_type>(threshold(m_sum_weight,0));
      this->reset();
      return r;
    }
  };

  template <class PixelT>
  class Albedo2BandAccumulator : public ImageAccumulatorBase<Albedo2BandAccumulator<PixelT> > {
    typedef typename vw::CompoundChannelCast<PixelT, double>::type accum_pixel_type;
    typedef typename vw::PixelChannelType<PixelT>::type channel_type;
    vw::ImageView<double> m_low_f_sum, m_sum_weight;
    vw::ImageView<accum_pixel_type> m_high_f_sum;

    // Intermediates
    vw::ImageView<double> m_curr_sum, m_curr_low, m_curr_high;

    void reset( void ) {
      vw::fill( m_low_f_sum, 0 );
      vw::fill( m_sum_weight, 0 );
      vw::fill( m_high_f_sum, accum_pixel_type() );
    }

    struct UpdateHighFFunctor {
      template <class Arg1T, class Arg2T, class Arg3T>
      struct result {
        typedef Arg1T type;
      };

      template <class Arg1T, class Arg2T, class Arg3T>
      inline Arg1T& operator()( Arg1T& high_f_result, Arg2T const& input, Arg3T const& input_alpha  ) const {
        if ( input_alpha > high_f_result[1] )
          high_f_result[0] = input;
        return high_f_result;
      }
    };

  public:
    Albedo2BandAccumulator( vw::int32 const& cols, vw::int32 const& rows ) :
    m_low_f_sum(cols,rows), m_sum_weight(cols,rows), m_high_f_sum(cols,rows),
    m_curr_sum(cols,rows), m_curr_low(cols,rows), m_curr_high(cols,rows) {
      this->reset();
    }

    // Accumulate
    template <class IViewT, class RViewT>
    void operator()( vw::ImageViewBase<IViewT> const& image,
                     vw::ImageViewBase<RViewT> const& reflectance,
                     double const& t ) {
      m_sum_weight += select_channel(image.impl(),1);

      m_curr_sum = vw::channel_cast<double>(select_channel(image.impl(),0))*
        vw::channel_cast<double>(select_channel(image.impl(),1)) / ( t*reflectance.impl() );
      m_curr_low = gaussian_filter( m_curr_sum, 2.0 );
      m_curr_high = m_curr_sum - m_curr_low;
      m_low_f_sum += m_curr_low;

      // High only makes it in if the input alpha is higher than the currently loaded
      for_each_pixel( m_high_f_sum, m_curr_high, select_chanel(image.impl(),1) );
    }

    // Result
    typedef vw::ImageView<PixelT> result_type;
    result_type result(void) {
      // Normalize
      result_type r = m_low_f_sum/m_sum_weight + m_high_f_sum;
      // Apply alpha
      select_channel(r,1) = vw::channel_cast_rescale<channel_type>(threshold(m_sum_weight,0));
      this->reset();
      return r;
    }
  };

  template <class PixelT>
  class Albedo2BandNRAccumulator : public ImageAccumulatorBase<Albedo2BandNRAccumulator<PixelT> > {
    typedef typename vw::CompoundChannelCast<PixelT, double>::type accum_pixel_type;
    typedef typename vw::PixelChannelType<PixelT>::type channel_type;
    vw::ImageView<double> m_low_f_sum, m_sum_weight;
    vw::ImageView<accum_pixel_type> m_high_f_sum;

    // Intermediates
    vw::ImageView<double> m_curr_sum, m_curr_low, m_curr_high;

    void reset( void ) {
      vw::fill( m_low_f_sum, 0 );
      vw::fill( m_sum_weight, 0 );
      vw::fill( m_high_f_sum, accum_pixel_type() );
    }

    struct UpdateHighFFunctor {
      template <class Arg1T, class Arg2T, class Arg3T>
      struct result {
        typedef Arg1T type;
      };

      template <class Arg1T, class Arg2T, class Arg3T>
      inline Arg1T& operator()( Arg1T& high_f_result, Arg2T const& input, Arg3T const& input_alpha  ) const {
        if ( input_alpha > high_f_result[1] )
          high_f_result[0] = input;
        return high_f_result;
      }
    };

  public:
    Albedo2BandNRAccumulator( vw::int32 const& cols, vw::int32 const& rows ) :
    m_low_f_sum(cols,rows), m_sum_weight(cols,rows), m_high_f_sum(cols,rows),
    m_curr_sum(cols,rows), m_curr_low(cols,rows), m_curr_high(cols,rows) {
      this->reset();
    }

    // Accumulate
    template <class IViewT>
    void operator()( vw::ImageViewBase<IViewT> const& image,
                     double const& t ) {
      m_sum_weight += select_channel(image.impl(),1);

      m_curr_sum = vw::channel_cast<double>(select_channel(image.impl(),0))*
        vw::channel_cast<double>(select_channel(image.impl(),1)) / t;
      m_curr_low = gaussian_filter( m_curr_sum, 2.0 );
      m_curr_high = m_curr_sum - m_curr_low;
      m_low_f_sum += m_curr_low;

      // High only makes it in if the input alpha is higher than the currently loaded
      for_each_pixel( m_high_f_sum, m_curr_high, select_channel(image.impl(),1), UpdateHighFFunctor() );
    }

    // Result
    typedef vw::ImageView<PixelT> result_type;
    result_type result(void) {
      // Normalize
      result_type r = m_low_f_sum/m_sum_weight + m_high_f_sum;
      // Apply alpha
      select_channel(r,1) = vw::channel_cast_rescale<channel_type>(threshold(m_sum_weight,0));
      this->reset();
      return r;
    }
  };

}} // end namespace asp::pho

#endif//__ASP_PHO_ALBEDO_ACCUMULATORS_H__
