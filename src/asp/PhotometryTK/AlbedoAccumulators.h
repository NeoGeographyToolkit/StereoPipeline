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
      m_sum += select_channel(image.impl(),0)*select_channel(image.impl(),1) /
        ( t*reflectance.impl() );
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
      m_sum += select_channel(image.impl(),0)*select_channel(image.impl(),1) / t;
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
    typedef typename vw::CompoundChannelCast<PixelT, double>::type accum_pixel_type;
    typedef typename vw::PixelChannelType<PixelT>::type channel_type;
    vw::ImageView<accum_pixel_type> m_nominator, m_denominator;
    vw::ImageView<accum_pixel_type> m_t_grad, m_t_error; // temporaries to avoid
                                                         // excessive memory allocations.
    vw::ImageView<double> m_sum_weight;
    void reset( void ) {
      vw::fill(m_nominator, accum_pixel_type() );
      vw::fill(m_denominator, accum_pixel_type() );
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
      m_t_grad = t * reflectance;
      m_t_error = select_channel(image,0) - t * previous_albedo * reflectance;

      // Update accumulators
      m_nominator += m_t_grad * m_t_error * select_channel(image,1);
      m_denominator += m_t_grad * m_t_grad * select_channel(image,1);

      m_sum_weight += select_channel(image,1);
    }

    // Result
    typedef vw::ImageView<PixelT> result_type;
    result_type result(void) {
      result_type r = m_nominator / m_denominator;
      select_channel(r,1) = vw::channel_cast_rescale<channel_type>(threshold(m_sum_weight,0));
      this->reset();
      return r;
    }
  };

  template <class PixelT>
  class AlbedoDeltaNRAccumulator : public ImageAccumulatorBase<AlbedoDeltaNRAccumulator<PixelT> > {
    typedef typename vw::CompoundChannelCast<PixelT, double>::type accum_pixel_type;
    typedef typename vw::PixelChannelType<PixelT>::type channel_type;
    vw::ImageView<PixelT> m_nominator, m_denominator;
    vw::ImageView<PixelT> m_t_error; // temporary to avoid
                                 // excessive memory allocations.
    vw::ImageView<double> m_sum_weight;
    void reset( void ) {
      vw::fill(m_nominator, PixelT() );
      vw::fill(m_denominator, PixelT() );
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
      m_t_error = select_channel(image,0) - t*previous_albedo;

      // Update accumulators
      m_nominator += t * m_t_error * select_channel(image,1);
      m_denominator += t * t * select_channel(image,1);

      m_sum_weight += select_channel(image,1);
    }

    // Result
    typedef vw::ImageView<PixelT> result_type;
    result_type result(void) {
      result_type r = m_nominator / m_denominator;
      select_channel(r,1) = vw::channel_cast_rescale<channel_type>(threshold(m_sum_weight,0));
      this->reset();
      return r;
    }
  };

}} // end namespace asp::pho

#endif//__ASP_PHO_ALBEDO_ACCUMULATORS_H__
