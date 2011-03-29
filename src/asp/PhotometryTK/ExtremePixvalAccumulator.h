// __BEGIN_LICENSE__
// Copyright (C) 2006-2010 United States Government as represented by
// the Administrator of the National Aeronautics and Space Administration.
// All Rights Reserved.
// __END_LICENSE__

#ifndef __ASP_PHO_EXTREME_PIXVAL_ACCUMULATORS_H__
#define __ASP_PHO_EXTREME_PIXVAL_ACCUMULATORS_H__

#include <vw/Core/FundamentalTypes.h>

namespace asp {
namespace pho {

  template<typename PixelT>
  class ExtremePixvalAccumulator : public vw::ReturnFixedType<void> {
    typedef typename vw::PixelChannelType<PixelT>::type ChannelType;

    ChannelType m_minpixval;
    ChannelType m_maxpixval;

  public:    
    ExtremePixvalAccumulator(ChannelType const& lastMin, ChannelType const& lastMax) : m_minpixval(lastMin), m_maxpixval(lastMax) {}
    
    void operator()( PixelT const& pix ) {
      ChannelType pixVal = pix[0];
      return (*this)(pixVal);
    }

    void operator()( ChannelType const& pixval ) {
      if (pixval > m_maxpixval) {
	m_maxpixval = pixval;
      }
      if (pixval < m_minpixval) {
	m_minpixval = pixval;
      }
    }

    void values(ChannelType& newMin, ChannelType& newMax) {
      newMin = m_minpixval;
      newMax = m_maxpixval;
    }
  };
  }
}

#endif//__ASP_PHO_EXTREME_PIXVAL_ACCUMULATORS_H__
