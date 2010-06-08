// __BEGIN_LICENSE__
// Copyright (C) 2006-2010 United States Government as represented by
// the Administrator of the National Aeronautics and Space Administration.
// All Rights Reserved.
// __END_LICENSE__


#ifndef __ASP_PHO_ERROR_ACCUMULATORS_H__
#define __ASP_PHO_ERROR_ACCUMULATORS_H__

namespace asp {
namespace pho {

  // Calculate error
  class ErrorAccumulator : public vw::ReturnFixedType<void> {
    double m_error_sum, m_curr_time;
  public:
    typedef double value_type;
    ErrorAccumulator(double t) : m_error_sum(0), m_curr_time(t) {}

    template <class AValT, class RValT>
    void operator()( AValT const& drg, AValT const& albedo, RValT const& reflectance ) {
      double i = drg[0], a=albedo[0], s=drg[1], r=reflectance;
      i -= a*m_curr_time*r;
      i *= s; i *= i;
      m_error_sum += i;
    }

    value_type value() const {
      return m_error_sum;
    }
  };

  // Calculate error
  class ErrorNRAccumulator : public vw::ReturnFixedType<void> {
    double m_error_sum, m_curr_time;
  public:
    typedef double value_type;
    ErrorNRAccumulator(double t) : m_error_sum(0), m_curr_time(t) {}

    template <class AValT>
    void operator()( AValT const& drg, AValT const& albedo ) {
      double i = drg[0], a=albedo[0], s=drg[1];
      i -= a*m_curr_time;
      i *= s; i *= i;
      m_error_sum += i;
    }

    value_type value() const {
      return m_error_sum;
    }
  };

}} // end namespace asp::pho

#endif//__ASP_PHO_ERROR_ACCUMULATORS_H__
