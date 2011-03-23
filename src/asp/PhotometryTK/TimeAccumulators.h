// __BEGIN_LICENSE__
// Copyright (C) 2006-2010 United States Government as represented by
// the Administrator of the National Aeronautics and Space Administration.
// All Rights Reserved.
// __END_LICENSE__


#ifndef __ASP_PHO_TIME_ACCUMULATORS_H__
#define __ASP_PHO_TIME_ACCUMULATORS_H__

namespace asp {
namespace pho {

  // Re-estimate time exposure with reflectance
  class TimeDeltaAccumulator : public vw::ReturnFixedType<void> {
    double m_numerator, m_denominator;
    double m_curr_time;
  public:
    typedef double value_type;

    TimeDeltaAccumulator(double t) : m_numerator(0),
      m_denominator(0), m_curr_time(t) {}

    template <class AValT, class RValT>
    void operator()( AValT const& drg, AValT const& albedo, RValT const& reflectance ) {
      double i = drg[0], a = albedo[0], s = drg[1], r = reflectance;
      double inter = a*r*s;
      m_numerator += (i - m_curr_time*a*r)*inter;
      m_denominator += inter*inter;
    }

    value_type value() const {
      return m_numerator/m_denominator;
    }
  };

  // Re-estimate time exposure with-out reflectance
  class TimeDeltaNRAccumulator : public vw::ReturnFixedType<void> {
    double m_numerator, m_denominator;
    double m_curr_time;
  public:
    typedef double value_type;

    TimeDeltaNRAccumulator(double t) : m_numerator(0),
      m_denominator(0), m_curr_time(t) {}

    template <class AValT>
    void operator()( AValT const& drg, AValT const& albedo ) {
      double i = drg[0], s = drg[1], a = albedo[0];
      //std::cout << "i=[" << i << "] s=[" << s << "] a=[" << a << "]\n";
      double inter = a*s;
      //std::cout << "inter=[" << inter << "]\n";
      m_numerator += (i - m_curr_time*a)*inter;
      //std::cout << "numerator=[" << m_numerator << "]\n";
      m_denominator += inter*inter;
      //std::cout << "denominator=[" << m_denominator << "]\n";
    }

    value_type value() const {
      return m_numerator/m_denominator;
    }
  };

}}

#endif//__ASP_PHO_TIME_ACCUMULATORS_H__
