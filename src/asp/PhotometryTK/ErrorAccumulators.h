// __BEGIN_LICENSE__
// Copyright (C) 2006-2010 United States Government as represented by
// the Administrator of the National Aeronautics and Space Administration.
// All Rights Reserved.
// __END_LICENSE__


#ifndef __ASP_PHO_ERROR_ACCUMULATORS_H__
#define __ASP_PHO_ERROR_ACCUMULATORS_H__

#include <asp/pho/RecursiveBBoxAccumulator.h>

using vw;
using vw::plate;

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

  // Wraps error accumulator
  template<typename ScalarT, typename ElementT>
  class ErrorAccumulatorFunc : public AccumulatorFunc<ScalarT, ElementT> {
  protected:
    int m_level;
    int32 m_maxTid;
    double m_exposureTime;
    boost::shared_ptr<PlateFile> m_drg;
    boost::shared_ptr<PlateFile> m_albedo;
    boost::shared_ptr<PlateFile> m_reflect;

    ScalarT m_sum;
    
  public:
    ErrorAccumulatorFunc(int level,
			 int32 maxTid,
			 double exposureTime,
			 boost::shared_ptr<PlateFile> drg,
			 boost::shared_ptr<PlateFile> albedo,
			 boost::shared_ptr<PlateFile> reflect) : 
    m_level(level), m_maxTid(maxTid), m_exposureTime(exposureTime), m_drg(drg), m_albedo(albedo), m_reflect(reflect), m_sum(0) {}
    
    ErrorAccumulatorFunc(ErrorAccumulatorFunc const& other) :
    m_level(level), m_maxTid(maxTid), m_exposureTime(other.m_exposureTime), m_drg(other.m_drg), m_albedo(other.m_albedo), m_reflect(other.m_reflect), m_sum(0) {}

    ~ErrorAccumulatorFunc() {}

    /**
     * For this class, ElementT must be something that holds to int values
     * and has array accessor operators.
     */
    void operator()(ElementT const& e) {
      int ix = e[0];
      int iy = e[1];

      ErrorAccumulator pixAccum(m_exposureTime);

      std::list<TileHeader> drg_tiles = 
	m_drg->search_by_location( ix, iy, m_level, 0, m_maxTid, true );

      BOOST_FOREACH(const TileHeader& drg_tile, drg_tiles) {
      }
    }

    void operator()(AccumulatorFunc<ScalarT,ElementT> const& e) {
      m_sum += e.value();
    }

    ScalarT value() const {
      return m_sum;
    }
  };

}} // end namespace asp::pho

#endif//__ASP_PHO_ERROR_ACCUMULATORS_H__
