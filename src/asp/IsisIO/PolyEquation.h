// __BEGIN_LICENSE__
// Copyright (C) 2006-2009 United States Government as represented by
// the Administrator of the National Aeronautics and Space Administration.
// All Rights Reserved.
// __END_LICENSE__


#ifndef __ASP_POLY_EQUATION__
#define __ASP_POLY_EQUATION__

#include <asp/IsisIO/BaseEquation.h>

namespace asp {

  // Polynomial Equation
  // .. is a vector equation described by an n'th order polynomial
  class PolyEquation : public BaseEquation {
    vw::Vector<double> m_x_coeff;
    vw::Vector<double> m_y_coeff;
    vw::Vector<double> m_z_coeff;

    void update ( double const& t );
  public:
    PolyEquation( int order = 0 );
    PolyEquation( int, int, int );
    PolyEquation( vw::Vector<double>& x,
                  vw::Vector<double>& y,
                  vw::Vector<double>& z ) : m_x_coeff(x), m_y_coeff(y), m_z_coeff(z) {
      m_cached_time = -1;
      m_time_offset = 0;
    }
    std::string type() const {  return "PolyEquation"; }

    unsigned size(void) const { return m_x_coeff.size()+m_y_coeff.size()+m_z_coeff.size(); }
    double& operator[]( unsigned const& n );

    void write( std::ofstream &f );
    void read( std::ifstream &f );
  };

}

#endif//__ASP_POLY_EQUATION__
