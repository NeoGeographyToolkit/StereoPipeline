// __BEGIN_LICENSE__
// 
// Copyright (C) 2006 United States Government as represented by the
// Administrator of the National Aeronautics and Space Administration
// (NASA).  All Rights Reserved.
// 
// Copyright 2006 Carnegie Mellon University. All rights reserved.
// 
// This software is distributed under the NASA Open Source Agreement
// (NOSA), version 1.3.  The NOSA has been approved by the Open Source
// Initiative.  See the file COPYING at the top of the distribution
// directory tree for the complete NOSA document.
// 
// THE SUBJECT SOFTWARE IS PROVIDED "AS IS" WITHOUT ANY WARRANTY OF ANY
// KIND, EITHER EXPRESSED, IMPLIED, OR STATUTORY, INCLUDING, BUT NOT
// LIMITED TO, ANY WARRANTY THAT THE SUBJECT SOFTWARE WILL CONFORM TO
// SPECIFICATIONS, ANY IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR
// A PARTICULAR PURPOSE, OR FREEDOM FROM INFRINGEMENT, ANY WARRANTY THAT
// THE SUBJECT SOFTWARE WILL BE ERROR FREE, OR ANY WARRANTY THAT
// DOCUMENTATION, IF PROVIDED, WILL CONFORM TO THE SUBJECT SOFTWARE.
// 
// __END_LICENSE__

#ifndef __VW_CAMERA_POLY_EQUATION__
#define __VW_CAMERA_POLY_EQUATION__

#include <Isis/BaseEquation.h>

namespace vw {
namespace camera {

  // Polynomial Equation
  // .. is a vector equation described by an n'th order polynomial
  class PolyEquation : public BaseEquation {
    Vector<double> m_x_coeff;
    Vector<double> m_y_coeff;
    Vector<double> m_z_coeff;

    void update ( double const& t );
  public:
    PolyEquation( int order = 0 );
    PolyEquation( int, int, int );
    PolyEquation( Vector<double>& x,
		  Vector<double>& y,
		  Vector<double>& z ) : m_x_coeff(x), m_y_coeff(y), m_z_coeff(z) {
      m_cached_time = -1;
      m_time_offset = 0;
    }
    std::string type() const {  return "PolyEquation"; }

    unsigned size(void) const { return m_x_coeff.size()+m_y_coeff.size()+m_z_coeff.size(); }
    double& operator[]( unsigned const& n );

    void write( std::ofstream &f );
    void read( std::ifstream &f );
  };

}}

#endif//__VW_CAMERA_POLY_EQUATION__
