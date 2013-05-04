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
    vw::uint8 m_max_length; // Maximum order + 1

    void update ( double const& t );
  public:
    PolyEquation( int order = 0 );
    PolyEquation( int, int, int );
    PolyEquation( vw::Vector<double>& x,
                  vw::Vector<double>& y,
                  vw::Vector<double>& z ) : m_x_coeff(x), m_y_coeff(y), m_z_coeff(z) {
      m_cached_time = -1;
      m_time_offset = 0;
      if ( x.size() > 254 || y.size() > 254 || z.size() > 254 )
        vw::vw_throw( vw::ArgumentErr() << "PolyEquation: Polynomial order must be less than 255" );
      m_max_length = vw::uint8( std::max( x.size(),
                                          std::max( y.size(), z.size() ) ) ) + 1;
    }
    std::string type() const {  return "PolyEquation"; }

    size_t size() const { return m_x_coeff.size()+m_y_coeff.size()+m_z_coeff.size(); }
    double& operator[]( size_t const& n );

    void write( std::ofstream &f );
    void read( std::ifstream &f );
  };

}

#endif//__ASP_POLY_EQUATION__
