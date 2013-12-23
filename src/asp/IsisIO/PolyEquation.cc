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

#include <vw/Core/FundamentalTypes.h>
#include <vw/Math/Vector.h>
#include <asp/IsisIO/PolyEquation.h>

#include <iomanip>
#include <vector>
#include <string>
#include <algorithm>

#include <boost/algorithm/string/split.hpp>
#include <boost/algorithm/string/classification.hpp>

using namespace vw;
using namespace asp;

// Constructors
//---------------------------------------------
PolyEquation::PolyEquation ( int order ) {
  if ( order < 0 )
    vw_throw( ArgumentErr() << "PolyEquation: Polynomial order must be greater than zero." );
  if ( order > 254 )
    vw_throw( ArgumentErr() << "PolyEquation: Polynomial order must be less than 255" );
  m_x_coeff.set_size( order + 1 );
  m_y_coeff.set_size( order + 1 );
  m_z_coeff.set_size( order + 1 );
  for ( int i = 0; i < order+1; i++ )
    m_x_coeff[i] = m_y_coeff[i] = m_z_coeff[i] = 0;
  m_cached_time = -1;
  m_time_offset = 0;
  m_max_length = uint8(order)+1;
}
PolyEquation::PolyEquation( int order_x,
                            int order_y,
                            int order_z ) {
  if ( order_x < 0 || order_y < 0 || order_z < 0 )
    vw_throw( ArgumentErr() << "PolyEquation: Polynomial order must be greater than zero." );
  if ( order_x > 254 || order_y > 254 || order_z > 254 )
    vw_throw( ArgumentErr() << "PolyEquation: Polynomial order must be less than 255" );
  m_x_coeff.set_size(order_x+1);
  m_y_coeff.set_size(order_y+1);
  m_z_coeff.set_size(order_z+1);
  for ( unsigned i = 0; i < m_x_coeff.size(); i++ )
    m_x_coeff[i] = 0;
  for ( unsigned i = 0; i < m_y_coeff.size(); i++ )
    m_y_coeff[i] = 0;
  for ( unsigned i = 0; i < m_z_coeff.size(); i++ )
    m_z_coeff[i] = 0;
  m_cached_time = -1;
  m_time_offset = 0;
  m_max_length = uint8( std::max( order_x, std::max( order_y, order_z ) ) ) + 1;
}

// Update
//-----------------------------------------------
void PolyEquation::update( double const& t ) {
  m_cached_time = t;
  double delta_t = t-m_time_offset;
  Vector<double> powers( m_max_length );
  powers[0] = 1;
  for ( uint8 i = 1; i < m_max_length; i++ )
    powers[i] = powers[i-1]*delta_t;
  m_cached_output[0] = sum( elem_prod(m_x_coeff,
                                      subvector(powers,0,m_x_coeff.size())) );
  m_cached_output[1] = sum( elem_prod(m_y_coeff,
                                      subvector(powers,0,m_y_coeff.size())) );
  m_cached_output[2] = sum( elem_prod(m_z_coeff,
                                      subvector(powers,0,m_z_coeff.size())) );
}

// FileIO
//-----------------------------------------------
void PolyEquation::write( std::ofstream& f ) {
  for ( int i = 0; i < 3; i++ ) {
    Vector<double>* pointer;
    switch(i) {
    case 0:
      pointer = &m_x_coeff;
      break;
    case 1:
      pointer = &m_y_coeff;
      break;
    default:
    case 2:
      pointer = &m_z_coeff;
      break;
    }

    f << std::setprecision( 15 );
    for ( unsigned j = 0; j < (*pointer).size(); j++ )
      f << (*pointer)[j] << " ";
    f << "\n";
  }
}

void PolyEquation::read( std::ifstream& f ) {
  std::string buffer;
  std::vector<std::string> tokens;
  m_cached_time = -1;
  for ( int i = 0; i < 3; i++ ) {
    buffer = "";
    std::getline( f, buffer );
    boost::split( tokens, buffer, boost::is_any_of(" =\n") );

    // Cleaning out any tokens that are just ""
    for(std::vector<std::string>::iterator iter = tokens.begin();
        iter != tokens.end(); ++iter )
      if ( (*iter) == "" ) {
        iter = tokens.erase(iter);
        iter--;
      }

    Vector<double>* pointer;
    switch(i) {
    case 0:
      pointer = &m_x_coeff;
      break;
    case 1:
      pointer = &m_y_coeff;
      break;
    default:
    case 2:
      pointer = &m_z_coeff;
      break;
    }

    pointer->set_size( tokens.size() );
    for ( unsigned j = 0; j < tokens.size(); j++ )
      (*pointer)[j] = atof( tokens[j].c_str() );

  }
}

// Constant Access
//-----------------------------------------------
double& PolyEquation::operator[]( size_t const& n ) {
  m_cached_time = -1;
  if ( n >= m_x_coeff.size()+m_y_coeff.size()+m_z_coeff.size() )
    vw_throw( ArgumentErr() << "PolyEquation: invalid index.");
  if ( n < m_x_coeff.size() ) {
    return m_x_coeff[n];
  } else if ( n < m_x_coeff.size()+m_y_coeff.size() ) {
    return m_y_coeff[n-m_x_coeff.size()];
  } else {
    return m_z_coeff[n-m_x_coeff.size()-m_y_coeff.size()];
  }
}
