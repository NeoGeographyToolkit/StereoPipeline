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

// STL
#include <iomanip>
// ASP
#include <Isis/PolyEquation.h>
// Boost
#include <boost/algorithm/string.hpp>

using namespace vw;
using namespace camera;

// Constructors
//---------------------------------------------
PolyEquation::PolyEquation ( int order ) {
  if ( order < 0 )
    vw_throw( ArgumentErr() << "PolyEquation: Polynomial order must be greater than zero." );
  m_x_coeff.set_size( order + 1 );
  m_y_coeff.set_size( order + 1 );
  m_z_coeff.set_size( order + 1 );
  for ( int i = 0; i < m_x_coeff.size(); i++ )
    m_x_coeff[i] = m_y_coeff[i] = m_z_coeff[i] = 0;
  m_cached_time = -1;
  m_time_offset = 0;
}
PolyEquation::PolyEquation( int order_x, 
			    int order_y, 
			    int order_z ) {
  if ( order_x < 0 || order_y < 0 || order_z < 0 ) 
    vw_throw( ArgumentErr() << "PolyEquation: Polynomial order must be greater than zero." );
  m_x_coeff.set_size(order_x+1);
  m_y_coeff.set_size(order_y+1);
  m_z_coeff.set_size(order_z+1);
  for ( int i = 0; i < m_x_coeff.size(); i++ )
    m_x_coeff[i] = 0;
  for ( int i = 0; i < m_y_coeff.size(); i++ )
    m_y_coeff[i] = 0;
  for ( int i = 0; i < m_z_coeff.size(); i++ )
    m_z_coeff[i] = 0;
  m_cached_time = -1;
  m_time_offset = 0;
}

// Update
//-----------------------------------------------
void PolyEquation::update( double const& t ) {
  m_cached_time = t;
  double delta_t = t-m_time_offset;
  Vector<double> powers;
  int max_len = m_x_coeff.size();
  if ( max_len < m_y_coeff.size() )
    max_len = m_y_coeff.size();
  if ( max_len < m_z_coeff.size() )
    max_len = m_z_coeff.size();
  powers.set_size( max_len );
  powers[0] = 1;
  for ( int i = 1; i < powers.size(); i++ )
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
    case 2:
      pointer = &m_z_coeff;
      break;
    }

    f << std::setprecision( 15 );
    for ( int j = 0; j < (*pointer).size(); j++ )
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
    case 2:
      pointer = &m_z_coeff;
      break;
    }

    pointer->set_size( tokens.size() );
    for ( int j = 0; j < tokens.size(); j++ ) 
      (*pointer)[j] = atof( tokens[j].c_str() );
   
  }
}

// Constant Access
//-----------------------------------------------
double& PolyEquation::operator[]( unsigned const& n ) {
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
