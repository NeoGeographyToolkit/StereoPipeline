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

#include <Isis/Equations.h>

// Boost
#include <boost/algorithm/string.hpp>

using namespace vw;
using namespace camera;

// Vector Equation
//----------------------------------------------------------------

// Updates Cache
void VectorEquation::update( double const& t ) {
  m_cached_time = t;
  double delta_t = t-m_time_offset;
  Vector<double> powers = m_x_coeff;
  powers[0] = 1;
  for ( int i = 1; i < powers.size(); i++ )
    powers[i] = powers[i-1]*delta_t;
  m_cached_output[0] = sum( elem_prod(m_x_coeff,powers) );
  m_cached_output[1] = sum( elem_prod(m_y_coeff,powers) );
  m_cached_output[2] = sum( elem_prod(m_z_coeff,powers) );
}

// Constructor
VectorEquation::VectorEquation ( int poly_order ) {
  if ( poly_order < 0 )
    vw_throw( ArgumentErr() << "Polynomial order must be non-negative.");
  m_x_coeff.set_size( poly_order + 1);
  m_y_coeff.set_size( poly_order + 1);
  m_z_coeff.set_size( poly_order + 1);
  for ( int i = 0; i < m_x_coeff.size(); i++ ) {
    m_x_coeff[i] = 0;
    m_y_coeff[i] = 0;
    m_z_coeff[i] = 0;
  }
  m_cached_time = -1;
  m_time_offset = 0;
}

// Sets values
void VectorEquation::set_constant(unsigned index, double value) {
  m_cached_time = -1;
  if ( index >= m_x_coeff.size()*3 )
    vw_throw(ArgumentErr() << "VectorEquation: invalid index.");
  unsigned temp_index = index;
  if ( temp_index >= m_x_coeff.size() )
    temp_index -= m_x_coeff.size();
  else {
    m_x_coeff[temp_index] = value;
    return;
  }
  if ( temp_index >= m_y_coeff.size() )
    temp_index -= m_y_coeff.size();
  else {
    m_y_coeff[temp_index] = value;
    return;
  }
  m_z_coeff[temp_index] = value;
}

// Gives access to the constants defining the equation
const double VectorEquation::operator[]( unsigned const& n ) const {
  if ( n >= m_x_coeff.size()*3) 
    vw_throw(ArgumentErr() << "VectorEquation: invalid index.");
  unsigned temp_index = n;
  if ( temp_index >= m_x_coeff.size() )
    temp_index -= m_x_coeff.size();
  else 
    return m_x_coeff[temp_index];
  if ( temp_index >= m_y_coeff.size() )
    temp_index -= m_y_coeff.size();
  else
    return m_y_coeff[temp_index];
  return m_z_coeff[temp_index];
}

// Writing Vector Equation Information to file
void VectorEquation::write( std::ofstream &f ) {
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

// Reading Vector Equation Information from file
void VectorEquation::read( std::ifstream &f ) {
  std::string buffer;
  std::vector<std::string> tokens;
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

// Quaternion Equation
//-----------------------------------------------------------------

// Updates cache
void QuaternionEquation::update ( double const& t ) {
  m_cached_time = t;
  double delta_t = t-m_time_offset;
  Vector<double> powers = m_x_coeff;
  powers[0] = 1;
  for ( int i = 1; i < powers.size(); i++ )
    powers[i] = powers[i-1]*delta_t;
  Vector3 eval;
  eval[0] = sum( elem_prod(m_x_coeff,powers));
  eval[1] = sum( elem_prod(m_y_coeff,powers));
  eval[2] = sum( elem_prod(m_z_coeff,powers));
  m_cached_output = vw::math::euler_to_quaternion( eval[0],
						   eval[1],
						   eval[2],
						   "xyz" );
  m_cached_output = m_cached_output / norm_2( m_cached_output );
}

// Constructor
QuaternionEquation::QuaternionEquation( int poly_order ) {
  if ( poly_order < 0 ) 
    vw_throw( ArgumentErr() << "Polynomial order must be non-negative.");
  m_x_coeff.set_size( poly_order + 1 );
  m_y_coeff.set_size( poly_order + 1 );
  m_z_coeff.set_size( poly_order + 1 );
  for ( int i = 0; i < m_x_coeff.size(); i++ ) {
    m_x_coeff[i] = 0;
    m_y_coeff[i] = 0;
    m_z_coeff[i] = 0;
  }
  m_cached_time = -1;
  m_time_offset = 0;
}

// Assigns Value
void QuaternionEquation::set_constant(unsigned index, double value) {
  m_cached_time = -1;
  if ( index >= m_x_coeff.size()*3 ) 
    vw_throw( ArgumentErr() << "VectorEquation: invalid index.");
  unsigned temp_index = index;
  if ( temp_index >= m_x_coeff.size() )
    temp_index -= m_x_coeff.size();
  else {
    m_x_coeff[temp_index] = value;
    return;
  }
  if ( temp_index >= m_y_coeff.size() )
    temp_index -= m_y_coeff.size();
  else {
    m_y_coeff[temp_index] = value;
    return;
  }
  m_z_coeff[temp_index] = value;
}

// Gives access to the constants defining the equation
const double QuaternionEquation::operator[]( unsigned const& n ) const {
  if ( n >= m_x_coeff.size()*3 ) 
    vw_throw(ArgumentErr() << "QuaternionEquation: invalid index.");
  unsigned temp_index = n;
  if ( temp_index >= m_x_coeff.size() )
    temp_index -= m_x_coeff.size();
  else 
    return m_x_coeff[temp_index];
  if ( temp_index >= m_y_coeff.size() )
    temp_index -= m_y_coeff.size();
  else
    return m_y_coeff[temp_index];
  return m_z_coeff[temp_index];
}

// Writing Quarternion Information to file
void QuaternionEquation::write( std::ofstream &f ) {
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

// Reading Quarternion Information from file
void QuaternionEquation::read( std::ifstream &f ) {
  std::string buffer;
  std::vector<std::string> tokens;
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
      (*pointer)[j] =  atof( tokens[j].c_str() );
  }
}

// Useful Tools
//-----------------------------------------------------------------

std::ostream& operator<<( std::ostream& os, VectorEquation const& eq ) {
  os << "Vector Equation-----------------------------\n";
  os << " Time Offset: " << eq.get_time_offset() << "\n";
  int size = eq.size();
  size /= 3;
  int sum_count = 0;
  os << " X Coeff:\n [ ";
  for ( int i = sum_count; i < sum_count+size; i++ )
    os << eq[i] << " ";
  sum_count+=size;
  os << "]\n Y Coeff:\n [ ";
  for ( int i = sum_count; i < sum_count+size; i++ )
    os << eq[i] << " ";
  sum_count+=size;
  os << "]\n Z Coeff:\n [ ";
  for ( int i = sum_count; i < sum_count+size; i++ )
    os << eq[i] << " ";
  os << "]\n";
  os << "--------------------------------------------\n";
  return os;
}
std::ostream& operator<<( std::ostream& os, QuaternionEquation const& eq ) {
  os << "Quaternion Equation-------------------------\n";
  os << " Time Offset: " << eq.get_time_offset() << "\n";
  int size = eq.size();
  size /= 3;
  int sum_count = 0;
  os << " X Coeff:\n [ ";
  for ( int i = sum_count; i < sum_count+size; i++ )
    os << eq[i] << " ";
  sum_count+=size;
  os << "]\n Y Coeff:\n [ ";
  for ( int i = sum_count; i < sum_count+size; i++ )
    os << eq[i] << " ";
  sum_count+=size;
  os << "]\n Z Coeff:\n [ ";
  for ( int i = sum_count; i < sum_count+size; i++ )
    os << eq[i] << " ";
  os << "]\n";
  os << "--------------------------------------------\n";
  return os;
}
