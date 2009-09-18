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

#include <asp/Core/Equation.h>

using namespace vw;
using namespace camera;

// Useful tools
//-------------------------------------------------------

// Debugging output of coefficients (constants)
std::ostream& vw::camera::operator<<( std::ostream& os,
                                      boost::shared_ptr<BaseEquation> eq ) {
  os << "Vector Equation-----------------------------\n";
  os << " " << eq->type() << std::endl;
  os << " Time Offset: " << eq->get_time_offset() << "\n";
  os << " Eq Coeff: " << "\n";
  unsigned working_idx = 0;
  while ( working_idx != eq->size() ) {
    os << " ";
    for ( int i = 0; i < 4; i++ )
      if ( working_idx != eq->size() ) {
        os << (*eq)[working_idx] << " ";
        working_idx++;
      }
    os << "\n";
  }
  os << "--------------------------------------------\n";
  return os;
}

// Determines equation type and write appropriate
void vw::camera::write_equation( std::ofstream& f, boost::shared_ptr<BaseEquation> eq ) {
  f << eq->type() << "\n";
  eq->write( f );
}
// Determines equation type from file and return appropriate
boost::shared_ptr<BaseEquation> vw::camera::read_equation( std::ifstream& f) {
  boost::shared_ptr<BaseEquation> eq;

  std::string buffer = "";
  std::getline( f, buffer );
  if ( buffer == "PolyEquation" ) {
    boost::shared_ptr<PolyEquation> eqn_new( new PolyEquation() );
    eq = boost::shared_dynamic_cast<BaseEquation>( eqn_new );
    eq->read( f );
  } else if ( buffer == "RPNEquation" ) {
    boost::shared_ptr<RPNEquation> eqn_new( new RPNEquation() );
    eq = boost::shared_dynamic_cast<BaseEquation>( eqn_new );
    eq->read( f );
  } else {
    vw_throw( IOErr() << "Unknown equation type: " << buffer << "\n" );
  }
  return eq;
}


