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

#include <vw/Core/Exception.h>
#include <asp/IsisIO/Equation.h>
#include <asp/IsisIO/BaseEquation.h>
#include <asp/IsisIO/PolyEquation.h>
#include <asp/IsisIO/RPNEquation.h>

#include <ostream>
#include <string>

using namespace vw;
using namespace asp;

// Useful tools
//-------------------------------------------------------

// Debugging output of coefficients (constants)
std::ostream& asp::operator<<( std::ostream& os,
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
void asp::write_equation( std::ofstream& f, boost::shared_ptr<BaseEquation> eq ) {
  f << eq->type() << "\n";
  eq->write( f );
}
// Determines equation type from file and return appropriate
boost::shared_ptr<BaseEquation> asp::read_equation( std::ifstream& f) {
  if ( !f.is_open() )
    vw_throw( IOErr() << "IsisIO/Equation: Unable to open equation file\n" );

  boost::shared_ptr<BaseEquation> eq;

  std::string buffer = "";
  std::getline( f, buffer );
  if ( buffer == "PolyEquation" ) {
    boost::shared_ptr<PolyEquation> eqn_new( new PolyEquation() );
    eq = boost::dynamic_pointer_cast<BaseEquation>( eqn_new );
    eq->read( f );
  } else if ( buffer == "RPNEquation" ) {
    boost::shared_ptr<RPNEquation> eqn_new( new RPNEquation() );
    eq = boost::dynamic_pointer_cast<BaseEquation>( eqn_new );
    eq->read( f );
  } else {
    vw_throw( IOErr() << "Unknown equation type: " << buffer << "\n" );
  }
  return eq;
}


