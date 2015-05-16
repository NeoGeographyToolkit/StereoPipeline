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
#include <vw/Math/Vector.h>
#include <asp/IsisIO/RPNEquation.h>

#include <iomanip>
#include <stack>
#include <vector>

#include <boost/algorithm/string/classification.hpp>
#include <boost/algorithm/string/split.hpp>

using namespace vw;
using namespace asp;

// Constructors
//-----------------------------------------------------
RPNEquation::RPNEquation() {
  m_x_eq.clear();
  m_x_consts.clear();
  m_y_eq.clear();
  m_y_consts.clear();
  m_z_eq.clear();
  m_z_consts.clear();
  m_cached_time = -1;
  m_time_offset = 0;
}
RPNEquation::RPNEquation( std::string x_eq,
                          std::string y_eq,
                          std::string z_eq ) {
  string_to_eqn( x_eq, m_x_eq, m_x_consts );
  string_to_eqn( y_eq, m_y_eq, m_y_consts );
  string_to_eqn( z_eq, m_z_eq, m_z_consts );
  m_cached_time = -1;
  m_time_offset = 0;
}

// Update
//-----------------------------------------------------
void RPNEquation::update( double const& t ) {
  m_cached_time = t;
  double delta_t = t - m_time_offset;
  m_cached_output[0] = evaluate( m_x_eq,
                                 m_x_consts,
                                 delta_t );
  m_cached_output[1] = evaluate( m_y_eq,
                                 m_y_consts,
                                 delta_t );
  m_cached_output[2] = evaluate( m_z_eq,
                                 m_z_consts,
                                 delta_t );
}
void RPNEquation::string_to_eqn( std::string& str,
                                 std::vector<std::string>& commands,
                                 std::vector<double>& consts ) {
  // Breaks a string into the equation format used internally
  commands.clear();
  consts.clear();
  boost::split( commands, str, boost::is_any_of(" ="));

  // Cleaning out any tokens that are just ""
  for(std::vector<std::string>::iterator iter = commands.begin();
      iter != commands.end(); ++iter ) {
    if ( (*iter) == "" ) {
      iter = commands.erase(iter);
      iter--;
    }
  }

  // Pulling out the numbers
  for(std::vector<std::string>::iterator iter = commands.begin();
      iter != commands.end(); ++iter ) {
    if ( isdigit( (*iter)[(*iter).size()-1] ) ) {
      consts.push_back( atof( iter->c_str() ) );
      *iter = "c";
    }
  }
}
double RPNEquation::evaluate( std::vector<std::string>& commands,
                              std::vector<double>& consts,
                              double const& t ) {
  // Evaluates an equation in the internal format
  if ( commands.empty() )
    return 0;
  int consts_index = 0;
  std::stack<double> rpn_stack;
  double buffer;
  for ( std::vector<std::string>::iterator iter = commands.begin();
        iter != commands.end(); ++iter ) {
    if ( *iter == "c" ) {
      rpn_stack.push( consts[consts_index] );
      consts_index++;
    } else if ( *iter == "t" ) {
      rpn_stack.push( t );
    } else if ( rpn_stack.size() < 1 ) {
      vw_throw( IOErr() << "Insufficient arguments for RPN command: "
                << *iter << "\n" );
    } else if ( *iter == "sin" ) {
      buffer = sin( rpn_stack.top() );
      rpn_stack.pop();
      rpn_stack.push( buffer );
    } else if ( *iter == "cos" ) {
      buffer = cos( rpn_stack.top() );
      rpn_stack.pop();
      rpn_stack.push( buffer );
    } else if ( *iter == "tan" ) {
      buffer = tan( rpn_stack.top() );
      rpn_stack.pop();
      rpn_stack.push( buffer );
    } else if ( *iter == "abs" ) {
      buffer = fabs( rpn_stack.top() );
      rpn_stack.pop();
      rpn_stack.push( buffer );
    } else if ( rpn_stack.size() < 2 ) {
      vw_throw( IOErr() << "Insufficient arguments for command: "
                << *iter << "\n" );
    } else if ( *iter == "*" ) {
      buffer = rpn_stack.top();
      rpn_stack.pop();
      buffer *= rpn_stack.top();
      rpn_stack.pop();
      rpn_stack.push( buffer );
    } else if ( *iter == "/" ) {
      buffer = rpn_stack.top();
      rpn_stack.pop();
      buffer = rpn_stack.top() / buffer;
      rpn_stack.pop();
      rpn_stack.push( buffer );
    } else if ( *iter == "-" ) {
      buffer = rpn_stack.top();
      rpn_stack.pop();
      buffer = rpn_stack.top() - buffer;
      rpn_stack.pop();
      rpn_stack.push( buffer );
    } else if ( *iter == "+" ) {
      buffer = rpn_stack.top();
      rpn_stack.pop();
      buffer += rpn_stack.top();
      rpn_stack.pop();
      rpn_stack.push( buffer );
    } else if ( *iter == "^" ) {
      buffer = rpn_stack.top();
      rpn_stack.pop();
      buffer = pow( rpn_stack.top(), buffer );
      rpn_stack.pop();
      rpn_stack.push( buffer );
    } else {
      vw_throw( IOErr() << "Unknown RPN operator: " << *iter << "\n" );
    }
  } // End of calculator

  if ( rpn_stack.size() != 1 )
    vw_throw( IOErr() << "Unbalanced RPN equation! More constants than need by operators.\n" );

  return rpn_stack.top();
}

// FileIO
//-----------------------------------------------------
void RPNEquation::write( std::ofstream &f ) {
  for ( int i = 0; i < 3; i++ ) {
    std::vector<std::string>* eq_ptr = NULL;
    std::vector<double>* cs_ptr = NULL;
    switch(i) {
    case 0:
      eq_ptr = &m_x_eq;
      cs_ptr = &m_x_consts;
      break;
    case 1:
      eq_ptr = &m_y_eq;
      cs_ptr = &m_y_consts;
      break;
    case 2:
      eq_ptr = &m_z_eq;
      cs_ptr = &m_z_consts;
      break;
    }

    f << std::setprecision( 15 );
    int cs_idx = 0;
    for ( unsigned j = 0; j < eq_ptr->size(); j++ ) {
      if ( (*eq_ptr)[j] == "c" ) {
        f << (*cs_ptr)[cs_idx] << " ";
        cs_idx++;
      } else {
        f << (*eq_ptr)[j] << " ";
      }
    }
    f << "\n";
  }
}
void RPNEquation::read( std::ifstream &f ) {
  std::string buffer;
  m_cached_time = -1;

  buffer = "";
  std::getline( f, buffer );
  string_to_eqn( buffer, m_x_eq, m_x_consts );
  buffer = "";
  std::getline( f, buffer );
  string_to_eqn( buffer, m_y_eq, m_y_consts );
  buffer = "";
  std::getline( f, buffer );
  string_to_eqn( buffer, m_z_eq, m_z_consts );
}

// Constant Access
//-----------------------------------------------------
double& RPNEquation::operator[]( size_t const& n ) {
  m_cached_time = -1;
  if ( n >= m_x_consts.size() + m_y_consts.size()
       + m_z_consts.size() )
    vw_throw( ArgumentErr() << "RPNEquation: invalid index." );
  if ( n < m_x_consts.size() )
    return m_x_consts[n];
  else if ( n < m_x_consts.size() + m_y_consts.size() )
    return m_y_consts[n-m_x_consts.size()];
  return m_z_consts[n-m_x_consts.size()-m_y_consts.size()];
}
