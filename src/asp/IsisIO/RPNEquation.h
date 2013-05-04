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


#ifndef __ASP_RPN_EQUATION__
#define __ASP_RPN_EQUATION__

// STL
#include <vector>
// ASP
#include <asp/IsisIO/BaseEquation.h>

namespace asp {

  // RPN Equation
  // .. is an equation defined by a string containing Reverse Polish
  // Notation (RPN) equation.
  // ex: 3 t + 4 t * sin * == sin(4t)*(3+t)
  //
  // Current supported commands
  //  t       (refers to the t this equation is evaluated with)
  //  sin, cos, tan, abs
  //  *, /, -, +, ^
  //
  // Remember: Have your equation space delimited
  // Also: 'c' is an internal place holder for RPNEquation
  class RPNEquation : public BaseEquation {
    std::vector<std::string> m_x_eq;
    std::vector<double> m_x_consts;
    std::vector<std::string> m_y_eq;
    std::vector<double> m_y_consts;
    std::vector<std::string> m_z_eq;
    std::vector<double> m_z_consts;

    void update( double const& t );
    void string_to_eqn( std::string& str,
                        std::vector<std::string>& commands,
                        std::vector<double>& consts );
    double evaluate( std::vector<std::string>& commands,
                     std::vector<double>& consts,
                     double const& t );
  public:
    RPNEquation();
    RPNEquation( std::string x_eq,
                 std::string y_eq,
                 std::string z_eq );
    std::string type() const { return "RPNEquation"; }

    size_t size() const { return m_x_consts.size() +
        m_y_consts.size() + m_z_consts.size(); }
    double& operator[]( size_t const& n );

    void write( std::ofstream &f );
    void read( std::ifstream &f );
  };

}

#endif//__ASP_RPN_EQUATION__
