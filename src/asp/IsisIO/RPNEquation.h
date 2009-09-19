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

#ifndef __VW_CAMERA_RPN_EQUATION__
#define __VW_CAMERA_RPN_EQUATION__

// STL
#include <vector>
// ASP
#include <asp/IsisIO/BaseEquation.h>

namespace vw{
namespace camera{

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

    unsigned size(void) const { return m_x_consts.size() +
        m_y_consts.size() + m_z_consts.size(); }
    double& operator[]( unsigned const& n );

    void write( std::ofstream &f );
    void read( std::ifstream &f );
  };

}}

#endif//__VW_CAMERA_RPN_EQUATION__
