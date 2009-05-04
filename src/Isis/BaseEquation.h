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

#ifndef __VW_CAMERA_BASE_EQUATION__
#define __VW_CAMERA_BASE_EQUATION__

// STL
#include <fstream>
#include <iostream>
// VW
#include <vw/Math/Vector.h>
// Other Forms

namespace vw {
namespace camera {

  // Vector Equation is a method
  // used to modify an IsisCameraModel through
  // IsisAdjustCameraModel.

  // BaseEquation
  // Returns a Vector3 that is evaluated on a double, time.
  //
  class BaseEquation {
  protected:
    Vector3 m_cached_output;
    double m_cached_time;
    double m_time_offset;

    // Update Cache (m_cached_output)
    virtual void update ( double const& t ) = 0;

  public:
    // Constructor
    virtual ~BaseEquation() {}
    virtual std::string type() const = 0;

    //Evaluates the equation at time T
    Vector3 evaluate( double const& t ) {
      if ( t != m_cached_time )
	update( t );
      return m_cached_output;
    }
    Vector3 operator()( double const& t ) { return evaluate(t);}

    // Tells the number of constants defining the equation
    // This is especially vague as it is meant for interaction with a
    // bundle adjuster. BA just wants to roll through the constants
    // and redefine them.
    virtual unsigned size( void ) const = 0;
    virtual double& operator[]( unsigned const& n ) = 0;
    const double& operator[]( unsigned const& n ) const {
      return this->operator[](n);
    }

    // Allows us to set the time offset
    void set_time_offset( double const& offset ) {
      m_cached_time = -1;
      m_time_offset = offset;
    }
    double get_time_offset(void) const { return m_time_offset; }

    // FileIO routines
    virtual void write( std::ofstream &f ) = 0;
    virtual void read( std::ifstream &f ) = 0;
   
  };

  
}}

#endif//__VW_CAMERA_BASE_EQUATION__
