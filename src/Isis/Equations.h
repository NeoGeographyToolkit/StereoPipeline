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

#ifndef __VW_CAMERA_EQUATION__
#define __VW_CAMERA_EQUATION__

#include <fstream>
#include <iostream>

#include <vw/Math/Vector.h>
#include <vw/Math/Matrix.h>
#include <vw/Math/EulerAngles.h>

#include <vw/Camera/CameraModel.h>

namespace vw {
namespace camera {

  // The following classes VectorEquation and QuaternionEquation are
  // methods used to modify an IsisCameraModel through
  // IsisAdjustCameraModel.

  // VectorEquation
  // Returns a Vector3 that is evaluated on a double, time.
  //
  // Each element of Vector3 is an n'th order polynomial.
  class VectorEquation {
  protected:
    Vector<double> m_x_coeff;
    Vector<double> m_y_coeff;
    Vector<double> m_z_coeff;
    Vector3 m_cached_output;
    double m_cached_time;
    double m_time_offset;

    // Update Cache
    void update ( double const& t );
      
  public:
    // Constructor
    VectorEquation ( int poly_order );
    ~VectorEquation() {}

    //Evaluates the equation at time T
    Vector3 evaluate( double const& t ) {
      if ( t != m_cached_time )
	update( t );
      return m_cached_output;
    }

    // Tells the number of constants defining the equation
    // This is especially vague as it is meant for interaction with a
    // bundle adjuster
    unsigned size( void ) const { return m_x_coeff.size()*3; }

    void set_constant(unsigned index, double value);
    
    // Gives access to the constants defining the equation
    const double operator[]( unsigned const& n ) const;

    // Allows us to set the time offset
    void set_time_offset( double const& offset ) {
      m_cached_time = -1;
      m_time_offset = offset;
    }
    double get_time_offset(void) const { return m_time_offset; }

    // File IO
    void write( std::ofstream &f );
    void read( std::ifstream &f );
  };

  // Quaternion Equation
  // Returns a Quaterion that is evaluated on a double, time.
  // 
  // The quaternion is actually defined by 3 n'th order polynomials
  // that define rotations about the x, y, and z axes.
  class QuaternionEquation {
  protected:
    Vector<double> m_x_coeff;
    Vector<double> m_y_coeff;
    Vector<double> m_z_coeff;
    Quaternion<double> m_cached_output;
    double m_cached_time;
    double m_time_offset;

    // Updates cache
    void update ( double const& t );

  public:
    // Constructor
    QuaternionEquation( int poly_order );
    ~QuaternionEquation() {}

    //Evaluate the equation at time T
    Quaternion<double> evaluate( double const& t ) {
      if ( t != m_cached_time )
	update(t);
      return m_cached_output;
    }

    // Tells the number of constants defining the equation
    // This is especially vague as it is meant for interaction with a
    // bundle adjuster
    unsigned size( void ) const {
      return m_x_coeff.size()*3;
    }

    void set_constant(unsigned index, double value);

    // Gives access to the constants defining the equation
    const double operator[]( unsigned const& n ) const;

    // Allows us to set the time offset
    void set_time_offset( double const& offset ) {
      m_cached_time = -1;
      m_time_offset = offset;
    }
    double get_time_offset( void ) const { return m_time_offset; }

    // File IO
    void write( std::ofstream &f );
    void read( std::ifstream &f );
  };

  // Other useful tools
  std::ostream& operator<<( std::ostream& os, VectorEquation const& eq );
  std::ostream& operator<<( std::ostream& os, QuaternionEquation const& eq );

}}

#endif//__VW_CAMERA_EQUATION__
