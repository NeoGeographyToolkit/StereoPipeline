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


#ifndef __ASP_BASE_EQUATION__
#define __ASP_BASE_EQUATION__

// STL
#include <fstream>
#include <iostream>
// VW
#include <vw/Math/Vector.h>

namespace asp {

  // Vector Equation is a method
  // used to modify an IsisCameraModel through
  // IsisAdjustCameraModel.

  // BaseEquation
  // Returns a Vector3 that is evaluated on a double, time.
  //
  class BaseEquation {
  protected:
    vw::Vector3 m_cached_output;
    double m_cached_time;
    double m_time_offset;

    // Update Cache (m_cached_output)
    virtual void update ( double const& t ) = 0;

  public:
    // Constructor
    virtual ~BaseEquation() {}
    virtual std::string type() const = 0;

    //Evaluates the equation at time T
    vw::Vector3 evaluate( double const& t ) {
      if ( t != m_cached_time )
        update( t );
      return m_cached_output;
    }
    vw::Vector3 operator()( double const& t ) { return evaluate(t);}

    // Tells the number of constants defining the equation
    // This is especially vague as it is meant for interaction with a
    // bundle adjuster. BA just wants to roll through the constants
    // and redefine them.
    virtual size_t size() const = 0;
    virtual double& operator[]( size_t const& n ) = 0;
    const double& operator[]( size_t const& n ) const {
      return this->operator[](n);
    }

    // Allows us to set the time offset
    void set_time_offset( double const& offset ) {
      m_cached_time = -1;
      m_time_offset = offset;
    }
    double get_time_offset() const { return m_time_offset; }

    // FileIO routines
    virtual void write( std::ofstream &f ) = 0;
    virtual void read( std::ifstream &f ) = 0;

  };


}

#endif//__ASP_BASE_EQUATION__
