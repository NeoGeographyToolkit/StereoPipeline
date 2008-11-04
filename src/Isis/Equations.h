#ifndef __VW_CAMERA_EQUATION__
#define __VW_CAMERA_EQUATION__

#include <vw/Math/Vector.h>
#include <vw/Math/Matrix.h>
#include <vw/Math/EulerAngles.h>

#include <vw/Camera/CameraModel.h>

namespace vw {
namespace camera {

  // This is used to communicate with the IsisAdjust Models
  class VectorEquation {
  protected:
    std::vector<double> m_constant;
    double m_time_offset;
  public:
    // Constructor
    VectorEquation ( void ) {
      //m_constant.clear();
      m_time_offset = 0;
    }
    virtual ~VectorEquation() {}

    //Evaluates the equation at time T
    virtual Vector3 evaluate( double const& t ) const {
      return Vector3( 0, 0, 0 );
    }
    // Tells the number of constants defining the equation
    virtual unsigned size( void ) const {
      return m_constant.size();
    }

    virtual void set_constant(unsigned index, double value) {
      if ( index >= m_constant.size() ) 
        vw_throw(ArgumentErr() << "VectorEquation: invalid index.");
      m_constant[index] = value;
    }
    
    // Gives access to the constants defining the equation
    virtual const double operator[]( unsigned const& n ) const {
      if ( n >= m_constant.size() ) 
        vw_throw(ArgumentErr() << "VectorEquation: invalid index.");
      return m_constant[n];
    }

    // Allows us to set the time offset
    virtual void set_time_offset( double const& offset ) {
      m_time_offset = offset;
    }
  };

  class QuaternionEquation {
  protected:
    std::vector<double> m_constant;
    double m_time_offset;
  public:
    // Constructor
    QuaternionEquation( void ) {
      //m_constant.clear();
      m_time_offset = 0;
    }
    virtual ~QuaternionEquation() {}

    //Evaluate the equation at time T
    virtual Quaternion<double> evaluate( double const& t ) const {
      return Quaternion<double>( 0, 0, 0, 0 );
    }
    // Tells the number of constants defining the equation
    virtual unsigned size( void ) const {
      return m_constant.size();
    }

    virtual void set_constant(unsigned index, double value) {
      if ( index >= m_constant.size() ) 
        vw_throw(ArgumentErr() << "VectorEquation: invalid index.");
      m_constant[index] = value;
    }

    // Gives access to the constants defining the equation
    virtual const double operator[]( unsigned const& n ) const {
      if ( n >= m_constant.size() ) 
        vw_throw(ArgumentErr() << "QuaternionEquation: invalid index.");
      return m_constant[n];
    }
    // Allows us to set the time offset
    virtual void set_time_offset( double const& offset ) {
      m_time_offset = offset;
    }
  };


  // Predefined equations. Might be helpful!
  class PositionZeroOrder : public VectorEquation {
  public:
    PositionZeroOrder( void ) { //If I'm feeling lazy
      m_constant.resize(3);
      m_time_offset = 0;
    }
    PositionZeroOrder( double x, double y, double z ) {
      m_constant.push_back(x);
      m_constant.push_back(y);
      m_constant.push_back(z);
      m_time_offset = 0;
    }
    virtual ~PositionZeroOrder() {}
    virtual Vector3 evaluate( double const& t) const {
      return Vector3( m_constant[0], m_constant[1], m_constant[2] );
    }
  };
  
  class PoseZeroOrder : public QuaternionEquation { 
    Quaternion<double> m_cached_pose;

    // The PoseZeroOrder correction does not depend on t, so we can
    // cache the quaternion in the constructor and return the cached
    // quaternion when evaluate() is called.
    void update_cache() {
      m_cached_pose = vw::math::euler_to_quaternion( m_constant[0],
                                                     m_constant[1],
                                                     m_constant[2],
                                                     "xyz" );
      // Need to do this, the rotation matrix equation for quaternion
      // expects a normalize quaternion
      m_cached_pose = m_cached_pose / norm_2(m_cached_pose);
    }

  public:
    PoseZeroOrder( void ) {
      m_constant.resize(3);
      m_time_offset = 0;
    }
    PoseZeroOrder( double x, double y, double z ) {
      m_constant.push_back(x);
      m_constant.push_back(y);
      m_constant.push_back(z);
      m_time_offset = 0;
      update_cache();
    }
    
    // Override the base class implemenation here so that we can
    // recompute the cached value of the quaternion after the
    // constants have been changed.
    virtual void set_constant(unsigned index, double value) {
      QuaternionEquation::set_constant(index,value);
      update_cache();
    }

    virtual ~PoseZeroOrder() {}
    virtual Quaternion<double> evaluate( double const& t ) const {
      return m_cached_pose;
    }
  };

}}

#endif//__VW_CAMERA_EQUATION__
