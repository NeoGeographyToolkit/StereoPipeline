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
    //Evaluates the equation at time T
    virtual Vector3 evaluate( double const& t ) const {
      return Vector3( 0, 0, 0 );
    }
    // Tells the number of constants defining the equation
    virtual unsigned size( void ) const {
      return m_constant.size();
    }
    // Gives access to the constants defining the equation
    virtual double& operator[]( unsigned const& n ) {
      if ( n >= m_constant.size() ) {
	std::cout << "ERROR" << std::endl;
	return m_constant[0];
      } else {
	return m_constant[n];
      }
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
    //Evaluate the equation at time T
    virtual Quaternion<double> evaluate( double const& t ) const {
      return Quaternion<double>( 0, 0, 0, 0 );
    }
    // Tells the number of constants defining the equation
    virtual unsigned size( void ) const {
      return m_constant.size();
    }
    // Gives access to the constants defining the equation
    virtual double& operator[]( unsigned const& n ) {
      if ( n >= m_constant.size() ) {
	std::cout << "ERROR" << std::endl;
	return m_constant[n];
      } else {
	return m_constant[n];
      }
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
    virtual Vector3 evaluate( double const& t) const {
      return Vector3( m_constant[0], m_constant[1], m_constant[2] );
    }
  };
  
  class PoseZeroOrder : public QuaternionEquation { 
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
    }
    virtual Quaternion<double> evaluate( double const& t ) const {
      Quaternion<double> quat = vw::math::euler_to_quaternion( m_constant[0],
							       m_constant[1],
							       m_constant[2],
							       "xyz" );
      quat = quat / norm_2(quat);
      return quat;
    }
  };

}
}

#endif//__VW_CAMERA_EQUATION__
