#ifndef __VW_CAMERA_EQUATION__
#define __VW_CAMERA_EQUATION__

#include <vw/Math/Vector.h>
#include <vw/Math/Matrix.h>

#include <vw/Camera/CameraModel.h>

namespace vw {
namespace camera {

  // This is used to communicate with the IsisAdjust Models
  class VectorEquation {
  public:
    //Evaluates the equation at time T
    virtual Vector3 evaluate( double t ) const = 0;
    // Tells the number of constants defining the equation
    virtual unsigned size( void ) const = 0;
    // Gives access to the constants defining the equation
    virtual double& operator[]( unsigned n ) = 0;
  };

  class QuaternionEquation {
  public:
    //Evaluate the equation at time T
    virtual Quaternion<double> evaluate( double t ) const = 0;
    virtual unsigned size( void ) const = 0;
    virtual double& operator[]( unsigned n ) = 0;
  };

}
}

#endif//__VW_CAMERA_EQUATION__
