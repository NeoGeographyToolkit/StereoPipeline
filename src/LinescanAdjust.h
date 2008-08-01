
// __BEGIN_LICENSE__
// 
// Copyright (C) 2008 United States Government as represented by the
// Administrator of the National Aeronautics and Space Administration
// (NASA).  All Rights Reserved.
// 
// Copyright 2008 Carnegie Mellon University. All rights reserved.
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

/// \file LinescanAdjust.h
///

#ifndef __LINESCAN_ADJUST_H__
#define __LINESCAN_ADJUST_H__

#include <vw/Math/Vector.h>
#include <vw/Math/Matrix.h>
#include <vw/Camera/CameraModel.h>
#include "Isis/IsisCameraModel.h"

namespace vw {

  // I recommend using this format for the PositionFuncT and PoseFuncT
  class VectorEquation {
  public:
    // Evaluates the equation at time T
    virtual Vector3 operator()( double t ) const = 0;
    // Tells the number of constants in the equation
    virtual unsigned size( void ) const = 0;
    // Gives access to the constants defining the equation
    virtual double& operator[]( unsigned n ) = 0;
  };
  
  
  class QuaternionEquation {
  public:
    virtual Quaternion<double> operator()( double t ) const = 0;
    virtual unsigned size( void ) const = 0;
    virtual double& operator[]( unsigned n ) = 0;
  };

}

// The Transformed Class
namespace vw {
namespace camera {

  template <class PositionFuncT, class PoseFuncT>
  class TransformedIsisCameraModel : public CameraModel {
    boost::shared_ptr<IsisCameraModel> m_camera;
    PositionFuncT m_position_func;
    PoseFuncT m_pose_func;

  public:
    //Generic Constructor
    TransformedIsisCameraModel( boost::shared_ptr<vw::camera::IsisCameraModel> camera_model,
				PositionFuncT const& position_func,
				PoseFuncT const& pose_func ) : 
      m_pose_func( pose_func),
      m_position_func( position_func ), m_camera( camera_model ) {
    }

    ~TransformedIsisCameraModel(){
      return;
    }

    virtual Vector2 point_to_pixel ( Vector3 const& point ) const {
      
      Vector2 orginal_pix = m_camera->point_to_pixel( point );
      Vector3 cam_center = m_camera->camera_center( orginal_pix );
      Vector3 vec = point - cam_center - m_position_func( orginal_pix[1] );
      
      Quaternion<double> rotation = m_pose_func( orginal_pix[1] );
      Vector3 new_pt = inverse( rotation ).rotate( vec ) + cam_center;
      return m_camera->point_to_pixel( new_pt );
    }

    virtual Vector3 pixel_to_vector ( Vector2 const& pix ) const{
      
      Quaternion<double> rotation = m_pose_func( pix[1] );
      return rotation.rotate( m_camera->pixel_to_vector(pix) );
    }

    virtual Vector3 camera_center ( Vector2 const& pix ) const {
      
      Vector3 translation = m_position_func( pix[1] );
      return m_camera->camera_center( pix ) + translation;
    }

    virtual Quaternion<double> camera_pose( Vector2 const& pix ) const {
      
      Quaternion<double> rotation = m_pose_func( pix[1] );
      Quaternion<double> rotation_inverse = inverse( rotation );
      return m_camera->camera_pose( pix ) * rotation_inverse;
    }

    virtual std::string type() const { return "AdjustedISIS"; }

    virtual int getLines( void ) {
      return m_camera->getLines();
    }

    virtual int getSamples( void ) {
      return m_camera->getSamples();
    }

    PositionFuncT& getPositionFunc ( void ) {
      return m_position_func;
    }

    PoseFuncT& getPoseFunc ( void ) {
      return m_pose_func;
    }

  private:
      
    };

}}


#endif // __LINESCAN_ADJUST_H__
