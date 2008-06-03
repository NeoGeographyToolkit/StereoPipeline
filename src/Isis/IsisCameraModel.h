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

/// \file PinholeModel.h
/// 
/// This file contains the ISIS camera model.
///
#ifndef __VW_CAMERAMODEL_ISIS_H__
#define __VW_CAMERAMODEL_ISIS_H__

#include <vw/Math/Vector.h>
#include <vw/Math/Matrix.h>

#include <vw/Camera/CameraModel.h>

namespace vw { 
namespace camera {

  class IsisCameraModel : public CameraModel {

    // A void pointer which is cast to Isis::Camera* in the
    // implentation of the methods below.
    void* m_isis_camera_ptr;
    void* m_isis_cube;

  public:
    //------------------------------------------------------------------
    // Constructors / Destructors
    //------------------------------------------------------------------
    IsisCameraModel(std::string cube_filename);

    virtual ~IsisCameraModel();
    virtual std::string type() const { return "Isis"; }
 
    //------------------------------------------------------------------
    // Methods
    //------------------------------------------------------------------


    //  Computes the image of the point 'point' in 3D space on the
    //  image plane.  Returns a pixel location (col, row) where the
    //  point appears in the image. 
    virtual Vector2 point_to_pixel(Vector3 const& point) const;    

    // Returns a (normalized) pointing vector from the camera center
    //  through the position of the pixel 'pix' on the image plane.
    virtual Vector3 pixel_to_vector (Vector2 const& pix) const;

    
    // Returns the position of the focal point of the camera
    virtual Vector3 camera_center(Vector2 const& pix = Vector2() ) const;

    //  Pose is a rotation which moves a vector in camera coordinates into world coordinates.
    virtual Quaternion<double> camera_pose(Vector2 const& pix = Vector2() ) const;

  private:
    mutable double m_current_line, m_current_sample;
    void set_image(double sample, double line) const;
  };

}}	// namespace vw::camera

#endif	//__VW_CAMERA_ISIS_H__
