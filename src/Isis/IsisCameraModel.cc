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

#include <vw/Cartography/PointImageManipulation.h>

#include "Isis/IsisCameraModel.h"

// Isis Headers
#include <Cube.h>
#include <Camera.h>

using namespace vw;
using namespace vw::camera;

IsisCameraModel::IsisCameraModel(std::string cube_filename) {
  Isis::Cube cube;
  cube.Open(cube_filename);

  if ( !(cube.IsOpen()) ) 
      vw_throw(IOErr() << "IsisCameraModel: Could not open cube file: \"" << cube_filename << "\".");

  try {
    m_isis_camera_ptr = cube.Camera();
  } catch (Isis::iException &e) {
    m_isis_camera_ptr = 0;
    vw_throw(IOErr() << "IsisCameraModel: failed to instantiate a camera model from " << cube_filename << ". " << e.what());
  }	
  
  cube.Close();
}

Vector2 IsisCameraModel::point_to_pixel(Vector3 const& point) const {
  Isis::Camera* cam  = static_cast<Isis::Camera*>(m_isis_camera_ptr);

//   // Convert into "Universal Ground" coordinates
//   Vector3 lon_lat_radius = cartography::xyz_to_lon_lat_radius(point);

//   // Set the current "active" pixel for the upcoming computation
//   cam->SetUniversalGround(lon_lat_radius[1], lon_lat_radius[0], lon_lat_radius[2]);

//   return Vector2(cam->Sample(), cam->Line());

  vw_throw (NoImplErr() << "IsisCameraModel::point_to_pixel() is not yet implemented.\n");
  return Vector2();  // Never reached
}

Vector3 IsisCameraModel::pixel_to_vector (Vector2 const& pix) const {
  Isis::Camera* cam  = static_cast<Isis::Camera*>(m_isis_camera_ptr);

  // Set the current "active" pixel for the upcoming computations
  cam->SetImage(pix[0],pix[1]);

  // Compute vector from the spacecraft to the ground intersection point
  double p[3];
  cam->Coordinate(p);
  Vector3 ground_pt(p[0],p[1],p[2]);

  cam->InstrumentPosition(p);
  Vector3 instrument_pt(p[0],p[1],p[2]);

  return normalize(ground_pt - instrument_pt);
}

Vector3 IsisCameraModel::camera_center(Vector2 const& pix ) const {  
  Isis::Camera* cam  = static_cast<Isis::Camera*>(m_isis_camera_ptr);

  double pos[3];
  cam->InstrumentPosition(pos);
  return Vector3(pos[0],pos[1],pos[2]);
}

Quaternion<double> IsisCameraModel::camera_pose(Vector2 const& pix ) const {
  Isis::Camera* cam  = static_cast<Isis::Camera*>(m_isis_camera_ptr);

  vw_throw(NoImplErr() << "IsisCameraModel::camera_pose() is not yet implemented.\n");
}



