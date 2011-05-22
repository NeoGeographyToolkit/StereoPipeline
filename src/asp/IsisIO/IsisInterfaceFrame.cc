// __BEGIN_LICENSE__
// Copyright (C) 2006-2010 United States Government as represented by
// the Administrator of the National Aeronautics and Space Administration.
// All Rights Reserved.
// __END_LICENSE__


// ASP
#include <asp/IsisIO/IsisInterfaceFrame.h>

using namespace vw;
using namespace asp;
using namespace asp::isis;

// Constructor
IsisInterfaceFrame::IsisInterfaceFrame( std::string const& filename ) :
  IsisInterface(filename), m_alphacube( m_label ) {

  // Gutting Isis::Camera
  m_distortmap = m_camera->DistortionMap();
  m_focalmap   = m_camera->FocalPlaneMap();
  m_detectmap  = m_camera->DetectorMap();

  // Calculating Center (just once)
  m_camera->InstrumentPosition(&m_center[0]);
  m_center *= 1000;

  // Calculating Pose (just once)
  std::vector<double> rot_inst = m_camera->InstrumentRotation()->Matrix();
  std::vector<double> rot_body = m_camera->BodyRotation()->Matrix();
  MatrixProxy<double,3,3> R_inst(&(rot_inst[0]));
  MatrixProxy<double,3,3> R_body(&(rot_body[0]));

  // Instrument Rotation = Spacecraft to Camera's Frame
  // Body Rotation = Spacecraft to World Frame
  m_pose = Quat(R_body*transpose(R_inst));
}

Vector2
IsisInterfaceFrame::point_to_pixel( Vector3 const& point ) const {

  Vector3 look = normalize( point - m_center );
  std::vector<double> lookB_copy(3);
  std::copy( look.begin(), look.end(), lookB_copy.begin() );
  lookB_copy = m_camera->BodyRotation()->J2000Vector(lookB_copy);
  lookB_copy = m_camera->InstrumentRotation()->ReferenceVector(lookB_copy);
  std::copy( lookB_copy.begin(), lookB_copy.end(), look.begin() );
  look = m_camera->FocalLength() * ( look / look[2] );

  // Back Projecting
  m_distortmap->SetUndistortedFocalPlane( look[0], look[1] );
  m_focalmap->SetFocalPlane( m_distortmap->FocalPlaneX(),
                             m_distortmap->FocalPlaneY() );
  m_detectmap->SetDetector( m_focalmap->DetectorSample(),
                            m_focalmap->DetectorLine() );
  return Vector2( m_alphacube.BetaSample( m_detectmap->ParentSample() ) - 1,
                  m_alphacube.BetaLine( m_detectmap->ParentLine() ) - 1 );
}

Vector3
IsisInterfaceFrame::pixel_to_vector( Vector2 const& px ) const {
  m_detectmap->SetParent( m_alphacube.AlphaSample(px[0]+1),
                          m_alphacube.AlphaLine(px[1]+1));
  m_focalmap->SetDetector( m_detectmap->DetectorSample(),
                           m_detectmap->DetectorLine() );
  m_distortmap->SetFocalPlane( m_focalmap->FocalPlaneX(),
                               m_focalmap->FocalPlaneY() );
  std::vector<double> look(3);
  look[0] = m_distortmap->UndistortedFocalPlaneX();
  look[1] = m_distortmap->UndistortedFocalPlaneY();
  look[2] = m_distortmap->UndistortedFocalPlaneZ();
  VectorProxy<double,3> result( &look[0] );
  result = normalize( result );
  look = m_camera->InstrumentRotation()->J2000Vector(look);
  look = m_camera->BodyRotation()->ReferenceVector(look);
  return result;
}

Vector3
IsisInterfaceFrame::camera_center( Vector2 const& /*pix*/ ) const {
  return m_center;
}

Quat
IsisInterfaceFrame::camera_pose( Vector2 const& /*pix*/ ) const {
  return m_pose;
}
