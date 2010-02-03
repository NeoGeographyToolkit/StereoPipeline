// __BEGIN_LICENSE__
// Copyright (C) 2006-2009 United States Government as represented by
// the Administrator of the National Aeronautics and Space Administration.
// All Rights Reserved.
// __END_LICENSE__

// ASP
#include <asp/IsisIO/IsisInterfaceMapFrame.h>
#include <vw/Cartography/SimplePointImageManipulation.h>

// ISIS
#include <ProjectionFactory.h>

using namespace vw;
using namespace asp;
using namespace asp::isis;

// Constructor
IsisInterfaceMapFrame::IsisInterfaceMapFrame( std::string const& filename ) :
  IsisInterface(filename) {

  // Gutting Isis::Camera
  m_groundmap = m_camera->GroundMap();
  m_distortmap = m_camera->DistortionMap();
  m_projection = Isis::ProjectionFactory::CreateFromCube( m_label );
  m_camera->Radii( m_radii );

  // Calculating Center (just once)
  double ipos[3];
  m_camera->InstrumentPosition(ipos);
  m_center[0] = ipos[0]*1000;
  m_center[1] = ipos[1]*1000;
  m_center[2] = ipos[2]*1000;

  // Calculating Pose (just once)
  std::vector<double> rot_inst = m_camera->InstrumentRotation()->Matrix();
  std::vector<double> rot_body = m_camera->BodyRotation()->Matrix();
  MatrixProxy<double,3,3> R_inst(&(rot_inst[0]));
  MatrixProxy<double,3,3> R_body(&(rot_body[0]));

  m_pose = Quaternion<double>(R_inst*transpose(R_body));
}

Vector2
IsisInterfaceMapFrame::point_to_pixel( Vector3 const& point ) const {
  Vector3 lon_lat_radius = cartography::xyz_to_lon_lat_radius( point );
  if ( lon_lat_radius[0] < 0 )
    lon_lat_radius[0] += 360;

  // Projecting into the camera
  m_groundmap->SetGround( lon_lat_radius[1],
                          lon_lat_radius[0],
                          lon_lat_radius[2] );
  m_distortmap->SetUndistortedFocalPlane( m_groundmap->FocalPlaneX(),
                                          m_groundmap->FocalPlaneY() );

  // Projection back out on to DEM
  m_groundmap->SetFocalPlane( m_distortmap->UndistortedFocalPlaneX(),
                              m_distortmap->UndistortedFocalPlaneY(),
                              m_distortmap->UndistortedFocalPlaneZ() );

  m_projection->SetGround( m_camera->UniversalLatitude(),
                           m_camera->UniversalLongitude() );
  Vector2 result( m_projection->WorldX(),
                  m_projection->WorldY() );
  return result - Vector2(1,1);
}

Vector3
IsisInterfaceMapFrame::pixel_to_vector( Vector2 const& pix ) const {
  Vector2 px = pix + Vector2(1,1);
  m_projection->SetWorld(px[0],px[1]);
  Vector3 lon_lat_radius( m_projection->UniversalLongitude(),
                          m_projection->UniversalLatitude(), 0 );

  // Solving for radius
  if ( m_camera->HasElevationModel() ) {
    lon_lat_radius[2] = m_camera->DemRadius( lon_lat_radius[1],
                                             lon_lat_radius[0] );
  } else {
    Vector2 lon_lat = subvector(lon_lat_radius,0,2);
    lon_lat = lon_lat * M_PI/180;
    double bclon = m_radii[1]*cos(lon_lat[0]);
    double aslon = m_radii[0]*sin(lon_lat[0]);
    double cclat = m_radii[2]*cos(lon_lat[1]);
    double xyradius = m_radii[0] * m_radii[1] / sqrt(bclon*bclon + aslon*aslon);
    double xyslat = xyradius*sin(lon_lat[1]);
    lon_lat_radius[2] = xyradius * m_radii[2] / sqrt(cclat*cclat + xyslat*xyslat );
  }
  lon_lat_radius[2] *= 1000;
  Vector3 point = cartography::lon_lat_radius_to_xyz(lon_lat_radius);
  return normalize(point-m_center);
}

Vector3 IsisInterfaceMapFrame::camera_center( Vector2 const& pix ) const {
  return m_center;
}

Quaternion<double> IsisInterfaceMapFrame::camera_pose( Vector2 const& pix ) const {
  return m_pose;
}
