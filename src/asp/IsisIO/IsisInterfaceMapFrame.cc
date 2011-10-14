// __BEGIN_LICENSE__
// Copyright (C) 2006-2011 United States Government as represented by
// the Administrator of the National Aeronautics and Space Administration.
// All Rights Reserved.
// __END_LICENSE__


// ASP
#include <asp/IsisIO/IsisInterfaceMapFrame.h>
#include <vw/Cartography/SimplePointImageManipulation.h>

// ISIS
#include <ProjectionFactory.h>
#include <Projection.h>
#include <CameraGroundMap.h>
#include <CameraDistortionMap.h>
#include <Distance.h>
#include <SurfacePoint.h>
#include <Latitude.h>
#include <Longitude.h>

using namespace vw;
using namespace asp;
using namespace asp::isis;

// Constructor
IsisInterfaceMapFrame::IsisInterfaceMapFrame( std::string const& filename ) :
  IsisInterface(filename), m_projection( Isis::ProjectionFactory::CreateFromCube( m_label ) ) {

  // Gutting Isis::Camera
  m_groundmap = m_camera->GroundMap();
  m_distortmap = m_camera->DistortionMap();
  m_camera->Radii( m_radii );

  // Calculating Center (just once)
  m_camera->InstrumentPosition(&m_center[0]);
  m_center *= 1000;

  // Calculating Pose (just once)
  std::vector<double> rot_inst = m_camera->InstrumentRotation()->Matrix();
  std::vector<double> rot_body = m_camera->BodyRotation()->Matrix();
  MatrixProxy<double,3,3> R_inst(&(rot_inst[0]));
  MatrixProxy<double,3,3> R_body(&(rot_body[0]));
  m_pose = Quat(R_body*transpose(R_inst));
}

Vector2
IsisInterfaceMapFrame::point_to_pixel( Vector3 const& point ) const {
  Vector3 lon_lat_radius = cartography::xyz_to_lon_lat_radius( point );
  if ( lon_lat_radius[0] < 0 )
    lon_lat_radius[0] += 360;

  // Projecting into the camera
  m_groundmap->SetGround(
    Isis::SurfacePoint( Isis::Latitude( lon_lat_radius[1], Isis::Angle::Degrees ),
                        Isis::Longitude( lon_lat_radius[0], Isis::Angle::Degrees ),
                        Isis::Distance( lon_lat_radius[2], Isis::Distance::Meters ) ) );
  m_distortmap->SetUndistortedFocalPlane( m_groundmap->FocalPlaneX(),
                                          m_groundmap->FocalPlaneY() );

  // Projection back out on to DEM
  m_groundmap->SetFocalPlane( m_distortmap->UndistortedFocalPlaneX(),
                              m_distortmap->UndistortedFocalPlaneY(),
                              m_distortmap->UndistortedFocalPlaneZ() );

  m_projection->SetGround( m_camera->UniversalLatitude(),
                           m_camera->UniversalLongitude() );
  return Vector2( m_projection->WorldX() - 1,
                  m_projection->WorldY() - 1 );
}

Vector3
IsisInterfaceMapFrame::pixel_to_vector( Vector2 const& px ) const {
  m_projection->SetWorld(px[0] + 1, px[1] + 1);
  Vector3 lon_lat_radius( m_projection->UniversalLongitude(),
                          m_projection->UniversalLatitude(), 0 );

  // Solving for radius
  if ( m_camera->HasElevationModel() ) {
    lon_lat_radius[2] =
      m_camera->DemRadius( Isis::Latitude(lon_lat_radius[1], Isis::Angle::Degrees),
                           Isis::Longitude(lon_lat_radius[0], Isis::Angle::Degrees) ).GetMeters();
  } else {
    Vector2 lon_lat = subvector(lon_lat_radius,0,2);
    lon_lat = lon_lat * M_PI/180;
    double bclon = m_radii[1].GetMeters()*cos(lon_lat[0]);
    double aslon = m_radii[0].GetMeters()*sin(lon_lat[0]);
    double cclat = m_radii[2].GetMeters()*cos(lon_lat[1]);
    double xyradius = m_radii[0].GetMeters() * m_radii[1].GetMeters() / sqrt(bclon*bclon + aslon*aslon);
    double xyslat = xyradius*sin(lon_lat[1]);
    lon_lat_radius[2] = xyradius * m_radii[2].GetMeters() / sqrt(cclat*cclat + xyslat*xyslat );
  }
  Vector3 point = cartography::lon_lat_radius_to_xyz(lon_lat_radius);
  return normalize(point-m_center);
}

Vector3 IsisInterfaceMapFrame::camera_center( Vector2 const& /*pix*/ ) const {
  return m_center;
}

Quat IsisInterfaceMapFrame::camera_pose( Vector2 const& /*pix*/ ) const {
  return m_pose;
}
