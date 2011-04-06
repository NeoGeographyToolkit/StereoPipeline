// __BEGIN_LICENSE__
// Copyright (C) 2006-2010 United States Government as represented by
// the Administrator of the National Aeronautics and Space Administration.
// All Rights Reserved.
// __END_LICENSE__


// ASP
#include <asp/IsisIO/IsisInterfaceMapLineScan.h>
#include <vw/Cartography/SimplePointImageManipulation.h>

// ISIS
#include <ProjectionFactory.h>

using namespace vw;
using namespace asp;
using namespace asp::isis;

// Constructor
IsisInterfaceMapLineScan::IsisInterfaceMapLineScan( std::string const& filename ) :
  IsisInterface( filename ) {

  // Gutting Isis::Camera
  m_detectmap = m_camera->DetectorMap();
  m_distortmap = m_camera->DistortionMap();
  m_groundmap = m_camera->GroundMap();
  m_focalmap = m_camera->FocalPlaneMap();
  m_alphacube = new Isis::AlphaCube( m_label );
  m_projection = Isis::ProjectionFactory::CreateFromCube( m_label );
  m_camera->Radii( m_radii );
}

// Custom Function to help avoid over invoking the deeply buried
// functions of Isis::Sensor
void IsisInterfaceMapLineScan::SetTime( vw::Vector2 const& px,
              bool calc ) const {
  if ( px != m_c_location ) {
    m_c_location = px;

    m_projection->SetWorld( px[0],
                            px[1] );
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

    // First seed LMA with an ephemeris time in the middle of the image
    double middle = lines() / 2;
    m_detectmap->SetParent( 1, m_alphacube->AlphaLine(middle) );
    double start_e = m_camera->EphemerisTime();

    // Build LMA
    EphemerisLMA model( point, m_camera, m_distortmap, m_focalmap );
    int status;
    Vector<double> objective(1), start(1);
    start[0] = start_e;
    Vector<double> solution_e = math::levenberg_marquardt( model,
                                                           start,
                                                           objective,
                                                           status );
    // Make sure we found ideal time
    VW_ASSERT( status > 0,
               MathErr() << " Unable to project point into linescan camera " );

    // Setting to camera time to solution
    m_camera->SetEphemerisTime( solution_e[0] );

    if ( calc ) {
      // Calculating Spacecraft position and pose
      m_camera->InstrumentPosition(&m_center[0]);
      m_center *= 1000;

      std::vector<double> rot_inst = m_camera->InstrumentRotation()->Matrix();
      std::vector<double> rot_body = m_camera->BodyRotation()->Matrix();
      MatrixProxy<double,3,3> R_inst(&(rot_inst[0]));
      MatrixProxy<double,3,3> R_body(&(rot_body[0]));
      m_pose = Quat(R_body*transpose(R_inst));
    }
  }
}

// LMA for projecting point to linescan camera
IsisInterfaceMapLineScan::EphemerisLMA::result_type
IsisInterfaceMapLineScan::EphemerisLMA::operator()( IsisInterfaceMapLineScan::EphemerisLMA::domain_type const& x ) const {

  // Setting Ephemeris Time
  m_camera->SetEphemerisTime( x[0] );

  // Calculating the look direction in camera frame
  Vector3 instru;
  m_camera->InstrumentPosition(&instru[0]);
  instru *= 1000;  // Spice gives in km
  Vector3 lookB = normalize( m_point - instru );
  std::vector<double> lookB_copy(3);
  std::copy(lookB.begin(),lookB.end(),lookB_copy.begin());
  std::vector<double> lookJ = m_camera->BodyRotation()->J2000Vector(lookB_copy);
  std::vector<double> lookC = m_camera->InstrumentRotation()->ReferenceVector(lookJ);
  Vector3 look;
  std::copy(lookC.begin(),lookC.end(),look.begin());

  // Projecting to mm focal plane
  look = m_camera->FocalLength() * (look / look[2]);
  m_distortmap->SetUndistortedFocalPlane(look[0], look[1]);
  m_focalmap->SetFocalPlane( m_distortmap->FocalPlaneX(),
                             m_distortmap->FocalPlaneY() );
  result_type result(1);
  // Not exactly sure about lineoffset .. but ISIS does it
  result[0] = m_focalmap->DetectorLineOffset() - m_focalmap->DetectorLine();

  return result;
}

Vector2
IsisInterfaceMapLineScan::point_to_pixel( Vector3 const& point ) const {

  // First seed LMA with an ephemeris time in the middle of the image
  double middle_et = m_camera->CacheStartTime() + (m_camera->CacheEndTime()-m_camera->CacheStartTime())/2.0;
  Vector3 lon_lat_radius = cartography::xyz_to_lon_lat_radius( point );

  // Build LMA
  EphemerisLMA model( point, m_camera, m_distortmap, m_focalmap );
  int status;
  Vector<double> objective(1), start(1);
  start[0] = middle_et;
  Vector<double> solution_e = math::levenberg_marquardt( model,
                                                         start,
                                                         objective,
                                                         status );

  // Make sure we found ideal time
  VW_ASSERT( status > 0,
             MathErr() << " Unable to project point into linescan camera " );

  // Setting to camera time to solution
  m_camera->SetEphemerisTime( solution_e[0] );

  // Working out pointing
  m_camera->InstrumentPosition(&m_center[0]);
  m_center *= 1000;
  Vector3 look = normalize(point-m_center);

  // Calculating Rotation to camera frame
  std::vector<double> rot_inst = m_camera->InstrumentRotation()->Matrix();
  std::vector<double> rot_body = m_camera->BodyRotation()->Matrix();
  MatrixProxy<double,3,3> R_inst(&(rot_inst[0]));
  MatrixProxy<double,3,3> R_body(&(rot_body[0]));
  m_pose = Quat(R_body*transpose(R_inst));

  look = inverse(m_pose).rotate( look );
  look = m_camera->FocalLength() * (look / look[2]);

  // Projecting back on to ground to find out time
  m_groundmap->SetFocalPlane( look[0],
                              look[1],
                              look[2] );

  m_projection->SetGround( m_camera->UniversalLatitude(),
                           m_camera->UniversalLongitude() );
  Vector2 result( m_projection->WorldX(),
                  m_projection->WorldY() );

  // Caching result
  m_c_location = result;

  return result - Vector2(1,1);
}

Vector3
IsisInterfaceMapLineScan::pixel_to_vector( Vector2 const& pix ) const {
  Vector2 px = pix + Vector2(1,1);
  SetTime( px, true );

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
  Vector3 result = normalize(point-m_center);

  /*
  m_camera->SetImage( px[0], px[1] );
  double ip[3], cd[3];
  m_camera->InstrumentPosition(ip);
  m_camera->Coordinate(cd);
  VectorProxy<double,3> ipv(ip), cdv(cd);
  */

  return result;
}

Vector3
IsisInterfaceMapLineScan::camera_center( Vector2 const& pix ) const {
  Vector2 px = pix + Vector2(1,1);
  SetTime( px, true );
  return m_center;
}

Quat
IsisInterfaceMapLineScan::camera_pose( Vector2 const& pix ) const {
  Vector2 px = pix + Vector2(1,1);
  SetTime( px, true );
  return m_pose;
}
