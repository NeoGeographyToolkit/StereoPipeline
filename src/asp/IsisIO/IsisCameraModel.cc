// __BEGIN_LICENSE__
// Copyright (C) 2006-2009 United States Government as represented by
// the Administrator of the National Aeronautics and Space Administration.
// All Rights Reserved.
// __END_LICENSE__

// ASP & VW
#include <asp/IsisIO/IsisCameraModel.h>
#include <vw/Cartography/PointImageManipulation.h>

// ISIS
#include <Filename.h>
#include <SerialNumber.h>

using namespace vw;
using namespace vw::camera;

// Constructor
IsisCameraModel::IsisCameraModel( std::string cube_filename ) {
  // Opening Label
  Isis::Filename cubefile( cube_filename.c_str() );
  m_label.Read( cubefile.Expanded() );

  // Opening Isis::Camera
  m_camera = Isis::CameraFactory::Create( m_label );

  // Creating copy of alpha cube
  m_alphacube = new Isis::AlphaCube( m_label );

  // Gutting Isis::Camera
  m_distortmap = m_camera->DistortionMap();
  m_focalmap = m_camera->FocalPlaneMap();
  m_detectmap = m_camera->DetectorMap();
}

// Destructor
IsisCameraModel::~IsisCameraModel() {
  if ( m_camera )
    delete m_camera;
  if ( m_alphacube )
    delete m_alphacube;
}

// Methods
//-----------------------------------------------
Vector2 IsisCameraModel::point_to_pixel( Vector3 const& point ) const {

  // There is no optimized method for this as we need to use the
  // linescan camera's ground map to figure out where the camera
  // is. That's an optimization problem :(.

  Vector3 lon_lat_radius = cartography::xyz_to_lon_lat_radius(point);
  m_camera->SetUniversalGround( lon_lat_radius[1],
                                lon_lat_radius[0],
                                lon_lat_radius[2] );
  Vector2 result;
  result[0] = m_camera->Sample() - 1;  // ISIS indexes on 1
  result[1] = m_camera->Line() - 1;

  return result;
}

Vector3 IsisCameraModel::pixel_to_vector( Vector2 const& pix ) const {
  // ISIS indexes from (1,1);
  Vector2 px = pix;
  px += Vector2(1,1);

  Vector3 result;
  if ( !m_camera->HasProjection() ) {
    // Standard Image
    px[0] = m_alphacube->AlphaSample(px[0]);
    px[1] = m_alphacube->AlphaLine(px[1]);
    m_detectmap->SetParent( px[0], px[1] );
    m_focalmap->SetDetector( m_detectmap->DetectorSample(),
                             m_detectmap->DetectorLine() );
    m_distortmap->SetFocalPlane( m_focalmap->FocalPlaneX(),
                                 m_focalmap->FocalPlaneY() );
    result[0] = m_distortmap->UndistortedFocalPlaneX();
    result[1] = m_distortmap->UndistortedFocalPlaneY();
    result[2] = m_distortmap->UndistortedFocalPlaneZ();
    result = normalize( result );
    std::vector<double> lookC(3);
    lookC[0] = result[0];
    lookC[1] = result[1];
    lookC[2] = result[2];
    std::vector<double> lookJ = m_camera->InstrumentRotation()->J2000Vector(lookC);
    lookC = m_camera->BodyRotation()->ReferenceVector(lookJ);
    result[0] = lookC[0];
    result[1] = lookC[1];
    result[2] = lookC[2];
  } else {
    // Map Projected Image
    m_camera->SetImage(pix[0],pix[1]);
    double p[3];

    // Compute Instrument and Ground Pts
    m_camera->InstrumentPosition(p);
    Vector3 instrument_pt(p[0],p[1],p[2]);
    m_camera->Coordinate(p);
    Vector3 ground_pt(p[0],p[1],p[2]);
    result = normalize( ground_pt - instrument_pt );
  }
  return result;
}

Vector3 IsisCameraModel::camera_center( Vector2 const& pix ) const {
  // ISIS indexes from (1,1);
  Vector2 px = pix;
  px += Vector2(1,1);

  if ( !m_camera->HasProjection() ) {
    // Standard Image
    px[0] = m_alphacube->AlphaSample(px[0]);
    px[1] = m_alphacube->AlphaLine(px[1]);
    m_detectmap->SetParent( px[0], px[1] );
  } else {
    // Map Projected Image
    m_camera->SetImage( px[0], px[1] );
  }
  double pos[3];
  m_camera->InstrumentPosition(pos);
  return Vector3(pos[0]*1000,pos[1]*1000,pos[2]*1000);
}

Quaternion<double> IsisCameraModel::camera_pose(Vector2 const& pix ) const {
  // ISIS indexes from (1,1);
  Vector2 px = pix;
  px += Vector2(1,1);

  if ( !m_camera->HasProjection() ) {
    // Standard Image
    px[0] = m_alphacube->AlphaSample(px[0]);
    px[1] = m_alphacube->AlphaLine(px[1]);
    m_detectmap->SetParent( px[0], px[1] );
  } else {
    // Map Projected Image
    m_camera->SetImage( px[0], px[1] );
  }

  // Convert from instrument frame -> J2000 -> Body Fixed frame
  std::vector<double> rot_inst = m_camera->InstrumentRotation()->Matrix();
  std::vector<double> rot_body = m_camera->BodyRotation()->Matrix();
  MatrixProxy<double,3,3> R_inst(&(rot_inst[0]));
  MatrixProxy<double,3,3> R_body(&(rot_body[0]));

  return Quaternion<double>(R_inst*inverse(R_body));
}

int IsisCameraModel::lines( void ) const {
  return m_camera->Lines();
}

int IsisCameraModel::samples( void ) const {
  return m_camera->Samples();
}

std::string IsisCameraModel::serial_number( void ) const {
  Isis::Pvl copy( m_label );
  return Isis::SerialNumber::Compose(copy,true);
}
