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
#include <LineScanCameraGroundMap.h>

using namespace vw;
using namespace vw::camera;

// Constructor
IsisCameraModel::IsisCameraModel( std::string cube_filename ) {
  // Opening Label
  Isis::Filename cubefile( cube_filename.c_str() );
  m_label.Read( cubefile.Expanded() );

  // Opening Isis::Camera
  m_camera = Isis::CameraFactory::Create( m_label );

  VW_ASSERT( m_camera->GetCameraType() == 2 ||
             m_camera->GetCameraType() == 0,
             NoImplErr() << "IsisCameraModel has not been tested on anything other than LineScan and Frame camera models" );

  // Creating copy of alpha cube
  m_alphacube = new Isis::AlphaCube( m_label );

  // Gutting Isis::Camera
  m_distortmap = m_camera->DistortionMap();
  m_focalmap = m_camera->FocalPlaneMap();
  m_detectmap = m_camera->DetectorMap();
  m_groundmap = m_camera->GroundMap();
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

  Vector2 result;

  if ( m_camera->GetCameraType() == 2 ) {
    // Isis's LineScanCameraGroundMap seems unreliable. It fails with
    // out response. This method was going to happening anyway when we
    // get to IsisAdjustCameraModel as I need to plug my own functions
    // to the spice and do not want to destroy the original spice data
    // and fit a polynomial to it like Isis's Jigsaw.
    //
    // This method also avoids conversion to LonLatRadius.
    result = optimized_linescan_point_to_pixel( point );
  } else if ( m_camera->GetCameraType() == 0 ) {
    // Frame Camera .. avoid conversion to LLA
    // (position is constant here .. so don't worry about setting ET)
    double ipos[3];
    m_camera->InstrumentPosition(ipos);
    Vector3 instru(ipos[0],ipos[1],ipos[2]);
    instru *= 1000;
    Vector3 lookB = normalize(point-instru);
    std::vector<double> lookB_copy(3);
    lookB_copy[0] = lookB[0];
    lookB_copy[1] = lookB[1];
    lookB_copy[2] = lookB[2];
    std::vector<double> lookJ = m_camera->BodyRotation()->J2000Vector(lookB_copy);
    std::vector<double> lookC = m_camera->InstrumentRotation()->ReferenceVector(lookJ);
    Vector3 look;
    look[0] = lookC[0];
    look[1] = lookC[1];
    look[2] = lookC[2];
    look = m_camera->FocalLength() * ( look / look[2] );
    m_distortmap->SetUndistortedFocalPlane( look[0], look[1] );
    m_focalmap->SetFocalPlane( m_distortmap->FocalPlaneX(),
                               m_distortmap->FocalPlaneY() );
    m_detectmap->SetDetector( m_focalmap->DetectorSample(),
                              m_focalmap->DetectorLine() );
    Vector2 pixel( m_detectmap->ParentSample(),
                   m_detectmap->ParentLine() );
    pixel[0] = m_alphacube->BetaSample( pixel[0] );
    pixel[1] = m_alphacube->BetaLine( pixel[1] );
    result = pixel;
  } else {
    vw_throw( NoImplErr() << "IsisCameraModel::point_to_pixel does not support any cameras other than LineScan and Frame" );
  }

  return result-Vector2(1,1); // (Isis references from 1)
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

IsisCameraModel::EphemerisLMA::result_type
IsisCameraModel::EphemerisLMA::operator()( IsisCameraModel::EphemerisLMA::domain_type const& x ) const {

  // Setting Ephemeris Time
  m_camera->SetEphemerisTime( x[0] );

  // Calculating the look direction in camera frame
  double ipos[3];
  m_camera->InstrumentPosition(ipos);
  Vector3 instru( ipos[0], ipos[1], ipos[2] );
  instru *= 1000;  // Spice gives in km
  Vector3 lookB = normalize( m_point - instru );
  std::vector<double> lookB_copy(3);
  lookB_copy[0] = lookB[0];
  lookB_copy[1] = lookB[1];
  lookB_copy[2] = lookB[2];
  std::vector<double> lookJ = m_camera->BodyRotation()->J2000Vector(lookB_copy);
  std::vector<double> lookC = m_camera->InstrumentRotation()->ReferenceVector(lookJ);
  Vector3 look;
  look[0] = lookC[0];
  look[1] = lookC[1];
  look[2] = lookC[2];

  // Projecting to pixel
  look = m_camera->FocalLength() * (look / look[2]);
  m_distortmap->SetUndistortedFocalPlane(look[0], look[1]);
  m_focalmap->SetFocalPlane( m_distortmap->FocalPlaneX(),
                             m_distortmap->FocalPlaneY() );
  result_type result(1);
  result[0] = m_focalmap->DetectorLineOffset() - m_focalmap->DetectorLine();

  return result;
}

Vector2 IsisCameraModel::optimized_linescan_point_to_pixel( Vector3 const& point ) const {

  // First seed LMA with an ephemeris time in the middle of the image
  double middle = m_camera->Lines() / 2;
  m_detectmap->SetParent( 1, m_alphacube->AlphaLine(middle) );
  double start_e = m_camera->EphemerisTime();

  // Build LMA
  EphemerisLMA model( point, m_camera, m_distortmap, m_focalmap );
  int status;
  Vector<double> objective(1);
  Vector<double> start(1);
  start[0] = start_e;
  Vector<double> solution_e = math::levenberg_marquardt( model,
                                                         start,
                                                         objective,
                                                         status );

  // Make sure we found ideal time
  VW_ASSERT( status > 0,
             MathErr() << " Unable to project point into linescan camera " );

  // Converting now to pixel
  m_camera->SetEphemerisTime( solution_e[0] );
  double ipos[3];
  m_camera->InstrumentPosition(ipos);
  Vector3 instru(ipos[0],ipos[1],ipos[2]);
  instru *= 1000;
  Vector3 lookB = normalize(point-instru);
  std::vector<double> lookB_copy(3);
  lookB_copy[0] = lookB[0];
  lookB_copy[1] = lookB[1];
  lookB_copy[2] = lookB[2];
  std::vector<double> lookJ = m_camera->BodyRotation()->J2000Vector(lookB_copy);
  std::vector<double> lookC = m_camera->InstrumentRotation()->ReferenceVector(lookJ);
  Vector3 look;
  look[0] = lookC[0];
  look[1] = lookC[1];
  look[2] = lookC[2];
  look = m_camera->FocalLength() * ( look / look[2] );
  m_distortmap->SetUndistortedFocalPlane( look[0], look[1] );
  m_focalmap->SetFocalPlane( m_distortmap->FocalPlaneX(),
                             m_distortmap->FocalPlaneY() );
  m_detectmap->SetDetector( m_focalmap->DetectorSample(),
                            m_focalmap->DetectorLine() );
  Vector2 pixel( m_detectmap->ParentSample(),
                 m_detectmap->ParentLine() );
  pixel[0] = m_alphacube->BetaSample( pixel[0] );
  pixel[1] = m_alphacube->BetaLine( pixel[1] );

  return pixel;
}
