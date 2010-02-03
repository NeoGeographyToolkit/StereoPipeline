// __BEGIN_LICENSE__
// Copyright (C) 2006-2009 United States Government as represented by
// the Administrator of the National Aeronautics and Space Administration.
// All Rights Reserved.
// __END_LICENSE__

// ASP & VW
#include <asp/IsisIO/IsisAdjustCameraModel.h>
#include <vw/Math/EulerAngles.h>

// Isis
#include <Filename.h>
#include <CameraFactory.h>
#include <SerialNumber.h>


using namespace vw;
using namespace vw::camera;
using namespace asp;

//-------------------------------------------------------------------------
//  Constructors / Deconstructor
//-------------------------------------------------------------------------

IsisAdjustCameraModel::IsisAdjustCameraModel( std::string cube_filename,
                                              boost::shared_ptr<BaseEquation> position_func,
                                              boost::shared_ptr<BaseEquation> pose_func ) :
  m_position_f( position_func ),
  m_pose_f( pose_func ) {

  // Opening labels and camera
  Isis::Filename cubefile( cube_filename.c_str() );
  m_label.Read( cubefile.Expanded() );
  m_camera = Isis::CameraFactory::Create( m_label );

  // Gutting Camera
  m_distortmap = m_camera->DistortionMap();
  m_focalmap   = m_camera->FocalPlaneMap();
  m_detectmap  = m_camera->DetectorMap();
  m_alphacube  = new Isis::AlphaCube( m_label );

  // Throw error if this is map projected image
  if ( m_camera->HasProjection() )
    vw_throw( NoImplErr() << "Don't support map projected images" );

  // Adjusting time offset so equations are reference from middle of cache
  double middle_et = m_camera->CacheStartTime() + (m_camera->CacheEndTime()-m_camera->CacheStartTime())/2.0;
  m_position_f->set_time_offset( middle_et );
  m_pose_f->set_time_offset( middle_et );
}

IsisAdjustCameraModel::~IsisAdjustCameraModel() {
  if ( m_camera )
    delete m_camera;
  if ( m_alphacube )
    delete m_alphacube;
}

//-------------------------------------------------------------------------
// Set Time Function ( used to avoid setting the camera too much )
//-------------------------------------------------------------------------

void IsisAdjustCameraModel::SetTime( Vector2 const& px,
                                     bool calc ) const {
  if ( px != m_c_location ) {
    m_c_location = px;
    m_detectmap->SetParent( m_alphacube->AlphaSample(px[0]),
                            m_alphacube->AlphaLine(px[1]) );

    if ( calc ) {
      // Calculating Spacecraft position and pose
      double ipos[3];
      m_camera->InstrumentPosition(ipos);
      m_center[0] = ipos[0];
      m_center[1] = ipos[1];
      m_center[2] = ipos[2];
      m_center *= 1000;
      m_center += m_position_f->evaluate( m_camera->EphemerisTime() );

      std::vector<double> rot_inst = m_camera->InstrumentRotation()->Matrix();
      std::vector<double> rot_body = m_camera->BodyRotation()->Matrix();
      MatrixProxy<double,3,3> R_inst(&(rot_inst[0]));
      MatrixProxy<double,3,3> R_body(&(rot_body[0]));
      Vector3 angles = m_pose_f->evaluate( m_camera->EphemerisTime() );

      m_pose = Quaternion<double>(R_inst*transpose(R_body)*
                                  math::euler_to_rotation_matrix( angles[0],
                                                                  angles[1],
                                                                  angles[2],
                                                                  "xyz") );
    }
  }
}

//-------------------------------------------------------------------------
//  Traditional Camera Routines
//-------------------------------------------------------------------------

Vector2 IsisAdjustCameraModel::point_to_pixel( Vector3 const& point ) const {
  Vector2 result;

  if ( m_camera->GetCameraType() == 2 ) {
    // Use own LMA to find correct ephemeris time. This also gives the
    // ability to use own functions for ET.
    double start_e = m_camera->CacheStartTime() + (m_camera->CacheEndTime()-m_camera->CacheStartTime())/2.0;

    // Build LMA
    EphemerisLMA model( point, m_camera, m_distortmap,
                        m_focalmap, m_position_f, m_pose_f );
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

    // Converting now to pixel
    m_camera->SetEphemerisTime( solution_e[0] );

  } else if ( m_camera->GetCameraType() != 0 ) {
    vw_throw( NoImplErr() << "IsisAdjustCameraModel::point_to_pixel does not support any cmaeras other than LineScane and Frame" );
  }

  // Pulling out camera position for current time
  double ipos[3];
  m_camera->InstrumentPosition(ipos);
  m_center[0] = ipos[0];
  m_center[1] = ipos[1];
  m_center[2] = ipos[2];
  m_center *= 1000;
  m_center += m_position_f->evaluate( m_camera->EphemerisTime() );

  // Pulling out camera pose
  std::vector<double> rot_inst = m_camera->InstrumentRotation()->Matrix();
  std::vector<double> rot_body = m_camera->BodyRotation()->Matrix();
  MatrixProxy<double,3,3> R_inst(&(rot_inst[0]));
  MatrixProxy<double,3,3> R_body(&(rot_body[0]));
  Vector3 angles = m_pose_f->evaluate( m_camera->EphemerisTime() );
  m_pose = Quaternion<double>(R_inst*transpose(R_body)*
                              math::euler_to_rotation_matrix( angles[0],
                                                              angles[1],
                                                              angles[2],
                                                              "xyz") );

  // Actually projecting point now
  Vector3 look = normalize( point - m_center );
  look = m_pose.rotate( look );
  look = m_camera->FocalLength() * ( look / look[2] );
  m_distortmap->SetUndistortedFocalPlane( look[0], look[1] );
  m_focalmap->SetFocalPlane( m_distortmap->FocalPlaneX(),
                             m_distortmap->FocalPlaneY() );
  m_detectmap->SetDetector( m_focalmap->DetectorSample(),
                            m_focalmap->DetectorLine() );
  result[0] = m_alphacube->BetaSample(m_detectmap->ParentSample());
  result[1] = m_alphacube->BetaLine(m_detectmap->ParentLine());

  SetTime(result,false);
  return result-Vector2(1,1);
}

Vector3 IsisAdjustCameraModel::pixel_to_vector( Vector2 const& pix ) const {
  // Converting to ISIS index
  Vector2 px = pix + Vector2(1,1);
  SetTime( px, true );

  // Finding rotation from camera frame to world frame
  Vector3 result;
  m_focalmap->SetDetector( m_detectmap->DetectorSample(),
                           m_detectmap->DetectorLine() );
  m_distortmap->SetFocalPlane( m_focalmap->FocalPlaneX(),
                               m_focalmap->FocalPlaneY() );
  result[0] = m_distortmap->UndistortedFocalPlaneX();
  result[1] = m_distortmap->UndistortedFocalPlaneY();
  result[2] = m_distortmap->UndistortedFocalPlaneZ();
  result = normalize( result );

  // Apply rotation of image camera
  result = inverse( m_pose ).rotate( result );
  return result;
}

Vector3
IsisAdjustCameraModel::camera_center( Vector2 const& pix ) const {
  // Converting to ISIS index
  Vector2 px = pix + Vector2(1,1);
  SetTime( px, true );
  return m_center;
}

Quaternion<double>
IsisAdjustCameraModel::camera_pose( Vector2 const& pix ) const {
  // Converting to ISIS index
  Vector2 px = pix + Vector2(1,1);
  SetTime( px, true );
  return m_pose;
}

std::string IsisAdjustCameraModel::serial_number() const {
  Isis::Pvl copy( m_label );
  return Isis::SerialNumber::Compose( copy, true );
}

//-------------------------------------------------------------------------
//  Solver for projecting points into the camera
//-------------------------------------------------------------------------

// EphemerisLMA::operator()
//
// LMA for projecting point into modified linescan camera
IsisAdjustCameraModel::EphemerisLMA::result_type
IsisAdjustCameraModel::EphemerisLMA::operator()( IsisAdjustCameraModel::EphemerisLMA::domain_type const& x ) const {

  // Setting Ephemeris Time
  m_camera->SetEphemerisTime( x[0] );

  double ipos[3];
  m_camera->InstrumentPosition(ipos);
  Vector3 instru(ipos[0],ipos[1],ipos[2]);
  instru *= 1000;
  instru += m_position_f->evaluate( x[0] );

  Vector3 lookB = normalize( m_point - instru );
  Vector3 angles = m_pose_f->evaluate( x[0] );
  lookB = math::euler_to_rotation_matrix( angles[0],
                                          angles[1],
                                          angles[2],
                                          "xyz" ) * lookB;

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
  result_type result(1);
  result[0] = m_focalmap->DetectorLineOffset() - m_focalmap->DetectorLine();

  return result;
}

/*
// project_using_current
//
// Assuming that m_camera's ephemeris time (position and pose) have
// been set correctly by previous functions, project this point into
// pixels on the camera.
//
// This is to reduce repeated code. Returns in ISIS indexed.
inline Vector2
IsisAdjustCameraModel::project_using_current( Vector3 const& point ) const {
  double ipos[3];
  m_camera->InstrumentPosition(ipos);
  Vector3 instru(ipos[0],ipos[1],ipos[2]);
  instru *= 1000;
  instru += m_position_f->evaluate( m_camera->EphemerisTime() );

  Vector3 lookB = normalize(point-instru);
  Vector3 angles = m_pose_f->evaluate( m_camera->EphemerisTime() );
  lookB = math::euler_to_rotation_matrix( angles[0],
                                          angles[1],
                                          angles[2],
                                          "xyz" ) * lookB;

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

// optimized_linescan_point_to_pixel
//
// This solves for the correct ephemeris time of a linescan camera
// that represents an accurate projection of a point into the
// camera. This is a replacement for LineScanCameraGroundMap from
// Isis.
//
// Returns ISIS indexed pixel location.
Vector2
IsisAdjustCameraModel::optimized_linescan_point_to_pixel( Vector3 const& point ) const {

  // First seed LMA with an ephemeris time in the middle of the image
  double middle = m_camera->Lines() / 2;
  m_detectmap->SetParent( 1, m_alphacube->AlphaLine(middle) );
  double start_e = m_camera->EphemerisTime();

  // Build LMA
  EphemerisLMA model( point, m_camera, m_distortmap,
                      m_focalmap, m_position_f, m_pose_f );
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

  // Converting now to pixel
  m_camera->SetEphemerisTime( solution_e[0] );
  return project_using_current( point );
}
*/
