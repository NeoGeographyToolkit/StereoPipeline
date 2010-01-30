// __BEGIN_LICENSE__
// Copyright (C) 2006-2009 United States Government as represented by
// the Administrator of the National Aeronautics and Space Administration.
// All Rights Reserved.
// __END_LICENSE__

#include <asp/IsisIO/IsisAdjustCameraModel.h>

using namespace vw;
using namespace vw::camera;

//-------------------------------------------------------------------------
//  Constructors / Deconstructor
//-------------------------------------------------------------------------

IsisAdjustCameraModel::IsisAdjustCameraModel( std::string cube_filename,
                                              boost::shared_ptr<BaseEquation> position_func,
                                              boost::shared_ptr<BaseEquation> pose_func ) :
  IsisCameraModel( cube_filename ),
  m_position_f( position_func ),
  m_pose_f( pose_func ) {

  // Adjusting time offset so equations are reference from middle of cache
  double middle_et = m_camera->CacheStartTime() + (m_camera->CacheEndTime()-m_camera->CacheStartTime())/2.0;
  m_position_f->set_time_offset( middle_et );
  m_pose_f->set_time_offset( middle_et );
}

IsisAdjustCameraModel::~IsisAdjustCameraModel() {}

//-------------------------------------------------------------------------
//  Traditional Camera Routines
//-------------------------------------------------------------------------

Vector2 IsisAdjustCameraModel::point_to_pixel( Vector3 const& point ) const {

  Vector2 result;

  if ( m_camera->GetCameraType() == 2 ) {
    // Use own LMA to find correct ephemeris time. This also gives the
    // ability to use own functions for ET.
    result = optimized_linescan_point_to_pixel( point );
  } else if ( m_camera->GetCameraType() == 0 ) {
    // Frame Camera .. avoid conversion to LLA
    // (position is constant here .. so don't worry about setting ET)
    result = project_using_current( point );
  } else {
    vw_throw( NoImplErr() << "IsisAdjustCameraModel::point_to_pixel does not support any cmaeras other than LineScane and Frame" );
  }

  return result-Vector2(1,1);
}

Vector3 IsisAdjustCameraModel::pixel_to_vector( Vector2 const& pix ) const {
  // Converting to ISIS index
  Vector2 px = pix + Vector2(1,1);

  // Finding rotation from camera frame to world frame
  Quaternion<double> look_transform = camera_pose( pix );
  result = inverse( look_transform ).rotate( result );

  Vector3 result;
  if ( !m_camera->HasProjection() ) {
    // Standard Image
    px[0] = m_alphacube->AlphaSample( px[0] );
    px[1] = m_alphacube->AlphaLine( px[1] );
    m_detectmap->SetParent( px[0], px[1] );
    m_focalmap->SetDetector( m_detectmap->DetectorSample(),
                             m_detectmap->DetectorLine() );
    m_distortmap->SetFocalPlane( m_focalmap->FocalPlaneX(),
                                 m_focalmap->FocalPlaneY() );
    result[0] = m_distortmap->UndistortedFocalPlaneX();
    result[1] = m_distortmap->UndistortedFocalPlaneY();
    result[2] = m_distortmap->UndistortedFocalPlaneZ();
    result = normalize( result );
  } else {
    // Map Projected Image
    m_camera->SetImage( px[0], px[1] );
    double p[3];

    // Compute Instrument and Ground Pts
    m_camera->InstrumentPosition(p);
    Vector3 instrument_pt(p[0],p[1],p[2]);
    m_camera->Coordinate(p);
    Vector3 ground_pt(p[0],p[1],p[2]);
    result = normalize( ground_pt - instrument_pt );
  }
  // Apply rotation of image camera
  result = inverse( look_transform ).rotate( result );

  return result;
}

Vector3
IsisAdjustCameraModel::camera_center( Vector2 const& pix = Vector2() ) const {
  // Converting to ISIS index
  Vector2 px = pix + Vector2(1,1);

  if ( !m_camera->HasProjection() ) {
    // Standard Image (from Frame or LineScan)
    px[0] = m_alphacube->AlphaSample(px[0]);
    px[1] = m_alphacube->AlphaLine(px[1]);
    m_detectmap->SetParent( px[0], px[1] ); // Sets ET in LineScan
  } else {
    // Map Projected Image
    m_camera->SetImage( px[0], px[1] );
  }
  double pos[3];
  m_camera->InstrumentPosition(pos);
  return Vector3(pos[0]*1000,pos[1]*1000,pos[2]*1000) +
    m_position_f->evaluate( m_camera->EphemerisTime() );
}

Quaternion<double>
IsisAdjustCameraModel::camera_pose( Vector2 const& pix = Vector2() ) const {
  // Converting to ISIS index
  Vector2 px = pix + Vector2(1,1);

  if ( !m_camera->HasProjection() ) {
    // Standard Image (from Frame or LineScan)
    px[0] = m_alphacube->AlphaSample(px[0]);
    px[1] = m_alphacube->AlphaLine(px[1]);
    m_detectmap->SetParent( px[0], px[1] ); // Sets ET in LineScan
  } else {
    // Map Projected Image
    m_camera->SetImage( px[0], px[1] );
  }
  // Convert from instrument frame-> J2000 frame --> Globe centered-fixed
  std::vector<double> rot_inst = cam->InstrumentRotation()->Matrix();
  std::vector<double> rot_body = cam->BodyRotation()->Matrix();
  MatrixProxy<double,3,3> R_inst(&(rot_inst[0]));
  MatrixProxy<double,3,3> R_body(&(rot_body[0]));
  // Evaluating pose function
  Vector3 angles = m_pose_func->evaluate( m_camera->EphemerisTime() );

  Quaternion<double> quat ( R_inst * transpose(R_body) *
                            euler_to_rotation_matrix( angles[0],
                                                      angles[1],
                                                      angles[2],
                                                      "xyz" ) );
  quat = quat / norm_2( quat );
  return quat;
}

//-------------------------------------------------------------------------
//  Solver for projecting points into the camera
//-------------------------------------------------------------------------

// EphemerisLMA::operator()
//
// LMA for projecting point into modified linescan camera
IsisAdjustCameraModel::EphemerisLMA::result_type
IsisAdjustCameraModel::EphemerisLMA::operator()( IsisCameraModel::EphemerisLMA::domain_type const& x ) const {

  // Setting Ephemeris Time
  m_camera->SetEphemerisTime( x[0] );

  double ipos[3];
  m_camera->InstrumentPosition(ipos);
  Vector3 instru(ipos[0],ipos[1],ipos[2]);
  instru *= 1000;
  instru += m_position_f->evaluate( x[0] );

  Vector3 lookB = normalize(point-instru);
  Vector3 angles = m_pose_func->evaluate( x[0] );
  lookB = euler_to_rotation_matrix( angles[0], angles[1], angles[2], "xyz" ) * lookB;

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
  Vector3 angles = m_pose_func->evaluate( m_camera->EphemerisTime() );
  lookB = euler_to_rotation_matrix( angles[0], angles[1], angles[2], "xyz" ) * lookB;

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
