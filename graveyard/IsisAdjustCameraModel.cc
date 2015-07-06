// __BEGIN_LICENSE__
//  Copyright (c) 2009-2013, United States Government as represented by the
//  Administrator of the National Aeronautics and Space Administration. All
//  rights reserved.
//
//  The NGT platform is licensed under the Apache License, Version 2.0 (the
//  "License"); you may not use this file except in compliance with the
//  License. You may obtain a copy of the License at
//  http://www.apache.org/licenses/LICENSE-2.0
//
//  Unless required by applicable law or agreed to in writing, software
//  distributed under the License is distributed on an "AS IS" BASIS,
//  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
//  See the License for the specific language governing permissions and
//  limitations under the License.
// __END_LICENSE__

#include <vw/Math/Quaternion.h>
#include <vw/Math/Vector.h>
#include <vw/Math/Matrix.h>
#include <vw/Math/LevenbergMarquardt.h>
#include <vw/Camera/CameraModel.h>
#include <asp/IsisIO/IsisAdjustCameraModel.h>
#include <asp/IsisIO/BaseEquation.h>

#include <vector>

#include <FileName.h>
#include <CameraFactory.h>
#include <Cube.h>
#include <SerialNumber.h>
#include <iTime.h>
#include <AlphaCube.h>                  // for AlphaCube
#include <Camera.h>                     // for Camera
#include <CameraDetectorMap.h>          // for CameraDetectorMap
#include <CameraDistortionMap.h>        // for CameraDistortionMap
#include <CameraFocalPlaneMap.h>        // for CameraFocalPlaneMap
#include <Pvl.h>                        // for Pvl
#include <SpiceRotation.h>              // for SpiceRotation

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
  Isis::FileName cubefile( cube_filename.c_str() );
  m_label.read( cubefile.expanded() );
  Isis::Cube tempCube(cubefile.expanded());
  m_camera = boost::shared_ptr<Isis::Camera>(Isis::CameraFactory::Create( tempCube ));

  // Gutting Camera
  m_distortmap = m_camera->DistortionMap();
  m_focalmap   = m_camera->FocalPlaneMap();
  m_detectmap  = m_camera->DetectorMap();
  m_alphacube  = boost::shared_ptr<Isis::AlphaCube>( new Isis::AlphaCube( tempCube ) );

  // Throw error if this is map projected image
  if ( m_camera->HasProjection() )
    vw_throw( NoImplErr() << "Don't support map projected images" );

  // Adjusting time offset so equations are reference from middle of cache
  double middle_et = m_camera->cacheStartTime().Et() + (m_camera->cacheEndTime().Et()-m_camera->cacheStartTime().Et())/2.0;
  m_position_f->set_time_offset( middle_et );
  m_pose_f->set_time_offset( middle_et );
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
      m_camera->instrumentPosition(&m_center[0]);
      m_center *= 1000;

      std::vector<double> rot_inst = m_camera->instrumentRotation()->Matrix();
      std::vector<double> rot_body = m_camera->bodyRotation()->Matrix();
      MatrixProxy<double,3,3> R_inst(&(rot_inst[0]));
      MatrixProxy<double,3,3> R_body(&(rot_body[0]));
      m_pose = Quat( R_inst*transpose(R_body) );
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
    double start_e = m_camera->cacheStartTime().Et() + (m_camera->cacheEndTime().Et()-m_camera->cacheStartTime().Et())/2.0;

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
               vw::camera::PointToPixelErr() << " Unable to project point into linescan camera " );

    // Converting now to pixel
    m_camera->setTime( Isis::iTime( solution_e[0] ) );
  } else if ( m_camera->GetCameraType() != 0 ) {
    vw_throw( NoImplErr() << "IsisAdjustCameraModel::point_to_pixel does not support any cameras other than LineScane and Frame" );
  }

  // Pulling out camera position for current time
  m_camera->instrumentPosition(&m_center[0]);
  m_center *= 1000;

  // Pulling out camera pose
  std::vector<double> rot_inst = m_camera->instrumentRotation()->Matrix();
  std::vector<double> rot_body = m_camera->bodyRotation()->Matrix();
  MatrixProxy<double,3,3> R_inst(&(rot_inst[0]));
  MatrixProxy<double,3,3> R_body(&(rot_body[0]));
  m_pose = Quat(R_inst*transpose(R_body));
  Vector3 angles = m_pose_f->evaluate( m_camera->time().Et() );
  Quat pose_adj( m_pose*math::axis_angle_to_quaternion( angles ) );

  // Actually projecting point now
  Vector3 look = normalize( point - (m_center+m_position_f->evaluate(m_camera->time().Et())) );
  look = pose_adj.rotate( look );
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
  Vector3 angles = m_pose_f->evaluate( m_camera->time().Et() );
  result = inverse( m_pose*math::axis_angle_to_quaternion(angles) ).rotate( result );
  return result;
}

Vector3
IsisAdjustCameraModel::camera_center( Vector2 const& pix ) const {
  // Converting to ISIS index
  Vector2 px = pix + Vector2(1,1);
  SetTime( px, true );
  return m_center+m_position_f->evaluate( m_camera->time().Et() );
}

Quat
IsisAdjustCameraModel::camera_pose( Vector2 const& pix ) const {
  // Converting to ISIS index
  Vector2 px = pix + Vector2(1,1);
  SetTime( px, true );
  Vector3 angles = m_pose_f->evaluate( m_camera->time().Et() );
  return inverse(m_pose*math::axis_angle_to_quaternion( angles ) );
}

std::string IsisAdjustCameraModel::serial_number() const {
  Isis::Pvl copy( m_label );
  return Isis::SerialNumber::Compose( copy, true ).toStdString();
}

double IsisAdjustCameraModel::ephemeris_time( Vector2 const& pix ) const {
  SetTime( pix+Vector2(1,1), false );
  return m_camera->time().Et();
}

Vector3 IsisAdjustCameraModel::sun_position( Vector2 const& pix ) const {
  SetTime( pix+Vector2(1,1), false );
  Vector3 sun;
  m_camera->sunPosition( &sun[0] );
  sun *= 1000;
  return sun;
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
  m_camera->setTime( Isis::iTime( x[0] ) );

  Vector3 instru;
  m_camera->instrumentPosition(&instru[0]);
  instru *= 1000;
  instru += m_position_f->evaluate( x[0] );

  Vector3 lookB = normalize( m_point - instru );
  Vector3 angles = m_pose_f->evaluate( x[0] );
  lookB = math::axis_angle_to_matrix( angles ) * lookB;

  std::vector<double> lookB_copy(3);
  lookB_copy[0] = lookB[0];
  lookB_copy[1] = lookB[1];
  lookB_copy[2] = lookB[2];
  std::vector<double> lookJ = m_camera->bodyRotation()->J2000Vector(lookB_copy);
  std::vector<double> lookC = m_camera->instrumentRotation()->ReferenceVector(lookJ);
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
