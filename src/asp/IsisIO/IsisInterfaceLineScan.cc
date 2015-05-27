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

#include <vw/Math/LevenbergMarquardt.h>
#include <vw/Math/Matrix.h>
#include <vw/Camera/CameraModel.h>
#include <asp/IsisIO/IsisInterfaceLineScan.h>

#include <algorithm>
#include <vector>

#include <Camera.h>
#include <CameraDetectorMap.h>
#include <CameraDistortionMap.h>
#include <CameraFocalPlaneMap.h>
#include <iTime.h>

#include <boost/smart_ptr/scoped_ptr.hpp>

using namespace vw;
using namespace asp;
using namespace asp::isis;

// Construct
IsisInterfaceLineScan::IsisInterfaceLineScan( std::string const& filename ) : IsisInterface(filename), m_alphacube( *m_cube ) {

  // Gutting Isis::Camera
  m_distortmap = m_camera->DistortionMap();
  m_focalmap   = m_camera->FocalPlaneMap();
  m_detectmap  = m_camera->DetectorMap();
}

// Custom Function to help avoid over invoking the deeply buried
// functions of Isis::Sensor
void IsisInterfaceLineScan::SetTime( Vector2 const& px, bool calc ) const {
  if ( px != m_c_location ) {
    m_c_location = px;
    m_detectmap->SetParent( m_alphacube.AlphaSample(px[0]),
                            m_alphacube.AlphaLine(px[1]) );

    if ( calc ) {
      // Calculating Spacecraft position and pose
      m_camera->instrumentPosition(&m_center[0]);
      m_center *= 1000;

      std::vector<double> rot_inst = m_camera->instrumentRotation()->Matrix();
      std::vector<double> rot_body = m_camera->bodyRotation()->Matrix();
      MatrixProxy<double,3,3> R_inst(&(rot_inst[0]));
      MatrixProxy<double,3,3> R_body(&(rot_body[0]));
      m_pose = Quat(R_body*transpose(R_inst));
    }
  }
}

class EphemerisLMA : public vw::math::LeastSquaresModelBase<EphemerisLMA> {
  vw::Vector3 m_point;
  Isis::Camera* m_camera;
  Isis::CameraDistortionMap *m_distortmap;
  Isis::CameraFocalPlaneMap *m_focalmap;
public:
  typedef vw::Vector<double> result_type; // Back project result
  typedef vw::Vector<double> domain_type; // Ephemeris time
  typedef vw::Matrix<double> jacobian_type;

  inline EphemerisLMA( vw::Vector3 const& point,
                       Isis::Camera* camera,
                       Isis::CameraDistortionMap* distortmap,
                       Isis::CameraFocalPlaneMap* focalmap ) : m_point(point), m_camera(camera), m_distortmap(distortmap), m_focalmap(focalmap) {}

  inline result_type operator()( domain_type const& x ) const;
};


// LMA for projecting point to linescan camera
EphemerisLMA::result_type
EphemerisLMA::operator()( EphemerisLMA::domain_type const& x ) const {

  // Setting Ephemeris Time
  m_camera->setTime(Isis::iTime(x[0]));

  // Calculating the look direction in camera frame
  Vector3 instru;
  m_camera->instrumentPosition(&instru[0]);
  instru *= 1000;  // Spice gives in km
  Vector3 lookB = normalize( m_point - instru );
  std::vector<double> lookB_copy(3);
  std::copy( lookB.begin(), lookB.end(), lookB_copy.begin() );
  std::vector<double> lookJ = m_camera->bodyRotation()->J2000Vector(lookB_copy);
  std::vector<double> lookC = m_camera->instrumentRotation()->ReferenceVector(lookJ);
  Vector3 look;
  std::copy( lookC.begin(), lookC.end(), look.begin() );

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
IsisInterfaceLineScan::point_to_pixel( Vector3 const& point ) const {

  // First seed LMA with an ephemeris time in the middle of the image
  double middle = lines() / 2;
  m_detectmap->SetParent( 1, m_alphacube.AlphaLine(middle) );
  double start_e = m_camera->time().Et();

  // Build LMA
  EphemerisLMA model( point, m_camera.get(), m_distortmap, m_focalmap );
  int status;
  Vector<double> objective(1), start(1);
  start[0] = start_e;
  Vector<double> solution_e = math::levenberg_marquardt( model,
                                                         start,
                                                         objective,
                                                         status );

  // Make sure we found ideal time
  VW_ASSERT( status > 0, vw::camera::PointToPixelErr() << " Unable to project point into ISIS linescan camera " );

  // Converting now to pixel
  m_camera->setTime(Isis::iTime( solution_e[0] ));

  // Working out pointing
  m_camera->instrumentPosition(&m_center[0]);
  m_center *= 1000;
  Vector3 look = normalize(point-m_center);

  // Calculating Rotation to camera frame
  std::vector<double> rot_inst = m_camera->instrumentRotation()->Matrix();
  std::vector<double> rot_body = m_camera->bodyRotation()->Matrix();
  MatrixProxy<double,3,3> R_inst(&(rot_inst[0]));
  MatrixProxy<double,3,3> R_body(&(rot_body[0]));
  m_pose = Quat(R_body*transpose(R_inst));

  look = inverse(m_pose).rotate( look );
  look = m_camera->FocalLength() * ( look / look[2] );
  m_distortmap->SetUndistortedFocalPlane( look[0], look[1] );
  m_focalmap->SetFocalPlane( m_distortmap->FocalPlaneX(),
                             m_distortmap->FocalPlaneY() );
  m_detectmap->SetDetector( m_focalmap->DetectorSample(),
                            m_focalmap->DetectorLine() );
  Vector2 pixel( m_detectmap->ParentSample(),
                 m_detectmap->ParentLine() );
  pixel[0] = m_alphacube.BetaSample( pixel[0] );
  pixel[1] = m_alphacube.BetaLine( pixel[1] );
  SetTime( pixel, false );

  pixel -= Vector2(1,1);
  return pixel;
}

Vector3
IsisInterfaceLineScan::pixel_to_vector( Vector2 const& pix ) const {
  Vector2 px = pix + Vector2(1,1);
  SetTime( px, true );

  // Projecting to get look direction
  Vector3 result;
  m_focalmap->SetDetector( m_detectmap->DetectorSample(),
                           m_detectmap->DetectorLine() );
  m_distortmap->SetFocalPlane( m_focalmap->FocalPlaneX(),
                               m_focalmap->FocalPlaneY() );
  result[0] = m_distortmap->UndistortedFocalPlaneX();
  result[1] = m_distortmap->UndistortedFocalPlaneY();
  result[2] = m_distortmap->UndistortedFocalPlaneZ();
  result = normalize( result );
  result = m_pose.rotate(result);
  return result;
}

Vector3
IsisInterfaceLineScan::camera_center( Vector2 const& pix ) const {
  Vector2 px = pix + Vector2(1,1);
  SetTime( px, true );
  return m_center;
}

Quat
IsisInterfaceLineScan::camera_pose( Vector2 const& pix ) const {
  Vector2 px = pix + Vector2(1,1);
  SetTime( px, true );
  return m_pose;
}
