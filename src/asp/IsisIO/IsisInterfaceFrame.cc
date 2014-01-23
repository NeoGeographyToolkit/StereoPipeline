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


#include <vw/Math/Matrix.h>
#include <asp/IsisIO/IsisInterfaceFrame.h>

#include <algorithm>
#include <vector>

#include <Camera.h>
#include <CameraDetectorMap.h>
#include <CameraDistortionMap.h>
#include <CameraFocalPlaneMap.h>

using namespace vw;
using namespace asp;
using namespace asp::isis;

// Constructor
IsisInterfaceFrame::IsisInterfaceFrame( std::string const& filename ) :
  IsisInterface(filename), m_alphacube( *m_cube ) {

  // Gutting Isis::Camera
  m_distortmap = m_camera->DistortionMap();
  m_focalmap   = m_camera->FocalPlaneMap();
  m_detectmap  = m_camera->DetectorMap();

  // Calculating Center (just once)
  m_camera->instrumentPosition(&m_center[0]);
  m_center *= 1000;

  // Calculating Pose (just once)
  std::vector<double> rot_inst = m_camera->instrumentRotation()->Matrix();
  std::vector<double> rot_body = m_camera->bodyRotation()->Matrix();
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
  lookB_copy = m_camera->bodyRotation()->J2000Vector(lookB_copy);
  lookB_copy = m_camera->instrumentRotation()->ReferenceVector(lookB_copy);
  std::copy( lookB_copy.begin(), lookB_copy.end(), look.begin() );
  look = m_camera->FocalLength() * ( look / std::abs(look[2]) );

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
  look = m_camera->instrumentRotation()->J2000Vector(look);
  look = m_camera->bodyRotation()->ReferenceVector(look);
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
