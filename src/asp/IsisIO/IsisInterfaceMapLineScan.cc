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
#include <vw/Math/Quaternion.h>
#include <vw/Math/Vector.h>
#include <vw/Camera/CameraModel.h>
#include <asp/IsisIO/IsisInterfaceMapLineScan.h>

#include <algorithm>
#include <limits>
#include <string>
#include <vector>

#include <Camera.h>
#include <CameraDistortionMap.h>
#include <CameraFocalPlaneMap.h>
#include <CameraGroundMap.h>
#include <Latitude.h>
#include <Longitude.h>
#include <Projection.h>
#include <ProjectionFactory.h>
#include <Sensor.h>
#include <SpiceRotation.h>
#include <iTime.h>


using namespace vw;
using namespace asp;
using namespace asp::isis;

// Constructor
IsisInterfaceMapLineScan::IsisInterfaceMapLineScan(std::string const& filename) :
  IsisInterface(filename){
  //,m_projection(Isis::ProjectionFactory::CreateFromCube(*m_label)) {

  Isis::TProjection* tempProj
    = (Isis::TProjection*)Isis::ProjectionFactory::CreateFromCube(*m_label);
  m_projection.reset(tempProj);

  // Gutting Isis::Camera
  m_distortmap = m_camera->DistortionMap();
  m_groundmap = m_camera->GroundMap();
  m_focalmap = m_camera->FocalPlaneMap();
  m_cache_px[0] = m_cache_px[1] = std::numeric_limits<double>::quiet_NaN();
}

Vector2 IsisInterfaceMapLineScan::point_to_pixel(Vector3 const& point) const {

  // Delegate to Isis::Camera::SetGround. Internally that uses
  // LineScanCameraGroundMap with secant + quadratic fallback + cache-bound
  // check. ASP's prior LMA loop here had the same wrong-root failure mode
  // as IsisInterfaceLineScan's secant on long along-track linescans.
  Isis::SurfacePoint sp(
      Isis::Displacement(point[0], Isis::Displacement::Meters),
      Isis::Displacement(point[1], Isis::Displacement::Meters),
      Isis::Displacement(point[2], Isis::Displacement::Meters));
  if (!m_camera->SetGround(sp))
    vw_throw(camera::PointToPixelErr()
             << " ISIS Camera::SetGround failed for point " << point);

  // Convert lat/lon at the ground point through the mapprojection to get
  // the mapprojected pixel (ISIS WorldX/Y is 1-based).
  m_projection->SetGround(m_camera->UniversalLatitude(),
                          m_camera->UniversalLongitude());
  m_cache_px = Vector2(m_projection->WorldX()-1,
                       m_projection->WorldY()-1);
  return m_cache_px;
}

Vector3 IsisInterfaceMapLineScan::camera_center(Vector2 const& px) const {
  if (px != m_cache_px) {
    m_cache_px = px;
    if (!m_projection->SetWorld(px[0]+1,
                                 px[1]+1))
      vw_throw(camera::PixelToRayErr() << "Failed to SetWorld.");
    if (!m_groundmap->SetGround(Isis::Latitude(m_projection->UniversalLatitude(),
                                               Isis::Angle::Degrees),
                                Isis::Longitude(m_projection->UniversalLongitude(),
                                                Isis::Angle::Degrees)))
      vw_throw(camera::PixelToRayErr() << "Failed to SetGround.");
  }
  Vector3 position;
  m_camera->instrumentPosition(&position[0]);
  return position * 1e3;
}

Vector3 IsisInterfaceMapLineScan::pixel_to_vector(Vector2 const& px) const {
  Vector3 sB = camera_center(px);
  Vector3 p_pB;
  m_camera->Sensor::Coordinate(&p_pB[0]);
  return normalize(p_pB*1000 - sB);
}

Quat IsisInterfaceMapLineScan::camera_pose(Vector2 const& px) const {
  camera_center(px);
  std::vector<double> rot_inst = m_camera->instrumentRotation()->Matrix();
  std::vector<double> rot_body = m_camera->bodyRotation()->Matrix();
  MatrixProxy<double,3,3> R_inst(&(rot_inst[0]));
  MatrixProxy<double,3,3> R_body(&(rot_body[0]));
  return Quat(R_body*transpose(R_inst));
}
