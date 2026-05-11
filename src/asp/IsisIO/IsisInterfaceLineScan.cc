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

#include <asp/IsisIO/IsisInterfaceLineScan.h>

#include <vw/Math/Matrix.h>
#include <vw/Camera/CameraModel.h>

// Isis headers
#include <Camera.h>
#include <CameraDetectorMap.h>
#include <CameraDistortionMap.h>
#include <CameraFocalPlaneMap.h>
#include <iTime.h>
#include <SurfacePoint.h>
#include <Latitude.h>
#include <Longitude.h>
#include <Angle.h>

#include <algorithm>
#include <vector>

using namespace vw;
using namespace asp;
using namespace asp::isis;

// Construct
IsisInterfaceLineScan::IsisInterfaceLineScan(std::string const& filename): IsisInterface(filename), m_alphacube(*m_cube) {

  // Gutting Isis::Camera
  m_distortmap = m_camera->DistortionMap();
  m_focalmap   = m_camera->FocalPlaneMap();
  m_detectmap  = m_camera->DetectorMap();
}

// Custom function to help avoid over invoking the deeply buried
// functions of Isis::Sensor
void IsisInterfaceLineScan::SetTime(Vector2 const& px, bool calc_pose) const {
  if (px != m_c_location) {
    m_c_location = px;
    m_detectmap->SetParent(m_alphacube.AlphaSample(px[0]),
                           m_alphacube.AlphaLine(px[1]));

    if (calc_pose) {
      // Calculating the spacecraft position and orientation (hence pose)
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


Vector2
IsisInterfaceLineScan::point_to_pixel(Vector3 const& point) const {

  // Delegate to Isis::Camera::SetGround. Internally that uses
  // LineScanCameraGroundMap with secant + quadratic fallback + cache-bound
  // check, which handles ill-conditioned cases (corner pixels on long
  // linescans, e.g. Chandrayaan-2 TMC at 190000 lines) where the prior
  // ASP-side custom secant could converge to a wrong root.
  Isis::SurfacePoint sp(
      Isis::Displacement(point[0], Isis::Displacement::Meters),
      Isis::Displacement(point[1], Isis::Displacement::Meters),
      Isis::Displacement(point[2], Isis::Displacement::Meters));
  if (!m_camera->SetGround(sp))
    vw_throw(vw::camera::PointToPixelErr()
             << " ISIS Camera::SetGround failed for point " << point);

  Vector2 pixel(m_camera->Sample(), m_camera->Line());
  pixel -= Vector2(1, 1); // ISIS 1-based -> ASP 0-based
  SetTime(pixel, false);  // refresh m_center cache
  return pixel;
}

Vector3
IsisInterfaceLineScan::pixel_to_vector(Vector2 const& pix) const {
  Vector2 px = pix + Vector2(1,1);
  SetTime(px, true);

  // Projecting to get look direction
  Vector3 result;
  m_focalmap->SetDetector(m_detectmap->DetectorSample(),
                          m_detectmap->DetectorLine());
  m_distortmap->SetFocalPlane(m_focalmap->FocalPlaneX(),
                              m_focalmap->FocalPlaneY());
  result[0] = m_distortmap->UndistortedFocalPlaneX();
  result[1] = m_distortmap->UndistortedFocalPlaneY();
  result[2] = m_distortmap->UndistortedFocalPlaneZ();
  result = normalize(result);
  result = m_pose.rotate(result);
  return result;
}

Vector3
IsisInterfaceLineScan::camera_center(Vector2 const& pix) const {
  Vector2 px = pix + Vector2(1,1);
  SetTime(px, true);
  return m_center;
}

Quat
IsisInterfaceLineScan::camera_pose(Vector2 const& pix) const {
  Vector2 px = pix + Vector2(1,1);
  SetTime(px, true);
  return m_pose;
}
