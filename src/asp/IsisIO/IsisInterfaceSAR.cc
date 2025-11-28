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
#include <vw/Camera/CameraModel.h>
#include <vw/Core/Exception.h>

#include <asp/IsisIO/IsisInterfaceSAR.h>
#include <vw/Cartography/PointImageManipulation.h>

// ISIS 
#include <Camera.h>
#include <CameraDetectorMap.h>
#include <CameraDistortionMap.h>
#include <CameraGroundMap.h>
#include <CameraFocalPlaneMap.h>
#include <Distance.h>
#include <iTime.h>
#include <Target.h>
#include <SurfacePoint.h>
#include <Latitude.h>
#include <Longitude.h>
#include <Angle.h>

#include <algorithm>
#include <vector>

using namespace vw;
using namespace asp;
using namespace asp::isis;

// Constructor
IsisInterfaceSAR::IsisInterfaceSAR(std::string const& filename):
  IsisInterface(filename), m_alphacube(*m_cube) {

  // Gutting Isis::Camera
  m_groundmap  = m_camera->GroundMap();
  m_distortmap = m_camera->DistortionMap();
  m_focalmap   = m_camera->FocalPlaneMap();
  m_detectmap  = m_camera->DetectorMap();

  // Set the shape to be the ellipsoid.  This will ensure that ground
  // points will be on the ellipsoid, which is needed in
  // pixel_to_vector().
  m_camera->IgnoreElevationModel(true);
}

Vector2 IsisInterfaceSAR::point_to_pixel(Vector3 const& point) const {

  // TODO(oalexan1): It looks that the code below assumes the datum to be spherical.
  Vector3 lon_lat_radius = cartography::xyz_to_lon_lat_radius_estimate(point); // TODO: INACCURATE!!!
  if (lon_lat_radius[0] < 0)
    lon_lat_radius[0] += 360.0;

  // Project into the camera. The ground was set to be the ellipsoid in the constructor.
  if (!m_groundmap->SetGround
      (Isis::SurfacePoint(Isis::Latitude (lon_lat_radius[1], Isis::Angle::Degrees  ),
                          Isis::Longitude(lon_lat_radius[0], Isis::Angle::Degrees  ),
                          Isis::Distance (lon_lat_radius[2], Isis::Distance::Meters))))
    vw_throw(camera::PixelToRayErr() << "Failed in SetGround().");
      
  m_distortmap->SetUndistortedFocalPlane(m_groundmap->FocalPlaneX(), m_groundmap->FocalPlaneY());

  // ISIS pixels start with 1.
  return Vector2(m_camera->Sample() - 1.0, m_camera->Line() - 1.0);
}

// TODO(oalexan1): There should be a better way of doing this
Vector3 IsisInterfaceSAR::pixel_to_vector(Vector2 const& pix) const {

  // Find the camera center
  Vector3 cam_ctr = camera_center(pix);

  // Find the intersection with the ground. The ground is defined
  // in the constructor to be the ellipsoid. The camera center also
  // sets the image pixel, so the intersection with ground is computed
  // already, need to just fetch it.
  Vector3 llh(m_camera->UniversalLongitude(), m_camera->UniversalLatitude(), 0.0);
  Vector3 ground_pt = m_datum.geodetic_to_cartesian(llh);

  // The desired vector is the normalized direction from the camera
  // center to the ground
  Vector3 dir = ground_pt - cam_ctr;
  return dir / norm_2(dir);
}

Vector3 IsisInterfaceSAR::camera_center(Vector2 const& pix) const {

  Vector2 isis_pix = pix + Vector2(1,1);

  if (!m_camera->SetImage(isis_pix[0], isis_pix[1]))
    vw_throw(camera::PixelToRayErr() << "Failed in SetImage().");

  vw::Vector3 center;
  m_camera->instrumentPosition(&center[0]);
  center *= 1000.0; // km to meters
  
  return center;
}

Quat IsisInterfaceSAR::camera_pose(Vector2 const& pix) const {
  vw_throw(NoImplErr() << "camera_pose() not implemented for ISIS SAR cameras.");

  return vw::Quat();
}
