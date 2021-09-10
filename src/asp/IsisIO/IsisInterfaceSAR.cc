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
#include <vw/Core/Exception.h>

#include <asp/IsisIO/IsisInterfaceSAR.h>

// ISIS 
#include <Camera.h>
#include <CameraDetectorMap.h>
#include <CameraDistortionMap.h>
#include <CameraFocalPlaneMap.h>
#include <Distance.h>
#include <iTime.h>
#include <Target.h>
#include <SurfacePoint.h>
#include <Latitude.h>
#include <Longitude.h>

#include <boost/smart_ptr/scoped_ptr.hpp>

#include <algorithm>
#include <vector>

using namespace vw;
using namespace asp;
using namespace asp::isis;

// Construct
IsisInterfaceSAR::IsisInterfaceSAR(std::string const& filename):
  IsisInterface(filename), m_alphacube(*m_cube) {

  // Gutting Isis::Camera
  m_distortmap = m_camera->DistortionMap();
  m_focalmap   = m_camera->FocalPlaneMap();
  m_detectmap  = m_camera->DetectorMap();

  // TODO(oalexan1): All this is fragile. Need to find the right internal ISIS
  // function to use to convert ECEF to lon-lat-height and vice-versa.
  Isis::Distance radii[3];
  m_camera->radii(radii);
  // Average the x and y axes (semi-major) 
  double radius1 = (radii[0].meters() + radii[1].meters()) / 2;
  double radius2 = radii[2].meters(); // the z radius (semi-minor axis)
  std::string target_name = m_camera->target()->name().toStdString();
  m_datum = vw::cartography::Datum("D_" + target_name, target_name,
                                   "Reference Meridian", radius1, radius2, 0);

  std::cout << "--datum is " << m_datum << std::endl;
  //std::cout << "---done with IsisInterfaceSAR loader, return this: " << (void*)this << std::endl;
}

// Custom Function to help avoid over invoking the deeply buried
// functions of Isis::Sensor. Here it is assumed that (1, 1)
// was already added to px.
void IsisInterfaceSAR::SetTime(Vector2 const& px, bool calc_pose) const {
  //std::cout << "---fix here!" << std::endl;

  if (!m_camera->SetImage(px[0], px[1])) {
    vw_throw( camera::PixelToRayErr() << "Failed in SetImage()." );
    return;
  }

  // TODO(oalexan1): Either this or the one below is not necessary
  m_detectmap->SetParent(m_alphacube.AlphaSample(px[0]),
                         m_alphacube.AlphaLine(px[1]));

  
  //std::cout << "--isis lat " << m_camera->UniversalLatitude() << std::endl;
  //std::cout << "isis lon " << m_camera->UniversalLongitude() << std::endl;
  
//   if (px != m_c_location || calc_pose) {
//     m_c_location = px;
//     m_detectmap->SetParent(m_alphacube.AlphaSample(px[0]),
//                             m_alphacube.AlphaLine(px[1]));

  if (calc_pose) {
    // Calculating the spacecraft position and orientation (hence pose)
    m_camera->instrumentPosition(&m_center[0]);
    m_center *= 1000;
    
    //std::cout << "--location is " << px << std::endl;
    //std::cout << "--isis ctr " << m_center << std::endl;
    
    std::vector<double> rot_inst = m_camera->instrumentRotation()->Matrix();
    std::vector<double> rot_body = m_camera->bodyRotation()->Matrix();
    MatrixProxy<double,3,3> R_inst(&(rot_inst[0]));
    MatrixProxy<double,3,3> R_body(&(rot_body[0]));
    m_pose = Quat(R_body*transpose(R_inst));
  }

  //std::cout << "--pose is " << m_pose << std::endl;
}


Vector2 IsisInterfaceSAR::point_to_pixel(Vector3 const& point) const {

  // TODO(oalexan1): Find the ISIS function for going from ECEF to llh.
  Vector3 llh = m_datum.cartesian_to_geodetic(point);
  if (llh[0] < 0)
    llh[0] += 360.0;

  Isis::SurfacePoint surfPt(Isis::Latitude (llh[1], Isis::Angle::Degrees),
                            Isis::Longitude(llh[0], Isis::Angle::Degrees),
                            Isis::Distance (llh[2], Isis::Distance::Meters));
  
  if (m_camera->SetGround(surfPt))
    vw_throw(camera::PixelToRayErr() << "Failed in SetGround().");

  return Vector2(m_camera->Sample() - 1.0, m_camera->Line() - 1.0);
}

Vector3 IsisInterfaceSAR::pixel_to_vector(Vector2 const& pix) const {

  std::cout << "--Fix here 4!" << std::endl;
  Vector2 isis_pix = pix + Vector2(1,1);

  bool calc_pose = true;
  SetTime(isis_pix, calc_pose);

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
  
#if 0
  
  Vector2 isis_pix = pix + Vector2(1, 1);

  if (!m_camera->SetImage(isis_pix[0], isis_pix[1]))
    vw_throw(camera::PixelToRayErr() << "Failed in SetImage().");

  double look[3];
  m_camera->Sensor::LookDirection(look);

  std::cout << "--look " << look[0] << ' ' << look[1] << ' ' << look[2] << std::endl;
  
  Vector3 result;
  for (size_t it = 0; it < 3; it++) 
    result[it] = look[it];

  return result;
#endif
  
}

Vector3 IsisInterfaceSAR::camera_center(Vector2 const& pix) const {

  Vector2 isis_pix = pix + Vector2(1,1);

  if (!m_camera->SetImage(isis_pix[0], isis_pix[1]))
    vw_throw(camera::PixelToRayErr() << "Failed in SetImage().");

  m_camera->instrumentPosition(&m_center[0]);
  m_center *= 1000;
  
  return m_center;
}

Quat IsisInterfaceSAR::camera_pose(Vector2 const& pix) const {
  vw_throw(NoImplErr() << "camera_pose() not implemented for ISIS SAR cameras.");
  return m_pose;
}
