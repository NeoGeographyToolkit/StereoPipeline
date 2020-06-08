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


#include <vw/Core/Exception.h>
#include <vw/Math/Vector.h>
#include <asp/IsisIO/IsisInterface.h>
#include <asp/IsisIO/IsisInterfaceMapFrame.h>
#include <asp/IsisIO/IsisInterfaceFrame.h>
#include <asp/IsisIO/IsisInterfaceMapLineScan.h>
#include <asp/IsisIO/IsisInterfaceLineScan.h>
#include <boost/filesystem.hpp>

#include <iomanip>
#include <ostream>

#include <Cube.h>
#include <Distance.h>
#include <Pvl.h>
#include <Camera.h>
#include <Target.h>
#include <FileName.h>
#include <CameraFactory.h>
#include <SerialNumber.h>
#include <iTime.h>

using namespace vw;
using namespace asp;
using namespace asp::isis;

IsisInterface::IsisInterface( std::string const& file ) {
  // Opening labels and camera
  Isis::FileName ifilename( QString::fromStdString(file) );
  m_label.reset( new Isis::Pvl() );
  m_label->read( ifilename.expanded() );

  // Opening Isis::Camera
  m_cube.reset( new Isis::Cube(QString::fromStdString(file)) );
  m_camera.reset(Isis::CameraFactory::Create( *m_cube ));
}

IsisInterface::~IsisInterface() {}

IsisInterface* IsisInterface::open( std::string const& filename ) {
  // Opening Labels (This should be done somehow though labels)
  Isis::FileName ifilename( QString::fromStdString(filename) );
  Isis::Pvl label;
  label.read( ifilename.expanded() );

  Isis::Cube tempCube(QString::fromStdString(filename));
  Isis::Camera* camera = Isis::CameraFactory::Create( tempCube );

  IsisInterface* result;

  // Instantiate the correct class type
  switch ( camera->GetCameraType() ) {
  case 0:
    // Framing Camera
    if ( camera->HasProjection() )
      result = new IsisInterfaceMapFrame( filename );
    else
      result = new IsisInterfaceFrame( filename );
    break;
  case 2:
    // Linescan Camera
    if ( camera->HasProjection() )
      result = new IsisInterfaceMapLineScan( filename );
    else
      result = new IsisInterfaceLineScan( filename );
    break;
  default:
    vw_throw( NoImplErr() << "Don't support Isis Camera Type " << camera->GetCameraType() << " at this moment" );
  }

  return result;
}

int IsisInterface::lines() const {
  return m_camera->Lines();
}

int IsisInterface::samples() const {
  return m_camera->Samples();
}

std::string IsisInterface::serial_number() const {
  Isis::Pvl copy( *m_label );
  return Isis::SerialNumber::Compose( copy, true ).toStdString();
}

double IsisInterface::ephemeris_time( vw::Vector2 const& pix ) const {
  m_camera->SetImage( pix[0]+1, pix[1]+1 );
  return m_camera->time().Et();
}

vw::Vector3 IsisInterface::sun_position( vw::Vector2 const& pix ) const {
  m_camera->SetImage( pix[0]+1, pix[1]+1 );
  Vector3 sun;
  m_camera->sunPosition( &sun[0] );
  return sun * 1000;
}

vw::Vector3 IsisInterface::target_radii() const {
  Isis::Distance radii[3];
  m_camera->radii(radii);
  return Vector3( radii[0].meters(),
                  radii[1].meters(),
                  radii[2].meters() );
}

std::string IsisInterface::target_name() const {
  return m_camera->target()->name().toStdString();
}

std::ostream& asp::isis::operator<<( std::ostream& os, IsisInterface* i ) {
  os << "IsisInterface" << i->type()
       << "( Serial=" << i->serial_number()
       << std::setprecision(9)
       << ", f=" << i->m_camera->FocalLength()
       << " mm, pitch=" << i->m_camera->PixelPitch()
       << " mm/px," << std::setprecision(6)
       << "Center=" << i->camera_center() << " )";
    return os;
}

// Check if ISISROOT and ISISDATA was set
bool asp::isis::IsisEnv() {
  char * isisroot_ptr = getenv("ISISROOT");
  char * isisdata_ptr = getenv("ISISDATA");

  if (isisroot_ptr == NULL || isisdata_ptr == NULL ||
      std::string(isisroot_ptr) == "" ||
      std::string(isisdata_ptr) == "" ||
      !boost::filesystem::exists(std::string(isisroot_ptr) + "/IsisPreferences") )
    return false;
  return true;
}
