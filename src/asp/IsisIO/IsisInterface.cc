// __BEGIN_LICENSE__
//  Copyright (c) 2009-2012, United States Government as represented by the
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


// ASP
#include <asp/IsisIO/IsisInterface.h>
#include <asp/IsisIO/IsisInterfaceMapFrame.h>
#include <asp/IsisIO/IsisInterfaceFrame.h>
#include <asp/IsisIO/IsisInterfaceMapLineScan.h>
#include <asp/IsisIO/IsisInterfaceLineScan.h>

// VW
#include <vw/Core.h>

// ISIS
#include <FileName.h>
#include <CameraFactory.h>
#include <SerialNumber.h>
#include <iTime.h>

using namespace vw;
using namespace asp;
using namespace asp::isis;

IsisInterface::IsisInterface( std::string const& file ) {
  // Opening labels and camera
  Isis::FileName cubefile( file.c_str() );
  m_label.Read( cubefile.expanded() );

  // Opening Isis::Camera
  m_camera.reset(Isis::CameraFactory::Create( m_label ));
}

IsisInterface* IsisInterface::open( std::string const& filename ) {
  // Opening Labels (This should be done somehow though labels)
  Isis::FileName cubefile( filename.c_str() );
  Isis::Pvl label;
  label.Read( cubefile.expanded() );

  Isis::Camera* camera = Isis::CameraFactory::Create( label );

  IsisInterface* result;

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

std::string IsisInterface::serial_number() const {
  Isis::Pvl copy( m_label );
  return Isis::SerialNumber::Compose( copy, true );
}

double IsisInterface::ephemeris_time( vw::Vector2 const& pix ) const {
  m_camera->SetImage( pix[0]+1, pix[1]+1 );
  return m_camera->Time().Et();
}

vw::Vector3 IsisInterface::sun_position( vw::Vector2 const& pix ) const {
  m_camera->SetImage( pix[0]+1, pix[1]+1 );
  Vector3 sun;
  m_camera->SunPosition( &sun[0] );
  return sun * 1000;
}

vw::Vector3 IsisInterface::target_radii() const {
  Isis::Distance radii[3];
  m_camera->Radii(radii);
  return Vector3( radii[0].meters(),
                  radii[1].meters(),
                  radii[2].meters() );
}
