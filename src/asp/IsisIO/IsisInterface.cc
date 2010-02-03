// __BEGIN_LICENSE__
// Copyright (C) 2006-2009 United States Government as represented by
// the Administrator of the National Aeronautics and Space Administration.
// All Rights Reserved.
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
#include <Filename.h>
#include <CameraFactory.h>
#include <SerialNumber.h>

using namespace vw;
using namespace asp;
using namespace asp::isis;

IsisInterface::IsisInterface( std::string const& file ) {
  // Opening labels and camera
  Isis::Filename cubefile( file.c_str() );
  m_label.Read( cubefile.Expanded() );

  // Opening Isis::Camera
  m_camera = Isis::CameraFactory::Create( m_label );
}

IsisInterface::~IsisInterface(void) {
  if (m_camera)
    delete m_camera;
}

IsisInterface* IsisInterface::open( std::string const& filename ) {
  // Opening Labels (This should be done somehow though labels)
  Isis::Filename cubefile( filename.c_str() );
  Isis::Pvl label;
  label.Read( cubefile.Expanded() );

  Isis::Camera* camera = Isis::CameraFactory::Create( label );

  IsisInterface* result;

  switch ( camera->GetCameraType() ) {
  case 0:
    // Framing Camera
    if ( camera->HasProjection() )
      result = new IsisInterfaceMapFrame( filename );
    else
      return new IsisInterfaceFrame( filename );
  case 2:
    // Linescan Camera
    if ( camera->HasProjection() )
      return new IsisInterfaceMapLineScan( filename );
    else
      return new IsisInterfaceLineScan( filename );
  default:
    vw_throw( NoImplErr() << "Don't support Isis Camera Type " << camera->GetCameraType() << " at this moment" );
  }

  return result;
}

std::string IsisInterface::serial_number( void ) const {
  Isis::Pvl copy( m_label );
  return Isis::SerialNumber::Compose( copy, true );
}
