// __BEGIN_LICENSE__
// Copyright (C) 2006-2010 United States Government as represented by
// the Administrator of the National Aeronautics and Space Administration.
// All Rights Reserved.
// __END_LICENSE__


/// \file IsisInterfaceFrame.h
///
/// Frame Camera Interface with ISIS
///
#ifndef __ASP_ISIS_INTERFACE_FRAME_H__
#define __ASP_ISIS_INTERFACE_FRAME_H__

// ASP
#include <asp/IsisIO/IsisInterface.h>

// Isis
#include <CameraDetectorMap.h>
#include <CameraDistortionMap.h>
#include <CameraFocalPlaneMap.h>
#include <AlphaCube.h>

namespace asp {
namespace isis {

  class IsisInterfaceFrame : public IsisInterface {

  public:
    IsisInterfaceFrame( std::string const& filename );

    virtual ~IsisInterfaceFrame() {
      if ( m_alphacube )
        delete m_alphacube;
    }

    virtual std::string type()  { return "Frame"; }

    // Standard Methods
    //-------------------------------------------------

    virtual vw::Vector2
      point_to_pixel( vw::Vector3 const& point ) const;
    virtual vw::Vector3
      pixel_to_vector( vw::Vector2 const& pix ) const;
    virtual vw::Vector3
      camera_center( vw::Vector2 const& pix = vw::Vector2(1,1) ) const;
    virtual vw::Quaternion<double>
      camera_pose( vw::Vector2 const& pix = vw::Vector2(1,1) ) const;

  protected:

    // Custom Variables
    Isis::CameraDistortionMap *m_distortmap;
    Isis::CameraFocalPlaneMap *m_focalmap;
    Isis::CameraDetectorMap   *m_detectmap;
    Isis::AlphaCube           *m_alphacube;

    vw::Vector3 m_center;
    vw::Quaternion<double> m_pose;
  };

}}

#endif//__ASP_ISIS_INTERFACE_FRAME_H__
