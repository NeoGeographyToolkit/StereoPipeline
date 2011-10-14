// __BEGIN_LICENSE__
// Copyright (C) 2006-2011 United States Government as represented by
// the Administrator of the National Aeronautics and Space Administration.
// All Rights Reserved.
// __END_LICENSE__


/// \file IsisInterfaceMapFrame.h
///
/// Map Projected Frame Camera Interface with ISIS
///
#ifndef __ASP_ISIS_INTERFACE_MAP_FRAME_H__
#define __ASP_ISIS_INTERFACE_MAP_FRAME_H__

// ASP
#include <asp/IsisIO/IsisInterface.h>

// Isis forward declaration
class Isis::CameraGroundMap;
class Isis::CameraDistortionMap;
#include <Distance.h>

namespace asp {
namespace isis {

  class IsisInterfaceMapFrame : public IsisInterface {

  public:
    IsisInterfaceMapFrame( std::string const& file );

    virtual std::string type()  { return "MapFrame"; }

    // Standard Methods
    //-------------------------------------------------

    virtual vw::Vector2
      point_to_pixel( vw::Vector3 const& point ) const;
    virtual vw::Vector3
      pixel_to_vector( vw::Vector2 const& pix ) const;
    virtual vw::Vector3
      camera_center( vw::Vector2 const& pix = vw::Vector2() ) const;
    virtual vw::Quat
      camera_pose( vw::Vector2 const& pix = vw::Vector2() ) const;

  protected:

    // Custom Variables
    boost::scoped_ptr<Isis::Projection> m_projection;
    Isis::CameraGroundMap *m_groundmap;
    Isis::CameraDistortionMap *m_distortmap;

    vw::Vector3 m_center;
    vw::Quat m_pose;
    Isis::Distance m_radii[3];
  };

}}

#endif//__ASP_ISIS_INTERFACE_MAP_FRAME_H__
