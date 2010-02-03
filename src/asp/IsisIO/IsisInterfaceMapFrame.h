// __BEGIN_LICENSE__
// Copyright (C) 2006-2009 United States Government as represented by
// the Administrator of the National Aeronautics and Space Administration.
// All Rights Reserved.
// __END_LICENSE__

/// \file IsisInterfaceMapFrame.h
///
/// Map Projected Frame Camera Interface with ISIS
///
#ifndef __ASP_ISIS_INTERFACE_MAP_FRAME_H__
#define __ASP_ISIS_INTERFACE_MAP_FRAME_H__

namespace asp {
namespace isis {

  class IsisInterfaceMapFrame : public IsisInterface {

  public:
    IsisInterfaceMapFrame( std::string const& file ) :
      IsisInterface(file) {}

    virtual ~IsisInterfaceMapFrame() {}

    virtual std::string type()  { return "MapFrame"; }

    // Standard Methods
    //-------------------------------------------------

    virtual vw::Vector2
      point_to_pixel( vw::Vector3 const& point ) const {
      vw::vw_throw( vw::NoImplErr() << "MapFrame unfinished" );
      return vw::Vector2();
    }
    virtual vw::Vector3
      pixel_to_vector( vw::Vector2 const& pix ) const {
      vw::vw_throw( vw::NoImplErr() << "MapFrame unfinished" );
      return vw::Vector3();
    }
    virtual vw::Vector3
      camera_center( vw::Vector2 const& pix = vw::Vector2(1,1) ) const {
      vw::vw_throw( vw::NoImplErr() << "MapFrame unfinished" );
      return vw::Vector3();
    }
    virtual vw::Quaternion<double>
      camera_pose( vw::Vector2 const& pix = vw::Vector2(1,1) ) const {
      vw::vw_throw( vw::NoImplErr() << "MapFrame unfinished" );
      return vw::Quaternion<double>();
    }

  };

}}

#endif//__ASP_ISIS_INTERFACE_MAP_FRAME_H__
