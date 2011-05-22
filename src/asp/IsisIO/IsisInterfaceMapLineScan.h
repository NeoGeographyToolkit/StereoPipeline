// __BEGIN_LICENSE__
// Copyright (C) 2006-2010 United States Government as represented by
// the Administrator of the National Aeronautics and Space Administration.
// All Rights Reserved.
// __END_LICENSE__


/// \file IsisInterfaceMapLineScane.h
///
/// Map Projected Line Scan Camera Interface with ISIS
///
#ifndef __ASP_ISIS_INTERFACE_MAP_LINESCAN_H__
#define __ASP_ISIS_INTERFACE_MAP_LINESCAN_H__

// ASP & VW
#include <asp/IsisIO/IsisInterface.h>

// Isis
#include <Projection.h>
#include <CameraDistortionMap.h>
#include <CameraGroundMap.h>
#include <CameraFocalPlaneMap.h>

namespace asp {
namespace isis {

  class IsisInterfaceMapLineScan : public IsisInterface {

  public:
    IsisInterfaceMapLineScan( std::string const& file );

    virtual std::string type()  { return "MapLineScan"; }

    // Standard Methods
    //-------------------------------------------------

    virtual vw::Vector2
      point_to_pixel( vw::Vector3 const& point ) const;
    virtual vw::Vector3
      pixel_to_vector( vw::Vector2 const& pix ) const;
    virtual vw::Vector3
      camera_center( vw::Vector2 const& pix = vw::Vector2(1,1) ) const;
    virtual vw::Quat
      camera_pose( vw::Vector2 const& pix = vw::Vector2(1,1) ) const;

  protected:

    // Custom Variables
    boost::scoped_ptr<Isis::Projection> m_projection;
    Isis::CameraDistortionMap *m_distortmap;
    Isis::CameraGroundMap     *m_groundmap;
    Isis::CameraFocalPlaneMap *m_focalmap;

  };

}}

#endif//__ASP_ISIS_INTERFACE_MAP_LINESCAN_H__
