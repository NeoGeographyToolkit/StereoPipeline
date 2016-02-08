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


/// \file IsisInterfaceMapLineScane.h
///
/// Map Projected Line Scan Camera Interface with ISIS
///
#ifndef __ASP_ISIS_INTERFACE_MAP_LINESCAN_H__
#define __ASP_ISIS_INTERFACE_MAP_LINESCAN_H__

// ASP & VW
#include <asp/IsisIO/IsisInterface.h>

// Isis
#include <TProjection.h>
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

    virtual vw::Vector2 point_to_pixel ( vw::Vector3 const& point ) const;
    virtual vw::Vector3 pixel_to_vector( vw::Vector2 const& pix   ) const;
    virtual vw::Vector3 camera_center  ( vw::Vector2 const& pix = vw::Vector2(1,1) ) const;
    virtual vw::Quat    camera_pose    ( vw::Vector2 const& pix = vw::Vector2(1,1) ) const;

  protected:

    // Custom Variables
    mutable vw::Vector2 m_cache_px;
    boost::scoped_ptr<Isis::TProjection> m_projection;
    Isis::CameraDistortionMap *m_distortmap;
    Isis::CameraGroundMap     *m_groundmap;
    Isis::CameraFocalPlaneMap *m_focalmap;

  };

}}

#endif//__ASP_ISIS_INTERFACE_MAP_LINESCAN_H__
