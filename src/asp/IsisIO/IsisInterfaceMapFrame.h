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


/// \file IsisInterfaceMapFrame.h
///
/// Map Projected Frame Camera Interface with ISIS
///
#ifndef __ASP_ISIS_INTERFACE_MAP_FRAME_H__
#define __ASP_ISIS_INTERFACE_MAP_FRAME_H__

#include <TProjection.h>
#include <vw/Math/Vector.h>
#include <vw/Math/Quaternion.h>
#include <asp/IsisIO/IsisInterface.h>

#include <string>

// Isis forward declaration
namespace Isis {
  class CameraGroundMap;
  class CameraDistortionMap;
  class Projection;
}
#include <Distance.h>

namespace asp {
namespace isis {

  /// ??
  class IsisInterfaceMapFrame : public IsisInterface {

  public:
    IsisInterfaceMapFrame( std::string const& file );

    virtual std::string type()  { return "MapFrame"; }

    // Standard Methods
    //-------------------------------------------------

    virtual vw::Vector2 point_to_pixel ( vw::Vector3 const& point ) const; // TODO: Inaccurate!  Needs DATUM!
    virtual vw::Vector3 pixel_to_vector( vw::Vector2 const& pix   ) const;
    virtual vw::Vector3 camera_center  ( vw::Vector2 const& pix = vw::Vector2() ) const;
    virtual vw::Quat    camera_pose    ( vw::Vector2 const& pix = vw::Vector2() ) const;

  protected:

    // Custom Variables
    boost::scoped_ptr<Isis::TProjection> m_projection;
    Isis::CameraGroundMap     *m_groundmap;
    Isis::CameraDistortionMap *m_distortmap;

    vw::Vector3    m_center;
    vw::Quat       m_pose;
    Isis::Distance m_radii[3];
  };

}}

#endif//__ASP_ISIS_INTERFACE_MAP_FRAME_H__
