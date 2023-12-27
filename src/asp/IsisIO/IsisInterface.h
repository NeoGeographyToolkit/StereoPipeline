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


/// \file IsisInterface.h
///
/// Generic Interface with ISIS
///
#ifndef __ASP_ISIS_INTERFACE_H__
#define __ASP_ISIS_INTERFACE_H__

// Must include foreach.hpp before Cube.h as otherwise there will be
// an issue with Qt imported by Cube.h.
#include <boost/foreach.hpp>

// VW & ASP
#include <vw/Math/Vector.h>
#include <vw/Math/Quaternion.h>
#include <vw/Cartography/Datum.h>

// Isis include
#include <Cube.h>

#include <string>

namespace Isis {
  class Pvl;
  class Camera;
}

namespace asp {
namespace isis {

  /// The IsisInterface abstract base class
  // -------------------------------------------------------

  class IsisInterface {
  public:
    IsisInterface( std::string const& file );
    virtual ~IsisInterface(); // Can't be declared here since we have
                              // incomplete types from Isis.

    virtual std::string type() = 0;
    
    /// Construct an IsisInterface-derived class of the correct type for the given file.
    static IsisInterface* open( std::string const& filename );

    // Standard Methods
    //------------------------------------------------------

    // These are the standard camera request; IsisInterface allows for
    // them to be customized for the type of camera so that they are
    // fast and not too full of conditionals.

    virtual vw::Vector2 point_to_pixel ( vw::Vector3 const& point               ) const = 0;
    virtual vw::Vector3 pixel_to_vector( vw::Vector2 const& pix                 ) const = 0;
    virtual vw::Vector3 camera_center  ( vw::Vector2 const& pix = vw::Vector2() ) const = 0;
    virtual vw::Quat    camera_pose    ( vw::Vector2 const& pix = vw::Vector2() ) const = 0;

    // General information
    //------------------------------------------------------
    int         lines         () const;
    int         samples       () const;
    std::string serial_number () const;
    double      ephemeris_time( vw::Vector2 const& pix ) const;
    vw::Vector3 sun_position  ( vw::Vector2 const& pix = vw::Vector2() ) const;
    vw::Vector3 target_radii  () const;
    std::string target_name   () const;
    vw::cartography::Datum get_datum(bool use_sphere_for_non_earth) const;

  protected:
    // Standard Variables
    //------------------------------------------------------
    boost::scoped_ptr<Isis::Pvl   > m_label;
    boost::scoped_ptr<Isis::Camera> m_camera;
    boost::scoped_ptr<Isis::Cube  > m_cube;

    vw::cartography::Datum m_datum;
    
    friend std::ostream& operator<<( std::ostream&, IsisInterface* );
  };

  // IOstream interface
  // -------------------------------------------------------
  std::ostream& operator<<( std::ostream& os, IsisInterface* i );

  bool IsisEnv();

  // Peek inside a Cube file to see if it has a CSM blob. This needs ISIS
  // logic, rather than any CSM-specific info.  
  bool IsisCubeHasCsmBlob(std::string const& CubeFile);
  
  // Read the CSM state (a string) from a cube file. Throw an exception
  // if missing.
  std::string csmStateFromIsisCube(std::string const& CubeFile);
  
  // Save a CSM state to an ISIS Cube file. Wipe any spice info. This may
  // throw if the file cannot be saved.
  void saveCsmStateToIsisCube(std::string const& CubeFile, std::string const& csmState);
}}

#endif //__ASP_ISIS_INTERFACE_H__
