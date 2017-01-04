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


/// \file StereoSessionDGMapRPC.h
///
/// This a session that support RPC Mapproject DG images. It is built
/// entirely so that left and right TX are objects and not TransformRefs.

#ifndef __STEREO_SESSION_DGMAPRPC_H__
#define __STEREO_SESSION_DGMAPRPC_H__

#include <asp/Sessions/StereoSessionConcrete.h>
#include <asp/Sessions/StereoSessionGdal.h>
#include <vw/Cartography/Map2CamTrans.h>
#include <vw/Image/Transform.h>

namespace asp {
 
    
  /// Specialization of the StereoSessionGDAL class to use (RPC) map-projected inputs with the DG sensor model.
  class StereoSessionDGMapRPC : public StereoSessionGdal<DISKTRANSFORM_TYPE_MAP_PROJECT_RPC, STEREOMODEL_TYPE_DG>  {
  public:
    StereoSessionDGMapRPC(){};
    virtual ~StereoSessionDGMapRPC(){};

    virtual std::string name() const { return "dgmaprpc"; }
    virtual bool        uses_map_projected_inputs() const {return true;}

    static StereoSession* construct() { return new StereoSessionDGMapRPC; }
  };



  /// Specialization of the StereoSessionGDAL class to use (RPC) map-projected inputs with the RPC sensor model.
  class StereoSessionRPCMapRPC : public StereoSessionGdal<DISKTRANSFORM_TYPE_MAP_PROJECT_RPC, STEREOMODEL_TYPE_RPC>  {
  public:
    StereoSessionRPCMapRPC(){};
    virtual ~StereoSessionRPCMapRPC(){};

    virtual std::string name() const { return "rpcmaprpc"; }
    virtual bool        uses_map_projected_inputs() const {return true;}

    static StereoSession* construct() { return new StereoSessionRPCMapRPC; }
  };


  // TODO: Clean things up so there are not ISIS and PINHOLE classes that are not actually used by these classes!


  /// Specialization of the StereoSessionGDAL class to use (ISIS) map-projected inputs with the ISIS sensor model.
  class StereoSessionIsisMapIsis : public StereoSessionGdal<DISKTRANSFORM_TYPE_MAP_PROJECT_ISIS, STEREOMODEL_TYPE_ISIS>  {
  public:
    StereoSessionIsisMapIsis(){
      // Supporting this option (or anything else ISIS specific) requires more class refactoring!
      if (stereo_settings().mask_flatfield)
        vw_throw( NoImplErr() << "StereoSessionIsisMapIsis does not support mask_flatfield" );
    };
    virtual ~StereoSessionIsisMapIsis(){};

    virtual std::string name() const { return "isismapisis"; }
    virtual bool        uses_map_projected_inputs() const {return true;}

    static StereoSession* construct() { return new StereoSessionIsisMapIsis; }
  };

  /// Specialization of the StereoSessionGDAL class to use (PINHOLE) map-projected inputs with the PINHOLE sensor model.
  class StereoSessionPinholeMapPinhole : public StereoSessionGdal<DISKTRANSFORM_TYPE_MAP_PROJECT_PINHOLE, STEREOMODEL_TYPE_PINHOLE>{
  public:
    StereoSessionPinholeMapPinhole() {}
    virtual ~StereoSessionPinholeMapPinhole() {}

    virtual std::string name() const { return "pinholemappinhole"; }
    virtual bool        uses_map_projected_inputs() const {return true;}

    static StereoSession* construct() { return new StereoSessionPinholeMapPinhole; }
  };

  /// Specialization of the StereoSessionGDAL class to use (RPC) map-projected inputs with the SPOT5 sensor model.
  class StereoSessionSpot5MapRPC : public StereoSessionGdal<DISKTRANSFORM_TYPE_MAP_PROJECT_RPC, STEREOMODEL_TYPE_SPOT5>  {
  public:
    StereoSessionSpot5MapRPC(){};
    virtual ~StereoSessionSpot5MapRPC(){};

    virtual std::string name() const { return "spot5maprpc"; }
    virtual bool        uses_map_projected_inputs() const {return true;}

    static StereoSession* construct() { return new StereoSessionSpot5MapRPC; }
  };

  /// Specialization of the StereoSessionGDAL class to use (RPC) map-projected inputs with the ASTER sensor model.
  class StereoSessionASTERMapRPC : public StereoSessionGdal<DISKTRANSFORM_TYPE_MAP_PROJECT_RPC, STEREOMODEL_TYPE_ASTER>  {
  public:
    StereoSessionASTERMapRPC(){};
    virtual ~StereoSessionASTERMapRPC(){};

    virtual std::string name() const { return "astermaprpc"; }
    virtual bool        uses_map_projected_inputs() const {return true;}

    static StereoSession* construct() { return new StereoSessionASTERMapRPC; }
  };

}

#endif//__STEREO_SESSION_DGMAPRPC_H__
