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

/// \file StereoSessionMapProj.h
///
/// This a session that support RPC Mapproject DG images. It is built
/// entirely so that left and right TX are objects and not TransformRefs.

#ifndef __STEREO_SESSION_MAPPROJ_H__
#define __STEREO_SESSION_MAPPROJ_H__

#include <asp/Sessions/StereoSession.h>
#include <asp/Sessions/StereoSessionGdal.h>
#include <asp/Sessions/StereoSessionPinhole.h>
#include <asp/Camera/CsmModel.h>
#include <asp/Core/CameraUtils.h>
#include <vw/Image/Transform.h>

namespace asp {
 
  /// Parent class for all map projected types
  /// - All map projected types inherit from the Gdal session since
  ///   our map projected images are tiff files.
  class StereoSessionMapProj: public StereoSessionGdal  {
  public:
    StereoSessionMapProj(){};
    virtual ~StereoSessionMapProj(){};
    
    virtual std::string name() const = 0;
    virtual bool isMapProjected() const { return true; }

    virtual vw::TransformPtr tx_left () const {return tx_left_map_trans ();}
    virtual vw::TransformPtr tx_right() const {return tx_right_map_trans();}
  };
  
  
  /// Specialization of the StereoSessionGDAL class to use RPC
  /// map-projected inputs with the DG sensor model.
  class StereoSessionDGMapRPC: public StereoSessionMapProj {
  public:
    StereoSessionDGMapRPC(){};
    virtual ~StereoSessionDGMapRPC(){};

    virtual std::string name() const { return "dgmaprpc"; }

    static StereoSession* construct() { return new StereoSessionDGMapRPC; }
    
  protected:
    /// Function to load a camera model of the particular type.
    virtual boost::shared_ptr<vw::camera::CameraModel>
    load_camera_model(std::string const& image_file, 
                      std::string const& camera_file,
                      std::string const& ba_prefix, 
                      vw::Vector2 pixel_offset) const {
      return load_adjusted_model(m_camera_loader.load_dg_camera_model(camera_file),
                                 image_file, camera_file, ba_prefix, pixel_offset);
    }
  };

  /// Specialization of the StereoSessionGDAL class to use DG
  /// map-projected inputs with the DG sensor model.
  class StereoSessionDGMapDG: public StereoSessionMapProj {
  public:
    StereoSessionDGMapDG(){};
    virtual ~StereoSessionDGMapDG(){};

    virtual std::string name() const { return "dgmapdg"; }

    static StereoSession* construct() { return new StereoSessionDGMapDG; }
    
  protected:
    /// Function to load a camera model of the particular type.
    virtual boost::shared_ptr<vw::camera::CameraModel>
    load_camera_model(std::string const& image_file, 
                      std::string const& camera_file,
                      std::string const& ba_prefix, 
                      vw::Vector2 pixel_offset) const {
      return load_adjusted_model(m_camera_loader.load_dg_camera_model(camera_file),
                                 image_file, camera_file, ba_prefix, pixel_offset);
    }
  };

  /// Specialization of the StereoSessionGDAL class to use RPC
  /// map-projected inputs with the RPC sensor model.
  class StereoSessionRPCMapRPC: public StereoSessionMapProj  {
  public:
    StereoSessionRPCMapRPC(){};
    virtual ~StereoSessionRPCMapRPC(){};

    virtual std::string name() const { return "rpcmaprpc"; }

    static StereoSession* construct() { return new StereoSessionRPCMapRPC; }
    
  protected:
    /// Function to load a camera model of the particular type.
    virtual boost::shared_ptr<vw::camera::CameraModel>
    load_camera_model(std::string const& image_file, 
                      std::string const& camera_file,
                      std::string const& ba_prefix, 
                      vw::Vector2 pixel_offset) const {
      return load_rpc_camera_model(image_file, camera_file, ba_prefix, pixel_offset);
  }
  
  };

  /// Specialization of the StereoSessionGDAL class to use (ISIS)
  /// map-projected inputs with the ISIS sensor model.
  class StereoSessionIsisMapIsis: public StereoSessionMapProj  {
  public:
    StereoSessionIsisMapIsis() {};
    virtual ~StereoSessionIsisMapIsis(){};

    virtual std::string name() const { return "isismapisis"; }

    virtual bool supports_multi_threading() const {
      return false;
    }

    static StereoSession* construct() { return new StereoSessionIsisMapIsis; }
    
  protected:
    /// Function to load a camera model of the particular type.
    virtual boost::shared_ptr<vw::camera::CameraModel>
    load_camera_model(std::string const& image_file, 
                      std::string const& camera_file,
                      std::string const& ba_prefix, 
                      vw::Vector2 pixel_offset) const {
    return load_adjusted_model(m_camera_loader.load_isis_camera_model(camera_file),
                               image_file, camera_file, ba_prefix, pixel_offset);
    }
    
  };

  /// Specialization of the StereoSessionGDAL class to use Pinhole
  /// map-projected inputs with the Pinhole sensor model.
  class StereoSessionPinholeMapPinhole: public StereoSessionMapProj{
  public:
    StereoSessionPinholeMapPinhole() {}
    virtual ~StereoSessionPinholeMapPinhole() {}

    virtual std::string name() const { return "pinholemappinhole"; }

    static StereoSession* construct() { return new StereoSessionPinholeMapPinhole; }
    
  protected:
    /// Function to load a camera model of the particular type.
    virtual boost::shared_ptr<vw::camera::CameraModel>
    load_camera_model(std::string const& image_file, 
                      std::string const& camera_file,
                      std::string const& ba_prefix, 
                      vw::Vector2 pixel_offset) const {
      return asp::load_adj_pinhole_model(image_file, camera_file,
                                         m_left_image_file, m_right_image_file,
                                         m_left_camera_file, m_right_camera_file,
                                         ba_prefix, isMapProjected());
    }
  };

  /// Specialization of the StereoSessionGDAL class to use optical bar
  /// map-projected inputs with the optical bar sensor model.
  class StereoSessionBarMapBar: public StereoSessionMapProj{
  public:
    StereoSessionBarMapBar() {}
    virtual ~StereoSessionBarMapBar() {}

    virtual std::string name() const { return "opticalbarmapopticalbar"; }

    static StereoSession* construct() { return new StereoSessionBarMapBar; }

  protected:
    /// Function to load a camera model of the particular type.
    virtual boost::shared_ptr<vw::camera::CameraModel>
    load_camera_model(std::string const& image_file, 
                      std::string const& camera_file,
                      std::string const& ba_prefix,
                      vw::Vector2 pixel_offset) const {
      return load_adjusted_model(m_camera_loader.load_optical_bar_camera_model(camera_file),
                                 image_file, camera_file, ba_prefix, pixel_offset);
    }
  };

  /// Specialization of the StereoSessionGDAL class to use CSM
  /// map-projected inputs with the CSM sensor model.
  class StereoSessionCsmMapCsm: public StereoSessionMapProj{
  public:
    StereoSessionCsmMapCsm() {}
    virtual ~StereoSessionCsmMapCsm() {}

    virtual std::string name() const { return "csmmapcsm"; }

    static StereoSession* construct() { return new StereoSessionCsmMapCsm; }

  protected:
    /// Function to load a camera model of the particular type.
    virtual boost::shared_ptr<vw::camera::CameraModel>
    load_camera_model(std::string const& image_file, 
                      std::string const& camera_file,
                      std::string const& ba_prefix,
                      vw::Vector2 pixel_offset) const {
    return load_adjusted_model(m_camera_loader.load_csm_camera_model(camera_file),
                               image_file, camera_file, ba_prefix, pixel_offset);
    }
  };

  /// Specialization of the StereoSessionGDAL class to use RPC
  /// map-projected inputs with the CSM sensor model. The 
  /// loading of the camera model used to undo the mapprojection
  /// happens in read_mapproj_cams().
  class StereoSessionCsmMapRpc: public StereoSessionMapProj{
  public:
    StereoSessionCsmMapRpc() {}
    virtual ~StereoSessionCsmMapRpc() {}

    virtual std::string name() const { return "csmmaprpc"; }

    static StereoSession* construct() { return new StereoSessionCsmMapRpc; }

  protected:
    /// Function to load a camera model of the particular type, for
    /// the purpose of triangulation.
    virtual boost::shared_ptr<vw::camera::CameraModel>
    load_camera_model(std::string const& image_file, 
                      std::string const& camera_file,
                      std::string const& ba_prefix,
                      vw::Vector2 pixel_offset) const {
    return load_adjusted_model(m_camera_loader.load_csm_camera_model(camera_file),
                               image_file, camera_file, ba_prefix, pixel_offset);
    }
  };

  /// Specialization of the StereoSessionGDAL class to use RPC
  /// map-projected inputs with the SPOT5 sensor model.
  class StereoSessionSpot5MapRPC: public StereoSessionMapProj  {
  public:
    StereoSessionSpot5MapRPC(){};
    virtual ~StereoSessionSpot5MapRPC(){};

    virtual std::string name() const { return "spot5maprpc"; }

    static StereoSession* construct() { return new StereoSessionSpot5MapRPC; }
    
   protected:
    /// Function to load a camera model of the particular type.
    virtual boost::shared_ptr<vw::camera::CameraModel>
    load_camera_model(std::string const& image_file, 
                      std::string const& camera_file,
                      std::string const& ba_prefix,
                      vw::Vector2 pixel_offset) const {
    return load_adjusted_model(m_camera_loader.load_spot5_camera_model(camera_file),
                               image_file, camera_file, ba_prefix, pixel_offset);
    }
    
  };

  /// Specialization of the StereoSessionGDAL class to use RPC
  /// map-projected inputs with the ASTER sensor model.
  class StereoSessionASTERMapRPC: public StereoSessionMapProj  {
  public:
    StereoSessionASTERMapRPC(){};
    virtual ~StereoSessionASTERMapRPC(){};

    virtual std::string name() const { return "astermaprpc"; }

    static StereoSession* construct() { return new StereoSessionASTERMapRPC; }
    
  protected:
    /// Function to load a camera model of the particular type.
    virtual boost::shared_ptr<vw::camera::CameraModel>
    load_camera_model(std::string const& image_file, 
                      std::string const& camera_file,
                      std::string const& ba_prefix,
                      vw::Vector2 pixel_offset) const {
    return load_adjusted_model(m_camera_loader.load_ASTER_camera_model(camera_file),
                               image_file, camera_file, ba_prefix, pixel_offset);
    }
    
  };

  /// Specialization of the StereoSessionGDAL class to use ASTER
  /// map-projected inputs with the ASTER sensor model.
  class StereoSessionASTERMapASTER: public StereoSessionMapProj  {
  public:
    StereoSessionASTERMapASTER(){};
    virtual ~StereoSessionASTERMapASTER(){};

    virtual std::string name() const { return "astermapaster"; }

    static StereoSession* construct() { return new StereoSessionASTERMapASTER; }
    
  protected:
    /// Function to load a camera model of the particular type.
    virtual boost::shared_ptr<vw::camera::CameraModel>
    load_camera_model(std::string const& image_file, 
                      std::string const& camera_file,
                      std::string const& ba_prefix,
                      vw::Vector2 pixel_offset) const {
    return load_adjusted_model(m_camera_loader.load_ASTER_camera_model(camera_file),
                                 image_file, camera_file, ba_prefix, pixel_offset);
    }
    
  };


  /// Specialization of the StereoSessionGDAL class to use Pleiades
  /// map-projected inputs with the Pleiades sensor model.
  class StereoSessionPleiadesMapPleiades: public StereoSessionMapProj  {
  public:
    StereoSessionPleiadesMapPleiades(){};
    virtual ~StereoSessionPleiadesMapPleiades(){};

    virtual std::string name() const { return "pleiadesmappleiades"; }

    static StereoSession* construct() { return new StereoSessionPleiadesMapPleiades; }
 
  protected:
    /// Function to load a camera model of the particular type.
    virtual boost::shared_ptr<vw::camera::CameraModel>
    load_camera_model(std::string const& image_file, 
                      std::string const& camera_file,
                      std::string const& ba_prefix,
                      vw::Vector2 pixel_offset) const {
    return load_adjusted_model(m_camera_loader.load_pleiades_camera_model(camera_file),
                               image_file, camera_file, ba_prefix, pixel_offset);
    }
    
  };

}

#endif//__STEREO_SESSION_MAPPROJ_H__
