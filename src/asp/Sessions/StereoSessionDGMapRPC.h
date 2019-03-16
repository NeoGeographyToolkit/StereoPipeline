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

#include <asp/Sessions/StereoSession.h>
#include <asp/Sessions/StereoSessionGdal.h>
#include <asp/Sessions/StereoSessionPinhole.h>
#include <asp/Camera/CsmModel.h>
#include <vw/Cartography/Map2CamTrans.h>
#include <vw/Image/Transform.h>

namespace asp {
 
  /// Parent class for all map projected types
  /// - All map projected types inherit from the Gdal session since
  ///   our map projected images are tiff files.
  class StereoSessionMapProj : public StereoSessionGdal  {
  public:
    StereoSessionMapProj(){};
    virtual ~StereoSessionMapProj(){};
    
    virtual std::string name() const = 0;
    virtual bool isMapProjected() const { return true; } // TODO: Delete?

    virtual tx_type tx_left () const {return tx_left_map_trans ();}
    virtual tx_type tx_right() const {return tx_right_map_trans();}
  };
  
  
  /// Specialization of the StereoSessionGDAL class to use (RPC) map-projected inputs with the DG sensor model.
  class StereoSessionDGMapRPC : public StereoSessionMapProj  {
  public:
    StereoSessionDGMapRPC(){};
    virtual ~StereoSessionDGMapRPC(){};

    virtual std::string name() const { return "dgmaprpc"; }

    static StereoSession* construct() { return new StereoSessionDGMapRPC; }

  protected:
    /// Function to load a camera model of the particular type.
    virtual boost::shared_ptr<vw::camera::CameraModel> load_camera_model(std::string const& image_file, 
                                                                         std::string const& camera_file,
                                                                         vw::Vector2 pixel_offset) const {
      return load_adjusted_model(m_camera_loader.load_dg_camera_model(camera_file),
                                 image_file, camera_file, pixel_offset);
    }
  };



  /// Specialization of the StereoSessionGDAL class to use (RPC) map-projected inputs with the RPC sensor model.
  class StereoSessionRPCMapRPC : public StereoSessionMapProj  {
  public:
    StereoSessionRPCMapRPC(){};
    virtual ~StereoSessionRPCMapRPC(){};

    virtual std::string name() const { return "rpcmaprpc"; }

    static StereoSession* construct() { return new StereoSessionRPCMapRPC; }
    
  protected:
    /// Function to load a camera model of the particular type.
    virtual boost::shared_ptr<vw::camera::CameraModel> load_camera_model(std::string const& image_file, 
                                                                         std::string const& camera_file,
                                                                         vw::Vector2 pixel_offset) const {
    return load_rpc_camera_model(image_file, camera_file, pixel_offset);
  }
  
  };


  /// Specialization of the StereoSessionGDAL class to use (ISIS) map-projected inputs with the ISIS sensor model.
  class StereoSessionIsisMapIsis : public StereoSessionMapProj  {
  public:
    StereoSessionIsisMapIsis(){
      // Supporting this option (or anything else ISIS specific) requires more class refactoring!
      if (stereo_settings().mask_flatfield)
        vw::vw_throw( vw::NoImplErr() << "StereoSessionIsisMapIsis does not support mask_flatfield" );
    };
    virtual ~StereoSessionIsisMapIsis(){};

    virtual std::string name() const { return "isismapisis"; }
    virtual bool uses_rpc_map_projection() const {return false;}

    /// Only the alternative CSM sensor model for ISIS images supports multi threading.
    virtual bool supports_multi_threading() const {
      return (asp::CsmModel::file_has_isd_extension(m_left_camera_file ) && 
              asp::CsmModel::file_has_isd_extension(m_right_camera_file)   );
    }

    static StereoSession* construct() { return new StereoSessionIsisMapIsis; }
    
  protected:
    /// Function to load a camera model of the particular type.
    virtual boost::shared_ptr<vw::camera::CameraModel> load_camera_model(std::string const& image_file, 
                                                                         std::string const& camera_file,
                                                                         vw::Vector2 pixel_offset) const {
      return load_adjusted_model(m_camera_loader.load_isis_camera_model(camera_file),
                                 image_file, camera_file, pixel_offset);
    }
    
  };

  /// Specialization of the StereoSessionGDAL class to use (PINHOLE) map-projected inputs with the PINHOLE sensor model.
  class StereoSessionPinholeMapPinhole : public StereoSessionMapProj{
  public:
    StereoSessionPinholeMapPinhole() {}
    virtual ~StereoSessionPinholeMapPinhole() {}

    virtual std::string name() const { return "pinholemappinhole"; }
    virtual bool uses_rpc_map_projection() const {return false;}

    static StereoSession* construct() { return new StereoSessionPinholeMapPinhole; }
    
  protected:
    /// Function to load a camera model of the particular type.
    virtual boost::shared_ptr<vw::camera::CameraModel> load_camera_model(std::string const& image_file, 
                                                                         std::string const& camera_file,
                                                                         vw::Vector2 pixel_offset) const {
      return StereoSessionPinhole::load_adj_pinhole_model(image_file, camera_file,
                                                          m_left_image_file, m_right_image_file,
                                                          m_left_camera_file, m_right_camera_file,
                                                          m_input_dem);
    }
  };


  /// Specialization of the StereoSessionGDAL class to use optical bar map-projected inputs with the optical bar sensor model.
  class StereoSessionBarMapBar : public StereoSessionMapProj{
  public:
    StereoSessionBarMapBar() {}
    virtual ~StereoSessionBarMapBar() {}

    virtual std::string name() const { return "opticalbarmapopticalbar"; }
    virtual bool uses_rpc_map_projection() const {return false;}

    static StereoSession* construct() { return new StereoSessionBarMapBar; }

  protected:
    /// Function to load a camera model of the particular type.
    virtual boost::shared_ptr<vw::camera::CameraModel> load_camera_model(std::string const& image_file, 
                                                                         std::string const& camera_file,
                                                                         vw::Vector2 pixel_offset) const {
      return load_adjusted_model(m_camera_loader.load_optical_bar_camera_model(camera_file),
                                 image_file, camera_file, pixel_offset);
    }
  };

  /// Specialization of the StereoSessionGDAL class to use CSM map-projected inputs with the CSM sensor model.
  class StereoSessionCsmMapCsm : public StereoSessionMapProj{
  public:
    StereoSessionCsmMapCsm() {}
    virtual ~StereoSessionCsmMapCsm() {}

    virtual std::string name() const { return "csmmapcsm"; }
    virtual bool uses_rpc_map_projection() const {return false;}

    static StereoSession* construct() { return new StereoSessionCsmMapCsm; }

  protected:
    /// Function to load a camera model of the particular type.
    virtual boost::shared_ptr<vw::camera::CameraModel> load_camera_model(std::string const& image_file, 
                                                                         std::string const& camera_file,
                                                                         vw::Vector2 pixel_offset) const {
      return load_adjusted_model(m_camera_loader.load_csm_camera_model(camera_file),
                                 image_file, camera_file, pixel_offset);
    }
  };

  /// Specialization of the StereoSessionGDAL class to use (RPC) map-projected inputs with the SPOT5 sensor model.
  class StereoSessionSpot5MapRPC : public StereoSessionMapProj  {
  public:
    StereoSessionSpot5MapRPC(){};
    virtual ~StereoSessionSpot5MapRPC(){};

    virtual std::string name() const { return "spot5maprpc"; }

    static StereoSession* construct() { return new StereoSessionSpot5MapRPC; }
    
   protected:
    /// Function to load a camera model of the particular type.
    virtual boost::shared_ptr<vw::camera::CameraModel> load_camera_model(std::string const& image_file, 
                                                                         std::string const& camera_file,
                                                                         vw::Vector2 pixel_offset) const {
      return load_adjusted_model(m_camera_loader.load_spot5_camera_model(camera_file),
                                 image_file, camera_file, pixel_offset);
    }
    
  };

  /// Specialization of the StereoSessionGDAL class to use (RPC) map-projected inputs with the ASTER sensor model.
  class StereoSessionASTERMapRPC : public StereoSessionMapProj  {
  public:
    StereoSessionASTERMapRPC(){};
    virtual ~StereoSessionASTERMapRPC(){};

    virtual std::string name() const { return "astermaprpc"; }

    static StereoSession* construct() { return new StereoSessionASTERMapRPC; }
    
  protected:
    /// Function to load a camera model of the particular type.
    virtual boost::shared_ptr<vw::camera::CameraModel> load_camera_model(std::string const& image_file, 
                                                                         std::string const& camera_file,
                                                                         vw::Vector2 pixel_offset) const {
      return load_adjusted_model(m_camera_loader.load_ASTER_camera_model(camera_file),
                                 image_file, camera_file, pixel_offset);
    }
    
  };

}

#endif//__STEREO_SESSION_DGMAPRPC_H__
