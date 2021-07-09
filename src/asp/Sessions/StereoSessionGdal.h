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


/// \file StereoSessionGdal.h
///
/// This a session to support Digital Globe images from Quickbird and World View.

#ifndef __STEREO_SESSION_GDAL_H__
#define __STEREO_SESSION_GDAL_H__

#include <asp/Sessions/StereoSession.h>
#include <asp/Camera/RPCModel.h>
#include <asp/Camera/RPC_XML.h>
#include <asp/Camera/CsmModel.h>
#include <asp/IsisIO/IsisCameraModel.h>

namespace asp {

  /// Generic stereoSession implementation for images which we can
  /// read/write with GDAL. This class adds a "preprocessing hook"
  /// which aligns and normalizes the images using the specified
  /// methods. Derived classes need to set up camera model loading.
  class StereoSessionGdal : public StereoSession {
    
  public:
    StereoSessionGdal(){}
    virtual ~StereoSessionGdal(){}
    
    virtual std::string name() const = 0;
    
    /// Stage 1: Preprocessing
    ///
    /// Pre file is a pair of images.            (ImageView<PixelT>)
    /// Post file is a pair of grayscale images. (ImageView<PixelGray<float> >)
    virtual void pre_preprocessing_hook(bool adjust_left_image_size,
                                        std::string const& left_input_file,
                                        std::string const& right_input_file,
                                        std::string      & left_output_file,
                                        std::string      & right_output_file);
  };
  
  //----------------------------------------------------------

  // Stereo session for Digital Globe images.
  class StereoSessionDG : public StereoSessionGdal {

  public:
    StereoSessionDG(){}
    virtual ~StereoSessionDG(){}

    virtual std::string name() const { return "dg"; }

    /// Simple factory function
    static StereoSession* construct() { return new StereoSessionDG;}
    
  protected:
    /// Function to load a camera model of the particular type.
    virtual boost::shared_ptr<vw::camera::CameraModel>
    load_camera_model(std::string const& image_file, 
                      std::string const& camera_file,
                      vw::Vector2 pixel_offset) const {
      return load_adjusted_model(m_camera_loader.load_dg_camera_model(camera_file),
                                 image_file, camera_file, pixel_offset);
    }
  };

  //----------------------------------------------------------

  /// Stereo session for optical bar cameras such as Corona and Hexagon.
  class StereoSessionOpticalBar : public StereoSessionGdal {

  public:
    StereoSessionOpticalBar(){}
    virtual ~StereoSessionOpticalBar(){}

    virtual std::string name() const { return "opticalbar"; }

    /// Simple factory function
    static StereoSession* construct() { return new StereoSessionOpticalBar;}

  protected:
    /// Function to load a camera model of the particular type.
    virtual boost::shared_ptr<vw::camera::CameraModel>
    load_camera_model(std::string const& image_file, 
                      std::string const& camera_file,
                      vw::Vector2 pixel_offset) const {
      return load_adjusted_model(m_camera_loader.load_optical_bar_camera_model(camera_file),
                                 image_file, camera_file, pixel_offset);
    }
  };

  //----------------------------------------------------------
  
  /// Stereo session for CSM camera models that use GDAL compatible image files.
  /// - CSM files can also be used with ISIS image data, in which case
  /// they use StereoSessionIsis.
  class StereoSessionCsm : public StereoSessionGdal {

    // TODO(oalexan1): Should one deal with ISIS special pixels like for ISIS?
    
  public:
    StereoSessionCsm(){}
    virtual ~StereoSessionCsm(){}
    
    virtual std::string name() const { return "csm"; }
    
    /// Simple factory function
    static StereoSession* construct() { return new StereoSessionCsm;}
    
    /// Returns the target datum to use for a given camera model
    virtual vw::cartography::Datum get_datum(const vw::camera::CameraModel* cam,
                                             bool use_sphere_for_datum) const {

      // Peek at the .cub file to get the planet name without reading
      // it as an ISIS camera (which can fail unless the ISISDATA
      // folder exists, and for CSM that is not guaranteed.)
      std::string datum_name = asp::read_target_name(m_left_image_file);

      const asp::CsmModel * cast_csm_cam
        = dynamic_cast<const asp::CsmModel*>(vw::camera::unadjusted_model(cam));
      VW_ASSERT(cast_csm_cam != NULL,
                vw::ArgumentErr() << "Could not load a CSM camera.\n");

      vw::Vector3 radii = cast_csm_cam->target_radii();
      double radius1 = (radii[0] + radii[1]) / 2; // average the x and y axes (semi-major) 
      double radius2 = radius1;
      if (!use_sphere_for_datum) {
        radius2 = radii[2]; // the z radius (semi-minor axis)
      }
      
      vw::cartography::Datum datum("D_" + datum_name, datum_name,
                                   "Reference Meridian", radius1, radius2, 0);

      return datum;
    }
    
  protected:
    /// Function to load a camera model of the particular type.
    virtual boost::shared_ptr<vw::camera::CameraModel>
    load_camera_model(std::string const& image_file, 
                      std::string const& camera_file,
                      vw::Vector2 pixel_offset) const {
      return load_adjusted_model(m_camera_loader.load_csm_camera_model(camera_file),
                                 image_file, camera_file, pixel_offset);
    }
  };
  
  /// StereoSession instance for processing SPOT5 data.
  class StereoSessionSpot : public StereoSessionGdal {

  public:
    StereoSessionSpot(){}
    virtual ~StereoSessionSpot(){}
    
    virtual std::string name() const { return "spot5"; }
    
    /// Simple factory function
    static StereoSession* construct() { return new StereoSessionSpot; }
    
  protected:
    /// Function to load a camera model of the particular type.
    virtual boost::shared_ptr<vw::camera::CameraModel>
    load_camera_model(std::string const& image_file,
                      std::string const& camera_file,
                      vw::Vector2 pixel_offset) const{
      
      return load_adjusted_model(m_camera_loader.load_spot5_camera_model(camera_file),
                                 image_file, camera_file, pixel_offset);
    }
  };
  
} // End namespace asp

#endif//__STEREO_SESSION_GDAL_H__
