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


/// \file CsmModel.h
///
/// Wrapper for Community Sensor Model implementations.
///
///
#ifndef __STEREO_CAMERA_CSM_MODEL_H__
#define __STEREO_CAMERA_CSM_MODEL_H__

#include <boost/shared_ptr.hpp>

#include <vw/Camera/CameraModel.h>

#include <asp/Core/StereoSettings.h>

namespace csm {
  class RasterGM; // Forward declaration
}

namespace asp {


  /// Class to load any cameras described by the Community Sensor Model (CSM)
  class CsmModel : public vw::camera::CameraModel {
  public:
    //------------------------------------------------------------------
    // Constructors / Destructors
    //------------------------------------------------------------------
    CsmModel();
    virtual ~CsmModel();
    virtual std::string type() const { return "CSM"; }

    
    /// Load the camera model from an ISD file.
    bool load_model(std::string const& isd_path);

    virtual vw::Vector2 point_to_pixel (vw::Vector3 const& point) const;

    virtual vw::Vector3 pixel_to_vector(vw::Vector2 const& pix) const;

    virtual vw::Vector3 camera_center(vw::Vector2 const& pix) const;

    virtual vw::Quaternion<double> camera_pose(vw::Vector2 const& pix) const {
      vw_throw( vw::NoImplErr() << "CsmModel: Cannot retrieve camera_pose!" );
      return vw::Quaternion<double>();
    }

  private:

    //const csm::Plugin* m_csm_plugin; // TODO: Do we need to hang on to this?
    boost::shared_ptr<csm::RasterGM> m_csm_model;
    //boost::shared_ptr<csm::Model> m_csm_model;

    /// Throw an exception if we have not loaded the model yet.
    void throw_if_not_init() const;

  }; // End class CsmModel



}      // namespace asp


#endif//__STEREO_CAMERA_CSM_MODEL_H__
