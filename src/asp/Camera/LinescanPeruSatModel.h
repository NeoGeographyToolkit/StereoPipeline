// __BEGIN_LICENSE__
//  Copyright (c) 2009-2026, United States Government as represented by the
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

/// \file LinescanPeruSatModel.h
///
/// CSM linescan camera model for PeruSat-1. Populates a UsgsAstroLsSensorModel
/// directly from the PeruSat XML metadata, using transverse distortion from
/// usgscsm to handle the tan(psi) look angle computation.

#ifndef __STEREO_CAMERA_LINESCAN_PERUSAT_MODEL_H__
#define __STEREO_CAMERA_LINESCAN_PERUSAT_MODEL_H__

#include <vw/Math/Matrix.h>
#include <vw/Math/Quaternion.h>
#include <asp/Camera/CsmModel.h>

// Forward declaration
class UsgsAstroLsSensorModel;

namespace asp {

  // CSM-based PeruSat camera model, following the Pleiades pattern.
  // Populates a UsgsAstroLsSensorModel directly, bypassing VW interpolation.
  class PeruSatCsmCameraModel: public asp::CsmModel {

  public:
    PeruSatCsmCameraModel(double time_t0, double time_dt,
                          std::vector<vw::Vector3> const& positions,
                          std::vector<vw::Vector3> const& velocities,
                          double pos_t0, double pos_dt,
                          std::vector<vw::Quaternion<double>> const& quaternions,
                          double quat_t0, double quat_dt,
                          vw::Vector2                        const& tan_psi_x,
                          vw::Vector2                        const& tan_psi_y,
                          vw::Quaternion<double>             const& instrument_biases,
                          vw::Vector2i                       const& image_size,
                          double min_time, double max_time);

    virtual ~PeruSatCsmCameraModel() {}
    virtual std::string type() const { return "LinescanPeruSat"; }

    // These delegate to the CSM model
    virtual vw::Vector3 pixel_to_vector(vw::Vector2 const& pix) const;
    virtual vw::Vector3 camera_center(vw::Vector2 const& pix) const;
    virtual vw::Vector2 point_to_pixel(vw::Vector3 const& point) const;

    virtual vw::Quaternion<double> camera_pose(vw::Vector2 const& pix) const {
      vw_throw(vw::NoImplErr() << "PeruSatCsmCameraModel: Cannot retrieve camera_pose!");
      return vw::Quaternion<double>();
    }

  private:
    void populateCsmModel();

    double m_time_t0, m_time_dt;
    std::vector<vw::Vector3> m_positions, m_velocities;
    double m_pos_t0, m_pos_dt;
    std::vector<vw::Quaternion<double>> m_quaternions;
    double m_quat_t0, m_quat_dt;
    vw::Vector2 m_tan_psi_x, m_tan_psi_y;
    vw::Quaternion<double> m_instrument_biases;
    vw::Vector2i m_image_size;
    double m_min_time, m_max_time;

    // Pointer to linescan sensor. Managed by CsmModel::m_gm_model.
    UsgsAstroLsSensorModel * m_ls_model;
  }; // End class PeruSatCsmCameraModel

  /// Load a PeruSat camera model from an XML file.
  boost::shared_ptr<PeruSatCsmCameraModel>
  load_perusat_camera_model_from_xml(std::string const& path);

} // end namespace asp

#endif//__STEREO_CAMERA_LINESCAN_PERUSAT_MODEL_H__
