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


/// \file LinescanPleiadesModel.h
///
/// Linescan camera model for Pleiades. Implemented based on
///  Pleiades Imagery User Guide
/// https://www.intelligence-airbusds.com/automne/api/docs/v1.0/document/download/ZG9jdXRoZXF1ZS1kb2N1bWVudC01NTY0Mw==/ZG9jdXRoZXF1ZS1maWxlLTU1NjQy/airbus-pleiades-imagery-user-guide-15042021.pdf
///
#ifndef __STEREO_CAMERA_LINESCAN_PLEIADES_MODEL_H__
#define __STEREO_CAMERA_LINESCAN_PLEIADES_MODEL_H__

#include <vw/Math/Matrix.h>
#include <vw/Camera/LinescanModel.h>
#include <vw/Camera/PinholeModel.h>
#include <vw/Camera/Extrinsics.h>

namespace asp {

  /// Specialization of the generic LinescanModel for Pleiades satellites.
  class PleiadesCameraModel : public vw::camera::LinescanModel {

  public:
    //------------------------------------------------------------------
    // Constructors / Destructors
    //------------------------------------------------------------------
    PleiadesCameraModel(vw::camera::LagrangianInterpolation const& position,
                       vw::camera::LagrangianInterpolation const& velocity,
                       vw::camera::SLERPPoseInterpolation  const& pose,
                       vw::camera::LinearTimeInterpolation const& time,
                       vw::Vector2                         const& tan_psi_x,
                       vw::Vector2                         const& tan_psi_y,
                       vw::Quaternion<double>              const& instrument_biases,
                       vw::Vector2i                        const& image_size,
                       double min_time, double max_time,
                       bool   correct_velocity, bool   correct_atmosphere):
      vw::camera::LinescanModel(image_size, correct_velocity, correct_atmosphere),
      m_position_func(position), m_velocity_func(velocity),
      m_pose_func(pose), m_time_func(time),
      m_tan_psi_x(tan_psi_x), m_tan_psi_y(tan_psi_y),
      m_inverse_instrument_biases(inverse(instrument_biases)),
      m_min_time(min_time), m_max_time(max_time) {}
    
    virtual ~PleiadesCameraModel() {}
    virtual std::string type() const { return "LinescanPleiades"; }

    // -- This set of functions implements virtual functions from LinescanModel.h --

    // Implement the functions from the LinescanModel class using functors
    virtual vw::Vector3 get_camera_center_at_time  (double time) const;
    virtual vw::Vector3 get_camera_velocity_at_time(double time) const;
    virtual vw::Quat    get_camera_pose_at_time    (double time) const;
    virtual double      get_time_at_line           (double line) const;
    
    /// As pixel_to_vector, but in the local camera frame.
    virtual vw::Vector3 get_local_pixel_vector(vw::Vector2 const& pix) const;

    // TODO: See if we can port these local changes to the parent class
    virtual vw::Vector2 point_to_pixel(vw::Vector3 const& point, double starty) const;
    virtual vw::Vector2 point_to_pixel(vw::Vector3 const& point) const {
      return point_to_pixel(point, -1); // Redirect to the function with no initial guess
    }
    
  private:

    vw::camera::LinearTimeInterpolation m_time_func;     ///< Yields time at a given line.
    vw::camera::LagrangianInterpolation m_position_func; ///< Yields position at time T
    vw::camera::LagrangianInterpolation m_velocity_func; ///< Yields velocity at time T
    vw::camera::SLERPPoseInterpolation  m_pose_func;     ///< Yields pose     at time T
    
    // These are used to find the look direction in camera coordinates at a given line
    vw::Vector2 m_tan_psi_x, m_tan_psi_y;
    vw::Quaternion<double> m_inverse_instrument_biases;

    /// These are the limits of when he have pose data available.
    double m_min_time, m_max_time;
    
    /// Throw an exception if the input time is outside the given bounds.
    /// - Pass the caller location in to get a nice error message.
    void check_time(double time, std::string const& location) const;

  }; // End class PleiadesCameraModel


  /// Load a Pleiades camera model from an XML file.
  /// - This function does not take care of Xerces XML init/de-init, the caller must
  ///   make sure this is done before/after this function is called!
  boost::shared_ptr<PleiadesCameraModel> load_pleiades_camera_model_from_xml(std::string const& path);

} // end namespace asp


#endif//__STEREO_CAMERA_LINESCAN_PLEIADES_MODEL_H__
