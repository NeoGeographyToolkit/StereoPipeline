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

// TODO(oalexan1): See if using proper earth radius and proper surface
// elevation improves the results! There is already some logic
// for extracting surface elevation from the RPC model.

// \file LinescanDGModel.h
// A wrapper around the CSM linescan model.

#ifndef __STEREO_CAMERA_LINESCAN_DG_MODEL_H__
#define __STEREO_CAMERA_LINESCAN_DG_MODEL_H__

#include <asp/Camera/TimeProcessing.h>
#include <vw/Camera/Extrinsics.h>
#include <vw/Cartography/Datum.h>
#include <vw/Math/EulerAngles.h>

// Forward declaration
class UsgsAstroLsSensorModel;

namespace asp {

  // Upper-right portion of the 3x3 satellite position covariance
  // matrix and 4x4 satellite quaternion matrix.
  const int SAT_POS_COV_SIZE = 6, SAT_QUAT_COV_SIZE = 10;
  
  // Forward declaration
  class CsmModel;

  // This is the standard DG implementation. In Extrinsics.cc there are
  // other ways of performing position and pose interpolation as well.
  // TODO(oalexan1): Eliminate all these VW functions and inheritance
  // from LinescanDGModel, and use directly the inputs from the XML file.
  class DGCameraModel: public vw::camera::CameraModel {
    
  public:
    
    DGCameraModel(vw::camera::PiecewiseAPositionInterpolation      const& position,
                  vw::camera::LinearPiecewisePositionInterpolation const& velocity,
                  vw::camera::SLERPPoseInterpolation               const& pose,
                  vw::camera::TLCTimeInterpolation                 const& time,
                  vw::Vector2i                                     const& image_size, 
                  vw::Vector2                                      const& detector_origin,
                  double                                           const  focal_length,
                  double                                           const  mean_ground_elevation);

    virtual ~DGCameraModel() {}
    virtual std::string type() const { return "LinescanDG"; }

    // Re-implement base class functions
    virtual double get_time_at_line(double line) const;

    virtual vw::Vector3 get_camera_center_at_time(double time) const;

    virtual vw::Vector3 get_camera_velocity_at_time(double time) const;

    virtual vw::Quat get_camera_pose_at_time(double time) const;

    // Gives a pointing vector in the world coordinates.
    virtual vw::Vector3 pixel_to_vector(vw::Vector2 const& pix) const;

    // Point to pixel with no initial guess
    virtual vw::Vector2 point_to_pixel(vw::Vector3 const& point) const;
    
    // Camera pose
    virtual vw::Quaternion<double> camera_pose(vw::Vector2 const& pix) const;

    /// Gives the camera position in world coordinates.
    virtual vw::Vector3 camera_center(vw::Vector2 const& pix) const;

    // CsmModel is ASP's wrapper around the CSM model. It holds a smart pointer
    // to the CSM linescan model, which is of type UsgsAstroLsSensorModel.
    boost::shared_ptr<CsmModel> m_csm_model; // wrapper
    boost::shared_ptr<UsgsAstroLsSensorModel> m_ls_model; // actual model

    // For error propagation
    std::vector<vw::CamPtr> m_perturbed_cams;
    std::vector<double> m_satellite_pos_cov, m_satellite_quat_cov;
    double m_satellite_pos_t0, m_satellite_pos_dt;
    double m_satellite_quat_t0, m_satellite_quat_dt;

    // Interpolate the satellite position covariance at given pixel
    void interpSatellitePosCov(vw::Vector2 const& pix, double p_cov[SAT_POS_COV_SIZE]) const;

    // Interpolate the satellite quaternion covariance at given pixel
    void interpSatelliteQuatCov(vw::Vector2 const& pix, double q_cov[SAT_QUAT_COV_SIZE]) const;
    
  private:
    // Function to interpolate quaternions with the CSM model. This is used
    // for validation of the CSM model but not in production.  
    void getQuaternions(const double& time, double q[4]) const;

    // Digital Globe implementation using CSM. Eventually this will replace
    // LinescanDGModel. Note that the CSM-based logic does not support velocity
    // aberration and atmospheric refraction correction. That needs to be
    // rectified before removing the older approach.
    void populateCsmModel();

    // Update poses for velocity aberration and/or atmospheric refraction.
    void orbitalCorrections();
    
    // Get the line number at a given time. This assumes a linear relationship
    // between them (rather than piecewise linear).
    double get_line_at_time(double time) const;
    
    // Extrinsics
    vw::camera::PiecewiseAPositionInterpolation m_position_func; // Position at given time
    vw::camera::LinearPiecewisePositionInterpolation m_velocity_func; // Velocity at given time
    vw::camera::SLERPPoseInterpolation m_pose_func; // Pose at given time
    vw::camera::TLCTimeInterpolation m_time_func; // Time at a given line

    // Intrinsics
    
    /// Location of (0,0) coordinate of the detector relative to the center of
    ///  the origin of the camera coordinate system.
    /// - Stored internally in pixels.
    vw::Vector2  m_detector_origin; 
    double       m_focal_length;  ///< The focal length, also stored in pixels.

    /// Image size in pixels: [num lines, num samples]
    vw::Vector2i m_image_size;      
  };

  /// Load a DG camera model from an XML file. This function does not
  /// take care of Xerces XML init/de-init, the caller must make sure
  /// this is done before/after this function is called.
  vw::CamPtr load_dg_camera_model_from_xml(std::string const& path);

} //end  namespace asp

#endif//__STEREO_CAMERA_LINESCAN_DG_MODEL_H__
