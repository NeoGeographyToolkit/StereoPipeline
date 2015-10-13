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


/// \file LinescanDGModel.h
///
/// A generic linescan camera model object
///
///
#ifndef __STEREO_CAMERA_LINESCAN_DG_MODEL_H__
#define __STEREO_CAMERA_LINESCAN_DG_MODEL_H__

#include <vw/Math/Quaternion.h>
#include <vw/Camera/CameraModel.h>
#include <vw/Camera/PinholeModel.h>
#include <vw/Camera/Extrinsics.h>
#include <vw/Math/EulerAngles.h>
#include <asp/Camera/RPCModel.h>
#include <asp/Camera/DG_XML.h>
#include <boost/date_time/posix_time/posix_time.hpp>


namespace asp {

  // This is potentially a more generic line scan camera model that
  // allows an offset in the line direction. This also allows a
  // different function for evaluating time for a specific line
  // location.
  //
  // This expects the pose to be a rotation from the camera frame to
  // the world frame. The position is a the camera's location in the
  // world frame.
  //
  // The intrinisic model expects +Z to be point out the camera. +X is
  // the column direction of the image and is perpendicular to
  // direction of flight. +Y is the row direction of the image (down
  // the image); it is also the flight direction. This is different
  // from Digital Globe model, but you can rotate pose beforehand.

  // Preview of the standard template arguments:
  //typedef LinescanDGModel<vw::camera::PiecewiseAPositionInterpolation,
  //                        vw::camera::LinearPiecewisePositionInterpolation,
  //                        vw::camera::SLERPPoseInterpolation,
  //                        vw::camera::TLCTimeInterpolation> DGCameraModel;

  // The useful load_dg_camera_model() function is at the end of the file.

  template <class PositionFuncT, class VelocityFuncT, class PoseFuncT, class TimeFuncT>
  class LinescanDGModel : public vw::camera::CameraModel {

  public:
    //------------------------------------------------------------------
    // Constructors / Destructors
    //------------------------------------------------------------------
    LinescanDGModel(PositionFuncT const& position,
		    VelocityFuncT const& velocity,
		    PoseFuncT     const& pose,
		    TimeFuncT     const& time,
		    vw::Vector2i  const& image_size,
		    vw::Vector2  const& detector_origin,
		    double focal_length,
		    bool   correct_velocity_aberration
		    ) :
      m_position_func(position), m_velocity_func(velocity),
      m_pose_func(pose), m_time_func(time),
      m_image_size(image_size), m_detector_origin(detector_origin),
      m_focal_length(focal_length),
      m_correct_velocity_aberration(correct_velocity_aberration){}

    virtual ~LinescanDGModel() {}
    virtual std::string type() const { return "LinescanDG"; }

    //------------------------------------------------------------------
    // Interface
    //------------------------------------------------------------------
    virtual vw::Vector2 point_to_pixel    (vw::Vector3 const& point) const;

    // Here we use an initial guess for the line number
    vw::Vector2 point_to_pixel            (vw::Vector3 const& point, double starty) const;
    vw::Vector2 point_to_pixel_uncorrected(vw::Vector3 const& point, double starty) const; ///< Never  corrects velocity aberration
    vw::Vector2 point_to_pixel_corrected  (vw::Vector3 const& point, double starty) const; ///< Always corrects velocity aberration

    /// Gives a pointing vector in the world coordinates.
    virtual vw::Vector3 pixel_to_vector(vw::Vector2 const& pix) const;

    /// Gives the camera position in world coordinates.
    virtual vw::Vector3 camera_center(vw::Vector2 const& pix ) const {
      return m_position_func( m_time_func( pix.y() ) );
    }

    /// Gives the camera velocity in world coordinates.
    vw::Vector3 camera_velocity(vw::Vector2 const& pix ) const {
      return m_velocity_func( m_time_func( pix.y() ) );
    }
    /// Gives a pose vector which represents the rotation from camera to world units
    virtual vw::Quat camera_pose(vw::Vector2 const& pix) const {
      return m_pose_func( m_time_func( pix.y() ) );
    }

    /// Create a fake pinhole model. It will return the same results
    /// as the linescan camera at current line y, but we will use it
    /// by extension at neighboring lines as well.
    vw::camera::PinholeModel linescan_to_pinhole(double y) const;

  public:
    // Extrinsics
    PositionFuncT m_position_func; // Function of time
    VelocityFuncT m_velocity_func; // Function of time
    PoseFuncT     m_pose_func;     // Function of time
    TimeFuncT     m_time_func;     // Function of line number

    // Intrinsics
    vw::Vector2i m_image_size;      // px
    vw::Vector2  m_detector_origin; // px
    double       m_focal_length;    // px

    bool m_correct_velocity_aberration;


  protected:
    // Levenberg Marquardt solver for linescan number
    //
    // We solve for the line number of the image that position the
    // camera so that the projection into the camera model actually
    // hits the detector. The detector is normally offset in the y
    // direction on the optical plane.
    class LinescanLMA : public vw::math::LeastSquaresModelBase<LinescanLMA> {
      const LinescanDGModel* m_model;
      vw::Vector3 m_point;
    public:
      typedef vw::Vector<double> result_type;   // 1D error on the optical plane.
      typedef result_type        domain_type;   // 1D linescan number
      typedef vw::Matrix<double> jacobian_type;

      LinescanLMA( const LinescanDGModel* model, const vw::Vector3& pt ) :
	m_model(model), m_point(pt) {}

      inline result_type operator()( domain_type const& y ) const;
    };

    // Levenberg Marquardt solver for linescan number (y) and pixel
    // number (x) for the given point in space. The obtained solution
    // pixel (x, y) must be such that the vector from this camera
    // pixel goes through the given point. The extra complication as
    // compared to LinescanLMA is the non-linear velocity aberration
    // correction in pixel_to_vector. This makes it for a more complex
    // equation and we need to solve for both x and y, rather than
    // just for y and getting x for free.
    class LinescanCorrLMA : public vw::math::LeastSquaresModelBase<LinescanCorrLMA> {
      const LinescanDGModel* m_model;
      vw::Vector3 m_point;
    public:
      typedef vw::Vector2 domain_type;     // 2D pixel, input to cost function vector
      typedef vw::Vector3 result_type;     // 3D error, output of cost function vector
      typedef vw::Matrix<double, 3, 2> jacobian_type;

      LinescanCorrLMA( const LinescanDGModel* model, const vw::Vector3& pt ) :
	m_model(model), m_point(pt) {}

      inline result_type operator()( domain_type const& pix ) const {
	return m_model->pixel_to_vector(pix) - normalize(m_point - m_model->camera_center(pix));
      }

    };

  }; // End class LinescanDGModel

  /// Currently this is the only variant of this we ever use
  typedef LinescanDGModel<vw::camera::PiecewiseAPositionInterpolation,
			  vw::camera::LinearPiecewisePositionInterpolation,
			  vw::camera::SLERPPoseInterpolation,
			  vw::camera::TLCTimeInterpolation> DGCameraModel;

  /// Load a DG camera model from an XML file.
  /// - This function does not take care of Xerces XML init/de-init, the caller must
  ///   make sure this is done before/after this function is called!
  inline boost::shared_ptr<DGCameraModel> load_dg_camera_model_from_xml(std::string const& path,
									bool correct_velocity_aberration);

#include <asp/Camera/LinescanDGModel.tcc>

}      // namespace asp

#endif//__STEREO_CAMERA_LINESCAN_DG_MODEL_H__
