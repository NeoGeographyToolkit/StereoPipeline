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

  template <class PositionFuncT, class VelocityFuncT, class PoseFuncT, class TimeFuncT>
  class LinescanDGModel : public vw::camera::CameraModel {
  protected:
    // Extrinsics
    PositionFuncT m_position_func; // Function of time
    VelocityFuncT m_velocity_func; // Function of time
    PoseFuncT m_pose_func;
    TimeFuncT m_time_func;         // Function of line number

    // Intrinsics
    vw::Vector2i m_image_size;     // px
    vw::Vector2 m_detector_origin; // px
    double m_focal_length;         // px

    bool m_correct_velocity_aberration;

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
      typedef vw::Vector<double> result_type; // 1D error on the optical plane.
      typedef result_type domain_type;        // 1D linescan number
      typedef vw::Matrix<double> jacobian_type;

      LinescanLMA( const LinescanDGModel* model, const vw::Vector3& pt ) :
        m_model(model), m_point(pt) {}

      inline result_type operator()( domain_type const& y ) const {
        double t = m_model->m_time_func( y[0] );

        // Rotate the point into our camera's frame
        vw::Vector3 pt = inverse( m_model->m_pose_func(t) ).rotate( m_point - m_model->m_position_func(t) );
        pt *= m_model->m_focal_length / pt.z(); // Rescale to pixel units
        result_type result(1);
        result[0] = pt.y() -
          m_model->m_detector_origin[1]; // Error against the location
                                         // of the detector
        return result;
      }

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

  public:
    //------------------------------------------------------------------
    // Constructors / Destructors
    //------------------------------------------------------------------
    LinescanDGModel(PositionFuncT const& position,
                    VelocityFuncT const& velocity,
                    PoseFuncT const& pose,
                    TimeFuncT const& time,
                    vw::Vector2i const& image_size,
                    vw::Vector2 const& detector_origin,
                    double focal_length,
                    bool correct_velocity_aberration
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
    virtual vw::Vector2 point_to_pixel(vw::Vector3 const& point) const {

      if (!m_correct_velocity_aberration) return point_to_pixel_uncorrected(point);
      return point_to_pixel_corrected(point);
    }

    vw::Vector2 point_to_pixel_uncorrected(vw::Vector3 const& point) const {

      using namespace vw;

      // Solve for the correct line number to use
      LinescanLMA model( this, point );
      int status;
      Vector<double> objective(1), start(1);
      start[0] = m_image_size.y()/2;
      Vector<double> solution =
        math::levenberg_marquardt( model, start, objective, status,
                                   1e-2, 1e-5, 1e3 );
      // The ending numbers define:
      //   Attempt to solve solution to 0.01 pixels.
      //   Give up with a relative change of 0.00001 pixels.
      //   Try with a max of a 1000 iterations.

      VW_ASSERT( status > 0,
                 camera::PointToPixelErr() << "Unable to project point into LinescanDG model" );

      // Solve for sample location
      double t = m_time_func( solution[0] );
      Vector3 pt = inverse( m_pose_func(t) ).rotate( point - m_position_func(t) );
      pt *= m_focal_length / pt.z();

      return vw::Vector2(pt.x() - m_detector_origin[0], solution[0]);
    }

    vw::Vector2 point_to_pixel_corrected(vw::Vector3 const& point) const {

      using namespace vw;

      LinescanCorrLMA model( this, point );
      int status;
      Vector2 start = point_to_pixel_uncorrected(point);

      Vector3 objective(0, 0, 0);
      // Need such tight tolerances below otherwise the solution is
      // inaccurate.
      Vector2 solution =
        math::levenberg_marquardt( model, start, objective, status,
                                   1e-10, 1e-10, 50 );
      VW_ASSERT( status > 0,
                 camera::PointToPixelErr() << "Unable to project point into LinescanDG model" );

      return solution;
    }

    // Gives a pointing vector in the world coordinates.
    virtual vw::Vector3 pixel_to_vector(vw::Vector2 const& pix) const {

      using namespace vw;

      double t = m_time_func( pix.y() );
      Vector3 pix_to_vec
        = normalize(m_pose_func( t ).rotate( vw::Vector3(pix[0]+m_detector_origin[0],
                                                         m_detector_origin[1],
                                                         m_focal_length) ) );

      if (!m_correct_velocity_aberration) return pix_to_vec;

      // Correct for velocity aberration

      // 1. Find the distance from the camera to the first
      // intersection of the current ray with the Earth surface.
      Vector3 cam_ctr          = camera_center(pix);
      double  earth_ctr_to_cam = norm_2(cam_ctr);
      double  cam_angle_cos    = dot_prod(pix_to_vec, -normalize(cam_ctr));
      double  len_cos          = earth_ctr_to_cam*cam_angle_cos;
      double  earth_rad        = 6371000.0;
      double  cam_to_surface   = len_cos -
        sqrt(earth_rad*earth_rad + len_cos*len_cos - earth_ctr_to_cam*earth_ctr_to_cam);

      // 2. Correct the camera velocity due to the fact that the Earth
      // rotates around its axis.
      double seconds_in_day = 86164.0905;
      Vector3 earth_rotation_vec(0.0, 0.0, 2*M_PI/seconds_in_day);
      Vector3 cam_vel = camera_velocity(pix);
      Vector3 cam_vel_corr1 = cam_vel - cam_to_surface * cross_prod(earth_rotation_vec, pix_to_vec);

      // 3. Find the component of the camera velocity orthogonal to the
      // direction the camera is pointing to.
      Vector3 cam_vel_corr2 = cam_vel_corr1 - dot_prod(cam_vel_corr1, pix_to_vec) * pix_to_vec;

      // 4. Correct direction for velocity aberration due to the speed of light.
      double light_speed = 299792458.0;
      Vector3 corr_pix_to_vec = pix_to_vec - cam_vel_corr2/light_speed;
      return normalize(corr_pix_to_vec);
    }

    // Gives the camera position in world coordinates.
    virtual vw::Vector3 camera_center(vw::Vector2 const& pix ) const {
      return m_position_func( m_time_func( pix.y() ) );
    }

    // Gives the camera velocity in world coordinates.
    vw::Vector3 camera_velocity(vw::Vector2 const& pix ) const {
      return m_velocity_func( m_time_func( pix.y() ) );
    }
    // Gives a pose vector which represents the rotation from camera to world units
    virtual vw::Quat camera_pose(vw::Vector2 const& pix) const {
      return m_pose_func( m_time_func( pix.y() ) );
    }

    vw::camera::PinholeModel linescan_to_pinhole(double y) const{

      // Create a fake pinhole model. It will return the same results
      // as the linescan camera at current line y, but we will use it
      // by extension at neighboring lines as well.
      double t = m_time_func( y );
      return vw::camera::PinholeModel(m_position_func(t),  m_pose_func(t).rotation_matrix(),
                                      m_focal_length, -m_focal_length,
                                      -m_detector_origin[0], y - m_detector_origin[1]
                                      );
    }

  };

  /// Currently this is the only variant of this we ever use
  typedef LinescanDGModel<vw::camera::PiecewiseAPositionInterpolation, 
                          vw::camera::LinearPiecewisePositionInterpolation, 
                          vw::camera::SLERPPoseInterpolation, 
                          vw::camera::TLCTimeInterpolation> DGCameraModel;


}      // namespace asp

#endif//__STEREO_CAMERA_LINESCAN_DG_MODEL_H__
