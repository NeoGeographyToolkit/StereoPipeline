// __BEGIN_LICENSE__
// Copyright (C) 2006-2011 United States Government as represented by
// the Administrator of the National Aeronautics and Space Administration.
// All Rights Reserved.
// __END_LICENSE__


/// \file LinescanModel.h
///
/// A generic linescan camera model object
///
///
#ifndef __STEREO_SESSION_DG_LINESCAN_DG_MODEL_H__
#define __STEREO_SESSION_DG_LINESCAN_DG_MODEL_H__

#include <vw/Math/Quaternion.h>
#include <vw/Camera/CameraModel.h>

namespace asp {

  // This is potentially a more generic line scan camera model that
  // allows a offset ine the line direction. This also allows a
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

  template <class PositionFuncT, class PoseFuncT, class TimeFuncT>
  class LinescanDGModel : public vw::camera::CameraModel {
  protected:
    // Extrinsics
    PositionFuncT m_position_func; // Function of time
    PoseFuncT m_pose_func;
    TimeFuncT m_time_func;         // Function of line number

    // Intrinsics
    vw::Vector2i m_image_size;     // px
    vw::Vector2 m_detector_origin; // px
    double m_focal_length;         // px

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
      typedef vw::Vector<double> result_type; // 1 D error on the optical plane.
      typedef result_type domain_type;        // 1 D linescan number
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

  public:
    //------------------------------------------------------------------
    // Constructors / Destructors
    //------------------------------------------------------------------
    LinescanDGModel( PositionFuncT const& position,
                     PoseFuncT const& pose,
                     TimeFuncT const& time,
                     vw::Vector2i const& image_size,
                     vw::Vector2 const& detector_origin,
                     double focal_length ) :
      m_position_func(position), m_pose_func(pose), m_time_func(time),
      m_image_size(image_size), m_detector_origin(detector_origin),
      m_focal_length(focal_length) {}

    virtual ~LinescanDGModel() {}
    virtual std::string type() const { return "LinescanDG"; }

    //------------------------------------------------------------------
    // Interface
    //------------------------------------------------------------------
    virtual vw::Vector2 point_to_pixel(vw::Vector3 const& point) const {
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

    // Gives a pointing vector in the world coordinates.
    virtual vw::Vector3 pixel_to_vector(vw::Vector2 const& pix) const {
      double t = m_time_func( pix.y() );
      return normalize(m_pose_func( t ).rotate( vw::Vector3(pix[0]+m_detector_origin[0],
                                                            m_detector_origin[1],
                                                            m_focal_length) ) );
    }

    // Gives a position in world coordinates.
    virtual vw::Vector3 camera_center(vw::Vector2 const& pix ) const {
      return m_position_func( m_time_func( pix.y() ) );
    }

    // Gives a pose vector which represents the rotation from camera to world units
    virtual vw::Quat camera_pose(vw::Vector2 const& pix) const {
      return m_pose_func( m_time_func( pix.y() ) );
    }
  };

}      // namespace asp

#endif//__STEREO_SESSION_DG_LINESCAN_DG_MODEL_H__
