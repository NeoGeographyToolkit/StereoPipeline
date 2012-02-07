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

    // Extrinsics
    PositionFuncT m_position_func; // Function of time
    PoseFuncT m_pose_func;
    TimeFuncT m_time_func;         // Function of line number

    // Intrinsics
    vw::Vector2i m_image_size;     // px
    vw::Vector2 m_detector_origin; // px
    double m_focal_length;         // px

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
    virtual vw::Vector2 point_to_pixel(vw::Vector3 const& /*point*/) const {
      vw_throw( vw::NoImplErr() << "LinescanModel::point_to_pixel is not yet implemented." );
      return vw::Vector2(); // never reached
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
