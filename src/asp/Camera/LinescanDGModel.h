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

#include <vw/Camera/LinescanModel.h>
#include <vw/Camera/PinholeModel.h>
#include <vw/Camera/Extrinsics.h>

namespace asp {


  // The intrinisic model expects +Z to be point out the camera. +X is
  // the column direction of the image and is perpendicular to
  // direction of flight. +Y is the row direction of the image (down
  // the image); it is also the flight direction. This is different
  // from Digital Globe model, but you can rotate pose beforehand.

  // The standard class variant is:
  //      typedef LinescanDGModel<vw::camera::PiecewiseAPositionInterpolation,
  //                      			  vw::camera::SLERPPoseInterpolation> DGCameraModel;

  // The useful load_dg_camera_model() function is at the end of the file.

  /// Specialization of the generic LinescanModel for Digital Globe satellites.
  /// - Two template types are left floating so that AdjustedLinescanDGModel can modify them.
  template <class PositionFuncT, class PoseFuncT>
  class LinescanDGModel : public vw::camera::LinescanModel {
  public:
    //------------------------------------------------------------------
    // Constructors / Destructors
    //------------------------------------------------------------------
    LinescanDGModel(PositionFuncT const& position,
		                vw::camera::LinearPiecewisePositionInterpolation const& velocity,
	                  PoseFuncT     const& pose,
	                  vw::camera::TLCTimeInterpolation                 const& time,
	                  vw::Vector2i  const& image_size,
	                  vw::Vector2   const& detector_origin,
	                  double        const  focal_length
		    ) : vw::camera::LinescanModel(image_size, true), // Always correct velocity aberration
		        m_position_func(position), m_velocity_func(velocity),
            m_pose_func(pose),         m_time_func(time),
            m_detector_origin(detector_origin),
            m_focal_length(focal_length) {} 
    virtual ~LinescanDGModel() {}
    virtual std::string type() const { return "LinescanDG"; }

    // -- This set of functions implements virtual functions from LinescanModel.h --

    // Implement the functions from the LinescanModel class using functors
    virtual vw::Vector3 get_camera_center_at_time  (double time) const { return m_position_func(time); }
    virtual vw::Vector3 get_camera_velocity_at_time(double time) const { return m_velocity_func(time); }
    virtual vw::Quat    get_camera_pose_at_time    (double time) const { return m_pose_func    (time); }
    virtual double      get_time_at_line           (double line) const { return m_time_func    (line); }
    
    /// As pixel_to_vector, but in the local camera frame.
    virtual vw::Vector3 get_local_pixel_vector(vw::Vector2 const& pix) const;
    
    // Override this implementation with a faster, more specialized implemenation.
    virtual vw::Vector2 point_to_pixel(vw::Vector3 const& point, double starty) const;
    
    // -- These are new functions --
    
    double       get_focal_length   () const {return m_focal_length;   } ///< Returns the focal length in pixels
    vw::Vector2  get_detector_origin() const {return m_detector_origin;} ///< Returns the detector origin in pixels    
    
    /// Create a fake pinhole model. It will return the same results
    /// as the linescan camera at current line y, but we will use it
    /// by extension at neighboring lines as well.
    vw::camera::PinholeModel linescan_to_pinhole(double y) const;


    PositionFuncT const& get_position_func() const {return m_position_func;} ///< Access the position function
    vw::camera::LinearPiecewisePositionInterpolation
                  const& get_velocity_func() const {return m_velocity_func;} ///< Access the velocity function
    PoseFuncT     const& get_pose_func    () const {return m_pose_func;    } ///< Access the pose     function
    vw::camera::TLCTimeInterpolation
                  const& get_time_func    () const {return m_time_func;    } ///< Access the time     function

  private:

    // TODO: We should not need to redefine this here but some weird compile issue
    //       is making things crash if we don't.
    /// Returns the velocity corrected to account for the planetary rotation.
    /// - For efficiency, requires the uncorrected look vector at this location.
    virtual vw::Vector3 get_rotation_corrected_velocity(vw::Vector2 const& pixel,
                                                    vw::Vector3 const& uncorrected_vector) const {
      return vw::camera::LinescanModel::get_rotation_corrected_velocity(pixel, uncorrected_vector);
    }

  protected: // Functions
  
    /// Low accuracy function used by point_to_pixel to get a good solver starting seed.
    vw::Vector2 point_to_pixel_uncorrected(vw::Vector3 const& point, double starty) const;

  protected: // Variables
  
    // Extrinsics
    PositionFuncT                                    m_position_func; ///< Yields position at time T
    vw::camera::LinearPiecewisePositionInterpolation m_velocity_func; ///< Yields velocity at time T
    PoseFuncT                                        m_pose_func;     ///< Yields pose     at time T
    vw::camera::TLCTimeInterpolation                 m_time_func;     ///< Yields time at a given line.

    // Intrinsics
    
    /// Location of (0,0) coordinate of the detector relative to the center of
    ///  the origin of the camera coordinate system.
    /// - Stored internally in pixels.
    vw::Vector2  m_detector_origin; 
    double       m_focal_length;    ///< The focal length, also stored in pixels.


    // Levenberg Marquardt solver for linescan number
    //
    // We solve for the line number of the image that position the
    // camera so that the projection into the camera model actually
    // hits the detector. The detector is normally offset in the y
    // direction on the optical plane. Once we have the line we don't
    // need to use a solver to compute the sample.
    // - This solver is used by the point_to_pixel_uncorrected function.
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

  }; // End class LinescanDGModel


  /// This is the standard DG implementation
  typedef LinescanDGModel<vw::camera::PiecewiseAPositionInterpolation,
                  			  vw::camera::SLERPPoseInterpolation> DGCameraModel;

  /// Load a DG camera model from an XML file.
  /// - This function does not take care of Xerces XML init/de-init, the caller must
  ///   make sure this is done before/after this function is called!
  inline boost::shared_ptr<DGCameraModel> load_dg_camera_model_from_xml(std::string const& path);

}      // namespace asp

#include <asp/Camera/LinescanDGModel.tcc>


#endif//__STEREO_CAMERA_LINESCAN_DG_MODEL_H__
