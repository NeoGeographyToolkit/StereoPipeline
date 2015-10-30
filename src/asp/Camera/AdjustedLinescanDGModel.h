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


/// \file AdjustedLinescanDGModel.h
///
/// Apply piecewise linear position and rotation adjustments to a LinescanDGModel.
/// The adjustments are placed along-track (along the central image column).
/// The original model is not duplicated, just a pointer to it is retained.
/// The user decides how many adjustments to create has to populate them.
/// This function manages their placement and applies the adjustments.

#ifndef __STEREO_CAMERA_ADJUSTED_LINESCAN_DG_MODEL_H__
#define __STEREO_CAMERA_ADJUSTED_LINESCAN_DG_MODEL_H__

#include <asp/Camera/LinescanDGModel.h>

namespace asp {

  enum InterpType {LinearInterp = 1, GaussianWeightsInterp};

  // A little function to pull the true camera model.
  // TODO: What if we are dealing with an adjusted camera model?
  inline DGCameraModel* get_dg_ptr(boost::shared_ptr<vw::camera::CameraModel> cam){
    DGCameraModel * dg_cam = dynamic_cast<DGCameraModel*>(cam.get());
    if (dg_cam == NULL)
      vw_throw( vw::ArgumentErr() << "Expecting a DG camera.\n" );
    return dg_cam;
  }

  // We would like to place num_adjustments between the first and last image
  // lines. Compute the starting time for that and the spacing.
  inline void compute_t0_dt(DGCameraModel const* cam_ptr,
                            vw::Vector2 const& adjustment_bounds,
                            int num_adjustments,
                            double & t0, double & dt){

    VW_ASSERT( num_adjustments >= 2,
               vw::ArgumentErr() << "Expecting at least two adjustments.\n" );

    // compute the times at the bounds between which we want to place
    // the adjustments.
    double beg_t = cam_ptr->m_time_func(adjustment_bounds[0]);
    double end_t = cam_ptr->m_time_func(adjustment_bounds[1]);

    // For images scanned in reverse, beg_t can be > end_t
    if (beg_t > end_t)
      std::swap(beg_t, end_t);

    t0 = beg_t;
    dt = (end_t - beg_t)/(num_adjustments - 1.0);
  }

  // This adjustment turned out to be necessary to deal with very subtle problems
  // with numerical precision, such as when we try to interpolate at position
  // say 7.00000000000000001 into an array whose indices are between 0 and 7.
  // The source of this problem is I think due to the fact that (a/b)*b can
  // be a tiny bit bigger than b sometimes. Such logic is used in get_tend()
  // for example.
  const double TINY_ADJ = 1.0 - 1.0e-10;

  class AdjustablePosition {
  public:
    AdjustablePosition(DGCameraModel const* cam_ptr,
                       int interp_type,
                       vw::Vector2 const& adjustment_bounds,
                       std::vector<vw::Vector3> const& position_adjustments,
                       int num_wts, double sigma):
      m_cam_ptr(cam_ptr),
      m_interp_type(static_cast<InterpType>(interp_type)) {

      // We will need to be able to linearly interpolate into the adjustments.
      double t0, dt;
      compute_t0_dt(cam_ptr, adjustment_bounds, position_adjustments.size(), t0, dt);

      // Linear interp
      if (m_interp_type == LinearInterp){
        m_linear_position_adjustments
          = vw::camera::LinearPiecewisePositionInterpolation(position_adjustments, t0, dt);
      }else{
        // Smooth interp
        m_smooth_position_adjustments
          = vw::camera::SmoothPiecewisePositionInterpolation(position_adjustments,
                                                             t0, dt, num_wts, sigma);
      }
    }

    // Return the original position plus the interpolated adjustment.
    vw::Vector3 operator()( double t ) const {

      // The adjustments by design can be applied only to t
      // corresponding to between first and last lines, as that's
      // where the adjustments are placed. Anything beyond will have
      // to use the adjustments at endpoints.
      if (m_interp_type == LinearInterp){
        double bound_t = t;
        bound_t = std::max(bound_t, m_linear_position_adjustments.get_t0());
        bound_t = std::min(bound_t, TINY_ADJ*m_linear_position_adjustments.get_tend());
        return m_linear_position_adjustments(bound_t) + m_cam_ptr->m_position_func(t);
      }

      double bound_t = t;
      bound_t = std::max(bound_t, m_smooth_position_adjustments.get_t0());
      bound_t = std::min(bound_t, TINY_ADJ*m_smooth_position_adjustments.get_tend());
      return m_smooth_position_adjustments(bound_t) + m_cam_ptr->m_position_func(t);
    }

    // Return the closest piecewise adjustment camera indices to given time.
    std::vector<int> get_closest_adj_indices(double t){

      if (m_interp_type == LinearInterp){

        double t0   = m_linear_position_adjustments.get_t0();
        double dt   = m_linear_position_adjustments.get_dt();
        double tend = m_linear_position_adjustments.get_tend();
        int    num  = round( (tend - t0)/dt ) + 1;

        double bound_t = t;
        bound_t = std::max(bound_t, m_linear_position_adjustments.get_t0());
        bound_t = std::min(bound_t, TINY_ADJ*m_linear_position_adjustments.get_tend());

        double ratio = (bound_t-t0)/dt;

        // if ratio == 7.2, return 6, 7, 8
        // if ratio == 7.9, return 7, 8, 9
        int i0 = floor(ratio - 0.5);
        int i1 = i0 + 1;
        int i2 = i1 + 1;

        std::vector<int> indices;
        if (i0 >= 0 && i0 < num) indices.push_back(i0);
        if (i1 >= 0 && i1 < num) indices.push_back(i1);
        if (i2 >= 0 && i2 < num) indices.push_back(i2);

        return indices;
      }

      double bound_t = t;
      bound_t = std::max(bound_t, m_smooth_position_adjustments.get_t0());
      bound_t = std::min(bound_t, TINY_ADJ*m_smooth_position_adjustments.get_tend());

      // Return the indices of the largest weights used in interp.
      return m_smooth_position_adjustments.get_indices_of_largest_weights(bound_t);
    }

  private:
    DGCameraModel const* m_cam_ptr;
    InterpType m_interp_type;
    vw::camera::LinearPiecewisePositionInterpolation m_linear_position_adjustments;
    vw::camera::SmoothPiecewisePositionInterpolation m_smooth_position_adjustments;
  };

  class AdjustablePose {
  public:
    AdjustablePose(DGCameraModel const* cam_ptr,
                   int interp_type,
                   vw::Vector2 const& adjustment_bounds,
                   std::vector<vw::Quat> const& pose_adjustments,
                   int num_wts, double sigma):
      m_cam_ptr(cam_ptr),
      m_interp_type(static_cast<InterpType>(interp_type)) {

      // We will need to be able to linearly interpolate into the adjustments.
      double t0, dt;
      compute_t0_dt(m_cam_ptr, adjustment_bounds, pose_adjustments.size(), t0, dt);

      if (m_interp_type == LinearInterp){
        m_linear_pose_adjustments
          = vw::camera::SLERPPoseInterpolation(pose_adjustments, t0, dt);
      }else{
        m_smooth_pose_adjustments
          = vw::camera::SmoothSLERPPoseInterpolation(pose_adjustments, t0, dt, num_wts, sigma);
      }
    }

    // Take the original rotation, and apply the adjustment on top of it. Both are
    // interpolated.
    vw::Quat operator()( double t ) const {

      // The adjustments by design can be applied only to t
      // corresponding to between first and last lines, as that's
      // where the adjustments are placed. Anything beyond will have
      // to use the adjustments at endpoints.

      if (m_interp_type == LinearInterp){
        double bound_t = t;
        bound_t = std::max(bound_t, m_linear_pose_adjustments.get_t0());
        bound_t = std::min(bound_t, TINY_ADJ*m_linear_pose_adjustments.get_tend());
        return m_linear_pose_adjustments(bound_t) * m_cam_ptr->m_pose_func(t);
      }

      double bound_t = t;
      bound_t = std::max(bound_t, m_smooth_pose_adjustments.get_t0());
      bound_t = std::min(bound_t, TINY_ADJ*m_smooth_pose_adjustments.get_tend());
      return m_smooth_pose_adjustments(bound_t) * m_cam_ptr->m_pose_func(t);
    }

  private:
    DGCameraModel const* m_cam_ptr;
    InterpType m_interp_type;
    vw::camera::SLERPPoseInterpolation       m_linear_pose_adjustments;
    vw::camera::SmoothSLERPPoseInterpolation m_smooth_pose_adjustments;
  };

  int g_num_wts = 4; // Modifying here will require modifying a lot of code
  double g_sigma = 1.0;

  // This class will have adjustable position and pose. Those are obtained by applying
  // adjustments to given position and pose. The adjustable position and pose
  // implement operator() so can be invoked exactly as the original position
  // and pose. Note that we don't adjust the velocity, maybe we should.
  class AdjustedLinescanDGModel:
    public LinescanDGModel<AdjustablePosition,                                 // position
                           vw::camera::LinearPiecewisePositionInterpolation,   // velocity
                           AdjustablePose,                                     // pose
                           vw::camera::TLCTimeInterpolation> {

  public:
    //------------------------------------------------------------------
    // Constructors / Destructors
    //------------------------------------------------------------------
    AdjustedLinescanDGModel(boost::shared_ptr<vw::camera::CameraModel> cam,
                            int interp_type,
                            vw::Vector2 const& adjustment_bounds,
                            std::vector<vw::Vector3> const& position_adjustments,
                            std::vector<vw::Quat>    const& pose_adjustments):
      // Initialize the base
      LinescanDGModel<AdjustablePosition,
                      vw::camera::LinearPiecewisePositionInterpolation,
                      AdjustablePose,
                      vw::camera::TLCTimeInterpolation>
    (AdjustablePosition(get_dg_ptr(cam), interp_type, adjustment_bounds, position_adjustments, g_num_wts, g_sigma),
     get_dg_ptr(cam)->m_velocity_func,
     AdjustablePose(get_dg_ptr(cam), interp_type, adjustment_bounds, pose_adjustments, g_num_wts, g_sigma),
     get_dg_ptr(cam)->m_time_func,
     get_dg_ptr(cam)->m_image_size,
     get_dg_ptr(cam)->m_detector_origin,
     get_dg_ptr(cam)->m_focal_length,
     get_dg_ptr(cam)->m_correct_velocity_aberration),
      // The line below is very important. We must make sure to keep track of
      // the smart pointer to the original camera, so it does not go out of scope.
      m_cam(cam)
    {

      VW_ASSERT( position_adjustments.size() == pose_adjustments.size(),
                 vw::ArgumentErr()
                 << "Expecting the number of position and pose adjustments to agree.\n" );

    }

    virtual ~AdjustedLinescanDGModel() {}

    virtual std::string type() const { return "AdjustedLinescanDG"; }

    // Need this function to decide which adjustments we need to vary
    // for the given line (image row) position.
    std::vector<int> get_closest_adj_indices(double line_pos){
      double t = m_time_func(line_pos);
      return m_position_func.get_closest_adj_indices(t);
    }

  private:
    boost::shared_ptr<vw::camera::CameraModel> m_cam;

  };

}      // namespace asp

#endif//__STEREO_CAMERA_ADJUSTED_LINESCAN_DG_MODEL_H__
