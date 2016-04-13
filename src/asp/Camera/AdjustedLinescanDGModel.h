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

// TODO: Move this code to VW.
#ifndef __STEREO_CAMERA_ADJUSTED_LINESCAN_DG_MODEL_H__
#define __STEREO_CAMERA_ADJUSTED_LINESCAN_DG_MODEL_H__

#include <asp/Camera/LinescanDGModel.h>

namespace asp {

  enum PiecewiseAdjustmentInterpType {LinearInterp = 1, GaussianWeightsInterp};

  // This adjustment turned out to be necessary to deal with very subtle problems
  // with numerical precision, such as when we try to interpolate at position
  // say 7.00000000000000001 into an array whose indices are between 0 and 7.
  // The source of this problem is I think due to the fact that (a/b)*b can
  // be a tiny bit bigger than b sometimes. Such logic is used in get_tend()
  // for example.
  const double TINY_ADJ = 1.0 - 1.0e-10;

  // Modifying g_num_wts will require modifying a lot of code. We
  // assume that when doing smooth interpolation, only that many
  // neighboring grid points will be used.
  int g_num_wts = 4; 
  double g_sigma = 1.0; // The sigma of the Gaussian used for smooth interpolation

  // We would like to place num_adjustments between the first and last image
  // lines. Compute the starting line and the spacing.
  inline void compute_t0_dt(double beg_t, double end_t,
                            int num_adjustments,
                            double & y0, double & dt){

    VW_ASSERT( num_adjustments >= 2,
               vw::ArgumentErr() << "Expecting at least two adjustments.\n" );

    // For images scanned in reverse, beg_t can be > end_t
    if (beg_t > end_t)
      std::swap(beg_t, end_t);

    y0 = beg_t;
    dt = (end_t - beg_t)/(num_adjustments - 1.0);
  }

  // Return the closest piecewise adjustment camera indices to given time.
  std::vector<int> get_closest_adjusted_indices
  (vw::camera::LinearPiecewisePositionInterpolation const& linear_position_adjustments,
   vw::camera::SmoothPiecewisePositionInterpolation const& smooth_position_adjustments,
   PiecewiseAdjustmentInterpType interp_type,
   double t){
    
    if (interp_type == LinearInterp){

      double t0   = linear_position_adjustments.get_t0();
      double dt   = linear_position_adjustments.get_dt();
      double tend = linear_position_adjustments.get_tend();
      int    num  = round( (tend - t0)/dt ) + 1;

      double bound_t = t;
      bound_t = std::max(bound_t, linear_position_adjustments.get_t0());
      bound_t = std::min(bound_t, TINY_ADJ*linear_position_adjustments.get_tend());

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
    bound_t = std::max(bound_t, smooth_position_adjustments.get_t0());
    bound_t = std::min(bound_t, TINY_ADJ*smooth_position_adjustments.get_tend());

    // Return the indices of the largest weights used in interp.
    return smooth_position_adjustments.get_indices_of_largest_weights(bound_t);
  }
  
  class AdjustablePosition {
  public:
    AdjustablePosition(int interp_type,
                       vw::Vector2 const& adjustment_bounds,
                       std::vector<vw::Vector3> const& position_adjustments,
                       int num_wts, double sigma):
      m_interp_type(static_cast<PiecewiseAdjustmentInterpType>(interp_type)) {

      // We will need to be able to linearly interpolate into the adjustments.
      double t0, dt;
      compute_t0_dt(adjustment_bounds[0], adjustment_bounds[1],
		    position_adjustments.size(), t0, dt);

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
    vw::Vector3 operator()(double t) const {

      // The adjustments by design can be applied only to t
      // corresponding to between first and last lines, as that's
      // where the adjustments are placed. Anything beyond will have
      // to use the adjustments at endpoints.
      if (m_interp_type == LinearInterp){
        double bound_t = t;
        bound_t = std::max(bound_t, m_linear_position_adjustments.get_t0());
        bound_t = std::min(bound_t, TINY_ADJ*m_linear_position_adjustments.get_tend());
        return m_linear_position_adjustments(bound_t);
      }

      double bound_t = t;
      bound_t = std::max(bound_t, m_smooth_position_adjustments.get_t0());
      bound_t = std::min(bound_t, TINY_ADJ*m_smooth_position_adjustments.get_tend());
      return m_smooth_position_adjustments(bound_t);
    }

    // Return the closest piecewise adjustment camera indices to given time.
    std::vector<int> get_closest_adj_indices(double t){
      return get_closest_adjusted_indices(m_linear_position_adjustments,
					  m_smooth_position_adjustments,
					  m_interp_type, t);
    }

  private:
    PiecewiseAdjustmentInterpType m_interp_type;
    vw::camera::LinearPiecewisePositionInterpolation m_linear_position_adjustments;
    vw::camera::SmoothPiecewisePositionInterpolation m_smooth_position_adjustments;
  };

  class AdjustablePose {
  public:
    AdjustablePose(int interp_type,
                   vw::Vector2 const& adjustment_bounds,
                   std::vector<vw::Quat> const& pose_adjustments,
                   int num_wts, double sigma):
      m_interp_type(static_cast<PiecewiseAdjustmentInterpType>(interp_type)) {

      // We will need to be able to linearly interpolate into the adjustments.
      double t0, dt;
      compute_t0_dt(adjustment_bounds[0], adjustment_bounds[1],
		    pose_adjustments.size(), t0, dt);

      if (m_interp_type == LinearInterp){
        m_linear_pose_adjustments
          = vw::camera::SLERPPoseInterpolation(pose_adjustments, t0, dt);
      }else{
        m_smooth_pose_adjustments
          = vw::camera::SmoothSLERPPoseInterpolation(pose_adjustments, t0, dt,
                                                     num_wts, sigma);
      }
    }

    // Take the original rotation, and apply the adjustment on top of it. Both are
    // interpolated.
    vw::Quat operator()(double t) const {

      // The adjustments by design can be applied only to t
      // corresponding to between first and last lines, as that's
      // where the adjustments are placed. Anything beyond will have
      // to use the adjustments at endpoints.

      if (m_interp_type == LinearInterp){
        double bound_t = t;
        bound_t = std::max(bound_t, m_linear_pose_adjustments.get_t0());
        bound_t = std::min(bound_t, TINY_ADJ*m_linear_pose_adjustments.get_tend());
        return m_linear_pose_adjustments(bound_t);
      }

      double bound_t = t;
      bound_t = std::max(bound_t, m_smooth_pose_adjustments.get_t0());
      bound_t = std::min(bound_t, TINY_ADJ*m_smooth_pose_adjustments.get_tend());
      return m_smooth_pose_adjustments(bound_t);
    }

  private:
    PiecewiseAdjustmentInterpType m_interp_type;
    vw::camera::SLERPPoseInterpolation       m_linear_pose_adjustments;
    vw::camera::SmoothSLERPPoseInterpolation m_smooth_pose_adjustments;
  };

  // This class is similar to AdjustedCameraModel, which has one rotation
  // and translation adjustment for the camera, except that this one
  // has n adjustments, placed along certain rows in the image. It can
  // be used with any camera model, but it is meaningful when the
  // camera is linescan, hence this adjusts the satellite position and
  // pose as it moves.
  
  // A point X gets mapped by the pieceiwe adjusted camera at pixel pix as
  // m_adj_pose(pix.y()) * ( X - m_cam.camera_center(pix) ) 
  //    + m_cam.camera_center(pix) + m_adj_position(pix.y())  
  class PiecewiseAdjustedLinescanModel: public vw::camera::CameraModel {

  public:
    //------------------------------------------------------------------
    // Constructors / Destructors
    //------------------------------------------------------------------
    PiecewiseAdjustedLinescanModel(boost::shared_ptr<vw::camera::CameraModel> cam,
                                   int interp_type,
                                   vw::Vector2               const& adjustment_bounds,
                                   std::vector<vw::Vector3>  const& position_adjustments,
                                   std::vector<vw::Quat>     const& pose_adjustments,
                                   vw::Vector2i              const& image_size):
      m_adj_position(interp_type, adjustment_bounds, position_adjustments,
                     g_num_wts, g_sigma),
      m_adj_pose(interp_type, adjustment_bounds, pose_adjustments,
                 g_num_wts, g_sigma),
      // The line below is very important. We must make sure to keep track of
      // the smart pointer to the original camera, so it does not go out of scope.
      m_cam(cam), m_image_size(image_size)
    {
      VW_ASSERT( position_adjustments.size() == pose_adjustments.size(),
		 vw::ArgumentErr()
		 << "Expecting the number of position and pose adjustments to agree.\n" );
    }
    
    virtual ~PiecewiseAdjustedLinescanModel() {}
    
    virtual std::string type() const { return "PiecewiseAdjustedLinescanModel"; }
    
    // Need this function to decide which adjustments we need to vary
    // for the given line (image row) position.
    std::vector<int> get_closest_adj_indices(double line_pos){
      return m_adj_position.get_closest_adj_indices(line_pos);
    }
    
    vw::Vector3 pixel_to_vector(vw::Vector2 const& pix) const {
      return m_adj_pose(pix.y()).rotate(m_cam->pixel_to_vector(pix));
    }
    
    vw::Vector3 camera_center(vw::Vector2 const& pix) const {
      return m_cam->camera_center(pix) + m_adj_position(pix.y());
    }
    
    // The function point_to_pixel() is inherited from the base LinescanModel class.
    // TODO: Test that the right functions are called.
    
    vw::Vector2 point_to_pixel(vw::Vector3 const& point, double starty) const {
      // Use the generic solver to find the pixel 
      // - This method will be slower but works for more complicated geometries
      vw::camera::CameraGenericLMA model( this, point );
      int status;
      vw::Vector2 start = m_image_size / 2.0; // Use the center as the initial guess
      if (starty >= 0) // If the user provided a line number guess..
	start[1] = starty;
      
      // Solver constants
      const double ABS_TOL = 1e-16;
      const double REL_TOL = 1e-16;
      const int    MAX_ITERATIONS = 1e+5;
      
      vw::Vector3 objective(0, 0, 0);
      vw::Vector2 solution = vw::math::levenberg_marquardt(model, start, objective, status,
						   ABS_TOL, REL_TOL, MAX_ITERATIONS);
      VW_ASSERT( status > 0,
		 vw::camera::PointToPixelErr() << "Unable to project point into camera." );
      
      return solution;
    }

    vw::Vector2 point_to_pixel(vw::Vector3 const& point) const {
      return this->point_to_pixel(point, -1); // Redirect to the function with no guess
    }
    
  private:
    AdjustablePosition m_adj_position; 
    AdjustablePose m_adj_pose;
    boost::shared_ptr<vw::camera::CameraModel> m_cam;
    vw::Vector2i m_image_size;
  };
  
  // A little function to pull the true camera model.
  // TODO: What if we are dealing with an adjusted camera model?
  inline DGCameraModel* get_dg_ptr(boost::shared_ptr<vw::camera::CameraModel> cam){
    DGCameraModel * dg_cam = dynamic_cast<DGCameraModel*>(cam.get());
    if (dg_cam == NULL)
      vw_throw( vw::ArgumentErr() << "Expecting a DG camera.\n" );
    return dg_cam;
  }

  class AdjustableDGPosition {
  public:
    AdjustableDGPosition(DGCameraModel const* cam_ptr,
                       int interp_type,
                       vw::Vector2 const& adjustment_bounds,
                       std::vector<vw::Vector3> const& position_adjustments,
                       int num_wts, double sigma):
      m_cam_ptr(cam_ptr),
      m_interp_type(static_cast<PiecewiseAdjustmentInterpType>(interp_type)) {

      // We will need to be able to linearly interpolate into the adjustments.
      double t0, dt;
      compute_t0_dt(cam_ptr->get_time_at_line(adjustment_bounds[0]),
		    cam_ptr->get_time_at_line(adjustment_bounds[1]),
		    position_adjustments.size(), t0, dt);
      
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
    vw::Vector3 operator()(double t) const {

      // The adjustments by design can be applied only to t
      // corresponding to between first and last lines, as that's
      // where the adjustments are placed. Anything beyond will have
      // to use the adjustments at endpoints.
      if (m_interp_type == LinearInterp){
        double bound_t = t;
        bound_t = std::max(bound_t, m_linear_position_adjustments.get_t0());
        bound_t = std::min(bound_t, TINY_ADJ*m_linear_position_adjustments.get_tend());
        return m_linear_position_adjustments(bound_t) + m_cam_ptr->get_camera_center_at_time(t);
      }

      double bound_t = t;
      bound_t = std::max(bound_t, m_smooth_position_adjustments.get_t0());
      bound_t = std::min(bound_t, TINY_ADJ*m_smooth_position_adjustments.get_tend());
      return m_smooth_position_adjustments(bound_t) + m_cam_ptr->get_camera_center_at_time(t);
    }

    // Return the closest piecewise adjustment camera indices to given time.
    std::vector<int> get_closest_adj_indices(double t){
      return get_closest_adjusted_indices(m_linear_position_adjustments,
					  m_smooth_position_adjustments,
					  m_interp_type, t);
    }
    
  private:
    DGCameraModel const* m_cam_ptr;
    PiecewiseAdjustmentInterpType m_interp_type;
    vw::camera::LinearPiecewisePositionInterpolation m_linear_position_adjustments;
    vw::camera::SmoothPiecewisePositionInterpolation m_smooth_position_adjustments;
  };

  class AdjustableDGPose {
  public:
    AdjustableDGPose(DGCameraModel const* cam_ptr,
                   int interp_type,
                   vw::Vector2 const& adjustment_bounds,
                   std::vector<vw::Quat> const& pose_adjustments,
                   int num_wts, double sigma):
      m_cam_ptr(cam_ptr),
      m_interp_type(static_cast<PiecewiseAdjustmentInterpType>(interp_type)) {

      // We will need to be able to linearly interpolate into the adjustments.
      double t0, dt;
      compute_t0_dt(cam_ptr->get_time_at_line(adjustment_bounds[0]),
		    cam_ptr->get_time_at_line(adjustment_bounds[1]),
		    pose_adjustments.size(), t0, dt);
      
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
    vw::Quat operator()(double t) const {

      // The adjustments by design can be applied only to t
      // corresponding to between first and last lines, as that's
      // where the adjustments are placed. Anything beyond will have
      // to use the adjustments at endpoints.

      if (m_interp_type == LinearInterp){
        double bound_t = t;
        bound_t = std::max(bound_t, m_linear_pose_adjustments.get_t0());
        bound_t = std::min(bound_t, TINY_ADJ*m_linear_pose_adjustments.get_tend());
        return m_linear_pose_adjustments(bound_t) * m_cam_ptr->get_camera_pose_at_time(t);
      }

      double bound_t = t;
      bound_t = std::max(bound_t, m_smooth_pose_adjustments.get_t0());
      bound_t = std::min(bound_t, TINY_ADJ*m_smooth_pose_adjustments.get_tend());
      return m_smooth_pose_adjustments(bound_t) * m_cam_ptr->get_camera_pose_at_time(t);
    }

  private:
    DGCameraModel const* m_cam_ptr;
    PiecewiseAdjustmentInterpType m_interp_type;
    vw::camera::SLERPPoseInterpolation       m_linear_pose_adjustments;
    vw::camera::SmoothSLERPPoseInterpolation m_smooth_pose_adjustments;
  };

  // This class will have adjustable position and pose. Those are obtained by applying
  // adjustments to given DG camera position and pose. The adjustable position and pose
  // implement operator() so can be invoked exactly as the original position
  // and pose. Note that we don't adjust the velocity, maybe we should.
  // This is a version of PiecewiseAdjustedLinescanModel tuned for DG.
  // TODO: Study whether this new class or the original perform better for DG
  // (a lot of work).
  class AdjustedLinescanDGModel:
    public LinescanDGModel<AdjustableDGPosition, AdjustableDGPose>
  {      
    
  public:
    //------------------------------------------------------------------
    // Constructors / Destructors
    //------------------------------------------------------------------
    AdjustedLinescanDGModel(boost::shared_ptr<vw::camera::CameraModel> cam,
                            int interp_type,
                            vw::Vector2 const& adjustment_bounds,
                            std::vector<vw::Vector3> const& position_adjustments,
                            std::vector<vw::Quat>    const& pose_adjustments,
			    vw::Vector2i              const& image_size):
      // Initialize the base
      LinescanDGModel<AdjustableDGPosition, AdjustableDGPose>
    (AdjustableDGPosition(get_dg_ptr(cam), interp_type, adjustment_bounds,
			  position_adjustments, g_num_wts, g_sigma),
     get_dg_ptr(cam)->get_velocity_func(),
     AdjustableDGPose(get_dg_ptr(cam), interp_type, adjustment_bounds,
		      pose_adjustments, g_num_wts, g_sigma),
     get_dg_ptr(cam)->get_time_func(),
     get_dg_ptr(cam)->get_image_size(),
     get_dg_ptr(cam)->get_detector_origin(),
     get_dg_ptr(cam)->get_focal_length()),
      // The line below is very important. We must make sure to keep track of
      // the smart pointer to the original camera, so it does not go out of scope.
      m_cam(cam), m_image_size(image_size)
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
    vw::Vector2i m_image_size;

  };

}      // namespace asp

#endif//__STEREO_CAMERA_ADJUSTED_LINESCAN_DG_MODEL_H__
