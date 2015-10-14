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

  // A little function to pull the true camera model
  inline DGCameraModel* get_dg_ptr(boost::shared_ptr<vw::camera::CameraModel> cam){
    DGCameraModel * dg_cam = dynamic_cast<DGCameraModel*>(cam.get());
    if (dg_cam == NULL)
      vw_throw( vw::ArgumentErr() << "Expecting a DG camera.\n" );
    return dg_cam;
  }

  // We would like to place num_adjustments between the first and last image
  // lines. Compute the starting time for that and the spacing.
  void compute_t0_dt(DGCameraModel const* cam_ptr,
		     int num_adjustments,
		     double & t0, double & dt){

    VW_ASSERT( num_adjustments >= 2,
	       vw::ArgumentErr() << "Expecting at least two adjustments.\n" );

    int num_image_lines = cam_ptr->m_image_size[1];

    // Note we compute beg_t and end_t at -1 and num_image_lines
    // rather than at the first and last line, 0 and
    // num_image_lines-1. This is intentional. Otherwise
    // LinearPiecewisePositionInterpolation and SLERPPoseInterpolation
    // may fail at line number 0 or num_image_lines as it always
    // expects us to not try to interpolate at endpoints.
    double beg_t = cam_ptr->m_time_func(-1);
    double end_t = cam_ptr->m_time_func(num_image_lines);

    //std::cout << "--beg and end t " << beg_t << ' ' << end_t << std::endl;

    // For images scanned in reverse, beg_t can be > end_t
    if (beg_t > end_t)
      std::swap(beg_t, end_t);

    //std::cout << "2beg and end t " << beg_t << ' ' << end_t << std::endl;

    t0 = beg_t;
    dt = (end_t - beg_t)/(num_adjustments - 1.0);

    //std::cout << "---num image lines " << num_image_lines << std::endl;
    //std::cout << "num adj is " << num_adjustments << std::endl;
    //std::cout << "t0 is " << t0 << std::endl;
    //std::cout << "--dt is " << dt << std::endl;
  }

  // Position adjustment. Note that the position class itself is of
  // type PiecewiseAPositionInterpolation (uses velocity) though the
  // adjustments themselves are, as expected, of the simpler
  // LinearPiecewisePositionInterpolation type.
  inline vw::camera::LinearPiecewisePositionInterpolation
  set_position_adjustments(DGCameraModel const* cam_ptr,
			   std::vector<vw::Vector3> const& position_adjustments){

    // We will need to be able to linearly interpolate into the adjustments.
    double t0, dt;
    compute_t0_dt(cam_ptr, position_adjustments.size(), t0, dt);
    return vw::camera::LinearPiecewisePositionInterpolation(position_adjustments, t0, dt);
  }

  class AdjustablePosition {
  public:
    AdjustablePosition(DGCameraModel const* cam_ptr,
		       std::vector<vw::Vector3> const& position_adjustments):
      m_cam_ptr(cam_ptr),
      m_position_adjustments(set_position_adjustments(cam_ptr,
						      position_adjustments)) {}

    // Return the original position plus the interpolated adjustment.
    vw::Vector3 operator()( double t ) const {
      return m_cam_ptr->m_position_func(t) + m_position_adjustments(t);
    }

    // Return the closest piecewise adjustment camera indices to given time.
    std::vector<int> get_closest_adj_indices(double t){
      double t0   = m_position_adjustments.get_t0();
      double dt   = m_position_adjustments.get_dt();
      double tend = m_position_adjustments.get_tend();

      //std::cout << "t0 t dt tend " << t0 << ' ' << t << ' ' << dt << ' ' << tend << std::endl;

      int    num  = round( (tend - t0)/dt ) + 1;

      //std::cout << "--num is " << num << std::endl;

      double ratio = (t-t0)/dt;

      // if ratio == 7.2, return 6, 7, 8
      // if ratio == 7.9, return 7, 8, 9
      int i0 = floor(ratio - 0.5);
      int i1 = i0 + 1;
      int i2 = i1 + 1;

      //std::cout << "--ratio is " << ratio  << std::endl;

      std::vector<int> indices;
      if (i0 >= 0 && i0 < num) indices.push_back(i0);
      if (i1 >= 0 && i1 < num) indices.push_back(i1);
      if (i2 >= 0 && i2 < num) indices.push_back(i2);

      return indices;
    }


  private:
    DGCameraModel const* m_cam_ptr;
    vw::camera::LinearPiecewisePositionInterpolation m_position_adjustments;
  };

  // Pose adjustment
  inline vw::camera::SLERPPoseInterpolation
  set_pose_adjustments(DGCameraModel const* cam_ptr,
		       std::vector<vw::Quat> const& pose_adjustments){

    // We will need to be able to linearly interpolate into the adjustments.
    double t0, dt;
    compute_t0_dt(cam_ptr, pose_adjustments.size(), t0, dt);
    return vw::camera::SLERPPoseInterpolation(pose_adjustments, t0, dt);
  }

  class AdjustablePose {
  public:
    AdjustablePose(DGCameraModel const* cam_ptr,
		   std::vector<vw::Quat> const& pose_adjustments):
      m_cam_ptr(cam_ptr),
      m_pose_adjustments(set_pose_adjustments(m_cam_ptr,
					      pose_adjustments)) {}

    // Take the original rotation, and apply the adjustment on top of it. Both are
    // interpolated.
    vw::Quat operator()( double t ) const {
      return  m_cam_ptr->m_pose_func(t)*m_pose_adjustments(t);
    }

  private:
    DGCameraModel const* m_cam_ptr;
    vw::camera::SLERPPoseInterpolation m_pose_adjustments;
  };

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
			    std::vector<vw::Vector3> const& position_adjustments,
			    std::vector<vw::Quat>    const& pose_adjustments):
      // Initialize the base
      LinescanDGModel<AdjustablePosition,
		      vw::camera::LinearPiecewisePositionInterpolation,
		      AdjustablePose,
		      vw::camera::TLCTimeInterpolation>
    (AdjustablePosition(get_dg_ptr(cam), position_adjustments),
     get_dg_ptr(cam)->m_velocity_func,
     AdjustablePose(get_dg_ptr(cam), pose_adjustments),
     get_dg_ptr(cam)->m_time_func,
     get_dg_ptr(cam)->m_image_size,
     get_dg_ptr(cam)->m_detector_origin,
     get_dg_ptr(cam)->m_focal_length,
     get_dg_ptr(cam)->m_correct_velocity_aberration),
      // The line below is very important. We must make sure to keep track of
      // the smart pointer to the original camera, so it does not go out of scope.
      m_cam(cam)
    {

      //std::cout << "sensor columns " << get_dg_ptr(cam)->m_image_size[0] << std::endl;

      //std::cout << "sensor lines: " << get_dg_ptr(cam)->m_image_size[1] << std::endl;
      //std::cout << "starting time " << get_dg_ptr(cam)->m_time_func(0) << std::endl;
      //std::cout << "--ending time " <<  get_dg_ptr(cam)->m_time_func(get_dg_ptr(cam)->m_image_size[1]) << std::endl;

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
      //std::cout << "y and t is " << line_pos << ' ' << t << std::endl;

      return m_position_func.get_closest_adj_indices(t);
    }

  private:
    boost::shared_ptr<vw::camera::CameraModel> m_cam;

  };

}      // namespace asp

#endif//__STEREO_CAMERA_ADJUSTED_LINESCAN_DG_MODEL_H__
