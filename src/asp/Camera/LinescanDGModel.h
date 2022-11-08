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

#include <asp/Camera/TimeProcessing.h>
#include <vw/Camera/CameraSolve.h>
#include <vw/Camera/LinescanModel.h>
#include <vw/Camera/Extrinsics.h>
#include <vw/Camera/PinholeModel.h>
#include <vw/Math/EulerAngles.h>

// TODO(oalexan1): Temporary! These will be moved to .cc
#include <asp/Camera/CsmModel.h>
#include <asp/Sessions/StereoSession.h>
#include <usgscsm/UsgsAstroLsSensorModel.h>

namespace asp {

  // The intrinisic model expects +Z to be point out the camera. +X is
  // the column direction of the image and is perpendicular to
  // direction of flight. +Y is the row direction of the image (down
  // the image); it is also the flight direction. This is different
  // from Digital Globe model, but you can rotate pose beforehand.

  // Specialization of the generic LinescanModel for Digital Globe
  // satellites.  Class AdjustedLinescanDGModel inherits from this.

  // Note: Correcting for atmospheric refraction and velocity
  // aberration greatly increases the run-time of ground-to-image logic.
  
  // TODO(oalexan1): Study the effect of other ways of doing position
  // and pose interpolation. These are 
  template <class PositionFuncT, class VelocityFuncT, class PoseFuncT>
  class LinescanDGModel : public vw::camera::LinescanModel {
  public:

    // TODO(oalexan1): This is temporary logic. Eventually this class will
    // replace LinescanDGModel, will be de-templated, moved to .cc,
    // and the class PiecewiseAdjustedLinescanModel will go away.
    struct DgCsmModel: public CsmModel {
      
      DgCsmModel() {}
      void populateCsmModel(LinescanDGModel * dg_model) {
        m_desired_precision = 1.0e-12;
        vw::cartography::Datum datum("WGS84"); // this sensor is used for Earth only
        m_semi_major_axis = datum.semi_major_axis();
        m_semi_minor_axis = datum.semi_minor_axis();

        // Create a linescan model as a smart pointer, and do smart pointer casting
        m_csm_model.reset(new UsgsAstroLsSensorModel); // follow CSM api, type is csm::RasterGM
        m_ls_model = boost::dynamic_pointer_cast<UsgsAstroLsSensorModel>(m_csm_model);
        if (m_ls_model == NULL)
          vw::vw_throw(vw::ArgumentErr() << "Invalid initialization of the linescan model.\n");
        
        // This performs many initializations apart from the above
        m_ls_model->reset();
        
        m_ls_model->m_nSamples = dg_model->m_image_size[0]; 
        m_ls_model->m_nLines   = dg_model->m_image_size[1];

        m_ls_model->m_platformFlag = 1; // For order 8 Lagrange interpolation
        m_ls_model->m_maxElevation =  10000.0; //  10 km
        m_ls_model->m_minElevation = -10000.0; // -10 km
        m_ls_model->m_focalLength  = 1.0;
        m_ls_model->m_zDirection   = 1.0;
        m_ls_model->m_halfSwath    = 1.0;
        m_ls_model->m_sensorIdentifier = "DigitalGlobeLinescan";
        m_ls_model->m_majorAxis = m_semi_major_axis;
        m_ls_model->m_minorAxis = m_semi_minor_axis;
  
        m_ls_model->m_iTransL[0]   = 0.0;  
        m_ls_model->m_iTransL[1]   = 1.0; // no scale
        m_ls_model->m_iTransL[2]   = 0.0; // no skew
        m_ls_model->m_iTransS[0]   = 0.0;
        m_ls_model->m_iTransS[1]   = 0.0; // no skew
        m_ls_model->m_iTransS[2]   = 1.0; // no scale
        m_ls_model->m_detectorLineOrigin   = 0.0;
        m_ls_model->m_detectorSampleOrigin = 0.0;
        
        // Quantities needed to find the ray direction in the sensor plane.
        // This needs to be consistent with usgscsm functions
        // computeDistortedFocalPlaneCoordinates() and
        // createCameraLookVector(), which requires a lot of care.
        // Need to emulate this
        // double x = m_coeff_psi_x[0] + m_coeff_psi_x[1] * (col  + m_ref_col);
        // double y = m_coeff_psi_y[0] + m_coeff_psi_y[1] * (col  + m_ref_col);
        // Using this:
        // double detSample = (col + 0.5) * sampleSumming + startingSample;
        // double detLine = line * lineSumming + startingLine; // but it will use line = 0
        vw::Vector2 m_coeff_psi_x(0, 0), m_coeff_psi_y(0, 0); // temporary
        double m_ref_col = 0;
        m_ls_model->m_detectorLineSumming    = 1.0;
        m_ls_model->m_startingDetectorLine   =  m_coeff_psi_y[0]; // note that m_coeff_psi_y[1] = 0
        m_ls_model->m_detectorSampleSumming  = -m_coeff_psi_x[1];
        m_ls_model->m_startingDetectorSample
          = -m_coeff_psi_x[0] - m_coeff_psi_x[1] * (m_ref_col - 0.5);

        // Time
        auto const& tlc = dg_model->m_time_func.m_tlc;
        double time_offset = dg_model->m_time_func.m_time_offset;
        if (tlc.size() < 2) 
          vw::vw_throw(vw::ArgumentErr()
                       << "Expecting at least two line and time sample pairs.\n");
        
        m_ls_model->m_intTimeLines.clear(); // not needed, but best for clarity
        m_ls_model->m_intTimeStartTimes.clear();
        m_ls_model->m_intTimes.clear();
        for (size_t i = 0; i < tlc.size(); i++) {
          m_ls_model->m_intTimeLines.push_back(tlc[i].first + 1.0); // line
          m_ls_model->m_intTimeStartTimes.push_back(tlc[i].second + time_offset); // time
          // Slope
          if (i + 1 < tlc.size()) {
            // Compute the slope between this time instance and the next
            double slope = (tlc[i+1].second - tlc[i].second) / (tlc[i+1].first - tlc[i].first);
            m_ls_model->m_intTimes.push_back(slope);
          } else{
            // Cannot have a slope for the last value, as there's no next one to use,
            // but for consistency, borrow the last slope. This is consistent
            // with lines out of range using the slopes closest to them.
            double slope = m_ls_model->m_intTimes.back();
            m_ls_model->m_intTimes.push_back(slope);
          }
        }

        
      }

      // Pointer to linescan sensor.
      boost::shared_ptr<UsgsAstroLsSensorModel> m_ls_model;
      
    };
      
    //------------------------------------------------------------------
    // Constructors / Destructors
    //------------------------------------------------------------------
    LinescanDGModel(PositionFuncT const& position,
                    VelocityFuncT const& velocity,
                    PoseFuncT     const& pose,
                    vw::camera::TLCTimeInterpolation const& time,
                    vw::Vector2i  const& image_size, 
                    vw::Vector2   const& detector_origin,
                    double        const  focal_length,
                    double        const  mean_ground_elevation,
                    bool                 correct_velocity,
                    bool                 correct_atmosphere):
      vw::camera::LinescanModel(image_size, correct_velocity, correct_atmosphere),
      m_position_func(position), m_velocity_func(velocity),
      m_pose_func(pose), m_time_func(time),
      m_detector_origin(detector_origin),
      m_focal_length(focal_length) {
      m_mean_surface_elevation = mean_ground_elevation; // Set base class value

      if (stereo_settings().dg_use_csm)
        m_dg_csm_model.populateCsmModel(this);
    } 
    virtual ~LinescanDGModel() {}
    virtual std::string type() const { return "LinescanDG"; }

    // -- This set of functions implements virtual functions from LinescanModel.h --
    
    // Implement the functions from the LinescanModel class using functors
    virtual vw::Vector3 get_camera_center_at_time(double time) const {
      return m_position_func(time);
    }
    virtual vw::Vector3 get_camera_velocity_at_time(double time) const {
      // TODO(oalexan1): This is not called, so it is tricky to test.
      return m_velocity_func(time);
    }
    virtual vw::Quat get_camera_pose_at_time(double time) const {
      return m_pose_func(time);
    }
    virtual double get_time_at_line(double line) const {
      if (stereo_settings().dg_use_csm) {
        csm::ImageCoord csm_pix;
        vw::Vector2 pix(0, line);
        asp::toCsmPixel(pix, csm_pix);
        return m_dg_csm_model.m_ls_model->getImageTime(csm_pix);
      }
      
      return m_time_func(line);
    }

    // Gives a pointing vector in the world coordinates.
    virtual vw::Vector3 pixel_to_vector(vw::Vector2 const& pix) const {
      return vw::camera::LinescanModel::pixel_to_vector(pix);
    }
    
    /// As pixel_to_vector, but in the local camera frame.
    virtual vw::Vector3 get_local_pixel_vector(vw::Vector2 const& pix) const {
      vw::Vector3 local_vec(pix[0]+m_detector_origin[0], m_detector_origin[1], m_focal_length);
      return normalize(local_vec);
    }
    
    // Override this implementation with a faster, more specialized implementation.
    virtual vw::Vector2 point_to_pixel(vw::Vector3 const& point, double starty) const {
      // Use the uncorrected function to get a fast but good starting seed.
      vw::camera::CameraGenericLMA model(this, point);
      int status;
      vw::Vector2 start = point_to_pixel_uncorrected(point, starty);
      
      // Run the solver
      vw::Vector3 objective(0, 0, 0);
      const double ABS_TOL = 1e-16;
      const double REL_TOL = 1e-16;
      const int    MAX_ITERATIONS = 1e+5;
      vw::Vector2 solution = vw::math::levenberg_marquardtFixed<vw::camera::CameraGenericLMA, 2,3>
        (model, start, objective, status,
         ABS_TOL, REL_TOL, MAX_ITERATIONS);
      VW_ASSERT(status > 0,
                vw::camera::PointToPixelErr() << "Unable to project point into LinescanDG model.");
      return solution;
    }
    
    // Camera pose
    virtual vw::Quaternion<double> camera_pose(vw::Vector2 const& pix) const {
      return vw::camera::LinescanModel::camera_pose(pix);
    }

    /// Gives the camera position in world coordinates.
    virtual vw::Vector3 camera_center(vw::Vector2 const& pix) const {
      return vw::camera::LinescanModel::camera_center(pix);
    }

    // -- These are new functions --

    ///< Returns the focal length in pixels
    double       get_focal_length   () const {return m_focal_length;   }
    ///< Returns the detector origin in pixels
    vw::Vector2  get_detector_origin() const {return m_detector_origin;} 
    
    /// Create a fake pinhole model. It will return the same results
    /// as the linescan camera at current line y, but we will use it
    /// by extension at neighboring lines as well.
    vw::camera::PinholeModel linescan_to_pinhole(double y) const {
      double t = this->m_time_func(y);
      return vw::camera::PinholeModel(this->m_position_func(t),
                                      this->m_pose_func(t).rotation_matrix(),
                                      this->m_focal_length, -this->m_focal_length,
                                      -this->m_detector_origin[0],
                                      y - this->m_detector_origin[1]);
    }
    
    ///< Access the position function
    PositionFuncT const& get_position_func() const {return m_position_func;}
    
    ///< Access the velocity function
    VelocityFuncT const& get_velocity_func() const {return m_velocity_func;}

    ///< Access the pose function
     PoseFuncT const& get_pose_func() const {return m_pose_func;}

    ///< Access the time function
    vw::camera::TLCTimeInterpolation
    const& get_time_func() const {return m_time_func;} 

  protected: // Functions
  
    /// Low accuracy function used by point_to_pixel to get a good solver starting seed.
    vw::Vector2 point_to_pixel_uncorrected(vw::Vector3 const& point, double starty) const {
      // Solve for the correct line number to use
      LinescanLMA model(this, point);
      int status;
      vw::Vector<double> objective(1), start(1);
      start[0] = m_image_size.y()/2; 
      // Use a refined guess, if available, otherwise the center line.
      if (starty >= 0)
        start[0] = starty;

      // Run the solver
      const double ABS_TOL = 1e-16;
      const double REL_TOL = 1e-16;
      const int    MAX_ITERATIONS = 1e+5;
      vw::Vector<double> solution = vw::math::levenberg_marquardt(model, start, objective, status,
                                                                  ABS_TOL, REL_TOL, MAX_ITERATIONS);

      VW_ASSERT(status > 0, vw::camera::PointToPixelErr()
                << "Unable to project point into LinescanDG model.");

      // Solve for sample location now that we know the correct line
      double t = m_time_func(solution[0]);
      // TODO(oalexan1): Replace inverse with transpose if it is a rotation matrix?
      vw::Vector3 pt = inverse(m_pose_func(t)).rotate(point - m_position_func(t));
      pt *= m_focal_length / pt.z();

      return vw::Vector2(pt.x() - m_detector_origin[0], solution[0]);
    }

  protected: // Variables
    
    // Extrinsics
    PositionFuncT m_position_func; ///< Position at given time
    VelocityFuncT m_velocity_func; ///< Velocity at given time
    PoseFuncT     m_pose_func;     ///< Pose     at given time
    vw::camera::TLCTimeInterpolation m_time_func;     ///< Time at a given line

    // Intrinsics
    
    /// Location of (0,0) coordinate of the detector relative to the center of
    ///  the origin of the camera coordinate system.
    /// - Stored internally in pixels.
    vw::Vector2  m_detector_origin; 
    double       m_focal_length;    ///< The focal length, also stored in pixels.

    DgCsmModel m_dg_csm_model;
    
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

      LinescanLMA(const LinescanDGModel* model, const vw::Vector3& pt):
        m_model(model), m_point(pt) {}

        inline result_type operator()(domain_type const& y) const {
          double       t        = m_model->get_time_at_line(y[0]);
          vw::Quat     pose     = m_model->get_camera_pose_at_time(t);
          vw::Vector3  position = m_model->m_position_func(t);
          
          // Get point in camera's frame and rescale to pixel units
          vw::Vector3 pt = vw::camera::point_to_camera_coord(position, pose, m_point);
          pt *= m_model->m_focal_length / pt.z();
          result_type result(1);

          // Error against the location of the detector
          result[0] = pt.y() - m_model->m_detector_origin[1]; 
          return result;
        }
    };

  }; // End class LinescanDGModel

  // This is the standard DG implementation. In Extrinsics.cc there are
  // other ways of performing position and pose interpolation as well.
  typedef LinescanDGModel<vw::camera::PiecewiseAPositionInterpolation,
                          vw::camera::LinearPiecewisePositionInterpolation,
                          vw::camera::SLERPPoseInterpolation> DGCameraModel;

  /// Load a DG camera model from an XML file. This function does not
  /// take care of Xerces XML init/de-init, the caller must make sure
  /// this is done before/after this function is called.
  boost::shared_ptr<vw::camera::CameraModel>
  load_dg_camera_model_from_xml(std::string const& path);

} //end  namespace asp

#endif//__STEREO_CAMERA_LINESCAN_DG_MODEL_H__
