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

#include <asp/Core/StereoSettings.h>  // TESTING

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
	                  double        const  focal_length,
	                  double        const  mean_ground_elevation=0
		    ) : vw::camera::LinescanModel(image_size, true), // Always correct velocity aberration
		        m_position_func(position), m_velocity_func(velocity),
            m_pose_func(pose),         m_time_func(time),
            m_detector_origin(detector_origin),
            m_focal_length(focal_length),
            m_mean_ground_elevation(mean_ground_elevation)
             {} 
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

//---------------- TESTING ---------------------------------
// -> If it works, move the atmosphere code up to VW!

    /// Compute the ray correction for atmospheric refraction using the 
    ///  Saastamoinen equation.
    static double saastamoinen_atmosphere_correction(double camera_alt, double ground_alt, double alpha) {

      double H      = camera_alt / 1000.0; // In kilometers
      double h      = ground_alt / 1000.0; // Height of target point over surface in km
      double h_diff = H - h;
      double p1     = (2335.0 / h_diff)*pow(1.0 - 0.02257*h, 5.256);
      double p2     = pow(0.8540, H-11.0) * (82.2 - 521.0/h_diff);
      double K      = (p1 - p2) * pow(10.0,-6.0);

      //K = K * 1.55976279684; // TESTING!

      //double tan_alpha   = norm_2(cross_prod(u, cam_to_earth_center_unit)) / dot_prod(cam_to_earth_center_unit,u);
      double delta_alpha = K * tan(alpha);
      return delta_alpha;
    }

    /// Gives a pointing vector in the world coordinates.
    virtual vw::Vector3 pixel_to_vector(vw::Vector2 const& pix) const
{

  try {
    // Compute local vector from the pixel out of the sensor
    // - m_detector_origin and m_focal_length have been converted into units of pixels
    vw::Vector3 local_vec = get_local_pixel_vector(pix);
    // Put the local vector in world coordinates using the pose information.
    vw::Vector3 uncorrected_vector = camera_pose(pix).rotate(local_vec);

    if (stereo_settings().correct_atmospheric_refraction)
    {

      // Correct for atmospheric refraction
      // - From "Atmospheric Correction and Velocity Aberration for Physical Sensor 
      //         Modeling of High-Resolution Satellite Images" by Oh and Lee

      // Get some information
      vw::Vector3 cam_ctr              = camera_center(pix); // ECF camera coords
      vw::Vector3 cam_ctr_norm         = normalize(cam_ctr);
      vw::Vector3 cam_to_earth_center_unit = -1.0 * cam_ctr_norm;
      double  earth_ctr_to_cam     = norm_2(cam_ctr);    // Distance in meters from cam to earth center
      double  earth_rad            = 6371000.0; // TODO: Vary by location?
      double  cam_to_earth_surface = earth_ctr_to_cam - earth_rad;

      // Compute angle alpha and correction angle
      vw::Vector3 u     = normalize(uncorrected_vector);
      double      alpha = acos(dot_prod(cam_to_earth_center_unit,u)); // Both get angle of normalized vectors.

      // TODO: Test out another correction method!
      double delta_alpha = saastamoinen_atmosphere_correction(cam_to_earth_surface,
                                                              m_mean_ground_elevation, alpha);

      // Rotate the vector by delta_alpha
      
      vw::Vector3 rotation_axis = normalize(cross_prod(u, cam_to_earth_center_unit));
      vw::Quaternion<double> refraction_rotation(rotation_axis, delta_alpha);
      vw::Vector3 u_prime = refraction_rotation.rotate(u);
      
      vw::Quaternion<double> inv_rot(rotation_axis, alpha);
      vw::Vector3 u_inv = inv_rot.rotate(u);
      /*
      double  check_delta  = acos(dot_prod(u_prime,u) / (norm_2(u_prime)*norm_2(u)) );
      if ( (pix[0] > 19117) && (pix[1] > 6043) &&
           (pix[0] < 19118) && (pix[1] < 6044) ) { 

        std::cout << "pix = " << pix << std::endl;
        std::cout << "Computed h_diff = "      << h_diff        << std::endl;
        std::cout << "Computed p1 = "          << p1            << std::endl;
        std::cout << "Computed p2 = "          << p2            << std::endl;
        std::cout << "Computed K = "           << K             << std::endl;
        std::cout << "Computed alpha = "       << alpha         << std::endl;
        std::cout << "Computed delta_alpha = " << delta_alpha   << std::endl;
        std::cout << "Computed check = "       << check_delta   << std::endl;  
        std::cout << "Computed rot axis = "    << rotation_axis << std::endl;
        std::cout << "Computed u = "           << u             << std::endl;
        std::cout << "Computed u prime = "     << u_prime       << std::endl;
        std::cout << "Computed cam_to_earth = " << cam_to_earth_center_unit << std::endl;
        std::cout << "Computed u inv = "       << u_inv       << std::endl;
      }*/
      uncorrected_vector = u_prime;
    } // End atmosphere correction

    if (!m_correct_velocity_aberration) 
      return uncorrected_vector;
    else
      return apply_velocity_aberration_correction(pix, uncorrected_vector);
      
  } catch(const vw::Exception &e) {
    // Repackage any of our exceptions thrown below this point as a 
    //  pixel to ray exception that other code will be able to handle.
    vw_throw(vw::camera::PixelToRayErr() << e.what());
  }

}

// ------------ TESTING -------------------------------------------------------

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

    /// Expected ground elevation in meters, used for atmospheric correction.
    double m_mean_ground_elevation; 

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
