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
#ifndef __STEREO_CAMERA_LINESCAN_SPOT_MODEL_H__
#define __STEREO_CAMERA_LINESCAN_SPOT_MODEL_H__

#include <vw/Math/Matrix.h>
#include <vw/Camera/LinescanModel.h>
#include <vw/Camera/PinholeModel.h>
#include <vw/Camera/Extrinsics.h>


namespace asp {


/* 
   The intrinisic model expects +Z to be point out the camera. +X is
   the column direction of the image and is perpendicular to
   direction of flight. +Y is the row direction of the image (down
   the image); it is also the flight direction. This is different
   from the SPOT5 model so the input data has to be handled carefully.
   
   The SPOT5 "Local Orbital Reference System" is defined by:
    Z = Ray from earth center through satellite center.
    X = Perpendicular to Z the direction of travel V(elocity) 
        (= perpendicular to the orbital plane).
    Y = Z cross X.  Will be close to V but not an exact match.
    Referred to in docs as (O2,X2,Y2,Z2)
    
    Compute O2:
      Z2 (gcc) = norm(pos)
      X2 (gcc) = norm(vel cross Z2)
      Y2 (gcc) = Z2 cross X2
      
    
    The SPOT5 "Navigation Reference Coordinate System" is actually
    tied to the satellite.  Ideally this is perfectly aligned with 
    the "Local Orbital Reference System" described above.  The local 
    pixel angles are relative to this coordinate system (O1,X1,Y1,Z1)
    
    Local angles phi_x and phi_y are expressed in O1 (Navigation) coord system.
    This will get you the look vector in O1 = u1.  Ultimately we need
    the look vector in GCC = u3.  
    The angles are actually expressed in a special coordinate frame:
      Xa = -X1, Ya = -Y1, Za = Z1.
    These inversions are taken into account in the following equations
    to compute the look vector in the O2 (Orbital) coord system = u2:
      u2 = Mp*Mr*My*u1
      Mp = [1, 0,           0          ]
           [0, cos(pitch),  sin(pitch) ]
           [0, -sin(pitch), cos(pitch) ]
      Mr = [cos(roll), 0, -sin(roll)]
           [0,         1,  0        ]
           [sin(roll), 0,  cos(roll)]
      My = [cos(yaw), -sin(yaw), 0]
           [sin(yaw),  cos(yaw), 0]
           [0,         0,        1]
   
    look(gcc=u3) = [X2 | Y2 | Z2] * look(orbital=u2)
  
  Take the simple case, zero angles and center pixel:
    The local look vector is [0, 0, -1]
    M = Identity
    The O2 vectors are in GCC coords, but not our standard frame.
    
    Rs = [-1  0  0]
         [ 0  1  0]
         [ 0  0 -1]   
*/


  // The useful load_spot_camera_model() function is at the end of the file.

  /// Specialization of the generic LinescanModel for SPOT satellites.
  class SPOTCameraModel : public vw::camera::LinescanModel {

  public:
    //------------------------------------------------------------------
    // Constructors / Destructors
    //------------------------------------------------------------------
    SPOTCameraModel(vw::camera::LagrangianInterpolation       const& position,
  		              vw::camera::LagrangianInterpolation       const& velocity,
		                vw::camera::SLERPPoseInterpolation        const& pose,
		                vw::camera::LinearTimeInterpolation       const& time,
                    std::vector<std::pair<int, vw::Vector2> > const& look_angles,
		                vw::Vector2i  const& image_size,
		                double min_time, double max_time
		    ) : vw::camera::LinescanModel(image_size, true), // Always correct velocity aberration
  		      m_position_func(position), m_velocity_func(velocity),
            m_pose_func(pose),         m_time_func(time),
            m_look_angles(look_angles),
            m_min_time(min_time), m_max_time(max_time) {}
		    
    virtual ~SPOTCameraModel() {}
    virtual std::string type() const { return "LinescanSPOT"; }

    // -- This set of functions implements virtual functions from LinescanModel.h --

    // Implement the functions from the LinescanModel class using functors
    virtual vw::Vector3 get_camera_center_at_time  (double time) const;
    virtual vw::Vector3 get_camera_velocity_at_time(double time) const;
    virtual vw::Quat    get_camera_pose_at_time    (double time) const;
    virtual double      get_time_at_line           (double line) const;
    
    /// As pixel_to_vector, but in the local camera frame.
    virtual vw::Vector3 get_local_pixel_vector(vw::Vector2 const& pix) const;

    // TODO: See if we can port these local changes to the parent class
    virtual vw::Vector2 point_to_pixel(vw::Vector3 const& point, double starty) const;
    virtual vw::Vector2 point_to_pixel(vw::Vector3 const& point) const {
      return point_to_pixel(point, -1); // Redirect to the function with no initial guess
    }
 
    // ---- Users probably won't ever need to call these functions ----
 
    /// Given the satellite's position and velocity in GCC coordinates, return
    ///  the O2 frame in the format [X2 | Y2 | Z2].
    /// - This matrix is needed to convert local look vectors to GCC coordinates.
    static vw::Matrix3x3 get_local_orbital_frame(vw::Vector3 const& position, vw::Vector3 const& velocity);
    
    /// Returns the matrix needed to convert an O1 look vector into an O2 look vector.
    /// = Mp*Mr*My
    static vw::Matrix3x3 get_look_rotation_matrix(double yaw, double pitch, double roll);

  private:

    /// SPOT5 velocities are already rotation velocity corrected!
    virtual vw::Vector3 get_rotation_corrected_velocity(vw::Vector2 const& pixel,
                                                        vw::Vector3 const& uncorrected_vector) const {
      return camera_velocity(pixel);
    }

  protected:

    // Extrinsics
    vw::camera::LagrangianInterpolation m_position_func; ///< Yields position at time T
    vw::camera::LagrangianInterpolation m_velocity_func; ///< Yields velocity at time T
    vw::camera::SLERPPoseInterpolation  m_pose_func;     ///< Yields pose     at time T
    vw::camera::LinearTimeInterpolation m_time_func;     ///< Yields time at a given line.
    
    // Intrinsics
    
    // TODO: Any reason to keep the int?
    /// This is a lookup table for local pixel ray vectors loaded from the XML file.
    std::vector<std::pair<int, vw::Vector2> > m_look_angles;
    
    /// These are the limits of when he have pose data available.
    /// - SPOT5 data only barely provides enough pose data for the time range,
    ///   so asking for data outside this range will throuw an exception.
    double m_min_time, m_max_time;
    
    /// Throw an exception if the input time is outside the given bounds.
    /// - Pass the caller location in to get a nice error message.
    void check_time(double time, std::string const& location) const;

  }; // End class SPOTCameraModel


  /// Load a SPOT5 camera model from an XML file.
  /// - This function does not take care of Xerces XML init/de-init, the caller must
  ///   make sure this is done before/after this function is called!
  boost::shared_ptr<SPOTCameraModel> load_spot5_camera_model_from_xml(std::string const& path);

}      // namespace asp


#endif//__STEREO_CAMERA_LINESCAN_SPOT_MODEL_H__
