// __BEGIN_LICENSE__
//  Copyright (c) 2006-2013, United States Government as represented by the
//  Administrator of the National Aeronautics and Space Administration. All
//  rights reserved.
//
//  The NASA Vision Workbench is licensed under the Apache License,
//  Version 2.0 (the "License"); you may not use this file except in
//  compliance with the License. You may obtain a copy of the License at
//  http://www.apache.org/licenses/LICENSE-2.0
//
//  Unless required by applicable law or agreed to in writing, software
//  distributed under the License is distributed on an "AS IS" BASIS,
//  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
//  See the License for the specific language governing permissions and
//  limitations under the License.
// __END_LICENSE__


/// \file OpticalBarModel.h
///
/// A generic linescan camera model object
///
///
#ifndef _ASP_CAMERA_OPTICALBAR_MODEL_H_
#define _ASP_CAMERA_OPTICALBAR_MODEL_H_

#include <vw/Math/Quaternion.h>
#include <vw/Math/LevenbergMarquardt.h>
#include <vw/Camera/CameraModel.h>

namespace asp {
namespace camera {

  // A camera model to approximate the type of optical bar cameras
  // that were used in the Corona and Hexagon satellites.
  
  // This implements the logic in: 
  // Rigorous Panoramic Camera Model for DISP Imagery
  // Tony Schenk, Beata Csatho, Sung Woong Shin
  // - Proceedings of Joint Workshop of ISPRS Working Groups, 2003
  
  // With the motion compensation logic taken from:
  // Mathematical modelling of historical reconnaissance CORONA KH-4B imagery
  // HG Sohn, GH Kim, JH Yom - The Photogrammetric Record, 2004
  
  // Other good papers:
  
  // An evaluation of the stereoscopic capabilities of CORONA
  // declassified spy satellite image data, Nikolaos Galiatsatos,
  // Daniel N. M. Donoghue, and Graham Philip.

  // Stereo analysis, DEM extraction and orthorectification of CORONA
  // satellite imagery: archaeological applications from the Near East
  // Jesse Casana1 & Jackson Cothren

  // Data is at https://earthexplorer.usgs.gov/

  class OpticalBarModel : public vw::camera::CameraModel {

  public:
    //------------------------------------------------------------------
    // Constructors / Destructors
    //------------------------------------------------------------------

    OpticalBarModel() 
      : m_motion_compensation(1.0),
        m_correct_velocity_aberration(true),
        m_correct_atmospheric_refraction(true) {
    }

    OpticalBarModel(std::string const& path)
      : m_motion_compensation(1.0),
        m_correct_velocity_aberration(true),
        m_correct_atmospheric_refraction(true) {
      read(path);
    } // Create from file

    OpticalBarModel(vw::Vector2i image_size,
                    vw::Vector2  center_offset_pixels,
                    double   pixel_size,
                    double   focal_length,
                    double   scan_angle_radians,
                    double   scan_rate_radians,
                    bool     scan_left_to_right,
                    double   forward_tilt_radians,
                    vw::Vector3  initial_position,
                    vw::Vector3  initial_orientation,
                    double   speed,
                    double   motion_compensation_factor) :
        m_image_size          (image_size),
        m_center_loc_pixels   (center_offset_pixels),
        m_pixel_size          (pixel_size),
        m_focal_length        (focal_length),
        m_scan_angle_radians  (scan_angle_radians),
        m_scan_rate_radians   (scan_rate_radians),
        m_scan_left_to_right  (scan_left_to_right),
        m_forward_tilt_radians(forward_tilt_radians),
        m_initial_position    (initial_position),
        m_initial_orientation (initial_orientation),
        m_speed               (speed),
        m_motion_compensation(motion_compensation_factor),
        m_correct_velocity_aberration(true),
        m_correct_atmospheric_refraction(true){

      // Set default values for these constants which can be overridden later on.
      const double DEFAULT_EARTH_RADIUS      = 6371000.0;  // In meters.
      const double DEFAULT_SURFACE_ELEVATION = 0.0;
      m_mean_earth_radius      = DEFAULT_EARTH_RADIUS;
      m_mean_surface_elevation = DEFAULT_SURFACE_ELEVATION;
    }

    virtual ~OpticalBarModel() {}
    virtual std::string type() const { return "OpticalBar"; }

    // TODO: Make compatible with .tsai files!
    /// Read / Write a from a file on disk.
    void read (std::string const& filename);
    void write(std::string const& filename) const;

    //------------------------------------------------------------------
    // Interface
    //------------------------------------------------------------------

    // -- This set of functions implements virtual functions from CameraModel.h --

    /// Get the pixel that observes a point in world coordinates.
    virtual vw::Vector2 point_to_pixel (vw::Vector3 const& point) const;

    /// Gives a pointing vector in the world coordinates.
    virtual vw::Vector3 pixel_to_vector(vw::Vector2 const& pix) const;

    /// Gives the camera position in world coordinates.
    virtual vw::Vector3 camera_center(vw::Vector2 const& pix) const;

    /// Gives a pose vector which represents the rotation from camera to world units
    virtual vw::Quat camera_pose(vw::Vector2 const& pix) const;

    // -- These are new functions --

    vw::Vector3 camera_center() const {return camera_center(vw::Vector2(0,0));}
    vw::Quat    camera_pose  () const {return camera_pose  (vw::Vector2(0,0));}

    /// Apply a given rotation + translation + scale transform to the camera.
    /// - TODO: This is identical to the function in PinholeModel.
    void apply_transform(vw::Matrix3x3 const & rotation,
                         vw::Vector3   const & translation,
                         double                scale);

    // Parameter accessors

    void set_camera_center(vw::Vector3 const& position   ) {m_initial_position    = position;}
    void set_camera_pose  (vw::Vector3 const& orientation) {m_initial_orientation = orientation;}
    void set_camera_pose  (vw::Quaternion<double> const& pose) {set_camera_pose(pose.axis_angle());};

    /// Returns the image size in pixels
    vw::Vector2i get_image_size    () const { return m_image_size;          }
    vw::Vector2  get_optical_center() const { return m_center_loc_pixels;   }
    double       get_focal_length  () const { return m_focal_length;        }
    double       get_scan_rate     () const { return m_scan_rate_radians;   }
    double       get_speed         () const { return m_speed;               }
    double       get_pixel_size    () const { return m_pixel_size;          }
    double       get_scan_angle    () const { return m_scan_angle_radians;  }
    bool         get_scan_dir      () const { return m_scan_left_to_right;  }
    double       get_forward_tilt  () const { return m_forward_tilt_radians;}


    void set_image_size    (vw::Vector2i image_size    ) { m_image_size           = image_size;     }
    void set_optical_center(vw::Vector2  optical_center) { m_center_loc_pixels    = optical_center; }
    void set_focal_length  (double       focal_length  ) { m_focal_length         = focal_length;   }
    void set_scan_rate     (double       scan_rate     ) { m_scan_rate_radians    = scan_rate;      }
    void set_speed         (double       speed         ) { m_speed                = speed;          }
    void set_pixel_size    (double       pixel_size    ) { m_pixel_size           = pixel_size;     }
    void set_scan_angle    (double       scan_angle    ) { m_scan_angle_radians   = scan_angle;     }
    void set_scan_dir      (bool         scan_l_to_r   ) { m_scan_left_to_right   = scan_l_to_r;    }
    void set_forward_tilt  (double       tilt_angle    ) { m_forward_tilt_radians = tilt_angle;     }

    double get_motion_compensation() const           { return m_motion_compensation;    }
    void   set_motion_compensation(double mc_factor) { m_motion_compensation = mc_factor;}

    friend std::ostream& operator<<(std::ostream&, OpticalBarModel const&);

  private:

    vw::Vector2 pixel_to_sensor_plane(vw::Vector2 const& pixel) const;

    vw::Vector3 pixel_to_vector_uncorrected(vw::Vector2 const& pixel) const;

    /// Returns the velocity in the GCC frame, not the sensor frame.
    vw::Vector3 get_velocity(vw::Vector2 const& pixel) const;

    /// Compute the time since start of scan for a given pixel.
    double pixel_to_time_delta(vw::Vector2 const& pixel) const;

  protected:

    /// Image size in pixels: [columns, rows]
    vw::Vector2i m_image_size;

    /// Offset from the top left of the image (film) to the camera center.
    vw::Vector2  m_center_loc_pixels;

    /// The physical size of each pixel in meters (scanner resolution).
    /// - Should be 7 or 14 microns.
    double m_pixel_size;

    /// The focal length in meters
    // Nominal focal length for Corona is 609.602mm
    // Nominal focal length for Hexagon is 1.5m inches
    double m_focal_length;

    /// The maximum scan angle reached in both direction.
    /// - The Corona scan angle is about +/- 35 degrees.
    double m_scan_angle_radians;

    /// The angular velocity of the scanner.
    /// - The Corona scan rate is nominally 192 degrees/second
    double m_scan_rate_radians;

    /// The tilt of the camera forward relative to nadir.
    /// - Some optical bar systems have a forward and backward aimed camera.
    /// - This tilt angle is needed to properly account for the camera's motion.
    double m_forward_tilt_radians;

    /// If true, the first column corresponds to the inital position,
    ///  otherwise the last column is at the initial position.
    bool m_scan_left_to_right;
    
    vw::Vector3 m_initial_position;
    vw::Vector3 m_initial_orientation; // TODO: Record as matrix
    double      m_speed; /// Velocity in the sensor Y axis only.

    // These are used for ray corrections.
    double m_mean_earth_radius;
    double m_mean_surface_elevation;

    /// Apply this fraction of the nominal motion compensation.
    double m_motion_compensation;

    /// Set this flag to enable velocity aberration correction.
    /// - For satellites this makes a big difference, make sure it is set!
    bool m_correct_velocity_aberration;

    /// Set this flag to enable atmospheric refraction correction.
    bool m_correct_atmospheric_refraction;

  protected:

    /// Returns the radius of the Earth under the current camera position.
    double get_earth_radius() const;

  }; // End class OpticalBarModel


  /// Output stream method.
  std::ostream& operator<<( std::ostream& os, OpticalBarModel const& camera_model);


}}      // namespace asp::camera

#endif  //_ASP_CAMERA_OPTICALBAR_MODEL_H_
