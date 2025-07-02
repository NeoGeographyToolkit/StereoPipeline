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


/// \file CsmModel.h
///
/// Wrapper for Community Sensor Model implementations.
///
///
#ifndef __STEREO_CAMERA_CSM_MODEL_H__
#define __STEREO_CAMERA_CSM_MODEL_H__

#include <vw/Camera/CameraModel.h>
#include <vw/Cartography/Datum.h>

#include <usgscsm/Distortion.h>

#include <boost/shared_ptr.hpp>

namespace vw {
  namespace camera {
    class PinholeModel;
  }
}

namespace csm {
  // Forward declarations
  class RasterGM; 
  class ImageCoord;
  class EcefVector;
  class EcefCoord;
}

namespace asp {

  // Do not set this lower than 1e-8, as then UsgsAstroLsSensorModel can return
  // junk because of numerical precision issues for high focal length.
  const double DEFAULT_CSM_DESIRED_PRECISION = 1.0e-8;
  
  /// Class to load any cameras described by the Community Sensor Model (CSM)
  class CsmModel : public vw::camera::CameraModel {
  public:
    //------------------------------------------------------------------
    // Constructors / Destructors
    //------------------------------------------------------------------
    CsmModel();
    CsmModel(std::string const& isd_path); ///< Construct from ISD file
    
    // Note: This class copy constructor is shallow. Use deep_copy() to
    // create a deep copy.

    virtual ~CsmModel();
    virtual std::string type() const { return "CSM"; }

    /// Load the camera model from an ISD file or model state.
    void load_model(std::string const& isd_path);

    /// Return the size of the associated image.
    vw::Vector2 get_image_size() const;

    virtual vw::Vector2 point_to_pixel (vw::Vector3 const& point) const;

    virtual vw::Vector3 pixel_to_vector(vw::Vector2 const& pix) const;

    virtual vw::Vector3 camera_center(vw::Vector2 const& pix) const;

    virtual vw::Quaternion<double> camera_pose(vw::Vector2 const& pix) const {
      vw_throw(vw::NoImplErr() << "CsmModel: Cannot retrieve camera_pose!");
      return vw::Quaternion<double>();
    }

    /// Return the path to the folder where we will look for CSM plugin DLLs.
    static std::string get_csm_plugin_folder();

    /// Get a list of all of the CSM plugin DLLs in the CSM plugin folder.
    static size_t find_csm_plugins(std::vector<std::string> &plugins);

    /// Print the CSM models that have been loaded into the main CSM plugin.
    static void print_available_models();

    /// Get the semi-axes of the datum. 
    vw::Vector3 target_radii() const;

    // Save the model state
    void saveState(std::string const& json_state_file) const;

    // Apply a transform to the model and save the transformed state as a JSON file.
    void saveTransformedState(std::string const& json_state_file,
                              vw::Matrix4x4 const& transform) const;

    // Apply a transform to a CSM model
    void applyTransform(vw::Matrix4x4 const& transform);
    
    // Create a CSM frame camera model. Assumes that focal length and optical
    // center are in pixels, the pixel pitch is 1.
    void createFrameModel(int cols, int rows,  // in pixels
        double cx, double cy, // col and row of optical center, in units of pixel pitch
        double focal_length,  // in units of pixel pitch
        double semi_major_axis, double semi_minor_axis, // in meters
        vw::Vector3 const& C, // camera center
        vw::Matrix3x3 const& R, // camera to world rotation matrix
        std::string const& distortionType = "",
        std::vector<double> const& distortion = std::vector<double>(),
        double ephem_time = 0.0,
        vw::Vector3 const& sun_position = vw::Vector3(0,0,0),
        std::string const& serial_number = "",
        std::string const& target_name = "",
        double pixel_pitch = 1.0);
        
    // Create a CSM frame camera model from pinhole camera model.
    // The distortion model must be set separately, as ASP pinhole
    // and CSM use different distortion models.
    void createFrameModel(vw::camera::PinholeModel const& pin_model,
                          int cols, int rows,  // in pixels
                          double semi_major_axis, double semi_minor_axis, // in meters
                          std::string const& distortionType = "", 
                          std::vector<double> const& distortion = std::vector<double>(),
                          double ephem_time = 0.0,
                          vw::Vector3 const& sun_position = vw::Vector3(0,0,0),
                          std::string const& serial_number = "",
                          std::string const& target_name = "");

    // Approximate conversion to a pinhole model. Will be exact only for the rad-tan
    // lens distortion and no unusual line or sample adjustments in CSM. Compare
    // these with cam_test.
    vw::camera::PinholeModel pinhole() const;
    
    // Sun position in ECEF
    vw::Vector3 sun_position() const;

    // For bundle adjustment need a higher precision as CERES needs to do accurate
    // numerical differences.
    void setDesiredPrecision(double desired_precision) {
      m_desired_precision = desired_precision;
    }

    /// Create the model from a state string. Use recreate_model = false,
    /// if desired to adjust an existing model.
    void setModelFromStateString(std::string const& model_state, bool recreate_model);

    /// Get intrinsic parameters
    std::vector<double> distortion() const;
    DistortionType distortion_type() const;
    double focal_length() const;
    vw::Vector2 optical_center() const; // return sample and line
    
    // Set intrinsics
    void set_distortion_type(DistortionType dist_type);
    void set_distortion(std::vector<double> const& distortion);
    void set_focal_length(double focal_length);
    void set_optical_center(vw::Vector2 const& optical_center); // sample and line

    // Set / get the position (camera center) in ECEF. Only for frame cameras.
    void set_frame_position(double x, double y, double z);
    void frame_position(double & x, double & y, double & z) const;

    // Set / get the rotation matrix from camera to world. Only for frame cameras.
    void set_frame_quaternion(double qx, double qy, double qz, double qw);
    void frame_quaternion(double & qx, double & qy, double & qz, double & qw) const;
        
    // Set / get quaternions (only for linescan cameras)
    void set_linescan_quaternions(std::vector<double> const& quaternions);
    std::vector<double> linescan_quaternions() const;
    
    double frame_pixel_pitch() const; // pixel pitch for frame camera
    
    // Target name
    std::string target_name() const;
    void set_target_name(std::string const& target_name);
    
    boost::shared_ptr<csm::RasterGM> m_gm_model;

    double m_desired_precision;
    
    // These are read from the json camera file
    double m_semi_major_axis, m_semi_minor_axis;
    
    // Create a deep copy of the model, so don't just copy the shared pointer.
    void deep_copy(boost::shared_ptr<CsmModel> & copy) const;
    void deep_copy(CsmModel & copy) const;
    
    std::string plugin_name() const;
    std::string model_name () const;
    std::string model_state() const; 
    
    bool isFrameCam() const;
    
    // Ensure the linescan model quaternions are always normalized and do not
    // suddenly flip sign
    void normalizeLinescanQuaternions();
    
    // Get the datum from the CSM model. It is suggested to use if possible
    // the function StereoSessionCsm::get_datum() which calls this one, as
    // that one also knows about the image and can find the datum name.
    // If the spheroid name is not known, use "unknown".
    vw::cartography::Datum get_datum_csm(std::string spheroid_name, 
                                         bool use_sphere_for_non_earth) const;
    
  protected:

    // Read the ellipsoid (datum) axes from the isd json file
    // (does not work for reading it from a json state file).
    void read_ellipsoid_from_isd(std::string const& isd_path);
    
    /// Load the camera model from an ISD file.
    void load_model_from_isd(std::string const& isd_path);
    
    /// Load the camera model from a model state written to disk.
    /// A model state is obtained from an ISD model by pre-processing
    /// and combining its data in a form ready to be used.
    void loadModelFromStateFile(std::string const& state_file);

    /// Find and load any available CSM plugin libraries from disk.
    /// - This does nothing after the first time it finds any plugins.
    void initialize_plugins();

    /// Throw an exception if we have not loaded the model yet.
    void throw_if_not_init() const;

    vw::Vector3 m_sun_position;
    
    std::string m_plugin_name;
    
  }; // End class CsmModel

  // Auxiliary non-member functions to convert a pixel from ASP
  // conventions to what CSM expects and vice versa
  void toCsmPixel(vw::Vector2 const& pix, csm::ImageCoord & csm);
  void fromCsmPixel(vw::Vector2 & pix, csm::ImageCoord const& csm);
  vw::Vector3 ecefVectorToVector(csm::EcefVector const& c);
  vw::Vector3 ecefCoordToVector(csm::EcefCoord const& c);
  vw::Vector2 imageCoordToVector(csm::ImageCoord const& c);
  
} // end namespace asp

#endif //__STEREO_CAMERA_CSM_MODEL_H__
