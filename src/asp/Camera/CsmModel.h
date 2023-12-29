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

#include <usgscsm/Distortion.h>

#include <boost/shared_ptr.hpp>

namespace csm {
  // Forward declarations
  class RasterGM; 
  class ImageCoord;
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
    // center are in pixels, the pixel pitch is 1, and no distortion.
    void createFrameModel(int cols, int rows,  // in pixels
        double cx, double cy, // col and row optical center, in pixels
        double focal_length,  // in pixels
        double semi_major_axis, double semi_minor_axis, // in meters
        vw::Vector3 C, // camera center
        vw::Matrix3x3 R); // camera to world rotation matrix

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
    double focal_length() const;
    vw::Vector2 optical_center() const; // return sample and line
    
    // Set intrinsics
    void set_distortion_type(DistortionType dist_type);
    void set_distortion(std::vector<double> const& distortion);
    void set_focal_length(double focal_length);
    void set_optical_center(vw::Vector2 const& optical_center); // sample and line

    // Set / get quaternions (only for linescan cameras)
    void set_linescan_quaternions(std::vector<double> const& quaternions);
    std::vector<double> linescan_quaternions() const;
    
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
  
} // end namespace asp

#endif //__STEREO_CAMERA_CSM_MODEL_H__
