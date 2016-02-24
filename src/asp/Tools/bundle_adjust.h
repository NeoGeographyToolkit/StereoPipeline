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


#ifndef __ASP_TOOLS_BUNDLEADJUST_H__
#define __ASP_TOOLS_BUNDLEADJUST_H__

#include <boost/algorithm/string.hpp>
#include <boost/program_options.hpp>
#include <boost/filesystem/operations.hpp>
#include <boost/filesystem/fstream.hpp>

#include <vw/Camera/CAHVORModel.h>
#include <vw/Camera/PinholeModel.h>
#include <vw/Camera/LensDistortion.h>
#include <vw/BundleAdjustment.h>
#include <vw/Math.h>

#include <stdlib.h>
#include <iostream>

#include <asp/Core/BundleAdjustUtils.h>

// TODO: Why is this in a seperate namespace?
namespace asp{

  /// Grab the extrinsic camera parameters from the parameter vector
  template<class camera_vector_t>
  void parse_camera_parameters(camera_vector_t cam_j,
                               vw::Vector3 &position_correction,
                               vw::Quat    &pose_correction) {
    position_correction = subvector(cam_j, 0, 3);
    pose_correction     = axis_angle_to_quaternion( subvector(cam_j,3,3) );
  }

  /// Copy the extrinsic camera parameters into the parameter vector
  template<class camera_vector_t>
  void set_camera_params(camera_vector_t & cam_j,
                         vw::Vector3 const& position_correction,
                         vw::Quat    const& pose_correction) {
    subvector(cam_j, 0, 3) = position_correction;
    subvector(cam_j, 3, 3) = pose_correction.axis_angle();
  }


  /// Copy both extrinsics and intrinsics into a presized parameter vector.
  template<class ModelT>
  void concat_extrinsics_intrinsics(const double* const extrinsics,
                                    const double* const intrinsics,
                                    typename ModelT::camera_intr_vector_t & concat){
    int intr_len = ModelT::intrinsic_params_n, 
        cam_len  = ModelT::camera_params_n;
    for (int c = 0; c < cam_len; c++)
      concat[c] = extrinsics[c];
    for (int i = 0; i < intr_len; i++)
      concat[cam_len + i] = intrinsics[i];
  }

}

// Bundle adjustment functor
class BundleAdjustmentModel:
  public vw::ba::ModelBase<BundleAdjustmentModel, 6, 3> {

public:
  typedef vw::Vector<double,camera_params_n>         camera_vector_t;
  typedef vw::Vector<double,point_params_n >         point_vector_t;
  typedef boost::shared_ptr<vw::camera::CameraModel> cam_ptr_t;
  // No intrinsic params
  const static size_t intrinsic_params_n = 0;
  typedef vw::Vector<double,intrinsic_params_n> intrinsic_vector_t;
  typedef vw::Vector<double,camera_params_n
                     +intrinsic_params_n> camera_intr_vector_t;
private:
  std::vector<cam_ptr_t>                    m_cameras;
  boost::shared_ptr<vw::ba::ControlNetwork> m_network;

  std::vector<camera_intr_vector_t> m_cam_vec;
  std::vector<point_vector_t      > m_point_vec;
  std::vector<camera_vector_t     > m_cam_target_vec;
  std::vector<point_vector_t      > m_point_target_vec;
  int m_num_pixel_observations;

public:
  BundleAdjustmentModel(std::vector<cam_ptr_t> const& cameras,
                        boost::shared_ptr<vw::ba::ControlNetwork> network) :
    m_cameras(cameras), m_network(network), m_cam_vec(cameras.size()),
    m_point_vec(network->size()), m_cam_target_vec(cameras.size()),
    m_point_target_vec(network->size()) {

    // Compute the number of observations from the bundle.
    m_num_pixel_observations = 0;
    for (unsigned i = 0; i < network->size(); ++i)
      m_num_pixel_observations += (*network)[i].size();

    // Set up the point vectors, storing the initial values.
    // The camera vectors however just start out zero.
    for (unsigned i = 0; i < network->size(); ++i) {
      m_point_vec[i] = (*m_network)[i].position();
      m_point_target_vec[i] = m_point_vec[i];
    }
  }

  // Return a reference to the camera and point parameters.
  camera_vector_t cam_params  (int j) const { return m_cam_vec[j];   }
  point_vector_t  point_params(int i) const { return m_point_vec[i]; }
  
  void set_cam_params  (int j, camera_intr_vector_t const& cam_j  ) { m_cam_vec  [j] = cam_j;   }
  void set_point_params(int i, point_vector_t       const& point_i) { m_point_vec[i] = point_i; }

  // Return the initial parameters
  camera_vector_t cam_target  (int j) const { return m_cam_target_vec[j];   }
  point_vector_t  point_target(int i) const { return m_point_target_vec[i]; }

  unsigned num_cameras()            const { return m_cam_vec.size();         }
  unsigned num_points ()            const { return m_point_vec.size();       }
  unsigned num_pixel_observations() const { return m_num_pixel_observations; }

  /// Return the covariance of the camera parameters for camera j.
  inline vw::Matrix<double,camera_params_n,camera_params_n>
  cam_inverse_covariance ( unsigned /*j*/ ) const {
    vw::Matrix<double,camera_params_n,camera_params_n> result;
    result(0,0) = 1/100.0; // TODO: What are these numbers?
    result(1,1) = 1/100.0;
    result(2,2) = 1/100.0;
    result(3,3) = 1/1e-1;
    result(4,4) = 1/1e-1;
    result(5,5) = 1/1e-1;
    return result;
  }

  /// Return the covariance of the point parameters for point i.
  inline vw::Matrix<double,point_params_n,point_params_n>
  point_inverse_covariance ( unsigned /*i*/ ) const {
    vw::Matrix<double,point_params_n,point_params_n> result;
    result(0,0) = 1/20; // TODO: What are these numbers?
    result(1,1) = 1/20;
    result(2,2) = 1/20;
    return result;
  }

  /// Given the camera model parameters for the j'th
  /// image, and the 3D point location for the i'th
  /// point, return the location of point_i on imager j in pixel coordinates.
  /// - The model parameters in this case are adjustment parameters for AdjustedCameraModel
  vw::Vector2 cam_pixel(unsigned /*i*/, unsigned j,
			                  camera_vector_t const& cam_j,
			                  point_vector_t  const& point_i) const {
    vw::Vector3 position_correction;
    vw::Quat    pose_correction;
    asp::parse_camera_parameters(cam_j, position_correction, pose_correction);
    vw::camera::AdjustedCameraModel cam(m_cameras[j],
                                        position_correction,
                                        pose_correction);
    try {
      //std::cout << "cam_pixel: " << point_i << " --> " << cam.point_to_pixel(point_i) << std::endl;
      return cam.point_to_pixel(point_i);
    }
    catch(...) { // If the camera parameters were bad, return a garbage pixel instead of crashing
      //std::cout << "MISSED point at: " << point_i << "   with CAM: " << cam_j << std::endl;
      return vw::Vector2(-999999,-999999);
    }
  }

  /// Write the adjusted camera at the given index to disk
  void write_adjustment(int j, std::string const& filename) const {
    vw::Vector3 position_correction;
    vw::Quat pose_correction;
    asp::parse_camera_parameters(m_cam_vec[j], position_correction, pose_correction);
    asp::write_adjustments(filename, position_correction, pose_correction);
  }

  /// Get a vector containing all of the adjusted camera models
  std::vector<cam_ptr_t> adjusted_cameras() const {
    std::vector<cam_ptr_t> result(m_cameras.size());
    for (unsigned j = 0; j < result.size(); ++j) {
      vw::Vector3 position_correction;
      vw::Quat pose_correction;
      asp::parse_camera_parameters(m_cam_vec[j], position_correction, pose_correction);
      result[j] = cam_ptr_t( new vw::camera::AdjustedCameraModel( m_cameras[j], position_correction, 
                                                                                pose_correction ) );
    }
    return result;
  }

  inline double image_compare( vw::Vector2 const& meas,
                               vw::Vector2 const& obj ) {
    return norm_2( meas - obj );
  }

  inline double position_compare( camera_vector_t const& meas,
                                  camera_vector_t const& obj ) {
    return norm_2( subvector(meas,0,3) - subvector(obj,0,3) );
  }

  inline double pose_compare( camera_vector_t const& meas,
                              camera_vector_t const& obj ) {
    return norm_2( subvector(meas,3,3) - subvector(obj,3,3) );
  }

  inline double gcp_compare( point_vector_t const& meas,
                             point_vector_t const& obj ) {
    return norm_2(meas - obj);
  }

  // Give access to the control network
  boost::shared_ptr<vw::ba::ControlNetwork> control_network() const {
    return m_network;
  }

  void bundlevis_cameras_append(std::string const& filename) const {
    std::ofstream ostr(filename.c_str(),std::ios::app);
    for ( unsigned j = 0; j < m_cam_vec.size(); j++ ) {
      vw::Vector3 position_correction;
      vw::Quat pose_correction;
      asp::parse_camera_parameters(m_cam_vec[j], position_correction, pose_correction);
      vw::camera::AdjustedCameraModel cam(m_cameras[j],
                                          position_correction,
                                          pose_correction);
      vw::Vector3 position = cam.camera_center( vw::Vector2() );
      vw::Quat pose = cam.camera_pose( vw::Vector2() );
      ostr << std::setprecision(18) << j << "\t" << position[0] << "\t"
           << position[1] << "\t" << position[2] << "\t";
      ostr << pose[0] << "\t" << pose[1] << "\t"
           << pose[2] << "\t" << pose[3] << "\n";
    }
  }

  void bundlevis_points_append(std::string const& filename) const {
    std::ofstream ostr(filename.c_str(),std::ios::app);
    unsigned i = 0;
    BOOST_FOREACH( point_vector_t const& p, m_point_vec ) {
      ostr << i++ << std::setprecision(18) << "\t" << p[0] << "\t"
           << p[1] << "\t" << p[2] << "\n";
    }
  }
}; // End class BundleAdjustmentModel

/// Model to be used to float all parameters of a pinhole model.  There
/// are 6 camera parameters, corresponding to: camera center (3), and
/// camera orientation (3). Also there are three intrinsic parameters:
/// focal length (1), and pixel offsets (2), which are shared
/// among the cameras.
/// - Accomodate any lens distortion model.
/// - Adjusting the input intrinsic values is not currently supported.
/// - If we want to enable solving for intrinsics, we should drop our old
///   bundle adjust code so we only have to fix things once.
class BAPinholeModel : public vw::ba::ModelBase<BAPinholeModel, 6, 3> {

public: // Definitions

  typedef vw::Vector<double,camera_params_n>          camera_vector_t;
  typedef vw::Vector<double,point_params_n >          point_vector_t;
  typedef boost::shared_ptr<vw::camera::PinholeModel> pin_cam_ptr_t;
  typedef boost::shared_ptr<vw::camera::CameraModel>  cam_ptr_t;
  // Currently all the intrinsic parameters are kept hidden within the class.
  const static size_t intrinsic_params_n = 0;
  typedef vw::Vector<double> intrinsic_vector_t; ///< Vector containing intrinsic parameters
  typedef vw::Vector<double,camera_params_n
                     +intrinsic_params_n> camera_intr_vector_t; ///< Vector containing all parameters
  
  // TODO: We don't solve from scratch, remove this!
  // Need this scale to force the rotations to not change that wildly
  // when determining pinhole cameras from scratch.
  const static double pose_scale = 1.0;//1.0e+3;
  
private: // Variables

  std::vector<cam_ptr_t>                    m_cameras; ///< Input camera models, barely used.
  boost::shared_ptr<vw::ba::ControlNetwork> m_network; ///< Little used control network pointer
  std::vector<camera_intr_vector_t>         m_cam_vec; ///< Vector of param vectors for each camera
  intrinsic_vector_t                        m_shared_intrinsics; ///< Record shared intrinsic values
  boost::shared_ptr<vw::camera::LensDistortion>  m_shared_lens_distortion; ///< Copy of input lens distortion object

public:
  /// Contructor requires a list of input pinhole models
  /// - Set constant_intrinsics if you do not want them changed
  BAPinholeModel(std::vector<cam_ptr_t> const& cameras,
                 boost::shared_ptr<vw::ba::ControlNetwork> network) :
    m_cameras(cameras), m_network(network), m_cam_vec(cameras.size()){

    // Copy the (shared) intrinsic values from the first camera
    const pin_cam_ptr_t pinhole_ptr = boost::dynamic_pointer_cast<vw::camera::PinholeModel>(cameras[0]);
    intrinsic_params_from_model(*pinhole_ptr, 
                                m_shared_intrinsics);
    // Make a copy of the lens distortion model object
    m_shared_lens_distortion =pinhole_ptr->lens_distortion()->copy();
  
    // Copy all of the input camera model information to the internal 
    //  camera parameters vector
    // - The input cameras must all be pinhole models or this will fail.
    for (unsigned int i=0; i<num_cameras(); ++i) {
      cam_ptr_t     cam_ptr = cameras[i];
      pin_cam_ptr_t pin_ptr = boost::dynamic_pointer_cast<vw::camera::PinholeModel>(cam_ptr);
      if (!pin_ptr)
        vw::vw_throw( vw::ArgumentErr() 
                      << "Non-pinhole camera passed to BAPinholeModel constructor!\n" );
      params_from_model(*pin_ptr, m_cam_vec[i]);
      std::cout << "BAPinhole input model: " << *pin_ptr << std::endl;
      std::cout << "Set up BAPinhole model params: " << m_cam_vec[i] << std::endl;
    }
  
  } // End constructor

  bool are_intrinsics_constant() const {return true; }

  unsigned num_cameras() const { return m_cameras.size();  }
  unsigned num_points () const { return m_network->size(); }

  void set_cam_params(int j, camera_intr_vector_t const& cam_j)       { m_cam_vec[j] = cam_j;       }
  void get_cam_params(int j, camera_intr_vector_t      & cam_j) const { cam_j        = m_cam_vec[j];}

  /// Return the i'th camera model using the current parameters
  vw::camera::PinholeModel get_camera_model(int icam) {
    return params_to_model(m_cam_vec[icam]);
  }

  /// Generate a pinhole camera model given input parametrs
  vw::camera::PinholeModel params_to_model(camera_intr_vector_t const& cam_vec){

    camera_intr_vector_t scaled_cam_vec = cam_vec;

    // Undo the scale of the rotation variables
    subvector(scaled_cam_vec,3,3) /= pose_scale;

    vw::Vector3 position;
    vw::Quat    pose;
    asp::parse_camera_parameters(scaled_cam_vec, position, pose);

    // Make a new PinholeModel object with a copy of the shared distortion model
    return vw::camera::PinholeModel(position,
                                    pose.rotation_matrix(),
                                    m_shared_intrinsics[0], m_shared_intrinsics[0], // focal lengths
                                    m_shared_intrinsics[1], m_shared_intrinsics[2], // pixel offsets
                                    *(m_shared_lens_distortion.get()),
                                     m_shared_intrinsics[3]); //pixel pitch

  }

  /// Extract the intrinsic parameters only from a model
  /// - Currently we pull them all out, but we never look at most of them again.
  void intrinsic_params_from_model(const vw::camera::PinholeModel & model,
                                         intrinsic_vector_t       & cam_vec) const{

    const size_t NUM_PINHOLE_INTRINSICS = 4; // Focal length and focal point x,y offset, pixel_pitch.
    
    // Get the lens distortion parameters and allocate the output vector
    vw::Vector<double> lens_distortion_params = model.lens_distortion()->distortion_parameters();
    const size_t num_lens_params = lens_distortion_params.size();
    cam_vec.set_size(num_lens_params + NUM_PINHOLE_INTRINSICS);
    
    // Get the parameters that are the same for all pinhole cameras
    vw::Vector2 fl = model.focal_length();
    vw::Vector2 po = model.point_offset();
    cam_vec[0] = fl[0];
    cam_vec[1] = po[0];
    cam_vec[2] = po[1];
    cam_vec[3] = model.pixel_pitch();
    
    // Copy the lens distortion parameters -> Not currently used!
    for (size_t i=0; i<num_lens_params; ++i)
      cam_vec[i+NUM_PINHOLE_INTRINSICS] = lens_distortion_params[i];
  }

  /// Extract the camera parameter vector from a pinhole model
  void params_from_model(const vw::camera::PinholeModel & model,
                               camera_intr_vector_t     & cam_vec){

    vw::Vector3 position = model.camera_center();
    vw::Quat    pose     = model.camera_pose();

    // Set the extrinsics (params 0-5)
    asp::set_camera_params(cam_vec, position, pose);
    // Scale the rotation variables
    subvector(cam_vec,3,3) *= pose_scale;

    // We don't import any of the intrinsic params here.
  }


  /// Given the 'cam_vec' vector (camera model parameters) for the j'th
  /// image, and the 'm_point_vec' vector (3D point location) for the i'th
  /// point, return the location of point_i on imager j in pixel coordinates.
  /// - For this class we modify the camera model directly instead of using 
  ///   the AdjustedCameraModel class.
  vw::Vector2 cam_pixel(unsigned /*i*/, unsigned j,
			camera_intr_vector_t const& cam_j,
			point_vector_t       const& point_i) {

    try {
      vw::camera::PinholeModel model = params_to_model(cam_j);
      //std::cout << "cam_pixel: " << point_i << " --> " << model.point_to_pixel(point_i) << std::endl;
      return model.point_to_pixel(point_i);
    }
    catch(...) { // If the camera parameters were bad, return a garbage pixel instead of crashing
      //std::cout << "MISSED point at: " << point_i << "   with CAM: " << cam_j << std::endl;
      return vw::Vector2(-999999,-999999);
    }
    
  }

  /// Give access to the control network
  boost::shared_ptr<vw::ba::ControlNetwork> control_network() const {
    return m_network;
  }

  /// Write complete camera models to disk as opposed to adjustment files
  void write_camera_models(std::vector<std::string> const& cam_files){

    VW_ASSERT(cam_files.size() == m_cam_vec.size(),
                  vw::ArgumentErr() << "Must have as many camera files as cameras.\n");

    for (int icam = 0; icam < (int)cam_files.size(); icam++){
      vw::vw_out() << "Writing: " << cam_files[icam] << std::endl;
      vw::camera::PinholeModel model = get_camera_model(icam);
      model.write(cam_files[icam]);
      //std::cout << "Writing BAPinhole model params: " << m_cam_vec[icam] << std::endl;
      std::cout << "Writing output model: " << model << std::endl;      
    }

  }
  
}; // End class BAPinholeModel

#endif // __ASP_TOOLS_BUNDLEADJUST_H__
