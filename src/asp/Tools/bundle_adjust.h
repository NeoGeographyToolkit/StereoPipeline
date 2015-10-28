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
#include <vw/BundleAdjustment.h>
#include <vw/Math.h>

#include <stdlib.h>
#include <iostream>

#include <asp/Core/BundleAdjustUtils.h>

namespace asp{

  template<class camera_vector_t>
  void parse_camera_parameters(camera_vector_t cam_j,
                               vw::Vector3 &position_correction,
                               vw::Quat &pose_correction) {
    position_correction = subvector(cam_j, 0, 3);
    pose_correction = axis_angle_to_quaternion( subvector(cam_j,3,3) );
  }

  template<class ModelT>
  void concat_extrinsics_intrinsics(const double* const extrinsics,
                                    const double* const intrinsics,
                                    typename ModelT::camera_intr_vector_t & concat){
    int intr_len = ModelT::intrinsic_params_n, cam_len = ModelT::camera_params_n;
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
  typedef vw::Vector<double,camera_params_n> camera_vector_t;
  typedef vw::Vector<double,point_params_n> point_vector_t;
  typedef boost::shared_ptr<vw::camera::CameraModel> cam_ptr_t;
  // No intrinsic params
  const static size_t intrinsic_params_n = 0;
  typedef vw::Vector<double,intrinsic_params_n> intrinsic_vector_t;
  typedef vw::Vector<double,camera_params_n
                     +intrinsic_params_n> camera_intr_vector_t;
private:
  std::vector<cam_ptr_t> m_cameras;
  boost::shared_ptr<vw::ba::ControlNetwork> m_network;

  std::vector<camera_intr_vector_t> m_cam_vec;
  std::vector<point_vector_t> m_point_vec;
  std::vector<camera_vector_t> m_cam_target_vec;
  std::vector<point_vector_t> m_point_target_vec;
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
  camera_vector_t cam_params(int j) const { return m_cam_vec[j]; }
  point_vector_t point_params(int i) const { return m_point_vec[i]; }
  void set_cam_params(int j, camera_intr_vector_t const& cam_j) {
    m_cam_vec[j] = cam_j;
  }
  void set_point_params(int i, point_vector_t const& point_i) {
    m_point_vec[i] = point_i;
  }

  // Return the initial parameters
  camera_vector_t cam_target(int j) const { return m_cam_target_vec[j]; }
  point_vector_t point_target(int i) const { return m_point_target_vec[i]; }

  unsigned num_cameras() const { return m_cam_vec.size(); }
  unsigned num_points() const { return m_point_vec.size(); }
  unsigned num_pixel_observations() const { return m_num_pixel_observations; }

  // Return the covariance of the camera parameters for camera j.
  inline vw::Matrix<double,camera_params_n,camera_params_n>
  cam_inverse_covariance ( unsigned /*j*/ ) const {
    vw::Matrix<double,camera_params_n,camera_params_n> result;
    result(0,0) = 1/100.0;
    result(1,1) = 1/100.0;
    result(2,2) = 1/100.0;
    result(3,3) = 1/1e-1;
    result(4,4) = 1/1e-1;
    result(5,5) = 1/1e-1;
    return result;
  }

  // Return the covariance of the point parameters for point i.
  inline vw::Matrix<double,point_params_n,point_params_n>
  point_inverse_covariance ( unsigned /*i*/ ) const {
    vw::Matrix<double,point_params_n,point_params_n> result;
    result(0,0) = 1/20;
    result(1,1) = 1/20;
    result(2,2) = 1/20;
    return result;
  }

  // Given the camera model parameters for the j'th
  // image, and the 3D point location for the i'th
  // point, return the location of point_i on imager j in pixel
  // coordinates.
  vw::Vector2 cam_pixel(unsigned /*i*/, unsigned j,
			camera_vector_t const& cam_j,
			point_vector_t const& point_i) const {
    vw::Vector3 position_correction;
    vw::Quat pose_correction;
    asp::parse_camera_parameters(cam_j, position_correction, pose_correction);
    vw::camera::AdjustedCameraModel cam(m_cameras[j],
                                        position_correction,
                                        pose_correction);
    return cam.point_to_pixel(point_i);
  }

  void write_adjustment(int j, std::string const& filename) const {
    vw::Vector3 position_correction;
    vw::Quat pose_correction;
    asp::parse_camera_parameters(m_cam_vec[j], position_correction, pose_correction);
    asp::write_adjustments(filename, position_correction, pose_correction);
  }

  std::vector<cam_ptr_t> adjusted_cameras() const {
    std::vector<cam_ptr_t> result(m_cameras.size());
    for (unsigned j = 0; j < result.size(); ++j) {
      vw::Vector3 position_correction;
      vw::Quat pose_correction;
      asp::parse_camera_parameters(m_cam_vec[j], position_correction, pose_correction);
      result[j] = cam_ptr_t( new vw::camera::AdjustedCameraModel( m_cameras[j], position_correction, pose_correction ) );
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
};

// Model to be used to float all parameters of a pinhole model.  There
// are 6 camera parameters, corresponding to: camera center (3), and
// camera orientation (3). Also there are three intrinsic parameters:
// focal length (1), and pixel offsets (2), which are shared
// among the cameras.
class BAPinholeModel:
  public vw::ba::ModelBase<BAPinholeModel, 6, 3> {

public:
  typedef vw::Vector<double,camera_params_n> camera_vector_t;
  typedef vw::Vector<double,point_params_n> point_vector_t;
  typedef boost::shared_ptr<vw::camera::CameraModel> cam_ptr_t;
  // Three intrinsic params
  const static size_t intrinsic_params_n = 3;
  typedef vw::Vector<double,intrinsic_params_n> intrinsic_vector_t;
  typedef vw::Vector<double,camera_params_n
                     +intrinsic_params_n> camera_intr_vector_t;
  // Need this scale to force the rotations to not change that wildly
  // when determining pinhole cameras from scratch.
  const static double pose_scale = 1.0e+6;
private:
  std::vector<cam_ptr_t> m_cameras;
  boost::shared_ptr<vw::ba::ControlNetwork> m_network;
  std::vector<camera_intr_vector_t> m_cam_vec;

public:
  BAPinholeModel(std::vector<cam_ptr_t> const& cameras,
                 boost::shared_ptr<vw::ba::ControlNetwork> network) :
    m_cameras(cameras), m_network(network), m_cam_vec(cameras.size()){}

  unsigned num_cameras() const { return m_cameras.size(); }
  unsigned num_points() const { return m_network->size(); }

  void set_cam_params(int j, camera_intr_vector_t const& cam_j) {
    // Set the camera parameters.
    m_cam_vec[j] = cam_j;
  }

  vw::camera::PinholeModel get_pinhole_model(camera_intr_vector_t const& cam_vec){

    camera_intr_vector_t scaled_cam_vec = cam_vec;

    // Undo the scale of the rotation variables
    subvector(scaled_cam_vec,3,3) /= pose_scale;

    vw::Vector3 position;
    vw::Quat pose;
    asp::parse_camera_parameters(scaled_cam_vec, position, pose);

    return vw::camera::PinholeModel(position,
                                    pose.rotation_matrix(),
                                    scaled_cam_vec[6], scaled_cam_vec[6],  // focal lengths
                                    scaled_cam_vec[7], scaled_cam_vec[8]); // pixel offsets
  }

  // Given the 'cam_vec' vector (camera model parameters) for the j'th
  // image, and the 'm_point_vec' vector (3D point location) for the i'th
  // point, return the location of point_i on imager j in pixel
  // coordinates.
  vw::Vector2 cam_pixel(unsigned /*i*/, unsigned j,
			camera_intr_vector_t const& cam_j,
			point_vector_t const& point_i) {

    return get_pinhole_model(cam_j).point_to_pixel(point_i);
  }

  // Give access to the control network
  boost::shared_ptr<vw::ba::ControlNetwork> control_network() const {
    return m_network;
  }

  void write_camera_models(std::vector<std::string> const& cam_files){

    VW_ASSERT(cam_files.size() == m_cam_vec.size(),
                  vw::ArgumentErr() << "Must have as many camera files as cameras.\n");

    for (int icam = 0; icam < (int)cam_files.size(); icam++){
      vw::vw_out() << "Writing: " << cam_files[icam] << std::endl;
      get_pinhole_model(m_cam_vec[icam]).write(cam_files[icam]);
    }

  }


};

#endif // __ASP_TOOLS_BUNDLEADJUST_H__
