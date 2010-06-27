// __BEGIN_LICENSE__
// Copyright (C) 2006-2010 United States Government as represented by
// the Administrator of the National Aeronautics and Space Administration.
// All Rights Reserved.
// __END_LICENSE__

#ifndef __ASP_TOOLS_BUNDLEADJUST_H__
#define __ASP_TOOLS_BUNDLEADJUST_H__

#include <boost/algorithm/string.hpp>
#include <boost/program_options.hpp>
#include <boost/filesystem/operations.hpp>
#include <boost/filesystem/fstream.hpp>

#include <vw/Camera/CAHVORModel.h>
#include <vw/BundleAdjustment.h>
#include <vw/Math.h>

#include <stdlib.h>
#include <iostream>

#include <asp/Sessions.h>
#include <asp/Core/BundleAdjustUtils.h>
#include <asp/Core/StereoSettings.h>

// Bundle adjustment functor
class BundleAdjustmentModel : public vw::ba::ModelBase<BundleAdjustmentModel, 6, 3> {

  typedef vw::Vector<double,6> camera_vector_t;
  typedef vw::Vector<double,3> point_vector_t;

  std::vector<boost::shared_ptr<vw::camera::CameraModel> > m_cameras;
  boost::shared_ptr<vw::ba::ControlNetwork> m_network;

  std::vector<camera_vector_t> a;
  std::vector<point_vector_t> b;
  std::vector<camera_vector_t> a_target;
  std::vector<point_vector_t> b_target;
  int m_num_pixel_observations;

public:
  BundleAdjustmentModel(std::vector<boost::shared_ptr<vw::camera::CameraModel> > const& cameras,
                        boost::shared_ptr<vw::ba::ControlNetwork> network) :
    m_cameras(cameras), m_network(network), a(cameras.size()),
    b(network->size()), a_target(cameras.size()), b_target(network->size()) {

    // Compute the number of observations from the bundle.
    m_num_pixel_observations = 0;
    for (unsigned i = 0; i < network->size(); ++i)
      m_num_pixel_observations += (*network)[i].size();

    // Set up the b vectors, storing the initial values.
    // a vector however just starts out zero
    for (unsigned i = 0; i < network->size(); ++i) {
      b[i] = (*m_network)[i].position();
      b_target[i] = b[i];
    }
  }

  // Return a reference to the camera and point parameters.
  camera_vector_t A_parameters(int j) const { return a[j]; }
  point_vector_t B_parameters(int i) const { return b[i]; }
  void set_A_parameters(int j, camera_vector_t const& a_j) {
    a[j] = a_j;
  }
  void set_B_parameters(int i, point_vector_t const& b_i) {
    b[i] = b_i;
  }

  // Return the initial parameters
  camera_vector_t A_target(int j) const { return a_target[j]; }
  point_vector_t B_target(int i) const { return b_target[i]; }

  unsigned num_cameras() const { return a.size(); }
  unsigned num_points() const { return b.size(); }
  unsigned num_pixel_observations() const { return m_num_pixel_observations; }

  // Return the covariance of the camera parameters for camera j.
  inline vw::Matrix<double,camera_params_n,camera_params_n>
  A_inverse_covariance ( unsigned /*j*/ ) const {
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
  B_inverse_covariance ( unsigned /*i*/ ) const {
    vw::Matrix<double,point_params_n,point_params_n> result;
    result(0,0) = 1/20;
    result(1,1) = 1/20;
    result(2,2) = 1/20;
    return result;
  }

  void parse_camera_parameters(camera_vector_t a_j,
                               vw::Vector3 &position_correction,
                               vw::Quat &pose_correction) const {
    position_correction = subvector(a_j, 0, 3);
    pose_correction = axis_angle_to_quaternion( subvector(a_j,3,3) );
  }

  // Given the 'a' vector (camera model parameters) for the j'th
  // image, and the 'b' vector (3D point location) for the i'th
  // point, return the location of b_i on imager j in pixel
  // coordinates.
  vw::Vector2 operator() ( unsigned /*i*/, unsigned j,
                           camera_vector_t const& a_j,
                           point_vector_t const& b_i ) const {
    vw::Vector3 position_correction;
    vw::Quat pose_correction;
    parse_camera_parameters(a_j, position_correction, pose_correction);
    vw::camera::AdjustedCameraModel cam(m_cameras[j],
                                        position_correction,
                                        pose_correction);
    return cam.point_to_pixel(b_i);
  }

  void write_adjustment(int j, std::string const& filename) const {
    vw::Vector3 position_correction;
    vw::Quat pose_correction;
    parse_camera_parameters(a[j], position_correction, pose_correction);
    write_adjustments(filename, position_correction, pose_correction);
  }

  std::vector<boost::shared_ptr<vw::camera::CameraModel> >
  adjusted_cameras() const {
    std::vector<boost::shared_ptr<vw::camera::CameraModel> > result(m_cameras.size());
    for (unsigned j = 0; j < result.size(); ++j) {
      vw::Vector3 position_correction;
      vw::Quat pose_correction;
      parse_camera_parameters(a[j], position_correction, pose_correction);
      result[j] = boost::shared_ptr<vw::camera::CameraModel>( new vw::camera::AdjustedCameraModel( m_cameras[j], position_correction, pose_correction ) );
    }
    return result;
  }

  // Errors on the image plane
  void image_errors( std::vector<double>& pix_errors ) {
    pix_errors.clear();
    for (unsigned i = 0; i < m_network->size(); ++i)
      for(unsigned m = 0; m < (*m_network)[i].size(); ++m) {
        int camera_idx = (*m_network)[i][m].image_id();
        vw::Vector2 pixel_error;
        try {
          pixel_error = (*m_network)[i][m].position() -
            (*this)(i, camera_idx,a[camera_idx],b[i]);
        } catch ( vw::camera::PixelToRayErr &e ) {}
        pix_errors.push_back(norm_2(pixel_error));
      }
  }

  // Errors for camera position
  void camera_position_errors( std::vector<double>& camera_position_errors ) {
    camera_position_errors.clear();
    for (unsigned j=0; j < this->num_cameras(); ++j) {
      vw::Vector3 position_initial, position_now;
      vw::Quat pose_initial, pose_now;

      parse_camera_parameters(a_target[j], position_initial, pose_initial);
      parse_camera_parameters(a[j], position_now, pose_now);

      camera_position_errors.push_back(norm_2(position_initial-position_now));
    }
  }

  // Errors for camera pose
  void camera_pose_errors( std::vector<double>& camera_pose_errors ) {
    camera_pose_errors.clear();
    for (unsigned j=0; j < this->num_cameras(); ++j) {
      vw::Vector3 position_initial, position_now;
      vw::Quat pose_initial, pose_now;

      parse_camera_parameters(a_target[j], position_initial, pose_initial);
      parse_camera_parameters(a[j], position_now, pose_now);

      vw::Vector3 axis_initial, axis_now;
      double angle_initial, angle_now;
      pose_initial.axis_angle(axis_initial, angle_initial);
      pose_now.axis_angle(axis_now, angle_now);

      camera_pose_errors.push_back(fabs(angle_initial-angle_now) * 180.0/M_PI);
    }
  }

  // Errors for gcp errors
  void gcp_errors( std::vector<double>& gcp_errors ) {
    gcp_errors.clear();
    for (unsigned i=0; i < this->num_points(); ++i)
      if ( (*m_network)[i].type() ==
           vw::ba::ControlPoint::GroundControlPoint ) {
        point_vector_t p1 = b_target[i]/b_target[i](3);
        point_vector_t p2 = b[i]/b[i](3);
        gcp_errors.push_back(norm_2(subvector(p1,0,3) - subvector(p2,0,3)));
      }
  }

  // Give access to the control network
  boost::shared_ptr<vw::ba::ControlNetwork> control_network(void) {
    return m_network;
  }

  void bundlevis_cameras_append(std::string const& filename) {
    std::ofstream ostr(filename.c_str(),std::ios::app);
    for ( unsigned j = 0; j < a.size(); j++ ) {
      vw::Vector3 position_correction;
      vw::Quat pose_correction;
      parse_camera_parameters(a[j], position_correction, pose_correction);
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

  void bundlevis_points_append(std::string const& filename) {
    std::ofstream ostr(filename.c_str(),std::ios::app);
    unsigned i = 0;
    BOOST_FOREACH( point_vector_t const& p, b ) {
      ostr << i++ << std::setprecision(18) << "\t" << p[0] << "\t"
           << p[1] << "\t" << p[2] << "\n";
    }
  }
};

#endif//__ASP_TOOLS_BUNDLEADJUST_H__
