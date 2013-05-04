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


/// \file CameraAdjust.h
///

#ifndef __BUNDLE_ADJUST_H__
#define __BUNDLE_ADJUST_H__

#include <vw/Math/Vector.h>
#include <vw/Math/Quaternion.h>
#include <vw/Camera/CameraModel.h>
#include <vw/Stereo/StereoModel.h>

#include <boost/shared_ptr.hpp>

namespace vw {
namespace camera {

  class TransformedCameraModel : public CameraModel {

    boost::shared_ptr<CameraModel> m_camera;
    Vector3 m_translation;
    Quaternion<double> m_rotation;
    Quaternion<double> m_rotation_inverse;

  public:
    TransformedCameraModel(boost::shared_ptr<CameraModel> camera_model) : m_camera(camera_model) {
      m_rotation = math::Quaternion<double>(math::identity_matrix<3>());
      m_rotation_inverse = math::Quaternion<double>(math::identity_matrix<3>());
    }

    virtual ~TransformedCameraModel() {}

    Vector3 translation() const { return m_translation; }
    Quaternion<double> rotation() const { return m_rotation; }
    Matrix<double,3,3> rotation_matrix() const { return m_rotation.rotation_matrix(); }

    void set_translation(Vector3 const& translation) { m_translation = translation; }
    void set_rotation(Quaternion<double> const& rotation) {
      m_rotation = rotation;
      m_rotation_inverse = inverse(m_rotation);
    }
    void set_rotation(Matrix<double,3,3> const& rotation) {
      m_rotation = Quaternion<double>(rotation);
      m_rotation_inverse = inverse(m_rotation);
    }

    virtual Vector2 point_to_pixel (Vector3 const& point) const {
      Vector2 original_pix = m_camera->point_to_pixel(point);
      Vector3 cam_center = m_camera->camera_center(original_pix);
      Vector3 vec = point-cam_center;
      return m_camera->point_to_pixel(m_rotation.rotate(vec) + this->camera_center(original_pix));  // Is this correct?
    }

    virtual Vector3 pixel_to_vector (Vector2 const& pix) const {
      return m_rotation_inverse.rotate(m_camera->pixel_to_vector(pix));
    }

    virtual Vector3 camera_center (Vector2 const& pix) const {
      return m_camera->camera_center(pix) + m_translation;
    }

    virtual Quaternion<double> camera_pose(Vector2 const& pix) const {
      return m_camera->camera_pose(pix)*m_rotation_inverse;
    }

  };
}

class CameraPointingOptimizeFunc {

  vw::camera::TransformedCameraModel m_camera1, m_camera2;
  vw::stereo::StereoModel m_stereo_model;
  std::vector<Vector2> m_pixel_list1, m_pixel_list2;

public:
  CameraPointingOptimizeFunc(boost::shared_ptr<camera::CameraModel> camera1,
                             boost::shared_ptr<camera::CameraModel> camera2,
                             std::vector<Vector2> pixel_list1,
                             std::vector<Vector2> pixel_list2) :
    m_camera1(camera1), m_camera2(camera2),
    m_stereo_model(m_camera1, m_camera2),
    m_pixel_list1(pixel_list1), m_pixel_list2(pixel_list2) {

    VW_ASSERT(m_pixel_list1.size() == m_pixel_list2.size(),
              ArgumentErr() << "CameraPointingOptimizeFunc: pixel lists are not the same size...");
  }

  double operator()(Vector<double,8> const& quaternions) {
    Vector4 q1 = normalize(subvector(quaternions,0,4));
    Vector4 q2 = normalize(subvector(quaternions,4,4));
    m_camera1.set_rotation(Quaternion<double>(q1[0], q1[1], q1[2], q1[3]));
    m_camera2.set_rotation(Quaternion<double>(q2[0], q2[1], q2[2], q2[3]));

    vw::stereo::StereoModel test_model( &m_camera1,
                                        &m_camera2);

    double error;
    double total_error = 0;
    for (unsigned i = 0; i < m_pixel_list1.size(); ++i) {
      test_model(m_pixel_list1[i], m_pixel_list2[i], error);
      total_error += error;
    }
    return total_error;
  }

};

class CameraToGroundOptimizePoseFunc {

  vw::camera::TransformedCameraModel m_camera;
  std::vector<Vector3> m_ground_pts;
  std::vector<Vector2> m_image_pts;

public:
  CameraToGroundOptimizePoseFunc(boost::shared_ptr<camera::CameraModel> camera,
                             std::vector<Vector3> ground_pts,
                             std::vector<Vector2> image_pts) :
    m_camera(camera), m_ground_pts(ground_pts), m_image_pts(image_pts) {

    VW_ASSERT(m_ground_pts.size() == m_image_pts.size(),
              ArgumentErr() << "CameraToGroundOptimizeFunc: ground and image point lists are not the same length.");
    VW_ASSERT(m_ground_pts.size() >= 4,
              ArgumentErr() << "CameraToGroundOptimizeFunc: not enought tie points (" << m_ground_pts.size() << " found and at least 4 are needed.");
  }

  double operator()(Vector<double,4> const& quaternion) {
    Vector4 q1 = normalize(quaternion);
    m_camera.set_rotation(Quaternion<double>(q1[0], q1[1], q1[2], q1[3]));

    double total_error = 0;
    for (unsigned i = 0; i < m_ground_pts.size(); ++i) {
      Vector2 image_pix = m_camera.point_to_pixel(m_ground_pts[i]);
      total_error += norm_2(image_pix - m_image_pts[i]);
    }
    return total_error;
  }

};


class CameraToGroundOptimizePositionFunc {

  vw::camera::TransformedCameraModel m_camera;
  std::vector<Vector3> m_ground_pts;
  std::vector<Vector2> m_image_pts;

public:
  CameraToGroundOptimizePositionFunc(boost::shared_ptr<camera::CameraModel> camera,
                             std::vector<Vector3> ground_pts,
                             std::vector<Vector2> image_pts) :
    m_camera(camera), m_ground_pts(ground_pts), m_image_pts(image_pts) {

    VW_ASSERT(m_ground_pts.size() == m_image_pts.size(),
              ArgumentErr() << "CameraToGroundOptimizeFunc: ground and image point lists are not the same length.");
    VW_ASSERT(m_ground_pts.size() >= 4,
              ArgumentErr() << "CameraToGroundOptimizeFunc: not enought tie points (" << m_ground_pts.size() << " found and at least 4 are needed.");
  }

  double operator()(Vector<double,3> const& translation) {
    m_camera.set_translation(translation);

    double total_error = 0;
    for (unsigned i = 0; i < m_ground_pts.size(); ++i) {
      Vector2 image_pix = m_camera.point_to_pixel(m_ground_pts[i]);
      total_error += norm_2(image_pix - m_image_pts[i]);
    }
    return total_error;
  }

};

class CameraToGroundOptimizeFunc {

  vw::camera::TransformedCameraModel m_camera;
  std::vector<Vector3> m_ground_pts;
  std::vector<Vector2> m_image_pts;

public:
  CameraToGroundOptimizeFunc(boost::shared_ptr<camera::CameraModel> camera,
                             std::vector<Vector3> ground_pts,
                             std::vector<Vector2> image_pts) :
    m_camera(camera), m_ground_pts(ground_pts), m_image_pts(image_pts) {

    VW_ASSERT(m_ground_pts.size() == m_image_pts.size(),
              ArgumentErr() << "CameraToGroundOptimizeFunc: ground and image point lists are not the same length.");
    VW_ASSERT(m_ground_pts.size() >= 4,
              ArgumentErr() << "CameraToGroundOptimizeFunc: not enought tie points (" << m_ground_pts.size() << " found and at least 4 are needed.");
  }

  double operator()(Vector<double,7> const& vals) {
    Vector4 q1 = normalize(subvector(vals, 0, 4));
    m_camera.set_rotation(Quaternion<double>(q1[0], q1[1], q1[2], q1[3]));
    m_camera.set_translation(subvector(vals, 4,3));

    double total_error = 0;
    for (unsigned i = 0; i < m_ground_pts.size(); ++i) {
      Vector2 image_pix = m_camera.point_to_pixel(m_ground_pts[i]);
      total_error += norm_2(image_pix - m_image_pts[i]);
    }
    return total_error;
  }

};




inline void create_bundle_adjustment_pixel_list(ImageView<PixelDisparity<double> > const& disparity_map,
                                                int step_size,
                                                std::vector<Vector2> &pixel_list1,
                                                std::vector<Vector2> &pixel_list2) {
  for (int j = 0; j < disparity_map.rows(); j += step_size) {
    for (int i = 0; i < disparity_map.cols(); i += step_size) {
      if (!disparity_map(i,j).missing()) {
        pixel_list1.push_back(Vector2(i,j));
        pixel_list2.push_back(Vector2(i+disparity_map(i,j).h(),
                                      j+disparity_map(i,j).v()));
      }
    }
  }
}

}
#endif // __BUNDLE_ADJUST_H__
