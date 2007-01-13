#ifndef __BUNDLE_ADJUST_H__
#define __BUNDLE_ADJUST_H__

#include <vw/Math/Vector.h>
#include <vw/Math/Quaternion.h>
#include <vw/Camera/CameraModel.h>

#include <boost/shared_ptr.hpp>

namespace vw {
namespace camera {

  class TransformedCameraModel : public CameraModel {
    
    const CameraModel* m_camera;
    Vector3 m_translation;
    Quaternion<double> m_rotation;
    Quaternion<double> m_rotation_inverse;
    
  public:
    TransformedCameraModel(CameraModel const& camera_model) : m_camera(&camera_model) {
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
      vw_throw(NoImplErr() << "TransformedCameraModel: point_to_pixel() not yet implemented.\n");
     }

    virtual Vector3 pixel_to_vector (Vector2 const& pix) const {
//       std::cout << m_rotation.rotation_matrix() << "\n";
//       std::cout << m_rotation_inverse.rotation_matrix() << "\n";
//      std::cout << m_rotation << "\n";
//       std::cout << m_rotation_inverse << "\n\n";
      
      return m_rotation_inverse.rotate(m_camera->pixel_to_vector(pix));
    }

    virtual Vector3 camera_center (Vector2 const& pix) const {
      return m_camera->camera_center(pix) + m_translation;
    }
  };
}}  
  
class CameraPointingOptimizeFunc {
  
  TransformedCameraModel m_camera1, m_camera2;
  StereoModel m_stereo_model;
  std::vector<Vector2> m_pixel_list1, m_pixel_list2;
  
public:
  CameraPointingOptimizeFunc(CameraModel const& camera1, 
                             CameraModel const& camera2,
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
    
    StereoModel test_model(m_camera1, m_camera2);

    double error;
    double total_error = 0;
    for (int i = 0; i < m_pixel_list1.size(); ++i) {
      test_model(m_pixel_list1[i], m_pixel_list2[i], error);
      total_error += error;
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


#endif // __BUNDLE_ADJUST_H__
