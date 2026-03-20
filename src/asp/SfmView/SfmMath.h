// __BEGIN_LICENSE__
//  Copyright (c) 2006-2026, United States Government as represented by the
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

// Derived from MVE (Simon Fuhrmann, TU Darmstadt, BSD 3-Clause).

#ifndef __ASP_SFMVIEW_SFM_MATH_H__
#define __ASP_SFMVIEW_SFM_MATH_H__

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <algorithm>
#include <cmath>
#include <memory>
#include <vector>

namespace sfm {

// Clamp value to range [lo, hi]
template <typename T>
T clamp(T const& v, T const& lo, T const& hi) {
  return std::min(hi, std::max(lo, v));
}

// Multiply a 4x4 matrix by a 3-vector with homogeneous w coordinate.
// Returns upper-left 3x3 block times v, plus w times the translation column.
template <typename T>
Eigen::Matrix<T, 3, 1>
mult_homogeneous(Eigen::Matrix<T, 4, 4> const& m,
                 Eigen::Matrix<T, 3, 1> const& v, T w) {
  return m.template block<3, 3>(0, 0) * v +
         w * m.template block<3, 1>(0, 3);
}

// OpenGL symmetric projection matrix (column-major for GL)
template <typename T>
Eigen::Matrix<T, 4, 4>
matrix_gl_projection(T const& znear, T const& zfar,
                     T const& top, T const& right) {
  Eigen::Matrix<T, 4, 4> proj = Eigen::Matrix<T, 4, 4>::Zero();
  proj(0, 0) = znear / right;
  proj(1, 1) = znear / top;
  proj(2, 2) = -(zfar + znear) / (zfar - znear);
  proj(2, 3) = T(-2) * zfar * znear / (zfar - znear);
  proj(3, 2) = T(-1);
  return proj;
}

// View transformation matrix from camera pos, viewing direction, up vector
template <typename T>
Eigen::Matrix<T, 4, 4>
matrix_viewtrans(Eigen::Matrix<T, 3, 1> const& campos,
                 Eigen::Matrix<T, 3, 1> const& viewdir,
                 Eigen::Matrix<T, 3, 1> const& upvec) {
  Eigen::Matrix<T, 3, 1> z = -viewdir;
  Eigen::Matrix<T, 3, 1> x = upvec.cross(z).normalized();
  Eigen::Matrix<T, 3, 1> y = z.cross(x);

  // Start with identity, replace the rotation and translation parts
  Eigen::Matrix<T, 4, 4> m = Eigen::Matrix<T, 4, 4>::Identity();
  m(0, 0) = x[0]; m(0, 1) = x[1]; m(0, 2) = x[2];
  m(1, 0) = y[0]; m(1, 1) = y[1]; m(1, 2) = y[2];
  m(2, 0) = z[0]; m(2, 1) = z[1]; m(2, 2) = z[2];

  Eigen::Matrix<T, 3, 1> t = -campos;
  m(0, 3) = m(0, 0)*t[0] + m(0, 1)*t[1] + m(0, 2)*t[2];
  m(1, 3) = m(1, 0)*t[0] + m(1, 1)*t[1] + m(1, 2)*t[2];
  m(2, 3) = m(2, 0)*t[0] + m(2, 1)*t[1] + m(2, 2)*t[2];
  return m;
}

// Inverse view transformation matrix
template <typename T>
Eigen::Matrix<T, 4, 4>
matrix_inverse_viewtrans(Eigen::Matrix<T, 3, 1> const& campos,
                         Eigen::Matrix<T, 3, 1> const& viewdir,
                         Eigen::Matrix<T, 3, 1> const& upvec) {
  Eigen::Matrix<T, 3, 1> z = -viewdir;
  Eigen::Matrix<T, 3, 1> x = upvec.cross(z).normalized();
  Eigen::Matrix<T, 3, 1> y = z.cross(x);

  // Start with identity, replace the rotation and translation parts
  Eigen::Matrix<T, 4, 4> m = Eigen::Matrix<T, 4, 4>::Identity();
  m(0, 0) = x[0]; m(0, 1) = y[0]; m(0, 2) = z[0]; m(0, 3) = campos[0];
  m(1, 0) = x[1]; m(1, 1) = y[1]; m(1, 2) = z[1]; m(1, 3) = campos[1];
  m(2, 0) = x[2]; m(2, 1) = y[2]; m(2, 2) = z[2]; m(2, 3) = campos[2];
  return m;
}

// Rotation matrix from axis and angle (Rodrigues)
template <typename T>
Eigen::Matrix<T, 3, 3>
matrix_rotation_from_axis_angle(Eigen::Matrix<T, 3, 1> const& axis,
                                T const& angle) {
  return Eigen::AngleAxis<T>(angle, axis.normalized()).toRotationMatrix();
}

// TriangleMesh

class TriangleMesh {
public:
  typedef std::shared_ptr<TriangleMesh> Ptr;
  typedef std::shared_ptr<TriangleMesh const> ConstPtr;

  typedef unsigned int VertexID;
  typedef std::vector<Eigen::Vector3f> VertexList;
  typedef std::vector<Eigen::Vector4f> ColorList;
  typedef std::vector<Eigen::Vector3f> NormalList;
  typedef std::vector<Eigen::Vector2f> TexCoordList;
  typedef std::vector<VertexID> FaceList;

  static Ptr create(void) { return Ptr(new TriangleMesh); }

  VertexList const& get_vertices(void) const { return vertices; }
  VertexList& get_vertices(void) { return vertices; }
  ColorList const& get_vertex_colors(void) const { return vertex_colors; }
  ColorList& get_vertex_colors(void) { return vertex_colors; }
  NormalList const& get_vertex_normals(void) const { return vertex_normals; }
  NormalList& get_vertex_normals(void) { return vertex_normals; }
  TexCoordList const& get_vertex_texcoords(void) const { return vertex_texcoords; }
  TexCoordList& get_vertex_texcoords(void) { return vertex_texcoords; }
  FaceList const& get_faces(void) const { return faces; }
  FaceList& get_faces(void) { return faces; }

private:
  VertexList vertices;
  ColorList vertex_colors;
  NormalList vertex_normals;
  TexCoordList vertex_texcoords;
  FaceList faces;
};

// Camera with viewing and projection matrices for OpenGL rendering.
class SfmCamera {
public:
  Eigen::Vector3f pos;
  Eigen::Vector3f viewing_dir;
  Eigen::Vector3f up_vec;

  float z_near;
  float z_far;
  float top;
  float right;

  int width;
  int height;

  Eigen::Matrix4f view;
  Eigen::Matrix4f proj;

  SfmCamera(void);
  void update_view_mat(void);
  void update_proj_mat(void);
};

inline SfmCamera::SfmCamera(void):
  pos(0.0f, 0.0f, 5.0f),
  viewing_dir(0.0f, 0.0f, -1.0f),
  up_vec(0.0f, 1.0f, 0.0f),
  z_near(0.1f), z_far(500.0f),
  top(0.1f), right(0.1f),
  width(0), height(0) {
  view = Eigen::Matrix4f::Zero();
  proj = Eigen::Matrix4f::Zero();
}

inline void SfmCamera::update_view_mat(void) {
  this->view = sfm::matrix_viewtrans(this->pos,
    this->viewing_dir, this->up_vec);
}

inline void SfmCamera::update_proj_mat(void) {
  this->proj = sfm::matrix_gl_projection(this->z_near,
    this->z_far, this->top, this->right);
}

} // namespace sfm

#endif // __ASP_SFMVIEW_SFM_MATH_H__
