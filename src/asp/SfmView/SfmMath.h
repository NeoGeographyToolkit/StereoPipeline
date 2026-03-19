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

#include <algorithm>
#include <cmath>
#include <cstring>
#include <memory>
#include <vector>

namespace sfm {

// Vector

template <typename T, int N>
class Vector {
public:
  T v[N];

  Vector(void) { std::fill(v, v + N, T(0)); }
  explicit Vector(T const& val) { std::fill(v, v + N, val); }
  Vector(T const& v0, T const& v1, T const& v2)
  { v[0] = v0; v[1] = v1; v[2] = v2; }
  Vector(T const& v0, T const& v1, T const& v2, T const& v3)
  { v[0] = v0; v[1] = v1; v[2] = v2; v[3] = v3; }

  T& operator[](int i) { return v[i]; }
  T const& operator[](int i) const { return v[i]; }
  T* operator*(void) { return v; }
  T const* operator*(void) const { return v; }

  Vector operator+(Vector const& o) const
  { Vector r; for (int i = 0; i < N; i++) r[i] = v[i] + o[i]; return r; }
  Vector operator-(Vector const& o) const
  { Vector r; for (int i = 0; i < N; i++) r[i] = v[i] - o[i]; return r; }
  Vector operator-(void) const
  { Vector r; for (int i = 0; i < N; i++) r[i] = -v[i]; return r; }
  Vector operator*(T const& s) const
  { Vector r; for (int i = 0; i < N; i++) r[i] = v[i] * s; return r; }
  Vector operator/(T const& s) const
  { Vector r; for (int i = 0; i < N; i++) r[i] = v[i] / s; return r; }

  // Conversion from different scalar type
  template <typename U>
  Vector(Vector<U, N> const& o)
  { for (int i = 0; i < N; i++) v[i] = T(o[i]); }

  // Construction from raw pointer
  explicit Vector(T const* ptr)
  { for (int i = 0; i < N; i++) v[i] = ptr[i]; }

  Vector& operator+=(Vector const& o)
  { for (int i = 0; i < N; i++) v[i] += o[i]; return *this; }
  Vector& operator/=(T const& s)
  { for (int i = 0; i < N; i++) v[i] /= s; return *this; }

  bool operator!=(Vector const& o) const
  { for (int i = 0; i < N; i++) if (v[i] != o[i]) return true; return false; }

  T dot(Vector const& o) const
  { T r = T(0); for (int i = 0; i < N; i++) r += v[i] * o[i]; return r; }
  T norm(void) const { return std::sqrt(this->dot(*this)); }
  Vector& normalize(void)
  { T n = norm(); for (int i = 0; i < N; i++) v[i] /= n; return *this; }
  Vector normalized(void) const
  { Vector r(*this); r.normalize(); return r; }

  // Cross product (only for N=3)
  Vector cross(Vector const& o) const {
    return Vector(v[1]*o[2] - v[2]*o[1],
                  v[2]*o[0] - v[0]*o[2],
                  v[0]*o[1] - v[1]*o[0]);
  }
};

template <typename T, int N>
Vector<T, N> operator*(T const& s, Vector<T, N> const& v) { return v * s; }

typedef Vector<float, 2> Vec2f;
typedef Vector<float, 3> Vec3f;
typedef Vector<double, 3> Vec3d;
typedef Vector<float, 4> Vec4f;

// Matrix

template <typename T, int R, int C>
class Matrix {
public:
  T m[R * C];

  Matrix(void) { std::fill(m, m + R*C, T(0)); }
  explicit Matrix(T const& val) { std::fill(m, m + R*C, val); }
  explicit Matrix(T const* ptr) { std::copy(ptr, ptr + R*C, m); }

  T& operator[](int i) { return m[i]; }
  T const& operator[](int i) const { return m[i]; }
  T* operator*(void) { return m; }
  T const* operator*(void) const { return m; }

  T& operator()(int r, int c) { return m[r * C + c]; }
  T const& operator()(int r, int c) const { return m[r * C + c]; }

  // Matrix<R,C> * Vector<C> -> Vector<R>
  Vector<T, R> operator*(Vector<T, C> const& v) const {
    Vector<T, R> r;
    for (int i = 0; i < R; i++)
      for (int j = 0; j < C; j++)
        r[i] += m[i*C + j] * v[j];
    return r;
  }

  // Matrix<R,C> * Matrix<C,C2> -> Matrix<R,C2>
  template <int C2>
  Matrix<T, R, C2> mult(Matrix<T, C, C2> const& o) const {
    Matrix<T, R, C2> r;
    for (int i = 0; i < R; i++)
      for (int j = 0; j < C2; j++)
        for (int k = 0; k < C; k++)
          r(i, j) += m[i*C + k] * o(k, j);
    return r;
  }

  // Multiply 4x4 matrix by 3-vector with homogeneous w coordinate
  Vector<T, 3> mult(Vector<T, 3> const& v, T const& w) const {
    Vector<T, 3> r;
    for (int i = 0; i < 3; i++)
      r[i] = m[i*C]*v[0] + m[i*C+1]*v[1] + m[i*C+2]*v[2] + m[i*C+3]*w;
    return r;
  }

  // Multiply 3x3 matrix by 3-vector
  Vector<T, 3> mult(Vector<T, 3> const& v) const {
    Vector<T, 3> r;
    for (int i = 0; i < 3; i++)
      r[i] = m[i*C]*v[0] + m[i*C+1]*v[1] + m[i*C+2]*v[2];
    return r;
  }

  Matrix<T, C, R> transposed(void) const {
    Matrix<T, C, R> r;
    for (int i = 0; i < R; i++)
      for (int j = 0; j < C; j++)
        r(j, i) = m[i*C + j];
    return r;
  }
};

typedef Matrix<float, 3, 3> Matrix3f;
typedef Matrix<double, 3, 3> Matrix3d;
typedef Matrix<float, 4, 4> Matrix4f;

// Free functions

template <typename T>
T clamp(T const& v, T const& lo, T const& hi) {
  return std::min(hi, std::max(lo, v));
}

// OpenGL symmetric projection matrix
template <typename T>
Matrix<T, 4, 4>
matrix_gl_projection(T const& znear, T const& zfar,
    T const& top, T const& right) {
  Matrix<T, 4, 4> proj(T(0));
  proj(0,0) = znear / right;
  proj(1,1) = znear / top;
  proj(2,2) = -(zfar + znear) / (zfar - znear);
  proj(2,3) = T(-2) * zfar * znear / (zfar - znear);
  proj(3,2) = T(-1);
  return proj;
}

// View transformation matrix from camera pos, viewing direction, up vector
template <typename T>
Matrix<T, 4, 4>
matrix_viewtrans(Vector<T, 3> const& campos,
    Vector<T, 3> const& viewdir, Vector<T, 3> const& upvec) {
  Vector<T, 3> z(-viewdir);
  Vector<T, 3> x(upvec.cross(z).normalized());
  Vector<T, 3> y(z.cross(x));

  Matrix<T, 4, 4> m;
  m(0,0) = x[0]; m(0,1) = x[1]; m(0,2) = x[2]; m(0,3) = T(0);
  m(1,0) = y[0]; m(1,1) = y[1]; m(1,2) = y[2]; m(1,3) = T(0);
  m(2,0) = z[0]; m(2,1) = z[1]; m(2,2) = z[2]; m(2,3) = T(0);
  m(3,0) = T(0); m(3,1) = T(0); m(3,2) = T(0); m(3,3) = T(1);

  Vector<T, 3> t(-campos);
  m(0,3) = m(0,0)*t[0] + m(0,1)*t[1] + m(0,2)*t[2];
  m(1,3) = m(1,0)*t[0] + m(1,1)*t[1] + m(1,2)*t[2];
  m(2,3) = m(2,0)*t[0] + m(2,1)*t[1] + m(2,2)*t[2];
  return m;
}

// Inverse view transformation matrix
template <typename T>
Matrix<T, 4, 4>
matrix_inverse_viewtrans(Vector<T, 3> const& campos,
    Vector<T, 3> const& viewdir, Vector<T, 3> const& upvec) {
  Vector<T, 3> z(-viewdir);
  Vector<T, 3> x(upvec.cross(z).normalized());
  Vector<T, 3> y(z.cross(x));

  Matrix<T, 4, 4> m;
  m(0,0) = x[0]; m(0,1) = y[0]; m(0,2) = z[0]; m(0,3) = campos[0];
  m(1,0) = x[1]; m(1,1) = y[1]; m(1,2) = z[1]; m(1,3) = campos[1];
  m(2,0) = x[2]; m(2,1) = y[2]; m(2,2) = z[2]; m(2,3) = campos[2];
  m(3,0) = T(0); m(3,1) = T(0); m(3,2) = T(0); m(3,3) = T(1);
  return m;
}

// Rotation matrix from axis and angle (Rodrigues)
template <typename T>
Matrix<T, 3, 3>
matrix_rotation_from_axis_angle(Vector<T, 3> const& axis, T const& angle) {
  T const ca = std::cos(angle);
  T const sa = std::sin(angle);
  T const omca = T(1) - ca;

  Matrix<T, 3, 3> rot;
  rot[0] = ca + axis[0]*axis[0] * omca;
  rot[1] = axis[0]*axis[1] * omca - axis[2] * sa;
  rot[2] = axis[0]*axis[2] * omca + axis[1] * sa;

  rot[3] = axis[1]*axis[0] * omca + axis[2] * sa;
  rot[4] = ca + axis[1]*axis[1] * omca;
  rot[5] = axis[1]*axis[2] * omca - axis[0] * sa;

  rot[6] = axis[2]*axis[0] * omca - axis[1] * sa;
  rot[7] = axis[2]*axis[1] * omca + axis[0] * sa;
  rot[8] = ca + axis[2]*axis[2] * omca;
  return rot;
}

// TriangleMesh

class TriangleMesh {
public:
  typedef std::shared_ptr<TriangleMesh> Ptr;
  typedef std::shared_ptr<TriangleMesh const> ConstPtr;

  typedef unsigned int VertexID;
  typedef std::vector<sfm::Vec3f> VertexList;
  typedef std::vector<sfm::Vec4f> ColorList;
  typedef std::vector<sfm::Vec3f> NormalList;
  typedef std::vector<sfm::Vec2f> TexCoordList;
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
  sfm::Vec3f pos;
  sfm::Vec3f viewing_dir;
  sfm::Vec3f up_vec;

  float z_near;
  float z_far;
  float top;
  float right;

  int width;
  int height;

  sfm::Matrix4f view;
  sfm::Matrix4f proj;

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
  width(0), height(0),
  view(0.0f), proj(0.0f) {
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
