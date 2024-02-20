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

// Logic for propagation of errors (covariance) through stereo triangulation 

#include <asp/Camera/Covariance.h>
#include <asp/Camera/LinescanDGModel.h>
#include <asp/Core/StereoSettings.h>

#include <vw/Stereo/StereoModel.h>
#include <vw/Math/LinearAlgebra.h>

#include <iostream>

using namespace vw::camera;

namespace asp {

// Change in satellite position (measured in meters) and satellite orientation
// measured in normalized quaternions, to be used for numerical differencing.
// One has to be mindful of the fact that the positions are on the order of
// 7.0e6 meters given the distance from satellite to Earth center in ECEF,
// so the position delta should not be too tiny.
const double deltaPosition = 0.01; // measured in meters
const double deltaQuat     = 1.0e-6; // given that quaternions are normalized

// Given 0 <= num < 15, return a perturbation in satellite position. The
// starting one is the zero perturbation, then perturb first
// coordinate in the positive and then negative direction, then same
// for second and third coordinate. The rest of the perturbations are
// 0 as those indices are used to perturb the quaternions.
// So, return (0, 0, 0), (deltaPosition, 0, 0), (-deltaPosition, 0, 0)
// (0, deltaPosition, 0), (0, -deltaPosition, 0), and so on.
vw::Vector<double, 3> positionDelta(int num) {

  vw::Vector<double, 3> ans; // this is 0
  if (num == 0)
    return ans; // nominal position value, no perturbation
  if (num > 6) 
    return ans; // nominal position value, will perturb the quaternion then

  double sign = 1.0; // sign of the perturbation: 1, -1, 1, -1, etc.
  if (num % 2 == 0) 
    sign = -1.0;

  int index = (num - 1)/2; // get 0, 1, or 2
  ans[index] = sign * deltaPosition;
  
  return ans;
}

// Similar logic as above for the quaternion, for indices 7, ..., 14 (8 of them)
vw::Vector<double, 4> quatDelta(int num) {

  vw::Vector<double, 4> ans; // this is 0
  if (num <= 6) 
    return ans; // nominal quat value, will perturb the positions then

  num = num - 6; // now num = 1, 2, ... 8
  if (num > 8) 
    vw::vw_throw(vw::ArgumentErr() << "Out of bounds in quatDelta().\n");
  
  double sign = 1.0; // sign of the perturbation: 1, -1, 1, -1, etc.
  if (num % 2 == 0) 
    sign = -1.0;

  int index = (num - 1)/2; // get 0, 1, 2, or 3
  ans[index] = sign * deltaQuat;
  
  return ans;
}

// Number of nominal and perturbed cameras when the covariance is computed with DG cameras.
int numCamsForCovariance() {
  // One nominal camera. Then one positive and negative perturbation
  // for each position (3) and quaternion (4).
  return 15; 
}

// Given two DG cameras and a pixel in each camera image, consider the
// following transform. Go from the perturbed joint vector of
// satellite positions and quaternions for this pixel pair to the
// perturbed triangulated point. Then, the vector from nominal to
// perturbed triangulation point is converted to North-East-Down
// relative to the nominal point. Use numerical differentiation to
// find the Jacobian of this transform with centered
// differences. This will be used to find the covariances of the
// triangulated point in NED coordinates given the input satellite
// covariances. This works only for Maxar (DigitalGlobe)
// cameras. This function may throw exceptions. Do not divide the
// numerical derivatives by deltaPosition and deltaQuat, but only by
// 2.0 (since these are centered differences). That because the
// division makes the partial derivatives in quaternions huge and is
// not good for numerical stability. We will compensate for this
// when we multiply by the actual covariances, which are huge, so
// those will be pre-multiplied by the squares of deltaPosition and
// deltaQuat, with the same final result.
void scaledDGTriangulationJacobian(vw::cartography::Datum const& datum,
                                   vw::camera::CameraModel const* cam1,
                                   vw::camera::CameraModel const* cam2,
                                   vw::Vector2 const& pix1,
                                   vw::Vector2 const& pix2,
                                   vw::Matrix<double> & J) {
  
  // Handle adjusted cameras
  bool adjusted_cameras = false;
  const AdjustedCameraModel *adj_cam1 = dynamic_cast<const AdjustedCameraModel*>(cam1);
  const AdjustedCameraModel *adj_cam2 = dynamic_cast<const AdjustedCameraModel*>(cam2);
  if ((adj_cam1 == NULL) != (adj_cam2 == NULL))
    vw::vw_throw(vw::ArgumentErr() << "The cameras must be either both "
                 << "adjusted or both unadjusted.\n");

  vw::Matrix3x3 cam1_rot, cam2_rot;
  vw::Vector3 cam1_shift, cam2_shift;
  if (adj_cam1 != NULL && adj_cam2 != NULL) {
    adjusted_cameras = true;
    // transforms from unadjusted to adjusted coordinates
    vw::Matrix4x4 cam1_adj = adj_cam1->ecef_transform();
    vw::Matrix4x4 cam2_adj = adj_cam2->ecef_transform();
    cam1_rot = submatrix(cam1_adj, 0, 0, 3, 3);
    cam2_rot = submatrix(cam2_adj, 0, 0, 3, 3);
    cam1_shift = vw::Vector3(cam1_adj(0, 3), cam1_adj(1, 3), cam1_adj(2, 3));
    cam2_shift = vw::Vector3(cam2_adj(0, 3), cam2_adj(1, 3), cam2_adj(2, 3));
  }
  
  DGCameraModel const* dg_cam1 = dynamic_cast<DGCameraModel const*>(unadjusted_model(cam1));
  DGCameraModel const* dg_cam2 = dynamic_cast<DGCameraModel const*>(unadjusted_model(cam2));
  if (dg_cam1 == NULL || dg_cam2 == NULL) 
    vw::vw_throw(vw::ArgumentErr() << "Expecting DG cameras.\n");

  // Numerical differences will be used. Camera models with deltaPosition and deltaQuat
  // perturbations have already been created in LinescanDGModel.cc using the positionDelta()
  // and quatDelta() functions from above.
  if (dg_cam1->m_perturbed_cams.empty() || dg_cam2->m_perturbed_cams.empty()) 
    vw::vw_throw(vw::ArgumentErr() << "The perturbed cameras were not set up.\n");
  
  if (dg_cam1->m_perturbed_cams.size() != dg_cam2->m_perturbed_cams.size())
    vw::vw_throw(vw::ArgumentErr()
                 << "The number of perturbations in the two cameras do not agree.\n");

  // Find the camera center and direction for first unperturbed
  // camera, and for the perturbed versions. Same for the second
  // camera.
  std::vector<vw::Vector3> cam1_dirs, cam1_ctrs, cam2_dirs, cam2_ctrs;
  cam1_dirs.push_back(dg_cam1->pixel_to_vector(pix1));
  cam1_ctrs.push_back(dg_cam1->camera_center(pix1));
  cam2_dirs.push_back(dg_cam2->pixel_to_vector(pix2));
  cam2_ctrs.push_back(dg_cam2->camera_center(pix2));
  for (size_t it = 0; it < dg_cam1->m_perturbed_cams.size(); it++) {
    cam1_dirs.push_back(dg_cam1->m_perturbed_cams[it]->pixel_to_vector(pix1));
    cam1_ctrs.push_back(dg_cam1->m_perturbed_cams[it]->camera_center(pix1));
    cam2_dirs.push_back(dg_cam2->m_perturbed_cams[it]->pixel_to_vector(pix2));
    cam2_ctrs.push_back(dg_cam2->m_perturbed_cams[it]->camera_center(pix2));
  }

  // Apply adjustments
  if (adjusted_cameras) {
    for (size_t it = 0; it < cam1_dirs.size(); it++) {
      cam1_dirs[it] = cam1_rot * cam1_dirs[it];
      cam2_dirs[it] = cam2_rot * cam2_dirs[it];
      cam1_ctrs[it] = cam1_rot * cam1_ctrs[it] + cam1_shift;
      cam2_ctrs[it] = cam2_rot * cam2_ctrs[it] + cam2_shift;
    }
  }
  
  // Nominal triangulation point
  vw::Vector3 tri_nominal, err_nominal;
  // If triangulation fails, it can return NaN
  tri_nominal
    = vw::stereo::triangulate_pair(cam1_dirs[0], cam1_ctrs[0], cam2_dirs[0], cam2_ctrs[0],
                                   err_nominal);
  if (tri_nominal != tri_nominal) // NaN
    vw::vw_throw(vw::ArgumentErr() << "Could not triangulate.\n");

  // The matrix to go from the NED coordinate system to ECEF
  vw::Vector3 llh = datum.cartesian_to_geodetic(tri_nominal);
  vw::Matrix3x3 NedToEcef = datum.lonlat_to_ned_matrix(llh);
  vw::Matrix3x3 EcefToNed = inverse(NedToEcef);

  // There are 14 input variables: 3 positions and 4 quaternions for
  // cam1, and same for cam2. For each of them must compute a centered
  // difference. The output has 3 variables. As documented above,
  // the vector from the nominal to perturbed
  // triangulated point will be converted to North-East-Down
  // coordinates at the nominal triangulated point.
  J.set_size(3, 14);
  J.set_zero();
  for (int coord = 0; coord < 14; coord++) {

    vw::Vector3 cam1_dir_plus, cam1_ctr_plus, cam2_dir_plus, cam2_ctr_plus;
    vw::Vector3 cam1_dir_minus, cam1_ctr_minus, cam2_dir_minus, cam2_ctr_minus;
    if (coord < 7) {
      // The perturbed cameras store positive and negative
      // perturbations, in alternating order. See positionDelta() and
      // quatDelta() for the book-keeping. Note that a perturbation in
      // the satellite quaternion also affects the camera center,
      // given how one converts from satellite to camera coordinates
      // when the DG model is created.

      // Since at position 0 in cam_dirs we store the nominal (unperturbed)
      // values, add 1 below.
      cam1_dir_plus  = cam1_dirs[2*coord + 1]; cam1_ctr_plus  = cam1_ctrs[2*coord + 1];
      cam1_dir_minus = cam1_dirs[2*coord + 2]; cam1_ctr_minus = cam1_ctrs[2*coord + 2];
    } else {
      // When variables affecting second camera change, the first one stays at nominal value.
      cam1_dir_plus  = cam1_dirs[0]; cam1_ctr_plus  = cam1_ctrs[0];
      cam1_dir_minus = cam1_dirs[0]; cam1_ctr_minus = cam1_ctrs[0];
    }

    // For the second camera, the book-keeping is reversed
    if (coord < 7) {
      // Second camera values do not change when first camera inputs change
      cam2_dir_plus  = cam2_dirs[0]; cam2_ctr_plus  = cam2_ctrs[0];
      cam2_dir_minus = cam2_dirs[0]; cam2_ctr_minus = cam2_ctrs[0];
    } else {
      int coord2 = coord - 7; // has values 0, 1, ..., 6
      cam2_dir_plus  = cam2_dirs[2*coord2 + 1]; cam2_ctr_plus  = cam2_ctrs[2*coord2 + 1];
      cam2_dir_minus = cam2_dirs[2*coord2 + 2]; cam2_ctr_minus = cam2_ctrs[2*coord2 + 2];
    }

    vw::Vector3 tri_plus, err_plus, tri_minus, err_minus;
    tri_plus = vw::stereo::triangulate_pair(cam1_dir_plus, cam1_ctr_plus,
                                            cam2_dir_plus, cam2_ctr_plus, err_plus);
    tri_minus = vw::stereo::triangulate_pair(cam1_dir_minus, cam1_ctr_minus,
                                             cam2_dir_minus, cam2_ctr_minus, err_minus);

    // Find the triangulated points in the local NED (horizontal-vertical)
    // coordinate system.
    vw::Vector3 ned_plus = EcefToNed * (tri_plus - tri_nominal);
    vw::Vector3 ned_minus = EcefToNed * (tri_minus - tri_nominal);

    // Find the numerical partial derivative, but do not divide by the
    // spacing (deltaPosition or deltaQuat) as that makes the numbers
    // huge. We will compensate for when use use this Jacobian to
    // propagate the satellite position and quaternion covariances
    // (matrix SC), by the formula J * SC * J^T. Then, we will divide SC
    // by these squared delta quantities, which is the right thing to
    // do, because the values in SC are tiny, and, in fact, on the
    // order of the squares of the delta values.
    vw::Vector3 ned_diff = (ned_plus - ned_minus)/2.0;
    
    for (int row = 0; row < 3; row++) 
      J(row, coord) = ned_diff[row];
  }

  return;
}

// Given upper-right values in a symmetric matrix of given size, find
// the lower-left values by reflection, and insert them as a block
// starting at the desired row and column. Used to populate the joint
// covariance matrix. Per DigitalGlobe's doc, the covariances are
// stored as c11, c12, c13, ..., c22, c23, ...
void insertBlock(int start, int size, double* inputVals, vw::Matrix<double> & C) {
  int count = 0;
  for (int row = 0; row < size; row++) {
    for (int col = row; col < size; col++) {
      C(start + row, start + col) = inputVals[count];
      C(start + col, start + row) = inputVals[count];
      count++;
    }
  }
}
  
// Based on tabulated satellite position and quaternion covariance
// for each DG camera, find the interpolated covariances for cam1 at
// pix1 (6 for position, 10 for orientation, as just the upper-right
// corner is used), same for cam2 at pix2, autocomplete these to the
// full matrices (3x3 and 4x4 for each), create a combined matrix of
// covariances (14 x 14), and divide the entries by squares of
// deltaPosition and deltaQuat which normalizes them, and which are
// compensated by not dividing by these numbers (without the square)
// what is found in scaledDGTriangulationJacobian(). Later we will do
// J * C * J^T. The same order of variables as in
// scaledDGTriangulationJacobian must be used.
void scaledDGSatelliteCovariance(vw::camera::CameraModel const* cam1,
                                 vw::camera::CameraModel const* cam2,
                                 vw::Vector2 const& pix1,
                                 vw::Vector2 const& pix2,
                                 vw::Matrix<double> & C) {
  
  // Initialize the output
  // 3 positions for cam 1, 4 orientations for cam1, 3 positions for cam2, 4 orientations
  // for cam2. So, four blocks in total. The resulting matrix must be symmetric.
  C.set_size(14, 14);
  C.set_zero();

  // Here it is not important that the camera are adjusted or not, as all that is needed
  // are the input covariances.
  DGCameraModel const* dg_cam1 = dynamic_cast<DGCameraModel const*>(unadjusted_model(cam1));
  DGCameraModel const* dg_cam2 = dynamic_cast<DGCameraModel const*>(unadjusted_model(cam2));
  if (dg_cam1 == NULL || dg_cam2 == NULL) 
    vw::vw_throw(vw::ArgumentErr() << "Expecting DG cameras.\n");

  // Find the covariances at given pixel by interpolation in the input table.
  // Use nearest neighbor interpolation as covariances are known with
  // just a few digits of precision and are not meant to be smooth.
  double p_cov1[SAT_POS_COV_SIZE], p_cov2[SAT_POS_COV_SIZE];
  double q_cov1[SAT_QUAT_COV_SIZE], q_cov2[SAT_QUAT_COV_SIZE];  
  dg_cam1->interpSatellitePosCov(pix1, p_cov1);
  dg_cam1->interpSatelliteQuatCov(pix1, q_cov1);
  dg_cam2->interpSatellitePosCov(pix2, p_cov2);
  dg_cam2->interpSatelliteQuatCov(pix2, q_cov2);

  // This is useful for seeing which input covariance has a bigger effect.
  // The default value of these factors is 1.
  double pf = asp::stereo_settings().position_covariance_factor;
  double qf = asp::stereo_settings().orientation_covariance_factor;
  
  // Scale these per scaledDGTriangulationJacobian().
  for (int ip = 0; ip < SAT_POS_COV_SIZE; ip++) {
    p_cov1[ip] = pf * p_cov1[ip] / (deltaPosition * deltaPosition); 
    p_cov2[ip] = pf * p_cov2[ip] / (deltaPosition * deltaPosition); 
  }
  for (int iq = 0; iq < SAT_QUAT_COV_SIZE; iq++) {
    q_cov1[iq] = qf * q_cov1[iq] / (deltaQuat * deltaQuat); 
    q_cov2[iq] = qf * q_cov2[iq] / (deltaQuat * deltaQuat); 
  }

  // Put these in the covariance matrix
  insertBlock(0,  3, p_cov1, C);
  insertBlock(3,  4, q_cov1, C);
  insertBlock(7,  3, p_cov2, C);
  insertBlock(10, 4, q_cov2, C);

#if 0
  std::cout << "Produced matrix " << std::endl;
  for (int row = 0; row < 14; row++) {
    for (int col = 0; col < 14; col++) {
      std::cout << C(row, col) << " ";
    }
    std::cout << std::endl;
  }

  // Debug code. This shows that some quaternion covariances have a
  // negative determinant. That is because an eigenvalue is very close
  // to 0 or even negative (but small). This singularity goes away
  // after the covariances are propagated.
  std::cout << "determinant1 " << det(submatrix(C, 0, 0, 3, 3)) << std::endl;
  std::cout << "determinant2 " << det(submatrix(C, 3, 3, 4, 4)) << std::endl;
  std::cout << "determinant3 " << det(submatrix(C, 7, 7, 3, 3)) << std::endl;
  std::cout << "determinant4 " << det(submatrix(C, 10, 10, 4, 4)) << std::endl;
#endif
  
  return;
}

// Given a North-East-Down coordinate system at a point on a planet surface,
// left camera center, the x and y coordinates of where the ray from that
// center intersects the plane z = 0, and the same for the right camera,
// all in NED coordinates, find where the rays intersect, also in NED.
vw::Vector3 nedTri(vw::Vector3 const& cam1_ctr, vw::Vector3 const& cam2_ctr,
                   double x1, double y1, double x2, double y2) {

  // Find the normalized direction from camera to ground
  vw::Vector3 ground_pt1(x1, y1, 0.0);
  vw::Vector3 cam1_dir = ground_pt1 - cam1_ctr; cam1_dir /= norm_2(cam1_dir);
  vw::Vector3 ground_pt2(x2, y2, 0.0);
  vw::Vector3 cam2_dir = ground_pt2 - cam2_ctr; cam2_dir /= norm_2(cam2_dir);

  vw::Vector3 tri, err;
  tri = vw::stereo::triangulate_pair(cam1_dir, cam1_ctr, cam2_dir, cam2_ctr, err);
  
  return tri;
}

// Given a triangulated point in ECEF, create the local
// North-East-Down (NED) coordinate system centered at that
// point. Find the Jacobian of the nedTri() function, which will
// propagate uncertainties from the North-East horizontal plane
// through triangulation, with the result also being in NED.
// Bundle-adjusted cameras need no special treatment.
void triangulationJacobian(vw::cartography::Datum const& datum,
                           vw::Vector3 const& tri_nominal,
                           vw::camera::CameraModel const* cam1,
                           vw::camera::CameraModel const* cam2,
                           vw::Vector2 const& pix1,
                           vw::Vector2 const& pix2,
                           vw::Matrix<double> & J) {
  
  // The matrix to go from the NED coordinate system to ECEF at the
  // nominal triangulation point
  vw::Vector3 llh = datum.cartesian_to_geodetic(tri_nominal);
  vw::Matrix3x3 NedToEcef = datum.lonlat_to_ned_matrix(llh);
  vw::Matrix3x3 EcefToNed = inverse(NedToEcef);

  // Camera centers and directions in ECEF
  vw::Vector3 cam1_ctr = cam1->camera_center(pix1), cam1_dir = cam1->pixel_to_vector(pix1);
  vw::Vector3 cam2_ctr = cam2->camera_center(pix2), cam2_dir = cam2->pixel_to_vector(pix2);

  // Convert to NED
  vw::Vector3 cam1_ctr_ned = EcefToNed * (cam1_ctr - tri_nominal);
  vw::Vector3 cam1_dir_ned = EcefToNed * cam1_dir;
  vw::Vector3 cam2_ctr_ned = EcefToNed * (cam2_ctr - tri_nominal);
  vw::Vector3 cam2_dir_ned = EcefToNed * cam2_dir;

  // See where the rays intersect the local horizontal plane Find
  // alpha1 so that cam1_ctr_ned + alpha1 * cam1_dir_ned has 3rd
  // coordinate equal to zero
  double alpha1 = -cam1_ctr_ned.z() / cam1_dir_ned.z();
  double x1 = cam1_ctr_ned.x() + alpha1 * cam1_dir_ned.x();
  double y1 = cam1_ctr_ned.y() + alpha1 * cam1_dir_ned.y();
  double alpha2 = -cam2_ctr_ned.z() / cam2_dir_ned.z();
  double x2 = cam2_ctr_ned.x() + alpha2 * cam2_dir_ned.x();
  double y2 = cam2_ctr_ned.y() + alpha2 * cam2_dir_ned.y();
  
  // There are 4 input variables: x and y position in the horizontal
  // plane for the first camera, then for the second one. For each of
  // them must compute a centered difference. The output has 3
  // variables, the NED triangulation point.
  J.set_size(3, 4);
  J.set_zero();

  for (int coord = 0; coord < 4; coord++) {

    // Perturb one variable at a time
    double x1_plus = x1, x1_minus = x1, x2_plus = x2, x2_minus = x2;
    double y1_plus = y1, y1_minus = y1, y2_plus = y2, y2_minus = y2;
    if (coord == 0) {
      x1_minus += -deltaPosition;
      x1_plus  +=  deltaPosition;
    } else if (coord == 1) {
      y1_minus += -deltaPosition;
      y1_plus  +=  deltaPosition;
    } else if (coord == 2) {
      x2_minus += -deltaPosition;
      x2_plus  +=  deltaPosition;
    } else if (coord == 3) {
      y2_minus += -deltaPosition;
      y2_plus  +=  deltaPosition;
    }

    vw::Vector3 xyz_plus = nedTri(cam1_ctr_ned, cam2_ctr_ned,
                                  x1_plus, y1_plus, x2_plus, y2_plus);
    vw::Vector3 xyz_minus = nedTri(cam1_ctr_ned, cam2_ctr_ned,
                                   x1_minus, y1_minus, x2_minus, y2_minus);

    // Centered difference
    vw::Vector3 partial_deriv = (xyz_plus - xyz_minus) / (2.0 * deltaPosition);
    
    for (int row = 0; row < 3; row++) 
      J(row, coord) = partial_deriv[row];
  }

  return;
}

// Propagate the covariances. Return propagated stddev. See the .h file for more info.
vw::Vector2 propagateCovariance(vw::Vector3 const& tri_nominal,
                                vw::cartography::Datum const& datum,
                                double stddev1, double stddev2,
                                vw::camera::CameraModel const* cam1,
                                vw::camera::CameraModel const* cam2,
                                vw::Vector2 const& pix1,
                                vw::Vector2 const& pix2) {

  // Return right away if triangulation was not successful. The caller will set the result
  // to (0, 0, 0).
  if (tri_nominal == vw::Vector3(0, 0, 0) || tri_nominal != tri_nominal) 
    vw::vw_throw(vw::ArgumentErr() << "Could not compute the covariance.\n");

  vw::Matrix<double> J, C;

  // variance is square of stddev
  vw::Vector2 variance;
  variance[0] = stddev1 * stddev1;
  variance[1] = stddev2 * stddev2;
  
  if (variance[0] > 0 && variance[1] > 0) {
    // The user set horizontal stddev
    triangulationJacobian(datum, tri_nominal, cam1, cam2, pix1, pix2, J);
    C = vw::math::identity_matrix(4);
    // The first two covariances are the left camera horizontal square stddev,
    // and last two are for the right camera.
    C(0, 0) = variance[0]; C(1, 1) = variance[0];
    C(2, 2) = variance[1]; C(3, 3) = variance[1];
  } else {
    // Will arrive here only for DG cameras and if the user did not
    // set --horizontal-stddev.  The Jacobian of the transform from
    // ephemeris and attitude to the triangulated point in NED
    // coordinates, multiplied by a scale factor.
    asp::scaledDGTriangulationJacobian(datum, cam1, cam2, pix1, pix2, J);
    
    // The input covariance, divided by the square of the above scale factor.
    asp::scaledDGSatelliteCovariance(cam1, cam2, pix1, pix2, C);
  }
  
  // Propagate the covariance
  // Per: https://en.wikipedia.org/wiki/Propagation_of_uncertainty#Non-linear_combinations
  vw::Matrix<double> JT = transpose(J);
  vw::Matrix<double> P = J * C * JT;

#if 0
  // Useful debug code
  std::cout << "NED covariance " << P << std::endl;
  vw::Vector<std::complex<double>> e;
  vw::math::eigen(P, e);
  std::cout << "Eigenvalues: " << e << std::endl;
#endif
  
  // Horizontal component is the square root of the determinant of the
  // upper-left 2x2 block (horizontal plane component), which is the
  // same as the square root of the product of eigenvalues of this
  // matrix.  Intuitively, the area of an ellipse is the product of
  // semi-axes, which is the product of eigenvalues. Then, a circle
  // with radius which is the square root of the product of semi-axes
  // has the same area.
  vw::Matrix2x2 H = submatrix(P, 0, 0, 2, 2);
  vw::Vector2 ans;
  ans[0] = sqrt(det(H));

  // Vertical component is the z variance
  ans[1] = P(2, 2);

  // Check for NaN. Then the caller will return the zero vector, which
  // signifies that the there is no valid data
  if (ans != ans) 
    vw::vw_throw(vw::ArgumentErr() << "Could not compute the covariance.\n");

  // Take the square root, so return the standard deviation
  return vw::Vector2(sqrt(ans[0]), sqrt(ans[1]));
}
  
} // end namespace asp
