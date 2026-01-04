/* Copyright (c) 2021, United States Government, as represented by the
 * Administrator of the National Aeronautics and Space Administration.
 *
 * All rights reserved.
 *
 * The "ISAAC - Integrated System for Autonomous and Adaptive Caretaking
 * platform" software is licensed under the Apache License, Version 2.0
 * (the "License"); you may not use this file except in compliance with the
 * License. You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the
 * License for the specific language governing permissions and limitations
 * under the License.
 */

#ifndef TRANSFORM_UTILS_H_
#define TRANSFORM_UTILS_H_

#include <asp/Rig/RigTypeDefs.h>
#include <vector>
#include <string>
#include <map>

#include <Eigen/Core>
#include <Eigen/Geometry>

namespace rig {
  class CameraParameters;
}

namespace rig {

const int NUM_SCALAR_PARAMS  = 1;  // Used to float single-value params // NOLINT
const int NUM_OPT_CTR_PARAMS = 2;  // optical center in x and y         // NOLINT
const int NUM_PIX_PARAMS     = 2;                                       // NOLINT
const int NUM_XYZ_PARAMS     = 3;                                       // NOLINT
const int NUM_RIGID_PARAMS   = 7;  // quaternion (4) + translation (3)  // NOLINT
const int NUM_AFFINE_PARAMS  = 12; // 3x3 matrix (9) + translation (3)  // NOLINT
  
class cameraImage;
class RigSet;
  
// Save an affine transform represented as a matrix to a string.
std::string affineToStr(Eigen::Affine3d const& M);

// Form an affine transform from 12 values
Eigen::Affine3d vecToAffine(Eigen::VectorXd const& vals);

// Calculate interpolated world to reference sensor transform. Take into account
// that the timestamp is for a sensor which may not be the reference one, so
// an offset needs to be applied. If beg_ref_stamp equals end_ref_stamp,
// end_world_to_ref_t is ignored.
Eigen::Affine3d calc_interp_world_to_ref(const double* beg_world_to_ref_t,
                                         const double* end_world_to_ref_t,
                                         double beg_ref_stamp,
                                         double end_ref_stamp,
                                         double ref_to_cam_offset,
                                         double cam_stamp);

// Calculate interpolated world to camera transform. Use the
// convention that if beg_ref_stamp == end_ref_stamp only
// beg_world_to_ref_t is used, while end_world_to_ref_t is
// ignored. For the reference camera it is also expected that
// ref_to_cam_aff is the identity. This saves some code duplication
// later as the ref cam need not be treated separately.
Eigen::Affine3d calcWorldToCamBase(const double* beg_world_to_ref_t,
                                   const double* end_world_to_ref_t,
                                   const double* ref_to_cam_trans,
                                   double beg_ref_stamp,
                                   double end_ref_stamp,
                                   double ref_to_cam_offset,
                                   double cam_stamp);

// Find the median of some matrices, by finding the median for each entry
Eigen::MatrixXd median_matrix(std::vector<Eigen::MatrixXd> const& transforms);
  
// Given the transforms from each camera to the world and their timestamps,
// find an initial guess for the relationship among the sensors on the rig.
// Note that strictly speaking the transforms in world_to_ref_vec are among
// those in world_to_cam, but we don't have a way of looking them up in that
// vector.
void calc_rig_trans(std::vector<rig::cameraImage> const& cams,
                    std::vector<Eigen::Affine3d>  const& world_to_ref,
                    std::vector<Eigen::Affine3d>  const& world_to_cam,
                    std::vector<double>           const& ref_timestamps,
                    rig::RigSet                        & R); // update this

// Compute the transforms from the world to every camera, based on the rig transforms.
void calcWorldToCamWithRig(// Inputs
                           bool have_rig,
                           std::vector<rig::cameraImage> const& cams,
                           std::vector<double> const& world_to_ref_vec,
                           std::vector<double> const& ref_timestamps,
                           std::vector<double> const& ref_to_cam_vec,
                           std::vector<double> const& ref_to_cam_timestamp_offsets,
                           // Output
                           std::vector<Eigen::Affine3d>& world_to_cam);
  
// A version of the above with the data stored differently
void calcWorldToCamWithRig(// Inputs
                           bool have_rig,
                           std::vector<rig::cameraImage> const& cams,
                           std::vector<Eigen::Affine3d> const& world_to_ref,
                           std::vector<double> const& ref_timestamps,
                           std::vector<Eigen::Affine3d> const& ref_to_cam,
                           std::vector<double> const& ref_to_cam_timestamp_offsets,
                           // Output
                           std::vector<Eigen::Affine3d>& world_to_cam);

void calcWorldToCamNoRig(// Inputs
                          std::vector<rig::cameraImage> const& cams,
                          std::vector<double> const& world_to_cam_vec,
                          // Output
                          std::vector<Eigen::Affine3d>& world_to_cam);
  
void calcWorldToCam(// Inputs
 bool no_rig, std::vector<rig::cameraImage> const& cams,
 std::vector<double> const& world_to_ref_vec, std::vector<double> const& ref_timestamps,
 std::vector<double> const& ref_to_cam_vec, std::vector<double> const& world_to_cam_vec,
 std::vector<double> const& ref_to_cam_timestamp_offsets,
 // Output
 std::vector<Eigen::Affine3d>& world_to_cam);
  
void affine_transform_to_array(Eigen::Affine3d const& aff, double* arr);
void array_to_affine_transform(Eigen::Affine3d& aff, const double* arr);

// Extract a rigid transform to an array of length NUM_RIGID_PARAMS
void rigid_transform_to_array(Eigen::Affine3d const& aff, double* arr);

// Convert an array of length NUM_RIGID_PARAMS to a rigid
// transform. Normalize the quaternion to make it into a rotation.
void array_to_rigid_transform(Eigen::Affine3d& aff, const double* arr);

// A function to compute the camera position in world coordinates given
// the world_to_cam array
Eigen::Vector3d calc_cam_position(double const* world_to_cam);

// Compute the n-weight slerp, analogous to the linear combination
// W[0]*Q[0] + ... + W[n-1]*Q[n-1]. This is experimental.
// We assume the sum of weights is 1.
// TODO(oalexan1): Move this to transform_utils.cc.
Eigen::Quaternion<double> slerp_n(std::vector<double> const& W,
                                  std::vector<Eigen::Quaternion<double>> const& Q);
  
// Given two sets of 3D points, find the rotation + translation + scale
// which best maps the first set to the second.
// Source: http://en.wikipedia.org/wiki/Kabsch_algorithm
// TODO(oalexan1): Use the version robust to outliers!  
void Find3DAffineTransform(Eigen::Matrix3Xd const & in,
                           Eigen::Matrix3Xd const & out,
                           Eigen::Affine3d* result);

// Extract control points and the images they correspond 2 from
// a hugin project file
void ParseHuginControlPoints(std::string const& hugin_file,
                             std::vector<std::string> * images,
                             Eigen::MatrixXd * points);

// Parse a file having on each line xyz coordinates
void ParseXYZ(std::string const& xyz_file,
              Eigen::MatrixXd * xyz);

// Apply a given transform to the given set of cameras.
// We assume that the transform is of the form
// T(x) = scale * rotation * x + translation
void TransformCameras(Eigen::Affine3d const& T, std::vector<Eigen::Affine3d> &world_to_cam);
  
// Apply same transform as above to points
void TransformPoints(Eigen::Affine3d const& T, std::vector<Eigen::Vector3d> *xyz);

// Apply a given transform to the specified xyz points, and adjust accordingly the cameras
// for consistency. We assume that the transform is of the form
// A(x) = scale * rotation * x + translation
void TransformCamerasAndPoints(Eigen::Affine3d const& A,
                               std::vector<Eigen::Affine3d> *cid_to_cam_t,
                               std::vector<Eigen::Vector3d> *xyz);
  
// Apply a registration transform to a rig. The only thing that
// changes is scale, as the rig transforms are between coordinate
// systems of various cameras.
void TransformRig(Eigen::Affine3d const& T, std::vector<Eigen::Affine3d> & ref_to_cam_trans);

// Find the name of the camera of the images used in registration.
// The registration images must all be acquired with the same sensor.  
std::string registrationCamName(std::string const& hugin_file,
                                std::vector<std::string> const& cam_names,
                                std::vector<rig::cameraImage> const & cams);

// Find the 3D transform from an abstract coordinate system to the world, given
// control points (pixel matches) and corresponding 3D measurements. It is
// assumed all images are acquired with the same camera.
Eigen::Affine3d registrationTransform(std::string                  const& hugin_file,
                                      std::string                  const& xyz_file,
                                      rig::CameraParameters     const& cam_params,
                                      std::vector<std::string>     const& cid_to_filename,
                                      std::vector<Eigen::Affine3d> const& world_to_cam_trans); 

// Apply a transform to inlier triangulated points  
void transformInlierTriPoints(// Inputs
                              Eigen::Affine3d const& trans,
                              rig::PidToCidFidVec const& pid_to_cid_fid,
                              PidCidFid const& pid_cid_fid_inlier,
                              std::vector<Eigen::Vector3d> & xyz_vec); // output

}  // end namespace rig

#endif  // TRANSFORM_UTILS_H_
