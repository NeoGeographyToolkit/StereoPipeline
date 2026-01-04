// __BEGIN_LICENSE__
//  Copyright (c) 2009-2025, United States Government as represented by the
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

#include <asp/Rig/transform_utils.h>
#include <asp/Rig/interpolation_utils.h>
#include <asp/Rig/rig_config.h>
#include <asp/Rig/camera_image.h>
#include <asp/Rig/basic_algs.h>
#include <asp/Rig/RigCameraParams.h>
#include <asp/Rig/triangulation.h>

#include <glog/logging.h>

#include <string>
#include <vector>
#include <iostream>
#include <fstream>
#include <set>
#include <string>

namespace rig {

// Save an affine transform represented as a matrix to a string.
std::string affineToStr(Eigen::Affine3d const& M) {
  Eigen::MatrixXd T = M.matrix();
  std::ostringstream os;
  os.precision(17);
  os << T(0, 0) << " " << T(0, 1) << " " << T(0, 2) << " "
     << T(1, 0) << " " << T(1, 1) << " " << T(1, 2) << " "
     << T(2, 0) << " " << T(2, 1) << " " << T(2, 2) << " "
     << T(0, 3) << " " << T(1, 3) << " " << T(2, 3);

  return os.str();
}

// Form an affine transform from 12 values
Eigen::Affine3d vecToAffine(Eigen::VectorXd const& vals) {
  if (vals.size() != 12)
    LOG(FATAL) << "An affine transform must have 12 parameters.\n";

  Eigen::Affine3d M = Eigen::Affine3d::Identity();
  Eigen::MatrixXd T = M.matrix();

  int count = 0;

  // linear part
  T(0, 0) = vals[count++];
  T(0, 1) = vals[count++];
  T(0, 2) = vals[count++];
  T(1, 0) = vals[count++];
  T(1, 1) = vals[count++];
  T(1, 2) = vals[count++];
  T(2, 0) = vals[count++];
  T(2, 1) = vals[count++];
  T(2, 2) = vals[count++];

  // translation part
  T(0, 3) = vals[count++];
  T(1, 3) = vals[count++];
  T(2, 3) = vals[count++];

  M.matrix() = T;

  return M;
}

// Calculate interpolated world to reference sensor transform. Take into account
// that the timestamp is for a sensor which may not be the reference one, so
// an offset needs to be applied. If beg_ref_stamp equals end_ref_stamp,
// end_world_to_ref_t is ignored.
Eigen::Affine3d calc_interp_world_to_ref(const double* beg_world_to_ref_t,
                                         const double* end_world_to_ref_t,
                                         double beg_ref_stamp,
                                         double end_ref_stamp,
                                         double ref_to_cam_offset,
                                         double cam_stamp) {
    Eigen::Affine3d beg_world_to_ref_aff;
    array_to_rigid_transform(beg_world_to_ref_aff, beg_world_to_ref_t);

    Eigen::Affine3d end_world_to_ref_aff;
    array_to_rigid_transform(end_world_to_ref_aff, end_world_to_ref_t);

    // Handle the degenerate case
    if (end_ref_stamp == beg_ref_stamp) 
      return beg_world_to_ref_aff;
    
    // Covert from cam time to ref time and normalize. It is very
    // important that below we subtract the big numbers from each
    // other first, which are the timestamps, then subtract whatever
    // else is necessary. Otherwise we get problems with numerical
    // precision with CERES.
    double alpha = ((cam_stamp - beg_ref_stamp) - ref_to_cam_offset)
        / (end_ref_stamp - beg_ref_stamp);
    
    if (alpha < 0.0 || alpha > 1.0) LOG(FATAL) << "Out of bounds in interpolation.\n";

    // Interpolate at desired time
    Eigen::Affine3d interp_world_to_ref_aff
      = rig::linearInterp(alpha, beg_world_to_ref_aff,
                                end_world_to_ref_aff);

    return interp_world_to_ref_aff;
}
  
// Calculate interpolated world to camera transform. Use the
// convention that if beg_ref_stamp == end_ref_stamp, then only
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
                                   double cam_stamp) {
  
  Eigen::Affine3d ref_to_cam_aff;
  array_to_rigid_transform(ref_to_cam_aff, // output
                           ref_to_cam_trans);
  
  Eigen::Affine3d interp_world_to_ref_aff
    = calc_interp_world_to_ref(beg_world_to_ref_t, end_world_to_ref_t,  
                               beg_ref_stamp, end_ref_stamp,  ref_to_cam_offset,  
                               cam_stamp);
  
  return ref_to_cam_aff * interp_world_to_ref_aff;
}


// Compute the transforms from the world to every camera, based on the rig transforms.
void calcWorldToCamWithRig(// Inputs
                           bool have_rig,
                           std::vector<rig::cameraImage> const& cams,
                           std::vector<double> const& world_to_ref_vec,
                           std::vector<double> const& ref_timestamps,
                           std::vector<double> const& ref_to_cam_vec,
                           std::vector<double> const& ref_to_cam_timestamp_offsets,
                           // Output
                           std::vector<Eigen::Affine3d>& world_to_cam) {
  
  if (ref_to_cam_vec.size() / rig::NUM_RIGID_PARAMS != ref_to_cam_timestamp_offsets.size())
    LOG(FATAL) << "Must have as many transforms to reference as timestamp offsets.\n";
  if (world_to_ref_vec.size() / rig::NUM_RIGID_PARAMS != ref_timestamps.size())
    LOG(FATAL) << "Must have as many reference timestamps as reference cameras.\n";

  // What is stored in "cams" is completely different when a rig is not used,
  // even one is available and is good, so then this code will give wrong results.
  if (!have_rig) 
    LOG(FATAL) << "calcWorldToCamWithRig: Must have a rig.\n";
  
  world_to_cam.resize(cams.size());

  for (size_t it = 0; it < cams.size(); it++) {
    int beg_index = cams[it].beg_ref_index;
    int end_index = cams[it].end_ref_index;
    int cam_type = cams[it].camera_type;
    world_to_cam[it] = rig::calcWorldToCamBase
      (&world_to_ref_vec[rig::NUM_RIGID_PARAMS * beg_index],
       &world_to_ref_vec[rig::NUM_RIGID_PARAMS * end_index],
       &ref_to_cam_vec[rig::NUM_RIGID_PARAMS * cam_type],
       ref_timestamps[beg_index], ref_timestamps[end_index],
       ref_to_cam_timestamp_offsets[cam_type],
       cams[it].timestamp);
  }
  return;
}

// A version of the above with the data stored differently
void calcWorldToCamWithRig(// Inputs
                           bool have_rig,
                           std::vector<rig::cameraImage> const& cams,
                           std::vector<Eigen::Affine3d> const& world_to_ref,
                           std::vector<double> const& ref_timestamps,
                           std::vector<Eigen::Affine3d> const& ref_to_cam,
                           std::vector<double> const& ref_to_cam_timestamp_offsets,
                           // Output
                           std::vector<Eigen::Affine3d>& world_to_cam) {
  
  int num_cam_types = ref_to_cam.size();
  std::vector<double> ref_to_cam_vec(num_cam_types * rig::NUM_RIGID_PARAMS);
  for (int cam_type = 0; cam_type < num_cam_types; cam_type++)
    rig::rigid_transform_to_array
      (ref_to_cam[cam_type], &ref_to_cam_vec[rig::NUM_RIGID_PARAMS * cam_type]);

  int num_ref_cams = world_to_ref.size();
  if (world_to_ref.size() != ref_timestamps.size())
    LOG(FATAL) << "Must have as many ref cam timestamps as ref cameras.\n";
  std::vector<double> world_to_ref_vec(num_ref_cams * rig::NUM_RIGID_PARAMS);
  for (int cid = 0; cid < num_ref_cams; cid++)
    rig::rigid_transform_to_array(world_to_ref[cid],
                                        &world_to_ref_vec[rig::NUM_RIGID_PARAMS * cid]);

  calcWorldToCamWithRig(// Inputs
                        have_rig, cams, world_to_ref_vec,
                        ref_timestamps, ref_to_cam_vec,  
                        ref_to_cam_timestamp_offsets,  
                        // Output
                        world_to_cam);
}
  
// Calculate world_to_cam transforms from their representation in a
// vector, rather than using reference cameras, extrinsics and
// timestamp interpolation. Only for use with --no_rig, when
// each camera varies independently.
void calcWorldToCamNoRig(// Inputs
                         std::vector<rig::cameraImage> const& cams,
                         std::vector<double> const& world_to_cam_vec,
                         // Output
                         std::vector<Eigen::Affine3d>& world_to_cam) {
  
  if (world_to_cam_vec.size() != cams.size() * rig::NUM_RIGID_PARAMS)
    LOG(FATAL) << "Incorrect size for world_to_cam_vec.\n";

  for (size_t cid = 0; cid < cams.size(); cid++)
    rig::array_to_rigid_transform(world_to_cam[cid],  // output
                                        &world_to_cam_vec[rig::NUM_RIGID_PARAMS * cid]);
}

// Use one of the two implementations above. Care is needed as when
// there are no extrinsics, each camera is on its own, so the input is
// in world_to_cam_vec and not in world_to_ref_vec
void calcWorldToCam(// Inputs
  bool no_rig, std::vector<rig::cameraImage> const& cams,
  std::vector<double> const& world_to_ref_vec, std::vector<double> const& ref_timestamps,
  std::vector<double> const& ref_to_cam_vec, std::vector<double> const& world_to_cam_vec,
  std::vector<double> const& ref_to_cam_timestamp_offsets,
  // Output
  std::vector<Eigen::Affine3d>& world_to_cam) {
  if (!no_rig)
    calcWorldToCamWithRig(// Inputs
                          !no_rig,
                          cams, world_to_ref_vec, ref_timestamps, ref_to_cam_vec,
                          ref_to_cam_timestamp_offsets,
                          // Output
                          world_to_cam);
  else
    calcWorldToCamNoRig(// Inputs
      cams, world_to_cam_vec,
      // Output
      world_to_cam);

  return;
}

// Extract a affine transform to an array of length NUM_AFFINE_PARAMS
void affine_transform_to_array(Eigen::Affine3d const& aff, double* arr) {
  Eigen::MatrixXd M = aff.matrix();
  int count = 0;
  // The 4th row always has 0, 0, 0, 1
  for (int row = 0; row < 3; row++) {
    for (int col = 0; col < 4; col++) {
      arr[count] = M(row, col);
      count++;
    }
  }
}

// Convert an array of length NUM_AFFINE_PARAMS to a affine
// transform. Normalize the quaternion to make it into a rotation.
void array_to_affine_transform(Eigen::Affine3d& aff, const double* arr) {
  Eigen::MatrixXd M = Eigen::Matrix<double, 4, 4>::Identity();

  int count = 0;
  // The 4th row always has 0, 0, 0, 1
  for (int row = 0; row < 3; row++) {
    for (int col = 0; col < 4; col++) {
      M(row, col) = arr[count];
      count++;
    }
    }

  aff.matrix() = M;
}

// Find the median of some matrices, by finding the median for each entry
Eigen::MatrixXd median_matrix(std::vector<Eigen::MatrixXd> const& transforms) {

  // Sanity checks
  if (transforms.empty()) 
    LOG(FATAL) << "Cannot find the median of an empty set of matrices.\n";

  for (size_t cam_it = 0; cam_it < transforms.size(); cam_it++) {
    if (transforms[cam_it].rows() != 4 || transforms[cam_it].cols() != 4) 
      LOG(FATAL) << "Expecting square matrices of size 4 in the median computation.\n";
  }
  
  Eigen::MatrixXd median_trans = Eigen::MatrixXd::Zero(4, 4);
  for (int col = 0; col < 4; col++) {
    for (int row = 0; row < 4; row++) {
      
      std::vector<double> vals;
      for (size_t cam_it = 0; cam_it < transforms.size(); cam_it++)
        vals.push_back(transforms[cam_it](col, row));
      
      median_trans(col, row) = vals[vals.size()/2];
    }
  }

  return median_trans;
}
  
// Given the transforms from each camera to the world and their timestamps,
// find an initial guess for the relationship among the sensors on the rig.
// Note that strictly speaking the transforms in world_to_ref_vec are among
// those in world_to_cam, but we don't have a way of looking them up in that
// vector.
void calc_rig_trans(std::vector<rig::cameraImage> const& cams,
                    std::vector<Eigen::Affine3d>  const& world_to_ref,
                    std::vector<Eigen::Affine3d>  const& world_to_cam,
                    std::vector<double>           const& ref_timestamps,
                    rig::RigSet                        & R) { // update this
  // Sanity check
  if (cams.size() != world_to_cam.size()) 
    LOG(FATAL) << "There must be as many world to cam transforms as metadata sets for them.\n";

  int num_ref_cams = world_to_ref.size();
  if (world_to_ref.size() != ref_timestamps.size())
    LOG(FATAL) << "Must have as many ref cam timestamps as ref cameras.\n";
  std::vector<double> world_to_ref_vec(num_ref_cams * rig::NUM_RIGID_PARAMS);
  for (int cid = 0; cid < num_ref_cams; cid++)
    rig::rigid_transform_to_array(world_to_ref[cid],
                                        &world_to_ref_vec[rig::NUM_RIGID_PARAMS * cid]);
  
  // Resize the output
  int num_cam_types = R.cam_names.size();
  R.ref_to_cam_trans.resize(num_cam_types);

  // Calc all transforms
  std::map<int, std::vector<Eigen::MatrixXd>> transforms_map;
  for (size_t cam_it = 0; cam_it < cams.size(); cam_it++) {
    int beg_index = cams[cam_it].beg_ref_index;
    int end_index = cams[cam_it].end_ref_index;
    int cam_type = cams[cam_it].camera_type;
    
    if (R.isRefSensor(R.cam_names[cam_type])) {
      // The identity transform, from the ref sensor to itself
      transforms_map[cam_type].push_back(Eigen::MatrixXd::Identity(4, 4));
    } else {
      // We have world_to_ref transform at times bracketing current time,
      // and world_to_cam at current time. Interp world_to_ref
      // at current time, then find ref_to_cam.
      Eigen::Affine3d interp_world_to_ref_aff
        = rig::calc_interp_world_to_ref
        (&world_to_ref_vec[rig::NUM_RIGID_PARAMS * beg_index],
         &world_to_ref_vec[rig::NUM_RIGID_PARAMS * end_index],
         ref_timestamps[beg_index], ref_timestamps[end_index],
         R.ref_to_cam_timestamp_offsets[cam_type],
         cams[cam_it].timestamp);
      
      Eigen::Affine3d ref_to_cam_aff
        = world_to_cam[cam_it] * (interp_world_to_ref_aff.inverse());
      transforms_map[cam_type].push_back(ref_to_cam_aff.matrix());
    }
  }
  
  // Find median, for robustness
  for (auto it = transforms_map.begin(); it != transforms_map.end(); it++) {
    int cam_type = it->first;
    auto & transforms = it->second;
    
    if (transforms.empty()) 
        LOG(FATAL) << "No poses were found for rig sensor with id: " << cam_type << "\n";

    Eigen::MatrixXd median_trans = median_matrix(transforms);
    R.ref_to_cam_trans[cam_type].matrix() = median_trans;
    R.ref_to_cam_trans[cam_type].linear() /= 
      pow(R.ref_to_cam_trans[cam_type].linear().determinant(), 1.0 / 3.0);
  }
  
  return;
}
  
// Extract a rigid transform to an array of length NUM_RIGID_PARAMS
void rigid_transform_to_array(Eigen::Affine3d const& aff, double* arr) {
  
  for (size_t it = 0; it < 3; it++) 
    arr[it] = aff.translation()[it];

  Eigen::Quaterniond R(aff.linear());
  arr[3] = R.x();
  arr[4] = R.y();
  arr[5] = R.z();
  arr[6] = R.w();
}

// Convert an array of length NUM_RIGID_PARAMS to a rigid
// transform. Normalize the quaternion to make it into a rotation.
void array_to_rigid_transform(Eigen::Affine3d& aff, const double* arr) {
  
  for (size_t it = 0; it < 3; it++) 
    aff.translation()[it] = arr[it];

  Eigen::Quaterniond R(arr[6], arr[3], arr[4], arr[5]);
  R.normalize();

  aff = Eigen::Affine3d(Eigen::Translation3d(arr[0], arr[1], arr[2])) * Eigen::Affine3d(R);
}

// A function to compute the camera position in world coordinates given
// the world_to_cam array
Eigen::Vector3d calc_cam_position(double const* world_to_cam) {
  
  Eigen::Affine3d world_to_cam_aff;
  array_to_rigid_transform(world_to_cam_aff, world_to_cam);
  Eigen::Vector3d t(world_to_cam_aff.translation());
  Eigen::Vector3d camera_center = -world_to_cam_aff.rotation().inverse() * t;  
  
  return camera_center;
}

// Compute the n-weight slerp, analogous to the linear combination
// W[0]*Q[0] + ... + W[n-1]*Q[n-1]. This is experimental.
// We assume the sum of weights is 1.
// TODO(oalexan1): Move this to transform_utils.cc.
Eigen::Quaternion<double> slerp_n(std::vector<double> const& W,
                                  std::vector<Eigen::Quaternion<double>> const& Q) {
  if (W.size() != Q.size())
    LOG(FATAL) << "Expecting as many quaternions as weights.";
  
  if (Q.empty())
    LOG(FATAL) << "Expecting at least one quaternion and weight.";

  if (Q.size() == 1)
    return Q[0];

  if (Q.size() == 2) {
    if (!(std::abs(W[0] + W[1] - 1.0) < 1e-6 && W[0] >= 0 && W[1] >= 0))
      LOG(FATAL) << "Expecting the weights to be >= 0 and sum up to 1.";
    return Q[0].slerp(W[1], Q[1]);
  }

  // Call recursively this function with fewer terms
  double sum = W[0] + W[1];
  if (sum == 0) sum = 1.0;
  Eigen::Quaternion<double> q = Q[0].slerp(W[1]/sum, Q[1]);
  std::vector<double> W2 = W;
  std::vector<Eigen::Quaternion<double>> Q2 = Q;
  W2.erase(W2.begin());
  Q2.erase(Q2.begin());
  W2[0] = sum;
  Q2[0] = q;
  return slerp_n(W2, Q2);
}
  
// Given two sets of 3D points, find the rotation + translation + scale
// which best maps the first set to the second.
// Source: http://en.wikipedia.org/wiki/Kabsch_algorithm
// TODO(oalexan1): Use the version robust to outliers!  
void Find3DAffineTransform(Eigen::Matrix3Xd const & in,
                           Eigen::Matrix3Xd const & out,
                           Eigen::Affine3d* result) {
  // Default output
  result->linear() = Eigen::Matrix3d::Identity(3, 3);
  result->translation() = Eigen::Vector3d::Zero();

  if (in.cols() != out.cols())
    LOG(FATAL) << "Find3DAffineTransform(): input data mis-match.";

  // Local copies we can modify
  Eigen::Matrix3Xd local_in = in, local_out = out;

  // First find the scale, by finding the ratio of sums of some distances,
  // then bring the datasets to the same scale.
  double dist_in = 0, dist_out = 0;
  for (int col = 0; col < local_in.cols()-1; col++) {
    dist_in  += (local_in.col(col+1) - local_in.col(col)).norm();
    dist_out += (local_out.col(col+1) - local_out.col(col)).norm();
  }
  if (dist_in <= 0 || dist_out <= 0)
    return;
  double scale = dist_out/dist_in;
  local_out /= scale;

  // Find the centroids then shift to the origin
  Eigen::Vector3d in_ctr = Eigen::Vector3d::Zero();
  Eigen::Vector3d out_ctr = Eigen::Vector3d::Zero();
  for (int col = 0; col < local_in.cols(); col++) {
    in_ctr  += local_in.col(col);
    out_ctr += local_out.col(col);
  }
  in_ctr /= local_in.cols();
  out_ctr /= local_out.cols();
  for (int col = 0; col < local_in.cols(); col++) {
    local_in.col(col)  -= in_ctr;
    local_out.col(col) -= out_ctr;
  }

  // SVD
  Eigen::Matrix3d Cov = local_in * local_out.transpose();
  Eigen::JacobiSVD<Eigen::Matrix3d> svd(Cov, Eigen::ComputeFullU | Eigen::ComputeFullV);

  // Find the rotation
  double d = (svd.matrixV() * svd.matrixU().transpose()).determinant();
  if (d > 0)
    d = 1.0;
  else
    d = -1.0;
  Eigen::Matrix3d I = Eigen::Matrix3d::Identity(3, 3);
  I(2, 2) = d;
  Eigen::Matrix3d R = svd.matrixV() * I * svd.matrixU().transpose();

  // The final transform
  result->linear() = scale * R;
  result->translation() = scale*(out_ctr - R*in_ctr);
}

// Extract control points and the images they correspond 2 from
// a Hugin project file
void ParseHuginControlPoints(std::string const& hugin_file,
                             std::vector<std::string> * images,
                             Eigen::MatrixXd * points) {
  
  // Initialize the outputs
  (*images).clear();
  *points = Eigen::MatrixXd(6, 0); // this will be resized as points are added

  std::ifstream hf(hugin_file.c_str());
  if (!hf.good())
    LOG(FATAL) << "ParseHuginControlPoints(): Could not open hugin file: " << hugin_file;

  int num_points = 0;
  std::string line;
  while (getline(hf, line)) {
    // Parse for images
    if (line.find("i ") == 0) {
      size_t it = line.find("n\"");
      if (it == std::string::npos)
        LOG(FATAL) << "ParseHuginControlPoints(): Invalid line: " << line;
      it += 2;
      std::string image;
      while (it < line.size() && line[it] != '"') {
        image += line[it];
        it++;
      }
      (*images).push_back(image);
    }

    // Parse control points
    if (line.find("c ") == 0) {
      // First wipe all letters
      std::string orig_line = line;
      char * ptr = const_cast<char*>(line.c_str());
      for (size_t i = 0; i < line.size(); i++) {
        // Wipe some extra chars
        if ( (ptr[i] >= 'a' && ptr[i] <= 'z') ||
             (ptr[i] >= 'A' && ptr[i] <= 'Z') )
          ptr[i] = ' ';
      }

      // Out of a line like:
      // c n0 N1 x367 y240 X144.183010710425 Y243.04008545843 t0
      // we store the numbers, 0, 1, 367, 240, 144.183010710425 243.04008545843
      // as a column.
      // The stand for left image cid, right image cid,
      // left image x, left image y, right image x, right image y.
      double a, b, c, d, e, f;
      if (sscanf(ptr, "%lf %lf %lf %lf %lf %lf", &a, &b, &c, &d, &e, &f) != 6)
        LOG(FATAL) << "ParseHuginControlPoints(): Could not scan line: " << line;

      // The left and right images must be different
      if (a == b)
        LOG(FATAL) << "The left and right images must be distinct. "
                   << "Offending line in " << hugin_file << " is:\n"
                   << orig_line << "\n";

      num_points++;
      (*points).conservativeResize(Eigen::NoChange_t(), num_points);
      (*points).col(num_points - 1) << a, b, c, d, e, f;
    }
  }

  return;
}

// A little helper function
bool is_blank(std::string const& line) {
  return (line.find_first_not_of(" \t\n\v\f\r") == std::string::npos);
}

// Parse a file having on each line xyz coordinates
void ParseXYZ(std::string const& xyz_file,
                              Eigen::MatrixXd * xyz) {
  // Initialize the outputs
  *xyz = Eigen::MatrixXd(3, 1);

  std::ifstream hf(xyz_file.c_str());
  if (!hf.good())
    LOG(FATAL) << "ParseXYZ(): Could not open hugin file: " << xyz_file;

  int num_points = 0;
  std::string line;
  while (getline(hf, line)) {
    // Ignore lines starting with comments and empty lines
    if (line.find("#") == 0 || is_blank(line)) continue;

    // Apparently sometimes empty lines show up as if of length 1
    if (line.size() == 1)
      continue;

    // Replace commas with spaces
    char * ptr = const_cast<char*>(line.c_str());
    for (size_t c = 0; c < line.size(); c++)
      if (ptr[c] == ',') ptr[c] = ' ';
    double x, y, z;
    if (sscanf(line.c_str(), "%lf %lf %lf", &x, &y, &z) != 3)
      LOG(FATAL) << "ParseXYZ(): Could not scan line: '" << line << "'\n";

    num_points++;
    (*xyz).conservativeResize(Eigen::NoChange_t(), num_points);
    (*xyz).col(num_points-1) << x, y, z;
  }
}
  
// Apply a given transform to the given set of cameras.
// We assume that the transform is of the form
// T(x) = scale * rotation * x + translation
void TransformCameras(Eigen::Affine3d const& T,
                      std::vector<Eigen::Affine3d> &world_to_cam) {
  
  // Inverse of rotation component
  double scale = pow(T.linear().determinant(), 1.0 / 3.0);
  Eigen::MatrixXd Tinv = (T.linear()/scale).inverse();

  for (size_t cid = 0; cid < world_to_cam.size(); cid++) {
    world_to_cam[cid].linear() = world_to_cam[cid].linear()*Tinv;
    world_to_cam[cid].translation() = scale*world_to_cam[cid].translation() -
      world_to_cam[cid].linear()*T.translation();
  }
}

// Apply same transform as above to points
void TransformPoints(Eigen::Affine3d const& T, std::vector<Eigen::Vector3d> *xyz) {
  for (size_t pid = 0; pid < (*xyz).size(); pid++)
    (*xyz)[pid] = T * (*xyz)[pid];
}

// Apply a given transform to the specified xyz points, and adjust accordingly the cameras
// for consistency. We assume that the transform is of the form
// A(x) = scale * rotation * x + translation
void TransformCamerasAndPoints(Eigen::Affine3d const& A,
                               std::vector<Eigen::Affine3d> *cid_to_cam_t,
                               std::vector<Eigen::Vector3d> *xyz) {
  TransformCameras(A, *cid_to_cam_t);
  TransformPoints(A, xyz);
}
  
// Apply a registration transform to a rig. The only thing that
// changes is scale, as the rig transforms are between coordinate
// systems of various cameras.
void TransformRig(Eigen::Affine3d const& T, std::vector<Eigen::Affine3d> & ref_to_cam_trans) {
  double scale = pow(T.linear().determinant(), 1.0 / 3.0);
  for (size_t cam_type = 0; cam_type < ref_to_cam_trans.size(); cam_type++) 
    ref_to_cam_trans[cam_type].translation() *= scale;
}

// Find the name of the camera type of the images used in registration.
// The registration images must all be acquired with the same sensor.  
std::string registrationCamName(std::string const& hugin_file,
                                std::vector<std::string> const& cam_names,
                                std::vector<rig::cameraImage> const & cams) {

  std::vector<std::string> images;
  Eigen::MatrixXd user_ip;
  Eigen::MatrixXd user_xyz;
  ParseHuginControlPoints(hugin_file, &images, &user_ip);

  // Must create a map from the image name in cams to sensor type
  std::map<std::string, int> image_to_cam_type;
  for (size_t cid = 0; cid < cams.size(); cid++)
    image_to_cam_type[cams[cid].image_name] = cams[cid].camera_type;
  
  std::set<std::string> sensors;
  for (size_t cid = 0; cid < images.size(); cid++) {
    // Find the image in the map
    auto it = image_to_cam_type.find(images[cid]);
    if (it == image_to_cam_type.end())
      LOG(FATAL) << "Cannot find image: " << images[cid] 
        << " from the Hugin file having control points in the input SfM map.\n";
      
     sensors.insert(cam_names.at(it->second));
  }
  
  if (sensors.size() != 1) 
    LOG(FATAL) << "All images used in registration must be for the same sensor. "
               << "Check the registration file: " << hugin_file << ".\n";
  
  return *sensors.begin();
}

// Find the 3D transform from an abstract coordinate system to the
// world, given control points (pixel matches) and corresponding 3D
// measurements. It is assumed all images are acquired with the same camera.
Eigen::Affine3d 
registrationTransform(std::string                  const& hugin_file,
                      std::string                  const& xyz_file,
                      rig::CameraParameters     const& cam_params,
                      std::vector<std::string>     const& cid_to_filename,
                      std::vector<Eigen::Affine3d> const& world_to_cam_trans) { 
  
  // Get the interest points in the images, and their positions in
  // the world coordinate system, as supplied by a user.
  // Parse and concatenate that information from multiple files.
  std::vector<std::string> images;
  Eigen::MatrixXd user_ip;
  Eigen::MatrixXd user_xyz;
  
  ParseHuginControlPoints(hugin_file, &images, &user_ip);
  ParseXYZ(xyz_file, &user_xyz);

  int num_points = user_ip.cols();
  if (num_points != user_xyz.cols())
    LOG(FATAL) << "Could not parse an equal number of control "
               << "points and xyz coordinates. Their numbers are "
               << num_points << " vs " << user_xyz.cols() << ".\n";


  std::map<std::string, int> filename_to_cid;
  for (size_t cid = 0; cid < cid_to_filename.size(); cid++)
    filename_to_cid[cid_to_filename[cid]] = cid;

  // Wipe images that are missing from the map
  std::map<int, int> cid2cid;
  int good_cid = 0;
  for (size_t cid = 0; cid < images.size(); cid++) {
    std::string image = images[cid];
    if (filename_to_cid.find(image) == filename_to_cid.end()) {
      LOG(WARNING) << "Will ignore image missing from map: " << image;
      continue;
    }
    cid2cid[cid] = good_cid;
    images[good_cid] = images[cid];
    good_cid++;
  }
  images.resize(good_cid);

  // Remove points corresponding to images missing from map
  int good_pid = 0;
  for (int pid = 0; pid < num_points; pid++) {
    int id1 = user_ip(0, pid);
    int id2 = user_ip(1, pid);
    if (cid2cid.find(id1) == cid2cid.end() || cid2cid.find(id2) == cid2cid.end()) {
      continue;
    }
    user_ip.col(good_pid) = user_ip.col(pid);
    user_xyz.col(good_pid) = user_xyz.col(pid);
    good_pid++;
  }
  user_ip.conservativeResize(Eigen::NoChange_t(), good_pid);
  user_xyz.conservativeResize(Eigen::NoChange_t(), good_pid);
  num_points = good_pid;
  for (int pid = 0; pid < num_points; pid++) {
    int id1 = user_ip(0, pid);
    int id2 = user_ip(1, pid);
    if (cid2cid.find(id1) == cid2cid.end() || cid2cid.find(id2) == cid2cid.end())
      LOG(FATAL) << "Book-keeping failure in registration.";
    user_ip(0, pid) = cid2cid[id1];
    user_ip(1, pid) = cid2cid[id2];
  }


  if (num_points < 3) 
    LOG(FATAL) << "Must have at least 3 points to apply registration. Got: "
               << num_points << "\n";
  
  // Iterate over the control points in the hugin file. Copy the
  // control points to the list of user keypoints, and create the
  // corresponding user_pid_to_cid_fid.
  rig::CidToKeypointMatVec user_cid_to_keypoint_map;
  std::vector<std::map<int, int> > user_pid_to_cid_fid;
  user_cid_to_keypoint_map.resize(cid_to_filename.size());
  user_pid_to_cid_fid.resize(num_points);
  for (int pid = 0; pid < num_points; pid++) {
    // Left and right image indices
    int id1 = user_ip(0, pid);
    int id2 = user_ip(1, pid);

    // Sanity check
    if (id1 < 0 || id2 < 0 ||
        id1 >= static_cast<int>(images.size()) ||
        id2 >= static_cast<int>(images.size()) )
      LOG(FATAL) << "Invalid image indices in the hugin file: " << id1 << ' ' << id2;

    // Find the corresponding indices in the map where these keypoints will go to
    if (filename_to_cid.find(images[id1]) == filename_to_cid.end())
      LOG(FATAL) << "File missing from map: " << images[id1];
    if (filename_to_cid.find(images[id2]) == filename_to_cid.end())
      LOG(FATAL) << "File missing from map: " << images[id2];
    int cid1 = filename_to_cid[images[id1]];
    int cid2 = filename_to_cid[images[id2]];

    // Append to the keypoints for cid1
    Eigen::Matrix<double, 2, -1> &M1 = user_cid_to_keypoint_map[cid1];  // alias
    Eigen::Matrix<double, 2, -1> N1(M1.rows(), M1.cols()+1);
    N1 << M1, user_ip.block(2, pid, 2, 1);  // left image pixel x and pixel y
    M1.swap(N1);

    // Append to the keypoints for cid2
    Eigen::Matrix<double, 2, -1> &M2 = user_cid_to_keypoint_map[cid2];  // alias
    Eigen::Matrix<double, 2, -1> N2(M2.rows(), M2.cols()+1);
    N2 << M2, user_ip.block(4, pid, 2, 1);  // right image pixel x and pixel y
    M2.swap(N2);

    // The corresponding user_pid_to_cid_fid
    user_pid_to_cid_fid[pid][cid1] = user_cid_to_keypoint_map[cid1].cols()-1;
    user_pid_to_cid_fid[pid][cid2] = user_cid_to_keypoint_map[cid2].cols()-1;
  }

  // Apply undistortion
  Eigen::Vector2d output;
  for (size_t cid = 0; cid < user_cid_to_keypoint_map.size(); cid++) {
    for (int i = 0; i < user_cid_to_keypoint_map[cid].cols(); i++) {
      cam_params.Convert<rig::DISTORTED, rig::UNDISTORTED_C>
        (user_cid_to_keypoint_map[cid].col(i), &output);
      user_cid_to_keypoint_map[cid].col(i) = output;
    }
  }

  // Triangulate to find the coordinates of the current points
  // in the virtual coordinate system
  std::vector<Eigen::Vector3d> unreg_pid_to_xyz;
  bool rm_invalid_xyz = false;  // there should be nothing to remove hopefully
  rig::Triangulate(rm_invalid_xyz,
              cam_params.GetFocalLength(),
              world_to_cam_trans,
              user_cid_to_keypoint_map,
              &user_pid_to_cid_fid,
              &unreg_pid_to_xyz);

  double mean_err = 0;
  for (int i = 0; i < user_xyz.cols(); i++) {
    Eigen::Vector3d a = unreg_pid_to_xyz[i];
    Eigen::Vector3d b = user_xyz.col(i);
    mean_err += (a-b).norm();
  }
  mean_err /= user_xyz.cols();
  std::cout << "Mean absolute error before registration: " << mean_err << " meters" << std::endl;
  std::cout << "Un-transformed computed xyz -- measured xyz -- error diff -- error norm (meters)"
            << std::endl;

  for (int i = 0; i < user_xyz.cols(); i++) {
    Eigen::Vector3d a = unreg_pid_to_xyz[i];
    Eigen::Vector3d b = user_xyz.col(i);
    std::cout << print_vec(a) << " -- "
              << print_vec(b) << " -- "
              << print_vec(a-b) << " -- "
              << print_vec((a - b).norm())
              << std::endl;
  }

  // Find the transform from the computed map coordinate system
  // to the world coordinate system.
  int np = unreg_pid_to_xyz.size();
  Eigen::Matrix3Xd in(3, np);
  for (int i = 0; i < np; i++)
    in.col(i) = unreg_pid_to_xyz[i];

  Eigen::Affine3d registration_trans;  
  Find3DAffineTransform(in, user_xyz, &registration_trans);

  mean_err = 0.0;
  for (int i = 0; i < user_xyz.cols(); i++)
    mean_err += (registration_trans*in.col(i) - user_xyz.col(i)).norm();
  mean_err /= user_xyz.cols();

  // We don't use LOG(INFO) below, as it does not play well with
  // Eigen.
  double scale = pow(registration_trans.linear().determinant(), 1.0 / 3.0);
  std::cout << "Registration transform (to measured world coordinates)." << std::endl;
  std::cout << "Rotation:\n" << registration_trans.linear() / scale << std::endl;
  std::cout << "Scale:\n" << scale << std::endl;
  std::cout << "Translation:\n" << registration_trans.translation().transpose()
            << std::endl;

  std::cout << "Mean absolute error after registration: "
            << mean_err << " meters" << std::endl;

  std::cout << "Transformed computed xyz -- measured xyz -- "
            << "error diff - error norm (meters)" << std::endl;
  for (int i = 0; i < user_xyz.cols(); i++) {
    Eigen::Vector3d a = registration_trans*in.col(i);
    Eigen::Vector3d b = user_xyz.col(i);
    int id1 = user_ip(0, i);
    int id2 = user_ip(1, i);

    std::cout << print_vec(a) << " -- "
              << print_vec(b) << " -- "
              << print_vec(a - b) << " -- "
              << print_vec((a - b).norm()) << " -- "
              << images[id1] << ' '
              << images[id2] << std::endl;
  }

  return registration_trans;
}

// Apply a transform to inlier triangulated points  
void transformInlierTriPoints(// Inputs
  Eigen::Affine3d const& trans,
  rig::PidCidFid const& pid_to_cid_fid,
  PidCidFidMap const& pid_cid_fid_inlier,
  std::vector<Eigen::Vector3d> & xyz_vec) { // output
  
  if (pid_to_cid_fid.size() != pid_cid_fid_inlier.size())
    LOG(FATAL) << "Expecting as many inlier flags as there are tracks.\n";
  if (pid_to_cid_fid.size() != xyz_vec.size()) 
    LOG(FATAL) << "Expecting as many tracks as there are triangulated points.\n";

  for (size_t pid = 0; pid < pid_to_cid_fid.size(); pid++) {

    bool isInlierXyz = false;
    for (auto cid_fid1 = pid_to_cid_fid[pid].begin();
         cid_fid1 != pid_to_cid_fid[pid].end(); cid_fid1++) {
      int cid1 = cid_fid1->first;
      int fid1 = cid_fid1->second;

      // Deal with inliers only
      if (!rig::getMapValue(pid_cid_fid_inlier, pid, cid1, fid1)) continue;

      isInlierXyz = true;
      break;
    }

    if (isInlierXyz) 
      xyz_vec[pid] = trans * xyz_vec[pid];
  }
  
  return;
}

}  // end namespace rig
