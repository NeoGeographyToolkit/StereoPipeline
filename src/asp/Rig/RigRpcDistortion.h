/* Copyright (c) 2017, United States Government, as represented by the
 * Administrator of the National Aeronautics and Space Administration.
 * 
 * All rights reserved.
 * 
 * The Astrobee platform is licensed under the Apache License, Version 2.0
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

#ifndef ASP_RIG_RPC_DISTORTION_H
#define ASP_RIG_RPC_DISTORTION_H

#include <Eigen/Core>

#include <string>
#include <vector>

namespace camera {
  // forward declaration
  class CameraParameters;
}

namespace rig {
class RPCLensDistortion {
  int m_rpc_degree;
  Eigen::VectorXd m_distortion;

 public:
  explicit RPCLensDistortion(); // NOLINT
  explicit RPCLensDistortion(Eigen::VectorXd const& params);
  void reset(int rpc_degree);  // Form the identity transform
  Eigen::VectorXd distortion_parameters() const;
  void set_image_size(Eigen::Vector2i const& image_size);
  void set_distortion_parameters(Eigen::VectorXd const& params);
  int num_dist_params() const { return m_distortion.size(); }

  Eigen::Vector2d distorted_coordinates(Eigen::Vector2d const& p) const;

  void write(std::ostream& os) const;
  void read(std::istream& os);

  static  std::string class_name()       { return "RPC"; }
  std::string name      () const { return class_name();  }

  static void init_as_identity(Eigen::VectorXd & params);
  static void increment_degree(Eigen::VectorXd & params);

  Eigen::VectorXd dist_undist_params();

  //Eigen::Vector2d distort_centered(Eigen::Vector2d const& p) const;
};

void unpack_params(Eigen::VectorXd const& params, 
                   Eigen::VectorXd& num_x, Eigen::VectorXd& den_x,
                   Eigen::VectorXd& num_y, Eigen::VectorXd& den_y);

void fitRpcDist(int rpc_degree, int num_samples, camera::CameraParameters const& cam_params,
                int num_opt_threads, int num_iterations, double parameter_tolerance,
                bool verbose,
                // Output
                Eigen::VectorXd & rpc_dist_coeffs);

void evalRpcDistUndist(int num_samples, camera::CameraParameters const& cam_params,
                       RPCLensDistortion const& rpc);
  
// Compute the RPC model with given coefficients at the given point. Recall that
// RPC is ratio of two polynomials in x and y. This assumes centered pixels that
// are normalized by the focal length.
Eigen::Vector2d compute_rpc(Eigen::Vector2d const& p, Eigen::VectorXd const& coeffs);
  
// Prepend a 1 to a vector
void prepend_1(Eigen::VectorXd & vec);
  
}
  
#endif  // ASP_RIG_RPC_DISTORTION_H
