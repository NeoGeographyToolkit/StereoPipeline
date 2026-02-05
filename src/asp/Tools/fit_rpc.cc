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

// TODO(oalexan1): Do not use -out_dir. Use -out_config.
// TODO(oalexan1): Print an underestimate and overestimate for the undistorted win.
// TODO(oalexan1): Must have sensor name as an option. For now it defaults to sensor 0.
// TODO(oalexan1): Must document this tool.

#include <ceres/ceres.h>
#include <ceres/rotation.h>
#include <ceres/problem.h>
#include <ceres/solver.h>
#include <ceres/cost_function.h>
#include <ceres/loss_function.h>
#include <ceres/dynamic_numeric_diff_cost_function.h>
#include <ceres/numeric_diff_cost_function.h>
#include <ceres/autodiff_cost_function.h>

#include <opencv2/imgproc.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/core/utility.hpp>

#include <Rig/RigRpcDistortion.h>
#include <Rig/RigCameraParams.h>
#include <Rig/RigUtils.h>
#include <Rig/RigConfig.h>

#include <gflags/gflags.h>
#include <glog/logging.h>

#include <Eigen/Geometry>
#include <Eigen/Core>

#include <boost/filesystem.hpp>

#include <string>
#include <map>
#include <iostream>
#include <fstream>
#include <iomanip>

namespace fs = boost::filesystem;

DEFINE_int32(rpc_degree, -1,
             "The degree of the RPC model to fit.");

DEFINE_int32(num_samples, -1,
             "The number of row and column samples to use to fit the RPC model.");

DEFINE_int32(num_iterations, 20, "How many solver iterations to perform in calibration.");

DECLARE_int32(num_threads); // declared externally

DEFINE_double(parameter_tolerance, 1e-12, "Stop when the optimization variables change by "
              "less than this.");

DEFINE_string(camera_config, "",
              "Read the camera configuration from this file.");

DEFINE_string(out_dir, "",
              "Write here the camera configuration having the RPC fit.");

DEFINE_bool(verbose, false,
            "Print more information about what the tool is doing.");


int main(int argc, char** argv) {
  google::InitGoogleLogging(argv[0]);
  google::ParseCommandLineFlags(&argc, &argv, true);

  if (FLAGS_camera_config.empty())
    LOG(FATAL) << "Camera config file was not specified.";
  
  if (FLAGS_out_dir.empty())
    LOG(FATAL) << "Output camera config directory was not specified.";

  if (FLAGS_rpc_degree <= 0)
    LOG(FATAL) << "The RPC degree must be positive.";

  if (FLAGS_num_samples <= 0)
      LOG(FATAL) << "The number of samples must be positive.";

  rig::RigSet R;
  bool use_initial_rig_transforms = true; // dictated by the api
  rig::readRigConfig(FLAGS_camera_config, use_initial_rig_transforms, R);
  
  std::cout << "Focal length is " << R.cam_params[0].GetFocalVector().transpose() << std::endl;

  Eigen::VectorXd rpc_dist_coeffs;
  rig::fitRpcDist(FLAGS_rpc_degree, FLAGS_num_samples,
                        R.cam_params[0],
                        FLAGS_num_threads, FLAGS_num_iterations,
                        FLAGS_parameter_tolerance,
                        FLAGS_verbose,
                        // Output
                        rpc_dist_coeffs);
  
  rig::RPCLensDistortion rpc;
  rpc.set_distortion_parameters(rpc_dist_coeffs);

  rig::evalRpcDistUndist(FLAGS_num_samples, R.cam_params[0], rpc);

  // Create the model with RPC distortion. Note how we pass both the distortion
  // and undistortion RPC coefficients.
  R.cam_params[0].SetDistortion(rpc_dist_coeffs);

  rig::writeRigConfig(FLAGS_out_dir + "/rig_config.txt", use_initial_rig_transforms, R);
  
  return 0;
}
