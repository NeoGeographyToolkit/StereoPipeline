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

#include <asp/Camera/CsmModelFit.h>
#include <asp/Camera/CsmUtils.h>

#include <vw/Math/Quaternion.h>
#include <vw/Math/Geometry.h>

#include <asp/Camera/CsmModel.h>
#include <vw/Camera/OpticalBarModel.h>

#include <boost/algorithm/string/replace.hpp>
#include <boost/algorithm/string/case_conv.hpp>

#include <ceres/ceres.h>
#include <ceres/loss_function.h>

#include <iostream>

namespace asp {

// Find the rotation matrices, focal length, and optical center,
// that best fit a 2D matrix of sight vectors. Used for ASTER.

// Write a function having the block above 
void linescanFitSaveResiduals(ceres::Problem & problem, std::string const& resFile) {
  double total_cost = 0.0;
  ceres::Problem::EvaluateOptions eval_options;
  eval_options.num_threads = 1; // Use one thread to ensure a unique solution
  eval_options.apply_loss_function = false;  // want raw residuals
  std::vector<double> residuals;
  problem.Evaluate(eval_options, &total_cost, &residuals, NULL, NULL);

  // Save the residuals
  vw::vw_out() << "Writing residual norms to: " << resFile << std::endl;
  std::ofstream ofs(resFile.c_str());
  for (int i = 0; i < residuals.size()/3; i++) {
    int j = 3*i;
    ofs << norm_2(vw::Vector3(residuals[j], residuals[j+1], residuals[j+2])) << std::endl;
  }
  ofs.close();
}

// Convert axis angle to quaternion, in the format CSM expects
void axisAngleToCsmQuatVec(int num_poses, 
                           const double * axis_angle_vec,
                           std::vector<double> & quat_vec) {

  quat_vec.resize(4*num_poses);
  
  auto const & rot = axis_angle_vec;
  for (int i = 0; i < num_poses; i++) {
    vw::Vector3 axis_angle(rot[3*i+0], rot[3*i+1], rot[3*i+2]);
    vw::Quat q = vw::math::axis_angle_to_quaternion(axis_angle);
  
    // CSM wants the quaternion order to be (x, y, z, w)
    int j = 4*i;
    quat_vec[j+0] = q.x();
    quat_vec[j+1] = q.y();
    quat_vec[j+2] = q.z();
    quat_vec[j+3] = q.w();
  }
}

// Convert CSM quaternions to axis angle format
void csmQuatVecToAxisAngle(int num_poses, 
                           const double* quat_vec,
                           std::vector<double> & axis_angle_vec) {

  auto & rot = axis_angle_vec;
  rot.resize(3*num_poses);
  for (int i = 0; i < num_poses; i++) {
    int j = 4*i;
    // Note how we switch from (x,y,z,w) to (w,x,y,z) order
    vw::Quat q(quat_vec[j+3], quat_vec[j+0], quat_vec[j+1], quat_vec[j+2]);
    vw::Vector3 axis_angle = q.axis_angle();
    rot[3*i+0] = axis_angle[0];
    rot[3*i+1] = axis_angle[1];
    rot[3*i+2] = axis_angle[2];
  }
}

// The error between sight vectors and the camera directions 
struct SightMatError {

  SightMatError(SightMatT const& world_sight_mat, int row, int col, int min_col, int d_col):
  m_world_sight_mat(world_sight_mat), m_row(row), m_col(col), 
  m_min_col(min_col), m_d_col(d_col) {}

  // Error operator
  bool operator()(double const* const* parameters, double* residuals) const {

    const double* rotation       = parameters[0];
    const double* optical_center = parameters[1];
    const double* focal_length   = parameters[2];
    
    // Current sight vector
    vw::Vector3 sight = m_world_sight_mat[m_row][m_col];
    
    // Find axis angle and then the rotation from the sensor to the world
    vw::Vector3 axis_angle(rotation[0], rotation[1], rotation[2]);
    vw::Matrix3x3 rot = vw::math::axis_angle_to_quaternion(axis_angle).rotation_matrix();
    
    // sight vec in sensor coordinates. Here likely the row index and min_row is 
    // not needed. Those are taken into account when the pixel is created later.
    vw::Vector3 in(m_min_col + m_col * m_d_col - optical_center[0], -optical_center[1], 
                    focal_length[0]);

    // Normalize
    in = in/norm_2(in);
    // rotate to world coordinates
    in = rot*in;

    // The sight vector we try to fit    
    vw::Vector3 out = m_world_sight_mat[m_row][m_col];
    out = out/norm_2(out);
    
    for (size_t p = 0; p < 3; p++)
      residuals[p] = in[p] - out[p];

    return true;
  }

  // Factory to hide the construction of the CostFunction object from
  // the client code.
  static ceres::CostFunction* Create(SightMatT const& world_sight_mat, 
                                     int row, int col, int min_col, int d_col) {
    // The numbers below are for residual, rotation, optical center, focal length
    
    ceres::DynamicNumericDiffCostFunction<SightMatError>* cost_function
     = new ceres::DynamicNumericDiffCostFunction<SightMatError>
            (new SightMatError(world_sight_mat, row, col, min_col, d_col));
    
    cost_function->SetNumResiduals(3);
    cost_function->AddParameterBlock(3); // rotation
    cost_function->AddParameterBlock(2); // optical center
    cost_function->AddParameterBlock(1); // focal length
          
    return cost_function;
  }

  // The sight matrix has samples of directions in the world coordinates 
  SightMatT const& m_world_sight_mat; // alias
  int m_row, m_col;
  int m_min_col, m_d_col; // to go from index in sight mat to image column
};

// See the .h file for the documentation
void fitBestRotationsIntrinsics(SightMatT const& world_sight_mat,
                                vw::Vector2i const& image_size, 
                                int min_col, int d_col,
                                // Outputs
                                double & focal_length, vw::Vector2 & optical_center,
                                std::vector<vw::Matrix<double,3,3>> & rotation_vec) {

  // Wipe the outputs
  rotation_vec.clear();
  
  int num_rows = world_sight_mat.size();
  int num_cols = world_sight_mat[0].size();

  // First and last sight vectors for first row 
  vw::Vector3 beg_v = world_sight_mat[0][0];
  vw::Vector3 end_v = world_sight_mat[0][num_cols-1];
  // Find the angle, in radians, between the two vectors
  double fov = acos(dot_prod(beg_v, end_v)/(norm_2(beg_v)*norm_2(end_v)));
  
  // Find initial focal length based on fov and image width
  focal_length = image_size[0]/(2.0*tan(fov/2.0));
  // Initial optical center (column and row)
  optical_center = vw::Vector2(image_size[0]/2.0, 0);
  
  // Find the initial rotation matrix for each row of world_sight_mat
  // make a vector of matrices, one for each row
  for (int row = 0; row < world_sight_mat.size(); row++) {

    // Find input-output pair correspondences
    std::vector<vw::Vector3> in_vec, out_vec;
    for (int col = 0; col < world_sight_mat[0].size(); col++) {
      vw::Vector3 in(min_col + col * d_col - optical_center[0], -optical_center[1], 
                     focal_length);
      vw::Vector3 out = world_sight_mat[0][col];
      // Normalize and push back
      in = in/norm_2(in);
      out = out/norm_2(out);
      in_vec.push_back(in);
      out_vec.push_back(out);
    }

    // Find the rotation matrix that fits best the input-output pairs    
    std::string transform_type = "rigid";
    vw::Matrix<double,3,3> rotation;
    vw::Vector3 translation;
    double scale = 1.0;
    vw::math::find_3D_transform_no_outliers(in_vec, out_vec, 
                                            rotation, translation, scale,
                                            transform_type);
    
    // append the rotation to the vec
    rotation_vec.push_back(rotation);
  }
  
  // Convert rotations to axis angle format
  std::vector<vw::Vector3> axis_angle_vec(num_rows);
  for (int row = 0; row < num_rows; row++) {
    vw::Quat q(rotation_vec[row]);
    vw::Vector3 axis_angle = q.axis_angle();
    axis_angle_vec[row] = axis_angle;
  }
  
  // Set up an optimization problem. Inputs is world_sight_mat. 
  // Find best-fitting rotation matrices for each row of world_sight_mat,
  // and the overall best focal length and optical center.
  ceres::Problem problem;
  for (int row = 0; row < num_rows; row++) {
    for (int col = 0; col < num_cols; col++) {
      
      ceres::CostFunction* cost_function
        = SightMatError::Create(world_sight_mat, row, col, min_col, d_col);
      // ceres::LossFunction* loss_function = NULL;
      // Prioritize for now the center of the image where the distortion
      // is less.
      ceres::LossFunction* loss_function = new ceres::CauchyLoss(1e-6);
      problem.AddResidualBlock(cost_function, loss_function, 
                                &axis_angle_vec[row][0], &optical_center[0], &focal_length);
    }
  }

  // linescanFitSaveResiduals(problem, "residuals1_before.txt"); // for debugging

  // Solve the problem
  ceres::Solver::Options options;
  options.linear_solver_type = ceres::ITERATIVE_SCHUR;
  options.num_threads = 1; 
  options.max_num_iterations = 50; // 50 iterations is enough
  options.minimizer_progress_to_stdout = false;
  ceres::Solver::Summary summary;
  ceres::Solve(options, &problem, &summary);

  // Copy back from axis_angle_vec to rotation_vec
  for (int row = 0; row < num_rows; row++)
    rotation_vec[row] 
      = vw::math::axis_angle_to_quaternion(axis_angle_vec[row]).rotation_matrix();
  
#if 0 // for debugging
  vw::vw_out() << summary.FullReport() << "\n"; 
  vw::vw_out() << "Starting average reprojection error: "
            << summary.initial_cost << "\n";
  vw::vw_out() << "Final average reprojection error:    "
            << summary.final_cost << "\n";
 #endif
 
  // linescanFitSaveResiduals(problem, "residuals1_after.txt"); // for debugging
 
  return;
}

// The error between sight vectors and a linescan CSM model that incorporates
// distortion. The satellite positions are assumed fixed.
struct SightMatLinescanError {

  SightMatLinescanError(SightMatT     const& world_sight_mat, 
                        asp::CsmModel const& csm_model,
                        int min_col,  int min_row, 
                        int d_col,    int d_row, int num_poses,
                        DistortionType dist_type, int num_dist_params):
  m_world_sight_mat(world_sight_mat), m_csm_model(csm_model),
  m_min_col(min_col), m_min_row(min_row), m_d_col(d_col), m_d_row(d_row), 
  m_num_poses(num_poses), m_dist_type(dist_type), m_num_dist_params(num_dist_params) {}

  // Members
  // The sight matrix has samples of directions in the world coordinates 
  SightMatT     const& m_world_sight_mat; // alias
  asp::CsmModel const& m_csm_model;       // alias
  int m_min_col, m_min_row, m_d_col, m_d_row, m_num_poses;
  DistortionType m_dist_type;
  int m_num_dist_params;

  // Error operator
  bool operator()(double const* const* parameters, double* residuals) const {

   const double* rotations      = parameters[0];
   const double* optical_center = parameters[1];
   const double* focal_length   = parameters[2];
   const double* distortion     = parameters[3];
   
   // Make a local model copy
   asp::CsmModel local_model;
   m_csm_model.deep_copy(local_model);
   
   // Set distortion
   local_model.set_distortion_type(m_dist_type);
   std::vector<double> dist_vec(m_num_dist_params);
   for (int i = 0; i < m_num_dist_params; i++)
     dist_vec[i] = distortion[i];
   local_model.set_distortion(dist_vec); 
   
   // Set focal length and optical center
   local_model.set_focal_length(focal_length[0]); 
   local_model.set_optical_center(vw::Vector2(optical_center[0], optical_center[1]));
   
   // Copy the rotations from axis angle format to quaternions, then set in model
   std::vector<double> quaternions(4*m_num_poses);
   axisAngleToCsmQuatVec(m_num_poses, rotations, quaternions); 
   local_model.set_linescan_quaternions(quaternions);
   
    // Find the residuals with the local model
    int num_rows = m_world_sight_mat.size();
    int num_cols = m_world_sight_mat[0].size();
    int count = 0;
    for (int row = 0; row < num_rows; row++) {
      for (int col = 0; col < num_cols; col++) {
        
        vw::Vector3 dir1 = m_world_sight_mat[row][col];
        dir1 = dir1/norm_2(dir1);

        // Vector in sensor coordinates. Convert to double
        // early on to avoid integer overflow.
        vw::Vector2 pix(double(m_min_col) + double(col) * m_d_col, 
                        double(m_min_row) + double(row) * m_d_row);
        vw::Vector3 dir2 = local_model.pixel_to_vector(pix);
        
        int j = 3 * count;
        for (int k = 0; k < 3; k++)
          residuals[j+k] = dir1[k] - dir2[k];
          
        count++;
      }
    }

    return true;
  }

  // Factory to hide the construction of the CostFunction object from
  // the client code.
  static ceres::CostFunction* Create(SightMatT const& world_sight_mat,
                                     asp::CsmModel const& csm_model,
                                     int min_col, int min_row, 
                                     int d_col, int d_row, int num_poses,
                                     DistortionType dist_type, int num_dist_params) {
    
    ceres::DynamicNumericDiffCostFunction<SightMatLinescanError>* cost_function
     = new ceres::DynamicNumericDiffCostFunction<SightMatLinescanError>
            (new SightMatLinescanError(world_sight_mat, csm_model, 
                                       min_col, min_row, d_col, d_row, 
                                       num_poses, dist_type, num_dist_params));
    
     int num_rows = world_sight_mat.size();
     int num_cols = world_sight_mat[0].size();

     cost_function->AddParameterBlock(3 * num_poses); // rotations
     cost_function->AddParameterBlock(2); // optical center
     cost_function->AddParameterBlock(1); // focal length
     cost_function->AddParameterBlock(num_dist_params); // distortion
     cost_function->SetNumResiduals(3 * num_rows * num_cols);

     return cost_function;
  }

};

// See the .h file for the documentation
void refineCsmLinescanFit(SightMatT const& world_sight_mat,
                          int min_col, int min_row,
                          int d_col, int d_row,
                          // This model will be modified
                          asp::CsmModel & csm_model) {

  // Read data from the model
  double focal_length = csm_model.focal_length();
  vw::Vector2 optical_center = csm_model.optical_center();
  std::vector<double> quaternions = csm_model.linescan_quaternions();

  // Sanity check
  int num_poses = quaternions.size()/4;
  if (num_poses == 0)
    vw::vw_throw(vw::ArgumentErr()
                 << "refineCsmLinescanFit: No poses found.\n");

  // Create rotations from quaternions
  std::vector<double> rotations;
  csmQuatVecToAxisAngle(num_poses, &quaternions[0], rotations);
  
  // Initial Kaguya distortion. These initial values were shown to work well.
  DistortionType dist_type = KAGUYALISM;
  std::vector<double> distortion = {1e-7, 1e-7, 1e-8, 1e-8, 1e-9, 
                                    1e-7, 1e-7, 1e-8, 1e-8, 1e-9};
  
  // Set up an optimization problem to refine the CSM model.
  ceres::Problem problem;
  ceres::CostFunction* cost_function
    = SightMatLinescanError::Create(world_sight_mat, csm_model, 
                                    min_col, min_row, d_col, d_row, 
                                    num_poses, dist_type, distortion.size());
  
  // Minimize all residuals equally
  ceres::LossFunction* loss_function = NULL;
  problem.AddResidualBlock(cost_function, loss_function, 
                           &rotations[0], &optical_center[0], &focal_length,
                           &distortion[0]);

  // linescanFitSaveResiduals(problem, "residuals2_before.txt"); // for debugging
   
  // Set up the solver options 
  ceres::Solver::Options options;
  options.linear_solver_type = ceres::ITERATIVE_SCHUR;
  options.num_threads = 1; // Use one thread for unique solution
  options.max_num_iterations = 50; // 50 iterations is enough
  options.gradient_tolerance  = 1e-16;
  options.function_tolerance  = 1e-16;
  options.parameter_tolerance = 1e-12; // should be enough
  options.minimizer_progress_to_stdout = false;

  // Solve the problem
  ceres::Solver::Summary summary;
  ceres::Solve(options, &problem, &summary);

  // linescanFitSaveResiduals(problem, "residuals2_after.txt"); // for debugging

  // Copy back rotations vec to quaternions
  axisAngleToCsmQuatVec(rotations.size()/3, &rotations[0], quaternions); 
  
  // Update the model quaternions, focal length, optical center, and distortion
  csm_model.set_linescan_quaternions(quaternions);
  csm_model.set_focal_length(focal_length);
  csm_model.set_optical_center(optical_center);
  csm_model.set_distortion_type(dist_type);
  csm_model.set_distortion(distortion);

  return;
}

// Fit a CSM sensor with distortion to given tabulated sight directions. This
// assumes the scan lines are horizontal, so the satellite moves vertically.
// There are as many rows in world_sight_mat as there are satellite positions.
// At each position there several sight vectors, along the scanline.
// We want: world_sight_mat[row][col] = csm_model.pixel_to_vector(col, row).
void fitCsmLinescan(std::string const& sensor_id, 
                    vw::cartography::Datum const& datum,
                    vw::Vector2i const& image_size,
                    std::vector<vw::Vector3> const& sat_pos,
                    SightMatT const& world_sight_mat,
                    int min_col, int min_row,
                    int d_col, int d_row, 
                    bool fit_distortion,
                    // This model will be modified
                    asp::CsmModel & csm_model) {

  // Sanity check
  if (sat_pos.size() != world_sight_mat.size())
    vw::vw_throw(vw::ArgumentErr()
                 << "fitCsmLinescan: The number of satellite positions does not "
                 << "agree with the number of rows in the sight matrix.\n");
  
  // Sanity check
  int last_sampled_row = min_row + d_row * (world_sight_mat.size() - 1);
  if (last_sampled_row < image_size[1] - 1)  
    vw::vw_out(vw::WarningMessage) 
      << "Warning: The last image row with a sight matrix sample is " 
      << last_sampled_row << ", which is less than the last image pixel line of " 
      << image_size[1] - 1 << ". This may lead to problems with the best-fit CSM model.\n";
                 
  // Find the rotation matrices, focal length, and optical center,
  // that best fit a 2D matrix of sight vectors. For now, assume
  // no distortion.
  double focal_length = 0.0; 
  vw::Vector2 optical_center;
  std::vector<vw::Matrix<double,3,3>> cam2world_vec;
  fitBestRotationsIntrinsics(world_sight_mat, image_size, min_col, d_col,
                             focal_length, optical_center, cam2world_vec);
  
  // Assume it takes one unit of time to scan one image line
  double first_line_time = 0; // time of the first scanned line
  double last_line_time = image_size[1] - 1;
  double dt_line = (last_line_time - first_line_time) / (image_size[1] - 1.0);
  
  // The image line for the first pose determines its time. The spacing
  // between poses determines dt_ephem. Note that min_row is negative,
  // because the first pose is measured before data starts being recorded.
  double t0_ephem = min_row * dt_line;
  double dt_ephem = d_row * dt_line;
  double t0_quat = t0_ephem;
  double dt_quat = dt_ephem;
  
  // We have no velocities in this context, so set them to 0
  std::vector<vw::Vector3> velocities(sat_pos.size(), vw::Vector3(0, 0, 0));

  // Initialize the CSM model
  asp::populateCsmLinescan(first_line_time, dt_line, 
                           t0_ephem, dt_ephem, t0_quat, dt_quat,
                           focal_length, optical_center, 
                           image_size, datum, sensor_id,
                           sat_pos, velocities, cam2world_vec, 
                           csm_model); // output

  // Refine the CSM model by also floating the distortion 
  if (fit_distortion)
    asp::refineCsmLinescanFit(world_sight_mat, min_col, min_row, d_col, d_row, csm_model);

  return;
}

// Create pixel samples. Make sure to sample the pixel at (width - 1, height - 1).
void createPixelSamples(int width, int height, int num_pixel_samples,
                        std::vector<vw::Vector2> & pix_samples) {
  
  // Sanity checks
  if (num_pixel_samples <= 0)
    vw::vw_throw(vw::ArgumentErr() << "The number of pixel samples must be positive.\n");
  if (width <= 1 || height <= 1)
    vw::vw_throw(vw::ArgumentErr() << "The image dimensions must be at least 2 pixels.\n");
  
  // Wipe the output
  pix_samples.clear();

  // Find how many samples we need for width and height
  double area = double(width) * double(height);  // avoid int32 overflow
  double len = sqrt(area / double(num_pixel_samples));
  int num_x = std::max(round((width - 1.0) / len) + 1, 2.0);
  int num_y = std::max(round((height - 1.0) / len) + 1, 2.0);
  
  // Take creat care to include the last pixel, which is (width - 1, height -
  // 1).
  double x_step = (width - 1.0) / double(num_x - 1);
  double y_step = (height - 1.0) / double(num_y - 1);
  
  // iterate with num_x samples with spacing x_step, and the same for y
  for (int ix = 0; ix < num_x; ix++) {
    for (int iy = 0; iy < num_y; iy++) {
      double x = ix * x_step;
      double y = iy * y_step;
      x = std::min(x, double(width - 1));
      y = std::min(y, double(height - 1));
      vw::Vector2 pix(x, y);
      pix_samples.push_back(pix);
    }
  }
  
  return;
}

void parseRefineIntrinsicsStr(std::string const& refine_intrinsics,
                              bool & fix_focal_length, 
                              bool & fix_optical_center, 
                              bool & fix_other_intrinsics) {
  
  // Initialize all to true
  fix_focal_length = true;
  fix_optical_center = true;
  fix_other_intrinsics = true;
  
  // Make a local copy of the string and convert it to lower case
  std::string local_refine_intrinsics = refine_intrinsics;
  boost::to_lower(local_refine_intrinsics);
  
  // Ensure that this was set to something, rather than "", which may be ambiguous.
  if (local_refine_intrinsics == "") 
    vw::vw_throw(vw::ArgumentErr() << "Error: refine intrinsics string is empty.\n");
    
  // Replace none with "".
  if (local_refine_intrinsics == "none") 
    local_refine_intrinsics = "";
   
  // Replace "all" with all
  if (local_refine_intrinsics == "all") 
   local_refine_intrinsics = "focal_length, optical_center, other_intrinsics";   
     
  // Replace commas with spaces
  boost::replace_all(local_refine_intrinsics, ",", " ");
  
  std::istringstream is(local_refine_intrinsics);
  std::string val;
  while (is >> val) {
    if (val == "focal_length")
      fix_focal_length = false;
    else if (val == "optical_center")
      fix_optical_center = false;
    else if (val == "other_intrinsics" || val == "distortion")
      fix_other_intrinsics = false;
    else
      vw::vw_throw(vw::ArgumentErr() << "Error: Found unknown intrinsic to float: " 
        << val << ".\n");
  }
  
  vw::vw_out() << "Refine focal length: " << !fix_focal_length << "\n";
  vw::vw_out() << "Refine optical center: " << !fix_optical_center << "\n";
  vw::vw_out() << "Refine other intrinsics (distortion): " << !fix_other_intrinsics << "\n";
}

// The error between projections of ground points in camera and known pixels.
// Optimize the intrinsics, including distortion. The satellite positions and
// orientations are optimized as well.
struct FrameCamReprojErr {

  FrameCamReprojErr(std::vector<vw::Vector2> const& pixels,
                   std::vector<vw::Vector3> const& xyz,
                   int num_distortion_params,
                   asp::CsmModel const& csm_model):
  m_pixels(pixels), m_xyz(xyz), 
  m_num_dist_params(num_distortion_params), m_csm_model(csm_model) {
    
    // There must be as many pixels as xyz
    if (m_pixels.size() != m_xyz.size())
      vw::vw_throw(vw::ArgumentErr() 
                   << "Error: The number of pixels and ground points must be same.\n");
  }

  // Members
  std::vector<vw::Vector2> const& m_pixels; // alias
  std::vector<vw::Vector3> const& m_xyz; // alias
  int m_num_dist_params;
  asp::CsmModel const& m_csm_model;       // alias

  // Error operator
  bool operator()(double const* const* parameters, double* residuals) const {

   const double* position      = parameters[0];
   const double* rotation      = parameters[1];
   const double* optical_center = parameters[2];
   const double* focal_length   = parameters[3];
   const double* distortion     = parameters[4];
   
   // Make a local model copy
   asp::CsmModel local_model;
   m_csm_model.deep_copy(local_model);
   
   // Set the position
   local_model.set_frame_position(position[0], position[1], position[2]);
       
   // Copy the rotation from axis angle format to quaternions, then set in model
   int num_poses = 1;
   std::vector<double> q(4*num_poses);
   axisAngleToCsmQuatVec(num_poses, rotation, q);
   local_model.set_frame_quaternion(q[0], q[1], q[2], q[3]);
  
   // Set optical center and focal length
   local_model.set_optical_center(vw::Vector2(optical_center[0], optical_center[1]));
   local_model.set_focal_length(focal_length[0]); 

   // Set distortion
   std::vector<double> dist_vec(m_num_dist_params);
   for (int i = 0; i < m_num_dist_params; i++)
     dist_vec[i] = distortion[i];
   local_model.set_distortion(dist_vec); 

    // Find the residuals with the local model
    int num_samples = m_pixels.size();
    for (int count = 0; count < num_samples; count++) {
        
      vw::Vector2 pix1 = m_pixels[count];
      vw::Vector2 pix2 = local_model.point_to_pixel(m_xyz[count]);
      
      int j = 2 * count;
      for (int k = 0; k < 2; k++)
        residuals[j+k] = pix1[k] - pix2[k];
    }

    return true;
  }

  // Factory to hide the construction of the CostFunction object from
  // the client code.
  static ceres::CostFunction* Create(std::vector<vw::Vector2> const& pixels,
                                     std::vector<vw::Vector3> const& xyz,
                                     int num_distortion_params,
                                     asp::CsmModel const& csm_model) {

    ceres::DynamicNumericDiffCostFunction<FrameCamReprojErr>* cost_function
     = new ceres::DynamicNumericDiffCostFunction<FrameCamReprojErr>
            (new FrameCamReprojErr(pixels, xyz, num_distortion_params, csm_model));

     cost_function->AddParameterBlock(3); // position
     cost_function->AddParameterBlock(3); // rotation (axis angle)
     cost_function->AddParameterBlock(2); // optical center
     cost_function->AddParameterBlock(1); // focal length
     cost_function->AddParameterBlock(num_distortion_params); // distortion
     cost_function->SetNumResiduals(2 * pixels.size()); // 2 residuals per pixel

     return cost_function;
  }

};

// See how well the optimized model fits the ground points
void computeCamStats(std::vector<vw::Vector2> const& pixels,
                     std::vector<vw::Vector3> const& xyz,
                     asp::CsmModel const& csm_model) {

  // Iterate over xyz and project into the camera
  std::vector<double> errors;
  for (size_t i = 0; i < xyz.size(); i++) {
    vw::Vector2 pix = csm_model.point_to_pixel(xyz[i]);
    errors.push_back(norm_2(pix - pixels[i]));
  }

  // Sort the errors
  std::sort(errors.begin(), errors.end());
  
  double mean = vw::math::mean(errors);
  double stdev = vw::math::standard_deviation(errors, mean);
  
  vw::vw_out() << "Errors of pixel projection in the camera with refined intrinsics:\n";
  vw::vw_out() << "Min:    " << errors[0] << "\n";
  vw::vw_out() << "Median: " << vw::math::destructive_median(errors) << "\n";
  vw::vw_out() << "Mean:   " << mean << "\n";
  vw::vw_out() << "StDev:  " << stdev << "\n";
  vw::vw_out() << "Max:    " << errors.back() << "\n";
  vw::vw_out() << "Num samples: " << errors.size() << "\n";

}
                                                   
// Refine a CSM frame camera model using a a set of ground points projecting at given pixels
void refineCsmFrameFit(std::vector<vw::Vector2> const& pixels,
                       std::vector<vw::Vector3> const& xyz,
                       std::string const& refine_intrinsics,
                       asp::CsmModel & csm_model) { // output

  vw::vw_out() << "Refining camera intrinsics and pose.\n";

  // See which intrinsics to fix
  bool fix_focal_length = true, fix_optical_center = true, fix_other_intrinsics = true;
  parseRefineIntrinsicsStr(refine_intrinsics, 
                           fix_focal_length, fix_optical_center, fix_other_intrinsics);

  // Read data from the model
  double x, y, z;
  csm_model.frame_position(x, y, z);
  double qx, qy, qz, qw;
  csm_model.frame_quaternion(qx, qy, qz, qw);
  double focal_length = csm_model.focal_length();
  vw::Vector2 optical_center = csm_model.optical_center();
  std::vector<double> distortion = csm_model.distortion();
  
#if 0 // This does not work and may not be needed
  // Find the longest distance from optical center to each pixel
  if (!fix_other_intrinsics) {
    double r = 0;
    for (size_t i = 0; i < pixels.size(); i++) 
      r = std::max(r, norm_2(pixels[i] - optical_center));
    if (r == 0)
      vw::vw_throw(vw::ArgumentErr() << "Error: Not enough samples.\n");
  
    // Give CERES a hint to not perturb the high order distortion terms too much
    // TODO(oalexan1): This should depend on image dimensions.
    // k2 gets multiplied by r^5, k3 gets multiplied by r^7. 
    if (distortion[1] == 0)
      distortion[1] = 1e-7 / pow(r, 5); // k2
    if (distortion[4] == 0)
      distortion[4] = 1e-7 / pow(r, 7);  // k3
  }
#endif
      
  // The position vector
  std::vector<double> position = {x, y, z};
      
  // Create rotation from quaternions
  std::vector<double> rotation;
  int num_poses = 1;
  std::vector<double> q = {qx, qy, qz, qw};
  csmQuatVecToAxisAngle(num_poses, &q[0], rotation);

  // Set up an optimization problem to refine the CSM model.
  ceres::Problem problem;
  ceres::CostFunction* cost_function
    = FrameCamReprojErr::Create(pixels, xyz, distortion.size(), csm_model);
  
  // Minimize all residuals equally
  ceres::LossFunction* loss_function = NULL;
  problem.AddResidualBlock(cost_function, loss_function, 
                           &position[0], &rotation[0], &optical_center[0], &focal_length,
                           &distortion[0]);

  // Set up the solver options 
  ceres::Solver::Options options;
  options.linear_solver_type = ceres::ITERATIVE_SCHUR;
  options.num_threads = 1; // Use one thread for unique solution
  options.max_num_iterations = 50; // 50 iterations is enough
  
  options.gradient_tolerance  = 1e-16;
  options.function_tolerance  = 1e-16;
  options.parameter_tolerance = 1e-12; // should be enough
  options.minimizer_progress_to_stdout = false; // true for debugging

  if (fix_focal_length)
    problem.SetParameterBlockConstant(&focal_length);
  if (fix_optical_center)
    problem.SetParameterBlockConstant(&optical_center[0]);
  if (fix_other_intrinsics)
    problem.SetParameterBlockConstant(&distortion[0]);
   
  // Solve the problem  
  ceres::Solver::Summary summary;
  ceres::Solve(options, &problem, &summary);
  //vw::vw_out() << summary.FullReport() << "\n";

  // Copy back
  csm_model.set_frame_position(position[0], position[1], position[2]);
  axisAngleToCsmQuatVec(num_poses, &rotation[0], q);
  csm_model.set_frame_quaternion(q[0], q[1], q[2], q[3]);
  csm_model.set_focal_length(focal_length);
  csm_model.set_optical_center(optical_center);
  csm_model.set_distortion(distortion);

  computeCamStats(pixels, xyz, csm_model);
  return;
}

// Fit a CSM camera to an optical bar camera. Optical bar cameras assumes the
// scan lines are vertical, as for KH-9, which differs with how a CSM model is
// fit, so below we will do some conversions. Additionally, the image file
// needs to be rotated 90 degrees counter-clockwise to match the CSM camera.
void fitCsmLinescanToOpticalBar(std::string const& camFile, 
                                vw::Vector2i const& OpticalBarimageColsRows,
                                vw::cartography::Datum const& datum,
                                asp::CsmModel & csm) {
  
  // See above for why OpticalBar rows become CSM columns, and vise versa.
  int num_csm_rows = OpticalBarimageColsRows[0];
  int num_csm_cols = OpticalBarimageColsRows[1];
  
  // This should be fine enough
  int num_lines_per_pose = 100;
  
  // Use at least 1000 samples, but no more than 10000.
  int num_row_samples = round(double(num_csm_rows) / double(num_lines_per_pose)) + 1;
  num_row_samples = std::max(1000, std::min(num_row_samples, 10000));
  
  // Spacing between row samples
  int d_row = round(double(num_csm_rows) / double(num_row_samples - 1));
  d_row = std::max(d_row, 1); // at least one pixel per sample
  
  // Add at least 10 row samples before first line and after last line, which
  // help with pose interpolation when solving for jitter.
  int min_row = -10 * d_row; // start at least 10 pixels before the first line
  int max_row = num_csm_rows + 10 * d_row; // end at least 10 pixels after the last line
  num_row_samples = ceil(double(max_row - min_row) / double(d_row)) + 1;
  
  // Along each row, likely 20 column samples should be enough, as those will result
  // in a single rotation matrix.
  int num_col_samples = 20;
  int d_col = ceil(double(num_csm_cols) / double(num_col_samples - 1));
  
  // max_col should be at least num_csm_cols - 1 to sample fully the last column.
  // Otherwise throw a warning.
  int min_col = 0; // start at the first column
  int max_col = min_col + (num_col_samples - 1) * d_col;
  if (max_col < num_csm_cols - 1)
    vw::vw_out(vw::WarningMessage)
      << "Warning: The last column with a sight matrix sample is " 
      << max_col << ", which is less than the last image pixel column of " 
      << num_csm_cols - 1 << ". This may lead to problems with the best-fit CSM model.\n";
  
  // Open the Optical bar file
  vw::camera::OpticalBarModel optical_bar_cam(camFile);
  vw::Vector2 optical_center = optical_bar_cam.get_optical_center();
  vw::Vector2 image_size = optical_bar_cam.get_image_size();

  // Image size must be what was set earlier
  if (image_size != OpticalBarimageColsRows)
    vw::vw_throw(vw::ArgumentErr() 
      << "Error: The image size set in the optical bar camera model does not match "
      << "the actual image size.\n");
  
  // Sample the camera poses along the track
  std::vector<vw::Vector3> sat_pos(num_row_samples);
  for (int i = 0; i < num_row_samples; i++) {
    double csm_row = min_row + i * d_row;
    // Convert to optical bar pixels. The optical bar column is csm row.
    // The optical bar row is the row coordinate of the optical center.
    vw::Vector2 optical_bar_pix(csm_row, optical_center[1]);
    sat_pos[i] = optical_bar_cam.camera_center(optical_bar_pix);
  }
  
  // Create the world sight matrix (camera directions in world coordinates)
  SightMatT world_sight_mat(num_row_samples, std::vector<vw::Vector3>(num_col_samples));
  for (int irow = 0; irow < num_row_samples; irow++) {
    for (int icol = 0; icol < num_col_samples; icol++) {
      double csm_col = min_col + icol * d_col;
      double csm_row = min_row + irow * d_row;
      // In-sensor rotation that interchanges rows and cols
      vw::Vector2 optical_bar_pix(csm_row, num_csm_cols - 1 - csm_col);
      world_sight_mat[irow][icol] = optical_bar_cam.pixel_to_vector(optical_bar_pix);
    }
  }

  // Fit the CSM model
  std::string sensor_id = "OpticalBar"; 
  bool fit_distortion = false;
  fitCsmLinescan(sensor_id, datum, vw::Vector2(num_csm_cols, num_csm_rows),
                 sat_pos, world_sight_mat, min_col, min_row, d_col, d_row, 
                 fit_distortion, csm);
    
}

} // end namespace asp
