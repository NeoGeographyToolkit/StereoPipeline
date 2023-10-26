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

#include <asp/Core/Common.h>
#include <asp/Camera/LinescanFit.h>
#include <asp/Camera/CsmUtils.h>

#include <vw/Math/Quaternion.h>
#include <vw/Math/Geometry.h>

#include <asp/Camera/CsmModel.h>

#include <ceres/ceres.h>
#include <ceres/loss_function.h>

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
struct SightVecError {

  typedef std::vector<std::vector<vw::Vector3>> SightVecT;
  
  SightVecError(SightVecT const& world_sight_mat, int row, int col, int d_col):
  m_world_sight_mat(world_sight_mat), m_row(row), m_col(col), m_d_col(d_col) {}

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
    
    // sight vec in sensor coordinates
    vw::Vector3 in(m_col * m_d_col - optical_center[0], -optical_center[1], 
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
  static ceres::CostFunction* Create(SightVecT const& world_sight_mat, 
                                     int row, int col, int d_col) {
    // The numbers below are for residual, rotation, optical center, focal length
    
    ceres::DynamicNumericDiffCostFunction<SightVecError>* cost_function
     = new ceres::DynamicNumericDiffCostFunction<SightVecError>
            (new SightVecError(world_sight_mat, row, col, d_col));
    
    cost_function->SetNumResiduals(3);
    cost_function->AddParameterBlock(3); // rotation
    cost_function->AddParameterBlock(2); // optical center
    cost_function->AddParameterBlock(1); // focal length
          
    return cost_function;
  }

  // The sight matrix has samples of directions in the world coordinates 
  SightVecT const& m_world_sight_mat; // alias
  int m_row, m_col;
  int m_d_col; // multiply col of sight mat by this to get the image column
};

// See the .h file for the documentation
void fitBestRotationsIntrinsics(
    std::vector<std::vector<vw::Vector3>> const& world_sight_mat,
    vw::Vector2i const& image_size, int d_col,
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
  // iterate throw rows, print each row
  // make a vector of matrices, one for each row
  for (int row = 0; row < world_sight_mat.size(); row++) {

    // Find input-output pair correspondences
    std::vector<vw::Vector3> in_vec, out_vec;
    for (int col = 0; col < world_sight_mat[0].size(); col++) {
      vw::Vector3 in(col * d_col - optical_center[0], -optical_center[1], focal_length);
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
        = SightVecError::Create(world_sight_mat, row, col, d_col);
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
struct SightVecLinescanError {

  typedef std::vector<std::vector<vw::Vector3>> SightVecT;
  
  SightVecLinescanError(SightVecT     const& world_sight_mat, 
                        asp::CsmModel const& csm_model,
                        int min_col,  int min_row, 
                        int d_col,    int d_row, int num_poses,
                        DistortionType dist_type, int num_dist_params):
  m_world_sight_mat(world_sight_mat), m_csm_model(csm_model),
  m_min_col(min_col), m_min_row(min_row), m_d_col(d_col), m_d_row(d_row), 
  m_num_poses(num_poses), m_dist_type(dist_type), m_num_dist_params(num_dist_params) {
    
    // This code was not tested with m_min_col not zero
    if (m_min_col != 0)
      vw::vw_throw(vw::ArgumentErr()
                    << "SightVecLinescanError: m_min_col must be zero.\n");
  }

  // Members
  // The sight matrix has samples of directions in the world coordinates 
  SightVecT     const& m_world_sight_mat; // alias
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
        vw::Vector2 pix(double(col) * m_d_col + double(m_min_col), 
                        double(row) * m_d_row + double(m_min_row));
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
  static ceres::CostFunction* Create(SightVecT const& world_sight_mat,
                                     asp::CsmModel const& csm_model,
                                     int min_col, int min_row, 
                                     int d_col, int d_row, int num_poses,
                                     DistortionType dist_type, int num_dist_params) {
    
    ceres::DynamicNumericDiffCostFunction<SightVecLinescanError>* cost_function
     = new ceres::DynamicNumericDiffCostFunction<SightVecLinescanError>
            (new SightVecLinescanError(world_sight_mat, csm_model, 
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
void refineCsmFit(
       std::vector<std::vector<vw::Vector3>> const& world_sight_mat,
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
                 << "refineCsmFit: No poses found.\n");

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
    = SightVecLinescanError::Create(world_sight_mat, csm_model, 
                                    min_col, min_row, d_col, d_row, 
                                    num_poses, dist_type, distortion.size());
  
  // Minimize all residuals equally
  ceres::LossFunction* loss_function = NULL;
  problem.AddResidualBlock(cost_function, loss_function, 
                           &rotations[0], &optical_center[0], &focal_length,
                           &distortion[0]);

  // linescanFitSaveResiduals(problem, "residuals2_before.txt"); // for debugging

  // Solve the problem
  ceres::Solver::Options options;
  options.linear_solver_type = ceres::ITERATIVE_SCHUR;
  options.num_threads = 1; // Use one thread for unique solution
  options.max_num_iterations = 50; // 50 iterations is enough
  options.minimizer_progress_to_stdout = false;
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

// Fit a CSM sensor with distortion to given tabulated sight directions
void fitCsmModel(
       std::string const& sensor_id, 
       vw::cartography::Datum const& datum,
       vw::Vector2i const& image_size,
       std::vector<vw::Vector3> const& sat_pos,
       std::vector<std::vector<vw::Vector3>> const& world_sight_mat,
       int min_col, int min_row,
       int d_col, int d_row, 
       // This model will be modified
       asp::CsmModel & csm_model) {

  // Find the rotation matrices, focal length, and optical center,
  // that best fit a 2D matrix of sight vectors. For now, assume
  // no distortion.
  double focal_length = 0.0; 
  vw::Vector2 optical_center;
  std::vector<vw::Matrix<double,3,3>> cam2world_vec;
  fitBestRotationsIntrinsics(world_sight_mat, image_size, d_col,
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
  asp::refineCsmFit(world_sight_mat, min_col, min_row, d_col, d_row, csm_model);

  return;
}

} // end namespace asp

