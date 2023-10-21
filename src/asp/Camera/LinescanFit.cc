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

#include <vw/Math/Quaternion.h>
#include <vw/Math/Geometry.h>

#include <ceres/ceres.h>
#include <ceres/loss_function.h>

namespace asp {

// Find the rotation matrices, focal length, and optical center,
// that best fit a 2D matrix of sight vectors. Used for ASTER.

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
  
  // Create a vector having the axis-angle representation for every rotation
  std::vector<vw::Vector3> axis_angle_vec(num_rows);
  // iterate over row and initialize axis_angle_vec
  for (int row = 0; row < num_rows; row++) {
    // Convert rotation matrix to axis-angle
    // for that, first convert to quaternion
    vw::Quat q(rotation_vec[row]);
    vw::Vector3 axis_angle = q.axis_angle();
    //std::cout << "axis angle is " << axis_angle << std::endl;
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
      ceres::LossFunction* loss_function = NULL;
      problem.AddResidualBlock(cost_function, loss_function, 
                                &axis_angle_vec[row][0], &optical_center[0], &focal_length);
    }
  }

  // Solve the problem
  ceres::Solver::Options options;
  options.linear_solver_type = ceres::ITERATIVE_SCHUR;
  options.num_threads = 1; 
  options.max_num_iterations = 50; // 50 iterations is enough
  options.minimizer_progress_to_stdout = true;
  ceres::Solver::Summary summary;
  ceres::Solve(options, &problem, &summary);
  
  std::cout << summary.FullReport() << "\n";
  std::cout << "Starting average reprojection error: "
            << summary.initial_cost << "\n";
  std::cout << "Final average reprojection error:    "
            << summary.final_cost << "\n";
  
  // Print final optical center
  std::cout << "final optical center is " << optical_center << std::endl;
  // print final focal length
  std::cout << "final focal length is " << focal_length << std::endl;

  // TODO(oalexan1): Turn this off
#if 1
  // Evaluate the final residuals. For debugging.  
  double total_cost = 0.0;
  ceres::Problem::EvaluateOptions eval_options;
  eval_options.num_threads = 1; // Use one thread to ensure a unique solution
  eval_options.apply_loss_function = false;  // want raw residuals
  std::vector<double> residuals;
  problem.Evaluate(eval_options, &total_cost, &residuals, NULL, NULL);

  // Save the residuals
  std::string resFile = "residual_norms.txt";
  std::cout << "Writing residual norms to: " << resFile << std::endl;
  std::ofstream ofs(resFile.c_str());
  for (int i = 0; i < residuals.size()/3; i++) {
    int j = 3*i;
    ofs << norm_2(vw::Vector3(residuals[j], residuals[j+1], residuals[j+2])) << std::endl;
  }
  ofs.close();
#endif

  // Copy back from axis_angle_vec to rotation_vec
  for (int row = 0; row < num_rows; row++)
    rotation_vec[row] 
    = vw::math::axis_angle_to_quaternion(axis_angle_vec[row]).rotation_matrix();

  // TODO(oalexan1): Check that this has same info as the residuals
  // saved to disk, then wipe this
  
  // Try to see if the sight vectors agree with the rotation matrices
  for (int row = 0; row < num_rows; row++) {
    for (int col = 0; col < num_cols; col++) {
    
      // Vector in sensor coordinates 
      vw::Vector3 in(col * d_col - optical_center[0], -optical_center[1], focal_length);
      in = in/norm_2(in);
      // Rotate to world coordinates
      in = rotation_vec[row] * in;
      
      vw::Vector3 out = world_sight_mat[row][col];
      out = out/norm_2(out);
      std::cout << "norm of diff is " << norm_2(in-out) << std::endl;
    }
  }  

  return;
}

} // end namespace asp

