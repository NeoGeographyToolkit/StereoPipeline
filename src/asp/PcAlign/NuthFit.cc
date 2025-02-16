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

#include <asp/PcAlign/NuthFit.h>

#include <ceres/ceres.h>

/// Best fit function for Nuth alignment with Ceres

namespace asp {

// Function for fitting Nuth and Kaab (2011)
// Can use phasor addition, but need to change conversion to offset dx and dy
// https://stackoverflow.com/questions/12397412/i-know-scipy-curve-fit-can-do-better?rq=1
inline double nuth_func(double x, double a, double b, double c) {
  return a * cos((b-x) * M_PI / 180.0) + c;
}

// Cost functor for Nuth and Kaab function
struct NuthResidual {

  NuthResidual(double x, double y): m_x(x), m_y(y) {}

  bool operator()(double const * const * params, double * residuals) const {
    // params[0][0] = a, params[0][1] = b, params[0][2] = c
    double predicted = nuth_func(m_x, params[0][0], params[0][1], params[0][2]);
    residuals[0] = predicted - m_y;
    return true;
  }

  // Factory to hide the construction of the CostFunction object from the client code.
  static ceres::CostFunction* CreateNuthCostFunction(double x, double y) {
    
    ceres::DynamicNumericDiffCostFunction<NuthResidual>* cost_function
      = new ceres::DynamicNumericDiffCostFunction<NuthResidual>(new NuthResidual(x, y));
      
    // The residual size is 1
    cost_function->SetNumResiduals(1);
    
    // Add a parameter block for each parameter
    cost_function->AddParameterBlock(3);
    
    return cost_function;
  }

  double m_x, m_y;
}; // End class NuthResidual

// Form a Ceres optimization problem. Will fit a curve to bin centers and bin
// medians, while taking into acount the bin count and min_bin_sample_count.
void nuthFit(std::vector<double> const& bin_count, 
             std::vector<double> const& bin_centers, 
             std::vector<double> const& bin_median, 
             int inner_iter,
             int num_threads,
             vw::Vector3 & fit_params) {

  ceres::Problem problem;
  
  int min_bin_sample_count = 9;
  int numBins = bin_centers.size();
  
  // Add residuals
  for (int i = 0; i < numBins; i++) {
    
     // Skip bin count less than min_bin_sample_count
     if (bin_count[i] < min_bin_sample_count)
       continue;
        
    // The residual is the difference between the data and the model
    // fit = optimization.curve_fit(nuth_func, bin_centers, bin_med, fit_params)[0]
    ceres::CostFunction* cost_function = 
      NuthResidual::CreateNuthCostFunction(bin_centers[i], bin_median[i]);
    
    // A lot of outlier removal took place. Likely there is no need for 
    // a robust loss function.
    ceres::LossFunction* loss_function = NULL;
    
    problem.AddResidualBlock(cost_function, loss_function, &fit_params[0]);
  }
  
  ceres::Solver::Options options;
  options.gradient_tolerance  = 1e-16;
  options.function_tolerance  = 1e-16;
  options.parameter_tolerance = 1e-12;
  options.max_num_iterations  = inner_iter;
  options.max_num_consecutive_invalid_steps = std::max(5, inner_iter/5); // try hard
  options.minimizer_progress_to_stdout = false; // verbose
  options.num_threads = num_threads;
  options.linear_solver_type = ceres::SPARSE_SCHUR;
  
  ceres::Solver::Summary summary;
  ceres::Solve(options, &problem, &summary);
  // vw::vw_out() << summary.FullReport() << "\n"; // verbose
  
} // End function nuthFit
    
} // end namespace asp
