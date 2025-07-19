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

/// \file ProjectiveCamApprox.cc

// Approximate a camera model with a projective transform

#include <asp/Core/ProjectiveCamApprox.h>

namespace asp {

// Compute the best-fitting projective transform that maps a set of 3D ground points
// to 2D image points. See also: applyProjTrans().
void calcProjTrans(std::vector<vw::Vector2> const& imagePts,
                   std::vector<vw::Vector3> const& groundPts,
                   std::vector<double> & transformCoeffs) {
  
  if (imagePts.size() != groundPts.size()) 
    vw::vw_throw(vw::ArgumentErr() 
       << "The number of inputs and outputs must agree.\n");
  
  int numPts = imagePts.size();
  if (numPts < 8)
    vw::vw_throw(vw::ArgumentErr() 
      << "At least 8 points are needed to fit a 3D-to-2D projective transform.\n");

  int numMatRows = 2 * numPts; // there exist x and y coords for each point
  int numMatCols = 14; // Number of variables in the projective transform
  
  Eigen::MatrixXd M = Eigen::MatrixXd::Zero(numMatRows, numMatCols);
  Eigen::VectorXd b = Eigen::VectorXd::Zero(numMatRows);
  
  for (int it = 0; it < numPts; it++) {

    double x = groundPts[it].x(), y = groundPts[it].y(), z = groundPts[it].z();
    double col = imagePts[it].x(), row = imagePts[it].y();

    // If the solution coefficients are u0, u1, ..., must have:
 
    // (u0 + u1 * x + u2 * y + u3  * z) / (1 + u4  * x + u5  * y + u6  * z) = col
    // (u7 + u8 * x + u9 * y + u10 * z) / (1 + u11 * x + u12 * y + u13 * z) = row

    // Multiply by the denominators. Get a linear regression in the coefficients.
    
    M.row(2 * it + 0) << 1, x, y, z, -x * col, -y * col, -z * col, 0, 0, 0, 0, 0, 0, 0;
    M.row(2 * it + 1) << 0, 0, 0, 0, 0, 0, 0, 1, x, y, z, -x * row, -y * row, -z * row;

    b[2 * it + 0] = col;
    b[2 * it + 1] = row;
  }

  // Solve the over-determined system, per:
  // https://eigen.tuxfamily.org/dox/group__LeastSquares.html
  Eigen::VectorXd u = M.colPivHouseholderQr().solve(b);

  // Copy back the result to a standard vector (to avoid using Eigen too much as
  // that is slow to compile).
  transformCoeffs.resize(numMatCols);
  for (int it = 0; it < numMatCols; it++)
    transformCoeffs[it] = u[it];
  return;
} 

// Apply a projective transform to a ground point and compute the image pixel
vw::Vector2 applyProjTrans(vw::Vector3 const& xyz,
                           std::vector<double> const& transformCoeffs) {

  std::vector<double> const& u = transformCoeffs; // alias
  
  // See the projective transform formula in calcProjTrans().
  double x = xyz.x(), y = xyz.y(), z = xyz.z();
  double col_den = 1 + u[4]  * x + u[5]  * y + u[6]  * z;
  double row_den = 1 + u[11] * x + u[12] * y + u[13] * z;
  
  // It is quite unusual for the denominator to be zero, as the ground point is
  // normally in front of the camera. In that case better return something large
  // rather than Inf.
  if (col_den == 0)
    col_den = 1e-8;
  if (row_den == 0)
    row_den = 1e-8;
    
  double col = (u[0] + u[1] * x + u[2] * y + u[3]  * z) / col_den;
  double row = (u[7] + u[8] * x + u[9] * y + u[10] * z) / row_den;

  return vw::Vector2(col, row);
}

} // End namespace asp
