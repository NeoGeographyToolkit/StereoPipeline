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

#include <asp/Core/AffineEpipolar.h>
#include <asp/Core/OpenCVUtils.h>
#include <asp/Core/StereoSettings.h>
#include <vw/Math/Vector.h>
#include <vw/Math/Matrix.h>
#include <vw/Math/RANSAC.h>
#include <vw/Math/LinearAlgebra.h>
#include <vw/InterestPoint/InterestData.h>

#include <opencv2/calib3d.hpp>

#include <vector>

using namespace vw;
using namespace vw::math;

namespace asp {

  // Solves for Affine Fundamental Matrix as per instructions in
  // Multiple View Geometry. Outlier elimination happens later. 
  Matrix<double>
  linear_affine_fundamental_matrix(std::vector<ip::InterestPoint> const& ip1,
                                   std::vector<ip::InterestPoint> const& ip2) {

    // (i) Compute the centroid of X and delta X
    Matrix<double> delta_x(ip1.size(), 4);
    Vector4 mean_x;
    for (size_t i = 0; i < ip1.size(); i++) {
      delta_x(i, 0) = ip2[i].x;
      delta_x(i, 1) = ip2[i].y;
      delta_x(i, 2) = ip1[i].x;
      delta_x(i, 3) = ip1[i].y;
      mean_x += select_row(delta_x, i) / double(ip1.size());
    }
    
    for (size_t i = 0; i < ip1.size(); i++) 
      select_row(delta_x,i) -= mean_x;

    Matrix<double> U, VT;
    Vector<double> S;
    svd(transpose(delta_x), U, S, VT);
    Vector<double> N = select_col(U, 3);
    double e = -transpose(N) * mean_x;
    Matrix<double> f(3,3);
    f(0,2) = N(0);
    f(1,2) = N(1);
    f(2,2) = e;
    f(2,0) = N(2);
    f(2,1) = N(3);

    return f;
  }

  void solve_y_scaling(std::vector<ip::InterestPoint> const & ip1,
                       std::vector<ip::InterestPoint> const & ip2,
                       Matrix<double>                       & affine_left,
                       Matrix<double>                       & affine_right) {
    
    Matrix<double> a(ip1.size(), 2);
    Vector<double> b(ip1.size());
    
    for (size_t i = 0; i < ip1.size(); i++) {
      select_row(a, i) = subvector(affine_right*Vector3(ip2[i].x, ip2[i].y, 1), 1, 2);
      b[i]             = (affine_left*Vector3(ip1[i].x, ip1[i].y, 1))(1);
    }

    Vector<double> scaling = least_squares(a, b);
    submatrix(affine_right,0,0,2,2) *= scaling[0];
    affine_right(1,2) = scaling[1];
  }
  
  void solve_x_shear(std::vector<ip::InterestPoint> const & ip1,
                     std::vector<ip::InterestPoint> const & ip2,
                     Matrix<double>                       & affine_left,
                     Matrix<double>                       & affine_right) {
    
    Matrix<double> a(ip1.size(), 3);
    Vector<double> b(ip1.size());
    
    for (size_t i = 0; i < ip1.size(); i++) {
      select_row(a, i) = affine_right * Vector3(ip2[i].x, ip2[i].y, 1);
      b[i] = (affine_left * Vector3(ip1[i].x, ip1[i].y, 1))(0);
    }

    Vector<double> shear = least_squares(a, b);
    Matrix<double> interm = math::identity_matrix<3>();
    interm(0, 1) = -shear[1] / 2.0;
    affine_left = interm * affine_left;
    interm = math::identity_matrix<3>();
    interm(0, 0) = shear[0];
    interm(0, 1) = shear[1] / 2.0;
    interm(0, 2) = shear[2];
    affine_right = interm * affine_right;
  }

  // A functor which returns the best fit left and right 3x3 matrices
  // for epipolar alignment. Store them as a single 3x7 matrix.  The
  // last column will have the upper-right corner of the intersections
  // of the domains of the left and right images with the resulting
  // transformed applied to them.

  struct BestFitEpipolarAlignment {

    Vector2i m_ldims, m_rdims;
    
    BestFitEpipolarAlignment(Vector2i const& left_image_dims, Vector2i const& right_image_dims):
      m_ldims(left_image_dims), m_rdims(right_image_dims) {}

    typedef vw::Matrix<double, 3, 7> result_type;

    /// The fundamental matrix needs 8 points.
    // TODO(oalexan1): Should a bigger minimum be used for robustness?
    template <class InterestPointT>
    size_t min_elements_needed_for_fit(InterestPointT const& /*example*/) const {
      return 8;
    }
  
    /// This function can match points in any container that supports
    /// the size() and operator[] methods.  The container is usually a
    /// vw::Vector<>, but you could substitute other classes here as
    /// well.
    template <class InterestPointT>
    vw::Matrix<double> operator()(std::vector<InterestPointT> const& ip1,
                                  std::vector<InterestPointT> const& ip2,
                                  vw::Matrix<double> const& /*seed_input*/
                                  = vw::Matrix<double>() ) const {
    
      // check consistency
      VW_ASSERT( ip1.size() == ip2.size(),
                 vw::ArgumentErr() << "Cannot compute fundamental matrix. "
                 << "ip1 and ip2 are not the same size." );
      VW_ASSERT( !ip1.empty() && ip1.size() >= min_elements_needed_for_fit(ip1[0]),
                 vw::ArgumentErr() << "Cannot compute fundamental matrix. "
                 << "Need at at least 8 points, but got: " << ip1.size() << ".\n");

      // Compute the affine fundamental matrix
      Matrix<double> fund = linear_affine_fundamental_matrix(ip1, ip2);

      // Solve for rotation matrices
      double Hl = sqrt(fund(2, 0)*fund(2, 0) + fund(2, 1)*fund(2, 1));
      double Hr = sqrt(fund(0, 2)*fund(0, 2) + fund(1, 2)*fund(1, 2));

      Vector2 epipole(-fund(2, 1), fund(2, 0)), epipole_prime(-fund(1, 2), fund(0, 2));

      if (epipole.x() < 0)
        epipole = -epipole;
      if (epipole_prime.x() < 0)
        epipole_prime = -epipole_prime;
      epipole.y() = -epipole.y();
      epipole_prime.y() = -epipole_prime.y();

      Matrix<double> left_matrix  = math::identity_matrix<3>();
      Matrix<double> right_matrix = math::identity_matrix<3>();
    
      left_matrix(0, 0)  = epipole[0]/Hl;
      left_matrix(0, 1)  = -epipole[1]/Hl;
      left_matrix(1, 0)  = epipole[1]/Hl;
      left_matrix(1, 1)  = epipole[0]/Hl;
      right_matrix(0, 0) = epipole_prime[0]/Hr;
      right_matrix(0, 1) = -epipole_prime[1]/Hr;
      right_matrix(1, 0) = epipole_prime[1]/Hr;
      right_matrix(1, 1) = epipole_prime[0]/Hr;

      // Solve for ideal scaling and translation
      solve_y_scaling(ip1, ip2, left_matrix, right_matrix);

      // Solve for ideal shear, scale, and translation of X axis
      solve_x_shear(ip1, ip2, left_matrix, right_matrix);

      // Work out the ideal render size
      BBox2i output_bbox, right_bbox;
      output_bbox.grow(subvector(left_matrix * Vector3(0,            0,           1), 0, 2));
      output_bbox.grow(subvector(left_matrix * Vector3(m_ldims.x(),  0,           1), 0, 2));
      output_bbox.grow(subvector(left_matrix * Vector3(m_ldims.x(),  m_ldims.y(), 1), 0, 2));
      output_bbox.grow(subvector(left_matrix * Vector3(0,            m_ldims.y(), 1), 0, 2));
      right_bbox.grow(subvector(right_matrix * Vector3(0,            0,           1), 0, 2));
      right_bbox.grow(subvector(right_matrix * Vector3(m_rdims.x(),  0,           1), 0, 2));
      right_bbox.grow(subvector(right_matrix * Vector3(m_rdims.x(),  m_rdims.y(), 1), 0, 2));
      right_bbox.grow(subvector(right_matrix * Vector3(0,            m_rdims.y(), 1), 0, 2));
      
      output_bbox.crop(right_bbox);
      
      left_matrix (0, 2) -= output_bbox.min().x();
      right_matrix(0, 2) -= output_bbox.min().x();
      left_matrix (1, 2) -= output_bbox.min().y();
      right_matrix(1, 2) -= output_bbox.min().y();
      
      // Concatenate these into the answer
      result_type T;
      submatrix(T, 0, 0, 3, 3) = left_matrix;
      submatrix(T, 0, 3, 3, 3) = right_matrix;

      // Also save the domain after alignment
      T(0, 6) = output_bbox.width();
      T(1, 6) = output_bbox.height();
      
      return T;
    }
  };

  // Find the absolute difference of the y components of the given
  // interest point pair after applying to those points the given
  // epipolar alignment matrices. If these matrices are correct,
  // and the interest point pair is not an outlier, this
  // absolute difference should be close to 0.
  struct EpipolarAlignmentError {
    template <class TransformT, class InterestPointT>
    double operator() (TransformT const& T,
                       InterestPointT const& ip1,
                       InterestPointT const& ip2) const {

      Matrix<double> left_matrix  = submatrix(T, 0, 0, 3, 3);
      Matrix<double> right_matrix = submatrix(T, 0, 3, 3, 3);

      Vector3 L = left_matrix  * Vector3(ip1.x, ip1.y, 1);
      Vector3 R = right_matrix * Vector3(ip2.x, ip2.y, 1);
      double diff = L[1] - R[1];
      return std::abs(diff);
    }
  };

  // Helper function to instantiate a RANSAC class object and immediately call it
  template <class ContainerT1, class ContainerT2, class FittingFuncT, class ErrorFuncT>
  typename FittingFuncT::result_type ransac(std::vector<ContainerT1> const& p1,
                                            std::vector<ContainerT2> const& p2,
                                            FittingFuncT             const& fitting_func,
                                            ErrorFuncT               const& error_func,
                                            int     num_iterations,
                                            double  inlier_threshold,
                                            int     min_num_output_inliers,
                                            bool    reduce_min_num_output_inliers_if_no_fit = false
                                            ) {
    RandomSampleConsensus<FittingFuncT, ErrorFuncT>
      ransac_instance(fitting_func,
                      error_func,
                      num_iterations,
                      inlier_threshold,
                      min_num_output_inliers,
                      reduce_min_num_output_inliers_if_no_fit
                      );
    return ransac_instance(p1,p2);
  }
    
  // Main function that other parts of ASP should use
  Vector2i affine_epipolar_rectification(Vector2i const& left_image_dims,
                                         Vector2i const& right_image_dims,
                                         std::vector<ip::InterestPoint> const& ip1,
                                         std::vector<ip::InterestPoint> const& ip2,
                                         Matrix<double>& left_matrix,
                                         Matrix<double>& right_matrix) {
  
    int  min_num_output_inliers = ip1.size() / 2;
    bool reduce_min_num_output_inliers_if_no_fit = true;
    int  num_ransac_iterations = 10000;

    vw::Matrix<double> T;
    double inlier_threshold = stereo_settings().local_alignment_threshold;

    vw_out() << "Computing the epipolar rectification matrices "
             << "using RANSAC with " << num_ransac_iterations
             << " iterations and inlier threshold " << inlier_threshold << ".\n";
    
    BestFitEpipolarAlignment func(left_image_dims, right_image_dims);
    EpipolarAlignmentError error_metric;
    std::vector<size_t> inlier_indices;
    try {
      RandomSampleConsensus<BestFitEpipolarAlignment, EpipolarAlignmentError> 
        ransac(func, error_metric,
               num_ransac_iterations, inlier_threshold,
               min_num_output_inliers, reduce_min_num_output_inliers_if_no_fit);
    
      T = ransac(ip1, ip2);
      
      inlier_indices = ransac.inlier_indices(T, ip1, ip2);
    } catch (const RANSACErr& e) {
      vw_out() << "RANSAC failed: " << e.what() << "\n";
    }
    vw_out() << "Found " << inlier_indices.size() << " / " << ip1.size() << " inliers.\n";

    // Extract the matrices and the cropped transformed box from the computed transform
    left_matrix  = submatrix(T, 0, 0, 3, 3);
    right_matrix = submatrix(T, 0, 3, 3, 3);
    Vector2i trans_crop_box(T(0, 6), T(1, 6));

    // Find the maximum error for inliers
    double max_err = 0.0;
    for (size_t it = 0; it < inlier_indices.size(); it++) {
      int i = inlier_indices[it];
      max_err = std::max(max_err, error_metric(T, ip1[i], ip2[i]));
    }
        
    vw_out() << "The maximum absolute difference of the y components of the "
             << "inlier interest points with the alignment matrices applied to them: "
             << max_err << "." << std::endl;
    
    return trans_crop_box;
  }

} // end namespace asp
