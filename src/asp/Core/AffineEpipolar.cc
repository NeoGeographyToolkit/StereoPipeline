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
#include <vw/Math/LinearAlgebra.h>
#include <vw/InterestPoint/InterestData.h>

#include <opencv2/calib3d.hpp>

#include <vector>

using namespace vw;
using namespace vw::math;

namespace asp {

  VW_DEFINE_EXCEPTION(CustomRANSACErr, Exception);

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
  // for epipolar alignment. Store them as a single 3x6 matrix.

  struct BestFitEpipolarAlignment {

    BestFitEpipolarAlignment(){}

    typedef vw::Matrix<double, 3, 6> result_type;

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
      double Hl = sqrt(fund(2,0)*fund(2,0) + fund(2,1)*fund(2,1));
      double Hr = sqrt(fund(0,2)*fund(0,2) + fund(1,2)*fund(1,2));

      Vector2 epipole(-fund(2,1),fund(2,0)), epipole_prime(-fund(1,2),fund(0,2));

      if (epipole.x() < 0)
        epipole = -epipole;
      if (epipole_prime.x() < 0)
        epipole_prime = -epipole_prime;
      epipole.y() = -epipole.y();
      epipole_prime.y() = -epipole_prime.y();

      Matrix<double> left_matrix  = math::identity_matrix<3>();
      Matrix<double> right_matrix = math::identity_matrix<3>();
    
      left_matrix(0,0)  = epipole[0]/Hl;
      left_matrix(0,1)  = -epipole[1]/Hl;
      left_matrix(1,0)  = epipole[1]/Hl;
      left_matrix(1,1)  = epipole[0]/Hl;
      right_matrix(0,0) = epipole_prime[0]/Hr;
      right_matrix(0,1) = -epipole_prime[1]/Hr;
      right_matrix(1,0) = epipole_prime[1]/Hr;
      right_matrix(1,1) = epipole_prime[0]/Hr;

      // Solve for ideal scaling and translation
      solve_y_scaling(ip1, ip2, left_matrix, right_matrix);

      // Solve for ideal shear, scale, and translation of X axis
      solve_x_shear(ip1, ip2, left_matrix, right_matrix);

      // Concatenate these into the answer
      result_type T;
      submatrix(T, 0, 0, 3, 3) = left_matrix;
      submatrix(T, 0, 3, 3, 3) = right_matrix;

      return T;
    }
  };

  // Apply the alignment transform to left and right ip and find the
  // median of the y coordinate differences. In a perfect scenario all
  // these differences must be equal.
  template <class TransformT, class InterestPointT>
  double median_post_alignment_y(TransformT const& T, 
                                 std::vector<InterestPointT> const& ip1,
                                 std::vector<InterestPointT> const& ip2) {

    Matrix<double> left_matrix  = submatrix(T, 0, 0, 3, 3);
    Matrix<double> right_matrix = submatrix(T, 0, 3, 3, 3);
  
    std::vector<double> diff(ip1.size());
    for (size_t it = 0; it < ip1.size(); it++) {
      Vector3 L = left_matrix  * Vector3(ip1[it].x, ip1[it].y, 0);
      Vector3 R = right_matrix * Vector3(ip2[it].x, ip2[it].y, 0);
      diff[it] = L[1] - R[1];
    }

    std::sort(diff.begin(), diff.end());
  
    if (diff.empty())
      vw_throw(CustomRANSACErr() << "An empty vector of interest points was provided.");
  
    int i = (diff.size() - 1)/2;
    int j = diff.size()/2;
    double median = (diff[i] + diff[j])/2.0;
    
    return median;
  }

  // Apply given epipolar alignment matrices to the given interest point
  // pair, and find the absolute difference of the y component of the
  // difference of the transformed interest points and the median of
  // such differences for all the points.
  struct EpipolarAlignmentError {
    template <class TransformT, class InterestPointT>
    double operator() (TransformT const& T, double median_y,
                       InterestPointT const& ip1,
                       InterestPointT const& ip2) const {

      Matrix<double> left_matrix  = submatrix(T, 0, 0, 3, 3);
      Matrix<double> right_matrix = submatrix(T, 0, 3, 3, 3);

      Vector3 L = left_matrix  * Vector3(ip1.x, ip1.y, 0);
      Vector3 R = right_matrix * Vector3(ip2.x, ip2.y, 0);
      double diff = L[1] - R[1];
    
      return std::abs(diff - median_y);
    }
  };
  
  // Custom RANSAC where the error function is the discrepancy of the
  // y component of the given aligned interest point pair and the
  // median among given all such interest point pairs. That can't be
  // expressed in the regular RANSAC, hence we fork it here.
  template <class FittingFuncT, class ErrorFuncT>
  class CustomRandomSampleConsensus {
    const FittingFuncT& m_fitting_func;
    const ErrorFuncT  & m_error_func;
    int           m_num_iterations;
    double        m_inlier_threshold;
    int           m_min_num_output_inliers;
    bool          m_reduce_min_num_output_inliers_if_no_fit;
  
    /// \cond INTERNAL
    // Utility Function: Pick N UNIQUE, random integers in the range [0, size]
    // TODO(oalexan1): This appears to have quadratic complexity.
    inline void get_n_unique_integers(int size, std::vector<int> & samples) const {
    
      // Note: We do not modify the initial random seed. As such, if
      // a program uses RANSAC, repeatedly running this program will
      // always return the same results. However, if that program
      // calls RANSAC twice while within the same instance of the
      // program, the second time the result of RANSAC will be
      // different, since we keep on pulling new random numbers.
        
      int n = samples.size();
      VW_ASSERT(size >= n, ArgumentErr() << "Not enough samples (" << n << " / " << size << ")\n");

      const double divisor = static_cast<double>(RAND_MAX) + 1.0;
      for (int i = 0; i < n; ++i) {
        bool done = false;
        while (!done) {
          samples[i] = static_cast<int>( (static_cast<double>(std::rand()) / divisor) * size );
          done = true;
          for (int j = 0; j < i; j++)
            if (samples[i] == samples[j])
              done = false;
        }
      }
    }
    /// \endcond

  public:

    // Returns the list of inliers.
    template <class ContainerT1, class ContainerT2>
    void inliers(typename FittingFuncT::result_type const& H,
                 std::vector<ContainerT1>  const& p1, 
                 std::vector<ContainerT2>  const& p2,
                 std::vector<ContainerT1>       & inliers1, 
                 std::vector<ContainerT2>       & inliers2) const {

      inliers1.clear();
      inliers2.clear();

      double median_y = median_post_alignment_y(H, p1, p2);
      
      for (size_t i = 0; i < p1.size(); i++) {
        if (m_error_func(H, median_y, p1[i], p2[i]) < m_inlier_threshold) {
          inliers1.push_back(p1[i]);
          inliers2.push_back(p2[i]);
        }
      }
    }

    // Returns the list of inlier indices.
    template <class ContainerT1, class ContainerT2>
    std::vector<size_t> inlier_indices(typename FittingFuncT::result_type const& H,
                                       std::vector<ContainerT1>  const& p1,
                                       std::vector<ContainerT2>  const& p2) const {
      
      double median_y = median_post_alignment_y(H, p1, p2);

      std::vector<size_t> result;
      for (size_t i = 0; i < p1.size(); i++)
        if (m_error_func(H, median_y, p1[i], p2[i]) < m_inlier_threshold)
          result.push_back(i);
      return result;
    }

    void reduce_min_num_output_inliers(){
      m_min_num_output_inliers = int(m_min_num_output_inliers/1.5);
    }
      
    /// Constructor - Stores all the inputs in member variables
    CustomRandomSampleConsensus(FittingFuncT const& fitting_func, 
                                ErrorFuncT   const& error_func,
                                int    num_iterations,
                                double inlier_threshold,
                                int    min_num_output_inliers,
                                bool   reduce_min_num_output_inliers_if_no_fit = false):
      m_fitting_func(fitting_func), m_error_func(error_func),
      m_num_iterations(num_iterations), 
      m_inlier_threshold(inlier_threshold),
      m_min_num_output_inliers(min_num_output_inliers),
      m_reduce_min_num_output_inliers_if_no_fit(reduce_min_num_output_inliers_if_no_fit){}

    /// As attempt_ransac but keep trying with smaller numbers of required inliers.
    template <class ContainerT1, class ContainerT2>
    typename FittingFuncT::result_type operator()(std::vector<ContainerT1> const& p1,
                                                  std::vector<ContainerT2> const& p2) {

      // Try to fit using RANSAC. Perform repeated fits with smaller
      // m_min_num_output_inliers if the fit fails and
      // m_reduce_min_num_output_inliers_if_no_fit is true.

      typename FittingFuncT::result_type H;
      bool success = false;
      
      for (int attempt = 0; attempt < 10; attempt++){
        try{
          H = attempt_ransac(p1, p2);
          success = true;
          break; 
        } catch ( const std::exception& e ) { 
          vw_out() << e.what() << "\n";
          if (!m_reduce_min_num_output_inliers_if_no_fit) 
            break;
          reduce_min_num_output_inliers();
          // Can't possibly compute a transform with 1 or 0 samples
          if (m_min_num_output_inliers < 2) 
            break;
          vw_out() << "Attempting RANSAC with " << m_min_num_output_inliers
                   << " of output inliers.\n";
          
        }
      }

      if (!success) 
        vw_throw(CustomRANSACErr() << "RANSAC was unable to find a fit "
                 << "that matched the supplied data.");

      return H;
    }

    /// Run RANSAC on two input data lists using the current parameters.
    template <class ContainerT1, class ContainerT2>
    typename FittingFuncT::result_type attempt_ransac(std::vector<ContainerT1> const& p1,
                                                      std::vector<ContainerT2> const& p2) const {

      VW_ASSERT(!p1.empty(),
                CustomRANSACErr() << "RANSAC error. Insufficient data.\n");
      VW_ASSERT(p1.size() == p2.size(),
                CustomRANSACErr() << "RANSAC error. Data vectors are not the same size." );

      int min_elems_for_fit = m_fitting_func.min_elements_needed_for_fit(p1[0]);

      VW_ASSERT( (int)p1.size() >= min_elems_for_fit,
                 CustomRANSACErr() << "RANSAC error. Not enough potential matches "
                 << "for this fitting functor. ("
                 << p1.size() << "/" << min_elems_for_fit << ")\n");
      
      VW_ASSERT(m_min_num_output_inliers >= min_elems_for_fit,
                CustomRANSACErr() << "RANSAC error. Number of requested inliers is less "
                << "than min number of elements needed for fit. ("
                << m_min_num_output_inliers << "/" << min_elems_for_fit << ")\n");
      
      typename FittingFuncT::result_type best_H;

      std::vector<ContainerT1> try1;
      std::vector<ContainerT2> try2;
      std::vector<int> random_indices(min_elems_for_fit);

      int num_inliers = 0;
      double min_err = std::numeric_limits<double>::max();
      
      for (int iteration = 0; iteration < m_num_iterations; iteration++) {

        // 0. Get min_elems_for_fit points at random, taking care not
        //    to select the same point twice.
        get_n_unique_integers(p1.size(), random_indices);
        // Resizing below is essential, as by now their size may have changed
        try1.resize(min_elems_for_fit);
        try2.resize(min_elems_for_fit);
        for (int i = 0; i < min_elems_for_fit; ++i) {
          try1[i] = p1[random_indices[i]];
          try2[i] = p2[random_indices[i]];
        }

        // 1. Compute the fit using these samples.
        typename FittingFuncT::result_type H = m_fitting_func(try1, try2);

        // 2. Find all the inliers for this fit.
        inliers(H, p1, p2, try1, try2);

        // 3. Skip this model if too few inliers.
        if ((int)try1.size() < m_min_num_output_inliers) 
          continue;

        // 4. Re-estimate the model using the inliers.
        H = m_fitting_func(try1, try2, H);
        
        // 5. Find the mean error for the inliers.
        double median_y = median_post_alignment_y(H, try1, try2);
        double err_val = 0.0;
        for (size_t i = 0; i < try1.size(); i++) 
          err_val += m_error_func(H, median_y, try1[i], try2[i]);
        err_val /= try1.size();

        // 6. Save this model if its error is lowest so far.
        if (err_val < min_err){
          min_err     = err_val;
          best_H      = H;
          num_inliers = try1.size();
        }

      }

      if (num_inliers < m_min_num_output_inliers) {
        vw_throw( CustomRANSACErr() << "RANSAC was unable to find a "
                  << "fit that matched the supplied data.");
      }

      // For debugging
      VW_OUT(InfoMessage, "interest_point") << "\nRANSAC dummary:" << std::endl;
      VW_OUT(InfoMessage, "interest_point") << "\tFit = " << best_H << std::endl;
      VW_OUT(InfoMessage, "interest_point") << "\tInliers / total  = "
                                            << num_inliers << " / " << p1.size() << "\n\n";
      
      return best_H;
    }

  }; // End of CustomRandomSampleConsensus class definition

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
    CustomRandomSampleConsensus<FittingFuncT, ErrorFuncT>
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
  Vector2i affine_epipolar_rectification(Vector2i const& left_size,
                                         Vector2i const& right_size,
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
    
    std::vector<size_t> inlier_indices;
    try {
      BestFitEpipolarAlignment func;
      EpipolarAlignmentError error_metric;
      CustomRandomSampleConsensus<BestFitEpipolarAlignment, EpipolarAlignmentError> 
        ransac(func, error_metric,
               num_ransac_iterations, inlier_threshold,
               min_num_output_inliers, reduce_min_num_output_inliers_if_no_fit);
    
      T = ransac(ip1, ip2);
      
      inlier_indices = ransac.inlier_indices(T, ip1, ip2);
    } catch (const CustomRANSACErr& e) {
      vw_out() << "RANSAC failed: " << e.what() << "\n";
    }
    vw_out() << "Found " << inlier_indices.size() << " / " << ip1.size() << " inliers.\n";

    left_matrix  = submatrix(T, 0, 0, 3, 3);
    right_matrix = submatrix(T, 0, 3, 3, 3);
    
    double min_y = std::numeric_limits<double>::max()/10.0;
    double max_y = -min_y;
    for (size_t it = 0; it < inlier_indices.size(); it++) {

      int i = inlier_indices[it];
      Vector3 L = left_matrix  * Vector3(ip1[i].x, ip1[i].y, 0);
      Vector3 R = right_matrix * Vector3(ip2[i].x, ip2[i].y, 0);

      double y = L[1] - R[1];
      if (y > max_y) 
        max_y = y;
      if (y < min_y) 
        min_y = y;
    }

    vw_out() << "The half-range in y of inlier interest points after epipolar alignment: "
             << (max_y - min_y)/2 << "." << std::endl;
    
    // Work out the ideal render size.
    BBox2i output_bbox, right_bbox;
    output_bbox.grow(subvector(left_matrix * Vector3(0,0,1),                           0, 2));
    output_bbox.grow(subvector(left_matrix * Vector3(left_size.x(),0,1),               0, 2));
    output_bbox.grow(subvector(left_matrix * Vector3(left_size.x(),left_size.y(),1),   0, 2));
    output_bbox.grow(subvector(left_matrix * Vector3(0,left_size.y(),1),               0, 2));
    right_bbox.grow(subvector(right_matrix * Vector3(0,0,1),                           0, 2));
    right_bbox.grow(subvector(right_matrix * Vector3(right_size.x(),0,1),              0, 2));
    right_bbox.grow(subvector(right_matrix * Vector3(right_size.x(),right_size.y(),1), 0, 2));
    right_bbox.grow(subvector(right_matrix * Vector3(0,right_size.y(),1),              0, 2));
    output_bbox.crop(right_bbox);

    left_matrix (0, 2) -= output_bbox.min().x();
    right_matrix(0, 2) -= output_bbox.min().x();
    left_matrix (1, 2) -= output_bbox.min().y();
    right_matrix(1, 2) -= output_bbox.min().y();

    
    return Vector2i(output_bbox.width(), output_bbox.height());
  }

} // end namespace asp
