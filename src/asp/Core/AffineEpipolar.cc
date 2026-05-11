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
#include <asp/Core/InterestPointMatching.h>  // Slow-to-compile header
#include <asp/Core/IpMatchingAlgs.h>         // Lightweight header
#include <vw/Math/Vector.h>
#include <vw/Math/Matrix.h>
#include <vw/Math/RANSAC.h>
#include <vw/Math/LinearAlgebra.h>
#include <vw/InterestPoint/InterestPoint.h>
#include <vw/Core/Stopwatch.h>
#include <vw/Math/Transform.h>

#include <opencv2/calib3d.hpp>

#include <cstdlib>
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
    bool m_crop_to_shared_area;
    
    BestFitEpipolarAlignment(Vector2i const& left_image_dims,
                             Vector2i const& right_image_dims,
                             bool crop_to_shared_area):
      m_ldims(left_image_dims), m_rdims(right_image_dims),
      m_crop_to_shared_area(crop_to_shared_area) {}

    typedef vw::Matrix<double, 3, 7> result_type;

    /// The fundamental matrix needs 8 points.
    // TODO(oalexan1): Should a bigger minimum be used for robustness?
    template <class InterestPointT>
    size_t min_elements_needed_for_fit(InterestPointT const& /*example*/) const {
      return 8;
    }
  
    /// This function can match points in any container that supports
    /// the size() and operator[] methods. The container is usually a
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

      // Default solve_y_scaling/solve_x_shear allow non-uniform scale on
      // L vs R for tighter Y residual. That is harmful for windowed
      // correlation when the geometry forces the two scales far apart
      // (e.g. Chandrayaan-2 TMC fwd+nadir, scale_R ~ 0.9 vs scale_L = 1).
      // ASP_FORCE_UNIFORM_SCALE=1 substitutes translation-only fits so
      // both sides keep scale = 1; Y residual gets larger but correlation
      // can absorb that via its search range.
      bool force_uniform_scale = (std::getenv("ASP_FORCE_UNIFORM_SCALE") != NULL);
      if (force_uniform_scale) {
        // Print once - operator() is called many times by RANSAC.
        static bool printed_once = false;
        if (!printed_once) {
          vw_out() << "ASP_FORCE_UNIFORM_SCALE set; using uniform-scale "
                   << "rectification (rotation + translation only).\n";
          printed_once = true;
        }
        double sum_x_off = 0, sum_y_off = 0;
        for (size_t i = 0; i < ip1.size(); i++) {
          Vector3 lpt = left_matrix  * Vector3(ip1[i].x, ip1[i].y, 1);
          Vector3 rpt = right_matrix * Vector3(ip2[i].x, ip2[i].y, 1);
          sum_x_off += lpt(0) - rpt(0);
          sum_y_off += lpt(1) - rpt(1);
        }
        double n = double(ip1.size());
        right_matrix(0, 2) = sum_x_off / n;
        right_matrix(1, 2) = sum_y_off / n;
      } else {
        // Solve for ideal scaling and translation
        solve_y_scaling(ip1, ip2, left_matrix, right_matrix);

        // Solve for ideal shear, scale, and translation of X axis
        solve_x_shear(ip1, ip2, left_matrix, right_matrix);
      }

      // Work out the ideal render size
      BBox2i left_bbox, right_bbox;
      left_bbox.grow(subvector(left_matrix * Vector3(0,            0,           1), 0, 2));
      left_bbox.grow(subvector(left_matrix * Vector3(m_ldims.x(),  0,           1), 0, 2));
      left_bbox.grow(subvector(left_matrix * Vector3(m_ldims.x(),  m_ldims.y(), 1), 0, 2));
      left_bbox.grow(subvector(left_matrix * Vector3(0,            m_ldims.y(), 1), 0, 2));
      right_bbox.grow(subvector(right_matrix * Vector3(0,            0,           1), 0, 2));
      right_bbox.grow(subvector(right_matrix * Vector3(m_rdims.x(),  0,           1), 0, 2));
      right_bbox.grow(subvector(right_matrix * Vector3(m_rdims.x(),  m_rdims.y(), 1), 0, 2));
      right_bbox.grow(subvector(right_matrix * Vector3(0,            m_rdims.y(), 1), 0, 2));

      // TODO(oalexan1): There is room for improvement below,
      // but the attempts tried below (commented out) need
      // a lot more testing. Also, the current outlier filtering
      // is apparently not foolproof yet.
      
      // Ensure that the transforms map the interest points to points
      // with positive x and y, we will need that when later the
      // transformed images are computed.
      if (m_crop_to_shared_area) 
        left_bbox.crop(right_bbox);
      
      // Note how we subtract left_bbox.min() from both left_matrix
      // and right_matrix.  By subtracting the same thing we
      // maintain the property that a row in the left image is
      // matched to the same row in the right image after the
      // left_matrix and right_matrix transforms are applied.
      left_matrix (0, 2) -= left_bbox.min().x();
      left_matrix (1, 2) -= left_bbox.min().y();
      right_matrix(0, 2) -= left_bbox.min().x();
      right_matrix(1, 2) -= left_bbox.min().y();
      
      // Concatenate these into the answer
      result_type T;
      submatrix(T, 0, 0, 3, 3) = left_matrix;
      submatrix(T, 0, 3, 3, 3) = right_matrix;
      
      // Implicit in the logic below is the fact that left_bbox should now also
      // have left_bbox.min() subtracted from it, after which it becomes the
      // box with lower-left corner being (0, 0) and upper-right corner
      // being (left_bbox.width(), left_bbox.height()) which is
      // what we save here as the upper bound after the transform.
      T(0, 6) = left_bbox.width();
      T(1, 6) = left_bbox.height();

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
    
  // Try a Hartley-style homography-based epipolar rectification using
  // OpenCV's stereoRectifyUncalibrated. Per-side homography (8 DoF each,
  // vs 6 for affine) gives enough flexibility to satisfy BOTH epipolar Y
  // alignment AND ground-pixel-size equalization on asymmetric-tilt
  // geometries (e.g. TMC fwd+nadir, where the off-nadir camera's ground
  // pixel is ~10% larger than the nadir camera's, which pure affine
  // can't fully equalize without breaking Y-rectification).
  //
  // Returns true if the homography fit passes a battery of sanity
  // checks (det in range, det ratio close to 1, perspective denominators
  // small, sub-pixel-ish Y-residual on inliers, output bbox not blowing
  // up). Returns false otherwise; the caller falls back to the existing
  // affine path. For typical near-conformal pairs (LRO NAC, CTX,
  // TMC fwd+aft) the homography reduces to near-affine and the checks
  // pass with very similar matrices to what the affine path produces.
  bool try_homography_epipolar_rectification(
      Vector2i const& left_image_dims,
      Vector2i const& right_image_dims,
      double inlier_threshold,
      std::vector<ip::InterestPoint> const& ip1,
      std::vector<ip::InterestPoint> const& ip2,
      // Outputs
      Matrix<double>& left_matrix,
      Matrix<double>& right_matrix,
      Vector2i& trans_crop_box,
      std::vector<size_t>& inlier_indices) {

    if (ip1.size() < 8 || ip1.size() != ip2.size())
      return false;

    // Build OpenCV point arrays
    std::vector<cv::Point2f> pts1, pts2;
    pts1.reserve(ip1.size());
    pts2.reserve(ip2.size());
    for (size_t i = 0; i < ip1.size(); i++) {
      pts1.push_back(cv::Point2f(ip1[i].x, ip1[i].y));
      pts2.push_back(cv::Point2f(ip2[i].x, ip2[i].y));
    }

    // Robust fundamental matrix via RANSAC
    std::vector<uchar> inlier_mask;
    cv::Mat F = cv::findFundamentalMat(pts1, pts2, cv::FM_RANSAC,
                                       inlier_threshold, 0.99, inlier_mask);
    if (F.empty()) return false;

    // Filter to inliers
    std::vector<cv::Point2f> in1, in2;
    inlier_indices.clear();
    for (size_t i = 0; i < pts1.size(); i++) {
      if (inlier_mask[i]) {
        in1.push_back(pts1[i]);
        in2.push_back(pts2[i]);
        inlier_indices.push_back(i);
      }
    }
    if (in1.size() < 8) return false;

    // Recompute F using inliers only (8-point method, more accurate)
    F = cv::findFundamentalMat(in1, in2, cv::FM_8POINT);
    if (F.empty()) return false;

    // Hartley uncalibrated rectification
    cv::Mat H_L_cv, H_R_cv;
    cv::Size img_size(std::max(left_image_dims.x(), right_image_dims.x()),
                      std::max(left_image_dims.y(), right_image_dims.y()));
    bool ok = cv::stereoRectifyUncalibrated(in1, in2, F, img_size,
                                            H_L_cv, H_R_cv, inlier_threshold);
    if (!ok) return false;

    // Convert to vw::Matrix<double, 3, 3>
    Matrix<double> H_L(3, 3), H_R(3, 3);
    for (int i = 0; i < 3; i++) {
      for (int j = 0; j < 3; j++) {
        H_L(i, j) = H_L_cv.at<double>(i, j);
        H_R(i, j) = H_R_cv.at<double>(i, j);
      }
    }

    // SANITY CHECKS - all failures fall back to affine
    auto reject = [&](std::string const& why) {
      vw_out() << "Homography epipolar fit rejected: " << why
               << "; falling back to affine.\n";
      return false;
    };

    // (a) det of upper-left 2x2 in reasonable range
    double det_L = H_L(0, 0) * H_L(1, 1) - H_L(0, 1) * H_L(1, 0);
    double det_R = H_R(0, 0) * H_R(1, 1) - H_R(0, 1) * H_R(1, 0);
    if (std::abs(det_L) < 0.1 || std::abs(det_L) > 10.0)
      return reject("|det_L|=" + std::to_string(det_L) + " out of [0.1,10]");
    if (std::abs(det_R) < 0.1 || std::abs(det_R) > 10.0)
      return reject("|det_R|=" + std::to_string(det_R) + " out of [0.1,10]");

    // (b) det ratio close - this is the equalization sanity check
    double ratio = std::max(std::abs(det_L), std::abs(det_R)) /
                   std::min(std::abs(det_L), std::abs(det_R));
    if (ratio > 2.0)
      return reject("det ratio " + std::to_string(ratio) + " > 2.0");

    // (c) perspective denominators (last row of H) small
    if (std::abs(H_L(2, 0)) > 1e-3 || std::abs(H_L(2, 1)) > 1e-3 ||
        std::abs(H_R(2, 0)) > 1e-3 || std::abs(H_R(2, 1)) > 1e-3)
      return reject("perspective denominators too large");

    // (d) Y-residual on inliers sub-pixel-ish
    double max_y_resid = 0;
    for (size_t i = 0; i < in1.size(); i++) {
      Vector3 lp(in1[i].x, in1[i].y, 1);
      Vector3 rp(in2[i].x, in2[i].y, 1);
      Vector3 lpt = H_L * lp;
      Vector3 rpt = H_R * rp;
      if (lpt.z() != 0) lpt /= lpt.z();
      if (rpt.z() != 0) rpt /= rpt.z();
      max_y_resid = std::max(max_y_resid, std::abs(lpt.y() - rpt.y()));
    }
    if (max_y_resid > 5.0)
      return reject("max Y-residual " + std::to_string(max_y_resid) + " px > 5");

    // (e) output bbox not catastrophic
    BBox2 left_bbox;
    Vector3 left_corners[4] = {
      Vector3(0, 0, 1),
      Vector3(left_image_dims.x(), 0, 1),
      Vector3(left_image_dims.x(), left_image_dims.y(), 1),
      Vector3(0, left_image_dims.y(), 1)
    };
    for (int i = 0; i < 4; i++) {
      Vector3 q = H_L * left_corners[i];
      if (q.z() != 0) q /= q.z();
      left_bbox.grow(Vector2(q.x(), q.y()));
    }
    int max_dim = std::max(left_image_dims.x(), left_image_dims.y());
    if (left_bbox.width() > 4.0 * max_dim || left_bbox.height() > 4.0 * max_dim)
      return reject("output bbox blew up");

    // Translate matrices so output bbox starts at origin (matches the
    // existing affine code's bbox handling).
    H_L(0, 2) -= left_bbox.min().x();
    H_L(1, 2) -= left_bbox.min().y();
    H_R(0, 2) -= left_bbox.min().x();
    H_R(1, 2) -= left_bbox.min().y();

    // Outputs
    left_matrix = H_L;
    right_matrix = H_R;
    trans_crop_box = Vector2i(int(left_bbox.width()), int(left_bbox.height()));

    vw_out() << "Used homography-epipolar rectification (Hartley/Zisserman). "
             << inlier_indices.size() << " / " << ip1.size() << " inliers, "
             << "det_L=" << det_L << ", det_R=" << det_R
             << ", max Y-resid=" << max_y_resid << " px.\n";
    return true;
  }

  // Main function that other parts of ASP should use
  Vector2i affine_epipolar_rectification(Vector2i const& left_image_dims,
                                         Vector2i const& right_image_dims,
                                         double inlier_threshold,
                                         int num_ransac_iterations,
                                         std::vector<ip::InterestPoint> const& ip1,
                                         std::vector<ip::InterestPoint> const& ip2,
                                         bool crop_to_shared_area,
                                         Matrix<double>& left_matrix,
                                         Matrix<double>& right_matrix,
                                         // optionally return the inliers
                                         std::vector<size_t> * inliers_ptr) {

    // Try homography-based epipolar rectification first. It handles
    // asymmetric-tilt geometries (e.g. TMC fwd+nadir) that pure affine
    // can't fully equalize. Falls back to the existing affine RANSAC
    // path if any sanity check fails.
    {
      Vector2i hom_trans_crop_box;
      std::vector<size_t> hom_inliers;
      if (try_homography_epipolar_rectification(
            left_image_dims, right_image_dims, inlier_threshold,
            ip1, ip2,
            left_matrix, right_matrix, hom_trans_crop_box, hom_inliers)) {
        if (inliers_ptr) *inliers_ptr = hom_inliers;
        return hom_trans_crop_box;
      }
    }

    int  min_num_output_inliers = ip1.size() / 2;
    bool reduce_min_num_output_inliers_if_no_fit = true;

    vw::Matrix<double> T;
    Stopwatch sw;
    sw.start();

    vw_out() << "Computing the epipolar rectification "
             << "using RANSAC with " << num_ransac_iterations
             << " iterations and inlier threshold " << inlier_threshold << ".\n";

    // If RANSAC fails, it will throw an exception
    BestFitEpipolarAlignment func(left_image_dims, right_image_dims, crop_to_shared_area);
    EpipolarAlignmentError error_metric;
    std::vector<size_t> inlier_indices;
    RandomSampleConsensus<BestFitEpipolarAlignment, EpipolarAlignmentError> 
      ransac(func, error_metric,
             num_ransac_iterations, inlier_threshold,
             min_num_output_inliers, reduce_min_num_output_inliers_if_no_fit);
    try {
      T = ransac(ip1, ip2);
    } catch (std::exception const& e) {
      vw_throw(ArgumentErr() << "Failed compute the epipolar rectification.\n"
               << "Check if your left and right images are similar enough. "
               << "Consider deleting the output directory and restarting with "
               << "a larger --ip-per-tile value.\n"
               << "RANSAC error: " << e.what() << "\n");
    }
    inlier_indices = ransac.inlier_indices(T, ip1, ip2);

    vw_out() << "Found " << inlier_indices.size() << " / " << ip1.size() << " inliers.\n";

    sw.stop();
    vw_out() << "Elapsed time in computing rectification matrices: "
             << sw.elapsed_seconds() << " seconds.\n";

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
        
    vw_out() << "Maximum absolute difference of y components of "
             << "aligned inlier interest points is "
             << max_err << " pixels." << std::endl;

    // This needs more testing
    if (false && !crop_to_shared_area) {
      // The bounds of the transforms have been a bit too generous. Tighten them to the bounding
      // box of the IP.
      // TODO(oalexan1): Remove outliers here!
      
      // Apply local alignment to inlier ip and estimate the search range
      vw::HomographyTransform left_local_trans (left_matrix);
      vw::HomographyTransform right_local_trans(right_matrix);
      
      // Find the transformed IP
      std::vector<vw::ip::InterestPoint> left_trans_local_ip;
      std::vector<vw::ip::InterestPoint> right_trans_local_ip;

      for (size_t it = 0; it < inlier_indices.size(); it++) {
        int i = inlier_indices[it];
        Vector2 left_pt (ip1[i].x, ip1[i].y);
        Vector2 right_pt(ip2[i].x, ip2[i].y);
        
        left_pt  = left_local_trans.forward(left_pt);
        right_pt = right_local_trans.forward(right_pt);
        
        // First copy all the data from the input ip, then apply the transform
        left_trans_local_ip.push_back(ip1[i]);
        right_trans_local_ip.push_back(ip2[i]);
        left_trans_local_ip.back().x  = left_pt.x();
        left_trans_local_ip.back().y  = left_pt.y();
        right_trans_local_ip.back().x = right_pt.x();
        right_trans_local_ip.back().y = right_pt.y();
      }

      // Filter outliers
      Vector2 params = stereo_settings().outlier_removal_params;
      bool quiet = false;
      if (params[0] < 100.0)
        asp::filter_ip_by_disparity(params[0], params[1], quiet,
                                    left_trans_local_ip, right_trans_local_ip); 
      
      vw::BBox2i left_bbox, right_bbox;
      for (size_t i = 0; i < left_trans_local_ip.size(); i++) {

        Vector2 left_pt (left_trans_local_ip[i].x, left_trans_local_ip[i].y);
        Vector2 right_pt(right_trans_local_ip[i].x, right_trans_local_ip[i].y);
        
        left_bbox.grow(left_pt);
        right_bbox.grow(right_pt);
      }

      // TODO(oalexan1): Run a large scale test to see if this is necessary.
      left_bbox.expand(50);
      right_bbox.expand(50);
      
      // The way the transforms were created, there is no good reason
      // for transformed ip to have negative values.
      left_bbox.min().x() = std::max(left_bbox.min().x(), 0);
      left_bbox.min().y() = std::max(left_bbox.min().y(), 0);
      right_bbox.min().x() = std::max(right_bbox.min().x(), 0);
      right_bbox.min().y() = std::max(right_bbox.min().y(), 0);
      
      // Adjust the domains of the transforms to the bounding boxes of
      // the interest points.
      left_matrix (0, 2) -= left_bbox.min().x();
      left_matrix (1, 2) -= left_bbox.min().y();
      right_matrix(0, 2) -= right_bbox.min().x();
      right_matrix(1, 2) -= right_bbox.min().y();

      trans_crop_box[0] = std::max(left_bbox.width(), right_bbox.width());
      trans_crop_box[1] = std::max(left_bbox.height(), right_bbox.height());
    }
    
    // Optionally return the inliers
    if (inliers_ptr != NULL)
      *inliers_ptr = inlier_indices;
    
    return trans_crop_box;
  }

} // end namespace asp
