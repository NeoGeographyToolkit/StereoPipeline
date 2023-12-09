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

// Implement least square alignment using the Ceres solver.

#include <asp/Tools/pc_align_ceres.h>
#include <asp/Core/EigenUtils.h>

#include <ceres/ceres.h>
#include <ceres/loss_function.h>

namespace asp {

using namespace vw;
  
// Discrepancy between a 3D point with the rotation to be solved
// applied to it, and its projection straight down onto the DEM. Used
// with the least squares method of finding the best transform between
// clouds.
struct PointToDemError {
  PointToDemError(Vector3 const& point,
		 ImageViewRef<vw::PixelMask<float> > const& dem,
		 cartography::GeoReference const& geo):
    m_point(point), m_dem(dem), m_geo(geo){}

  template <typename F>
  bool operator()(const F* const transform, const F* const scale, F* residuals) const {

    // Default residuals are zero, if we can't project into the DEM
    residuals[0] = F(0.0);

    // Extract the translation, and rotation
    Vector3 translation;
    Quat rotation;
    extract_rotation_translation(transform, rotation, translation);

    Vector3 trans_point = scale[0]*rotation.rotate(m_point) + translation;
    
    // Convert from GDC to GCC
    Vector3 llh = m_geo.datum().cartesian_to_geodetic(trans_point); // lon-lat-height

    // Interpolate the point at this location
    double dem_height_here;
    if (!interp_dem_height(m_dem, m_geo, llh, dem_height_here)) {
      // If we did not intersect the DEM, record a flag error value here.
      residuals[0] = F(0.0);
      return true;
    }
    
    residuals[0] = llh[2] - dem_height_here;
    return true;
  }
  
  // Factory to hide the construction of the CostFunction object from
  // the client code.
  static ceres::CostFunction* Create(Vector3 const& point,
				     vw::ImageViewRef<vw::PixelMask<float> > const& dem,
				     vw::cartography::GeoReference const& geo){
    return (new ceres::NumericDiffCostFunction<PointToDemError,
	    ceres::CENTRAL, 1, 6, 1>
	    (new PointToDemError(point, dem, geo)));
  }

  Vector3                                  m_point;
  ImageViewRef< PixelMask<float> > const & m_dem;    // alias
  cartography::GeoReference        const & m_geo;    // alias
};

/// Compute alignment using least squares
PointMatcher<RealT>::Matrix
least_squares_alignment(DP const& source_point_cloud, // Should not be modified
			vw::Vector3 const& point_cloud_shift,
			vw::cartography::GeoReference        const& dem_georef,
			vw::ImageViewRef< PixelMask<float> > const& dem_ref,
      std::string const& alignment_method,
      int num_iter, int num_threads) { 

  ceres::Problem problem;

  // The final transform as a axis angle and translation pair
  std::vector<double> transform(6, 0.0);

  double scale = 1.0;
  
  // Add a residual block for every source point
  const std::int64_t num_pts = source_point_cloud.features.cols();

  // Loop through every point in the point cloud
  for (std::int64_t i = 0; i < num_pts; i++){
    
    // Extract and un-shift the point to get the real GCC coordinate
    Vector3 gcc_coord = get_cloud_gcc_coord(source_point_cloud, point_cloud_shift, i);

    ceres::CostFunction* cost_function =
      PointToDemError::Create(gcc_coord, dem_ref, dem_georef);
    ceres::LossFunction* loss_function = new ceres::CauchyLoss(0.5); // NULL;
    problem.AddResidualBlock(cost_function, loss_function, &transform[0], &scale);
    
  } // End loop through all points

  if (alignment_method == "least-squares") {
    // Only solve for rotation and translation
    problem.SetParameterBlockConstant(&scale);
  }
  
  ceres::Solver::Options options;
  options.gradient_tolerance = 1e-16;
  options.function_tolerance = 1e-16;
  options.max_num_iterations = num_iter;
  options.minimizer_progress_to_stdout = 1;
  options.num_threads = num_threads;
  options.linear_solver_type = ceres::SPARSE_SCHUR;

  // Solve the problem
  ceres::Solver::Summary summary;
  ceres::Solve(options, &problem, &summary);

  vw_out() << summary.FullReport() << "\n" << std::endl;

  Quat rotation;
  Vector3 translation;
  extract_rotation_translation(&transform[0], rotation, translation);
  vw::Matrix<double,3,3> rot_matrix = rotation.rotation_matrix();
  
  PointMatcher<RealT>::Matrix T = PointMatcher<RealT>::Matrix::Identity(DIM + 1, DIM + 1);
  for (int row = 0; row < DIM; row++){
    for (int col = 0; col < DIM; col++){
      T(row, col) = scale*rot_matrix(col, row);
    }
  }

  for (int row = 0; row < DIM; row++)
    T(row, DIM) = translation[row];

  // This transform is in the world coordinate system (as that's the natural
  // coord system for the DEM). Transform it to the internal shifted coordinate
  // system.
  T = apply_shift(T, point_cloud_shift);

  return T;
}

/// Extracts the full GCC coordinate of a single point from a LibPointMatcher point cloud.
/// - The shift converts from the normalized coordinate to the actual GCC coordinate.
/// - No bounds checking is performed on the point index.
Vector3 get_cloud_gcc_coord(DP const& point_cloud, vw::Vector3 const& shift, int index) {
  Vector3 gcc_coord;
  for (int row = 0; row < DIM; ++row)
     gcc_coord[row] = point_cloud.features(row, index) + shift[row];
  return gcc_coord;
}

bool interp_dem_height(vw::ImageViewRef<vw::PixelMask<float> > const& dem,
                       vw::cartography::GeoReference const & georef,
                       vw::Vector3                   const & lonlat,
                       double                              & dem_height) {
  // Convert the lon/lat location into a pixel in the DEM.
  vw::Vector2 pix;
  try {
    pix = georef.lonlat_to_pixel(subvector(lonlat, 0, 2));
  }catch(...){
    return false;
  }
  
  double c = pix[0], r = pix[1];

  // Quit if the pixel falls outside the DEM.
  if (c < 0 || c >= dem.cols()-1 || // TODO: This ought to be an image class function
      r < 0 || r >= dem.rows()-1 )
    return false;

  // Interpolate the DEM height at the pixel location
  vw::PixelMask<float> v = dem(c, r);
  if (!is_valid(v))
    return false;

  dem_height = v.child();
  return true;
}

PointMatcher<RealT>::Matrix apply_shift(PointMatcher<RealT>::Matrix const& T,
                                        vw::Vector3 const& shift){

  // Consider a 4x4 matrix T which implements a rotation + translation
  // y = A*x + b. Consider a point s in space close to the points
  // x. We want to make that the new origin, so the points x get
  // closer to origin. In the coordinates (x2 = x - s, y2 = y - s) the
  // transform becomes y2 + s = A*(x2 + s) + b, or
  // y2 = A*x2 + b + A*s - s. Encode the obtained transform into another
  // 4x4 matrix T2.

  VW_ASSERT(T.cols() == 4 && T.rows() == 4,
            vw::ArgumentErr() << "Expected square matrix of size 4.");

  Eigen::MatrixXd A = T.block(0, 0, 3, 3);
  Eigen::MatrixXd b = T.block(0, 3, 3, 1);

  Eigen::MatrixXd s = b;
  for (int i = 0; i < 3; i++) s(i, 0) = shift[i];

  Eigen::MatrixXd b2 = b + A*s - s;
  PointMatcher<RealT>::Matrix T2 = T;
  T2.block(0, 3, 3, 1) = b2;

  return T2;
}
    
} // end namespace asp
