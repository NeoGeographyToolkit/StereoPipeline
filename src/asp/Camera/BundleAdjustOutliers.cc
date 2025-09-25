// __BEGIN_LICENSE__
//  Copyright (c) 2009-2025, United States Government as represented by the
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

// \file BundleAdjustOutliers.cc
//

// Logic for handling outliers in bundle adjustment. 

#include <asp/Camera/BundleAdjustOutliers.h>
#include <asp/Camera/BundleAdjustCamera.h>
#include <asp/Camera/BundleAdjustOptions.h>
#include <asp/Camera/BundleAdjustResiduals.h>
#include <asp/Core/OutlierProcessing.h>

#include <vw/Cartography/GeoReference.h>
#include <vw/BundleAdjustment/ControlNetwork.h>

// Turn off warnings from eigen
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-local-typedefs"
#include <ceres/ceres.h>
#include <ceres/loss_function.h>
#pragma GCC diagnostic pop

namespace  asp {

using namespace vw;
using namespace vw::camera;
using namespace vw::ba;

// Update the set of outliers based on param_storage
void updateOutliers(vw::ba::ControlNetwork const& cnet, 
                    asp::BAParams const& param_storage,
                    std::set<int> & outliers) {
  outliers.clear(); 
  for (int i = 0; i < param_storage.num_points(); i++)
    if (param_storage.get_point_outlier(i))
      outliers.insert(i); 
}

// Filter matches by projection window.
// TODO(oalexan1): Use this in jitter_solve.
// TODO(oalexan1): This needs to be done before subsampling the matches
void filterOutliersProjWin(asp::BaBaseOptions          & opt,
                           asp::BAParams               & param_storage, 
                           vw::ba::ControlNetwork const& cnet) {

  // Swap y. Sometimes it is convenient to specify these on input in reverse.
  if (opt.proj_win.min().y() > opt.proj_win.max().y())
    std::swap(opt.proj_win.min().y(), opt.proj_win.max().y());

  // Set the projection. The function set_proj4_projection_str() does not set the
  // datum radii, which is confusing. Use vw::cartography::set_srs_string().
  vw::cartography::GeoReference georef;
  bool have_datum = (opt.datum.name() != asp::UNSPECIFIED_DATUM);
  vw::cartography::set_srs_string(opt.proj_str, have_datum, opt.datum, georef);

  int num_points = param_storage.num_points();
  for (int i = 0; i < num_points; i++) {
      
    if (param_storage.get_point_outlier(i))
      continue;
      
    double* point = param_storage.get_point_ptr(i);
    Vector3 xyz(point[0], point[1], point[2]);
    Vector3 llh = georef.datum().cartesian_to_geodetic(xyz);
    Vector2 proj_pt = georef.lonlat_to_point(subvector(llh, 0, 2));

    if (!opt.proj_win.contains(proj_pt))
      param_storage.set_point_outlier(i, true);
  }
}

void filterOutliersByConvergenceAngle(asp::BaBaseOptions const& opt,
                                      vw::ba::ControlNetwork const& cnet,
                                      asp::BAParams & param_storage) {

  std::vector<vw::CamPtr> optimized_cams;
  std::vector<vw::Vector3> opt_cam_positions;
  asp::calcOptimizedCameras(opt, param_storage, optimized_cams);
  asp::calcCameraCenters(optimized_cams, opt_cam_positions);
  int num_outliers_by_conv_angle = 0;

  for (size_t ipt = 0; ipt < param_storage.num_points(); ipt++) {

    if (cnet[ipt].type() == ControlPoint::GroundControlPoint)
      continue; // don't filter out GCP
    if (param_storage.get_point_outlier(ipt))
      continue; // skip outliers
    
    // The GCC coordinate of this point
    const double * point = param_storage.get_point_ptr(ipt);
    Vector3 xyz(point[0], point[1], point[2]);
    
    // Control point
    auto const& cp = cnet[ipt];
    double max_angle = 0;
    for (size_t j = 0; j < cp.size(); j++) {
      size_t j_cam_id = cp[j].image_id();
      vw::Vector3 P1 = opt_cam_positions[j_cam_id];
      vw::Vector3 dir1 = xyz - P1;
      if (norm_2(dir1) > 1e-8) 
        dir1 = normalize(dir1);
      
      for (size_t k = j + 1; k < cp.size(); k++) {
        size_t k_cam_id = cp[k].image_id();
        vw::Vector3 P2 = opt_cam_positions[k_cam_id];
        vw::Vector3 dir2 = xyz - P2;
        if (norm_2(dir2) > 1e-8) 
          dir2 = normalize(dir2);
        
        double angle = (180.0 / M_PI) * acos(dot_prod(dir1, dir2));
        if (std::isnan(angle) || std::isinf(angle)) 
          continue;
        max_angle = std::max(max_angle, angle);
      }
    }
    
    if (max_angle < opt.min_triangulation_angle) {
      param_storage.set_point_outlier(ipt, true);
      num_outliers_by_conv_angle++;
    }
    
    if (opt.max_triangulation_angle > 0 && max_angle > opt.max_triangulation_angle) {
      param_storage.set_point_outlier(ipt, true);
      num_outliers_by_conv_angle++;
    }
  }
  
  int num_pts = param_storage.num_points();
  vw::vw_out() << std::setprecision(4) 
               << "Removed " << num_outliers_by_conv_angle 
               << " triangulated points out of " << num_pts
               << " (" << (100.0 * num_outliers_by_conv_angle) / num_pts << "%)" 
               << " by ray convergence angle.\n";
}

// Add to the outliers based on the large residuals
int add_to_outliers(vw::ba::ControlNetwork & cnet,
                    asp::CRN const& crn,
                    asp::BAParams & param_storage,
                    asp::BaOptions const& opt,
                    std::vector<size_t> const& cam_residual_counts,
                    std::vector<std::map<int, vw::Vector2>> const& pixel_sigmas,
                    size_t num_gcp_or_dem_residuals,
                    size_t num_uncertainty_residuals,
                    size_t num_tri_residuals,
                    size_t num_cam_pos_residuals,
                    std::vector<vw::Vector3> const& reference_vec,
                    ceres::Problem &problem) {

  vw_out() << "Removing outliers.\n";

  size_t num_points  = param_storage.num_points();
  size_t num_cameras = param_storage.num_cameras();

  // Compute the reprojection error. Adjust for residuals being divided by pixel sigma.
  // Do not use the attenuated residuals due to the loss function.
  std::vector<double> residuals;
  asp::compute_residuals(opt, crn, param_storage,  cam_residual_counts, pixel_sigmas,
                         num_gcp_or_dem_residuals, num_uncertainty_residuals,
                         num_tri_residuals, num_cam_pos_residuals,
                         reference_vec, problem,
                         residuals); // output

  // Compute the mean residual at each xyz, and how many times that residual is seen
  std::vector<double> mean_residuals;
  std::vector<int> num_point_observations;
  asp::compute_mean_residuals_at_xyz(crn, residuals, param_storage,
                                     mean_residuals, num_point_observations); // outputs

  // The number of mean residuals is the same as the number of points,
  // of which some are outliers. Hence need to collect only the
  // non-outliers so far to be able to remove new outliers.  Need to
  // follow the same logic as when residuals were formed. And also ignore GCP.
  std::vector<double> actual_residuals;
  std::set<int> was_added;
  for (size_t icam = 0; icam < num_cameras; icam++) {
    for (auto fiter = crn[icam].begin(); fiter != crn[icam].end(); fiter++) {

      // The index of the 3D point
      int ipt = (**fiter).m_point_id;

      // skip existing outliers
      if (param_storage.get_point_outlier(ipt))
        continue;

      // Skip gcp, those are never outliers no matter what.
      if (cnet[ipt].type() == ControlPoint::GroundControlPoint)
        continue;

      // We already encountered this residual in the previous camera
      if (was_added.find(ipt) != was_added.end())
        continue;

      was_added.insert(ipt);
      actual_residuals.push_back(mean_residuals[ipt]);
      //vw_out() << "XYZ residual " << ipt << " = " << mean_residuals[ipt] << std::endl;
    }
  } // End double loop through all the observations

  double pct      = 1.0 - opt.remove_outliers_params[0]/100.0;
  double factor   = opt.remove_outliers_params[1];
  double max_pix1 = opt.remove_outliers_params[2];
  double max_pix2 = opt.remove_outliers_params[3];

  double b, e;
  vw::math::find_outlier_brackets(actual_residuals, pct, factor, b, e);
  vw_out() << "Percentile-based outlier bounds: b = " << b << ", e = " << e << ".\n";

  // If this is too aggressive, the user can tame it. It is
  // unreasonable to throw out pixel residuals as small as 1 or 2
  // pixels. We will not use the b, because the residuals start at 0.
  // "max_pix2" sets the minimum error that can be thrown out.
  e = std::min(std::max(e, max_pix1), max_pix2);

  vw_out() << "Removing as outliers points with mean reprojection error > " << e << ".\n";

  // Add to the outliers by reprojection error. Must repeat the same logic as above.
  // TODO(oalexan1): This removes a 3D point altogether if any reprojection
  // errors for it are big. Need to only remove bad reprojection errors
  // and keep a 3D point if it is left with at least two reprojection residuals.
  int num_outliers_by_reprojection = 0, total = 0;
  for (size_t icam = 0; icam < num_cameras; icam++) {
    typedef CameraNode<JFeature>::const_iterator crn_iter;
    for (crn_iter fiter = crn[icam].begin(); fiter != crn[icam].end(); fiter++) {

      // The index of the 3D point
      int ipt = (**fiter).m_point_id;

      total++;

      // skip existing outliers
      if (param_storage.get_point_outlier(ipt))
        continue;

      // Skip gcp
      if (cnet[ipt].type() == ControlPoint::GroundControlPoint)
        continue;

      if (mean_residuals[ipt] > e) {
        param_storage.set_point_outlier(ipt, true);
        num_outliers_by_reprojection++;
      }
    }
  } // End double loop through all the observations
  vw_out() << "Removed " << num_outliers_by_reprojection << " outliers out of "
           << total << " interest points by reprojection error. Ratio: "
           << double(num_outliers_by_reprojection) / double(total) <<".\n";

  // Remove outliers by elevation limit
  int num_outliers_by_elev_or_lonlat = 0;
  if (opt.elevation_limit[0] < opt.elevation_limit[1] || !opt.lon_lat_limit.empty()) {

    for (size_t ipt = 0; ipt < param_storage.num_points(); ipt++) {

      if (cnet[ipt].type() == ControlPoint::GroundControlPoint)
        continue; // don't filter out GCP
      if (param_storage.get_point_outlier(ipt))
        continue; // skip outliers

      // The GCC coordinate of this point
      const double * point = param_storage.get_point_ptr(ipt);
      Vector3 xyz(point[0], point[1], point[2]);
      Vector3 llh = opt.datum.cartesian_to_geodetic(xyz);
      if (opt.elevation_limit[0] < opt.elevation_limit[1] &&
          (llh[2] < opt.elevation_limit[0] ||
           llh[2] > opt.elevation_limit[1])) {
        param_storage.set_point_outlier(ipt, true);
        num_outliers_by_elev_or_lonlat++;
        continue;
      }

      Vector2 lon_lat = subvector(llh, 0, 2);
      if (!opt.lon_lat_limit.empty() && !opt.lon_lat_limit.contains(lon_lat)) {
        param_storage.set_point_outlier(ipt, true);
        num_outliers_by_elev_or_lonlat++;
        continue;
      }

    }
    vw_out() << "Removed " << num_outliers_by_elev_or_lonlat
             << " outliers by elevation range and/or lon-lat range.\n";
  }

  // Remove outliers by convergence angle
  if (opt.min_triangulation_angle > 0)
    asp::filterOutliersByConvergenceAngle(opt, cnet, param_storage);

  // Remove outliers based on spatial extent. Be more generous with
  // leaving data in than what the input parameters suggest, because
  // sometimes inliers in space need not be uniformly distributed.
  double pct_factor = (3.0 + opt.remove_outliers_params[0]/100.0)/4.0; // e.g., 0.9375
  double outlier_factor = 2 * opt.remove_outliers_params[1];           // e.g., 6.0.
  std::vector<double> x_vals, y_vals, z_vals;
  for (size_t ipt = 0; ipt < param_storage.num_points(); ipt++) {

    if (cnet[ipt].type() == ControlPoint::GroundControlPoint)
      continue; // don't filter out GCP
    if (param_storage.get_point_outlier(ipt))
      continue; // skip outliers

    // The GCC coordinate of this point
    const double * point = param_storage.get_point_ptr(ipt);
    x_vals.push_back(point[0]);
    y_vals.push_back(point[1]);
    z_vals.push_back(point[2]);
  }
  vw::BBox3 estim_box;
  asp::estimate_inliers_bbox(pct_factor, pct_factor, pct_factor,
                             outlier_factor,
                             x_vals, y_vals, z_vals,
                             estim_box); // output

  int num_box_outliers = 0;
  for (size_t ipt = 0; ipt < param_storage.num_points(); ipt++) {

    if (cnet[ipt].type() == ControlPoint::GroundControlPoint)
      continue; // don't filter out GCP
    if (param_storage.get_point_outlier(ipt))
      continue; // skip outliers

    // The GCC coordinate of this point
    const double * point = param_storage.get_point_ptr(ipt);
    Vector3 xyz(point[0], point[1], point[2]);
    if (!estim_box.contains(xyz)) {
      param_storage.set_point_outlier(ipt, true);
      num_box_outliers++;
    }
  }

  vw_out() << "Removed " << num_box_outliers << " "
           << "outlier(s) based on spatial distribution of triangulated points.\n";

  int num_remaining_points = num_points - param_storage.get_num_outliers();

  return num_outliers_by_reprojection + num_outliers_by_elev_or_lonlat;
}

} // end namespace asp 
