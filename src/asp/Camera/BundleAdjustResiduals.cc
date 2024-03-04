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

/// \file BundleAdjustResiduals.cc
///

// Logic for computing residuals for bundle adjustment. 

#include <asp/Camera/BundleAdjustResiduals.h>

#include <vw/BundleAdjustment/ControlNetwork.h>
#include <vw/BundleAdjustment/ControlNetworkLoader.h>
#include <vw/Math/Functors.h>

#include <string>

using namespace vw;
using namespace vw::camera;
using namespace vw::ba;

namespace  asp {

/// Compute the bundle_adjust residuals
void compute_residuals(bool apply_loss_function,
                       asp::BaBaseOptions const& opt,
                       asp::BAParams const& param_storage,
                       std::vector<size_t> const& cam_residual_counts,
                       size_t num_gcp_or_dem_residuals,
                       size_t num_uncertainty_residuals,
                       size_t num_tri_residuals,
                       size_t num_cam_position_residuals,
                       std::vector<vw::Vector3> const& reference_vec,
                       ceres::Problem & problem,
                       // Output
                       std::vector<double> & residuals) {

  // TODO(oalexan1): Associate residuals with cameras!
  // Generate some additional diagnostic info

  double cost = 0.0;
  ceres::Problem::EvaluateOptions eval_options;
  eval_options.apply_loss_function = apply_loss_function;
  if (opt.single_threaded_cameras)
    eval_options.num_threads = 1; // ISIS must be single threaded!
  else
    eval_options.num_threads = opt.num_threads;

  problem.Evaluate(eval_options, &cost, &residuals, 0, 0);
  const size_t num_residuals = residuals.size();
  
  // Verify our book-keeping is correct
  size_t num_expected_residuals
    = (num_gcp_or_dem_residuals + num_tri_residuals) * param_storage.params_per_point();

  size_t total_num_cam_params = param_storage.num_cameras()*param_storage.params_per_camera();

  for (size_t i = 0; i < param_storage.num_cameras(); i++)
    num_expected_residuals += cam_residual_counts[i]*PIXEL_SIZE;
  if (opt.camera_weight > 0)
    num_expected_residuals += total_num_cam_params;
  if (opt.rotation_weight > 0)
    num_expected_residuals += total_num_cam_params;
  num_expected_residuals += num_uncertainty_residuals * PIXEL_SIZE;
  num_expected_residuals += reference_vec.size() * PIXEL_SIZE;
  num_expected_residuals += num_cam_position_residuals * param_storage.params_per_camera();
  
  if (num_expected_residuals != num_residuals)
    vw_throw(LogicErr() << "Expected " << num_expected_residuals
                        << " residuals but instead got " << num_residuals);
}

/// Compute residual map by averaging all the reprojection error at a given point
void compute_mean_residuals_at_xyz(asp::CRNJ const& crn,
                                  std::vector<double> const& residuals,
                                  asp::BAParams const& param_storage,
                                  // outputs
                                  std::vector<double> & mean_residuals,
                                  std::vector<int>  & num_point_observations) {

  mean_residuals.resize(param_storage.num_points());
  num_point_observations.resize(param_storage.num_points());
  
  // Observation residuals are stored at the beginning of the residual vector in the 
  // same order they were originally added to Ceres.
  
  size_t residual_index = 0;
  // Double loop through cameras and crn entries will give us the correct order
  for (size_t icam = 0; icam < param_storage.num_cameras(); icam++) {
    for (auto fiter = crn[icam].begin(); fiter != crn[icam].end(); fiter++) {

      // The index of the 3D point
      int ipt = (**fiter).m_point_id;

      if (param_storage.get_point_outlier(ipt))
        continue; // skip outliers

      // Get the residual norm for this observation
      double errorX         = residuals[residual_index];
      double errorY         = residuals[residual_index+1];
      double residual_error = norm_2(vw::Vector2(errorX, errorY));
      residual_index += PIXEL_SIZE;

      // Update information for this point
      num_point_observations[ipt] += 1;
      mean_residuals        [ipt] += residual_error;
    }
  } // End double loop through all the observations

  // Do the averaging
  for (size_t i = 0; i < param_storage.num_points(); i++) {
    if (param_storage.get_point_outlier(i)) {
      // Skip outliers. But initialize to something.
      mean_residuals        [i] = std::numeric_limits<double>::quiet_NaN();
      num_point_observations[i] = std::numeric_limits<int>::quiet_NaN();
      continue;
    }
    mean_residuals[i] /= static_cast<double>(num_point_observations[i]);
  }
  
} // End function compute_mean_residuals_at_xyz
  
  /// Write out a .csv file recording the residual error at each location on the ground
void write_residual_map(std::string const& output_prefix,
                        // Mean residual of each point
                        std::vector<double> const& mean_residuals,
                        // Num non-outlier pixels per point
                        std::vector<int> const& num_point_observations, 
                        asp::BAParams const& param_storage,
                        vw::ba::ControlNetwork const& cnet,
                        asp::BaBaseOptions const& opt) {

  std::string output_path = output_prefix + ".csv";

  if (opt.datum.name() == asp::UNSPECIFIED_DATUM) {
    vw_out(WarningMessage) 
      << "No datum specified, cannot write file: " << output_path << ". "
      << "Specify: '--datum <planet name>'.\n";
    return;
  }
  if (mean_residuals.size() != param_storage.num_points())
    vw_throw(LogicErr() << "Point count mismatch in write_residual_map().\n");

  if (cnet.size() != param_storage.num_points()) 
    vw_throw(LogicErr()
              << "The number of stored points "
              << "does not agree with number of points in cnet.\n");
  
  // Open the output file and write the header
  vw_out() << "Writing: " << output_path << std::endl;
  std::ofstream file;
  file.open(output_path.c_str());
  file.precision(17);
  file << "# lon, lat, height_above_datum, mean_residual, num_observations\n";

  // stereo_gui counts on being able to parse the datum from this file, so
  // do not modify the line below.
  file << "# " << opt.datum << std::endl;
  
  // Now write all the points to the file
  for (size_t i = 0; i < param_storage.num_points(); i++) {

    if (param_storage.get_point_outlier(i))
      continue; // skip outliers
    
      // The final GCC coordinate of this point
      const double * point = param_storage.get_point_ptr(i);
      Vector3 xyz(point[0], point[1], point[2]);

      Vector3 llh = opt.datum.cartesian_to_geodetic(xyz);

      std::string comment = "";
      if (cnet[i].type() == ControlPoint::GroundControlPoint)
        comment = " # GCP";
      else if (cnet[i].type() == ControlPoint::PointFromDem)
        comment = " # from DEM";
      
      file << llh[0] <<", "<< llh[1] <<", "<< llh[2] <<", "<< mean_residuals[i] <<", "
           << num_point_observations[i] << comment << std::endl;
  }
  file.close();

} // End function write_residual_map

/// Write log files describing all residual errors. The order of data stored
/// in residuals must mirror perfectly the way residuals were created. 
void write_residual_logs(std::string const& residual_prefix, bool apply_loss_function,
                         asp::BaBaseOptions const& opt,
                         asp::BAParams const& param_storage,
                         std::vector<size_t> const& cam_residual_counts,
                         size_t num_gcp_or_dem_residuals,
                         size_t num_uncertainty_residuals,
                         size_t num_tri_residuals,
                         size_t num_cam_position_residuals,
                         std::vector<vw::Vector3> const& reference_vec,
                         vw::ba::ControlNetwork const& cnet, 
                         asp::CRNJ const& crn, 
                         ceres::Problem &problem) {

  std::vector<double> residuals;
  asp::compute_residuals(apply_loss_function, opt, param_storage,
                    cam_residual_counts, num_gcp_or_dem_residuals, 
                    num_uncertainty_residuals,
                    num_tri_residuals, num_cam_position_residuals,
                    reference_vec, problem,
                    // Output
                    residuals);
    
  const size_t num_residuals = residuals.size();

  const std::string residual_path               = residual_prefix + "_stats.txt";
  const std::string residual_raw_pixels_path    = residual_prefix + "_raw_pixels.txt";
  const std::string residual_reference_xyz_path = residual_prefix + "_reference_terrain.txt";

  // Write a report on residual errors
  std::ofstream residual_file, residual_file_raw_pixels, residual_file_reference_xyz;
  vw_out() << "Writing: " << residual_path << std::endl;
  vw_out() << "Writing: " << residual_raw_pixels_path << std::endl;
  
  residual_file.open(residual_path.c_str());
  residual_file.precision(17);
  residual_file_raw_pixels.open(residual_raw_pixels_path.c_str());
  residual_file_raw_pixels.precision(17);

  if (reference_vec.size() > 0) {
    //vw_out() << "Writing: " << residual_reference_xyz_path << std::endl;
    residual_file_reference_xyz.open(residual_reference_xyz_path.c_str());
    residual_file_reference_xyz.precision(17);
  }
  
  size_t index = 0;
  // For each camera, average together all the point observation residuals
  residual_file << "Mean and median pixel reprojection error and point count for cameras:\n";
  for (size_t c = 0; c < param_storage.num_cameras(); c++) {
    size_t num_this_cam_residuals = cam_residual_counts[c];
    
    // Write header for the raw file
    std::string name = opt.camera_files[c];
    if (name == "")
      name = opt.image_files[c];
    
    residual_file_raw_pixels << name << ", " << num_this_cam_residuals << std::endl;

    // All residuals are for inliers, as we do not even add a residual
    // for an outlier
    
    double mean_residual = 0; // Take average of all pixel coord errors
    std::vector<double> residual_norms;
    for (size_t i = 0; i < num_this_cam_residuals; i++) {
      double ex = residuals[index]; index++;
      double ey = residuals[index]; index++;
      double residual_norm = std::sqrt(ex * ex + ey * ey);
      mean_residual += residual_norm;
      residual_norms.push_back(residual_norm);
      residual_file_raw_pixels << ex << ", " << ey << std::endl;
    }
    // Write line for the summary file
    mean_residual /= static_cast<double>(num_this_cam_residuals);
    double median_residual = std::numeric_limits<double>::quiet_NaN();
    if (residual_norms.size() > 0) {
      std::sort(residual_norms.begin(), residual_norms.end());
      median_residual = residual_norms[residual_norms.size()/2];
    }
    
    residual_file << name                   << ", "
                  << mean_residual          << ", "
                  << median_residual        << ", "
                  << num_this_cam_residuals << std::endl;
  }
  
  residual_file_raw_pixels.close();
  residual_file.close();
  
  // Go through the GCP residuals
  if (num_gcp_or_dem_residuals > 0) {
    for (size_t i = 0; i < num_gcp_or_dem_residuals; i++) {
      double mean_residual = 0; // Take average of XYZ error for each point
      for (size_t j = 0; j < param_storage.params_per_point(); j++) {
        mean_residual += fabs(residuals[index]);
        index++;
      }
      mean_residual /= static_cast<double>(param_storage.params_per_point());
    }
  }
  
  // List the camera weight residuals
  int num_passes = int(opt.camera_weight > 0) +
    int(opt.rotation_weight > 0);
  for (int pass = 0; pass < num_passes; pass++) {
    const size_t part_size = param_storage.params_per_camera()/2;
    for (size_t c = 0; c < param_storage.num_cameras(); c++) {
      // Separately compute the mean position and rotation error
      double mean_residual_pos = 0, mean_residual_rot = 0;
      for (size_t j = 0; j < part_size; j++) {
        mean_residual_pos += fabs(residuals[index]);
        index++;
      }
      for (size_t j = 0; j < part_size; j++) {
        mean_residual_rot += fabs(residuals[index]);
        index++;
      }
      mean_residual_pos /= static_cast<double>(part_size);
      mean_residual_rot /= static_cast<double>(part_size);
    }
  }

  // Keep track of number of camera uncertainty residuals but don't save those
  index += PIXEL_SIZE * num_uncertainty_residuals;

  // List residuals for matching input terrain (lidar)
  if (reference_vec.size() > 0) {
    residual_file << "reference terrain residual errors:\n";
    residual_file_reference_xyz << "# lon, lat, height_above_datum, pixel_error_norm\n";
    for (size_t i = 0; i < reference_vec.size(); i++) {

      Vector3 llh = opt.datum.cartesian_to_geodetic(reference_vec[i]);
      double err = norm_2(Vector2(residuals[index], residuals[index + 1]));

      // Divide back the residual by the multiplier weight
      if (opt.reference_terrain_weight > 0) 
        err /= opt.reference_terrain_weight;
      
      index += PIXEL_SIZE;
      residual_file_reference_xyz << llh[0] << ", " << llh[1] << ", " << llh[2] << ", "
                                  << err << "\n";
      residual_file << i << ", " << err << "\n";
      
    }
    residual_file_reference_xyz.close();
  }

  // Keep track of number of triangulation constraint residuals but don't save those
  index += asp::PARAMS_PER_POINT * num_tri_residuals;
  index +=  param_storage.params_per_camera() * num_cam_position_residuals;
  
  if (index != num_residuals)
    vw_throw( LogicErr() << "Have " << num_residuals << " residuals, but iterated through "
              << index);

  // Generate the location based file
  std::string map_prefix = residual_prefix + "_pointmap";
  std::vector<double> mean_residuals;
  std::vector<int> num_point_observations;
  compute_mean_residuals_at_xyz(crn, residuals, param_storage,
                                mean_residuals, num_point_observations);

  write_residual_map(map_prefix, mean_residuals, num_point_observations,
                     param_storage, cnet, opt);

} // End function write_residual_logs

// Find the offsets between initial and final triangulated points
// TODO(oalexan1): Copy the data from param storage, then
// use the 2nd function called saveTriOffsetsPerCamera instead
// to reduce code duplication.
void saveTriOffsetsPerCamera(std::vector<std::string> const& image_files,
                             asp::BAParams const& orig_params,
                             asp::BAParams const& param_storage,
                             asp::CRNJ const& crn,
                             std::string const& tri_offsets_file) {

  // Number of cameras and points
  int num_cams = param_storage.num_cameras();
  int num_points = param_storage.num_points();
  
  // Need to have a vector of vectors, one for each camera
  std::vector<std::vector<double>> tri_offsets(num_cams);
  
  for (int icam = 0; icam < num_cams; icam++) {
    
    for (auto fiter = crn[icam].begin(); fiter != crn[icam].end(); fiter++) {

      // The index of the 3D point
      int ipt = (**fiter).m_point_id;
  
      // Sanity check
      if (ipt < 0 || ipt >= num_points)
        vw_throw(LogicErr() << "Invalid point index " << ipt 
                 << " in saveTriOffsetsPerCamera().\n");
        
      if (param_storage.get_point_outlier(ipt))
        continue; // skip outliers
      
      // Initial ECEF triangulated point
      double const* orig_point = orig_params.get_point_ptr(ipt);
      vw::Vector3 initial_xyz(orig_point[0], orig_point[1], orig_point[2]);
      
      // Optimized ECEF triangulated point
      double const* point = param_storage.get_point_ptr(ipt);
      vw::Vector3 final_xyz(point[0], point[1], point[2]);
     
      // Append the norm of offset
      tri_offsets[icam].push_back(norm_2(final_xyz - initial_xyz));   
    }
  }
  
  asp::writeTriOffsetsPerCamera(num_cams, image_files, tri_offsets, tri_offsets_file);
}

// Analogous version to the above, but keep the original and current triangulated points
// in std<vector>, for use in jitter_solve. Note that tri_points_vec may have
// anchor points later on, but we don't get to them.
void saveTriOffsetsPerCamera(std::vector<std::string> const& image_files,
                             std::set<int>            const& outliers,
                             std::vector<double>      const& orig_tri_points_vec,
                             std::vector<double>      const& tri_points_vec, 
                             asp::CRNJ                const& crn,
                             std::string const& tri_offsets_file) {

  if (orig_tri_points_vec.size() != tri_points_vec.size())
    vw_throw(ArgumentErr() << "Expecting the same number of original and current 3D "
                           << "points.\n");

  // Number of cameras and points
  int num_cams = image_files.size();
  int num_points = orig_tri_points_vec.size()/3;
  
  // Need to have a vector of vectors, one for each camera
  std::vector<std::vector<double>> tri_offsets(num_cams);

  for (int icam = 0; icam < num_cams; icam++) {
    
    for (auto fiter = crn[icam].begin(); fiter != crn[icam].end(); fiter++) {

      // The index of the 3D point
      int ipt = (**fiter).m_point_id;
  
      // Sanity check
      if (ipt < 0 || ipt >= num_points)
        vw_throw(LogicErr() << "Invalid point index " << ipt 
                 << " in saveTriOffsetsPerCamera().\n");
        
      if (outliers.find(ipt) != outliers.end())
        continue; // Skip outliers
      
      // Initial and optimized ECEF triangulated points
      double const* orig_point = &orig_tri_points_vec[3*ipt];
      double const* point      = &tri_points_vec[3*ipt];

      vw::Vector3 initial_xyz(orig_point[0], orig_point[1], orig_point[2]);      
      vw::Vector3 final_xyz(point[0], point[1], point[2]);
     
      // Append the norm of offset
      tri_offsets[icam].push_back(norm_2(final_xyz - initial_xyz));   
    }
  }
  
  asp::writeTriOffsetsPerCamera(num_cams, image_files, tri_offsets, tri_offsets_file);
}

// Write the offsets between initial and final triangulated points
void writeTriOffsetsPerCamera(int num_cams,
                              std::vector<std::string> const& image_files,
                              std::vector<std::vector<double>> & tri_offsets,
                              std::string const& tri_offsets_file) {
  
  // Write to disk with 8 digits of precision
  vw::vw_out() << "Writing: " << tri_offsets_file << std::endl;
  std::ofstream ofs(tri_offsets_file.c_str());
  ofs.precision(8);
  ofs << "# Per-image offsets between initial and final triangulated points (meters)\n";
  ofs << "# Image mean median count\n";
  
  // Iterate through the cameras
  double nan = std::numeric_limits<double>::quiet_NaN();
  for (int icam = 0; icam < num_cams; icam++) {
    auto & offsets = tri_offsets[icam];
    double mean = nan, median = nan, count = 0;
    if (!offsets.empty()) {
      mean = vw::math::mean(offsets);
      median = vw::math::destructive_median<double>(offsets);
      count = offsets.size();
    }
    ofs << image_files[icam] << " " << mean << " " << median << " " << count << "\n";  
  }
  ofs.close();
}

// Compute the horizontal and vertical change in camera positions
void saveCameraOffsets(vw::cartography::Datum   const& datum,
                       std::vector<std::string> const& image_files,
                       std::vector<vw::Vector3> const& orig_cam_positions,
                       std::vector<vw::Vector3> const& opt_cam_positions,
                       std::string              const& camera_offset_file) {

  vw::vw_out() << "Writing: " << camera_offset_file << std::endl;
  std::ofstream ofs(camera_offset_file.c_str());
  ofs.precision(8);
  ofs << "# Per-image absolute horizontal and vertical change in camera center (meters)\n";
  
  // Loop through the cameras and find the change in their centers
  for (size_t icam = 0; icam < orig_cam_positions.size(); icam++) {
    vw::Vector3 orig_ctr = orig_cam_positions[icam];
    vw::Vector3 opt_ctr  = opt_cam_positions [icam];
    
    vw::Vector3 llh = datum.cartesian_to_geodetic(orig_ctr);
    vw::Matrix3x3 NedToEcef = datum.lonlat_to_ned_matrix(llh);
    vw::Matrix3x3 EcefToNed = vw::math::inverse(NedToEcef);
    vw::Vector3 NedDir = EcefToNed * (opt_ctr - orig_ctr);
    
    // Find horizontal and vertical change
    double horiz_change = norm_2(subvector(NedDir, 0, 2));
    double vert_change  = std::abs(NedDir[2]);
    
    ofs << image_files[icam] << " " << horiz_change << " " << vert_change << std::endl;
  }
  ofs.close();

  return;
}

// This is used in jitter_solve
void compute_residuals(asp::BaBaseOptions const& opt,
                       ceres::Problem & problem,
                       // Output
                       std::vector<double> & residuals) {

  double cost = 0.0;
  ceres::Problem::EvaluateOptions eval_options;
  eval_options.apply_loss_function = false;
  if (opt.single_threaded_cameras)
    eval_options.num_threads = 1; // ISIS must be single threaded!
  else
    eval_options.num_threads = opt.num_threads;
  
  problem.Evaluate(eval_options, &cost, &residuals, 0, 0);
}

void saveJitterResiduals(ceres::Problem                             & problem, 
                         std::string                           const& residual_prefix,
                         asp::BaBaseOptions                    const& opt,
                         vw::ba::ControlNetwork                const& cnet,
                         asp::CRNJ                             const& crn,
                         vw::cartography::Datum                const& datum,
                         std::vector<double>                   const& tri_points_vec,
                         std::set<int>                         const& outliers,
                         std::vector<double>                   const& weight_per_residual,
                         std::vector<std::vector<vw::Vector2>> const& pixel_vec,
                         std::vector<std::vector<double>>      const& weight_vec,
                         std::vector<std::vector<int>>         const& isAnchor_vec,
                         std::vector<std::vector<int>>         const& pix2xyz_index) {
  
  // Compute the residuals before optimization
  std::vector<double> residuals;
  compute_residuals(opt, problem, residuals);
  if (residuals.size() != weight_per_residual.size()) 
    vw_throw(ArgumentErr() << "There must be as many residuals as weights for them.\n");

  //  Find the mean of all residuals corresponding to the same xyz point
  int num_tri_points = cnet.size();
  std::vector<double> mean_pixel_residual_norm(num_tri_points, 0.0);
  std::vector<int>    pixel_residual_count(num_tri_points, 0);
  std::vector<double> xyz_residual_norm; // This is unfinished logic
  
  int ires = 0;
  for (int icam = 0; icam < (int)crn.size(); icam++) {
    for (auto fiter = crn[icam].begin(); fiter != crn[icam].end(); fiter++) {
      
      // The index of the 3D point
      int ipt = (**fiter).m_point_id;
      
      if (outliers.find(ipt) != outliers.end())
        continue; // Skip outliers

      // Norm of pixel residual
      double norm = norm_2(Vector2(residuals[ires + 0] / weight_per_residual[ires + 0],
                                   residuals[ires + 1] / weight_per_residual[ires + 1]));

      mean_pixel_residual_norm[ipt] += norm;
      pixel_residual_count[ipt]++;
      
      ires += PIXEL_SIZE; // Update for the next iteration
    }
  }

  // Average all pixel residuals for a given xyz
  for (int ipt = 0; ipt < num_tri_points; ipt++) {
    if (outliers.find(ipt) != outliers.end() || pixel_residual_count[ipt] <= 0)
      continue; // Skip outliers
    mean_pixel_residual_norm[ipt] /= pixel_residual_count[ipt];
  }

  // Save the residuals
  write_per_xyz_pixel_residuals(cnet, residual_prefix, datum, outliers,  
                                tri_points_vec, mean_pixel_residual_norm,  
                                pixel_residual_count);

  // Add residuals for anchor points. That is pass 1 from
  // addReprojectionErrors(). We imitate here the same logic for that
  // pass. We continue to increment the ires counter from above.
  std::vector<Vector3> anchor_xyz;
  std::vector<double> anchor_residual_norm;
  for (int pass = 1; pass < 2; pass++) {
    for (int icam = 0; icam < (int)crn.size(); icam++) {
      for (size_t ipix = 0; ipix < pixel_vec[icam].size(); ipix++) {

        Vector2 observation =  pixel_vec[icam][ipix];
        double weight = weight_vec[icam][ipix];
        bool isAnchor = isAnchor_vec[icam][ipix];

        // Pass 0 is without anchor points, while pass 1 uses them.
        // Here we only do pass 1.
        if ((int)isAnchor != pass) 
          continue;

        // Norm of pixel residual
        double norm = norm_2(Vector2(residuals[ires + 0] / weight_per_residual[ires + 0],
                                     residuals[ires + 1] / weight_per_residual[ires + 1]));
        norm /= weight; // Undo the weight, to recover the pixel norm
        
        ires += PIXEL_SIZE; // Update for the next iteration

        double const* tri_point = &tri_points_vec[3 * pix2xyz_index[icam][ipix]];
        Vector3 xyz(tri_point[0], tri_point[1], tri_point[2]);
        anchor_xyz.push_back(xyz);
        anchor_residual_norm.push_back(norm);
      }
    }
  }
  write_anchor_residuals(residual_prefix, datum, anchor_xyz, anchor_residual_norm);
  
  // Ensure we did not process more residuals than what we have.
  // Likely there are more residuals than what we handled now.
  if (ires > (int)residuals.size())
    vw_throw(ArgumentErr() << "More residuals found than expected.\n");

  return;
}

// This is used in jitter_solve. There can be more tri points than in cnet,
// because we add anchor points. Those do not get processed here.
void write_per_xyz_pixel_residuals(vw::ba::ControlNetwork const& cnet,
                                   std::string            const& residual_prefix,
                                   vw::cartography::Datum const& datum,
                                   std::set<int>          const& outliers,
                                   std::vector<double>    const& tri_points_vec,
                                   std::vector<double>    const& mean_pixel_residual_norm,
                                   std::vector<int>       const& pixel_residual_count) {
    
  std::string map_prefix = residual_prefix + "_pointmap";
  std::string output_path = map_prefix + ".csv";

  int num_tri_points = cnet.size();
  
  // Open the output file and write the header. TODO(oalexan1): See
  // if it is possible to integrate this with the analogous
  // bundle_adjust function.
  vw_out() << "Writing: " << output_path << std::endl;

  std::ofstream file;
  file.open(output_path.c_str());
  file.precision(17);
  file << "# lon, lat, height_above_datum, mean_residual, num_observations\n";
  file << "# " << datum << std::endl;

  // Write all the points to the file
  for (int ipt = 0; ipt < num_tri_points; ipt++) {

    if (outliers.find(ipt) != outliers.end() || pixel_residual_count[ipt] <= 0)
      continue; // Skip outliers
    
    // The final GCC coordinate of this point
    const double * tri_point = &tri_points_vec[0] + ipt * NUM_XYZ_PARAMS;
    Vector3 xyz(tri_point[0], tri_point[1], tri_point[2]);
    Vector3 llh = datum.cartesian_to_geodetic(xyz);
    
    std::string comment = "";
    if (cnet[ipt].type() == vw::ba::ControlPoint::GroundControlPoint)
      comment = " # GCP";
    file << llh[0] << ", " << llh[1] <<", " << llh[2] << ", "
         << mean_pixel_residual_norm[ipt] << ", "
         << pixel_residual_count[ipt] << comment << std::endl;
  }
  file.close();
}

// This is used in jitter_solve
void write_anchor_residuals(std::string              const& residual_prefix,
                            vw::cartography::Datum   const& datum,
                            std::vector<vw::Vector3> const& anchor_xyz,
                            std::vector<double>      const& anchor_residual_norm) {
  
  std::string map_prefix = residual_prefix + "_anchor_points";
  std::string output_path = map_prefix + ".csv";
  vw_out() << "Writing: " << output_path << std::endl;
  std::ofstream file;
  file.open(output_path.c_str());
  file.precision(17);
  file << "# lon, lat, height_above_datum, anchor_residual_pixel_norm\n";
  file << "# " << datum << std::endl;

  for (size_t anchor_it = 0; anchor_it < anchor_xyz.size(); anchor_it++) {
    Vector3 llh = datum.cartesian_to_geodetic(anchor_xyz[anchor_it]);
    file << llh[0] <<", "<< llh[1] << ", " << llh[2] << ", "
         << anchor_residual_norm[anchor_it] << std::endl;
  }
  
  file.close();
}

} // end namespace asp 
