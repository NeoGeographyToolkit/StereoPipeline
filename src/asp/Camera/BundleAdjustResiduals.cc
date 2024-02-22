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
  size_t total_num_cam_params   = param_storage.num_cameras()*param_storage.params_per_camera();
  for (size_t i=0; i<param_storage.num_cameras(); i++)
    num_expected_residuals += cam_residual_counts[i]*PIXEL_SIZE;
  if (opt.camera_weight > 0)
    num_expected_residuals += total_num_cam_params;
  if (opt.rotation_weight > 0 || opt.translation_weight > 0)
    num_expected_residuals += total_num_cam_params;
  num_expected_residuals += num_uncertainty_residuals * PIXEL_SIZE;
  num_expected_residuals += reference_vec.size() * PIXEL_SIZE;
  
  if (num_expected_residuals != num_residuals)
    vw_throw(LogicErr() << "Expected " << num_expected_residuals
                        << " residuals but instead got " << num_residuals);
}

/// Compute residual map by averaging all the reprojection error at a given point
void compute_mean_residuals_at_xyz(vw::ba::CameraRelationNetwork<vw::ba::JFeature> const& crn,
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
                         std::vector<vw::Vector3> const& reference_vec,
                         vw::ba::ControlNetwork const& cnet, 
                         vw::ba::CameraRelationNetwork<vw::ba::JFeature> const& crn, 
                         ceres::Problem &problem) {

  std::vector<double> residuals;
  asp::compute_residuals(apply_loss_function, opt, param_storage,
                    cam_residual_counts, num_gcp_or_dem_residuals, 
                    num_uncertainty_residuals,
                    num_tri_residuals,
                    reference_vec, problem,
                    // Output
                    residuals);
    
  const size_t num_residuals = residuals.size();

  const std::string residual_path               = residual_prefix + "_stats.txt";
  const std::string residual_raw_pixels_path    = residual_prefix + "_raw_pixels.txt";
  const std::string residual_raw_gcp_path       = residual_prefix + "_raw_gcp.txt";
  const std::string residual_raw_cams_path      = residual_prefix + "_raw_cameras.txt";
  const std::string residual_reference_xyz_path = residual_prefix + "_reference_terrain.txt";

  // Write a report on residual errors
  std::ofstream residual_file, residual_file_raw_pixels, residual_file_raw_gcp,
    residual_file_raw_cams, residual_file_reference_xyz;
  vw_out() << "Writing: " << residual_path << std::endl;
  vw_out() << "Writing: " << residual_raw_pixels_path << std::endl;
  vw_out() << "Writing: " << residual_raw_gcp_path << std::endl;
  vw_out() << "Writing: " << residual_raw_cams_path << std::endl;
  
  residual_file.open(residual_path.c_str());
  residual_file.precision(17);
  residual_file_raw_pixels.open(residual_raw_pixels_path.c_str());
  residual_file_raw_pixels.precision(17);
  residual_file_raw_cams.open(residual_raw_cams_path.c_str());
  residual_file_raw_cams.precision(17);

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
  
  // List the GCP residuals
  if (num_gcp_or_dem_residuals > 0) {
    residual_file_raw_gcp.open(residual_raw_gcp_path.c_str());
    residual_file_raw_gcp.precision(17);
    residual_file << "GCP or DEM residual errors:\n";
    for (size_t i = 0; i < num_gcp_or_dem_residuals; i++) {
      double mean_residual = 0; // Take average of XYZ error for each point
      residual_file_raw_gcp << i;
      for (size_t j = 0; j < param_storage.params_per_point(); j++) {
        mean_residual += fabs(residuals[index]);
        residual_file_raw_gcp << ", " << residuals[index]; // Write all values in this file
        index++;
      }
      mean_residual /= static_cast<double>(param_storage.params_per_point());
      residual_file << i << ", " << mean_residual << std::endl;
      residual_file_raw_gcp << std::endl;
    }
    residual_file_raw_gcp.close();
  }
  
  // List the camera weight residuals
  int num_passes = int(opt.camera_weight > 0) +
    int(opt.rotation_weight > 0 || opt.translation_weight > 0);
  for (int pass = 0; pass < num_passes; pass++) {
    residual_file << "Camera weight position and orientation residual errors:\n";
    const size_t part_size = param_storage.params_per_camera()/2;
    for (size_t c=0; c<param_storage.num_cameras(); c++) {
      residual_file_raw_cams << opt.camera_files[c];
      // Separately compute the mean position and rotation error
      double mean_residual_pos = 0, mean_residual_rot = 0;
      for (size_t j = 0; j < part_size; j++) {
        mean_residual_pos += fabs(residuals[index]);
        residual_file_raw_cams << ", " << residuals[index]; // Write all values in this file
        index++;
      }
      for (size_t j = 0; j < part_size; j++) {
        mean_residual_rot += fabs(residuals[index]);
        residual_file_raw_cams << ", " << residuals[index]; // Write all values in this file
        index++;
      }
      mean_residual_pos /= static_cast<double>(part_size);
      mean_residual_rot /= static_cast<double>(part_size);
    
      residual_file << opt.camera_files[c] << ", " << mean_residual_pos << ", "
                    << mean_residual_rot << std::endl;
      residual_file_raw_cams << std::endl;
    }
  }
  residual_file_raw_cams.close();
  residual_file.close();

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
void saveTriOffsetsPerCamera(std::vector<std::string> const& image_files,
                             asp::BAParams const& param_storage,
                             vw::ba::ControlNetwork const& cnet,
                             vw::ba::CameraRelationNetwork<vw::ba::JFeature> const& crn,
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
        
      Vector3 initial_xyz = cnet[ipt].position();
      double const* point = param_storage.get_point_ptr(ipt);
      vw::Vector3 final_xyz(point[0], point[1], point[2]);
     
      // Append the norm of offset
      tri_offsets[icam].push_back(norm_2(final_xyz - initial_xyz));   
    }
  }
  
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
      median = vw::math::destructive_median(offsets);
      count = offsets.size();
    }
    ofs << image_files[icam] << " " << mean << " " << median << " " << count << "\n";  
  }
  ofs.close();
}

// Compute the horizontal and vertical change in camera positions
void saveCameraOffsets(vw::cartography::Datum   const& datum,
                       std::vector<std::string> const& image_files,
                       std::vector<vw::CamPtr>  const& orig_cams,
                       std::vector<vw::CamPtr>  const& opt_cams,
                       std::string              const& camera_offset_file) {

  vw::vw_out() << "Writing: " << camera_offset_file << std::endl;
  std::ofstream ofs(camera_offset_file.c_str());
  ofs.precision(8);
  ofs << "# Per-image absolute horizontal and vertical change in camera center (meters)\n";
  
  // Loop through the cameras and find the change in their centers
  for (size_t icam = 0; icam < orig_cams.size(); icam++) {
    vw::Vector3 orig_ctr = orig_cams[icam]->camera_center(vw::Vector2());
    vw::Vector3 opt_ctr  = opt_cams [icam]->camera_center(vw::Vector2());
    
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

} // end namespace asp 
