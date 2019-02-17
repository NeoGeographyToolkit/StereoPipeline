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


/// \file bundle_adjust.cc
///

#include <vw/Camera/CameraUtilities.h>
#include <vw/Core/CmdUtils.h>
#include <vw/BundleAdjustment/BundleAdjustReport.h>
#include <vw/BundleAdjustment/AdjustRef.h>
#include <asp/Core/Macros.h>
#include <asp/Sessions/StereoSession.h>
#include <asp/Sessions/StereoSessionFactory.h>
#include <asp/Core/StereoSettings.h>
#include <asp/Core/PointUtils.h>
#include <asp/Core/InterestPointMatching.h>
#include <asp/Core/EigenUtils.h>

#include <asp/Tools/bundle_adjust.h>

#include <xercesc/util/PlatformUtils.hpp>

namespace po = boost::program_options;
namespace fs = boost::filesystem;

using namespace vw;
using namespace vw::camera;
using namespace vw::ba;

typedef boost::scoped_ptr<asp::StereoSession> SessionPtr;

typedef CameraRelationNetwork<JFeature> CRNJ;

//=========================================================================

// Wrapper for the AdjustedCameraBundleModel type
void add_reprojection_residual_block(Vector2 const& observation, Vector2 const& pixel_sigma,
                                     int point_index, int camera_index,
                                     BAParamStorage & param_storage,
                                     Options const& opt,
                                     ceres::Problem & problem){

  ceres::LossFunction* loss_function = get_loss_function(opt);

  boost::shared_ptr<CameraModel> camera_model = opt.camera_models[camera_index];

  double* camera = param_storage.get_camera_ptr(camera_index);
  double* point  = param_storage.get_point_ptr (point_index );

  if (opt.camera_type == BaCameraType_Other) {
    // The generic camera case
    boost::shared_ptr<CeresBundleModelBase> wrapper(new AdjustedCameraBundleModel(camera_model));
      ceres::CostFunction* cost_function =
        BaReprojectionError::Create(observation, pixel_sigma, wrapper);
      problem.AddResidualBlock(cost_function, loss_function, point, camera);

  } else { // Pinhole and optical bar

    double* center     = param_storage.get_intrinsic_center_ptr    (camera_index);
    double* focus      = param_storage.get_intrinsic_focus_ptr     (camera_index);
    double* distortion = param_storage.get_intrinsic_distortion_ptr(camera_index);

    boost::shared_ptr<CeresBundleModelBase> wrapper;

    if (opt.camera_type == BaCameraType_Pinhole) {

      boost::shared_ptr<PinholeModel> pinhole_model = 
        boost::dynamic_pointer_cast<PinholeModel>(camera_model);
      if (pinhole_model.get() == 0)
        vw::vw_throw( vw::ArgumentErr() << "Tried to add pinhole block with non-pinhole camera!");
      wrapper.reset(new PinholeBundleModel(pinhole_model));

    } else { // Optical bar

      boost::shared_ptr<asp::camera::OpticalBarModel> bar_model = 
        boost::dynamic_pointer_cast<asp::camera::OpticalBarModel>(camera_model);
      if (bar_model.get() == 0)
        vw::vw_throw( vw::ArgumentErr() << "Tried to add optical bar block with non-OB camera!");
      wrapper.reset(new OpticalBarBundleModel(bar_model));
    }

    ceres::CostFunction* cost_function =
      BaReprojectionError::Create(observation, pixel_sigma, wrapper);
    problem.AddResidualBlock(cost_function, loss_function, point, camera, 
                            center, focus, distortion);

    // If we don't want to solve for something, just tell Ceres not to adjust the values.
    if (opt.intrinisc_options.center_constant)
      problem.SetParameterBlockConstant(center);
    if (opt.intrinisc_options.focus_constant)
      problem.SetParameterBlockConstant(focus);
    if (opt.intrinisc_options.distortion_constant)
      problem.SetParameterBlockConstant(distortion);
  } // End non-generic camera case.

  // Fix this camera if requested
  if (opt.fixed_cameras_indices.find(camera_index) != opt.fixed_cameras_indices.end()) 
    problem.SetParameterBlockConstant(param_storage.get_camera_ptr(camera_index));
}

/// Add residual block for the error using reference xyz.
void add_disparity_residual_block(Vector3 const& reference_xyz,
                                  ImageViewRef<DispPixelT> const& interp_disp, 
                                  int left_cam_index, int right_cam_index,
                                  BAParamStorage & param_storage,
                                  Options const& opt,
                                  ceres::Problem & problem){

  ceres::LossFunction* loss_function = get_loss_function(opt);

  boost::shared_ptr<CameraModel> left_camera_model  = opt.camera_models[left_cam_index ];
  boost::shared_ptr<CameraModel> right_camera_model = opt.camera_models[right_cam_index];

  const bool inline_adjustments = (opt.camera_type != BaCameraType_Other);

  // Get the list of residual pointers that will be passed to ceres.
  std::vector<double*> residual_ptrs;
  BaDispXyzError::get_residual_pointers(param_storage,
                                        left_cam_index, right_cam_index,
                                        inline_adjustments, opt.intrinisc_options,
                                        residual_ptrs);
 if (opt.camera_type == BaCameraType_Other) {

    boost::shared_ptr<CeresBundleModelBase> left_wrapper (new AdjustedCameraBundleModel(left_camera_model ));
    boost::shared_ptr<CeresBundleModelBase> right_wrapper(new AdjustedCameraBundleModel(right_camera_model));
    ceres::CostFunction* cost_function =
      BaDispXyzError::Create(reference_xyz, interp_disp, left_wrapper, right_wrapper,
                             inline_adjustments, opt.intrinisc_options);

    problem.AddResidualBlock(cost_function, loss_function, residual_ptrs);

  } else { // Pinhole or optical bar

    boost::shared_ptr<CeresBundleModelBase> left_wrapper, right_wrapper;

    if (opt.camera_type == BaCameraType_Pinhole) {
      boost::shared_ptr<PinholeModel> left_pinhole_model = 
        boost::dynamic_pointer_cast<vw::camera::PinholeModel>(left_camera_model);
      boost::shared_ptr<PinholeModel> right_pinhole_model = 
        boost::dynamic_pointer_cast<vw::camera::PinholeModel>(right_camera_model);

      left_wrapper.reset (new PinholeBundleModel(left_pinhole_model ));
      right_wrapper.reset(new PinholeBundleModel(right_pinhole_model));

    } else { // Optical bar
      boost::shared_ptr<asp::camera::OpticalBarModel> left_bar_model = 
        boost::dynamic_pointer_cast<asp::camera::OpticalBarModel>(left_camera_model);
      boost::shared_ptr<asp::camera::OpticalBarModel> right_bar_model = 
        boost::dynamic_pointer_cast<asp::camera::OpticalBarModel>(right_camera_model);

      left_wrapper.reset (new OpticalBarBundleModel(left_bar_model ));
      right_wrapper.reset(new OpticalBarBundleModel(right_bar_model));
    }

    ceres::CostFunction* cost_function =
      BaDispXyzError::Create(reference_xyz, interp_disp, left_wrapper, right_wrapper,
                             inline_adjustments, opt.intrinisc_options);
    problem.AddResidualBlock(cost_function, loss_function, residual_ptrs);

  }
  
} // End function add_disparity_residual_block


//----------------------------------------------------------------
// Residuals functions

/// Compute the residuals
void compute_residuals(bool apply_loss_function,
                       Options const& opt,
                       BAParamStorage const& param_storage,
                       std::vector<size_t> const& cam_residual_counts,
                       size_t num_gcp_residuals,
                       std::vector<vw::Vector3> const& reference_vec,
                       ceres::Problem &problem,
                       std::vector<double> & residuals // output
                       ) {
  // TODO: Associate residuals with cameras!
  // Generate some additional diagnostic info
  double cost = 0;
  ceres::Problem::EvaluateOptions eval_options;
  eval_options.apply_loss_function = apply_loss_function;
  if (opt.stereo_session_string == "isis")
    eval_options.num_threads = 1; // ISIS must be single threaded!
  else
    eval_options.num_threads = opt.num_threads;
  problem.Evaluate(eval_options, &cost, &residuals, 0, 0);
  const size_t num_residuals = residuals.size();
  
  // Verify our residual calculations are correct
  size_t num_expected_residuals = num_gcp_residuals*param_storage.params_per_point();
  size_t total_num_cam_params   = param_storage.num_cameras()*param_storage.params_per_camera();
  for (size_t i=0; i<param_storage.num_cameras(); ++i)
    num_expected_residuals += cam_residual_counts[i]*PIXEL_SIZE;
  if (opt.camera_weight > 0)
    num_expected_residuals += total_num_cam_params;
  if (opt.rotation_weight > 0 || opt.translation_weight > 0)
    num_expected_residuals += total_num_cam_params;
  num_expected_residuals += reference_vec.size() * PIXEL_SIZE;
  
  if (num_expected_residuals != num_residuals)
    vw_throw( LogicErr() << "Expected " << num_expected_residuals
                         << " residuals but instead got " << num_residuals);
}

/// Compute residual map by averaging all the reprojection error at a given point
void compute_mean_residuals_at_xyz(CRNJ & crn,
                                  std::vector<double> const& residuals,
                                  BAParamStorage const& param_storage,
                                  // outputs
                                  std::vector<double> & mean_residuals,
                                  std::vector<int>  & num_point_observations
                                  ) {

  mean_residuals.resize(param_storage.num_points());
  num_point_observations.resize(param_storage.num_points());
  
  // Observation residuals are stored at the beginning of the residual vector in the 
  //  same order they were originally added to Ceres.
  
  size_t residual_index = 0;
  // Double loop through cameras and crn entries will give us the correct order
  for ( size_t icam = 0; icam < param_storage.num_cameras(); icam++ ) {
    typedef CameraNode<JFeature>::const_iterator crn_iter;
    for ( crn_iter fiter = crn[icam].begin(); fiter != crn[icam].end(); fiter++ ){

      // The index of the 3D point
      int ipt = (**fiter).m_point_id;

      if (param_storage.get_point_outlier(ipt))
        continue; // skip outliers

      // Get the residual error for this observation
      double errorX         = residuals[residual_index  ];
      double errorY         = residuals[residual_index+1];
      double residual_error = (fabs(errorX) + fabs(errorY)) / 2;
      residual_index += PIXEL_SIZE;

      // Update information for this point
      num_point_observations[ipt] += 1;
      mean_residuals        [ipt] += residual_error;
    }
  } // End double loop through all the observations

  // Do the averaging
  for (size_t i = 0; i < param_storage.num_points(); ++i) {
    if (param_storage.get_point_outlier(i)) {
      // Skip outliers. But initialize to something.
      mean_residuals        [i] = std::numeric_limits<double>::quiet_NaN();
      num_point_observations[i] = std::numeric_limits<double>::quiet_NaN();
      continue;
    }
    mean_residuals[i] /= static_cast<double>(num_point_observations[i]);
  }
  
} // End function compute_mean_residuals_at_xyz

/// Write out a .csv file recording the residual error at each location on the ground
void write_residual_map(std::string const& output_prefix,
                        std::vector<double> const& mean_residuals, // Mean residual of each point
                        std::vector<int   > const& num_point_observations, // Num non-outlier pixels per point
                        BAParamStorage const& param_storage,
                        ControlNetwork const& cnet,
                        Options const& opt) {

  std::string output_path = output_prefix + "_point_log.csv";

  if (opt.datum.name() == UNSPECIFIED_DATUM) {
    vw_out(WarningMessage) << "No datum specified, can't write file: " << output_path << std::endl;
    return;
  }
  if (mean_residuals.size() != param_storage.num_points())
    vw_throw( LogicErr() << "Point count mismatch in write_residual_map.\n");

  if (cnet.size() != param_storage.num_points()) 
    vw_throw( LogicErr()
              << "The number of stored points "
              << "does not agree with number of points in cnet.\n");
  
  // Open the output file and write the header
  //vw_out() << "Writing: " << output_path << std::endl;
  
  std::ofstream file;
  file.open(output_path.c_str()); file.precision(18);
  file << "# lon, lat, height_above_datum, mean_residual, num_observations\n";
  file << "# " << opt.datum << std::endl;
  
  // Now write all the points to the file
  for (size_t i = 0; i < param_storage.num_points(); ++i) {

    if (param_storage.get_point_outlier(i))
      continue; // skip outliers
    
      // The final GCC coordinate of this point
      const double * point = param_storage.get_point_ptr(i);
      Vector3 xyz(point[0], point[1], point[2]);

      Vector3 llh = opt.datum.cartesian_to_geodetic(xyz);

      std::string comment = "";
      if (cnet[i].type() == ControlPoint::GroundControlPoint)
        comment = " # GCP";
      file << llh[0] <<", "<< llh[1] <<", "<< llh[2] <<", "<< mean_residuals[i] <<", "
           << num_point_observations[i] << comment << std::endl;
  }
  file.close();

} // End function write_residual_map


/// Write log files describing all residual errors. The order of data stored
/// in residuals must mirror perfectly the way residuals were created. 
void write_residual_logs(std::string const& residual_prefix, bool apply_loss_function,
                         Options const& opt,
                         BAParamStorage const& param_storage,
                         std::vector<size_t> const& cam_residual_counts,
                         size_t num_gcp_residuals, 
                         std::vector<vw::Vector3> const& reference_vec,
                         ControlNetwork const& cnet, CRNJ & crn, 
                         ceres::Problem &problem) {
  
  std::vector<double> residuals;
  compute_residuals(apply_loss_function, opt, param_storage,
                    cam_residual_counts,  num_gcp_residuals, reference_vec, problem,  
                    residuals // output
                    );
    
  const size_t num_residuals = residuals.size();

  const std::string residual_path               = residual_prefix + "_averages.txt";
  const std::string residual_raw_pixels_path    = residual_prefix + "_raw_pixels.txt";
  const std::string residual_raw_gcp_path       = residual_prefix + "_raw_gcp.txt";
  const std::string residual_raw_cams_path      = residual_prefix + "_raw_cameras.txt";
  const std::string residual_reference_xyz_path = residual_prefix + "_reference_terrain.txt";

  // Write a report on residual errors
  std::ofstream residual_file, residual_file_raw_pixels, residual_file_raw_gcp,
    residual_file_raw_cams, residual_file_reference_xyz;
  //vw_out() << "Writing: " << residual_path << std::endl;
  //vw_out() << "Writing: " << residual_raw_pixels_path << std::endl;
  //vw_out() << "Writing: " << residual_raw_gcp_path << std::endl;
  //vw_out() << "Writing: " << residual_raw_cams_path << std::endl;
  
  residual_file.open(residual_path.c_str());
  residual_file.precision(18);
  residual_file_raw_pixels.open(residual_raw_pixels_path.c_str());
  residual_file_raw_pixels.precision(18);
  residual_file_raw_cams.open(residual_raw_cams_path.c_str());
  residual_file_raw_cams.precision(18);

  if (reference_vec.size() > 0) {
    //vw_out() << "Writing: " << residual_reference_xyz_path << std::endl;
    residual_file_reference_xyz.open(residual_reference_xyz_path.c_str());
    residual_file_reference_xyz.precision(18);
  }
  
  size_t index = 0;
  // For each camera, average together all the point observation residuals
  residual_file << "Mean residual error and point count for cameras:\n";
  for (size_t c = 0; c < param_storage.num_cameras(); ++c) {
    size_t num_this_cam_residuals = cam_residual_counts[c];
    
    // Write header for the raw file
    residual_file_raw_pixels << opt.camera_files[c] << ", " << num_this_cam_residuals << std::endl;
    
    double mean_residual = 0; // Take average of all pixel coord errors
    for (size_t i=0; i<num_this_cam_residuals; ++i) {
      double ex = residuals[index];
      ++index;
      double ey = residuals[index];
      ++index;
      mean_residual += fabs(ex) + fabs(ey);
      
      residual_file_raw_pixels << ex << ", " << ey << std::endl; // Write ex, ey on raw file
    }
    // Write line for the summary file
    mean_residual /= static_cast<double>(num_this_cam_residuals);
    residual_file << opt.camera_files[c] << ", " << mean_residual << ", "
                  << num_this_cam_residuals << std::endl;
  }
  residual_file_raw_pixels.close();
  
  // List the GCP residuals
  if (num_gcp_residuals > 0) {
    residual_file_raw_gcp.open(residual_raw_gcp_path.c_str());
    residual_file_raw_gcp.precision(18);
    residual_file << "GCP residual errors:\n";
    for (size_t i=0; i<num_gcp_residuals; ++i) {
      double mean_residual = 0; // Take average of XYZ error for each point
      residual_file_raw_gcp << i;
      for (size_t j=0; j<param_storage.params_per_point(); ++j) {
        mean_residual += fabs(residuals[index]);
        residual_file_raw_gcp << ", " << residuals[index]; // Write all values in this file
        ++index;
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
    for (size_t c=0; c<param_storage.num_cameras(); ++c) {
      residual_file_raw_cams << opt.camera_files[c];
      // Separately compute the mean position and rotation error
      double mean_residual_pos = 0, mean_residual_rot = 0;
      for (size_t j=0; j<part_size; ++j) {
        mean_residual_pos += fabs(residuals[index]);
        residual_file_raw_cams << ", " << residuals[index]; // Write all values in this file
        ++index;
      }
      for (size_t j=0; j<part_size; ++j) {
        mean_residual_rot += fabs(residuals[index]);
        residual_file_raw_cams << ", " << residuals[index]; // Write all values in this file
        ++index;
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

  // List residuals for matching input terrain (lidar)
  if (reference_vec.size() > 0) {
    residual_file << "reference terrain residual errors:\n";
    residual_file_reference_xyz << "# lon, lat, height_above_datum, pixel_error_norm\n";
    for (size_t i = 0; i < reference_vec.size(); ++i) {

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
  
  if (index != num_residuals)
    vw_throw( LogicErr() << "Have " << num_residuals << " residuals but iterated through " << index);

  // Generate the location based file
  std::string map_prefix = residual_prefix + "_pointmap";
  std::vector<double> mean_residuals;
  std::vector<int   > num_point_observations;
  compute_mean_residuals_at_xyz(crn,  residuals,  param_storage,
                                mean_residuals, num_point_observations);

  write_residual_map(map_prefix, mean_residuals, num_point_observations,
                     param_storage, cnet, opt);

} // End function write_residual_logs


// End residual functions
// ----------------------------------------------------------------

// ----------------------------------------------------------------
// Start outlier functions

/// Add to the outliers based on the large residuals
int update_outliers(ControlNetwork   & cnet,
                    CRNJ & crn,
                    BAParamStorage & param_storage,
                    Options const& opt,
                    std::vector<size_t> const& cam_residual_counts,
                    size_t num_gcp_residuals,
                    std::vector<vw::Vector3> const& reference_vec, 
                    ceres::Problem &problem) {
  
  vw_out() << "Removing pixel outliers in preparation for another solver attempt.\n";

  const size_t num_points  = param_storage.num_points();
  const size_t num_cameras = param_storage.num_cameras();
  
  // Compute the reprojection error. Hence we should not add the contribution
  // of the loss function.
  bool apply_loss_function = false;
  std::vector<double> residuals;
  compute_residuals(apply_loss_function,  
                    opt, param_storage,  cam_residual_counts,  
                    num_gcp_residuals, reference_vec, problem,
                    residuals // output
                   );

  // Compute the mean residual at each xyz, and how many times that residual is seen
  std::vector<double> mean_residuals;
  std::vector<int   > num_point_observations;
  compute_mean_residuals_at_xyz(crn,  residuals,  param_storage,
                                // outputs
                                mean_residuals, num_point_observations);

  // The number of mean residuals is the same as the number of points,
  // of which some are outliers. Hence need to collect only the
  // non-outliers so far to be able to remove new outliers.  Need to
  // follow the same logic as when residuals were formed. And also ignore GCP.
  std::vector<double> actual_residuals;
  std::set<int> was_added;
  for ( size_t icam = 0; icam < num_cameras; icam++ ) {
    typedef CameraNode<JFeature>::const_iterator crn_iter;
    for ( crn_iter fiter = crn[icam].begin(); fiter != crn[icam].end(); fiter++ ){

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
  vw_out() << "Outlier statistics: b = " << b << ", e = " << e << ".\n";
  
  // If this is too aggressive, the user can tame it. It is
  // unreasonable to throw out pixel residuals as small as 1 or 2
  // pixels.  We will not use the b, because the residuals start at 0.
  // - "max_pix" sets the minimum error that can be thrown out.
  e = std::min(std::max(e, max_pix1), max_pix2);

  vw_out() << "Removing as outliers points with mean reprojection error > " << e << ".\n";
  
  // Add to the outliers by reprojection error. Must repeat the same logic as above. 
  int num_outliers_by_reprojection = 0;
  for ( size_t icam = 0; icam < num_cameras; icam++ ) {
    typedef CameraNode<JFeature>::const_iterator crn_iter;
    for ( crn_iter fiter = crn[icam].begin(); fiter != crn[icam].end(); fiter++ ){

      // The index of the 3D point
      int ipt = (**fiter).m_point_id;

      // skip existing outliers
      if (param_storage.get_point_outlier(ipt))
        continue; 

      // Skip gcp
      if (cnet[ipt].type() == ControlPoint::GroundControlPoint)
        continue;

      if (mean_residuals[ipt] > e) {
        //vw_out() << "Removing " << ipt << " with residual " << mean_residuals[ipt] << std::endl;
        param_storage.set_point_outlier(ipt, true);
        ++num_outliers_by_reprojection;
      }
    }
  } // End double loop through all the observations
  vw_out() << "Removed " << num_outliers_by_reprojection << " outliers by reprojection error.\n";

  // Remove outliers by elevation limit
  int num_outliers_by_elevation = 0;
  if ( opt.elevation_limit[0] < opt.elevation_limit[1] ) {
    
    for (size_t ipt = 0; ipt < param_storage.num_points(); ipt++) {

      if (cnet[ipt].type() == ControlPoint::GroundControlPoint)
        continue; // don't filter out GCP
      if (param_storage.get_point_outlier(ipt))
        continue; // skip outliers
      
      // The GCC coordinate of this point
      const double * point = param_storage.get_point_ptr(ipt);
      Vector3 xyz(point[0], point[1], point[2]);
      Vector3 llh = opt.datum.cartesian_to_geodetic(xyz);
      // Below check for NaN as well just in case
      if (llh[2] != llh[2] || llh[2] < opt.elevation_limit[0] || llh[2] > opt.elevation_limit[1]) {
        param_storage.set_point_outlier(ipt, true);
        num_outliers_by_elevation++;
      }
    }
    vw_out() << "Removed " << num_outliers_by_elevation << " outliers by elevation range.\n";
  }

  int num_remaining_points = num_points - param_storage.get_num_outliers();
  vw_out() << "Now have " << num_remaining_points << " points remaining.\n";

  return num_outliers_by_reprojection + num_outliers_by_elevation;
}

// TODO: At least part of this should be a class function??
/// Remove the outliers flagged earlier
void remove_outliers(ControlNetwork const& cnet, BAParamStorage &param_storage,
                     Options const& opt){

  // Work on individual image pairs
  typedef std::map< std::pair<int, int>, std::string>::const_iterator match_type;
  for (match_type match_it = opt.match_files.begin(); match_it != opt.match_files.end();
       match_it++){

    // IP from the control network, for which we flagged outliers
    std::vector<vw::ip::InterestPoint> left_ip, right_ip;
      
    std::pair<int, int> cam_pair   = match_it->first;
    std::string         match_file = match_it->second;
    size_t left_cam  = cam_pair.first;
    size_t right_cam = cam_pair.second;

    // Read the original IP, to ensure later we write to disk only
    // the subset of the IP from the control network which
    // are part of these original ones. 
    std::vector<ip::InterestPoint> orig_left_ip, orig_right_ip;
    ip::read_binary_match_file(match_file, orig_left_ip, orig_right_ip);
    std::map< std::pair<double, double>, std::pair<double, double> > lookup;
    for (size_t ip_iter = 0; ip_iter < orig_left_ip.size(); ip_iter++) {
      lookup [ std::pair<double, double>(orig_left_ip[ip_iter].x, orig_left_ip[ip_iter].y) ]
        =  std::pair<double, double>(orig_right_ip[ip_iter].x, orig_right_ip[ip_iter].y);
    }

    // TODO: ???
    // Iterate over the control network, and, for each control point,
    // look only at the measure for left_cam and right_cam
    int ipt = -1;
    for ( ControlNetwork::const_iterator iter = cnet.begin();
      iter != cnet.end(); ++iter ) {

      ipt++; // control point index

      // Skip gcp
      if (cnet[ipt].type() == ControlPoint::GroundControlPoint) {
        continue;
      }
        
      bool has_left = true, has_right = false;
      ip::InterestPoint lip, rip;
      for ( ControlPoint::const_iterator measure = (*iter).begin();
            measure != (*iter).end(); ++measure ) {
        if (measure->image_id() == left_cam) {
          has_left = true;
          lip = ip::InterestPoint( measure->position()[0], measure->position()[1],
                                   measure->sigma()[0] );
        }else if (measure->image_id() == right_cam) {
          has_right = true;
          rip = ip::InterestPoint( measure->position()[0], measure->position()[1],
                                   measure->sigma()[0] );
        }
      }

      // Keep only ip for these two images
      if (!has_left || !has_right)
        continue;

      if (param_storage.get_point_outlier(ipt))
        continue; // skip outliers

      // Only add ip that were there originally
      std::pair<double, double> left (lip.x, lip.y);
      std::pair<double, double> right(rip.x, rip.y);
      if (lookup.find(left) == lookup.end() || lookup[left] != right)
        continue;
      
      left_ip.push_back (lip);
      right_ip.push_back(rip);
    }
    
    // Filter by disparity
    asp::filter_ip_by_disparity(opt.remove_outliers_by_disp_params[0],
                                opt.remove_outliers_by_disp_params[1],
                                left_ip, right_ip);
      
    if ( param_storage.num_cameras() == 2 ){
      // Compute the coverage fraction
      Vector2i right_image_size = file_image_size(opt.image_files[1]);
      int right_ip_width = right_image_size[0]*
          static_cast<double>(100-opt.ip_edge_buffer_percent)/100.0;
      Vector2i ip_size(right_ip_width, right_image_size[1]);
      double ip_coverage = asp::calc_ip_coverage_fraction(right_ip, ip_size);
      // Careful with the line below, it gets used in process_icebridge_batch.py.
      vw_out() << "IP coverage fraction after cleaning = " << ip_coverage << "\n";
    }

    // Make a clean copy of the file
    match_file = fs::path(match_file).replace_extension("").string();
    match_file += "-clean.match";
    
    vw_out() << "Writing: " << match_file << std::endl;
    ip::write_binary_match_file(match_file, left_ip, right_ip);
  }
}

// End outlier functions
// ----------------------------------------------------------------

int do_ba_ceres_one_pass(Options             & opt,
                         CRNJ                & crn,
                         bool                  first_pass,
                         bool                  last_pass,
                         BAParamStorage      & param_storage, 
                         BAParamStorage const& orig_parameters,
                         bool                & convergence_reached,
                         double              & final_cost){

  ceres::Problem problem;

  ControlNetwork & cnet = *opt.cnet;
  const int num_cameras = param_storage.num_cameras();
  const int num_points  = param_storage.num_points();

  convergence_reached = true;
  
  // Add the cost function component for difference of pixel observations
  // - Reduce error by making pixel projection consistent with observations.

  // How many times an xyz point shows up in the problem
  std::vector<int> count_map(num_points);
  for (int i=0; i<num_points; ++i) {
    if (param_storage.get_point_outlier(i))
      count_map[i] = 0; // skip outliers
    else
      count_map[i] = cnet[i].size(); // Get number of observations of this point.
  }

  // We will optimize multipliers of the intrinsics. This way
  // each intrinsic changes by a scale specific to it.
  // Note: If an intrinsic starts as 0, it will then stay as 0. This is documented.
  // Can be both useful and confusing.
  
  vw::cartography::GeoReference dem_georef;
  ImageViewRef< PixelMask<double> >  interp_dem;
  if (opt.heights_from_dem != "") 
    create_interp_dem(opt.heights_from_dem, dem_georef, interp_dem);
  
  // TODO: Stop using the CRN, store residual blocks in point-major order?
  
  // Add the various cost functions the solver will optimize over.
  std::vector<size_t> cam_residual_counts(num_cameras);
  typedef CameraNode<JFeature>::iterator crn_iter;
  for ( int icam = 0; icam < num_cameras; icam++ ) { // Camera loop
    cam_residual_counts[icam] = 0;
    for ( crn_iter fiter = crn[icam].begin(); fiter != crn[icam].end(); fiter++ ){ // IP loop

      // The index of the 3D point this IP is for.
      int ipt = (**fiter).m_point_id;
      if (param_storage.get_point_outlier(ipt))
        continue; // skip outliers

      VW_ASSERT(int(icam) < num_cameras,
                ArgumentErr() << "Out of bounds in the number of cameras");
      VW_ASSERT(int(ipt)  < num_points,
                ArgumentErr() << "Out of bounds in the number of points");

      // The observed value for the projection of point with index ipt into
      // the camera with index icam.
      Vector2 observation = (**fiter).m_location;
      Vector2 pixel_sigma = (**fiter).m_scale;

      // This is a bugfix
      if (pixel_sigma != pixel_sigma) // nan check
        pixel_sigma = Vector2(1, 1);

      double p = opt.overlap_exponent;
      if (p > 0 && count_map[ipt] > 2) {
        // Give more weight to points that are seen in more images.
        // This should not be overused. 
        double delta = pow(count_map[ipt] - 1.0, p);
        pixel_sigma /= delta;
      }

      // Call function to add the appropriate Ceres residual block.
      add_reprojection_residual_block(observation, pixel_sigma, ipt, icam,
                                      param_storage, opt, problem);

      if (opt.heights_from_dem != "") {
        // For non-GCP points, copy the heights for xyz points from the DEM.
        // Fix the obtained xyz points as they are considered reliable
        // and we should have the cameras and intrinsics params to conform to these.
        if (cnet[ipt].type() != ControlPoint::GroundControlPoint){
          double* point = param_storage.get_point_ptr(ipt);
          update_point_from_dem(point, dem_georef, interp_dem);
          if (opt.heights_from_dem_weight <= 0) 
            problem.SetParameterBlockConstant(point);
          else{
            // Make this into a GCP so we can float it while not deviating
            // too much from what we have now.
            cnet[ipt].set_type(ControlPoint::GroundControlPoint);
            double s = 1.0/opt.heights_from_dem_weight;
            cnet[ipt].set_position(Vector3(point[0], point[1], point[2]));
            cnet[ipt].set_sigma(Vector3(s, s, s));
          }
        }
      }

      cam_residual_counts[icam] += 1; // Track the number of residual blocks for each camera
    } // end iterating over points
  } // end iterating over cameras

  // Add ground control points
  // - Error goes up as GCP's move from their input positions.
  int    num_gcp = 0;
  size_t num_gcp_residuals = 0;
  for (int ipt = 0; ipt < num_points; ipt++){
    if (cnet[ipt].type() != ControlPoint::GroundControlPoint)
      continue; // Skip non-GCP's

    if (param_storage.get_point_outlier(ipt))
      continue; // skip outliers
    
    num_gcp++;
    
    Vector3 observation = cnet[ipt].position();
    Vector3 xyz_sigma   = cnet[ipt].sigma();

    ceres::CostFunction* cost_function;
    if (!opt.use_llh_error) 
      cost_function = XYZError::Create(observation, xyz_sigma);
    else{
      Vector3 llh_sigma = xyz_sigma;
      // make lat,lon into lon,lat
      std::swap(llh_sigma[0], llh_sigma[1]);
      cost_function = LLHError::Create(observation, llh_sigma, opt.datum);
    }

    // Don't use the same loss function as for pixels since that one discounts
    //  outliers and the cameras should never be discounted.
    ceres::LossFunction* loss_function = new ceres::TrivialLoss();

    double * point  = param_storage.get_point_ptr(ipt);
    problem.AddResidualBlock(cost_function, loss_function, point);
    ++num_gcp_residuals;

    if (opt.fix_gcp_xyz) 
      problem.SetParameterBlockConstant(point);
  } // End loop through GCP's

  // Add camera constraints
  // - Error goes up as cameras move and rotate from their input positions.
  if (opt.camera_weight > 0){

    for (int icam = 0; icam < num_cameras; icam++){

      double const* orig_cam_ptr = orig_parameters.get_camera_ptr(icam);
      ceres::CostFunction* cost_function = CamError::Create(orig_cam_ptr, opt.camera_weight);

      // Don't use the same loss function as for pixels since that one discounts
      //  outliers and the cameras should never be discounted.
      ceres::LossFunction* loss_function = new ceres::TrivialLoss();

      double * camera  = param_storage.get_camera_ptr(icam);
      problem.AddResidualBlock(cost_function, loss_function, camera);
    } // End loop through cameras.
  }

  // Finer level control of only rotation and translation.
  // This will need to be merged with the above but note that the loss is NULL here. 
  // - Error goes up as cameras move and rotate from their input positions.
  if (opt.rotation_weight > 0 || opt.translation_weight > 0){

    for (int icam = 0; icam < num_cameras; icam++){

      double const* orig_cam_ptr = orig_parameters.get_camera_ptr(icam);
      ceres::CostFunction* cost_function
        = RotTransError::Create(orig_cam_ptr, opt.rotation_weight, opt.translation_weight);
      ceres::LossFunction* loss_function = new ceres::TrivialLoss();

      double * camera  = param_storage.get_camera_ptr(icam);
      problem.AddResidualBlock(cost_function, loss_function, camera);
    }
  }

  // TODO: Can we split out this giant section?
  // Add a cost function meant to tie up to known disparity
  // form left to right image and known ground truth reference terrain.
  // This was only tested for local pinhole cameras.
  // Disparity must be created with stereo -e 3 with the
  // option --unalign-disparity. If there are n images,
  // there must be n-1 disparities, from each image to the next.
  // The doc has more info in the bundle_adjust chapter.
  std::vector< ImageView   <DispPixelT> > disp_vec;
  std::vector< ImageViewRef<DispPixelT> > interp_disp; 
  std::vector< vw::Vector3              > reference_vec;
  if (opt.reference_terrain != "") {
    // TODO: Pass these properly
    g_max_disp_error           = opt.max_disp_error;
    g_reference_terrain_weight = opt.reference_terrain_weight;
    
    // Set up a GeoReference object using the datum
    vw::cartography::GeoReference geo;
    geo.set_datum(opt.datum); // We checked for a datum earlier

    asp::CsvConv csv_conv;
    csv_conv.parse_csv_format(opt.csv_format_str, opt.csv_proj4_str);

    // Use user's csv_proj4 string, if provided, to add info to the georef.
    csv_conv.parse_georef(geo);

    vw::BBox2 lonlat_box; // not used
    bool      calc_shift = false;
    Vector3   shift; // must be set to 0
    bool      is_lola_rdr_format;
    double    mean_longitude;
    bool      verbose = true;
    asp::DoubleMatrix data;
    
    // Read the reference terrain
    vw_out() << "Loading at most " << opt.max_num_reference_points << " points from "
             << opt.reference_terrain << std::endl;
    std::string file_type = asp::get_cloud_type(opt.reference_terrain);
    if (file_type == "DEM") 
      asp::load_dem(opt.reference_terrain,  
               opt.max_num_reference_points, lonlat_box,  
               calc_shift, shift, verbose, data);
      
    else if (file_type == "CSV")
      asp::load_csv(opt.reference_terrain,  opt.max_num_reference_points,
                    lonlat_box, calc_shift, shift, geo,  
                    csv_conv, is_lola_rdr_format, mean_longitude, verbose,  
                    data);
    else
      vw_throw( ArgumentErr() << "Unsupported file: " << opt.reference_terrain << " of type"
                              << file_type << ".\n");

    if (load_reference_disparities(opt.disparity_list, disp_vec, interp_disp) != num_cameras-1)
      vw_throw( ArgumentErr() << "Expecting one less disparity than there are cameras.\n");
    
    std::vector<vw::BBox2i> image_boxes;
    for ( int icam = 0; icam < num_cameras; icam++){
      DiskImageView<float> img(opt.image_files[icam]);
      BBox2i bbox = vw::bounding_box(img);
      image_boxes.push_back(bbox);
    }

    vw_out() << "Setting up the error to the reference terrain.\n";
    TerminalProgressCallback tpc("", "\t--> ");
    tpc.report_progress(0);
    int num_cols = data.cols();
    //double inc_amount = 1.0/double(pos_records.size());
    double inc_amount = 1.0/double(num_cols);

    reference_vec.clear();
    for (int data_col = 0; data_col < num_cols; data_col++) {

      //vw::Vector3 reference_xyz = csv_conv.csv_to_cartesian(*iter, geo);
      vw::Vector3 reference_xyz;
      for (int row = 0; row < asp::DIM; row++)
        reference_xyz[row] = data(row, data_col);

      // Filter by lonlat box if provided, this is very much recommended
      // to quickly discard most points in the huge reference terrain.
      // Let's hope there is no 360 degree offset when computing
      // the longitude. 
      if ( asp::stereo_settings().lon_lat_limit != BBox2(0,0,0,0) ) {
        vw::Vector3 llh = geo.datum().cartesian_to_geodetic(reference_xyz);
        vw::Vector2 ll  = subvector(llh, 0, 2);
        if (!asp::stereo_settings().lon_lat_limit.contains(ll)) {
          continue;
        }
      }

      Vector2 left_pred, right_pred;

      // Iterate over the cameras, add a residual for each point and each camera pair.
      for (int icam = 0; icam < num_cameras - 1; icam++) {

        boost::shared_ptr<CameraModel> left_camera  = opt.camera_models[icam  ];
        boost::shared_ptr<CameraModel> right_camera = opt.camera_models[icam+1];

        try {
          left_pred  = left_camera->point_to_pixel (reference_xyz);
          right_pred = right_camera->point_to_pixel(reference_xyz);
        } catch (const camera::PointToPixelErr& e) {
          continue; // Skip point if there is a projection issue.
        }

        if ( (left_pred != left_pred) || (right_pred != right_pred) )
          continue; // nan check

        if (!interp_disp[icam].pixel_in_bounds(left_pred))
          continue; // Interp check

        DispPixelT dispPix = interp_disp[icam](left_pred[0], left_pred[1]);
        if (!is_valid(dispPix))
          continue;

        // Check if the current point projects in the cameras
        if ( !image_boxes[icam  ].contains(left_pred ) || 
             !image_boxes[icam+1].contains(right_pred)   ) {
          continue;
        }

        Vector2 right_pix = left_pred + dispPix.child();
        if (!image_boxes[icam+1].contains(right_pix)) 
          continue; // Check offset location too

        if (right_pix != right_pix || norm_2(right_pix - right_pred) > opt.max_disp_error) {
          // Ignore pixels which are too far from where they should be before optimization
          continue;
        }

        reference_vec.push_back(reference_xyz);

        // Call function to select the appropriate Ceres residual block to add.
        add_disparity_residual_block(reference_xyz, interp_disp[icam],
                                     icam, icam+1, // left icam and right icam
                                     param_storage, opt, problem);
      }
      tpc.report_incremental_progress( inc_amount );
    }
    
    tpc.report_finished();
    vw_out() << "Found " << reference_vec.size() << " reference points in range.\n";
  } // End if (opt.reference_terrain != "")
  
  const size_t MIN_KML_POINTS = 20;  
  size_t kmlPointSkip = 30;
  // Figure out a good KML point skip aount
  if (num_points / kmlPointSkip < MIN_KML_POINTS)
    kmlPointSkip = num_points / MIN_KML_POINTS;
  if (kmlPointSkip < 1)
    kmlPointSkip = 1;

  std::string residual_prefix = opt.out_prefix + "-initial_residuals_loss_function";
  std::string point_kml_path  = opt.out_prefix + "-initial_points.kml";
    
  if (first_pass) { 
    vw_out() << "Writing initial condition files..." << std::endl;

    write_residual_logs(residual_prefix, true,  opt, param_storage, 
                        cam_residual_counts, num_gcp_residuals,
                        reference_vec, cnet, crn, problem);
    residual_prefix = opt.out_prefix + "-initial_residuals_no_loss_function";
    write_residual_logs(residual_prefix, false, opt, param_storage, 
                        cam_residual_counts, num_gcp_residuals,
                        reference_vec, cnet, crn, problem);

    param_storage.record_points_to_kml(point_kml_path, opt.datum, 
                         kmlPointSkip, "initial_points",
                        "http://maps.google.com/mapfiles/kml/shapes/placemark_circle.png");
  }

  // Solve the problem
  ceres::Solver::Options options;
  options.gradient_tolerance  = 1e-16;
  options.function_tolerance  = 1e-16;
  options.parameter_tolerance = opt.parameter_tolerance; // default is 1e-8

  options.max_num_iterations                = opt.max_iterations;
  options.max_num_consecutive_invalid_steps = std::max(5, opt.max_iterations/5); // try hard
  options.minimizer_progress_to_stdout      = true;//(opt.report_level >= vw::ba::ReportFile);

  if (opt.stereo_session_string == "isis")
    options.num_threads = 1;
  else
    options.num_threads = opt.num_threads;

  // Set solver options according to the recommendations in the Ceres solving FAQs
  options.linear_solver_type = ceres::SPARSE_SCHUR;
  if (num_cameras < 100)
    options.linear_solver_type = ceres::DENSE_SCHUR;
  if (num_cameras > 3500) {
    options.use_explicit_schur_complement = true; // This is supposed to help with speed in a certain size range
    options.linear_solver_type  = ceres::ITERATIVE_SCHUR;
    options.preconditioner_type = ceres::SCHUR_JACOBI;
  }
  if (num_cameras > 7000)
    options.use_explicit_schur_complement = false; // Only matters with ITERATIVE_SCHUR

  //options.ordering_type = ceres::SCHUR;
  //options.eta = 1e-3; // FLAGS_eta;
  //options->max_solver_time_in_seconds = FLAGS_max_solver_time;
  //options->use_nonmonotonic_steps = FLAGS_nonmonotonic_steps;
  //if (FLAGS_line_search) {
  //  options->minimizer_type = ceres::LINE_SEARCH;
  //}

  vw_out() << "Starting the Ceres optimizer..." << std::endl;
  ceres::Solver::Summary summary;
  ceres::Solve(options, &problem, &summary);
  final_cost = summary.final_cost;
  vw_out() << summary.FullReport() << "\n";
  if (summary.termination_type == ceres::NO_CONVERGENCE){
    // Print a clarifying message, so the user does not think that the algorithm failed.
    vw_out() << "Found a valid solution, but did not reach the actual minimum." << std::endl;
    convergence_reached = false;
  }

  if (last_pass) {
    vw_out() << "Writing final condition log files..." << std::endl;
    residual_prefix = opt.out_prefix + "-final_residuals_loss_function";
    write_residual_logs(residual_prefix, true,  opt, param_storage, cam_residual_counts,
                        num_gcp_residuals, reference_vec, cnet, crn, problem);
    residual_prefix = opt.out_prefix + "-final_residuals_no_loss_function";
    write_residual_logs(residual_prefix, false, opt, param_storage, cam_residual_counts,
                        num_gcp_residuals, reference_vec, cnet, crn, problem);

    point_kml_path = opt.out_prefix + "-final_points.kml";
    param_storage.record_points_to_kml(point_kml_path, opt.datum,
                        kmlPointSkip, "final_points",
                        "http://maps.google.com/mapfiles/kml/shapes/placemark_circle_highlight.png");
  }

  // If heights_from_dem_weight > 0, each point is a gcp, and this stats becomes too long
  if (num_gcp > 0 && opt.heights_from_dem_weight <= 0)
    param_storage.print_gcp_stats(cnet, opt.datum);

  int num_new_outliers = 0;
  if (!last_pass) 
    num_new_outliers =
      update_outliers(cnet, crn,
                      param_storage,   // in-out
                      opt, cam_residual_counts,  
                      num_gcp_residuals, reference_vec, problem);

  // Remove flagged outliers and create clean match files.
  // Do this even when no new outliers are found, to
  // make sure the clean match files are written at least once.
  if (opt.num_ba_passes > 1) 
    remove_outliers(cnet, param_storage, opt);

  return num_new_outliers;
} // End function do_ba_ceres_one_pass

/// Use Ceres to do bundle adjustment.
void do_ba_ceres(Options & opt, std::vector<Vector3> const& estimated_camera_gcc){

  // Try to set up the control network, ie the list of point coordinates.
  // - This triangulates from the camera models to determine the initial
  //   world coordinate estimate for each matched IP.
  opt.cnet.reset( new ControlNetwork("BundleAdjust") );
  ControlNetwork & cnet = *(opt.cnet.get());

  bool success = vw::ba::build_control_network( true, // Always have input cameras
                                                cnet, opt.camera_models,
                                                opt.image_files,
                                                opt.match_files,
                                                opt.min_matches,
                                                opt.min_triangulation_angle*(M_PI/180),
                                                opt.forced_triangulation_distance);
  if (!success) {
    vw_out() << "Failed to build a control network. Consider removing "
             << "the currently found interest point matches and increasing "
             << "the number of interest points per tile using "
             << "--ip-per-tile, or decreasing --min-matches. Will continue "
             << "if ground control points are present.\n";
  }
  vw_out() << "Loading GCP files...\n";
  vw::ba::add_ground_control_points( cnet, opt.gcp_files, opt.datum);
  
  // If we change the cameras, we must rebuild the control network
  bool cameras_changed = false;
  
  // If camera positions were provided for local inputs, align to them.
  const bool have_est_camera_positions = (opt.camera_position_file != "");
  if ((opt.camera_type==BaCameraType_Pinhole) && have_est_camera_positions) {
    init_pinhole_model_with_camera_positions(opt.cnet, opt.camera_models,
                                             opt.image_files, estimated_camera_gcc);
    cameras_changed = true;
  }

  // If we have GPC's for pinhole cameras, try to do a simple affine
  // initialization of the camera parameters.
  // - This function also updates all the ControlNetwork world point
  //   positions.
  // - We could do this for other camera types too, but it would
  //   require us to be able to adjust our camera model positions.
  //   Otherwise we could init the adjustment values.
  if (opt.gcp_files.size() > 0) {
    if ((opt.camera_type==BaCameraType_Pinhole) && 
        !have_est_camera_positions &&
        !opt.disable_pinhole_gcp_init)
      init_pinhole_model_with_gcp(opt.cnet, opt.camera_models);

    // Issue a warning if the GCPs are far away from the camera coords
    check_gcp_dists(opt.camera_models, opt.cnet, opt.forced_triangulation_distance);
    cameras_changed = true;
  }

  int num_points  = cnet.size();
  const int num_cameras = opt.image_files.size();

  // This is important to prevent a crash later
  if (num_points == 0) {
    vw_out() << "No points to optimize (GCP or otherwise). Cannot continue.\n";
    return;
  }
  
  // Create the storage arrays for the variables we will adjust.
  int num_lens_distortion_params = 0;
  if (opt.camera_type == BaCameraType_Pinhole) {
    boost::shared_ptr<vw::camera::PinholeModel> pinhole_ptr = 
            boost::dynamic_pointer_cast<vw::camera::PinholeModel>(opt.camera_models[0]);
    num_lens_distortion_params = pinhole_ptr->lens_distortion()->distortion_parameters().size();
    if (num_lens_distortion_params < 1) {
      // For the case where the camera has zero distortion parameters, use one dummy parameter
      //  just so we don't have to change the parameter block logic later on.
      num_lens_distortion_params = 1;
      opt.intrinisc_options.distortion_constant = true;
      opt.intrinisc_options.distortion_shared   = true;
    }
  }
  if (opt.camera_type == BaCameraType_OpticalBar) {
    num_lens_distortion_params = 2; // TODO: Share this constant!
  }
  BAParamStorage param_storage(num_points, num_cameras,
                               // Optical bar and pinhole are similar
                               opt.camera_type != BaCameraType_Other, 
                               // Must be the same for each pinhole camera
                               num_lens_distortion_params, 
                               opt.intrinisc_options);

  // Fill in the camera and intrinsic parameters.
  std::vector<boost::shared_ptr<camera::CameraModel> > new_cam_models;
  bool ans = false;
  switch(opt.camera_type) {
    case BaCameraType_Pinhole:
      ans = init_cams_pinhole(opt, param_storage, new_cam_models); break;
    case BaCameraType_OpticalBar:
      ans = init_cams_optical_bar(opt, param_storage, new_cam_models); break;
    default:
      ans = init_cams(opt, param_storage, new_cam_models);
  };

  if (ans)
    cameras_changed = true;
  
  // Certain input options change the cameras inside init_cams and we need to update the
  // point coordinates for the new cameras.
  // - It is ok to leave the original vector of camera models unchanged.
  if (cameras_changed) {
    vw_out() <<"Updating the control network." << std::endl;
    cnet = ControlNetwork("Updated network"); // Wipe it all first
    /*bool success = */
    // Building the control network below may fail if there are only GCP,
    // but we will continue nevertheless.
    vw::ba::build_control_network( true, // Always have input cameras
                                   cnet, new_cam_models,
                                   opt.image_files,
                                   opt.match_files,
                                   opt.min_matches,
                                   opt.min_triangulation_angle*(M_PI/180),
                                   opt.forced_triangulation_distance);
    
    // Restore the rest of the cnet object
    vw::ba::add_ground_control_points(cnet, opt.gcp_files, opt.datum);
    
    // Must update the number of points after the control network is recomputed
    num_points = cnet.size();
    param_storage.get_point_vector().resize(num_points*BAParamStorage::PARAMS_PER_POINT);
  }
  
  if (opt.save_cnet_as_csv) {
    std::string cnet_file = opt.out_prefix + "-cnet.csv";
    vw_out() << "Writing: " << cnet_file << std::endl;
    cnet.write_in_gcp_format(cnet_file, opt.datum);
  }
  
  // Fill in the point vector with the starting values.
  for (int ipt = 0; ipt < num_points; ipt++)
    param_storage.set_point(ipt, cnet[ipt].position());

  // The camera positions and orientations before we float them
  // - This includes modifications from any initial transforms that were specified.
  BAParamStorage orig_parameters(param_storage);

  // TODO: Possible to avoid using CRNs?
  CRNJ crn;
  crn.read_controlnetwork(cnet);

  if (opt.num_ba_passes <= 0)
    vw_throw(ArgumentErr() << "Error: Expecting at least one bundle adjust pass.\n");
  
  double final_cost;
  for (int pass = 0; pass < opt.num_ba_passes; pass++) {

    vw_out() << "--> Bundle adjust pass: " << pass << std::endl;
    if (pass > 0) {
      // Go back to the original inputs to optimize, but keep our outlier list.
      param_storage.copy_points    (orig_parameters);
      param_storage.copy_cameras   (orig_parameters);
      param_storage.copy_intrinsics(orig_parameters);
    }

    // Do another pass of bundle adjustment.
    bool last_pass = (pass == opt.num_ba_passes - 1);
    bool convergence_reached = true;
    int  num_new_outliers    = do_ba_ceres_one_pass(opt, crn, (pass==0), last_pass,
                                                    param_storage, orig_parameters,
                                                    convergence_reached, final_cost);

    if (!last_pass && num_new_outliers == 0 && convergence_reached) {
      vw_out() << "No new outliers removed, and the algorithm converged. "
               << "No more passes are needed.\n";
      break;
    }

    int num_points_remaining = num_points - param_storage.get_num_outliers();
    if (opt.num_ba_passes > 1 && num_points_remaining < opt.min_matches) {
      // Do not throw if there were is just one pass, as no outlier filtering happened.
      // This is needed to not break functionality when only gcp are passed as inputs.
      vw_throw(ArgumentErr() << "Error: Too few points remain after filtering!.\n");
    }
  } // End loop through passes

  double best_cost = final_cost;
  boost::shared_ptr<BAParamStorage> best_params_ptr(new BAParamStorage(param_storage));

  std::string orig_out_prefix = opt.out_prefix;
  for (int pass = 0; pass < opt.num_random_passes; pass++) {

    vw_out() << "\n--> Running bundle adjust pass " << pass 
             << " with random initial parameter offsets.\n";

    // Go back to the original inputs to optimize, but keep our outlier list.
    param_storage.copy_points    (orig_parameters);
    param_storage.copy_cameras   (orig_parameters);
    param_storage.copy_intrinsics(orig_parameters);

    // Randomly distort the original inputs.
    param_storage.randomize_cameras();
    if (opt.solve_intrinsics)
      param_storage.randomize_intrinsics(); // The function call handles sharing etc.

    // Write output files to a temporary prefix
    opt.out_prefix = orig_out_prefix + "_rand";

    // Do another pass of bundle adjustment.
    bool first_pass = true;
    bool last_pass  = true;
    bool convergence_reached = true;
    int  num_new_outliers    = do_ba_ceres_one_pass(opt, crn, first_pass, last_pass,
                                                    param_storage, orig_parameters,
                                                    convergence_reached, final_cost);
    // Record the parameters of the best result.
    if (final_cost < best_cost) {
      vw_out() << "  --> Found a better solution!\n\n";
      best_cost = final_cost;
      best_params_ptr.reset(new BAParamStorage(param_storage));

      // Get a list of all the files that were generated in the random step.
      std::vector<std::string> rand_files;
      get_files_with_prefix(opt.out_prefix, rand_files);

      // Replace the existing output files with them.
      for (size_t i=0; i<rand_files.size(); ++i) {
        std::string new_path = rand_files[i];
        boost::replace_all(new_path, opt.out_prefix, orig_out_prefix);
        boost::filesystem::copy_file(rand_files[i], new_path,
                                     boost::filesystem::copy_option::overwrite_if_exists);
      }
    }

    // Clear out the extra files that were generated
    std::string cmd("rm -f " + opt.out_prefix + "*");
    vw::exec_cmd(cmd.c_str());
  }
  opt.out_prefix = orig_out_prefix; // So the cameras are written to the expected paths.

  // Write the results to disk.
  for (int icam = 0; icam < num_cameras; icam++){

    switch(opt.camera_type) {
      case BaCameraType_Pinhole:
        write_pinhole_output_file(opt, icam, *best_params_ptr);
        break;
      case BaCameraType_OpticalBar:
        write_optical_bar_output_file(opt, icam, *best_params_ptr);
        break;
      default:
        std::string adjust_file = asp::bundle_adjust_file_name(opt.out_prefix,
                                                              opt.image_files[icam],
                                                              opt.camera_files[icam]);
        vw_out() << "Writing: " << adjust_file << std::endl;

        CameraAdjustment cam_adjust(best_params_ptr->get_camera_ptr(icam));
        asp::write_adjustments(adjust_file, cam_adjust.position(), cam_adjust.pose());
    };
  } // End loop through cameras

} // end do_ba_ceres



/// Looks in the input camera position file to generate a GCC position for
/// each input camera.
/// - If no match is found, the coordinate is (0,0,0)
int load_estimated_camera_positions(Options &opt,
                                    std::vector<Vector3> & estimated_camera_gcc) {
  estimated_camera_gcc.clear();
  if (opt.camera_position_file == "")
    return 0;
  
  // Read the input csv file
  asp::CsvConv conv;
  conv.parse_csv_format(opt.csv_format_str, opt.csv_proj4_str);
  std::list<asp::CsvConv::CsvRecord> pos_records;
  typedef std::list<asp::CsvConv::CsvRecord>::const_iterator RecordIter;
  conv.read_csv_file(opt.camera_position_file, pos_records);

  // Set up a GeoReference object using the datum
  vw::cartography::GeoReference geo;
  geo.set_datum(opt.datum); // We checked for a datum earlier
  // Use user's csv_proj4 string, if provided, to add info to the georef.
  conv.parse_georef(geo);

  // For each input camera, find the matching position in the record list
  const int num_cameras = opt.image_files.size();
  estimated_camera_gcc.resize(num_cameras);
  
  const RecordIter no_match = pos_records.end();
  int num_matches_found = 0;
  for (int i=0; i<num_cameras; ++i) {

    // Search for this image file in the records
    std::string file_name = opt.image_files[i];
    RecordIter iter;
    for (iter=pos_records.begin(); iter!=pos_records.end(); ++iter) {
      // Match if the string in the file is contained in the input image string.
      // - May need to play around with this in the future!
      std::string field = iter->file;
      if (file_name.find(field) != std::string::npos) {
        estimated_camera_gcc[i] = conv.csv_to_cartesian(*iter, geo);
        break; // Match found, stop the iterator here.
      }
    }
    if (iter == no_match) {
      vw_out() << "WARNING: Camera file " << file_name << " not found in camera position file.\n";
      estimated_camera_gcc[i] = Vector3(0,0,0);
    }else
      ++num_matches_found;
  } // End loop to find position record for each camera

  return num_matches_found;  
}

void handle_arguments(int argc, char *argv[], Options& opt) {
  const double nan = std::numeric_limits<double>::quiet_NaN();
  std::string intrinsics_to_float_str, intrinsics_to_share_str;
  float auto_overlap_buffer;
  bool inline_adjustments;
  po::options_description general_options("");
  general_options.add_options()
    ("output-prefix,o",  po::value(&opt.out_prefix), "Prefix for output filenames.")
    ("cost-function",    po::value(&opt.cost_function)->default_value("Cauchy"),
            "Choose a cost function from: Cauchy, PseudoHuber, Huber, L1, L2.")
    ("robust-threshold", po::value(&opt.robust_threshold)->default_value(0.5),
            "Set the threshold for robust cost functions. Increasing this makes the solver focus harder on the larger errors.")
    ("inline-adjustments",   po::bool_switch(&inline_adjustments)->default_value(false),
     "If this is set, and the input cameras are of the pinhole or panoramic type, apply the adjustments directly to the cameras, rather than saving them separately as .adjust files.")
    ("approximate-pinhole-intrinsics", po::bool_switch(&opt.approximate_pinhole_intrinsics)->default_value(false),
            "If it reduces computation time, approximate the lens distortion model.")
    ("solve-intrinsics",    po::bool_switch(&opt.solve_intrinsics)->default_value(false)->implicit_value(true),
            "Optimize intrinsic camera parameters.  Only used for pinhole cameras.")
    ("intrinsics-to-float", po::value(&intrinsics_to_float_str)->default_value(""),
            "If solving for intrinsics and desired to float only a few of them, specify here, in quotes, one or more of: focal_length, optical_center, other_intrinsics.")
    ("intrinsics-to-share", po::value(&intrinsics_to_share_str)->default_value(""),
            "If solving for intrinsics and desired to share only a few of them, specify here, in quotes, one or more of: focal_length, optical_center, other_intrinsics.")
    ("camera-positions",    po::value(&opt.camera_position_file)->default_value(""),
            "Specify a csv file path containing the estimated positions of the input cameras.  Only used with the inline-adjustments option.")
    ("disable-pinhole-gcp-init",  po::bool_switch(&opt.disable_pinhole_gcp_init)->default_value(false)->implicit_value(true),
            "Don't try to initialize the positions of pinhole cameras based on input GCPs.")
    ("input-adjustments-prefix",  po::value(&opt.input_prefix),
            "Prefix to read initial adjustments from, written by a previous invocation of this program.")
    ("initial-transform",   po::value(&opt.initial_transform_file)->default_value(""),
            "Before optimizing the cameras, apply to them the 4x4 rotation + translation transform from this file. The transform is in respect to the planet center, such as written by pc_align's source-to-reference or reference-to-source alignment transform. Set the number of iterations to 0 to stop at this step. If --input-adjustments-prefix is specified, the transform gets applied after the adjustments are read.")
    ("fixed-camera-indices",    po::value(&opt.fixed_cameras_indices_str)->default_value(""),
            "A list of indices, in quotes and starting from 0, with space as separator, corresponding to cameras to keep fixed during the optimization process.")
    ("fix-gcp-xyz",       po::bool_switch(&opt.fix_gcp_xyz)->default_value(false)->implicit_value(true),
            "If the GCP are highly accurate, use this option to not float them during the optimization.")

    ("csv-format",        po::value(&opt.csv_format_str)->default_value(""), asp::csv_opt_caption().c_str())
    ("csv-proj4",         po::value(&opt.csv_proj4_str)->default_value(""),
            "The PROJ.4 string to use to interpret the entries in input CSV files.")
    ("reference-terrain", po::value(&opt.reference_terrain)->default_value(""),
            "An externally provided trustworthy 3D terrain, either as a DEM or as a lidar file, very close (after alignment) to the stereo result from the given images and cameras that can be used as a reference, instead of GCP, to optimize the intrinsics of the cameras.")
    ("max-num-reference-points", po::value(&opt.max_num_reference_points)->default_value(100000000),
     "Maximum number of (randomly picked) points from the reference terrain to use.")
    ("disparity-list",           po::value(&opt.disparity_list)->default_value(""),
            "The unaligned disparity files to use when optimizing the intrinsics based on a reference terrain. Specify them as a list in quotes separated by spaces. First file is for the first two images, second is for the second and third images, etc. If an image pair has no disparity file, use 'none'.")
    ("max-disp-error",           po::value(&opt.max_disp_error)->default_value(-1),
            "When using a reference terrain as an external control, ignore as outliers xyz points which projected in the left image and transported by disparity to the right image differ by the projection of xyz in the right image by more than this value in pixels.")
    ("reference-terrain-weight", po::value(&opt.reference_terrain_weight)->default_value(1.0),
            "How much weight to give to the cost function terms involving the reference terrain.")
    ("heights-from-dem",   po::value(&opt.heights_from_dem)->default_value(""),
            "If the cameras have already been bundle-adjusted and aligned to a known high-quality DEM, in the triangulated xyz points replace the heights with the ones from this DEM, and fix those points unless --heights-from-dem-weight is positive.")
    ("heights-from-dem-weight", po::value(&opt.heights_from_dem_weight)->default_value(-1.0),
     "How much weight to give to keep the triangulated points close to the DEM if specified via --heights-from-dem. If the weight is not positive, keep the triangulated points fixed.")
    ("datum",            po::value(&opt.datum_str)->default_value(""),
            "Use this datum. Needed only for ground control points, a camera position file, or for RPC sessions. Options: WGS_1984, D_MOON (1,737,400 meters), D_MARS (3,396,190 meters), MOLA (3,396,000 meters), NAD83, WGS72, and NAD27. Also accepted: Earth (=WGS_1984), Mars (=D_MARS), Moon (=D_MOON).")
    ("semi-major-axis",  po::value(&opt.semi_major)->default_value(0),
            "Explicitly set the datum semi-major axis in meters (see above).")
    ("semi-minor-axis",  po::value(&opt.semi_minor)->default_value(0),
            "Explicitly set the datum semi-minor axis in meters (see above).")
    ("session-type,t",   po::value(&opt.stereo_session_string)->default_value(""),
            "Select the stereo session type to use for processing. Options: pinhole nadirpinhole isis dg rpc spot5 aster opticalbar. Usually the program can select this automatically by the file extension.")
    ("min-matches",      po::value(&opt.min_matches)->default_value(30),
            "Set the minimum  number of matches between images that will be considered.")
    ("ip-detect-method", po::value(&opt.ip_detect_method)->default_value(0),
            "Interest point detection algorithm (0: Integral OBALoG (default), 1: OpenCV SIFT, 2: OpenCV ORB.")
    ("epipolar-threshold",      po::value(&opt.epipolar_threshold)->default_value(-1),
            "Maximum distance from the epipolar line to search for IP matches. Default: automatic calculation.")
    ("ip-inlier-factor",        po::value(&opt.ip_inlier_factor)->default_value(1.0/15.0),
            "A higher factor will result in more interest points, but perhaps also more outliers.")
    ("ip-uniqueness-threshold", po::value(&opt.ip_uniqueness_thresh)->default_value(0.7),
            "A higher threshold will result in more interest points, but perhaps less unique ones.")
    ("ip-side-filter-percent",  po::value(&opt.ip_edge_buffer_percent)->default_value(-1),
            "Remove matched IPs this percentage from the image left/right sides.")
    ("normalize-ip-tiles", 
            po::bool_switch(&opt.ip_normalize_tiles)->default_value(false)->implicit_value(true),
            "Individually normalize tiles used for IP detection.")
    ("disable-tri-ip-filter",
            po::bool_switch(&opt.disable_tri_filtering)->default_value(false)->implicit_value(true),
            "Skip tri_ip filtering.")
    ("ip-debug-images",        po::value(&opt.ip_debug_images)->default_value(false)->implicit_value(true),
            "Write debug images to disk when detecting and matching interest points.")
    ("elevation-limit",        po::value(&opt.elevation_limit)->default_value(Vector2(0,0), "auto"),
            "Limit on expected elevation range: Specify as two values: min max.")
    // Note that we count later on the default for lon_lat_limit being BBox2(0,0,0,0).
    ("lon-lat-limit",          po::value(&opt.lon_lat_limit)->default_value(BBox2(0,0,0,0), "auto"),
            "Limit the triangulated interest points to this longitude-latitude range. The format is: lon_min lat_min lon_max lat_max.")
    ("num-obalog-scales",      po::value(&opt.num_scales)->default_value(-1),
            "How many scales to use if detecting interest points with OBALoG. If not specified, 8 will be used. More can help for images with high frequency artifacts.")
    ("nodata-value",           po::value(&opt.nodata_value)->default_value(nan),
            "Pixels with values less than or equal to this number are treated as no-data. This overrides the no-data values from input images.")
    ("skip-rough-homography",
            po::bool_switch(&opt.skip_rough_homography)->default_value(false)->implicit_value(true),
            "Skip the step of performing datum-based rough homography if it fails.")
    ("individually-normalize", 
            po::bool_switch(&opt.individually_normalize)->default_value(false)->implicit_value(true),
            "Individually normalize the input images instead of using common values.")
    ("max-iterations",       po::value(&opt.max_iterations)->default_value(1000),
            "Set the maximum number of iterations.")
    ("parameter-tolerance",  po::value(&opt.parameter_tolerance)->default_value(1e-8),
            "Stop when the relative error in the variables being optimized is less than this.")
    ("overlap-limit",        po::value(&opt.overlap_limit)->default_value(0),
            "Limit the number of subsequent images to search for matches to the current image to this value.  By default match all images.")
    ("overlap-list",         po::value(&opt.overlap_list_file)->default_value(""),
            "A file containing a list of image pairs, one pair per line, separated by a space, which are expected to overlap. Matches are then computed only among the images in each pair.")
    ("auto-overlap-buffer",  po::value(&auto_overlap_buffer)->default_value(-1),
            "Try to automatically guess which images overlap with the provided buffer in lonlat degrees.")
    ("position-filter-dist", po::value(&opt.position_filter_dist)->default_value(-1),
            "Set a distance in meters and don't perform IP matching on images with an estimated camera center farther apart than this distance.  Requires --camera-positions.")
    ("rotation-weight",      po::value(&opt.rotation_weight)->default_value(0.0),
            "A higher weight will penalize more rotation deviations from the original configuration.")
    ("translation-weight",   po::value(&opt.translation_weight)->default_value(0.0),
            "A higher weight will penalize more translation deviations from the original configuration.")
    ("camera-weight",        po::value(&opt.camera_weight)->default_value(1.0),
            "The weight to give to the constraint that the camera positions/orientations stay close to the original values (only for the Ceres solver).  A higher weight means that the values will change less. The options --rotation-weight and --translation-weight can be used for finer-grained control and a stronger response.")
    ("overlap-exponent",     po::value(&opt.overlap_exponent)->default_value(0.0),
            "If a feature is seen in n >= 2 images, give it a weight proportional with (n-1)^exponent.")
    ("ip-per-tile",          po::value(&opt.ip_per_tile)->default_value(0),
            "How many interest points to detect in each 1024^2 image tile (default: automatic determination).")
    ("num-passes",           po::value(&opt.num_ba_passes)->default_value(1),
            "How many passes of bundle adjustment to do. If more than one, outliers will be removed between passes using --remove-outliers-params and --remove-outliers-by-disparity-params, and re-optimization will take place. Residual files and a copy of the match files with the outliers removed will be written to disk.")
    ("num-random-passes",           po::value(&opt.num_random_passes)->default_value(0),
            "After performing the normal bundle adjustment passes, do this many more passes using the same matches but adding random offsets to the initial parameter values with the goal of avoiding local minima that the optimizer may be getting stuck in.")
    ("ip-triangulation-max-error",  po::value(&opt.ip_triangulation_max_error)->default_value(-1),
            "When matching IP, filter out any pairs with a triangulation error higher than this.")
    ("remove-outliers-params", 
            po::value(&opt.remove_outliers_params_str)->default_value("75.0 3.0 2.0 3.0", "'pct factor err1 err2'"),
            "Outlier removal based on percentage, when more than one bundle adjustment pass is used. Triangulated points with reprojection error in pixels larger than min(max('pct'-th percentile * 'factor', err1), err2) will be removed as outliers. Hence, never remove errors smaller than err1 but always remove those bigger than err2. Specify as a list in quotes. Default: '75.0 3.0 2.0 3.0'.")
    ("remove-outliers-by-disparity-params",  
            po::value(&opt.remove_outliers_by_disp_params)->default_value(Vector2(90.0,3.0), "pct factor"),
            "Outlier removal based on the disparity of interest points (difference between right and left pixel), when more than one bundle adjustment pass is used. For example, the 10% and 90% percentiles of disparity are computed, and this interval is made three times bigger. Interest points whose disparity fall outside the expanded interval are removed as outliers. Instead of the default 90 and 3 one can specify pct and factor, without quotes.")
    ("min-triangulation-angle",      po::value(&opt.min_triangulation_angle)->default_value(0.1),
            "The minimum angle, in degrees, at which rays must meet at a triangulated point to accept this point as valid.")
    ("forced-triangulation-distance",      po::value(&opt.forced_triangulation_distance)->default_value(-1),
            "When triangulation fails, for example, when input cameras are inaccurate, artificially create a triangulation point this far ahead of the camera, in units of meter.")
    ("use-lon-lat-height-gcp-error",
     po::bool_switch(&opt.use_llh_error)->default_value(false)->implicit_value(true),
     "When having GCP, interpret the three standard deviations in the GCP file as applying not to x, y, and z, but rather to latitude, longitude, and height.")
    ("force-reuse-match-files", po::bool_switch(&opt.force_reuse_match_files)->default_value(false)->implicit_value(true),
     "Force reusing the match files even if older than the images or cameras.")
    ("mapprojected-data",  po::value(&opt.mapprojected_data)->default_value(""),
            "Given map-projected versions of the input images, the DEM they were mapprojected onto, and IP matches among the mapprojected images, create IP matches among the un-projected images before doing bundle adjustment. Specify the mapprojected images and the DEM as a string in quotes, separated by spaces. An example is in the documentation.")
    ("save-cnet-as-csv", po::bool_switch(&opt.save_cnet_as_csv)->default_value(false)->implicit_value(true),
     "Save the control network containing all interest points in the format used by ground control points, so it can be inspected.")
    ("gcp-from-mapprojected-images", po::value(&opt.gcp_from_mapprojected)->default_value(""),
     "Given map-projected versions of the input images, the DEM the were mapprojected onto, and interest point matches among all of these created in stereo_gui, create GCP for the input images to align them better to the DEM. This is experimental and not documented.")
    ("lambda,l",           po::value(&opt.lambda)->default_value(-1),
            "Set the initial value of the LM parameter lambda (ignored for the Ceres solver).")
    
    ("instance-count",      po::value(&opt.instance_count)->default_value(1),
            "The number of bundle_adjustment processes being run in parallel.")
    ("instance-index",      po::value(&opt.instance_index)->default_value(0),
            "The index of this parallel bundle adjustment process.")
    ("stop-after-statistics",    po::bool_switch(&opt.stop_after_stats)->default_value(false)->implicit_value(true),
            "Quit after computing image statistics.")
    ("stop-after-matching",    po::bool_switch(&opt.stop_after_matching)->default_value(false)->implicit_value(true),
            "Quit after writing all match files.")
    ("skip-matching",    po::bool_switch(&opt.skip_matching)->default_value(false)->implicit_value(true),
            "Only use image matches which can be loaded from disk.")
    
    ("report-level,r",     po::value(&opt.report_level)->default_value(10),
            "Use a value >= 20 to get increasingly more verbose output.");
//     ("save-iteration-data,s", "Saves all camera information between iterations to output-prefix-iterCameraParam.txt, it also saves point locations for all iterations in output-prefix-iterPointsParam.txt.");
  general_options.add( vw::cartography::GdalWriteOptionsDescription(opt) );

  // TODO: When finding the min and max bounds, do a histogram, throw away 5% of points
  // or something at each end.

  po::options_description positional("");
  positional.add_options()
    ("input-files", po::value(&opt.image_files));

  po::positional_options_description positional_desc;
  positional_desc.add("input-files", -1);

  std::string usage("<images> <cameras> <optional ground control points> -o <output prefix> [options]");
  bool allow_unregistered = false;
  std::vector<std::string> unregistered;
  po::variables_map vm =
    asp::check_command_line(argc, argv, opt, general_options, general_options,
                            positional, positional_desc, usage,
                             allow_unregistered, unregistered);

  // Separate out GCP files
  opt.gcp_files = asp::get_files_with_ext(opt.image_files, ".gcp", true);
  const size_t num_gcp_files = opt.gcp_files.size();
  vw_out() << "Found " << num_gcp_files << " GCP files on the command line.\n";

  // Separate the cameras from the images
  std::vector<std::string> inputs = opt.image_files;
  bool ensure_equal_sizes = true;
  asp::separate_images_from_cameras(inputs,
                                    opt.image_files, opt.camera_files, // outputs
                                    ensure_equal_sizes); 
  
  // If there are no camera files, then the image files have the camera information.
  if (opt.camera_files.empty()){
    for (int i = 0; i < opt.image_files.size(); i++)
      opt.camera_files.push_back("");
  }
  
  // Throw if there are duplicate camera file names.
  opt.check_for_duplicate_camera_names();
  
  // Sanity check
  if (opt.image_files.size() != (int)opt.camera_files.size()){
    vw_out() << "Detected " << opt.image_files.size() << " images and "
             << opt.camera_files.size() << " cameras.\n";
    vw_throw(ArgumentErr() << "Must have as many cameras as we have images.\n");
  }
  
  // TODO: Check for duplicates in opt.image_files!

  if ( opt.image_files.empty() )
    vw_throw( ArgumentErr() << "Missing input image files.\n"
                            << usage << general_options );


  // Work out the camera model type to use
  boost::to_lower( opt.stereo_session_string );
  opt.camera_type = BaCameraType_Other;
  if (inline_adjustments) {

    // Try to guess the session 
    if (opt.stereo_session_string == ""){
      try {
        // If we can open a pinhole camera file, that means
        // we are good. We prefer nadirpinhole to pinhole
        // session.
        PinholeModel(opt.camera_files[0]);
        opt.stereo_session_string = "nadirpinhole";
      }catch(std::exception const& e){}
    }
    
    if ((opt.stereo_session_string == "pinhole") || 
        (opt.stereo_session_string == "nadirpinhole")) {
      opt.camera_type = BaCameraType_Pinhole;
    } else {
      if (opt.stereo_session_string == "opticalbar")
        opt.camera_type = BaCameraType_OpticalBar;
      else
        vw_throw( ArgumentErr() << "Cannot use inline adjustments with session: "
                  << opt.stereo_session_string << "\n"
                                << usage << general_options );
    }
  } // End resolving the model type
  
  
  
  if (opt.overlap_list_file != "" && opt.overlap_limit > 0)
    vw_throw( ArgumentErr() << "Cannot specify both the overlap limit and the overlap list.\n"
              << usage << general_options );
  
  if ( opt.overlap_limit < 0 )
    vw_throw( ArgumentErr() << "Must allow search for matches between "
              << "at least each image and its subsequent one.\n" << usage << general_options );
  // By default, try to match all of the images!
  if ( opt.overlap_limit == 0 )
    opt.overlap_limit = opt.image_files.size();

  if (opt.overlap_list_file != "") {
    if (!fs::exists(opt.overlap_list_file))
      vw_throw( ArgumentErr() << "The overlap list does not exist.\n" << usage
                              << general_options );
    opt.overlap_list.clear();
    std::string image1, image2;
    std::ifstream ifs(opt.overlap_list_file.c_str());
    while (ifs >> image1 >> image2){
      opt.overlap_list.insert(std::pair<std::string, std::string>(image1, image2));
      opt.overlap_list.insert(std::pair<std::string, std::string>(image2, image1));
    }
    ifs.close();
  } else {
    if (!vm["auto-overlap-buffer"].defaulted())
      auto_build_overlap_list(opt, auto_overlap_buffer);
  }
  
  if ( opt.camera_weight < 0.0 )
    vw_throw( ArgumentErr() << "The camera weight must be non-negative.\n" << usage
                            << general_options );

  if ( opt.rotation_weight < 0.0 )
    vw_throw( ArgumentErr() << "The rotation weight must be non-negative.\n" << usage
                            << general_options );

  if ( opt.translation_weight < 0.0 )
    vw_throw( ArgumentErr() << "The translation weight must be non-negative.\n" << usage
                            << general_options );

  // TODO: Make sure the normal model loading catches this error.
  //if (opt.create_pinhole && !asp::has_pinhole_extension(opt.camera_files[0]))
  //  vw_throw( ArgumentErr() << "Cannot use special pinhole handling with non-pinhole input!\n");

  if ((opt.camera_type==BaCameraType_Other) && opt.solve_intrinsics)
    vw_throw( ArgumentErr() << "Solving for intrinsic parameters is only supported with pinhole and optical bar cameras.\n");

  if ((opt.camera_type!=BaCameraType_Pinhole) && opt.approximate_pinhole_intrinsics)
    vw_throw( ArgumentErr() << "Cannot approximate intrinsics unless using pinhole cameras.\n");

  if (opt.approximate_pinhole_intrinsics && opt.solve_intrinsics)
    vw_throw( ArgumentErr() << "Cannot approximate intrinsics while solving for them.\n");

  if ((opt.camera_type!=BaCameraType_Other) && opt.input_prefix != "")
    vw_throw( ArgumentErr() << "Can only use initial adjustments with camera type 'other'.\n");

  vw::string_replace(opt.remove_outliers_params_str, ",", " "); // replace any commas
  opt.remove_outliers_params = vw::str_to_vec<vw::Vector<double, 4> >(opt.remove_outliers_params_str);
  
  // Copy the IP settings to the global stereo_settings() object
  opt.copy_to_asp_settings();

  // Ensure good order
  if ( asp::stereo_settings().lon_lat_limit != BBox2(0,0,0,0) ) {
    if ( asp::stereo_settings().lon_lat_limit.min().y() >
         asp::stereo_settings().lon_lat_limit.max().y() ) 
      std::swap( asp::stereo_settings().lon_lat_limit.min().y(),
                 asp::stereo_settings().lon_lat_limit.max().y() );
    if ( asp::stereo_settings().lon_lat_limit.min().x() >
         asp::stereo_settings().lon_lat_limit.max().x() ) 
      std::swap( asp::stereo_settings().lon_lat_limit.min().x(),
                 asp::stereo_settings().lon_lat_limit.max().x() );
  }
  
  if (!opt.camera_position_file.empty() && opt.csv_format_str == "")
    vw_throw( ArgumentErr() << "When using a camera position file, the csv-format "
                            << "option must be set.\n" << usage << general_options );
  
  // Try to infer the datum, if possible, from the images. For
  // example, Cartosat-1 has that info in the Tif file.
  bool guessed_datum = false;
  if (opt.datum_str == "") {
    vw::cartography::GeoReference georef;
    for (size_t it = 0; it < opt.image_files.size(); it++) {
      bool is_good = vw::cartography::read_georeference(georef, opt.image_files[it]);
      if (is_good){
        opt.datum = georef.datum();
        opt.datum_str = opt.datum.name();
        guessed_datum = true;
      }
    }
  }

  // Try to infer the datum from the reference terrain
  if (opt.reference_terrain != "") {
    std::string file_type = asp::get_cloud_type(opt.reference_terrain);
    if (file_type == "DEM") {
      vw::cartography::GeoReference georef;
      bool is_good = vw::cartography::read_georeference(georef, opt.reference_terrain);
      if (!is_good)
        vw_throw( ArgumentErr() << "The reference terrain DEM does not have a georeference.\n"
                  << usage << general_options );
      if (opt.datum_str == ""){
        opt.datum = georef.datum();
        opt.datum_str = opt.datum.name();
        guessed_datum = true;
      }
    }
  }

  // Try to infer the datum from the heights-from-dem
  if (opt.heights_from_dem != "") {
    std::string file_type = asp::get_cloud_type(opt.heights_from_dem);
    if (file_type == "DEM") {
      vw::cartography::GeoReference georef;
      bool is_good = vw::cartography::read_georeference(georef, opt.heights_from_dem);
      if (!is_good)
        vw_throw( ArgumentErr() << "The DEM does not have a georeference.\n"
                  << usage << general_options );
      if (opt.datum_str == "" ){
        opt.datum = georef.datum();
        opt.datum_str = opt.datum.name();
        guessed_datum = true;
      }
    }
  }

  // Based on the cameras, try to guess the session, if not specified. If the session is isis,
  // then we can pull the datum from the .cub files. 
  {
    SessionPtr session(asp::StereoSessionFactory::create(opt.stereo_session_string, // may change
                                                         opt,
                                                         opt.image_files [0], opt.image_files [0],
                                                         opt.camera_files[0], opt.camera_files[0],
                                                         opt.out_prefix));
    
    if (opt.datum_str == "" && opt.stereo_session_string == "isis"){
      try {
        bool use_sphere_for_isis = false;
        opt.datum = session->get_datum(session->camera_model(opt.image_files [0],
                                                             opt.camera_files[0]).get(),
                                       use_sphere_for_isis);
        opt.datum_str = opt.datum.name();
        guessed_datum = true;
      }catch(...){}
    }
  }
  
  // Many of the sessions are for Earth, when we use WGS84 by default
  if (opt.stereo_session_string == "nadirpinhole" || opt.stereo_session_string == "dg"    ||
      opt.stereo_session_string == "spot5"        || opt.stereo_session_string == "aster" ||
      opt.stereo_session_string == "opticalbar") {
    if (opt.datum_str == "" ){
      opt.datum_str = "WGS84";
      opt.datum.set_well_known_datum(opt.datum_str);
      guessed_datum = true;
    }
  }

  // If nothing worked, the datum must be specified
  if (opt.stereo_session_string == "rpc" && opt.datum_str == "")
    vw_throw( ArgumentErr() << "When the session type is RPC, the datum must be specified.\n"
                            << usage << general_options );       

  if (opt.datum_str != ""){
    // If the user set the datum, use it.
    if (!guessed_datum)
      opt.datum.set_well_known_datum(opt.datum_str);
  }else if (opt.semi_major > 0 && opt.semi_minor > 0){
    // Otherwise, if the user set the semi-axes, use that.
    opt.datum = cartography::Datum("User Specified Datum",
                                   "User Specified Spheroid",
                                   "Reference Meridian",
                                   opt.semi_major, opt.semi_minor, 0.0);
    opt.datum_str = opt.datum.name();
  }else{ // Datum not specified
    if ( !opt.gcp_files.empty() || !opt.camera_position_file.empty() )
      vw_throw( ArgumentErr() << "When ground control points or a camera position file are used, "
                              << "the datum must be specified.\n" << usage << general_options );

    if ( opt.elevation_limit[0] < opt.elevation_limit[1] )
      vw_throw( ArgumentErr() << "When filtering by elevation limit, the datum must be specified.\n"
                << usage << general_options );
  }
  
  vw_out() << "Will use the datum:\n" << opt.datum << std::endl;
  
  asp::stereo_settings().datum = opt.datum.name(); // for RPC
  
  if ( opt.out_prefix.empty() )
    vw_throw( ArgumentErr() << "Missing output prefix.\n" << usage << general_options  );

  // Create the output directory
  vw::create_out_dir(opt.out_prefix);

  // Turn on logging to file
  asp::log_to_file(argc, argv, "", opt.out_prefix);

  opt.load_intrinsics_options(intrinsics_to_float_str, intrinsics_to_share_str);

  opt.save_iteration = vm.count("save-iteration-data");
  boost::to_lower( opt.cost_function );

  if (opt.initial_transform_file != "") {
    std::ifstream is(opt.initial_transform_file.c_str());
    for (size_t row = 0; row < opt.initial_transform.rows(); row++){
      for (size_t col = 0; col < opt.initial_transform.cols(); col++){
        double a;
        if (! (is >> a) )
          vw_throw( vw::IOErr() << "Failed to read initial transform from: "
                                << opt.initial_transform_file << "\n" );
        opt.initial_transform(row, col) = a;
      }
    }
    vw_out() << "Initial transform:\n" << opt.initial_transform << std::endl;
  }

  // Parse the indices of cameras not to float
  if (opt.fixed_cameras_indices_str != "") {
    opt.fixed_cameras_indices.clear();
    std::istringstream is(opt.fixed_cameras_indices_str);
    int val;
    while (is >> val) {
      opt.fixed_cameras_indices.insert(val);
      if (val < 0 || val >= (int)opt.image_files.size()) 
        vw_throw( vw::IOErr() << "The camera index to keep fixed " << val
                              << " is out of bounds.\n" );
    }
  }

  if (opt.reference_terrain != "") {
    std::string file_type = asp::get_cloud_type(opt.reference_terrain);
    if (file_type == "CSV" && opt.csv_format_str == "") 
      vw_throw( ArgumentErr() << "When using a csv reference terrain, "
                              << "must specify the csv-format.\n");
    if (opt.datum_str == "")
      vw_throw( ArgumentErr() << "When using a reference terrain, must specify the datum.\n");
    if (opt.disparity_list == "") 
      vw_throw( ArgumentErr() << "When using a reference terrain, must specify a list "
                              << "of disparities.\n");
    if (opt.max_disp_error <= 0) 
      vw_throw( ArgumentErr() << "Must specify --max-disp-error in pixels as a positive value.\n");
    if (opt.reference_terrain_weight < 0) 
      vw_throw( ArgumentErr() << "The value of --reference-terrain-weight must be non-negative.\n");
  }
  
}


// ================================================================================

int main(int argc, char* argv[]) {

  Options opt;
  try {
    xercesc::XMLPlatformUtils::Initialize();

    handle_arguments( argc, argv, opt );

    const int num_images = opt.image_files.size();

    // Assign the images which this instance should compute statistics for.
    std::vector<size_t> image_stats_indices;
    for (size_t i=opt.instance_index; i<num_images; i+=opt.instance_count)
      image_stats_indices.push_back(i);
    
    // Compute statistics for the designated images
    for (size_t i=0; i<image_stats_indices.size(); ++i) {
      
      size_t index = image_stats_indices[i];
      
      std::string image_path  = opt.image_files [index];
      std::string camera_path = opt.camera_files[index];

      // Call a bunch of stuff to get the nodata value
      SessionPtr session(asp::StereoSessionFactory::create(opt.stereo_session_string, opt,
                                                            image_path,  image_path,
                                                            camera_path, camera_path,
                                                            opt.out_prefix));
      boost::shared_ptr<DiskImageResource> rsrc(vw::DiskImageResourcePtr(image_path));
      float nodata, dummy;
      session->get_nodata_values(rsrc, rsrc, nodata, dummy);

      // Set up the image view
      DiskImageView<float> image_view(rsrc);
      ImageViewRef< PixelMask<float> > masked_image
        = create_mask_less_or_equal(image_view,  nodata);

      // Use caching function call to compute the image statistics.
      asp::StereoSession::gather_stats(masked_image, image_path,
                                       opt.out_prefix, image_path);
    }
    if (opt.stop_after_stats){
      vw_out() << "Quitting after statistics computation.\n";
      return 0;
    }
    // Done computing image statistics.
      
    
    
    
    // Create the stereo session. This will attempt to identify the session type.
    // Read in the camera model and image info for the input images.
    for (int i = 0; i < num_images; i++){
      vw_out(DebugMessage,"asp") << "Loading: " << opt.image_files [i] << ' '
                                                << opt.camera_files[i] << "\n";

      // The same camera is double-loaded into the same session instance.
      // TODO: One day replace this with a simpler camera model loader class
      SessionPtr session(asp::StereoSessionFactory::create(opt.stereo_session_string, opt,
                                                           opt.image_files [i], opt.image_files [i],
                                                           opt.camera_files[i], opt.camera_files[i],
                                                           opt.out_prefix));
      
      opt.camera_models.push_back(session->camera_model(opt.image_files [i],
                                                        opt.camera_files[i]));
      if (opt.approximate_pinhole_intrinsics) {
        boost::shared_ptr<vw::camera::PinholeModel> pinhole_ptr = 
                boost::dynamic_pointer_cast<vw::camera::PinholeModel>(opt.camera_models.back());
        // Replace lens distortion with fast approximation
        vw::camera::update_pinhole_for_fast_point2pixel<TsaiLensDistortion, 
            TsaiLensDistortion::num_distortion_params>(*(pinhole_ptr.get()),
                                                       file_image_size(opt.image_files[i]));
      }

    } // End loop through images loading all the camera models

    // Create match files from mapprojection.
    if (opt.mapprojected_data != "")
      create_matches_from_mapprojected_images(opt);

    // Create match files from mapprojection.
    if (opt.gcp_from_mapprojected != "") {
      create_gcp_from_mapprojected_images(opt);
      return 0;
    }

    // Create the match points.
    // Iterate through each pair of input images

    // Load estimated camera positions if they were provided.
    std::vector<Vector3> estimated_camera_gcc;
    load_estimated_camera_positions(opt, estimated_camera_gcc);
    const bool got_est_cam_positions =
      (estimated_camera_gcc.size() == static_cast<size_t>(num_images));

    // Find interest points between all of the image pairs.
    int num_pairs_matched = 0;

    // Make a list of all of the image pairs to find matches for.
    std::vector<std::pair<int,int> > all_pairs;
    for (int i = 0; i < num_images; i++){
      for (int j = i+1; j <= std::min(num_images-1, i+opt.overlap_limit); j++){

        // Apply the overlap list if manually specified.
        if (!opt.overlap_list.empty()) {
          std::pair<std::string, std::string> pair(opt.image_files[i],
                                                   opt.image_files[j]);
          if (opt.overlap_list.find(pair) == opt.overlap_list.end())
            continue;
        }

        // If this option is set, don't try to match cameras that are too far apart.
        if (got_est_cam_positions && (opt.position_filter_dist > 0)) {
          Vector3 this_pos  = estimated_camera_gcc[i];
          Vector3 other_pos = estimated_camera_gcc[j];
          if ( (this_pos  != Vector3(0,0,0)) && // If both positions are known
               (other_pos != Vector3(0,0,0)) && // and they are too far apart
               (norm_2(this_pos - other_pos) > opt.position_filter_dist) ) {
            vw_out() << "Skipping position: " << this_pos << " and "
                      << other_pos << " with distance " << norm_2(this_pos - other_pos)
                      << std::endl;
            continue; // Skip this image pair
          }
        }

        all_pairs.push_back(std::pair<int,int>(i,j));
      }
    }

    // TODO: Make this a function
    // Assign the matches which this instance should compute.
    size_t per_instance = all_pairs.size()/opt.instance_count; // Round down
    size_t remainder    = all_pairs.size()%opt.instance_count;
    size_t start_index  = 0, this_count = 0;
    for (size_t i=0; i<=opt.instance_index; ++i) {
      this_count = per_instance;
      if (i < remainder)
        ++this_count;
      start_index += this_count;
    }
    start_index -= this_count;

    std::vector<std::pair<int,int> > this_instance_pairs;
    for (size_t i=0; i<this_count; ++i)
      this_instance_pairs.push_back(all_pairs[i+start_index]);

    // Now process the selected pairs
    for (size_t k=0; k<this_instance_pairs.size(); ++k) {
      const int i = this_instance_pairs[k].first;
      const int j = this_instance_pairs[k].second;

      std::string image1_path  = opt.image_files[i];
      std::string image2_path  = opt.image_files[j];

      // Load both images into a new StereoSession object and use it to find interest points.
      // - The points are written to a file on disk.
      std::string camera1_path   = opt.camera_files[i];
      std::string camera2_path   = opt.camera_files[j];
      std::string match_filename = ip::match_filename(opt.out_prefix, image1_path, image2_path);
      std::string ip_file1       = ip::ip_filename(opt.out_prefix, image1_path);
      std::string ip_file2       = ip::ip_filename(opt.out_prefix, image2_path);
      opt.match_files[ std::pair<int, int>(i, j) ] = match_filename;

      // TODO: Need to make sure this works with the parallel script!
      bool inputs_changed = (!asp::is_latest_timestamp(match_filename,
                                                       image1_path,  image2_path,
                                                       camera1_path, camera2_path));

      // We make an exception and not rebuild if explicitly asked
      if (asp::stereo_settings().force_reuse_match_files &&
          boost::filesystem::exists(match_filename))
        inputs_changed = false;

      if (!inputs_changed) {
        vw_out() << "\t--> Using cached match file: " << match_filename << "\n";
        ++num_pairs_matched;
        continue;
      }
      if (opt.skip_matching)
        continue;
      boost::shared_ptr<DiskImageResource>
        rsrc1(vw::DiskImageResourcePtr(image1_path)),
        rsrc2(vw::DiskImageResourcePtr(image2_path));
      if ( (rsrc1->channels() > 1) || (rsrc2->channels() > 1) )
        vw_throw(ArgumentErr() << "Error: Input images can only have a single channel!\n\n");
      float nodata1, nodata2;
      SessionPtr session(asp::StereoSessionFactory::create(opt.stereo_session_string, opt,
                                                            image1_path,  image2_path,
                                                            camera1_path, camera2_path,
                                                            opt.out_prefix));

      session->get_nodata_values(rsrc1, rsrc2, nodata1, nodata2);
      try{
        // IP matching may not succeed for all pairs

        // Get masked views of the images to get statistics from
        DiskImageView<float> image1_view(rsrc1), image2_view(rsrc2);
        ImageViewRef< PixelMask<float> > masked_image1
          = create_mask_less_or_equal(image1_view,  nodata1);
        ImageViewRef< PixelMask<float> > masked_image2
          = create_mask_less_or_equal(image2_view, nodata2);

        // Since we computed statistics earlier, this will just be loading files.
        vw::Vector<vw::float32,6> image1_stats, image2_stats;
        image1_stats = asp::StereoSession::gather_stats(masked_image1,
                                                        image1_path, opt.out_prefix, image1_path);
        image2_stats = asp::StereoSession::gather_stats(masked_image2,
                                                        image2_path, opt.out_prefix, image2_path);

        // The match files are cached unless the images or camera
        // are newer than them. The IP files are cached for certain
        // IP matching options.
        session->ip_matching(image1_path, image2_path,
                              Vector2(masked_image1.cols(), masked_image1.rows()),
                              image1_stats, image2_stats,
                              opt.ip_per_tile,
                              nodata1, nodata2,
                              opt.camera_models[i].get(),
                              opt.camera_models[j].get(),
                              match_filename, ip_file1, ip_file2
                            );

        // Compute the coverage fraction
        std::vector<ip::InterestPoint> ip1, ip2;
        ip::read_binary_match_file(match_filename, ip1, ip2);
        int right_ip_width = rsrc1->cols()*
                              static_cast<double>(100-opt.ip_edge_buffer_percent)/100.0;
        Vector2i ip_size(right_ip_width, rsrc1->rows());
        double ip_coverage = asp::calc_ip_coverage_fraction(ip2, ip_size);
        vw_out() << "IP coverage fraction = " << ip_coverage << std::endl;

        ++num_pairs_matched;
      } catch ( const std::exception& e ){
        vw_out() << "Could not find interest points between images "
                  << opt.image_files[i] << " and " << opt.image_files[j] << std::endl;
        vw_out(WarningMessage) << e.what() << std::endl;
      } //End try/catch
    } // End loop through all input image pairs

    if (opt.stop_after_matching){
      vw_out() << "Quitting after matches computation.\n";
      return 0;
    }

    // All the work happens here! It also writes out the results.
    do_ba_ceres(opt, estimated_camera_gcc);

    xercesc::XMLPlatformUtils::Terminate();

  } ASP_STANDARD_CATCHES;
}
