// __BEGIN_LICENSE__
//  Copyright (c) 2009-2026, United States Government as represented by the
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

// TODO(oalexan1): Break this up into several files grouped by functionality.
// See existing BundleAdjustCamera.cc, etc.
#include <asp/Sessions/BundleAdjustParse.h>
#include <asp/Sessions/StereoSession.h>
#include <asp/Sessions/CameraUtils.h>
#include <asp/Sessions/BundleAdjustSession.h>
#include <asp/Camera/BundleAdjustOptions.h>
#include <asp/Camera/BundleAdjustResiduals.h>
#include <asp/Camera/BundleAdjustOutliers.h>
#include <asp/Camera/BundleAdjustIsis.h>
#include <asp/Camera/BundleAdjustEigen.h>
#include <asp/Camera/BaseCostFuns.h>
#include <asp/Camera/BundleAdjustCostFuns.h>
#include <asp/Camera/BundleAdjustOrbital.h>
#include <asp/Camera/LinescanUtils.h>
#include <asp/Camera/CsmModel.h>
#include <asp/Core/StereoSettings.h>
#include <asp/Core/Nvm.h>
#include <asp/Core/PointUtils.h>
#include <asp/Core/Macros.h>
#include <asp/Core/StereoSettings.h>
#include <asp/Core/IpMatchingAlgs.h>
#include <asp/Core/ImageUtils.h>
#include <asp/Core/ImageNormalization.h>
#include <asp/Core/OutlierProcessing.h>
#include <asp/Core/BundleAdjustUtils.h>
#include <asp/Core/InterestPointMatching.h>
#include <asp/Core/FileUtils.h>

#include <vw/Camera/CameraUtilities.h>
#include <vw/Core/CmdUtils.h>
#include <vw/FileIO/MatrixIO.h>
#include <vw/InterestPoint/MatcherIO.h>
#include <vw/Cartography/GeoTransform.h>
#include <vw/Cartography/GeoReferenceBaseUtils.h>
#include <vw/Cartography/DatumUtils.h>
#include <vw/BundleAdjustment/ControlNetworkLoader.h>
#include <vw/BundleAdjustment/CameraRelation.h>
#include <vw/Camera/PinholeModel.h>
#include <vw/Camera/LensDistortion.h>
#include <vw/Camera/OpticalBarModel.h>
#include <vw/FileIO/FileTypes.h>
#include <vw/FileIO/FileUtils.h>

// Can't do much about warnings in boost except to hide them
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
#include <boost/algorithm/string.hpp>
#include <boost/filesystem/operations.hpp>
#include <boost/filesystem/fstream.hpp>
#include <boost/foreach.hpp>
#pragma GCC diagnostic pop

#include <xercesc/util/PlatformUtils.hpp>

#include <set>

namespace fs = boost::filesystem;

using namespace vw;
using namespace asp;
using namespace vw::camera;
using namespace vw::ba;

// A callback to invoke at each iteration if desiring to save the cameras
// at that time.
class BaCallback: public ceres::IterationCallback {
public:

  BaCallback(asp::BaOptions const& opt, asp::BaState const& ba_state):
    m_opt(opt), m_ba_state(ba_state) {}

  virtual ceres::CallbackReturnType operator() (const ceres::IterationSummary& summary) {
    saveUpdatedCameras(m_opt, m_ba_state);
    return ceres::SOLVER_CONTINUE;
  }

private:
  asp::BaOptions const& m_opt;
  asp::BaState const& m_ba_state;
};

// One pass of bundle adjustment
int baOnePass(asp::BaOptions                & opt,
              asp::CRN                const & crn,
              bool                            first_pass,
              bool                            remove_outliers,
              asp::BaState                  & ba_state, // output
              asp::BaState const            & orig_ba_state,
              std::vector<vw::CamPtr>  const& orig_cams,
              std::vector<std::vector<vw::Vector3>> const& orig_cam_positions,
              asp::OrbitalGroups            & orbital_groups,
              bool                          & convergence_reached,
              double                        & final_cost) {

  ControlNetwork & cnet = *opt.cnet;
  int num_cameras = ba_state.num_cameras();
  int num_points  = ba_state.num_points();

  if ((int)crn.size() != num_cameras)
    vw_throw(ArgumentErr() << "Book-keeping error, the size of CameraRelationNetwork "
             << "must equal the number of images.\n");

  convergence_reached = true;

  if (opt.proj_win != BBox2(0, 0, 0, 0) && (!opt.proj_str.empty()))
    asp::filterOutliersProjWin(opt, ba_state, cnet);

  // How many times an xyz point shows up in the problem
  std::vector<int> count_map(num_points);
  for (int i = 0; i < num_points; i++) {
    if (ba_state.get_point_outlier(i))
      count_map[i] = 0; // skip outliers
    else
      count_map[i] = cnet[i].size(); // Get number of observations of this point.
  }

  // We will optimize multipliers of the intrinsics. This way each intrinsic
  // changes by a scale specific to it. Note: See --min-distortion for initial
  // values of the lens distortion coeffs when distortion is optimized.

  // Prepare for the DEM constraint.
  // TODO(oalexan1): Study how to best pass the DEM to avoid the code
  // below not being slow. It is not clear if the DEM tiles are cached
  // when passing around an ImageViewRef.
  bool have_dem = (!opt.heights_from_dem.empty());
  std::vector<Vector3> dem_xyz_vec;
  vw::cartography::GeoReference dem_georef;
  ImageViewRef<PixelMask<double>> masked_dem;
  std::set<int> outliers;
  if (have_dem) {
    for (int ipt = 0; ipt < num_points; ipt++) {
      if (ba_state.get_point_outlier(ipt))
        outliers.insert(ipt);
    }
  }
  if (opt.heights_from_dem != "") {
    vw::vw_out() << "Constraining against DEM: " << opt.heights_from_dem << "\n";
    asp::create_masked_dem(opt.heights_from_dem, dem_georef, masked_dem);
    // Re-triangulate the DEM points against the current cameras, not the frozen
    // opt.camera_models. In bundle_adjust the base cameras are never updated in the
    // pass loop; the latest state is base + ba_state, so rebuild them via
    // calcOptimizedCameras. This keeps the DEM anchors tracking the cameras across
    // passes instead of pinning to the starting geometry. (jitter_solve updates its
    // cameras in place each pass, so it does not need this.)
    std::vector<vw::CamPtr> curr_cams;
    asp::calcOptimizedCameras(opt, ba_state, curr_cams);
    asp::updateTriPtsFromDem(cnet, outliers, curr_cams,
                             dem_georef, masked_dem,
                             dem_xyz_vec); // output
  }

  // If to use a weight image
  bool have_weight_image = (!opt.weight_image.empty());
  vw::ImageViewRef<vw::PixelMask<float>> weight_image;
  float weight_image_nodata = -std::numeric_limits<float>::max();
  vw::cartography::GeoReference weight_image_georef;
  if (have_weight_image)
    vw::cartography::readGeorefImage(opt.weight_image,
      weight_image_nodata, weight_image_georef, weight_image);

  // Add the various cost functions the solver will optimize over.
  ceres::Problem problem;

  // Handle fixed distortion indices. Manually freeing up this pointer at the
  // end results in a crash.
  ceres::SubsetManifold *dist_opts = NULL;
  if (!opt.fixed_distortion_indices.empty())
    dist_opts = new ceres::SubsetManifold(ba_state.m_max_num_dist_params,
                                          opt.fixed_distortion_indices);

  // Pixel reprojection error
  std::vector<size_t> cam_residual_counts, num_pixels_per_cam;
  std::vector<std::vector<vw::Vector2>> pixels_per_cam;
  std::vector<std::vector<vw::Vector3>> tri_points_per_cam;
  std::vector<std::map<int, vw::Vector2>> pixel_sigmas;
  asp::addPixelReprojCostFun(opt, crn, count_map, weight_image, weight_image_georef,
                             dem_xyz_vec, have_weight_image, have_dem, orbital_groups,
                             // Outputs
                             cnet, ba_state, dist_opts, problem, cam_residual_counts,
                             num_pixels_per_cam, pixels_per_cam, tri_points_per_cam,
                             pixel_sigmas);

  // Add ground control points or points based on a DEM constraint
  int num_gcp = 0, num_gcp_or_dem_residuals = 0;
  asp::addGcpOrDemConstraint(opt, opt.cost_function, opt.use_llh_error, opt.fix_gcp_xyz,
                             // Outputs
                             cnet, num_gcp, num_gcp_or_dem_residuals,
                             ba_state, problem);

  // Add camera constraints
  if (opt.camera_weight > 0) {
    for (int icam = 0; icam < num_cameras; icam++) {
      double const* orig_cam_ptr = orig_ba_state.get_camera_ptr(icam);
      ceres::CostFunction* cost_function = CamError::Create(orig_cam_ptr, opt.camera_weight);

      // Don't use the same loss function as for pixels since that one discounts
      //  outliers and the cameras should never be discounted.
      // TODO(oalexan1): This will prevent convergence in some cases!
      ceres::LossFunction* loss_function = new ceres::TrivialLoss();

      double * camera  = ba_state.get_camera_ptr(icam);
      problem.AddResidualBlock(cost_function, loss_function, camera);
    } // End loop through cameras.
  }

  // Finer level control of only rotation. See also --camera-position-weight.
  // Note: A strong constraint here can prevent convergence as the is no loss function.
  // Camera position and tri constraints are suggested instead.
  if (opt.rotation_weight > 0) {
    for (int icam = 0; icam < num_cameras; icam++) {
      double const* orig_cam_ptr = orig_ba_state.get_camera_ptr(icam);
      double translation_weight = 0.0; // This is handled separately
      ceres::CostFunction* cost_function
        = RotTransError::Create(orig_cam_ptr, opt.rotation_weight, translation_weight);
      ceres::LossFunction* loss_function = new ceres::TrivialLoss();
      double * camera  = ba_state.get_camera_ptr(icam);
      problem.AddResidualBlock(cost_function, loss_function, camera);
    }
  }

  // For cameras in an orbital group, the position is derived from the shared group
  // pose, so fix the per-camera translation before the solve.
  asp::fixGroupedCameraTranslations(orbital_groups, ba_state, problem);

  // Soft constraint keeping each camera position near its original value.
  int num_uncertainty_residuals = 0;
  asp::addCamPositionUncertaintyCostFun(opt, orig_cams, orig_ba_state, orbital_groups,
                                        ba_state, problem, num_uncertainty_residuals);

  // Add a soft constraint to keep the cameras near the original position. Add one
  // constraint per reprojection error.
  int num_cam_pos_residuals = 0;
  if (opt.camera_position_weight > 0)
    asp::addCamPosCostFun(opt, orig_ba_state, pixels_per_cam,
                          tri_points_per_cam, pixel_sigmas, orig_cams,
                          ba_state, problem, num_cam_pos_residuals);

  // Add a cost function meant to tie up to known disparity
  // (option --reference-terrain).
  std::vector<vw::Vector3> reference_vec; // must be persistent
  std::vector<ImageViewRef<DispPixelT>> interp_disp; // must be persistent
  if (opt.reference_terrain != "")
    asp::addRefTerrainCostFun(opt, ba_state, problem,
                              reference_vec, interp_disp);

  // Add a ground constraints to keep points close to their initial positions
  int num_tri_residuals = 0;
  if (opt.tri_weight > 0)
    asp::addTriConstraint(opt, cnet, crn, opt.image_files, orig_cams,
                          opt.tri_weight, opt.cost_function, opt.tri_robust_threshold,
                          // Outputs
                          ba_state, problem, num_tri_residuals);

  const size_t MIN_KML_POINTS = 50;
  size_t kmlPointSkip = 30;
  // Figure out a good KML point skip amount
  if (num_points / kmlPointSkip < MIN_KML_POINTS)
    kmlPointSkip = num_points / MIN_KML_POINTS;
  if (kmlPointSkip < 1)
    kmlPointSkip = 1;

  if (first_pass) {
    vw_out() << "Writing initial condition files." << "\n";
    std::string residual_prefix = opt.out_prefix + "-initial_residuals";
    write_residual_logs(residual_prefix, opt, ba_state,
                        cam_residual_counts, pixel_sigmas,
                        num_gcp_or_dem_residuals,
                        num_uncertainty_residuals, num_tri_residuals,
                        num_cam_pos_residuals,
                        reference_vec, cnet, crn, problem);

    std::string point_kml_path  = opt.out_prefix + "-initial_points.kml";
    ba_state.record_points_to_kml(point_kml_path, opt.datum,
                         kmlPointSkip, "initial_points");
  }

  // Solve the problem
  ceres::Solver::Options options;
  options.gradient_tolerance  = 1e-16;
  options.function_tolerance  = 1e-16;
  options.parameter_tolerance = opt.parameter_tolerance;
  options.max_num_iterations  = opt.num_iterations;
  options.max_num_consecutive_invalid_steps = std::max(5, opt.num_iterations/5); // try hard
  options.minimizer_progress_to_stdout = true;

  if (opt.single_threaded_cameras)
    options.num_threads = 1;
  else
    options.num_threads = opt.num_threads;

  // Use a callback function at every iteration, if desired to save the intermediate results
  BaCallback callback(opt, ba_state);
  if (opt.save_intermediate_cameras) {
    options.callbacks.push_back(&callback);
    options.update_state_every_iteration = true;
  }

  // Set solver options according to the recommendations in the Ceres solving FAQs
  options.linear_solver_type = ceres::SPARSE_SCHUR;
  if (num_cameras < 100)
    options.linear_solver_type = ceres::DENSE_SCHUR;
  if (num_cameras > 3500) {
    // This is supposed to help with speed in a certain size range
    options.use_explicit_schur_complement = true;
    options.linear_solver_type  = ceres::ITERATIVE_SCHUR;
    options.preconditioner_type = ceres::SCHUR_JACOBI;
  }
  if (num_cameras > 7000)
    options.use_explicit_schur_complement = false; // Only matters with ITERATIVE_SCHUR

  //options.ordering_type = ceres::SCHUR;
  //options.eta = 1e-3; // FLAGS_eta;
  //options->max_solver_time_in_seconds = FLAGS_max_solver_time;
  //if (FLAGS_line_search) {
  //  options->minimizer_type = ceres::LINE_SEARCH;
  //}

  vw_out() << "Starting the Ceres optimizer." << "\n";
  ceres::Solver::Summary summary;
  ceres::Solve(options, &problem, &summary);
  final_cost = summary.final_cost;
  vw_out() << summary.FullReport() << "\n";
  if (summary.termination_type == ceres::NO_CONVERGENCE) {
    // Print a clarifying message, so the user does not think that the algorithm failed.
    vw_out() << "Found a valid solution, but did not reach the actual minimum. This is expected and likely the produced solution is good enough.\n";
    convergence_reached = false;
  }

  // For cameras in an orbital group, the optimized position lives in the shared group
  // pose. Write the derived positions back into each camera's translation adjustment,
  // so all downstream code (final residuals, camera writing) sees the correct positions.
  asp::updateGroupedCameraPositions(orbital_groups, ba_state);

  // Write the condition files after each pass, as we never know which pass will be the last
  // since we may stop the passes prematurely if no more outliers are present.
  vw_out() << "Writing final condition log files." << "\n";
  std::string residual_prefix = opt.out_prefix + "-final_residuals";
  write_residual_logs(residual_prefix, opt, ba_state,
                      cam_residual_counts, pixel_sigmas,
                      num_gcp_or_dem_residuals,
                      num_uncertainty_residuals, num_tri_residuals,
                      num_cam_pos_residuals,
                      reference_vec, cnet, crn, problem);

  std::string point_kml_path = opt.out_prefix + "-final_points.kml";
  ba_state.record_points_to_kml(point_kml_path, opt.datum, kmlPointSkip,
                                "final_points");

  // Outlier filtering
  if (remove_outliers)
      add_to_outliers(cnet, crn,
                      ba_state,   // in-out
                      opt, cam_residual_counts, pixel_sigmas, num_gcp_or_dem_residuals,
                      num_uncertainty_residuals, num_tri_residuals,
                      num_cam_pos_residuals, reference_vec, problem);

  return 0;
} // End function baOnePass

// Run several more passes with random initial parameter offsets. This flow is
// only kicked in if opt.num_random_passes is positive, which is not the
void runRandomPasses(asp::BaOptions & opt, asp::BaState & ba_state,
                     double & final_cost, asp::CRN const& crn,
                     bool remove_outliers,
                     asp::BaState const& orig_ba_state) {

  // Record the parameters of the best result so far
  double best_cost = final_cost;
  boost::shared_ptr<asp::BaState> best_params_ptr(new asp::BaState(ba_state));

  // Must recompute these, as what is passed in as orig_ba_state is actually
  // the latest parameters after optimization, not the original ones. 
  // TODO(oalexan1): Think of this more. All this runRundomPasses logic
  // may need to go away.
  std::vector<vw::CamPtr> orig_cams;
  asp::calcOptimizedCameras(opt, orig_ba_state, orig_cams); // orig cameras

  std::vector<std::vector<vw::Vector3>> orig_cam_positions;
  asp::calcCameraCenters(opt.stereo_session, orig_cams, orig_cam_positions);

  // Orbital groups are disallowed with random passes (see handleBaArgs), so pass an
  // empty groups object here. The randomization would not perturb the shared group
  // pose, which would make grouped positions look frozen and mislead the search.
  asp::OrbitalGroups orbital_groups;

  // Back up the output prefix
  std::string orig_out_prefix = opt.out_prefix;

  for (int pass = 0; pass < opt.num_random_passes; pass++) {

    vw_out() << "\n--> Running bundle adjust pass " << pass
             << " with random initial parameter offsets.\n";

    // Randomly distort the original inputs.
    ba_state.randomize_cameras();
    if (opt.solve_intrinsics)
      ba_state.randomize_intrinsics(opt.intrinsics_limits);

    // Write output files to a temporary prefix
    opt.out_prefix = orig_out_prefix + "_rand";

    // Do another pass of bundle adjustment.
    bool first_pass = true; // this needs more thinking
    bool convergence_reached = true;
    double curr_cost = 0.0; // will be set
    baOnePass(opt, crn, first_pass, remove_outliers,
              ba_state, orig_ba_state,
              orig_cams, orig_cam_positions, orbital_groups,
              convergence_reached, curr_cost);

    // Record the parameters of the best result.
    if (curr_cost < best_cost) {
      vw_out() << "  --> Found a better solution using random passes.\n";
      best_cost = curr_cost;
      best_params_ptr = boost::make_shared<asp::BaState>(ba_state);

      // Get a list of all the files that were generated in the random step.
      std::vector<std::string> rand_files;
      get_files_with_prefix(opt.out_prefix, rand_files);

      // Replace the existing output files with them.
      for (size_t i = 0; i < rand_files.size(); i++) {
        std::string new_path = rand_files[i];
        boost::replace_all(new_path, opt.out_prefix, orig_out_prefix);
        boost::filesystem::copy_file(rand_files[i], new_path,
                                     fs::copy_options::overwrite_existing);
      }
    }

    // Clear out the extra files that were generated
    std::string cmd("rm -f " + opt.out_prefix + "*");
    vw_out() << "Deleting temporary files: " << cmd << "\n";
    vw::exec_cmd(cmd.c_str());
  }
  opt.out_prefix = orig_out_prefix; // So the cameras are written to the expected paths.

  // Copy back to the original parameters
  ba_state = *best_params_ptr;

  // Copy back the best cost
  final_cost = best_cost;
}

/// Use Ceres to do bundle adjustment.
void do_ba_ceres(asp::BaOptions & opt, std::vector<Vector3> const& estimated_camera_gcc) {

  // Try to set up the control network, ie the list of point coordinates.
  // - This triangulates from the camera models to determine the initial
  //   world coordinate estimate for each matched IP.
  opt.cnet = boost::make_shared<ControlNetwork>("BundleAdjust");
  int num_gcp = 0;
  ControlNetwork & cnet = *(opt.cnet.get()); // alias to ASP cnet
  asp::IsisCnetData isisCnetData; // isis cnet (if loaded)
  std::vector<Eigen::Affine3d> world_to_cam; // for nvm (if applicable)
  std::map<std::string, Eigen::Vector2d> optical_offsets; // for nvm
  if (!opt.apply_initial_transform_only) {
    // TODO(oalexan1): This whole block must be a function
    if (opt.isis_cnet != "") {
      vw::vw_out() << "Reading ISIS control network: " << opt.isis_cnet << "\n";
      asp::loadIsisCnet(opt.isis_cnet, opt.image_files,
                        cnet, isisCnetData); // outputs
    } else if (opt.nvm != "") {
      // Assume the features are stored shifted relative to optical center
      bool nvm_no_shift = false;
      asp::readNvmAsCnet(opt.nvm, opt.image_files, nvm_no_shift,
                         cnet, world_to_cam, optical_offsets);// outputs
      // For pinhole and csm frame cameras also read the poses from nvm unless told not to
      if (!opt.no_poses_from_nvm)
        asp::updateCameraPoses(world_to_cam, opt.camera_models);
    } else {

      // Read matches into a control network
      bool triangulate_control_points = true;
      bool success = vw::ba::build_control_network(triangulate_control_points,
                                                   cnet, opt.camera_models,
                                                   opt.image_files,
                                                   opt.match_files,
                                                   opt.min_matches,
                                                   opt.min_triangulation_angle*(M_PI/180.0),
                                                   opt.forced_triangulation_distance,
                                                   opt.max_pairwise_matches,
                                                   stereo_settings().matches_as_txt,
                                                   opt.bathy_data);
      vw_out() << "Number of triangulated control points: " << cnet.size() << "\n";
      if (!success) {
        vw_out() << "Failed to build a control network.\n"
                 << " - Consider removing all .vwip and .match files and \n"
                 << "   increasing the number of interest points per tile using\n "
                 << "   --ip-per-tile, or decreasing --min-matches.\n"
                 << " - Check if your images are similar enough in illumination,\n"
                 << "   and if they have enough overlap.\n"
                 << "Will continue if ground control points are present.\n";
      }
    }
  }

  if (!opt.gcp_files.empty()) {
    num_gcp = vw::ba::add_ground_control_points(cnet, opt.gcp_files, opt.datum);
    checkGcpRadius(opt.datum, cnet);
    vw::vw_out() << "Loaded " << num_gcp << " ground control points.\n";
  }

  // If we change the cameras, we must rebuild the control network
  bool cameras_changed = false;

  // If camera positions were provided for local inputs, align to them.
  const bool have_est_camera_positions = (opt.camera_position_file != "");
  if ((opt.camera_type == BaCameraType_Pinhole) && have_est_camera_positions) {
    asp::init_pinhole_model_with_camera_positions(opt.cnet, opt.camera_models,
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
    if ((opt.camera_type == BaCameraType_Pinhole) &&
        !have_est_camera_positions) {
      if (opt.transform_cameras_using_gcp) {
        asp::transform_cameras_with_indiv_image_gcp(opt.cnet, opt.camera_models);
        cameras_changed = true;
      } else if (opt.transform_cameras_with_shared_gcp) {
        asp::transform_cameras_with_shared_gcp(opt.cnet, opt.camera_models);
            cameras_changed = true;
      } else if (opt.init_camera_using_gcp) {
        asp::init_camera_using_gcp(opt.cnet, opt.camera_models);
            cameras_changed = true;
      }
    }
  }

  int num_points = cnet.size();
  int num_cameras = opt.image_files.size();

  // This is important to prevent a crash later
  if (num_points == 0 && !opt.apply_initial_transform_only) {
    vw_out() << "No points to optimize (GCP or otherwise). Cannot continue.\n";
    return;
  }

  // Calculate the max length of a distortion vector. It will be convenient to
  // have all distortion vectors have this max length, even if fewer are needed
  // for some cameras.
  int max_num_dist_params
    = asp::calcMaxNumDistParams(opt.camera_models, opt.camera_type,
                                opt.intrinsics_options, opt.intrinsics_limits);

  // This is needed to ensure distortion coefficients are not so small as to not
  // get optimized. This modifies the cameras and must happen before
  // ba_state is populated.
  if (opt.solve_intrinsics && !opt.apply_initial_transform_only)
    asp::ensureMinDistortion(opt.camera_models, opt.camera_type,
                             opt.intrinsics_options,
                             opt.fixed_distortion_indices,
                             max_num_dist_params,
                             opt.min_distortion);

  // Create the storage arrays for the variables we will adjust.
  asp::BaState ba_state(num_points, num_cameras,
                        // Distinguish when we solve for intrinsics
                        opt.camera_type != BaCameraType_Other,
                        max_num_dist_params,
                        opt.intrinsics_options);

  // Sync up any camera intrinsics that should be shared. Do this before
  // populating the param storage values.
  cameras_changed = cameras_changed ||
    syncUpInitialSharedParams(opt.camera_type, ba_state, opt.camera_models);

  // Fill in the camera and intrinsic parameters.
  std::vector<vw::CamPtr> new_cam_models;
  bool ans = false;
  switch (opt.camera_type) {
    case BaCameraType_Pinhole:
      ans = init_cams_pinhole(opt, ba_state,
                              opt.initial_transform_file, opt.initial_transform,
                              new_cam_models); break;
    case BaCameraType_OpticalBar:
      ans = init_cams_optical_bar(opt, ba_state,
                                  opt.initial_transform_file, opt.initial_transform,new_cam_models); break;
    case BaCameraType_CSM: // CSM while optimizing intrinsics
      ans = init_cams_csm(opt, ba_state,
                          opt.initial_transform_file, opt.initial_transform,
                          new_cam_models); break;
    case BaCameraType_Other:
      ans = init_cams(opt, ba_state,
                      opt.initial_transform_file, opt.initial_transform,
                      new_cam_models); break;
    default:
      vw_throw(ArgumentErr() << "Unknown camera type.\n");
  };

  // Certain input options change the cameras inside init_cams and we
  // need to update the point coordinates for the new cameras. It is
  // ok to leave the original vector of camera models unchanged.
  if (ans)
    cameras_changed = true;

  // When the cameras changed, must re-triangulate the points.
  // This is much cheaper than rebuilding the control network.
  if (!opt.apply_initial_transform_only && cameras_changed) {
    vw_out() << "Re-triangulating the control points as the cameras changed.\n";
    // Do not triangulate the GCP or the height-from-dem points
    vw::ba::triangulate_control_network(cnet, new_cam_models,
                                        opt.min_triangulation_angle*(M_PI/180.0),
                                        opt.forced_triangulation_distance,
                                        opt.bathy_data);
    if (num_points != cnet.size()) // Must not happen
      vw_throw(ArgumentErr() << "The number of points changed after re-triangulation.\n");
  }

  // Warn if the GCP sit far from the tie points, for example if the lat and lon
  // are swapped in the GCP file. The cnet is now in its final triangulated
  // state, whether or not the cameras changed above.
  if (opt.gcp_files.size() > 0)
    check_gcp_dists(opt.cnet);

  // Fill in the point vector with the starting values
  for (int ipt = 0; ipt < num_points; ipt++)
    ba_state.set_point(ipt, cnet[ipt].position());

  // Flag any outliers read from an isis cnet
  for (auto const& ipt: isisCnetData.isisOutliers) {
    if (ipt < 0 || ipt >= num_points)
      vw_throw(ArgumentErr() << "Invalid point index.\n");
    ba_state.set_point_outlier(ipt, true);
  }

  // Flag outliers in the cnet
  for (int ipt = 0; ipt < num_points; ipt++) {
    if (cnet[ipt].ignore())
      ba_state.set_point_outlier(ipt, true);
  }

  // The camera positions and orientations before we float them
  // This includes modifications from any initial transforms that were specified.
  asp::BaState orig_ba_state(ba_state);

  // TODO(oalexan1): Likely orig_cams have the info as new_cam_models. But need
  // to test this.
  std::vector<vw::CamPtr> orig_cams;
  asp::calcOptimizedCameras(opt, orig_ba_state, orig_cams); // orig cameras
  std::vector<std::vector<vw::Vector3>> orig_cam_positions;
  asp::calcCameraCenters(opt.stereo_session, orig_cams, orig_cam_positions);

  // Build the orbital groups (from --orbital-group-list) once, before the pass loop.
  // When active, the cameras of each orbit share one rigid 6-DOF pose that derives
  // their positions. Building it here (not per pass) lets the shared group pose
  // persist and accumulate across passes, like the per-camera adjustments and the
  // triangulated points. Its init_pos is the original camera centers, so the pose
  // holds the total rigid transform from the original orbit.
  asp::OrbitalGroups orbital_groups;
  asp::buildOrbitalGroups(opt, orig_cams, orbital_groups);

  // For nadirpinhole and pinhole cameras, save a report
  bool has_datum = (opt.datum.name() != asp::UNSPECIFIED_DATUM);
  if (has_datum && opt.stereo_session.find("pinhole") != std::string::npos)
    asp::saveCameraReport(opt, ba_state, opt.datum, "initial");

  // TODO(oalexan1): Is it possible to avoid using CRNs?
  asp::CRN crn;
  crn.from_cnet(cnet);

  if (opt.num_passes <= 0)
    vw_throw(ArgumentErr() << "Error: Expecting at least one bundle adjust pass.\n");

  bool remove_outliers = (opt.num_passes > 1);
  double final_cost = 0.0;
  for (int pass = 0; pass < opt.num_passes; pass++) {

    if (opt.apply_initial_transform_only)
      continue;

    vw_out() << "--> Bundle adjust pass: " << pass << "\n";

    bool first_pass = (pass == 0);
    bool convergence_reached = true; // will change
    baOnePass(opt, crn, first_pass, remove_outliers,
              ba_state, orig_ba_state,
              orig_cams, orig_cam_positions, orbital_groups,
              convergence_reached, final_cost);
    int num_points_remaining = num_points - ba_state.get_num_outliers();
    if (num_points_remaining < opt.min_matches && num_gcp == 0) {
      // Do not throw if there exist gcp, as maybe that's all there is, and there
      // can be just a few of them. Also, do not throw if we are using an ISIS cnet,
      // unless we have less than 10 points, as that one can make too few points.
      std::ostringstream os;
      os << "Too few points remain after filtering. Number of remaining "
         << "points is " << num_points_remaining
         << ", but value of --min-matches is " << opt.min_matches << ".\n";
      if (opt.isis_cnet != "" && num_points_remaining > 10)
        vw_out(vw::WarningMessage) << os.str();
      else
        vw_throw(ArgumentErr() << "Error: " << os.str());
    }
  } // End loop through passes

  // Running random passes is not the default
  if (!opt.apply_initial_transform_only && opt.num_random_passes > 0)
    runRandomPasses(opt, ba_state, final_cost, crn, remove_outliers, orig_ba_state);

  // Always save the updated cameras, even if we are not doing any optimization
  saveUpdatedCameras(opt, ba_state);

  // If we are only applying an initial transform, we are done
  if (opt.apply_initial_transform_only)
    return;

  // Find the cameras with the latest adjustments. Note that we do not modify
  // opt.camera_models, but make copies as needed.
  std::vector<vw::CamPtr> optimized_cams;
  asp::calcOptimizedCameras(opt, ba_state, optimized_cams);

  // Find the camera centers. For linescan, this will return all samples.
  std::vector<std::vector<vw::Vector3>> opt_cam_positions;
  asp::calcCameraCenters(opt.stereo_session, optimized_cams, opt_cam_positions);

  // Fetch the latest outliers from ba_state and put them in the 'outliers' set
  std::set<int> outliers;
  updateOutliers(cnet, ba_state, outliers);

  // Write the ground control point offset report
  if (num_gcp > 0) {
    std::vector<double> tri_points_vec(ba_state.num_points() * 3);
    for (int ipt = 0; ipt < ba_state.num_points(); ipt++) {
      vw::Vector3 pt = ba_state.get_point(ipt);
      for (int q = 0; q < 3; q++)
        tri_points_vec[ipt*3 + q] = pt[q];
    }
    asp::saveGcpReport(opt.out_prefix, cnet, tri_points_vec, outliers, opt.datum);
  }

  // Write clean matches and many types of stats. These are done together as
  // they rely on reloading interest point matches, which is expensive.
  bool save_clean_matches = true;
  asp::matchFilesProcessing(cnet,
                            asp::BaBaseOptions(opt), // note the slicing
                            optimized_cams, remove_outliers, outliers, opt.mapproj_dem,
                            opt.propagate_errors, opt.horizontal_stddev_vec,
                            save_clean_matches, opt.match_files,
                            stereo_settings().matches_as_txt);

  // Compute the change in camera centers. For that, we need the original cameras.
  std::string cam_offsets_file = opt.out_prefix + "-camera_offsets.txt";
  if (opt.datum.name() != asp::UNSPECIFIED_DATUM)
    asp::saveCameraOffsets(opt.datum, opt.image_files,
                           orig_cam_positions, opt_cam_positions,
                           cam_offsets_file);
  else
    vw::vw_out() << "Cannot compute camera offsets as the datum is unspecified.\n";

  std::string tri_offsets_file = opt.out_prefix + "-triangulation_offsets.txt";
  asp::saveTriOffsetsPerCamera(opt.image_files, orig_ba_state, ba_state, crn,
                               tri_offsets_file);

  if (has_datum &&
      (opt.stereo_session == "pinhole") || (opt.stereo_session == "nadirpinhole"))
    saveCameraReport(opt, ba_state, opt.datum, "final");

  // Save the updated cnet to ISIS or nvm format. Note that ba_state has
  // the latest triangulated points and outlier info, while the cnet has the
  // initially triangulated points and the interest point matches.
  if (opt.isis_cnet != "" && opt.output_cnet_type == "isis-cnet")
    asp::saveUpdatedIsisCnet(opt.out_prefix, cnet, ba_state, isisCnetData);
  else if (opt.output_cnet_type == "isis-cnet")
    asp::saveIsisCnet(opt.out_prefix, opt.datum, cnet, ba_state);
  else if (opt.output_cnet_type == "nvm") {
    asp::saveNvm(opt, opt.no_poses_from_nvm, cnet, ba_state,
                  world_to_cam, optical_offsets);
  }

  // Save the optimized control network in GCP format, after outlier filtering
  if (opt.save_cnet_as_gcp) {
    std::string gcp_file = opt.out_prefix + "-cnet.gcp";
    asp::saveCnetAsGcp(cnet, ba_state, opt.datum, opt.image_files, gcp_file);
  }

} // end do_ba_ceres

int main(int argc, char* argv[]) {

  asp::BaOptions opt;
  try {
    xercesc::XMLPlatformUtils::Initialize();

    // Process the bundle_adjust options and sanity checks
    handleBaArgs(argc, argv, opt);

    asp::load_cameras(opt.image_files, opt.camera_files, opt.out_prefix, opt,
                      opt.approximate_pinhole_intrinsics,
                      // Outputs
                      opt.stereo_session,
                      opt.single_threaded_cameras,
                      opt.camera_models);

    // Parse data needed for error propagation
    if (opt.propagate_errors)
      asp::setup_error_propagation(opt.stereo_session, opt.horizontal_stddev,
                                   opt.camera_models,
                                   opt.horizontal_stddev_vec); // output

    bool need_no_matches = (opt.apply_initial_transform_only ||
                            !opt.isis_cnet.empty()           ||
                            !opt.nvm.empty());

    // Read mapprojected images if using --mapprojected-data 
    std::vector<std::string> map_files;
    std::string mapproj_dem;
    asp::setupMapprojectedData(opt, need_no_matches, map_files, mapproj_dem);

    // The stats need to be for the mapprojected image, if provided
    std::vector<std::string> files_for_stats = opt.image_files;
    if (!map_files.empty())
      files_for_stats = map_files;

    // The file having the image normalization bounds  
    std::string boundsFile = opt.out_prefix + "-normalization-bounds.txt";

    // Compute stats in the batch of images given by opt.job_id, etc.
    // Skip this in several situations, including when we just want to accumulate
    // the stats for the images in the list.
    bool skip_stats = (need_no_matches || opt.skip_matching ||
                       opt.clean_match_files_prefix != ""   ||
                       opt.match_files_prefix != ""         ||
                       opt.calc_normalization_bounds);

    bool calcIp = false;
    if (!skip_stats)
      computeStatsOrIp(opt, files_for_stats, opt.dem_file_for_overlap,
                       boundsFile, calcIp);

    if (opt.stop_after_stats) {
      vw_out() << "Quitting after statistics computation.\n";
      xercesc::XMLPlatformUtils::Terminate();
      return 0;
    }

    // Compute normalization bounds if requested. This is done in a separate
    // process in parallel_bundle_adjust.
    if (opt.calc_normalization_bounds) {
      calcNormalizationBounds(opt.out_prefix, files_for_stats, boundsFile);
      vw_out() << "Quitting after calculating normalization bounds.\n";
      xercesc::XMLPlatformUtils::Terminate();
      return 0;
    }

    // Compute ip if requested. This is done in multiple processes in 
    // parallel_bundle_adjust. For standalone bundle_adjust, this will
    // happen when matching occurs, which is then serial.
    if (opt.calc_ip) {
      bool calcIp = true;
      computeStatsOrIp(opt, files_for_stats, opt.dem_file_for_overlap,
                       boundsFile, calcIp);
      vw_out() << "Quitting after computing interest points.\n";
      xercesc::XMLPlatformUtils::Terminate();
      return 0;
    }

    // Calculate which images overlap based on the DEM
    if (opt.auto_overlap_params != "") {
      opt.have_overlap_list = true;
      asp::buildOverlapList(opt.out_prefix,
                            opt.dem_file_for_overlap, opt.pct_for_overlap,
                            opt.overlap_limit,
                            opt.match_first_to_last,
                            opt.image_files, opt.camera_models,
                            opt.overlap_list); // output
    }

    // Load estimated camera positions if they were provided.
    std::vector<Vector3> estimated_camera_gcc;
    asp::loadEstimCameraPositions(opt, estimated_camera_gcc);

    // Find or list matches
    asp::findPairwiseMatches(opt, map_files, mapproj_dem,
                             estimated_camera_gcc, need_no_matches);

    if (opt.stop_after_matching) {
      vw_out() << "Quitting after matches computation.\n";
      return 0;
    }

    // All the work happens here. It also writes out the results.
    do_ba_ceres(opt, estimated_camera_gcc);

    xercesc::XMLPlatformUtils::Terminate();

  } ASP_STANDARD_CATCHES;
}
