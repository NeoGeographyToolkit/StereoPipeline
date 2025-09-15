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

/// \file bundle_adjust.cc

// TODO(oalexan1): Break this up into several files grouped by functionality.
// See existing BundleAdjustCamera.cc, etc.
#include <asp/Sessions/StereoSession.h>
#include <asp/Sessions/StereoSessionFactory.h>
#include <asp/Sessions/CameraUtils.h>
#include <asp/Sessions/BundleAdjustSession.h>
#include <asp/Camera/BundleAdjustOptions.h>
#include <asp/Camera/BundleAdjustResiduals.h>
#include <asp/Camera/BundleAdjustOutliers.h>
#include <asp/Camera/BundleAdjustIsis.h>
#include <asp/Camera/BundleAdjustEigen.h>
#include <asp/Camera/BaseCostFuns.h>
#include <asp/Camera/BundleAdjustCostFuns.h> // Ceres included in this file
#include <asp/Camera/LinescanUtils.h>
#include <asp/Camera/CsmModel.h>
#include <asp/Core/nvm.h>
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
#include <asp/Core/AspProgramOptions.h>

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
#include <boost/program_options.hpp>
#include <boost/filesystem/operations.hpp>
#include <boost/filesystem/fstream.hpp>
#include <boost/foreach.hpp>
#pragma GCC diagnostic pop

#include <xercesc/util/PlatformUtils.hpp>

namespace po = boost::program_options;
namespace fs = boost::filesystem;

using namespace vw;
using namespace asp;
using namespace vw::camera;
using namespace vw::ba;

// A callback to invoke at each iteration if desiring to save the cameras
// at that time.
class BaCallback: public ceres::IterationCallback {
public:

  BaCallback(asp::BaOptions const& opt, asp::BAParams const& param_storage):
    m_opt(opt), m_param_storage(param_storage) {}

  virtual ceres::CallbackReturnType operator() (const ceres::IterationSummary& summary) {
    saveUpdatedCameras(m_opt, m_param_storage);
    return ceres::SOLVER_CONTINUE;
  }

private:
  asp::BaOptions const& m_opt;
  asp::BAParams const& m_param_storage;
};


// One pass of bundle adjustment
int do_ba_ceres_one_pass(asp::BaOptions                & opt,
                         asp::CRNJ               const & crn,
                         bool                           first_pass,
                         bool                           remove_outliers,
                         asp::BAParams                 & param_storage, // output
                         asp::BAParams const           & orig_parameters,
                         std::vector<vw::CamPtr>  const& orig_cams,
                         std::vector<std::vector<vw::Vector3>> const& orig_cam_positions,
                         bool                          & convergence_reached,
                         double                        & final_cost) {

  ControlNetwork & cnet = *opt.cnet;
  int num_cameras = param_storage.num_cameras();
  int num_points  = param_storage.num_points();

  if ((int)crn.size() != num_cameras)
    vw_throw(ArgumentErr() << "Book-keeping error, the size of CameraRelationNetwork "
             << "must equal the number of images.\n");

  convergence_reached = true;

  if (opt.proj_win != BBox2(0, 0, 0, 0) && (!opt.proj_str.empty()))
    asp::filterOutliersProjWin(opt, param_storage, cnet);

  // How many times an xyz point shows up in the problem
  std::vector<int> count_map(num_points);
  for (int i = 0; i < num_points; i++) {
    if (param_storage.get_point_outlier(i))
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
  ImageViewRef<PixelMask<double>> interp_dem;
  std::set<int> outliers;
  if (have_dem) {
    for (int ipt = 0; ipt < num_points; ipt++) {
      if (param_storage.get_point_outlier(ipt))
        outliers.insert(ipt);
    }
  }
  if (opt.heights_from_dem != "") {
    vw::vw_out() << "Constraining against DEM: " << opt.heights_from_dem << "\n";
    asp::create_interp_dem(opt.heights_from_dem, dem_georef, interp_dem);
    asp::update_tri_pts_from_dem(cnet, crn, outliers, opt.camera_models,
                               dem_georef, interp_dem,
                               // Output
                               dem_xyz_vec);
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

  // Pixel reprojection error
  std::vector<size_t> cam_residual_counts, num_pixels_per_cam;
  std::vector<std::vector<vw::Vector2>> pixels_per_cam;
  std::vector<std::vector<vw::Vector3>> tri_points_per_cam;
  std::vector<std::map<int, vw::Vector2>> pixel_sigmas;
  asp::addPixelReprojCostFun(opt, crn, count_map, weight_image, weight_image_georef,
                             dem_xyz_vec, have_weight_image, have_dem,
                             // Outputs
                             cnet, param_storage, problem, cam_residual_counts,
                             num_pixels_per_cam, pixels_per_cam, tri_points_per_cam,
                             pixel_sigmas);

  // Add ground control points or points based on a DEM constraint
  int num_gcp = 0, num_gcp_or_dem_residuals = 0;
  asp::addGcpOrDemConstraint(opt, opt.cost_function, opt.use_llh_error, opt.fix_gcp_xyz,
                             // Outputs
                             cnet, num_gcp, num_gcp_or_dem_residuals,
                             param_storage, problem);

  // Add camera constraints
  if (opt.camera_weight > 0) {
    for (int icam = 0; icam < num_cameras; icam++) {
      double const* orig_cam_ptr = orig_parameters.get_camera_ptr(icam);
      ceres::CostFunction* cost_function = CamError::Create(orig_cam_ptr, opt.camera_weight);

      // Don't use the same loss function as for pixels since that one discounts
      //  outliers and the cameras should never be discounted.
      // TODO(oalexan1): This will prevent convergence in some cases!
      ceres::LossFunction* loss_function = new ceres::TrivialLoss();

      double * camera  = param_storage.get_camera_ptr(icam);
      problem.AddResidualBlock(cost_function, loss_function, camera);
    } // End loop through cameras.
  }

  // Finer level control of only rotation. See also --camera-position-weight.
  // Note: A strong constraint here can prevent convergence as the is no loss function.
  // Camera position and tri constraints are suggested instead.
  if (opt.rotation_weight > 0) {
    for (int icam = 0; icam < num_cameras; icam++) {
      double const* orig_cam_ptr = orig_parameters.get_camera_ptr(icam);
      double translation_weight = 0.0; // This is handled separately
      ceres::CostFunction* cost_function
        = RotTransError::Create(orig_cam_ptr, opt.rotation_weight, translation_weight);
      ceres::LossFunction* loss_function = new ceres::TrivialLoss();
      double * camera  = param_storage.get_camera_ptr(icam);
      problem.AddResidualBlock(cost_function, loss_function, camera);
    }
  }

  int num_uncertainty_residuals = 0;
  if (opt.camera_position_uncertainty.size() > 0) {
    for (int icam = 0; icam < num_cameras; icam++) {
      // orig_ctr has the actual camera center, but orig_cam_ptr may have only an adjustment.
      // For linescan camera, pick the camera center from the middle of the array of 
      // centers. It will be used to determine horizontal and vertical components.
      vw::Vector3 orig_ctr = orig_cams[icam]->camera_center(vw::Vector2());
      double const* orig_cam_ptr = orig_parameters.get_camera_ptr(icam);
      double * cam_ptr  = param_storage.get_camera_ptr(icam);
      int param_len = 6; // bundle_adjust and jitter_solve expect different lengths
      double weight = 1.0;
      ceres::CostFunction* cost_function
        = CamUncertaintyError::Create(orig_ctr, orig_cam_ptr, param_len,
                                      opt.camera_position_uncertainty[icam],
                                      weight, opt.datum,
                                      opt.camera_position_uncertainty_power);
      ceres::LossFunction* loss_function = new ceres::TrivialLoss();
      problem.AddResidualBlock(cost_function, loss_function, cam_ptr);
      num_uncertainty_residuals++;
    }
  }

  // Add a soft constraint to keep the cameras near the original position. Add one
  // constraint per reprojection error.
  int num_cam_pos_residuals = 0;
  if (opt.camera_position_weight > 0)
    asp::addCamPosCostFun(opt, orig_parameters, pixels_per_cam,
                          tri_points_per_cam, pixel_sigmas, orig_cams,
                          param_storage, problem, num_cam_pos_residuals);

  // Add a cost function meant to tie up to known disparity
  // (option --reference-terrain).
  std::vector<vw::Vector3> reference_vec; // must be persistent
  std::vector<ImageViewRef<DispPixelT>> interp_disp; // must be persistent
  if (opt.reference_terrain != "")
    asp::addReferenceTerrainCostFunction(opt, param_storage, problem,
                                         reference_vec, interp_disp);

  // Add a ground constraints to keep points close to their initial positions
  int num_tri_residuals = 0;
  if (opt.tri_weight > 0)
    asp::addTriConstraint(opt, cnet, crn, opt.image_files, orig_cams,
                          opt.tri_weight, opt.cost_function, opt.tri_robust_threshold,
                          // Outputs
                          param_storage, problem, num_tri_residuals);

  const size_t MIN_KML_POINTS = 50;
  size_t kmlPointSkip = 30;
  // Figure out a good KML point skip amount
  if (num_points / kmlPointSkip < MIN_KML_POINTS)
    kmlPointSkip = num_points / MIN_KML_POINTS;
  if (kmlPointSkip < 1)
    kmlPointSkip = 1;

  if (first_pass) {
    // Save the cnet
    if (opt.save_cnet_as_csv) {
      std::string cnet_file = opt.out_prefix + "-cnet.csv";
      vw_out() << "Writing: " << cnet_file << std::endl;
      cnet.write_in_gcp_format(cnet_file, opt.datum);
    }

    vw_out() << "Writing initial condition files." << std::endl;
    std::string residual_prefix = opt.out_prefix + "-initial_residuals";
    write_residual_logs(residual_prefix, opt, param_storage,
                        cam_residual_counts, pixel_sigmas,
                        num_gcp_or_dem_residuals,
                        num_uncertainty_residuals, num_tri_residuals,
                        num_cam_pos_residuals,
                        reference_vec, cnet, crn, problem);

    std::string point_kml_path  = opt.out_prefix + "-initial_points.kml";
    std::string url = "http://maps.google.com/mapfiles/kml/shapes/placemark_circle.png";
    param_storage.record_points_to_kml(point_kml_path, opt.datum,
                         kmlPointSkip, "initial_points", url);
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
  BaCallback callback(opt, param_storage);
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

  vw_out() << "Starting the Ceres optimizer." << std::endl;
  ceres::Solver::Summary summary;
  ceres::Solve(options, &problem, &summary);
  final_cost = summary.final_cost;
  vw_out() << summary.FullReport() << "\n";
  if (summary.termination_type == ceres::NO_CONVERGENCE) {
    // Print a clarifying message, so the user does not think that the algorithm failed.
    vw_out() << "Found a valid solution, but did not reach the actual minimum. This is expected and likely the produced solution is good enough.\n";
    convergence_reached = false;
  }

  // Write the condition files after each pass, as we never know which pass will be the last
  // since we may stop the passes prematurely if no more outliers are present.
  vw_out() << "Writing final condition log files." << std::endl;
  std::string residual_prefix = opt.out_prefix + "-final_residuals";
  write_residual_logs(residual_prefix, opt, param_storage,
                      cam_residual_counts, pixel_sigmas,
                      num_gcp_or_dem_residuals,
                      num_uncertainty_residuals, num_tri_residuals,
                      num_cam_pos_residuals,
                      reference_vec, cnet, crn, problem);

  std::string point_kml_path = opt.out_prefix + "-final_points.kml";
  std::string url
   = "http://maps.google.com/mapfiles/kml/shapes/placemark_circle_highlight.png";
  param_storage.record_points_to_kml(point_kml_path, opt.datum, kmlPointSkip,
                                     "final_points", url);

  // Outlier filtering
  if (remove_outliers)
      add_to_outliers(cnet, crn,
                      param_storage,   // in-out
                      opt, cam_residual_counts, pixel_sigmas, num_gcp_or_dem_residuals,
                      num_uncertainty_residuals, num_tri_residuals,
                      num_cam_pos_residuals, reference_vec, problem);

  return 0;
} // End function do_ba_ceres_one_pass

// Run several more passes with random initial parameter offsets. This flow is
// only kicked in if opt.num_random_passes is positive, which is not the
void runRandomPasses(asp::BaOptions & opt, asp::BAParams & param_storage,
                     double & final_cost, asp::CRNJ const& crn,
                     bool remove_outliers,
                     asp::BAParams const& orig_parameters) {

  // Record the parameters of the best result so far
  double best_cost = final_cost;
  boost::shared_ptr<asp::BAParams> best_params_ptr(new asp::BAParams(param_storage));

  // Must recompute these, as what is passed in as orig_parameters is actually
  // the latest parameters after optimization, not the original ones. 
  // TODO(oalexan1): Think of this more. All this runRundomPasses logic
  // may need to go away.
  std::vector<vw::CamPtr> orig_cams;
  asp::calcOptimizedCameras(opt, orig_parameters, orig_cams); // orig cameras
  
  std::vector<std::vector<vw::Vector3>> orig_cam_positions;
  asp::calcCameraCenters(opt.stereo_session, orig_cams, orig_cam_positions);

  // Back up the output prefix
  std::string orig_out_prefix = opt.out_prefix;

  for (int pass = 0; pass < opt.num_random_passes; pass++) {

    vw_out() << "\n--> Running bundle adjust pass " << pass
             << " with random initial parameter offsets.\n";

    // Randomly distort the original inputs.
    param_storage.randomize_cameras();
    if (opt.solve_intrinsics)
      param_storage.randomize_intrinsics(opt.intrinsics_limits);

    // Write output files to a temporary prefix
    opt.out_prefix = orig_out_prefix + "_rand";

    // Do another pass of bundle adjustment.
    bool first_pass = true; // this needs more thinking
    bool convergence_reached = true;
    double curr_cost = 0.0; // will be set
    do_ba_ceres_one_pass(opt, crn, first_pass, remove_outliers,
                         param_storage, orig_parameters,
                         orig_cams, orig_cam_positions,
                         convergence_reached, curr_cost);

    // Record the parameters of the best result.
    if (curr_cost < best_cost) {
      vw_out() << "  --> Found a better solution using random passes.\n";
      best_cost = curr_cost;
      best_params_ptr.reset(new asp::BAParams(param_storage));

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
    vw_out() << "Deleting temporary files: " << cmd << std::endl;
    vw::exec_cmd(cmd.c_str());
  }
  opt.out_prefix = orig_out_prefix; // So the cameras are written to the expected paths.

  // Copy back to the original parameters
  param_storage = *best_params_ptr;

  // Copy back the best cost
  final_cost = best_cost;
}

/// Use Ceres to do bundle adjustment.
void do_ba_ceres(asp::BaOptions & opt, std::vector<Vector3> const& estimated_camera_gcc) {

  // Try to set up the control network, ie the list of point coordinates.
  // - This triangulates from the camera models to determine the initial
  //   world coordinate estimate for each matched IP.
  opt.cnet.reset(new ControlNetwork("BundleAdjust"));
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
                                                   opt.max_pairwise_matches);
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
    vw::vw_out() << "Found " << opt.gcp_files.size() << " ground control point files.\n";
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

    // Issue a warning if the GCPs are far away from the camera coordinates.
    // Do it only if the cameras did not change, as otherwise the cnet is outdated.
    if (!cameras_changed)
      check_gcp_dists(opt.camera_models, opt.cnet, opt.forced_triangulation_distance);
  }

  int num_points = cnet.size();
  int num_cameras = opt.image_files.size();

  // This is important to prevent a crash later
  if (num_points == 0 && !opt.apply_initial_transform_only) {
    vw_out() << "No points to optimize (GCP or otherwise). Cannot continue.\n";
    return;
  }

  // Calculate the max length of a distortion vector
  int max_num_dist_params
    = asp::calcMaxNumDistParams(opt.camera_models, opt.camera_type,
                                opt.intrinsics_options, opt.intrinsics_limits);

  // This is needed to ensure distortion coefficients are not so small as to not get
  // optimized.
   if (opt.solve_intrinsics && !opt.apply_initial_transform_only)
     asp::ensureMinDistortion(opt.camera_models, opt.camera_type,
                              opt.intrinsics_options, opt.min_distortion);

  // Create the storage arrays for the variables we will adjust.
  asp::BAParams param_storage(num_points, num_cameras,
                              // Distinguish when we solve for intrinsics
                              opt.camera_type != BaCameraType_Other,
                              max_num_dist_params,
                              opt.intrinsics_options);

  // Sync up any camera intrinsics that should be shared. Do this before
  // populating the param storage values.
  cameras_changed = cameras_changed ||
    syncUpInitialSharedParams(opt.camera_type, param_storage, opt.camera_models);

  // Fill in the camera and intrinsic parameters.
  std::vector<vw::CamPtr> new_cam_models;
  bool ans = false;
  switch (opt.camera_type) {
    case BaCameraType_Pinhole:
      ans = init_cams_pinhole(opt, param_storage,
                              opt.initial_transform_file, opt.initial_transform,
                              new_cam_models); break;
    case BaCameraType_OpticalBar:
      ans = init_cams_optical_bar(opt, param_storage,
                                  opt.initial_transform_file, opt.initial_transform,new_cam_models); break;
    case BaCameraType_CSM: // CSM while optimizing intrinsics
      ans = init_cams_csm(opt, param_storage,
                          opt.initial_transform_file, opt.initial_transform,
                          new_cam_models); break;
    case BaCameraType_Other:
      ans = init_cams(opt, param_storage,
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
                                        opt.forced_triangulation_distance);
    check_gcp_dists(new_cam_models, opt.cnet, opt.forced_triangulation_distance);
    if (num_points != cnet.size()) // Must not happen
      vw_throw(ArgumentErr() << "The number of points changed after re-triangulation.\n");
  }

  // Fill in the point vector with the starting values
  for (int ipt = 0; ipt < num_points; ipt++)
    param_storage.set_point(ipt, cnet[ipt].position());

  // Flag any outliers read from an isis cnet
  for (auto const& ipt: isisCnetData.isisOutliers) {
    if (ipt < 0 || ipt >= num_points)
      vw_throw(ArgumentErr() << "Invalid point index.\n");
    param_storage.set_point_outlier(ipt, true);
  }

  // Flag outliers in the cnet
  for (int ipt = 0; ipt < num_points; ipt++) {
    if (cnet[ipt].ignore())
      param_storage.set_point_outlier(ipt, true);
  }

  // The camera positions and orientations before we float them
  // This includes modifications from any initial transforms that were specified.
  asp::BAParams orig_parameters(param_storage);

  // TODO(oalexan1): Likely orig_cams have the info as new_cam_models. But need
  // to test this.
  std::vector<vw::CamPtr> orig_cams;
  asp::calcOptimizedCameras(opt, orig_parameters, orig_cams); // orig cameras
  std::vector<std::vector<vw::Vector3>> orig_cam_positions;
  asp::calcCameraCenters(opt.stereo_session, orig_cams, orig_cam_positions);

  bool has_datum = (opt.datum.name() != asp::UNSPECIFIED_DATUM);
  if (has_datum && (opt.stereo_session == "pinhole") ||
      (opt.stereo_session == "nadirpinhole"))
    asp::saveCameraReport(opt, param_storage,  opt.datum, "initial");

  // TODO(oalexan1): Is it possible to avoid using CRNs?
  asp::CRNJ crn;
  crn.from_cnet(cnet);

  if (opt.num_passes <= 0)
    vw_throw(ArgumentErr() << "Error: Expecting at least one bundle adjust pass.\n");

  bool remove_outliers = (opt.num_passes > 1);
  double final_cost = 0.0;
  for (int pass = 0; pass < opt.num_passes; pass++) {

    if (opt.apply_initial_transform_only)
      continue;

    vw_out() << "--> Bundle adjust pass: " << pass << std::endl;

    bool first_pass = (pass == 0);
    bool convergence_reached = true; // will change
    do_ba_ceres_one_pass(opt, crn, first_pass, remove_outliers,
                         param_storage, orig_parameters,
                         orig_cams, orig_cam_positions,
                         convergence_reached, final_cost);
    int num_points_remaining = num_points - param_storage.get_num_outliers();
    if (num_points_remaining < opt.min_matches && num_gcp == 0) {
      // Do not throw if there exist gcp, as maybe that's all there is, and there
      // can be just a few of them. Also, do not throw if we are using an ISIS cnet,
      // unless we have less than 10 points, as that one can make too few points.
      std::ostringstream ostr;
      ostr << "Too few points remain after filtering. Number of remaining "
           << "points is " << num_points_remaining
           << ", but value of --min-matches is " << opt.min_matches << ".\n";
      if (opt.isis_cnet != "" && num_points_remaining > 10)
        vw_out(vw::WarningMessage) << ostr.str();
      else
        vw_throw(ArgumentErr() << "Error: " << ostr.str());
    }
  } // End loop through passes

  // Running random passes is not the default
  if (!opt.apply_initial_transform_only && opt.num_random_passes > 0)
    runRandomPasses(opt, param_storage, final_cost, crn, remove_outliers, orig_parameters);

  // Always save the updated cameras, even if we are not doing any optimization
  saveUpdatedCameras(opt, param_storage);

  // If we are only applying an initial transform, we are done
  if (opt.apply_initial_transform_only)
    return;

  // Write the GCP stats to a file
  if (num_gcp > 0)
    param_storage.print_gcp_stats(opt.out_prefix, cnet, opt.datum);

  // Find the cameras with the latest adjustments. Note that we do not modify
  // opt.camera_models, but make copies as needed.
  std::vector<vw::CamPtr> optimized_cams;
  asp::calcOptimizedCameras(opt, param_storage, optimized_cams);
  
  // Find the camera centers. For linescan, this will return all samples.
  std::vector<std::vector<vw::Vector3>> opt_cam_positions;
  asp::calcCameraCenters(opt.stereo_session, optimized_cams, opt_cam_positions);

  // Fetch the latest outliers from param_storage and put them in the 'outliers' set
  std::set<int> outliers;
  updateOutliers(cnet, param_storage, outliers);

  // Write clean matches and many types of stats. These are done together as
  // they rely on reloading interest point matches, which is expensive.
  bool save_clean_matches = true;
  asp::matchFilesProcessing(cnet,
                            asp::BaBaseOptions(opt), // note the slicing
                            optimized_cams, remove_outliers, outliers, opt.mapproj_dem,
                            opt.propagate_errors, opt.horizontal_stddev_vec,
                            save_clean_matches, opt.match_files);

  // Compute the change in camera centers. For that, we need the original cameras.
  std::string cam_offsets_file = opt.out_prefix + "-camera_offsets.txt";
  if (opt.datum.name() != asp::UNSPECIFIED_DATUM)
    asp::saveCameraOffsets(opt.datum, opt.image_files,
                           orig_cam_positions, opt_cam_positions,
                           cam_offsets_file);
  else
    vw::vw_out() << "Cannot compute camera offsets as the datum is unspecified.\n";

  std::string tri_offsets_file = opt.out_prefix + "-triangulation_offsets.txt";
  asp::saveTriOffsetsPerCamera(opt.image_files, orig_parameters, param_storage, crn,
                               tri_offsets_file);

  if (has_datum &&
      (opt.stereo_session == "pinhole") || (opt.stereo_session == "nadirpinhole"))
    saveCameraReport(opt, param_storage, opt.datum, "final");

  // Save the updated cnet to ISIS or nvm format. Note that param_storage has
  // the latest triangulated points and outlier info, while the cnet has the
  // initially triangulated points and the interest point matches.
  if (opt.isis_cnet != "" && opt.output_cnet_type == "isis-cnet")
    asp::saveUpdatedIsisCnet(opt.out_prefix, cnet, param_storage, isisCnetData);
  else if (opt.output_cnet_type == "isis-cnet")
    asp::saveIsisCnet(opt.out_prefix, opt.datum, cnet, param_storage);
  else if (opt.output_cnet_type == "nvm") {
    asp::saveNvm(opt, opt.no_poses_from_nvm, cnet, param_storage,
                  world_to_cam, optical_offsets);
  }

} // end do_ba_ceres

void handle_arguments(int argc, char *argv[], asp::BaOptions& opt) {

  const double nan = std::numeric_limits<double>::quiet_NaN();
  std::string intrinsics_to_float_str, intrinsics_to_share_str,
    intrinsics_limit_str;
  bool inline_adjustments = false;
  int max_iterations_tmp = -1;
  po::options_description general_options("");
  general_options.add_options()
    ("output-prefix,o",  po::value(&opt.out_prefix), "Prefix for output filenames.")
    ("cost-function",    po::value(&opt.cost_function)->default_value("Cauchy"),
     "Choose a cost function from: Cauchy, PseudoHuber, Huber, L1, L2, Trivial.")
    ("robust-threshold", po::value(&opt.robust_threshold)->default_value(0.5),
     "Set the threshold for robust cost functions. Increasing this makes the solver focus harder on the larger errors.")
    ("inline-adjustments",   po::bool_switch(&inline_adjustments)->default_value(false),
     "If this is set, and the input cameras are of the pinhole or panoramic type, apply "
     "the adjustments directly to the cameras, rather than saving them separately as "
     ".adjust files.")
    ("approximate-pinhole-intrinsics", po::bool_switch(&opt.approximate_pinhole_intrinsics)->default_value(false),
     "If it reduces computation time, approximate the lens distortion model.")
    ("solve-intrinsics",    po::bool_switch(&opt.solve_intrinsics)->default_value(false)->implicit_value(true),
     "Optimize intrinsic camera parameters. Only used for pinhole, optical bar, and CSM "
     "(frame and linescan) cameras. This implies --inline-adjustments.")
    ("intrinsics-to-float", po::value(&intrinsics_to_float_str)->default_value(""),
     "If solving for intrinsics and is desired to float only a few of them, specify here, "
     "in quotes, one or more of: focal_length, optical_center, other_intrinsics "
     "(distortion). Not specifying anything will float all of them. Also can specify 'all' "
     "or 'none'. See the documentation for when intrinsics are optimized per sensor.")
    ("intrinsics-to-share", po::value(&intrinsics_to_share_str)->default_value(""),
     "If solving for intrinsics and is desired to share only a few of them across all "
     "cameras, specify here, in quotes, one or more of: focal_length, optical_center, "
     "other_intrinsics (distortion). By default all of the intrinsics are shared, so to "
     "not share any of them pass in an empty string. Also can specify 'all' or 'none'. If "
     "sharing intrinsics per sensor, this option is ignored, as then the sharing is more "
     "fine-grained.")
    ("intrinsics-limits", po::value(&intrinsics_limit_str)->default_value(""),
     "Specify minimum and maximum ratios for the intrinsic parameters. Values must be in "
     "min max pairs and are applied in the order [focal length, optical center, other "
     "intrinsics] until all of the limits are used. Check the documentation to determine "
     "how many intrinsic parameters are used for your cameras.")
    ("camera-position-uncertainty",
     po::value(&opt.camera_position_uncertainty_str)->default_value(""),
     "A file having on each line the image name and the horizontal and vertical camera "
     "position uncertainty (1 sigma, in meters). This strongly constrains the movement of "
     "cameras to within the given values, potentially at the expense of accuracy. Add here value.")
    ("camera-position-uncertainty-power",
     po::value(&opt.camera_position_uncertainty_power)->default_value(2.0),
     "A higher value makes the cost function rise more steeply when "
     "--camera-position-uncertainty is close to being violated. This is an advanced "
      "option. The default should be good enough.")
    ("camera-positions",
     po::value(&opt.camera_position_file)->default_value(""),
     "CSV file containing estimated position of each camera in ECEF coordinates. For this "
     "to work well the camera must travel not along linear path, as this data will be used "
     "to find an alignment transform. Only used with the inline-adjustments setting to "
     "initialize global camera coordinates. If used, the csv-format setting must also be "
     "set. The 'file' field is searched for strings that are found in the input image "
     "files to match locations to cameras.")
    ("init-camera-using-gcp",  po::bool_switch(&opt.init_camera_using_gcp)->default_value(false)->implicit_value(true),
     "Given an image, a pinhole camera lacking correct position and orientation, and a GCP "
     "file, find the pinhole camera with given intrinsics most consistent with the GCP.")
    ("transform-cameras-with-shared-gcp",
     po::bool_switch(&opt.transform_cameras_with_shared_gcp)->default_value(false)->implicit_value(true),
     "Given at least 3 GCP, with each seen in at least 2 images, find the triangulated "
     "positions based on pixels values in the GCP, and apply a rotation + translation + "
     "scale transform to the entire camera system so that the triangulated points get "
     "mapped to the ground coordinates in the GCP.")
    ("transform-cameras-using-gcp",  po::bool_switch(&opt.transform_cameras_using_gcp)->default_value(false)->implicit_value(true),
     "Given a set of GCP, with at least two images having at least three GCP each (but "
     "with each GCP not shared among the images), transform the cameras to ground "
     "coordinates. This is not as robust as --transform-cameras-with-shared-gcp.")
    ("disable-pinhole-gcp-init",  po::bool_switch(&opt.disable_pinhole_gcp_init)->default_value(false)->implicit_value(true),
     "Do not try to initialize the positions of pinhole cameras based on input GCPs. This "
     "ignored as is now the default. See also: --init-camera-using-gcp.")
    ("input-adjustments-prefix",  po::value(&opt.input_prefix),
     "Prefix to read initial adjustments from, written by a previous invocation of "
     "this program.")
    ("initial-transform",  po::value(&opt.initial_transform_file)->default_value(""),
     "Before optimizing the cameras, apply to them the 4x4 rotation + translation transform "
     "from this file. The transform is in respect to the planet center, such as written by "
     "pc_align's source-to-reference or reference-to-source alignment transform. Set the "
     "number of iterations to 0 to stop at this step. If --input-adjustments-prefix is "
     "specified, the transform gets applied after the adjustments are read.")
    ("fixed-camera-indices",    po::value(&opt.fixed_cameras_indices_str)->default_value(""),
     "A list of indices, in quotes and starting from 0, with space as separator, corresponding to cameras to keep fixed during the optimization process.")
    ("fixed-image-list",    po::value(&opt.fixed_image_list)->default_value(""),
     "A file having a list of images (separated by spaces or newlines) whose cameras should be fixed during optimization.")
    ("fix-gcp-xyz", 
     po::bool_switch(&opt.fix_gcp_xyz)->default_value(false)->implicit_value(true),
     "If the GCP are highly accurate, use this option to not float them during the optimization.")
    ("csv-format",
     po::value(&opt.csv_format_str)->default_value(""), asp::csv_opt_caption().c_str())
    ("csv-srs",
     po::value(&opt.csv_srs)->default_value(""),
     "The PROJ or WKT string for interpreting the entries in input CSV files.")
    ("reference-terrain", po::value(&opt.reference_terrain)->default_value(""),
     "An externally provided trustworthy 3D terrain, either as a DEM or as a lidar file, "
     "very close (after alignment) to the stereo result from the given images and cameras "
     "that can be used as a reference, to optimize the intrinsics of the "
     "cameras.")
    ("max-num-reference-points", po::value(&opt.max_num_reference_points)->default_value(100000000),
     "Maximum number of (randomly picked) points from the reference terrain to use.")
    ("disparity-list", po::value(&opt.disparity_list)->default_value(""),
     "The unaligned disparity files to use when optimizing the intrinsics based on a "
     "reference terrain. Specify them as a list in quotes separated by spaces. First file "
     "is for the first two images, second is for the second and third images, etc. If an "
     "image pair has no disparity file, use 'none'.")
    ("max-disp-error", po::value(&opt.max_disp_error)->default_value(-1),
     "When using a reference terrain as an external control, ignore as outliers xyz points which projected in the left image and transported by disparity to the right image differ by the projection of xyz in the right image by more than this value in pixels.")
    ("reference-terrain-weight", po::value(&opt.reference_terrain_weight)->default_value(1.0),
     "How much weight to give to the cost function terms involving the reference terrain.")
    ("heights-from-dem",   po::value(&opt.heights_from_dem)->default_value(""),
     "Assuming the cameras have already been bundle-adjusted and aligned to a "
     "known DEM, constrain the triangulated points to be close to this DEM. See also "
     "--heights-from-dem-uncertainty.")
    ("heights-from-dem-uncertainty",
     po::value(&opt.heights_from_dem_uncertainty)->default_value(10.0),
     "The DEM uncertainty (1 sigma, in meters). A smaller value constrain more the "
     "triangulated points to the DEM specified via --heights-from-dem.")
    ("heights-from-dem-robust-threshold",
     po::value(&opt.heights_from_dem_robust_threshold)->default_value(0.1),
     "The robust threshold to use to keep the triangulated points close to the DEM if "
      "specified via --heights-from-dem. This is applied after the point differences "
      "are divided by --heights-from-dem-uncertainty. It will attenuate large height "
      "difference outliers. It is suggested to not modify this value, and adjust instead "
      "--heights-from-dem-uncertainty.")
    ("mapproj-dem", po::value(&opt.mapproj_dem)->default_value(""),
     "If specified, mapproject every pair of matched interest points onto this DEM "
     "and compute their distance, then percentiles of such distances for each image "
     "vs the rest and each image pair. This is done after bundle adjustment "
     "and outlier removal. Measured in meters. Not related to --mapprojected-data.")
    ("weight-image", po::value(&opt.weight_image)->default_value(""),
     "Given a georeferenced image with float values, for each initial triangulated "
     "point find its location in the image and closest pixel value. Multiply the "
     "reprojection errors in the cameras for this point by this weight value. The solver "
     "will focus more on optimizing points with a higher weight. Points that fall "
     "outside the image and weights that are non-positive, NaN, or equal to nodata "
     "will be ignored.")
    ("datum", po::value(&opt.datum_str)->default_value(""),
     "Use this datum. Needed only for ground control points, a camera position file, or "
     "for RPC sessions. Options: WGS_1984, D_MOON (1,737,400 meters), D_MARS (3,396,190 "
     "meters), MOLA (3,396,000 meters), NAD83, WGS72, and NAD27. Also accepted: Earth "
     "(=WGS_1984), Mars (=D_MARS), Moon (=D_MOON).")
    ("semi-major-axis",  po::value(&opt.semi_major)->default_value(0),
     "Explicitly set the datum semi-major axis in meters (see above).")
    ("semi-minor-axis",  po::value(&opt.semi_minor)->default_value(0),
     "Explicitly set the datum semi-minor axis in meters (see above).")
    ("session-type,t",   po::value(&opt.stereo_session)->default_value(""),
     "Select the stereo session type to use for processing. Usually the program can select "
     "this automatically by the file extension, except for xml cameras. See the doc for "
     "options.")
    ("min-matches",      po::value(&opt.min_matches)->default_value(5),
     "Set the minimum  number of matches between images that will be considered.")
    ("max-pairwise-matches", po::value(&opt.max_pairwise_matches)->default_value(10000),
     "Reduce the number of matches per pair of images to at most this "
     "number, by selecting a random subset, if needed. This happens "
     "when setting up the optimization, and before outlier filtering.")
    ("ip-detect-method", po::value(&opt.ip_detect_method)->default_value(0),
     "Interest point detection algorithm (0: Integral OBALoG (default), 1: OpenCV SIFT, 2: OpenCV ORB.")
    ("epipolar-threshold",      po::value(&opt.epipolar_threshold)->default_value(-1),
     "Maximum distance from the epipolar line to search for IP matches. Default: automatic calculation. A higher values will result in more matches.")
    ("ip-inlier-factor",        po::value(&opt.ip_inlier_factor)->default_value(0.2),
     "A higher factor will result in more interest points, but perhaps also more outliers.")
    ("ip-uniqueness-threshold", po::value(&opt.ip_uniqueness_thresh)->default_value(0.8),
     "A higher threshold will result in more interest points, but perhaps less unique ones.")
    ("ip-side-filter-percent",  po::value(&opt.ip_edge_buffer_percent)->default_value(-1),
     "Remove matched IPs this percentage from the image left/right sides.")
    ("normalize-ip-tiles",
     po::bool_switch(&opt.ip_normalize_tiles)->default_value(false)->implicit_value(true),
     "Individually normalize tiles used for IP detection.")
    ("num-obalog-scales",      po::value(&opt.num_scales)->default_value(-1),
     "How many scales to use if detecting interest points with OBALoG. If not specified, 8 will be used. More can help for images with high frequency artifacts.")
    ("nodata-value",           po::value(&opt.nodata_value)->default_value(nan),
     "Pixels with values less than or equal to this number are treated as no-data. This overrides the no-data values from input images.")
    ("num-iterations",       po::value(&opt.num_iterations)->default_value(1000),
     "Set the maximum number of iterations.")
    ("max-iterations",       po::value(&max_iterations_tmp)->default_value(1000),
     "Set the maximum number of iterations.") // alias for num-iterations
    ("parameter-tolerance",  po::value(&opt.parameter_tolerance)->default_value(1e-8),
     "Stop when the relative error in the variables being optimized is less than this. "
     "When --solve-intrinsics is used, the default is 1e-12.")
    ("overlap-limit",        po::value(&opt.overlap_limit)->default_value(0),
     "Limit the number of subsequent images to search for matches to the current image to this value. By default match all images.")
    ("overlap-list",         po::value(&opt.overlap_list_file)->default_value(""),
     "A file containing a list of image pairs, one pair per line, separated by a space, which are expected to overlap. Matches are then computed only among the images in each pair.")
    ("auto-overlap-params",  po::value(&opt.auto_overlap_params)->default_value(""),
     "Determine which camera images overlap by finding the bounding boxes of their "
     "ground footprints given the specified DEM, expanding them by a given percentage, "
     "and see if those intersect. A higher percentage should be used when there is more "
     "uncertainty about the input camera poses. Example: 'dem.tif 15'. "
     "Using this with --mapprojected-data will restrict the "
     "matching only on the overlap regions (expanded by this percentage).")
    ("auto-overlap-buffer",  po::value(&opt.auto_overlap_buffer)->default_value(-1.0),
     "Try to automatically determine which images overlap. Used only if "
     "this option is explicitly set. Only supports Worldview style XML "
     "camera files. The lon-lat footprints of the cameras are expanded "
     "outwards on all sides by this value (in degrees), before checking "
     "if they intersect.")
    ("image-list", po::value(&opt.image_list)->default_value(""),
     "A file containing the list of images, when they are too many to specify on the command line. Use space or newline as separator. See also --camera-list and --mapprojected-data-list.")
    ("camera-list", po::value(&opt.camera_list)->default_value(""),
     "A file containing the list of cameras, when they are too many to specify on "
     "the command line. If the images have embedded camera information, such as for ISIS, "
     "this file may be omitted, or specify the image names instead of camera names.")
    ("mapprojected-data-list", po::value(&opt.mapprojected_data_list)->default_value(""),
     "A file containing the list of mapprojected images and the DEM (see --mapprojected-data), when they are too many to specify on the command line.")
    ("position-filter-dist", po::value(&opt.position_filter_dist)->default_value(-1),
     "Set a distance in meters and don't perform IP matching on images with an estimated camera center farther apart than this distance.  Requires --camera-positions.")
    ("match-first-to-last", po::bool_switch(&opt.match_first_to_last)->default_value(false)->implicit_value(true),
     "Match first several images to last several images by extending the logic of --overlap-limit past the last image to the earliest ones.")
    ("camera-position-weight", po::value(&opt.camera_position_weight)->default_value(0.0),
     "A soft constraint to keep the camera positions close to the original values. "
     "It is meant to prevent a wholesale shift of the cameras. It can impede "
     "the reduction in reprojection errors. It adjusts to the ground sample distance "
     "and the number of interest points in the images. The computed "
     "discrepancy is attenuated with --camera-position-robust-threshold. "
     "See --camera-position-uncertainty for a hard constraint.")
    ("camera-position-robust-threshold",
     po::value(&opt.camera_position_robust_threshold)->default_value(0.1),
     "The robust threshold to attenuate large discrepancies between initial and optimized "
     "camera positions with the option --camera-position-weight. This is less than "
     "--robust-threshold, as the primary goal is to reduce pixel reprojection errors, even "
     "if that results in big differences in the camera positions. It is suggested to not "
     "modify this value, and adjust instead --camera-position-weight.")
    ("rotation-weight",
     po::value(&opt.rotation_weight)->default_value(0.0),
     "A higher weight will penalize more rotation deviations from the original "
     "configuration.")
    ("camera-weight", po::value(&opt.camera_weight)->default_value(0.0),
     "The weight to give to the constraint that the camera positions/orientations "
     "stay close to the original values. A higher weight means that the values will "
     "change less. This option is deprecated. Use instead --camera-position-weight "
     "and --tri-weight.")
    ("tri-weight", po::value(&opt.tri_weight)->default_value(0.1),
     "The weight to give to the constraint that optimized triangulated points stay "
      "close to original triangulated points. A positive value will help ensure the "
      "cameras do not move too far, but a large value may prevent convergence. It is "
      "suggested to use here 0.1 to 0.5. This will be divided by ground sample distance "
      "(GSD) to convert this constraint to pixel units, since the reprojection errors "
      "are in pixels. See also --tri-robust-threshold. Does not apply to GCP or points "
      "constrained by a DEM.")
    ("tri-robust-threshold", po::value(&opt.tri_robust_threshold)->default_value(0.1),
     "The robust threshold to attenuate large differences between initial and optimized "
     "triangulation points, after multiplying them by --tri-weight and dividing by GSD. "
     "This is less than --robust-threshold, as the primary goal is to reduce pixel "
     "reprojection errors, even if that results in big differences in the triangulated "
     "points. It is suggested to not modify this value, and adjust instead --tri-weight.")
    ("isis-cnet", po::value(&opt.isis_cnet)->default_value(""),
     "Read a control network having interest point matches from this binary file "
     "in the ISIS jigsaw format. This can be used with any images and cameras "
     "supported by ASP. For Pinhole cameras, the (optimized) camera poses will be "
     "read/written to NVM as well.  See also --output-cnet-type.")
    ("nvm", po::value(&opt.nvm)->default_value(""),
     "Read a control network having interest point matches from this file in the NVM "
     "format. This can be used with any images and cameras supported by ASP. For Pinhole "
     "or CSM frame cameras, the (optimized) camera poses will be read from / written to "
     "NVM as well. See also --output-cnet-type, --no-poses-from-nvm.")
    ("output-cnet-type", po::value(&opt.output_cnet_type)->default_value(""),
      "The format in which to save the control network of interest point matches. "
      "Options: 'match-files' (match files in ASP's format), 'isis-cnet' (ISIS "
      "jigsaw format), 'nvm' (plain text VisualSfM NVM format). If not set, the same "
      "format as for the input is used.")
    ("no-poses-from-nvm",
      po::bool_switch(&opt.no_poses_from_nvm)->default_value(false)->implicit_value(true),
     "Do not read the camera poses from the NVM file or write them to such a file. "
     "Applicable only with the option --nvm and Pinhole camera models.")
    ("overlap-exponent",  po::value(&opt.overlap_exponent)->default_value(0.0),
     "If a feature is seen in n >= 2 images, give it a weight proportional with "
     "(n-1)^exponent.")
    ("ip-per-tile", po::value(&opt.ip_per_tile)->default_value(0),
     "How many interest points to detect in each 1024^2 image tile (default: automatic "
     "determination). This is before matching. Not all interest points will have a match. "
     "See also --matches-per-tile.")
    ("ip-per-image", po::value(&opt.ip_per_image)->default_value(0),
     "How many interest points to detect in each image (default: automatic determination). It is overridden by --ip-per-tile if provided.")
    ("num-passes",
     po::value(&opt.num_passes)->default_value(2),
     "How many passes of bundle adjustment to do, with given number of iterations in each "
     "pass. For more than one pass, outliers will be removed between passes using "
     "--remove-outliers-params, and re-optimization will take place. Residual files and a "
     "copy of the match files with the outliers removed (*-clean.match) will be written to "
     "disk.")
    ("num-random-passes",           po::value(&opt.num_random_passes)->default_value(0),
     "After performing the normal bundle adjustment passes, do this many more passes using the same matches but adding random offsets to the initial parameter values with the goal of avoiding local minima that the optimizer may be getting stuck in.")
    ("remove-outliers-params",
     po::value(&opt.remove_outliers_params_str)->default_value("75.0 3.0 5.0 8.0", "'pct factor err1 err2'"),
     "Outlier removal based on percentage, when more than one bundle adjustment pass is "
     "used. Triangulated points (that are not GCP) with reprojection error in pixels "
     "larger than min(max('pct'-th percentile * 'factor', err1), err2) will be removed as "
     "outliers. Hence, never remove errors smaller than err1 but always remove those "
     "bigger than err2. Specify as a list in quotes. Also remove outliers based on "
     "distribution of interest point matches and triangulated points.")
    ("elevation-limit", po::value(&opt.elevation_limit)->default_value(Vector2(0,0), "auto"),
     "Remove as outliers interest points (that are not GCP) for which the elevation of the triangulated position (after cameras are optimized) is outside of this range. Specify as two values: min max.")
    // Note that we count later on the default for lon_lat_limit being BBox2(0,0,0,0).
    ("lon-lat-limit", po::value(&opt.lon_lat_limit)->default_value(BBox2(0,0,0,0), "auto"),
     "Remove as outliers interest points (that are not GCP) for which the longitude and latitude of the triangulated position (after cameras are optimized) are outside of this range. Specify as: min_lon min_lat max_lon max_lat.")
    ("match-files-prefix",  po::value(&opt.match_files_prefix)->default_value(""),
     "Use the match files from this prefix instead of the current output prefix. See the "
     "naming convention in the documentation. This implies --skip-matching. The order of "
     "images in each interest point match file need not be the same as for input images. "
     "See also --clean-match-files-prefix. Only one of these two options can be specified.")
    ("clean-match-files-prefix",  po::value(&opt.clean_match_files_prefix)->default_value(""),
     "Use as input the *-clean.match file from this prefix (this had the "
     "outliers filtered out by bundle_adjust). See also match-files-prefix.")
    ("update-isis-cubes-with-csm-state",
     po::bool_switch(&opt.update_isis_cubes_with_csm_state)->default_value(false)->implicit_value(true),
     "Save the model state of optimized CSM cameras as part of the .cub files. Any prior "
     "version and any SPICE data will be deleted. Mapprojected images obtained with prior "
     "version of the cameras must no longer be used in stereo.")
    ("save-adjusted-rpc",
     po::bool_switch(&opt.save_adjusted_rpc)->default_value(false)->implicit_value(true),
     "In addition to external adjustments to the input cameras, save RPC cameras with "
     "the adjustments applied to them, in XML format. This recomputes the RPC models.")
    ("enable-rough-homography",
     po::bool_switch(&opt.enable_rough_homography)->default_value(false)->implicit_value(true),
     "Enable the step of performing datum-based rough homography for interest point matching. This is best used with reasonably reliable input cameras and a wide footprint on the ground.")
    ("skip-rough-homography",
     po::bool_switch(&opt.skip_rough_homography)->default_value(false)->implicit_value(true),
     "Skip the step of performing datum-based rough homography. This obsolete option is ignored as is the default.")
    ("enable-tri-ip-filter",
     po::bool_switch(&opt.enable_tri_filtering)->default_value(false)->implicit_value(true),
     "Enable triangulation-based interest points filtering. This is best used with reasonably reliable input cameras.")
    ("disable-tri-ip-filter",
     po::bool_switch(&opt.disable_tri_filtering)->default_value(false)->implicit_value(true),
     "Disable triangulation-based interest points filtering. This obsolete option is ignored as is the default.")
    ("no-datum", po::bool_switch(&opt.no_datum)->default_value(false)->implicit_value(true),
     "Do not assume a reliable datum exists, such as for irregularly shaped bodies.")
    ("individually-normalize",
     po::bool_switch(&opt.individually_normalize)->default_value(false)->implicit_value(true),
     "Individually normalize the input images instead of using common values.")
    ("ip-triangulation-max-error",  po::value(&opt.ip_triangulation_max_error)->default_value(-1),
     "When matching IP, filter out any pairs with a triangulation error higher than this.")
    ("ip-num-ransac-iterations", po::value(&opt.ip_num_ransac_iterations)->default_value(1000),
     "How many RANSAC iterations to do in interest point matching.")
    ("min-triangulation-angle", po::value(&opt.min_triangulation_angle)->default_value(0.1),
     "Filter as outlier any triangulation point for which all rays converging to "
      "it have an angle less than this (measured in degrees). This happens on "
      "loading the match files and after each optimization pass. This should be used "
      "cautiously with very uncertain input cameras.")
    ("max-triangulation-angle", po::value(&opt.max_triangulation_angle)->default_value(-1.0),
     "Filter as outlier any triangulation points for which the maximum angle of rays "
     "converging to it are more than this (measured in degrees). Set to a positive value.")
    ("forced-triangulation-distance", po::value(&opt.forced_triangulation_distance)->default_value(-1),
     "When triangulation fails, for example, when input cameras are inaccurate, "
     "artificially create a triangulation point this far ahead of the camera, "
     "in units of meter. Can also set a small --min-triangulation-angle in this case.")
    ("use-lon-lat-height-gcp-error",
     po::bool_switch(&opt.use_llh_error)->default_value(false)->implicit_value(true),
     "When having GCP (or a DEM constraint), constrain the triangulated points in the "
     "longitude, latitude, and height space, instead of ECEF. The standard deviations "
     "in the GCP file (or DEM uncertainty) are applied accordingly.")
    ("aster-use-csm", po::bool_switch(&opt.aster_use_csm)->default_value(false)->implicit_value(true),
     "Use the CSM model with ASTER cameras (-t aster).")
    ("mapprojected-data",  po::value(&opt.mapprojected_data)->default_value(""),
     "Given map-projected versions of the input images "
     "and the DEM they were mapprojected onto, create interest point matches between "
     "the mapprojected images, unproject and save those matches, then continue "
     "with bundle adjustment. Existing match files will be reused. Specify the "
     "mapprojected images and the DEM as a string in quotes, separated by spaces. "
     "The DEM must be the last file. It is suggested to use this with "
     "--auto-overlap-params.")
    ("matches-per-tile",  po::value(&opt.matches_per_tile)->default_value(0),
     "How many interest point matches to compute in each image tile (of size "
      "normally 1024^2 pixels). Use a value of --ip-per-tile a few times larger "
      "than this. See also --matches-per-tile-params.")
    ("save-cnet-as-csv", po::bool_switch(&opt.save_cnet_as_csv)->default_value(false)->implicit_value(true),
     "Save the control network containing all interest points in the format used by ground control points, so it can be inspected. The triangulated points are before optimization.")
    ("num-parallel-jobs", po::value(&opt.num_parallel_jobs)->default_value(1),
     "The number of bundle_adjustment processes being run in parallel over all nodes.")
    ("job-id", po::value(&opt.job_id)->default_value(0),
     "The id of this parallel bundle adjustment job.")
    ("force-reuse-match-files", po::bool_switch(&opt.force_reuse_match_files)->default_value(false)->implicit_value(true),
     "Force reusing the match files even if older than the images or cameras. "
     "Then the order of images in each interest point match file need not be the same "
     "as for input images. Additional match files will be created if needed.")
    ("skip-matching",    po::bool_switch(&opt.skip_matching)->default_value(false)->implicit_value(true),
     "Only use the match files that can be loaded from disk. This implies --force-reuse-match-files.")
    ("save-intermediate-cameras", po::bool_switch(&opt.save_intermediate_cameras)->default_value(false)->implicit_value(true),
     "Save the values for the cameras at each iteration.")
    ("apply-initial-transform-only", po::bool_switch(&opt.apply_initial_transform_only)->default_value(false)->implicit_value(true),
     "Apply to the cameras the transform given by --initial-transform. "
     "No iterations, GCP loading, image matching, or report generation "
     "take place. Using --num-iterations 0 and without this option "
     "will create those.")
    ("proj-win", po::value(&opt.proj_win)->default_value(BBox2(0,0,0,0), "auto"),
     "Flag as outliers input triangulated points not in this proj win (box in projected "
     "units as provided by --proj_str). This should be generous if the input cameras have "
     "significant errors.")
    ("proj-str",   po::value(&opt.proj_str)->default_value(""),
     "To be used in conjunction with --proj-win.")
    ("matches-per-tile-params",  po::value(&opt.matches_per_tile_params)->default_value(Vector2(1024, 1280), "1024 1280"),
     "To be used with --matches-per-tile. The first value is the image tile size for both "
     "images. A larger second value allows each right tile to further expand to this size, "
     "resulting in the tiles overlapping. This may be needed if the homography alignment "
     "between these images is not great, as this transform is used to pair up left and "
     "right image tiles.")
    ("propagate-errors",
     po::bool_switch(&opt.propagate_errors)->default_value(false)->implicit_value(true),
     "Propagate the errors from the input cameras to the triangulated points for all pairs "
     "of match points, and produce a report having the median, mean, standard deviation, "
     "and number of samples for each camera pair.")
    ("horizontal-stddev", po::value(&opt.horizontal_stddev)->default_value(0),
     "If positive, propagate this stddev of horizontal ground plane camera uncertainty "
     "through triangulation for all cameras. To be used with --propagate-errors.")
    ("min-distortion",
     po::value(&opt.min_distortion)->default_value(1e-7),
     "When lens distortion is optimized, all initial distortion parameters that are "
     "smaller in magnitude than this value are set to this value. This is to ensure the "
     "parameters are big enough to be optimized. Can be negative. Applies to Pinhole "
     "cameras (all distortion models) and CSM (radial-tangential distortion only). Does "
     "not apply to optical bar models.")
    ("flann-method",  po::value(&opt.flann_method)->default_value("auto"),
      "Choose the FLANN method for matching interest points. Options: 'kmeans': "
      "slower but deterministic, 'kdtree': faster (up to 6x) but not deterministic "
      "(starting with FLANN 1.9.2). The default ('auto') is to use 'kmeans' for "
      "25,000 features or less and 'kdtree' otherwise. This does not apply to ORB "
      "feature matching.")
    ("csv-proj4", po::value(&opt.csv_proj4_str)->default_value(""),
     "An alias for --csv-srs, for backward compatibility.")
    ("save-vwip", po::bool_switch(&opt.save_vwip)->default_value(false)->implicit_value(true),
     "Save .vwip files (interest point matches per image, before matching). This option "
     "is currently ignored as .vwip are always saved.")
    ("ip-nodata-radius", po::value(&opt.ip_nodata_radius)->default_value(4),
     "Remove IP near nodata with this radius, in pixels.")
    ("accept-provided-mapproj-dem", 
     po::bool_switch(&asp::stereo_settings().accept_provided_mapproj_dem)->default_value(false)->implicit_value(true),
     "Accept the DEM provided on the command line as the one mapprojection was done with, "
     "even if it disagrees with the DEM recorded in the geoheaders of input images.")
    ("stop-after-statistics",    
      po::bool_switch(&opt.stop_after_stats)->default_value(false)->implicit_value(true),
     "Quit after computing image statistics.")
    ("stop-after-matching",
      po::bool_switch(&opt.stop_after_matching)->default_value(false)->implicit_value(true),
     "Quit after writing all match files.")
    ("calc-normalization-bounds",
     po::bool_switch(&opt.calc_normalization_bounds)->default_value(false)->implicit_value(true),
     "This is called in parallel_bundle_adjust just once to calculate all image bounds "
     "for normalization, after statistics were computed by separate processes.") 
    ("calc-ip",
     po::bool_switch(&opt.calc_ip)->default_value(false)->implicit_value(true),
     "Compute interest points for each image. This is before interest point matching."
     "This is called by parallel_bundle_adjust with multiple processes.")
    ("ip-debug-images",
     po::bool_switch(&opt.ip_debug_images)->default_value(false)->implicit_value(true),
     "Write debug images to disk when detecting and matching interest points.")
    ("query-num-image-pairs", 
     po::bool_switch(&opt.query_num_image_pairs)->default_value(false)->implicit_value(true), 
     "Print how many image pairs need to find matches for, and exit.")
    ;

  general_options.add(vw::GdalWriteOptionsDescription(opt));

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

  // Do this check first, as the output prefix is needed to log to file. This
  // will be triggered when called with no arguments, so print the general
  // options, which functions as the help message.
  if (opt.out_prefix.empty())
    vw_throw(ArgumentErr() << "Missing the output prefix.\n" << usage 
             << general_options);

  // Create the output directory
  vw::create_out_dir(opt.out_prefix);

  // Turn on logging to file
  asp::log_to_file(argc, argv, "", opt.out_prefix);

  // This must be done early
  boost::to_lower(opt.stereo_session);

  // Separate out GCP files
  bool rm_from_input_list = true;
  opt.gcp_files = vw::get_files_with_ext(opt.image_files, ".gcp", rm_from_input_list);

  // Handle the situation when the images and cameras are in lists
  if (!opt.image_list.empty()) {
    // Read the images and cameras and put them in 'images_or_cams' to be parsed later
    if (!opt.image_files.empty())
      vw_throw(ArgumentErr() << "The option --image-list was specified, but also "
               << "images or cameras on the command line.\n");

    // Read image and camera lists. Consider he case of sharing intrinsics per sensor.
    read_image_cam_lists(opt.image_list, opt.camera_list,
                         opt.image_files, opt.camera_files,
                         opt.intrinsics_options); // outputs
  } else {
    // The images and cameras are passed on the command line
    std::vector<std::string> images_or_cams = opt.image_files;
    bool ensure_equal_sizes = true;
    asp::separate_images_from_cameras(images_or_cams,
                                      opt.image_files, opt.camera_files, // outputs
                                      ensure_equal_sizes);
  }

  // Sanity checks
  if (!opt.mapprojected_data_list.empty() && opt.image_list.empty())
    vw_throw(ArgumentErr() << "Found --mapprojected-data-list, "
             << "but not --image-list.\n");
  if (!opt.mapprojected_data.empty() && !opt.mapprojected_data_list.empty())
    vw_throw(ArgumentErr() << "Cannot specify both --mapprojected-data and "
             << "--mapprojected-data-list.\n");

  // Sanity checks
  asp::check_for_duplicates(opt.image_files, opt.camera_files, opt.out_prefix);
  if (opt.image_files.size() != (int)opt.camera_files.size()) {
    vw_out() << "Detected " << opt.image_files.size() << " images and "
             << opt.camera_files.size() << " cameras.\n";
    vw_throw(ArgumentErr() << "Must have as many cameras as images.\n");
  }
  if (opt.image_files.empty())
    vw_throw(ArgumentErr() << "Missing input image files.\n");

  // Guess the session if not provided. Do this as soon as we have
  // the cameras figured out.
  asp::SessionPtr session(NULL);
  if (opt.stereo_session.empty()) {
    session.reset(asp::StereoSessionFactory::create
                        (opt.stereo_session, // may change
                        opt, opt.image_files[0], opt.image_files[0],
                        opt.camera_files[0], opt.camera_files[0],
                        opt.out_prefix));
  }

  // Reusing match files implies that we skip matching
  if (opt.clean_match_files_prefix != "" || opt.match_files_prefix != "" ||
      opt.isis_cnet != "" || opt.nvm != "")
    opt.skip_matching = true;

  // If opt.output_cnet_type is not set, set it to the same format as the input
  if (opt.output_cnet_type == "") {
    if (opt.isis_cnet != "")
      opt.output_cnet_type = "isis-cnet";
    else if (opt.nvm != "")
      opt.output_cnet_type = "nvm";
    else
      opt.output_cnet_type = "match-files";
  }
  // Sanity check in case the user set this option manually.
  if (opt.output_cnet_type != "match-files" && opt.output_cnet_type != "isis-cnet" &&
      opt.output_cnet_type != "nvm")
    vw_throw(ArgumentErr() << "Unknown value for --output-cnet-type: "
                           << opt.output_cnet_type << ".\n");

  //  When skipping matching, we are already forced to reuse match
  //  files based on the logic in the code, but here enforce it
  //  explicitly anyway.
  if (opt.skip_matching)
    opt.force_reuse_match_files = true;

    // Must specify either csv_srs or csv_proj4_str, but not both. The latter is
  // for backward compatibility.
  if (!opt.csv_srs.empty() && !opt.csv_proj4_str.empty())
    vw_throw(ArgumentErr() << "Cannot specify both --csv-srs and --csv-proj4.\n");
  if (!opt.csv_proj4_str.empty() && opt.csv_srs.empty())
    opt.csv_srs = opt.csv_proj4_str;

  if (opt.auto_overlap_params != "" && opt.skip_matching) {
    vw_out() << "Ignoring --auto-overlap-params since no matching takes place.\n";
    opt.auto_overlap_params = "";
  }

  // Sanity checks for solving for intrinsics
  if (opt.intrinsics_options.share_intrinsics_per_sensor && !opt.solve_intrinsics)
    vw_throw(ArgumentErr()
      << "Must set --solve-intrinsics to solve for intrinsics per sensor.\n");
  if (opt.solve_intrinsics && !inline_adjustments) {
    vw_out() << "Solving for intrinsics, so assuming --inline-adjustments.\n";
    inline_adjustments = true;
  }

  // Work out the camera model type. This must happen before early.
  // Cameras are not loaded yet.
  // TODO(oalexan1): Maybe we need to load cameras by now?
  opt.camera_type = BaCameraType_Other;
  if (inline_adjustments) {
    if ((opt.stereo_session == "pinhole") ||
        (opt.stereo_session == "nadirpinhole"))
      opt.camera_type = BaCameraType_Pinhole;
    else if (opt.stereo_session == "opticalbar")
        opt.camera_type = BaCameraType_OpticalBar;
    else if (opt.stereo_session == "csm")
      opt.camera_type = BaCameraType_CSM;
    else
      vw_throw(ArgumentErr() << "Cannot use inline adjustments with session: "
                << opt.stereo_session << ".\n");
  }

  // Solving for intrinsics requires working harder to push the error down
  if (opt.solve_intrinsics && vm["parameter-tolerance"].defaulted())
    opt.parameter_tolerance = 1e-12;

  // Sharing intrinsics per sensor is not supported with reference terrain.
  // It would be too much work to fix the BaDispXyzError() cost function in that
  // case. Would need to handle all intrinsics being shared, only shared per sensor,
  // and none being shared. Same with random passes, there also new logic is needed.
  if (opt.intrinsics_options.share_intrinsics_per_sensor) {
    if (opt.reference_terrain != "")
      vw_throw(ArgumentErr() << "Cannot share intrinsics per sensor with "
        << "--reference-terrain.\n");
    if (opt.num_random_passes > 0)
      vw_throw(ArgumentErr() << "Cannot share intrinsics per sensor with "
        << "--num-random-passes.\n");
  }

  bool external_matches = (!opt.clean_match_files_prefix.empty() ||
                           !opt.match_files_prefix.empty());
  if (external_matches && (opt.isis_cnet != "" || opt.nvm != ""))
    vw_throw(ArgumentErr() << "Cannot use more than one of: ISIS cnet, nvm file, "
             << "match files.\n");

  if (opt.transform_cameras_using_gcp &&
      (!inline_adjustments) &&
      (opt.camera_type != BaCameraType_Pinhole)) {
    vw_throw(ArgumentErr() << "Transforming cameras using GCP works only for pinhole "
              << "cameras and with the --inline-adjustments flag.\n");
  }

  if (opt.overlap_list_file != "" && opt.overlap_limit > 0)
    vw_throw(ArgumentErr()
              << "Cannot specify both the overlap limit and the overlap list.\n");

  if (opt.overlap_list_file != "" && opt.match_first_to_last > 0)
    vw_throw(ArgumentErr()
      << "Cannot specify both the overlap limit and --match-first-to-last.\n");

  if (opt.overlap_limit < 0)
    vw_throw(ArgumentErr() << "Must allow search for matches between "
      << "at least each image and its subsequent one.\n");

  // By default, try to match all of the images!
  if (opt.overlap_limit == 0)
    opt.overlap_limit = opt.image_files.size();

  if (int(opt.overlap_list_file != "") + int(!vm["auto-overlap-buffer"].defaulted()) +
      int(opt.auto_overlap_params != "") > 1)
    vw_throw(ArgumentErr() << "Cannot specify more than one of --overlap-list, "
              << "--auto-overlap-params, and --auto-overlap-buffer.\n");

  opt.have_overlap_list = false;
  if (opt.overlap_list_file != "") {
   opt.have_overlap_list = true;
    if (!fs::exists(opt.overlap_list_file))
      vw_throw(ArgumentErr() << "The overlap list does not exist.\n");
    opt.overlap_list.clear();
    std::string image1, image2;
    std::ifstream ifs(opt.overlap_list_file.c_str());
    while (ifs >> image1 >> image2) {
      opt.overlap_list.insert(std::make_pair(image1, image2));
      opt.overlap_list.insert(std::make_pair(image2, image1));
    }
    ifs.close();
  } else if (!vm["auto-overlap-buffer"].defaulted()) {
    opt.have_overlap_list = true;
    auto_build_overlap_list(opt, opt.auto_overlap_buffer);
  }
  // The third alternative, --auto-overlap-params will be handled when we have cameras

  if (opt.camera_weight < 0.0)
    vw_throw(ArgumentErr() << "The camera weight must be non-negative.\n");

  if (opt.rotation_weight < 0.0)
    vw_throw(ArgumentErr() << "The rotation weight must be non-negative.\n");

  if (opt.camera_position_weight < 0.0)
    vw_throw(ArgumentErr() << "The camera position weight must be non-negative.\n");

  if (opt.tri_weight < 0.0)
    vw_throw(ArgumentErr() << "The triangulation weight must be non-negative.\n");

  // NOTE(oalexan1): The reason min_triangulation_angle cannot be 0 is deep inside
  // StereoModel.cc. Better keep it this way than make too many changes there.
  if (opt.min_triangulation_angle <= 0.0)
    vw_throw(ArgumentErr() << "The minimum triangulation angle must be positive.\n");

  // TODO: Make sure the normal model loading catches this error.
  //if (opt.create_pinhole && !vw::has_pinhole_extension(opt.camera_files[0]))
  //  vw_throw(ArgumentErr() << "Cannot use special pinhole handling with non-pinhole input!\n");

  if ((opt.camera_type == BaCameraType_Other) && opt.solve_intrinsics)
    vw_throw(ArgumentErr() << "Solving for intrinsic parameters is only supported with "
              << "pinhole, optical bar, and CSM cameras.\n");

  if ((opt.camera_type!=BaCameraType_Pinhole) && opt.approximate_pinhole_intrinsics)
    vw_throw(ArgumentErr() << "Cannot approximate intrinsics unless using pinhole cameras.\n");

  if (opt.approximate_pinhole_intrinsics && opt.solve_intrinsics)
    vw_throw(ArgumentErr() << "Cannot approximate intrinsics while solving for them.\n");

  if (opt.camera_type != BaCameraType_Other   &&
      opt.camera_type != BaCameraType_Pinhole &&
      opt.camera_type != BaCameraType_CSM     &&
      opt.input_prefix != "")
    vw_throw(ArgumentErr() << "Can only use initial adjustments with camera type "
              << "'pinhole', 'csm', or 'other'. Here likely having optical bar cameras.\n");

  vw::string_replace(opt.remove_outliers_params_str, ",", " "); // replace any commas
  opt.remove_outliers_params = vw::str_to_vec<vw::Vector<double, 4>>(opt.remove_outliers_params_str);

  // Ensure good order
  if (opt.lon_lat_limit != BBox2(0,0,0,0)) {
    if (opt.lon_lat_limit.min().y() > opt.lon_lat_limit.max().y())
      std::swap(opt.lon_lat_limit.min().y(), opt.lon_lat_limit.max().y());
    if (opt.lon_lat_limit.min().x() > opt.lon_lat_limit.max().x())
      std::swap(opt.lon_lat_limit.min().x(), opt.lon_lat_limit.max().x());
  }

  if (!opt.camera_position_file.empty() && opt.csv_format_str == "")
    vw_throw(ArgumentErr() << "When using a camera position file, the csv-format "
              << "option must be set.\n");

  if (opt.max_pairwise_matches <= 0)
    vw_throw(ArgumentErr() << "Must have a positive number of max pairwise matches.\n");

  // Copy the IP settings to the global stereo_settings() object
  opt.copy_to_asp_settings();

  // Try to infer the datum, if possible, from the images. For
  // example, Cartosat-1 has that info in the Tif file.
  bool have_datum = false;
  // For pinhole session the guessed datum may be unreliable, so warn only
  bool warn_only = (opt.stereo_session.find("pinhole") != std::string::npos);

  vw::cartography::GeoReference georef;
  for (size_t it = 0; it < opt.image_files.size(); it++) {
    bool is_good = vw::cartography::read_georeference(georef, opt.image_files[it]);

    // Must check the consistency of the datums
    if (is_good && have_datum)
      vw::checkDatumConsistency(opt.datum, georef.datum(), warn_only);

    if (is_good && !have_datum) {
      opt.datum = georef.datum();
      opt.datum_str = opt.datum.name();
      have_datum = true;
    }
  }

  // Try to infer the datum from the reference terrain
  if (opt.reference_terrain != "") {
    std::string file_type = asp::get_cloud_type(opt.reference_terrain);
    if (file_type == "DEM") {
      vw::cartography::GeoReference georef;
      bool is_good = vw::cartography::read_georeference(georef, opt.reference_terrain);
      if (!is_good)
        vw_throw(ArgumentErr()
                 << "The reference terrain DEM does not have a georeference.\n");
      // Ensure the datum read from the DEM agrees with the one from the cameras/user
      if (is_good && have_datum)
        vw::checkDatumConsistency(opt.datum, georef.datum(), warn_only);
      if (opt.datum_str == "") {
        opt.datum = georef.datum();
        opt.datum_str = opt.datum.name();
        have_datum = true;
      }
    }
  }

  if (opt.robust_threshold <= 0.0)
    vw_throw(ArgumentErr() << "The value of --robust-threshold must be positive.\n");

  if (opt.tri_robust_threshold <= 0.0)
    vw_throw(ArgumentErr() << "The value of --tri-robust-threshold must be positive.\n");

  if (opt.camera_position_robust_threshold <= 0.0)
    vw_throw(ArgumentErr() << "The value of --camera-position-robust-threshold "
              << "must be positive.\n");

  // This is a bug fix. The user by mistake passed in an empty height-from-dem string.
  if (!vm["heights-from-dem"].defaulted() && opt.heights_from_dem.empty())
    vw_throw(ArgumentErr()
             << "The value of --heights-from-dem is empty. "
             << "Then it must not be set at all.\n");
  if (!vm["heights-from-dem-uncertainty"].defaulted() &&
      vm["heights-from-dem"].defaulted())
    vw_throw(ArgumentErr()
             << "The value of --heights-from-dem-uncertainty is set, "
             << "but --heights-from-dem is not set.\n");
  if (opt.heights_from_dem_uncertainty <= 0.0)
    vw_throw(ArgumentErr() << "The value of --heights-from-dem-uncertainty must be "
              << "positive.\n");
  if (opt.heights_from_dem_robust_threshold <= 0.0)
    vw_throw(ArgumentErr() << "The value of --heights-from-robust-threshold must be "
              << "positive.\n");

  bool have_dem = (!opt.heights_from_dem.empty());

  // Try to infer the datum from the heights-from-dem
  std::string dem_file;
  if (opt.heights_from_dem != "")
    dem_file = opt.heights_from_dem;
  if (dem_file != "") {
    std::string file_type = asp::get_cloud_type(dem_file);
    if (file_type == "DEM") {
      vw::cartography::GeoReference georef;
      bool is_good = vw::cartography::read_georeference(georef, dem_file);
      if (!is_good)
        vw_throw(ArgumentErr() << "The DEM " << dem_file
                  << " does not have a georeference.\n");

      // Must check the consistency of the datums
      if (have_datum)
        vw::checkDatumConsistency(opt.datum, georef.datum(), warn_only);

      if (opt.datum_str == "") {
        opt.datum = georef.datum();
        opt.datum_str = opt.datum.name();
        have_datum = true;
      }
    }
  }

  // Set the datum, either based on what the user specified or the axes
  vw::cartography::Datum user_datum;
  bool have_user_datum = false;
  if (opt.datum_str != "") {
    try {
      user_datum.set_well_known_datum(opt.datum_str);
      have_user_datum = true;
    } catch(...) {
      // Whatever datum name we had, it was bad, so we'll make more attempts below
      opt.datum_str = "";
      have_user_datum = false;
    }
  } else if (opt.semi_major > 0 && opt.semi_minor > 0) {
    // Otherwise, if the user set the semi-axes, use that.
    user_datum = cartography::Datum("User Specified Datum",
                                    "User Specified Spheroid",
                                    "Reference Meridian",
                                    opt.semi_major, opt.semi_minor, 0.0);
    have_user_datum = true;
  }

  // Must check the consistency of the datums
  if (have_datum && have_user_datum)
    vw::checkDatumConsistency(opt.datum, user_datum, warn_only);

  if (!have_datum && have_user_datum) {
    opt.datum = user_datum;
    have_datum = true;
  }

  if (opt.datum_str.empty() && have_datum)
    opt.datum_str = opt.datum.name();

  // Try to find the datum from the cameras.
  vw::cartography::Datum cam_datum;
  bool have_cam_datum = asp::datum_from_camera(opt.image_files[0], opt.camera_files[0],
                                               // Outputs
                                               opt.stereo_session, session, cam_datum);

  // Must check the consistency of the datums. Skip that for pinhole cameras
  // if we already have a prior datum, as the result of that could be wrong
  // and it would just confuse the user.
  bool have_pinhole = (opt.stereo_session.find("pinhole") != std::string::npos);
  if (have_cam_datum && have_datum && !have_pinhole)
    vw::checkDatumConsistency(opt.datum, cam_datum, have_pinhole);

  // Otherwise try to set the datum based on cameras. It will not work for Pinhole.
  if (!have_datum && have_cam_datum) {
    opt.datum = cam_datum;
    opt.datum_str = opt.datum.name();
    have_datum = true;
  }

  // Many times the datum is mandatory
  if (!have_datum) {
    if (!opt.gcp_files.empty() || !opt.camera_position_file.empty())
      vw_throw(ArgumentErr() << "When ground control points or a camera position "
               << "file are used, option --datum must be specified.\n");
    if (opt.elevation_limit[0] < opt.elevation_limit[1])
      vw_throw(ArgumentErr()
                << "When filtering by elevation limit, option --datum must be specified.\n");
  }

  if (have_datum)
    vw_out() << "Datum:\n" << opt.datum << std::endl;
  else
    vw_out() << "No datum specified or detected.\n";

  // This is a little clumsy, but need to see whether the user set --max-iterations
  // or --num-iterations. They are aliases to each other.
  if (!vm["max-iterations"].defaulted() && !vm["num-iterations"].defaulted())
    vw_throw(ArgumentErr() << "Cannot set both --num-iterations and --max-iterations.\n");
  if (!vm["max-iterations"].defaulted())
    opt.num_iterations = max_iterations_tmp;

  load_intrinsics_options(opt.solve_intrinsics, !vm["intrinsics-to-share"].defaulted(),
                          intrinsics_to_float_str, intrinsics_to_share_str,
                          opt.intrinsics_options);
  asp::parse_intrinsics_limits(intrinsics_limit_str, opt.intrinsics_limits);

  boost::to_lower(opt.cost_function);

  if (opt.apply_initial_transform_only && opt.initial_transform_file == "")
    vw_throw(vw::IOErr() << "Cannot use --apply-initial-transform-only "
              << "without --initial-transform.\n");

  if (opt.initial_transform_file != "") {
    vw_out() << "Reading the alignment transform from: "
             << opt.initial_transform_file << "\n";
    vw::read_matrix_as_txt(opt.initial_transform_file, opt.initial_transform);
    if (opt.initial_transform.cols() != 4 || opt.initial_transform.rows() != 4)
      vw_throw(ArgumentErr() << "Could not read the initial transform.\n");
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
        vw_throw(vw::IOErr() << "The camera index to keep fixed " << val
                              << " is out of bounds.\n");
    }
  }

  if (!opt.fixed_cameras_indices.empty() && !opt.fixed_image_list.empty())
    vw_throw(ArgumentErr() << "Cannot specify both --fixed-camera-indices and "
             << "--fixed-image-list.\n");
  if (!opt.fixed_image_list.empty()) {

    opt.fixed_cameras_indices.clear();

    std::vector<std::string> fixed_images;
    asp::read_list(opt.fixed_image_list, fixed_images);

    // Find the indices of all images
    std::map<std::string, int> all_indices;
    for (size_t image_it = 0; image_it < opt.image_files.size(); image_it++)
      all_indices[opt.image_files[image_it]] = image_it;

    // Find the indices of images to fix
    for (size_t image_it = 0; image_it < fixed_images.size(); image_it++) {
      auto map_it = all_indices.find(fixed_images[image_it]);
      if (map_it == all_indices.end())
        vw_throw(ArgumentErr() << "Could not find image " << fixed_images[image_it]
                 << " read via --fixed-image-list among the input images.\n");
      opt.fixed_cameras_indices.insert(map_it->second);
    }
  }

  if (opt.reference_terrain != "") {
    std::string file_type = asp::get_cloud_type(opt.reference_terrain);
    if (file_type == "CSV" && opt.csv_format_str == "")
      vw_throw(ArgumentErr()
               << "When using a csv reference terrain, "
               << "must specify the csv-format.\n");
    if (!have_datum)
      vw_throw(ArgumentErr()
               << "When using a reference terrain, must specify the datum.\n");
    if (opt.disparity_list == "")
      vw_throw(ArgumentErr()
               << "When using a reference terrain, must specify a list "
               << "of disparities.\n");
    if (opt.max_disp_error <= 0)
      vw_throw(ArgumentErr()
               << "Must specify --max-disp-error in pixels as a positive value.\n");
    if (opt.reference_terrain_weight < 0)
      vw_throw(ArgumentErr()
               << "The value of --reference-terrain-weight must be non-negative.\n");
  }

  if (opt.match_files_prefix != "" && opt.clean_match_files_prefix != "")
    vw_throw(ArgumentErr()
              << "Cannot specify both --match-files-prefix and "
              << "--clean-match-files-prefix.\n");

  if (int(opt.proj_win != BBox2(0, 0, 0, 0)) + int(!opt.proj_str.empty()) == 1)
    vw_throw(ArgumentErr()
             << "Must specify both or neither of --proj-win and --proj-str.\n");

  if (int(opt.transform_cameras_using_gcp) +
      int(opt.transform_cameras_with_shared_gcp) +
      int(opt.init_camera_using_gcp) > 1)
    vw::vw_throw(vw::ArgumentErr()
                 << "Cannot specify more than one of --transform-cameras-using-gcp, "
                 << "--transform-cameras-with-shared-gcp, --init-camera-using-gcp.\n");

  if (opt.propagate_errors && !have_datum)
    vw_throw(ArgumentErr() << "Cannot propagate errors without a datum. Set --datum.\n");

  if (opt.update_isis_cubes_with_csm_state) {
    // This must happen after the session was auto-detected.
    bool have_csm = (opt.stereo_session == "csm");
    bool have_cub_input = boost::iends_with(boost::to_lower_copy(opt.image_files[0]),
                                            ".cub");
    if (!have_csm || !have_cub_input)
      vw::vw_throw(vw::ArgumentErr() << "Cannot update ISIS cubes with CSM state "
               << "unless using the CSM session with ISIS .cub images.\n");
  }

  // Prepare for computing footprints of images. Do this before loading all
  // cameras, which can take time. So fail early if things are not working.
  opt.pct_for_overlap = -1.0;
  if (opt.auto_overlap_params != "") {
    std::istringstream is(opt.auto_overlap_params);
    if (!(is >> opt.dem_file_for_overlap >> opt.pct_for_overlap))
      vw_throw(ArgumentErr()
                << "Could not parse correctly option --auto-overlap-params.\n");
      try {
        DiskImageView<float> dem(opt.dem_file_for_overlap);
      } catch (const Exception& e) {
        vw_throw(ArgumentErr()
                  << "Could not load DEM: " << opt.dem_file_for_overlap << "\n");
      }
      if (opt.pct_for_overlap < 0)
        vw_throw(ArgumentErr()
                  << "Invalid value for --auto-overlap-params.\n");
  }

  // If opt.camera_position_uncertainty is non-empty, read this file. It
  // has the image name and the uncertainty in the camera position.
  bool have_camera_position_uncertainty = !opt.camera_position_uncertainty_str.empty();
  if (have_camera_position_uncertainty)
   asp::handleCameraPositionUncertainty(opt, have_datum);

  // Set camera weight to 0 if camera position weight is positive
  if (opt.camera_position_weight > 0) {
    if (opt.camera_weight > 0) {
      vw::vw_out() << "Setting --camera-weight to 0 as --camera-position-weight "
                    << "is positive.\n";
      opt.camera_weight = 0;
    }
  }

  if (opt.use_llh_error && !have_datum)
    vw::vw_throw(vw::ArgumentErr()
              << "Cannot use --use-lon-lat-height-gcp-error without a datum. Set --datum.\n");

  if ((opt.calc_normalization_bounds || opt.calc_ip) &&
      (opt.stop_after_stats || opt.stop_after_matching))
    vw::vw_throw(vw::ArgumentErr()
              << "Cannot use --calc-normalization-bounds or --calc-ip with "
              << "--stop-after-stats or --stop-after-matching.\n");

  return;
}


int main(int argc, char* argv[]) {

  asp::BaOptions opt;
  try {
    xercesc::XMLPlatformUtils::Initialize();

    handle_arguments(argc, argv, opt);

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

    // Calculate which images overlap
    if (opt.auto_overlap_params != "") {
      opt.have_overlap_list = true;
      asp::build_overlap_list_based_on_dem(opt.out_prefix,
                                           opt.dem_file_for_overlap, opt.pct_for_overlap,
                                           opt.image_files, opt.camera_models,
                                           // output
                                           opt.overlap_list);
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
