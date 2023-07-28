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

/// \file jitter_adjust.cc
///
/// Use n adjustments for every camera, placed at several lines in the image
// with interpolation between them. The pdf doc has more info.

// TODO(oalexan1): Move some UsgsAstroLsSensorModel functions from
// here and from LinescanDGModel.cc to CsmUtils.cc.

// TODO(oalexan1): Add two passes and outlier filtering. For now
// try to use clean matches.

#include <asp/Sessions/StereoSessionFactory.h>
#include <asp/Sessions/CameraUtils.h>
#include <asp/Camera/CsmModel.h>
#include <asp/Camera/CsmUtils.h>
#include <asp/Camera/BundleAdjustCamera.h>
#include <asp/Camera/JitterSolveCostFuns.h>
#include <asp/Camera/JitterSolveUtils.h>
#include <asp/Core/Macros.h>
#include <asp/Core/Common.h>
#include <asp/Core/StereoSettings.h>
#include <asp/Core/BundleAdjustUtils.h>
#include <asp/Core/IpMatchingAlgs.h> // Lightweight header for matching algorithms
#include <asp/Core/SatSimBase.h>
#include <asp/Core/CameraTransforms.h>

#include <vw/BundleAdjustment/ControlNetwork.h>
#include <vw/BundleAdjustment/ControlNetworkLoader.h>
#include <vw/Core/Stopwatch.h>
#include <vw/Cartography/CameraBBox.h>

#include <usgscsm/UsgsAstroLsSensorModel.h>
#include <usgscsm/UsgsAstroFrameSensorModel.h>
#include <usgscsm/Utilities.h>

#include <xercesc/util/PlatformUtils.hpp>

#include <ceres/ceres.h>
#include <ceres/loss_function.h>

namespace po = boost::program_options;
namespace fs = boost::filesystem;

using namespace vw;
using namespace vw::math;

namespace asp {

struct Options: public asp::BaBaseOptions {
  int num_lines_per_position, num_lines_per_orientation, num_anchor_points;
  double quat_norm_weight, anchor_weight, roll_weight, yaw_weight;
  std::string anchor_dem;
  int num_anchor_points_extra_lines;
  bool initial_camera_constraint;
  std::map<int, int> orbital_groups;
};
    
void handle_arguments(int argc, char *argv[], Options& opt) {

  po::options_description general_options("");
  general_options.add_options()
    ("output-prefix,o",  po::value(&opt.out_prefix), "Prefix for output filenames.")
    ("session-type,t",   po::value(&opt.stereo_session)->default_value(""),
     "Select the stereo session type to use for processing. Usually the program "
     "can select this automatically by the file extension, except for xml cameras. "
     "See the doc for options.")
    ("input-adjustments-prefix",  po::value(&opt.input_prefix),
     "Prefix to read initial adjustments from, written by bundle_adjust. "
     "Not required. Cameras in .json files in ISD or model state format "
     "can be passed in with no adjustments.")
    ("num-lines-per-position", po::value(&opt.num_lines_per_position)->default_value(-1),
     "Resample the input camera positions and velocities, using this many lines per "
     "produced position and velocity. If not set, use the positions and velocities "
     "from the CSM file as they are.")
    ("num-lines-per-orientation", po::value(&opt.num_lines_per_orientation)->default_value(-1),
     "Resample the input camera orientations, using this many lines per produced orientation. "
     "If not set, use the orientations from the CSM file as they are.")
    ("match-first-to-last",
     po::value(&opt.match_first_to_last)->default_value(false)->implicit_value(true),
     "Match first several images to last several images by extending the logic of "
     "--overlap-limit past the last image to the earliest ones.")
    ("overlap-limit",        po::value(&opt.overlap_limit)->default_value(0),
     "Limit the number of subsequent images to search for matches to the current image "
     "to this value. By default match all images.")
    ("match-files-prefix",  po::value(&opt.match_files_prefix)->default_value(""),
     "Use the match files from this prefix instead of the current output prefix.")
    ("clean-match-files-prefix",  po::value(&opt.clean_match_files_prefix)->default_value(""),
     "Use as input match files the *-clean.match files from this prefix.")
    ("min-matches", po::value(&opt.min_matches)->default_value(30),
     "Set the minimum  number of matches between images that will be considered.")
    ("max-pairwise-matches", po::value(&opt.max_pairwise_matches)->default_value(10000),
     "Reduce the number of matches per pair of images to at most this "
     "number, by selecting a random subset, if needed. This happens "
     "when setting up the optimization, and before outlier filtering.")
    ("min-triangulation-angle", po::value(&opt.min_triangulation_angle)->default_value(0.1),
     "The minimum angle, in degrees, at which rays must meet at a triangulated point to "
     "accept this point as valid. It must be a positive value.")
    ("max-initial-reprojection-error", po::value(&opt.max_init_reproj_error)->default_value(10),
     "Filter as outliers triangulated points project using initial cameras with error more than "
     "this, measured in pixels. Since jitter corrections are supposed to be small and cameras "
     "bundle-adjusted by now, this value need not be too big.")
    ("robust-threshold", po::value(&opt.robust_threshold)->default_value(0.5),
     "Set the threshold for the Cauchy robust cost function. Increasing this makes "
     "the solver focus harder on the larger errors.")
    ("parameter-tolerance",  po::value(&opt.parameter_tolerance)->default_value(1e-12),
     "Stop when the relative error in the variables being optimized is less than this.")
    ("num-iterations",       po::value(&opt.num_iterations)->default_value(500),
     "Set the maximum number of iterations.")
    ("tri-weight", po::value(&opt.tri_weight)->default_value(0.0),
     "The weight to give to the constraint that optimized triangulated "
     "points stay close to original triangulated points. A positive "
     "value will help ensure the cameras do not move too far, but a "
     "large value may prevent convergence. Does not apply to GCP or "
     "points constrained by a DEM. This adds a robust cost function  "
     "with the threshold given by --tri-robust-threshold. "
     "The suggested value is 0.1 to 0.5 divided by the image ground "
     "sample distance.")
    ("tri-robust-threshold",
     po::value(&opt.tri_robust_threshold)->default_value(0.1),
     "Use this robust threshold to attenuate large differences "
     "between initial and optimized triangulation points, after multiplying "
     "them by --tri-weight.")
    ("heights-from-dem",   po::value(&opt.heights_from_dem)->default_value(""),
     "If the cameras have already been bundle-adjusted and aligned "
     "to a known DEM, in the triangulated points obtained from "
     "interest point matches replace the heights with the ones from this "
     "DEM before optimizing them while tying the points to this DEM via "
     "--heights-from-dem-weight and --heights-from-dem-robust-threshold.")
    ("heights-from-dem-weight", po::value(&opt.heights_from_dem_weight)->default_value(0.5),
     "How much weight to give to keep the triangulated points close "
     "to the DEM if specified via --heights-from-dem. This value "
     "should be about 0.1 to 0.5 divided by the image ground sample "
     "distance, as then it will convert the measurements from meters to "
     "pixels, which is consistent with the pixel reprojection error term.")
    ("heights-from-dem-robust-threshold",
     po::value(&opt.heights_from_dem_robust_threshold)->default_value(0.5),
     "The robust threshold to use keep the triangulated points "
     "close to the DEM if specified via --heights-from-dem. This is applied after the "
     "point differences are multiplied by --heights-from-dem-weight. It should help with "
     "attenuating large height difference outliers. It is suggested to make this equal to "
     "--heights-from-dem-weight.")
    ("reference-dem",  po::value(&opt.ref_dem)->default_value(""),
     "If specified, constrain every ground point where rays from matching pixels intersect "
     "to be not too far from the average of intersections of those rays with this DEM.")
    ("reference-dem-weight", po::value(&opt.ref_dem_weight)->default_value(1.0),
     "Multiply the xyz differences for the --reference-dem option by this weight.")
    ("reference-dem-robust-threshold", po::value(&opt.ref_dem_robust_threshold)->default_value(0.5),
     "Use this robust threshold for the weighted xyz differences.")
    ("num-anchor-points", po::value(&opt.num_anchor_points)->default_value(0),
     "How many anchor points to create. They will be uniformly distributed "
     "across each input image. Only applies to linescan cameras.")
    ("anchor-weight", po::value(&opt.anchor_weight)->default_value(0.0),
     "How much weight to give to each anchor point. Anchor points are "
     "obtained by intersecting rays from initial cameras with the DEM given by "
     "--heights-from-dem. A larger weight will make it harder for "
     "the cameras to move, hence preventing unreasonable changes. "
     "Set also --anchor-weight and --anchor-dem.")
    ("anchor-dem",  po::value(&opt.anchor_dem)->default_value(""),
     "Use this DEM to create anchor points.")
    ("num-anchor-points-extra-lines",
     po::value(&opt.num_anchor_points_extra_lines)->default_value(0),
     "Start placing anchor points this many lines before first image line "
     "and after last image line.")
    ("rotation-weight", po::value(&opt.rotation_weight)->default_value(0.0),
     "A higher weight will penalize more deviations from the original camera orientations.")
    ("translation-weight", po::value(&opt.translation_weight)->default_value(0.0),
     "A higher weight will penalize more deviations from "
     "the original camera positions.")
    ("quat-norm-weight", po::value(&opt.quat_norm_weight)->default_value(1.0),
     "How much weight to give to the constraint that the norm of each quaternion must be 1.")
    ("roll-weight", po::value(&opt.roll_weight)->default_value(0.0),
     "A weight to penalize the deviation of camera roll orientation as measured from the "
     "along-track direction. Pass in a large value, such as 1e+5. This is best used only with "
     "linescan cameras created with sat_sim.")
    ("yaw-weight", po::value(&opt.yaw_weight)->default_value(0.0),
     "A weight to penalize the deviation of camera yaw orientation as measured from the "
     "along-track direction. Pass in a large value, such as 1e+5. This is best used only "
     "with linescan cameras created with sat_sim.")
    ("ip-side-filter-percent",  po::value(&opt.ip_edge_buffer_percent)->default_value(-1.0),
     "Remove matched IPs this percentage from the image left/right sides.")
    ("initial-camera-constraint", 
    po::bool_switch(&opt.initial_camera_constraint)->default_value(false),
     "When constraining roll and yaw, measure these not in the satellite along-track/ "
     "across-track/down coordinate system, but relative to the initial camera poses. This "
     "is experimental. Internally, the roll weight will then be applied to the camera pitch "
     "angle (rotation around the camera y axis), because the camera coordinate system is "
     "rotated by 90 degrees in the sensor plane relative to the satellite coordinate system. "
     "The goal is the same, to penalize deviations that are not aligned with satellite pitch.")
    ;
  
    general_options.add(vw::GdalWriteOptionsDescription(opt));

  // TODO(oalexan1): This old option may need to be wiped given the newer
  // recent outlier filtering.
  asp::stereo_settings().ip_edge_buffer_percent = opt.ip_edge_buffer_percent;

  po::options_description positional("");
  positional.add_options()
    ("input-files", po::value(&opt.image_files));
  
  po::positional_options_description positional_desc;
  positional_desc.add("input-files", -1);

  std::string usage("<images> <cameras> -o <output prefix> [options]");
  bool allow_unregistered = false;
  std::vector<std::string> unregistered;
  po::variables_map vm =
    asp::check_command_line(argc, argv, opt, general_options, general_options,
                            positional, positional_desc, usage,
                            allow_unregistered, unregistered);

  // Do this check first, as the output prefix is used below many times
  if (opt.out_prefix == "") 
    vw_throw(ArgumentErr() << "Must specify the output prefix.\n" << usage << "\n");

  // Create the output directory
  vw::create_out_dir(opt.out_prefix);

  // Set this before loading cameras, as jitter for DG can be modeled only with CSM
  // cameras.
  asp::stereo_settings().dg_use_csm = true;
  
  std::vector<std::string> inputs = opt.image_files;
  bool ensure_equal_sizes = true;
  asp::separate_images_from_cameras(inputs,
                                    opt.image_files, opt.camera_files, // outputs
                                    ensure_equal_sizes); 

  // This is needed when several images are acquired in quick succession
  // and we want to impose roll and yaw constraints given their orbital 
  // trajectory.
  asp::readGroupStructure(inputs, opt.orbital_groups);

  // Throw if there are duplicate camera file names.
  asp::check_for_duplicates(opt.image_files, opt.camera_files, opt.out_prefix);
  
  const int num_images = opt.image_files.size();
  
  // Sanity check
  if (opt.image_files.size() != opt.camera_files.size())
    vw_throw(ArgumentErr() << "Must have as many cameras as  have images.\n");
  
  if (opt.image_files.empty())
    vw_throw(ArgumentErr() << "Missing input image files.\n");
  
  if (opt.overlap_limit < 0)
    vw_throw(ArgumentErr() << "Must allow search for matches between "
             << "at least each image and its subsequent one.\n");
  
  // By default, try to match all of the images
  if (opt.overlap_limit == 0)
    opt.overlap_limit = opt.image_files.size();
  
  if (int(opt.match_files_prefix.empty()) + int(opt.clean_match_files_prefix.empty()) != 1) 
    vw_throw(ArgumentErr() << "Must specify precisely one of: --match-files-prefix, "
             << "--clean-match-files-prefix.\n");

  if (opt.max_init_reproj_error <= 0.0)
    vw_throw(ArgumentErr() << "Must have a positive --max-initial-reprojection-error.\n");

  if (!opt.heights_from_dem.empty() && !opt.ref_dem.empty()) 
    vw_throw(ArgumentErr() << "Cannot specify more than one of: --heights-from-dem "
             << "and --reference-dem.\n");

  if (opt.tri_weight < 0.0) 
    vw_throw(ArgumentErr() << "The value of --tri-weight must be non-negative.\n");

  if (opt.robust_threshold <= 0.0) 
    vw_throw(ArgumentErr() << "The value of --robust-threshold must be positive.\n");

  if (opt.tri_robust_threshold <= 0.0) 
    vw_throw(ArgumentErr() << "The value of --tri-robust-threshold must be positive.\n");
  
  if (opt.heights_from_dem_weight <= 0.0) 
    vw_throw(ArgumentErr() << "The value of --heights-from-dem-weight must be positive.\n");
  
  if (opt.heights_from_dem_robust_threshold <= 0.0) 
    vw_throw(ArgumentErr() << "The value of --heights-from-robust-threshold must be positive.\n");

  if (opt.ref_dem_weight <= 0.0) 
    vw_throw(ArgumentErr() << "The value of --reference-dem-weight must be positive.\n");
  
  if (opt.ref_dem_robust_threshold <= 0.0) 
    vw_throw(ArgumentErr() << "The value of --reference-dem-robust-threshold must be positive.\n");

  if (opt.rotation_weight < 0 || opt.translation_weight < 0)
    vw_throw(ArgumentErr() << "Rotation and translation weights must be non-negative.\n");
    
  if (opt.quat_norm_weight <= 0)
    vw_throw(ArgumentErr() << "Quaternion norm weight must be positive.\n");

  if (opt.roll_weight < 0.0)
    vw_throw(ArgumentErr() << "Roll weight must be non-negative.\n");

  if (opt.yaw_weight < 0.0)
    vw_throw(ArgumentErr() << "Yaw weight must be non-negative.\n");

  // Handle the roll/yaw constraint DEM
  if ((opt.roll_weight > 0 || opt.yaw_weight > 0) &&
     opt.heights_from_dem == "" && opt.ref_dem == "" && opt.anchor_dem == "")
      vw::vw_throw(ArgumentErr() << "Cannot use the roll/yaw constraint without a DEM. "
        << "Set either --heights-from-dem, --anchor-dem, or --reference-dem.\n");

  if (opt.num_anchor_points < 0)
    vw_throw(ArgumentErr() << "The number of anchor points must be non-negative.\n");

  if (opt.anchor_weight < 0)
    vw_throw(ArgumentErr() << "Anchor weight must be non-negative.\n");

  if (opt.anchor_weight > 0 && opt.anchor_dem.empty()) 
    vw::vw_throw(vw::ArgumentErr() << "If --anchor-weight is positive, set --anchor-dem.\n");
  
  // Turn on logging to file
  asp::log_to_file(argc, argv, "", opt.out_prefix);
  
  return;
}

void compute_residuals(Options const& opt,
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

void write_per_xyz_pixel_residuals(vw::ba::ControlNetwork const& cnet,
                                   std::string            const& residual_prefix,
                                   vw::cartography::Datum const& datum,
                                   std::set<int>          const& outliers,
                                   std::vector<double>    const& tri_points_vec,
                                   std::vector<double>    const& mean_pixel_residual_norm,
                                   std::vector<int>       const& pixel_residual_count) {
    
  std::string map_prefix = residual_prefix + "_pointmap";
  std::string output_path = map_prefix + ".csv";

  int num_tri_points = tri_points_vec.size() / NUM_XYZ_PARAMS;
  
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

void write_anchor_residuals(std::string            const& residual_prefix,
                            vw::cartography::Datum const& datum,
                            std::vector<Vector3>   const& anchor_xyz,
                            std::vector<double>    const& anchor_residual_norm) {
  
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
                           
// TODO(oalexan1): Add here residuals for xyz discrepancy to DEM, if applicable
void save_residuals(std::string const& residual_prefix,
                    ceres::Problem & problem, Options const& opt,
                    vw::ba::ControlNetwork const& cnet,
                    vw::ba::CameraRelationNetwork<vw::ba::JFeature> const& crn,
                    bool have_dem, vw::cartography::Datum const& datum,
                    std::vector<double> const& tri_points_vec,
                    std::vector<Vector3> const& dem_xyz_vec,
                    std::set<int> const& outliers,
                    std::vector<double> const& weight_per_residual,
                    // These are needed for anchor points
                    std::vector<std::vector<Vector2>>                    const& pixel_vec,
                    std::vector<std::vector<boost::shared_ptr<Vector3>>> const& xyz_vec,
                    std::vector<std::vector<double*>>                    const& xyz_vec_ptr,
                    std::vector<std::vector<double>>                     const& weight_vec,
                    std::vector<std::vector<int>>                        const& isAnchor_vec) {
  
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
  if (have_dem)
    xyz_residual_norm.resize(num_tri_points, -1.0); // so we can ignore bad ones
  
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
        double * tri_point = xyz_vec_ptr[icam][ipix];
        double weight = weight_vec[icam][ipix];
        bool isAnchor = isAnchor_vec[icam][ipix];

        // Pass 0 is without anchor points, while pass 1 uses them.
        // Here we only do pass 1.
        if ((int)isAnchor != pass) 
          continue;

        if (weight != opt.anchor_weight)
          vw::vw_throw(vw::ArgumentErr() << "Expecting the weight to equal the anchor weight.\n");
        
        // Norm of pixel residual
        double norm = norm_2(Vector2(residuals[ires + 0] / weight_per_residual[ires + 0],
                                     residuals[ires + 1] / weight_per_residual[ires + 1]));
        norm /= weight; // Undo the weight, to recover the pixel norm
        
        ires += PIXEL_SIZE; // Update for the next iteration

        Vector3 xyz(tri_point[0], tri_point[1], tri_point[2]);
        anchor_xyz.push_back(xyz);
        anchor_residual_norm.push_back(norm);
      }
    }
  }
  write_anchor_residuals(residual_prefix, datum, anchor_xyz, anchor_residual_norm);
  
  // TODO(oalexan1): Add here per-camera median residuals.
  // TODO(oalexan1): Save the xyz residual norms as well.
  if (have_dem) {
    for (int ipt = 0; ipt < num_tri_points; ipt++) {

      Vector3 observation = dem_xyz_vec.at(ipt);
      if (outliers.find(ipt) != outliers.end() || observation == Vector3(0, 0, 0)) 
        continue; // outlier

      // This is a Vector3 residual 
      double norm = norm_2(Vector3(residuals[ires + 0] / weight_per_residual[ires + 0],
                                   residuals[ires + 1] / weight_per_residual[ires + 1],
                                   residuals[ires + 2] / weight_per_residual[ires + 2]));
      xyz_residual_norm[ipt] = norm;
      
      ires += NUM_XYZ_PARAMS; // Update for the next iteration
    }
  }

  // Ensure we did not process more residuals than what we have.
  // (Here we may not necessarily process all residuals.)
  if (ires > (int)residuals.size())
    vw_throw(ArgumentErr() << "More residuals found than expected.\n");

  return;
}


// Calc the time of first image line, last image line, elapsed time
// between these lines, and elapsed time per line.  This assumes a
// linear relationship between lines and time.
// TODO(oalexan1): This is fragile. Maybe it can be avoided.
void calcTimes(UsgsAstroLsSensorModel const* ls_model,
               double & earlier_line_time, double & later_line_time,
               double & elapsed_time, double & dt_per_line) {

  int numLines = ls_model->m_nLines;
  csm::ImageCoord imagePt;

  asp::toCsmPixel(vw::Vector2(0, 0), imagePt);
  earlier_line_time = ls_model->getImageTime(imagePt);

  asp::toCsmPixel(vw::Vector2(0, numLines - 1), imagePt);
  later_line_time = ls_model->getImageTime(imagePt);

  // See note in resampleModel().
  if (earlier_line_time > later_line_time)
    std::swap(earlier_line_time, later_line_time);
  
  elapsed_time = later_line_time - earlier_line_time;
  dt_per_line = elapsed_time / (numLines - 1.0);

  if (later_line_time <= earlier_line_time)
    vw::vw_throw(vw::ArgumentErr()
                 << "The time of the last line (in scanning order) must be larger than "
                 << "first line time.\n");
  
  return;
}

// Calculate the line index for first and last tabulated position.
// We always expect these to be less than first line index (0), and no less
// than last valid image line index (numLines - 1), respectively.
// TODO(oalexan1): This assumes a linear relationship between time and lines,
// which is fragile. At least need to check that this assumption is satisfied.
void calcFirstLastPositionLines(UsgsAstroLsSensorModel const* ls_model, 
                                double & beg_position_line, double & end_position_line) {

  double earlier_line_time = -1.0, later_line_time = -1.0, 
         elapsed_time = -1.0, dt_per_line = -1.0;
  calcTimes(ls_model, earlier_line_time, later_line_time, elapsed_time,  
               dt_per_line);
  
  // Find time of first and last tabulated position.
  double bt = ls_model->m_t0Ephem;
  double et = bt + (ls_model->m_positions.size()/NUM_XYZ_PARAMS - 1) * ls_model->m_dtEphem;

  // Use the equation: time = earlier_line_time + line * dt_per_line.
  // See note in resampleModel() about scan direction.
  beg_position_line = (bt - earlier_line_time) / dt_per_line;
  end_position_line = (et - earlier_line_time) / dt_per_line;

  // Sanity checks
  if (beg_position_line > 1e-3) // allow for rounding errors 
    vw::vw_throw(vw::ArgumentErr() << "Line of first tabulated position is "
                 << beg_position_line << ", which is after first image line, which is "
                 << 0 << ".\n");
  int numLines = ls_model->m_nLines;
  if (end_position_line < numLines - 1 - 1e-3)  // allow for rounding errors
    vw::vw_throw(vw::ArgumentErr() << "Line of last tabulated position is "
                 << end_position_line << ", which is before last image line, which is "
                 << numLines - 1 << ".\n");
}
  
// Calculate the line index for first and last tabulated orientation.
// We always expect these to be less than first line index (0), and no less
// than last valid image line index (numLines - 1), respectively.
void calcFirstLastOrientationLines(UsgsAstroLsSensorModel const* ls_model, 
                                   double & beg_orientation_line, double & end_orientation_line) {

  double earlier_line_time = -1.0, later_line_time = -1.0, 
         elapsed_time = -1.0, dt_per_line = -1.0;
  calcTimes(ls_model, earlier_line_time, later_line_time, elapsed_time,  
               dt_per_line);
  
  // Find time of first and last tabulated orientation.
  double bt = ls_model->m_t0Quat;
  double et = bt + (ls_model->m_quaternions.size()/NUM_QUAT_PARAMS - 1) * ls_model->m_dtQuat;
  
  // Use the equation: time = earlier_line_time + line * dt_per_line.
  beg_orientation_line = (bt - earlier_line_time) / dt_per_line;
  end_orientation_line = (et - earlier_line_time) / dt_per_line;

  // Sanity checks
  if (beg_orientation_line > 1e-3) // allow for rounding errors 
    vw::vw_throw(vw::ArgumentErr() << "Line of first tabulated orientation is "
                 << beg_orientation_line << ", which is after first image line, which is "
                   << 0 << ".\n");
  int numLines = ls_model->m_nLines;
  if (end_orientation_line < numLines - 1 - 1e-3)  // allow for rounding errors
    vw::vw_throw(vw::ArgumentErr() << "Line of last tabulated orientation is "
                 << end_orientation_line << ", which is before last image line, which is "
                   << numLines - 1 << ".\n");
}

// The provided tabulated positions, velocities and quaternions may be too few,
// so resample them with --num-lines-per-position and --num-lines-per-orientation,
// if those are set. Throughout this function the lines are indexed in the order
// they are acquired, which can be the reverse of the order they are eventually
// stored in the file if the scan direction is reverse.
void resampleModel(Options const& opt, UsgsAstroLsSensorModel * ls_model) {
  
  // The positions and quaternions can go way beyond the valid range of image lines,
  // so need to estimate how many of them are within the range.
  
  int numLines = ls_model->m_nLines;
  vw_out() << "Number of lines: " << numLines << ".\n";

  double earlier_line_time = -1.0, later_line_time = -1.0, elapsed_time = -1.0, dt_per_line = -1.0;
  calcTimes(ls_model, earlier_line_time, later_line_time, elapsed_time,  
            dt_per_line);

  // Line index of first and last tabulated position
  double beg_position_line = -1.0, end_position_line = -1.0;
  calcFirstLastPositionLines(ls_model, beg_position_line, end_position_line);
  vw_out() << std::setprecision (17) << "Line of first and last tabulated position: "
           << beg_position_line << ' ' << end_position_line << "\n";

  // Line index of first and last tabulated orientation
  double beg_orientation_line = -1.0, end_orientation_line = -1.0;
  calcFirstLastOrientationLines(ls_model, beg_orientation_line, end_orientation_line);
  vw_out() << std::setprecision (17) << "Line of first and last tabulated orientation: "
           << beg_orientation_line << ' ' << end_orientation_line << "\n";

  double numInputLinesPerPosition = (numLines - 1) * ls_model->m_dtEphem / elapsed_time;
  double numInputLinesPerOrientation = (numLines - 1) * ls_model->m_dtQuat / elapsed_time;
  vw_out() << "Number of image lines per input position: "
           << round(numInputLinesPerPosition) << "\n";
  vw_out() << "Number of image lines per input orientation: "
           << round(numInputLinesPerOrientation) << "\n";

  if (opt.num_lines_per_position > 0) {
    // Resample in such a way that first and last samples are preserved. This is tricky.
    double posFactor = double(numInputLinesPerPosition) / double(opt.num_lines_per_position);
    if (posFactor <= 0.0)
      vw::vw_throw(vw::ArgumentErr() << "Invalid image.\n");

    int numOldMeas = ls_model->m_numPositions / NUM_XYZ_PARAMS;
    int numNewMeas = round(posFactor * (numOldMeas - 1.0)) + 1; // careful here
    numNewMeas = std::max(numNewMeas, 2);

    posFactor = double(numNewMeas - 1.0) / double(numOldMeas - 1.0);
    double currDtEphem = ls_model->m_dtEphem / posFactor;
    double numLinesPerPosition = (numLines - 1.0) * currDtEphem / elapsed_time;
    vw_out() << "Resampled number of lines per position: "
             << numLinesPerPosition << "\n";
    std::vector<double> positions(NUM_XYZ_PARAMS * numNewMeas, 0);
    std::vector<double> velocities(NUM_XYZ_PARAMS * numNewMeas, 0);
    for (int ipos = 0; ipos < numNewMeas; ipos++) {
      double time = ls_model->m_t0Ephem + ipos * currDtEphem;
      asp::interpPositions(ls_model, time, &positions[NUM_XYZ_PARAMS * ipos]);
      asp::interpVelocities(ls_model, time, &velocities[NUM_XYZ_PARAMS * ipos]);
    }
    
    // Overwrite in the model. Time of first tabulated position does not change.
    ls_model->m_dtEphem = currDtEphem;
    ls_model->m_numPositions = positions.size();
    ls_model->m_positions = positions;
    ls_model->m_velocities = velocities;

    // Sanity check
    double new_beg_position_line = -1.0, new_end_position_line = -1.0;
    calcFirstLastPositionLines(ls_model, new_beg_position_line, new_end_position_line);
    if (std::abs(beg_position_line - new_beg_position_line) > 1.0e-3 ||
        std::abs(end_position_line - new_end_position_line) > 1.0e-3)
      vw::vw_throw(vw::ArgumentErr() << "Bookkeeping failure. Resampling was done "
                   << "without preserving first and last tabulated position time.\n");
  }

  if (opt.num_lines_per_orientation > 0) {
    // Resample in such a way that first and last samples are preserved. This is tricky.
    double posFactor = double(numInputLinesPerOrientation) / double(opt.num_lines_per_orientation);
    if (posFactor <= 0.0)
      vw::vw_throw(vw::ArgumentErr() << "Invalid image.\n");

    int numOldMeas = ls_model->m_numQuaternions / NUM_QUAT_PARAMS;
    int numNewMeas = round(posFactor * (numOldMeas - 1.0)) + 1; // careful here
    numNewMeas = std::max(numNewMeas, 2);
    
    posFactor = double(numNewMeas - 1.0) / double(numOldMeas - 1.0);
    double currDtQuat = ls_model->m_dtQuat / posFactor;
    double numLinesPerOrientation = (numLines - 1.0) * currDtQuat / elapsed_time;
    vw_out() << "Resampled number of lines per orientation: "
             << numLinesPerOrientation << "\n";
    std::vector<double> quaternions(NUM_QUAT_PARAMS * numNewMeas, 0);
    for (int ipos = 0; ipos < numNewMeas; ipos++) {
      double time = ls_model->m_t0Quat + ipos * currDtQuat;
      asp::interpQuaternions(ls_model, time, &quaternions[NUM_QUAT_PARAMS * ipos]);
    }
    
    // Overwrite in the model. Time of first tabulated orientation does not change.
    ls_model->m_dtQuat = currDtQuat;
    ls_model->m_numQuaternions = quaternions.size();
    ls_model->m_quaternions = quaternions;

    // Sanity check
    double new_beg_orientation_line = -1.0, new_end_orientation_line = -1.0;
    calcFirstLastOrientationLines(ls_model, new_beg_orientation_line, new_end_orientation_line);
    if (std::abs(beg_orientation_line - new_beg_orientation_line) > 1.0e-3 ||
        std::abs(end_orientation_line - new_end_orientation_line) > 1.0e-3)
      vw::vw_throw(vw::ArgumentErr() << "Bookkeeping failure. Resampling was done "
                   << "without preserving first and last tabulated orientation time.\n");
  }

  return;
}

// Calculate a set of anchor points uniformly distributed over the image
// Will use opt.num_anchor_points_extra_lines.
void calcAnchorPoints(Options                              const  & opt,
                      ImageViewRef<PixelMask<double>>               interp_anchor_dem,
                      vw::cartography::GeoReference         const & anchor_georef,
                      std::vector<asp::CsmModel*>           const & csm_models,
                      // Append to these, they already have entries
                      std::vector<std::vector<Vector2>>                    & pixel_vec,
                      std::vector<std::vector<boost::shared_ptr<Vector3>>> & xyz_vec,
                      std::vector<std::vector<double*>>                    & xyz_vec_ptr,
                      std::vector<std::vector<double>>                     & weight_vec,
                      std::vector<std::vector<int>>                        & isAnchor_vec) {

  if (opt.num_anchor_points <= 0)
    vw::vw_throw(vw::ArgumentErr() << "Expecting a positive number of anchor points.\n");

  int extra = opt.num_anchor_points_extra_lines;
    
  int num_cams = csm_models.size();
  for (int icam = 0; icam < num_cams; icam++) {

    UsgsAstroLsSensorModel * ls_model
      = dynamic_cast<UsgsAstroLsSensorModel*>((csm_models[icam]->m_gm_model).get());
    if (ls_model == NULL)
      continue; // anchor points not implemented for Frame cameras

    // Use int64 and double to avoid int32 overflow
    std::int64_t numLines   = ls_model->m_nLines;
    std::int64_t numSamples = ls_model->m_nSamples;
    double area = double(numSamples) * double(numLines + 2 * extra);
    double bin_len = sqrt(area/double(opt.num_anchor_points));
    bin_len = std::max(bin_len, 1.0);
    int lenx = ceil(double(numSamples) / bin_len); lenx = std::max(1, lenx);
    int leny = ceil(double(numLines + 2 * extra) / bin_len); leny = std::max(1, leny);

    std::int64_t numAnchorPoints = 0;
    for (int binx = 0; binx <= lenx; binx++) {
      double posx = binx * bin_len;
      for (int biny = 0; biny <= leny; biny++) {
        double posy = biny * bin_len - extra;
        
        if (posx > numSamples - 1 || posy < -extra || posy > numLines - 1 + extra) 
          continue;
        
        Vector2 pix(posx, posy);
        Vector3 xyz_guess(0, 0, 0);
        
        bool treat_nodata_as_zero = false;
        bool has_intersection = false;
        double height_error_tol = 0.001; // 1 mm should be enough
        double max_abs_tol      = 1e-14; // abs cost fun change b/w iterations
        double max_rel_tol      = 1e-14;
        int num_max_iter        = 50;   // Using many iterations can be very slow
          
        Vector3 dem_xyz = vw::cartography::camera_pixel_to_dem_xyz
          (opt.camera_models[icam]->camera_center(pix),
           opt.camera_models[icam]->pixel_to_vector(pix),
           interp_anchor_dem, anchor_georef, treat_nodata_as_zero, has_intersection,
           height_error_tol, max_abs_tol, max_rel_tol, num_max_iter, xyz_guess);

        if (!has_intersection) 
          continue;

        Vector2 pix_out;
        try {
          pix_out = opt.camera_models[icam]->point_to_pixel(dem_xyz);
        } catch (...) {
          continue;
        }
        
        if (norm_2(pix - pix_out) > 10 * height_error_tol)
          continue; // this is likely a bad point

        pixel_vec[icam].push_back(pix);
        weight_vec[icam].push_back(opt.anchor_weight);
        isAnchor_vec[icam].push_back(1);

        // Create a shared_ptr as we need a pointer per the api to use later
        xyz_vec[icam].push_back(boost::shared_ptr<Vector3>(new Vector3()));
        Vector3 & xyz = *xyz_vec[icam].back().get(); // alias to the element we just made
        xyz = dem_xyz; // copy the value, but the pointer does not change
        xyz_vec_ptr[icam].push_back(&xyz[0]); // keep the pointer to the first element
        numAnchorPoints++;
      }   
    }

    vw_out() << std::endl;
    vw_out() << "Image file: " << opt.image_files[icam] << std::endl;
    vw_out() << "Lines and samples: " << numLines << ' ' << numSamples << std::endl;
    vw_out() << "Num anchor points per image: " << numAnchorPoints     << std::endl;
  }   
}

// Add the linescan model reprojection error to the cost function
void addLsReprojectionErr(Options          const & opt,
                          UsgsAstroLsSensorModel * ls_model,
                          vw::Vector2      const & observation,
                          double                 * tri_point,
                          double                   weight,
                          ceres::Problem         & problem) {

  // Must grow the number of quaternions and positions a bit
  // because during optimization the 3D point and corresponding
  // pixel may move somewhat.
  double line_extra = opt.max_init_reproj_error + 5.0; // add some more just in case
  csm::ImageCoord imagePt1, imagePt2;
  asp::toCsmPixel(observation - Vector2(0.0, line_extra), imagePt1);
  asp::toCsmPixel(observation + Vector2(0.0, line_extra), imagePt2);
  double time1 = ls_model->getImageTime(imagePt1);
  double time2 = ls_model->getImageTime(imagePt2);

  // Handle quaternions. We follow closely the conventions for UsgsAstroLsSensorModel.
  int numQuatPerObs = 8; // Max num of quaternions used in pose interpolation 
  int numQuat       = ls_model->m_quaternions.size() / NUM_QUAT_PARAMS;
  double quatT0     = ls_model->m_t0Quat;
  double quatDt     = ls_model->m_dtQuat;

  // Starting and ending quat index (ending is exclusive). Based on lagrangeInterp().
  int qindex1      = static_cast<int>((time1 - quatT0) / quatDt);
  int qindex2      = static_cast<int>((time2 - quatT0) / quatDt);
  int begQuatIndex = std::min(qindex1, qindex2) - numQuatPerObs / 2 + 1;
  int endQuatIndex = std::max(qindex1, qindex2) + numQuatPerObs / 2 + 1;

  // Keep in bounds
  begQuatIndex = std::max(0, begQuatIndex);
  endQuatIndex = std::min(endQuatIndex, numQuat);
  if (begQuatIndex >= endQuatIndex) {
    // Must not happen 
    vw::vw_throw(vw::ArgumentErr() << "Book-keeping error for quaternions for pixel: " 
      << observation << ". Check your image dimensions and compare "
      << "with the camera file.\n"); 
  }

  // Same for positions
  int numPosPerObs = 8;
  int numPos       = ls_model->m_positions.size() / NUM_XYZ_PARAMS;
  double posT0     = ls_model->m_t0Ephem;
  double posDt     = ls_model->m_dtEphem;

  // Starting and ending pos index (ending is exclusive). Based on lagrangeInterp().
  int pindex1 = static_cast<int>((time1 - posT0) / posDt);
  int pindex2 = static_cast<int>((time2 - posT0) / posDt);
  int begPosIndex = std::min(pindex1, pindex2) - numPosPerObs / 2 + 1;
  int endPosIndex = std::max(pindex1, pindex2) + numPosPerObs / 2 + 1;

  // Keep in bounds
  begPosIndex = std::max(0, begPosIndex);
  endPosIndex = std::min(endPosIndex, numPos);
  if (begPosIndex >= endPosIndex) // Must not happen 
    vw_throw(ArgumentErr() << "Book-keeping error for positions for pixel: " 
      << observation << ". Check your image dimensions and compare "
      << "with the camera file.\n");

  ceres::CostFunction* pixel_cost_function =
    LsPixelReprojErr::Create(observation, weight, ls_model,
                              begQuatIndex, endQuatIndex,
                              begPosIndex, endPosIndex);
  ceres::LossFunction* pixel_loss_function = new ceres::CauchyLoss(opt.robust_threshold);

  // The variable of optimization are camera quaternions and positions stored in the
  // camera models, and the triangulated point.
  std::vector<double*> vars;
  for (int it = begQuatIndex; it < endQuatIndex; it++)
    vars.push_back(&ls_model->m_quaternions[it * NUM_QUAT_PARAMS]);
  for (int it = begPosIndex; it < endPosIndex; it++)
    vars.push_back(&ls_model->m_positions[it * NUM_XYZ_PARAMS]);
  vars.push_back(tri_point);
  problem.AddResidualBlock(pixel_cost_function, pixel_loss_function, vars);

  return;   
}

// Add the frame camera model reprojection error to the cost function
void addFrameReprojectionErr(Options             const & opt,
                             UsgsAstroFrameSensorModel * frame_model,
                             vw::Vector2         const & observation,
                             double                    * frame_params,
                             double                    * tri_point,
                             double                      weight,
                             ceres::Problem            & problem) {

  ceres::CostFunction* pixel_cost_function =
    FramePixelReprojErr::Create(observation, weight, frame_model);
  ceres::LossFunction* pixel_loss_function = new ceres::CauchyLoss(opt.robust_threshold);

  // The variable of optimization are camera positions and quaternion stored 
  // in frame_cam_params, in this order, and the triangulated point.
  // This is different from the linescan model, where we can directly access
  // these quantities inside the model, so they need not be stored separately.
  std::vector<double*> vars;
  vars.push_back(&frame_params[0]);              // positions start here
  vars.push_back(&frame_params[NUM_XYZ_PARAMS]); // quaternions start here
  vars.push_back(tri_point);
  problem.AddResidualBlock(pixel_cost_function, pixel_loss_function, vars);

  return;   
}

void addReprojectionErrors
(Options                                              const & opt,
 vw::ba::CameraRelationNetwork<vw::ba::JFeature>      const & crn,
 std::vector<std::vector<Vector2>>                    const & pixel_vec,
 std::vector<std::vector<boost::shared_ptr<Vector3>>> const & xyz_vec,
 std::vector<std::vector<double*>>                    const & xyz_vec_ptr,
 std::vector<std::vector<double>>                     const & weight_vec,
 std::vector<std::vector<int>>                        const & isAnchor_vec,
 std::vector<asp::CsmModel*>                          const & csm_models,
 // Outputs
 std::vector<double>                                        & frame_params,
 std::vector<double>                                        & weight_per_residual, // append
 ceres::Problem                                             & problem) {

  // Do here two passes, first for non-anchor points and then for anchor ones.
  // This way it is easier to do the bookkeeping when saving the residuals.
  // Note: The same motions as here are repeated in save_residuals().
  for (int pass = 0; pass < 2; pass++) {
    for (int icam = 0; icam < (int)crn.size(); icam++) {

      for (size_t ipix = 0; ipix < pixel_vec[icam].size(); ipix++) {

        Vector2 observation = pixel_vec[icam][ipix];
        double * tri_point  = xyz_vec_ptr[icam][ipix];
        double weight       = weight_vec[icam][ipix];
        bool isAnchor       = isAnchor_vec[icam][ipix];

        // Pass 0 is without anchor points, while pass 1 uses them
        if ((int)isAnchor != pass) 
          continue;

        // We can have linescan or frame cameras 
        UsgsAstroLsSensorModel * ls_model
          = dynamic_cast<UsgsAstroLsSensorModel*>((csm_models[icam]->m_gm_model).get());
        UsgsAstroFrameSensorModel * frame_model
          = dynamic_cast<UsgsAstroFrameSensorModel*>((csm_models[icam]->m_gm_model).get());
  
        // Note how for the frame model we pass the frame_params for the current camera.
        if (ls_model != NULL)
          addLsReprojectionErr(opt, ls_model, observation, tri_point, weight, problem);
        else if (frame_model != NULL)
          addFrameReprojectionErr(opt, frame_model, observation, 
              &frame_params[icam * (NUM_XYZ_PARAMS + NUM_QUAT_PARAMS)],
              tri_point, weight, problem);                   
        else
          vw::vw_throw(vw::ArgumentErr() << "Unknown camera model.\n");

        // Two residuals were added. Save the corresponding weights.
        for (int c = 0; c < PIXEL_SIZE; c++)
          weight_per_residual.push_back(weight);

        // Anchor points are fixed by definition. They try to prevent
        // the cameras from moving too much from original poses.
        if (isAnchor) 
          problem.SetParameterBlockConstant(tri_point);
      }
    }
  }
}

// Add the constraint based on DEM
void addDemConstraint
(Options                                              const& opt,
 std::vector<std::vector<boost::shared_ptr<Vector3>>> const& xyz_vec,
 std::vector<std::vector<double*>>                    const& xyz_vec_ptr,
 std::vector<vw::Vector3>                             const& dem_xyz_vec,
 std::set<int>                                        const& outliers,
 vw::ba::ControlNetwork                               const& cnet,
 // Outputs
 std::vector<double>                                       & tri_points_vec,
 std::vector<double>                                       & weight_per_residual, // append
 ceres::Problem                                            & problem) {
  
  double xyz_weight = -1.0, xyz_threshold = -1.0;
    
  if (!opt.heights_from_dem.empty()) {
    xyz_weight = opt.heights_from_dem_weight;
    xyz_threshold = opt.heights_from_dem_robust_threshold;
  } else if (!opt.ref_dem.empty()) {
    xyz_weight = opt.ref_dem_weight;
    xyz_threshold = opt.ref_dem_robust_threshold;
  } else {
    vw::vw_throw(vw::ArgumentErr() << "No input DEM was provided.\n");
  }
  
  if (dem_xyz_vec.size() != cnet.size()) 
    vw_throw(ArgumentErr() << "Must have as many xyz computed from DEM as xyz "
             << "triangulated from match files.\n");
  if (xyz_weight <= 0 || xyz_threshold <= 0)
    vw_throw(ArgumentErr() << "Detected invalid robust threshold or weights.\n");

  int num_tri_points = cnet.size();
  for (int ipt = 0; ipt < num_tri_points; ipt++) {
      
    if (cnet[ipt].type() == vw::ba::ControlPoint::GroundControlPoint)
      vw_throw(ArgumentErr() << "Found GCP where not expecting any.\n");

    // Note that we get tri points from dem_xyz_vec, based on the input DEM
    Vector3 observation = dem_xyz_vec.at(ipt);
    if (outliers.find(ipt) != outliers.end() || observation == Vector3(0, 0, 0)) 
      continue; // outlier
      
    ceres::CostFunction* xyz_cost_function = weightedXyzError::Create(observation, xyz_weight);
    ceres::LossFunction* xyz_loss_function = new ceres::CauchyLoss(xyz_threshold);
    double * tri_point = &tri_points_vec[0] + ipt * NUM_XYZ_PARAMS;

    // Add cost function
    problem.AddResidualBlock(xyz_cost_function, xyz_loss_function, tri_point);

    for (int c = 0; c < NUM_XYZ_PARAMS; c++)
      weight_per_residual.push_back(xyz_weight);
  }
}

// Add the constraint to keep triangulated points close to initial values
// This does not need a DEM or alignment
void addTriConstraint
(Options                                              const& opt,
 std::set<int>                                        const& outliers,
 vw::ba::ControlNetwork                               const& cnet,
 // Outputs
 std::vector<double>                                       & tri_points_vec,
 std::vector<double>                                       & weight_per_residual, // append
 ceres::Problem                                            & problem) {

  int num_tri_points = cnet.size();
  for (int ipt = 0; ipt < num_tri_points; ipt++) {
    if (cnet[ipt].type() == vw::ba::ControlPoint::GroundControlPoint ||
        cnet[ipt].type() == vw::ba::ControlPoint::PointFromDem)
      continue; // Skip GCPs and height-from-dem points which have their own constraint

    if (outliers.find(ipt) != outliers.end()) 
      continue; // skip outliers
      
    double * tri_point = &tri_points_vec[0] + ipt * NUM_XYZ_PARAMS;
      
    // Use as constraint the initially triangulated point
    vw::Vector3 observation(tri_point[0], tri_point[1], tri_point[2]);

    ceres::CostFunction* cost_function = weightedXyzError::Create(observation, opt.tri_weight);
    ceres::LossFunction* loss_function = new ceres::CauchyLoss(opt.tri_robust_threshold);
    problem.AddResidualBlock(cost_function, loss_function, tri_point);
    
    for (int c = 0; c < NUM_XYZ_PARAMS; c++)
      weight_per_residual.push_back(opt.tri_weight);
      
  } // End loop through xyz
}

void addQuatNormRotationTranslationConstraints(
    Options                                         const & opt,
    std::set<int>                                   const & outliers,
    vw::ba::CameraRelationNetwork<vw::ba::JFeature> const & crn,
    std::vector<asp::CsmModel*>                     const & csm_models,
    // Outputs
    std::vector<double>                                   & frame_params,
    std::vector<double>                                   & tri_points_vec,
    std::vector<double>                                   & weight_per_residual, // append
    ceres::Problem                                        & problem) {
  
  // Constrain the rotations
  // TODO(oalexan1): Make this a standalone function
  if (opt.rotation_weight > 0.0) {
    for (int icam = 0; icam < (int)crn.size(); icam++) {

      UsgsAstroLsSensorModel * ls_model
        = dynamic_cast<UsgsAstroLsSensorModel*>((csm_models[icam]->m_gm_model).get());
      UsgsAstroFrameSensorModel * frame_model
        = dynamic_cast<UsgsAstroFrameSensorModel*>((csm_models[icam]->m_gm_model).get());

      if (ls_model != NULL) {
        // There are multiple quaternion parameters per camera
        int numQuat = ls_model->m_quaternions.size() / NUM_QUAT_PARAMS;
        for (int iq = 0; iq < numQuat; iq++) {
          ceres::CostFunction* rotation_cost_function
            = weightedRotationError::Create(&ls_model->m_quaternions[iq * NUM_QUAT_PARAMS],
                                            opt.rotation_weight);
          // We use no loss function, as the quaternions have no outliers
          ceres::LossFunction* rotation_loss_function = NULL;
          problem.AddResidualBlock(rotation_cost_function, rotation_loss_function,
                                  &ls_model->m_quaternions[iq * NUM_QUAT_PARAMS]);
          
          for (int c = 0; c < NUM_QUAT_PARAMS; c++)
            weight_per_residual.push_back(opt.rotation_weight);
        }

      } else if (frame_model != NULL) {
        // There is one quaternion per camera, stored after the translation
        double * curr_params = &frame_params[icam * (NUM_XYZ_PARAMS + NUM_QUAT_PARAMS)];
          
        // Copy from curr_params the initial quaternion
        ceres::CostFunction* rotation_cost_function
          = weightedRotationError::Create(&curr_params[NUM_XYZ_PARAMS], // quat starts here
                                          opt.rotation_weight);
        // Pass the quaternion to optimize to the problem                                  
        // We use no loss function, as the quaternions have no outliers
        ceres::LossFunction* rotation_loss_function = NULL;
        problem.AddResidualBlock(rotation_cost_function, rotation_loss_function,
                                &curr_params[NUM_XYZ_PARAMS]); // quat starts here
        
        for (int c = 0; c < NUM_QUAT_PARAMS; c++)
          weight_per_residual.push_back(opt.rotation_weight);
      } else {
         vw::vw_throw(vw::ArgumentErr() << "Unknown camera model.\n");
      }

    } // end loop through cameras
  }

  // Constrain the translations
  // TODO(oalexan1): Make this a standalone function
  if (opt.translation_weight > 0.0) {
    for (int icam = 0; icam < (int)crn.size(); icam++) {

      UsgsAstroLsSensorModel * ls_model
        = dynamic_cast<UsgsAstroLsSensorModel*>((csm_models[icam]->m_gm_model).get());
      UsgsAstroFrameSensorModel * frame_model
        = dynamic_cast<UsgsAstroFrameSensorModel*>((csm_models[icam]->m_gm_model).get());

      if (ls_model != NULL) {

        int numPos = ls_model->m_positions.size() / NUM_XYZ_PARAMS;
        for (int ip = 0; ip < numPos; ip++) {
          ceres::CostFunction* translation_cost_function
            = weightedTranslationError::Create(&ls_model->m_positions[ip * NUM_XYZ_PARAMS],
                                            opt.translation_weight);
          // We use no loss function, as the positions have no outliers
          ceres::LossFunction* translation_loss_function = NULL;
          problem.AddResidualBlock(translation_cost_function, translation_loss_function,
                                  &ls_model->m_positions[ip * NUM_XYZ_PARAMS]);
          
          for (int c = 0; c < NUM_XYZ_PARAMS; c++)
            weight_per_residual.push_back(opt.translation_weight);
        }

      } else if (frame_model != NULL) {

        // There is only one position per camera
        double * curr_params = &frame_params[icam * (NUM_XYZ_PARAMS + NUM_QUAT_PARAMS)];
        // we will copy from curr_params the initial position
        ceres::CostFunction* translation_cost_function
          = weightedTranslationError::Create(&curr_params[0], // translation starts here
                                             opt.translation_weight);

        // We use no loss function, as the positions have no outliers
        ceres::LossFunction* translation_loss_function = NULL;
        problem.AddResidualBlock(translation_cost_function, translation_loss_function,
                                &curr_params[0]); // translation starts here
        
        for (int c = 0; c < NUM_XYZ_PARAMS; c++)
          weight_per_residual.push_back(opt.translation_weight);

      } else {
         vw::vw_throw(vw::ArgumentErr() << "Unknown camera model.\n");
      }

    }
  }

  // Try to make the norm of quaternions be close to 1
  // TODO(oalexan1): Make this a standalone function
  if (opt.quat_norm_weight > 0.0) {
    for (int icam = 0; icam < (int)crn.size(); icam++) {

      UsgsAstroLsSensorModel * ls_model
        = dynamic_cast<UsgsAstroLsSensorModel*>((csm_models[icam]->m_gm_model).get());
      UsgsAstroFrameSensorModel * frame_model
        = dynamic_cast<UsgsAstroFrameSensorModel*>((csm_models[icam]->m_gm_model).get());

      if (ls_model != NULL) {

        int numQuat = ls_model->m_quaternions.size() / NUM_QUAT_PARAMS;
        for (int iq = 0; iq < numQuat; iq++) {
          ceres::CostFunction* quat_norm_cost_function
            = weightedQuatNormError::Create(opt.quat_norm_weight);
          // We use no loss function, as the quaternions have no outliers
          ceres::LossFunction* quat_norm_loss_function = NULL;
          problem.AddResidualBlock(quat_norm_cost_function, quat_norm_loss_function,
                                  &ls_model->m_quaternions[iq * NUM_QUAT_PARAMS]);
          
          weight_per_residual.push_back(opt.quat_norm_weight); // 1 single residual
        }

      } else if (frame_model != NULL) {

        // There is one quaternion per camera, stored after the translation
        double * curr_params = &frame_params[icam * (NUM_XYZ_PARAMS + NUM_QUAT_PARAMS)];

        ceres::CostFunction* quat_norm_cost_function
          = weightedQuatNormError::Create(opt.quat_norm_weight);
        // We use no loss function, as the quaternions have no outliers
        ceres::LossFunction* quat_norm_loss_function = NULL;
        problem.AddResidualBlock(quat_norm_cost_function, quat_norm_loss_function,
                                &curr_params[NUM_XYZ_PARAMS]); // quat starts here
        
        weight_per_residual.push_back(opt.quat_norm_weight); // 1 single residual

      } else {
         vw::vw_throw(vw::ArgumentErr() << "Unknown camera model.\n");
      }
    }
  }
}

// Add roll / yaw constraints. For linescan, use the whole set of samples for given
// camera model. For frame cameras, use the trajectory of all cameras in the same orbital
// group as the current camera.
void addRollYawConstraint
   (Options                                         const& opt,
    vw::ba::CameraRelationNetwork<vw::ba::JFeature> const& crn,
    std::vector<asp::CsmModel*>                     const& csm_models,
    vw::cartography::GeoReference                   const& georef,
    // Outputs (append to residual)
    std::vector<double>                                  & frame_params,
    std::vector<double>                                  & weight_per_residual,
    ceres::Problem                                       & problem) {
  
  if (opt.roll_weight <= 0.0 && opt.yaw_weight <= 0.0)
     vw::vw_throw(vw::ArgumentErr() 
         << "addRollYawConstraint: The roll or yaw weight must be positive.\n");

  int num_cams = crn.size();

  // Frame cameras can be grouped by orbital portion. Ensure that all cameras
  // belong to a group.
  if (num_cams != int(opt.orbital_groups.size()))
    vw::vw_throw(vw::ArgumentErr() 
         << "addRollYawConstraint: Failed to add each input camera to an orbital group.\n");

  // Create the orbital trajectory for each group of frame cameras
  std::map<int, std::vector<double>> orbital_group_positions;
  std::map<int, std::vector<double>> orbital_group_quaternions;
  formPositionQuatVecPerGroup(opt.orbital_groups, csm_models, 
    orbital_group_positions, orbital_group_quaternions); // outputs

  for (int icam = 0; icam < num_cams; icam++) {

    UsgsAstroLsSensorModel * ls_model
      = dynamic_cast<UsgsAstroLsSensorModel*>((csm_models[icam]->m_gm_model).get());
    UsgsAstroFrameSensorModel * frame_model
      = dynamic_cast<UsgsAstroFrameSensorModel*>((csm_models[icam]->m_gm_model).get());

    if (ls_model != NULL) {
      // Linescan cameras. Use the full sequence of cameras in the model
      // to enforce the roll/yaw constraint for each camera in the sequence.
      int numQuat = ls_model->m_quaternions.size() / NUM_QUAT_PARAMS;
      for (int iq = 0; iq < numQuat; iq++) {
        ceres::CostFunction* roll_yaw_cost_function
          = weightedRollYawError::Create(ls_model->m_positions, 
                                    ls_model->m_quaternions,
                                    georef, iq,
                                    opt.roll_weight, opt.yaw_weight, 
                                    opt.initial_camera_constraint);

        // We use no loss function, as the quaternions have no outliers
        ceres::LossFunction* roll_yaw_loss_function = NULL;
        problem.AddResidualBlock(roll_yaw_cost_function, roll_yaw_loss_function,
                                &ls_model->m_quaternions[iq * NUM_QUAT_PARAMS]);
        // The recorded weight should not be 0 as we will divide by it
        weight_per_residual.push_back(opt.roll_weight || 1.0);
        weight_per_residual.push_back(opt.yaw_weight  || 1.0);
      } // end loop through quaternions for given camera
    
    } else if (frame_model != NULL) {
      // Frame cameras. Use the positions and quaternions of the cameras
      // in the same orbital group to enforce the roll/yaw constraint for
      // each camera in the group.
      auto it = opt.orbital_groups.find(icam);
      if (it == opt.orbital_groups.end())
        vw::vw_throw(vw::ArgumentErr() 
           << "addRollYawConstraint: Failed to find orbital group for camera.\n"); 
      int group_id = it->second;

      int index_in_group = indexInGroup(icam, opt.orbital_groups);
      std::vector<double> positions = orbital_group_positions[group_id];
      std::vector<double> quaternions = orbital_group_quaternions[group_id];
      ceres::CostFunction* roll_yaw_cost_function
        = weightedRollYawError::Create(positions, quaternions, 
                                   georef, index_in_group,
                                   opt.roll_weight, opt.yaw_weight, 
                                   opt.initial_camera_constraint);

      // We use no loss function, as the quaternions have no outliers
      ceres::LossFunction* roll_yaw_loss_function = NULL;

      // Note how we set the quaternions to be optimized from frame_params.
      // Above, we only cared for initial positions and quaternions.
      double * curr_params = &frame_params[icam * (NUM_XYZ_PARAMS + NUM_QUAT_PARAMS)];
      problem.AddResidualBlock(roll_yaw_cost_function, roll_yaw_loss_function,
                                &curr_params[NUM_XYZ_PARAMS]); // quat starts here

      // The recorded weight should not be 0 as we will divide by it
      weight_per_residual.push_back(opt.roll_weight || 1.0);
      weight_per_residual.push_back(opt.yaw_weight  || 1.0);
    } else {
      vw::vw_throw(vw::ArgumentErr() 
         << "addRollYawConstraint: Expecting CSM linescan or frame cameras.\n");
    }

  } // end loop through cameras

  return;
}

// Apply the input adjustments to the CSM cameras. Resample linescan models.
// Get pointers to the underlying CSM cameras, as need to manipulate
// those directly. This modifies camera_models in place.
void prepareCsmCameras(Options const& opt,
  std::vector<vw::CamPtr>      const& camera_models,
  std::vector<asp::CsmModel*>       & csm_models) {

  // Wipe the output
  csm_models.clear();
  
  for (size_t icam = 0; icam < camera_models.size(); icam++) {
    asp::CsmModel * csm_cam = asp::csm_model(camera_models[icam], opt.stereo_session);

    // Sanity check
    if (csm_cam == NULL)
      vw::vw_throw(vw::ArgumentErr() << "Expecting CSM cameras.\n");

    if (!opt.input_prefix.empty()) {
      std::string adjust_file
        = asp::bundle_adjust_file_name(opt.input_prefix, opt.image_files[icam],
                                      opt.camera_files[icam]);
      vw_out() << "Reading input adjustment: " << adjust_file << std::endl;
      // This modifies camera_models
      vw::camera::AdjustedCameraModel
        adj_cam(vw::camera::unadjusted_model(camera_models[icam]));
      adj_cam.read(adjust_file);
      vw::Matrix4x4 ecef_transform = adj_cam.ecef_transform();
      csm_cam->applyTransform(ecef_transform);
    }

    // Get the underlying linescan model or frame model
    UsgsAstroLsSensorModel * ls_model
      = dynamic_cast<UsgsAstroLsSensorModel*>((csm_cam->m_gm_model).get());
    UsgsAstroFrameSensorModel * frame_model
      = dynamic_cast<UsgsAstroFrameSensorModel*>((csm_cam->m_gm_model).get());

    if (ls_model == NULL && frame_model == NULL)
      vw_throw(ArgumentErr() << "Expecting the cameras to be of CSM linescan or frame type.\n");

    // Normalize quaternions. Later, the quaternions being optimized will
    // be kept close to being normalized.  This makes it easy to ensure
    // that quaternion interpolation gives good results, especially that
    // some quaternions may get optimized and some not.
    if (ls_model != NULL) {
      asp::normalizeQuaternions(ls_model);
      // The provided tabulated positions, velocities and quaternions may be too few,
      // so resample them with --num-lines-per-position and --num-lines-per-orientation,
      // if those are set.
      resampleModel(opt, ls_model);
    } else if (frame_model != NULL) {
      normalizeQuaternions(frame_model);
    } else {
      vw::vw_throw(vw::ArgumentErr() 
        << "Expecting the cameras to be of CSM linescan or frame type.\n");
    }

    csm_models.push_back(csm_cam);
  }
}

// Create structures for pixels, xyz, and weights, to be used in optimization
// TODO(oalexan1): Avoid using xyz_vec_ptr. If the vector gets resized, the pointers
// will be invalidated.
typedef vw::ba::CameraRelationNetwork<vw::ba::JFeature> CrnT;
typedef boost::shared_ptr<Vector3> Vec3Ptr;
void createProblemStructure(Options                      const& opt,
                            CrnT                         const& crn,
                            vw::ba::ControlNetwork       const& cnet, 
                            std::set<int>                const& outliers,
                            // Outputs
                            std::vector<double>               & tri_points_vec,
                            std::vector<std::vector<Vector2>> & pixel_vec,
                            std::vector<std::vector<Vec3Ptr>> & xyz_vec,
                            std::vector<std::vector<double*>> & xyz_vec_ptr,
                            std::vector<std::vector<double>>  & weight_vec,
                            std::vector<std::vector<int>>     & isAnchor_vec) {

  int num_cameras = opt.camera_models.size();

  pixel_vec.resize(num_cameras);
  xyz_vec.resize(num_cameras);
  xyz_vec_ptr.resize(num_cameras);
  weight_vec.resize(num_cameras);
  isAnchor_vec.resize(num_cameras);

  for (int icam = 0; icam < (int)crn.size(); icam++) {
    for (auto fiter = crn[icam].begin(); fiter != crn[icam].end(); fiter++) {
      
      // The index of the 3D point
      int ipt = (**fiter).m_point_id;
      
      if (outliers.find(ipt) != outliers.end())
        continue; // Skip outliers
      
      // The observed value for the projection of point with index ipt into
      // the camera with index icam.
      Vector2 observation = (**fiter).m_location;

      // Ideally this point projects back to the pixel observation.
      double * tri_point = &tri_points_vec[0] + ipt * NUM_XYZ_PARAMS;

      double weight = 1.0;
      
      pixel_vec[icam].push_back(observation);
      weight_vec[icam].push_back(weight);
      isAnchor_vec[icam].push_back(0);
      // It is bad logic to store pointers as below. What if tri_points_vec
      // later gets resized?
      xyz_vec_ptr[icam].push_back(tri_point); 
    }
  }

  return;
}

// Put the triangulated points in a vector. Update the cnet from the DEM,
// if we have one.
void formTriVec(std::vector<Vector3> const& dem_xyz_vec,
                bool have_dem,
                // Outputs
                ba::ControlNetwork  & cnet,
                std::vector<double> & tri_points_vec) {

  int num_tri_points = cnet.size();
  if (num_tri_points == 0)
    vw::vw_throw(ArgumentErr() << "No triangulated ground points were found.\n"); 

  tri_points_vec.resize(num_tri_points*NUM_XYZ_PARAMS, 0.0);

  for (int ipt = 0; ipt < num_tri_points; ipt++) {
    // We overwrite the triangulated point when we have an input DEM.
    // It is instructive to examine the pointmap residual file to see
    // what effect that has on residuals.  This point will likely try
    // to move back somewhat to its triangulated position during
    // optimization, depending on the strength of the weight which
    // tries to keep it back in place.
    Vector3 tri_point = cnet[ipt].position();
    if (have_dem && dem_xyz_vec.at(ipt) != Vector3(0, 0, 0)) {
      tri_point = dem_xyz_vec.at(ipt);

      // Update in the cnet too
      cnet[ipt].set_position(Vector3(tri_point[0], tri_point[1], tri_point[2]));
      
      // Ensure we can track it later
      cnet[ipt].set_type(vw::ba::ControlPoint::PointFromDem); 
    }
    
    for (int q = 0; q < NUM_XYZ_PARAMS; q++)
      tri_points_vec[ipt*NUM_XYZ_PARAMS + q] = tri_point[q];
  }
  return;
}

void run_jitter_solve(int argc, char* argv[]) {

  // Parse arguments and perform validation
  Options opt;
  handle_arguments(argc, argv, opt);

  bool approximate_pinhole_intrinsics = false;
  asp::load_cameras(opt.image_files, opt.camera_files, opt.out_prefix, opt,  
                    approximate_pinhole_intrinsics,  
                    // Outputs
                    opt.stereo_session,  // may change
                    opt.single_threaded_cameras,  
                    opt.camera_models);

  // Find the datum
  vw::cartography::Datum datum;
  asp::datum_from_cameras(opt.image_files, opt.camera_files,  
                          opt.stereo_session,  // may change
                          // Outputs
                          datum);
  
  // Apply the input adjustments to the cameras. Resample linescan models.
  // Get pointers to the underlying CSM cameras, as need to manipulate
  // those directly.
  std::vector<asp::CsmModel*> csm_models;
  prepareCsmCameras(opt, opt.camera_models, csm_models);
  
  // Make a list of all the image pairs to find matches for. Some quantities
  // below are not needed but are part of the API.
  bool got_est_cam_positions = false;
  double position_filter_dist = -1.0;
  std::vector<vw::Vector3> estimated_camera_gcc;
  bool have_overlap_list = false;
  std::set<std::pair<std::string, std::string>> overlap_list;
  std::vector<std::pair<int,int>> all_pairs;
  asp::determine_image_pairs(// Inputs
                             opt.overlap_limit, opt.match_first_to_last,  
                             opt.image_files, 
                             got_est_cam_positions, position_filter_dist,
                             estimated_camera_gcc, have_overlap_list, overlap_list,
                             // Output
                             all_pairs);

  // Load match files
  std::map<std::pair<int, int>, std::string> match_files;
  for (size_t k = 0; k < all_pairs.size(); k++) {
    int i = all_pairs[k].first;
    int j = all_pairs[k].second;
    std::string const& image1_path  = opt.image_files[i];  // alias
    std::string const& image2_path  = opt.image_files[j];  // alias
    std::string const& camera1_path = opt.camera_files[i]; // alias
    std::string const& camera2_path = opt.camera_files[j]; // alias

    // List existing match files
    std::string prefix = asp::match_file_prefix(opt.clean_match_files_prefix,
                                                opt.match_files_prefix,  
                                                opt.out_prefix);
    std::set<std::string> existing_files;
    asp::listExistingMatchFiles(prefix, existing_files);

      // Load match files from a different source
    std::string match_file 
      = asp::match_filename(opt.clean_match_files_prefix, opt.match_files_prefix,  
                            opt.out_prefix, image1_path, image2_path);

    // The external match file does not exist, don't try to load it
    if (existing_files.find(match_file) == existing_files.end())
      continue;
    
    match_files[std::make_pair(i, j)] = match_file;
  }

  // Build control network and perform triangulation with adjusted input cameras
  ba::ControlNetwork cnet("jitter_solve");
  bool triangulate_control_points = true;
  double forced_triangulation_distance = -1.0;
  bool success = vw::ba::build_control_network(triangulate_control_points,
                                               cnet, // output
                                               opt.camera_models, opt.image_files,
                                               match_files, opt.min_matches,
                                               opt.min_triangulation_angle*(M_PI/180.0),
                                               forced_triangulation_distance,
                                               opt.max_pairwise_matches);
  if (!success)
    vw::vw_throw(vw::ArgumentErr()
             << "Failed to build a control network. Check the bundle adjustment directory "
             << "for clean matches. Or, consider removing all .vwip and "
             << ".match files and increasing the number of interest points "
             << "using --ip-per-image or --ip-per-tile, or decreasing --min-matches, "
             << "and then re-running bundle adjustment.\n");

  // TODO(oalexan1): Is it possible to avoid using CRNs?
  vw::ba::CameraRelationNetwork<vw::ba::JFeature> crn;
  crn.from_cnet(cnet);
  
  if ((int)crn.size() != opt.camera_models.size()) 
    vw_throw(ArgumentErr() << "Book-keeping error, the size of CameraRelationNetwork "
             << "must equal the number of images.\n");

  // Flag as outliers points with initial reprojection error bigger than
  // a certain amount. This assumes that the input cameras are very accurate.
  std::set<int> outliers;
  flag_initial_outliers(cnet, crn, opt.camera_models, opt.max_init_reproj_error,  
                        // Output
                        outliers);
  vw_out() << "Removed " << outliers.size() 
    << " outliers based on initial reprojection error.\n";
  
  bool have_dem = (!opt.heights_from_dem.empty() || !opt.ref_dem.empty());

  // Create anchor xyz with the help of a DEM in two ways.
  // TODO(oalexan1): Study how to best pass the DEM to avoid the code
  // below not being slow. It is not clear if the DEM tiles are cached
  // when passing around an ImageViewRef.
  std::vector<Vector3> dem_xyz_vec;
  vw::cartography::GeoReference dem_georef, anchor_georef;
  ImageViewRef<PixelMask<double>> interp_dem, interp_anchor_dem;
  if (opt.heights_from_dem != "") {
    asp::create_interp_dem(opt.heights_from_dem, dem_georef, interp_dem);
    asp::update_point_height_from_dem(cnet, outliers, dem_georef, interp_dem,  
                                      // Output
                                      dem_xyz_vec);
  } else if (opt.ref_dem != "") {
    asp::create_interp_dem(opt.ref_dem, dem_georef, interp_dem);
    asp::calc_avg_intersection_with_dem(cnet, crn, outliers, opt.camera_models,
                                        dem_georef, interp_dem,
                                        // Output
                                        dem_xyz_vec);
  }
  
  if (opt.anchor_dem != "")
    asp::create_interp_dem(opt.anchor_dem, anchor_georef, interp_anchor_dem);
  
  // Handle the roll/yaw constraint DEM. We already checked that one of thse cases should work
  vw::cartography::GeoReference roll_yaw_georef;
  if (opt.roll_weight > 0 || opt.yaw_weight > 0) {
    if (opt.heights_from_dem != "" || opt.ref_dem != "") {
      roll_yaw_georef = dem_georef;
      vw::vw_out() << "Using the DEM from --heights-from-dem or --reference-dem "
                   << "for the roll/yaw constraint.\n";
    } else if (opt.anchor_dem != "") {
      roll_yaw_georef = anchor_georef;
      vw::vw_out() << "Using the DEM from --anchor-dem for the roll/yaw constraint.\n";
    }
  } 

  int num_cameras = opt.camera_models.size();
  if (num_cameras < 2)
    vw_throw(ArgumentErr() << "Expecting at least two input cameras.\n");

  // If some of the input cameras are frame, need to store position and
  // quaternion variables for them outside the camera model, as these are
  // private for UsgsAstroFrameCameraModel, unlike for UsgsAstroLsSensorModel.
  // It is easier to just allocate the space for all cameras, even if it may go
  // unused mostly or at all.
  std::vector<double> frame_params;
  initFrameCameraParams(csm_models, frame_params);

  // Put the triangulated points in a vector. Update the cnet from the DEM,
  // if we have one.
  std::vector<double> tri_points_vec;
  formTriVec(dem_xyz_vec, have_dem, 
    cnet, tri_points_vec); // outputs
  
  // Create structures for pixels, xyz, and weights, to be used in optimization
  std::vector<std::vector<Vector2>> pixel_vec;
  std::vector<std::vector<boost::shared_ptr<Vector3>>> xyz_vec;
  std::vector<std::vector<double*>> xyz_vec_ptr;
  std::vector<std::vector<double>> weight_vec;
  std::vector<std::vector<int>> isAnchor_vec;
  createProblemStructure(opt, crn, cnet, outliers,
                         // Outputs
                         tri_points_vec, pixel_vec, xyz_vec, xyz_vec_ptr,
                         weight_vec, isAnchor_vec);

  // Find anchor points and append to pixel_vec, weight_vec, etc.
  if (opt.num_anchor_points > 0 && opt.anchor_weight > 0)
    calcAnchorPoints(opt, interp_anchor_dem, anchor_georef, csm_models,  
                     // Append to these
                     pixel_vec, xyz_vec, xyz_vec_ptr, weight_vec, isAnchor_vec);

  // Need this in order to undo the multiplication by weight before saving the residuals
  std::vector<double> weight_per_residual;

  // The problem to solve
  ceres::Problem problem;
  
  // Add reprojection errors
  addReprojectionErrors(opt, crn, pixel_vec, xyz_vec, xyz_vec_ptr, weight_vec,
                        isAnchor_vec, csm_models,
                        // Outputs
                        frame_params, weight_per_residual, problem);
 
  // Add the DEM constraint. We check earlier that only one
  // of the two options below can be set at a time.
  if (have_dem)
    addDemConstraint(opt, xyz_vec, xyz_vec_ptr, dem_xyz_vec, outliers, cnet,  
                     // Outputs
                     tri_points_vec, 
                     weight_per_residual,  // append
                     problem);

  // Add the constraint to keep triangulated points close to initial values
  // This does not need a DEM or alignment.
  // This must happen after any DEM-based constraint is set, and won't
  // apply to tri points already constrained by the DEM (so it will
  // work only where the DEM is missing).
  if (opt.tri_weight > 0) 
    addTriConstraint(opt, outliers, cnet,  
                     // Outputs
                     tri_points_vec,  
                     weight_per_residual,  // append
                     problem);

  // Add constraints to keep quat norm close to 1, and make rotations and translations
  // not change too much
  addQuatNormRotationTranslationConstraints(opt, outliers, crn, csm_models,  
                                            // Outputs
                                            frame_params,
                                            tri_points_vec,  
                                            weight_per_residual,  // append
                                            problem);

  if (opt.roll_weight > 0 || opt.yaw_weight > 0)
    addRollYawConstraint(opt, crn, csm_models, roll_yaw_georef,
                        frame_params, weight_per_residual, problem); // outputs

  // Save residuals before optimization
  std::string residual_prefix = opt.out_prefix + "-initial_residuals";
  save_residuals(residual_prefix, problem, opt, cnet, crn, have_dem, datum,
                 tri_points_vec, dem_xyz_vec, outliers, weight_per_residual,
                 // These are needed for anchor points
                 pixel_vec, xyz_vec, xyz_vec_ptr, weight_vec, isAnchor_vec);
  
  // Set up the problem
  ceres::Solver::Options options;
  options.gradient_tolerance  = 1e-16;
  options.function_tolerance  = 1e-16;
  options.parameter_tolerance = opt.parameter_tolerance; // default is 1e-12
  options.max_num_iterations                = opt.num_iterations;
  options.max_num_consecutive_invalid_steps = std::max(20, opt.num_iterations/5); // try hard
  options.minimizer_progress_to_stdout      = true;
  if (opt.single_threaded_cameras)
    options.num_threads = 1;
  else
    options.num_threads = opt.num_threads;
  // This is supposed to help with speed in a certain size range
  options.linear_solver_type = ceres::SPARSE_SCHUR;
  options.use_explicit_schur_complement = true; 
  options.linear_solver_type  = ceres::ITERATIVE_SCHUR;
  options.preconditioner_type = ceres::SCHUR_JACOBI;
  options.use_explicit_schur_complement = false; // Only matters with ITERATIVE_SCHUR
  
  // Solve the problem
  vw_out() << "Starting the Ceres optimizer." << std::endl;
  ceres::Solver::Summary summary;
  ceres::Solve(options, &problem, &summary);
  vw_out() << summary.FullReport() << "\n";
  if (summary.termination_type == ceres::NO_CONVERGENCE) 
    vw_out() << "Found a valid solution, but did not reach the actual minimum.\n";

  // With the problem solved, update camera_models based on frame_params
  // (applies only to frame cameras, if any)
  updateFrameCameras(csm_models, frame_params);  

  // Save residuals after optimization
  // TODO(oalexan1): Add here the anchor residuals
  residual_prefix = opt.out_prefix + "-final_residuals";
  save_residuals(residual_prefix, problem, opt, cnet, crn, have_dem, datum,
                 tri_points_vec, dem_xyz_vec, outliers, weight_per_residual,
                 // These are needed for anchor points
                 pixel_vec, xyz_vec, xyz_vec_ptr, weight_vec, isAnchor_vec);

  // TODO(oalexan1): Make this a function
  // Save the optimized model states. Note that we optimized directly the camera
  // model states, so there's no need to update them from some optimization
  // workspace.
  for (size_t icam = 0; icam < opt.camera_models.size(); icam++) {
    std::string adjustFile = asp::bundle_adjust_file_name(opt.out_prefix,
                                                          opt.image_files[icam],
                                                          opt.camera_files[icam]);
    std::string csmFile = asp::csmStateFile(adjustFile);
    asp::CsmModel * csm_cam = asp::csm_model(opt.camera_models[icam], opt.stereo_session);
    csm_cam->saveState(csmFile);
  }
  
  return;
}

} // end namespace asp
  
int main(int argc, char* argv[]) {

  try {
    xercesc::XMLPlatformUtils::Initialize();
    
    asp::run_jitter_solve(argc, argv);
    
    xercesc::XMLPlatformUtils::Terminate();
  } ASP_STANDARD_CATCHES;

  return 0;
}
