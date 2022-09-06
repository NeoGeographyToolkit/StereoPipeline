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

// TODO(oalexan1): Add two passes and outlier filtering. For now
// try to use clean matches.

#include <vw/BundleAdjustment/ControlNetwork.h>
#include <vw/BundleAdjustment/ControlNetworkLoader.h>
#include <vw/Core/Stopwatch.h>
#include <vw/Cartography/CameraBBox.h>

#include <asp/Sessions/StereoSessionFactory.h>
#include <asp/Sessions/CameraUtils.h>
#include <asp/Camera/CsmModel.h>
#include <asp/Camera/BundleAdjustCamera.h>
#include <asp/Core/Macros.h>
#include <asp/Core/StereoSettings.h>
#include <asp/Core/BundleAdjustUtils.h>
#include <asp/Core/IpMatchingAlgs.h> // Lightweight header for matching algorithms

#include <usgscsm/UsgsAstroLsSensorModel.h>
#include <usgscsm/Utilities.h>

#include <xercesc/util/PlatformUtils.hpp>

#include <ceres/ceres.h>
#include <ceres/loss_function.h>

namespace po = boost::program_options;
namespace fs = boost::filesystem;

using namespace vw;
using namespace vw::math;


namespace asp {

const int NUM_XYZ_PARAMS  = 3;
const int NUM_QUAT_PARAMS = 4;
const int PIXEL_SIZE      = 2;

const double g_big_pixel_value = 1000.0;  // don't make this too big

// An error function minimizing the error of projecting an xyz point
// into a given camera pixel. The variables of optimization are a
// portion of the position and quaternion variables affected by this.
  
struct pixelReprojectionError {
  pixelReprojectionError(vw::Vector2 const& observation, double weight,
                         UsgsAstroLsSensorModel* ls_model,
                         int begQuatIndex, int endQuatIndex, int begPosIndex, int endPosIndex):
    m_observation(observation), m_weight(weight),
    m_begQuatIndex(begQuatIndex), m_endQuatIndex(endQuatIndex),
    m_begPosIndex(begPosIndex),   m_endPosIndex(endPosIndex),
    m_ls_model(ls_model){}

  // Call to work with ceres::DynamicCostFunction.
  bool operator()(double const * const * parameters, double * residuals) const {

    try {
      // Make a copy of the model, as we will update quaternion and position values
      // that are being modified now. This may be expensive.
      UsgsAstroLsSensorModel cam = *m_ls_model;

      // Update the relevant quaternions in the local copy
      int shift = 0;
      for (int qi = m_begQuatIndex; qi < m_endQuatIndex; qi++) {
        for (int coord = 0; coord < NUM_QUAT_PARAMS; coord++) {
          cam.m_quaternions[NUM_QUAT_PARAMS * qi + coord]
            = parameters[qi + shift - m_begQuatIndex][coord];
        }
      }

      // Same for the positions. Note how we move forward in the parameters array,
      // as this is after the quaternions
      shift += (m_endQuatIndex - m_begQuatIndex);
      for (int pi = m_begPosIndex; pi < m_endPosIndex; pi++) {
        for (int coord = 0; coord < NUM_XYZ_PARAMS; coord++) {
          cam.m_positions[NUM_XYZ_PARAMS * pi + coord]
            = parameters[pi + shift - m_begPosIndex][coord];
        }
      }

      // Move forward in the array of parameters, then recover the triangulated point
      shift += (m_endPosIndex - m_begPosIndex);
      csm::EcefCoord P;
      P.x = parameters[shift][0];
      P.y = parameters[shift][1];
      P.z = parameters[shift][2];

      // Project in the camera with high precision
      double desired_precision = 1e-12;
      csm::ImageCoord imagePt = cam.groundToImage(P, desired_precision);

      // Convert to what ASP expects
      vw::Vector2 pix;
      asp::fromCsmPixel(pix, imagePt);

      residuals[0] = m_weight*(pix[0] - m_observation[0]);
      residuals[1] = m_weight*(pix[1] - m_observation[1]);
      
    } catch (std::exception const& e) {
      residuals[0] = g_big_pixel_value;
      residuals[1] = g_big_pixel_value;
      return true; // accept the solution anyway
    }
    return true;
  }

  // Factory to hide the construction of the CostFunction object from the client code.
  static ceres::CostFunction* Create(vw::Vector2 const& observation, double weight,
                                     UsgsAstroLsSensorModel* ls_model,
                                     int begQuatIndex, int endQuatIndex,
                                     int begPosIndex, int endPosIndex){

    // TODO(oalexan1): Try using here the analytical cost function
    ceres::DynamicNumericDiffCostFunction<pixelReprojectionError>* cost_function =
      new ceres::DynamicNumericDiffCostFunction<pixelReprojectionError>
      (new pixelReprojectionError(observation, weight, ls_model,
                                  begQuatIndex, endQuatIndex,
                                  begPosIndex, endPosIndex));

    // The residual size is always the same.
    cost_function->SetNumResiduals(PIXEL_SIZE);

    // Add a parameter block for each quaternion and each position
    for (int it = begQuatIndex; it < endQuatIndex; it++)
      cost_function->AddParameterBlock(NUM_QUAT_PARAMS);
    for (int it = begPosIndex; it < endPosIndex; it++)
      cost_function->AddParameterBlock(NUM_XYZ_PARAMS);

    // Add a parameter block for the xyz point
    cost_function->AddParameterBlock(NUM_XYZ_PARAMS);
    
    return cost_function;
  }

private:
  Vector2 m_observation; // The pixel observation for this camera/point pair
  double m_weight;
  UsgsAstroLsSensorModel* m_ls_model;
  int m_begQuatIndex, m_endQuatIndex;
  int m_begPosIndex, m_endPosIndex;
}; // End class pixelReprojectionError

/// A ceres cost function. The residual is the difference between the
/// observed 3D point and the current (floating) 3D point, multiplied by given weight.
struct weightedXyzError {
  weightedXyzError(Vector3 const& observation, double weight):
    m_observation(observation), m_weight(weight){}

  template <typename T>
  bool operator()(const T* point, T* residuals) const {
    for (size_t p = 0; p < m_observation.size(); p++)
      residuals[p] = m_weight * (point[p] - m_observation[p]);

    return true;
  }

  // Factory to hide the construction of the CostFunction object from
  // the client code.
  static ceres::CostFunction* Create(Vector3 const& observation, double const& weight){
    return (new ceres::AutoDiffCostFunction<weightedXyzError, 3, 3>
            (new weightedXyzError(observation, weight)));
  }

  Vector3 m_observation;
  double  m_weight;
};

/// A Ceres cost function. The residual is the difference between the
/// initial quaternion and optimized quaternion, multiplied by given weight.
struct weightedRotationError {
  weightedRotationError(const double * init_quat, double weight):
    m_weight(weight) {

    // Make a copy, as later the value at the pointer will change
    m_init_quat.resize(NUM_QUAT_PARAMS);
    for (int it = 0; it < NUM_QUAT_PARAMS; it++)
      m_init_quat[it] = init_quat[it];
  }

  template <typename T>
  bool operator()(const T* quat, T* residuals) const {
    for (size_t p = 0; p < m_init_quat.size(); p++)
      residuals[p] = m_weight * (quat[p] - m_init_quat[p]);

    return true;
  }

  // Factory to hide the construction of the CostFunction object from
  // the client code.
  static ceres::CostFunction* Create(const double * init_quat, double weight){
    return (new ceres::AutoDiffCostFunction<weightedRotationError, NUM_QUAT_PARAMS, NUM_QUAT_PARAMS>
            (new weightedRotationError(init_quat, weight)));
  }

  std::vector<double> m_init_quat;
  double  m_weight;
};

/// A Ceres cost function. The residual is the difference between the
/// initial position and optimized position, multiplied by given weight.
struct weightedTranslationError {
  weightedTranslationError(const double * init_position, double weight):
    m_weight(weight) {

    // Make a copy, as later the value at the pointer will change
    m_init_position.resize(NUM_XYZ_PARAMS);
    for (int it = 0; it < NUM_XYZ_PARAMS; it++)
      m_init_position[it] = init_position[it];
  }

  template <typename T>
  bool operator()(const T* position, T* residuals) const {
    for (size_t p = 0; p < m_init_position.size(); p++)
      residuals[p] = m_weight * (position[p] - m_init_position[p]);

    return true;
  }

  // Factory to hide the construction of the CostFunction object from
  // the client code.
  static ceres::CostFunction* Create(const double * init_position, double weight){
    return (new ceres::AutoDiffCostFunction
            <weightedTranslationError, NUM_XYZ_PARAMS, NUM_XYZ_PARAMS>
            (new weightedTranslationError(init_position, weight)));
  }

  std::vector<double> m_init_position;
  double  m_weight;
};


/// A Ceres cost function. The residual is the weighted difference between 1 and
/// norm of quaternion.
struct weightedQuatNormError {
  weightedQuatNormError(double weight):
    m_weight(weight) {}

  template <typename T>
  bool operator()(const T* quat, T* residuals) const {
    residuals[0] = T(0.0);
    for (size_t p = 0; p < NUM_QUAT_PARAMS; p++)
      residuals[0] += quat[p] * quat[p];

    residuals[0] = m_weight * (residuals[0] - 1.0);
    
    return true;
  }

  // Factory to hide the construction of the CostFunction object from
  // the client code.
  static ceres::CostFunction* Create(double weight) {
    return (new ceres::AutoDiffCostFunction<weightedQuatNormError, 1, NUM_QUAT_PARAMS>
            (new weightedQuatNormError(weight)));
  }

  double  m_weight;
};

struct Options: public asp::BaBaseOptions {
  double quat_norm_weight, anchor_weight;
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
     "Prefix to read initial adjustments from, if available, as written by bundle_adjust.")
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
     "bundle-adjusted by now, this value should be small.")
    ("robust-threshold", po::value(&opt.robust_threshold)->default_value(0.5),
     "Set the threshold for the Cauchy robust cost function. Increasing this makes "
     "the solver focus harder on the larger errors.")
    ("parameter-tolerance",  po::value(&opt.parameter_tolerance)->default_value(1e-12),
     "Stop when the relative error in the variables being optimized is less than this.")
    ("num-iterations",       po::value(&opt.num_iterations)->default_value(500),
     "Set the maximum number of iterations.")
    ("heights-from-dem",   po::value(&opt.heights_from_dem)->default_value(""),
     "If the cameras have already been bundle-adjusted and aligned to a known high-quality DEM, "
     "in the triangulated xyz points replace the heights with the ones from this DEM before "
     "optimizing them.")
    ("heights-from-dem-weight", po::value(&opt.heights_from_dem_weight)->default_value(1.0),
     "How much weight to give to keep the triangulated points close to the DEM if specified via "
     "--heights-from-dem.")
    ("heights-from-dem-robust-threshold",
     po::value(&opt.heights_from_dem_robust_threshold)->default_value(0.5),
     "If positive, this is the robust threshold to use keep the triangulated points "
     "close to the DEM if specified via --heights-from-dem. This is applied after the "
     "point differences are multiplied by --heights-from-dem-weight. It should help with "
     "attenuating large height difference outliers.")
    ("reference-dem",  po::value(&opt.ref_dem)->default_value(""),
     "If specified, constrain every ground point where rays from matching pixels intersect "
     "to be not too far from the average of intersections of those rays with this DEM.")
    ("reference-dem-weight", po::value(&opt.ref_dem_weight)->default_value(1.0),
     "Multiply the xyz differences for the --reference-dem option by this weight.")
    ("reference-dem-robust-threshold", po::value(&opt.ref_dem_robust_threshold)->default_value(0.5),
     "Use this robust threshold for the weighted xyz differences.")
    ("rotation-weight", po::value(&opt.rotation_weight)->default_value(1.0),
     "How much weight to give to keep cameras from rotating from initial values.")
    ("translation-weight", po::value(&opt.translation_weight)->default_value(1.0),
     "How much weight to give to keep cameras from moving from initial values.")
    ("quat-norm-weight", po::value(&opt.quat_norm_weight)->default_value(1.0),
     "How much weight to give to the constraint that quaternion norm must be 1.")
    ("anchor-weight", po::value(&opt.anchor_weight)->default_value(1.0),
     "How much weight to give to each anchor point.")
    ("ip-side-filter-percent",  po::value(&opt.ip_edge_buffer_percent)->default_value(-1.0),
     "Remove matched IPs this percentage from the image left/right sides.");
  
    general_options.add(vw::GdalWriteOptionsDescription(opt));

  // TODO(oalexan1): Replace this param here and in bundle_adjust with
  // opt.remove_outliers_params, then make this an option in jitter_solve.
  opt.remove_outliers_by_disp_params = Vector2(0.0, 0.0);

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
    vw_throw(ArgumentErr() << "Must specify the output prefix.\n");

  // Create the output directory
  vw::create_out_dir(opt.out_prefix);

  std::vector<std::string> inputs = opt.image_files;
  bool ensure_equal_sizes = true;
  asp::separate_images_from_cameras(inputs,
                                    opt.image_files, opt.camera_files, // outputs
                                    ensure_equal_sizes); 

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
    
  if (opt.quat_norm_weight < 0)
    vw_throw(ArgumentErr() << "Quaternion norm weight must be positive.\n");

  if (opt.anchor_weight < 0)
    vw_throw(ArgumentErr() << "Anchor weight must be positive.\n");

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
  
  // Open the output file and write the header
  // TODO(oalexan1): Make this a function. See if it is possible to integrate
  // it with the analogous bundle_adjust function.
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
    file << llh[0] <<", "<< llh[1] <<", "<< llh[2] <<", "<< mean_pixel_residual_norm[ipt] <<", "
         << pixel_residual_count[ipt] << comment << std::endl;
  }
  file.close();
}

// TODO(oalexan1): Add here residuals for anchor points, and also for 
// xyz discrepancy to DEM
void save_residuals(std::string const& residual_prefix,
                    ceres::Problem & problem, Options const& opt,
                    vw::ba::ControlNetwork const& cnet,
                    vw::ba::CameraRelationNetwork<vw::ba::JFeature> const& crn,
                    bool have_dem, vw::cartography::Datum const& datum,
                    std::vector<double> const& tri_points_vec,
                    std::vector<Vector3> const& dem_xyz_vec,
                    std::set<int> const& outliers,
                    std::vector<double> const& weight_per_residual) {

  // Compute the residuals before optimization
  std::vector<double> residuals;
  compute_residuals(opt, problem, residuals);
  if (residuals.size() != weight_per_residual.size()) 
    vw_throw(ArgumentErr() << "There must be as many residuals as weights for them.\n");

  //  Find the mean of all residuals corresponding to the same xyz point
  int num_tri_points = cnet.size();
  std::vector<double> mean_pixel_residual_norm(num_tri_points, 0.0);
  std::vector<int>    pixel_residual_count(num_tri_points, 0);
  std::vector<double> xyz_residual_norm;
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

  write_per_xyz_pixel_residuals(cnet, residual_prefix, datum, outliers,  
                                tri_points_vec, mean_pixel_residual_norm,  
                                pixel_residual_count);
  
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
  
  // Apply the input adjustments to the CSM cameras. Get pointers to the underlying
  // linescan cameras, as need to manipulate those directly.
  std::vector<UsgsAstroLsSensorModel*> ls_cams;
  
  for (size_t it = 0; it < opt.camera_models.size(); it++) {
    vw::camera::CameraModel * base_cam = vw::camera::unadjusted_model(opt.camera_models[it]).get();
    asp::CsmModel * csm_cam = dynamic_cast<asp::CsmModel*>(base_cam);
    if (csm_cam == NULL)
      vw_throw(ArgumentErr() << "Expecting the cameras to be of CSM type.\n");

    if (!opt.input_prefix.empty()) {
      std::string adjust_file
        = asp::bundle_adjust_file_name(opt.input_prefix, opt.image_files[it],
                                       opt.camera_files[it]);
      vw_out() << "Reading input adjustment: " << adjust_file << std::endl;
      // This modifies opt.camera_models
      vw::camera::AdjustedCameraModel adj_cam(vw::camera::unadjusted_model(opt.camera_models[it]));
      adj_cam.read(adjust_file);
      vw::Matrix4x4 ecef_transform = adj_cam.ecef_transform();
      csm_cam->applyTransform(ecef_transform);
    }
    
    UsgsAstroLsSensorModel * ls_cam
      = dynamic_cast<UsgsAstroLsSensorModel*>((csm_cam->m_csm_model).get());
    if (ls_cam == NULL)
      vw_throw(ArgumentErr() << "Expecting the cameras to be of CSM linescan type.\n");
    ls_cams.push_back(ls_cam);
  }

  // Quantities that are not needed but are part of the API below
  bool got_est_cam_positions = false;
  double position_filter_dist = -1.0;
  std::vector<vw::Vector3> estimated_camera_gcc;
  std::set<std::pair<std::string, std::string>> overlap_list;

  // Make a list of all the image pairs to find matches for
  std::vector<std::pair<int,int>> all_pairs;
  asp::determine_image_pairs(// Inputs
                             opt.overlap_limit, opt.match_first_to_last,  
                             opt.image_files, 
                             got_est_cam_positions, position_filter_dist,
                             estimated_camera_gcc, overlap_list,
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

    // Load match files from a different source
    bool allow_missing_match_file = true; 
    std::string match_filename 
      = asp::match_filename(opt.clean_match_files_prefix, opt.match_files_prefix,  
                            opt.out_prefix, image1_path, image2_path, allow_missing_match_file);
    match_files[std::make_pair(i, j)] = match_filename;
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
    vw_throw(ArgumentErr()
             << "Failed to build a control network. Consider removing "
             << "all .vwip and .match files and increasing "
             << "the number of interest points per tile using "
             << "--ip-per-tile, or decreasing --min-matches.\n");

  // TODO(oalexan1): Is it possible to avoid using CRNs?
  vw::ba::CameraRelationNetwork<vw::ba::JFeature> crn;
  crn.read_controlnetwork(cnet);
  
  if ((int)crn.size() != opt.camera_models.size()) 
    vw_throw(ArgumentErr() << "Book-keeping error, the size of CameraRelationNetwork "
             << "must equal the number of images.\n");

  // Flag as outliers points with initial reprojection error bigger than
  // a certain amount. This assumes that the input cameras are very accurate.
  std::set<int> outliers;
  flag_initial_outliers(cnet, crn, opt.camera_models, opt.max_init_reproj_error,  
                        // Output
                        outliers);
  vw_out() << "Removed " << outliers.size() << " outliers based on initial reprojection error.\n";
  
  bool have_dem = (!opt.heights_from_dem.empty() || !opt.ref_dem.empty());
  
  // Create anchor xyz with the help of a DEM in two ways.
  // TODO(oalexan1): Study how to best pass the DEM to avoid the code
  // below not being slow. It is not clear if the DEM tiles are cached
  // when passing around an ImageViewRef.
  std::vector<Vector3> dem_xyz_vec;
  vw::cartography::GeoReference dem_georef;
  ImageViewRef<PixelMask<double>> interp_dem;
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
  
  int num_cameras = opt.camera_models.size();
  if (num_cameras < 2)
    vw_throw(ArgumentErr() << "Expecting at least two input cameras.\n");
    
  // Put the triangulated points in a vector
  int num_tri_points = cnet.size();
  if (num_tri_points == 0)
   vw_throw(ArgumentErr() << "No triangulated ground points were found.\n"); 
  std::vector<double> tri_points_vec(num_tri_points*NUM_XYZ_PARAMS, 0.0);
  for (int ipt = 0; ipt < num_tri_points; ipt++) {

    Vector3 tri_point = cnet[ipt].position();
    
    // We overwrite the triangulated point when we have an input DEM.
    // It is instructive to examine the pointmap residual file to see
    // what effect that has on residuals.  This point will likely try
    // to move back somewhat to its triangulated position during
    // optimization, depending on the strength of the weight which
    // tries to keep it back in place.
    if (have_dem && dem_xyz_vec.at(ipt) != Vector3(0, 0, 0)) 
      tri_point = dem_xyz_vec.at(ipt);
    
    for (int q = 0; q < NUM_XYZ_PARAMS; q++)
      tri_points_vec[ipt*NUM_XYZ_PARAMS + q] = tri_point[q];
  }

  // Set up the cost function
  // TODO(oalexan1): Make this into a function
  ceres::Problem problem;

  // Need this in order to undo the multiplication by weight before saving the residuals
  std::vector<double> weight_per_residual;

  std::map<int, std::vector<Vector2>> pixel_vec;
  std::map<int, std::vector<boost::shared_ptr<Vector3>>> xyz_vec;
  std::map<int, std::vector<double*>> xyz_vec_ptr;
  std::map<int, std::vector<double>> weight_vec;
  std::map<int, std::vector<int>> isAnchor_vec;
  
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
      xyz_vec_ptr[icam].push_back(tri_point);
    }
  }

  // TODO(oalexan1): Make this a function
  if (opt.anchor_weight > 0) {
    
    for (int icam = 0; icam < (int)crn.size(); icam++) {

      int numLines   = ls_cams[icam]->m_nLines;
      int numSamples = ls_cams[icam]->m_nSamples;
      int numQuat    = ls_cams[icam]->m_quaternions.size() / NUM_QUAT_PARAMS;
      double r       = double(numLines) / numQuat;

      int numAnchorPts = 0;
      for (double line = 0; line < numLines; line += r/3.0) {
        for (double sample = 0; sample < numSamples; sample += numSamples/3.0) {

          Vector2 pix(sample, line);
          Vector3 xyz_guess(0, 0, 0);
          
          bool treat_nodata_as_zero = false;
          bool has_intersection = false;
          double height_error_tol = 0.001; // 1 mm should be enough
          double max_abs_tol      = 1e-14; // abs cost fun change b/w iterations
          double max_rel_tol      = 1e-14;
          int num_max_iter        = 25;   // Using many iterations can be very slow
          
          Vector3 dem_xyz = vw::cartography::camera_pixel_to_dem_xyz
            (opt.camera_models[icam]->camera_center(pix),
             opt.camera_models[icam]->pixel_to_vector(pix),
             interp_dem, dem_georef, treat_nodata_as_zero, has_intersection,
             height_error_tol, max_abs_tol, max_rel_tol, num_max_iter, xyz_guess);

          if (!has_intersection) 
            continue;
          
          Vector2 pix_out = opt.camera_models[icam]->point_to_pixel(dem_xyz);
          if (norm_2(pix - pix_out) > 5 * height_error_tol)
            continue; // this is likely a bad point

          pixel_vec[icam].push_back(pix);
          weight_vec[icam].push_back(opt.anchor_weight);
          isAnchor_vec[icam].push_back(1);

          // Create a shared_ptr as we need a pointer per the api to use later
          xyz_vec[icam].push_back(boost::shared_ptr<Vector3>(new Vector3()));
          Vector3 & xyz = *xyz_vec[icam].back().get(); // alias to the element we just made
          xyz = dem_xyz; // copy the value, but the pointer does not change
          xyz_vec_ptr[icam].push_back(&xyz[0]); // keep the pointer to the first element

          numAnchorPts++;
        }   
      }
      
      std::cout << "--lines and samples: " << numLines << ' ' << numSamples << std::endl;
      std::cout << "num lines per quat: " << double(numLines) / numQuat  << std::endl;
      std::cout << "--num anchor points " << numAnchorPts << std::endl;
      
    }   
  }

  // Do here two passes, first for non-anchor points and then for anchor ones.
  // This way it is easier to do the bookkeeping when saving the residuals.
  // TODO(oalexan1): Make this a function
  for (int pass = 0; pass < 2; pass++) {
    for (int icam = 0; icam < (int)crn.size(); icam++) {
      for (size_t ipix = 0; ipix < pixel_vec[icam].size(); ipix++) {

        Vector2 observation =  pixel_vec[icam][ipix];
        double * tri_point = xyz_vec_ptr[icam][ipix];
        double weight = weight_vec[icam][ipix];
        bool isAnchor = isAnchor_vec[icam][ipix];

        // Pass 0 is without anchor points, while pass 1 uses them
        if ((int)isAnchor != pass) 
          continue;
        
        // Must grow the number of quaternions and positions a bit
        // because during optimization the 3D point and corresponding
        // pixel may move somewhat.
        double line_extra = opt.max_init_reproj_error + 5.0; // add some more just in case
        csm::ImageCoord imagePt1, imagePt2;
        asp::toCsmPixel(observation - Vector2(0.0, line_extra), imagePt1);
        asp::toCsmPixel(observation + Vector2(0.0, line_extra), imagePt2);
        double time1 = ls_cams[icam]->getImageTime(imagePt1);
        double time2 = ls_cams[icam]->getImageTime(imagePt2);

        // Handle quaternions. We follow closely the conventions for UsgsAstroLsSensorModel.
        int numQuatPerObs = 8; // Max num of quaternions used in pose interpolation 
        int numQuat       = ls_cams[icam]->m_quaternions.size() / NUM_QUAT_PARAMS;
        double quatT0     = ls_cams[icam]->m_t0Quat;
        double quatDt     = ls_cams[icam]->m_dtQuat;

        // Starting and ending quat index (ending is exclusive). Based on lagrangeInterp().
        int qindex1      = static_cast<int>((time1 - quatT0) / quatDt);
        int qindex2      = static_cast<int>((time2 - quatT0) / quatDt);
        int begQuatIndex = std::min(qindex1, qindex2) - numQuatPerObs / 2 + 1;
        int endQuatIndex = std::max(qindex1, qindex2) + numQuatPerObs / 2 + 1;

        // Keep in bounds
        begQuatIndex = std::max(0, begQuatIndex);
        endQuatIndex = std::min(endQuatIndex, numQuat);
        if (begQuatIndex >= endQuatIndex) // Must not happen 
          vw_throw(ArgumentErr() << "Book-keeping error for pixel: " << observation << ".\n"); 

        // Same for positions
        int numPosPerObs = 8;
        int numPos       = ls_cams[icam]->m_positions.size() / NUM_XYZ_PARAMS;
        double posT0     = ls_cams[icam]->m_t0Ephem;
        double posDt     = ls_cams[icam]->m_dtEphem;
      
        // Starting and ending pos index (ending is exclusive). Based on lagrangeInterp().
        int pindex1 = static_cast<int>((time1 - posT0) / posDt);
        int pindex2 = static_cast<int>((time2 - posT0) / posDt);
        int begPosIndex = std::min(pindex1, pindex2) - numPosPerObs / 2 + 1;
        int endPosIndex = std::max(pindex1, pindex2) + numPosPerObs / 2 + 1;

        // Keep in bounds
        begPosIndex = std::max(0, begPosIndex);
        endPosIndex = std::min(endPosIndex, numPos);
        if (begPosIndex >= endPosIndex) // Must not happen 
          vw_throw(ArgumentErr() << "Book-keeping error for pixel: " << observation << ".\n"); 

        ceres::CostFunction* pixel_cost_function =
          pixelReprojectionError::Create(observation, weight, ls_cams[icam],
                                         begQuatIndex, endQuatIndex,
                                         begPosIndex, endPosIndex);
        ceres::LossFunction* pixel_loss_function = new ceres::CauchyLoss(opt.robust_threshold);
      
        // The variable of optimization are camera quaternions and positions stored in the
        // camera models, and the triangulated point.
        std::vector<double*> vars;
        for (int it = begQuatIndex; it < endQuatIndex; it++)
          vars.push_back(&ls_cams[icam]->m_quaternions[it * NUM_QUAT_PARAMS]);
        for (int it = begPosIndex; it < endPosIndex; it++)
          vars.push_back(&ls_cams[icam]->m_positions[it * NUM_XYZ_PARAMS]);
        vars.push_back(tri_point);
        problem.AddResidualBlock(pixel_cost_function, pixel_loss_function, vars);

        for (int c = 0; c < PIXEL_SIZE; c++)
          weight_per_residual.push_back(weight);

        // Anchor points are fixed by definition. They try to prevent
        // the cameras from moving too much from original poses.
        if (isAnchor) 
          problem.SetParameterBlockConstant(tri_point);
      }
    }
  }
  
  // Add the triangulated points constraint. We check earlier that only one
  // of the two options below can be set at a time.
  // TODO(oalexan1): Make this into a function.
  double xyz_weight = -1.0, xyz_threshold = -1.0;
  if (have_dem) {

    if (!opt.heights_from_dem.empty()) {
      xyz_weight = opt.heights_from_dem_weight;
      xyz_threshold = opt.heights_from_dem_robust_threshold;
    }
  
    if (!opt.ref_dem.empty()) {
      xyz_weight = opt.ref_dem_weight;
      xyz_threshold = opt.ref_dem_robust_threshold;
    }

    if (dem_xyz_vec.size() != cnet.size()) 
      vw_throw(ArgumentErr() << "Must have as many xyz computed from DEM as xyz "
               << "triangulated from match files.\n");
    if (xyz_weight <= 0 || xyz_threshold <= 0)
      vw_throw(ArgumentErr() << "Detected invalid robust threshold or weights.\n");

    for (int ipt = 0; ipt < num_tri_points; ipt++) {
      
      if (cnet[ipt].type() == vw::ba::ControlPoint::GroundControlPoint)
        vw_throw(ArgumentErr() << "Found GCP where not expecting any.\n");

      // Note that we get tri points from dem_xyz_vec, and not from cnet, as the latter
      // doesn't have adjustments for the input DEM.
      Vector3 observation = dem_xyz_vec.at(ipt);
      if (outliers.find(ipt) != outliers.end() || observation == Vector3(0, 0, 0)) 
        continue; // outlier
      
      ceres::CostFunction* xyz_cost_function = weightedXyzError::Create(observation, xyz_weight);
      ceres::LossFunction* xyz_loss_function = new ceres::CauchyLoss(xyz_threshold);
      double * tri_point = &tri_points_vec[0] + ipt * NUM_XYZ_PARAMS;
      problem.AddResidualBlock(xyz_cost_function, xyz_loss_function, tri_point);

      for (int c = 0; c < NUM_XYZ_PARAMS; c++)
        weight_per_residual.push_back(xyz_weight);
    }
    
  } // end considering xyz constraint in the presence of a DEM

  // Add regularization terms, from keeping the positions and quaternions from going wild

  if (opt.rotation_weight > 0.0) {
    for (int icam = 0; icam < (int)crn.size(); icam++) {
      int numQuat = ls_cams[icam]->m_quaternions.size() / NUM_QUAT_PARAMS;
      for (int iq = 0; iq < numQuat; iq++) {
        ceres::CostFunction* rotation_cost_function
          = weightedRotationError::Create(&ls_cams[icam]->m_quaternions[iq * NUM_QUAT_PARAMS],
                                          opt.rotation_weight);
        // We use no loss function, as the quaternions have no outliers
        ceres::LossFunction* rotation_loss_function = NULL;
        problem.AddResidualBlock(rotation_cost_function, rotation_loss_function,
                                 &ls_cams[icam]->m_quaternions[iq * NUM_QUAT_PARAMS]);
        
        for (int c = 0; c < NUM_QUAT_PARAMS; c++)
          weight_per_residual.push_back(opt.rotation_weight);
      }
    }
  }
  
  if (opt.translation_weight > 0.0) {
    for (int icam = 0; icam < (int)crn.size(); icam++) {
      int numPos = ls_cams[icam]->m_positions.size() / NUM_XYZ_PARAMS;
      for (int ip = 0; ip < numPos; ip++) {
        ceres::CostFunction* translation_cost_function
          = weightedTranslationError::Create(&ls_cams[icam]->m_positions[ip * NUM_XYZ_PARAMS],
                                          opt.translation_weight);
        // We use no loss function, as the positions have no outliers
        ceres::LossFunction* translation_loss_function = NULL;
        problem.AddResidualBlock(translation_cost_function, translation_loss_function,
                                 &ls_cams[icam]->m_positions[ip * NUM_XYZ_PARAMS]);
        
        for (int c = 0; c < NUM_XYZ_PARAMS; c++)
          weight_per_residual.push_back(opt.translation_weight);
      }
    }
  }

  if (opt.quat_norm_weight > 0.0) {
    for (int icam = 0; icam < (int)crn.size(); icam++) {
      int numQuat = ls_cams[icam]->m_quaternions.size() / NUM_QUAT_PARAMS;
      for (int iq = 0; iq < numQuat; iq++) {
        ceres::CostFunction* quat_norm_cost_function
          = weightedQuatNormError::Create(opt.quat_norm_weight);
        // We use no loss function, as the quaternions have no outliers
        ceres::LossFunction* quat_norm_loss_function = NULL;
        problem.AddResidualBlock(quat_norm_cost_function, quat_norm_loss_function,
                                 &ls_cams[icam]->m_quaternions[iq * NUM_QUAT_PARAMS]);
        
        weight_per_residual.push_back(opt.quat_norm_weight); // 1 single residual
      }
    }
  }

  // Save residuals before optimization
  std::string residual_prefix = opt.out_prefix + "-initial_residuals";
  save_residuals(residual_prefix, problem, opt, cnet, crn, have_dem, datum,
                 tri_points_vec, dem_xyz_vec, outliers, weight_per_residual);
  
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

  // Save residuals after optimization
  // TODO(oalexan1): Add here the anchor residuals
  residual_prefix = opt.out_prefix + "-final_residuals";
  save_residuals(residual_prefix, problem, opt, cnet, crn, have_dem, datum,
                 tri_points_vec, dem_xyz_vec, outliers, weight_per_residual);


  // Save the optimized model states. Note that we optimized directly the camera
  // model states, so there's no need to update them from some optimization
  // workspace.
  for (size_t icam = 0; icam < opt.camera_models.size(); icam++) {
    std::string adjustFile = asp::bundle_adjust_file_name(opt.out_prefix,
                                                          opt.image_files[icam],
                                                          opt.camera_files[icam]);
    std::string csmFile = asp::csmStateFile(adjustFile);
    
    vw::camera::CameraModel * base_cam
      = vw::camera::unadjusted_model(opt.camera_models[icam]).get();
    asp::CsmModel * csm_cam = dynamic_cast<asp::CsmModel*>(base_cam);
    if (csm_cam == NULL)
      vw_throw(ArgumentErr() << "Expecting the cameras to be of CSM type.\n");

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
