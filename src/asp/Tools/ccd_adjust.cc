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


/// \file ccd_adjust.cc
///
/// Find the CCD offsets in WV images. This code is not finished
/// and was not tested.

#include <vw/BundleAdjustment/ControlNetwork.h>
#include <vw/BundleAdjustment/ControlNetworkLoader.h>
#include <asp/Core/InterestPointMatching.h>
#include <asp/Core/BundleAdjustUtils.h>
#include <asp/Tools/ccd_adjust.h>

// Turn off warnings from eigen
#if defined(__GNUC__) || defined(__GNUG__)
#define LOCAL_GCC_VERSION (__GNUC__ * 10000                    \
			   + __GNUC_MINOR__ * 100              \
			   + __GNUC_PATCHLEVEL__)
#if LOCAL_GCC_VERSION >= 40600
#pragma GCC diagnostic push
#endif
#if LOCAL_GCC_VERSION >= 40202
#pragma GCC diagnostic ignored "-Wunused-local-typedefs"
#endif
#endif

#include <ceres/ceres.h>
#include <ceres/loss_function.h>

#if defined(__GNUC__) || defined(__GNUG__)
#if LOCAL_GCC_VERSION >= 40600
#pragma GCC diagnostic pop
#endif
#undef LOCAL_GCC_VERSION
#endif

namespace po = boost::program_options;
namespace fs = boost::filesystem;

using namespace vw;
using namespace vw::camera;
using namespace vw::ba;


namespace asp{

  int g_ccd_num_errors = 0;
  Mutex g_ccd_mutex;

const int NUM_PIXEL_PARAMS = 2;
const int NUM_POINT_PARAMS = 3;

// If positions are 1000 and 2000, there are three offsets,
// before 1000, between 1000 and 2000, and after 2000.
int get_cdd_offset_index(double col, std::vector<double> const& ccd_pos){

  if (col <= ccd_pos[0]) return 0;

  for (size_t t = 0; t < ccd_pos.size(); t++){
    if (ccd_pos[t] < col && (t+1 >= ccd_pos.size() || col <= ccd_pos[t+1])) {
      return t + 1;
    }
  }

  VW_ASSERT( false,
             vw::ArgumentErr()
             << "get_offset_index: Must never arrive here.\n" );

  return -1;
}

// A ceres cost function. We pass in the observation, the model, and
// the current camera and point indices. The result is the residual,
// the difference in the observation and the projection of the point
// into the camera, normalized by pixel_sigma.
struct CCDReprojectionError {
  CCDReprojectionError(Vector2 const& observation,
                       Vector2 const& pixel_sigma,
                       boost::shared_ptr<vw::camera::CameraModel> cam,
                       std::vector<double> const& ccd_pos):
    m_observation(observation),
    m_pixel_sigma(pixel_sigma),
    m_cam(cam),
    m_ccd_pos(ccd_pos){}

  ///  
  template <typename T>
  bool operator()(const T* const offset, const T* const point, T* residuals) const {

    try{

      Vector<double, NUM_POINT_PARAMS> point_vec;
      for (int c = 0; c < NUM_POINT_PARAMS; c++)
        point_vec[c] = (double)point[c];

      Vector<double, NUM_PIXEL_PARAMS> offset_vec;
      for (int c = 0; c < NUM_PIXEL_PARAMS; c++)
        offset_vec[c] = (double)offset[c];

      // TODO: Pass here the initial guess!
      // TODO: Think of the line below!!! + or -!!!
      Vector2 prediction = m_cam->point_to_pixel(point_vec) + offset_vec;

      // The error is the difference between the predicted and observed position,
      // normalized by sigma.
      residuals[0] = (prediction[0] - m_observation[0])/m_pixel_sigma[0];
      residuals[1] = (prediction[1] - m_observation[1])/m_pixel_sigma[1];

    } catch (std::exception const& e) {

      // Failed to compute residuals

      Mutex::Lock lock( g_ccd_mutex );
      g_ccd_num_errors++;
      if (g_ccd_num_errors < 100) {
	vw_out(ErrorMessage) << e.what() << std::endl;
      }else if (g_ccd_num_errors == 100) {
	vw_out() << "Will print no more error messages about "
                 << "failing to compute residuals.\n";
      }

      residuals[0] = T(1e+20);
      residuals[1] = T(1e+20);
      return false;
    }

    return true;
  }

  /// Factory to hide the construction of the CostFunction object from the client code.
  /// - Pass in pixel location, sigma, camera model, and vector of default CCD positions?
  static ceres::CostFunction* Create(Vector2 const& observation,
                                     Vector2 const& pixel_sigma,
                                     boost::shared_ptr<vw::camera::CameraModel> cam,
                                     std::vector<double> const& ccd_pos){

    return (new ceres::NumericDiffCostFunction<CCDReprojectionError,
            ceres::CENTRAL, NUM_PIXEL_PARAMS, NUM_PIXEL_PARAMS, NUM_POINT_PARAMS>
            (new CCDReprojectionError(observation, pixel_sigma, cam, ccd_pos)));
  }

  Vector2 m_observation;
  Vector2 m_pixel_sigma;
  boost::shared_ptr<vw::camera::CameraModel> m_cam;

  std::vector<double> const& m_ccd_pos; // alias
};

ceres::LossFunction* get_ccd_loss_function(){
  return new ceres::CauchyLoss(0.5);
}


// Will be called at each iteration. Here we can put any desired logging info.
class CCDAdjCallback: public ceres::IterationCallback {
public:
  virtual ceres::CallbackReturnType operator()
    (const ceres::IterationSummary& summary) {

    return ceres::SOLVER_CONTINUE;
  }
};

// Use Ceres to do piecewise bundle adjustment (multiple along-track
// adjustments per linescan camera to solve for ccd).
void ccd_adjust(std::vector<std::string> const& image_files,
                std::vector<std::string> const& camera_files,
                std::vector< boost::shared_ptr<vw::camera::CameraModel> > const& camera_models,
                std::string const& out_prefix,
                std::string const& match_file, // Input match file
                int num_threads){

  vw_out() << "Identifying CCD offsets.\n";

  if (image_files.size() != camera_files.size())
    vw_throw( ArgumentErr() << "Expecting as many images as cameras.\n" );

  int num_cameras = camera_models.size();
  if (num_cameras != 2)
    vw_throw( ArgumentErr() << "Can solve for CCD offsets only for two cameras.\n" );

  std::map< std::pair<int, int>, std::string> match_files;
  match_files[std::pair<int, int>(0, 1)] = match_file;

  // Set up control network containing all the input matched pixel pairs
  int min_matches = 30;   // TODO: Think more here
  double min_angle = 0.1; // in degrees
  ba::ControlNetwork cnet("CcdAdjust");
  bool triangulate_control_points = true;
  bool success = build_control_network(triangulate_control_points,
                                       cnet, // output
                                       camera_models, image_files, match_files, min_matches,
                                       min_angle*(M_PI/180));

  if (!success)
    vw_throw( ArgumentErr() << "Insufficient number of matches to solve for ccd.\n" );

  int num_points = cnet.size();

  // TODO: This must come from outside!!!
  std::vector<double>  ccd_pos; //TODO: THIS IS NOT SET!!!!!!!!!!!!!
  int num_offsets = ccd_pos.size() + 1; // TODO: Explain this!

  // Pack all the CCD offset parameters into a vector (x,y per CCD)
  std::vector<double> ccd_offsets_vec(NUM_PIXEL_PARAMS*num_offsets, 0);
  double* ccd_offsets = &ccd_offsets_vec[0];

  // Pack all the point parameters into a vector
  std::vector<double> points_vec(num_points*NUM_POINT_PARAMS, 0.0);
  for (int ipt = 0; ipt < num_points; ipt++){
    for (int q = 0; q < NUM_POINT_PARAMS; q++){
      points_vec[ipt*NUM_POINT_PARAMS + q] = cnet[ipt].position()[q];
    }
  }
  double* points = &points_vec[0];

  // The ceres problem
  ceres::Problem problem;

  // Add the cost function component for difference of pixel observations
  CameraRelationNetwork<JFeature> crn;
  crn.read_controlnetwork(cnet);
  for (int icam = 0; icam < (int)crn.size(); icam++) {

    typedef CameraNode<JFeature>::iterator crn_iter;
    for (crn_iter fiter = crn[icam].begin(); fiter != crn[icam].end(); fiter++){

      // The index of the 3D point
      int ipt = (**fiter).m_point_id;

      VW_ASSERT(icam < num_cameras, ArgumentErr() << "Out of bounds in the number of cameras.\n");
      VW_ASSERT(ipt < num_points,   ArgumentErr() << "Out of bounds in the number of points.\n");

      // The observed value for the projection of point with index ipt into
      // the camera with index icam.
      Vector2 observation = (**fiter).m_location;
      Vector2 pixel_sigma = (**fiter).m_scale;

      // This is a bugfix
      if (pixel_sigma != pixel_sigma)
        pixel_sigma = Vector2(1, 1);

      int offset_index = get_cdd_offset_index(observation.x(), ccd_pos);

      VW_ASSERT(0 <= offset_index && offset_index < (int)ccd_offsets_vec.size(),
                ArgumentErr() << "Out of bounds in the offset index.\n");

      // Each observation corresponds to a pair of a camera and a point
      double * offset = ccd_offsets + offset_index * NUM_PIXEL_PARAMS;
      double * point  = points      + ipt          * NUM_POINT_PARAMS;

      ceres::LossFunction* loss_function = get_ccd_loss_function();

      ceres::CostFunction* cost_function
                = CCDReprojectionError::Create(observation, pixel_sigma,
                                               camera_models[icam], ccd_pos);
      problem.AddResidualBlock(cost_function, loss_function,
                               offset, point);
    } // End loop through points seen by this camera
  } // End loop through cameras

  // Set up Ceres options
  ceres::Solver::Options options;
  options.gradient_tolerance = 1e-16;
  options.function_tolerance = 1e-16;
  options.max_num_iterations = 1000;
  options.max_num_consecutive_invalid_steps = 100; // try hard
  options.minimizer_progress_to_stdout = false;

  options.num_threads = num_threads;

  options.linear_solver_type = ceres::SPARSE_SCHUR;
  //options.ordering_type = ceres::SCHUR;
  //options.eta = 1e-3; // FLAGS_eta;
  //options->max_solver_time_in_seconds = FLAGS_max_solver_time;
  //options->use_nonmonotonic_steps = FLAGS_nonmonotonic_steps;
  //if (FLAGS_line_search) {
  //  options->minimizer_type = ceres::LINE_SEARCH;
  //}

  // Use a callback function at every iteration.
  CCDAdjCallback callback;
  options.callbacks.push_back(&callback);
  options.update_state_every_iteration = true; // ensure we have the latest adjustments

  // Solve the problem using Ceres
  ceres::Solver::Summary summary;
  ceres::Solve(options, &problem, &summary);
  vw_out() << summary.FullReport() << "\n";
  if (summary.termination_type == ceres::NO_CONVERGENCE){
    // Print a clarifying message, so the user does not think that the algorithm failed.
    vw_out() << "Found a valid solution, but did not reach the actual minimum." << std::endl;
  }

  std::cout << "--ofsets: " << std::endl;
  for (size_t ioff = 0; ioff < ccd_offsets_vec.size(); ioff++) {
    std::cout << ccd_offsets_vec[ioff] << std::endl;
  }

#if 0
  // Save the adjustments. We will recover them later based on the output prefix.
  start_index = 0;
  for (int icam = 0; icam < (int)crn.size(); icam++) {

    if (icam > 0)
      start_index += num_adj_per_cam[icam - 1];
    int end_index = start_index + num_adj_per_cam[icam];

    std::vector<vw::Vector3> position_adjustments;
    std::vector<vw::Quat>    pose_adjustments;

    std::string adjust_file = asp::bundle_adjust_file_name(out_prefix,
							   image_files[icam],
							   camera_files[icam]);
    vw_out() << "Writing: " << adjust_file << std::endl;
    std::string session = "dg";
    asp::write_adjustments(adjust_file,
			   adjustment_bounds[icam],
			   position_adjustments, pose_adjustments, session);
  }
#endif

}

} // end namespace asp
