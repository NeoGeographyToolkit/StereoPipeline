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

// Cost functions used in bundle adjustment. These need access to the camera
// models, so they are stored in the Camera folder.

#include <asp/Camera/BundleAdjustCostFuns.h>
#include <asp/Camera/BaseCostFuns.h>

using namespace vw;
using namespace vw::camera;

// Return the size of each parameter block.
// These should sum up to equal num_params.
// The first block is always the point block (3) and
// the second block is always the pose block (6).
std::vector<int> CeresBundleModelBase::get_block_sizes() const {
  std::vector<int> result(2);
  result[0] = num_point_params();
  result[1] = num_pose_params();
  return result;
}

// Read in all of the parameters and compute the residuals.
vw::Vector2 AdjustedCameraBundleModel::evaluate(
  std::vector<double const*> const param_blocks) const {

  double const* raw_point = param_blocks[0];
  double const* raw_pose  = param_blocks[1];

  // Read the point location and camera information from the raw arrays.
  Vector3          point(raw_point[0], raw_point[1], raw_point[2]);
  CameraAdjustment correction(raw_pose);

  vw::camera::AdjustedCameraModel cam(m_underlying_camera,
        correction.position(),
        correction.pose());
  try {
    return cam.point_to_pixel(point);
  } catch(std::exception const& e) {
  }

  // We must not allow one bad point to ruin the optimization
  return vw::Vector2(g_big_pixel_value, g_big_pixel_value);
}

std::vector<int> PinholeBundleModel::get_block_sizes() const {
  std::vector<int> result = CeresBundleModelBase::get_block_sizes();
  result.push_back(asp::NUM_CENTER_PARAMS);
  result.push_back(asp::NUM_FOCUS_PARAMS);
  result.push_back(num_dist_params());
  return result;
}

// Read in all of the parameters and compute the residuals.
vw::Vector2 PinholeBundleModel::evaluate(
  std::vector<double const*> const param_blocks) const {

  double const* raw_point  = param_blocks[0];
  double const* raw_pose   = param_blocks[1];
  double const* raw_center = param_blocks[2];
  double const* raw_focus  = param_blocks[3];
  double const* raw_lens   = param_blocks[4];

  // TODO: Should these values also be scaled?
  // Read the point location and camera information from the raw arrays.
  Vector3          point(raw_point[0], raw_point[1], raw_point[2]);
  CameraAdjustment correction(raw_pose);

  // We actually solve for scale factors for intrinsic values, so multiply them
  //  by the original intrinsic values to get the updated values.
  double center_x = raw_center[0] * m_underlying_camera->point_offset()[0];
  double center_y = raw_center[1] * m_underlying_camera->point_offset()[1];
  double focus    = raw_focus [0] * m_underlying_camera->focal_length()[0];

  // Update the lens distortion parameters in the new camera.
  // - These values are also optimized as scale factors.
  // TODO: This approach FAILS when the input value is zero!!
  boost::shared_ptr<LensDistortion> distortion 
    = m_underlying_camera->lens_distortion()->copy();
  vw::Vector<double> lens = distortion->distortion_parameters();
  for (size_t i=0; i<lens.size(); ++i)
    lens[i] *= raw_lens[i];
  distortion->set_distortion_parameters(lens);

  // Duplicate the input camera model with the pose, focus, center, and lens updated.
  // Respect m_u_direction, m_v_direction, m_w_direction in the original model.
  vw::camera::PinholeModel cam = *m_underlying_camera;
  cam.set_camera_center(correction.position());
  cam.set_camera_pose(correction.pose().rotation_matrix());
  cam.set_focal_length(vw::Vector2(focus, focus));
  cam.set_point_offset(vw::Vector2(center_x, center_y));
  cam.set_lens_distortion(distortion.get());
  cam.set_pixel_pitch(m_underlying_camera->pixel_pitch());
  try {
    // Project the point into the camera.
    Vector2 pixel = cam.point_to_pixel_no_check(point);
    return pixel;
  } catch(...) {
  }

  // Do not allow one bad pixel value to ruin the whole problem
  return vw::Vector2(g_big_pixel_value, g_big_pixel_value);
}

std::vector<int> OpticalBarBundleModel::get_block_sizes() const {
  std::vector<int> result = CeresBundleModelBase::get_block_sizes();
  result.push_back(asp::NUM_CENTER_PARAMS);
  result.push_back(asp::NUM_FOCUS_PARAMS);
  result.push_back(asp::NUM_OPTICAL_BAR_EXTRA_PARAMS);
  return result;
}

// Read in all of the parameters and compute the residuals.
vw::Vector2  OpticalBarBundleModel::evaluate(
  std::vector<double const*> const param_blocks) const {

  double const* raw_point  = param_blocks[0];
  double const* raw_pose   = param_blocks[1];
  double const* raw_center = param_blocks[2];
  double const* raw_focus  = param_blocks[3];
  double const* raw_intrin = param_blocks[4];

  // TODO: Should these values also be scaled?
  // Read the point location and camera information from the raw arrays.
  Vector3          point(raw_point[0], raw_point[1], raw_point[2]);
  CameraAdjustment correction(raw_pose);

  // We actually solve for scale factors for intrinsic values, so multiply them
  //  by the original intrinsic values to get the updated values.
  double center_x  = raw_center[0] * m_underlying_camera->get_optical_center()[0];
  double center_y  = raw_center[1] * m_underlying_camera->get_optical_center()[1];
  double focus     = raw_focus [0] * m_underlying_camera->get_focal_length  ();
  double speed     = raw_intrin[0] * m_underlying_camera->get_speed();
  double mcf       = raw_intrin[1] * m_underlying_camera->get_motion_compensation();
  double scan_time = raw_intrin[2] * m_underlying_camera->get_scan_time();

  // Duplicate the input camera model with the pose, focus, center, speed, and MCF updated.
  vw::camera::OpticalBarModel cam(m_underlying_camera->get_image_size(),
                                    vw::Vector2(center_x, center_y),
                                    m_underlying_camera->get_pixel_size(),
                                    focus,
                                    scan_time,
                                    //m_underlying_camera->get_scan_rate(),
                                    m_underlying_camera->get_scan_dir(),
                                    m_underlying_camera->get_forward_tilt(),
                                    correction.position(),
                                    correction.pose().axis_angle(),
                                    speed,  mcf);

  // Project the point into the camera.
  try {
    return cam.point_to_pixel(point);
  } catch(std::exception const& e){
  }
  
  // We must not allow one bad point to ruin the optimization
  return vw::Vector2(g_big_pixel_value, g_big_pixel_value);
}

std::vector<int> CsmBundleModel::get_block_sizes() const {
  std::vector<int> result = CeresBundleModelBase::get_block_sizes();
  result.push_back(asp::NUM_CENTER_PARAMS);
  result.push_back(asp::NUM_FOCUS_PARAMS);
  result.push_back(num_dist_params());
  return result;
}

// Read in all of the parameters and compute the residuals.
vw::Vector2 CsmBundleModel::evaluate(std::vector<double const*> const param_blocks) const {

  // TODO(oalexan1): Use here transformedCsmCamera() to avoid code repetition. 
  // But note that that one may set zero distortion to 1e-16 which likely here
  // we don't need to do.
  double const* raw_point  = param_blocks[0];
  double const* raw_pose   = param_blocks[1];
  double const* raw_center = param_blocks[2];
  double const* raw_focus  = param_blocks[3];
  double const* raw_dist   = param_blocks[4];

  // TODO: Should these values also be scaled?
  // Read the point location and camera information from the raw arrays.
  Vector3          point(raw_point[0], raw_point[1], raw_point[2]);
  CameraAdjustment correction(raw_pose);

  // We actually solve for scale factors for intrinsic values, so multiply them
  //  by the original intrinsic values to get the updated values.
  vw::Vector2 optical_center = m_underlying_camera->optical_center();
  double focal_length        = m_underlying_camera->focal_length();
  optical_center[0] = raw_center[0] * optical_center[0];
  optical_center[1] = raw_center[1] * optical_center[1];
  focal_length      = raw_focus [0] * focal_length;

  // Update the lens distortion parameters in the new camera.
  // - These values are also optimized as scale factors.
  std::vector<double> distortion = m_underlying_camera->distortion();
  for (size_t i = 0; i < distortion.size(); i++) {
    distortion[i] = raw_dist[i] * distortion[i];
  }

  // Duplicate the input camera model
  boost::shared_ptr<asp::CsmModel> copy;
  m_underlying_camera->deep_copy(copy);
  
  // Update the intrinsics of the copied model
  copy->set_optical_center(optical_center);
  copy->set_focal_length(focal_length);
  copy->set_distortion(distortion);

  // Form the adjusted camera. Note that unlike for Pinhole and Optical
  // bar, the parameters being optimized adjust the initial CSM camera,
  // rather than replacing it altogether. The CSM camera can in fact
  // be even linescan, when there would be many pose samples, in fact,
  // so it makes sense to work this way. 
  AdjustedCameraModel adj_cam(copy, correction.position(), correction.pose());

  try {
    // Project the point into the camera.
    Vector2 pixel = adj_cam.point_to_pixel(point);
    return pixel;
  } catch(...) {
  }

  // Do not allow one bad pixel value to ruin the whole problem
  return vw::Vector2(g_big_pixel_value, g_big_pixel_value);
}

// Call to work with ceres::DynamicCostFunctions.
// - Takes array of arrays.
bool BaReprojectionError::operator()(double const * const * parameters, 
                                     double * residuals) const {

  try {
    // Unpack the parameter blocks
    std::vector<double const*> param_blocks(m_num_param_blocks);
    for (size_t i=0; i<m_num_param_blocks; ++i) {
      param_blocks[i] = parameters[i];
    }

    // Use the camera model wrapper to handle all of the parameter blocks.
    Vector2 prediction = m_camera_wrapper->evaluate(param_blocks);

    // The error is the difference between the predicted and observed pixel
    // position, normalized by sigma.
    residuals[0] = (prediction[0] - m_observation[0])/m_pixel_sigma[0];
    residuals[1] = (prediction[1] - m_observation[1])/m_pixel_sigma[1];

  } catch (std::exception const& e) { // TODO: Catch only projection errors?
    // Failed to compute residuals

    Mutex::Lock lock(g_ba_mutex);
    g_ba_num_errors++;
    if (g_ba_num_errors < 100) {
      vw_out(ErrorMessage) << e.what() << std::endl;
    }else if (g_ba_num_errors == 100) {
      vw_out() << "Will print no more error messages about "
                << "failing to compute residuals.\n";
    }

    residuals[0] = g_big_pixel_value;
    residuals[1] = g_big_pixel_value;
    return false;
  }
  return true;
}

// Factory to hide the construction of the CostFunction object from the client code.
ceres::CostFunction*  
BaReprojectionError::Create(Vector2 const& observation,
                            Vector2 const& pixel_sigma,
                            boost::shared_ptr<CeresBundleModelBase> camera_wrapper) {
  const int NUM_RESIDUALS = 2;

  ceres::DynamicNumericDiffCostFunction<BaReprojectionError>* cost_function =
      new ceres::DynamicNumericDiffCostFunction<BaReprojectionError>(
          new BaReprojectionError(observation, pixel_sigma, camera_wrapper));

  // The residual size is always the same.
  cost_function->SetNumResiduals(NUM_RESIDUALS);

  // The camera wrapper knows all of the block sizes to add.
  std::vector<int> block_sizes = camera_wrapper->get_block_sizes();
  for (size_t i=0; i<block_sizes.size(); ++i) {
    cost_function->AddParameterBlock(block_sizes[i]);
  }
  return cost_function;
}

// Adaptor to work with ceres::DynamicCostFunctions.
bool BaDispXyzError::operator()(double const* const* parameters, double* residuals) const {

  try{
    // Split apart the input parameter blocks and hand them to the camera wrappers.
    std::vector<double const*> left_param_blocks, right_param_blocks;
    unpack_residual_pointers(parameters, left_param_blocks, right_param_blocks);

    // Get pixel projection in both cameras.
    Vector2 left_prediction  = m_left_camera_wrapper->evaluate (left_param_blocks );
    Vector2 right_prediction = m_right_camera_wrapper->evaluate(right_param_blocks);

    // See how consistent that is with the observed disparity.
    bool good_ans = true;
    if (!m_interp_disp.pixel_in_bounds(left_prediction)) {
      good_ans = false;
    }else{
      DispPixelT dispPix = m_interp_disp(left_prediction[0], left_prediction[1]);
      if (!is_valid(dispPix)) {
        good_ans = false;
      }else{
        Vector2 right_prediction_from_disp = left_prediction + dispPix.child();
        residuals[0] = right_prediction_from_disp[0] - right_prediction[0];
        residuals[1] = right_prediction_from_disp[1] - right_prediction[1];
        for (size_t it = 0; it < 2; it++) 
          residuals[it] *= m_reference_terrain_weight;
      }
    }

    // TODO: Think more of what to do below. The hope is that the robust cost
    // function will take care of big residuals graciously.
    if (!good_ans) {
      // Failed to find the residuals
      for (size_t it = 0; it < 2; it++) 
        residuals[it] = m_max_disp_error * m_reference_terrain_weight;
      return true;
    }

  } catch (const camera::PointToPixelErr& e) {
    // Failed to project into the camera
    for (size_t it = 0; it < 2; it++) 
      residuals[it] = m_max_disp_error * m_reference_terrain_weight;
    return true;
  }
  return true;
}

// TODO: Should this logic live somewhere else?
/// Create the list of residual pointers when solving for intrinsics.
/// - Extra logic is needed to avoid duplicate pointers.
void BaDispXyzError::get_residual_pointers(asp::BAParams &param_storage,
                                  int left_cam_index, int right_cam_index,
                                  bool solve_intrinsics,
                                  asp::IntrinsicOptions const& intrinsics_opt,
                                  std::vector<double*> &residual_ptrs) {

  double* left_camera  = param_storage.get_camera_ptr(left_cam_index);
  double* right_camera = param_storage.get_camera_ptr(right_cam_index);
  residual_ptrs.clear();
  if (solve_intrinsics) {
    double* left_center      = param_storage.get_intrinsic_center_ptr    (left_cam_index );
    double* left_focus       = param_storage.get_intrinsic_focus_ptr     (left_cam_index );
    double* left_distortion  = param_storage.get_intrinsic_distortion_ptr(left_cam_index );
    double* right_center     = param_storage.get_intrinsic_center_ptr    (right_cam_index);
    double* right_focus      = param_storage.get_intrinsic_focus_ptr     (right_cam_index);
    double* right_distortion = param_storage.get_intrinsic_distortion_ptr(right_cam_index);

    residual_ptrs.push_back(left_camera    );
    residual_ptrs.push_back(left_center    );
    residual_ptrs.push_back(left_focus     );
    residual_ptrs.push_back(left_distortion);
    residual_ptrs.push_back(right_camera   );
    if (!intrinsics_opt.center_shared    ) residual_ptrs.push_back(right_center    );
    if (!intrinsics_opt.focus_shared     ) residual_ptrs.push_back(right_focus     );
    if (!intrinsics_opt.distortion_shared) residual_ptrs.push_back(right_distortion);
  }
  else { // This handles the generic camera case.
    residual_ptrs.push_back(left_camera );
    residual_ptrs.push_back(right_camera);
  }
  return;
}

void BaDispXyzError::unpack_residual_pointers(double const* const* parameters,
                              std::vector<double const*> & left_param_blocks,
                              std::vector<double const*> & right_param_blocks) const {
  
  left_param_blocks.resize (m_num_left_param_blocks );
  right_param_blocks.resize(m_num_right_param_blocks);

  double const* raw_point = &(m_reference_xyz[0]);
  left_param_blocks [0] = raw_point; // The first input is always the point param block.
  right_param_blocks[0] = raw_point;

  int index = 0;
  for (size_t i=1; i<m_num_left_param_blocks; ++i) {
    left_param_blocks[i] = parameters[index];
    ++index;
  }
  if (!m_solve_intrinsics) {
    // Unpack everything from the right block in order.
    for (size_t i=1; i<m_num_right_param_blocks; ++i) {
      right_param_blocks[i] = parameters[index];
      ++index;
    }
  } else { // Solve for intrinsics. Handle shared intrinsics.
    right_param_blocks[1] = parameters[index]; // Pose and position
    ++index;
    if (m_intrinsics_opt.center_shared)
      right_param_blocks[2] = left_param_blocks[2];
    else {
      right_param_blocks[2] = parameters[index];
      ++index;
    }
    if (m_intrinsics_opt.focus_shared)
      right_param_blocks[3] = left_param_blocks[3];
    else {
      right_param_blocks[3] = parameters[index];
      ++index;
    }
    if (m_intrinsics_opt.distortion_shared)
      right_param_blocks[4] = left_param_blocks[4];
    else {
      right_param_blocks[4] = parameters[index];
      ++index;
    }
  } // End pinhole case
}
