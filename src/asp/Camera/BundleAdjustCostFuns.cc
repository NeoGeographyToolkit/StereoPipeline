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

// Cost functions used in bundle adjustment. These need access to the camera
// models, so they are stored in the Camera folder.

#include <asp/Core/StereoSettings.h>
#include <asp/Core/DataLoader.h>
#include <asp/Camera/BundleAdjustOptions.h>
#include <asp/Camera/BundleAdjustCostFuns.h>
#include <asp/Camera/BaseCostFuns.h>
#include <vw/Camera/OpticalBarModel.h>
#include <vw/Camera/CameraImage.h>
#include <vw/Camera/LensDistortion.h>
#include <vw/Camera/CameraUtilities.h>
#include <vw/Cartography/GeoReferenceBaseUtils.h>

using namespace vw;
using namespace vw::camera;

namespace asp {

double g_big_pixel_value = 1000.0;  // don't make this too big

/// Used to accumulate the number of reprojection errors in bundle adjustment.
int g_ba_num_errors = 0;
vw::Mutex g_ba_mutex;

// Return the size of each parameter block.
// These should sum up to equal num_params.
// The first block is always the point block (3) and
// the second block is always the pose block (6).
std::vector<int> BaCamBase::get_block_sizes() const {
  std::vector<int> result(2);
  result[0] = num_point_params();
  result[1] = num_pose_params();
  return result;
}

// Read in all of the parameters and compute the residuals.
vw::Vector2 BaAdjCam::evaluate(std::vector<double const*> const param_blocks) const {

  double const* raw_point = param_blocks[0];
  double const* raw_pose  = param_blocks[1];

  // Read the point location and camera information from the raw arrays.
  Vector3          point(raw_point[0], raw_point[1], raw_point[2]);
  CameraAdjustment correction(raw_pose);

  // Create adjusted camera that applies corrections to the underlying camera
  vw::camera::AdjustedCameraModel cam(m_underlying_camera,
        correction.position(),
        correction.pose());
  try {
    // Bathy or not
    if (asp::hasBathy(m_bathy_data))
      return vw::point_to_pixel(&cam, m_bathy_data.bathy_planes[m_camera_index],
                                m_bathy_data.refraction_index, point);
    else
      return cam.point_to_pixel(point);
      
  } catch(std::exception const& e) {
  }

  // We must not allow one bad point to ruin the optimization
  return vw::Vector2(g_big_pixel_value, g_big_pixel_value);
}

BaPinholeCam::BaPinholeCam(boost::shared_ptr<vw::camera::PinholeModel> cam):
  m_underlying_camera(cam) {}

// The number of lens distortion parameters.
int BaPinholeCam::num_dist_params() const {
  vw::Vector<double> lens_params
    = m_underlying_camera->lens_distortion()->distortion_parameters();
  return lens_params.size();
}

std::vector<int> BaPinholeCam::get_block_sizes() const {
  std::vector<int> result = BaCamBase::get_block_sizes();
  result.push_back(asp::NUM_CENTER_PARAMS);
  result.push_back(asp::NUM_FOCUS_PARAMS);
  result.push_back(num_dist_params());
  return result;
}

// Read in all of the parameters and compute the residuals.
vw::Vector2 BaPinholeCam::evaluate(std::vector<double const*> const param_blocks) const {

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
  for (size_t i = 0; i < lens.size(); i++)
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

BaOpticalBarCam::BaOpticalBarCam(
    boost::shared_ptr<vw::camera::OpticalBarModel> cam):
    m_underlying_camera(cam) {}

int BaOpticalBarCam::num_intrinsic_params() const {
   return asp::NUM_CENTER_PARAMS + asp::NUM_FOCUS_PARAMS + asp::NUM_OPTICAL_BAR_EXTRA_PARAMS;
}

std::vector<int> BaOpticalBarCam::get_block_sizes() const {
  std::vector<int> result = BaCamBase::get_block_sizes();
  result.push_back(asp::NUM_CENTER_PARAMS);
  result.push_back(asp::NUM_FOCUS_PARAMS);
  result.push_back(asp::NUM_OPTICAL_BAR_EXTRA_PARAMS);
  return result;
}

// Read in all of the parameters and compute the residuals.
vw::Vector2  BaOpticalBarCam::evaluate(std::vector<double const*> const param_blocks) const {

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
  double focus     = raw_focus [0] * m_underlying_camera->get_focal_length();
  double speed     = raw_intrin[0] * m_underlying_camera->get_speed();
  double mcf       = raw_intrin[1] * m_underlying_camera->get_motion_compensation();
  double scan_time = raw_intrin[2] * m_underlying_camera->get_scan_time();

  // The velocity is a 3-vector 
  bool have_velocity_vec = m_underlying_camera->get_have_velocity_vec();
  vw::Vector3 vel(0, 0, 0);
  vw::Vector3 final_pose(std::numeric_limits<double>::quiet_NaN(), 0, 0);

  if (have_velocity_vec) {
    vel = m_underlying_camera->get_velocity();
    final_pose = m_underlying_camera->get_final_pose();
    vel[0] *= raw_intrin[3];
    vel[1] *= raw_intrin[4];
    vel[2] *= raw_intrin[5];
    final_pose[0] *= raw_intrin[6];
    final_pose[1] *= raw_intrin[7];
    final_pose[2] *= raw_intrin[8];
  }

  // Create an optical bar camera with updated pose, focus, center, speed, and MCF
  vw::camera::OpticalBarModel cam(m_underlying_camera->get_image_size(),
                                  vw::Vector2(center_x, center_y),
                                  m_underlying_camera->get_pixel_size(),
                                  focus,
                                  scan_time,
                                  m_underlying_camera->get_scan_dir(),
                                  m_underlying_camera->get_forward_tilt(),
                                  correction.position(),
                                  correction.pose().axis_angle(),
                                  speed, mcf, have_velocity_vec, vel, final_pose);

  // Project the point into the camera.
  try {
    return cam.point_to_pixel(point);
  } catch(std::exception const& e) {
  }

  // We must not allow one bad point to ruin the optimization
  return vw::Vector2(g_big_pixel_value, g_big_pixel_value);
}

std::vector<int> BaCsmCam::get_block_sizes() const {
  std::vector<int> result = BaCamBase::get_block_sizes();
  result.push_back(asp::NUM_CENTER_PARAMS);
  result.push_back(asp::NUM_FOCUS_PARAMS);
  result.push_back(num_dist_params());
  return result;
}

// Read in all of the parameters and compute the residuals.
vw::Vector2 BaCsmCam::evaluate(std::vector<double const*> const param_blocks) const {

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
  for (size_t i = 0; i < distortion.size(); i++)
    distortion[i] = raw_dist[i] * distortion[i];

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
    for (size_t i = 0; i < m_num_param_blocks; i++) {
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
    } else if (g_ba_num_errors == 100) {
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
                            boost::shared_ptr<BaCamBase> camera_wrapper) {
  const int NUM_RESIDUALS = 2;

  ceres::DynamicNumericDiffCostFunction<BaReprojectionError>* cost_function =
      new ceres::DynamicNumericDiffCostFunction<BaReprojectionError>(
          new BaReprojectionError(observation, pixel_sigma, camera_wrapper));

  // The residual size is always the same.
  cost_function->SetNumResiduals(NUM_RESIDUALS);

  // The camera wrapper knows all of the block sizes to add.
  std::vector<int> block_sizes = camera_wrapper->get_block_sizes();
  for (size_t i = 0; i < block_sizes.size(); i++) {
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
    Vector2 left_prediction  = m_left_camera_wrapper->evaluate (left_param_blocks);
    Vector2 right_prediction = m_right_camera_wrapper->evaluate(right_param_blocks);

    // See how consistent that is with the observed disparity.
    bool good_ans = true;
    if (!m_interp_disp.pixel_in_bounds(left_prediction)) {
      good_ans = false;
    } else {
      DispPixelT dispPix = m_interp_disp(left_prediction[0], left_prediction[1]);
      if (!is_valid(dispPix)) {
        good_ans = false;
      } else {
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
void BaDispXyzError::get_residual_pointers(asp::BaParams &param_storage,
                                  int left_cam_index, int right_cam_index,
                                  bool solve_intrinsics,
                                  asp::IntrinsicOptions const& intrinsics_opt,
                                  std::vector<double*> &residual_ptrs) {

  double* left_camera  = param_storage.get_camera_ptr(left_cam_index);
  double* right_camera = param_storage.get_camera_ptr(right_cam_index);
  residual_ptrs.clear();
  if (solve_intrinsics) {
    double* left_center      = param_storage.get_intrinsic_center_ptr    (left_cam_index);
    double* left_focus       = param_storage.get_intrinsic_focus_ptr     (left_cam_index);
    double* left_distortion  = param_storage.get_intrinsic_distortion_ptr(left_cam_index);
    double* right_center     = param_storage.get_intrinsic_center_ptr    (right_cam_index);
    double* right_focus      = param_storage.get_intrinsic_focus_ptr     (right_cam_index);
    double* right_distortion = param_storage.get_intrinsic_distortion_ptr(right_cam_index);

    residual_ptrs.push_back(left_camera);
    residual_ptrs.push_back(left_center);
    residual_ptrs.push_back(left_focus);
    residual_ptrs.push_back(left_distortion);
    residual_ptrs.push_back(right_camera);
    if (!intrinsics_opt.center_shared) residual_ptrs.push_back(right_center);
    if (!intrinsics_opt.focus_shared) residual_ptrs.push_back(right_focus);
    if (!intrinsics_opt.distortion_shared) residual_ptrs.push_back(right_distortion);
  } else { // This handles the generic camera case.
    residual_ptrs.push_back(left_camera);
    residual_ptrs.push_back(right_camera);
  }
  return;
}

void BaDispXyzError::unpack_residual_pointers(double const* const* parameters,
                              std::vector<double const*> & left_param_blocks,
                              std::vector<double const*> & right_param_blocks) const {

  left_param_blocks.resize (m_num_left_param_blocks);
  right_param_blocks.resize(m_num_right_param_blocks);

  double const* raw_point = &(m_reference_xyz[0]);
  left_param_blocks [0] = raw_point; // The first input is always the point param block.
  right_param_blocks[0] = raw_point;

  int index = 0;
  for (size_t i = 1; i < m_num_left_param_blocks; i++) {
    left_param_blocks[i] = parameters[index];
    index++;
  }
  if (!m_solve_intrinsics) {
    // Unpack everything from the right block in order.
    for (size_t i = 1; i < m_num_right_param_blocks; i++) {
      right_param_blocks[i] = parameters[index];
      index++;
    }
  } else { // Solve for intrinsics. Handle shared intrinsics.
    right_param_blocks[1] = parameters[index]; // Pose and position
    index++;
    if (m_intrinsics_opt.center_shared)
      right_param_blocks[2] = left_param_blocks[2];
    else {
      right_param_blocks[2] = parameters[index];
      index++;
    }
    if (m_intrinsics_opt.focus_shared)
      right_param_blocks[3] = left_param_blocks[3];
    else {
      right_param_blocks[3] = parameters[index];
      index++;
    }
    if (m_intrinsics_opt.distortion_shared)
      right_param_blocks[4] = left_param_blocks[4];
    else {
      right_param_blocks[4] = parameters[index];
      index++;
    }
  } // End pinhole case
}

// Factory to hide the construction of the CostFunction object from
// the client code.
ceres::CostFunction* BaDispXyzError::Create(
    double max_disp_error, double reference_terrain_weight,
    Vector3 const& reference_xyz, ImageViewRef<DispPixelT> const& interp_disp,
    boost::shared_ptr<BaCamBase> left_camera_wrapper,
    boost::shared_ptr<BaCamBase> right_camera_wrapper,
    bool solve_intrinsics, asp::IntrinsicOptions intrinsics_opt) {

  const int NUM_RESIDUALS = 2;

  ceres::DynamicNumericDiffCostFunction<BaDispXyzError>* cost_function =
      new ceres::DynamicNumericDiffCostFunction<BaDispXyzError>(
          new BaDispXyzError(max_disp_error, reference_terrain_weight,
                              reference_xyz, interp_disp,
                              left_camera_wrapper, right_camera_wrapper,
                              solve_intrinsics, intrinsics_opt));

  // The residual size is always the same.
  cost_function->SetNumResiduals(NUM_RESIDUALS);

  // Add all of the blocks for each camera, except for the first (point)
  // block which is provided at creation time.
  std::vector<int> block_sizes = left_camera_wrapper->get_block_sizes();
  for (size_t i = 1; i < block_sizes.size(); i++) {
    cost_function->AddParameterBlock(block_sizes[i]);
  }
  block_sizes = right_camera_wrapper->get_block_sizes();
  if (!solve_intrinsics) {
    for (size_t i = 1; i < block_sizes.size(); i++) {
      cost_function->AddParameterBlock(block_sizes[i]);
    }
  } else { // Pinhole handling
    if (block_sizes.size() != 5)
      vw_throw(LogicErr() << "Error: Pinhole camera model parameter number error!");
    cost_function->AddParameterBlock(block_sizes[1]); // The camera position/pose
    if (!intrinsics_opt.center_shared) cost_function->AddParameterBlock(block_sizes[2]);
    if (!intrinsics_opt.focus_shared) cost_function->AddParameterBlock(block_sizes[3]);
    if (!intrinsics_opt.distortion_shared) cost_function->AddParameterBlock(block_sizes[4]);
  }
  return cost_function;
}  // End function Create

/// From the input options select the correct Ceres loss function.
ceres::LossFunction* get_loss_function(std::string const& cost_function, double th) {

  ceres::LossFunction* loss_function = NULL;
  if (cost_function == "l2")
    loss_function = NULL;
  else if (cost_function == "trivial")
    loss_function = new ceres::TrivialLoss();
  else if (cost_function == "huber")
    loss_function = new ceres::HuberLoss(th);
  else if (cost_function == "cauchy")
    loss_function = new ceres::CauchyLoss(th);
  else if (cost_function == "l1")
    loss_function = new ceres::SoftLOneLoss(th);
  else {
    vw::vw_throw(vw::ArgumentErr() << "Unknown cost function: " << cost_function << ".\n");
  }
  return loss_function;
}

/// Add error source for projecting a 3D point into the camera.
void add_reprojection_residual_block(vw::Vector2 const& observation,
                                     vw::Vector2 const& pixel_sigma,
                                     int point_index, int camera_index,
                                     asp::BaParams & param_storage,
                                     asp::BaOptions const& opt,
                                     ceres::SubsetManifold * dist_opts,
                                     ceres::Problem & problem) {

  ceres::LossFunction* loss_function;
  loss_function = get_loss_function(opt.cost_function, opt.robust_threshold);

  boost::shared_ptr<CameraModel> camera_model = opt.camera_models[camera_index];

  double* camera = param_storage.get_camera_ptr(camera_index);
  double* point  = param_storage.get_point_ptr(point_index);

  if (opt.camera_type == asp::BaCameraType_Other) {
    // The generic camera case. This includes pinhole and CSM too, when
    // the adjustments are external and intrinsics are not solved for.
    boost::shared_ptr<BaCamBase> wrapper(new BaAdjCam(camera_model,
                                                       opt.bathy_data,
                                                       camera_index));
      ceres::CostFunction* cost_function =
        BaReprojectionError::Create(observation, pixel_sigma, wrapper);
      problem.AddResidualBlock(cost_function, loss_function, point, camera);

  } else { // Solve for intrinsics for Pinhole, optical bar, or CSM camera
    double* center     = param_storage.get_intrinsic_center_ptr    (camera_index);
    double* focus      = param_storage.get_intrinsic_focus_ptr     (camera_index);
    double* distortion = param_storage.get_intrinsic_distortion_ptr(camera_index);

    boost::shared_ptr<BaCamBase> wrapper;
    if (opt.camera_type == asp::BaCameraType_Pinhole) {

      boost::shared_ptr<PinholeModel> pinhole_model =
        boost::dynamic_pointer_cast<PinholeModel>(camera_model);
      if (pinhole_model.get() == NULL)
        vw::vw_throw(vw::ArgumentErr()
                      << "Tried to add pinhole block with non-pinhole camera.");
      wrapper.reset(new BaPinholeCam(pinhole_model));

    } else if (opt.camera_type == asp::BaCameraType_OpticalBar) {

      boost::shared_ptr<vw::camera::OpticalBarModel> bar_model =
        boost::dynamic_pointer_cast<vw::camera::OpticalBarModel>(camera_model);
      if (bar_model.get() == NULL)
        vw::vw_throw(vw::ArgumentErr() << "Tried to add optical bar block with "
                      << "non-optical bar camera.");
      wrapper.reset(new BaOpticalBarCam(bar_model));

    } else if (opt.camera_type == asp::BaCameraType_CSM) {
      boost::shared_ptr<asp::CsmModel> csm_model =
        boost::dynamic_pointer_cast<asp::CsmModel>(camera_model);
      if (csm_model.get() == NULL)
        vw::vw_throw(vw::ArgumentErr() << "Tried to add CSM block with "
                      << "non-CSM camera.");
      wrapper.reset(new BaCsmCam(csm_model));
    } else {
      vw::vw_throw(vw::ArgumentErr() << "Unknown camera type.");
    }

    ceres::CostFunction* cost_function =
      BaReprojectionError::Create(observation, pixel_sigma, wrapper);
    problem.AddResidualBlock(cost_function, loss_function, point, camera,
                             center, focus, distortion);

    // Apply the residual limits
    size_t num_limits = opt.intrinsics_limits.size() / 2;
    if ((num_limits > 0) && (num_limits > wrapper->num_intrinsic_params())) {
      vw::vw_throw(vw::ArgumentErr() << "Error: Too many intrinsic limits provided!"
        << " This model has " << wrapper->num_intrinsic_params() << " intrinsic parameters.");
    }
    size_t intrinsics_index = 0;
    // Focal length
    if (num_limits > 0) {
      problem.SetParameterLowerBound(focus, 0, opt.intrinsics_limits[0]);
      problem.SetParameterUpperBound(focus, 0, opt.intrinsics_limits[1]);
      intrinsics_index++;
    }
    // Optical center
    while ((intrinsics_index < 3) && (intrinsics_index < num_limits)) {
      problem.SetParameterLowerBound(center, intrinsics_index-1,
                                     opt.intrinsics_limits[2*intrinsics_index]);
      problem.SetParameterUpperBound(center, intrinsics_index-1,
                                     opt.intrinsics_limits[2*intrinsics_index + 1]);
      intrinsics_index++;
    }
    // Distortion
    while (intrinsics_index < num_limits) {
      problem.SetParameterLowerBound(distortion, intrinsics_index-3,
                                     opt.intrinsics_limits[2*intrinsics_index]);
      problem.SetParameterUpperBound(distortion, intrinsics_index-3,
                                     opt.intrinsics_limits[2*intrinsics_index + 1]);
      intrinsics_index++;
    }

    // If we don't want to solve for something, just tell Ceres not to adjust the values.
    if (!opt.intrinsics_options.float_optical_center(camera_index))
      problem.SetParameterBlockConstant(center);
    if (!opt.intrinsics_options.float_focal_length(camera_index))
      problem.SetParameterBlockConstant(focus);
    if (opt.intrinsics_options.float_distortion_params(camera_index)) {
      if (!opt.fixed_distortion_indices.empty() && dist_opts != NULL)
        problem.SetManifold(distortion, dist_opts);
    } else {
      problem.SetParameterBlockConstant(distortion);
    }
  } // End non-generic camera case.

  // Fix this camera if requested
  if (opt.fixed_cameras_indices.find(camera_index) != opt.fixed_cameras_indices.end())
    problem.SetParameterBlockConstant(param_storage.get_camera_ptr(camera_index));
}

/// Add residual block for the error using reference xyz.
void add_disparity_residual_block(vw::Vector3 const& reference_xyz,
                                  vw::ImageViewRef<DispPixelT> const& interp_disp,
                                  int left_cam_index, int right_cam_index,
                                  asp::BaParams & param_storage,
                                  asp::BaOptions const& opt,
                                  ceres::Problem & problem) {

  ceres::LossFunction* loss_function
   = get_loss_function(opt.cost_function, opt.robust_threshold);

  boost::shared_ptr<CameraModel> left_camera_model  = opt.camera_models[left_cam_index ];
  boost::shared_ptr<CameraModel> right_camera_model = opt.camera_models[right_cam_index];

  const bool inline_adjustments = (opt.camera_type != asp::BaCameraType_Other);

  // Get the list of residual pointers that will be passed to ceres.
  std::vector<double*> residual_ptrs;
  BaDispXyzError::get_residual_pointers(param_storage,
                                        left_cam_index, right_cam_index,
                                        inline_adjustments, opt.intrinsics_options,
                                        residual_ptrs);
 if (opt.camera_type == asp::BaCameraType_Other) {

    boost::shared_ptr<BaCamBase>
      left_wrapper (new BaAdjCam(left_camera_model,
                                 opt.bathy_data,
                                 left_cam_index));
    boost::shared_ptr<BaCamBase>
      right_wrapper(new BaAdjCam(right_camera_model,
                                 opt.bathy_data,
                                 right_cam_index));
    ceres::CostFunction* cost_function =
      BaDispXyzError::Create(opt.max_disp_error, opt.reference_terrain_weight,
        reference_xyz, interp_disp, left_wrapper, right_wrapper,
        inline_adjustments, opt.intrinsics_options);

    problem.AddResidualBlock(cost_function, loss_function, residual_ptrs);

  } else { // Inline adjustments

    boost::shared_ptr<BaCamBase> left_wrapper, right_wrapper;

    if (opt.camera_type == asp::BaCameraType_Pinhole) {
      boost::shared_ptr<PinholeModel> left_pinhole_model =
        boost::dynamic_pointer_cast<vw::camera::PinholeModel>(left_camera_model);
      boost::shared_ptr<PinholeModel> right_pinhole_model =
        boost::dynamic_pointer_cast<vw::camera::PinholeModel>(right_camera_model);
      left_wrapper.reset (new BaPinholeCam(left_pinhole_model));
      right_wrapper.reset(new BaPinholeCam(right_pinhole_model));

    } else if (opt.camera_type == asp::BaCameraType_OpticalBar) {
      boost::shared_ptr<vw::camera::OpticalBarModel> left_bar_model =
        boost::dynamic_pointer_cast<vw::camera::OpticalBarModel>(left_camera_model);
      boost::shared_ptr<vw::camera::OpticalBarModel> right_bar_model =
        boost::dynamic_pointer_cast<vw::camera::OpticalBarModel>(right_camera_model);
      left_wrapper.reset (new BaOpticalBarCam(left_bar_model));
      right_wrapper.reset(new BaOpticalBarCam(right_bar_model));

    } else if (opt.camera_type == asp::BaCameraType_CSM) {
      boost::shared_ptr<asp::CsmModel> left_csm_model =
        boost::dynamic_pointer_cast<asp::CsmModel>(left_camera_model);
      boost::shared_ptr<asp::CsmModel> right_csm_model =
        boost::dynamic_pointer_cast<asp::CsmModel>(right_camera_model);
      left_wrapper.reset (new BaCsmCam(left_csm_model));
      right_wrapper.reset(new BaCsmCam(right_csm_model));

    } else {
      vw::vw_throw(vw::ArgumentErr() << "Unknown camera type.");
    }

    ceres::CostFunction* cost_function =
      BaDispXyzError::Create(opt.max_disp_error, opt.reference_terrain_weight,
                             reference_xyz, interp_disp, left_wrapper, right_wrapper,
                             inline_adjustments, opt.intrinsics_options);
    problem.AddResidualBlock(cost_function, loss_function, residual_ptrs);

  }

} // End function add_disparity_residual_block

// Pixel reprojection error. Note: cam_residual_counts and num_pixels_per_cam
// serve different purposes. 
void addPixelReprojCostFun(asp::BaOptions                         const& opt,
                           asp::CRN                              const& crn,
                           std::vector<int>                       const& count_map,
                           vw::ImageViewRef<vw::PixelMask<float>> const& weight_image,
                           vw::cartography::GeoReference          const& weight_image_georef,
                           std::vector<vw::Vector3>               const& dem_xyz_vec,
                           bool have_weight_image,
                           bool have_dem,
                           // Outputs
                           vw::ba::ControlNetwork                  & cnet,
                           asp::BaParams                           & param_storage,
                           ceres::SubsetManifold                   * dist_opts,
                           ceres::Problem                          & problem,
                           std::vector<size_t>                     & cam_residual_counts,
                           std::vector<size_t>                     & num_pixels_per_cam,
                           std::vector<std::vector<vw::Vector2>>   & pixels_per_cam,
                           std::vector<std::vector<vw::Vector3>>   & tri_points_per_cam,
                           std::vector<std::map<int, vw::Vector2>> & pixel_sigmas) {

  int num_cameras = param_storage.num_cameras();
  int num_points  = param_storage.num_points();
  if ((int)crn.size() != num_cameras)
    vw_throw(ArgumentErr() << "Book-keeping error, the size of CameraRelationNetwork "
             << "must equal the number of images.\n");

  cam_residual_counts.resize(num_cameras);
  num_pixels_per_cam.resize(num_cameras);
  pixels_per_cam.resize(num_cameras);
  tri_points_per_cam.resize(num_cameras);
  pixel_sigmas.resize(num_cameras);

  for (int icam = 0; icam < num_cameras; icam++) { // Camera loop
    cam_residual_counts[icam] = 0;
    num_pixels_per_cam[icam] = 0;
    for (auto fiter = crn[icam].begin(); fiter != crn[icam].end(); fiter++) { // IP loop

      // The index of the 3D point this IP is for.
      int ipt = (**fiter).m_point_id;
      if (param_storage.get_point_outlier(ipt))
        continue; // skip outliers

      VW_ASSERT(int(icam) < num_cameras,
                ArgumentErr() << "Out of bounds in the number of cameras.");
      VW_ASSERT(int(ipt)  < num_points,
                ArgumentErr() << "Out of bounds in the number of points.");

      double* point = param_storage.get_point_ptr(ipt);
      if (point[0] == 0 && point[1] == 0 && point[2] == 0) {
        // Flag points in the center of the planet as outliers
        param_storage.set_point_outlier(ipt, true);
        continue;
      }

      // Weight from image, if provided
      vw::PixelMask<float> img_wt = 1.0;
      if (have_weight_image) {
        Vector3 ecef(point[0], point[1], point[2]);
        img_wt = vw::cartography::closestPixelVal(weight_image, weight_image_georef, ecef);

        // Flag bad weights as outliers
        if (!is_valid(img_wt) || std::isnan(img_wt.child()) || img_wt.child() <= 0.0) {
          param_storage.set_point_outlier(ipt, true);
          continue;
        }
      }

      // Adjust non-GCP triangulated points based on the DEM, if
      // provided.
      bool is_gcp = (cnet[ipt].type() == vw::ba::ControlPoint::GroundControlPoint);
      if (have_dem && !is_gcp && dem_xyz_vec.at(ipt) != Vector3(0, 0, 0)) {
        // Update the tri point in param_storage based on the DEM.
        for (int p = 0; p < 3; p++)
          point[p] = dem_xyz_vec.at(ipt)[p];
        // Set the point type, so we can track it later
        cnet[ipt].set_type(vw::ba::ControlPoint::PointFromDem);
        // Update the cnet as well. This will be used later.
        cnet[ipt].set_position(Vector3(point[0], point[1], point[2]));

        // Set the uncertainty to the uncertainty of the DEM 
        double s = opt.heights_from_dem_uncertainty;
        cnet[ipt].set_sigma(Vector3(s, s, s));
      }

      // The observed value for the projection of point with index ipt into
      // the camera with index icam.
      Vector2 observation = (**fiter).m_location;
      Vector2 pixel_sigma = (**fiter).m_scale;

      // This is a bugfix
      if (pixel_sigma != pixel_sigma) // nan check
        pixel_sigma = Vector2(1, 1);

      if (pixel_sigma[0] <= 0.0 || pixel_sigma[1] <= 0.0) {
        // Cannot add a cost function term with non-positive pixel sigma
        param_storage.set_point_outlier(ipt, true);
        continue;
      }

      double p = opt.overlap_exponent;
      if (p > 0 && count_map[ipt] > 2) {
        // Give more weight to points that are seen in more images.
        // This should not be overused. 
        double delta = pow(count_map[ipt] - 1.0, p);
        pixel_sigma /= delta;
      }

      // Apply the weight image
      if (have_weight_image)
        pixel_sigma /= img_wt.child();

      // Need this for --camera-position-weight 
      if (opt.camera_position_weight > 0) {
        pixels_per_cam[icam].push_back(observation);
        tri_points_per_cam[icam].push_back(cnet[ipt].position());
      }
      // For computing pixel reprojection errors
      pixel_sigmas[icam][ipt] = pixel_sigma;

      // Call function to add the appropriate Ceres residual block.
      add_reprojection_residual_block(observation, pixel_sigma, ipt, icam,
                                      param_storage, opt, dist_opts, problem);
      cam_residual_counts[icam] += 1; // Track the number of residual blocks for each camera
      num_pixels_per_cam[icam] += 1;  // Track the number of pixels for each camera

    } // end iterating over points
  } // end iterating over cameras

  return;
}

// Add a soft constraint that ties triangulated points close to their initial positions.
// This is adjusted for GSD.
void addTriConstraint(asp::BaOptions           const& opt,
                      vw::ba::ControlNetwork   const& cnet,
                      asp::CRN                const& crn,
                      std::vector<std::string> const& image_files,
                      std::vector<vw::CamPtr>  const& orig_cams,
                      double tri_weight,
                      std::string cost_function_str,
                      double tri_robust_threshold,
                      // Outputs
                      asp::BaParams  & param_storage,
                      ceres::Problem & problem,
                      int            & num_tri_residuals) {

  // Initialize the output
  num_tri_residuals = 0;

  // Tri weight must be positive
  if (tri_weight <= 0)
    vw::vw_throw(vw::ArgumentErr() << "The triangulation weight must be positive.\n");

  int num_points = param_storage.num_points();
  if ((int)cnet.size() != num_points)
  vw_throw(ArgumentErr() << "Book-keeping error, the size of the control network "
            "must be the same as the number of triangulated points.\n");

  // Add triangulation weight to make each triangulated point not move too far
  std::vector<double> gsds;
  asp::estimateGsdPerTriPoint(image_files, orig_cams, crn, param_storage, gsds);
  for (int ipt = 0; ipt < num_points; ipt++) {
    if (cnet[ipt].type() == vw::ba::ControlPoint::GroundControlPoint ||
        cnet[ipt].type() == vw::ba::ControlPoint::PointFromDem)
      continue; // Skip GCPs and height-from-dem points which have their own constraint

    if (param_storage.get_point_outlier(ipt))
      continue; // skip outliers

    // Use as constraint the triangulated point optimized at the previous
    // iteration. That is on purpose, to ensure that the triangulated point is
    // more accurate than it was at the start of the optimization. For the
    // first iteration, that will be what is read from the cnet. Note how the
    // weight is normalized by the GSD, to make it in pixel coordinates, as
    // the rest of the residuals.
    double * point = param_storage.get_point_ptr(ipt);
    Vector3 observation(point[0], point[1], point[2]);
    double gsd = gsds[ipt];
    if (gsd <= 0 || std::isnan(gsd))
      continue; // GSD calculation failed. Do not use a constraint.

    double s = gsd/tri_weight;
    Vector3 xyz_sigma(s, s, s);

    ceres::CostFunction* cost_function = XYZError::Create(observation, xyz_sigma);
    ceres::LossFunction* loss_function
      = get_loss_function(cost_function_str, tri_robust_threshold);
    problem.AddResidualBlock(cost_function, loss_function, point);

    num_tri_residuals++;
  } // End loop through xyz

} // end adding a triangulation constraint

// Add a cost function meant to tie up to known disparity form left to right
// image and known ground truth reference terrain (option --reference-terrain).
// This was only tested for pinhole cameras. Disparity must be created with
// stereo with the option --unalign-disparity. If there are n images, there must
// be n-1 disparities, from each image to the next.
void addReferenceTerrainCostFunction(
         asp::BaOptions  & opt,
         asp::BaParams       & param_storage,
         ceres::Problem      & problem,
         std::vector<vw::Vector3> & reference_vec,
         std::vector<vw::ImageViewRef<DispPixelT>> & interp_disp) {

  size_t num_cameras = param_storage.num_cameras();

  // Set up a GeoReference object using the datum, it may get modified later
  vw::cartography::GeoReference geo;
  geo.set_datum(opt.datum); // We checked for a datum earlier

  // Load the reference data
  std::vector<vw::Vector3> input_reference_vec;
  std::vector<ImageView<DispPixelT>> disp_vec;
  asp::load_csv_or_dem(opt.csv_format_str, opt.csv_srs, opt.reference_terrain,
                        opt.max_num_reference_points,
                        geo,       // may change
                        input_reference_vec); // output

  if (load_reference_disparities(opt.disparity_list, disp_vec, interp_disp) != num_cameras-1)
    vw_throw(ArgumentErr() << "Expecting one less disparity than there are cameras.\n");

  // Read the image boxes. They are needed to find the GSD per camera.
  std::vector<vw::BBox2i> image_boxes;
  for (int icam = 0; icam < num_cameras; icam++) {
    vw::DiskImageView<float> img(opt.image_files[icam]);
    vw::BBox2i bbox = vw::bounding_box(img);
    image_boxes.push_back(bbox);
  }

  vw::vw_out() << "Setting up the error to the reference terrain.\n";
  vw::TerminalProgressCallback tpc("", "\t--> ");
  tpc.report_progress(0);
  double inc_amount = 1.0/double(input_reference_vec.size());

  reference_vec.clear();
  for (size_t data_col = 0; data_col < input_reference_vec.size(); data_col++) {

    vw::Vector3 reference_xyz = input_reference_vec[data_col];

    // Filter by lonlat box if provided, this is very much recommended
    // to quickly discard most points in the huge reference terrain.
    // Let's hope there is no 360 degree offset when computing
    // the longitude. 
    if (asp::stereo_settings().lon_lat_limit != BBox2(0,0,0,0)) {
      vw::Vector3 llh = geo.datum().cartesian_to_geodetic(reference_xyz);
      vw::Vector2 ll  = subvector(llh, 0, 2);
      if (!asp::stereo_settings().lon_lat_limit.contains(ll)) {
        continue;
      }
    }

    Vector2 left_pred, right_pred;

    // Iterate over the cameras, add a residual for each point and each camera pair.
    for (int icam = 0; icam < num_cameras - 1; icam++) {

      boost::shared_ptr<CameraModel> left_camera  = opt.camera_models[icam];
      boost::shared_ptr<CameraModel> right_camera = opt.camera_models[icam+1];

      try {
        left_pred  = left_camera->point_to_pixel (reference_xyz);
        right_pred = right_camera->point_to_pixel(reference_xyz);
      } catch (const camera::PointToPixelErr& e) {
        continue; // Skip point if there is a projection issue.
      }

      if ((left_pred != left_pred) || (right_pred != right_pred))
        continue; // nan check

      if (!interp_disp[icam].pixel_in_bounds(left_pred))
        continue; // Interp check

      DispPixelT dispPix = interp_disp[icam](left_pred[0], left_pred[1]);
      if (!is_valid(dispPix))
        continue;

      // Check if the current point projects in the cameras
      if (!image_boxes[icam  ].contains(left_pred) ||
          !image_boxes[icam+1].contains(right_pred)) {
        continue;
      }

      Vector2 right_pix = left_pred + dispPix.child();
      if (!image_boxes[icam+1].contains(right_pix))
        continue; // Check offset location too

      if (right_pix != right_pix || norm_2(right_pix - right_pred) > opt.max_disp_error) {
        // Ignore pixels which are too far from where they should be before optimization
        continue;
      }

      // Only the used reference points are stored here
      reference_vec.push_back(reference_xyz);

      // Call function to select the appropriate Ceres residual block to add.
      add_disparity_residual_block(reference_xyz, interp_disp[icam],
                                    icam, icam + 1, // left icam and right icam
                                    param_storage, opt, problem);
    }
    tpc.report_incremental_progress(inc_amount);
  }

  tpc.report_finished();
  vw_out() << "Found " << reference_vec.size() << " reference points in range.\n";
}

// Add a soft constraint to keep the cameras near the original position. 
// Add a combined constraint for all reprojection errors in given camera.
void addCamPosCostFun(asp::BaOptions                          const& opt,
                      asp::BaParams                           const& orig_parameters,
                      std::vector<std::vector<vw::Vector2>>   const& pixels_per_cam,
                      std::vector<std::vector<vw::Vector3>>   const& tri_points_per_cam,
                      std::vector<std::map<int, vw::Vector2>> const& pixel_sigmas,
                      std::vector<vw::CamPtr>                 const& orig_cams,
                      // Outputs
                      asp::BaParams                              & param_storage,
                      ceres::Problem                             & problem,
                      int                                        & num_cam_pos_residuals) {

  num_cam_pos_residuals = 0;

  // Image bboxes
  std::vector<vw::BBox2> bboxes;
  for (size_t i = 0; i < opt.image_files.size(); i++) {
    vw::DiskImageView<float> img(opt.image_files[i]);
    bboxes.push_back(bounding_box(img));
  }

  int num_cameras = param_storage.num_cameras();
  for (int icam = 0; icam < num_cameras; icam++) {

    // There must be as many pixels_per_cam as pixel_sigmas per cam
    if (pixels_per_cam[icam].size() != pixel_sigmas[icam].size())
      vw_throw(ArgumentErr() << "Expecting as many pixels as pixel sigmas per camera.\n");

    // Adjustments to initial and current cameras
    double const* orig_cam_ptr = orig_parameters.get_camera_ptr(icam);
    double * cam_ptr  = param_storage.get_camera_ptr(icam);

    double sum = 0.0;
    int count = 0;
    std::vector<double> pos_wts;
    auto pix_sigma_it = pixel_sigmas[icam].begin();
    for (size_t ipix = 0; ipix < pixels_per_cam[icam].size(); ipix++) {

      vw::Vector2 pixel_obs = pixels_per_cam[icam][ipix];
      vw::Vector3 xyz_obs = tri_points_per_cam[icam][ipix];
      double pixel_sigma = norm_2(pix_sigma_it->second);
      if (pix_sigma_it == pixel_sigmas[icam].end())
        vw::vw_throw(vw::ArgumentErr() << "Out of bounds for pixel sigmas.\n");
      pix_sigma_it++; // Update for the next iteration
      if (pixel_sigma <= 0.0 || std::isnan(pixel_sigma))
        continue;

      double gsd = 0.0;
      try {
        gsd = vw::camera::estimatedGSD(orig_cams[icam].get(), bboxes[icam],
                                       pixel_obs, xyz_obs);
      } catch (...) {
        continue;
      }
      if (gsd <= 0)
        continue;

      // Care with computing the weight
      double position_wt = opt.camera_position_weight / (gsd * pixel_sigma);
      sum += position_wt;
      count++;
      pos_wts.push_back(position_wt);
    }

    // Skip for zero count
    if (count == 0)
      continue;

    // The median weight was shown to be more robust to outliers 
    // than the mean weight.
    double median_wt = vw::math::destructive_median(pos_wts);

    // Based on the CERES loss function formula, adding N loss functions each 
    // with weight w and robust threshold t is equivalent to adding one loss 
    // function with weight sqrt(N)*w and robust threshold sqrt(N)*t.
    double combined_wt  = sqrt(count * 1.0) * median_wt;
    double combined_th = sqrt(count * 1.0) * opt.camera_position_robust_threshold;
    double rotation_wt = 0.0; // This will be handled separately
    ceres::CostFunction* cost_function
        = RotTransError::Create(orig_cam_ptr, rotation_wt, combined_wt);
    ceres::LossFunction* loss_function
       = get_loss_function(opt.cost_function, combined_th);
    problem.AddResidualBlock(cost_function, loss_function, cam_ptr);
    num_cam_pos_residuals++;
  }
}

// Add a ground constraint (GCP or height from DEM)
void addGcpOrDemConstraint(asp::BaBaseOptions const& opt,
                           std::string        const& cost_function_str,
                           bool                      use_llh_error,
                           bool                      fix_gcp_xyz,
                           // Outputs
                           vw::ba::ControlNetwork & cnet,
                           int                    & num_gcp,
                           int                    & num_gcp_or_dem_residuals,
                           asp::BaParams          & param_storage,
                           ceres::Problem         & problem) {

  int num_tri_points  = param_storage.num_points();
  if (num_tri_points != (int)cnet.size())
    vw::vw_throw(vw::ArgumentErr() << "Book-keeping error, the size of the control network "
             << "must equal the number of points.\n");

  num_gcp = 0;
  num_gcp_or_dem_residuals = 0;

  for (int ipt = 0; ipt < num_tri_points; ipt++) {
    if (cnet[ipt].type() != vw::ba::ControlPoint::GroundControlPoint &&
        cnet[ipt].type() != vw::ba::ControlPoint::PointFromDem)
      continue; // Skip non-GCP's and points which do not need special treatment

    if (param_storage.get_point_outlier(ipt))
      continue; // skip outliers

    if (cnet[ipt].type() == vw::ba::ControlPoint::GroundControlPoint)
      num_gcp++;

    vw::Vector3 observation = cnet[ipt].position();
    vw::Vector3 xyz_sigma   = cnet[ipt].sigma();

    ceres::CostFunction* cost_function = NULL;
    if (!use_llh_error) {
      cost_function = asp::XYZError::Create(observation, xyz_sigma);
    } else {
      vw::Vector3 llh_sigma = xyz_sigma;
      // make lat, lon into lon, lat
      std::swap(llh_sigma[0], llh_sigma[1]);
      cost_function = LLHError::Create(observation, llh_sigma, opt.datum);
    }

    // For GCP a robust cost function is not used, as those are assumed to be
    // accurate. This may need to change for GCP produced with dem2gcp. With the
    // --heights-from-dem option, a robust cost function is used, with its own
    // robust threshold.
    ceres::LossFunction* loss_function = NULL;
    if (opt.heights_from_dem != ""           &&
        opt.heights_from_dem_uncertainty > 0 &&
        opt.heights_from_dem_robust_threshold > 0) {
      loss_function
      = get_loss_function(cost_function_str, opt.heights_from_dem_robust_threshold);
    } else {
      loss_function = new ceres::TrivialLoss();
    }
    double * tri_point  = param_storage.get_point_ptr(ipt);
    problem.AddResidualBlock(cost_function, loss_function, tri_point);

    num_gcp_or_dem_residuals++;

    // Ground xyz whose sigma is asp::FIXED_GCP_SIGMA (a tiny positive value) are set to fixed
    double s = asp::FIXED_GCP_SIGMA;
    if (cnet[ipt].type() == vw::ba::ControlPoint::GroundControlPoint &&
        (fix_gcp_xyz || xyz_sigma == vw::Vector3(s, s, s))) {
      cnet[ipt].set_sigma(Vector3(s, s, s)); // will be saved in the ISIS cnet
      problem.SetParameterBlockConstant(tri_point);
    }

  } // End loop through triangulated points

}

} // end namespace asp
