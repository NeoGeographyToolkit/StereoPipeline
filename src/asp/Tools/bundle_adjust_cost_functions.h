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


#ifndef __ASP_TOOLS_BUNDLEADJUST_COST_FUNCTIONS_H__
#define __ASP_TOOLS_BUNDLEADJUST_COST_FUNCTIONS_H__

/**
  This file breaks out the Ceres cost functions used by bundle_adjust.cc.
*/


#include <vw/Camera/CameraUtilities.h>
#include <asp/Core/Macros.h>
#include <asp/Core/StereoSettings.h>
#include <asp/Camera/OpticalBarModel.h>


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

using namespace vw;
using namespace vw::camera;

const size_t PIXEL_SIZE = 2;

typedef PixelMask< Vector<float, 2> > DispPixelT;

/// Used to accumulate the number of reprojection errors in bundle adjustment.
int g_ba_num_errors = 0;
Mutex g_ba_mutex;

// TODO: Pass these properly
double g_max_disp_error = -1.0, g_reference_terrain_weight = 1.0;


//=====================================================================

/// Simple base class for unpacking Ceres parameter blocks into
///  a camera model which can do point projections.
class CeresBundleModelBase {
public:

  // These are the same for every camera.
  int num_point_params() const {return 3;}
  int num_pose_params () const {return 6;}

  /// This is for all camera parameters other than the pose parameters.
  /// - These can be spread out across multiple parameter blocks.
  virtual int num_intrinsic_params() const = 0;

  int num_params() const {
    return num_point_params() + num_pose_params() + num_intrinsic_params();
  }

  /// Return the number of Ceres input parameter blocks.
  virtual int num_parameter_blocks() const = 0;

  /// Return the size of each parameter block.
  /// - These should sum up to equal num_params.
  /// - The first block is always the point block (3) and
  ///   the second block is always the pose block (6).
  virtual std::vector<int> get_block_sizes() const {
    std::vector<int> result(2);
    result[0] = num_point_params();
    result[1] = num_pose_params();
    return result;
  }

  /// Read in all of the parameters and generate an output pixel observation.
  /// - Throws if the point does not project in to the camera.
  virtual vw::Vector2 evaluate(std::vector<double const*> const param_blocks) const = 0;
  
}; // End class CeresBundleModelBase



/// Simple wrapper for the vw::camera::AdjustedCameraModel class with a 
/// preconfigured underlying camera.  Only uses translation and rotation.
/// - Just vary the six camera adjustment parameters which are all in 
///   a single parameter block.
class AdjustedCameraBundleModel: public CeresBundleModelBase {
public:

  AdjustedCameraBundleModel(boost::shared_ptr<vw::camera::CameraModel> cam) 
    : m_underlying_camera(cam) {}

  virtual int num_intrinsic_params() const {return 0;}

  /// Return the number of Ceres input parameter blocks.
  /// - (camera), (point)
  virtual int num_parameter_blocks() const {return 2;}

  /// Read in all of the parameters and compute the residuals.
  virtual vw::Vector2 evaluate(std::vector<double const*> const param_blocks) const {

    double const* raw_point = param_blocks[0];
    double const* raw_pose  = param_blocks[1];

    // Read the point location and camera information from the raw arrays.
    Vector3          point(raw_point[0], raw_point[1], raw_point[2]);
    CameraAdjustment correction(raw_pose);

    //std::cout << "point2pixel with correction position = " << correction.position()
    //              << ", pose = " << correction.pose() << std::endl;
    
    vw::camera::AdjustedCameraModel cam(m_underlying_camera,
                                        correction.position(),
                                        correction.pose());
    return cam.point_to_pixel(point);
  }

private:

  /// This camera will be adjusted by the input parameters.
  boost::shared_ptr<vw::camera::CameraModel> m_underlying_camera;

}; // End class CeresBundleModelBase



/// "Full service" pinhole model which solves for all desired camera parameters.
/// - If the current run does not want to solve for everything, those parameter
///   blocks should be set as constant so that Ceres does not change them.
class PinholeBundleModel: public CeresBundleModelBase {
public:

  PinholeBundleModel(boost::shared_ptr<vw::camera::PinholeModel> cam)
    : m_underlying_camera(cam) {}

  /// The number of lens distortion parametrs.
  int num_distortion_params() const {
    vw::Vector<double> lens_params = m_underlying_camera->lens_distortion()->distortion_parameters();
    return lens_params.size();
  }

  virtual int num_intrinsic_params() const {
    return 3 + num_distortion_params(); //Center, focus, and lens distortion.
  }

  /// Return the number of Ceres input parameter blocks.
  /// - (camera), (point), (center), (focus), (lens distortion)
  virtual int num_parameter_blocks() const {return 5;}

  virtual std::vector<int> get_block_sizes() const {
    std::vector<int> result = CeresBundleModelBase::get_block_sizes();
    result.push_back(2); // Center
    result.push_back(1); // Focus
    result.push_back(num_distortion_params());
    return result;
  }

  /// Read in all of the parameters and compute the residuals.
  virtual vw::Vector2 evaluate(std::vector<double const*> const param_blocks) const {

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
    boost::shared_ptr<LensDistortion> distortion = m_underlying_camera->lens_distortion()->copy();
    vw::Vector<double> lens = distortion->distortion_parameters();
    for (size_t i=0; i<lens.size(); ++i)
      lens[i] *= raw_lens[i];
    distortion->set_distortion_parameters(lens);

    // Duplicate the input camera model with the pose, focus, center, and lens updated.
    vw::camera::PinholeModel cam(correction.position(),
                                 correction.pose().rotation_matrix(),
                                 focus, focus, // focal lengths
                                 center_x, center_y, // pixel offsets
                                 distortion.get(),
                                 m_underlying_camera->pixel_pitch());

    // Project the point into the camera.
    Vector2 pixel = cam.point_to_pixel_no_check(point);


    // Try to do the error check for point_to_pixel, but the RPC lens distortion
    //  model does not support it (without a lot of extra computation time) so if we
    //  can't do the check we just have to live without it.
    const double ERROR_THRESHOLD = 0.01;
    double diff = 0.0;
    try {
      Vector3 pixel_vector = cam.pixel_to_vector(pixel);
      Vector3 phys_vector  = normalize(point - cam.camera_center());
      diff = norm_2(pixel_vector - phys_vector);
    }
    catch (...){}
    VW_ASSERT(diff < ERROR_THRESHOLD, vw::camera::PointToPixelErr());

    return pixel;
  }

private:

  // TODO: Cache the constructed camera to save time when just the point changes!

  // TODO: Make const
  /// This camera is used for all of the intrinsic values.
  boost::shared_ptr<vw::camera::PinholeModel> m_underlying_camera;

}; // End class PinholeBundleModel



/// "Full service" pinhole model which solves for all desired camera parameters.
/// - If the current run does not want to solve for everything, those parameter
///   blocks should be set as constant so that Ceres does not change them.
class OpticalBarBundleModel: public CeresBundleModelBase {
public:

  OpticalBarBundleModel(boost::shared_ptr<asp::camera::OpticalBarModel> cam)
    : m_underlying_camera(cam) {}


  virtual int num_intrinsic_params() const {
    return 5; // Center, focus, scan rate, and speed.
  }

  /// Return the number of Ceres input parameter blocks.
  /// - (camera), (point), (center), (focus), (lens distortion)
  virtual int num_parameter_blocks() const {return 5;}

  virtual std::vector<int> get_block_sizes() const {
    std::vector<int> result = CeresBundleModelBase::get_block_sizes();
    result.push_back(2); // Center
    result.push_back(1); // Focus
    result.push_back(2); // MCF and speed
    return result;
  }

  /// Read in all of the parameters and compute the residuals.
  virtual vw::Vector2 evaluate(std::vector<double const*> const param_blocks) const {

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
    double scan_rate = raw_intrin[0] * m_underlying_camera->get_scan_rate();
    double mcf       = raw_intrin[0] * m_underlying_camera->get_motion_compensation();
    double speed     = raw_intrin[1] * m_underlying_camera->get_speed();

    // Duplicate the input camera model with the pose, focus, center, speed, and MCF updated.
    asp::camera::OpticalBarModel cam(m_underlying_camera->get_image_size(),
                                     vw::Vector2(center_x, center_y),
                                     m_underlying_camera->get_pixel_size(),
                                     focus,
                                     m_underlying_camera->get_scan_angle(),
                                     scan_rate,
                                     m_underlying_camera->get_scan_dir(),
                                     m_underlying_camera->get_forward_tilt(),
                                     correction.position(),
                                     correction.pose().axis_angle(),
                                     speed,  mcf);

    // Project the point into the camera.
    return cam.point_to_pixel(point);
  }

private:

  // TODO: Cache the constructed camera to save time when just the point changes!

  // TODO: Make const
  /// This camera is used for all of the intrinsic values.
  boost::shared_ptr<asp::camera::OpticalBarModel> m_underlying_camera;

}; // End class PinholeBundleModel



//=========================================================================
// Cost functions for Ceres



/// A Ceres cost function. We pass in the observation and the model.
///  The result is the residual, the difference in the observation 
///  and the projection of the point into the camera, normalized by pixel_sigma.
struct BaReprojectionError {
  BaReprojectionError(Vector2 const& observation, Vector2 const& pixel_sigma,
                      boost::shared_ptr<CeresBundleModelBase> camera_wrapper):
    m_observation(observation),
    m_pixel_sigma(pixel_sigma),
    m_num_param_blocks(camera_wrapper->num_parameter_blocks()),
    m_camera_wrapper(camera_wrapper)
    {}

  // Call to work with ceres::DynamicCostFunctions.
  // - Takes array of arrays.
  bool operator()(double const * const * parameters, double * residuals) const {

    try {
      // Unpack the parameter blocks
      std::vector<double const*> param_blocks(m_num_param_blocks);
      for (size_t i=0; i<m_num_param_blocks; ++i) {
        param_blocks[i] = parameters[i];
        //std::cout << "param block " << i << " = " << param_blocks[i]
        //          << ", [0] = " << param_blocks[i][0] << std::endl;
      }

      // Use the camera model wrapper to handle all of the parameter blocks.
      Vector2 prediction = m_camera_wrapper->evaluate(param_blocks);

      //std::cout << "Got prediction " << prediction << std::endl;

      // The error is the difference between the predicted and observed position,
      // normalized by sigma.
      residuals[0] = (prediction[0] - m_observation[0])/m_pixel_sigma[0]; // Input units are pixels
      residuals[1] = (prediction[1] - m_observation[1])/m_pixel_sigma[1];

      //std::cout << "Got residuals " << residuals[0] << ", " << residuals[1] << std::endl;

    } catch (std::exception const& e) { // TODO: Catch only projection errors?
      // Failed to compute residuals

      Mutex::Lock lock( g_ba_mutex );
      g_ba_num_errors++;
      if (g_ba_num_errors < 100) {
        vw_out(ErrorMessage) << e.what() << std::endl;
      }else if (g_ba_num_errors == 100) {
        vw_out() << "Will print no more error messages about "
                 << "failing to compute residuals.\n";
      }

      residuals[0] = 1e+20;
      residuals[1] = 1e+20;
      return false;
    }
    return true;
  }


  // Factory to hide the construction of the CostFunction object from the client code.
  static ceres::CostFunction* Create(Vector2 const& observation,
                                     Vector2 const& pixel_sigma,
                                     boost::shared_ptr<CeresBundleModelBase> camera_wrapper){
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

private:
  Vector2 m_observation;     ///< The pixel observation for this camera/point pair.
  Vector2 m_pixel_sigma;
  size_t  m_num_param_blocks;
  boost::shared_ptr<CeresBundleModelBase> m_camera_wrapper; ///< Pointer to the camera model object.

}; // End class BaReprojectionError




/// A ceres cost function. Here we float two pinhole camera's
/// intrinsic and extrinsic parameters. We take as input a reference
/// xyz point and a disparity from left to right image. The
/// error metric is the following: The reference xyz point is projected in the
/// left image. It is mapped via the disparity to the right
/// image. There, the residual error is the difference between that
/// pixel and the pixel obtained by projecting the xyz point
/// straight into the right image.
struct BaDispXyzError {
  BaDispXyzError(Vector3 const& reference_xyz,
                 ImageViewRef<DispPixelT> const& interp_disp,
                 boost::shared_ptr<CeresBundleModelBase> left_camera_wrapper,
                 boost::shared_ptr<CeresBundleModelBase> right_camera_wrapper,
                 bool is_pinhole, // Would like to remove these!
                 IntrinsicOptions intrin_opt)
      : m_reference_xyz(reference_xyz),
        m_interp_disp  (interp_disp  ),
        m_num_left_param_blocks (left_camera_wrapper->num_parameter_blocks ()),
        m_num_right_param_blocks(right_camera_wrapper->num_parameter_blocks()),
        m_left_camera_wrapper   (left_camera_wrapper ),
        m_right_camera_wrapper  (right_camera_wrapper),
        m_is_pinhole(is_pinhole),
        m_intrin_opt(intrin_opt)   {}

  // Adaptor to work with ceres::DynamicCostFunctions.
  bool operator()(double const* const* parameters, double* residuals) const {

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
            residuals[it] *= g_reference_terrain_weight;
        }
      }

      // TODO: Think more of what to do below. The hope is that the robust cost
      // function will take care of big residuals graciously.
      if (!good_ans) {
        // Failed to find the residuals
        for (size_t it = 0; it < 2; it++) 
          residuals[it] = g_max_disp_error * g_reference_terrain_weight;
        return true;
      }

    } catch (const camera::PointToPixelErr& e) {
      // Failed to project into the camera
      for (size_t it = 0; it < 2; it++) 
        residuals[it] = g_max_disp_error * g_reference_terrain_weight;
      return true;
    }
    return true;
  }


  // TODO: Should this logic live somewhere else?
  /// Create the list of residual pointers for pinhole images.
  /// - Extra logic is needed to avoid duplicate pointers.
  static void get_residual_pointers(BAParamStorage &param_storage,
                                    int left_cam_index, int right_cam_index,
                                    bool is_pinhole,
                                    IntrinsicOptions const& intrin_opt,
                                    std::vector<double*> &residual_ptrs) {
   
    double* left_camera  = param_storage.get_camera_ptr(left_cam_index );
    double* right_camera = param_storage.get_camera_ptr(right_cam_index);
    residual_ptrs.clear();
    if (is_pinhole) {
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
      if (!intrin_opt.center_shared    ) residual_ptrs.push_back(right_center    );
      if (!intrin_opt.focus_shared     ) residual_ptrs.push_back(right_focus     );
      if (!intrin_opt.distortion_shared) residual_ptrs.push_back(right_distortion);
    }
    else { // This handles the generic camera case.
      residual_ptrs.push_back(left_camera );
      residual_ptrs.push_back(right_camera);
    }
    return;
  }
  
  void unpack_residual_pointers(double const* const* parameters,
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
    if (!m_is_pinhole) {
      // Unpack everything from the right block in order.
      for (size_t i=1; i<m_num_right_param_blocks; ++i) {
        right_param_blocks[i] = parameters[index];
        ++index;
      }
    } else { // Pinhole case, handle shared intrinsics.
      right_param_blocks[1] = parameters[index]; // Pose and position
      ++index;
      if (m_intrin_opt.center_shared)
        right_param_blocks[2] = left_param_blocks[2];
      else {
        right_param_blocks[2] = parameters[index];
        ++index;
      }
      if (m_intrin_opt.focus_shared)
        right_param_blocks[3] = left_param_blocks[3];
      else {
        right_param_blocks[3] = parameters[index];
        ++index;
      }
      if (m_intrin_opt.distortion_shared)
        right_param_blocks[4] = left_param_blocks[4];
      else {
        right_param_blocks[4] = parameters[index];
        ++index;
      }
    } // End pinhole case
  }


  // Factory to hide the construction of the CostFunction object from
  // the client code.
  static ceres::CostFunction* Create(
      Vector3 const& reference_xyz, ImageViewRef<DispPixelT> const& interp_disp,
      boost::shared_ptr<CeresBundleModelBase> left_camera_wrapper,
      boost::shared_ptr<CeresBundleModelBase> right_camera_wrapper,
      bool is_pinhole, IntrinsicOptions intrin_opt = IntrinsicOptions()) {

    const int NUM_RESIDUALS = 2;

    ceres::DynamicNumericDiffCostFunction<BaDispXyzError>* cost_function =
        new ceres::DynamicNumericDiffCostFunction<BaDispXyzError>(
            new BaDispXyzError(reference_xyz, interp_disp, 
                               left_camera_wrapper, right_camera_wrapper,
                               is_pinhole, intrin_opt));

    // The residual size is always the same.
    cost_function->SetNumResiduals(NUM_RESIDUALS);

    // Add all of the blocks for each camera, except for the first (point)
    // block which is provided at creation time.
    std::vector<int> block_sizes = left_camera_wrapper->get_block_sizes();
    for (size_t i=1; i<block_sizes.size(); ++i) {
      cost_function->AddParameterBlock(block_sizes[i]);
    }
    block_sizes = right_camera_wrapper->get_block_sizes();
    if (!is_pinhole) {
      for (size_t i=1; i<block_sizes.size(); ++i) {
        cost_function->AddParameterBlock(block_sizes[i]);
      }
    } else { // Pinhole handling
      if (block_sizes.size() != 5)
        vw_throw(LogicErr() << "Error: Pinhole camera model parameter number error!");
      cost_function->AddParameterBlock(block_sizes[1]); // The camera position/pose
      if (!intrin_opt.center_shared    ) cost_function->AddParameterBlock(block_sizes[2]);
      if (!intrin_opt.focus_shared     ) cost_function->AddParameterBlock(block_sizes[3]);
      if (!intrin_opt.distortion_shared) cost_function->AddParameterBlock(block_sizes[4]);
    }
    return cost_function;
  }  // End function Create

  Vector3 m_reference_xyz;
  ImageViewRef<DispPixelT> const& m_interp_disp;
  size_t m_num_left_param_blocks, m_num_right_param_blocks;
  // TODO: Make constant!
  boost::shared_ptr<CeresBundleModelBase> m_left_camera_wrapper;
  boost::shared_ptr<CeresBundleModelBase> m_right_camera_wrapper;

  // Would like to not have these two!
  bool m_is_pinhole;
  IntrinsicOptions m_intrin_opt;
};


//===================================================================

/// A ceres cost function. The residual is the difference between the
/// observed 3D point and the current (floating) 3D point, normalized by
/// xyz_sigma. Used only for ground control points.
struct XYZError {
  XYZError(Vector3 const& observation, Vector3 const& xyz_sigma):
    m_observation(observation), m_xyz_sigma(xyz_sigma){}

  template <typename T>
  bool operator()(const T* point, T* residuals) const {
    for (size_t p = 0; p < m_observation.size(); p++)
      residuals[p] = (point[p] - m_observation[p])/m_xyz_sigma[p]; // Input units are meters

    return true;
  }

  // Factory to hide the construction of the CostFunction object from
  // the client code.
  static ceres::CostFunction* Create(Vector3 const& observation,
                                     Vector3 const& xyz_sigma){
    return (new ceres::AutoDiffCostFunction<XYZError, 3, 3>
            (new XYZError(observation, xyz_sigma)));
  }

  Vector3 m_observation;
  Vector3 m_xyz_sigma;
};

/// A ceres cost function. The residual is the difference between the
/// observed 3D point lon-lat-height, and the current (floating) 3D
/// point lon-lat-height, normalized by sigma. Used only for
/// ground control points. This has the advantage, unlike
/// XYZError, that when the height is not known reliably,
/// but lon-lat is, we can, in the GCP file, assign a bigger
/// sigma to the latter.
struct LLHError {
  LLHError(Vector3 const& observation_xyz, Vector3 const& sigma, vw::cartography::Datum & datum):
    m_observation_xyz(observation_xyz), m_sigma(sigma), m_datum(datum){}

  template <typename T>
  bool operator()(const T* point, T* residuals) const {
    Vector3 observation_llh, point_xyz, point_llh;
    for (size_t p = 0; p < m_observation_xyz.size(); p++) {
      point_xyz[p] = double(point[p]);
    }

    point_llh       = m_datum.cartesian_to_geodetic(point_xyz);
    observation_llh = m_datum.cartesian_to_geodetic(m_observation_xyz);

    for (size_t p = 0; p < m_observation_xyz.size(); p++) 
      residuals[p] = (point_llh[p] - observation_llh[p])/m_sigma[p]; // Input units are meters

    return true;
  }

  // Factory to hide the construction of the CostFunction object from
  // the client code.
  static ceres::CostFunction* Create(Vector3 const& observation_xyz,
                                     Vector3 const& sigma,
                                     vw::cartography::Datum & datum){

    return (new ceres::NumericDiffCostFunction<LLHError, ceres::CENTRAL, 3, 3>
            (new LLHError(observation_xyz, sigma, datum)));
  }

  Vector3 m_observation_xyz;
  Vector3 m_sigma;
  vw::cartography::Datum m_datum;
};


/// A ceres cost function. The residual is the difference between the
/// original camera center and the current (floating) camera center.
/// This cost function prevents the cameras from straying too far from
/// their starting point.
struct CamError {

  CamError(double const* orig_cam, double weight):
    m_orig_cam(DATA_SIZE), m_weight(weight){
      for (int i=0; i<DATA_SIZE; ++i)
        m_orig_cam[i] = orig_cam[i];
    }

  template <typename T>
  bool operator()(const T* cam_vec, T* residuals) const {

    const double POSITION_WEIGHT = 1e-2;  // Units are meters.  Don't lock the camera down too tightly.
    const double ROTATION_WEIGHT = 5e1;   // Units are in radianish range 

    for (size_t p = 0; p < DATA_SIZE/2; p++) {
      residuals[p] = POSITION_WEIGHT*m_weight*(cam_vec[p] - m_orig_cam[p]);
    }
    for (size_t p = DATA_SIZE/2; p < DATA_SIZE; p++) {
      residuals[p] = ROTATION_WEIGHT*m_weight*(cam_vec[p] - m_orig_cam[p]);
    }

    return true;
  }

  // Factory to hide the construction of the CostFunction object from the client code.
  static ceres::CostFunction* Create(const double *const orig_cam, double weight){
    return (new ceres::AutoDiffCostFunction<CamError, DATA_SIZE, DATA_SIZE>
            (new CamError(orig_cam, weight)));
  }

private:

  // The camera must be represented by a six element array.
  static const int DATA_SIZE = 6;

  std::vector<double> m_orig_cam;
  double m_weight;
};

/// A ceres cost function. The residual is the rotation + translation
/// vector difference, each multiplied by a weight. Hence, a larger
/// rotation weight will result in less rotation change in the final
/// result, etc. This is somewhat different than CamError as there is no
/// penalty here for this cost function going very large, the scaling is
/// different, and there is finer-grained control. 
struct RotTransError {

  RotTransError(double const* orig_cam, double rotation_weight, double translation_weight):
    m_orig_cam(DATA_SIZE), m_rotation_weight(rotation_weight),
    m_translation_weight(translation_weight) {
    for (int i=0; i<DATA_SIZE; ++i)
        m_orig_cam[i] = orig_cam[i];
    }

  template <typename T>
  bool operator()(const T* cam_vec, T* residuals) const {

    for (size_t p = 0; p < DATA_SIZE/2; p++) {
      residuals[p] = m_translation_weight*(cam_vec[p] - m_orig_cam[p]);
    }

    for (size_t p = DATA_SIZE/2; p < DATA_SIZE; p++) {
      residuals[p] = m_rotation_weight*(cam_vec[p] - m_orig_cam[p]);
    }

    return true;
  }

  // Factory to hide the construction of the CostFunction object from
  // the client code.
  static ceres::CostFunction* Create(const double *const orig_cam,
                                     double rotation_weight, double translation_weight){
    return (new ceres::AutoDiffCostFunction<RotTransError, DATA_SIZE, DATA_SIZE>
            (new RotTransError(orig_cam, rotation_weight, translation_weight)));

  }

private:

  // The camera must be represented by a six element array.
  static const int DATA_SIZE = 6;

  std::vector<double> m_orig_cam;
  double m_rotation_weight, m_translation_weight;
};


#endif // __ASP_TOOLS_BUNDLEADJUST_COST_FUNCTIONS_H__

