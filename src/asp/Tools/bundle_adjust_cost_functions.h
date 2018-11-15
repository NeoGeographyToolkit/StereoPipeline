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

//=========================================================================
// Cost functions for Ceres

// TODO: We could probably consolidate the following two classes!

/// A Ceres cost function. Templated by the BundleAdjust model. We pass
/// in the observation, the model, and the current camera and point
/// indices. The result is the residual, the difference in the
/// observation and the projection of the point into the camera,
/// normalized by pixel_sigma.
template<class ModelT>
struct BaReprojectionError {
  BaReprojectionError(Vector2 const& observation, Vector2 const& pixel_sigma,
                      ModelT * const ba_model, size_t icam, size_t ipt):
    m_observation(observation),
    m_pixel_sigma(pixel_sigma),
    m_ba_model(ba_model),
    m_icam(icam), m_ipt(ipt){}

  // This is the function that Ceres will call during optimization.
  template <typename T>
  bool operator()(const T* camera, const T* point, T* residuals) const {

    try{

      size_t num_cameras = m_ba_model->num_cameras();
      size_t num_points  = m_ba_model->num_points();
      VW_ASSERT(m_icam < num_cameras, ArgumentErr() << "Out of bounds in the number of cameras.");
      VW_ASSERT(m_ipt  < num_points , ArgumentErr() << "Out of bounds in the number of points." );

      // Copy the input data to structures expected by the BA model.
      // - This class does not optimize intrinsics so just use a null pointer for them.
      typename ModelT::camera_intr_vector_t cam_intr_vec;
      double * intrinsics = NULL; // part of the interface
      (*m_ba_model).concat_extrinsics_intrinsics(camera, intrinsics, cam_intr_vec);

      typename ModelT::point_vector_t  point_vec;
      for (size_t p = 0; p < point_vec.size(); p++)
        point_vec[p] = point[p];

      // Project the current point into the current camera
      Vector2 prediction = (*m_ba_model).cam_pixel(m_ipt, m_icam, cam_intr_vec, point_vec);

      // The error is the difference between the predicted and observed position,
      // normalized by sigma.
      residuals[0] = (prediction[0] - m_observation[0])/m_pixel_sigma[0]; // Input units are pixels
      residuals[1] = (prediction[1] - m_observation[1])/m_pixel_sigma[1];

    } catch (std::exception const& e) {
      // Failed to compute residuals

      Mutex::Lock lock( g_ba_mutex );
      g_ba_num_errors++;
      if (g_ba_num_errors < 100) {
        vw_out(ErrorMessage) << e.what() << std::endl;
      }else if (g_ba_num_errors == 100) {
        vw_out() << "Will print no more error messages about "
                 << "failing to compute residuals.\n";
      }

      residuals[0] = T(1e+20);
      residuals[1] = T(1e+20);
      return false;
    }

    return true;
  }

  // Factory to hide the construction of the CostFunction object from the client code.
  static ceres::CostFunction* Create(Vector2 const& observation,
                                     Vector2 const& pixel_sigma,
                                     ModelT * const ba_model,
                                     size_t icam, // camera index
                                     size_t ipt // point index
                                     ){
    const int NUM_RESIDUALS = 2;
    return (new ceres::NumericDiffCostFunction<BaReprojectionError,
            ceres::CENTRAL, NUM_RESIDUALS, ModelT::camera_params_n, ModelT::point_params_n>
            (new BaReprojectionError(observation, pixel_sigma,
                                     ba_model, icam, ipt)));
  }

  Vector2 m_observation;     ///< The pixel observation for this camera/point pair.
  Vector2 m_pixel_sigma;
  ModelT * const m_ba_model; ///< Pointer to the camera model object.
  size_t m_icam, m_ipt;      ///< This instantiation is always for the same camera/point pair.
};

/// A ceres cost function. Here we float a pinhole camera's intrinsic
/// and extrinsic parameters. The result is the residual, the
/// difference in the observation and the projection of the point into
/// the camera, normalized by pixel_sigma.
struct BaPinholeError {
  BaPinholeError(Vector2 const& observation, Vector2 const& pixel_sigma,
                 BAPinholeModel * const ba_model, size_t icam, size_t ipt, int num_parameters):
    m_observation(observation),
    m_pixel_sigma(pixel_sigma),
    m_ba_model(ba_model),
    m_icam(icam), m_ipt(ipt),
    m_num_parameters(num_parameters) {}

  // Adaptor to work with ceres::DynamicCostFunctions.
  // - Takes array of arrays.
  bool operator()(double const * const * parameters, double * residuals) const {
    VW_ASSERT(m_num_parameters >= 2, ArgumentErr() << "Require at least parameters for camera and point.");
    const double * const camera_ptr     = parameters[0];
    const double * const point_ptr      = parameters[1];
    const double * const focal_ptr      = m_num_parameters >= 3 ? parameters[2] : NULL;
    const double * const center_ptr     = m_num_parameters >= 4 ? parameters[3] : NULL;
    const double * const distortion_ptr = m_num_parameters >= 5 ? parameters[4] : NULL;
    return Evaluation(camera_ptr, point_ptr, focal_ptr, center_ptr, distortion_ptr, residuals);
  }

  // Compute residuals of observing this point with these camera parameters
  bool Evaluation(const double * const camera,
                  const double * const point,
                  const double * const scaled_focal_length,
                  const double * const scaled_optical_center,
                  const double * const scaled_distortion_intrinsics,
                  double       * residuals) const {
    try{
      int num_cameras = m_ba_model->num_cameras();
      int num_points  = m_ba_model->num_points();
      VW_ASSERT(int(m_icam) < num_cameras, ArgumentErr()
                << "Out of bounds in the number of cameras");
      VW_ASSERT(int(m_ipt)  < num_points,  ArgumentErr()
                << "Out of bounds in the number of points" );

      // Copy the input data to structures expected by the BA model
      BAPinholeModel::camera_intr_vector_t cam_intr_vec; // <-- All camera params packed in here!
      BAPinholeModel::point_vector_t       point_vec;

      m_ba_model->concat_extrinsics_intrinsics(camera,
                                               scaled_focal_length,
                                               scaled_optical_center,
                                               scaled_distortion_intrinsics,
                                               cam_intr_vec);
      for (size_t p = 0; p < point_vec.size(); p++)
        point_vec[p] = point[p];

      BAPinholeModel::intrinsic_vector_t orig_intrinsics = m_ba_model->get_intrinsics();

      size_t ncp = BAPinholeModel::camera_params_n;

      if (!m_ba_model->are_intrinsics_constant()) {
        if (orig_intrinsics.size() + ncp != cam_intr_vec.size()) 
          vw_throw(LogicErr() << "Wrong number of intrinsics!");
      }else{
        if (ncp != cam_intr_vec.size()) 
          vw_throw(LogicErr() << "Wrong number of intrinsics!");
      }

      // Multiply the original intrinsics by their relative intrinsics being optimized
      for (size_t intrIter = ncp; intrIter < cam_intr_vec.size(); intrIter++)
        cam_intr_vec[intrIter] *= orig_intrinsics[intrIter - ncp];
      
      // Project the current point into the current camera
      Vector2 prediction = m_ba_model->cam_pixel(m_ipt, m_icam, cam_intr_vec, point_vec);

      // The error is the difference between the predicted and observed position,
      // normalized by sigma.
      residuals[0] = (prediction[0] - m_observation[0])/m_pixel_sigma[0];
      residuals[1] = (prediction[1] - m_observation[1])/m_pixel_sigma[1];

    } catch (const camera::PointToPixelErr& e) {
      // Failed to project into the camera
      residuals[0] = 1e+20;
      residuals[1] = 1e+20;
      return false;
    }
    return true;
  }
  
  // Factory to hide the construction of the CostFunction object from
  // the client code.
  static ceres::CostFunction* Create(Vector2 const& observation,
                                     Vector2 const& pixel_sigma,
                                     BAPinholeModel * const ba_model,
                                     size_t icam, // camera index
                                     size_t ipt // point index
                                     ){
    const int nob = PIXEL_SIZE; // Num observation elements: Column, row
    const int ncp = BAPinholeModel::camera_params_n;
    const int npp = BAPinholeModel::point_params_n;
    const int nf  = BAPinholeModel::focal_length_params_n;
    const int nc  = BAPinholeModel::optical_center_params_n;
    const int num_intrinsics        = ba_model->num_intrinsic_params();

    // DynamicNumericDiffCostFunction is a little new. It doesn't tell the cost
    // function how many parameters are available. So here we are calculating
    // it before hand to tell the cost function.
    static const int kFocalLengthAndPrincipalPoint = 3;
    int num_parameters = 2;
    if (num_intrinsics > 0 && num_intrinsics <= kFocalLengthAndPrincipalPoint) {
      num_parameters += 2;
    } else if (num_intrinsics > kFocalLengthAndPrincipalPoint) {
      num_parameters += 3;
    }

    ceres::DynamicNumericDiffCostFunction<BaPinholeError>* cost_function =
        new ceres::DynamicNumericDiffCostFunction<BaPinholeError>(
            new BaPinholeError(observation, pixel_sigma, ba_model, icam, ipt,
                               num_parameters));
    cost_function->SetNumResiduals(nob);
    cost_function->AddParameterBlock(ncp); // Cam params
    cost_function->AddParameterBlock(npp); // Point params

    if (num_intrinsics == 0) {
      // This is the special case that we are not solving camera intrinsics.
      return cost_function;
    }

    cost_function->AddParameterBlock(nf); // Focus params
    cost_function->AddParameterBlock(nc); // Optical center params

    if (num_intrinsics < kFocalLengthAndPrincipalPoint) {
      vw_throw(LogicErr()
               << "bundle_adjust.cc not set up for 1 or 2 intrinsics params!");
    } else if (num_intrinsics == kFocalLengthAndPrincipalPoint) {
      // Early exit if we want to solve only for focal length and optical
      // center.
      return cost_function;
    }

    // Larger intrinsics also mean we are solving for lens distortion. We
    // support a variable number of parameters here.
    cost_function->AddParameterBlock(num_intrinsics -
                                     kFocalLengthAndPrincipalPoint);
    return cost_function;
  }  // End function Create

  Vector2 m_observation;
  Vector2 m_pixel_sigma;
  BAPinholeModel * const m_ba_model;
  size_t m_icam, m_ipt;
  int m_num_parameters;
};


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

// TODO: consolidate these two classes into a single one!  Don't need both!

/// A ceres cost function. The residual is the difference between the
/// original camera center and the current (floating) camera center.
/// This cost function prevents the cameras from straying too far from
/// their starting point.
template<class ModelT>
struct CamError {
  typedef typename ModelT::camera_vector_t CamVecT;

  CamError(CamVecT const& orig_cam, double weight):
    m_orig_cam(orig_cam), m_weight(weight){}

  template <typename T>
  bool operator()(const T* cam_vec, T* residuals) const {

    const double POSITION_WEIGHT = 1e-2;  // Units are meters.  Don't lock the camera down too tightly.
    const double ROTATION_WEIGHT = 5e1;   // Units are in radianish range 
    
    //std::cout << "CamError: ";
    for (size_t p = 0; p < 3; p++) {
      residuals[p] = POSITION_WEIGHT*m_weight*(cam_vec[p] - m_orig_cam[p]);
      //std::cout << residuals[p] << ",  ";
    }
    //std::cout << "  ::  ";
    for (size_t p = 3; p < m_orig_cam.size(); p++) {
      residuals[p] = ROTATION_WEIGHT*m_weight*(cam_vec[p] - m_orig_cam[p]);
      //std::cout << residuals[p] << ",  ";
    }
    //std::cout << std::endl;

    return true;
  }

  // Factory to hide the construction of the CostFunction object from
  // the client code.
  static ceres::CostFunction* Create(CamVecT const& orig_cam, double weight){
    return (new ceres::AutoDiffCostFunction<CamError, ModelT::camera_params_n,
            ModelT::camera_params_n>
            (new CamError(orig_cam, weight)));
  }

  CamVecT m_orig_cam;
  double m_weight;
};

/// A ceres cost function. The residual is the rotation + translation
/// vector difference, each multiplied by a weight. Hence, a larger
/// rotation weight will result in less rotation change in the final
/// result, etc. This is somewhat different than CamError as there is no
/// penalty here for this cost function going very large, the scaling is
/// different, and there is finer-grained control. 
template<class ModelT>
struct RotTransError {
  typedef typename ModelT::camera_vector_t CamVecT;

  RotTransError(CamVecT const& orig_cam, double rotation_weight, double translation_weight):
    m_orig_cam(orig_cam), m_rotation_weight(rotation_weight),
    m_translation_weight(translation_weight) {}

  template <typename T>
  bool operator()(const T* const cam_vec, T* residuals) const {

    for (size_t p = 0; p < 3; p++) {
      residuals[p] = m_translation_weight*(cam_vec[p] - m_orig_cam[p]);
    }

    for (size_t p = 3; p < m_orig_cam.size(); p++) {
      residuals[p] = m_rotation_weight*(cam_vec[p] - m_orig_cam[p]);
    }

    return true;
  }

  // Factory to hide the construction of the CostFunction object from
  // the client code.
  static ceres::CostFunction* Create(CamVecT const& orig_cam,
                                     double rotation_weight, double translation_weight){
    return (new ceres::AutoDiffCostFunction<RotTransError, ModelT::camera_params_n,
            ModelT::camera_params_n>
            (new RotTransError(orig_cam, rotation_weight, translation_weight)));

  }

  CamVecT m_orig_cam;
  double m_rotation_weight, m_translation_weight;
};

// TODO: Shrink this!!!!

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
                 BAPinholeModel const& ba_model, size_t left_icam,
                 size_t right_icam, int num_parameters)
      : m_reference_xyz(reference_xyz),
        m_interp_disp(interp_disp),
        m_ba_model(ba_model),
        m_left_icam(left_icam),
        m_right_icam(right_icam),
        m_num_parameters(num_parameters) {}

  // Adaptor to work with ceres::DynamicCostFunctions.
  bool operator()(double const* const* parameters, double* residuals) const {
    VW_ASSERT(m_num_parameters >= 2,
              ArgumentErr()
                  << "Require at least parameters for camera and point.");
    const double* const left_camera_ptr  = parameters[0];
    const double* const right_camera_ptr = parameters[1];
    const double* const focal_ptr        = m_num_parameters >= 3 ? parameters[2] : NULL;
    const double* const center_ptr       = m_num_parameters >= 4 ? parameters[3] : NULL;
    const double* const distortion_ptr   = m_num_parameters >= 5 ? parameters[4] : NULL;
    return Evaluation(left_camera_ptr, right_camera_ptr, focal_ptr, center_ptr,
                      distortion_ptr, residuals);
  }

  /// Compute residuals of observing this point with these camera parameters
  bool Evaluation(const double * const left_camera,
                  const double * const right_camera,
                  const double * const scaled_focal_length,
                  const double * const scaled_optical_center,
                  const double * const scaled_distortion_intrinsics,
                  double       * residuals) const {
    try{
      int num_cameras = m_ba_model.num_cameras();
      VW_ASSERT(int(m_left_icam) < num_cameras, ArgumentErr()
                << "Out of bounds in the number of cameras");
      VW_ASSERT(int(m_right_icam) < num_cameras, ArgumentErr()
                << "Out of bounds in the number of cameras");

      // Copy the input data to structures expected by the BA model
      BAPinholeModel::camera_intr_vector_t left_cam_intr_vec, right_cam_intr_vec;
      BAPinholeModel::point_vector_t       point_vec;

      m_ba_model.concat_extrinsics_intrinsics(left_camera,
                                              scaled_focal_length,
                                              scaled_optical_center,
                                              scaled_distortion_intrinsics,
                                              left_cam_intr_vec);
      m_ba_model.concat_extrinsics_intrinsics(right_camera,
                                              scaled_focal_length, scaled_optical_center,
                                              scaled_distortion_intrinsics,
                                              right_cam_intr_vec);

      VW_ASSERT(m_reference_xyz.size() == point_vec.size(), ArgumentErr()
                << "Inconsistency in point size.");

      for (size_t p = 0; p < point_vec.size(); p++)
        point_vec[p] = m_reference_xyz[p];

      // Original intrinsics
      BAPinholeModel::intrinsic_vector_t orig_intrinsics = m_ba_model.get_intrinsics();
      size_t ncp = BAPinholeModel::camera_params_n;

      if (!m_ba_model.are_intrinsics_constant()) {
        if (orig_intrinsics.size() + ncp != left_cam_intr_vec.size()) 
          vw_throw(LogicErr() << "Wrong number of intrinsics!");
      }else{
        if (ncp != left_cam_intr_vec.size()) 
          vw_throw(LogicErr() << "Wrong number of intrinsics!");
      }

      // So far we had scaled intrinsics, very close to 1. Multiply them
      // by the original intrinsics, to get the actual intrinsics. 
      for (size_t intrIter = ncp; intrIter < left_cam_intr_vec.size(); intrIter++) {
        left_cam_intr_vec[intrIter]  *= orig_intrinsics[intrIter - ncp];
        right_cam_intr_vec[intrIter] *= orig_intrinsics[intrIter - ncp];
      }

      // Project the current point into the current camera
      Vector2 left_prediction = m_ba_model.cam_pixel(0, m_left_icam,
                                                     left_cam_intr_vec, point_vec);
      Vector2 right_prediction = m_ba_model.cam_pixel(0, m_right_icam,
                                                      right_cam_intr_vec, point_vec);

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

  // Factory to hide the construction of the CostFunction object from
  // the client code.
  static ceres::CostFunction* Create(
      Vector3 const& reference_xyz, ImageViewRef<DispPixelT> const& interp_disp,
      BAPinholeModel const& ba_model, size_t left_icam, size_t right_icam) {
    const int nob = PIXEL_SIZE;  // Num observation elements: Column, row
    const int ncp = BAPinholeModel::camera_params_n;
    const int nf  = BAPinholeModel::focal_length_params_n;
    const int nc  = BAPinholeModel::optical_center_params_n;
    const int num_intrinsics = ba_model.num_intrinsic_params();

    //TODO: Clarify!
    // DynamicNumericDiffCostFunction is a little new. It doesn't tell the cost
    // function how many parameters are available. So here we are calculating
    // it before hand to tell the cost function.
    static const int kFocalLengthAndPrincipalPoint = 3;
    int num_parameters = 2;
    if (num_intrinsics > 0 && num_intrinsics <= kFocalLengthAndPrincipalPoint) {
      num_parameters += 2;
    } else if (num_intrinsics > kFocalLengthAndPrincipalPoint) {
      num_parameters += 3;
    }

    ceres::DynamicNumericDiffCostFunction<BaDispXyzError>* cost_function =
        new ceres::DynamicNumericDiffCostFunction<BaDispXyzError>(
            new BaDispXyzError(reference_xyz, interp_disp, ba_model, left_icam,
                               right_icam, num_parameters));
    cost_function->SetNumResiduals(nob);
    cost_function->AddParameterBlock(ncp); // Left cam params
    cost_function->AddParameterBlock(ncp); // Right cam params

    if (num_intrinsics == 0) {
      // This is the special case that we are not solving camera intrinsics.
      return cost_function;
    }

    cost_function->AddParameterBlock(nf); // Focal length
    cost_function->AddParameterBlock(nc); // Optical center

    if (num_intrinsics < kFocalLengthAndPrincipalPoint) {
      vw_throw(LogicErr() << "bundle_adjust.cc not set up for 1 or 2 intrinsics params!");
    } else if (num_intrinsics == kFocalLengthAndPrincipalPoint) {
      // Early exit if we want to solve only for focal length and optical center.
      return cost_function;
    }

    // Larger intrinsics also mean we are solving for lens distortion. We
    // support a variable number of parameters here.
    cost_function->AddParameterBlock(num_intrinsics - kFocalLengthAndPrincipalPoint);
    return cost_function;
  }  // End function Create

  Vector3 m_reference_xyz;
  ImageViewRef<DispPixelT> const& m_interp_disp;
  BAPinholeModel const& m_ba_model;
  size_t m_left_icam, m_right_icam;
  int    m_num_parameters;
};


#endif // __ASP_TOOLS_BUNDLEADJUST_COST_FUNCTIONS_H__
