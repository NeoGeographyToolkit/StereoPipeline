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

#include <asp/Core/Macros.h>
#include <asp/Sessions/ResourceLoader.h>
#include <asp/Sessions/StereoSession.h>
#include <asp/Sessions/StereoSessionFactory.h>
#include <asp/Core/StereoSettings.h>
#include <asp/Core/PointUtils.h>
#include <asp/Tools/bundle_adjust.h>
#include <asp/Core/InterestPointMatching.h>
#include <xercesc/util/PlatformUtils.hpp>


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

std::string UNSPECIFIED_DATUM = "unspecified_datum";
typedef boost::scoped_ptr<asp::StereoSession> SessionPtr;

int g_ba_num_errors = 0;
Mutex g_ba_mutex;


struct Options : public vw::cartography::GdalWriteOptions {
  std::vector<std::string> image_files, camera_files, gcp_files;
  std::string cnet_file, out_prefix, stereo_session_string,
    cost_function, ba_type, mapprojected_data, gcp_data;
  int    ip_per_tile;
  double min_triangulation_angle, lambda, camera_weight, robust_threshold;
  int    report_level, min_matches, max_iterations, overlap_limit;

  bool   save_iteration, local_pinhole_input, fix_gcp_xyz, solve_intrinsics;
  std::string datum_str, camera_position_file, csv_format_str, csv_proj4_str,
    intrinsics_to_float_str, init_transform_file;
  double semi_major, semi_minor, position_filter_dist;

  boost::shared_ptr<ControlNetwork> cnet;
  std::vector<boost::shared_ptr<CameraModel> > camera_models;
  cartography::Datum datum;
  int  ip_detect_method;
  bool individually_normalize;
  std::set<std::string> intrinsics_to_float;
  std::string overlap_list_file;
  std::set< std::pair<std::string, std::string> > overlap_list;
  
  // Make sure all values are initialized, even though they will be
  // over-written later.
  Options(): ip_per_tile(0), min_triangulation_angle(0), lambda(-1.0), camera_weight(-1),
             robust_threshold(0), report_level(0), min_matches(0),
             max_iterations(0), overlap_limit(0), save_iteration(false),
             local_pinhole_input(false), fix_gcp_xyz(false), solve_intrinsics(false),
             semi_major(0), semi_minor(0),
             datum(cartography::Datum(UNSPECIFIED_DATUM, "User Specified Spheroid",
                                      "Reference Meridian", 1, 1, 0)),
             ip_detect_method(0), individually_normalize(false){}
};

// TODO: This update stuff should really be done somewhere else!
//       Also the comments may be wrong.

/// This version does nothing.  All camera parameters will start at zero.
/// - This is for the BundleAdjustmentModel class where the camera parameters
///   are a rotation/offset that is applied on top of the existing camera model.
template<class ModelT> void
update_cnet_and_init_cams(ModelT & ba_model, Options & opt,
                          ControlNetwork& cnet,
                          std::vector<double> & cameras_vec,
                          std::vector<double> & intrinsics_vec){
}

/// This function should be called when no input cameras were
/// provided. We set initial guesses for the control point
/// positions and for the (pinhole) cameras.
template<> void
update_cnet_and_init_cams<BAPinholeModel>(
                          BAPinholeModel & ba_model, Options & opt,
                          ControlNetwork& cnet,
                          std::vector<double> & cameras_vec,
                          std::vector<double> & intrinsics_vec){

  // Set the size of cameras_vec
  const int num_cameras           = ba_model.num_cameras();
  const int num_params_per_camera = BAPinholeModel::camera_params_n;
  const int num_camera_params     = num_cameras * num_params_per_camera;
  const int num_intrinsic_params  = ba_model.num_intrinsic_params();
  cameras_vec.resize(num_camera_params);

  // Copy the camera parameters from the model to cameras_vec
  int index = 0;
  for (int i=0; i<num_cameras; ++i) {
    // Note that the inner loop stops before it gets to the intrinsic parameters
    BAPinholeModel::camera_intr_vector_t cam_vec;
    ba_model.get_cam_params(i, cam_vec);
    for (int p=0; p<num_params_per_camera; ++p) {
      cameras_vec[index] = cam_vec[p];
      ++index;
    } // End loop through camera parameters
  } // End loop through cameras

  // Get the intrinsics vector which is shared across all cameras.
  intrinsics_vec.resize(num_intrinsic_params);
  BAPinholeModel::camera_intr_vector_t cam_vec;
  ba_model.get_cam_params(0, cam_vec); // Just pull from the first camera
  for (int i=0; i < num_intrinsic_params; ++i) {
    intrinsics_vec[i] = cam_vec[num_params_per_camera+i];
  }

  return;
}

//=========================================================================
// Cost functions for Ceres

// TODO: We could probably consolidate the following two classes!

// A ceres cost function. Templated by the BundleAdjust model. We pass
// in the observation, the model, and the current camera and point
// indices. The result is the residual, the difference in the
// observation and the projection of the point into the camera,
// normalized by pixel_sigma.
template<class ModelT>
struct BaReprojectionError {
  BaReprojectionError(Vector2 const& observation, Vector2 const& pixel_sigma,
                      ModelT * const ba_model, size_t icam, size_t ipt):
    m_observation(observation),
    m_pixel_sigma(pixel_sigma),
    m_ba_model(ba_model),
    m_icam(icam), m_ipt(ipt){}

  template <typename T>
  bool operator()(const T* const camera, const T* const point,
                  T* residuals) const {

    try{

      size_t num_cameras = m_ba_model->num_cameras();
      size_t num_points  = m_ba_model->num_points();
      VW_ASSERT(m_icam < num_cameras, ArgumentErr() << "Out of bounds in the number of cameras");
      VW_ASSERT(m_ipt  < num_points , ArgumentErr() << "Out of bounds in the number of points" );

      // Copy the input data to structures expected by the BA model
      typename ModelT::camera_vector_t cam_vec;
      int cam_len = cam_vec.size();
      for (int c = 0; c < cam_len; c++)
        cam_vec[c] = (double)camera[c];

      typename ModelT::point_vector_t  point_vec;
      for (size_t p = 0; p < point_vec.size(); p++)
        point_vec[p]  = (double)point[p];

      // Project the current point into the current camera
      Vector2 prediction = (*m_ba_model).cam_pixel(m_ipt, m_icam, cam_vec, point_vec);
      
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

  // Factory to hide the construction of the CostFunction object from
  // the client code.
  static ceres::CostFunction* Create(Vector2 const& observation,
                                     Vector2 const& pixel_sigma,
                                     ModelT * const ba_model,
                                     size_t icam, // camera index
                                     size_t ipt // point index
                                     ){
    return (new ceres::NumericDiffCostFunction<BaReprojectionError,
            ceres::CENTRAL, 2, ModelT::camera_params_n, ModelT::point_params_n>
            (new BaReprojectionError(observation, pixel_sigma,
                                     ba_model, icam, ipt)));

  }

  Vector2 m_observation;
  Vector2 m_pixel_sigma;
  ModelT * const m_ba_model;
  size_t m_icam, m_ipt;
};

/// A ceres cost function. Here we float a pinhole camera's intrinsic
/// and extrinsic parameters. The result is the residual, the
/// difference in the observation and the projection of the point into
/// the camera, normalized by pixel_sigma.
struct BaPinholeError {
  BaPinholeError(Vector2 const& observation, Vector2 const& pixel_sigma,
                 BAPinholeModel * const ba_model, size_t icam, size_t ipt):
    m_observation(observation),
    m_pixel_sigma(pixel_sigma),
    m_ba_model(ba_model),
    m_icam(icam), m_ipt(ipt){}

  /// Compute residuals of observing this point with these camera parameters
  bool operator()(const double * const camera,
                  const double * const point,
                  const double * const focal_length,
                  const double * const optical_center,
                  const double * const nonlens_intrinsics,
                  double       * residuals) const {
    try{
      int num_cameras = m_ba_model->num_cameras();
      int num_points  = m_ba_model->num_points();
      VW_ASSERT(int(m_icam) < num_cameras, ArgumentErr()
                << "Out of bounds in the number of cameras");
      VW_ASSERT(int(m_ipt)  < num_points,  ArgumentErr()
                << "Out of bounds in the number of points" );

      // Copy the input data to structures expected by the BA model
      BAPinholeModel::camera_intr_vector_t cam_intr_vec;
      BAPinholeModel::point_vector_t       point_vec;

      m_ba_model->concat_extrinsics_intrinsics(camera,
                                               focal_length, optical_center, nonlens_intrinsics,
                                               cam_intr_vec);
      for (size_t p = 0; p < point_vec.size(); p++)
        point_vec[p] = point[p];

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
  
  /// Overload for when there is no distortion
  bool operator()(const double * const camera,
                  const double * const point,
                  const double * const focal_length,
                  const double * const optical_center,
                  double       * residuals) const {
    return this->operator()(camera, point, focal_length, optical_center, 0, residuals);
  }
  
  /// Overload for when all intrinsics are in one vector
  bool operator()(const double * const camera,
                  const double * const point,
                  const double * const intrinsic,
                  double       * residuals) const {
    
    int nf = BAPinholeModel::focal_length_params_n;
    int nc = BAPinholeModel::optical_center_params_n;
    return this->operator()(camera, point,
                            intrinsic, intrinsic + nf, intrinsic + nf + nc,
                            residuals);
  }
  
  /// Overload for when intrinsic parameters are not provided.
  bool operator()(const double * const camera,
                  const double * const point,
                  double       * residuals) const {
    // Just call the other function with a dummy intrinsic vector which will not be used.
    return this->operator()(camera, point, 0, residuals);
  }

  // Factory to hide the construction of the CostFunction object from
  // the client code.
  static ceres::CostFunction* Create(Vector2 const& observation,
                                     Vector2 const& pixel_sigma,
                                     BAPinholeModel * const ba_model,
                                     size_t icam, // camera index
                                     size_t ipt // point index
                                     ){
    const int nob = 2; // Num observation elements: Column, row
    const int ncp = BAPinholeModel::camera_params_n;
    const int npp = BAPinholeModel::point_params_n;
    const int nf  = BAPinholeModel::focal_length_params_n;
    const int nc  = BAPinholeModel::optical_center_params_n;
    const int num_intrinsics        = ba_model->num_intrinsic_params();

    // Create a ceres::NumericDiffCostFunction object templated to the
    // exact problem sizes we need. Notice that if we have more than 3 intrinsics that
    // means focal length (1 param), optical center (2 params), and the rest are
    // distortion params.
    
    switch(num_intrinsics) {
    case 0: // This case is different, it does not set an intrinsic size.
      return (new ceres::NumericDiffCostFunction<BaPinholeError,ceres::CENTRAL, nob, ncp, npp>(new BaPinholeError(observation, pixel_sigma, ba_model, icam, ipt)));
      // All of the other cases hard code the intrinsic length.
    case 1:
      vw_throw(LogicErr() << "bundle_adjust.cc not set up for 1 intrinsic param!");
    case 2:
      vw_throw(LogicErr() << "bundle_adjust.cc not set up for 2 intrinsic params!");
    case 3:
      return (new ceres::NumericDiffCostFunction<BaPinholeError,ceres::CENTRAL, nob, ncp, npp, nf, nc>(new BaPinholeError(observation, pixel_sigma, ba_model, icam, ipt))); // no distortion
      
    case 4:  return (new ceres::NumericDiffCostFunction<BaPinholeError,ceres::CENTRAL, nob, ncp, npp, nf, nc, 1>(new BaPinholeError(observation, pixel_sigma, ba_model, icam, ipt)));
    case 5:  return (new ceres::NumericDiffCostFunction<BaPinholeError,ceres::CENTRAL, nob, ncp, npp, nf, nc, 2>(new BaPinholeError(observation, pixel_sigma, ba_model, icam, ipt)));
    case 6:  return (new ceres::NumericDiffCostFunction<BaPinholeError,ceres::CENTRAL, nob, ncp, npp, nf, nc, 3>(new BaPinholeError(observation, pixel_sigma, ba_model, icam, ipt)));
    case 7:  return (new ceres::NumericDiffCostFunction<BaPinholeError,ceres::CENTRAL, nob, ncp, npp, nf, nc, 4>(new BaPinholeError(observation, pixel_sigma, ba_model, icam, ipt)));
    case 8:  return (new ceres::NumericDiffCostFunction<BaPinholeError,ceres::CENTRAL, nob, ncp, npp, nf, nc, 5>(new BaPinholeError(observation, pixel_sigma, ba_model, icam, ipt)));
    case 9:  return (new ceres::NumericDiffCostFunction<BaPinholeError,ceres::CENTRAL, nob, ncp, npp, nf, nc, 6>(new BaPinholeError(observation, pixel_sigma, ba_model, icam, ipt)));
    case 10:  return (new ceres::NumericDiffCostFunction<BaPinholeError,ceres::CENTRAL, nob, ncp, npp, nf, nc, 7>(new BaPinholeError(observation, pixel_sigma, ba_model, icam, ipt)));
    case 11:  return (new ceres::NumericDiffCostFunction<BaPinholeError,ceres::CENTRAL, nob, ncp, npp, nf, nc, 8>(new BaPinholeError(observation, pixel_sigma, ba_model, icam, ipt)));
    case 12:  return (new ceres::NumericDiffCostFunction<BaPinholeError,ceres::CENTRAL, nob, ncp, npp, nf, nc, 9>(new BaPinholeError(observation, pixel_sigma, ba_model, icam, ipt)));
    case 13:  return (new ceres::NumericDiffCostFunction<BaPinholeError,ceres::CENTRAL, nob, ncp, npp, nf, nc, 10>(new BaPinholeError(observation, pixel_sigma, ba_model, icam, ipt)));
    case 14:  return (new ceres::NumericDiffCostFunction<BaPinholeError,ceres::CENTRAL, nob, ncp, npp, nf, nc, 11>(new BaPinholeError(observation, pixel_sigma, ba_model, icam, ipt)));
    case 15:  return (new ceres::NumericDiffCostFunction<BaPinholeError,ceres::CENTRAL, nob, ncp, npp, nf, nc, 12>(new BaPinholeError(observation, pixel_sigma, ba_model, icam, ipt)));
      
    default:
      vw_throw(LogicErr() << "bundle_adjust.cc not set up for this many intrinsic params!");
    };
    return 0;
  }

  Vector2 m_observation;
  Vector2 m_pixel_sigma;
  BAPinholeModel * const m_ba_model;
  size_t m_icam, m_ipt;
};


/// A ceres cost function. The residual is the difference between the
/// observed 3D point and the current (floating) 3D point, normalized by
/// xyz_sigma. Used only for ground control points.
struct XYZError {
  XYZError(Vector3 const& observation, Vector3 const& xyz_sigma):
    m_observation(observation), m_xyz_sigma(xyz_sigma){}

  template <typename T>
  bool operator()(const T* const point, T* residuals) const {
    for (size_t p = 0; p < m_observation.size(); p++)
      residuals[p] = ((double)point[p] - m_observation[p])/m_xyz_sigma[p]; // Input units are meters

    return true;
  }

  // Factory to hide the construction of the CostFunction object from
  // the client code.
  static ceres::CostFunction* Create(Vector3 const& observation,
                                     Vector3 const& xyz_sigma){
    return (new ceres::NumericDiffCostFunction<XYZError, ceres::CENTRAL, 3, 3>
            (new XYZError(observation, xyz_sigma)));

  }

  Vector3 m_observation;
  Vector3 m_xyz_sigma;
};

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
  bool operator()(const T* const cam_vec, T* residuals) const {

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
    return (new ceres::NumericDiffCostFunction<CamError, ceres::CENTRAL,
            ModelT::camera_params_n, ModelT::camera_params_n>
            (new CamError(orig_cam, weight)));

  }

  CamVecT m_orig_cam;
  double m_weight;
};

//=========================================================================

ceres::LossFunction* get_loss_function(Options const& opt ){
  double th = opt.robust_threshold;
  ceres::LossFunction* loss_function;
  if      ( opt.cost_function == "l2"     )
    loss_function = NULL;
  else if ( opt.cost_function == "huber"  )
    loss_function = new ceres::HuberLoss(th);
  else if ( opt.cost_function == "cauchy" )
    loss_function = new ceres::CauchyLoss(th);
  else if ( opt.cost_function == "l1"     )
    loss_function = new ceres::SoftLOneLoss(th);
  else{
    vw_throw( ArgumentErr() << "Unknown cost function: " << opt.cost_function
              << " used with solver: " << opt.ba_type << ".\n" );
  }
  return loss_function;
}

// Add residual block without floating intrinsics
template<class ModelT>
void add_residual_block(ModelT & ba_model,
                        Vector2 const& observation, Vector2 const& pixel_sigma,
                        size_t icam, size_t ipt,
                        double * camera, double * point, double * intrinsics,
                        std::set<std::string> const& intrinsics_to_float,
                        ceres::LossFunction* loss_function,
                        ceres::Problem & problem){

  ceres::CostFunction* cost_function =
    BaReprojectionError<ModelT>::Create(observation, pixel_sigma,
                                        &ba_model, icam, ipt);
  problem.AddResidualBlock(cost_function, loss_function, camera, point);
}

// Add residual block, optionally floating the intrinsics
template<>
void add_residual_block<BAPinholeModel>
                  (BAPinholeModel & ba_model,
                   Vector2 const& observation, Vector2 const& pixel_sigma,
                   size_t icam, size_t ipt,
                   double * camera, double * point, double * intrinsics,
                   std::set<std::string> const& intrinsics_to_float,
                   ceres::LossFunction* loss_function,
                   ceres::Problem & problem){
  // If the intrinsics are constant use the default method above
  if (ba_model.are_intrinsics_constant()) {
    ceres::CostFunction* cost_function =
      BaReprojectionError<BAPinholeModel>::Create(observation, pixel_sigma,
                                                  &ba_model, icam, ipt);
    problem.AddResidualBlock(cost_function, loss_function, camera, point);
  }
  else {

    // Use a special const function using intrinsics

    ceres::CostFunction* cost_function =
      BaPinholeError::Create(observation, pixel_sigma, &ba_model, icam, ipt);

    int nf = BAPinholeModel::focal_length_params_n;
    int nc = BAPinholeModel::optical_center_params_n;

    const int num_distortion_params = ba_model.num_distortion_params();
    if (num_distortion_params == 0) 
      problem.AddResidualBlock(cost_function, loss_function, camera, point,
                               intrinsics,          // focal length
                               intrinsics + nf      // optical center
                               );
    else{
      problem.AddResidualBlock(cost_function, loss_function, camera, point,
                               intrinsics,          // focal length      (1 param)
                               intrinsics + nf,     // optical center    (2 params)
                               intrinsics + nf + nc // distortion params (all else)
                               );
    }
    
    // See if to float only certain intrinsics
    if (!intrinsics_to_float.empty()) {

      if (intrinsics_to_float.find("focal_length") == intrinsics_to_float.end()) {
        //vw_out() << "Will not float focal length.\n";
        problem.SetParameterBlockConstant(intrinsics);
      }else{
        //vw_out() << "Will float focal length.\n";
      }

      if (intrinsics_to_float.find("optical_center") == intrinsics_to_float.end()) {
        //vw_out() << "Will not float optical center.\n";
        problem.SetParameterBlockConstant(intrinsics + nf);
      }else{
        //vw_out() << "Will float optical center.\n";
      }

      if (intrinsics_to_float.find("distortion_params") == intrinsics_to_float.end()) {
        //vw_out() << "Will not float distortion parameters.\n";
        if (num_distortion_params > 0)
          problem.SetParameterBlockConstant(intrinsics + nf + nc);
      }else{
        //vw_out() << "Will float distortion parameters.\n";
      }
      
    } 
    
  } // end dealing with intrinsics
  
}


// Use Ceres to do bundle adjustment. The camera and point variables
// are stored in arrays.  The projection of point into camera is
// accomplished by interfacing with the bundle adjustment model. In
// the future this class can be bypassed.
template <class ModelT>
void do_ba_ceres(ModelT & ba_model, Options& opt ){

  ControlNetwork & cnet = *(ba_model.control_network().get());

  int num_camera_params    = ModelT::camera_params_n;
  int num_point_params     = ModelT::point_params_n;
  int num_intrinsic_params = ba_model.num_intrinsic_params();
  int num_cameras          = ba_model.num_cameras();
  int num_points           = ba_model.num_points();

  // The camera adjustment and point variables concatenated into
  // vectors. The camera adjustments start as 0. The points come from the network.
  std::vector<double> cameras_vec(num_cameras*num_camera_params, 0.0);
  std::vector<double> intrinsics_vec(num_intrinsic_params, 0.0);

  // Read initial adjustments
  if (opt.init_transform_file != "") {
    Matrix4x4 T;
    std::ifstream is(opt.init_transform_file.c_str());
    for (int row = 0; row < 4; row++){
      for (int col = 0; col < 4; col++){
        double a;
        if (! (is >> a) )
          vw_throw( vw::IOErr() << "Failed to read initial transform from: "
                                << opt.init_transform_file << "\n" );
        T(row, col) = a;
      }
    }
    vw_out() << "Initial guess transform:\n" << T << "\n";
    Matrix3x3 rot = submatrix(T,0,0,3,3);
    std::cout << "--rot is " << rot << std::endl;
    vw::Quat q(rot);
    Vector3 P = matrix_to_vector(submatrix(T,0,3,1,3));
    std::cout << "P is " << P << std::endl;
    Vector3 axis_angle = q.axis_angle();
    std::cout << "--axis angle is " << axis_angle << std::endl;
  }
  
  // Do any init required for this camera model.
  // - Currently we don't do anything except for pinhole models with no input cameras.
  update_cnet_and_init_cams(ba_model, opt, (*opt.cnet), cameras_vec, intrinsics_vec);
/*
  // DEBUG
  std::cout << "Initial camera parameters: ";
  for (size_t i=0; i<cameras_vec.size(); ++i)
    std::cout << cameras_vec[i] << "  ";
  std::cout << "\nInitial intrinsic parameters: ";
  for (size_t i=0; i<intrinsics_vec.size(); ++i)
    std::cout << intrinsics_vec[i] << "  ";
  std::cout << std::endl;
*/

  // Camera extrinsics and intrinsics
  double* cameras    = &cameras_vec[0];
  double* intrinsics = NULL;
  if (num_intrinsic_params > 0)
    intrinsics = &intrinsics_vec[0];

  // Points
  std::vector<double> points_vec(num_points*num_point_params, 0.0);
  for (int ipt = 0; ipt < num_points; ipt++){
    for (int q = 0; q < num_point_params; q++){
      points_vec[ipt*num_point_params + q] = cnet[ipt].position()[q];
    }
  }
  double* points = &points_vec[0];

  // The camera positions and orientations before we float them
  std::vector<double> orig_cameras_vec = cameras_vec;

  ceres::Problem problem;

  CameraRelationNetwork<JFeature> crn;
  crn.read_controlnetwork(cnet);
  
  // Now add the various cost functions the solver will optimize over.

  // Add the cost function component for difference of pixel observations
  // - Reduce error by making pixel projection consistent with observations.
  typedef CameraNode<JFeature>::iterator crn_iter;
  for ( size_t icam = 0; icam < crn.size(); icam++ ) {
    for ( crn_iter fiter = crn[icam].begin(); fiter != crn[icam].end(); fiter++ ){

      // The index of the 3D point
      int ipt = (**fiter).m_point_id;

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

      // Each observation corresponds to a pair of a camera and a point
      // which are identified by indices icam and ipt respectively.
      double * camera = cameras + icam * num_camera_params;
      double * point  = points  + ipt  * num_point_params;

      ceres::LossFunction* loss_function = get_loss_function(opt);

      // Call function to select the appropriate Ceres residual block to add.
      add_residual_block(ba_model, observation, pixel_sigma, icam, ipt,
                         camera, point, intrinsics, opt.intrinsics_to_float,
                         loss_function, problem);
    }
  }

  // Add ground control points
  // - Error goes up as GCP's move from their input positions.
  int num_gcp = 0;
  for (int ipt = 0; ipt < num_points; ipt++){
    if (cnet[ipt].type() != ControlPoint::GroundControlPoint) continue;

    num_gcp++;
    
    Vector3 observation = cnet[ipt].position();
    Vector3 xyz_sigma   = cnet[ipt].sigma();

    ceres::CostFunction* cost_function = XYZError::Create(observation, xyz_sigma);

    ceres::LossFunction* loss_function = get_loss_function(opt);

    double * point  = points  + ipt * num_point_params;
    problem.AddResidualBlock(cost_function, loss_function, point);

    if (opt.fix_gcp_xyz) 
      problem.SetParameterBlockConstant(point);
  }

  // Add camera constraints
  // - Error goes up as cameras move and rotate from their input positions.
  if (opt.camera_weight > 0){
    for (int icam = 0; icam < num_cameras; icam++){

      typename ModelT::camera_vector_t orig_cam;
      for (int q = 0; q < num_camera_params; q++)
        orig_cam[q] = orig_cameras_vec[icam * num_camera_params + q];

      ceres::CostFunction* cost_function = CamError<ModelT>::Create(orig_cam, opt.camera_weight);

      ceres::LossFunction* loss_function = get_loss_function(opt);

      double * camera  = cameras  + icam * num_camera_params;
      problem.AddResidualBlock(cost_function, loss_function, camera);
    }
  }

  // Solve the problem
  ceres::Solver::Options options;
  options.gradient_tolerance = 1e-16;
  options.function_tolerance = 1e-16;
  options.max_num_iterations = opt.max_iterations;
  options.max_num_consecutive_invalid_steps = std::max(5, opt.max_iterations/5); // try hard
  options.minimizer_progress_to_stdout = (opt.report_level >= vw::ba::ReportFile);

  if (opt.stereo_session_string == "isis")
    options.num_threads = 1;
  else
    options.num_threads = opt.num_threads;

  options.linear_solver_type = ceres::SPARSE_SCHUR;
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
  vw_out() << summary.FullReport() << "\n";
  if (summary.termination_type == ceres::NO_CONVERGENCE){
    // Print a clarifying message, so the user does not think that the algorithm failed.
    vw_out() << "Found a valid solution, but did not reach the actual minimum." << std::endl;
  }

  // Copy the latest version of the optimized intrinsic variables back
  // into the the separate parameter vectors in ba_model, right after
  // the already updated extrinsic parameters.
  typename ModelT::camera_intr_vector_t concat;
  for (int icam = 0; icam < num_cameras; icam++){
    ba_model.concat_extrinsics_intrinsics(&cameras_vec[icam*num_camera_params],
                                          intrinsics, concat);
    ba_model.set_cam_params(icam, concat);
  }

  // Print stats for optimized gcp
  if (num_gcp > 0) {
    vw_out() << "input_gcp_xyz optimized_gcp_xyz diff_norm\n";
    for (int ipt = 0; ipt < num_points; ipt++){
      if (cnet[ipt].type() != ControlPoint::GroundControlPoint) continue;
      
      Vector3 input_gcp = cnet[ipt].position();
      
      double * point  = points  + ipt * num_point_params;
      Vector3 opt_gcp;
      for (int p = 0; p < 3; p++) opt_gcp[p] = point[p];
      
      vw_out() << input_gcp << ' ' << opt_gcp << ' ' << norm_2(input_gcp - opt_gcp) << std::endl;
    }
  }
  
} // end do_ba_ceres



/// The older approach, using VW's solver
template <class AdjusterT>
void do_ba_nonceres(typename AdjusterT::model_type & ba_model,
                    typename AdjusterT::cost_type const& cost_function,
                    Options const& opt) {

  AdjusterT bundle_adjuster(ba_model, cost_function, false, false);

  if ( opt.lambda > 0 )
    bundle_adjuster.set_lambda( opt.lambda );

  std::string iterCameraFile = opt.out_prefix + "iterCameraParam.txt";
  std::string iterPointsFile = opt.out_prefix + "iterPointsParam.txt";

  //Clearing the monitoring text files to be used for saving camera params
  if (opt.save_iteration){
    fs::remove(iterCameraFile);
    fs::remove(iterPointsFile);

    // Write the starting locations
    vw_out() << "Writing: " << iterCameraFile << std::endl;
    vw_out() << "Writing: " << iterPointsFile << std::endl;
    ba_model.bundlevis_cameras_append(iterCameraFile);
    ba_model.bundlevis_points_append(iterPointsFile);
  }

  BundleAdjustReport<AdjusterT> reporter("Bundle Adjust", ba_model, bundle_adjuster,
                                          opt.report_level);

  double abs_tol = 1e10, rel_tol=1e10;
  double overall_delta = 2;
  int    no_improvement_count = 0;
  while ( true ) {
    // Determine if it is time to quit
    if ( bundle_adjuster.iterations() >= opt.max_iterations ) {
      reporter() << "Triggered 'Max Iterations'\n";
      break;
    } else if ( abs_tol < 0.01 ) {
      reporter() << "Triggered 'Abs Tol " << abs_tol << " < 0.01'\n";
      break;
    } else if ( rel_tol < 1e-6 ) {
      reporter() << "Triggered 'Rel Tol " << rel_tol << " < 1e-10'\n";
      break;
    } else if ( no_improvement_count > 4 ) {
      reporter() << "Triggered break, unable to improve after " << no_improvement_count << " iterations\n";
      break;
    }

    overall_delta = bundle_adjuster.update( abs_tol, rel_tol );
    reporter.loop_tie_in();

    // Writing Current Camera Parameters to file for later reading
    if (opt.save_iteration) {
      ba_model.bundlevis_cameras_append(iterCameraFile);
      ba_model.bundlevis_points_append(iterPointsFile);
    }
    if ( overall_delta == 0 )
      no_improvement_count++;
    else
      no_improvement_count = 0;
  }
  reporter.end_tie_in();

} // end do_ba_nonceres

void save_cnet_as_csv(Options& opt, std::string const& cnetFile){

  // Save the input control network in the csv file format used by ground
  // control points.

  if (opt.datum.name() == UNSPECIFIED_DATUM)
    vw_throw( ArgumentErr() << "FATAL: No datum was specified. "
                            << "Cannot save control network as csv.\n" );

  vw_out() << "Writing: " << cnetFile << std::endl;
  std::ofstream ofs(cnetFile.c_str());
  ofs.precision(18);

  int count = 0;
  ControlNetwork & cnet = *opt.cnet.get();
  for ( ControlNetwork::const_iterator iter = cnet.begin();
        iter != cnet.end(); ++iter ) {

    // If to dump only gcp
    //if ( (*iter).type() != ControlPoint::GroundControlPoint ) continue;

    count++;

    // lon,lat,height
    Vector3 llr = opt.datum.cartesian_to_geodetic((*iter).position());

    // convert to lat,lon,height
    std::swap(llr[0], llr[1]);

    Vector3 sigma = (*iter).sigma();

    ofs << count     << ' ' << llr  [0] << ' ' << llr  [1] << ' ' << llr[2] << ' ';
    ofs << sigma[0]  << ' ' << sigma[1] << ' ' << sigma[2] << ' ';

    for ( ControlPoint::const_iterator measure = (*iter).begin();
          measure != (*iter).end(); ++measure ) {

      ofs << opt.image_files[measure->image_id()] << ' '
          << measure->position()[0] << ' ' << measure->position()[1] << ' '
          << measure->sigma()[0]    << ' ' << measure->sigma()[1];

      if ( measure+1 != (*iter).end())
        ofs << ' ';
      else
        ofs << std::endl;
    }
  }
  return;
}

// Note - The following two functions are required for our BA stuff, Ceres does not
//        require all the template switches.

// Use given cost function. Switch based on solver.
template<class ModelType, class CostFunType>
void do_ba_costfun(CostFunType const& cost_fun, Options& opt){

  ModelType ba_model(opt.camera_models, opt.cnet);

  if ( opt.ba_type == "ceres" ) {
    do_ba_ceres<ModelType>(ba_model, opt);
  } else if ( opt.ba_type == "robustsparse" ) {
    do_ba_nonceres<AdjustRobustSparse< ModelType,CostFunType> >(ba_model, cost_fun, opt);
  } else if ( opt.ba_type == "robustref" ) {
    do_ba_nonceres<AdjustRobustRef< ModelType,CostFunType> >(ba_model, cost_fun, opt);
  } else if ( opt.ba_type == "sparse" ) {
    do_ba_nonceres<AdjustSparse< ModelType, CostFunType > >(ba_model, cost_fun, opt);
  }else if ( opt.ba_type == "ref" ) {
    do_ba_nonceres<AdjustRef< ModelType, CostFunType > >(ba_model, cost_fun, opt);
  }

  // Save the models to disk.
  for (size_t icam = 0; icam < ba_model.num_cameras(); icam++){
    std::string adjust_file = asp::bundle_adjust_file_name(opt.out_prefix,
                                                           opt.image_files[icam],
                                                           opt.camera_files[icam]);
    vw_out() << "Writing: " << adjust_file << std::endl;
    ba_model.write_adjustment(icam, adjust_file);
  }

}

// Do BA with given model. Switch based on cost function.
template<class ModelType>
void do_ba_with_model(Options& opt){

  if ( opt.cost_function == "cauchy" ) {
    do_ba_costfun<ModelType, CauchyError>(CauchyError(opt.robust_threshold), opt);
  }else if ( opt.cost_function == "pseudohuber" ) {
    do_ba_costfun<ModelType, PseudoHuberError>(PseudoHuberError(opt.robust_threshold), opt );
  } else if ( opt.cost_function == "huber" ) {
    do_ba_costfun<ModelType, HuberError>(HuberError(opt.robust_threshold), opt );
  } else if ( opt.cost_function == "l1" ) {
    do_ba_costfun<ModelType, L1Error>( L1Error(), opt );
  } else if ( opt.cost_function == "l2" ) {
    do_ba_costfun<ModelType, L2Error>( L2Error(), opt );
  }else{
    vw_throw( ArgumentErr() << "Unknown cost function: " << opt.cost_function
              << ". Options are: Cauchy, PseudoHuber, Huber, L1, L2.\n" );
  }
}

/// Given a vector of strings, identify and store separately the list of camera models.
std::vector<std::string>
extract_cameras_bundle_adjust( std::vector<std::string>& image_files ) {
  std::vector<std::string> cam_files;
  std::vector<std::string>::iterator it = image_files.begin();
  while ( it != image_files.end() ) {

    if (asp::has_pinhole_extension(*it) ||
        boost::iends_with(boost::to_lower_copy(*it), ".xml") || // TODO: This should be a function call!
        boost::iends_with(boost::to_lower_copy(*it), ".cub") ||
        boost::iends_with(boost::to_lower_copy(*it), ".dim")
        ){
      cam_files.push_back( *it );
      it = image_files.erase( it );
    } else { // Not a camera, double check that it is an image.
      if (!asp::has_image_extension(*it))
        vw_throw( ArgumentErr() << "Error - unrecognized input: '" << *it <<"'");
      it++;
    }
  }
  
  return cam_files;
}

/// Apply a scale-rotate-translate transform to pinhole cameras and control points
void apply_rigid_transform(vw::Matrix3x3 const & rotation,
                           vw::Vector3   const & translation,
                           double                scale,
                           Options             & opt) {

  // Apply the transform to the cameras
  for (size_t icam = 0; icam < opt.camera_models.size(); icam++){
    vw::camera::PinholeModel * pincam
      = dynamic_cast<vw::camera::PinholeModel*>(opt.camera_models[icam].get());
    VW_ASSERT(pincam != NULL, vw::ArgumentErr() << "A pinhole camera expected.\n");

    pincam->apply_transform(rotation, translation, scale);
    
    //std::cout << "model: " << *pincam << std::endl;
    //std::cout << "New GDC coordinate: " << opt.datum.cartesian_to_geodetic(position) << std::endl;
  } // End loop through cameras

  // Apply the transform to all of the world points in the ControlNetwork
  ControlNetwork::iterator iter;
  for (iter=opt.cnet->begin(); iter!=opt.cnet->end(); ++iter) {
    if (iter->type() == ControlPoint::GroundControlPoint)
      continue; // Don't convert the ground control points!

    Vector3 position     = iter->position();
    Vector3 new_position = scale*rotation*position + translation;
    //std::cout << "Converted position: " << position << " --> " << new_position << std::endl;
    //std::cout << "          =======>  " << opt.datum.cartesian_to_geodetic(position) << " --> "
    //          << opt.datum.cartesian_to_geodetic(new_position) << std::endl;
    iter->set_position(new_position);
  }
} // End function ApplyRigidTransform


/// Generate a warning if the GCP's are really far from the IP points
/// - This is intended to help catch the common lat/lon swap in GCP files.
void check_gcp_dists(Options const &opt) {

    // Make one iteration just to count the points.
    const ControlNetwork & cnet = *opt.cnet.get(); // Helper alias
    const int num_cnet_points = static_cast<int>(cnet.size());
    double gcp_count=0, ip_count=0;
    for (int ipt = 0; ipt < num_cnet_points; ipt++){

      if (cnet[ipt].type() == ControlPoint::GroundControlPoint)
        gcp_count += 1.0;
      else {
        // Use triangulation to estimate the position of this control point using
        //   the current set of camera models.
        ControlPoint cp_new = cnet[ipt];
        double minimum_angle = 0;
        vw::ba::triangulate_control_point(cp_new, opt.camera_models, minimum_angle);
        if (cp_new.position() == Vector3())
          continue; // Skip points that fail to triangulate

        ip_count += 1.0;
      }
    } // End loop through control network points

    // Make another iteration to compute the mean.
    Vector3 mean_gcp(0,0,0);
    Vector3 mean_ip(0,0,0);
    for (int ipt = 0; ipt < num_cnet_points; ipt++){

      if (cnet[ipt].type() == ControlPoint::GroundControlPoint) {
        mean_gcp += (cnet[ipt].position() / gcp_count);
      }
      else {
        // Use triangulation to estimate the position of this control point using
        //   the current set of camera models.
        ControlPoint cp_new = cnet[ipt];
        double minimum_angle = 0;
        vw::ba::triangulate_control_point(cp_new, opt.camera_models, minimum_angle);
        if (cp_new.position() == Vector3())
          continue; // Skip points that fail to triangulate

        mean_ip += (cp_new.position() / ip_count);
      }
    } // End loop through control network points

    double dist = norm_2(mean_ip - mean_gcp);
    if (dist > 100000)
      std::cout << "WARNING: GCPs are over 100 KM from the other points. Are your lat/lon GCP coordinates swapped?\n";
}


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
    //std::cout << "file_name = " << file_name << std::endl;
    RecordIter iter;
    for (iter=pos_records.begin(); iter!=pos_records.end(); ++iter) {
      // Match if the string in the file is contained in the input image string.
      // - May need to play around with this in the future!
      std::string field = iter->file;
      //std::cout << "field = " << field << std::endl;
      if (file_name.find(field) != std::string::npos) {
        estimated_camera_gcc[i] = conv.csv_to_cartesian(*iter, geo);
        break; // Match found, stop the iterator here.
      }
    }
    if (iter == no_match) {
      std::cout << "WARNING: Camera file " << file_name << " not found in camera position file.\n";
      estimated_camera_gcc[i] = Vector3(0,0,0);
    }else
      ++num_matches_found;
  } // End loop to find position record for each camera

  return num_matches_found;  
}


/// Initialize the position and orientation of each pinhole camera model using
///  a least squares error transform to match the provided camera positions.
/// - This function overwrites the camera parameters in-place
bool init_pinhole_model_with_camera_positions(Options &opt,
                                              std::vector<Vector3> const & estimated_camera_gcc) {

  std::cout << "Initializing camera positions from input file..." << std::endl;

  // Count the number of matches and check for problems
  const int num_cameras = opt.image_files.size();
  if (int(estimated_camera_gcc.size()) != num_cameras)
    vw_throw( ArgumentErr() << "No camera matches provided to init function!\n" );
  
  std::cout << "Num cameras: " << num_cameras << std::endl;
    
  int num_matches_found = 0;
  for (int i=0; i<num_cameras; ++i)
    if (estimated_camera_gcc[i] != Vector3(0,0,0))
      ++num_matches_found;

  std::cout << "Number of matches found: " << num_matches_found << std::endl;
  
  const int MIN_NUM_MATCHES = 3;
  if (num_matches_found < MIN_NUM_MATCHES)
    vw_throw( ArgumentErr() << "Not enough camera position matches to initialize sensor models!\n" );
  
  // Populate matrices containing the current and known camera positions.
  vw::Matrix<double> points_in(3, num_matches_found), points_out(3, num_matches_found);
  typedef vw::math::MatrixCol<vw::Matrix<double> > ColView;
  int index = 0;
  for (int i=0; i<num_cameras; ++i) {
    // Skip cameras with no matching record
    if (estimated_camera_gcc[i] == Vector3(0,0,0))
      continue;

    // Get the two GCC positions
    Vector3 gcc_in  = opt.camera_models[i]->camera_center(Vector2(0,0));
    Vector3 gcc_out = estimated_camera_gcc[i];
    
    // Store in matrices
    ColView colIn (points_in,  index); 
    ColView colOut(points_out, index);
    colIn  = gcc_in;
    colOut = gcc_out;
    ++index;

  } // End matrix populating loop

  // Call function to compute a 3D affine transform between the two point sets
  vw::Matrix3x3 rotation;
  vw::Vector3   translation;
  double        scale;
  asp::find_3D_affine_transform(points_in, points_out, rotation, translation, scale);

/*
  // Debug: Test transform on cameras
  for (int i=0; i<num_cameras; ++i) {
    // Skip cameras with no matching record
    if (estimated_camera_gcc[i] == Vector3(0,0,0))
      continue;

    // Get the two GCC positions
    Vector3 gcc_in    = opt.camera_models[i]->camera_center(Vector2(0,0));
    Vector3 gcc_out  = estimated_camera_gcc[i];
    Vector3 gcc_trans = scale*rotation*gcc_in + translation;

    std::cout << "--gdc_file  is " << opt.datum.cartesian_to_geodetic(gcc_out ) << std::endl;
    std::cout << "--gdc_trans is " << opt.datum.cartesian_to_geodetic(gcc_trans) << std::endl;
    std::cout << "--gcc_diff  is " << norm_2(gcc_trans - gcc_out)               << std::endl;
  } // End debug loop
*/

  // Update the camera and point information with the new transform
  apply_rigid_transform(rotation, translation, scale, opt);
  return true;
}

/// Initialize the position and orientation of each pinhole camera model using
///  a least squares error transform to match the provided control points file.
/// - This function overwrites the camera parameters in-place
bool init_pinhole_model_with_gcp(Options &opt, bool check_only=false) {

    std::cout << "Initializing camera positions from ground control points..." << std::endl;

    const ControlNetwork & cnet = *opt.cnet.get(); // Helper alias

    // DEBUG: Print out all pinhole cameras and verify they are pinhole cameras.
    for (size_t icam = 0; icam < opt.camera_models.size(); icam++){
      vw::camera::PinholeModel * pincam
        = dynamic_cast<vw::camera::PinholeModel*>(opt.camera_models[icam].get());
      VW_ASSERT(pincam != NULL,
                vw::ArgumentErr() << "A pinhole camera expected.\n");
    }

    // Count up the number of good ground control points
    // - Maybe this should be a function of the ControlNet class?
    const int num_cnet_points = static_cast<int>(cnet.size());
    int num_good_gcp = 0;
    for (int ipt = 0; ipt < num_cnet_points; ipt++){
      if (cnet[ipt].type() != ControlPoint::GroundControlPoint)
        continue;
        
      // Use triangulation to estimate the position of this control point using
      //   the current set of camera models.
      ControlPoint cp_new = cnet[ipt];
      // Making minimum_angle below big may throw away valid points at this stage // really???
      double minimum_angle = 0;
      vw::ba::triangulate_control_point(cp_new, opt.camera_models, minimum_angle);
      if (cp_new.position() != Vector3() && cnet[ipt].position() != Vector3())
        ++num_good_gcp; // Only count points that triangulate
    }
    
    // Update the number of GCP that we are using
    const int MIN_NUM_GOOD_GCP = 3;
    if (num_good_gcp < MIN_NUM_GOOD_GCP)
      vw_throw( ArgumentErr() << "Not enough valid GCPs for affine initalization!\n" );
    
    /*
    // DEBUG: Print out a measure of the triangulation error
    for (int ipt = 0; ipt < num_cnet_points; ipt++){
      // Skip ground control points
      if (cnet[ipt].type() == ControlPoint::GroundControlPoint)
        continue;

      // Use triangulation to estimate the position of this control point using
      //   the current set of camera models.
      ControlPoint cp_new = cnet[ipt];
      double minimum_angle = 0;
      double tri_err = vw::ba::triangulate_control_point(cp_new,
                                                         opt.camera_models,
                                                         minimum_angle);
      std::cout << "Err = " << tri_err << std::endl;
    } // End loop through control network points
    */

    vw::Matrix<double> points_in(3, num_good_gcp), points_out(3, num_good_gcp);
    typedef vw::math::MatrixCol<vw::Matrix<double> > ColView;
    int index = 0;
    for (int ipt = 0; ipt < num_cnet_points; ipt++){
      // Loop through all the ground control points only
      if (cnet[ipt].type() != ControlPoint::GroundControlPoint)
        continue;

      // Use triangulation to estimate the position of this control point using
      //   the current set of camera models.
      ControlPoint cp_new = cnet[ipt];
      // Making minimum_angle below big may throw away valid points at this stage // really???
      double minimum_angle = 0;
      vw::ba::triangulate_control_point(cp_new,
                                        opt.camera_models,
                                        minimum_angle);

      // Store the computed and correct position of this point in Eigen matrices
      Vector3 inp  = cp_new.position();
      Vector3 outp = cnet[ipt].position();
      if (inp == Vector3() || outp == Vector3())
        continue; // Skip points that fail to triangulate

      // Store in matrices
      ColView colIn (points_in,  index); 
      ColView colOut(points_out, index);
      colIn  = inp;
      colOut = outp;

      //if (check_only) // In geocentric coords, convert to GDC
      //  std::cout << "--in is " << opt.datum.cartesian_to_geodetic(inp) << std::endl;
      //else // In local coords, leave as-is
      //  std::cout << "--in is " << inp << std::endl;
      //std::cout << "--ou is " << opt.datum.cartesian_to_geodetic(outp) << std::endl;
      ++index;
    } // End loop through control network points

    // Call function to compute a 3D affine transform between the two point sets
    vw::Matrix3x3 rotation;
    vw::Vector3   translation;
    double        scale;
    asp::find_3D_affine_transform(points_in, points_out, rotation, translation, scale);

    if (check_only)
      return true;

/*
    // DEBUG - Test the transform on the GCP's
    std::cout << "==== GCP fit quality ====\n";
    for (int ipt = 0; ipt < num_cnet_points; ipt++){
      // Loop through all the ground control points only
      if (cnet[ipt].type() != ControlPoint::GroundControlPoint)
        continue;

      // Use triangulation to estimate the position of this control point using
      //   the current set of camera models.
      ControlPoint cp_new = cnet[ipt];
      double minimum_angle = 0;
      vw::ba::triangulate_control_point(cp_new,
                                        opt.camera_models,
                                        minimum_angle);

      // Store the computed and correct position of this point in Eigen matrices
      Vector3 inp    = cp_new.position();
      Vector3 transp = scale*rotation*inp + translation;
      Vector3 outp   = cnet[ipt].position();
      std::cout << "---triangulated: " << cnet[ipt].position() << ' '
                << cp_new.position() << std::endl;
      if (inp == Vector3() || outp == Vector3())
        continue; // Skip points that fail to triangulate

      std::cout << "--tr is " << opt.datum.cartesian_to_geodetic(transp) << std::endl;
      std::cout << "--ou is " << opt.datum.cartesian_to_geodetic(outp) << std::endl;
    } // End loop through control network points
*/

  // Update the camera and point information with the new transform
  apply_rigid_transform(rotation, translation, scale, opt);

  return true;
} // End function init_pinhole_model_with_gcp

// If the user map-projected the images and created matches by hand
// (this is useful when the illumination conditions are too different,
// and automated matching fails), project those matching ip back
// into the cameras, creating matches between the raw images
// that then bundle_adjust can use. 
void create_matches_from_mapprojected_images(Options const& opt){
  
  std::istringstream is(opt.mapprojected_data);
  std::vector<std::string> map_files;
  std::string file;
  while (is >> file){
    map_files.push_back(file); 
  }
  std::string dem_file = map_files.back();
  map_files.erase(map_files.end() - 1);
  vw_out() << "Loading DEM: " << dem_file << std::endl;
  double nodata_val = -std::numeric_limits<float>::max(); // note we use a float nodata
  if (vw::read_nodata_val(dem_file, nodata_val)){
    vw_out() << "Found DEM nodata value: " << nodata_val << std::endl;
  }
  
  ImageView< PixelMask<double> > dem = create_mask(DiskImageView<double>(dem_file), nodata_val);
  InterpolationView<EdgeExtensionView< ImageView< PixelMask<double> >, ConstantEdgeExtension >, BilinearInterpolation> interp_dem = interpolate(dem, BilinearInterpolation(),
																		ConstantEdgeExtension());
  vw::cartography::GeoReference dem_georef;
  bool is_good = vw::cartography::read_georeference(dem_georef, dem_file);
  if (!is_good) {
    vw_throw(ArgumentErr() << "Error: Cannot read georeference from DEM: "
	     << dem_file << ".\n");
  }
  
  for (size_t i = 0; i < map_files.size(); i++) {
    for (size_t j = i+1; j < map_files.size(); j++) {
      
      vw::cartography::GeoReference georef1, georef2;
      vw_out() << "Reading georef from " << map_files[i] << ' ' << map_files[j] << std::endl;
      bool is_good1 = vw::cartography::read_georeference(georef1, map_files[i]);
      bool is_good2 = vw::cartography::read_georeference(georef2, map_files[j]);
      if (!is_good1 || !is_good2) {
	vw_throw(ArgumentErr() << "Error: Cannot read georeference.\n");
      }
      
      std::string match_filename = ip::match_filename(opt.out_prefix,
						      map_files[i], map_files[j]);
      if (!fs::exists(match_filename)) {
	vw_out() << "Missing: " << match_filename << "\n";
            continue;
      }
      vw_out() << "Reading: " << match_filename << std::endl;
      std::vector<ip::InterestPoint> ip1, ip2;
      std::vector<ip::InterestPoint> ip1_cam, ip2_cam;
      ip::read_binary_match_file( match_filename, ip1, ip2 );
      
      
      // Undo the map-projection
      for (size_t ip_iter = 0; ip_iter < ip1.size(); ip_iter++) {
            
	vw::ip::InterestPoint P1 = ip1[ip_iter];
	Vector2 pix1(P1.x, P1.y);
	Vector2 ll1 = georef1.pixel_to_lonlat(pix1);
	Vector2 dem_pix1 = dem_georef.lonlat_to_pixel(ll1);
	if (dem_pix1[0] < 0 || dem_pix1[0] >= dem.cols() - 1) continue;
	if (dem_pix1[1] < 0 || dem_pix1[1] >= dem.rows() - 1) continue;
	PixelMask<double> dem_val1 = interp_dem(dem_pix1[0], dem_pix1[1]);
	if (!is_valid(dem_val1)) continue;
	Vector3 llh1(ll1[0], ll1[1], dem_val1.child());
	Vector3 xyz1 = dem_georef.datum().geodetic_to_cartesian(llh1);
	Vector2 cam_pix1;
	try { cam_pix1 = opt.camera_models[i]->point_to_pixel(xyz1); }
	catch(...){ continue; }
	P1.x = cam_pix1.x(); P1.y = cam_pix1.y(); P1.ix = P1.x; P1.iy = P1.y;
	
	vw::ip::InterestPoint P2 = ip2[ip_iter];
	Vector2 pix2(P2.x, P2.y);
	Vector2 ll2 = georef2.pixel_to_lonlat(pix2);
	Vector2 dem_pix2 = dem_georef.lonlat_to_pixel(ll2);
	if (dem_pix2[0] < 0 || dem_pix2[0] >= dem.cols() - 1) continue;
	if (dem_pix2[1] < 0 || dem_pix2[1] >= dem.rows() - 1) continue;
	PixelMask<double> dem_val2 = interp_dem(dem_pix2[0], dem_pix2[1]);
	if (!is_valid(dem_val2)) continue;
	Vector3 llh2(ll2[0], ll2[1], dem_val2.child());
	Vector3 xyz2 = dem_georef.datum().geodetic_to_cartesian(llh2);
	Vector2 cam_pix2;
	try { cam_pix2 = opt.camera_models[j]->point_to_pixel(xyz2); }
	catch(...){ continue; }
	P2.x = cam_pix2.x(); P2.y = cam_pix2.y(); P2.ix = P2.x; P2.iy = P2.y;
	
	ip1_cam.push_back(P1);
	ip2_cam.push_back(P2);
      }
      
      // TODO: There is a problem if the number of matches changes!!!
      vw_out() << "Saving " << ip1_cam.size() << " matches.\n";
      std::string image1_path  = opt.image_files[i];
      std::string image2_path  = opt.image_files[j];
      match_filename = ip::match_filename(opt.out_prefix, image1_path, image2_path);
      
      vw_out() << "Writing: " << match_filename << std::endl;
      ip::write_binary_match_file(match_filename, ip1_cam, ip2_cam);
      
    }
  }
}

// If the user map-projected the images and created matches by hand
// from each map-projected image to the DEM it was map-projected onto,
// project those matches back into the camera image, and crate gcp
// tying each camera image match to its desired location on the DEM.
void create_gcp_from_mapprojected_images(Options const& opt){

  // Read the map-projected images and the dem
  std::istringstream is(opt.gcp_data);
  std::vector<std::string> image_files;
  std::string file;
  while (is >> file){
    image_files.push_back(file); 
  }
  std::string dem_file = image_files.back();
  image_files.erase(image_files.end() - 1); // wipe the dem from the list
  vw_out() << "Loading DEM: " << dem_file << std::endl;
  double nodata_val = -std::numeric_limits<float>::max(); // note we use a float nodata
  if (vw::read_nodata_val(dem_file, nodata_val)){
    vw_out() << "Found DEM nodata value: " << nodata_val << std::endl;
  }
  
  ImageView< PixelMask<double> > dem = create_mask(DiskImageView<double>(dem_file), nodata_val);
  InterpolationView<EdgeExtensionView< ImageView< PixelMask<double> >, ConstantEdgeExtension >, BilinearInterpolation> interp_dem = interpolate(dem, BilinearInterpolation(),
																		ConstantEdgeExtension());
  vw::cartography::GeoReference georef_dem;
  bool is_good = vw::cartography::read_georeference(georef_dem, dem_file);
  if (!is_good) {
    vw_throw(ArgumentErr() << "Error: Cannot read georeference from DEM: "
	     << dem_file << ".\n");
  }

  int num_images = image_files.size();
  std::vector<std::vector<vw::ip::InterestPoint> > matches;
  std::vector<vw::cartography::GeoReference> img_georefs;
  matches.resize(num_images + 1); // the last match will be for the DEM

  // Read the matches and georefs
  for (int i = 0; i < num_images; i++) {
      
    vw::cartography::GeoReference img_georef;
    vw_out() << "Reading georef from " << image_files[i]  << std::endl;
    bool is_good_img = vw::cartography::read_georeference(img_georef, image_files[i]);
    if (!is_good_img) {
      vw_throw(ArgumentErr() << "Error: Cannot read georeference.\n");
    }
    img_georefs.push_back(img_georef);
    
    std::string match_filename = ip::match_filename(opt.out_prefix,
                                                    image_files[i], dem_file);
    if (!fs::exists(match_filename)) 
      vw_throw(ArgumentErr() << "Missing: " << match_filename << ".\n");
    
    vw_out() << "Reading: " << match_filename << std::endl;
    std::vector<ip::InterestPoint> ip1, ip2;
    ip::read_binary_match_file( match_filename, ip1, ip2 );
    
    if (matches[num_images].size() > 0 && matches[num_images].size() != ip2.size()) {
      vw_throw(ArgumentErr() << "All match files must have the same number of IP.\n");
    }
    matches[i]          = ip1;
    matches[num_images] = ip2;
  }

  std::vector<std::vector<vw::ip::InterestPoint> > cam_matches = matches;

  std::string gcp_file;
  for (int i = 0; i < num_images; i++) {
    gcp_file += fs::basename(opt.image_files[i]);
    if (i < num_images - 1) gcp_file += "__"; 
  }
  gcp_file = opt.out_prefix + "-" + gcp_file + ".gcp";
  
  vw_out() << "Writing: " << gcp_file << std::endl;
  std::ofstream output_handle(gcp_file.c_str());

  int num_ips = matches[0].size();
  int pts_count = 0;
  for (int p = 0; p < num_ips; p++) { // Loop through IPs

    // Compute the GDC coordinate of the point
    ip::InterestPoint dem_ip = matches[num_images][p];
    Vector2 dem_pixel(dem_ip.x, dem_ip.y);
    Vector2 lonlat = georef_dem.pixel_to_lonlat(dem_pixel);
    
    if (dem_pixel[0] < 0 || dem_pixel[0] >= dem.cols() - 1 ||
        dem_pixel[1] < 0 || dem_pixel[1] >= dem.rows() - 1) {
      vw_out() << "Skipping pixel outside of DEM: " << dem_pixel << std::endl;
      continue;
    }
    
    PixelMask<float> mask_height = interp_dem(dem_pixel[0], dem_pixel[1])[0];
    if (!is_valid(mask_height)) continue;
    
    Vector3 llh(lonlat[0], lonlat[1], mask_height.child());
    //Vector3 dem_xyz = georef_dem.datum().geodetic_to_cartesian(llh);

    // The ground control point ID
    output_handle << pts_count;
    
    // Lat, lon, height
    output_handle << ", " << lonlat[1] << ", " << lonlat[0] << ", " << mask_height.child();

    // Sigma values
    output_handle << ", " << 1 << ", " << 1 << ", " << 1;

    // Write the per-image information
    for (int i = 0; i < num_images; i++) {

      // Take the ip in the map-projected image, and back-project it into
      // the camera
      ip::InterestPoint ip = matches[i][p];
      Vector2 ip_pix(ip.x, ip.y);
      Vector2 ll = img_georefs[i].pixel_to_lonlat(ip_pix);
      
      Vector2 dem_pix = georef_dem.lonlat_to_pixel(ll);
      if (dem_pix[0] < 0 || dem_pix[0] >= dem.cols() - 1) continue;
      if (dem_pix[1] < 0 || dem_pix[1] >= dem.rows() - 1) continue;
      PixelMask<double> dem_val = interp_dem(dem_pix[0], dem_pix[1]);
      if (!is_valid(dem_val)) continue;
      Vector3 llh(ll[0], ll[1], dem_val.child());
      Vector3 xyz = georef_dem.datum().geodetic_to_cartesian(llh);
      Vector2 cam_pix;
      try { cam_pix = opt.camera_models[i]->point_to_pixel(xyz); }
      catch(...){ continue; }
      ip.x = cam_pix.x(); ip.y = cam_pix.y();

      // TODO: Here we can have a book-keeping problem!
      cam_matches[i][p] = ip;
      
      output_handle << ", " <<  opt.image_files[i];
      output_handle << ", " << ip.x << ", " << ip.y; // IP location in image
      output_handle << ", " << 1 << ", " << 1; // Sigma values
    } // End loop through IP sets
    output_handle << std::endl; // Finish the line
    pts_count++;

  } // End loop through IPs
  output_handle.close();


  for (int i = 0; i < num_images; i++) {
    for (int j = i; j < num_images; j++) { // write also for i, i. Useful for only 1 image.

      std::string image1_path  = opt.image_files[i];
      std::string image2_path  = opt.image_files[j];
      std::string match_filename = ip::match_filename(opt.out_prefix, image1_path, image2_path);
      
      vw_out() << "Writing: " << match_filename << std::endl;
      ip::write_binary_match_file(match_filename, cam_matches[i], cam_matches[j]);
      
    }
  }

}

void handle_arguments( int argc, char *argv[], Options& opt ) {
  po::options_description general_options("");
  general_options.add_options()
//     ("cnet,c", po::value(&opt.cnet_file),
//      "Load a control network from a file (optional).")
    ("output-prefix,o",  po::value(&opt.out_prefix), "Prefix for output filenames.")
    ("bundle-adjuster",  po::value(&opt.ba_type)->default_value("Ceres"),
                        "Choose a solver from: Ceres, RobustSparse, RobustRef, Sparse, Ref.")
    ("cost-function",    po::value(&opt.cost_function)->default_value("Cauchy"),
                         "Choose a cost function from: Cauchy, PseudoHuber, Huber, L1, L2.")
    ("robust-threshold", po::value(&opt.robust_threshold)->default_value(0.5),
                         "Set the threshold for robust cost functions. Increasing this makes the solver focus harder on the larger errors.")
    ("local-pinhole",    po::bool_switch(&opt.local_pinhole_input)->default_value(false),
                         "Use special methods to handle a local coordinate input pinhole model.")
    ("fix-gcp-xyz",  po::bool_switch(&opt.fix_gcp_xyz)->default_value(false)->implicit_value(true),
                         "If the GCP are highly accurate, use this option to not float them during the optimization.")
    ("solve-intrinsics",  po::bool_switch(&opt.solve_intrinsics)->default_value(false)->implicit_value(true),
                         "Optimize intrinsic camera parameters.  Only used for pinhole cameras.")
    ("intrinsics-to-float", po::value(&opt.intrinsics_to_float_str)->default_value(""),
     "If solving for intrinsics and desired to float only a few of them, specify here, in quotes, one or more of: focal_length, optical_center, distortion_params.")
    ("camera-positions", po::value(&opt.camera_position_file)->default_value(""),
          "Specify a csv file path containing the estimated positions of the input cameras.  Only used with the local-pinhole option.")
    ("csv-format",       po::value(&opt.csv_format_str)->default_value(""), asp::csv_opt_caption().c_str())
    ("csv-proj4",        po::value(&opt.csv_proj4_str)->default_value(""),
                                 "The PROJ.4 string to use to interpret the entries in input CSV files.")
    ("datum",            po::value(&opt.datum_str)->default_value(""),
                         "Use this datum (needed only for ground control points or a camera position file). Options: WGS_1984, D_MOON (1,737,400 meters), D_MARS (3,396,190 meters), MOLA (3,396,000 meters), NAD83, WGS72, and NAD27. Also accepted: Earth (=WGS_1984), Mars (=D_MARS), Moon (=D_MOON).")
    ("semi-major-axis",  po::value(&opt.semi_major)->default_value(0),
                         "Explicitly set the datum semi-major axis in meters (needed only if ground control points are used).")
    ("semi-minor-axis",  po::value(&opt.semi_minor)->default_value(0),
                         "Explicitly set the datum semi-minor axis in meters (needed only if ground control points are used).")
    ("session-type,t",   po::value(&opt.stereo_session_string)->default_value(""),
                         "Select the stereo session type to use for processing. Options: pinhole isis dg rpc spot5 aster. Usually the program can select this automatically by the file extension.")
    ("min-matches",      po::value(&opt.min_matches)->default_value(30),
                         "Set the minimum  number of matches between images that will be considered.")
    ("ip-detect-method",po::value(&opt.ip_detect_method)->default_value(0),
                         "Interest point detection algorithm (0: Integral OBALoG (default), 1: OpenCV SIFT, 2: OpenCV ORB.")
    ("individually-normalize",   po::bool_switch(&opt.individually_normalize)->default_value(false)->implicit_value(true),
                        "Individually normalize the input images instead of using common values.")
    ("max-iterations",   po::value(&opt.max_iterations)->default_value(1000),
                         "Set the maximum number of iterations.")
    ("overlap-limit",    po::value(&opt.overlap_limit)->default_value(0),
                         "Limit the number of subsequent images to search for matches to the current image to this value.  By default match all images.")
    ("overlap-list",    po::value(&opt.overlap_list_file)->default_value(""),
     "A list of image pairs, one pair per line, separated by a space, which are expected to overlap. Matches are then computed only among the images in each pair.")
    ("position-filter-dist", po::value(&opt.position_filter_dist)->default_value(-1),
                         "Set a distance in meters and don't perform IP matching on images with an estimated camera center farther apart than this distance.  Requires --camera-positions.")
    ("camera-weight",    po::value(&opt.camera_weight)->default_value(1.0),
                         "The weight to give to the constraint that the camera positions/orientations stay close to the original values (only for the Ceres solver).  A higher weight means that the values will change less, a lower weight means more change.")
    ("ip-per-tile",             po::value(&opt.ip_per_tile)->default_value(0),
     "How many interest points to detect in each 1024^2 image tile (default: automatic determination).")
    ("min-triangulation-angle",             po::value(&opt.min_triangulation_angle)->default_value(0.1),
     "The minimum angle, in degrees, at which rays must meet at a triangulated point to accept this point as valid.")
    ("initial-transform",        po::value(&opt.init_transform_file)->default_value(""),
     "A file containing a 4x4 rotation + translation matrix to be used to initialize the adjustments (rather than using the identity). For example, a transform output by pc_align can be used.")
    
    ("mapprojected-data",  po::value(&opt.mapprojected_data)->default_value(""),
     "Given map-projected versions of the input images and the DEM mapprojected onto, and IP matches among them, create IP matches among the un-projected images before doing bundle adjustment. Niche and experimental, not for general use.")
    ("gcp-data",  po::value(&opt.gcp_data)->default_value(""),
     "Given map-projected versions of the input images and the DEM mapprojected onto, create GCP so that during bundle adjustment the original unprojected images are adjusted to mapproject where desired onto the DEM. Niche and experimental, not for general use.")
    ("lambda,l",         po::value(&opt.lambda)->default_value(-1),
                         "Set the initial value of the LM parameter lambda (ignored for the Ceres solver).")
    ("report-level,r",   po::value(&opt.report_level)->default_value(10),
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

  opt.gcp_files    = asp::get_files_with_ext( opt.image_files, ".gcp", true ); // Seperate out GCP files
  opt.camera_files = extract_cameras_bundle_adjust( opt.image_files );

  // If all we have are cubes, those are both images and cameras
  if (opt.image_files.empty())
    opt.image_files = opt.camera_files;

  // TODO: Check for duplicates in opt.image_files!

  if ( opt.image_files.empty() )
    vw_throw( ArgumentErr() << "Missing input image files.\n"
              << usage << general_options );

  if (opt.overlap_list_file != "" && opt.overlap_limit > 0)
    vw_throw( ArgumentErr() << "Cannot specify both the overlap limit and the overlap list.\n" << usage << general_options );
    
  if ( opt.overlap_limit < 0 )
    vw_throw( ArgumentErr() << "Must allow search for matches between "
              << "at least each image and its subsequent one.\n" << usage << general_options );
  // By default, try to match all of the images!
  if ( opt.overlap_limit == 0 )
    opt.overlap_limit = opt.image_files.size();

  if (opt.overlap_list_file != "") {
    if (!fs::exists(opt.overlap_list_file))
      vw_throw( ArgumentErr() << "The overlap list does not exist.\n" << usage << general_options );
    opt.overlap_list.clear();
    std::string image1, image2;
    std::ifstream ifs(opt.overlap_list_file.c_str());
    while (ifs >> image1 >> image2){
      opt.overlap_list.insert(std::pair<std::string, std::string>(image1, image2));
      opt.overlap_list.insert(std::pair<std::string, std::string>(image2, image1));
    }
    ifs.close();
  }
  
  if ( opt.camera_weight < 0.0 )
    vw_throw( ArgumentErr() << "The camera weight must be non-negative.\n" << usage << general_options );

  if (opt.local_pinhole_input && !asp::has_pinhole_extension(opt.camera_files[0]))
    vw_throw( ArgumentErr() << "Can't use special pinhole handling with non-pinhole input!\n");

  if (!opt.local_pinhole_input && opt.solve_intrinsics)
    vw_throw( ArgumentErr() << "Solving for intrinsic parameters is only supported with pinhole cameras.\n");

  // Copy the IP settings to the global stereosettings() object
  asp::stereo_settings().ip_matching_method     = opt.ip_detect_method;
  asp::stereo_settings().individually_normalize = opt.individually_normalize;

  if (!opt.camera_position_file.empty() && opt.csv_format_str == "")
    vw_throw( ArgumentErr() << "When using a camera position file, the csv-format option must be set.\n"
                            << usage << general_options );

  if ( !opt.gcp_files.empty() || !opt.camera_position_file.empty() ){
    // Need to read the datum if we have gcps or a camera position file.
    if (opt.datum_str != ""){
      // If the user set the datum, use it.
      opt.datum.set_well_known_datum(opt.datum_str);
    }else if (opt.semi_major > 0 && opt.semi_minor > 0){
      // Otherwise, if the user set the semi-axes, use that.
      opt.datum = cartography::Datum("User Specified Datum",
                                     "User Specified Spheroid",
                                     "Reference Meridian",
                                     opt.semi_major, opt.semi_minor, 0.0);
    }else{
      vw_throw( ArgumentErr() << "When ground control points or a camera position file are used, "
                              << "the datum must be specified.\n" << usage << general_options );
    }
    vw_out() << "Will use datum: " << opt.datum << std::endl;
  } // End datum finding

  if ( opt.out_prefix.empty() )
    vw_throw( ArgumentErr() << "Missing output prefix.\n"
              << usage << general_options  );

  // Create the output directory
  vw::create_out_dir(opt.out_prefix);

  // Turn on logging to file
  asp::log_to_file(argc, argv, "", opt.out_prefix);

  // Parse the intrinsics to float in a vector
  if (opt.intrinsics_to_float_str != "" && !opt.solve_intrinsics) {
    vw_throw( ArgumentErr() << "To be able to float only certain intrinsics, the option --solve-intrinsics must be on.\n" );
  }

  opt.intrinsics_to_float.clear();
  std::istringstream is(opt.intrinsics_to_float_str);
  std::string val;
  while (is >> val) 
    opt.intrinsics_to_float.insert(val);
  
  opt.save_iteration = vm.count("save-iteration-data");
  boost::to_lower( opt.stereo_session_string );
  boost::to_lower( opt.ba_type );
  boost::to_lower( opt.cost_function );
  if ( !( opt.ba_type == "ceres"        ||
          opt.ba_type == "robustsparse" ||
          opt.ba_type == "robustref"    ||
          opt.ba_type == "sparse"       ||
          opt.ba_type == "ref"
          ) )
    vw_throw( ArgumentErr() << "Unknown bundle adjustment version: " << opt.ba_type
              << ". Options are: [Ceres, RobustSparse, RobustRef, Sparse, Ref]\n" );
}

// ================================================================================

int main(int argc, char* argv[]) {

  Options opt;
  try {
    xercesc::XMLPlatformUtils::Initialize();
    
    handle_arguments( argc, argv, opt );

    int num_images = opt.image_files.size();
    // Create the stereo session. Try to auto-guess the session type.
    //if (num_images <= 1)
    //  vw_throw( ArgumentErr() << "Must have at least two image files to do bundle adjustment.\n" );

    // If there are no camera files, then the image files have the camera information.
    if (opt.camera_files.empty()){
      for (int i = 0; i < num_images; i++)
        opt.camera_files.push_back("");
    }

    // Ensure that no camera files have duplicate names.  This will cause the output files
    // to overwrite each other!
    for (int i=0; i< int(opt.camera_files.size()) - 1; ++i) {   
      std::string filename_1 = asp::bundle_adjust_file_name(opt.out_prefix,
                                              opt.image_files[i], opt.camera_files[i]);
      for (int j=i+1; j< int(opt.camera_files.size()); ++j) {
        std::string filename_2 = asp::bundle_adjust_file_name(opt.out_prefix,
                                                opt.image_files[j], opt.camera_files[j]);
        if (filename_1 == filename_2)
          vw_throw( ArgumentErr() << "All camera model files must have unique names!\n" );
      } 
    } // End loops for finding duplicate camera model file names

    // Sanity check
    if (num_images != (int)opt.camera_files.size()){
      vw_out() << "Detected " << num_images << " images and "
               << opt.camera_files.size() << " cameras.\n";
      vw_throw(ArgumentErr() << "Must have as many cameras as we have images.\n");
    }

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
                                                           opt.out_prefix
                                                           ));

      opt.camera_models.push_back(session->camera_model(opt.image_files [i],
                                                        opt.camera_files[i]));
    } // End loop through images loading all the camera models

    // Create match files from mapprojection.
    if (opt.mapprojected_data != "")
      create_matches_from_mapprojected_images(opt);

    // Create match files from mapprojection.
    if (opt.gcp_data != "") {
      create_gcp_from_mapprojected_images(opt);
      return 0;
    }
    
    // Create the match points
    // Iterate through each pair of input images
    std::map< std::pair<int, int>, std::string> match_files;

    // Load estimated camera positions if they were provided.
    std::vector<Vector3> estimated_camera_gcc;
    load_estimated_camera_positions(opt, estimated_camera_gcc);
    const bool got_est_cam_positions
      = (estimated_camera_gcc.size() == static_cast<size_t>(num_images));
    
    int num_pairs_matched = 0;
    for (int i = 0; i < num_images; i++){
      for (int j = i+1; j <= std::min(num_images-1, i+opt.overlap_limit); j++){

        std::string image1_path  = opt.image_files[i];
        std::string image2_path  = opt.image_files[j];
        
        // Look only at these pairs, if specified in a list
        if (!opt.overlap_list.empty()) {
          std::pair<std::string, std::string> pair(image1_path, image2_path);
          if (opt.overlap_list.find(pair) == opt.overlap_list.end()) continue;
        }
        
        // If this option is set, don't try to match cameras that are too far apart.
        if (got_est_cam_positions && (opt.position_filter_dist > 0)) {
          Vector3 this_pos  = estimated_camera_gcc[i];
          Vector3 other_pos = estimated_camera_gcc[j];
          if ( (this_pos  != Vector3(0,0,0)) && // If both positions are known
               (other_pos != Vector3(0,0,0)) && // and they are too far apart
               (norm_2(this_pos - other_pos) > opt.position_filter_dist) ) {
            vw_out() << "Skipping position: " << this_pos << " and "
		     << other_pos << " with distance " << norm_2(this_pos - other_pos) << std::endl;
            continue; // Skip this image pair
          }
        } // End estimated camera position filtering

      
        // Load both images into a new StereoSession object and use it to find interest points.
        // - The points are written to a file on disk.
        std::string camera1_path = opt.camera_files[i];
        std::string camera2_path = opt.camera_files[j];        
        std::string match_filename = ip::match_filename(opt.out_prefix, image1_path, image2_path);
        match_files[ std::pair<int, int>(i, j) ] = match_filename;
        if (fs::exists(match_filename)) {
          vw_out() << "\t--> Using cached match file: " << match_filename << "\n";
          ++num_pairs_matched;
          continue;
        }
        boost::shared_ptr<DiskImageResource>
          rsrc1(asp::load_disk_image_resource(image1_path, camera1_path)),
          rsrc2(asp::load_disk_image_resource(image2_path, camera2_path));
        if ( (rsrc1->channels() > 1) || (rsrc2->channels() > 1) )
          vw_throw(ArgumentErr() << "Error: Input images can only have a single channel!\n\n");
        float nodata1, nodata2;
        SessionPtr session(asp::StereoSessionFactory::create(opt.stereo_session_string, opt,
                                                             image1_path,  image2_path,
                                                             camera1_path, camera2_path,
                                                             opt.out_prefix
                                                             ));
        session->get_nodata_values(rsrc1, rsrc2, nodata1, nodata2);
        try{
          // IP matching may not succeed for all pairs

          // Get masked views of the images to get statistics from
          DiskImageView<float> image1_view(rsrc1), image2_view(rsrc2);
          ImageViewRef< PixelMask<float> > masked_image1
            = create_mask_less_or_equal(image1_view,  nodata1);
          ImageViewRef< PixelMask<float> > masked_image2
            = create_mask_less_or_equal(image2_view, nodata2);
          vw::Vector<vw::float32,6> image1_stats = asp::gather_stats(masked_image1, image1_path);
          vw::Vector<vw::float32,6> image2_stats = asp::gather_stats(masked_image2, image2_path);

          session->ip_matching(image1_path, image2_path,
                               Vector2(masked_image1.cols(), masked_image1.rows()),
                               image1_stats,
                               image2_stats,
                               opt.ip_per_tile,
                               nodata1, nodata2, match_filename,
                               opt.camera_models[i].get(),
                               opt.camera_models[j].get());
          ++num_pairs_matched;
        } catch ( const std::exception& e ){
          vw_out() << "Could not find interest points between images "
                   << opt.image_files[i] << " and " << opt.image_files[j] << std::endl;
          vw_out(WarningMessage) << e.what() << std::endl;
        } //End try/catch
      }
    } // End loop through all input image pairs

    //if (num_pairs_matched == 0) {
    //  vw_throw( ArgumentErr() << "Unable to find an IP based match between any input image pair!\n");
    // }

    // Try to set up the control network, ie the list of point coordinates.
    // - This triangulates from the camera models to determine the initial
    //   world coordinate estimate for each matched IP.
    opt.cnet.reset( new ControlNetwork("BundleAdjust") );
    if ( opt.cnet_file.empty() ) {
      bool success = vw::ba::build_control_network( true, // Always have input cameras
                                                    (*opt.cnet), opt.camera_models,
                                                    opt.image_files,
                                                    match_files,
                                                    opt.min_matches,
                                                    opt.min_triangulation_angle*(M_PI/180));
      if (!success) {
        vw_out() << "Failed to build a control network. Consider removing "
                 << "the currently found interest point matches and increasing "
                 << "the number of interest points per tile using "
                 << "--ip-per-tile, or decreasing --min-matches. Will continue "
                 << "if ground control points are present.\n";
        //return 1; // continue, hoping for gcp
      }

      vw::ba::add_ground_control_points( (*opt.cnet), opt.image_files,
                                         opt.gcp_files, opt.datum);
      // DEBUG
      //opt.cnet->write_binary(opt.out_prefix + "-control");
      //save_cnet_as_csv(opt, opt.out_prefix + "-cnet.csv");

      // End case where we had to build the control networks
    } else  {
      vw_out() << "Loading control network from file: "
               << opt.cnet_file << "\n";

      // Deciding which Control Network we have
      std::vector<std::string> tokens;
      boost::split( tokens, opt.cnet_file, boost::is_any_of(".") );
      if ( tokens.back() == "net" ) {
        // An ISIS style control network
        opt.cnet->read_isis( opt.cnet_file );
      } else if ( tokens.back() == "cnet" ) {
        // A VW binary style
        opt.cnet->read_binary( opt.cnet_file );
      } else {
        vw_throw( IOErr() << "Unknown Control Network file extension, \""
                  << tokens.back() << "\"." );
      }
    } // End control network loading case

    // If camera positions were provided for local inputs, align to them.
    const bool have_est_camera_positions = (opt.camera_position_file != "");
    if (opt.local_pinhole_input && have_est_camera_positions)
      init_pinhole_model_with_camera_positions(opt, estimated_camera_gcc);

    // If we have GPC's for pinhole cameras, try to do a simple affine initialization
    //  of the camera parameters.
    // - This function also updates all the ControlNetwork world point positions
    // - We could do this for other camera types too, but it would require us to be able
    //   to adjust our camera model positions.  Otherwise we could init the adjustment values...
    if (opt.gcp_files.size() > 0) {

      if (opt.local_pinhole_input && !have_est_camera_positions)
        init_pinhole_model_with_gcp(opt);

      // Issue a warning if the GCPs are far away from the camera coords
      check_gcp_dists(opt);
    }


    if (opt.local_pinhole_input == false) {
      do_ba_with_model<BundleAdjustmentModel>(opt);
    }
    else{ // Use for local pinhole models, could also be used for other pinhole models.

      BAPinholeModel ba_model(opt.camera_models, opt.cnet, opt.solve_intrinsics);

      // Create new camera models from scratch
      do_ba_ceres<BAPinholeModel>(ba_model, opt);

      // Save the camera models to disk
      std::vector<std::string> cam_files;
      for (int icam = 0; icam < (int)opt.camera_models.size(); icam++){
        std::string cam_file = asp::bundle_adjust_file_name(opt.out_prefix,
                                                            opt.image_files[icam],
                                                            opt.camera_files[icam]);
        cam_file = fs::path(cam_file).replace_extension("tsai").string();
        cam_files.push_back(cam_file);

        //std::cout << "Camera output GDC coordinate: " <<
        //   opt.datum.cartesian_to_geodetic(ba_model.get_camera_model(icam).camera_center()) << std::endl;
        // std::cout << "Pixel vector 100,100: " <<
        //   ba_model.get_camera_model(icam).pixel_to_vector(Vector2(100, 100)) << std::endl;

      }
      ba_model.write_camera_models(cam_files);

      //double error=0;
      //stereo::StereoModel sm(ba_model.get_camera_model(0), ba_model.get_camera_model(1));
      //Vector3 position sm(Vector2(), Vector2(), error);
      //std::cout << "Tri test position: "<< position << " error " << error << std::endl;

    } // End BAPinhole case

    //// Verify the solver accuracy
    //if ((opt.gcp_files.size() > 0) && (opt.stereo_session_string == "pinhole"))
    //  init_pinhole_model_with_gcp(opt, true);

    xercesc::XMLPlatformUtils::Terminate();

  } ASP_STANDARD_CATCHES;
}
