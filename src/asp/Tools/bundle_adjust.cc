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

#include <vw/FileIO/KML.h>
#include <vw/Camera/CameraUtilities.h>
#include <asp/Core/Macros.h>
#include <asp/Sessions/StereoSession.h>
#include <asp/Sessions/StereoSessionFactory.h>
#include <asp/Core/StereoSettings.h>
#include <asp/Core/PointUtils.h>
#include <asp/Tools/bundle_adjust.h>
#include <asp/Core/InterestPointMatching.h>
#include <xercesc/util/PlatformUtils.hpp>
#include <asp/Core/EigenUtils.h>

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

const size_t PIXEL_SIZE = 2;

std::string UNSPECIFIED_DATUM = "unspecified_datum";
typedef boost::scoped_ptr<asp::StereoSession> SessionPtr;

int g_ba_num_errors = 0;
Mutex g_ba_mutex;

// TODO: Pas this properly
double g_max_disp_error = -1;

typedef PixelMask< Vector<float, 2> > DispPixelT;


//==================================================================================
// IP filtering functions -> Move them somewhere else once they are settled

/// Filter IP points by how reasonably the disparity can change along rows
size_t filter_ip_homog(std::vector<ip::InterestPoint> const& ip1_in,
                       std::vector<ip::InterestPoint> const& ip2_in,
                       std::vector<ip::InterestPoint>      & ip1_out,
                       std::vector<ip::InterestPoint>      & ip2_out,
                       int inlier_threshold = 1) {

  std::vector<size_t> indices;
  try {
  
    std::vector<Vector3> ransac_ip1 = iplist_to_vectorlist(ip1_in),
                         ransac_ip2 = iplist_to_vectorlist(ip2_in);
  
    typedef math::RandomSampleConsensus<math::HomographyFittingFunctor, math::InterestPointErrorMetric> RansacT;
    const int    MIN_NUM_OUTPUT_INLIERS = ransac_ip1.size()/2;
    const int    NUM_ITERATIONS         = 100;
    RansacT ransac( math::HomographyFittingFunctor(),
                    math::InterestPointErrorMetric(), NUM_ITERATIONS,
                    inlier_threshold,
                    MIN_NUM_OUTPUT_INLIERS, true
                  );
    Matrix<double> H(ransac(ransac_ip2,ransac_ip1)); // 2 then 1 is used here for legacy reasons
    vw_out() << "\t--> Homography: " << H << "\n";
    indices = ransac.inlier_indices(H,ransac_ip2,ransac_ip1);
  } catch (const math::RANSACErr& e ) {
    vw_out() << "RANSAC Failed: " << e.what() << "\n";
    return false;
  }

  // Assemble the remaining interest points
  const size_t num_left = indices.size();
  std::vector<ip::InterestPoint> final_ip1, final_ip2;
  ip1_out.resize(num_left);
  ip2_out.resize(num_left);
  for (size_t i=0; i<num_left; ++i) {
    size_t index = indices[i];
    ip1_out[i] = ip1_in[index];
    ip2_out[i] = ip2_in[index];
  }

  return num_left;
}

// IP pair sorting functions
typedef std::pair<ip::InterestPoint, ip::InterestPoint> IpPair;

bool comp_ip_col(IpPair const& a, IpPair const& b) {
  if (a.first.x == b.first.x)
    return a.first.y < b.first.y;
  return a.first.x < b.first.x;
}
bool comp_ip_row(IpPair const& a, IpPair const& b) {
  if (a.first.y == b.first.y)
    return a.first.x < b.first.x;
  return a.first.y < b.first.y;
}

/// Filter IP points by how reasonably the disparity can change along rows
size_t filter_ip_sides(std::vector<ip::InterestPoint> const& ip1_in,
                       std::vector<ip::InterestPoint> const& ip2_in,
                       std::vector<ip::InterestPoint>      & ip1_out,
                       std::vector<ip::InterestPoint>      & ip2_out,
                       int image_width1, int image_width2,
                       int side_percentage = 20) {

  const size_t num_pairs_in = ip1_in.size();
  /*
  // Create list of IP pairs sorted by column 1 (low to high)
  std::vector<IpPair> ip_pairs_in(num_pairs_in);
  for (size_t i=0; i<num_pairs_in; ++i) {
    ip_pairs_in[i].first  = ip1_in[i];
    ip_pairs_in[i].second = ip2_in[i];
  }
  std::sort(ip_pairs_in.begin(), ip_pairs_in.end(), comp_ip_col);

  // Get disparities for each pair
  std::vector<double> dx(num_pairs_in);
  for (size_t i=0; i<num_pairs_in; ++i) {
    dx[i] = ip_pairs_in[i].second.x - ip_pairs_in[i].first.x;
    std::cout << dx[i] << std::endl;
  }
  */
  
  // TODO: Automatically calculate what counts as the "side"
  // Filter out IP too close to the sides
  
  double crop_percent = static_cast<double>(side_percentage) / 100.0;
  vw_out() << "Removing IP within " << crop_percent << " percent of the outer left/right sides of the images.\n";
  int crop_size1 = image_width1 * crop_percent;
  int crop_size2 = image_width1 * crop_percent;
  
  ip1_out.clear();
  ip2_out.clear();
  double kept_mean = 0, reject_mean = 0;
  for (size_t i=0; i<num_pairs_in; ++i) {
    bool   keep = ((ip1_in[i].x >= crop_size1) && ((image_width2 - ip2_in[i].x) > crop_size2));
    double disp = ip2_in[i].x - ip1_in[i].x;
    if (keep) {
      ip1_out.push_back(ip1_in[i]);
      ip2_out.push_back(ip2_in[i]);
      kept_mean += disp;
    }
    else {
      reject_mean += disp;
    }
  }
  size_t num_left     = ip1_out.size();
  size_t num_rejected = num_pairs_in - num_left;
  
  if (num_left > 0) {
    kept_mean /= static_cast<double>(num_left);
    vw_out() << "Mean disparity of points kept by side disparity = " << kept_mean << std::endl;
  }
  if (num_rejected > 0) {
    reject_mean /= static_cast<double>(num_rejected);
    vw_out() << "Mean disparity of points rejected by side disparity = " << reject_mean << std::endl;
  }
  
  return num_left;
}

double calc_ip_coverage_fraction(std::vector<ip::InterestPoint> const& ip,
                                 Vector2i const& image_size, int tile_size=1024) {

  if (tile_size < 1)
    vw_throw(LogicErr() << "calc_ip_coverage_fraction: tile size is " << tile_size);

  // Generate a grid of ROIs covering the entire image
  BBox2i full_bbox(Vector2i(0,0), image_size);
  bool include_partials = false;
  std::vector<BBox2i> rois;
  rois = subdivide_bbox(full_bbox, tile_size, tile_size, include_partials);
  const size_t num_rois = rois.size();
  if (num_rois == 0)
    return 0; // Cannot have any coverage in the degenerate case!
  
  // Pack all IP into a list for speed
  std::list<Vector2i> ip_list;
  for (size_t i=0; i<ip.size(); ++i) {
    ip_list.push_back(Vector2i(ip[i].x, ip[i].y));
  }
  
  // Count up the pecentage of ROIs which have at least this many IP.
  const int MIN_ROI_IP = 2;
  
  size_t num_filled_rois = 0;
  for (size_t i=0; i<num_rois; ++i) { // Loop through ROIs
    int ip_in_roi = 0;
    
    // Check if each point is in this ROI
    std::list<Vector2i>::iterator iter;
    for (iter=ip_list.begin(); iter!=ip_list.end(); ++iter) {
      
      // If the IP is in the ROI, remove it from the IP list so it
      // does not get searched again.
      if (rois[i].contains(*iter)) {
        iter = ip_list.erase(iter);
        ++ip_in_roi;
        --iter;
      }
    } // End IP loop
    if (ip_in_roi > MIN_ROI_IP)
      ++num_filled_rois;
  }// End ROI loop

  return static_cast<double>(num_filled_rois) / static_cast<double>(num_rois);
}

//==================================================================================


struct Options : public vw::cartography::GdalWriteOptions {
  std::vector<std::string> image_files, camera_files, gcp_files;
  std::string cnet_file, out_prefix, input_prefix, stereo_session_string,
    cost_function, ba_type, mapprojected_data, gcp_data;
  int    ip_per_tile, ip_edge_buffer_percent;
  double min_triangulation_angle, lambda, camera_weight, rotation_weight, 
    translation_weight, overlap_exponent, robust_threshold, parameter_tolerance;
  int    report_level, min_matches, max_iterations, overlap_limit;
  bool   save_iteration, create_pinhole, approximate_pinhole_intrinsics,
         fix_gcp_xyz, solve_intrinsics,
         disable_tri_filtering, ip_normalize_tiles, ip_debug_images;
  std::string datum_str, camera_position_file, initial_transform_file,
    csv_format_str, csv_proj4_str, reference_terrain, disparity_list, intrinsics_to_float_str,
    heights_from_dem;
  double semi_major, semi_minor, position_filter_dist;
  int num_ba_passes, max_num_reference_points;
  std::string remove_outliers_params_str;
  vw::Vector<double, 4> remove_outliers_params;
  vw::Vector2 remove_outliers_by_disp_params;
  boost::shared_ptr<ControlNetwork> cnet;
  std::vector<boost::shared_ptr<CameraModel> > camera_models;
  cartography::Datum datum;
  int    ip_detect_method, num_scales;
  double epipolar_threshold; // Max distance from epipolar line to search for IP matches.
  double ip_inlier_factor, ip_uniqueness_thresh, nodata_value, max_disp_error;
  bool   skip_rough_homography, individually_normalize, use_llh_error;
  vw::Vector2  elevation_limit;     // Expected range of elevation to limit results to.
  vw::BBox2    lon_lat_limit;       // Limit the triangulated interest points to this lonlat range
  std::set<std::string> intrinsics_to_float;
  std::string           overlap_list_file;
  std::set< std::pair<std::string, std::string> > overlap_list;
  vw::Matrix4x4 initial_transform;
  std::string fixed_cameras_indices_str;
  std::set<int> fixed_cameras_indices;
  
  // Make sure all values are initialized, even though they will be
  // over-written later.
  Options(): ip_per_tile(0), min_triangulation_angle(0), lambda(-1.0), camera_weight(-1),
             rotation_weight(0), translation_weight(0), overlap_exponent(0), 
             robust_threshold(0), report_level(0), min_matches(0),
             max_iterations(0), overlap_limit(0), save_iteration(false),
             create_pinhole(false), fix_gcp_xyz(false), solve_intrinsics(false),
             semi_major(0), semi_minor(0), position_filter_dist(-1),
             num_ba_passes(1), max_num_reference_points(-1),
             datum(cartography::Datum(UNSPECIFIED_DATUM, "User Specified Spheroid",
                                      "Reference Meridian", 1, 1, 0)),
             ip_detect_method(0), num_scales(-1), skip_rough_homography(false),
             individually_normalize(false), use_llh_error(false){}
};

// TODO: This update stuff should really be done somewhere else!
//       Also the comments may be wrong.

// This version does nothing, all camera parameters will start at zero,
// unless an initial transform is to be applied.
// - This is for the BundleAdjustmentModel class where the camera parameters
//   are a rotation/offset that is applied on top of the existing camera model.
template<class ModelT> void
update_cnet_and_init_cams(ModelT & ba_model, Options & opt,
                          ControlNetwork& cnet,
                          std::vector<double> & cameras_vec,
                          std::vector<double> & intrinsics_vec){

  if (opt.initial_transform_file != "" && opt.input_prefix != "")
    vw_throw( ArgumentErr() << "Cannot have both input adjustments and an input transform.\n" );

  // Read the pc_align transform from disk and apply it
  if (opt.initial_transform_file != "")
    ba_model.import_transform(opt.initial_transform, cameras_vec);

  // Read the adjustments from a previous run
  if (opt.input_prefix != "") {
    for (size_t icam = 0; icam < ba_model.num_cameras(); icam++){
      std::string adjust_file = asp::bundle_adjust_file_name(opt.input_prefix,
                                                             opt.image_files[icam],
                                                             opt.camera_files[icam]);

      ba_model.read_adjustment(icam, adjust_file, cameras_vec);
    }
  }
  
}

/// Specialization for pinhole cameras, copy the camera
///  parameters from the control network into the vectors.
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

  // First apply any transform to the pinhole cameras
  if (opt.initial_transform_file != "") 
    ba_model.import_transform(opt.initial_transform, cameras_vec);
  
  // Copy the camera parameters from the model to cameras_vec
  int index = 0;
  for (int i=0; i < num_cameras; ++i) {
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
  bool operator()(const T* camera, const T* point, T* residuals) const {

    try{

      size_t num_cameras = m_ba_model->num_cameras();
      size_t num_points  = m_ba_model->num_points();
      VW_ASSERT(m_icam < num_cameras, ArgumentErr() << "Out of bounds in the number of cameras.");
      VW_ASSERT(m_ipt  < num_points , ArgumentErr() << "Out of bounds in the number of points." );

      // Copy the input data to structures expected by the BA model
      typename ModelT::camera_vector_t cam_vec;
      int cam_len = cam_vec.size();
      for (int c = 0; c < cam_len; c++)
        cam_vec[c] = camera[c];

      typename ModelT::point_vector_t  point_vec;
      for (size_t p = 0; p < point_vec.size(); p++)
        point_vec[p] = point[p];

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
                 BAPinholeModel * const ba_model, size_t icam, size_t ipt, int num_parameters):
    m_observation(observation),
    m_pixel_sigma(pixel_sigma),
    m_ba_model(ba_model),
    m_icam(icam), m_ipt(ipt),
    m_num_parameters(num_parameters) {}

  // Adaptor to work with ceres::DynamicCostFunctions.
  bool operator()(double const * const * parameters, double * residuals) const {
    VW_ASSERT(m_num_parameters >= 2, ArgumentErr() << "Require at least parameters for camera and point.");
    const double * const camera_ptr = parameters[0];
    const double * const point_ptr  = parameters[1];
    const double * const focal_ptr  = m_num_parameters >= 3 ? parameters[2] : nullptr;
    const double * const center_ptr = m_num_parameters >= 4 ? parameters[3] : nullptr;
    const double * const distortion_ptr = m_num_parameters >= 5 ? parameters[4] : nullptr;
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
      BAPinholeModel::camera_intr_vector_t cam_intr_vec;
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

      // Multiply the original intrinsics by their relative intrinsics
      // being optimized
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
    if (num_parameters > 0 && num_parameters <= kFocalLengthAndPrincipalPoint) {
      num_parameters += 2;
    } else if (num_parameters > kFocalLengthAndPrincipalPoint) {
      num_parameters += 3;
    }

    ceres::DynamicNumericDiffCostFunction<BaPinholeError>* cost_function =
        new ceres::DynamicNumericDiffCostFunction<BaPinholeError>(
            new BaPinholeError(observation, pixel_sigma, ba_model, icam, ipt,
                               num_parameters));
    cost_function->SetNumResiduals(nob);
    cost_function->AddParameterBlock(ncp);
    cost_function->AddParameterBlock(npp);

    if (num_intrinsics == 0) {
      // This is the special case that we are not solving camera intrinsics.
      return cost_function;
    }

    cost_function->AddParameterBlock(nf);
    cost_function->AddParameterBlock(nc);

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
                   BAPinholeModel const& ba_model, size_t left_icam, size_t right_icam):
    m_reference_xyz(reference_xyz),
    m_interp_disp(interp_disp),
    m_ba_model(ba_model),
    m_left_icam(left_icam), m_right_icam(right_icam){}

  /// Compute residuals of observing this point with these camera parameters
  bool operator()(const double * const left_camera,
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
      if (left_prediction[0] < 0 || left_prediction[0] > m_interp_disp.cols()-1 ||
	  left_prediction[1] < 0 || left_prediction[1] > m_interp_disp.rows()-1) {
	good_ans = false;
      }else{

        DispPixelT dispPix = m_interp_disp(left_prediction[0], left_prediction[1]);
        if (!is_valid(dispPix)) {
	  good_ans = false;
	}else{
	  Vector2 right_prediction_from_disp = left_prediction + dispPix.child();
	  residuals[0] = right_prediction_from_disp[0] - right_prediction [0];
	  residuals[1] = right_prediction_from_disp[1] - right_prediction [1];
	}
      }

      // TODO: Think more of what to do below. The hope is that the robust cost
      // function will take care of big residuals graciously.
      if (!good_ans) {
	// Failed to find the residuals
	residuals[0] = g_max_disp_error;
	residuals[1] = g_max_disp_error;
	return true;
      }

    } catch (const camera::PointToPixelErr& e) {
      // Failed to project into the camera
      residuals[0] = g_max_disp_error;
      residuals[1] = g_max_disp_error;
      return true;
    }
    return true;
  }

  /// Overload for when there is no distortion
  bool operator()(const double * const left_camera,
                  const double * const right_camera,
                  const double * const scaled_focal_length,
                  const double * const scaled_optical_center,
                  double       * residuals) const {
    return this->operator()(left_camera, right_camera, scaled_focal_length,
                            scaled_optical_center, 0, residuals);
  }
  
  /// Overload for when all intrinsics are in one vector
  bool operator()(const double * const left_camera,
                  const double * const right_camera,
                  const double * const scaled_intrinsics,
                  double       * residuals) const {
    int nf = BAPinholeModel::focal_length_params_n;
    int nc = BAPinholeModel::optical_center_params_n;
    return this->operator()(left_camera, right_camera,
                            scaled_intrinsics, scaled_intrinsics + nf,
                            scaled_intrinsics + nf + nc,
                            residuals);
  }
  
  /// Overload for when intrinsic parameters are not provided.
  bool operator()(const double * const left_camera,
                  const double * const right_camera,
                  double       * residuals) const {
    // Just call the other function with a dummy intrinsic vector which will not be used.
    return this->operator()(left_camera, right_camera, 0, residuals);
  }

  // Factory to hide the construction of the CostFunction object from
  // the client code.
  static ceres::CostFunction* Create(Vector3 const& reference_xyz,
				     ImageViewRef<DispPixelT> const& interp_disp,
                                     BAPinholeModel const& ba_model,
                                     size_t left_icam,
                                     size_t right_icam){
    
    const int nob = PIXEL_SIZE; // Num observation elements: Column, row
    const int ncp = BAPinholeModel::camera_params_n;
    const int nf  = BAPinholeModel::focal_length_params_n;
    const int nc  = BAPinholeModel::optical_center_params_n;
    const int num_intrinsics = ba_model.num_intrinsic_params();

    // Create a ceres::AutoDiffCostFunction object templated to the
    // exact problem sizes we need. Notice that if we have more than 3 intrinsics that
    // means focal length (1 param), optical center (2 params), and the rest are
    // distortion params.

    // TODO: Use a macro below
    
    switch (num_intrinsics) {
    case 0: // This case is different, it does not set an intrinsic size.
      return (new ceres::NumericDiffCostFunction<BaDispXyzError,ceres::CENTRAL, nob, ncp, ncp>(new BaDispXyzError(reference_xyz, interp_disp, ba_model, left_icam, right_icam)));
      // All of the other cases hard code the intrinsic length.
    case 1:
      vw_throw(LogicErr() << "bundle_adjust.cc not set up for 1 intrinsic param!");
    case 2:
      vw_throw(LogicErr() << "bundle_adjust.cc not set up for 2 intrinsic params!");
    case 3: return (new ceres::NumericDiffCostFunction<BaDispXyzError,ceres::CENTRAL, nob, ncp, ncp, nf, nc>(new BaDispXyzError(reference_xyz, interp_disp, ba_model, left_icam, right_icam))); // no distortion
    case 4:  return (new ceres::NumericDiffCostFunction<BaDispXyzError,ceres::CENTRAL, nob, ncp, ncp, nf, nc, 1>(new BaDispXyzError(reference_xyz, interp_disp, ba_model, left_icam, right_icam)));
    case 5:  return (new ceres::NumericDiffCostFunction<BaDispXyzError,ceres::CENTRAL, nob, ncp, ncp, nf, nc, 2>(new BaDispXyzError(reference_xyz, interp_disp, ba_model, left_icam, right_icam)));
    case 6:  return (new ceres::NumericDiffCostFunction<BaDispXyzError,ceres::CENTRAL, nob, ncp, ncp, nf, nc, 3>(new BaDispXyzError(reference_xyz, interp_disp, ba_model, left_icam, right_icam)));
    case 7:  return (new ceres::NumericDiffCostFunction<BaDispXyzError,ceres::CENTRAL, nob, ncp, ncp, nf, nc, 4>(new BaDispXyzError(reference_xyz, interp_disp, ba_model, left_icam, right_icam)));
    case 8:  return (new ceres::NumericDiffCostFunction<BaDispXyzError,ceres::CENTRAL, nob, ncp, ncp, nf, nc, 5>(new BaDispXyzError(reference_xyz, interp_disp, ba_model, left_icam, right_icam)));
    case 9:  return (new ceres::NumericDiffCostFunction<BaDispXyzError,ceres::CENTRAL, nob, ncp, ncp, nf, nc, 6>(new BaDispXyzError(reference_xyz, interp_disp, ba_model, left_icam, right_icam)));
    case 10:  return (new ceres::NumericDiffCostFunction<BaDispXyzError,ceres::CENTRAL, nob, ncp, ncp, nf, nc, 7>(new BaDispXyzError(reference_xyz, interp_disp, ba_model, left_icam, right_icam)));
    case 11:  return (new ceres::NumericDiffCostFunction<BaDispXyzError,ceres::CENTRAL, nob, ncp, ncp, nf, nc, 8>(new BaDispXyzError(reference_xyz, interp_disp, ba_model, left_icam, right_icam)));
    case 12:  return (new ceres::NumericDiffCostFunction<BaDispXyzError,ceres::CENTRAL, nob, ncp, ncp, nf, nc, 9>(new BaDispXyzError(reference_xyz, interp_disp, ba_model, left_icam, right_icam)));
    case 13:  return (new ceres::NumericDiffCostFunction<BaDispXyzError,ceres::CENTRAL, nob, ncp, ncp, nf, nc, 10>(new BaDispXyzError(reference_xyz, interp_disp, ba_model, left_icam, right_icam)));
    case 14:  return (new ceres::NumericDiffCostFunction<BaDispXyzError,ceres::CENTRAL, nob, ncp, ncp, nf, nc, 11>(new BaDispXyzError(reference_xyz, interp_disp, ba_model, left_icam, right_icam)));
    case 15:  return (new ceres::NumericDiffCostFunction<BaDispXyzError,ceres::CENTRAL, nob, ncp, ncp, nf, nc, 12>(new BaDispXyzError(reference_xyz, interp_disp, ba_model, left_icam, right_icam)));
    case 16:  return (new ceres::NumericDiffCostFunction<BaDispXyzError,ceres::CENTRAL, nob, ncp, ncp, nf, nc, 13>(new BaDispXyzError(reference_xyz, interp_disp, ba_model, left_icam, right_icam)));
    case 17:  return (new ceres::NumericDiffCostFunction<BaDispXyzError,ceres::CENTRAL, nob, ncp, ncp, nf, nc, 14>(new BaDispXyzError(reference_xyz, interp_disp, ba_model, left_icam, right_icam)));
    case 18:  return (new ceres::NumericDiffCostFunction<BaDispXyzError,ceres::CENTRAL, nob, ncp, ncp, nf, nc, 15>(new BaDispXyzError(reference_xyz, interp_disp, ba_model, left_icam, right_icam)));
    case 19:  return (new ceres::NumericDiffCostFunction<BaDispXyzError,ceres::CENTRAL, nob, ncp, ncp, nf, nc, 16>(new BaDispXyzError(reference_xyz, interp_disp, ba_model, left_icam, right_icam)));
    case 20:  return (new ceres::NumericDiffCostFunction<BaDispXyzError,ceres::CENTRAL, nob, ncp, ncp, nf, nc, 17>(new BaDispXyzError(reference_xyz, interp_disp, ba_model, left_icam, right_icam)));

    case vw::camera::RPCLensDistortion::num_distortion_params+3:  return (new ceres::NumericDiffCostFunction<BaDispXyzError,ceres::CENTRAL, nob, ncp, ncp, nf, nc, vw::camera::RPCLensDistortion::num_distortion_params>(new BaDispXyzError(reference_xyz, interp_disp, ba_model, left_icam, right_icam)));

    case vw::camera::RPCLensDistortion5::num_distortion_params+3:  return (new ceres::NumericDiffCostFunction<BaDispXyzError,ceres::CENTRAL, nob, ncp, ncp, nf, nc, vw::camera::RPCLensDistortion5::num_distortion_params>(new BaDispXyzError(reference_xyz, interp_disp, ba_model, left_icam, right_icam)));
      
    case vw::camera::RPCLensDistortion6::num_distortion_params+3:  return (new ceres::NumericDiffCostFunction<BaDispXyzError,ceres::CENTRAL, nob, ncp, ncp, nf, nc, vw::camera::RPCLensDistortion6::num_distortion_params>(new BaDispXyzError(reference_xyz, interp_disp, ba_model, left_icam, right_icam)));

    default:
      vw_throw(LogicErr() << "bundle_adjust.cc not set up for this many intrinsic params!");
    };
    return 0;
  } // End function Create

  Vector3 m_reference_xyz;
  ImageViewRef<DispPixelT> const& m_interp_disp;
  BAPinholeModel const& m_ba_model;
  size_t m_left_icam, m_right_icam;
  
};

// Add residual block for the error using reference xyz
// This one does nothing, as the functionality is implemented only for pinhole models
template<class ModelT>
void add_residual_block(Vector3 const& reference_xyz,
                        ImageViewRef<DispPixelT> const& interp_disp, 
                        ModelT & ba_model,
                        size_t left_icam, size_t right_icam,
                        double * left_camera, double * right_camera,
                        double * scaled_intrinsics,
                        std::set<std::string> const& intrinsics_to_float,
                        ceres::LossFunction* loss_function,
                        ceres::Problem & problem){
}

// Add residual block for the error using reference xyz for pinhole
template<>
void add_residual_block<BAPinholeModel>(Vector3 const& reference_xyz,
                                        ImageViewRef<DispPixelT> const& interp_disp, 
                                        BAPinholeModel & ba_model,
                                        size_t left_icam, size_t right_icam,
                                        double * left_camera, double * right_camera,
                                        double * scaled_intrinsics,
                                        std::set<std::string> const& intrinsics_to_float,
                                        ceres::LossFunction* loss_function,
                                        ceres::Problem & problem){

  ceres::CostFunction* cost_function =
    BaDispXyzError::Create(reference_xyz, interp_disp, ba_model, left_icam, right_icam);

    // If the intrinsics are constant, do not even include them in the optimization
  if (ba_model.are_intrinsics_constant()) {
    problem.AddResidualBlock(cost_function, loss_function, left_camera, right_camera);
  } else {

    // We will float some or more of the intrinsics

    int nf = BAPinholeModel::focal_length_params_n;
    int nc = BAPinholeModel::optical_center_params_n;

    const int num_distortion_params = ba_model.num_distortion_params();
    if (num_distortion_params == 0) 
      problem.AddResidualBlock(cost_function, loss_function, left_camera, right_camera,
                               scaled_intrinsics,          // focal length
                               scaled_intrinsics + nf      // optical center
                               );
    else{
      problem.AddResidualBlock(cost_function, loss_function, left_camera, right_camera,
                               scaled_intrinsics,          // focal length      (1 param)
                               scaled_intrinsics + nf,     // optical center    (2 params)
                               scaled_intrinsics + nf + nc // distortion params (all else)
                               );
    }
    
    // See if to float only certain intrinsics
    if (!intrinsics_to_float.empty()) {

      if (intrinsics_to_float.find("focal_length") == intrinsics_to_float.end()) {
        //vw_out() << "Will not float focal length.\n";
        problem.SetParameterBlockConstant(scaled_intrinsics);
      }else{
        //vw_out() << "Will float focal length.\n";
      }

      if (intrinsics_to_float.find("optical_center") == intrinsics_to_float.end()) {
        //vw_out() << "Will not float optical center.\n";
        problem.SetParameterBlockConstant(scaled_intrinsics + nf);
      }else{
        //vw_out() << "Will float optical center.\n";
      }

      if (intrinsics_to_float.find("distortion_params") == intrinsics_to_float.end()) {
        //vw_out() << "Will not float distortion parameters.\n";
        if (num_distortion_params > 0)
          problem.SetParameterBlockConstant(scaled_intrinsics + nf + nc);
      }else{
        //vw_out() << "Will float distortion parameters.\n";
      }
      
    } 
    
  } // end dealing with intrinsics
  
}

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

// A ceres cost function. The residual is the rotation + translation
// vector difference, each multiplied by a weight. Hence, a larger
// rotation weight will result in less rotation change in the final
// result, etc. This is somewhat different than CamError as there is no
// penalty here for this cost function going very large, the scaling is
// different, and there is finger-grained control. 
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


// Add residual block without floating intrinsics for the reprojection error
template<class ModelT>
void add_residual_block(ModelT & ba_model,
                        Vector2 const& observation, Vector2 const& pixel_sigma,
                        size_t icam, size_t ipt,
                        double * camera, double * point, double * scaled_intrinsics,
                        std::set<std::string> const& intrinsics_to_float,
                        ceres::LossFunction* loss_function,
                        ceres::Problem & problem){

  ceres::CostFunction* cost_function =
    BaReprojectionError<ModelT>::Create(observation, pixel_sigma,
                                        &ba_model, icam, ipt);
  problem.AddResidualBlock(cost_function, loss_function, camera, point);
}

// Add residual block for the reprojection error, optionally floating the intrinsics
template<>
void add_residual_block<BAPinholeModel>
                  (BAPinholeModel & ba_model,
                   Vector2 const& observation, Vector2 const& pixel_sigma,
                   size_t icam, size_t ipt,
                   double * camera, double * point, double * scaled_intrinsics,
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

    // Use a special cost function using intrinsics

    ceres::CostFunction* cost_function =
      BaPinholeError::Create(observation, pixel_sigma, &ba_model, icam, ipt);

    int nf = BAPinholeModel::focal_length_params_n;
    int nc = BAPinholeModel::optical_center_params_n;

    const int num_distortion_params = ba_model.num_distortion_params();
    if (num_distortion_params == 0) 
      problem.AddResidualBlock(cost_function, loss_function, camera, point,
                               scaled_intrinsics,          // focal length
                               scaled_intrinsics + nf      // optical center
                               );
    else{
      problem.AddResidualBlock(cost_function, loss_function, camera, point,
                               scaled_intrinsics,          // focal length      (1 param)
                               scaled_intrinsics + nf,     // optical center    (2 params)
                               scaled_intrinsics + nf + nc // distortion params (all else)
                               );
    }
    
    // See if to float only certain intrinsics
    if (!intrinsics_to_float.empty()) {

      if (intrinsics_to_float.find("focal_length") == intrinsics_to_float.end()) {
        //vw_out() << "Will not float focal length.\n";
        problem.SetParameterBlockConstant(scaled_intrinsics);
      }else{
        //vw_out() << "Will float focal length.\n";
      }

      if (intrinsics_to_float.find("optical_center") == intrinsics_to_float.end()) {
        //vw_out() << "Will not float optical center.\n";
        problem.SetParameterBlockConstant(scaled_intrinsics + nf);
      }else{
        //vw_out() << "Will float optical center.\n";
      }

      if (intrinsics_to_float.find("distortion_params") == intrinsics_to_float.end()) {
        //vw_out() << "Will not float distortion parameters.\n";
        if (num_distortion_params > 0)
          problem.SetParameterBlockConstant(scaled_intrinsics + nf + nc);
      }else{
        //vw_out() << "Will float distortion parameters.\n";
      }
      
    } 
    
  } // end dealing with intrinsics
  
}

/// Compute residual map by averaging all the reprojection error at a given point
void compute_mean_residuals_at_xyz(CameraRelationNetwork<JFeature> & crn,
				   std::vector<double> const& residuals,
				   const size_t num_points,
				   std::set<int>  const& outlier_xyz,
				   const size_t num_cameras,
				   // outputs
				   std::vector<double> & mean_residuals,
				   std::vector<int>  & num_point_observations
				   ) {

  mean_residuals.resize(num_points);
  num_point_observations.resize(num_points);
  
  // Observation residuals are stored at the beginning of the residual vector in the 
  //  same order they were originally added to Ceres.
  
  size_t residual_index = 0;
  // Double loop through cameras and crn entries will give us the correct order
  for ( size_t icam = 0; icam < num_cameras; icam++ ) {
    typedef CameraNode<JFeature>::const_iterator crn_iter;
    for ( crn_iter fiter = crn[icam].begin(); fiter != crn[icam].end(); fiter++ ){

      // The index of the 3D point
      int ipt = (**fiter).m_point_id;

      if (outlier_xyz.find(ipt) != outlier_xyz.end()) continue; // skip outliers
      
      // Get the residual error for this observation
      double errorX = residuals[residual_index ];
      double errorY = residuals[residual_index+1];
      double residual_error = (fabs(errorX) + fabs(errorY)) / 2;
      residual_index += PIXEL_SIZE;

      // Update information for this point
      num_point_observations[ipt] += 1;
      mean_residuals       [ipt] += residual_error;
    }
  } // End double loop through all the observations

  // Do the averaging
  for (size_t i = 0; i < num_points; ++i) {
    if (outlier_xyz.find(i) != outlier_xyz.end()) {
      // Skip outliers. But initialize to something.
      mean_residuals[i] = std::numeric_limits<double>::quiet_NaN();
      num_point_observations[i] = std::numeric_limits<double>::quiet_NaN();
      continue;
    }
    mean_residuals[i] /= static_cast<double>(num_point_observations[i]);
  }
  
}

/// Write out a .csv file recording the residual error at each location on the ground
void write_residual_map(std::string const& output_prefix, CameraRelationNetwork<JFeature> & crn,
                        std::vector<double> const& residuals,
                        const double *points, const size_t num_points,
			std::set<int>  const& outlier_xyz,
                        const size_t num_point_params, 
                        const size_t num_cameras,
                        Options const& opt) {

  // Mean residual, and how many times that residual is seen
  std::vector<double> mean_residuals;
  std::vector<int>  num_point_observations;
  
  compute_mean_residuals_at_xyz(crn,  residuals,  num_points, outlier_xyz,  num_cameras,
				// outputs
				mean_residuals, num_point_observations);
  
  // Open the output file and write the header
  std::string output_path = output_prefix + "_point_log.csv";
  vw_out() << "Writing: " << output_path << std::endl;
  
  std::ofstream file;
  file.open(output_path.c_str()); file.precision(18);
  file << "# lon, lat, height_above_datum, mean_residual, num_observations\n";
  
  // Now write all the points to the file
  for (size_t i = 0; i < num_points; ++i) {

    if (outlier_xyz.find(i) != outlier_xyz.end()) continue; // skip outliers
    
      // The final GCC coordinate of this point
      const double * point = points + i * num_point_params;
      Vector3 xyz(point[0], point[1], point[2]);
      //xyz = xyz + Vector3(393898.51796331455, 209453.73827364066, -6350895.8432638068);
  
      Vector3 llh = opt.datum.cartesian_to_geodetic(xyz);
  
     file << llh[0] <<", "<< llh[1] <<", "<< llh[2] <<", "<< mean_residuals[i] <<", "
          << num_point_observations[i] << std::endl;
  }
  file.close();
    
} // End function write_residual_map

/// Compute the residuals
void compute_residuals(bool apply_loss_function,
		       Options const& opt, size_t num_cameras,
		       size_t num_camera_params, size_t num_point_params,
		       std::vector<size_t> const& cam_residual_counts,
		       size_t num_gcp_residuals, 
                       std::vector<vw::Vector3> const& reference_vec,
		       CameraRelationNetwork<JFeature> & crn,
		       ceres::Problem &problem,
		       std::vector<double> & residuals // output
		       ) {
  
  // TODO: Associate residuals with cameras!
  // Generate some additional diagnostic info
  double cost = 0;
  ceres::Problem::EvaluateOptions eval_options;
  eval_options.apply_loss_function = apply_loss_function;
  if (opt.stereo_session_string == "isis")
    eval_options.num_threads = 1;
  else
    eval_options.num_threads = opt.num_threads;
  problem.Evaluate(eval_options, &cost, &residuals, 0, 0);
  const size_t num_residuals = residuals.size();
  
  // Verify our residual calculations are correct
  size_t num_expected_residuals = num_gcp_residuals*num_point_params;
  for (size_t i=0; i<num_cameras; ++i)
    num_expected_residuals += cam_residual_counts[i]*PIXEL_SIZE;
  if (opt.camera_weight > 0)
    num_expected_residuals += num_cameras*num_camera_params;
  if (opt.rotation_weight > 0 || opt.translation_weight > 0)
    num_expected_residuals += num_cameras*num_camera_params;
  num_expected_residuals += reference_vec.size() * PIXEL_SIZE;
  
  if (num_expected_residuals != num_residuals)
    vw_throw( LogicErr() << "Expected " << num_expected_residuals
	      << " residuals but instead got " << num_residuals);

}

/// Write log files describing all residual errors. The order of data stored
/// in residuals must mirror perfectly the way residuals were created. 
void write_residual_logs(std::string const& residual_prefix, bool apply_loss_function,
			 Options const& opt, size_t num_cameras,
			 size_t num_camera_params, size_t num_point_params,
			 std::vector<size_t> const& cam_residual_counts,
			 size_t num_gcp_residuals, 
                         std::vector<vw::Vector3> const& reference_vec,
			 CameraRelationNetwork<JFeature> & crn,
			 const double *points, const size_t num_points,
			 std::set<int>  const& outlier_xyz,
			 ceres::Problem &problem) {
  
  std::vector<double> residuals;
  compute_residuals(apply_loss_function, opt, num_cameras, num_camera_params, num_point_params,  
		    cam_residual_counts,  num_gcp_residuals, reference_vec, crn, problem,  
		    residuals // output
		    );
    
  const size_t num_residuals = residuals.size();
  
  const std::string residual_path            = residual_prefix + "_averages.txt";
  const std::string residual_raw_pixels_path = residual_prefix + "_raw_pixels.txt";
  const std::string residual_raw_gcp_path    = residual_prefix + "_raw_gcp.txt";
  const std::string residual_raw_cams_path   = residual_prefix + "_raw_cameras.txt";
  const std::string residual_reference_xyz_path      = residual_prefix + "_reference_terrain.txt";

  // Write a report on residual errors
  std::ofstream residual_file, residual_file_raw_pixels, residual_file_raw_gcp,
    residual_file_raw_cams, residual_file_reference_xyz;
  vw_out() << "Writing: " << residual_path << std::endl;
  vw_out() << "Writing: " << residual_raw_pixels_path << std::endl;
  vw_out() << "Writing: " << residual_raw_gcp_path << std::endl;
  vw_out() << "Writing: " << residual_raw_cams_path << std::endl;
  
  residual_file.open(residual_path.c_str());
  residual_file.precision(18);
  residual_file_raw_pixels.open(residual_raw_pixels_path.c_str());
  residual_file_raw_pixels.precision(18);
  residual_file_raw_cams.open(residual_raw_cams_path.c_str());
  residual_file_raw_cams.precision(18);

  if (reference_vec.size() > 0) {
    vw_out() << "Writing: " << residual_reference_xyz_path << std::endl;
    residual_file_reference_xyz.open(residual_reference_xyz_path.c_str());
    residual_file_reference_xyz.precision(18);
  }
  
  size_t index = 0;
  // For each camera, average together all the point observation residuals
  residual_file << "Mean residual error and point count for cameras:\n";
  for (size_t c = 0; c < num_cameras; ++c) {
    size_t num_this_cam_residuals = cam_residual_counts[c];
    
    // Write header for the raw file
    residual_file_raw_pixels << opt.camera_files[c] << ", " << num_this_cam_residuals << std::endl;
    
    double mean_residual = 0; // Take average of all pixel coord errors
    for (size_t i=0; i<num_this_cam_residuals; ++i) {
      double ex = residuals[index];
      ++index;
      double ey = residuals[index];
      ++index;
      mean_residual += fabs(ex) + fabs(ey);
      
      residual_file_raw_pixels << ex << ", " << ey << std::endl; // Write ex, ey on raw file
    }
    // Write line for the summary file
    mean_residual /= static_cast<double>(num_this_cam_residuals);
    residual_file << opt.camera_files[c] << ", " << mean_residual << ", "
		  << num_this_cam_residuals << std::endl;
  }
  residual_file_raw_pixels.close();
  
  // List the GCP residuals
  if (num_gcp_residuals > 0) {
    residual_file_raw_gcp.open(residual_raw_gcp_path.c_str());
    residual_file_raw_gcp.precision(18);
    residual_file << "GCP residual errors:\n";
    for (size_t i=0; i<num_gcp_residuals; ++i) {
      double mean_residual = 0; // Take average of XYZ error for each point
      residual_file_raw_gcp << i;
      for (size_t j=0; j<num_point_params; ++j) {
        mean_residual += fabs(residuals[index]);
        residual_file_raw_gcp << ", " << residuals[index]; // Write all values in this file
        ++index;
      }
      mean_residual /= static_cast<double>(num_point_params);
      residual_file << i << ", " << mean_residual << std::endl;
      residual_file_raw_gcp << std::endl;
    }
    residual_file_raw_gcp.close();
  }
  
  // List the camera weight residuals
  int num_passes = int(opt.camera_weight > 0) +
    int(opt.rotation_weight > 0 || opt.translation_weight > 0);
  for (int pass = 0; pass < num_passes; pass++) {
    residual_file << "Camera weight position and orientation residual errors:\n";
    const size_t part_size = num_camera_params/2;
    for (size_t c=0; c<num_cameras; ++c) {
      residual_file_raw_cams << opt.camera_files[c];
      // Separately compute the mean position and rotation error
      double mean_residual_pos = 0, mean_residual_rot = 0;
      for (size_t j=0; j<part_size; ++j) {
        mean_residual_pos += fabs(residuals[index]);
        residual_file_raw_cams << ", " << residuals[index]; // Write all values in this file
        ++index;
      }
      for (size_t j=0; j<part_size; ++j) {
        mean_residual_rot += fabs(residuals[index]);
        residual_file_raw_cams << ", " << residuals[index]; // Write all values in this file
        ++index;
      }
      mean_residual_pos /= static_cast<double>(part_size);
      mean_residual_rot /= static_cast<double>(part_size);
    
      residual_file << opt.camera_files[c] << ", " << mean_residual_pos << ", "
		    << mean_residual_rot << std::endl;
      residual_file_raw_cams << std::endl;
    }
  }
  residual_file_raw_cams.close();
  residual_file.close();

  if (reference_vec.size() > 0) {
    residual_file << "reference terrain residual errors:\n";
    residual_file_reference_xyz << "# lon, lat, height_above_datum, pixel_error_norm\n";
    for (size_t i = 0; i < reference_vec.size(); ++i) {

      Vector3 llh = opt.datum.cartesian_to_geodetic(reference_vec[i]);
      double err = norm_2(Vector2(residuals[index], residuals[index + 1]));
      index += PIXEL_SIZE;
      residual_file_reference_xyz << llh[0] << ", " << llh[1] << ", " << llh[2] << ", " << err << "\n";
      residual_file << i << ", " << err << "\n";
      
    }
    residual_file_reference_xyz.close();
  }
  
  if (index != num_residuals)
    vw_throw( LogicErr() << "Have " << num_residuals << " residuals but iterated through "
	      << index);

  // Generate the location based files
  std::string map_prefix = residual_prefix + "_pointmap";
  write_residual_map(map_prefix, crn, residuals, points, num_points, outlier_xyz,
		     num_point_params, num_cameras, opt);

} // End function write_residual_logs

/// Add to the outliers based on the large residuals
int update_outliers(ControlNetwork                  & cnet,
                    CameraRelationNetwork<JFeature> & crn,
                    const double *points, const size_t num_points,
                    std::set<int> & outlier_xyz,
                    Options const& opt,
                    size_t num_cameras,
                    size_t num_camera_params, size_t num_point_params,
                    std::vector<size_t> const& cam_residual_counts,
                    size_t num_gcp_residuals,
                    std::vector<vw::Vector3> const& reference_vec, 
                    ceres::Problem &problem) {
  
  vw_out() << "Removing pixel outliers in preparation for another solver attempt.\n";

  // Compute the reprojection error. Hence we should not add the contribution
  // of the loss function.
  bool apply_loss_function = false;
  std::vector<double> residuals;
  compute_residuals(apply_loss_function,  
                    opt, num_cameras, num_camera_params, num_point_params,  cam_residual_counts,  
                    num_gcp_residuals, reference_vec, crn, problem,
                    residuals // output
                   );

  // Compute the mean residual at each xyz, and how many times that residual is seen
  std::vector<double> mean_residuals;
  std::vector<int>  num_point_observations;
  compute_mean_residuals_at_xyz(crn,  residuals,  num_points, outlier_xyz,  num_cameras,
                                // outputs
                                mean_residuals, num_point_observations);


  // The number of mean residuals is the same as the number of points,
  // of which some are outliers. Hence need to collect only the
  // non-outliers so far to be able to remove new outliers.  Need to
  // follow the same logic as when residuals were formed. And also ignore GCP.
  std::vector<double> actual_residuals;
  std::set<int> was_added;
  for ( size_t icam = 0; icam < num_cameras; icam++ ) {
    typedef CameraNode<JFeature>::const_iterator crn_iter;
    for ( crn_iter fiter = crn[icam].begin(); fiter != crn[icam].end(); fiter++ ){

      // The index of the 3D point
      int ipt = (**fiter).m_point_id;

      // skip existing outliers
      if (outlier_xyz.find(ipt) != outlier_xyz.end()) continue; 

      // Skip gcp, those are never outliers no matter what.
      if (cnet[ipt].type() == ControlPoint::GroundControlPoint) continue;

      // We already encountered this residual in the previous camera
      if (was_added.find(ipt) != was_added.end()) 
        continue;
      
      was_added.insert(ipt);
      actual_residuals.push_back(mean_residuals[ipt]);
      //vw_out() << "XYZ residual " << ipt << " = " << mean_residuals[ipt] << std::endl;
    }
  } // End double loop through all the observations

  double pct      = 1.0 - opt.remove_outliers_params[0]/100.0;
  double factor   = opt.remove_outliers_params[1];
  double max_pix1 = opt.remove_outliers_params[2];
  double max_pix2 = opt.remove_outliers_params[3];

  double b, e; 
  vw::math::find_outlier_brackets(actual_residuals, pct, factor, b, e);
  vw_out() << "Outlier statistics: b = " << b << ", e = " << e << ".\n";
  
  // If this is too aggressive, the user can tame it. It is
  // unreasonable to throw out pixel residuals as small as 1 or 2
  // pixels.  We will not use the b, because the residuals start at 0.
  // - "max_pix" sets the minimum error that can be thrown out.
  e = std::min(std::max(e, max_pix1), max_pix2);

  vw_out() << "Removing as outliers points with mean reprojection error > " << e << ".\n";
  
  // Now add to the outliers. Must repeat the same logic as above. 
  std::set<int>  new_outliers = outlier_xyz;
  for ( size_t icam = 0; icam < num_cameras; icam++ ) {
    typedef CameraNode<JFeature>::const_iterator crn_iter;
    for ( crn_iter fiter = crn[icam].begin(); fiter != crn[icam].end(); fiter++ ){

      // The index of the 3D point
      int ipt = (**fiter).m_point_id;

      // skip existing outliers
      if (outlier_xyz.find(ipt) != outlier_xyz.end()) continue; 

      // Skip gcp
      if (cnet[ipt].type() == ControlPoint::GroundControlPoint) continue;

      if (mean_residuals[ipt] > e) {
        //vw_out() << "Removing " << ipt << " with residual " << mean_residuals[ipt] << std::endl;
        new_outliers.insert(ipt);
      }

      /*
      const double * point = points + ipt * num_point_params;
      Vector3 xyz(point[0], point[1], point[2]);
      Vector3 llh = opt.datum.cartesian_to_geodetic(xyz);
      // Also filter by elevation limit
      if ( (asp::stereo_settings().elevation_limit[0] <
            asp::stereo_settings().elevation_limit[1]) && 
	           ( (llh[2] < asp::stereo_settings().elevation_limit[0]) ||
             (llh[2] > asp::stereo_settings().elevation_limit[1]) ) ) {
        vw_out() << "Removing " << ipt << " with elevation " << llh[2] << std::endl;
        new_outliers.insert(ipt); 
      }

      // And by lonlat limit
      Vector2 lon_lat = subvector(llh, 0, 2);
      if ( (!asp::stereo_settings().lon_lat_limit.empty()) &&
           (!asp::stereo_settings().lon_lat_limit.contains(lon_lat)) ) {
        vw_out() << "Removing " << ipt << " with lonlat " << lon_lat << std::endl;
        new_outliers.insert(ipt); 
      }*/
    }
  } // End double loop through all the observations

  int num_new_outliers     = new_outliers.size() - outlier_xyz.size();
  int num_remaining_points = num_points - new_outliers.size();
  vw_out() << "Removed " << num_new_outliers << " outliers, now have "
           << num_remaining_points << " points remaining.\n";

  // TODO: Write how many outliers were removed because of the lonlat check and elevation check
  
  // Overwrite the outliers
  outlier_xyz = new_outliers;

  return num_new_outliers;
}


// TODO: Move this somewhere else?
/// Create a KML file containing the positions of the given points.
/// - Points are stored as x,y,z in the points vector up to num_points.
/// - Only every skip'th point is recorded to the file.
void record_points_to_kml(const std::string &kml_path, const cartography::Datum& datum,
                          const double *points, const size_t num_points,
			  std::set<int>  const& outlier_xyz,
                          size_t skip=100, const std::string name="points",
                          const std::string icon="http://maps.google.com/mapfiles/kml/shapes/placemark_circle.png") {

  if (datum.name() == UNSPECIFIED_DATUM) {
    vw_out(WarningMessage) << "No datum specified, can't write file: " << kml_path << std::endl;
    return;
  }

  // Open the file
  KMLFile kml(kml_path, name);

  // Set up a simple point icon with no labels
  const bool hide_labels = true;
  kml.append_style( "point", "", 1.0, icon, hide_labels);
  kml.append_style( "point_highlight", "", 1.1, icon, hide_labels);
  kml.append_stylemap( "point_placemark", "point",
                       "point_highlight");

  // Loop through the points
  const size_t POINT_SIZE = 3;
  const bool extrude = true;
  for (size_t i=0; i<num_points; i+=skip) {

    if (outlier_xyz.find(i) != outlier_xyz.end()) continue; // skip outliers
    
    // Convert the point to GDC coords
    size_t index = i*POINT_SIZE;
    Vector3 xyz(points[index], points[index+1], points[index+2]);

    Vector3 lon_lat_alt = datum.cartesian_to_geodetic(xyz);
      
    // Add this to the output file
    kml.append_placemark( lon_lat_alt.x(), lon_lat_alt.y(),
                          "", "", "point_placemark",
                          lon_lat_alt[2], extrude );
        
  }
  kml.close_kml();
}


void create_interp_dem(std::string & dem_file,
                       vw::cartography::GeoReference & dem_georef,
                       ImageViewRef< PixelMask<double> > & interp_dem){
  
  vw_out() << "Loading DEM: " << dem_file << std::endl;
  double nodata_val = -std::numeric_limits<float>::max(); // note we use a float nodata
  if (vw::read_nodata_val(dem_file, nodata_val)){
    vw_out() << "Found DEM nodata value: " << nodata_val << std::endl;
  }
  
  ImageView< PixelMask<double> > dem = create_mask(DiskImageView<double>(dem_file), nodata_val);
  
  interp_dem = interpolate(dem, BilinearInterpolation(), ConstantEdgeExtension());
  bool is_good = vw::cartography::read_georeference(dem_georef, dem_file);
  if (!is_good) {
    vw_throw(ArgumentErr() << "Error: Cannot read georeference from DEM: "
             << dem_file << ".\n");
  }
}

template <class ModelT>
int do_ba_ceres_one_pass(ModelT                          & ba_model,
                          Options                         & opt,
                          ControlNetwork                  & cnet,
                          CameraRelationNetwork<JFeature> & crn,
                          bool                              first_pass,
                          bool                              last_pass,
                          int                               num_camera_params,
                          int                               num_point_params,
                          int                               num_intrinsic_params,
                          int                               num_cameras,
                          int                               num_points,
                          std::vector<double>       const & orig_cameras_vec,
                          double                          * cameras,
                          double                          * intrinsics,
                          double                          * points,
                          std::set<int>                   & outlier_xyz){

  ceres::Problem problem;

  // Add the cost function component for difference of pixel observations
  // - Reduce error by making pixel projection consistent with observations.
  typedef CameraNode<JFeature>::iterator crn_iter;
  if (num_cameras != static_cast<int>(crn.size()))
    vw_throw( LogicErr() << "Expected " << num_cameras << " cameras but crn has " << crn.size());

  // How many times an xyz point shows up in the problem
  std::map<double*, int> count_map;
  if (opt.overlap_exponent > 0) {
    for ( int icam = 0; icam < num_cameras; icam++ ) {
      for ( crn_iter fiter = crn[icam].begin(); fiter != crn[icam].end(); fiter++ ){
        int ipt = (**fiter).m_point_id;
        if (outlier_xyz.find(ipt) != outlier_xyz.end())
          continue; // skip outliers
        double * point  = points  + ipt  * num_point_params;
        count_map[point]++;
      }
    }
  }

  // We will optimize multipliers of the intrinsics. This way
  // each intrinsic changes by a scale specific to it.
  // TODO: If an intrinsic starts as 0, it will then stay as 0 which is not good.
  std::vector<double> scaled_intrinsics(num_intrinsic_params);
  for (int intrIter = 0; intrIter < num_intrinsic_params; intrIter++)
    scaled_intrinsics[intrIter] = 1.0;
  double * scaled_intrinsics_ptr = NULL;
  if (num_intrinsic_params > 0)
    scaled_intrinsics_ptr = &scaled_intrinsics[0];

  
  vw::cartography::GeoReference dem_georef;
  ImageViewRef< PixelMask<double> >  interp_dem;
  if (opt.heights_from_dem != "") {
    if (!opt.create_pinhole) 
      vw_throw( ArgumentErr() << "When using a high quality DEM, must use the --create-pinhole-cameras option.\n");
    create_interp_dem(opt.heights_from_dem, dem_georef, interp_dem);
  }
  
  // Add the various cost functions the solver will optimize over.
  std::vector<size_t> cam_residual_counts(num_cameras);
  for ( int icam = 0; icam < num_cameras; icam++ ) {
    cam_residual_counts[icam] = 0;
    for ( crn_iter fiter = crn[icam].begin(); fiter != crn[icam].end(); fiter++ ){

      // The index of the 3D point
      int ipt = (**fiter).m_point_id;
      if (outlier_xyz.find(ipt) != outlier_xyz.end())
        continue; // skip outliers

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

      double p = opt.overlap_exponent;
      if (p > 0 && count_map.find(point) != count_map.end() && count_map[point] > 1) {
        // Give more weight to points that are seen in more images.
        // This should not be overused. 
        double delta = pow(count_map[point] - 1.0, p);
        pixel_sigma /= delta;
      }
      
      ceres::LossFunction* loss_function = get_loss_function(opt);

      // Call function to select the appropriate Ceres residual block to add.
      add_residual_block(ba_model, observation, pixel_sigma, icam, ipt,
                         camera, point, scaled_intrinsics_ptr, opt.intrinsics_to_float,
                         loss_function, problem);

      // Fix this camera if requested
      if (opt.fixed_cameras_indices.find(icam) != opt.fixed_cameras_indices.end()) 
	problem.SetParameterBlockConstant(camera);
            
      if (opt.heights_from_dem != "") {
        // For non-GCP points, copy the heights for xyz points from the DEM.
        // Fix the obtained xyz points as they are considered reliable
        // and we should have the cameras and intrinsics params to conform
        // to these.
        if (cnet[ipt].type() != ControlPoint::GroundControlPoint){
          Vector3 xyz(point[0], point[1], point[2]);
          vw::Vector3 llh = dem_georef.datum().cartesian_to_geodetic(xyz);
          vw::Vector2 ll = subvector(llh, 0, 2);
          Vector2 pix = dem_georef.lonlat_to_pixel(ll);
          if (pix[0] >= 0 &&
              pix[1] >= 0 &&
              pix[0] <= interp_dem.cols()-1 &&
              pix[1] <= interp_dem.rows()-1) {
            PixelMask<double> ht = interp_dem(pix[0], pix[1]);
            if (is_valid(ht)) {
              llh[2] = ht.child();
              xyz = dem_georef.datum().geodetic_to_cartesian(llh);
              for (size_t it = 0; it < xyz.size(); it++) 
                point[it] = xyz[it];
            }
          }
          problem.SetParameterBlockConstant(point);
        }
      }
      
      cam_residual_counts[icam] += 1; // Track the number of residual blocks for each camera

    } // end iterating over points
  } // end iterating over cameras

  // Add ground control points
  // - Error goes up as GCP's move from their input positions.
  int num_gcp = 0;
  size_t num_gcp_residuals = 0;
  for (int ipt = 0; ipt < num_points; ipt++){
    if (cnet[ipt].type() != ControlPoint::GroundControlPoint) continue;

    if (outlier_xyz.find(ipt) != outlier_xyz.end())
      continue; // skip outliers
    
    num_gcp++;
    
    Vector3 observation = cnet[ipt].position();
    Vector3 xyz_sigma   = cnet[ipt].sigma();

    ceres::CostFunction* cost_function;
    if (!opt.use_llh_error) 
      cost_function = XYZError::Create(observation, xyz_sigma);
    else
      cost_function = LLHError::Create(observation, xyz_sigma, opt.datum);

    // Don't use the same loss function as for pixels since that one discounts
    //  outliers and the cameras should never be discounted.
    ceres::LossFunction* loss_function = new ceres::TrivialLoss();

    double * point  = points  + ipt * num_point_params;
    problem.AddResidualBlock(cost_function, loss_function, point);
    ++num_gcp_residuals;

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

      // Don't use the same loss function as for pixels since that one discounts
      //  outliers and the cameras should never be discounted.
      ceres::LossFunction* loss_function = new ceres::TrivialLoss();

      double * camera  = cameras  + icam * num_camera_params;
      problem.AddResidualBlock(cost_function, loss_function, camera);
    }
  }

  // Finer level control of only rotation and translation.
  // This will need to be merged with the above but note that the loss is NULL here. 
  // - Error goes up as cameras move and rotate from their input positions.
  if (opt.rotation_weight > 0 || opt.translation_weight > 0){

    for (int icam = 0; icam < num_cameras; icam++){

      typename ModelT::camera_vector_t orig_cam;
      for (int q = 0; q < num_camera_params; q++)
        orig_cam[q] = orig_cameras_vec[icam * num_camera_params + q];

      ceres::CostFunction* cost_function
        = RotTransError<ModelT>::Create(orig_cam, opt.rotation_weight, opt.translation_weight);
      ceres::LossFunction* loss_function = new ceres::TrivialLoss();

      double * camera  = cameras  + icam * num_camera_params;
      problem.AddResidualBlock(cost_function, loss_function, camera);
    }
  }

  // Add a cost function meant to tie up to known disparity
  // form left to right image and known ground truth reference terrain.
  // This was only tested for local pinhole cameras.
  // Disparity must be created with stereo -e 3 with the
  // options --enable-fill-holes --unalign-disparity.
  // This will work with an even number of images/cameras.
  // For images 2*i and 2*i+1 there must be one disparity.
  // The doc has more info in the bundle_adjust chapter.
  std::vector< ImageView<DispPixelT> > disp_vec;
  std::vector< ImageViewRef<DispPixelT> > interp_disp; 
  std::vector<vw::Vector3> reference_vec;
  if (opt.create_pinhole && opt.reference_terrain != "") {

    std::string file_type = asp::get_cloud_type(opt.reference_terrain);

    if (file_type == "CSV" && opt.csv_format_str == "") 
      vw_throw( ArgumentErr() << "When using a csv reference terrain, "
                << "must specify the csv-format.\n");
    
    if (opt.datum_str == "")
      vw_throw( ArgumentErr() << "When using a reference terrain, must specify the datum.\n");
    if (opt.disparity_list == "") 
      vw_throw( ArgumentErr() << "When using a reference terrain, must specify a list "
                << "of disparities.\n");
    if (num_cameras%2 != 0) 
      vw_throw( ArgumentErr() << "A reference terrain can only be used with an even number "
                << "of cameras.\n");
    if (opt.max_disp_error <= 0) 
      vw_throw( ArgumentErr() << "Must specify --max-disp-error in pixels as a positive value.\n");

    g_max_disp_error = opt.max_disp_error;
    
    // Set up a GeoReference object using the datum
    vw::cartography::GeoReference geo;
    geo.set_datum(opt.datum); // We checked for a datum earlier

    asp::CsvConv csv_conv;
    csv_conv.parse_csv_format(opt.csv_format_str, opt.csv_proj4_str);

    // Use user's csv_proj4 string, if provided, to add info to the georef.
    csv_conv.parse_georef(geo);

    vw::BBox2 lonlat_box; // not used
    bool calc_shift = false;
    Vector3 shift; // must be set to 0
    bool is_lola_rdr_format;
    double mean_longitude;
    bool verbose = true;
    asp::DoubleMatrix data;
    
    // Read the reference terrain
    vw_out() << "Loading at most " << opt.max_num_reference_points << " points from "
             << opt.reference_terrain << std::endl;
    if (file_type == "DEM") 
      asp::load_dem(opt.reference_terrain,  
               opt.max_num_reference_points, lonlat_box,  
               calc_shift, shift, verbose, data);
      
    else if (file_type == "CSV")
      asp::load_csv(opt.reference_terrain,  opt.max_num_reference_points,
		    lonlat_box, calc_shift, shift, geo,  
		    csv_conv, is_lola_rdr_format, mean_longitude, verbose,  
		    data);
    else
      vw_throw( ArgumentErr() << "Unsupported file: " << opt.reference_terrain << " of type" <<
		file_type << ".\n");
    
    // TODO: Disparities can be large, but if small it is better to
    // read them in memory.
    std::istringstream is(opt.disparity_list);
    std::string disp_file;
    while (is >> disp_file) {
      vw_out() << "Reading: " << disp_file << std::endl;
      disp_vec.push_back(copy(DiskImageView<DispPixelT>(disp_file)));
      interp_disp.push_back(interpolate(disp_vec.back(),
                                        BilinearInterpolation(), ConstantEdgeExtension()));
    }
    if (2*int(disp_vec.size()) != num_cameras)
      vw_throw( ArgumentErr() << "Expecting one disparity for each pair of images.\n");
    
    std::vector<vw::BBox2i> image_boxes;
    for ( int icam = 0; icam < num_cameras; icam++){
      DiskImageView<float> img(opt.image_files[icam]);
      BBox2i bbox = vw::bounding_box(img);
      image_boxes.push_back(bbox);
    }

    vw_out() << "Setting up the error to the reference terrain.\n";
    TerminalProgressCallback tpc("", "\t--> ");
    tpc.report_progress(0);
    int num_cols = data.cols();
    //double inc_amount = 1.0/double(pos_records.size());
    double inc_amount = 1.0/double(num_cols);
				   
    reference_vec.clear();
    for (int data_col = 0; data_col < num_cols; data_col++) {

      //vw::Vector3 reference_xyz = csv_conv.csv_to_cartesian(*iter, geo);
      vw::Vector3 reference_xyz;
      for (int row = 0; row < asp::DIM; row++)
        reference_xyz[row] = data(row, data_col);

      // Filter by lonlat box if provided, this is very much recommended
      // to quickly discard most points in the huge reference terrain.
      // Let's hope there is no 360 degree offset when computing
      // the longitude. 
      if ( asp::stereo_settings().lon_lat_limit != BBox2(0,0,0,0) ) {
	vw::Vector3 llh = geo.datum().cartesian_to_geodetic(reference_xyz);
	vw::Vector2 ll = subvector(llh, 0, 2);
	if (!asp::stereo_settings().lon_lat_limit.contains(ll)) {
	  continue;
	}
      }

      bool good_point = true;
      Vector2 left_pred;

      // Iterate over the cameras, add a residual for each point and each camera pair.
      // Camera 0 is paired with camera 1, camera 2 with camera 3, etc. 
      for ( int icam = 0; icam < num_cameras; icam++ ) {

        if (icam%2 == 0) {
          // Reset this for every pair of images
          good_point = true; 
        }

        if (!good_point) continue; 
          
        // Get pointers to the camera and point coordinates
        double * camera = cameras + icam * num_camera_params;
        double * point  = &reference_xyz[0];

        // Project the current point into the current camera
        typename ModelT::camera_intr_vector_t cam_intr_vec;
        typename ModelT::point_vector_t       point_vec;
        ba_model.concat_extrinsics_intrinsics(camera, intrinsics, cam_intr_vec);
        for (size_t p = 0; p < point_vec.size(); p++) point_vec[p] = point[p];
        Vector2 prediction, right_pred;
        try {
          prediction = ba_model.cam_pixel(0, icam, cam_intr_vec, point_vec);
        }catch(std::exception const& e){
          good_point = false;
          continue;
        }

	// Check if the current point projects in the camera
        if (!image_boxes[icam].contains(prediction)) {
          good_point = false;
          continue;
	}

        if (icam%2 == 0) {
          left_pred = prediction;
	  // Record where we projected in the left camera, and then switch to the right camera
          continue;
        }

	if (icam%2 != 1)
	  vw_throw(ArgumentErr() << "Expecting an odd camera here.\n");

        right_pred = prediction;
        
        // Check for out of range, etc
	if (!good_point) continue;
        if (left_pred != left_pred) continue; // nan check
        if (left_pred[0] < 0 || left_pred[0] > interp_disp[icam/2].cols() - 1 ) continue;
        if (left_pred[1] < 0 || left_pred[1] > interp_disp[icam/2].rows() - 1 ) continue;

        DispPixelT dispPix = interp_disp[icam/2](left_pred[0], left_pred[1]);
        if (!is_valid(dispPix)) continue;
          
        Vector2 right_pix = left_pred + dispPix.child();
        if (!image_boxes[icam].contains(right_pix)) 
          continue;

        if (right_pred != right_pred || norm_2(right_pix - right_pred) > opt.max_disp_error) {
          // Ignore pixels which are too far from where they should be before optimization
          continue;
        }
        
	reference_vec.push_back(reference_xyz);

        ceres::LossFunction* loss_function = get_loss_function(opt);
        
        // Call function to select the appropriate Ceres residual block to add.
        add_residual_block(reference_xyz, interp_disp[icam/2], ba_model,
                           icam-1, icam, // left icam and right icam
                           cameras + (icam-1) * num_camera_params, // left cam
                           cameras + icam * num_camera_params, // right cam
                           scaled_intrinsics_ptr,
                           opt.intrinsics_to_float,
                           loss_function, problem);
      }
      tpc.report_incremental_progress( inc_amount );
    }
    
    tpc.report_finished();
    vw_out() << "Found " << reference_vec.size() << " reference points in range.\n";
  }
  
  const size_t MIN_KML_POINTS = 20;  
  size_t kmlPointSkip = 30;
  // Figure out a good KML point skip aount
  if (num_points / kmlPointSkip < MIN_KML_POINTS)
    kmlPointSkip = num_points / MIN_KML_POINTS;
  if (kmlPointSkip < 1)
    kmlPointSkip = 1;

  std::string residual_prefix = opt.out_prefix + "-initial_residuals_loss_function";
  std::string point_kml_path  = opt.out_prefix + "-initial_points.kml";
    
  if (first_pass) { 
    vw_out() << "Writing initial condition files..." << std::endl;

    write_residual_logs(residual_prefix, true,  opt, num_cameras, num_camera_params,
                        num_point_params, cam_residual_counts, num_gcp_residuals,
                        reference_vec, crn, points, num_points, outlier_xyz, problem);
    residual_prefix = opt.out_prefix + "-initial_residuals_no_loss_function";
    write_residual_logs(residual_prefix, false, opt, num_cameras, num_camera_params,
                        num_point_params, cam_residual_counts, num_gcp_residuals,
                        reference_vec, crn, points, num_points, outlier_xyz, problem);


      
    record_points_to_kml(point_kml_path, opt.datum, points, num_points, outlier_xyz,
		         kmlPointSkip, "initial_points",
                        "http://maps.google.com/mapfiles/kml/shapes/placemark_circle.png");
  }

  // Solve the problem
  ceres::Solver::Options options;
  options.gradient_tolerance = 1e-16;
  options.function_tolerance = 1e-16;
  options.parameter_tolerance = opt.parameter_tolerance; // default is 1e-8

  options.max_num_iterations = opt.max_iterations;
  options.max_num_consecutive_invalid_steps = std::max(5, opt.max_iterations/5); // try hard
  options.minimizer_progress_to_stdout = true;//(opt.report_level >= vw::ba::ReportFile);

  if (opt.stereo_session_string == "isis")
    options.num_threads = 1;
  else
    options.num_threads = opt.num_threads;

  // Set solver options according to the recommendations in the Ceres solving FAQs
  options.linear_solver_type = ceres::SPARSE_SCHUR;
  if (num_cameras < 100)
    options.linear_solver_type = ceres::DENSE_SCHUR;
  if (num_cameras > 3500) {
    options.use_explicit_schur_complement = true; // This is supposed to help with speed in a certain size range
    options.linear_solver_type  = ceres::ITERATIVE_SCHUR;
    options.preconditioner_type = ceres::SCHUR_JACOBI;
  }
  if (num_cameras > 7000)
    options.use_explicit_schur_complement = false; // Only matters with ITERATIVE_SCHUR

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

  // Multiply the original intrinsics by the scaled optimized values
  for (int intrIter = 0; intrIter < num_intrinsic_params; intrIter++)
    intrinsics[intrIter] = intrinsics[intrIter] * scaled_intrinsics_ptr[intrIter];

  if (opt.create_pinhole && opt.solve_intrinsics) {
    vw_out() << "Final scaled intrinsics:\n";
    for (int intrIter = 0; intrIter < num_intrinsic_params; intrIter++) 
      vw_out() << scaled_intrinsics[intrIter] << " ";
    vw_out() << std::endl;
  }
  
  vw_out() << "Writing final condition log files..." << std::endl;
  residual_prefix = opt.out_prefix + "-final_residuals_loss_function";
  write_residual_logs(residual_prefix, true,  opt, num_cameras, num_camera_params,
                      num_point_params, cam_residual_counts,
                      num_gcp_residuals, reference_vec, crn,
                      points, num_points, outlier_xyz, problem);
  residual_prefix = opt.out_prefix + "-final_residuals_no_loss_function";
  write_residual_logs(residual_prefix, false, opt, num_cameras, num_camera_params,
                      num_point_params, cam_residual_counts,
                      num_gcp_residuals, reference_vec, crn,
                      points, num_points, outlier_xyz, problem);

  point_kml_path = opt.out_prefix + "-final_points.kml";
  record_points_to_kml(point_kml_path, opt.datum, points, num_points, outlier_xyz,
		       kmlPointSkip, "final_points",
                       "http://maps.google.com/mapfiles/kml/shapes/placemark_circle_highlight.png");

  // Print stats for optimized gcp
  if (num_gcp > 0) {
    vw_out() << "input_gcp optimized_gcp diff\n";
    for (int ipt = 0; ipt < num_points; ipt++){
      if (cnet[ipt].type() != ControlPoint::GroundControlPoint) continue;
      if (outlier_xyz.find(ipt) != outlier_xyz.end()) continue; // skip outliers

      Vector3 input_gcp = cnet[ipt].position();
      
      double * point  = points  + ipt * num_point_params;
      Vector3 opt_gcp;
      for (int p = 0; p < 3; p++) opt_gcp[p] = point[p];
      
      vw_out() << "xyz: " << input_gcp << ' ' << opt_gcp << ' ' << input_gcp - opt_gcp
               << std::endl;

      input_gcp = opt.datum.cartesian_to_geodetic(input_gcp);
      opt_gcp   = opt.datum.cartesian_to_geodetic(opt_gcp);
      vw_out() << "llh: " << input_gcp << ' ' << opt_gcp << ' ' << input_gcp - opt_gcp
               << std::endl;
    }
  }

  int num_new_outliers = 0;
  if (!last_pass) 
    num_new_outliers =
      update_outliers(cnet, crn, points, num_points,
                      outlier_xyz,   // in-out
                      opt, num_cameras, num_camera_params, num_point_params,
                      cam_residual_counts,  
                      num_gcp_residuals, reference_vec, problem);

  // Create a match file with clean points.
  // TODO: Make this a function.
  // TODO: Create this for every pair of images.
  if (opt.num_ba_passes > 1 && num_cameras == 2 && (num_new_outliers > 0)) {

    vw_out() << "Building clean IP list...\n";
    std::vector<vw::ip::InterestPoint> left_ip, right_ip;
    std::vector<int> left_pt, right_pt; // these are used for bookkeeping

    for ( int icam = 0; icam < num_cameras; icam++ ) {
      for ( crn_iter fiter = crn[icam].begin(); fiter != crn[icam].end(); fiter++ ){
        
        // The index of the 3D point
        int ipt = (**fiter).m_point_id;
        if (outlier_xyz.find(ipt) != outlier_xyz.end())
          continue; // skip outliers
        
        // Skip gcp
        if (cnet[ipt].type() == ControlPoint::GroundControlPoint)
          continue;
        
        VW_ASSERT(int(icam) < num_cameras,
                  ArgumentErr() << "Out of bounds in the number of cameras");
        VW_ASSERT(int(ipt)  < num_points,
                  ArgumentErr() << "Out of bounds in the number of points");
        
        Vector2 observation = (**fiter).m_location; // pixel value
        ip::InterestPoint P;
        P.x = observation.x();
        P.y = observation.y();
        if (icam == 0) {
          left_ip.push_back(P);
          left_pt.push_back(ipt);
        }else if (icam == 1){
          right_ip.push_back(P);
          right_pt.push_back(ipt);
        }
      }
    }

    // Book-keeping, a given left ip and right ip must correspond to same xyz
    if (left_pt.size() != right_pt.size()) 
      vw_throw(ArgumentErr() << "Book-keeping error 1 in bundle adjustment.\n");

    for (size_t pcount = 0; pcount < left_pt.size(); pcount++) {
      if (left_pt[pcount] != right_pt[pcount]) 
        vw_throw(ArgumentErr() << "Book-keeping error 2 in bundle adjustment.\n");
    }

    // Also filter by disparity.
    asp::filter_ip_by_disparity(opt.remove_outliers_by_disp_params[0],
                                opt.remove_outliers_by_disp_params[1],
                                left_ip, right_ip);
    
    // TODO: Move this into the IP finding code!
    // Compute the coverage fraction
    Vector2i right_image_size = file_image_size(opt.image_files[1]);
    int right_ip_width = right_image_size[0]*
                          static_cast<double>(100-opt.ip_edge_buffer_percent)/100.0;
    Vector2i ip_size(right_ip_width, right_image_size[1]);
    double ip_coverage = calc_ip_coverage_fraction(right_ip, ip_size);
    // Careful with the line below, it gets used in process_icebridge_batch.py.
    vw_out() << "IP coverage fraction after cleaning = " << ip_coverage << "\n";

    std::string match_file = opt.out_prefix  + "-clean.match";
    vw_out() << "Writing: " << match_file << std::endl;
    ip::write_binary_match_file(match_file, left_ip, right_ip);
    
  } // End outlier update case
      
  return num_new_outliers;
}

/// Use Ceres to do bundle adjustment. The camera and point variables
/// are stored in arrays.  The projection of point into camera is
/// accomplished by interfacing with the bundle adjustment model. In
/// the future this class can be bypassed.
template <class ModelT>
void do_ba_ceres(ModelT & ba_model, Options & opt ){

  ControlNetwork & cnet = *(ba_model.control_network().get());

  const int num_camera_params    = ModelT::camera_params_n;
  const int num_point_params     = ModelT::point_params_n;
  const int num_intrinsic_params = ba_model.num_intrinsic_params();
  const int num_cameras          = ba_model.num_cameras();
  const int num_points           = ba_model.num_points();

  // The camera adjustment and point variables concatenated into
  // vectors. The camera adjustments start as 0. The points come from the network.
  std::vector<double> cameras_vec(num_cameras*num_camera_params, 0.0);
  std::vector<double> intrinsics_vec(num_intrinsic_params, 0.0);

  // Fill in the camera vectors with their starting values.
  // TODO: This does not update the cnet anymore!
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

  // Points
  std::vector<double> points_vec(num_points*num_point_params, 0.0);
  for (int ipt = 0; ipt < num_points; ipt++){
    for (int q = 0; q < num_point_params; q++){
      points_vec[ipt*num_point_params + q] = cnet[ipt].position()[q];
    }
  }

  // The camera positions and orientations before we float them
  std::vector<double> orig_cameras_vec = cameras_vec;

  std::vector<double> orig_points_vec, orig_intrinsics_vec;
  if (opt.num_ba_passes > 1) {
    orig_points_vec = points_vec;
    orig_intrinsics_vec = intrinsics_vec;
  }
  
  CameraRelationNetwork<JFeature> crn;
  crn.read_controlnetwork(cnet);

  // We will keep here the outliers
  std::set<int> outlier_xyz;

  if (opt.num_ba_passes <= 0)
    vw_throw(ArgumentErr() << "Error: Expecting at least one bundle adjust pass.\n");
  
  for (int pass = 0; pass < opt.num_ba_passes; pass++) {

    if (opt.num_ba_passes > 1) {
      vw_out() << "Bundle adjust pass: " << pass << std::endl;
      // Go back to the original inputs to optimize, sans the outliers. Note that we
      // copy values, to not disturb the pointer of each vector.
      for (size_t i = 0; i < cameras_vec.size(); i++)    cameras_vec[i]    = orig_cameras_vec[i];
      for (size_t i = 0; i < points_vec.size(); i++)     points_vec[i]     = orig_points_vec[i];
      for (size_t i = 0; i < intrinsics_vec.size(); i++) intrinsics_vec[i] = orig_intrinsics_vec[i];
    }
    
    // Camera extrinsics and intrinsics
    double* cameras    = &cameras_vec[0];
    double* points     = &points_vec[0];
    double* intrinsics = NULL;
    if (num_intrinsic_params > 0) intrinsics = &intrinsics_vec[0];
    
    bool last_pass = (pass == opt.num_ba_passes - 1);
    int num_new_outliers = do_ba_ceres_one_pass(ba_model, opt,  cnet,  crn, (pass==0), last_pass,
                                                num_camera_params,  num_point_params,  
                                                num_intrinsic_params, num_cameras, num_points,  
                                                orig_cameras_vec,  cameras,  intrinsics,  points,  
                                                outlier_xyz);

    if (!last_pass && num_new_outliers == 0) {
      vw_out() << "No new outliers removed. No more passes are needed.\n";
      break;
    }

    int num_points_remaining = num_points - outlier_xyz.size();
    if (opt.num_ba_passes > 1 && num_points_remaining < opt.min_matches) {
      // Do not throw if there were is just one pass, as no outlier filtering happened.
      // This is needed to not break functionality when only gcp are passed as inputs.
      vw_throw(ArgumentErr() << "Error: Too few points remain after filtering!.\n");
    }
  }

  // Copy the latest version of the optimized intrinsic variables back
  // into the the separate parameter vectors in ba_model, right after
  // the already updated extrinsic parameters.
  typename ModelT::camera_intr_vector_t concat;
  double* intrinsics = NULL;
  if (num_intrinsic_params > 0) intrinsics = &intrinsics_vec[0];
  for (int icam = 0; icam < num_cameras; icam++){
    ba_model.concat_extrinsics_intrinsics(&cameras_vec[icam*num_camera_params],
                                          intrinsics, concat);
    ba_model.set_cam_params(icam, concat);
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
template<class CostFunType>
void do_ba_costfun(CostFunType const& cost_fun, Options& opt){

  BundleAdjustmentModel ba_model(opt.camera_models, opt.cnet);

  if ( opt.ba_type == "ceres" ) {
    do_ba_ceres<BundleAdjustmentModel>(ba_model, opt);
  } else if ( opt.ba_type == "robustsparse" ) {
    do_ba_nonceres<AdjustRobustSparse< BundleAdjustmentModel,CostFunType> >(ba_model, cost_fun, opt);
  } else if ( opt.ba_type == "robustref" ) {
    do_ba_nonceres<AdjustRobustRef< BundleAdjustmentModel,CostFunType> >(ba_model, cost_fun, opt);
  } else if ( opt.ba_type == "sparse" ) {
    do_ba_nonceres<AdjustSparse< BundleAdjustmentModel, CostFunType > >(ba_model, cost_fun, opt);
  }else if ( opt.ba_type == "ref" ) {
    do_ba_nonceres<AdjustRef< BundleAdjustmentModel, CostFunType > >(ba_model, cost_fun, opt);
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

// Do BA with BundleAdjustmentModel model. Switch based on cost function.
void do_ba_with_model(Options& opt){

  if ( opt.cost_function == "cauchy" ) {
    do_ba_costfun<CauchyError>(CauchyError(opt.robust_threshold), opt);
  }else if ( opt.cost_function == "pseudohuber" ) {
    do_ba_costfun<PseudoHuberError>(PseudoHuberError(opt.robust_threshold), opt );
  } else if ( opt.cost_function == "huber" ) {
    do_ba_costfun<HuberError>(HuberError(opt.robust_threshold), opt );
  } else if ( opt.cost_function == "l1" ) {
    do_ba_costfun<L1Error>( L1Error(), opt );
  } else if ( opt.cost_function == "l2" ) {
    do_ba_costfun<L2Error>( L2Error(), opt );
  }else{
    vw_throw( ArgumentErr() << "Unknown cost function: " << opt.cost_function
                            << ". Options are: Cauchy, PseudoHuber, Huber, L1, L2.\n" );
  }
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
      vw_out() << "WARNING: GCPs are over 100 KM from the other points. Are your lat/lon GCP coordinates swapped?\n";
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
      vw_out() << "WARNING: Camera file " << file_name << " not found in camera position file.\n";
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

  vw_out() << "Initializing camera positions from input file..." << std::endl;

  // Count the number of matches and check for problems
  const int num_cameras = opt.image_files.size();
  if (int(estimated_camera_gcc.size()) != num_cameras)
    vw_throw( ArgumentErr() << "No camera matches provided to init function!\n" );
  
  vw_out() << "Num cameras: " << num_cameras << std::endl;
    
  int num_matches_found = 0;
  for (int i=0; i<num_cameras; ++i)
    if (estimated_camera_gcc[i] != Vector3(0,0,0))
      ++num_matches_found;

  vw_out() << "Number of matches found: " << num_matches_found << std::endl;
  
  const int MIN_NUM_MATCHES = 3;
  if (num_matches_found < MIN_NUM_MATCHES)
    vw_throw( ArgumentErr() << "At least " << MIN_NUM_MATCHES 
	      << " camera position matches are required to initialize sensor models!\n" );
  
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

    vw_out() << "Initializing camera positions from ground control points..." << std::endl;

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
    int num_gcp      = 0;
    int num_good_gcp = 0;
    for (int ipt = 0; ipt < num_cnet_points; ipt++){
      if (cnet[ipt].type() != ControlPoint::GroundControlPoint)
        continue;
      ++num_gcp;
        
      // Use triangulation to estimate the position of this control point using
      //   the current set of camera models.
      ControlPoint cp_new = cnet[ipt];
      // Making minimum_angle below big may throw away valid points at this stage // really???
      double minimum_angle = 0;
      vw::ba::triangulate_control_point(cp_new, opt.camera_models, minimum_angle);
      if (cp_new.position() != Vector3() && cnet[ipt].position() != Vector3())
        ++num_good_gcp; // Only count points that triangulate
      else {
        vw_out() << "Discarding GCP: " << cnet[ipt] << "\n" << cp_new << std::endl;
      }
    }
    
    // Update the number of GCP that we are using
    const int MIN_NUM_GOOD_GCP = 3;
    if (num_good_gcp < MIN_NUM_GOOD_GCP) {
      vw_out() << "Num GCP       = " << num_gcp         << std::endl;
      vw_out() << "Num valid GCP = " << num_good_gcp    << std::endl;
      vw_throw( ArgumentErr() << "Not enough valid GCPs for affine initalization!\n" );
    }
    
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

  vw::cartography::GeoReference dem_georef;
  ImageViewRef< PixelMask<double> > interp_dem;
  create_interp_dem(dem_file, dem_georef, interp_dem);
  
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
        if (dem_pix1[0] < 0 || dem_pix1[0] >= interp_dem.cols() - 1) continue;
        if (dem_pix1[1] < 0 || dem_pix1[1] >= interp_dem.rows() - 1) continue;
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
        if (dem_pix2[0] < 0 || dem_pix2[0] >= interp_dem.cols() - 1) continue;
        if (dem_pix2[1] < 0 || dem_pix2[1] >= interp_dem.rows() - 1) continue;
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

  vw::cartography::GeoReference dem_georef;
  ImageViewRef< PixelMask<double> > interp_dem;
  create_interp_dem(dem_file, dem_georef, interp_dem);
  
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
    Vector2 lonlat = dem_georef.pixel_to_lonlat(dem_pixel);
    
    if (dem_pixel[0] < 0 || dem_pixel[0] >= interp_dem.cols() - 1 ||
        dem_pixel[1] < 0 || dem_pixel[1] >= interp_dem.rows() - 1) {
      vw_out() << "Skipping pixel outside of DEM: " << dem_pixel << std::endl;
      continue;
    }
    
    PixelMask<float> mask_height = interp_dem(dem_pixel[0], dem_pixel[1])[0];
    if (!is_valid(mask_height)) continue;
    
    Vector3 llh(lonlat[0], lonlat[1], mask_height.child());
    //Vector3 dem_xyz = dem_georef.datum().geodetic_to_cartesian(llh);

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
      
      Vector2 dem_pix = dem_georef.lonlat_to_pixel(ll);
      if (dem_pix[0] < 0 || dem_pix[0] >= interp_dem.cols() - 1) continue;
      if (dem_pix[1] < 0 || dem_pix[1] >= interp_dem.rows() - 1) continue;
      PixelMask<double> dem_val = interp_dem(dem_pix[0], dem_pix[1]);
      if (!is_valid(dem_val)) continue;
      Vector3 llh(ll[0], ll[1], dem_val.child());
      Vector3 xyz = dem_georef.datum().geodetic_to_cartesian(llh);
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

// TODO: Change something so we don't have to repeat all the stereo IP options here!

void handle_arguments( int argc, char *argv[], Options& opt ) {
  double nan = std::numeric_limits<double>::quiet_NaN();
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
    ("create-pinhole-cameras",    po::bool_switch(&opt.create_pinhole)->default_value(false),
                         " If the input cameras are of the pinhole type, apply the adjustments directly to the cameras, rather than saving them separately as .adjust files.")
    ("approximate-pinhole-intrinsics", po::bool_switch(&opt.approximate_pinhole_intrinsics)->default_value(false),
                         "If it reduces computation time, approximate the lens distortion model.")
                         
    ("solve-intrinsics",  po::bool_switch(&opt.solve_intrinsics)->default_value(false)->implicit_value(true),
                         "Optimize intrinsic camera parameters.  Only used for pinhole cameras.")
    ("intrinsics-to-float", po::value(&opt.intrinsics_to_float_str)->default_value(""),
     "If solving for intrinsics and desired to float only a few of them, specify here, in quotes, one or more of: focal_length, optical_center, distortion_params.")
    ("camera-positions", po::value(&opt.camera_position_file)->default_value(""),
          "Specify a csv file path containing the estimated positions of the input cameras.  Only used with the create-pinhole-cameras option.")
    ("input-adjustments-prefix",  po::value(&opt.input_prefix), "Prefix to read initial adjustments from, written by a previous invocation of this program.")
    ("initial-transform", po::value(&opt.initial_transform_file)->default_value(""),
     "Before optimizing the cameras, apply to them the 4x4 rotation + translation transform from this file. The transform is in respect to the planet center, such as written by pc_align's source-to-reference or reference-to-source alignment transform. Set the number of iterations to 0 to stop at this step.")
    ("fixed-camera-indices",    po::value(&opt.fixed_cameras_indices_str)->default_value(""),
     "A list of indices, in quotes and starting from 0, with space as separator, corresponding to cameras to keep fixed during the optimization process.")
    ("fix-gcp-xyz",  po::bool_switch(&opt.fix_gcp_xyz)->default_value(false)->implicit_value(true),
     "If the GCP are highly accurate, use this option to not float them during the optimization.")

    ("csv-format",       po::value(&opt.csv_format_str)->default_value(""), asp::csv_opt_caption().c_str())
    ("csv-proj4",        po::value(&opt.csv_proj4_str)->default_value(""),
                                 "The PROJ.4 string to use to interpret the entries in input CSV files.")
    ("reference-terrain",        po::value(&opt.reference_terrain)->default_value(""),
     "An externally provided trustworthy 3D terrain, either as a DEM or as a lidar file, very close (after alignment) to the stereo result from the given images and cameras that can be used as a reference, instead of GCP, to optimize the intrinsics of the cameras.")
    ("max-num-reference-points", po::value(&opt.max_num_reference_points)->default_value(100000000),
     "Maximum number of (randomly picked) points from the reference terrain to use.")
    ("disparity-list",        po::value(&opt.disparity_list)->default_value(""),
                                 "The disparity files, one for each camera pair, to use when optimizing the intrinsics based on a reference terrain. Specify them as a list in quotes separated by spaces. First file is for the first two cameras, second for the next two cameras, etc.")
    ("max-disp-error",             po::value(&opt.max_disp_error)->default_value(-1),
     "When using a reference terrain as an external control, ignore as outliers xyz points which projected in the left image and transported by disparity to the right image differ by the projection of xyz in the right image by more than this value in pixels.")
    ("datum",            po::value(&opt.datum_str)->default_value(""),
                         "Use this datum. Needed only for ground control points, a camera position file, or for RPC sessions. Options: WGS_1984, D_MOON (1,737,400 meters), D_MARS (3,396,190 meters), MOLA (3,396,000 meters), NAD83, WGS72, and NAD27. Also accepted: Earth (=WGS_1984), Mars (=D_MARS), Moon (=D_MOON).")
    ("semi-major-axis",  po::value(&opt.semi_major)->default_value(0),
                         "Explicitly set the datum semi-major axis in meters (see above).")
    ("semi-minor-axis",  po::value(&opt.semi_minor)->default_value(0),
                         "Explicitly set the datum semi-minor axis in meters (see above).")
    ("session-type,t",   po::value(&opt.stereo_session_string)->default_value(""),
                         "Select the stereo session type to use for processing. Options: pinhole nadirpinhole isis dg rpc spot5 aster. Usually the program can select this automatically by the file extension.")
    ("min-matches",      po::value(&opt.min_matches)->default_value(30),
                         "Set the minimum  number of matches between images that will be considered.")
    ("ip-detect-method", po::value(&opt.ip_detect_method)->default_value(0),
                         "Interest point detection algorithm (0: Integral OBALoG (default), 1: OpenCV SIFT, 2: OpenCV ORB.")
    ("epipolar-threshold",       po::value(&opt.epipolar_threshold)->default_value(-1),
     "Maximum distance from the epipolar line to search for IP matches. Default: automatic calculation.")
    ("ip-inlier-factor",          po::value(&opt.ip_inlier_factor)->default_value(1.0/15.0),
     "A higher factor will result in more interest points, but perhaps also more outliers.")
    ("ip-uniqueness-threshold",          po::value(&opt.ip_uniqueness_thresh)->default_value(0.7),
     "A higher threshold will result in more interest points, but perhaps less unique ones.")
    ("ip-side-filter-percent",        po::value(&opt.ip_edge_buffer_percent)->default_value(-1),
     "Remove matched IPs this percentage from the image left/right sides.")
    ("normalize-ip-tiles", po::bool_switch(&opt.ip_normalize_tiles)->default_value(false)->implicit_value(true),
       "Individually normalize tiles used for IP detection.")
    ("disable-tri-ip-filter",    po::bool_switch(&opt.disable_tri_filtering)->default_value(false)->implicit_value(true),
     "Skip tri_ip filtering.")
    ("ip-debug-images",     po::value(&opt.ip_debug_images)->default_value(false)->implicit_value(true),
                    "Write debug images to disk when detecting and matching interest points.")
    ("elevation-limit",        po::value(&opt.elevation_limit)->default_value(Vector2(0,0), "auto"),
     "Limit on expected elevation range: Specify as two values: min max.")
    // Note that we count later on the default for lon_lat_limit being BBox2(0,0,0,0).
    ("lon-lat-limit",     po::value(&opt.lon_lat_limit)->default_value(BBox2(0,0,0,0), "auto"),
     "Limit the triangulated interest points to this longitude-latitude range. The format is: lon_min lat_min lon_max lat_max.")
    ("num-obalog-scales",              po::value(&opt.num_scales)->default_value(-1),
     "How many scales to use if detecting interest points with OBALoG. If not specified, 8 will be used. More can help for images with high frequency artifacts.")
    ("nodata-value",             po::value(&opt.nodata_value)->default_value(nan),
     "Pixels with values less than or equal to this number are treated as no-data. This overrides the no-data values from input images.")
    ("skip-rough-homography", po::bool_switch(&opt.skip_rough_homography)->default_value(false)->implicit_value(true),
     "Skip the step of performing datum-based rough homography if it fails.")
    ("individually-normalize",   po::bool_switch(&opt.individually_normalize)->default_value(false)->implicit_value(true),
                        "Individually normalize the input images instead of using common values.")
    ("max-iterations",   po::value(&opt.max_iterations)->default_value(1000),
                         "Set the maximum number of iterations.")
    ("parameter-tolerance",   po::value(&opt.parameter_tolerance)->default_value(1e-8),
     "Making this smaller will result in more iterations.")
    ("overlap-limit",    po::value(&opt.overlap_limit)->default_value(0),
                         "Limit the number of subsequent images to search for matches to the current image to this value.  By default match all images.")
    ("overlap-list",    po::value(&opt.overlap_list_file)->default_value(""),
     "A file containing a list of image pairs, one pair per line, separated by a space, which are expected to overlap. Matches are then computed only among the images in each pair.")
    ("position-filter-dist", po::value(&opt.position_filter_dist)->default_value(-1),
                         "Set a distance in meters and don't perform IP matching on images with an estimated camera center farther apart than this distance.  Requires --camera-positions.")
    ("rotation-weight",  po::value(&opt.rotation_weight)->default_value(0.0), "A higher weight will penalize more rotation deviations from the original configuration.")
    ("translation-weight",  po::value(&opt.translation_weight)->default_value(0.0), "A higher weight will penalize more translation deviations from the original configuration.")
    ("camera-weight",    po::value(&opt.camera_weight)->default_value(1.0),
                         "The weight to give to the constraint that the camera positions/orientations stay close to the original values (only for the Ceres solver).  A higher weight means that the values will change less. The options --rotation-weight and --translation-weight can be used for finer-grained control and a stronger response.")
    ("overlap-exponent",    po::value(&opt.overlap_exponent)->default_value(0.0),
     "If a feature is seen in n >= 2 images, give it a weight proportional with (n-1)^exponent.")
    ("ip-per-tile",             po::value(&opt.ip_per_tile)->default_value(0),
     "How many interest points to detect in each 1024^2 image tile (default: automatic determination).")
    ("num-passes",             po::value(&opt.num_ba_passes)->default_value(1),
     "How many passes of bundle adjustment to do. If more than one, outliers will be removed between passes using --remove-outliers-params, and re-optimization will take place. Match files and residual files with the outliers removed will be written to disk.")
    ("remove-outliers-params",        po::value(&opt.remove_outliers_params_str)->default_value("75.0 3.0 2.0 3.0", "'pct factor err1 err2'"),
     "Outlier removal based on percentage, when more than one bundle adjustment pass is used. Triangulated points with reprojection error in pixels larger than min(max('pct'-th percentile * 'factor', err1), err2) will be removed as outliers. Hence, never remove errors smaller than err1 but always remove those bigger than err2. Specify as a list in quotes. Default: '75.0 3.0 2.0 3.0'.")
    ("remove-outliers-by-disparity-params",  po::value(&opt.remove_outliers_by_disp_params)->default_value(Vector2(90.0,3.0), "pct factor"),
     "Outlier removal based on the disparity of interest points, when more than one bundle adjustment pass is used. Points with x or y disparity not within the 100-'pct' to 'pct' percentile interval expanded by 'factor' will be removed as outliers. Default: pct = 90.0 and factor = 3.0.")
    ("min-triangulation-angle",             po::value(&opt.min_triangulation_angle)->default_value(0.1),
     "The minimum angle, in degrees, at which rays must meet at a triangulated point to accept this point as valid.")
    ("use-lon-lat-height-gcp-error", po::bool_switch(&opt.use_llh_error)->default_value(false)->implicit_value(true),
     "When having GCP, instead of minimizing the xyz error (computed vs what is in the GCP file), minimize the lon-lat-height error. This way, if the height is not known accurately, in the GCP file the third sigma (corresponding to the height), can be given a bigger weight to reduce its importance.")

    ("mapprojected-data",  po::value(&opt.mapprojected_data)->default_value(""),
     "Given map-projected versions of the input images and the DEM mapprojected onto, and IP matches among them, create IP matches among the un-projected images before doing bundle adjustment. Niche and experimental, not for general use.")
    
    ("heights-from-dem",  po::value(&opt.heights_from_dem)->default_value(""),
     "If the cameras have already been bunde-adjusted and rigidly transformed to create a DEM aligned to a known high-quality DEM, in the original triangulated xyz points replace the heights with the ones from this high quality DEM and fix those points. This can be used to refine camera positions and intrinsics and works only for pinhole images. Niche and experimental, not for general use.")
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

  boost::to_lower( opt.stereo_session_string );
  
  // Separate out GCP files
  opt.gcp_files = asp::get_files_with_ext( opt.image_files, ".gcp", true );
  const size_t num_gcp_files = opt.gcp_files.size();
  vw_out() << "Found " << num_gcp_files << " GCP files on the command line.\n";
  

  // Separate the cameras from the images
  std::vector<std::string> inputs = opt.image_files;
  bool ensure_equal_sizes = true;
  asp::separate_images_from_cameras(inputs,
				    opt.image_files, opt.camera_files, // outputs
				    ensure_equal_sizes); 
  
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
      vw_throw( ArgumentErr() << "The overlap list does not exist.\n" << usage
		<< general_options );
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
    vw_throw( ArgumentErr() << "The camera weight must be non-negative.\n" << usage
              << general_options );

  if ( opt.rotation_weight < 0.0 )
    vw_throw( ArgumentErr() << "The rotation weight must be non-negative.\n" << usage
              << general_options );

  if ( opt.translation_weight < 0.0 )
    vw_throw( ArgumentErr() << "The translation weight must be non-negative.\n" << usage
              << general_options );


  if (opt.create_pinhole && !asp::has_pinhole_extension(opt.camera_files[0]))
    vw_throw( ArgumentErr() << "Cannot use special pinhole handling with non-pinhole input!\n");

  if (!opt.create_pinhole && opt.solve_intrinsics)
    vw_throw( ArgumentErr() << "Solving for intrinsic parameters is only supported with pinhole cameras.\n");

  if (!opt.create_pinhole && opt.approximate_pinhole_intrinsics)
    vw_throw( ArgumentErr() << "Cannot approximate intrinsics unless using pinhole cameras.\n");

  if (opt.approximate_pinhole_intrinsics && opt.solve_intrinsics)
    vw_throw( ArgumentErr() << "Cannot approximate intrinsics while solving for them.\n");

  if (opt.create_pinhole && opt.input_prefix != "")
    vw_throw( ArgumentErr() << "Cannot use initial adjustments with pinhole cameras. Read the cameras directly.\n");

  vw::string_replace(opt.remove_outliers_params_str, ",", " "); // replace any commas
  opt.remove_outliers_params = vw::str_to_vec<vw::Vector<double, 4> >(opt.remove_outliers_params_str);
  
  // Copy the IP settings to the global stereo_settings() object
  asp::stereo_settings().ip_matching_method      = opt.ip_detect_method;
  asp::stereo_settings().epipolar_threshold      = opt.epipolar_threshold;
  asp::stereo_settings().ip_inlier_factor        = opt.ip_inlier_factor;
  asp::stereo_settings().ip_uniqueness_thresh    = opt.ip_uniqueness_thresh;
  asp::stereo_settings().num_scales              = opt.num_scales;
  asp::stereo_settings().nodata_value            = opt.nodata_value;
  asp::stereo_settings().skip_rough_homography   = opt.skip_rough_homography;
  asp::stereo_settings().elevation_limit         = opt.elevation_limit;
  asp::stereo_settings().lon_lat_limit           = opt.lon_lat_limit;
  asp::stereo_settings().individually_normalize  = opt.individually_normalize;
  asp::stereo_settings().min_triangulation_angle = opt.min_triangulation_angle;
  asp::stereo_settings().disable_tri_filtering   = opt.disable_tri_filtering;
  asp::stereo_settings().ip_edge_buffer_percent  = opt.ip_edge_buffer_percent;
  asp::stereo_settings().ip_debug_images         = opt.ip_debug_images;
  asp::stereo_settings().ip_normalize_tiles      = opt.ip_normalize_tiles;

  // Ensure good order
  if ( asp::stereo_settings().lon_lat_limit != BBox2(0,0,0,0) ) {
    if ( asp::stereo_settings().lon_lat_limit.min().y() >
	 asp::stereo_settings().lon_lat_limit.max().y() ) 
      std::swap( asp::stereo_settings().lon_lat_limit.min().y(),
		 asp::stereo_settings().lon_lat_limit.max().y() );
    if ( asp::stereo_settings().lon_lat_limit.min().x() >
	 asp::stereo_settings().lon_lat_limit.max().x() ) 
      std::swap( asp::stereo_settings().lon_lat_limit.min().x(),
		 asp::stereo_settings().lon_lat_limit.max().x() );
      
  }
  
  if (!opt.camera_position_file.empty() && opt.csv_format_str == "")
    vw_throw( ArgumentErr() << "When using a camera position file, the csv-format "
	      << "option must be set.\n"
	      << usage << general_options );
  
  // Try to infer the datum, if possible, from the images. For
  // example, Cartosat-1 has that info in the Tif file.
  if (opt.datum_str == "") {
    vw::cartography::GeoReference georef;
    for (size_t it = 0; it < opt.image_files.size(); it++) {
      bool is_good = vw::cartography::read_georeference(georef, opt.image_files[it]);
      if (is_good && opt.datum_str == "" ){
        opt.datum_str = georef.datum().name();
        vw_out() << "Using the datum: " << opt.datum_str << ".\n";
      }
    }
  }
  
  if (opt.stereo_session_string == "rpc" && opt.datum_str == "")
    vw_throw( ArgumentErr() << "When the session type is RPC, the datum must be specified.\n"
              << usage << general_options );       
       
  if (opt.datum_str != ""){
    // If the user set the datum, use it.
    opt.datum.set_well_known_datum(opt.datum_str);
    asp::stereo_settings().datum = opt.datum_str; // for RPC
    vw_out() << "Will use datum: " << opt.datum << std::endl;
  }else if (opt.semi_major > 0 && opt.semi_minor > 0){
    // Otherwise, if the user set the semi-axes, use that.
    opt.datum = cartography::Datum("User Specified Datum",
                                   "User Specified Spheroid",
                                   "Reference Meridian",
                                   opt.semi_major, opt.semi_minor, 0.0);
    vw_out() << "Will use datum: " << opt.datum << std::endl;
  }else{ // Datum not specified
    if ( !opt.gcp_files.empty() || !opt.camera_position_file.empty() )
      vw_throw( ArgumentErr() << "When ground control points or a camera position file are used, "
                << "the datum must be specified.\n" << usage << general_options );
  }
  

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

  if (opt.initial_transform_file != "") {
    std::ifstream is(opt.initial_transform_file.c_str());
    for (size_t row = 0; row < opt.initial_transform.rows(); row++){
      for (size_t col = 0; col < opt.initial_transform.cols(); col++){
        double a;
        if (! (is >> a) )
          vw_throw( vw::IOErr() << "Failed to read initial transform from: "
                                << opt.initial_transform_file << "\n" );
        opt.initial_transform(row, col) = a;
      }
    }
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
	vw_throw( vw::IOErr() << "The camera index to keep fixed " << val
		  << " is out of bounds.\n" );
    }
  }
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


    boost::shared_ptr<LensDistortion> input_lens_distortion;
    
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
      if (opt.approximate_pinhole_intrinsics) {
        boost::shared_ptr<vw::camera::PinholeModel> pinhole_ptr = 
                boost::dynamic_pointer_cast<vw::camera::PinholeModel>(opt.camera_models.back());
        if (i == 0) // Record a copy of the input lens distortion
          input_lens_distortion = pinhole_ptr->lens_distortion()->copy();
        // Replace lens distortion with fast approximation
        vw::camera::update_pinhole_for_fast_point2pixel<TsaiLensDistortion, 
            TsaiLensDistortion::num_distortion_params>(*(pinhole_ptr.get()),
                                                       file_image_size(opt.image_files[i]));
      }

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
    const bool got_est_cam_positions =
      (estimated_camera_gcc.size() == static_cast<size_t>(num_images));
    
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
                     << other_pos << " with distance " << norm_2(this_pos - other_pos)
                     << std::endl;
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
          rsrc1(vw::DiskImageResourcePtr(image1_path)),
          rsrc2(vw::DiskImageResourcePtr(image2_path));
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
        
        // TODO: Move this into the IP finding code!
        // Compute the coverage fraction
        std::vector<ip::InterestPoint> ip1, ip2;
        ip::read_binary_match_file(match_filename, ip1, ip2);       
        int right_ip_width = rsrc1->cols()*
                              static_cast<double>(100-opt.ip_edge_buffer_percent)/100.0;
        Vector2i ip_size(right_ip_width, rsrc1->rows());
        double ip_coverage = calc_ip_coverage_fraction(ip2, ip_size);
        vw_out() << "IP coverage fraction = " << ip_coverage << std::endl;
          
                               
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
      vw_out() << "Loading GCP files...\n";
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
    if (opt.create_pinhole && have_est_camera_positions)
      init_pinhole_model_with_camera_positions(opt, estimated_camera_gcc);

    // If we have GPC's for pinhole cameras, try to do a simple affine
    // initialization of the camera parameters.
    // - This function also updates all the ControlNetwork world point
    //   positions.
    // - We could do this for other camera types too, but it would
    //   require us to be able to adjust our camera model positions.
    //   Otherwise we could init the adjustment values.
    if (opt.gcp_files.size() > 0) {

      if (opt.create_pinhole && !have_est_camera_positions)
        init_pinhole_model_with_gcp(opt);

      // Issue a warning if the GCPs are far away from the camera coords
      check_gcp_dists(opt);
    }


    if (opt.create_pinhole == false) {
      do_ba_with_model(opt);
    } else {

      // Use for local pinhole models, could also be used for other pinhole models.

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

      // If we approximated the lens distortions re-insert the original value here.
      bool has_datum = (opt.datum.name() != UNSPECIFIED_DATUM);
      ba_model.write_camera_models(cam_files, has_datum, opt.datum, input_lens_distortion.get());

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
