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

#include <asp/Core/Macros.h>
#include <asp/Camera/AdjustedLinescanDGModel.h>
#include <asp/Core/StereoSettings.h>
#include <vw/BundleAdjustment/ControlNetwork.h>
#include <vw/BundleAdjustment/ControlNetworkLoader.h>
#include <asp/Core/InterestPointMatching.h>
#include <asp/Core/BundleAdjustUtils.h>
#include <asp/Tools/jitter_adjust.h>
#include <vw/Core/Stopwatch.h>

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

int g_jitter_num_errors = 0;
Mutex g_jitter_mutex;

const int NUM_CAMERA_PARAMS = 6;
const int NUM_POINT_PARAMS = 3;

typedef Vector<double, NUM_CAMERA_PARAMS> camera_vector_t;

// In spite of trying a lot, I could not find a way to make a base class
// from which both AdjustedLinescanDGModel and PiecewiseAdjustedLinescanModel
// could inherit. Hence this wrapper.
class AdjustedModelWrapper {
public:
  AdjustedModelWrapper(std::string               const& session,
                       boost::shared_ptr<vw::camera::CameraModel> cam,
                       int interp_type,
                       vw::Vector2               const& adjustment_bounds,
                       std::vector<vw::Vector3>  const& position_adjustments,
                       std::vector<vw::Quat>     const& pose_adjustments,
                       vw::Vector2i              const& image_size):
    m_session(session)
  {
    if (m_session == "dg" || m_session == "dgmaprpc") {
      m_cam = boost::shared_ptr<vw::camera::CameraModel>
        (new asp::AdjustedLinescanDGModel(cam, interp_type, adjustment_bounds,
                                          position_adjustments, pose_adjustments,
                                          image_size));
    } else {
      m_cam = boost::shared_ptr<vw::camera::CameraModel>
        (new asp::PiecewiseAdjustedLinescanModel(cam, interp_type, adjustment_bounds,
                                                 position_adjustments, pose_adjustments,
                                                 image_size));
    }
    
  }

  vw::Vector2 point_to_pixel(vw::Vector3 const& point, double starty) const {
    if (m_session == "dg" || m_session == "dgmaprpc") 
      return
        dynamic_cast<asp::AdjustedLinescanDGModel*>
	(m_cam.get())->point_to_pixel(point, starty);
    
    return
      dynamic_cast<asp::PiecewiseAdjustedLinescanModel*>
      (m_cam.get())->point_to_pixel(point, starty);
  }
  
  std::vector<int> get_closest_adj_indices(double t){
    if (m_session == "dg" || m_session == "dgmaprpc") 
      return
        dynamic_cast<asp::AdjustedLinescanDGModel*>(m_cam.get())->get_closest_adj_indices(t);
    
    return
      dynamic_cast<asp::PiecewiseAdjustedLinescanModel*>(m_cam.get())->get_closest_adj_indices(t);
  }
  
private:
  std::string m_session;
  boost::shared_ptr<vw::camera::CameraModel> m_cam;

  
};
  
  
void populate_adjustements(std::vector<double> const& cameras_vec,
			   int start_index, int end_index,
			   std::vector<vw::Vector3> & position_adjustments,
			   std::vector<vw::Quat>    & pose_adjustments){

  // Extract the adjustments for just the current camera, and copy
  // them in arrays of vectors and quaternions.

  int num_adjustments = cameras_vec.size()/NUM_CAMERA_PARAMS;
  VW_ASSERT(0 <= start_index && start_index < end_index && end_index <= num_adjustments,
	    ArgumentErr() << "Book-keeping failure in camera indicies.");

  position_adjustments.clear();
  pose_adjustments.clear();

  for (int cam_index = start_index; cam_index < end_index; cam_index++) {
    Vector3 position, pose;
    for (int b = 0; b < NUM_CAMERA_PARAMS/2; b++) {
      position[b] = cameras_vec[NUM_CAMERA_PARAMS * cam_index + b + 0];
      pose[b]     = cameras_vec[NUM_CAMERA_PARAMS * cam_index + b + NUM_CAMERA_PARAMS/2];
    }

    position_adjustments.push_back(position);
    pose_adjustments.push_back(axis_angle_to_quaternion(pose));
  }
}

// A ceres cost function. We pass in the observation, the model, and
// the current camera and point indices. The result is the residual,
// the difference in the observation and the projection of the point
// into the camera, normalized by pixel_sigma.
struct PiecewiseReprojectionError {
  PiecewiseReprojectionError(Vector2 const& observation, Vector2 const& pixel_sigma,
			     Vector2 const& adjustment_bounds,
			     std::vector<double> const& cameras_vec,
			     boost::shared_ptr<vw::camera::CameraModel> cam,
			     Vector2i const& image_size,
			     std::string const& session,
			     int start_index,
			     int camera_index1, int camera_index2,
			     int camera_index3, int camera_index4,
			     int end_index,
			     size_t ipt):
    m_observation(observation),
    m_pixel_sigma(pixel_sigma),
    m_adjustment_bounds(adjustment_bounds),
    m_cameras_vec(cameras_vec),
    m_cam(cam),
    m_image_size(image_size),
    m_session(session),
    m_start_index(start_index),
    m_camera_index1(camera_index1),
    m_camera_index2(camera_index2),
    m_camera_index3(camera_index3),
    m_camera_index4(camera_index4),
    m_end_index(end_index),
    m_ipt(ipt){}

  template <typename T>
  bool do_calc(const T* const camera1, const T* const camera2,
	       const T* const camera3, const T* const camera4,
	       const T* const point, T* residuals) const {

    try{

      int num_cameras = m_cameras_vec.size();
      std::vector<double> local_cameras_vec = m_cameras_vec;

	// Copy the camera adjustments to local storage.  Update them
	// with the latest value for the current camera being floated.

      for (int i = 1; i <= 4; i++) {

	int camera_index = -1;
	const T * camera = NULL;
	if (i == 1 && m_camera_index1 >= 0 && camera1 != NULL) {
	  camera_index = m_camera_index1;
	  camera       = camera1;
	} else if (i == 2 && m_camera_index2 >= 0 && camera2 != NULL) {
	  camera_index = m_camera_index2;
	  camera       = camera2;
	} else if (i == 3 && m_camera_index3 >= 0 && camera3 != NULL) {
	  camera_index = m_camera_index3;
	  camera       = camera3;
	} else if (i == 4 && m_camera_index4 >= 0 && camera4 != NULL) {
	  camera_index = m_camera_index4;
	  camera       = camera4;
	}

	if (camera == NULL) continue;

	VW_ASSERT(0 <= camera_index && camera_index < num_cameras,
		  ArgumentErr() << "Out of bounds in the number of cameras");

	VW_ASSERT(0 <= m_start_index && m_start_index <= camera_index
		  && camera_index < m_end_index && m_end_index <= num_cameras,
		  ArgumentErr() << "Book-keeping failure in camera indicies");

	for (int p = 0; p < NUM_CAMERA_PARAMS; p++) {
	  local_cameras_vec[NUM_CAMERA_PARAMS*camera_index + p] = camera[p];
	}
      }

      // Extract the adjustments specific to the current camera
      std::vector<vw::Vector3> position_adjustments;
      std::vector<vw::Quat>   pose_adjustments;
      populate_adjustements(local_cameras_vec,
			    m_start_index, m_end_index,
			    position_adjustments, pose_adjustments);

      // The adjusted camera has just the adjustments, it does not create a full
      // copy of the camera.
      int interp_type = stereo_settings().piecewise_adjustment_interp_type;

      asp::AdjustedModelWrapper cam_wrapper(m_session, m_cam,
                                            interp_type,
                                            m_adjustment_bounds,
                                            position_adjustments, pose_adjustments,
                                            m_image_size);
      
      // Copy the input data to structures expected by the BA model
      Vector3 point_vec;
      for (size_t p = 0; p < point_vec.size(); p++)
	point_vec[p]  = (double)point[p];

      // Project the current point into the current camera.  Note that
      // we pass the observation as an initial guess, as the
      // prediction is hopefully not too far from it.
      Vector2 prediction = cam_wrapper.point_to_pixel(point_vec, m_observation.y());

      // The error is the difference between the predicted and observed position,
      // normalized by sigma.
      residuals[0] = (prediction[0] - m_observation[0])/m_pixel_sigma[0];
      residuals[1] = (prediction[1] - m_observation[1])/m_pixel_sigma[1];

    } catch (std::exception const& e) {

      // Failed to compute residuals

      Mutex::Lock lock( g_jitter_mutex );
      g_jitter_num_errors++;
      if (g_jitter_num_errors < 100) {
	vw_out(ErrorMessage) << e.what() << std::endl;
      }else if (g_jitter_num_errors == 100) {
	vw_out() << "Will print no more error messages about "
                 << "failing to compute residuals.\n";
      }

      residuals[0] = T(1e+20);
      residuals[1] = T(1e+20);
      return false;
    }

    return true;
  }

  template <typename T>
  bool operator()(const T* const camera1, const T* const point, T* residuals) const {
    return do_calc<T>(camera1, NULL, NULL, NULL, point, residuals);
  }

  template <typename T>
  bool operator()(const T* const camera1,
		  const T* const camera2,
		  const T* const point, T* residuals) const {
    return do_calc<T>(camera1, camera2, NULL, NULL,  point, residuals);
  }

  template <typename T>
  bool operator()(const T* const camera1,
		  const T* const camera2,
		  const T* const camera3,
		  const T* const point, T* residuals) const {
    return do_calc<T>(camera1, camera2, camera3, NULL, point, residuals);
  }

  template <typename T>
  bool operator()(const T* const camera1,
		  const T* const camera2,
		  const T* const camera3,
		  const T* const camera4,
		  const T* const point, T* residuals) const {
    return do_calc<T>(camera1, camera2, camera3, camera4, point, residuals);
  }

  // Factory to hide the construction of the CostFunction object from
  // the client code.
  static ceres::CostFunction* Create(Vector2 const& observation,
				     Vector2 const& pixel_sigma,
				     Vector2 const& adjustment_bounds,
				     std::vector<double> const& cameras_vec,
				     boost::shared_ptr<vw::camera::CameraModel> cam,
				     Vector2i const& image_size,
				     std::string const& session,
				     int start_index,
				     int camera_index1,
				     int camera_index2,
				     int camera_index3,
				     int camera_index4,
				     int end_index,
				     size_t ipt // point index
				     ){

    if (camera_index2 < 0 && camera_index3 < 0 && camera_index4 < 0)
      return (new ceres::NumericDiffCostFunction<PiecewiseReprojectionError,
	      ceres::CENTRAL, 2, NUM_CAMERA_PARAMS, NUM_POINT_PARAMS>
	      (new PiecewiseReprojectionError(observation, pixel_sigma, adjustment_bounds,
					      cameras_vec, cam, image_size, session,
					      start_index,
					      camera_index1, camera_index2,
					      camera_index3, camera_index4,
					      end_index,
					      ipt)));

    if (camera_index3 < 0 && camera_index4 < 0)
      return (new ceres::NumericDiffCostFunction<PiecewiseReprojectionError,
	      ceres::CENTRAL, 2, NUM_CAMERA_PARAMS, NUM_CAMERA_PARAMS, NUM_POINT_PARAMS>
	      (new PiecewiseReprojectionError(observation, pixel_sigma, adjustment_bounds,
					      cameras_vec, cam, image_size, session,
					      start_index,
					      camera_index1, camera_index2,
					      camera_index3, camera_index4,
					      end_index,
					      ipt)));

    if (camera_index4 < 0)
      return (new ceres::NumericDiffCostFunction<PiecewiseReprojectionError,
	      ceres::CENTRAL, 2, NUM_CAMERA_PARAMS, NUM_CAMERA_PARAMS,
              NUM_CAMERA_PARAMS, NUM_POINT_PARAMS>
	      (new PiecewiseReprojectionError(observation, pixel_sigma, adjustment_bounds,
					      cameras_vec, cam, image_size, session,
					      start_index,
					      camera_index1, camera_index2,
					      camera_index3, camera_index4,
					      end_index,
					      ipt)));

    if (camera_index1 < 0 || camera_index2 < 0 || camera_index3 < 0)
      vw_throw( ArgumentErr() << "Book-keeping failure in camera indices: "
		<< camera_index1 << ' ' << camera_index2 << ' ' << camera_index3 << ".\n" );


    return (new ceres::NumericDiffCostFunction<PiecewiseReprojectionError,
	    ceres::CENTRAL, 2, NUM_CAMERA_PARAMS, NUM_CAMERA_PARAMS,
            NUM_CAMERA_PARAMS, NUM_CAMERA_PARAMS, NUM_POINT_PARAMS>
	    (new PiecewiseReprojectionError(observation, pixel_sigma, adjustment_bounds,
					    cameras_vec, cam, image_size, session,
					    start_index,
					    camera_index1, camera_index2,
					    camera_index3, camera_index4,
					    end_index,
					    ipt)));
  }

  Vector2 m_observation;
  Vector2 m_pixel_sigma;
  Vector2 m_adjustment_bounds;
  std::vector<double> const& m_cameras_vec;  // alias
  boost::shared_ptr<vw::camera::CameraModel> m_cam;
  Vector2i m_image_size; // TODO: Group this with the above
  std::string m_session;
								   
  // all adjustments for the current camera will be >= this
  int m_start_index;

  // indices of the current adjustments
  int m_camera_index1, m_camera_index2, m_camera_index3, m_camera_index4;

  int m_end_index;    // all adjustment indices for current camera will be < this
  int m_ipt;          // index of the current 3D point in the vector of points
};

// A ceres cost function. The residual is the difference between the
// original camera center and the current (floating) camera center.
// This cost function prevents the cameras from straying too far from
// their starting point.
struct CamError {

  CamError(camera_vector_t const& orig_cam, double weight):
    m_orig_cam(orig_cam), m_weight(weight){}

  template <typename T>
  bool operator()(const T* const cam_vec, T* residuals) const {

    // Note that we allow the position to vary more than the orientation.
    for (size_t p = 0; p < 3; p++)
      residuals[p] = 1e-6*m_weight*(cam_vec[p] - m_orig_cam[p]);
    for (size_t p = 3; p < m_orig_cam.size(); p++)
      residuals[p] = m_weight*(cam_vec[p] - m_orig_cam[p]);

    return true;
  }

  // Factory to hide the construction of the CostFunction object from
  // the client code.
  static ceres::CostFunction* Create(camera_vector_t const& orig_cam, double weight){
    return (new ceres::NumericDiffCostFunction<CamError, ceres::CENTRAL,
	    NUM_CAMERA_PARAMS, NUM_CAMERA_PARAMS>
	    (new CamError(orig_cam, weight)));

  }

  camera_vector_t m_orig_cam;
  double m_weight;
};

ceres::LossFunction* get_jitter_loss_function(){
  return new ceres::CauchyLoss(0.5);
}


std::vector<double> * g_cameras_vec;

// Will be called at each iteration. Here we can put any desired logging info.
class PiecewiseBaCallback: public ceres::IterationCallback {
public:
  virtual ceres::CallbackReturnType operator()
    (const ceres::IterationSummary& summary) {

    return ceres::SOLVER_CONTINUE;
  }
};

// Given a set of interest points, find their y values, and return the bounds
// corresponding to the given percentiles.
vw::Vector2 find_bounds_from_percentiles(std::vector<ip::InterestPoint> const& ip,
                                         vw::Vector2 const& percentiles){

  if (percentiles[0] < 0 || percentiles[0] >= percentiles[1] || percentiles[1] > 100)
    vw_throw( ArgumentErr() << "Percentiles must be between 0 and 100, "
              << "with the second larger than the first.\n" );

  int len = ip.size();
  if (len <= 0)
    vw_throw( ArgumentErr() << "No interest points found. Cannot compute "
              << "piecewise adjustments for jitter correction.\n" );

  std::vector<double> Y(len);
  for (int ip_iter = 0; ip_iter < len; ip_iter++) Y[ip_iter] = ip[ip_iter].y;
  std::sort(Y.begin(), Y.end());

  int beg = round(percentiles[0]*len/100.0); beg = std::min(beg, len - 1);
  int end = round(percentiles[1]*len/100.0); end = std::min(end, len - 1);

  return Vector2(Y[beg], Y[end]);
}

// Use Ceres to do piecewise bundle adjustment (multiple along-track
// adjustments per linescan camera to solve for jitter).
void jitter_adjust(std::vector<std::string> const& image_files,
                   std::vector<std::string> const& camera_files,
                   std::vector< boost::shared_ptr<vw::camera::CameraModel> >
                   const& input_camera_models,
                   std::string const& out_prefix,
                   std::string const& session,
                   std::string const& match_file,
                   int num_threads){

  vw_out() << "Performing piecewise adjustments to correct for jitter.\n";

  if (image_files.size() != camera_files.size())
    vw_throw( ArgumentErr() << "Expecting as many images as cameras.\n" );

  int num_cameras = input_camera_models.size();
  if (num_cameras != 2)
    vw_throw( ArgumentErr() << "Can solve for jitter only for two cameras.\n" );

  std::map< std::pair<int, int>, std::string> match_files;
  match_files[std::pair<int, int>(0, 1)] = match_file;

  int min_matches = 30;   // TODO: Think more here
  double min_angle = 0.1; // in degrees
  ba::ControlNetwork cnet("JitterAdjust");
  bool triangulate_control_points = true;
  bool success = build_control_network(triangulate_control_points,
                                       cnet, // output
                                       input_camera_models, image_files,
                                       match_files, min_matches,
                                       min_angle*(M_PI/180));

  if (!success)
    vw_throw( ArgumentErr() << "Insufficient number of matches to solve for jitter.\n" );

  int num_points = cnet.size();

  // Create the adjustment bounds based on percentiles of interest points.
  std::vector<Vector2> adjustment_bounds;
  adjustment_bounds.resize(num_cameras);
  std::vector<ip::InterestPoint> ip0, ip1;
  ip::read_binary_match_file(match_file, ip0, ip1);
  adjustment_bounds[0]
    = find_bounds_from_percentiles(ip0, stereo_settings().piecewise_adjustment_percentiles);
  adjustment_bounds[1]
    = find_bounds_from_percentiles(ip1, stereo_settings().piecewise_adjustment_percentiles);

  for (int icam = 0; icam < num_cameras; icam++)
    vw_out() << "Placing first and last adjustment for image "
             << image_files[icam] << " at (un-transformed) lines: "
             << adjustment_bounds[icam][0] << ' '
             << adjustment_bounds[icam][1] << "\n";

  // Decide how many adjustments to use per camera
  int num_total_adj = 0;
  std::vector<int> num_adj_per_cam;
  int num_images = image_files.size();
  VW_ASSERT(num_images == num_cameras,
            ArgumentErr() << "Expecting as many images as cameras.\n");

  for (int icam = 0; icam < num_cameras; icam++) {

    // We get the number of image lines from the bounds on where to
    // place the adjustments.
    int num_lines = adjustment_bounds[icam][1] - adjustment_bounds[icam][0];
    VW_ASSERT(num_lines > 0, ArgumentErr() << "Invalid bounds for piecewise adjustment.\n");

    int num_adj = round(double(num_lines)/stereo_settings().image_lines_per_piecewise_adjustment);
    if (num_adj <= 0) num_adj = 1;

    // Ensure we have at least two adjustments, at endpoints.
    // Otherwise one could simply use the single-camera adjustment
    // flow and not come here. So, if the image has 2000 lines, and we
    // need 1000 lines per adjustment, we'll get 3 adjustments, at
    // line 0, 1000, 2000.
    num_adj++;

    num_adj_per_cam.push_back(num_adj);
    vw_out() << "Number of adjustments for image " << image_files[icam]
             << ": " << num_adj << std::endl;

    num_total_adj += num_adj;
  }

  // The camera adjustment and point variables concatenated into
  // vectors. The camera adjustments start as 0. The points come from
  // the network.
  std::vector<double> cameras_vec(num_total_adj*NUM_CAMERA_PARAMS, 0.0);

  // If the input cameras are bundle-adjusted, use those adjustments
  // as initial guess for piecewise adjustments. also pull the
  // unadjusted DG cameras.
  int start_index = 0;
  std::vector< boost::shared_ptr<vw::camera::CameraModel> > camera_models;
  for (int icam = 0; icam < (int)input_camera_models.size(); icam++) {

    if (icam > 0)
      start_index += num_adj_per_cam[icam - 1];
    int end_index = start_index + num_adj_per_cam[icam];

    AdjustedCameraModel* adj_cam
      = dynamic_cast<AdjustedCameraModel*>(input_camera_models[icam].get());

    if (adj_cam == NULL) {
      camera_models.push_back(input_camera_models[icam]);
    }else{
      camera_models.push_back(adj_cam->unadjusted_model());

      Vector<double, NUM_CAMERA_PARAMS/2> position = adj_cam->translation();
      Vector<double, NUM_CAMERA_PARAMS/2> pose     = adj_cam->axis_angle_rotation();

      for (int cam_index = start_index; cam_index < end_index; cam_index++) {
        int curr_start = NUM_CAMERA_PARAMS * cam_index;
        for (int b = 0; b < NUM_CAMERA_PARAMS/2; b++) {
          cameras_vec[curr_start + 0                   + b] = position[b];
          cameras_vec[curr_start + NUM_CAMERA_PARAMS/2 + b] = pose[b];
        }

      }
    }
  }

  // To be able to use it from the callback
  g_cameras_vec = &cameras_vec;

  // Camera extrinsics
  double* cameras = &cameras_vec[0];

  // Points
  std::vector<double> points_vec(num_points*NUM_POINT_PARAMS, 0.0);
  for (int ipt = 0; ipt < num_points; ipt++){
    for (int q = 0; q < NUM_POINT_PARAMS; q++){
      points_vec[ipt*NUM_POINT_PARAMS + q] = cnet[ipt].position()[q];
    }
  }
  double* points = &points_vec[0];

  // The camera positions and orientations before we float them
  std::vector<double> orig_cameras_vec = cameras_vec;

  // The ceres problem
  ceres::Problem problem;

  // Add the cost function component for difference of pixel observations
  CameraRelationNetwork<JFeature> crn;
  crn.read_controlnetwork(cnet);
  start_index = 0;
  std::vector<Vector2i> sizes;
  
  for (int icam = 0; icam < (int)crn.size(); icam++) {

    if (icam > 0)
      start_index += num_adj_per_cam[icam - 1];
    int end_index = start_index + num_adj_per_cam[icam];

    // Initialize an adjusted model with no adjustments. We need it simply to
    // look up the index adjustments.
    std::vector<vw::Vector3> position_adjustments(num_adj_per_cam[icam]);
    std::vector<vw::Quat> pose_adjustments(num_adj_per_cam[icam]);

    DiskImageView<float> img(image_files[icam]);
    Vector2i image_size(img.cols(), img.rows());
    sizes.push_back(image_size);

    asp::AdjustedModelWrapper cam_wrapper(session, 
                                          camera_models[icam],
                                          stereo_settings().piecewise_adjustment_interp_type,
                                          adjustment_bounds[icam],
                                          position_adjustments, pose_adjustments,
                                          image_size);
    
    typedef CameraNode<JFeature>::iterator crn_iter;
    for (crn_iter fiter = crn[icam].begin(); fiter != crn[icam].end(); fiter++){

      // The index of the 3D point
      int ipt = (**fiter).m_point_id;

      VW_ASSERT(icam < num_cameras, ArgumentErr() << "Out of bounds in the number of cameras");
      VW_ASSERT(ipt < num_points,   ArgumentErr() << "Out of bounds in the number of points");

      // The observed value for the projection of point with index ipt into
      // the camera with index icam.
      Vector2 observation = (**fiter).m_location;
      Vector2 pixel_sigma = (**fiter).m_scale;

      // This is a bugfix for NaN problems
      if (pixel_sigma != pixel_sigma)
	pixel_sigma = Vector2(1, 1);

      // The adjustments that will be affected by the current observation (recall that
      // the adjustments are placed at several scan lines (image rows)).
      std::vector<int> indices = cam_wrapper.get_closest_adj_indices(observation.y());

      VW_ASSERT(indices.size() >= 1 && indices.size() <= 4,
		ArgumentErr() << "Expecting between 1 and 4 camera indices.");

      // We assume below there are only 4 closest indices!
      int camera_index1 = -1, camera_index2 = -1, camera_index3 = -1, camera_index4 = -1;

      if (indices.size() >= 1) camera_index1 = indices[1-1] + start_index;
      if (indices.size() >= 2) camera_index2 = indices[2-1] + start_index;
      if (indices.size() >= 3) camera_index3 = indices[3-1] + start_index;
      if (indices.size() >= 4) camera_index4 = indices[4-1] + start_index;

      VW_ASSERT(            0 <= camera_index1 &&
		camera_index1 <  num_total_adj &&
		camera_index2 <  num_total_adj &&
		camera_index3 <  num_total_adj &&
		camera_index4 <  num_total_adj,
		ArgumentErr() << "Out of bounds in the camera index");

      // Each observation corresponds to a pair of a camera and a point
      double * point = points  + ipt * NUM_POINT_PARAMS;

      ceres::LossFunction* loss_function = get_jitter_loss_function();

      ceres::CostFunction* cost_function
	= PiecewiseReprojectionError::Create(observation, pixel_sigma,
					     adjustment_bounds[icam],
					     cameras_vec, camera_models[icam],
					     sizes[icam],
					     session,
					     start_index,
					     camera_index1, camera_index2,
					     camera_index3, camera_index4,
					     end_index,
					     ipt);
      if      (camera_index2 < 0) {
	problem.AddResidualBlock(cost_function, loss_function,
				 cameras + camera_index1 * NUM_CAMERA_PARAMS,
				 point);

      }else if (camera_index3 < 0) {
	problem.AddResidualBlock(cost_function, loss_function,
				 cameras + camera_index1 * NUM_CAMERA_PARAMS,
				 cameras + camera_index2 * NUM_CAMERA_PARAMS,
				 point);

      }else if (camera_index4 < 0) {
	problem.AddResidualBlock(cost_function, loss_function,
				 cameras + camera_index1 * NUM_CAMERA_PARAMS,
				 cameras + camera_index2 * NUM_CAMERA_PARAMS,
				 cameras + camera_index3 * NUM_CAMERA_PARAMS,
				 point);

      }else if (camera_index1 >= 0 && camera_index2 >= 0 &&
		camera_index3 >= 0 && camera_index4 >= 0){
	problem.AddResidualBlock(cost_function, loss_function,
				 cameras + camera_index1 * NUM_CAMERA_PARAMS,
				 cameras + camera_index2 * NUM_CAMERA_PARAMS,
				 cameras + camera_index3 * NUM_CAMERA_PARAMS,
				 cameras + camera_index4 * NUM_CAMERA_PARAMS,
				 point);

      }else{
	vw_throw( ArgumentErr() << "Book-keeping failure in camera indices! "
		  << "Values of start_index, camera indices, and end_index are "
		  << start_index << ' ' << camera_index1 << ' ' << camera_index2
		  << ' ' << camera_index3 << ' ' << camera_index4 << ' ' << end_index << "\n" );
      }

    }

  }

  // Add camera constraints
  double camera_weight = stereo_settings().piecewise_adjustment_camera_weight;
  for (int cam_index = 0; cam_index < num_total_adj; cam_index++){

    camera_vector_t orig_cam;
    for (int q = 0; q < NUM_CAMERA_PARAMS; q++)
      orig_cam[q] = orig_cameras_vec[NUM_CAMERA_PARAMS * cam_index + q];

    ceres::CostFunction* cost_function = CamError::Create(orig_cam, camera_weight);

    ceres::LossFunction* loss_function = get_jitter_loss_function();

    double * camera  = cameras  + NUM_CAMERA_PARAMS * cam_index;
    problem.AddResidualBlock(cost_function, loss_function, camera);
  }

  vw::vw_out() << "Solving for jitter" << std::endl;
  Stopwatch sw;
  sw.start();
  // Solve the problem
  ceres::Solver::Options options;
  options.gradient_tolerance = 1e-16;
  options.function_tolerance = 1e-16;
  options.max_num_iterations = 1000;
  options.max_num_consecutive_invalid_steps = 100; // try hard
  options.minimizer_progress_to_stdout = true;

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
  PiecewiseBaCallback callback;
  options.callbacks.push_back(&callback);
  options.update_state_every_iteration = true; // ensure we have the latest adjustments

  ceres::Solver::Summary summary;
  ceres::Solve(options, &problem, &summary);
  vw_out() << summary.FullReport() << "\n";
  if (summary.termination_type == ceres::NO_CONVERGENCE){
    // Print a clarifying message, so the user does not think that the algorithm failed.
    vw_out() << "Found a valid solution, but did not reach the actual minimum." << std::endl;
  }

  sw.stop();
  vw::vw_out() << "Jitter solve elapsed time: " << sw.elapsed_seconds() << std::endl;
  
  // Save the adjustments. We will recover them later based on the output prefix.
  start_index = 0;
  for (int icam = 0; icam < (int)crn.size(); icam++) {

    if (icam > 0)
      start_index += num_adj_per_cam[icam - 1];
    int end_index = start_index + num_adj_per_cam[icam];

    std::vector<vw::Vector3> position_adjustments;
    std::vector<vw::Quat>    pose_adjustments;
    populate_adjustements(cameras_vec,
			  start_index, end_index,
			  position_adjustments,
			  pose_adjustments);

    std::string adjust_file = asp::bundle_adjust_file_name(out_prefix,
							   image_files[icam],
							   camera_files[icam]);
    vw_out() << "Writing: " << adjust_file << std::endl;
    asp::write_adjustments(adjust_file,
			   adjustment_bounds[icam],
			   position_adjustments, pose_adjustments,
                           session);
  }

}

} // end namespace asp
