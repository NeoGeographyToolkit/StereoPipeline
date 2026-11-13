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

/// \file BundleAdjustCamera.cc
///

#include <asp/Camera/BundleAdjustCamera.h>
#include <asp/Camera/CameraResectioning.h>
#include <asp/Camera/Covariance.h>
#include <asp/Camera/LinescanUtils.h>
#include <asp/Camera/RPC_XML.h>
#include <asp/Camera/CameraErrorPropagation.h>
#include <asp/Core/IpMatchingAlgs.h>
#include <asp/Core/StereoSettings.h>
#include <asp/Core/ImageUtils.h>

#include <vw/BundleAdjustment/ControlNetworkLoader.h>
#include <vw/Cartography/CameraBBox.h>
#include <vw/Cartography/BathyStereoModel.h>
#include <vw/InterestPoint/MatcherIO.h>
#include <vw/Math/Geometry.h>
#include <vw/Camera/OpticalBarModel.h>
#include <vw/Camera/CameraUtilities.h>
#include <vw/Camera/CameraImage.h>
#include <vw/Camera/LensDistortion.h>
#include <vw/Image/ImageChannels.h>
#include <vw/Core/StringUtils.h>

#include <usgscsm/UsgsAstroLsSensorModel.h>

using namespace vw;

namespace asp {

/// Simple class to manage position/rotation information.
/// - This is the data type stored in pc_align output files,
///   bundle adjustment files, and the position of pinhole cameras.

// Constructors

void check_gcp_dists(boost::shared_ptr<vw::ControlNetwork> const& cnet_ptr) {

  const ControlNetwork & cnet = *cnet_ptr.get(); // Helper alias

  // Grow a box over the GCP and another over the triangulated tie points
  BBox3 gcp_box, ip_box;
  for (size_t ipt = 0; ipt < cnet.size(); ipt++) {

    if (cnet[ipt].position() == Vector3() || cnet[ipt].size() <= 1)
      continue;

    if (cnet[ipt].type() == ControlPoint::GroundControlPoint)
      gcp_box.grow(cnet[ipt].position());
    else
      ip_box.grow(cnet[ipt].position());
  }

  // Can do this check only if we have both point types
  if (gcp_box.empty() || ip_box.empty())
    return;

  // Distance between the two boxes, per axis. It is zero when they overlap.
  Vector3 gap;
  for (int i = 0; i < 3; i++)
    gap[i] = std::max(0.0, std::max(gcp_box.min()[i] - ip_box.max()[i],
                                    ip_box.min()[i] - gcp_box.max()[i]));

  double dist = norm_2(gap);
  if (dist > 100000.0)
    vw_out() << "WARNING: The GCP are about " << dist/1000.0 << " km away from the "
             << "triangulated interest points. Check the GCP file for a lat-lon "
             << "coordinate swap or other error.\n";
}

// Initialize the position and orientation of each pinhole camera model using
// a least squares error transform to match the provided camera positions.
// This function overwrites the camera parameters in-place
bool init_pinhole_model_with_camera_positions(boost::shared_ptr<vw::ControlNetwork> const& cnet,
 std::vector<vw::CamPtr> & camera_models,
 std::vector<std::string> const& image_files,
 std::vector<vw::Vector3> const & estimated_camera_gcc) {

  vw_out() << "Initializing camera positions from input file." << std::endl;

  // Count the number of matches and check for problems
  const int num_cameras = image_files.size();
  if (int(estimated_camera_gcc.size()) != num_cameras)
    vw_throw(ArgumentErr() << "No camera matches provided to init function!\n");

  vw_out() << "Num cameras: " << num_cameras << std::endl;

  int num_matches_found = 0;
  for (int i = 0; i < num_cameras; i++)
    if (estimated_camera_gcc[i] != Vector3(0,0,0))
      ++num_matches_found;

  vw_out() << "Number of matches found: " << num_matches_found << std::endl;

  const int MIN_NUM_MATCHES = 3;
  if (num_matches_found < MIN_NUM_MATCHES)
    vw_throw(ArgumentErr() << "At least " << MIN_NUM_MATCHES
              << " camera position matches are required to initialize cameras "
              << "based on camera positions only.\n");

  // Populate matrices containing the current and known camera positions.
  vw::Matrix<double> points_in(3, num_matches_found), points_out(3, num_matches_found);
  typedef vw::MatrixCol<vw::Matrix<double>> ColView;
  int index = 0;
  for (int i = 0; i < num_cameras; i++) {
    // Skip cameras with no matching record
    if (estimated_camera_gcc[i] == Vector3(0,0,0))
      continue;

    // Get the two GCC positions
    Vector3 gcc_in  = camera_models[i]->camera_center(Vector2(0,0));
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
  vw::find_3D_transform(points_in, points_out, rotation, translation, scale);

  // Update the camera and point information with the new transform
  apply_rigid_transform(rotation, translation, scale, camera_models, cnet);
  return true;
}

// Given at least two images, each having at least 3 GCP that are not seen in other
// images, find and apply a transform to the camera system based on them.
void transform_cameras_with_indiv_image_gcp
(boost::shared_ptr<ControlNetwork> const& cnet_ptr,
 std::vector<vw::CamPtr> & camera_models) {

  vw_out() << "Applying transform to cameras given several GCP not shared "
           << "among the images.\n";

  int num_cams = camera_models.size();

  // Create pinhole cameras
  std::vector<PinholeModel> pinhole_cams;
  for (int icam = 0; icam < num_cams; icam++) {
    vw::PinholeModel * pincam
      = dynamic_cast<vw::PinholeModel*>(camera_models[icam].get());
    VW_ASSERT(pincam != NULL,
          vw::ArgumentErr() << "A pinhole camera expected.\n");
    pinhole_cams.push_back(*pincam);
  }

  // Extract from the control network each pixel for each camera together
  // with its xyz.
  std::vector<std::vector<Vector3>> xyz;
  std::vector<std::vector<Vector2>> pix;
  xyz.resize(num_cams);
  pix.resize(num_cams);
  const ControlNetwork & cnet = *cnet_ptr.get(); // Helper alias

  for (int ipt = 0; ipt < cnet.size(); ipt++) {

    // Keep only gcp
    if (cnet[ipt].type() != ControlPoint::GroundControlPoint)
      continue;

    for (auto measure = cnet[ipt].begin(); measure != cnet[ipt].end(); measure++) {
      int cam_it = measure->image_id();
      if (cam_it < 0 || cam_it >= num_cams)
        vw_throw(ArgumentErr() << "Error: cnet index out of range.\n");

      Vector2 pixel(measure->position()[0],  measure->position()[1]);
      pix[cam_it].push_back(pixel);
      xyz[cam_it].push_back(cnet[ipt].position());
    }
  }

  Matrix3x3 rotation;
  Vector3   translation;
  double    scale;
  vw::align_cameras_to_ground(xyz, pix, pinhole_cams, rotation, translation, scale);

  // Update the camera and point information with the new transform
  vw_out() << "Applying transform based on GCP:\n";
  vw_out() << "Rotation:    " << rotation    << "\n";
  vw_out() << "Translation: " << translation << "\n";
  vw_out() << "Scale:       " << scale       << "\n";
  apply_rigid_transform(rotation, translation, scale, camera_models, cnet_ptr);
}

// RMS distance between the input points mapped by a similarity transform and
// the output points. Used to judge a fit of triangulated GCP to ground GCP.
double similarity_fit_rms(std::vector<vw::Vector3> const& in_vec,
                          std::vector<vw::Vector3> const& out_vec,
                          vw::Matrix3x3 const& rotation,
                          vw::Vector3 const& translation,
                          double scale) {
  if (in_vec.empty())
    return 0.0;

  double sum = 0.0;
  for (size_t i = 0; i < in_vec.size(); i++) {
    vw::Vector3 pred = scale * rotation * in_vec[i] + translation;
    double dist = norm_2(pred - out_vec[i]);
    sum += dist * dist;
  }

  return std::sqrt(sum / in_vec.size());
}

/// Initialize the position and orientation of each pinhole camera model using
/// a least squares error transform to match the provided control points file.
/// This function overwrites the camera parameters in-place. It works
/// if at least three GCP are seen in no less than two images.
void transform_cameras_with_shared_gcp(
            boost::shared_ptr<ControlNetwork> const& cnet_ptr,
            std::vector<vw::CamPtr> & camera_models) {

  vw_out() << "Applying transform to cameras given several GCP shared among the images.\n";

  const ControlNetwork & cnet = *cnet_ptr.get(); // Helper alias

  // Verify that all cameras are pinhole
  for (size_t icam = 0; icam < camera_models.size(); icam++) {
    vw::PinholeModel * pincam
      = dynamic_cast<vw::PinholeModel*>(camera_models[icam].get());
    VW_ASSERT(pincam != NULL,
          vw::ArgumentErr() << "A pinhole camera expected.\n");
  }

  // Put the good ground control points in a vector.
  int num_cnet_points = cnet.size();
  std::vector<vw::Vector3> in_xyz, out_xyz;
  int num_gcp      = 0;
  int num_good_gcp = 0;
  for (int ipt = 0; ipt < num_cnet_points; ipt++) {

    if (cnet[ipt].type() != ControlPoint::GroundControlPoint)
      continue;

    num_gcp++;

    // Use triangulation to estimate the position of this control point using
    //   the current set of camera models.
    ControlPoint untrans_cp = cnet[ipt];
    double minimum_angle = 1.0e-3; // Likely this is too small for a good GCP, but better than 0.
    double forced_triangulation_distance = -1.0;
    double err = vw::triangulate_control_point(untrans_cp, camera_models,
                           minimum_angle, forced_triangulation_distance);

    if (untrans_cp.position() != Vector3() && cnet[ipt].position()  != Vector3() &&
        err >= 0) {
      // Store the computed and correct position of this point
      in_xyz.push_back(untrans_cp.position());
      out_xyz.push_back(cnet[ipt].position());
      num_good_gcp++; // Only count points that triangulate
    } else {
      vw_out() << "Discarding GCP that could not be triangulated: " << cnet[ipt] << ".\n";
    }
  } // End good GCP counting

  // Sanity check
  const int MIN_GCP_COUNT = 3;
  if (num_good_gcp < MIN_GCP_COUNT) {
    vw_out() << "Num GCP       = " << num_gcp      << std::endl;
    vw_out() << "Num valid GCP = " << num_good_gcp << std::endl;
    vw_throw(ArgumentErr()
          << "Not enough valid GCPs to apply a transform to the cameras. "
          << "You may need to use --transform-cameras-using-gcp.\n");
  }

  // Fit a 3D similarity transform from the triangulated GCP positions to the
  // GCP ground positions, robust to outlier GCP. A single blunder (such as a
  // lat-lon swap) can so corrupt a plain least-squares fit that every residual
  // is large and the blunder no longer stands out. So, instead, drop GCP one at
  // a time by leave-one-out: remove the GCP whose removal most reduces the fit
  // RMS error, but only when that drop reduces the error by a large factor,
  // which marks a true outlier rather than ordinary noise. Clean GCP sets are
  // left unchanged. The GCP count on this camera-init path is modest, so the
  // leave-one-out cost is not a concern.
  vw::Matrix3x3 rotation;
  vw::Vector3   translation;
  double        scale = 1.0;
  std::string transform_type = "similarity";
  std::vector<vw::Vector3> in_fit = in_xyz, out_fit = out_xyz;
  double outlier_factor = 20.0; // a drop must cut the RMS error by this factor
  while (true) {
    vw::find_3D_transform_no_outliers(in_fit, out_fit, rotation, translation,
                                      scale, transform_type);
    double full_rms = similarity_fit_rms(in_fit, out_fit, rotation, translation, scale);
    if (int(in_fit.size()) <= MIN_GCP_COUNT || full_rms <= 0.0)
      break;

    // Leave-one-out: find the GCP whose removal most reduces the fit RMS error
    int drop_idx = -1;
    double best_rms = full_rms;
    for (size_t i = 0; i < in_fit.size(); i++) {
      std::vector<vw::Vector3> in_sub, out_sub;
      for (size_t j = 0; j < in_fit.size(); j++) {
        if (j == i)
          continue;
        in_sub.push_back(in_fit[j]);
        out_sub.push_back(out_fit[j]);
      }
      vw::Matrix3x3 sub_rotation;
      vw::Vector3   sub_translation;
      double        sub_scale = 1.0;
      vw::find_3D_transform_no_outliers(in_sub, out_sub, sub_rotation,
                                        sub_translation, sub_scale, transform_type);
      double sub_rms = similarity_fit_rms(in_sub, out_sub, sub_rotation,
                                          sub_translation, sub_scale);
      if (sub_rms < best_rms) {
        best_rms = sub_rms;
        drop_idx = i;
      }
    }

    // Drop the worst GCP only when that clearly improves the fit, then refit
    if (drop_idx >= 0 && best_rms * outlier_factor < full_rms) {
      vw_out() << "Discarding a likely outlier GCP. The fit RMS error drops from "
               << full_rms << " m to " << best_rms << " m when it is removed.\n";
      in_fit.erase(in_fit.begin() + drop_idx);
      out_fit.erase(out_fit.begin() + drop_idx);
      continue;
    }
    break;
  }

  // Update the camera and point information with the new transform
  vw_out() << "Applying transform based on GCP:\n";
  vw_out() << "Rotation:    " << rotation    << "\n";
  vw_out() << "Translation: " << translation << "\n";
  vw_out() << "Scale:       " << scale       << "\n";
  apply_rigid_transform(rotation, translation, scale, camera_models, cnet_ptr);

  return;
} // End function transform_cameras_with_shared_gcp

/// Initialize the position and orientation of a pinhole camera model using
/// GCP. It invokes OpenCV's PnP functionality.
void init_camera_using_gcp(boost::shared_ptr<vw::ControlNetwork> const& cnet_ptr,
                           std::vector<vw::CamPtr> & camera_models) {

  // Sanity check
  if (camera_models.size() != 1)
    vw::vw_throw(vw::ArgumentErr()
                 << "Cannot initialize more than a camera at a time using GCP. "
                 << "Consider using --transform-cameras-with-shared-gcp or "
                 << "--transform-cameras-using-gcp.\n");

  vw_out() << "Initializing a Pinhole camera using GCP.\n";

  int icam = 0;
  vw::PinholeModel * pincam
    = dynamic_cast<vw::PinholeModel*>(camera_models[icam].get());
  VW_ASSERT(pincam != NULL, vw::ArgumentErr() << "A pinhole camera expected.\n");

  std::vector<vw::Vector2> pixel_observations;
  std::vector<vw::Vector3> ground_points;
  const ControlNetwork & cnet = *cnet_ptr.get(); // Helper alias
  for (int ipt = 0; ipt < cnet.size(); ipt++) {

    // Loop through all the ground control points only
    if (cnet[ipt].type() != ControlPoint::GroundControlPoint)
      continue;
    ground_points.push_back(cnet[ipt].position());

    int num_meas = 0;
    for (ControlPoint::const_iterator measure = cnet[ipt].begin();
         measure != cnet[ipt].end(); measure++) {

      int cam_it = measure->image_id();
      if (cam_it != 0)
        vw_throw(ArgumentErr() << "Error: Expecting GCP for a single camera.\n");

      Vector2 pixel(measure->position()[0], measure->position()[1]);
      num_meas++;
      if (num_meas > 1)
        vw::vw_throw(vw::ArgumentErr() << "Expecting a single camera pixel per gcp.\n");

      pixel_observations.push_back(pixel);
    }
  }

  // Update the camera pose with given observations and intrinsics
  asp::findCameraPose(ground_points, pixel_observations, *pincam);

  return;

} // End function init_camera_using_gcp

// Given an input pinhole camera and param changes, apply those, returning
// the new camera. Note that all intrinsic parameters are stored as multipliers
// in asp::BaState.
vw::PinholeModel transformedPinholeCamera(int camera_index,
                                          asp::BaState const& ba_state,
                                          vw::PinholeModel const& in_cam) {

  // Start by making a copy of the camera. Note that this does not make a copy of the
  // distortion params, as that's a pointer. So will have to make a copy of it further down.
  vw::PinholeModel out_cam = in_cam;

  double const* pos_pose_ptr   = ba_state.get_camera_ptr(camera_index);
  double const* center_ptr     = ba_state.get_intrinsic_center_ptr    (camera_index);
  double const* focus_ptr      = ba_state.get_intrinsic_focus_ptr     (camera_index);
  double const* distortion_ptr = ba_state.get_intrinsic_distortion_ptr(camera_index);

  // Update position and pose
  CameraAdjustment pos_pose_info(pos_pose_ptr);
  out_cam.set_camera_center(pos_pose_info.position());
  out_cam.set_camera_pose(pos_pose_info.pose());

  // Update the lens distortion parameters. The arrays hold the values
  // directly (additive). Note how we make a new copy of the distortion object.
  boost::shared_ptr<LensDistortion> distortion = out_cam.lens_distortion()->copy();
  vw::Vector<double> lens = distortion->distortion_parameters();
  for (size_t i = 0; i < lens.size(); i++)
    lens[i] = distortion_ptr[i];
  distortion->set_distortion_parameters(lens);
  out_cam.set_lens_distortion(distortion.get());

  // Update the center and focus
  out_cam.set_point_offset(Vector2(center_ptr[0], center_ptr[1]),
                           false); // do not update the internals yet
  double new_focus = focus_ptr[0];
  // At the last step, recompute the internals given the new values
  out_cam.set_focal_length(Vector2(new_focus,new_focus), true);

  return out_cam;
}

// Given an input optical bar camera and param changes, apply those, returning
// the new camera.
vw::OpticalBarModel
transformedOpticalBarCamera(int camera_index,
                            asp::BaState const& ba_state,
                            vw::OpticalBarModel const& in_cam) {

  // Start by making a copy of the camera
  vw::OpticalBarModel out_cam = in_cam;

  double const* pos_pose_ptr  = ba_state.get_camera_ptr(camera_index);
  double const* center_ptr    = ba_state.get_intrinsic_center_ptr    (camera_index);
  double const* focus_ptr     = ba_state.get_intrinsic_focus_ptr     (camera_index);
  double const* intrinsic_ptr = ba_state.get_intrinsic_distortion_ptr(camera_index);

  // Update position and pose
  CameraAdjustment pos_pose_info(pos_pose_ptr);
  out_cam.set_camera_center(pos_pose_info.position());
  out_cam.set_camera_pose  (pos_pose_info.pose    ());

  // Intrinsic parameters are stored as values (additive).

  // Update the other intrinsic parameters.
  out_cam.set_speed              (intrinsic_ptr[0]);
  out_cam.set_motion_compensation(intrinsic_ptr[1]);
  out_cam.set_scan_time          (intrinsic_ptr[2]);

  if (out_cam.get_have_velocity_vec()) {
    vw::Vector3 vel(intrinsic_ptr[3], intrinsic_ptr[4], intrinsic_ptr[5]);
    vw::Vector3 final_pose(intrinsic_ptr[6], intrinsic_ptr[7], intrinsic_ptr[8]);
    out_cam.set_velocity(vel);
    out_cam.set_final_pose(final_pose);
  }

  // Update the center and focus
  out_cam.set_optical_center(Vector2(center_ptr[0], center_ptr[1]));
  double new_focus = focus_ptr[0];
  out_cam.set_focal_length(new_focus);

  return out_cam;
}

// Given an input CSM camera, intrinsic and extrinsic param changes, apply
// those, returning the new camera.
boost::shared_ptr<asp::CsmModel> transformedCsmCamera(int camera_index,
                                                      asp::BaState const& ba_state,
                                                      asp::CsmModel const& in_cam) {
  // Get the latest version of the camera parameters
  double const* pos_pose_ptr  = ba_state.get_camera_ptr(camera_index);
  double const* center_ptr    = ba_state.get_intrinsic_center_ptr    (camera_index);
  double const* focus_ptr     = ba_state.get_intrinsic_focus_ptr     (camera_index);
  double const* dist_ptr      = ba_state.get_intrinsic_distortion_ptr(camera_index);

  // Read the position and pose
  CameraAdjustment correction(pos_pose_ptr);

  // All intrinsic parameters are stored as values (additive).
  vw::Vector2 optical_center = in_cam.optical_center();
  double focal_length        = in_cam.focal_length();
  optical_center[0] = center_ptr[0];
  optical_center[1] = center_ptr[1];
  focal_length      = focus_ptr [0];

  // Update the lens distortion parameters. The arrays hold the values
  // directly, so a zero-seeded coefficient can be optimized.
  std::vector<double> distortion = in_cam.distortion();
  for (size_t i = 0; i < distortion.size(); i++)
    distortion[i] = dist_ptr[i];

  // Duplicate the input camera model
  boost::shared_ptr<asp::CsmModel> copy;
  in_cam.deep_copy(copy);

  // Update the intrinsics of the copied model
  copy->set_optical_center(optical_center);
  copy->set_focal_length(focal_length);
  copy->set_distortion(distortion);

  // Form the adjusted camera having the updated position and pose
  AdjustedCameraModel adj_cam(copy, correction.position(), correction.pose());

  // Apply the adjustment to the camera 
  vw::Matrix4x4 ecef_transform = adj_cam.ecef_transform();
  copy->applyTransform(ecef_transform);

  // TODO(oalexan1): Test if adj cam and cam with applied adjustment return same
  // results

  return copy;
}

// Find stats of propagated errors
void propagatedErrorStats(size_t left_cam_index, size_t right_cam_index,
                          vw::CameraModel const * left_cam,
                          vw::CameraModel const * right_cam,
                          std::vector<vw::InterestPoint> const& left_ip,
                          std::vector<vw::InterestPoint> const& right_ip,
                          double stddev1, double stddev2,
                          vw::Datum const& datum,
                          vw::BathyData const& bathy_data,
                          // Outputs
                          std::vector<vw::Vector<double, 5>> & uncertPoints,
                          asp::HorizVertErrorStats & stats) {

  // Create a stereo model, to be used for triangulation
  double angle_tol = vw::StereoModel::robust_1_minus_cos
                        (asp::stereo_settings().min_triangulation_angle*M_PI/180);
  vw::StereoModel stereo_model(left_cam, right_cam,
                               angle_tol);

  // If bathymetry is modeled, set up a bathy stereo model with the two planes
  // for this pair, so underwater points are triangulated with bent rays and the
  // covariance is propagated through the same bending (:numref:`error_propagation`).
  bool has_bathy = !bathy_data.bathy_planes.empty() && !bathy_data.bathy_masks.empty();
  std::vector<vw::BathyPlane> plane_pair;
  vw::BathyStereoModel bathy_model(left_cam, right_cam, angle_tol);
  if (has_bathy) {
    plane_pair.push_back(bathy_data.bathy_planes[left_cam_index]);
    plane_pair.push_back(bathy_data.bathy_planes[right_cam_index]);
    bathy_model.set_bathy(bathy_data.refraction_index, plane_pair);
  }

  // Create space for horiz and vert vectors of size num_ip
  int num_ip = left_ip.size();
  std::vector<double> horiz_errors, vert_errors;

  // Find the triangulated point and propagate the errors
  for (int ip_it = 0; ip_it < num_ip; ip_it++) {
    // Compute the error in the horizontal and vertical directions
    vw::Vector2 left_pix(left_ip[ip_it].x, left_ip[ip_it].y);
    vw::Vector2 right_pix(right_ip[ip_it].x, right_ip[ip_it].y);

    Vector3 triVec(0, 0, 0), errorVec(0, 0, 0);
    vw::Vector2 outStdev;
    bool do_bend = false;
    try {
      if (has_bathy) {
        // Bend when both pixels are under water. did_bathy is the actual
        // outcome and is frozen into the covariance finite differences.
        bool do_bathy = vw::areMasked(bathy_data.bathy_masks[left_cam_index],
                                      bathy_data.bathy_masks[right_cam_index],
                                      left_pix, right_pix);
        std::vector<vw::Vector2> pixVec = {left_pix, right_pix};
        bool did_bathy = false;
        triVec = bathy_model(pixVec, errorVec, do_bathy, did_bathy);
        do_bend = did_bathy;
      } else {
        triVec = stereo_model(left_pix, right_pix, errorVec);
      }
      outStdev = asp::propagateCovariance(triVec, datum,
                                          stddev1, stddev2,
                                          left_cam, right_cam,
                                          left_pix, right_pix,
                                          do_bend, bathy_data.refraction_index,
                                          plane_pair);
    } catch (std::exception const& e) {
      errorVec = Vector3(0, 0, 0);
    }

    if (errorVec == Vector3(0, 0, 0))
      continue; // this can happen either because triangulation failed or an exception

    horiz_errors.push_back(outStdev[0]);
    vert_errors.push_back(outStdev[1]);

    // Save the per-match uncertainty with its ground location, so it can be
    // gridded or plotted (lon, lat, height, horiz stddev, vert stddev).
    vw::Vector3 llh = datum.cartesian_to_geodetic(triVec);
    vw::Vector<double, 5> pt;
    pt[0] = llh[0]; pt[1] = llh[1]; pt[2] = llh[2];
    pt[3] = outStdev[0]; pt[4] = outStdev[1];
    uncertPoints.push_back(pt);
  }

  // Initialize the output
  stats = asp::HorizVertErrorStats();
  stats.left_cam_index  = left_cam_index;
  stats.right_cam_index = right_cam_index;

  if (!horiz_errors.empty()) {
    stats.num_errors = horiz_errors.size();
    stats.horiz_error_mean = vw::mean(horiz_errors);
    stats.vert_error_mean = vw::mean(vert_errors);

    if (horiz_errors.size() > 1) {
      // This divides by num - 1
      stats.horiz_error_stddev
        = vw::standard_deviation(horiz_errors, stats.horiz_error_mean);
      stats.vert_error_stddev
        = vw::standard_deviation(vert_errors, stats.vert_error_mean);
    }

    // Leave this for the last
    stats.horiz_error_median = vw::median_in_place(horiz_errors);
    stats.vert_error_median = vw::median_in_place(vert_errors);
  }

  return;
} // End function propagatedErrorStats

/// Attempt to automatically create the overlap list file estimated
///  footprints for each of the input images.
/// - Currently this only supports cameras with Worldview style XML files.
void auto_build_overlap_list(asp::BaBaseOptions &opt, double lonlat_buffer) {

  typedef std::pair<std::string, std::string> StringPair;

  const size_t num_images = opt.camera_files.size();
  opt.overlap_list.clear();

  vw_out() << "Attempting to automatically estimate image overlaps...\n";
  int  num_overlaps = 0;
  bool read_success = false;

  // Loop through all image pairs
  for (size_t i = 0; i < num_images - 1; i++) {

    // Try to get the lonlat bounds for this image
    std::vector<vw::Vector2> pixel_corners_i, lonlat_corners_i;
    try {
      read_success = asp::read_WV_XML_corners(opt.camera_files[i], pixel_corners_i,
                                              lonlat_corners_i);
    } catch(...) {
      read_success = false;
    }
    if (!read_success) {
      vw_throw(ArgumentErr() << "Unable to get corner estimate from file: "
                              << opt.camera_files[i] << ".\n");
    }

    vw::BBox2 bbox_i; // Convert to BBox
    for (size_t p = 0; p < lonlat_corners_i.size(); p++)
      bbox_i.grow(lonlat_corners_i[p]);
    bbox_i.expand(lonlat_buffer); // Only expand this bounding box by the buffer.

    for (size_t j = i+1; j < num_images; j++) {

      std::vector<vw::Vector2> pixel_corners_j, lonlat_corners_j;
      try {
        read_success = asp::read_WV_XML_corners(opt.camera_files[j], pixel_corners_j,
                                                lonlat_corners_j);
      } catch(...) {
        read_success = false;
      }
      if (!read_success) {
        vw_throw(ArgumentErr() << "Unable to get corner estimate from file: "
                                << opt.camera_files[j] << ".\n");
      }

      vw::BBox2 bbox_j; // Convert to BBox
      for (size_t p = 0;  p < lonlat_corners_j.size(); p++)
        bbox_j.grow(lonlat_corners_j[p]);

      // Record the files if the bboxes overlap
      // - TODO: Use polygon intersection instead of bounding boxes!
      if (bbox_i.intersects(bbox_j)) {
        vw_out() << "Predicted overlap between images " << opt.image_files[i]
                 << " and " << opt.image_files[j] << "\n";
        opt.overlap_list.insert(StringPair(opt.image_files[i], opt.image_files[j]));
        opt.overlap_list.insert(StringPair(opt.image_files[j], opt.image_files[i]));
        ++num_overlaps;
      }
    } // End inner loop through cameras
  } // End outer loop through cameras

  if (num_overlaps == 0)
    vw_throw(ArgumentErr() << "Failed to automatically detect any overlapping images!");

  vw_out() << "Will try to match at " << num_overlaps << " detected overlaps\n.";
} // End function auto_build_overlap_list

// Parse data needed for error propagation. Note that horizontal_stddevs
// comes from the user, or is otherwise populated from cameras.
void setup_error_propagation(std::string const& session_name,
                             double horizontal_stddev,
                             std::vector<vw::CamPtr> const& cameras,
                             vw::Vector<double> & horizontal_stddev_vec) {

  // Initialize the output
  horizontal_stddev_vec.set_size(cameras.size());
  horizontal_stddev_vec.set_all(horizontal_stddev);

  bool message_printed = false;
  if (horizontal_stddev == 0.0) {
    // Read from cameras
    for (size_t icam = 0; icam < cameras.size(); icam++)
      horizontal_stddev_vec[icam]
        = asp::horizontalStDevFromCamera(cameras[icam], message_printed);
  }

  asp::horizontalStdDevCheck(horizontal_stddev_vec, session_name);
}

// Find the cameras with the latest adjustments. Note that we do not modify
// opt.camera_models, but make copies as needed. The optimizations (adjustments)
// may be inline, or external.

void estimateGsdPerTriPoint(std::vector<std::string> const& images,
                            std::vector<vw::CamPtr>  const& cameras,
                            asp::CRN                const& crn,
                            asp::BaState             const& ba_state,
                            // Output
                            std::vector<double>     & gsds) {

  // Sanity checks
  if (crn.size() != images.size())
    vw_throw(ArgumentErr() << "Expecting the same number of images and crn points.\n");
  if (crn.size() != cameras.size())
    vw_throw(ArgumentErr() << "Expecting the same number of images and cameras.\n");
  if (crn.size() != ba_state.num_cameras())
    vw_throw(ArgumentErr() << "Expecting the same number of images and cameras.\n");

  // Image bboxes
  std::vector<vw::BBox2> bboxes;
  for (size_t i = 0; i < images.size(); i++) {
    vw::DiskImageView<float> img(images[i]);
    bboxes.push_back(bounding_box(img));
  }

  int num_cameras = ba_state.num_cameras();
  int num_points  = ba_state.num_points();

  // Initialize all gsd to 0
  gsds.resize(num_points, 0.0);
  std::vector<int> count(num_points, 0);

  for (int icam = 0; icam < num_cameras; icam++) { // Camera loop
    for (auto fiter = crn[icam].begin(); fiter != crn[icam].end(); fiter++) { // IP loop

      // The index of the 3D point this IP is for.
      int ipt = (**fiter).m_point_id;

      VW_ASSERT(int(icam) < num_cameras,
                ArgumentErr() << "Out of bounds in the number of cameras.");
      VW_ASSERT(int(ipt)  < num_points,
                ArgumentErr() << "Out of bounds in the number of points.");

      if (ba_state.get_point_outlier(ipt))
        continue; // skip outliers

      vw::Vector2 pix = (**fiter).m_location;

      double const* point = ba_state.get_point_ptr(ipt);
      Vector3 xyz(point[0], point[1], point[2]);

      // Estimate the GSD at the given pixel given an estimate of the ground point
      double gsd = 0.0;
      try {
        gsd = vw::estimatedGSD(cameras[icam].get(), bboxes[icam], pix, xyz);
      } catch (...) {
        continue;
      }
      gsds[ipt] += gsd;
      count[ipt]++;
    }
  }

  // Find the average gsd
  for (int ipt = 0; ipt < num_points; ipt++) {
    if (count[ipt] > 0)
      gsds[ipt] /= count[ipt];
  }

  return;
}

// Find the average for the gsd for all pixels whose rays intersect at the given
// triangulated point. This is used in jitter solving. Note that tri_points_vec
// may have anchor points at the end, but we don't get to those.
void estimateGsdPerTriPoint(std::vector<std::string> const& images,
                            std::vector<vw::CamPtr>  const& cameras,
                            asp::CRN                const& crn,
                            std::set<int>            const& outliers,
                            std::vector<double>      const& tri_points_vec,
                            // Output
                            std::vector<double>     & gsds) {

  // Sanity checks
  if (crn.size() != images.size())
    vw_throw(ArgumentErr() << "Expecting the same number of images and crn points.\n");
  if (crn.size() != cameras.size())
    vw_throw(ArgumentErr() << "Expecting the same number of images and cameras.\n");

  // Image bboxes
  std::vector<vw::BBox2> bboxes;
  for (size_t i = 0; i < images.size(); i++) {
    vw::DiskImageView<float> img(images[i]);
    bboxes.push_back(bounding_box(img));
  }

  int num_cameras = cameras.size();
  int num_points  = tri_points_vec.size()/3;

  // Initialize all gsd to 0
  gsds.resize(num_points, 0.0);
  std::vector<int> count(num_points, 0);

  for (int icam = 0; icam < num_cameras; icam++) { // Camera loop
    for (auto fiter = crn[icam].begin(); fiter != crn[icam].end(); fiter++) { // IP loop

      // The index of the 3D point this IP is for.
      int ipt = (**fiter).m_point_id;

      VW_ASSERT(int(icam) < num_cameras,
                ArgumentErr() << "Out of bounds in the number of cameras.");
      VW_ASSERT(int(ipt)  < num_points,
                ArgumentErr() << "Out of bounds in the number of points.");

      if (outliers.find(ipt) != outliers.end())
        continue; // Skip outliers

      vw::Vector2 pix = (**fiter).m_location;

      double const* point = &tri_points_vec[3*ipt];
      Vector3 xyz(point[0], point[1], point[2]);

      // Estimate the GSD at the given pixel given an estimate of the ground point
      double gsd = 0.0;
      try {
        gsd = vw::estimatedGSD(cameras[icam].get(), bboxes[icam], pix, xyz);
      } catch (...) {
        continue;
      }

      gsds[ipt] += gsd;
      count[ipt]++;
    }
  }

  // Find the average gsd
  for (int ipt = 0; ipt < num_points; ipt++) {
    if (count[ipt] > 0)
      gsds[ipt] /= count[ipt];
  }

  return;
}

// This function returns only one camera center per camera. See below
// for linescan.
void calcCameraCenters(std::vector<vw::CamPtr>  const& cams,
                       std::vector<vw::Vector3>      & cam_positions) {

  cam_positions.resize(cams.size());
  for (size_t icam = 0; icam < cams.size(); icam++) {
    vw::Vector3 ctr = cams[icam]->camera_center(vw::Vector2());
    cam_positions[icam] = ctr;
  }
}

// This function returns all camera center samples for linescan cameras. These
// cameras may have an external adjustment. Otherwise it returns only the adjusted
// camera center at pixel (0,0).
void calcCameraCenters(std::string const& stereo_session,
                       std::vector<vw::CamPtr> const& camera_models,
                       std::vector<std::vector<vw::Vector3>> & cam_positions) {

  cam_positions.resize(camera_models.size());
  for (size_t icam = 0; icam < camera_models.size(); icam++) {

    // Fetch the CSM model without any adjustment
    asp::CsmModel * csm_cam = NULL;
    try {
      csm_cam
        = asp::csm_model(vw::unadjusted_model(camera_models[icam]));
    } catch(...) {}
    if (csm_cam == NULL) {
      // Not a CSM camera, pull directly the camera center
      vw::Vector3 ctr = camera_models[icam]->camera_center(vw::Vector2());
      cam_positions[icam].push_back(ctr);
      continue;
    }
    csm::RasterGM const* gm = csm_cam->m_gm_model.get();
    UsgsAstroLsSensorModel const* ls_model
      = dynamic_cast<UsgsAstroLsSensorModel const*>(gm);
    if (ls_model == NULL) {
      // Not a linescan camera. Return, as before the adjusted camera center
      vw::Vector3 ctr = camera_models[icam]->camera_center(vw::Vector2());
      cam_positions[icam].push_back(ctr);
      continue;
    }

    // Handle the case when the CSM camera has an external adjustment. In that case
    // need to apply it before can find the camera positions, and update ls_model.
    vw::AdjustedCameraModel *adj_cam
      = dynamic_cast<vw::AdjustedCameraModel*>(camera_models[icam].get());
    asp::CsmModel local_csm; // Ensure it does not go out of scope
    if (adj_cam != NULL) {
      vw::Matrix4x4 ecef_transform = adj_cam->ecef_transform();
      csm_cam->deep_copy(local_csm);
      local_csm.applyTransform(ecef_transform);
      csm::RasterGM const* gm = local_csm.m_gm_model.get();
      ls_model = dynamic_cast<UsgsAstroLsSensorModel const*>(gm);
      if (ls_model == NULL)
        vw::vw_throw(vw::ArgumentErr() << "Expecting a linescan camera.\n");
    }

    // Get the number of positions
    int numPos = ls_model->m_positions.size() / NUM_XYZ_PARAMS;

    // The positions are in a single vector, ls_model->m_positions
    // Append them to cam_positions[icam]
    for (int i = 0; i < numPos; i++) {
      int j = i * NUM_XYZ_PARAMS;
      double x = ls_model->m_positions[j+0];
      double y = ls_model->m_positions[j+1];
      double z = ls_model->m_positions[j+2];
      cam_positions[icam].push_back(vw::Vector3(x, y, z));
    }
  }

  return;
}

void checkGcpRadius(vw::Datum const& datum,
                    vw::ControlNetwork const& cnet) {

  int num_points = cnet.size();
  for (int ipt = 0; ipt < num_points; ipt++) {
    if (cnet[ipt].type() != ControlPoint::GroundControlPoint)
      continue;

    vw::Vector3 observation = cnet[ipt].position();
    double thresh = 2e+5; // 200 km
    if (std::abs(norm_2(observation) - datum.semi_major_axis()) > thresh ||
        std::abs(norm_2(observation) - datum.semi_minor_axis()) > thresh)
      vw_out(vw::WarningMessage) << "GCP " << ipt
        << " has ECEF radius differing from the datum by more than "
        << thresh << " m. Check your GCPs and datum.\n";
  }

  return;
}

// Some logic for camera position uncertainty, used in bundle_adjust and jitter_solve
void handleCameraPositionUncertainty(asp::BaBaseOptions & opt, bool have_datum) {

  // Create map from image name to index
  std::map<std::string, int> image_name_to_index;
  for (int i = 0; i < (int)opt.image_files.size(); i++)
    image_name_to_index[opt.image_files[i]] = i;

  // Resize opt.camera_position_uncertainty to the number of images
  opt.camera_position_uncertainty.resize(opt.image_files.size(), vw::Vector2(0, 0));

  // Handle the case when uncertainty is two values separated by a comma
  std::string sep = ",";
  std::vector<double> vals = vw::str_to_std_vec(opt.camera_position_uncertainty_str, sep);
  if (vals.size() == 2) {
    for (int i = 0; i < (int)opt.image_files.size(); i++)
      opt.camera_position_uncertainty[i] = vw::Vector2(vals[0], vals[1]);
  } else {
    // Read the uncertainties per image from file
    std::string image_name;
    double horiz = 0, vert = 0;
    std::ifstream ifs(opt.camera_position_uncertainty_str.c_str());
    if (!ifs.good())
      vw_throw(ArgumentErr() << "Cannot read camera position uncertainty from: "
                << opt.camera_position_uncertainty_str << ".\n");
    while (ifs >> image_name >> horiz >> vert) {
      auto it = image_name_to_index.find(image_name);
      if (it == image_name_to_index.end())
        continue; // skip images having uncertainty that are not in the image list
      int index = it->second;
      opt.camera_position_uncertainty[index] = Vector2(horiz, vert);
    }
  }

  // This constraint requires the solver to work harder to converge.
  opt.parameter_tolerance = std::min(opt.parameter_tolerance, 1e-10);

  // Ensure each horizontal and vertical uncertainty is positive
  for (int i = 0; i < (int)opt.image_files.size(); i++) {
    if (opt.camera_position_uncertainty[i][0] <= 0 ||
        opt.camera_position_uncertainty[i][1] <= 0)
      vw::vw_throw(vw::ArgumentErr()
                  << "The camera uncertainty for each image must be set and be positive.\n");
  }

  // The power must be positive
  if (opt.camera_position_uncertainty_power <= 0.0)
    vw::vw_throw(vw::ArgumentErr()
                << "The value of --camera-position-uncertainty-power must be positive.\n");

  // When there is camera position uncertainty, the other camera weights must be 0.    
  if (opt.camera_position_weight > 0) {
    vw::vw_out() << "Setting --camera-position-weight to 0 as "
                  << "--camera-position-uncertainty is positive.\n";
    opt.camera_position_weight = 0;
  }

  if (opt.camera_weight > 0) {
    vw::vw_out() << "Setting --camera-weight to 0 as --camera-position-uncertainty "
                  << "is positive.\n";
    opt.camera_weight = 0;
  }

  if (!have_datum)
    vw::vw_throw(vw::ArgumentErr()
            << "Cannot use camera uncertainties without a datum. Set --datum.\n");
}

} // end namespace asp
