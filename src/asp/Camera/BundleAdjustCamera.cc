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
#include <asp/Camera/BundleAdjustParamMap.h>
#include <asp/Camera/CameraResectioning.h>
#include <asp/Camera/CsmModel.h>
#include <asp/Camera/Covariance.h>
#include <asp/Camera/LinescanUtils.h>
#include <asp/Camera/RPC_XML.h>
#include <asp/Camera/CameraErrorPropagation.h>
#include <asp/Core/IpMatchingAlgs.h>
#include <asp/Core/StereoSettings.h>
#include <asp/Core/BaseCameraUtils.h>
#include <asp/Core/ImageUtils.h>
#include <asp/Core/FileUtils.h>

#include <vw/Math/Statistics.h>
#include <vw/BundleAdjustment/CameraRelation.h>
#include <vw/BundleAdjustment/ControlNetwork.h>
#include <vw/BundleAdjustment/ControlNetworkLoader.h>
#include <vw/Cartography/CameraBBox.h>
#include <vw/Cartography/BathyStereoModel.h>
#include <vw/Cartography/BathyData.h>
#include <vw/InterestPoint/MatcherIO.h>
#include <vw/Stereo/StereoModel.h>
#include <vw/Math/Geometry.h>
#include <vw/Math/BBox.h>
#include <vw/Camera/PinholeModel.h>
#include <vw/Camera/OpticalBarModel.h>
#include <vw/Camera/CameraUtilities.h>
#include <vw/Camera/CameraImage.h>
#include <vw/Camera/LensDistortion.h>
#include <vw/Image/PixelMask.h>
#include <vw/Image/ImageChannels.h>
#include <vw/Core/Stopwatch.h>
#include <vw/Core/StringUtils.h>

#include <usgscsm/UsgsAstroLsSensorModel.h>
#include <boost/algorithm/string.hpp>

#include <cmath>
#include <string>
#include <fstream>
#include <vector>
#include <algorithm>

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

  // Update the lens distortion parameters. Note how we make a new copy of the
  // distortion object.
  boost::shared_ptr<LensDistortion> distortion = out_cam.lens_distortion()->copy();
  vw::Vector<double> lens = distortion->distortion_parameters();
  for (size_t i = 0; i < lens.size(); i++)
    lens[i] *= distortion_ptr[i];
  distortion->set_distortion_parameters(lens);
  out_cam.set_lens_distortion(distortion.get());

  // Update the center and focus
  Vector2 old_center = out_cam.point_offset();
  Vector2 old_focus  = out_cam.focal_length();
  out_cam.set_point_offset(Vector2(center_ptr[0]*old_center[0],
                                  center_ptr[1]*old_center[1]),
                           false); // do not update the internals yet
  double new_focus = old_focus[0]*focus_ptr[0];
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

  // All intrinsic parameters are stored as multipliers!

  // Update the other intrinsic parameters.
  out_cam.set_speed              (out_cam.get_speed()*intrinsic_ptr[0]);
  out_cam.set_motion_compensation(out_cam.get_motion_compensation()*intrinsic_ptr[1]);
  out_cam.set_scan_time          (out_cam.get_scan_time()*intrinsic_ptr[2]);

  if (out_cam.get_have_velocity_vec()) {
    vw::Vector3 vel = out_cam.get_velocity();
    vw::Vector3 final_pose = out_cam.get_final_pose();
    vel[0] *= intrinsic_ptr[3];
    vel[1] *= intrinsic_ptr[4];
    vel[2] *= intrinsic_ptr[5];
    final_pose[0] *= intrinsic_ptr[6];
    final_pose[1] *= intrinsic_ptr[7];
    final_pose[2] *= intrinsic_ptr[8];
    out_cam.set_velocity(vel);
    out_cam.set_final_pose(final_pose);
  }

  // Update the center and focus
  Vector2 old_center = out_cam.get_optical_center();
  float   old_focus  = out_cam.get_focal_length();
  out_cam.set_optical_center(Vector2(center_ptr[0]*old_center[0],
                                    center_ptr[1]*old_center[1]));
  double new_focus = old_focus*focus_ptr[0];
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

  // All intrinsic parameters are stored as multipliers
  vw::Vector2 optical_center = in_cam.optical_center();
  double focal_length        = in_cam.focal_length();
  optical_center[0] = center_ptr[0] * optical_center[0];
  optical_center[1] = center_ptr[1] * optical_center[1];
  focal_length      = focus_ptr [0] * focal_length;

  // Update the lens distortion parameters in the new camera.
  // - These values are also optimized as scale factors.
  std::vector<double> distortion = in_cam.distortion();
  for (size_t i = 0; i < distortion.size(); i++) {

    // Ensure this approach does not fail when the input distortion is 0.
    // TODO(oalexan1): This is not enough, however. The user must choose
    // to manually enter some small fake distortion, maybe on the order
    // of 1e-8. It is hard to tell what numbers to put below. They should
    // not be as small as 1e-16 but sometimes 1e-8 (for higher order polynomial
    // coefficients) may be too much.
    if (distortion[i] == 0.0)
      distortion[i] = 1e-16;

    distortion[i] = dist_ptr[i] * distortion[i];
  }

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

// Mapproject interest points onto a DEM and find the norm of their
// disagreement in meters. It is assumed that dem_georef
// was created by bilinear interpolation. The cameras must be with
// the latest adjustments applied to them.
void calcPairMapprojOffsets(int left_cam_index, int right_cam_index,
                            std::vector<vw::CamPtr>            const& optimized_cams,
                            std::vector<vw::InterestPoint> const& left_ip,
                            std::vector<vw::InterestPoint> const& right_ip,
                            vw::GeoReference      const& dem_georef,
                            vw::ImageViewRef<vw::PixelMask<double>> const& interp_dem,
                            // Will append below
                            std::vector<vw::Vector<float, 4>>       & mapprojPoints,
                            std::vector<float>                      & mapprojOffsets) {

  // Wipe mapprojOffsets
  mapprojOffsets.clear();

  // Will append to mapprojPoints, so don't wipe it

  for (size_t ip_it = 0; ip_it < left_ip.size(); ip_it++) {

    bool treat_nodata_as_zero = false;
    bool has_intersection = false;
    double height_error_tol = 0.001; // 1 mm should be enough
    Vector3 xyz_guess;

    Vector2 left_pix(left_ip[ip_it].x, left_ip[ip_it].y);
    Vector3 left_dem_xyz = vw::camera_pixel_to_dem_xyz
      (optimized_cams[left_cam_index]->camera_center(left_pix),
       optimized_cams[left_cam_index]->pixel_to_vector(left_pix),
       vw::pixel_cast<vw::PixelMask<float>>(interp_dem), dem_georef,
       treat_nodata_as_zero, has_intersection,
       height_error_tol, xyz_guess);
    if (!has_intersection)
      continue;

    // Do the same for right. Use left pixel as initial guess
    xyz_guess = left_dem_xyz;
    Vector2 right_pix(right_ip[ip_it].x, right_ip[ip_it].y);
    Vector3 right_dem_xyz = vw::camera_pixel_to_dem_xyz
      (optimized_cams[right_cam_index]->camera_center(right_pix),
       optimized_cams[right_cam_index]->pixel_to_vector(right_pix),
       vw::pixel_cast<vw::PixelMask<float>>(interp_dem), dem_georef,
       treat_nodata_as_zero, has_intersection,
       height_error_tol, xyz_guess);
    if (!has_intersection)
      continue;

    Vector3 mid_pt = (left_dem_xyz + right_dem_xyz)/2.0;
    double dist = norm_2(left_dem_xyz - right_dem_xyz);

    // Keep in the same structure both the midpoint between these two
    // mapprojected ip, as lon-lat,height, and their distance, as
    // later the bookkeeping of mapprojOffsets will be different.
    // Float precision is enough, and will save on memory.
    Vector<float, 4> point;
    subvector(point, 0, 3) = dem_georef.datum().cartesian_to_geodetic(mid_pt);
    point[3] = dist;

    mapprojPoints.push_back(point);
    mapprojOffsets.push_back(dist);
  }

  return;
}

// This is called either with original or inlier ip
void processMatchPair(size_t left_index, size_t right_index,
                      std::vector<vw::InterestPoint> const& left_ip,
                      std::vector<vw::InterestPoint> const& right_ip,
                      std::vector<vw::CamPtr> const& optimized_cams,
                      vw::GeoReference const& mapproj_dem_georef,
                      vw::ImageViewRef<vw::PixelMask<double>> const& interp_mapproj_dem,
                      vw::Datum const& datum,
                      bool save_mapproj_match_points_offsets,
                      bool propagate_errors,
                      vw::Vector<double> const& horizontal_stddev_vec,
                      vw::BathyData const& bathy_data,
                      // Outputs
                      // Will append to entities below
                      std::vector<asp::MatchPairStats>  & convAngles,
                      std::vector<vw::Vector<float, 4>> & mapprojPoints,
                      std::vector<asp::MatchPairStats> & mapprojOffsets,
                      std::vector<std::vector<float>>  & mapprojOffsetsPerCam,
                      std::vector<asp::HorizVertErrorStats> & horizVertErrors,
                      std::vector<vw::Vector<double, 5>> & uncertPoints) {

  convAngles.push_back(asp::MatchPairStats()); // add an element, then populate it
  std::vector<double> sorted_angles;
  asp::convergence_angles(optimized_cams[left_index].get(), optimized_cams[right_index].get(),
                          left_ip, right_ip, sorted_angles);
  convAngles.back().populate(left_index, right_index, sorted_angles);

  if (save_mapproj_match_points_offsets) {
    std::vector<float> localMapprojOffsets;
    asp::calcPairMapprojOffsets(left_index, right_index,
                                optimized_cams,
                                left_ip, right_ip,
                                mapproj_dem_georef, interp_mapproj_dem,
                                mapprojPoints, // will append here
                                localMapprojOffsets);
    mapprojOffsets.push_back(asp::MatchPairStats()); // add an elem, then populate it
    mapprojOffsets.back().populate(left_index, right_index, localMapprojOffsets);
    for (size_t map_it = 0; map_it < localMapprojOffsets.size(); map_it++) {
      mapprojOffsetsPerCam[left_index].push_back(localMapprojOffsets[map_it]);
      mapprojOffsetsPerCam[right_index].push_back(localMapprojOffsets[map_it]);
    }
  }

  if (propagate_errors) {
    // Ensure her that proper values are passed for the input std devs
    horizVertErrors.push_back(asp::HorizVertErrorStats()); // add an elem, then populate it
    asp::propagatedErrorStats(left_index, right_index,
                              optimized_cams[left_index].get(),
                              optimized_cams[right_index].get(),
                              left_ip, right_ip,
                              horizontal_stddev_vec[left_index],
                              horizontal_stddev_vec[right_index],
                              datum,
                              bathy_data,
                              uncertPoints,
                              horizVertErrors.back());
  }

  return;
}

// Calculate convergence angles. Remove the outliers flagged earlier,
// if remove_outliers is true. Compute offsets of mapprojected matches,
// if a DEM is given. These are done together as they rely on
// reloading interest point matches, which is expensive so the matches
// are used for both operations.
void matchFilesProcessing(vw::ControlNetwork       const& cnet,
                          asp::BaBaseOptions           const& opt,
                          std::vector<vw::CamPtr>      const& optimized_cams,
                          bool                                remove_outliers,
                          std::set<int>                const& outliers,
                          std::set<std::pair<int, int>> const& outlier_obs,
                          std::string                  const& mapproj_dem,
                          bool                                propagate_errors,
                          vw::Vector<double>           const& horizontal_stddev_vec,
                          bool                                save_clean_matches,
                          std::map<std::pair<int, int>, std::string> const& match_files,
                          bool                                matches_as_txt) {

  vw_out() << "Creating reports.\n";

  std::vector<vw::Vector<float, 4>> mapprojPoints; // all points, not just stats
  std::vector<asp::MatchPairStats> convAngles, mapprojOffsets;
  std::vector<std::vector<float>> mapprojOffsetsPerCam;
  std::vector<asp::HorizVertErrorStats> horizVertErrors;
  std::vector<vw::Vector<double, 5>> uncertPoints; // per-match propagated errors

  // Wipe the outputs
  mapprojPoints.clear();
  convAngles.clear();
  mapprojOffsets.clear();
  mapprojOffsetsPerCam.clear();
  horizVertErrors.clear();

  bool save_mapproj_match_points_offsets = (!mapproj_dem.empty());
  vw::GeoReference mapproj_dem_georef;
  ImageViewRef<PixelMask<double>> interp_mapproj_dem;
  if (save_mapproj_match_points_offsets)
    asp::create_interp_dem(mapproj_dem, mapproj_dem_georef, interp_mapproj_dem);

  int num_cameras = opt.image_files.size();
  mapprojOffsetsPerCam.resize(num_cameras);

  // Iterate over the control network, and, for each inlier pair of matches,
  // remember what pair it is from. Needed only if there is outlier filtering
  // or matches were read from an isis cnet.
  // TODO(oalexan1): This uses a lot of memory. Need to keep just indices,
  // somehow, not quadruplets of floats.
  // TODO(oalexan1): Make this into a function.
  typedef std::tuple<float, float, float, float> Quadruplet;
  // A per-image index: for each image, the (control point index, local measure
  // index) of every surviving measure in it, sorted by control point index. This
  // is linear in the number of measures. A track seen in k images used to add
  // k*(k-1) per-pair entries (quadratic in the overlap, a large memory cost).
  // Instead, a single image pair's matches are derived on demand by intersecting
  // the two images' point lists (makePairQuads below), so nothing quadratic is
  // stored. The positions and scales are read from the control measures on demand.
  std::map<int, std::vector<std::pair<int, int>>> image_index;
  // The set of image pairs sharing at least one surviving measure. Needed only to
  // enumerate match files when there are no input match files (ISIS cnet or nvm).
  std::set<std::pair<int, int>> match_pairs;
  bool need_match_pairs = (!opt.isis_cnet.empty() || !opt.nvm.empty());
  if (remove_outliers || need_match_pairs) {
    for (int ipt = 0; ipt < cnet.size(); ipt++) {
      // Skip outliers
      if (outliers.find(ipt) != outliers.end())
        continue;
      // Skip gcp
      if (cnet[ipt].type() == ControlPoint::GroundControlPoint)
        continue;

      int num_meas = cnet[ipt].size();
      // Record the surviving measures of this point, by image.
      for (int m = 0; m < num_meas; m++) {
        int img = cnet[ipt][m].image_id();
        // Skip measurements dropped as per-observation outliers, so the clean
        // match files and the stats reflect only the measurements that survived.
        if (outlier_obs.find(std::make_pair(img, ipt)) != outlier_obs.end())
          continue;
        image_index[img].push_back(std::make_pair(ipt, m));
      }
      // Record the image pairs this point contributes to. Only needed to create
      // match files from scratch, so skip the quadratic loop otherwise.
      if (need_match_pairs) {
        for (int i1 = 0; i1 < num_meas; i1++) {
          for (int i2 = 0; i2 < num_meas; i2++) {
            int a = cnet[ipt][i1].image_id(), b = cnet[ipt][i2].image_id();
            if (a == b)
              continue;
            if (outlier_obs.find(std::make_pair(a, ipt)) != outlier_obs.end() ||
                outlier_obs.find(std::make_pair(b, ipt)) != outlier_obs.end())
              continue;
            match_pairs.insert(std::make_pair(a, b));
          }
        }
      }
    }
    // Sort each image's measures by control point index for the merge below.
    for (auto & kv: image_index)
      std::sort(kv.second.begin(), kv.second.end());
  }

  // Reconstruct, for a single image pair, the map from a match's position
  // quadruplet to its (left, right) interest-point scales, by intersecting the two
  // images' sorted point lists. Built on demand for one pair at a time, so memory
  // stays bounded. The std::map reproduces the exact sorted, de-duplicated per-pair
  // structure used below, so the clean match files are byte-identical to before.
  auto makePairQuads = [&cnet, &image_index](int left_index, int right_index) {
    std::map<Quadruplet, std::pair<float, float>> quads;
    auto la = image_index.find(left_index), ra = image_index.find(right_index);
    if (la == image_index.end() || ra == image_index.end())
      return quads;
    std::vector<std::pair<int, int>> const& lv = la->second;
    std::vector<std::pair<int, int>> const& rv = ra->second;
    size_t i = 0, j = 0;
    while (i < lv.size() && j < rv.size()) {
      if (lv[i].first < rv[j].first) { i++; continue; }
      if (rv[j].first < lv[i].first) { j++; continue; }
      // Same control point: a match for this image pair.
      int ipt = lv[i].first;
      auto const& ml = cnet[ipt][lv[i].second];
      auto const& mr = cnet[ipt][rv[j].second];
      quads[Quadruplet(ml.position()[0], ml.position()[1],
                       mr.position()[0], mr.position()[1])]
        = std::make_pair(ml.sigma()[0], mr.sigma()[0]);
      i++; j++;
    }
    return quads;
  };

  // If we read the matches from an ISIS cnet or nvm, there are no match files.
  // Then create them. 
  std::map<std::pair<int, int>, std::string> local_match_files = match_files;
  if (opt.isis_cnet != "" || opt.nvm != "") {
    // iterate over match pairs
    local_match_files.clear();
    for (auto const& pr: match_pairs) {
      int left_index  = pr.first;
      int right_index = pr.second;
      // When creating match files from scratch, let the first index
      // be less than the second.
      if (left_index > right_index)
        continue;
      std::string match_file
        = vw::match_filename(opt.out_prefix,
                             opt.image_files[left_index],
                             opt.image_files[right_index],
                             matches_as_txt);
      local_match_files[std::make_pair(left_index, right_index)] = match_file;
    }
  }

  // Print only the first few match files, to avoid verbosity.
  size_t const max_verbose_match_msgs = 100;
  size_t num_match_msgs = 0;

  // Work on individual image pairs
  for (const auto& match_it: local_match_files) {

    std::pair<int, int> cam_pair   = match_it.first;
    std::string         match_file = match_it.second;
    size_t left_index  = cam_pair.first;
    size_t right_index = cam_pair.second;
    if (left_index == right_index)
      vw::vw_throw(vw::ArgumentErr() << "Bookkeeping failure. Cannot have interest point "
                   << "matches between an image and itself.\n");

    // Reconstruct this pair's position->scale map from the control network, once,
    // for the uses below. Empty if the pair shares no surviving measures.
    std::map<Quadruplet, std::pair<float, float>> curr_quads
      = makePairQuads((int)left_index, (int)right_index);

    std::vector<vw::InterestPoint> orig_left_ip, orig_right_ip;
    if (opt.isis_cnet != "" || opt.nvm != "") {
      // Must create the matches from the cnet.
      auto const& match_pair = curr_quads; // alias
      // Iterate over the quadruplets and build matches. The scale is not tracked
      // for an ISIS cnet or nvm, so use a unit scale.
      for (auto const& q: match_pair) {
        auto const& pos = q.first; // position quadruplet (x1, y1, x2, y2)
        double s = 1.0; // scale
        orig_left_ip.push_back(vw::InterestPoint(std::get<0>(pos), std::get<1>(pos), s));
        orig_right_ip.push_back(vw::InterestPoint(std::get<2>(pos), std::get<3>(pos), s));
      }

      // Write the matches formed from the cnet to disk
      if (opt.output_cnet_type == "match-files") {
        if (num_match_msgs < max_verbose_match_msgs)
          vw::vw_out() << "Writing: " << match_file << "\n";
        vw::write_match_file(match_file, orig_left_ip, orig_right_ip, matches_as_txt);
      }

    } else if (remove_outliers && !curr_quads.empty()) {
      // The matches were already read into the control network by
      // build_control_network, so do not re-read them from disk here, which is
      // slow at scale over a network file system. Rebuild the per-pair interest
      // points from the control network: the positions are the measure
      // positions and the scale is the measure sigma.
      auto const& quads = curr_quads;
      for (auto const& q: quads) {
        auto const& pos   = q.first;  // position quadruplet (x1, y1, x2, y2)
        auto const& scale = q.second; // interest point scale (left, right)
        orig_left_ip.push_back
          (vw::InterestPoint(std::get<0>(pos), std::get<1>(pos), scale.first));
        orig_right_ip.push_back
          (vw::InterestPoint(std::get<2>(pos), std::get<3>(pos), scale.second));
      }
    } else {
      // Read existing matches. Skip over match files that don't exist.
      if (!boost::filesystem::exists(match_file)) {
        if (num_match_msgs < max_verbose_match_msgs)
          vw_out() << "Skipping non-existent match file: " << match_file << "\n";
        continue;
      }
      // Read the original IP, to ensure later we write to disk only the subset
      // of the IP from the control network which are part of these original
      // ones.
      vw::read_match_file(match_file, orig_left_ip, orig_right_ip, matches_as_txt);
    }

    num_match_msgs++;
    if (num_match_msgs == max_verbose_match_msgs)
      vw::vw_out() << "Processed " << max_verbose_match_msgs
                   << " match files. Not printing more individual ones.\n";

    // Create a new convergence angle storage struct
    asp::MatchPairStats & convAngle = convAngles.back(); // alias
    if (!remove_outliers) {
      // Do some processing with orig ip. Otherwise this will be done below
      // with the inlier ip.
      processMatchPair(left_index, right_index,
                       orig_left_ip, orig_right_ip,
                       optimized_cams,
                       mapproj_dem_georef, interp_mapproj_dem, opt.datum,
                       save_mapproj_match_points_offsets,
                       propagate_errors, horizontal_stddev_vec, opt.bathy_data,
                       // Will append to entities below
                       convAngles, mapprojPoints, mapprojOffsets, mapprojOffsetsPerCam,
                       horizVertErrors, uncertPoints);
      // Since no outliers are removed, nothing else to do
      continue;
    }
    // Keep only inliers and non-gcp. GCP are used in optimization but are not
    // part of the originally found interest point matches.
    std::vector<vw::InterestPoint> left_ip, right_ip;
    for (size_t ip_iter = 0; ip_iter < orig_left_ip.size(); ip_iter++) {
      Quadruplet q(orig_left_ip[ip_iter].x, orig_left_ip[ip_iter].y,
                   orig_right_ip[ip_iter].x, orig_right_ip[ip_iter].y);
      auto const& match_pair = curr_quads; // alias
      if (match_pair.find(q) == match_pair.end())
        continue;

      // We do not copy descriptors, those take storage
      left_ip.push_back(vw::InterestPoint(orig_left_ip[ip_iter].x,
                                          orig_left_ip[ip_iter].y,
                                          orig_left_ip[ip_iter].scale));
      right_ip.push_back(vw::InterestPoint(orig_right_ip[ip_iter].x,
                                           orig_right_ip[ip_iter].y,
                                           orig_right_ip[ip_iter].scale));
    }

    // Filter by disparity
    // TODO(oalexan1): Note that this does not update the outliers set. Likely this
    // processing needs to move where other outlier filtering logic is.
    bool quiet = true; // Otherwise too many messages are printed
    if (opt.remove_outliers_params[0] > 0 && opt.remove_outliers_params[1] > 0.0) {
      // The typical value of 75 for opt.remove_outliers_params[1] may be too low.
      // Adjust it. pct = 75 becomes pct = 90. pct = 100 becomes pct = 100. So,
      // if starting under 100, it gets closer to 100 but stays under it.
      double pct = opt.remove_outliers_params[0];
      pct = 100.0 * (pct + 150.0) / 250.0;
      asp::filter_ip_by_disparity(pct, opt.remove_outliers_params[1],
                                  quiet, left_ip, right_ip);
    }

    if (num_cameras == 2) {
      // Compute the coverage fraction
      Vector2i right_image_size = file_image_size(opt.image_files[1]);
      int right_ip_width = right_image_size[0]*
        static_cast<double>(100.0 - std::max(opt.ip_edge_buffer_percent, 0))/100.0;
      Vector2i ip_size(right_ip_width, right_image_size[1]);
      double ip_coverage = asp::calc_ip_coverage_fraction(right_ip, ip_size);
      // Careful with the line below, it gets used in process_icebridge_batch.py.
      vw_out() << "IP coverage fraction after cleaning = " << ip_coverage << "\n";
    }

    // Process the inlier ip
    processMatchPair(left_index, right_index, left_ip, right_ip,
                     optimized_cams, mapproj_dem_georef, interp_mapproj_dem,
                     opt.datum,
                     save_mapproj_match_points_offsets,
                     propagate_errors, horizontal_stddev_vec, opt.bathy_data,
                     // Will append to entities below
                     convAngles, mapprojPoints, mapprojOffsets, mapprojOffsetsPerCam,
                     horizVertErrors, uncertPoints);

    if (opt.output_cnet_type != "match-files" || !save_clean_matches)
      continue; // Do not write match files

    // Make a clean copy of the file
    std::string clean_match_file = vw::clean_match_filename(match_file, matches_as_txt);
    if (opt.clean_match_files_prefix != "") {
      // Ensure "clean" does not show up twice
      clean_match_file = match_file;
      // Write the clean match file in the current dir, not where it was read from
      clean_match_file.replace(0, opt.clean_match_files_prefix.size(), opt.out_prefix);
    } else if (opt.match_files_prefix != "") {
      // Write the clean match file in the current dir, not where it was read from
      clean_match_file.replace(0, opt.match_files_prefix.size(), opt.out_prefix);
    }

    vw_out() << "Saving " << left_ip.size() << " filtered interest points.\n";
    vw_out() << "Writing: " << clean_match_file << "\n";
    vw::write_match_file(clean_match_file, left_ip, right_ip, matches_as_txt);

  } // End loop through the match files

  // Also find convergence angles from GCP, but only for pairs not already
  // covered by a match file above. So the report is the union of the two.
  {
    std::map<std::pair<int, int>,
             std::pair<std::vector<vw::InterestPoint>,
                       std::vector<vw::InterestPoint>>> gcp_pairs;
    for (int ipt = 0; ipt < cnet.size(); ipt++) {
      if (outliers.find(ipt) != outliers.end())
        continue;
      if (cnet[ipt].type() != ControlPoint::GroundControlPoint)
        continue;
      for (auto m1 = cnet[ipt].begin(); m1 != cnet[ipt].end(); m1++) {
        for (auto m2 = cnet[ipt].begin(); m2 != cnet[ipt].end(); m2++) {
          int left_index  = m1->image_id();
          int right_index = m2->image_id();
          if (left_index >= right_index)
            continue; // one direction only, and skip self-pairs
          // Skip pairs already handled through a match file above
          if (local_match_files.find(std::make_pair(left_index, right_index))
                != local_match_files.end() ||
              local_match_files.find(std::make_pair(right_index, left_index))
                != local_match_files.end())
            continue;
          auto & pr = gcp_pairs[std::make_pair(left_index, right_index)];
          pr.first.push_back
            (vw::InterestPoint(m1->position()[0], m1->position()[1], 1.0));
          pr.second.push_back
            (vw::InterestPoint(m2->position()[0], m2->position()[1], 1.0));
        }
      }
    }
    for (auto const& it: gcp_pairs) {
      int left_index  = it.first.first;
      int right_index = it.first.second;
      convAngles.push_back(asp::MatchPairStats());
      std::vector<double> sorted_angles;
      asp::convergence_angles(optimized_cams[left_index].get(),
                              optimized_cams[right_index].get(),
                              it.second.first, it.second.second, sorted_angles);
      convAngles.back().populate(left_index, right_index, sorted_angles);
    }
  }

  // Save the produced files
  std::string conv_angles_file = opt.out_prefix + "-convergence_angles.txt";
  asp::saveConvergenceAngles(conv_angles_file, convAngles, opt.image_files);

  if (!opt.mapproj_dem.empty())
    asp::saveMapprojOffsets(opt.out_prefix,
                            mapproj_dem_georef,
                            mapprojPoints,
                            mapprojOffsets,
                            mapprojOffsetsPerCam, // will change
                            opt.image_files);

  if (opt.propagate_errors) {
    std::string horiz_vert_errors_file = opt.out_prefix + "-triangulation_uncertainty.txt";
    asp::saveHorizVertErrors(horiz_vert_errors_file, horizVertErrors, opt.image_files);

    // Per-match propagated errors with their ground location, for gridding or
    // plotting (:numref:`error_propagation`). Very analogous to pointmap.csv.
    std::string pts_file = opt.out_prefix + "-triangulation_uncertainty_points.csv";
    vw_out() << "Writing: " << pts_file << "\n";
    std::ofstream ofs(pts_file.c_str());
    ofs << "# lon, lat, height_above_datum, horizontal_stddev, vertical_stddev\n";
    ofs << "# " << opt.datum << "\n";
    ofs.precision(17); // full double precision, as in pointmap.csv
    for (size_t it = 0; it < uncertPoints.size(); it++)
      ofs << uncertPoints[it][0] << ", " << uncertPoints[it][1] << ", "
          << uncertPoints[it][2] << ", " << uncertPoints[it][3] << ", "
          << uncertPoints[it][4] << "\n";
    ofs.close();
  }

  return;
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

void distortion_sanity_check(std::vector<int> const& num_dist_params,
                             IntrinsicOptions const& intrinsics_opts,
                             std::vector<double> const& intrinsics_limits) {

  // If nothing is shared, there is nothing to do

  // If all distortion params are shared, all sizes must agree
  if (!intrinsics_opts.share_intrinsics_per_sensor &&
      intrinsics_opts.distortion_shared) {
    for (size_t it = 1; it < num_dist_params.size(); it++) {
      if (num_dist_params[it] != num_dist_params[0])
           vw_throw(ArgumentErr() << "When sharing distortion parameters, "
             << "they must have the same size.\n");
    }
  }

  // If distortion is shared per sensor
  if (intrinsics_opts.share_intrinsics_per_sensor) {
    std::vector<std::set<int>> dist_sizes(intrinsics_opts.num_sensors);
    for (size_t cam_it = 0; cam_it < num_dist_params.size(); cam_it++) {
      int sensor_it = intrinsics_opts.cam2sensor[cam_it];
      dist_sizes[sensor_it].insert(num_dist_params[cam_it]); // all found sizes
    }
    // Now check that each dist_sizes[sensor_it] has size 1
    for (size_t sensor_it = 0; sensor_it < intrinsics_opts.num_sensors; sensor_it++) {
      if (dist_sizes[sensor_it].size() != 1)
        vw_throw(ArgumentErr() << "When sharing distortion parameters per sensor, "
         << "they must have the same size for all cameras of the same sensor.\n");
    }
  }

  // Intrinsics limits only can be used for now when all distortion vectors have
  // the same size. This could be fixed but it is a rarely used option
  // and would require a lot of bookkeeping.
  if (!intrinsics_limits.empty()) {
    bool have_same_size = true;
    for (size_t it = 1; it < num_dist_params.size(); it++) {
      if (num_dist_params[it] != num_dist_params[0]) {
        have_same_size = false;
        break;
      }
    }
    if (!have_same_size)
      vw_throw(ArgumentErr() << "When using --intrinsics-limits, all cameras "
               << "must have the same number of distortion coefficients.\n");
  }

  return;
}

// A little function to replace separators with space. Note that the backslash
// is a separator, in case, it used as a continuation line.
void replace_separators_with_space(std::string & str) {
  std::string sep = "\\:;, \t\r\n";
  for (size_t it = 0; it < sep.size(); it++)
    std::replace(str.begin(), str.end(), sep[it], ' ');
}

// Split a string into a vector of strings with space as separator
std::vector<std::string> split_str_with_space(std::string const& str) {
  std::istringstream is(str);
  std::vector<std::string> ret;
  std::string val;
  while (is >> val)
    ret.push_back(val);
  return ret;
}

// Function that tells if a string is a non-negative integer
bool is_str_non_neg_integer(std::string const& str) {
  if (str.empty()) return false;
  for (size_t it = 0; it < str.size(); it++) {
    if (!isdigit(str[it])) return false;
  }
  return true;
}

// Parse format:
// "1:focal_length,optical_center 2:focal_length,other_intrinsics 3:none"  
// Applies when optimizing intrinsics per sensor.
// The numbers are sensor indices, starting with 1.
void fine_grained_parse(bool share_intrinsics_per_sensor,
                        int num_sensors,
                        std::vector<std::string> const& options,
                        // Outputs
                        std::vector<bool> & float_center,
                        std::vector<bool> & float_focus,
                        std::vector<bool> & float_distortion) {

  // Sanity checks
  if (!share_intrinsics_per_sensor)
    vw_throw(ArgumentErr() << "Intrinsics are not being optimized per sensor. Remove any "
             << "fields of the form 1:, etc., from the options for floating intrinsics.\n");

  if (num_sensors <= 0)
    vw_throw(ArgumentErr() << "Expecting a positive number of sensors.\n");
  if (options.empty())
    vw_throw(ArgumentErr() << "Expecting at least one option.\n");

  // Wipe the outputs
  float_center.clear();
  float_focus.clear();
  float_distortion.clear();

  // It is convenient to initialize all to false. 0th element must always exist.
  for (int i = 0; i < std::max(num_sensors, 1); i++) {
    float_center.push_back(false);
    float_focus.push_back(false);
    float_distortion.push_back(false);
  }

  // First entity must be an integer. That is sensor id, starting from 1.
  if (!is_str_non_neg_integer(options[0]))
    vw_throw(ArgumentErr() << "Expecting an integer as the first option.\n");

  int sensor_id = atoi(options[0].c_str()) - 1; // subtract 1 to make it zero-based

  // check for duplicate ids
  std::set<int> seen_ids;
  // Iterate over options
  for (size_t it = 0; it < options.size(); it++) {

    // If it is an integer, update the sensor id, and continue.
    // Subtract 1 to make it zero-based.
    if (is_str_non_neg_integer(options[it])) {
      sensor_id = atoi(options[it].c_str()) - 1;

      // Sensor id must be in bounds
      if (sensor_id < 0 || sensor_id >= num_sensors)
        vw_throw(ArgumentErr() << "Sensor id " << options[it] << " is out of bounds.\n");

      // If seen, that's a problem
      if (seen_ids.find(sensor_id) != seen_ids.end())
        vw_throw(ArgumentErr() << "Sensor id " << options[it] << " is repeated.\n");
      // Record as seen
      seen_ids.insert(sensor_id);

      continue;
    }

    // Handle the optical center
    if (options[it] == "optical_center") {
      float_center[sensor_id] = true;
      continue;
    }

    // Handle the focal length
    if (options[it] == "focal_length") {
      float_focus[sensor_id] = true;
      continue;
    }

    // Handle the distortion (other_intrinsics)
    if (options[it] == "other_intrinsics" || options[it] == "distortion") {
      float_distortion[sensor_id] = true;
      continue;
    }

    // For all, populate all fields
    if (options[it] == "all") {
      float_center[sensor_id]     = true;
      float_focus[sensor_id]      = true;
      float_distortion[sensor_id] = true;
      continue;
    }

    // For none, just skip
    if (options[it] == "none") {
      continue;
    }

    vw_throw(ArgumentErr() << "Found unknown option when parsing which "
              << "sensor intrinsics to float: " << options[it] << ".\n");
  }
}

// Parse format:
// "focal_length optical_center other_intrinsics"
// Applies to all sensors and when not optimizing intrinsics per sensor.
void coarse_grained_parse(int num_sensors,
                          std::vector<std::string> const& options,
                          // Outputs
                          std::vector<bool> & float_center,
                          std::vector<bool> & float_focus,
                          std::vector<bool> & float_distortion) {

  // Wipe the outputs
  float_center.clear();
  float_focus.clear();
  float_distortion.clear();

  if (num_sensors < 0)
    vw_throw(ArgumentErr() << "Cameras were not parsed correctly.\n");

   // It is convenient to initialize all to false. 0th element must always exist.
  for (int i = 0; i < std::max(num_sensors, 1); i++) {
    float_center.push_back(false);
    float_focus.push_back(false);
    float_distortion.push_back(false);
  }

  // For now, populate only for sensor with id 0
  int sensor_id = 0;

  // Iterate over options
  for (size_t it = 0; it < options.size(); it++) {

     // Must not have an integer here
     if (is_str_non_neg_integer(options[0]))
        vw_throw(ArgumentErr() << "When parsing intrinsics to float, expecting a "
                 << "string, not an integer. Check your inputs.\n");

    // Handle the optical center
    if (options[it] == "optical_center") {
      float_center[sensor_id] = true;
      continue;
    }

    // Handle the focal length
    if (options[it] == "focal_length") {
      float_focus[sensor_id] = true;
      continue;
    }

    // Handle the distortion (other_intrinsics)
    if (options[it] == "other_intrinsics" || options[it] == "distortion") {
      float_distortion[sensor_id] = true;
      continue;
    }

    // For all, populate all fields
    if (options[it] == "all") {
      float_center[sensor_id]     = true;
      float_focus[sensor_id]      = true;
      float_distortion[sensor_id] = true;
      continue;
    }

    // For none, just skip
    if (options[it] == "none") {
      continue;
    }

    // We should not arrive here
    vw_throw(ArgumentErr() << "Found unknown option when parsing which "
              << "sensor intrinsics to float: " << options[it] << ".\n");
  }

  // Distribute for all sensors.
  // This will happen only if we share intrinsics per sensor.
  for (int sensor_id = 0; sensor_id < num_sensors; sensor_id++) {
     float_center[sensor_id]     = float_center[0];
     float_focus[sensor_id]      = float_focus[0];
     float_distortion[sensor_id] = float_distortion[0];
  }

  return;
}

void print_float(bool do_float) {
  if (do_float)
    vw_out() << "floated\n";
  else
    vw_out() << "fixed\n";
}

void print_float_vec(std::vector<bool> const& intrinsics, std::string const& name) {
  vw_out() << name << ": ";
  for (size_t it = 0; it < intrinsics.size(); it++) {
    if (intrinsics[it])
      vw_out() << "floated ";
    else
      vw_out() << "fixed ";
  }
  vw_out() << "\n";
}

void print_shared(bool shared) {
  if (shared)
    vw_out() << "shared\n";
  else
    vw_out() << "not shared\n";
}

/// For each option, the string must include a subset of the entries:
///  "focal_length, optical_center, distortion_params"
/// - Need the extra boolean to handle the case where --intrinsics-to-share
///   is provided as "" in order to share none of them.
void load_intrinsics_options(bool        solve_intrinsics,
                             bool        shared_is_specified,
                             std::string intrinsics_to_float_str, // make a copy
                             std::string intrinsics_to_share_str, // make a copy
                             asp::IntrinsicOptions & intrinsics_options) {

  // Share everything unless told otherwise or not solving for intrinsics
  intrinsics_options.focus_shared        = solve_intrinsics;
  intrinsics_options.center_shared       = solve_intrinsics;
  intrinsics_options.distortion_shared   = solve_intrinsics;

  // We need these to be initialized even when not solving for intrinsics, as
  // the intrinsics are always added to the cost function when
  // --inline-adjustments is used. In that case they are just expected to be
  // kept fixed.
  intrinsics_options.float_center.resize(1, false);
  intrinsics_options.float_focus.resize(1, false);
  intrinsics_options.float_distortion.resize(1, false);

  if (((intrinsics_to_float_str != "" && intrinsics_to_float_str != "none") ||
      (intrinsics_to_share_str != "" && intrinsics_to_share_str != "none"))
      && !solve_intrinsics) {
    vw::vw_throw(vw::ArgumentErr() << "To be able set intrinsics to float or share, "
             << "the option --solve-intrinsics must be on.\n");
  }

  if (!solve_intrinsics)
    return;

  // If the user did not specify which intrinsics to float, float all of them.
  boost::to_lower(intrinsics_to_float_str);
  if (intrinsics_to_float_str == "" || intrinsics_to_float_str == "all")
    intrinsics_to_float_str = "focal_length optical_center other_intrinsics";
  // This is the right place in which to turn 'none' to empty string,
  // which now will mean float nothing.
  if (intrinsics_to_float_str == "none")
    intrinsics_to_float_str = "";

  // If the user did not specify which intrinsics to share, share all of them.
  boost::to_lower(intrinsics_to_share_str);
  if (!shared_is_specified) {
    intrinsics_to_share_str = "focal_length optical_center other_intrinsics";
  } else {
    // Otherwise, 'all' also means share all of them, 'none' means share none
    if (intrinsics_to_share_str == "all")
      intrinsics_to_share_str = "focal_length optical_center other_intrinsics";
    if (intrinsics_to_share_str == "none")
      intrinsics_to_share_str = "";
  }

  if (intrinsics_options.share_intrinsics_per_sensor && shared_is_specified)
    vw_out() << "When sharing intrinsics per sensor, option "
              << "--intrinsics-to-share is ignored. The intrinsics will "
              << "always be shared for a sensor and never across sensors.\n";

  // If sharing intrinsics per sensor, the only supported mode is that 
  // the intrinsics are always shared per sensor and never across sensors.
  if (shared_is_specified && !intrinsics_options.share_intrinsics_per_sensor) {
    intrinsics_options.focus_shared      = false;
    intrinsics_options.center_shared     = false;
    intrinsics_options.distortion_shared = false;
  }

  // Replace any separators (:;, \t\r\n) with spaces. It can be convenient to
  // use commas and colons as separators when passing in the options from the command line.
  asp::replace_separators_with_space(intrinsics_to_float_str);
  asp::replace_separators_with_space(intrinsics_to_share_str);

  // Parse float options. Supported formats:
  // "1:focal_length,optical_center 2:focal_length,other_intrinsics 3:none"  
  // "focal_length optical_center other_intrinsics"
  // In the first case, the numbers are sensor indices, starting with 1.
  std::vector<std::string> float_options = asp::split_str_with_space(intrinsics_to_float_str);
  if (!float_options.empty() && asp::is_str_non_neg_integer(float_options[0])) {
    asp::fine_grained_parse(intrinsics_options.share_intrinsics_per_sensor,
                            intrinsics_options.num_sensors,
                            float_options,
                            intrinsics_options.float_center,
                            intrinsics_options.float_focus,
                            intrinsics_options.float_distortion);
  } else {
    asp::coarse_grained_parse(intrinsics_options.num_sensors,
                              float_options,
                              intrinsics_options.float_center,
                              intrinsics_options.float_focus,
                              intrinsics_options.float_distortion);
  }

  // Useful reporting
  std::string center_name = "Optical center";
  std::string focus_name  = "Focal length";
  std::string dist_name   = "Other intrinsics (distortion)";
  if (intrinsics_options.share_intrinsics_per_sensor) {
    vw_out() << "Intrinsics are shared for all cameras with given sensor.\n";
    vw_out() << "Number of sensors: " << intrinsics_options.num_sensors << "\n";
    vw_out() << "For each sensor:\n";
    print_float_vec(intrinsics_options.float_center, center_name);
    print_float_vec(intrinsics_options.float_focus, focus_name);
    print_float_vec(intrinsics_options.float_distortion, dist_name);
  } else {
    vw_out() << center_name << ": "; print_float(intrinsics_options.float_center[0]);
    vw_out() << focus_name  << ": "; print_float(intrinsics_options.float_focus[0]);
    vw_out() << dist_name   << ": "; print_float(intrinsics_options.float_distortion[0]);
  }

  // No parsing is done when sharing intrinsics per sensor, per above 
  std::string val;
  if (shared_is_specified && !intrinsics_options.share_intrinsics_per_sensor) {
    std::istringstream is(intrinsics_to_share_str);
    while (is >> val) {
      if (val == "focal_length")
        intrinsics_options.focus_shared = true;
      else if (val == "optical_center")
        intrinsics_options.center_shared = true;
      else if (val == "other_intrinsics" || val == "distortion")
        intrinsics_options.distortion_shared = true;
      else
        vw_throw(ArgumentErr() << "Error: Found unknown intrinsic to share: "
          << val << ".\n");
    }
  }

  // Useful info
  std::string sensor_mode = " (across sensors): ";
  if (intrinsics_options.share_intrinsics_per_sensor)
    sensor_mode = " (per sensor): "; // useful clarification
  vw_out() << center_name << sensor_mode; print_shared(intrinsics_options.center_shared);
  vw_out() << focus_name << sensor_mode;  print_shared(intrinsics_options.focus_shared);
  vw_out() << dist_name << sensor_mode;   print_shared(intrinsics_options.distortion_shared);

} // End function load_intrinsics_options

/// Parse the string of limits and make sure they are all valid pairs.
void parse_intrinsics_limits(std::string const& intrinsics_limits_str,
                             std::vector<double> & intrinsics_limits) {

  intrinsics_limits.clear();
  std::istringstream is(intrinsics_limits_str);
  double val;
  int    count = 0;
  while (is >> val) {
    intrinsics_limits.push_back(val);
    if (count % 2 == 1) {
      if (intrinsics_limits[count] < intrinsics_limits[count-1])
        vw_throw(vw::ArgumentErr()
                  << "Error: Intrinsic limit pairs must be min before max.\n");
    }
    count++;
  }

  if (count % 2 != 0)
    vw::vw_throw(vw::ArgumentErr()
              << "Error: Intrinsic limits must always be provided in min max pairs.\n");
}

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
