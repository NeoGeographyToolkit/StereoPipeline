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

/// \file BundleAdjustGcp.cc
///
/// GCP-based camera initialization and transform, and GCP sanity checks,
/// for bundle_adjust.

#include <asp/Camera/BundleAdjustCamera.h>
#include <asp/Camera/BundleAdjustTransform.h>
#include <asp/Camera/CameraResectioning.h>

#include <vw/BundleAdjustment/ControlNetwork.h>
#include <vw/BundleAdjustment/ControlNetworkLoader.h>
#include <vw/Camera/PinholeModel.h>
#include <vw/Camera/CameraUtilities.h>
#include <vw/Math/Geometry.h>
#include <vw/Math/Matrix.h>

#include <string>
#include <vector>

using namespace vw;

namespace asp {

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

} // end namespace asp
