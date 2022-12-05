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


/// \file BundleAdjustCamera.cc
///

// TODO(oalexan1): Move most of BundleAdjustCamera.h code to here, and put it
// all in the asp namespace. 

#include <asp/Camera/BundleAdjustCamera.h>
#include <asp/Core/IpMatchingAlgs.h>         // Lightweight header

#include <vw/Cartography/CameraBBox.h>
#include <vw/InterestPoint/Matcher.h>
#include <vw/FileIO/KML.h>
#include <asp/Camera/CameraResectioning.h>

#include <string>

using namespace vw;
using namespace vw::camera;
using namespace vw::ba;

void asp::BAParams::record_points_to_kml(const std::string &kml_path,
                                         const vw::cartography::Datum& datum,
                                         size_t skip, const std::string name,
                                         const std::string icon) {
  if (datum.name() == asp::UNSPECIFIED_DATUM) {
    vw::vw_out(vw::WarningMessage) << "No datum specified, can't write file: "
                                   << kml_path << std::endl;
    return;
  }
  
  // Open the file
  vw::vw_out() << "Writing: " << kml_path << std::endl;
  vw::KMLFile kml(kml_path, name);
  
    // Set up a simple point icon with no labels
  const bool hide_labels = true;
  kml.append_style( "point", "", 1.0, icon, hide_labels);
  kml.append_style( "point_highlight", "", 1.1, icon, hide_labels);
  kml.append_stylemap( "point_placemark", "point",
                       "point_highlight");
  
  // Loop through the points
  const bool extrude = true;
  for (size_t i=0; i<num_points(); i+=skip) {
    
    if (get_point_outlier(i))
      continue; // skip outliers
    
    // Convert the point to GDC coords
    vw::Vector3 xyz         = get_point(i);
    vw::Vector3 lon_lat_alt = datum.cartesian_to_geodetic(xyz);

    // Add this to the output file
    kml.append_placemark( lon_lat_alt.x(), lon_lat_alt.y(),
                            "", "", "point_placemark",
                          lon_lat_alt[2], extrude );
  }
    kml.close_kml();
}

void pack_pinhole_to_arrays(vw::camera::PinholeModel const& camera,
                            int camera_index,
                            asp::BAParams & param_storage) {

  double* pos_pose_ptr   = param_storage.get_camera_ptr              (camera_index);
  double* center_ptr     = param_storage.get_intrinsic_center_ptr    (camera_index);
  double* focus_ptr      = param_storage.get_intrinsic_focus_ptr     (camera_index);
  double* distortion_ptr = param_storage.get_intrinsic_distortion_ptr(camera_index);

  // Handle position and pose
  CameraAdjustment pos_pose_info;
  pos_pose_info.copy_from_pinhole(camera);
  pos_pose_info.pack_to_array(pos_pose_ptr);

  // We are solving for multipliers to the intrinsic values, so they all start at 1.0.

  // Center point and focal length
  center_ptr[0] = 1.0; //camera.point_offset()[0];
  center_ptr[1] = 1.0; //camera.point_offset()[1];
  focus_ptr [0] = 1.0; //camera.focal_length()[0];

  // Pack the lens distortion parameters.
  vw::Vector<double> lens = camera.lens_distortion()->distortion_parameters();
  for (size_t i=0; i<lens.size(); ++i)
    distortion_ptr[i] = 1.0;
}

void pack_optical_bar_to_arrays(vw::camera::OpticalBarModel const& camera,
                                int camera_index,
                                asp::BAParams & param_storage) {

  double* pos_pose_ptr   = param_storage.get_camera_ptr              (camera_index);
  double* center_ptr     = param_storage.get_intrinsic_center_ptr    (camera_index);
  double* focus_ptr      = param_storage.get_intrinsic_focus_ptr     (camera_index);
  double* intrinsics_ptr = param_storage.get_intrinsic_distortion_ptr(camera_index);

  // Handle position and pose
  CameraAdjustment pos_pose_info;
  pos_pose_info.copy_from_optical_bar(camera);
  pos_pose_info.pack_to_array(pos_pose_ptr);

  // We are solving for multipliers to the intrinsic values, so they all start at 1.0.

  // Center point and focal length
  center_ptr[0] = 1.0; //camera.point_offset()[0];
  center_ptr[1] = 1.0; //camera.point_offset()[1];
  focus_ptr [0] = 1.0; //camera.focal_length()[0];

  // Pack the speed, MCF, and scan time into the distortion pointer.
  intrinsics_ptr[0] = 1.0;
  intrinsics_ptr[1] = 1.0;
  intrinsics_ptr[2] = 1.0;
}

/// Given a transform with origin at the planet center, like output
/// by pc_align, read the adjustments from cameras_vec, apply this
/// transform on top of them, and write the adjustments back to the vector.
/// - Works for pinhole and non-pinhole case.
void apply_transform_to_cameras(vw::Matrix4x4 const& M, asp::BAParams &param_storage,
                                std::vector<asp::CameraModelPtr>
                                const& cam_ptrs) {

  for (unsigned i = 0; i < param_storage.num_cameras(); i++) {

    // Load the current position/pose of this camera.
    double* cam_ptr = param_storage.get_camera_ptr(i);
    CameraAdjustment cam_adjust(cam_ptr);

    // Create the adjusted camera model
    vw::camera::AdjustedCameraModel cam(cam_ptrs[i], cam_adjust.position(), cam_adjust.pose());
    // Apply the transform
    cam.apply_transform(M);

    // Copy back the adjustments to the camera array.
    cam_adjust.copy_from_adjusted_camera(cam);
    cam_adjust.pack_to_array(cam_ptr);
  }
} // end function apply_transform_to_cameras

// This function takes advantage of the fact that when it is called the cam_ptrs have the same
//  information as is in param_storage!
void apply_transform_to_cameras_pinhole(vw::Matrix4x4 const& M,
                                        asp::BAParams & param_storage,
                                        std::vector<asp::CameraModelPtr>
                                        const& cam_ptrs){

  for (unsigned i = 0; i < param_storage.num_cameras(); i++) {
    // Apply the transform
    boost::shared_ptr<camera::PinholeModel> pin_ptr = 
      boost::dynamic_pointer_cast<vw::camera::PinholeModel>(cam_ptrs[i]);
    pin_ptr->apply_transform(M);

    // Write out to param_storage
    pack_pinhole_to_arrays(*pin_ptr, i, param_storage);    
  }

} // end function apply_transform_to_cameras_pinhole

/// Apply a scale-rotate-translate transform to pinhole cameras and control points
void apply_rigid_transform(vw::Matrix3x3 const & rotation,
                           vw::Vector3   const & translation,
                           double                scale,
                           std::vector<asp::CameraModelPtr> &camera_models,
                           boost::shared_ptr<ControlNetwork> const& cnet) {

  // Apply the transform to the cameras
  for (size_t icam = 0; icam < camera_models.size(); icam++){
    vw::camera::PinholeModel * pincam
      = dynamic_cast<vw::camera::PinholeModel*>(camera_models[icam].get());
    VW_ASSERT(pincam != NULL, vw::ArgumentErr() << "A pinhole camera expected.\n");

    pincam->apply_transform(rotation, translation, scale);
  } // End loop through cameras

  // Apply the transform to all of the world points in the ControlNetwork
  ControlNetwork::iterator iter;
  for (iter=cnet->begin(); iter!=cnet->end(); ++iter) {
    if (iter->type() == ControlPoint::GroundControlPoint)
      continue; // Don't convert the ground control points!

    Vector3 position     = iter->position();
    Vector3 new_position = scale*rotation*position + translation;
    iter->set_position(new_position);
  }
} // End function ApplyRigidTransform


/// Generate a warning if the GCP's are really far from the IP points
/// - This is intended to help catch the common lat/lon swap in GCP files.
void check_gcp_dists(std::vector<asp::CameraModelPtr> const& camera_models,
                     boost::shared_ptr<ControlNetwork> const& cnet_ptr,
                     double forced_triangulation_distance) {

  // Make one iteration just to count the points.
  const ControlNetwork & cnet = *cnet_ptr.get(); // Helper alias
  const int num_cnet_points = static_cast<int>(cnet.size());
  double gcp_count=0, ip_count=0;
  for (int ipt = 0; ipt < num_cnet_points; ipt++){

    if (cnet[ipt].position() == Vector3() || cnet[ipt].size() <= 1)
      continue;
    
    if (cnet[ipt].type() == ControlPoint::GroundControlPoint)
      gcp_count += 1.0;
    else {
      // Use triangulation to estimate the position of this control point using
      //   the current set of camera models.
      ControlPoint cp_new = cnet[ipt];
      double minimum_angle = 0;
      double ans = vw::ba::triangulate_control_point(cp_new, camera_models, minimum_angle,
						     forced_triangulation_distance);
      if (ans < 0 || cp_new.position() == Vector3())
        continue; // Skip points that fail to triangulate

      ip_count += 1.0;
    }
  } // End loop through control network points
  if (ip_count == 0 || gcp_count == 0)
    return; // Can't do this check if we don't have both point types.

  // Make another iteration to compute the mean.
  Vector3 mean_gcp(0,0,0);
  Vector3 mean_ip (0,0,0);
  for (int ipt = 0; ipt < num_cnet_points; ipt++){

    if (cnet[ipt].position() == Vector3() || cnet[ipt].size() <= 1)
      continue;
    
    if (cnet[ipt].type() == ControlPoint::GroundControlPoint) {

      mean_gcp += (cnet[ipt].position() / gcp_count);
      ++gcp_count;
    }else {
      // Use triangulation to estimate the position of this control point using
      //   the current set of camera models.
      ControlPoint cp_new = cnet[ipt];
      double minimum_angle = 0;
      double ans = vw::ba::triangulate_control_point(cp_new, camera_models, minimum_angle,
						     forced_triangulation_distance);
      // Skip points for which triangulation failed
      if (cp_new.position() == Vector3() || ans < 0)
	continue;

      mean_ip += (cp_new.position() / ip_count);
    }
  } // End loop through control network points

  double dist = norm_2(mean_ip - mean_gcp);
  if (dist > 100000)
    vw_out() << "WARNING: GCPs are over 100 km from the other points. Are your lat/lon GCP coordinates swapped?\n";
}

//============================================================================

// Initialize the position and orientation of each pinhole camera model using
// a least squares error transform to match the provided camera positions.
// This function overwrites the camera parameters in-place
bool asp::init_pinhole_model_with_camera_positions
(boost::shared_ptr<ControlNetwork> const& cnet, 
 std::vector<asp::CameraModelPtr> & camera_models,
 std::vector<std::string> const& image_files,
 std::vector<Vector3> const & estimated_camera_gcc) {

  vw_out() << "Initializing camera positions from input file." << std::endl;

  // Count the number of matches and check for problems
  const int num_cameras = image_files.size();
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
  vw::math::find_3D_transform(points_in, points_out, rotation, translation, scale);

  // Update the camera and point information with the new transform
  apply_rigid_transform(rotation, translation, scale, camera_models, cnet);
  return true;
}

// Given at least two images, each having at least 3 GCP that are not seen in other
// images, find and apply a transform to the camera system based on them.
void asp::transform_cameras_with_indiv_image_gcp
(boost::shared_ptr<ControlNetwork> const& cnet_ptr,
 std::vector<asp::CameraModelPtr> & camera_models) {
  
  vw_out() << "Applying transform to cameras given several GCP not shared among the images.\n";

  int num_cams = camera_models.size();

  // Create pinhole cameras
  std::vector<PinholeModel> pinhole_cams;
  for (int icam = 0; icam < num_cams; icam++){
    vw::camera::PinholeModel * pincam
      = dynamic_cast<vw::camera::PinholeModel*>(camera_models[icam].get());
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

  int ipt = - 1;
  for (auto iter = cnet.begin(); iter != cnet.end(); iter++) {
    ipt++;
    
    // Keep only gcp
    if (cnet[ipt].type() != ControlPoint::GroundControlPoint) {
      continue;
    }
        
    for (auto measure = (*iter).begin(); measure != (*iter).end(); measure++) {
      int cam_it = measure->image_id();
      if (cam_it < 0 || cam_it >= num_cams) 
	vw_throw(ArgumentErr() << "Error: cnet index out of range.\n");

      Vector2 pixel( measure->position()[0],  measure->position()[1]);
      pix[cam_it].push_back(pixel);
      xyz[cam_it].push_back(cnet[ipt].position());
    }
  }  

  Matrix3x3 rotation;
  Vector3   translation;
  double    scale;
  asp::align_cameras_to_ground(xyz, pix, pinhole_cams, rotation, translation, scale);

  // Update the camera and point information with the new transform
  vw_out() << "Applying transform based on GCP:\n";
  vw_out() << "Rotation:    " << rotation    << "\n";
  vw_out() << "Translation: " << translation << "\n";
  vw_out() << "Scale:       " << scale       << "\n";
  apply_rigid_transform(rotation, translation, scale, camera_models, cnet_ptr);
}

/// Initialize the position and orientation of each pinhole camera model using
/// a least squares error transform to match the provided control points file.
/// This function overwrites the camera parameters in-place. It works
/// if at least three GCP are seen in no less than two images.
void asp::transform_cameras_with_shared_gcp(boost::shared_ptr<ControlNetwork> const& cnet_ptr,
                                            std::vector<asp::CameraModelPtr> & camera_models) {
  
  vw_out() << "Applying transform to cameras given several GCP shared among the images.\n";
  const ControlNetwork & cnet = *cnet_ptr.get(); // Helper alias
  
  // DEBUG: Print out all pinhole cameras and verify they are pinhole cameras.
  for (size_t icam = 0; icam < camera_models.size(); icam++){
    vw::camera::PinholeModel * pincam
      = dynamic_cast<vw::camera::PinholeModel*>(camera_models[icam].get());
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
    double forced_triangulation_distance = -1;
    double err = vw::ba::triangulate_control_point(cp_new, camera_models,
						   minimum_angle, forced_triangulation_distance);

    if ((cp_new.position() != Vector3()) && (cnet[ipt].position() != Vector3()) &&
	// Note that if there is only one camera, we allow
	// for triangulation to return a half-baked answer,
	// as then there is nothing we can do. We need this
	// answer, as imperfect as it is, to create initial
	// camera models from GCP.
	(err > 0 || camera_models.size() == 1))
      ++num_good_gcp; // Only count points that triangulate
    else {
      vw_out() << "Discarding GCP: " << cnet[ipt]; // Built in endl
    }
  } // End good GCP counting

  // Update the number of GCP that we are using
  const int MIN_NUM_GOOD_GCP = 3;
  if (num_good_gcp < MIN_NUM_GOOD_GCP) {
    vw_out() << "Num GCP       = " << num_gcp         << std::endl;
    vw_out() << "Num valid GCP = " << num_good_gcp    << std::endl;
    vw_throw( ArgumentErr()
	      << "Not enough valid GCPs for affine transform pinhole initialization. "
	      << "You may need to use --disable-pinhole-gcp-init or "
	      << "--transform-cameras-using-gcp.\n" );
  }
  
  vw::Matrix<double> points_in(3, num_good_gcp), points_out(3, num_good_gcp);
  typedef vw::math::MatrixCol<vw::Matrix<double>> ColView;
  int index = 0;
  for (int ipt = 0; ipt < num_cnet_points; ipt++){
    // Loop through all the ground control points only
    if (cnet[ipt].type() != ControlPoint::GroundControlPoint)
      continue;
    
    // Use triangulation to estimate the position of this control point using
    //   the current set of camera models.
    ControlPoint curr_cp = cnet[ipt];
    // Making minimum_angle below big may throw away valid points at this stage // really???
    double minimum_angle = 0;
    double forced_triangulation_distance = -1;
    double ans = vw::ba::triangulate_control_point(curr_cp, camera_models,
                                                   minimum_angle, forced_triangulation_distance);
    // TODO(oalexan1): Need to check here if triangulation is successful
    
    // Store the computed and correct position of this point
    Vector3 inp  = curr_cp.position();
    Vector3 outp = cnet[ipt].position();
    
    if (inp == Vector3() || outp == Vector3())
      continue; // Skip points that fail to triangulate
    
    // Store in matrices
    ColView colIn (points_in,  index); 
    ColView colOut(points_out, index);
    colIn  = inp;
    colOut = outp;
    
    ++index;
  } // End loop through control network points

  if (camera_models.size() == 1) {
    int icam = 0;
    vw::camera::PinholeModel * pincam
      = dynamic_cast<vw::camera::PinholeModel*>(camera_models[icam].get());
    VW_ASSERT(pincam != NULL, vw::ArgumentErr() << "A pinhole camera expected.\n");

    std::vector<vw::Vector2> pixel_observations;
    std::vector<vw::Vector3> ground_points;
    for (int ipt = 0; ipt < num_cnet_points; ipt++){
      // Loop through all the ground control points only
      if (cnet[ipt].type() != ControlPoint::GroundControlPoint)
        continue;
      ground_points.push_back(cnet[ipt].position());

      int num_meas = 0;
      for (ControlPoint::const_iterator measure = cnet[ipt].begin();
           measure != cnet[ipt].end(); measure++) {
        
        // TODO(oalexan1): What if there's more gcp?
        int cam_it = measure->image_id();
        if (cam_it != 0) 
          vw_throw(ArgumentErr() << "Error: Expecting a single camera.\n");
        
        Vector2 pixel(measure->position()[0], measure->position()[1]);
        num_meas++;
        if (num_meas > 1)
          vw::vw_throw(vw::ArgumentErr() << "Expecting a single camera pixel per gcp.\n");

        pixel_observations.push_back(pixel);
      }
    }

    // Find the camera pose with given observations and intrinsics
    vw::camera::PinholeModel updated_cam = *pincam; // deep copy, including of distortion
    asp::findCameraPose(ground_points, pixel_observations, updated_cam);
    *pincam = updated_cam;
    
  } else {
    // Call function to compute a 3D affine transform between the two point sets
    vw::Matrix3x3 rotation;
    vw::Vector3   translation;
    double        scale;
    vw::math::find_3D_transform(points_in, points_out, rotation, translation, scale);
  
    // Update the camera and point information with the new transform
    vw_out() << "Applying transform based on GCP:\n";
    vw_out() << "Rotation:    " << rotation    << "\n";
    vw_out() << "Translation: " << translation << "\n";
    vw_out() << "Scale:       " << scale       << "\n";
    vw_out() << "This transform can be disabled with --disable-pinhole-gcp-init.\n";
    apply_rigid_transform(rotation, translation, scale, camera_models, cnet_ptr);
  }
  
} // End function init_pinhole_model_with_gcp

// Given original cams in sfm_cams and individually scaled cameras in
// aux_cams, get the median scale change from the first set to the second one.
// It is important to do the median, since scaling the cameras individually
// is a bit of a shaky business.
double asp::find_median_scale_change(std::vector<PinholeModel> const & sfm_cams,
                                     std::vector<PinholeModel> const & aux_cams,
                                     std::vector< std::vector<Vector3> > const& xyz){
  
  int num_cams = sfm_cams.size();

  std::vector<double> scales;
  
  for (int it1 = 0; it1 < num_cams; it1++) {

    bool is_good = (xyz[it1].size() >= 3);
    if (!is_good)
      continue;
    
    for (int it2 = it1 + 1; it2 < num_cams; it2++) {
      
      bool is_good = (xyz[it2].size() >= 3);
      if (!is_good)
	continue;
    
      double len1 = norm_2(sfm_cams[it1].camera_center()
			   - sfm_cams[it2].camera_center());
      double len2 = norm_2(aux_cams[it1].camera_center()
			   - aux_cams[it2].camera_center());
      
      double scale = len2/len1;
      scales.push_back(scale);
    }
  }

  if (scales.empty())
    vw_throw( LogicErr() << "Could not find two images with at least 3 GCP each.\n");
    
  double median_scale = vw::math::destructive_median(scales);

  return median_scale;
}


// Given some GCP so that at least two images have at at least three GCP each,
// but each GCP is allowed to show in one image only, use the GCP
// to transform cameras to ground coordinates.
void asp::align_cameras_to_ground(std::vector< std::vector<Vector3> > const& xyz,
                                  std::vector< std::vector<Vector2> > const& pix,
                                  std::vector<PinholeModel> & sfm_cams,
                                  Matrix3x3 & rotation, 
                                  Vector3 & translation,
                                  double & scale){
  
  std::string camera_type = "pinhole";
  bool refine_camera = true;
  bool verbose = false; 

  // Cameras individually aligned to ground using GCP. They may not be
  // self-consistent, and are only used to give an idea of the
  // transform to apply to the unaligned cameras.
  std::vector<PinholeModel> aux_cams;

  int num_cams = sfm_cams.size();
  for (int it = 0; it < num_cams; it++) {
    // Export to the format used by the API
    std::vector<double> pixel_values;
    for (size_t c = 0; c < pix[it].size(); c++) {
      pixel_values.push_back(pix[it][c][0]);
      pixel_values.push_back(pix[it][c][1]);
    }

    asp::CameraModelPtr out_cam(new PinholeModel(sfm_cams[it]));

    bool is_good = (xyz[it].size() >= 3);
    if (is_good) 
      fit_camera_to_xyz(camera_type, refine_camera,  
			xyz[it], pixel_values, verbose, out_cam);
    
    aux_cams.push_back(*((PinholeModel*)out_cam.get()));
  }

  double world_scale = asp::find_median_scale_change(sfm_cams, aux_cams, xyz);
  vw_out() << "Initial guess scale to apply when converting to world coordinates using GCP: "
	   << world_scale << ".\n";

  // So far we aligned both cameras individually to GCP and we got an
  // idea of scale.  Yet we would like to align them without changing
  // the relationship between them, so using a single transform for
  // all not an individual transform for each.  This way we will
  // transform the SfM-computed cameras to the new coordinate system.

  // Start by estimating a such a transform.
  int num_pts = 0;
  for (int it = 0; it < num_cams; it++) {
    bool is_good = (xyz[it].size() >= 3);
    if (is_good) 
      num_pts += pix[it].size();
  }
  
  vw::Matrix<double> in_pts, out_pts;
  in_pts.set_size(3, num_pts);
  out_pts.set_size(3, num_pts);
  
  int col = 0;
  for (int it = 0; it < num_cams; it++) {
    
    bool is_good = (xyz[it].size() >= 3);
    if (is_good) {
      // For each camera, find xyz values in the input cameras
      // that map to GCP. Use the scale for that.
      for (int c = 0; c < xyz[it].size(); c++) {
	
	// Distance from camera center to xyz for the individually aligned cameras
	double len = norm_2(aux_cams[it].camera_center() - xyz[it][c]);
	len = len / world_scale;
	Vector3 trans_xyz = sfm_cams[it].camera_center()
	  + len * sfm_cams[it].pixel_to_vector(pix[it][c]);
	for (int row = 0; row < in_pts.rows(); row++) {
	  in_pts(row, col)  = trans_xyz[row];
	  out_pts(row, col) = xyz[it][c][row];
	}
	
	col++;
      }
    }
  }
  
  if (col != num_pts) 
    vw_throw( LogicErr() << "Book-keeping failure in aligning cameras to ground.\n");

  // The initial transform to world coordinates
  Vector<double> C;
  vw::math::find_3D_transform(in_pts, out_pts, rotation, translation, scale);

  // Copy into C
  transform_to_vector(C, rotation, translation, scale);

  // Form the pixel vector
  int pixel_vec_len = 0;
  for (size_t it = 0; it < pix.size(); it++) {
    bool is_good = (xyz[it].size() >= 3);
    if (is_good)
      pixel_vec_len += pix[it].size() * 2;
  }
  Vector<double> pixel_vec;
  pixel_vec.set_size(pixel_vec_len);
  int count = 0;
  for (size_t it = 0; it < pix.size(); it++) {
    bool is_good = (xyz[it].size() >= 3);
    if (is_good) {
      for (size_t c = 0; c < pix[it].size(); c++) {
	Vector2 pixel = pix[it][c];
	pixel_vec[2*count  ] = pixel[0];
	pixel_vec[2*count+1] = pixel[1];
	count++;
      }
    }
  }
  if (2*count != pixel_vec_len)
    vw_throw( LogicErr() << "Book-keeping failure in cam_gen.\n");
  
  // Optimize the transform
  double abs_tolerance  = 1e-24;
  double rel_tolerance  = 1e-24;
  int    max_iterations = 2000;
  int status = 0;
  CameraSolveRotTransScale<PinholeModel> lma_model(xyz, pixel_vec, sfm_cams);
  Vector<double> final_params
    = vw::math::levenberg_marquardt(lma_model, C, pixel_vec,
				    status, abs_tolerance, rel_tolerance,
				    max_iterations);

  Vector<double>  final_residual = lma_model(final_params, verbose);
  
  // Bring the cameras to world coordinates
  for (int it = 0; it < num_cams; it++) 
    apply_rot_trans_scale(sfm_cams[it], final_params);

  // Unpack the final vector into a rotation + translation + scale
  vector_to_transform(final_params, rotation, translation, scale);

}

/// Take an interest point from a map projected image and convert it
/// to the corresponding IP in the original non-map-projected image.
/// - Return false if the pixel could not be converted.
bool asp::projected_ip_to_raw_ip(vw::ip::InterestPoint &P,
                                 vw::ImageViewRef<vw::PixelMask<double>> const& interp_dem,
                                 asp::CameraModelPtr camera_model,
                                 vw::cartography::GeoReference const& georef,
                                 vw::cartography::GeoReference const& dem_georef) {
  // Get IP coordinate in the DEM
  Vector2 pix(P.x, P.y);
  Vector2 ll      = georef.pixel_to_lonlat(pix);
  Vector2 dem_pix = dem_georef.lonlat_to_pixel(ll);
  if (!interp_dem.pixel_in_bounds(dem_pix))
    return false;
  // Load the elevation from the DEM
  PixelMask<double> dem_val = interp_dem(dem_pix[0], dem_pix[1]);
  if (!is_valid(dem_val))
    return false;
  Vector3 llh(ll[0], ll[1], dem_val.child());
  Vector3 xyz = dem_georef.datum().geodetic_to_cartesian(llh);

  // Project into the camera
  Vector2 cam_pix;
  try {
   cam_pix = camera_model->point_to_pixel(xyz);
  } catch(...) {
    return false; // Don't update the point.
  }
  P.x  = cam_pix.x();
  P.y  = cam_pix.y();
  P.ix = P.x;
  P.iy = P.y;
  return true;
}

// This function takes advantage of the fact that when it is called the cam_ptrs have the same
//  information as is in param_storage!
void apply_transform_to_cameras_optical_bar(vw::Matrix4x4 const& M,
                                            asp::BAParams & param_storage,
                                            std::vector<asp::CameraModelPtr> const& cam_ptrs){

  // Convert the transform format
  vw::Matrix3x3 R = submatrix(M, 0, 0, 3, 3);
  vw::Vector3   T;
  for (int r = 0; r < 3; r++) 
    T[r] = M(r, 3);
  
  double scale = pow(det(R), 1.0/3.0);
  for (size_t r = 0; r < R.rows(); r++)
    for (size_t c = 0; c < R.cols(); c++)
      R(r, c) /= scale;

  for (unsigned i = 0; i < param_storage.num_cameras(); i++) {

    // Apply the transform
    boost::shared_ptr<vw::camera::OpticalBarModel> bar_ptr = 
      boost::dynamic_pointer_cast<vw::camera::OpticalBarModel>(cam_ptrs[i]);
    bar_ptr->apply_transform(R, T, scale);

    // Write out to param_storage
    pack_optical_bar_to_arrays(*bar_ptr, i, param_storage);    
  }

} // end function apply_transform_to_cameras_pinhole

// Given an input pinhole camera and param changes, apply those, returning
// the new camera. Note that all intrinsic parameters are stored as multipliers
// in asp::BAParams.
vw::camera::PinholeModel transformedPinholeCamera(int camera_index,
                                                  asp::BAParams const& param_storage,
                                                  vw::camera::PinholeModel const& in_cam) {

  // Start by making a copy of the camera. Note that this does not make a copy of the
  // distortion params, as that's a pointer. So will have to make a copy of it further down.
  vw::camera::PinholeModel out_cam = in_cam;

  double const* pos_pose_ptr   = param_storage.get_camera_ptr(camera_index);
  double const* center_ptr     = param_storage.get_intrinsic_center_ptr    (camera_index);
  double const* focus_ptr      = param_storage.get_intrinsic_focus_ptr     (camera_index);
  double const* distortion_ptr = param_storage.get_intrinsic_distortion_ptr(camera_index);

  // Update position and pose
  CameraAdjustment pos_pose_info(pos_pose_ptr);
  out_cam.set_camera_center(pos_pose_info.position());
  out_cam.set_camera_pose  (pos_pose_info.pose    ());

  // Update the lens distortion parameters. Note how we make a new copy of the distortion object.
  boost::shared_ptr<LensDistortion> distortion = out_cam.lens_distortion()->copy();
  vw::Vector<double> lens = distortion->distortion_parameters();
  for (size_t i=0; i<lens.size(); ++i)
    lens[i] *= distortion_ptr[i];
  distortion->set_distortion_parameters(lens);
  out_cam.set_lens_distortion(distortion.get());

  // Update the center and focus
  Vector2 old_center = out_cam.point_offset();
  Vector2 old_focus  = out_cam.focal_length();
  out_cam.set_point_offset(Vector2(center_ptr[0]*old_center[0],
                                  center_ptr[1]*old_center[1]), false);
  double new_focus = old_focus[0]*focus_ptr[0];
  out_cam.set_focal_length(Vector2(new_focus,new_focus), true); // Recompute internals.
  
  return out_cam;
}

// Given an input optical bar camera and param changes, apply those, returning
// the new camera.
vw::camera::OpticalBarModel transformedOpticalBarCamera(int camera_index,
                                                        asp::BAParams const& param_storage,
                                                        vw::camera::OpticalBarModel const& in_cam) {
  
  // Start by making a copy of the camera.
  vw::camera::OpticalBarModel out_cam = in_cam;

  double const* pos_pose_ptr  = param_storage.get_camera_ptr(camera_index);
  double const* center_ptr    = param_storage.get_intrinsic_center_ptr    (camera_index);
  double const* focus_ptr     = param_storage.get_intrinsic_focus_ptr     (camera_index);
  double const* intrinsic_ptr = param_storage.get_intrinsic_distortion_ptr(camera_index);

  // Update position and pose
  CameraAdjustment pos_pose_info(pos_pose_ptr);
  out_cam.set_camera_center(pos_pose_info.position());
  out_cam.set_camera_pose  (pos_pose_info.pose    ());

  // All intrinsic parameters are stored as multipliers!

  // Update the other intrinsic parameters.
  out_cam.set_speed              (out_cam.get_speed()*intrinsic_ptr[0]);
  out_cam.set_motion_compensation(out_cam.get_motion_compensation()*intrinsic_ptr[1]);
  out_cam.set_scan_time          (out_cam.get_scan_time()*intrinsic_ptr[2]);

  // Update the center and focus
  Vector2 old_center = out_cam.get_optical_center();
  float   old_focus  = out_cam.get_focal_length();
  out_cam.set_optical_center(Vector2(center_ptr[0]*old_center[0],
                                    center_ptr[1]*old_center[1]));
  double new_focus = old_focus*focus_ptr[0];
  out_cam.set_focal_length(new_focus);

  return out_cam;
}


// Save convergence angle percentiles for each image pair having matches
void asp::saveConvergenceAngles(std::string const& conv_angles_file,
                                std::vector<asp::MatchPairStats> const& convAngles,
                                std::vector<std::string> const& imageFiles) {

  vw_out() << "Writing: " << conv_angles_file << "\n";
  std::ofstream ofs (conv_angles_file.c_str());
  ofs << "# Convergence angle percentiles (in degrees) for each image pair having matches\n";
  ofs << "# left_image right_image 25% 50% 75% num_angles_per_pair\n";
  ofs.precision(17);
  for (size_t conv_it = 0; conv_it < convAngles.size(); conv_it++) {
    auto const & c = convAngles[conv_it]; // alias
    ofs << imageFiles[c.left_cam_index] << ' ' << imageFiles[c.right_cam_index] << ' '
        << c.val25 << ' ' << c.val50 << ' '  << c.val75 << ' ' << c.num_vals << "\n";
  }
  ofs.close();

  return;
}

// Mapproject interest points onto a DEM and find the norm of their
// disagreement in meters. It is assumed that dem_georef
// was created by bilinear interpolation. The cameras must be with
// the latest adjustments applied to them.
void asp::calcPairMapprojOffsets(std::vector<asp::CameraModelPtr> const& optimized_cams,
                                 int left_cam_index, int right_cam_index,
                                 std::vector<vw::ip::InterestPoint> const& left_ip,
                                 std::vector<vw::ip::InterestPoint> const right_ip,
                                 vw::cartography::GeoReference const& dem_georef,
                                 vw::ImageViewRef<vw::PixelMask<double>> interp_mapproj_dem,
                                 std::vector<vw::Vector<double, 4>> & mapprojPoints,  // will append
                                 std::vector<double> & mapproj_offsets) {
  // Wipe the output
  mapproj_offsets.clear();
  // Will append to mapprojPoints, so don't wipe it
  
  for (size_t ip_it = 0; ip_it < left_ip.size(); ip_it++) {
    
    bool treat_nodata_as_zero = false;
    bool has_intersection = false;
    double height_error_tol = 0.001; // 1 mm should be enough
    double max_abs_tol      = 1e-14; // abs cost fun change b/w iterations
    double max_rel_tol      = 1e-14;
    int num_max_iter        = 50;   // Using many iterations can be very slow
    Vector3 xyz_guess;
    
    Vector2 left_pix(left_ip[ip_it].x, left_ip[ip_it].y);
    Vector3 left_dem_xyz = vw::cartography::camera_pixel_to_dem_xyz
      (optimized_cams[left_cam_index]->camera_center(left_pix),
       optimized_cams[left_cam_index]->pixel_to_vector(left_pix),
       interp_mapproj_dem, dem_georef, treat_nodata_as_zero, has_intersection,
       height_error_tol, max_abs_tol, max_rel_tol, num_max_iter, xyz_guess);
    if (!has_intersection) 
      continue;
    
    // Do the same for right. Use left pixel as initial guess
    xyz_guess = left_dem_xyz;
    Vector2 right_pix(right_ip[ip_it].x, right_ip[ip_it].y);
    Vector3 right_dem_xyz = vw::cartography::camera_pixel_to_dem_xyz
      (optimized_cams[right_cam_index]->camera_center(right_pix),
       optimized_cams[right_cam_index]->pixel_to_vector(right_pix),
       interp_mapproj_dem, dem_georef, treat_nodata_as_zero, has_intersection,
       height_error_tol, max_abs_tol, max_rel_tol, num_max_iter, xyz_guess);
    if (!has_intersection) 
      continue;

    Vector3 mid_pt = (left_dem_xyz + right_dem_xyz)/2.0;
    double dist = norm_2(left_dem_xyz - right_dem_xyz);

    // Keep in the same structure both the midpoint between these two mapprojected ip,
    // and their distance, as later the bookkeeping of mapproj_offsets will be different.
    Vector<double, 4> point;
    subvector(point, 0, 3) = mid_pt;
    point[3] = dist;
    
    mapprojPoints.push_back(point);
    mapproj_offsets.push_back(dist);
  }
}

// Save mapprojected matches offsets for each image pair having matches
void asp::saveMapprojOffsets(std::string const& mapproj_offsets_stats_file,
                             std::string const& mapproj_offsets_file,
                             vw::cartography::GeoReference const& mapproj_dem_georef,
                             std::vector<vw::Vector<double, 4>> const & mapprojPoints,
                             std::vector<asp::MatchPairStats> const& mapprojOffsets,
                             std::vector<std::vector<double>> & mapprojOffsetsPerCam, 
                             std::vector<std::string> const& imageFiles) {
  
  vw_out() << "Writing: " << mapproj_offsets_stats_file << "\n";
  std::ofstream ofs (mapproj_offsets_stats_file.c_str());

  ofs << "# Percentiles of distances between mapprojected matching pixels in an "
      << "image and the others.\n";
  ofs << "# image_name 25% 50% 75% count\n";
  for (size_t image_it = 0; image_it < imageFiles.size(); image_it++) {
    auto & vals = mapprojOffsetsPerCam[image_it]; // alias
    int len = vals.size();
    double val25 = -1.0, val50 = -1.0, val75 = -1.0, count = 0;
    if (!vals.empty()) {
      std::sort(vals.begin(), vals.end());
      val25 = vals[0.25 * len];
      val50 = vals[0.50 * len];
      val75 = vals[0.75 * len];
      count = len;
    }

    ofs << imageFiles[image_it] << ' '
        << val25 << ' ' << val50 << ' ' << val75 << ' ' << count << "\n";
  }

  ofs << "# Percentiles of distances between matching pixels after mapprojecting onto DEM.\n"
      << "# Per image pair and measured in DEM pixel units.\n";
  ofs << "# left_image right_image 25% 50% 75% num_matches_per_pair\n";
  ofs.precision(17);
  for (size_t conv_it = 0; conv_it < mapprojOffsets.size(); conv_it++) {
    auto const & c = mapprojOffsets[conv_it]; // alias
    ofs << imageFiles[c.left_cam_index] << ' ' << imageFiles[c.right_cam_index] << ' '
        << c.val25 << ' ' << c.val50 << ' '  << c.val75 << ' ' << c.num_vals << "\n";
  }

  ofs.close();

  vw_out() << "Writing: " << mapproj_offsets_file << "\n";
  ofs = std::ofstream(mapproj_offsets_file.c_str());
  ofs.precision(17);
  ofs << "# lon, lat, height_above_datum, mapproj_ip_dist_meters\n";
  ofs << "# " << mapproj_dem_georef.datum() << std::endl;

  // Write all the points to the file
  for (size_t it = 0; it < mapprojPoints.size(); it++) {
    
    Vector3 xyz = subvector(mapprojPoints[it], 0, 3);
    Vector3 llh = mapproj_dem_georef.datum().cartesian_to_geodetic(xyz);
    
    ofs << llh[0] << ", " << llh[1] <<", " << llh[2] << ", "
         << mapprojPoints[it][3] << std::endl;
  }
  
  ofs.close();
  
  return;
}

// Calculate convergence angles. Remove the outliers flagged earlier,
// if remove_outliers is true. Compute offsets of mapprojected matches,
// if a DEM is given. These are done together as they rely on
// reloading interest point matches, which is expensive so the matches
// are used for both operations.
void asp::matchFilesProcessing(vw::ba::ControlNetwork const& cnet,
                               asp::BaBaseOptions const& opt,
                               std::vector<asp::CameraModelPtr> const& optimized_cams,
                               bool remove_outliers, std::set<int> const& outliers,
                               std::vector<asp::MatchPairStats> & convAngles,
                               std::string const& mapproj_dem,
                               std::vector<vw::Vector<double, 4>> & mapprojPoints,
                               std::vector<asp::MatchPairStats> & mapprojOffsets,
                               std::vector<std::vector<double>> & mapprojOffsetsPerCam) {

  mapprojPoints.clear();
  convAngles.clear();
  mapprojOffsets.clear();
  mapprojOffsetsPerCam.clear();

  bool save_mapproj_match_points_offsets = (!mapproj_dem.empty());
  vw::cartography::GeoReference mapproj_dem_georef;
  ImageViewRef<PixelMask<double>> interp_mapproj_dem;
  if (save_mapproj_match_points_offsets)
    asp::create_interp_dem(mapproj_dem, mapproj_dem_georef, interp_mapproj_dem);
  
  int num_cameras = opt.image_files.size();
  mapprojOffsetsPerCam.resize(num_cameras);
  
  // Work on individual image pairs
  for (auto match_it = opt.match_files.begin(); match_it != opt.match_files.end(); match_it++) {

    std::vector<double> mapproj_offsets;

    // IP from the control network, for which we flagged outliers
    std::vector<vw::ip::InterestPoint> left_ip, right_ip;

    std::pair<int, int> cam_pair   = match_it->first;
    std::string         match_file = match_it->second;
    size_t left_index  = cam_pair.first;
    size_t right_index = cam_pair.second;

    // Just skip over match files that don't exist.
    if (!boost::filesystem::exists(match_file)) {
      vw_out() << "Skipping non-existent match file: " << match_file << std::endl;
      continue;
    }

    // Read the original IP, to ensure later we write to disk only
    // the subset of the IP from the control network which
    // are part of these original ones. 
    std::vector<ip::InterestPoint> orig_left_ip, orig_right_ip;
    ip::read_binary_match_file(match_file, orig_left_ip, orig_right_ip);

    // Create a new convergence angle storage struct
    convAngles.push_back(asp::MatchPairStats()); // add an element, will populate it soon
    asp::MatchPairStats & convAngle = convAngles.back(); // alias
    std::vector<double> sorted_angles;

    asp::MatchPairStats * mapprojOffset = NULL; // may not always exist
    if (save_mapproj_match_points_offsets) {
      mapprojOffsets.push_back(asp::MatchPairStats()); // add an elem
      mapprojOffset = &mapprojOffsets.back(); // pointer
    }
    
    if (!remove_outliers) {
      asp::convergence_angles(optimized_cams[left_index].get(), optimized_cams[right_index].get(),
                              orig_left_ip, orig_right_ip, sorted_angles);
      convAngle.populate(left_index, right_index, sorted_angles);

      if (mapprojOffset != NULL) {
        asp::calcPairMapprojOffsets(optimized_cams,
                                    left_index, right_index,
                                    orig_left_ip, orig_right_ip,
                                    mapproj_dem_georef, interp_mapproj_dem,  
                                    mapprojPoints, // will append here
                                    mapproj_offsets);
        mapprojOffset->populate(left_index, right_index, mapproj_offsets);
        for (size_t map_it = 0; map_it < mapproj_offsets.size(); map_it++) {
          mapprojOffsetsPerCam[left_index].push_back(mapproj_offsets[map_it]);
          mapprojOffsetsPerCam[right_index].push_back(mapproj_offsets[map_it]);
        }
      }
      
      // Since no outliers are removed, nothing else to do
      continue;
    }
    
    typedef std::tuple<double, double, double> Triplet;
    std::map<Triplet, Triplet> lookup;
    for (size_t ip_iter = 0; ip_iter < orig_left_ip.size(); ip_iter++) {
      // Note how we store both the camera index and the ip coordinates
      Triplet left_set(left_index, orig_left_ip[ip_iter].x, orig_left_ip[ip_iter].y); 
      Triplet right_set(right_index, orig_right_ip[ip_iter].x, orig_right_ip[ip_iter].y); 
      lookup[left_set] = right_set;
    }

    // Iterate over the control network, and, for each control point,
    // look only at the measure for left_index and right_index
    int ipt = -1;
    for (ControlNetwork::const_iterator iter = cnet.begin(); iter != cnet.end(); iter++) {

      ipt++; // control point index

      // Skip gcp
      if (cnet[ipt].type() == ControlPoint::GroundControlPoint) {
        continue;
      }
        
      bool has_left = true, has_right = false;
      ip::InterestPoint cnet_left_ip, cnet_right_ip;
      for (ControlPoint::const_iterator measure = (*iter).begin();
            measure != (*iter).end(); ++measure) {
        if (measure->image_id() == left_index) {
          has_left = true;
          cnet_left_ip = ip::InterestPoint(measure->position()[0], measure->position()[1],
                                   measure->sigma()[0]);
        }else if (measure->image_id() == right_index) {
          has_right = true;
          cnet_right_ip = ip::InterestPoint(measure->position()[0], measure->position()[1],
                                   measure->sigma()[0]);
        }
      }

      // Keep only ip for these two images
      if (!has_left || !has_right)
        continue;

      if (outliers.find(ipt) != outliers.end())
        continue; // skip outliers

      // Only add ip that were there originally
      Triplet left_set(left_index, cnet_left_ip.x, cnet_left_ip.y); 
      Triplet right_set(right_index, cnet_right_ip.x, cnet_right_ip.y); 
      if (lookup.find(left_set) == lookup.end() || lookup[left_set] != right_set)
        continue;
      
      left_ip.push_back (cnet_left_ip);
      right_ip.push_back(cnet_right_ip);
    }
    
    // Filter by disparity
    // TODO(oalexan1): Remove this param. Use instead --outlier-removal-params.
    // Not sure this code should be here to start with. Note that it
    // does not update the outliers set.
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
    
    if (num_cameras == 2){
      // Compute the coverage fraction
      Vector2i right_image_size = file_image_size(opt.image_files[1]);
      int right_ip_width = right_image_size[0]*
        static_cast<double>(100.0 - std::max(opt.ip_edge_buffer_percent, 0))/100.0;
      Vector2i ip_size(right_ip_width, right_image_size[1]);
      double ip_coverage = asp::calc_ip_coverage_fraction(right_ip, ip_size);
      // Careful with the line below, it gets used in process_icebridge_batch.py.
      vw_out() << "IP coverage fraction after cleaning = " << ip_coverage << "\n";
    }

    // Make a clean copy of the file
    std::string clean_match_file = ip::clean_match_filename(match_file);
    if (opt.clean_match_files_prefix != "") {
      // Avoid saving clean-clean.match.
      clean_match_file = match_file;
      // Write the clean match file in the current dir, not where it was read from
      clean_match_file.replace(0, opt.clean_match_files_prefix.size(), opt.out_prefix);
    }
    else if (opt.match_files_prefix != "") {
      // Write the clean match file in the current dir, not where it was read from
      clean_match_file.replace(0, opt.match_files_prefix.size(), opt.out_prefix);
    }
    
    vw_out() << "Saving " << left_ip.size() << " filtered interest points.\n";

    vw_out() << "Writing: " << clean_match_file << std::endl;
    ip::write_binary_match_file(clean_match_file, left_ip, right_ip);

    // Find convergence angles based on clean ip
    asp::convergence_angles(optimized_cams[left_index].get(), optimized_cams[right_index].get(),
                            left_ip, right_ip, sorted_angles);
    convAngle.populate(left_index, right_index, sorted_angles);

    if (mapprojOffset != NULL) {
      asp::calcPairMapprojOffsets(optimized_cams,
                                  left_index, right_index,
                                  left_ip, right_ip,
                                  mapproj_dem_georef, interp_mapproj_dem,  
                                  mapprojPoints, // will append here
                                  mapproj_offsets);
      mapprojOffset->populate(left_index, right_index, mapproj_offsets);
      for (size_t map_it = 0; map_it < mapproj_offsets.size(); map_it++) {
        mapprojOffsetsPerCam[left_index].push_back(mapproj_offsets[map_it]);
        mapprojOffsetsPerCam[right_index].push_back(mapproj_offsets[map_it]);
      }
    }
    
  } // End loop through the match files
}
