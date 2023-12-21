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

/// \file BundleAdjustCamera2.cc
///

// Move some code here because BundleAdjustCamera.cc is getting too big.
// These are all very related functions that are not easy to split into
// separate files by functionality.
// TODO(oalexan1): Move most of BundleAdjustCamera.h code to here, and put it
// all in the asp namespace. 

#include <asp/Camera/BundleAdjustCamera.h>
#include <asp/Camera/CsmModel.h>
#include <asp/Core/IpMatchingAlgs.h>         // Lightweight header
#include <asp/Camera/LinescanUtils.h>
#include <asp/Camera/RPC_XML.h>
#include <asp/Camera/CameraErrorPropagation.h>

#include <vw/Camera/PinholeModel.h>
#include <vw/Camera/OpticalBarModel.h>
#include <boost/algorithm/string.hpp>

#include <string>

using namespace vw;
using namespace vw::camera;
using namespace vw::ba;

namespace asp {

/// This is for the BundleAdjustmentModel class where the camera parameters
/// are a rotation/offset that is applied on top of the existing camera model.
/// First read initial adjustments, if any, and apply perhaps a pc_align transform.
/// We assume the initial transform was already read and validated.
bool init_cams(asp::BaBaseOptions const& opt, asp::BAParams & param_storage,
    std::string const& initial_transform_file, vw::Matrix<double> const& initial_transform,
    std::vector<boost::shared_ptr<camera::CameraModel> > &new_cam_models) {

  bool cameras_changed = false;
  // Initialize all of the camera adjustments to zero.
  param_storage.init_cams_as_zero();
  const size_t num_cameras = param_storage.num_cameras();

  // Sanity check, must have same number of cameras
  if (num_cameras != opt.camera_models.size())
      vw_throw(ArgumentErr() << "Expecting " << num_cameras << " cameras, got "
                           << opt.camera_models.size() << ".\n");

  // Read the adjustments from a previous run, if present
  if (opt.input_prefix != "") {
    for (size_t icam = 0; icam < num_cameras; icam++) {
      std::string adjust_file
        = asp::bundle_adjust_file_name(opt.input_prefix, opt.image_files[icam],
                                       opt.camera_files[icam]);
      vw_out() << "Reading input adjustment: " << adjust_file << std::endl;
      double * cam_ptr = param_storage.get_camera_ptr(icam);
      CameraAdjustment adjustment;
      adjustment.read_from_adjust_file(adjust_file);
      adjustment.pack_to_array(cam_ptr);
    }
    cameras_changed = true;
  }

  // Get an updated list of camera models
  new_cam_models.resize(num_cameras);
  for (size_t icam = 0; icam < num_cameras; icam++) {
    CameraAdjustment correction(param_storage.get_camera_ptr(icam));
    camera::CameraModel* cam = new camera::AdjustedCameraModel(opt.camera_models[icam],
                                        correction.position(), correction.pose());
    new_cam_models[icam] = boost::shared_ptr<camera::CameraModel>(cam);
  }

  // Apply any initial transform to the pinhole cameras
  if (initial_transform_file != "") {
    if (opt.stereo_session == "csm") {
      double scale = pow(vw::math::det(initial_transform), 1.0/3.0);
      if (std::abs(scale - 1.0) > 1e-6) {
        // TODO(oalexan1): This gives wrong results for now so needs to be sorted out.
        // Likely the only way to apply a scale to a linescan camera is to multiply
        // all camera centers by the scale. Using a rotation and translation center
        // like for AdjustedCameraModel is not enough.
        vw_throw(ArgumentErr()
                 << "CSM camera models do not support applying a transform with a scale.\n");
      }
    }

    // TODO(oalexan1): Does this modify param_storage?
    apply_transform_to_cameras(initial_transform, param_storage, new_cam_models);
    cameras_changed = true;
  }

  // Fill out the new camera model vector
  // TODO(oalexan1): Why the repeated code?
  new_cam_models.resize(num_cameras);
  for (size_t icam = 0; icam < num_cameras; icam++){
    CameraAdjustment correction(param_storage.get_camera_ptr(icam));
    camera::CameraModel* cam = new camera::AdjustedCameraModel(opt.camera_models[icam],
                                        correction.position(), correction.pose());
    new_cam_models[icam] = boost::shared_ptr<camera::CameraModel>(cam);
  }
  return cameras_changed;
}

/// Specialization for pinhole cameras
bool init_cams_pinhole(asp::BaBaseOptions const& opt, asp::BAParams & param_storage,
     std::string const& initial_transform_file, vw::Matrix<double> const& initial_transform,
     std::vector<vw::CamPtr> & new_cam_models) {

  bool cameras_changed = false;
  
  // Copy the camera parameters from the models to param_storage
  const size_t num_cameras = param_storage.num_cameras();

  for (int icam=0; icam < num_cameras; icam++) {
    PinholeModel* pin_ptr = dynamic_cast<PinholeModel*>(opt.camera_models[icam].get());
    vw::vw_out() << "Loading input model: " << *pin_ptr << std::endl;

    // Make a deep copy of the camera, including of the lens distortion
    PinholeModel pin_cam = *pin_ptr;
    
    // Read the adjustments from a previous run, if present
    if (opt.input_prefix != "") {
      std::string adjust_file
        = asp::bundle_adjust_file_name(opt.input_prefix, opt.image_files[icam],
                                       opt.camera_files[icam]);
      vw_out() << "Reading input adjustment: " << adjust_file << std::endl;
      CameraAdjustment adjustment;
      adjustment.read_from_adjust_file(adjust_file);

      // Strictly speaking, it is not necessary to call unadjusted_model(), as
      // in bundle_adjust the input cameras are loaded unadjusted, unlike in stereo.
      AdjustedCameraModel adj_cam(vw::camera::unadjusted_model(opt.camera_models[icam]),
                                  adjustment.position(), adjustment.pose());
      vw::Matrix4x4 ecef_transform = adj_cam.ecef_transform();
      pin_cam.apply_transform(ecef_transform);
    
      cameras_changed = true;
    }
    
    // Apply any initial transform to the pinhole cameras. This may be on top
    // of any initial adjustment. 
    if (initial_transform_file != "") {
      pin_cam.apply_transform(initial_transform);
      cameras_changed = true;
    }
    
    pack_pinhole_to_arrays(pin_cam, icam, param_storage);
  } // End loop through cameras

  // Fill out the new camera model vector
  new_cam_models.resize(num_cameras);
  for (size_t icam = 0; icam < num_cameras; icam++){

    PinholeModel* in_cam  = dynamic_cast<PinholeModel*>(opt.camera_models[icam].get());

    // Start with a copy of the input camera. Then overwrite its parameters.
    // Note that out_cam originally shares the distortion with in_cam, as
    // that one is a pointer. But the second line below takes care of that.
    // This is awkward. An object having members as pointers is not safe.
    PinholeModel* out_cam = new PinholeModel(*in_cam);
    *out_cam = transformedPinholeCamera(icam, param_storage, *in_cam);

    new_cam_models[icam] = vw::CamPtr(out_cam);
  }

  return cameras_changed;
}

// TODO: Share more code with the similar pinhole case.
/// Specialization for optical bar cameras.
bool init_cams_optical_bar(asp::BaBaseOptions const& opt, asp::BAParams & param_storage,
                    std::string const& initial_transform_file, 
                    vw::Matrix<double> const& initial_transform,
                    std::vector<vw::CamPtr> &new_cam_models) {

  if (opt.input_prefix != "")
    vw::vw_throw(vw::ArgumentErr()
                 << "Applying initial adjustments to optical bar cameras "
                 << "and --inline-adjustments is not implemented. "
                 << "Remove this option.\n");

  bool cameras_changed = false;

  // Copy the camera parameters from the models to param_storage
  const size_t num_cameras = param_storage.num_cameras();
  for (int icam=0; icam < num_cameras; icam++) {
    vw::camera::OpticalBarModel* bar_ptr
      = dynamic_cast<vw::camera::OpticalBarModel*>(opt.camera_models[icam].get());
    vw::vw_out() << "Loading input model: " << *bar_ptr << std::endl;
    pack_optical_bar_to_arrays(*bar_ptr, icam, param_storage);
  } // End loop through cameras

  // Apply any initial transform to the pinhole cameras
  if (initial_transform_file != "") {
    apply_transform_to_cameras_optical_bar(initial_transform, param_storage, opt.camera_models);
    cameras_changed = true;
  }

  // Fill out the new camera model vector
  new_cam_models.resize(num_cameras);
  for (size_t icam = 0; icam < num_cameras; icam++) {

    vw::camera::OpticalBarModel* in_cam
      = dynamic_cast<vw::camera::OpticalBarModel*>(opt.camera_models[icam].get());

    // Start with a copy of the input camera, then overwrite its content
    vw::camera::OpticalBarModel* out_cam = new vw::camera::OpticalBarModel(*in_cam); 
    *out_cam = transformedOpticalBarCamera(icam, param_storage, *in_cam);

    new_cam_models[icam] = boost::shared_ptr<camera::CameraModel>(out_cam);
  }

  return cameras_changed;
}

// TODO: Share more code with the similar pinhole case.
/// Specialization for CSM
bool init_cams_csm(asp::BaBaseOptions const& opt, asp::BAParams & param_storage,
                   std::string const& initial_transform_file, 
                   vw::Matrix<double> const& initial_transform,
                   std::vector<vw::CamPtr> &new_cam_models) {

  bool cameras_changed = false;

  // Apply any adjustments inline. Copy the camera parameters from the models to
  // param_storage. Do not copy the adjustments, as they are already applied
  // to the camera proper.
  const size_t num_cameras = param_storage.num_cameras();
  for (int icam = 0; icam < num_cameras; icam++) {
    asp::CsmModel* csm_ptr
        = dynamic_cast<asp::CsmModel*>(opt.camera_models[icam].get());

    // Read the adjustments from a previous run, if present. Apply them
    // inline to the camera model.
    if (opt.input_prefix != "") {
      std::string adjust_file
        = asp::bundle_adjust_file_name(opt.input_prefix, opt.image_files[icam],
                                       opt.camera_files[icam]);
      vw_out() << "Reading input adjustment: " << adjust_file << std::endl;
      CameraAdjustment adjustment;
      adjustment.read_from_adjust_file(adjust_file);

      // Strictly speaking, it is not necessary to call unadjusted_model(), as
      // in bundle_adjust the input cameras are loaded unadjusted, unlike in stereo.
      AdjustedCameraModel adj_cam(vw::camera::unadjusted_model(opt.camera_models[icam]),
                                  adjustment.position(), adjustment.pose());
      vw::Matrix4x4 ecef_transform = adj_cam.ecef_transform();
      csm_ptr->applyTransform(ecef_transform);
    
      cameras_changed = true;
    }

    // This does not copy the camera position and orientation, only the intrinsics    
    pack_csm_to_arrays(*csm_ptr, icam, param_storage); 
  } // End loop through cameras

  // Apply any initial transform to the CSM cameras
  if (initial_transform_file != "") {
    // Apply the transform to the cameras inline. This modifies opt.camera_models.
    // The transform does not get copied to param_storage. Only intrinsics get copied.
    apply_transform_to_cameras_csm(initial_transform, param_storage, opt.camera_models);
    cameras_changed = true;
  }

  // Fill out the new camera model vector
  new_cam_models.resize(num_cameras);
  for (size_t icam = 0; icam < num_cameras; icam++){

    asp::CsmModel* in_cam
      = dynamic_cast<asp::CsmModel*>(opt.camera_models[icam].get());
    if (in_cam == NULL)
      vw_throw(ArgumentErr() << "Expecting a CSM camera.\n");

    auto out_cam = transformedCsmCamera(icam, param_storage, *in_cam);
    new_cam_models[icam] = boost::shared_ptr<camera::CameraModel>(out_cam);
  }

  return cameras_changed;
}

/// Write a pinhole camera file to disk.
void write_pinhole_output_file(asp::BaBaseOptions const& opt, int icam,
                               vw::cartography::Datum const& datum,
                               asp::BAParams const& param_storage) {

  // Get the output file path
  std::string cam_file = asp::bundle_adjust_file_name(opt.out_prefix,
                                                      opt.image_files [icam],
                                                      opt.camera_files[icam]);
  cam_file = boost::filesystem::path(cam_file).replace_extension("tsai").string();

  // Get the camera model from the original one with parameters in
  // param_storage applied to it (which could be original ones or optimized). 
  // Note that we do not modify the original camera.
  vw::camera::PinholeModel const* in_cam
    = dynamic_cast<vw::camera::PinholeModel const*>(opt.camera_models[icam].get());
  if (in_cam == NULL)
    vw_throw(ArgumentErr() << "Expecting a pinhole camera.\n");
  vw::camera::PinholeModel out_cam = transformedPinholeCamera(icam, param_storage, *in_cam);

  vw::vw_out() << "Writing: " << cam_file << std::endl;
  out_cam.write(cam_file);
  vw::vw_out() << "Writing output model: " << out_cam << std::endl;

  bool has_datum = (datum.name() != asp::UNSPECIFIED_DATUM);
  if (has_datum) {
    vw::vw_out() << "Camera center for " << cam_file << ": "
                  << datum.cartesian_to_geodetic(out_cam.camera_center())
                  << " (longitude, latitude, height above datum(m))\n\n";
  }
}


/// Write an optical bar camera file to disk.
void write_optical_bar_output_file(asp::BaBaseOptions const& opt, int icam,
                                   vw::cartography::Datum const& datum,
                                   asp::BAParams const& param_storage) {

  // Get the output file path
  std::string cam_file = asp::bundle_adjust_file_name(opt.out_prefix,
                                                      opt.image_files [icam],
                                                      opt.camera_files[icam]);
  cam_file = boost::filesystem::path(cam_file).replace_extension("tsai").string();

  // Get the final camera model from the original one with optimized
  // parameters applied to it. Note that we do not modify the original
  // camera.
  vw::camera::OpticalBarModel* in_cam
    = dynamic_cast<vw::camera::OpticalBarModel*>(opt.camera_models[icam].get());
  if (in_cam == NULL)
    vw_throw(ArgumentErr() << "Expecting an optical bar camera.\n");
  vw::camera::OpticalBarModel out_cam 
    = transformedOpticalBarCamera(icam, param_storage, *in_cam);
  
  vw::vw_out() << "Writing: " << cam_file << std::endl;
  out_cam.write(cam_file);
  vw::vw_out() << "Writing output model: " << out_cam << std::endl;

  bool has_datum = (datum.name() != asp::UNSPECIFIED_DATUM);
  if (has_datum) {
    vw::vw_out() << "Camera center for " << cam_file << ": "
                  << datum.cartesian_to_geodetic(out_cam.camera_center())
                  << " (longitude, latitude, height above datum(m))\n\n";
  }
}

/// Write a CSM camera file to disk.
void write_csm_output_file(asp::BaBaseOptions const& opt, int icam,
                           vw::cartography::Datum const& datum,
                           asp::BAParams const& param_storage) {

  // Get the output file path
  std::string cam_file = asp::bundle_adjust_file_name(opt.out_prefix,
                                                      opt.image_files [icam],
                                                      opt.camera_files[icam]);
  cam_file = asp::csmStateFile(cam_file);

  // Get the final camera model from the original one with optimized
  // parameters applied to it. Note that we do not modify the original
  // camera.
  asp::CsmModel const* in_cam
    = dynamic_cast<asp::CsmModel const*>(opt.camera_models[icam].get()); 
  if (in_cam == NULL)
    vw_throw(ArgumentErr() << "Expecting a CSM camera.\n");
   boost::shared_ptr<asp::CsmModel> out_cam
     = transformedCsmCamera(icam, param_storage, *in_cam);
   out_cam->saveState(cam_file);

   bool has_datum = (datum.name() != asp::UNSPECIFIED_DATUM);
   if (has_datum)
     vw::vw_out() << "Camera center for " << cam_file << ": "
                   << datum.cartesian_to_geodetic(out_cam->camera_center(vw::Vector2()))
                   << " (longitude, latitude, height above datum(m))\n\n";
}

/// Write a csm camera state file to disk. Assumes no intrinsics are optimized.
void write_csm_output_file_no_intr(asp::BaBaseOptions const& opt, int icam,
                                   std::string const& adjustFile, 
                                   asp::BAParams const& param_storage) {
  
  CameraAdjustment cam_adjust(param_storage.get_camera_ptr(icam));
  
  AdjustedCameraModel adj_cam(vw::camera::unadjusted_model(opt.camera_models[icam]),
                              cam_adjust.position(), cam_adjust.pose());
  
  vw::Matrix4x4 ecef_transform = adj_cam.ecef_transform();
  std::string csmFile          = asp::csmStateFile(adjustFile);
  asp::CsmModel * csm_model    = asp::csm_model(opt.camera_models[icam], opt.stereo_session);
  csm_model->saveTransformedState(csmFile, ecef_transform);
}

// Read image and camera lists. Can have several comma-separated lists
// in image_list and camera_list, when sharing intrinsics per sensor.
void read_image_cam_lists(std::string const& image_list, 
                std::string const& camera_list,
                std::vector<std::string> & images_or_cams,
                asp::IntrinsicOptions & intrinsics_opts) {

  // wipe the output
  images_or_cams.clear();
  intrinsics_opts.share_intrinsics_per_sensor = false;
  intrinsics_opts.cam2sensor.clear();
  intrinsics_opts.num_sensors = 0;

  // See if there comma-separated lists passed in the image list
  if (image_list.find(",") == std::string::npos && 
      camera_list.find(",") == std::string::npos) {
    // Single list, so just read the lists as usual, and return
    asp::read_list(image_list, images_or_cams);
    if (camera_list.empty()) {
      // This is usual for ISIS cameras.
      vw_out() << "An image list was provided but not a camera list.\n";
    } else {
      std::vector<std::string> cams;
      asp::read_list(camera_list, cams);
      if (images_or_cams.size() != cams.size())
        vw_throw(ArgumentErr() << "Expecting the same number of images and cameras.\n");
      for (size_t it = 0; it < cams.size(); it++) 
      images_or_cams.push_back(cams[it]);
    }
    return;
  }
  
  vw_out() << "Multiple image lists and camera lists were passed in. " 
           << "Solving for intrinsics per sensor.\n";

  // This is a very important bit
  intrinsics_opts.share_intrinsics_per_sensor = true;

  std::vector<std::string> image_lists, camera_lists;
  boost::split(image_lists, image_list, boost::is_any_of(","));
  boost::split(camera_lists, camera_list, boost::is_any_of(","));

  if (image_lists.size() != camera_lists.size())
    vw_throw(ArgumentErr() << "Expecting the same number of image and camera lists. "
      << "They must be separated by commas on input.\n");

  // Read separately the images and cameras
  std::vector<std::string> images, cameras;
  for (size_t sensor_it = 0; sensor_it < image_lists.size(); sensor_it++) {
    std::vector<std::string> local_images, local_cameras;
    asp::read_list(image_lists[sensor_it], local_images);
    asp::read_list(camera_lists[sensor_it], local_cameras);
    if (local_images.size() != local_cameras.size() || local_images.empty())
      vw_throw(ArgumentErr() << "Expecting the same positive number of images and cameras "
      << "in lists: '" << image_lists[sensor_it] << "' and '" 
      << camera_lists[sensor_it] << "'.\n");

    // Append to the global lists
    images.insert(images.end(), local_images.begin(), local_images.end());
    cameras.insert(cameras.end(), local_cameras.begin(), local_cameras.end());

    // Create the map from camera index to sensor index
    for (size_t cam_it = 0; cam_it < local_cameras.size(); cam_it++)
      intrinsics_opts.cam2sensor.push_back(sensor_it);
  }

  intrinsics_opts.num_sensors = image_lists.size();
  vw_out() << "Number of sensors: " << intrinsics_opts.num_sensors << std::endl;

  // Append both images and cameras to images_or_cams
  images_or_cams.insert(images_or_cams.end(), images.begin(), images.end());
  images_or_cams.insert(images_or_cams.end(), cameras.begin(), cameras.end());
  return;
} 

// When distortion params are shared, their number must agree
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

/// For each option, the string must include a subset of the entries:
///  "focal_length, optical_center, distortion_params"
/// - Need the extra boolean to handle the case where --intrinsics-to-share
///   is provided as "" in order to share none of them.
void load_intrinsics_options(bool        solve_intrinsics,
                             bool        shared_is_specified,
                             std::string intrinsics_to_float_str, // make a copy
                             std::string intrinsics_to_share_str, // make a copy
                             asp::IntrinsicOptions & intrinsics_options) {

  // Float and share everything unless specific options are provided.
  intrinsics_options.focus_constant      = true;
  intrinsics_options.center_constant     = true;
  intrinsics_options.distortion_constant = true;
  intrinsics_options.focus_shared        = true;
  intrinsics_options.center_shared       = true;
  intrinsics_options.distortion_shared   = true;

  if (((intrinsics_to_float_str != "") || (intrinsics_to_share_str != "")) 
      && !solve_intrinsics) {
    vw_throw( ArgumentErr() << "To be able to specify only certain intrinsics, "
                            << "the option --solve-intrinsics must be on.\n" );
  }

  if (!solve_intrinsics)
    return;
  
  // If the user did not specify which intrinsics to float, float all of them.
  boost::to_lower(intrinsics_to_float_str);
  if (intrinsics_to_float_str == "" || intrinsics_to_float_str == "all")
    intrinsics_to_float_str = "focal_length optical_center other_intrinsics";

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

  // By default solve for everything
  intrinsics_options.focus_constant      = false;
  intrinsics_options.center_constant     = false;
  intrinsics_options.distortion_constant = false;

  if (intrinsics_to_float_str != "") {
    intrinsics_options.focus_constant      = true;
    intrinsics_options.center_constant     = true;
    intrinsics_options.distortion_constant = true;
    // These will be individually changed further down
  }

  // If sharing intrinsics per sensor, the only supported mode is that 
  // the intrinsics are always shared per sensor and never across sensors.
  if (shared_is_specified && !intrinsics_options.share_intrinsics_per_sensor) {
    intrinsics_options.focus_shared      = false;
    intrinsics_options.center_shared     = false;
    intrinsics_options.distortion_shared = false;
  }

  // This is the right place in which to turn 'none' to empty string,
  // which now will mean float nothing.
  if (intrinsics_to_float_str == "none") 
    intrinsics_to_float_str = "";
  // Parse the values  
  std::istringstream is(intrinsics_to_float_str);
  std::string val;
  while (is >> val) {
    if (val == "focal_length")
      intrinsics_options.focus_constant = false;
    else if (val == "optical_center")
      intrinsics_options.center_constant = false;
    else if (val == "other_intrinsics")
      intrinsics_options.distortion_constant = false;
    else
      vw_throw(ArgumentErr() << "Error: Found unknown intrinsic to float: " 
        << val << ".\n");
  }

  // No parsing is done when sharing intrinsics per sensor, per above 
  if (shared_is_specified && !intrinsics_options.share_intrinsics_per_sensor) {
    std::istringstream is2(intrinsics_to_share_str);
    while (is2 >> val) {
      if (val == "focal_length")
        intrinsics_options.focus_shared = true;
      else if (val == "optical_center")
        intrinsics_options.center_shared = true;
      else if (val == "other_intrinsics")
        intrinsics_options.distortion_shared = true;
      else
        vw_throw(ArgumentErr() << "Error: Found unknown intrinsic to share: " 
          << val << ".\n");
    }
  }

  std::string sensor_mode = "(across sensors)";
  if (intrinsics_options.share_intrinsics_per_sensor)
    sensor_mode = "(per sensor)"; // useful clarification

  // These will be useful for a while
  vw_out() << "Sharing focal length " << sensor_mode << " is: "
      << intrinsics_options.focus_shared << std::endl;
  vw_out() << "Sharing optical center " << sensor_mode << " is: "
      << intrinsics_options.center_shared << std::endl;
  vw_out() << "Sharing distortion " << sensor_mode << " is: "
      << intrinsics_options.distortion_shared << std::endl;
  vw_out() << "Floating focal length is: " 
    << !intrinsics_options.focus_constant << std::endl;
  vw_out() << "Floating optical center is: " 
    << !intrinsics_options.center_constant << std::endl;
  vw_out() << "Floating distortion is: " 
    << !intrinsics_options.distortion_constant << std::endl;
} // End function load_intrinsics_options

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
      vw_throw( ArgumentErr() << "Unable to get corner estimate from file: "
                              << opt.camera_files[i] << ".\n" );
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
        vw_throw( ArgumentErr() << "Unable to get corner estimate from file: "
                                << opt.camera_files[j] << ".\n" );
      }

      vw::BBox2 bbox_j; // Convert to BBox
      for (size_t p = 0;  p < lonlat_corners_j.size(); p++)
        bbox_j.grow(lonlat_corners_j[p]);

      // Record the files if the bboxes overlap
      // - TODO: Use polygon intersection instead of bounding boxes!
      if (bbox_i.intersects(bbox_j)) {
        vw_out() << "Predicted overlap between images " << opt.image_files[i]
                 << " and " << opt.image_files[j] << std::endl;
        opt.overlap_list.insert(StringPair(opt.image_files[i], opt.image_files[j]));
        opt.overlap_list.insert(StringPair(opt.image_files[j], opt.image_files[i]));
        ++num_overlaps;
      }
    } // End inner loop through cameras
  } // End outer loop through cameras

  if (num_overlaps == 0)
    vw_throw( ArgumentErr() << "Failed to automatically detect any overlapping images!" );

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

} // end namespace asp
