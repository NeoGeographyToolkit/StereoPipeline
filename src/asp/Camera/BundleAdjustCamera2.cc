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
    for (size_t icam = 0; icam < num_cameras; icam++){
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
    // TODO(oalexan1): This gives wrong results for now so needs to be sorted out.
    // Likely the only way to apply a scale to a linescan camera is to multiply
    // all camera centers by the scale. Using a rotation and translation center
    // like for AdjustedCameraModel is not enough.
    if (opt.stereo_session == "csm") {
      double scale = pow(vw::math::det(initial_transform), 1.0/3.0);
      if (std::abs(scale - 1.0) > 1e-6)
        vw_throw(ArgumentErr()
                 << "CSM camera models do not support applying a transform with a scale.\n");
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
     std::vector<boost::shared_ptr<vw::camera::CameraModel>> & new_cam_models) {

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

    new_cam_models[icam] = boost::shared_ptr<vw::camera::CameraModel>(out_cam);
  }

  return cameras_changed;
}

// TODO: Share more code with the similar pinhole case.
/// Specialization for optical bar cameras.
bool init_cams_optical_bar(asp::BaBaseOptions const& opt, asp::BAParams & param_storage,
                    std::string const& initial_transform_file, 
                    vw::Matrix<double> const& initial_transform,
                    std::vector<boost::shared_ptr<vw::camera::CameraModel>> &new_cam_models) {

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
  for (size_t icam = 0; icam < num_cameras; icam++){

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
                    std::vector<boost::shared_ptr<vw::camera::CameraModel>> &new_cam_models) {
  if (opt.input_prefix != "")
    vw::vw_throw(vw::ArgumentErr()
                 << "Applying initial adjustments to CSM cameras "
                 << "and --inline-adjustments is not implemented. "
                 << "Remove this option.\n");

  bool cameras_changed = false;

  // Copy the camera parameters from the models to param_storage
  const size_t num_cameras = param_storage.num_cameras();
  for (int icam = 0; icam < num_cameras; icam++) {
    asp::CsmModel* csm_ptr
        = dynamic_cast<asp::CsmModel*>(opt.camera_models[icam].get());
    pack_csm_to_arrays(*csm_ptr, icam, param_storage); 
  } // End loop through cameras

  // Apply any initial transform to the CSM cameras
  if (initial_transform_file != "") {
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

} // end namespace asp