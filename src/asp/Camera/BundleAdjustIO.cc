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

/// \file BundleAdjustIO.cc
///

#include <asp/Camera/BundleAdjustCamera.h>
#include <asp/Camera/BundleAdjustOptions.h>
#include <asp/Camera/CameraResectioning.h>
#include <asp/Camera/CsmModel.h>
#include <asp/Camera/Covariance.h>
#include <asp/Camera/LinescanUtils.h>
#include <asp/Camera/RPC_XML.h>
#include <asp/Camera/RPCModel.h>
#include <asp/Camera/RPCModelGen.h>
#include <asp/Camera/CameraErrorPropagation.h>
#include <asp/Core/IpMatchingAlgs.h>
#include <asp/Core/StereoSettings.h>
#include <asp/Core/BaseCameraUtils.h>
#include <asp/Core/ImageUtils.h>
#include <asp/Core/FileUtils.h>

#include <asp/asp_config.h>
#if defined(ASP_HAVE_PKG_ISIS) && ASP_HAVE_PKG_ISIS == 1
#include <asp/IsisIO/IsisInterface.h>
#endif // ASP_HAVE_PKG_ISIS

#include <vw/BundleAdjustment/CameraRelation.h>
#include <vw/BundleAdjustment/ControlNetwork.h>
#include <vw/BundleAdjustment/ControlNetworkLoader.h>
#include <vw/Cartography/CameraBBox.h>
#include <vw/InterestPoint/MatcherIO.h>
#include <vw/Math/Statistics.h>
#include <vw/Math/Geometry.h>
#include <vw/Math/Functors.h>
#include <vw/Camera/PinholeModel.h>
#include <vw/Camera/OpticalBarModel.h>
#include <vw/Camera/CameraUtilities.h>
#include <vw/Camera/LensDistortion.h>
#include <vw/Core/Stopwatch.h>
#include <vw/Core/StringUtils.h>

#include <usgscsm/UsgsAstroLsSensorModel.h>

#include <string>

using namespace vw;
using namespace vw::camera;
using namespace vw::ba;

namespace asp {


// Save mapprojected matches offsets for each image pair having matches
void saveMapprojOffsets(
     std::string                       const& out_prefix,
     vw::cartography::GeoReference     const& mapproj_dem_georef,
     std::vector<vw::Vector<float, 4>> const& mapprojPoints,
     std::vector<asp::MatchPairStats>  const& mapprojOffsets,
     std::vector<std::vector<float>>        & mapprojOffsetsPerCam,
     std::vector<std::string>          const& imageFiles) {
  
  std::string mapproj_offsets_stats_file 
    = out_prefix + "-mapproj_match_offset_stats.txt";
  vw_out() << "Writing: " << mapproj_offsets_stats_file << "\n";
  std::ofstream ofs (mapproj_offsets_stats_file.c_str());
  ofs.precision(8); // 8 digits of precision for errors is enough

  ofs << "# Percentiles of distances between mapprojected matching pixels in an "
      << "image and the others.\n";
  ofs << "# image_name 25% 50% 75% 85% 95% count\n";
  for (size_t image_it = 0; image_it < imageFiles.size(); image_it++) {
    auto & vals = mapprojOffsetsPerCam[image_it]; // alias
    int len = vals.size();
    float val25 = -1.0, val50 = -1.0, val75 = -1.0, val85 = -1.0, val95 = -1.0, count = 0;
    if (!vals.empty()) {
      std::sort(vals.begin(), vals.end());
      val25 = vals[0.25 * len];
      val50 = vals[0.50 * len];
      val75 = vals[0.75 * len];
      val85 = vals[0.85 * len];
      val95 = vals[0.95 * len];
      count = len;
    }

    ofs << imageFiles[image_it] << ' '
        << val25 << ' ' << val50 << ' ' << val75 << ' '
        << val85 << ' ' << val95 << ' ' << count << "\n";
  }
  ofs.close();

  std::string mapproj_offsets_pair_stats_file 
    = out_prefix + "-mapproj_match_offset_pair_stats.txt";
  vw::vw_out() << "Writing: " << mapproj_offsets_pair_stats_file << "\n";
  ofs = std::ofstream(mapproj_offsets_pair_stats_file.c_str());

  ofs << "# Percentiles of distances between matching pixels after mapprojecting onto DEM.\n"
      << "# Per image pair and measured in DEM pixel units.\n";
  ofs << "# left_image right_image 25% 50% 75% 85% 95% num_matches_per_pair\n";
  ofs.precision(8); // 8 digits of precision for errors is enough
  for (size_t conv_it = 0; conv_it < mapprojOffsets.size(); conv_it++) {
    auto const & c = mapprojOffsets[conv_it]; // alias
    ofs << imageFiles[c.left_cam_index] << ' ' << imageFiles[c.right_cam_index] << ' '
        << c.val25 << ' ' << c.val50 << ' ' << c.val75 << ' '
        << c.val85 << ' ' << c.val95 << ' ' << c.num_vals << "\n";
  }
  ofs.close();

  std::string mapproj_offsets_file = out_prefix + "-mapproj_match_offsets.txt";
  vw_out() << "Writing: " << mapproj_offsets_file << "\n";
  ofs = std::ofstream(mapproj_offsets_file.c_str());
  // 12 digits of precision for errors is enough. 
  // That is 9 digits after decimal period for lon and lat.
  ofs.precision(12); 
  ofs << "# lon, lat, height_above_datum, mapproj_ip_dist_meters\n";
  ofs << "# " << mapproj_dem_georef.datum() << std::endl;

  // Write all the points to the file
  for (size_t it = 0; it < mapprojPoints.size(); it++) {
    Vector3 llh = subvector(mapprojPoints[it], 0, 3);
    ofs << llh[0] << ", " << llh[1] <<", " << llh[2] << ", "
         << mapprojPoints[it][3] << std::endl;
  }
  
  ofs.close();
  
  return;
}


// Save pinhole camera positions and orientations in a single file.
// Only works with Pinhole cameras.
void saveCameraReport(asp::BaBaseOptions const& opt, asp::BaParams const& param_storage,
                      vw::cartography::Datum const& datum, 
                      std::string const& prefix) {

  std::string output_path = opt.out_prefix + "-" + prefix + "-cameras.csv";
  vw_out() << "Writing: " << output_path << std::endl;
  std::ofstream fh(output_path.c_str());
  fh.precision(17);
  fh << "# input_cam_file, cam_ctr_x, cam_ctr_y, cam_ctr_z (ecef meters), "
     << "cam2ned rotation rows\n";
  
  int num_cameras = opt.image_files.size();

  // TODO(oalexan1): Create here a report file. Write camera name,
  // camera center, ecef position, ecef quaternion, and ned roll-pitch-yaw.
  // Use same Euler angles as in numpy. Likely eigen can do it.
  for (int icam = 0; icam < num_cameras; icam++) {

    vw::Vector3 cam_ctr;
    vw::Matrix3x3 cam2ecef;
    switch(opt.camera_type) {
      case BaCameraType_Pinhole: {
        // Get the camera model from the original one with parameters in
        // param_storage applied to it (which could be original ones or optimized). 
        // Note that we do not modify the original camera.
        vw::camera::PinholeModel const* in_cam
          = dynamic_cast<vw::camera::PinholeModel const*>(opt.camera_models[icam].get());
        if (in_cam == NULL)
          vw_throw(ArgumentErr() << "Expecting a pinhole camera.\n");
        // Apply current intrinsics and extrinsics to the camera
        vw::camera::PinholeModel out_cam 
          = transformedPinholeCamera(icam, param_storage, *in_cam);
        cam_ctr = out_cam.camera_center(vw::Vector2());
        cam2ecef = out_cam.get_rotation_matrix();
        break;
      }
      case BaCameraType_OpticalBar:
        vw::vw_throw(vw::ArgumentErr() << "Saving a camera report is not implemented "
                    << "for optical bar cameras.\n");
        break;
      case BaCameraType_CSM:
        vw::vw_throw(vw::ArgumentErr() << "Saving a camera report is not implemented "
                      << "for CSM cameras.\n");
        break;
      default: {
        // Apply extrinsics adjustments to a pinhole camera
        // TODO(oalexan1): Make this into a function called adjustedPinholeCamera().
        // Use it where needed.
        CameraAdjustment adjustment(param_storage.get_camera_ptr(icam));
        PinholeModel* in_cam = dynamic_cast<PinholeModel*>(opt.camera_models[icam].get());
        if (in_cam == NULL)
          vw_throw(ArgumentErr() << "Expecting a pinhole camera.\n");
        
        // Make a copy of the camera, and apply the adjustments to the copy. Need to go back
        // to the original camera to get the adjustments needed to apply.
        // TODO(oalexan1): This is a little awkward.
        PinholeModel out_cam = *in_cam;
        AdjustedCameraModel adj_cam(vw::camera::unadjusted_model(opt.camera_models[icam]),
                                    adjustment.position(), adjustment.pose());
        vw::Matrix4x4 ecef_transform = adj_cam.ecef_transform();
        out_cam.apply_transform(ecef_transform);
        cam_ctr = out_cam.camera_center(vw::Vector2());
        cam2ecef = out_cam.get_rotation_matrix();
      }
    }

    fh << opt.camera_files[icam] << ", "
       << cam_ctr[0] << ", " << cam_ctr[1] << ", " << cam_ctr[2];

    // Find the matrix for converting NED to ECEF
    vw::Vector3 loc_llh = datum.cartesian_to_geodetic(cam_ctr);
    vw::Matrix3x3 ned2ecef = datum.lonlat_to_ned_matrix(loc_llh);

    // How a camera moves relative to the world is given by the camera-to-world
    // matrix. That is a little counter-intuitive.
    vw::Matrix3x3 cam2ned = inverse(ned2ecef) * cam2ecef;
    for (int row = 0; row < cam2ned.rows(); row++) {
      for (int col = 0; col < cam2ned.cols(); col++) {
        fh << ", " << cam2ned(row, col);
      } 
    }
    fh << "\n";
  }

  fh.close();
  return;
}


// Save stats of horizontal and vertical errors propagated from cameras
// to triangulation
void saveHorizVertErrors(std::string const& horiz_vert_errors_file,
                         std::vector<asp::HorizVertErrorStats> const& horizVertErrors,
                         std::vector<std::string> const& imageFiles) {

  vw_out() << "Writing: " << horiz_vert_errors_file << "\n";
  std::ofstream ofs (horiz_vert_errors_file.c_str());
  ofs.precision(8);
  ofs << "# Horizontal and vertical propagated triangulation uncertainties (in meters) for each image pair having matches.\n";
  ofs << "# left_image right_image horiz_error_median vert_error_median horiz_error_mean vert_error_mean horiz_error_stddev vert_error_stddev num_meas\n";
  for (size_t conv_it = 0; conv_it < horizVertErrors.size(); conv_it++) {
     auto const & c = horizVertErrors[conv_it]; // alias
     ofs << imageFiles[c.left_cam_index] << ' ' << imageFiles[c.right_cam_index] << ' '
         << c.horiz_error_median << ' ' << c.vert_error_median << ' '
         << c.horiz_error_mean   << ' ' << c.vert_error_mean   << ' '
         << c.horiz_error_stddev << ' ' << c.vert_error_stddev << ' '
         << c.num_errors << "\n";
  }
  ofs.close();

  return;
} 


/// Write a pinhole camera file to disk after updating the intrinsics and
/// extrinsics. Return the path to the saved file.
std::string savePinholeCam(asp::BaBaseOptions const& opt, int icam,
                           vw::cartography::Datum const& datum,
                           asp::BaParams const& param_storage) {

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

  #pragma omp critical
  {
    // Ensure this text is not messed up when writing in parallel
    vw::vw_out() << "Writing: " << cam_file << "\n";
    bool has_datum = (datum.name() != asp::UNSPECIFIED_DATUM);
    if (has_datum)
      vw::vw_out() << std::setprecision(8)
                   << "Camera center for " << cam_file << ": "
                   << datum.cartesian_to_geodetic(out_cam.camera_center())
                   << " (longitude, latitude, height above datum(m))\n\n";
  }
    
  out_cam.write(cam_file);
  
  return cam_file;
}


/// Write an optical bar camera file to disk after updating the intrinsics and
// extrinsics. Return the path to the saved file.
std::string saveOpticalBarCam(asp::BaBaseOptions const& opt, int icam,
                              vw::cartography::Datum const& datum,
                              asp::BaParams const& param_storage) {

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
  
  #pragma omp critical
  {
    // Ensure this text is not messed up when writing in parallel
    vw::vw_out() << "Writing: " << cam_file << "\n";
    bool has_datum = (datum.name() != asp::UNSPECIFIED_DATUM);
    if (has_datum)
      vw::vw_out() << std::setprecision(8)
                   << "Camera center for " << cam_file << ": "
                   << datum.cartesian_to_geodetic(out_cam.camera_center())
                   << " (longitude, latitude, height above datum(m))\n\n";
  }
  
  out_cam.write(cam_file);
  
  return cam_file;
}


// Write a CSM camera file to disk. Assumes that the intrinsics are optimized.
// Return the path to the saved file.
std::string saveCsmCamUpdateIntr(asp::BaBaseOptions const& opt, int icam,
                                 vw::cartography::Datum const& datum,
                                 asp::BaParams const& param_storage) {

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
     
  #pragma omp critical
  {
    // Ensure this text is not messed up when writing in parallel
    bool has_datum = (datum.name() != asp::UNSPECIFIED_DATUM);
    if (has_datum)
      vw::vw_out() << std::setprecision(8)
                   << "Camera center for " << cam_file << ": "
                   << datum.cartesian_to_geodetic(out_cam->camera_center(vw::Vector2()))
                   << " (longitude, latitude, height above datum(m))\n";
    vw::vw_out() << "Writing: " << cam_file << "\n";
  }
  
  // Save the updated state
  out_cam->saveState(cam_file);

  if (opt.update_isis_cubes_with_csm_state) {
    // Save the CSM state to the image file. Wipe any spice info.
    std::string image_name = opt.image_files[icam]; 
    std::string plugin_name = out_cam->plugin_name();
    std::string model_name  = out_cam->model_name();
    std::string model_state = out_cam->model_state();
    #pragma omp critical
    {
      // Ensure this text is not messed up when writing in parallel
      vw::vw_out() << "Adding updated CSM state to image file: " << image_name << "\n";
    }
#if defined(ASP_HAVE_PKG_ISIS) && ASP_HAVE_PKG_ISIS == 1
    asp:isis::saveCsmStateToIsisCube(image_name, plugin_name, model_name, model_state);
#endif // ASP_HAVE_PKG_ISIS
  }
  
  return cam_file;
}

// Save convergence angle percentiles for each image pair having matches
void saveConvergenceAngles(std::string const& conv_angles_file,
                           std::vector<asp::MatchPairStats> const& convAngles,
                           std::vector<std::string> const& imageFiles) {

  vw_out() << "Writing: " << conv_angles_file << "\n";
  std::ofstream ofs (conv_angles_file.c_str());
  ofs.precision(8);
  ofs << "# Convergence angle percentiles (in degrees) for each image pair having matches\n";
  ofs << "# left_image right_image 25% 50% 75% num_matches\n";
  for (size_t conv_it = 0; conv_it < convAngles.size(); conv_it++) {
    auto const & c = convAngles[conv_it]; // alias
    ofs << imageFiles[c.left_cam_index] << ' ' << imageFiles[c.right_cam_index] << ' '
        << c.val25 << ' ' << c.val50 << ' ' << c.val75  << ' ' << c.num_vals << "\n";
  }
  ofs.close();

  return;
}


// Read image and camera lists. Can have several comma-separated lists
// in image_list and camera_list, when sharing intrinsics per sensor.
void read_image_cam_lists(std::string const& image_list, 
                std::string const& camera_list,
                std::vector<std::string> & images,
                std::vector<std::string> & cameras,
                asp::IntrinsicOptions & intrinsics_opts) {

  // Wipe the output
  images.clear();
  cameras.clear();
  intrinsics_opts.share_intrinsics_per_sensor = false;
  intrinsics_opts.cam2sensor.clear();
  intrinsics_opts.num_sensors = 0; // must be initialized to zero

  // See if we have a single list or multiple lists
  if (image_list.find(",") == std::string::npos && 
      camera_list.find(",") == std::string::npos) {
    // Single list, so just read the lists as usual, and return
    asp::read_list(image_list, images);

    // Use the images as cameras if no camera list is provided. Sometimes the
    // images have camera information.
    if (!camera_list.empty())
      asp::read_list(camera_list, cameras);
    if (cameras.empty())
      cameras = images;
    
    // There must be as many images as cameras
    if (images.size() != cameras.size())
      vw_throw(ArgumentErr() << "Expecting the same number of images and cameras.\n");
      
    return;
  }
  
  // This when we have multiple image lists and camera lists, for when
  // we solve for intrinsics per sensor.
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
  vw_out() << "Number of sensors: " << intrinsics_opts.num_sensors << "\n";

  // Must have the same number of cameras as images
  if (images.size() != cameras.size())
    vw_throw(ArgumentErr() << "Expecting the same number of images and cameras.\n");
    
  return;
} 


// Write an updated csm camera state file to disk. Assumes no intrinsics are optimized.
std::string saveUpdatedCsm(asp::BaBaseOptions const& opt, int icam,
                           std::string const& adjustFile, 
                           asp::BaParams const& param_storage) {

  // Get the unadjusted CSM model and the adjustment as a transform
  CameraAdjustment cam_adjust(param_storage.get_camera_ptr(icam));
  AdjustedCameraModel adj_cam(vw::camera::unadjusted_model(opt.camera_models[icam]),
                              cam_adjust.position(), cam_adjust.pose());
  vw::Matrix4x4 ecef_transform = adj_cam.ecef_transform();
  std::string csmFile          = asp::csmStateFile(adjustFile);
  asp::CsmModel * csm_model    
    = asp::csm_model(vw::camera::unadjusted_model(opt.camera_models[icam]),
                                                opt.stereo_session);

  // Apply the adjustment and save a transformed copy of the camera model
  boost::shared_ptr<asp::CsmModel> out_cam;
  csm_model->deep_copy(out_cam);
  out_cam->applyTransform(ecef_transform);
  #pragma omp critical
  {
    // Ensure this text is not messed up when writing in parallel
    vw::vw_out() << "Writing: " << csmFile << "\n";
  }
  out_cam->saveState(csmFile);

  if (opt.update_isis_cubes_with_csm_state) {
    // Save the CSM state to the image file. Wipe any spice info.
    std::string image_name = opt.image_files[icam]; 
    std::string plugin_name = out_cam->plugin_name();
    std::string model_name  = out_cam->model_name();
    std::string model_state = out_cam->model_state();
    #pragma omp critical
    {
      // Ensure this text is not messed up when writing in parallel
      vw::vw_out() << "Adding updated CSM state to image file: " << image_name << "\n";
    }
    
#if defined(ASP_HAVE_PKG_ISIS) && ASP_HAVE_PKG_ISIS == 1
    asp:isis::saveCsmStateToIsisCube(image_name, plugin_name, model_name, model_state);
#endif // ASP_HAVE_PKG_ISIS
  }
  
  return csmFile;
}


// Write an updated RPC camera file to disk. Assumes no intrinsics are optimized.
std::string saveUpdatedRpc(asp::BaBaseOptions const& opt, int icam,
                           std::string const& adjustFile, 
                           asp::BaParams const& param_storage) {
  
  std::string imageFile = opt.image_files[icam];
  vw::DiskImageView<float> image(imageFile);
  BBox2 image_box = vw::bounding_box(image);
  
  CameraAdjustment cam_adjust(param_storage.get_camera_ptr(icam));
  AdjustedCameraModel adj_cam(vw::camera::unadjusted_model(opt.camera_models[icam]),
                              cam_adjust.position(), cam_adjust.pose());
  
  vw::Matrix4x4 ecef_transform = adj_cam.ecef_transform();
  std::string rpcFile = asp::rpcAdjustedFile(adjustFile);
  
  // Get the underlying RPC model
  vw::CamPtr unadjCam = vw::camera::unadjusted_model(opt.camera_models[icam]);
  asp::RPCModel * rpc = dynamic_cast<asp::RPCModel*>(unadjCam.get());
  if (rpc == NULL)
    vw_throw(ArgumentErr() << "Expecting an RPC camera.\n");
  
  #pragma omp critical
  {
    // Ensure this text is not messed up when writing in parallel
    vw::vw_out() << "Saving adjusted RPC model: " << rpcFile << "\n";
  }

  // Produced a transformed copy of the RPC model. This can be slow.
  double pixel_err = 0.0;
  asp::RPCModel trans_rpc = asp::transformRpc(*rpc, ecef_transform, image_box, pixel_err);

  #pragma omp critical
  {
    vw::vw_out() << "Discrepancy between the initial RPC model with the external adjustment "
      << "and the refit model incorporating the adjustment for image: " 
      << imageFile << " is " << pixel_err  << " pixels.\n";
    if (pixel_err > 1.0)
       vw::vw_out(vw::WarningMessage) << "The adjusted RPC model is not accurate enough. "
        << "Use the original RPC model with external adjustments as applied via "
        << "--bundle-adjust-prefix. Test the original model self-consistency with "
        << "cam_test with --height-above-datum within valid RPC height range.\n";
  }

  trans_rpc.saveXML(rpcFile);
  
  return rpcFile;
}


// Write a camera adjustment file to disk, and potentially a camera file with
// the adjustments applied to it. Return the path to the saved file.
std::string saveAdjustedCam(asp::BaBaseOptions const& opt, int icam,
                            asp::BaParams const& param_storage) {

  std::string adjust_file = asp::bundle_adjust_file_name(opt.out_prefix,
                                                         opt.image_files[icam],
                                                         opt.camera_files[icam]);

  #pragma omp critical
  {
    // Ensure this text is not messed up when writing in parallel
    vw::vw_out() << "Writing: " << adjust_file << "\n";
  }

  // The cam_file will be overwritten below for CSM cameras
  CameraAdjustment cam_adjust(param_storage.get_camera_ptr(icam));
  asp::write_adjustments(adjust_file, cam_adjust.position(), 
                                    cam_adjust.pose());

  std::string cam_file = adjust_file;
    
  // For CSM camera models export, in addition, the JSON state with the
  // adjustment applied to it. This applies when not solving for intrinsics and
  // using CSM. Do something analogous for RPC.
  if (opt.stereo_session == "csm" || opt.stereo_session == "pleiades" ||
      opt.stereo_session == "dg"  ||
      (opt.stereo_session == "aster" && asp::stereo_settings().aster_use_csm))
    cam_file = saveUpdatedCsm(opt, icam, adjust_file, param_storage);
  else if (opt.stereo_session == "rpc" && opt.save_adjusted_rpc)
    cam_file = saveUpdatedRpc(opt, icam, adjust_file, param_storage);
  
  return cam_file;
}


// Save the updated camera model to disk. Return the name of the file written.
std::string saveUpdatedCamera(asp::BaBaseOptions const& opt, 
                              asp::BaParams const& param_storage,
                              int icam) {

  // Must have a try block, as otherwise OpenMP crashes the program
  // as the caller seems unable to catch the exception from threads.
  std::string cam_file; 
  try {

    switch (opt.camera_type) {
    case BaCameraType_Pinhole:
      cam_file = savePinholeCam(opt, icam, opt.datum, param_storage);
      break;
    case BaCameraType_OpticalBar:
      cam_file = saveOpticalBarCam(opt, icam, opt.datum, param_storage);
      break;
    case BaCameraType_CSM:
      // When solving for intrinsics and using CSM
      cam_file = saveCsmCamUpdateIntr(opt, icam, opt.datum, param_storage);
      break;
    case BaCameraType_Other:
      // This includes the CSM/pinhole/etc cases when not solving for intrinsics
      cam_file = saveAdjustedCam(opt, icam, param_storage);
      break;
    default:
      vw::vw_throw(vw::ArgumentErr() << "Unknown camera type.\n");
    }
  } catch (const std::exception& e) {
    vw::vw_out() << e.what() << "\n";
  }
  
  return cam_file;
}


// Write updated camera models to disk
void saveUpdatedCameras(asp::BaBaseOptions const& opt, 
                        asp::BaParams const& param_storage) {
  
  int num_cameras = opt.image_files.size();
  std::vector<std::string> cam_files(num_cameras);
  vw::Stopwatch sw;
  sw.start();
  
  // For pinhole and nadirpinhole sessions, save the cameras sequentially, as
  // some info is printed along the way and can get messed up if done in
  // parallel.
  if (!opt.single_threaded_cameras && !opt.update_isis_cubes_with_csm_state &&
      opt.stereo_session.find("pinhole") == std::string::npos) {
    #pragma omp parallel for 
    for (int icam = 0; icam < num_cameras; icam++)
      cam_files[icam] = saveUpdatedCamera(opt, param_storage, icam);
  } else {
    for (int icam = 0; icam < num_cameras; icam++)
      cam_files[icam] = saveUpdatedCamera(opt, param_storage, icam);
  }

  sw.stop();
  vw::vw_out() << "Saving cameras elapsed time: " << sw.elapsed_seconds() << " seconds.\n";

  // Write the image lists
  std::string img_list_file = opt.out_prefix + "-image_list.txt";
  vw::vw_out() << "Writing: " << img_list_file << "\n";
  asp::write_list(img_list_file, opt.image_files);
  
  // Write the camera lists
  std::string cam_list_file = opt.out_prefix + "-camera_list.txt";
  vw::vw_out() << "Writing: " << cam_list_file << "\n";
  asp::write_list(cam_list_file, cam_files);
  
  return;
}


// Save the CSM cameras. It is assumed there are no external adjustments
// applied to them.
void saveCsmCameras(std::string const& out_prefix,
                    std::string const& stereo_session, 
                    std::vector<std::string> const& image_files,
                    std::vector<std::string> const& camera_files,
                    std::vector<vw::CamPtr>  const& camera_models,
                    bool update_isis_cubes_with_csm_state) {

  int num_cameras = camera_models.size();
  std::vector<std::string> cam_files(num_cameras);
  for (int icam = 0; icam < num_cameras; icam++) {
    std::string adjustFile = asp::bundle_adjust_file_name(out_prefix,
                                                          image_files[icam],
                                                          camera_files[icam]);
    std::string csmFile = asp::csmStateFile(adjustFile);
    asp::CsmModel * csm_cam 
      = asp::csm_model(camera_models[icam], stereo_session);
    vw::vw_out() << "Writing: " << csmFile << "\n";
    csm_cam->saveState(csmFile);
    cam_files[icam] = csmFile;

    if (update_isis_cubes_with_csm_state) {
      // Save the CSM state to the image file. Wipe any spice info.
      std::string image_name = image_files[icam]; 
      std::string plugin_name = csm_cam->plugin_name();
      std::string model_name  = csm_cam->model_name();
      std::string model_state = csm_cam->model_state();
      vw::vw_out() << "Adding updated CSM state to image file: " << image_name << "\n";
#if defined(ASP_HAVE_PKG_ISIS) && ASP_HAVE_PKG_ISIS == 1
      asp:isis::saveCsmStateToIsisCube(image_name, plugin_name, model_name, model_state);
#endif // ASP_HAVE_PKG_ISIS
    }
  }
  
  // Write the image lists
  std::string img_list_file = out_prefix + "-image_list.txt";
  vw::vw_out() << "Writing: " << img_list_file << std::endl;
  asp::write_list(img_list_file, image_files);
  
  // Write the camera lists
  std::string cam_list_file = out_prefix + "-camera_list.txt";
  vw::vw_out() << "Writing: " << cam_list_file << std::endl;
  asp::write_list(cam_list_file, cam_files);
  
}


/// Load all of the reference disparities specified in the input text file
/// and store them in the vectors.  Return the number loaded.
int load_reference_disparities(std::string const& disp_list_filename,
                               std::vector<vw::ImageView<vw::PixelMask<vw::Vector2f>>> &
                                  disp_vec,
                               std::vector<vw::ImageViewRef<vw::PixelMask<vw::Vector2f>>> &
                                  interp_disp) {
  // TODO: Disparities can be large, but if small it is better to
  // read them in memory.
  std::istringstream is(disp_list_filename);
  std::string disp_file;
  while (is >> disp_file) {
    if (disp_file != "none") {
      vw::vw_out() << "Reading: " << disp_file << std::endl;
      disp_vec.push_back(copy(vw::DiskImageView<vw::PixelMask<vw::Vector2f>>(disp_file)));
    }else{
      // Read in an empty disparity
      disp_vec.push_back(vw::ImageView<vw::PixelMask<vw::Vector2f>>());
    }
    interp_disp.push_back(interpolate(disp_vec.back(),
                                      vw::BilinearInterpolation(), 
                                      vw::ConstantEdgeExtension()));
  }
  return static_cast<int>(disp_vec.size());
}

} // end namespace asp
