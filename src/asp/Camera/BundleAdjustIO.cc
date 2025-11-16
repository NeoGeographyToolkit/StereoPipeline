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
#include <asp/Core/IpMatchingAlgs.h>
#include <asp/Camera/CameraResectioning.h>
#include <asp/Camera/CsmModel.h>
#include <asp/Camera/Covariance.h>
#include <asp/Core/StereoSettings.h>
#include <asp/Core/BaseCameraUtils.h>
#include <asp/Core/ImageUtils.h>
#include <asp/Core/FileUtils.h>
#include <asp/asp_config.h>

#include <vw/BundleAdjustment/CameraRelation.h>
#include <vw/BundleAdjustment/ControlNetwork.h>
#include <vw/BundleAdjustment/ControlNetworkLoader.h>
#include <vw/Cartography/CameraBBox.h>
#include <vw/InterestPoint/MatcherIO.h>
#include <vw/FileIO/KML.h>
#include <vw/Stereo/StereoModel.h>
#include <vw/Math/Statistics.h>
#include <vw/Math/Geometry.h>
#include <vw/Math/Functors.h>
#include <vw/Camera/OpticalBarModel.h>
#include <vw/Camera/CameraUtilities.h>
#include <vw/Camera/LensDistortion.h>

#include <boost/random/uniform_int_distribution.hpp>

#include <string>

#if defined(ASP_HAVE_PKG_ISIS) && ASP_HAVE_PKG_ISIS == 1
#include <asp/IsisIO/IsisInterface.h>
#endif // ASP_HAVE_PKG_ISIS

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
void saveCameraReport(asp::BaBaseOptions const& opt, asp::BAParams const& param_storage,
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

} // end namespace asp
