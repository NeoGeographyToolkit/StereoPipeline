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

/// Camera utilities that need the stereo session

/// \file CameraUtils.cc

#include <asp/Sessions/CameraUtils.h>
#include <asp/Sessions/StereoSessionFactory.h>
#include <asp/Core/AspStringUtils.h>
#include <asp/Core/StereoSettings.h>
#include <asp/Core/DemUtils.h>

#include <vw/Cartography/Map2CamTrans.h>
#include <vw/Core/Exception.h>
#include <vw/Camera/PinholeModel.h>
#include <vw/Camera/CameraUtilities.h>
#include <vw/InterestPoint/InterestData.h>
#include <vw/Core/Stopwatch.h>

#include <boost/filesystem.hpp>
#include <omp.h>

#include <string>
#include <iostream>

namespace fs = boost::filesystem;

namespace asp {

void load_camera(std::string const& image_file,
                 std::string const& camera_file,
                 std::string const& out_prefix,
                 vw::GdalWriteOptions const& opt,
                 bool approximate_pinhole_intrinsics,
                 bool quiet,
                 // Outputs
                 asp::SessionPtr & session,
                 std::string     & stereo_session,
                 vw::CamPtr      & camera_model,
                 bool            & single_threaded_camera) {

  // Must have a try block, as otherwise OpenMP crashes the program
  // as the caller seems unable to catch the exception from threads.
  try {
    std::string input_dem = ""; // No DEM
    bool allow_map_promote = false;
    if (session.get() == NULL) {
      session.reset(asp::StereoSessionFactory::create
                      (stereo_session, // may change
                      opt, image_file, image_file,
                      camera_file, camera_file,
                      out_prefix, input_dem,
                      allow_map_promote, quiet));
    
      // This is necessary to avoid a crash with ISIS cameras which is single-threaded
      // TODO(oalexan1): Check this value for csm cameras embedded in ISIS images.
      single_threaded_camera = (!session->supports_multi_threading());
    }
    
    bool local_quiet = true; // To not print messages about loading each camera
    camera_model = session->camera_model(image_file, camera_file, local_quiet);
    if (approximate_pinhole_intrinsics) {
      boost::shared_ptr<vw::camera::PinholeModel> pinhole_ptr = 
        boost::dynamic_pointer_cast<vw::camera::PinholeModel>(camera_model);
      // Replace lens distortion with fast approximation
      vw::camera::update_pinhole_for_fast_point2pixel<vw::camera::TsaiLensDistortion>
        (*(pinhole_ptr.get()), vw::file_image_size(image_file));
    }
  } catch (const std::exception& e) {
    vw::vw_out() << e.what() << "\n";
  }
}

// Load cameras from given image and camera files. Load them in parallel except
// for ISIS which is not thread-safe.
void load_cameras(std::vector<std::string> const& image_files,
                  std::vector<std::string> const& camera_files,
                  std::string              const& out_prefix, 
                  vw::GdalWriteOptions     const& opt,
                  bool approximate_pinhole_intrinsics,
                  // Outputs
                  std::string             & stereo_session,
                  bool                    & single_threaded_camera,
                  std::vector<vw::CamPtr> & camera_models) {

  // Print a message as this may take a while
  vw::vw_out() << "Loading the cameras.\n";
  
  vw::Stopwatch sw;
  sw.start();
  
  // Sanity check
  if (image_files.size() != camera_files.size()) 
    vw::vw_throw(vw::ArgumentErr() << "Expecting as many images as cameras.\n");  

  camera_models.resize(image_files.size(), vw::CamPtr(NULL));

  // First invocation. Will create the session. Will update
  // single_threaded_camera, and stereo_session. All subsequent invocations will
  // not change these so will be thread-safe.
  asp::SessionPtr session(NULL); // will change
  single_threaded_camera = false; // may change
  bool quiet = false;
  int i = 0;
  load_camera(image_files[i], camera_files[i], out_prefix, opt,
              approximate_pinhole_intrinsics, quiet,
              session, stereo_session, camera_models[i], single_threaded_camera);

  quiet = true;
  if (!single_threaded_camera) {
    // Use OpenMP to load the cameras in parallel
    #pragma omp parallel for
    for (size_t i = 0; i < image_files.size(); i++)
      load_camera(image_files[i], camera_files[i], out_prefix, opt,
                  approximate_pinhole_intrinsics, quiet,
                  session, stereo_session, camera_models[i], single_threaded_camera);
      
    // Check that the cameras got loaded
    for (size_t i = 0; i < camera_models.size(); i++) {
      if (camera_models[i].get() == NULL) 
        vw::vw_throw(vw::ArgumentErr() << "Failed to load all camera models.\n");
    }  
  } else {
    // Load the cameras one by one. This is needed for ISIS cameras.
    for (size_t i = 0; i < image_files.size(); i++)
      load_camera(image_files[i], camera_files[i], out_prefix, opt,
                  approximate_pinhole_intrinsics, quiet,
                  session, stereo_session, camera_models[i], single_threaded_camera);
  }
  
  sw.stop();
  vw::vw_out() << "Loading cameras elapsed time: " << sw.elapsed_seconds() << " seconds.\n";
  return;
}

// Guess the based on camera position. Usually one arrives here for pinhole
// cameras. This function is to be used when anything else fails.
bool guessDatum(double cam_center_radius, vw::cartography::Datum & datum) {

  bool success = false;
  
  // Datums for Earth, Mars, and Moon
  vw::cartography::Datum earth("WGS84");
  vw::cartography::Datum mars("D_MARS");
  vw::cartography::Datum moon("D_MOON");
  
  double km = 1000.0;
  if (cam_center_radius > earth.semi_major_axis() - 100*km && 
      cam_center_radius < earth.semi_major_axis() + 5000*km) {
    datum = earth;
    success = true;
  } else if (cam_center_radius > mars.semi_major_axis() - 100*km && 
             cam_center_radius < mars.semi_major_axis() + 1500*km) {
    datum = mars;
    success = true;
  } else if (cam_center_radius > moon.semi_major_axis() - 100*km && 
             cam_center_radius < moon.semi_major_axis() + 1000*km) {
    datum = moon;
    success = true;
  }

  return success;
}

// Find the datum based on cameras. Return true on success. Otherwise don't set it.
bool datum_from_camera(std::string const& image_file,
                       std::string const& camera_file, 
                       std::string & stereo_session, // may change
                       asp::SessionPtr & session, // may be null on input
                       // Outputs
                       vw::cartography::Datum & datum) {
  
  std::string out_prefix = "run";

  // Look for a non-pinole camera, as a pinhole camera does not have a datum
  bool success = false;
  double cam_center_radius = 0.0;
  if (session.get() == NULL) {
    // If a session was not passed in, create it here.
    std::string input_dem = ""; // No DEM
    bool allow_map_promote = false, quiet = true;
    session.reset(asp::StereoSessionFactory::create(stereo_session, // may change
                                                vw::GdalWriteOptions(),
                                                image_file, image_file,
                                                camera_file, camera_file,
                                                out_prefix, input_dem,
                                                allow_map_promote, quiet));
  }
    
  bool use_sphere_for_non_earth = true;
  auto cam = session->camera_model(image_file, camera_file);
  datum = session->get_datum(cam.get(), use_sphere_for_non_earth);
  success = session->have_datum(); 

  // TODO(oalexan1): Must have the function get_datum() return success or not.
  // That must be checked at each location. then the block below can be removed.
  if (!success && !asp::stereo_settings().no_datum && !stereo_settings().correlator_mode) {
    double cam_center_radius = norm_2(cam->camera_center(vw::Vector2()));
    success = guessDatum(cam_center_radius, datum);
    if (success) 
      vw::vw_out() << "Guessed the datum from camera position.\n";
  }
  
  return success;
}

// Validate that key exists in the map and the value is a vector of 4 entries,
// each equal to "0".
void validateCropWin(std::string const& key, std::string const& file,  
                     std::map<std::string, std::vector<std::string>> const& vals) {

  if (vals.find(key) == vals.end())
    vw::vw_throw(vw::ArgumentErr() << "Missing entry for " << key << " in " << file << ".\n");
     
  std::vector<std::string> const& v = vals.find(key)->second;
  if (v.size() != 4)
    vw::vw_throw(vw::ArgumentErr() << "Expecting 4 entries for " << key << " in " 
                 << file << ".\n");
  
  for (int i = 0; i < 4; i++)
    if (v[i] != "0")
        vw::vw_throw(vw::ArgumentErr() << "Cannot use a run produced with cropped images.\n");
}

// Given a list of stereo prefixes, extract the indices of the left and right
// images in the full list of images, the alignment transforms, disparities,
// and sessions.
void parseStereoRuns(std::string const& prefix_file,
                     std::vector<std::string> const& all_image_files,
                     // Outputs
                     std::vector<int> & left_indices,
                     std::vector<int> & right_indices,
                     std::vector<asp::SessionPtr>  & sessions,
                     std::vector<vw::TransformPtr> & left_trans,
                     std::vector<vw::TransformPtr> & right_trans,
                     std::vector<std::string>      & disp_files) {

  // Must have at least one stereo run
  if (prefix_file.empty())
    vw::vw_throw(vw::ArgumentErr() << "No list of stereo runs was specified.\n");

  // Wipe the outputs
  left_indices.clear();
  right_indices.clear();
  sessions.clear();
  left_trans.clear();
  right_trans.clear();
  disp_files.clear();
  
  // Read the list of stereo prefixes
  std::vector<std::string> prefix_list;
  asp::read_list(prefix_file, prefix_list);
  if (prefix_list.empty())
    vw::vw_throw(vw::ArgumentErr() << "Expecting at least one stereo prefix.\n");
  
  // For each left and right image in the stereo run, find the index in 
  // the input cameras, and the transforms.
  for (size_t i = 0; i < prefix_list.size(); i++) {
    
    std::string stereo_prefix = prefix_list[i];
    std::string info_file = stereo_prefix + "-info.txt";
    // If it does not exist, the run is invalid
    if (!fs::exists(info_file))
      vw::vw_throw(vw::ArgumentErr() << "Missing: " << info_file 
                     << ". Invalid stereo run.\n");
   
    std::map<std::string, std::vector<std::string>> vals;
    asp::parseKeysVals(info_file, vals);
    
    // Must not have a run with crop wins
    validateCropWin("left_image_crop_win:", info_file, vals);
    validateCropWin("right_image_crop_win:", info_file, vals);
    
    // This DEM is nonempty if the images are mapprojected
    std::string input_dem;
    auto dem_val = vals.find("input_dem:");
    if (dem_val != vals.end() && !dem_val->second.empty())
      input_dem = dem_val->second[0];
    
    // Must have only two images in each run
    auto img_val = vals.find("images:");
    if (img_val == vals.end())
      vw::vw_throw(vw::ArgumentErr() << "Missing left_image_path in " << info_file << ".\n");
    auto images = img_val->second;
    if (images.size() > 2)
      vw::vw_throw(vw::ArgumentErr() << "Cannot handle multiview stereo runs.\n"); 
    if (images.size() != 2)
      vw::vw_throw(vw::ArgumentErr() << "Expecting two images in " << info_file << ".\n");
    
    // Find the camera files
    auto cam_val = vals.find("cameras:");
    if (cam_val == vals.end())
      vw::vw_throw(vw::ArgumentErr() << "Missing camera_files in " << info_file << ".\n");
    auto cameras = cam_val->second;
    if (cameras.size() != 2)
      vw::vw_throw(vw::ArgumentErr() << "Expecting two camera files in " 
                   << info_file << ".\n");
      
    // Find the raw images, for when the images are mapprojected
    std::vector<std::string> raw_images = images; 
    for (size_t i = 0; i < images.size(); i++) {
      
      if (input_dem.empty())
        continue; 
        
      std::string img = images[i];
      // Look up the raw image in the mapprojected image
      std::string img_file_key = "INPUT_IMAGE_FILE";
      std::string raw_img; 
      boost::shared_ptr<vw::DiskImageResource> rsrc(new vw::DiskImageResourceGDAL(img));
      vw::cartography::read_header_string(*rsrc.get(), img_file_key, raw_img);
      // If empty, that's a failure
      if (raw_img.empty())
        vw::vw_throw(vw::ArgumentErr() << "Failed to find the raw image name in "
                     << "the geoheader of " << img << ".\n");
      raw_images[i] = raw_img;
    }
    
    // Each raw image must be among the input images
    int left_index = -1, right_index = -1;
    for (size_t i = 0; i < all_image_files.size(); i++) {
      if (all_image_files[i] == raw_images[0])
        left_index = i;
      if (all_image_files[i] == raw_images[1])
        right_index = i;
    }
    // If not found, that's a failure
    if (left_index < 0 || right_index < 0)
      vw::vw_throw(vw::ArgumentErr() << "Some images for the stereo run: " << stereo_prefix
                   << " are not among the input images for the jitter solver.\n");
    
    auto session_val = vals.find("stereo_session:");
    if (session_val == vals.end() || session_val->second.empty())
      vw::vw_throw(vw::ArgumentErr() << "Missing stereo_session in " << info_file << ".\n");
    std::string stereo_session = session_val->second[0];
    
    auto alignment_val = vals.find("alignment_method:");
    if (alignment_val == vals.end() || alignment_val->second.empty())
      vw::vw_throw(vw::ArgumentErr() << "Missing alignment_method in " << info_file << ".\n");
    std::string curr_alignment_method = alignment_val->second[0];
    // For epipolar alignment, the transforms need a different approach
    if (curr_alignment_method == "epipolar")
      vw::vw_throw(vw::ArgumentErr() 
                   << "Cannot use a run produced with epipolar alignment.\n");
      
    // Temporarily replace the alignment method so we can fetch the transform
    std::string orig_alignment_method = asp::stereo_settings().alignment_method;
    
    asp::stereo_settings().alignment_method = curr_alignment_method;
    
    // The prefix from the info file must agree with the one passed in
    auto prefix_val = vals.find("output_prefix:");
    if (prefix_val == vals.end() || prefix_val->second.empty())
      vw::vw_throw(vw::ArgumentErr() << "Missing output_prefix in " << info_file << ".\n");
    if (prefix_val->second[0] != stereo_prefix)
      vw::vw_throw(vw::ArgumentErr() << "Mismatch between output prefix in " << info_file
                   << " and the stereo prefix.\n");
     
    vw::GdalWriteOptions opt;   
    asp::SessionPtr session
     (asp::StereoSessionFactory::create(stereo_session, // may change
                                       opt, images[0], images[1],
                                       cameras[0], cameras[1],
                                       stereo_prefix, input_dem));
  
     std::string disp_file = stereo_prefix + "-F.tif";
     if (!fs::exists(disp_file))
       vw::vw_throw(vw::ArgumentErr() << "Missing: " << disp_file 
                      << ". Invalid stereo run.\n"); 
    
    // Must not have an -L_cropped.tif file as then cannot use the run.
    // Same for -R_cropped.tif and for L.tsai and R.tsai.
    std::string left_cropped = stereo_prefix + "-L-cropped.tif";
    std::string right_cropped = stereo_prefix + "-R-cropped.tif"; 
    if (fs::exists(left_cropped) || fs::exists(right_cropped))
      vw::vw_throw(vw::ArgumentErr()
                   << "Cannot use a run produced with cropped images.\n");
    std::string left_tsai = stereo_prefix + "-L.tsai";
    std::string right_tsai = stereo_prefix + "-R.tsai";
    if (fs::exists(left_tsai) || fs::exists(right_tsai))
      vw::vw_throw(vw::ArgumentErr() 
                   << "Cannot use a run produced with epipolar alignment.\n");
    
    // Add to the indices and transforms
    left_indices.push_back(left_index);
    right_indices.push_back(right_index);
    sessions.push_back(session);
    left_trans.push_back(session->tx_left());
    right_trans.push_back(session->tx_right());
    disp_files.push_back(disp_file);
                                 
    // Put back the original alignment method
    asp::stereo_settings().alignment_method = orig_alignment_method;
  } // end loop through stereo prefixes
  
}

// Computes a Map2CamTrans given a DEM, image, and a sensor model. Can take a
// DEM height instead of a DEM file.
vw::TransformPtr
transformFromMapProject(std::string dem_path,
                        const std::string &img_file_path,
                        vw::CamPtr map_proj_model_ptr,
                        vw::GdalWriteOptions const& options,
                        std::string const& tag, 
                        std::string const& out_prefix, 
                        double dem_height) {

  // If the input DEM is empty, the dem height must not be nan
  if (dem_path.empty() && std::isnan(dem_height))
    vw::vw_throw(vw::ArgumentErr() << "Must provide either a DEM file or a DEM height.");
  // But must have just one
  if (!dem_path.empty() && !std::isnan(dem_height))
    vw::vw_throw(vw::ArgumentErr() 
      << "Must provide either a DEM file or a DEM height, not both.");
  
  vw::DiskImageView<float> img(img_file_path);
  vw::cartography::GeoReference image_georef;
  if (!read_georeference(image_georef, img_file_path))
    vw::vw_throw(vw::ArgumentErr() << "Missing georeference in image: " << img_file_path);

  // For an orthoimage without a DEM, prepare one, unless it was already made.
  // The produced dem will be at location dem_path.
  if (!std::isnan(dem_height))
    asp::setupOrCheckDem(options, img, image_georef, tag, out_prefix, dem_height,
                         dem_path);
  
  // Read in data necessary for the Map2CamTrans object
  vw::cartography::GeoReference dem_georef;
  if (!read_georeference(dem_georef, dem_path))
    vw::vw_throw(vw::ArgumentErr() << "No georeference found in DEM: " << dem_path);
  bool call_from_mapproject = false;
  return vw::TransformPtr(new vw::cartography::Map2CamTrans(map_proj_model_ptr.get(),
                                   image_georef, dem_georef, dem_path,
                                   vw::Vector2(img.cols(), img.rows()),
                                   call_from_mapproject));
}

} // end namespace asp
