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

#include <vw/Core/Exception.h>
#include <vw/Camera/PinholeModel.h>
#include <vw/Camera/CameraUtilities.h>
#include <vw/InterestPoint/InterestData.h>
#include <vw/Core/Stopwatch.h>

#include <omp.h>
#include <string>
#include <iostream>

using namespace vw;

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
        (*(pinhole_ptr.get()), file_image_size(image_file));
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
  std::cout << "Loading cameras elapsed time: " << sw.elapsed_seconds() << " seconds.\n";
  return;
}

// Guess the based on camera position. Usually one arrives here for pinhole
// cameras.
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
      vw_out() << "Guessed the datum from camera position.\n";
  }
  
  return success;
}

// This can catch user mistakes
void checkDatumConsistency(vw::cartography::Datum const& datum1,                        
                           vw::cartography::Datum const& datum2,
                           bool warn_only) {

  double err1 = std::abs(datum1.semi_major_axis() - datum2.semi_major_axis());
  double err2 = std::abs(datum1.semi_minor_axis() - datum2.semi_minor_axis());
  double err = std::max(err1, err2);
  
  if (err >= 1e-6) {
    std::ostringstream oss;
    oss.precision(8);
    oss << "Found mis-matched datums. The difference in semi-axes is: " 
        << err << " meters.\n"
        << "Datum 1: " << datum1 << "\n"
        << "Datum 2: " << datum2 << "\n";
    if (err < 500.0 || warn_only) // this is mild
       vw::vw_out(vw::WarningMessage) << oss.str();
    else // this is severe
      vw::vw_throw(vw::ArgumentErr() << oss.str());
  }
}

} // end namespace asp
