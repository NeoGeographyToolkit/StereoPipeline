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

#include <string>
#include <iostream>


using namespace vw;

namespace asp {

void load_camera(std::string const& image_file,
                 std::string const& camera_file,
                 std::string const& out_prefix,
                 vw::GdalWriteOptions const& opt,
                 bool approximate_pinhole_intrinsics,
                 // Outputs
                 std::string & stereo_session,
                 vw::CamPtr  & camera_model,
                 int         & single_threaded_camera) {

  // TODO(oalexan1): Replace this with a simpler camera model loader class.
  // But note that this call also refines the stereo session name.
  asp::SessionPtr session
    (asp::StereoSessionFactory::create(stereo_session, opt,
                                        image_file, image_file,
                                        camera_file, camera_file,
                                        out_prefix));
  
  camera_model = session->camera_model(image_file, camera_file);
  
  // This is necessary to avoid a crash with ISIS cameras which is single-threaded
  // TODO(oalexan1): Check this value for csm cameras embedded in ISIS images.
  single_threaded_camera = (!session->supports_multi_threading());
  
  if (approximate_pinhole_intrinsics) {
    boost::shared_ptr<vw::camera::PinholeModel> pinhole_ptr = 
      boost::dynamic_pointer_cast<vw::camera::PinholeModel>(camera_model);
    // Replace lens distortion with fast approximation
    vw::camera::update_pinhole_for_fast_point2pixel<vw::camera::TsaiLensDistortion>
      (*(pinhole_ptr.get()), file_image_size(image_file));
  }

}

// Load cameras from given image and camera files
void load_cameras(std::vector<std::string> const& image_files,
                  std::vector<std::string> const& camera_files,
                  std::string const& out_prefix, 
                  vw::GdalWriteOptions const& opt,
                  bool approximate_pinhole_intrinsics,
                  // Outputs
                  std::string & stereo_session, // may change
                  bool & single_threaded_camera,
                  std::vector<boost::shared_ptr<vw::camera::CameraModel>> & camera_models) {

  // Initialize the outputs
  camera_models.clear();
  single_threaded_camera = false; // may change
  
  if (image_files.size() != camera_files.size()) 
    vw_throw(ArgumentErr() << "Expecting as many images as cameras.\n");  

  // TODO(oalexan1): Must load one camera first. Then, if it is not single-threaded,
  // load the rest in parallel. This can greatly speed up loading of many cameras.
  // After the first camera is loaded, must populate the vector single_threaded_cameras_vec
  // before continuing with the rest of the cameras.
  
  // Must initialize a vector of outputs, as we plan to load the cameras in parallel,
  // and do not want to have thread-safety issues. Use a vector of ints, not bool,
  // as compiling fails with a vector of bools.
  std::vector<int> single_threaded_cameras_vec(image_files.size(), 
                                               int(single_threaded_camera));
  std::vector<std::string> stereo_sessions(image_files.size(), stereo_session);
  camera_models.resize(image_files.size());
  
  for (size_t i = 0; i < image_files.size(); i++) {
    load_camera(image_files[i], camera_files[i], out_prefix, opt,
                approximate_pinhole_intrinsics,
                stereo_sessions[i], camera_models[i], single_threaded_cameras_vec[i]);

    // Update these based on the camera just loaded
    single_threaded_camera = single_threaded_camera || single_threaded_cameras_vec[i];
    stereo_session = stereo_sessions[i];
  }
  
  return;
}

// Find the datum based on cameras. Return true on success. Otherwise don't set it.
bool datum_from_cameras(std::vector<std::string> const& image_files,
                        std::vector<std::string> const& camera_files,
                        std::string & stereo_session, // may change
                        // Outputs
                        vw::cartography::Datum & datum) {
  
  std::string out_prefix = "run";

  // Look for a non-pinole camera, as a pinhole camera does not have a datum
  bool success = false;
  double cam_center_radius = 0.0;
  for (size_t i = 0; i < image_files.size(); i++) {

    // This is for the case when there is a mix of pinhole and non-pinhole
    // cameras. In that case, the pinhole cameras will be ignored. Must
    // reset the session to be able to load the non-pinhole cameras.
    if (stereo_session == "pinhole")
      stereo_session = "";

    std::string input_dem = ""; // No DEM
    bool allow_map_promote = false, quiet = true;
    asp::SessionPtr 
      session(asp::StereoSessionFactory::create(stereo_session, // may change
                                                vw::GdalWriteOptions(),
                                                image_files [i], image_files [i],
                                                camera_files[i], camera_files[i],
                                                out_prefix, input_dem,
                                                allow_map_promote, quiet));
    
    auto cam = session->camera_model(image_files[i], camera_files[i]);
    cam_center_radius = norm_2(cam->camera_center(vw::Vector2()));
    
    // Pinhole and nadirpinhole cameras do not have a datum
    if (stereo_session == "pinhole" || stereo_session == "nadirpinhole")
      continue;

    bool use_sphere_for_non_earth = true;
    datum = session->get_datum(cam.get(), use_sphere_for_non_earth);
    success = true;
    return success; // found the datum
  }

  // Guess the based on camera position. Usually one arrives here for pinhole
  // cameras.

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
  
  if (success)
    vw_out() << "Guessed the datum from camera position.\n";

  return success;
}

} // end namespace asp
