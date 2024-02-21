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

typedef boost::shared_ptr<asp::StereoSession> SessionPtr;

using namespace vw;

namespace asp {

// Load cameras from given image and camera files
void load_cameras(std::vector<std::string> const& image_files,
                  std::vector<std::string> const& camera_files,
                  std::string const& out_prefix, 
                  vw::GdalWriteOptions const& opt,
                  bool approximate_pinhole_intrinsics,
                  // Outputs
                  std::string & stereo_session, // may change
                  bool & single_threaded_cameras,
                  std::vector<boost::shared_ptr<vw::camera::CameraModel>> & camera_models) {

  // Initialize the outputs
  camera_models.clear();
  single_threaded_cameras = false;
  
  if (image_files.size() != camera_files.size()) 
    vw_throw(ArgumentErr() << "Expecting as many images as cameras.\n");  
  
  for (size_t i = 0; i < image_files.size(); i++) {
    vw_out(DebugMessage,"asp") << "Loading: " << image_files [i] << ' '
                               << camera_files[i] << "\n";
    
    // The same camera is double-loaded into the same session instance.
    // TODO: One day replace this with a simpler camera model loader class.
    // But note that this call also refines the stereo session name.
    SessionPtr session
      (asp::StereoSessionFactory::create(stereo_session, opt,
                                         image_files [i], image_files [i],
                                         camera_files[i], camera_files[i],
                                         out_prefix));
    
    camera_models.push_back(session->camera_model(image_files [i],
                                                  camera_files[i]));
    
    // This is necessary to avoid a crash with ISIS cameras which is single-threaded
    if (!session->supports_multi_threading())
      single_threaded_cameras = true;
    
    if (approximate_pinhole_intrinsics) {
      boost::shared_ptr<vw::camera::PinholeModel> pinhole_ptr = 
        boost::dynamic_pointer_cast<vw::camera::PinholeModel>(camera_models.back());
      // Replace lens distortion with fast approximation
      vw::camera::update_pinhole_for_fast_point2pixel<vw::camera::TsaiLensDistortion>
        (*(pinhole_ptr.get()), file_image_size(image_files[i]));
    }
  } // End loop through images loading all the camera models
  
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
  for (size_t i = 0; i < image_files.size(); i++) {

    // This is for the case when there is a mix of pinhole and non-pinhole
    // cameras. In that case, the pinhole cameras will be ignored. Must
    // reset the session to be able to load the non-pinhole cameras.
    if (stereo_session == "pinhole")
      stereo_session = "";

    SessionPtr session(asp::StereoSessionFactory::create(stereo_session, // may change
                                                         vw::GdalWriteOptions(),
                                                         image_files [i], image_files [i],
                                                         camera_files[i], camera_files[i],
                                                         out_prefix)); 
    
    // Pinhole and nadirpinhole cameras do not have a datum
    if (stereo_session == "pinhole" || stereo_session == "nadirpinhole")
      continue;

    bool use_sphere_for_non_earth = true;
    datum = session->get_datum(session->camera_model(image_files [i],
                                                     camera_files[i]).get(),
                               use_sphere_for_non_earth);
    success = true;
    return success; // found the datum
  }

  return success;
}

} // end namespace asp
