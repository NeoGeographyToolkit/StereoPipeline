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


/// \file CameraUtils.cc

#include <asp/Sessions/CameraUtils.h>
#include <asp/Sessions/StereoSessionFactory.h>
#include <asp/Camera/CsmModel.h>

#include <vw/FileIO/GdalWriteOptions.h>
#include <vw/Core/Exception.h>
#include <vw/Camera/PinholeModel.h>
#include <vw/Camera/CameraUtilities.h>

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
    
    // Since CERES does numerical differences, it needs high precision in the inputs.
    // Inform about that the CSM cameras, which normally settle for less.
    // TODO(oalexan1): Need to examine other cameras too.
    if (stereo_session == "csm") {
      vw::camera::CameraModel * base_cam = vw::camera::unadjusted_model(camera_models[i]).get();
      asp::CsmModel * csm_cam = dynamic_cast<asp::CsmModel*>(base_cam);
      if (csm_cam == NULL) 
        vw::vw_throw(vw::ArgumentErr() << "Expected a CSM camera model.");
      // When asked to get a 1.0e-12 precision, at most it delivers 1.0e-11.
      csm_cam->setDesiredPrecision(1.0e-12); 
    }
  } // End loop through images loading all the camera models
  
  return;
}

} // end namespace asp
