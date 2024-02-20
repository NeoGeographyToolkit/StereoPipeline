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


/// \file Camerautils.h
/// Camera utilities that need the stereo session

#ifndef __STEREO_SESSION_CAMERAUTILS_H__
#define __STEREO_SESSION_CAMERAUTILS_H__

#include <vector>
#include <string>

#include <vw/Camera/CameraModel.h>
#include <vw/FileIO/GdalWriteOptions.h>

namespace vw {
  namespace cartography {
    class Datum;
  }
  namespace ip {
    class InterestPoint;
  }
}

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
                  std::vector<boost::shared_ptr<vw::camera::CameraModel>> & camera_models);
  
// Find the datum based on cameras. Return true on success.
bool datum_from_cameras(std::vector<std::string> const& image_files,
                        std::vector<std::string> const& camera_files,
                        std::string & stereo_session, // may change
                        // Outputs
                        vw::cartography::Datum & datum);

} // end namespace asp

#endif // __STEREO_SESSION_CAMERAUTILS_H__
