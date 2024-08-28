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


/// \file CameraUtils.h
/// Camera utilities that need the stereo session

#ifndef __STEREO_SESSION_CAMERA_UTILS_H__
#define __STEREO_SESSION_CAMERA_UTILS_H__

#include <vector>
#include <string>

#include <vw/Camera/CameraModel.h>
#include <vw/Math/Transform.h>
#include <vw/FileIO/GdalWriteOptions.h>
#include <vw/FileIO/DiskImageView.h>

namespace vw {
  namespace cartography {
    class Datum;
  }
  namespace ip {
    class InterestPoint;
  }
}

namespace asp {

class StereoSession;
typedef boost::shared_ptr<StereoSession> SessionPtr;

// Load cameras from given image and camera files
void load_cameras(std::vector<std::string> const& image_files,
                  std::vector<std::string> const& camera_files,
                  std::string const& out_prefix, 
                  vw::GdalWriteOptions const& opt,
                  bool approximate_pinhole_intrinsics,
                  // Outputs
                  std::string & stereo_session, // may change
                  bool & single_threaded_cameras,
                  std::vector<vw::CamPtr> & camera_models);

// Guess the based on camera position. Usually one arrives here for pinhole
// cameras.
bool guessDatum(double cam_center_radius, vw::cartography::Datum & datum);
  
// Find the datum based on cameras. Return true on success.
bool datum_from_camera(std::string const& image_file, 
                       std::string const& camera_file,
                       std::string & stereo_session, // may change
                       asp::SessionPtr & session, // may be null on input
                       // Outputs
                       vw::cartography::Datum & datum);

// Given a list of stereo prefixes, extract some info about them
typedef std::vector<boost::shared_ptr<vw::DiskImageView<vw::PixelMask<vw::Vector2f>>>> DispVec;
void parseStereoRuns(std::string              const& prefix_file,
                     std::vector<std::string> const& all_image_files,
                     // Outputs
                     std::vector<int>              & left_indices,
                     std::vector<int>              & right_indices,
                     std::vector<asp::SessionPtr>  & sessions,
                     std::vector<vw::TransformPtr> & left_trans,
                     std::vector<vw::TransformPtr> & right_trans,
                     DispVec                       & disparities);

} // end namespace asp

#endif // __STEREO_SESSION_CAMERA_UTILS_H__
