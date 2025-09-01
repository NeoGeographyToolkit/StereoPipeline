// __BEGIN_LICENSE__
//  Copyright (c) 2009-2025, United States Government as represented by the
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

// \file BundleAdjustSession.h
// Bundle adjustment functions that need StereoSession

#ifndef __BUNDLE_ADJUST_SESSION_H__
#define __BUNDLE_ADJUST_SESSION_H__

#include <vw/Cartography/GeoReference.h>
#include <vw/Image/ImageViewRef.h>
#include <vw/Image/PixelMask.h>

#include <boost/shared_ptr.hpp>
#include <string>

namespace vw {
  namespace camera {
    class CameraModel;
  }
}

namespace asp {

class BaOptions;
class StereoSession;
typedef boost::shared_ptr<StereoSession> SessionPtr;

// A wrapper around ip matching. Can also work with NULL cameras.
void ba_match_ip(asp::BaOptions & opt, asp::SessionPtr session,
                 std::string const& image1_path,  std::string const& image2_path,
                 vw::camera::CameraModel* cam1,   vw::camera::CameraModel* cam2,
                 std::string const& match_filename);

// Create matches among the mapprojected images (or use any such matches created
// beforehand manually by the user), and undo the mapprojection. All matches are
// saved to files.
void matches_from_mapproj_images(int i, int j,
                                 asp::BaOptions& opt, asp::SessionPtr session,
                                 std::vector<std::string> const& map_files,
                                 std::string mapproj_dem,
                                 vw::cartography::GeoReference const& dem_georef,
                                 vw::ImageViewRef<vw::PixelMask<double>> & interp_dem,
                                 std::string const& match_filename);

void findPairwiseMatches(asp::BaOptions & opt, // will change
                         std::vector<std::string> const& map_files,
                         std::string const& mapproj_dem,
                         std::vector<vw::Vector3> const& estimated_camera_gcc,
                         bool need_no_matches);

} // end namespace asp

#endif // __BUNDLE_ADJUST_SESSION_H__
