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

/// \file SyntheticLinescan.h
///
/// A synthetic linescan model as a wrapper around CSM.

#ifndef __STEREO_CAMERA_SYNTHETIC_LINESCAN_H__
#define __STEREO_CAMERA_SYNTHETIC_LINESCAN_H__

#include <vw/Camera/CameraModel.h>
#include <vw/Image/ImageViewRef.h>

namespace vw {
  namespace cartography {
    class GeoReference;
  }
}

namespace asp {

struct SatSimOptions;

// Create and save a linescan camera with given camera positions and orientations.
// There will be just one of them, as all poses are part of the same linescan camera.
void genLinescanCameras(double orbit_len,     
                        vw::cartography::GeoReference  const & dem_georef,
                        vw::ImageViewRef<vw::PixelMask<float>> dem,
                        std::map<int, vw::Vector3>      const & positions,
                        std::map<int, vw::Matrix3x3>    const & cam2world,
                        std::map<int, vw::Matrix3x3>    const & cam2world_no_jitter,
                        double                                height_guess,
                        // Outputs
                        SatSimOptions                         & opt, 
                        std::vector<std::string>              & cam_names,
                        std::vector<vw::CamPtr>               & cams);

// A function to read Linescan cameras from disk in CSM format. There will
// be just one of them.
void readLinescanCameras(SatSimOptions const& opt, 
    std::vector<std::string> & cam_names,
    std::vector<vw::CamPtr> & cams);

} // end namespace asp


#endif//__STEREO_CAMERA_SYNTHETIC_LINESCAN_H__
