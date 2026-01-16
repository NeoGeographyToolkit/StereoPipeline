// __BEGIN_LICENSE__
//  Copyright (c) 2009-2026, United States Government as represented by the
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

#include <asp/Core/Bathymetry.h>
#include <asp/Core/StereoSettings.h>
#include <asp/Core/FileUtils.h>
#include <vw/Camera/CameraModel.h>
#include <vw/Cartography/BathyStereoModel.h>
#include <vw/Core/Exception.h>
#include <vw/Math/LevenbergMarquardt.h>
#include <vw/Image/ImageViewRef.h>
#include <vw/Image/PixelMask.h>
#include <vw/Image/MaskViews.h>
#include <vw/FileIO/DiskImageView.h>
#include <vw/FileIO/DiskImageUtils.h>

namespace asp {

// Read all bathy data
void read_bathy_data(int num_images,
                     std::string const& bathy_mask_list,
                     std::string const& bathy_plane_files,
                     float refraction_index,
                     vw::BathyData & bathy_data) {
  
  std::vector<std::string> bathy_mask_files;
  if (bathy_mask_list != "")
    asp::read_list(bathy_mask_list, bathy_mask_files);

  if (int(bathy_mask_files.size()) != num_images)
    vw::vw_throw(vw::ArgumentErr() << "The number of bathy masks must agree with "
             << "the number of images.\n");

  vw::read_bathy_masks(bathy_mask_files, bathy_data.bathy_masks);
  vw::readBathyPlanes(bathy_plane_files, num_images, bathy_data.bathy_planes);
  bathy_data.refraction_index = refraction_index;
}

// For stereo will use left and right bathy masks. For bundle adjustment and
// jitter_solve will use a list of masks.
bool doBathy(asp::StereoSettings const& stereo_settings) {
  return (stereo_settings.left_bathy_mask  != "" ||
          stereo_settings.right_bathy_mask != "" ||
          stereo_settings.bathy_mask_list  != "");
}

void bathyChecks(std::string const& session_name,
                 asp::StereoSettings const& stereo_settings,
                 int num_images) {

  if (doBathy(stereo_settings)) {
    // If only the topo cloud needs computing, will use only the info from the
    // left and right bathy masks, so does not need the refraction index and
    // the bathy plane.
    if (stereo_settings.output_cloud_type != "topo") {

      if (stereo_settings.refraction_index <= 1.0)
        vw::vw_throw(vw::ArgumentErr() << "The water index of refraction to be used in "
                  << "bathymetry correction must be bigger than 1.\n");

      if (stereo_settings.bathy_plane == "")
        vw::vw_throw(vw::ArgumentErr() << "The value of --bathy-plane was unspecified.\n");

      // Sanity check reading the bathy plane
      std::vector<vw::BathyPlane> bathy_plane_vec;
      vw::readBathyPlanes(stereo_settings.bathy_plane, num_images, bathy_plane_vec);
    }

    if (session_name.find("isis") != std::string::npos)
      vw::vw_throw(vw::ArgumentErr() 
                   << "Bathymetry correction does not work with ISIS cameras.\n");

    if (stereo_settings.alignment_method != "homography"     &&
        stereo_settings.alignment_method != "affineepipolar" &&
        stereo_settings.alignment_method != "local_epipolar" &&
        stereo_settings.alignment_method != "none")
      vw::vw_throw(vw::ArgumentErr() 
          << "Bathymetry correction only works with alignment methods "
          << "homography, affineepipolar, local_epipolar, and none.\n");

    if (stereo_settings.propagate_errors)
      vw::vw_throw(vw::ArgumentErr() << "Error propagation is not implemented when "
                << "bathymetry is modeled.\n");
  }

  // Ensure that either both or none of these settings are specified
  if ((stereo_settings.refraction_index > 1.0 || 
       stereo_settings.bathy_plane != "") &&
      !doBathy(stereo_settings))
    vw::vw_throw(vw::ArgumentErr() 
          << "When bathymetry correction is not on, it is not necessary to "
          << "specify the water refraction index or the bathy plane.\n");

}

} // end namespace asp
