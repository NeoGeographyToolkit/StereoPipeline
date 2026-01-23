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

// Helper function to read bathy planes from either a direct string or a list file.
// Returns space-separated string of plane files suitable for vw::readBathyPlanes().
std::string readBathyPlanesStrOrList(std::string const& bathy_plane_files,
                                     std::string const& bathy_plane_list) {

  bool have_plane_files = (bathy_plane_files != "");
  bool have_plane_list = (bathy_plane_list != "");
  
  if (have_plane_files && have_plane_list)
    vw::vw_throw(vw::ArgumentErr() 
             << "Cannot specify both --bathy-plane and --bathy-plane-list.\n");

  std::string planes_to_load;
  if (have_plane_list) {
    std::vector<std::string> plane_files;
    asp::read_list(bathy_plane_list, plane_files);
    
    // Join vector into space-separated string
    for (size_t i = 0; i < plane_files.size(); i++) {
      if (i > 0)
        planes_to_load += " ";
      planes_to_load += plane_files[i];
    }
  } else {
    planes_to_load = bathy_plane_files;
  }
  
  return planes_to_load;
}

// Read all bathy data
void readBathyData(int num_images,
                   std::string const& bathy_mask_list,
                   std::string const& bathy_plane_files,
                   std::string const& bathy_plane_list,
                   float refraction_index,
                   vw::BathyData & bathy_data) {

  std::vector<std::string> bathy_mask_files;
  if (bathy_mask_list != "")
    asp::read_list(bathy_mask_list, bathy_mask_files);

  if (int(bathy_mask_files.size()) != num_images)
    vw::vw_throw(vw::ArgumentErr() << "The number of bathy masks must agree with "
             << "the number of images.\n");

  std::string planes_to_load = readBathyPlanesStrOrList(bathy_plane_files, 
                                                        bathy_plane_list);
  vw::readBathyPlanes(planes_to_load, num_images, bathy_data.bathy_planes);
  vw::read_bathy_masks(bathy_mask_files, bathy_data.bathy_masks);
  bathy_data.refraction_index = refraction_index;

  // Validate consistency
  validateBathyData(bathy_data, num_images);
}

// For stereo will use left and right bathy masks. For bundle adjustment and
// jitter_solve will use a list of masks. Only checks if bathy settings strings
// are non-empty. Full consistency validation (all-or-nothing, sizes,
// refraction_index) is done by validateBathyData() which is called
// automatically in readBathyData().
bool doBathy(asp::StereoSettings const& stereo_settings) {
  return (stereo_settings.left_bathy_mask  != "" ||
          stereo_settings.right_bathy_mask != "" ||
          stereo_settings.bathy_mask_list  != "");
}

// Validate loaded bathymetry data for internal consistency.
// This is a data consistency validator that checks the loaded BathyData structure
// for all-or-nothing configuration and size consistency.
// Called after readBathyData() loads masks and planes into BathyData.
// Used by: bundle_adjust, jitter_solve.
void validateBathyData(vw::BathyData const& bathy_data, int num_images) {

  bool has_planes = !bathy_data.bathy_planes.empty();
  bool has_masks  = !bathy_data.bathy_masks.empty();
  bool has_refr   = (bathy_data.refraction_index > 1.0);

  // All or nothing
  if (has_planes || has_masks || has_refr) {
    if (!has_planes)
      vw::vw_throw(vw::ArgumentErr()
                   << "Bathy masks/refraction set but no planes.\n");
    if (!has_masks)
      vw::vw_throw(vw::ArgumentErr()
                   << "Bathy planes/refraction set but no masks.\n");
    if (!has_refr)
      vw::vw_throw(vw::ArgumentErr()
                   << "Bathy planes/masks set but refraction_index <= 1.0.\n");

    // Size checks
    if (bathy_data.bathy_planes.size() != (size_t)num_images)
      vw::vw_throw(vw::ArgumentErr()
                   << "Bathy planes count does not match number of images.\n");
    if (bathy_data.bathy_masks.size() != (size_t)num_images)
      vw::vw_throw(vw::ArgumentErr()
                   << "Bathy masks count does not match number of images.\n");
  }
}

// Lightweight check. The fully checking is done on loading.
bool hasBathy(vw::BathyData const& bathy_data) {
  return !bathy_data.bathy_planes.empty();
}

// Validate bathymetry policy and tool compatibility.
// This is a policy/compatibility validator that checks command-line options
// against tool-specific constraints before loading any data.
// Checks: tool compatibility (ISIS, alignment methods), option presence, and basic values.
// Called early by: stereo, bundle_adjust, jitter_solve.
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

      if (stereo_settings.bathy_plane == "" && stereo_settings.bathy_plane_list == "")
        vw::vw_throw(vw::ArgumentErr() 
                  << "Either --bathy-plane or --bathy-plane-list must be specified.\n");

      // Check mutual exclusion
      if (stereo_settings.bathy_plane != "" && stereo_settings.bathy_plane_list != "")
        vw::vw_throw(vw::ArgumentErr() 
                  << "Cannot specify both --bathy-plane and --bathy-plane-list.\n");

      // Sanity check reading the bathy plane
      std::string planes_to_check = readBathyPlanesStrOrList(stereo_settings.bathy_plane,
                                                             stereo_settings.bathy_plane_list);
      std::vector<vw::BathyPlane> bathy_plane_vec;
      vw::readBathyPlanes(planes_to_check, num_images, bathy_plane_vec);
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
       stereo_settings.bathy_plane != "" ||
       stereo_settings.bathy_plane_list != "") &&
      !doBathy(stereo_settings))
    vw::vw_throw(vw::ArgumentErr()
          << "When bathymetry correction is not on, it is not necessary to "
          << "specify the water refraction index or the bathy plane.\n");

}

} // end namespace asp
