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
//  distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
//  WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the
//  License for the specific language governing permissions and limitations
//  under the License.
// __END_LICENSE__

/// \file ExteriorOrientation.h
///
/// Create ASP cameras from a vendor exterior-orientation (EO) delivery: a table
/// of per-image camera positions (in a projected coordinate system) and
/// omega/phi/kappa orientation angles, plus an interior-orientation (camera)
/// description. One Pinhole (.tsai) camera is written per image, along with a
/// camera list (in the same order as the input image list) that can be passed
/// to bundle_adjust.
///
/// Vendors express these angles in their own conventions; currently only the
/// ESRI convention (omega/phi/kappa referenced to the projected grid, so a
/// grid-to-true-north convergence correction is applied) is supported.

#ifndef __ASP_CAMERA_EXTERIOR_ORIENTATION_H__
#define __ASP_CAMERA_EXTERIOR_ORIENTATION_H__

#include <string>

namespace asp {

// Options controlling exterior-orientation ingest. Kept independent of the
// cam_gen Options struct so this logic stays in the Camera library.
struct EoOptions {
  std::string vendor;          // Only "esri" is supported for now.
  std::string extrinsics_file; // Per-image exterior orientation (position + angles).
  std::string intrinsics_file; // Vendor interior-orientation file (may be empty).
  std::string sample_tsai;     // Alternative interior orientation: a sample .tsai.
  std::string image_list;      // Input images (matched to EO rows by file name).
  std::string output_dir;      // Cameras go here. The camera list (in image-list
                               // order) is saved as output_dir/camera_list.txt.
  std::string proj_str;        // CRS of the EO positions. Empty => auto-guess (UTM).
  std::string datum_str;       // Datum name, e.g. WGS_1984. Empty => WGS_1984.
};

// Parse the vendor EO delivery and write one camera per image to opt.output_dir,
// then write opt.camera_list. Throws on an unsupported vendor or malformed input.
void camerasFromExteriorOrientation(EoOptions const& opt);

// Batch creation of Pinhole cameras from a plain roll/pitch/yaw exterior-orientation
// report (columns image, lon, lat, height_above_datum, roll, pitch, yaw). The
// intrinsics come from opt.sample_tsai and the positions are relative to
// opt.datum_str. Each camera is written next to its image, with a .tsai extension.
void camerasFromExtrinsics(EoOptions const& opt);

} // end namespace asp

#endif // __ASP_CAMERA_EXTERIOR_ORIENTATION_H__
