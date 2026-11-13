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

/// \file CameraGenUtils.h
///
/// Generic camera-construction helpers used by cam_gen. These are kept free of
/// the cam_gen Options struct so the logic can live in the Camera library.

#ifndef __ASP_CAMERA_CAMERA_GEN_UTILS_H__
#define __ASP_CAMERA_CAMERA_GEN_UTILS_H__

#include <vw/Math/Vector.h>
#include <vw/Image/ImageViewRef.h>
#include <vw/Image/PixelMask.h>
#include <boost/shared_ptr.hpp>

#include <vector>
#include <string>

namespace vw {
  class CameraModel;
  class GeoReference;
  class Datum;
}

namespace asp {

/// Parse a SkySat frame-index file to find this image's lon-lat corner values,
/// and optionally the camera center and quaternion in ECEF coordinates.
void parseFrameIndex(std::string const& frame_index,
                     std::string const& image_file,
                     bool parse_ecef,
                     double cam_weight,
                     // Outputs
                     std::string & lon_lat_values_str,
                     std::string & parsed_camera_center_str,
                     std::string & parsed_cam_quat_str);

/// Fetch metadata (ephemeris time, sun position, serial number, target name)
/// from an ISIS cube camera, for saving to an output CSM camera. Returns zeros
/// and empty strings when the camera is not an ISIS model.
void isisCubMetadata(boost::shared_ptr<vw::CameraModel> in_cam,
                     // Outputs
                     double & ephem_time,
                     vw::Vector3 & sun_pos,
                     std::string & serial_number,
                     std::string & target_name);

/// Average the pairwise triangulated intersection of rays (given by their
/// centers and directions) to estimate a single camera center. Needed for
/// models like RPC that do not store a camera center.
vw::Vector3 triangulateCameraCenter(std::vector<vw::Vector3> const& ctrs,
                                    std::vector<vw::Vector3> const& dirs);

/// Read a Pinhole camera from Planet's *_pinhole.json format. The datum is set
/// in the georeference and the camera is returned.
void readPlanetPinhole(std::string const& input_camera,
                       vw::Datum const& datum,
                       // Outputs
                       vw::GeoReference & geo,
                       boost::shared_ptr<vw::CameraModel> & out_cam);

/// Populate the CSM lens distortion parameters when not set, and validate the
/// count for the given type. For a radtan target with a tsai input camera the
/// exact distortion is copied and exact_tsai_to_csm_conv is set.
void setupCsmDistortion(std::string const& camera_type,
                        std::string const& out_ext,
                        std::string const& input_ext,
                        std::string const& input_camera,
                        bool csm_refit_pose,
                        std::string const& distortion_type,
                        // Outputs
                        std::vector<double> & distortion,
                        bool & exact_tsai_to_csm_conv);

/// For each lon-lat pair, compute the ground xyz point. The height is taken from
/// cam_heights (when an input camera was used to trace rays to the ground),
/// otherwise from the DEM at that location, otherwise from height_above_datum.
/// Also return the per-corner longitude-latitude-height.
void cornersToGroundXyz(std::vector<double> const& lon_lat_values,
                        std::vector<double> const& cam_heights,
                        bool has_input_camera,
                        vw::ImageViewRef<vw::PixelMask<float>> const& interp_dem,
                        bool has_dem,
                        vw::GeoReference const& geo,
                        double height_above_datum,
                        std::vector<vw::Vector3> & xyz_vec,   // output
                        std::vector<vw::Vector3> & llh_vec);  // output

/// Build an initial camera of the given image dimensions. For camera_type
/// "pinhole", use the intrinsics passed in, or read them from a sample file
/// (.tsai, or a CSM .json whose intrinsics are read back through the in-out
/// parameters). For "opticalbar", read the sample optical bar camera and set its
/// image size and optical center.
void manufactureCam(std::string const& camera_type,
                    std::string const& sample_file,
                    int wid, int hgt,
                    double & focal_length,
                    double & pixel_pitch,
                    vw::Vector2 & optical_center,
                    std::vector<double> & distortion,
                    std::string & distortion_type,
                    boost::shared_ptr<vw::CameraModel> & out_cam);

} // end namespace asp

#endif // __ASP_CAMERA_CAMERA_GEN_UTILS_H__
