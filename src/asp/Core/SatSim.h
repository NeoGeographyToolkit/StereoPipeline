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

/// \file SatSim.h

// Functions used for the sat_sim.cc tool that are not general enough to put
// somewhere else.

#ifndef __ASP_CORE_SATSIM_H__
#define __ASP_CORE_SATSIM_H__

#include <vw/Cartography/GeoReference.h>
#include <vw/Image/ImageViewRef.h>
#include <vw/FileIO/DiskImageView.h>
#include <vw/Image/PixelMask.h>
#include <vw/FileIO/GdalWriteOptions.h>
#include <vw/Camera/CameraModel.h>

#include <string>

namespace asp {

struct SatSimOptions : vw::GdalWriteOptions {
  std::string dem_file, ortho_file, out_prefix, camera_list, sensor_type;
  vw::Vector3 first, last; // dem pixel and height above dem datum
  int num_cameras, first_index, last_index;
  vw::Vector2 optical_center, image_size, first_ground_pos, last_ground_pos;
  double focal_length, dem_height_error_tol;
  double roll, pitch, yaw, velocity, frame_rate;
  std::vector<double> jitter_frequency, jitter_amplitude, jitter_phase, horizontal_uncertainty;
  std::string jitter_frequency_str, jitter_amplitude_str, jitter_phase_str, 
    horizontal_uncertainty_str;
  bool no_images, save_ref_cams, square_pixels;
  SatSimOptions() {}
};

// Find a handful of valid DEM values and average them. It helps later when
// intersecting with the DEM, especially for Mars, where the DEM heights ca be
// very far from the datum. 
double findDemHeightGuess(vw::ImageViewRef<vw::PixelMask<float>> const& dem);

// A function that will read a geo-referenced image, its nodata value,
// and the georeference, and will return a PixelMasked image, the nodata
// value, and the georeference.
void readGeorefImage(std::string const& image_file, 
  float & nodata_val, vw::cartography::GeoReference & georef,
  vw::ImageViewRef<vw::PixelMask<float>> & masked_image);

// A function that will take as input the endpoints and will compute the
// satellite trajectory and along track/across track/down directions in ECEF,
// which will give the camera to world rotation matrix.
// The key observation is that the trajectory will be a straight edge in
// projected coordinates so will be computed there first. In some usage
// modes we will adjust the end points of the trajectory along the way.
void calcTrajectory(SatSimOptions & opt,
                    vw::cartography::GeoReference const& dem_georef,
                    vw::ImageViewRef<vw::PixelMask<float>> dem,
                    double height_guess,
                    // Outputs
                    double                       & orbit_len,
                    std::vector<vw::Vector3>     & trajectory,
                    std::map<int, vw::Matrix3x3> & cam2world,
                    std::map<int, vw::Matrix3x3> & cam2world_no_jitter,
                    std::vector<vw::Matrix3x3>   & ref_cam2world);

// A function to read the cameras from a file
void readPinholeCameras(SatSimOptions const& opt, 
    std::vector<std::string> & cam_names,
    std::vector<vw::CamPtr> & cams);

// A function to create and save the cameras. Assume no distortion, and pixel
// pitch = 1.
void genPinholeCameras(SatSimOptions const& opt, 
                std::vector<vw::Vector3> const & trajectory,
                std::map<int, vw::Matrix3x3> const & cam2world,
                std::vector<vw::Matrix3x3> const & ref_cam2world,
                // outputs
                std::vector<std::string> & cam_names,
                std::vector<vw::CamPtr> & cams);

// Generate images by projecting rays from the sensor to the ground
void genImages(SatSimOptions const& opt,
    bool external_cameras,
    std::vector<std::string>      const& cam_names,
    std::vector<vw::CamPtr>       const& cams,
    vw::cartography::GeoReference const& dem_georef,
    vw::ImageViewRef<vw::PixelMask<float>> dem,
    double height_guess,
    vw::cartography::GeoReference const& ortho_georef,
    vw::ImageViewRef<vw::PixelMask<float>> ortho,
    float ortho_nodata_val);

} // end namespace asp

#endif//__ASP_CORE_SATSIM_H__
