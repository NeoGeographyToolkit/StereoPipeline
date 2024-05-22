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

#ifndef __ASP_CAMERA_SATSIM_H__
#define __ASP_CAMERA_SATSIM_H__

#include <vw/Cartography/GeoReference.h>
#include <vw/Image/ImageViewRef.h>
#include <vw/FileIO/DiskImageView.h>
#include <vw/Image/PixelMask.h>
#include <vw/FileIO/GdalWriteOptions.h>
#include <vw/Camera/CameraModel.h>
#include <vw/Core/Exception.h>

#include <Eigen/Dense>

#include <string>
#include <map>

namespace rig {
  class RigSet;
}

namespace asp {

struct SatSimOptions: vw::GdalWriteOptions {
  std::string dem_file, ortho_file, out_prefix, camera_list, sensor_type;
  vw::Vector3 first, last; // dem pixel and height above dem datum
  int num_cameras, first_index, last_index;
  vw::Vector2 optical_center, image_size, first_ground_pos, last_ground_pos;
  double focal_length, dem_height_error_tol;
  double roll, pitch, yaw, velocity, frame_rate, ref_time;
  std::vector<double> jitter_frequency, jitter_amplitude, jitter_phase, horizontal_uncertainty;
  std::string jitter_frequency_str, jitter_amplitude_str, jitter_phase_str, 
    horizontal_uncertainty_str, rig_config, sensor_name;
  bool no_images, save_ref_cams, non_square_pixels, save_as_csm, model_time;
  SatSimOptions() {}
};

// Find a handful of valid DEM values and average them. It helps later when
// intersecting with the DEM, especially for Mars, where the DEM heights ca be
// very far from the datum. 
double findDemHeightGuess(vw::ImageViewRef<vw::PixelMask<float>> const& dem);

// A function that will take as input the endpoints and will compute the
// satellite positions and along track/across track/down directions in ECEF,
// which will give the camera to world rotation matrix.
// The key observation is that the trajectory will be a straight edge in
// projected coordinates so will be computed there first. In some usage
// modes we will adjust the end points of the trajectory along the way.
void calcTrajectory(SatSimOptions & opt,
                    vw::cartography::GeoReference const& dem_georef,
                    vw::ImageViewRef<vw::PixelMask<float>> dem,
                    double height_guess,
                    // Outputs
                    int                        & first_pos,
                    double                     & first_line_time,
                    double                     & orbit_len,
                    std::vector<vw::Vector3>   & positions,
                    std::vector<vw::Matrix3x3> & cam2world,
                    std::vector<vw::Matrix3x3> & cam2world_no_jitter,
                    std::vector<vw::Matrix3x3> & ref_cam2world,
                    std::vector<double>        & cam_times);

// A function to read the cameras from a file
void readPinholeCameras(SatSimOptions const& opt, 
    std::vector<std::string> & cam_names,
    std::vector<vw::CamPtr> & cams);

// A function to create and save the cameras. Assume no distortion, and pixel
// pitch = 1.
void genPinholeCameras(SatSimOptions          const& opt, 
                vw::cartography::GeoReference const& dem_georef,
                std::vector<vw::Vector3>      const& positions,
                std::vector<vw::Matrix3x3>    const& cam2world,
                std::vector<vw::Matrix3x3>    const& ref_cam2world,
                std::vector<double>           const& cam_times,
                bool                                 have_rig,
                Eigen::Affine3d               const& ref2sensor,
                std::string                   const& suffix, 
                // outputs
                std::vector<std::string> & cam_names,
                std::vector<vw::CamPtr>  & cams);

// Generate images by projecting rays from the sensor to the ground
void genImages(SatSimOptions const& opt,
    bool external_cameras,
    std::vector<std::string>      const& cam_names,
    std::vector<vw::CamPtr>       const& cams,
    std::string                   const& suffix, 
    vw::cartography::GeoReference const& dem_georef,
    vw::ImageViewRef<vw::PixelMask<float>> dem,
    double height_guess,
    vw::cartography::GeoReference const& ortho_georef,
    vw::ImageViewRef<vw::PixelMask<float>> ortho,
    float ortho_nodata_val);

// Generate cameras and images for a sensor
void genCamerasImages(float ortho_nodata_val,
            bool have_rig,
            vw::ImageViewRef<vw::PixelMask<float>> dem,
            double height_guess,
            vw::cartography::GeoReference const& ortho_georef,
            vw::ImageViewRef<vw::PixelMask<float>> ortho,
            SatSimOptions                      & opt,
            rig::RigSet                   const& rig,
            vw::cartography::GeoReference const& dem_georef,
            Eigen::Affine3d               const& ref2sensor,
            std::string                   const& suffix);

// Generate the cameras and images for a rig
void genRigCamerasImages(SatSimOptions          & opt,
            rig::RigSet                    const& rig,
            vw::cartography::GeoReference const & dem_georef,
            vw::ImageViewRef<vw::PixelMask<float>> dem,
            double height_guess,
            vw::cartography::GeoReference const& ortho_georef,
            vw::ImageViewRef<vw::PixelMask<float>> ortho,
            float ortho_nodata_val);

// Given a transform from ref sensor to world, the ref sensor to current sensor,
// create the transform from current sensor to world. Do it in-place. 
void applyRigTransform(Eigen::Affine3d const & ref_to_sensor,
                       vw::Vector3 & ctr, vw::Matrix3x3 & cam2world);

// A little function to avoid repetitive code in many places.
// Get the value of a map key if known to exist.
template<class A, class B>
A mapVal(std::map<B, A> const& m, B const& key){
  typename std::map<B, A>::const_iterator it = m.find(key);
  if (it == m.end()) 
    vw::vw_throw(vw::ArgumentErr() << "Could not find key " << key << " in map.\n");
  return it->second;
}

} // end namespace asp

#endif//__ASP_CAMERA_SATSIM_H__
