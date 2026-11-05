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

/// \file SatSimSensor.h
/// Sensor projection and synthetic imagery generation for sat_sim.

#ifndef __ASP_CAMERA_SATSIM_SENSOR_H__
#define __ASP_CAMERA_SATSIM_SENSOR_H__

#include <asp/Camera/SatSim.h>

#include <vw/Cartography/GeoReference.h>
#include <vw/Image/ImageViewRef.h>
#include <vw/Image/PixelMask.h>
#include <vw/Camera/CameraModel.h>

#include <Eigen/Dense>

#include <vector>
#include <string>

namespace asp {

// Read pinhole cameras from a file
void readPinholeCameras(SatSimOptions const& opt, 
                        std::vector<std::string> & cam_names,
                        std::vector<vw::CamPtr> & cams);

// Perturb the cameras with random noise
void perturbCameras(SatSimOptions const& opt, 
                    std::string const& suffix,
                    vw::GeoReference const& georef,
                    std::vector<std::string> const& cam_names,
                    std::vector<vw::CamPtr> & cams);

// Create and save pinhole cameras
void genPinholeCameras(SatSimOptions const& opt,
                       vw::GeoReference const& dem_georef,
                       std::vector<vw::Vector3> const& positions,
                       std::vector<vw::Matrix3x3> const& cam2world,
                       std::vector<vw::Matrix3x3> const& ref_cam2world,
                       std::vector<double> const& cam_times,
                       bool have_rig,
                       Eigen::Affine3d const& ref2sensor,
                       std::string const& suffix, 
                       // Outputs
                       std::vector<std::string> & cam_names,
                       std::vector<vw::CamPtr> & cams);

// Generate images by projecting rays from sensor to ground
void genImages(SatSimOptions const& opt,
               bool external_cameras,
               std::vector<std::string> const& cam_names,
               std::vector<vw::CamPtr> const& cams,
               std::string const& suffix, 
               vw::GeoReference const& dem_georef,
               vw::ImageViewRef<vw::PixelMask<float>> dem,
               double height_guess,
               vw::GeoReference const& ortho_georef,
               vw::ImageViewRef<vw::PixelMask<float>> ortho,
               float ortho_nodata_val);

// Handle sensor type selection (frame vs linescan)
void handleSensorType(int num_sensors, 
                      std::string const& sensor_type, 
                      std::vector<std::string> & sensor_types);

} // namespace asp

#endif // __ASP_CAMERA_SATSIM_SENSOR_H__
