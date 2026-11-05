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

/// \file SatSim.h

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
  std::string dem_file, ortho_file, out_prefix, camera_list, sensor_type, 
    rig_sensor_ground_offsets, rig_sensor_rotation_angles;
  vw::Vector3 first, last; // dem pixel and height above dem datum
  int num_cameras, first_index, last_index;
  vw::Vector2 optical_center, image_size, first_ground_pos, last_ground_pos;
  double focal_length, dem_height_error_tol, blur_sigma;
  double roll, pitch, yaw, velocity, frame_rate, ref_time, random_position_perturbation;
  std::vector<double> jitter_frequency, jitter_amplitude, jitter_phase, horizontal_uncertainty;
  std::string jitter_frequency_str, jitter_amplitude_str, jitter_phase_str, 
    horizontal_uncertainty_str, rig_config, sensor_name;
  bool no_images, save_ref_cams, non_square_pixels, save_as_csm, model_time,
    perturb_cameras, random_pose_perturbation, flip_cross_track;
  SatSimOptions() {}
};

// Generate a prefix for produced image and camera names.
std::string camPrefix(SatSimOptions const& opt, int iFrame, double timestamp, bool isRef,
                      bool isFrame, std::string const& suffix);

// Given a transform from ref sensor to world, the ref sensor to current sensor,
// create the transform from current sensor to world. Do it in-place. 
void applyRigTransform(Eigen::Affine3d const & ref_to_sensor,
                       vw::Vector3 & ctr, vw::Matrix3x3 & cam2world);

// Generate cameras and images for a sensor
void genCamerasImages(float ortho_nodata_val,
                      bool have_rig,
                      int rig_sensor_index,
                      vw::ImageViewRef<vw::PixelMask<float>> dem,
                      double height_guess,
                      vw::GeoReference const& ortho_georef,
                      vw::ImageViewRef<vw::PixelMask<float>> ortho,
                      SatSimOptions & opt,
                      rig::RigSet & rig,
                      vw::GeoReference const& dem_georef,
                      std::string const& suffix);

// Generate the cameras and images for a rig
void genRigCamerasImages(SatSimOptions & opt,
                         rig::RigSet & rig,
                         vw::GeoReference const& dem_georef,
                         vw::ImageViewRef<vw::PixelMask<float>> dem,
                         double height_guess,
                         vw::GeoReference const& ortho_georef,
                         vw::ImageViewRef<vw::PixelMask<float>> ortho,
                         float ortho_nodata_val);

// Write the rig used by this program and the rig in which each sensor
// is relative to the first sensor. 
void writeRelRig(std::string const& out_prefix, rig::RigSet const& rig);

// A little function to avoid repetitive code in many places.
template<class A, class B>
A mapVal(std::map<B, A> const& m, B const& key) {
  auto it = m.find(key);
  if (it == m.end()) 
    vw::vw_throw(vw::ArgumentErr() << "Could not find key " << key << " in map.\n");
  return it->second;
}

} // end namespace asp

#include <asp/Camera/SatSimTrajectory.h>
#include <asp/Camera/SatSimSensor.h>

#endif // __ASP_CAMERA_SATSIM_H__
