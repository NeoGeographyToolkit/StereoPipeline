/* Copyright (c) 2021-2026, United States Government, as represented by the
 * Administrator of the National Aeronautics and Space Administration.
 *
 * All rights reserved.
 *
 * The "ISAAC - Integrated System for Autonomous and Adaptive Caretaking
 * platform" software is licensed under the Apache License, Version 2.0
 * (the "License"); you may not use this file except in compliance with the
 * License. You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the
 * License for the specific language governing permissions and limitations
 * under the License.
 */

#ifndef ASP_RIG_RIG_IMAGE_IO_H
#define ASP_RIG_RIG_IMAGE_IO_H

#include <string>
#include <vector>
#include <map>
#include <opencv2/core.hpp>
#include <Eigen/Geometry>

namespace rig {

class CameraParameters;
struct ImageMessage;
struct cameraImage;

// The images from the bag may need to be resized to be the same
// size as in the calibration file.
void adjustImageSize(rig::CameraParameters const& cam_params, cv::Mat & image);

// Read an image with 3 floats per pixel. OpenCV's imread() cannot do that.
void readXyzImage(std::string const& filename, cv::Mat & img);

// Write an image with 3 floats per pixel.
//
// This is used for saving 3D point cloud data (XYZ) as an image.
// OpenCV's imwrite() cannot do this.
void saveXyzImage(std::string const& filename, cv::Mat const& img);

// Read an image and potentially its depth cloud
void readImageEntry(// Inputs
                    std::string const& image_file,
                    Eigen::Affine3d const& world_to_cam,
                    std::vector<std::string> const& cam_names,
                    int cam_type,
                    double timestamp,
                    // Outputs
                    std::vector<std::map<double, rig::ImageMessage>> & image_maps,
                    std::vector<std::map<double, rig::ImageMessage>> & depth_maps);

// Save images and their corresponding depth clouds to disk.
void saveImagesAndDepthClouds(std::vector<rig::cameraImage> const& cams);

// Save the list of images, for use with bundle_adjust.
void saveImageList(std::vector<rig::cameraImage> const& cams,
                   std::string const& image_list);

}  // namespace rig

#endif  // ASP_RIG_RIG_IMAGE_IO_H
