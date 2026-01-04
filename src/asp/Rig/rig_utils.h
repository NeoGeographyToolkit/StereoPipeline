/* Copyright (c) 2021, United States Government, as represented by the
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

#ifndef ASP_RIG_UTILS_H_
#define ASP_RIG_UTILS_H_

#include <asp/Rig/RigTypeDefs.h>

#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/core/utility.hpp>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <boost/shared_ptr.hpp>

#include <map>
#include <set>
#include <string>
#include <vector>

// Forward declarations
namespace rig {
  class CameraParameters;
}

namespace rig {

// Forward declarations  
class cameraImage;
class ImageMessage;
class RigSet;

const std::string NO_DEPTH_FILE = "no_depth_file";

// A function to parse a string like
// 'cam1:focal_length,optical_center,distortion cam2:focal_length' and
// extract the intrinsics to float. Separators can be space, comma,
// colon.
void parse_intrinsics_to_float(std::string const& intrinsics_to_float_str,
                               std::vector<std::string> const& cam_names,
                               std::vector<std::set<std::string>>& intrinsics_to_float);

// A  function to split a string like 'haz_cam sci_cam' into
// its two constituents and validate against the list of known cameras.
void parse_camera_names(std::vector<std::string> const& cam_names,
                        std::string const&
                        depth_to_image_transforms_to_float_str,
                        std::set<std::string>&
                        depth_to_image_transforms_to_float);
  
// Convert a string of values separated by spaces to a vector of doubles.
std::vector<double> string_to_vector(std::string const& str);

// Read a 4x4 pose matrix of doubles from disk
void readPoseMatrix(cv::Mat& pose, std::string const& filename);

// Read an affine matrix with double values
bool readAffine(Eigen::Affine3d& T, std::string const& filename);

// Write a matrix with double values
void writeMatrix(Eigen::MatrixXd const& M, std::string const& filename);

void writeCloud(std::vector<float> const& points, size_t point_size, std::string const& filename);

// Return the type of an opencv matrix
std::string matType(cv::Mat const& mat);

// Implement some heuristic to find the maximum rotation angle that can result
// from applying the given transform. It is assumed that the transform is not
// too different from the identity.
double maxRotationAngle(Eigen::Affine3d const& T);

// A class to store timestamped poses, implementing O(log(n)) linear
// interpolation at a desired timestamp. For fast access, keep the
// poses in bins obtained by flooring the timestamp, which is measured
// in seconds. It is assumed that there are a handful of poses
// measured every second, so in each bin. When bins get large, or too
// many bins are empty, the efficiency of this algorithm goes down.
class StampedPoseStorage {
 public:
  void addPose(Eigen::Affine3d const& pose, double timestamp);

  // Find the interpolated pose by looking up the two poses with
  // closest timestamps that are below and above input_timestamp. If
  // the gap between those timestamps is more than max_gap, return
  // failure, as then likely the interpolation result is not accurate.
  bool interpPose(double input_timestamp, double max_gap, Eigen::Affine3d& out_pose) const;

  void clear();

  bool empty() const;

 private:
  std::map<int, std::map<double, Eigen::Affine3d>> m_poses;
};

// Compute the azimuth and elevation for a (normal) vector
void normalToAzimuthAndElevation(Eigen::Vector3d const& normal, double& azimuth, double& elevation);

// Compute a normal vector based on the azimuth and elevation angles
void azimuthAndElevationToNormal(Eigen::Vector3d& normal, double azimuth, double elevation);

// Snap the normal to the plane (and the plane itself) to make
// all angles multiple of 45 degrees with the coordinate axes.
void snapPlaneNormal(Eigen::Vector3d& plane_normal);

// Find the best fitting plane to a set of points
void bestFitPlane(const std::vector<Eigen::Vector3d>& points, Eigen::Vector3d& centroid, Eigen::Vector3d& plane_normal);

// Extract from a string of the form someDir/1234.5678.jpg the number 123.456.
double fileNameToTimestamp(std::string const& file_name);

// A little holding structure for nav, sci, and haz poses
struct CameraPoses {
  std::map<double, double> haz_depth_to_image_timestamps;
  std::map<std::string, std::map<double, Eigen::Affine3d>> world_to_cam_poses;
};

// Some small utilities for writing a file having poses for nav, sci, and haz cam,
// and also the depth timestamp corresponding to given haz intensity timestamp
void writeCameraPoses(std::string const& filename,
                      std::map<double, double> const& haz_depth_to_image_timestamps,
                      std::map<std::string, std::map<double, Eigen::Affine3d>> const&
                      world_to_cam_poses);

void readCameraPoses(std::string const& filename,
                     std::map<double, double>& haz_depth_to_image_timestamps,
                     std::map<std::string, std::map<double, Eigen::Affine3d>>&
                     world_to_cam_poses);

// Gamma and inverse gamma functions
// https://en.wikipedia.org/wiki/SRGB#Specification_of_the_transformation
double gamma(double x);
double inv_gamma(double x);

// Apply the inverse gamma transform to images, multiply them by
// max_iso_times_exposure/ISO/exposure_time to adjust for
// lightning differences, then apply the gamma transform back.
void exposureCorrection(double max_iso_times_exposure, double iso,
                        double exposure, cv::Mat const& input_image,
                        cv::Mat& output_image);

// Scale an image to correct for lightning variations by taking into
// account that JPEG images have gamma correction applied to them.
// See https://en.wikipedia.org/wiki/Gamma_correction.
void scaleImage(double max_iso_times_exposure, double iso, double exposure,
                cv::Mat const& input_image,
                cv::Mat& output_image);

// Given two bounds, pick two timestamps within these bounds, the one
// closest to the left bound and the one to the right bound. Take into
// account that the timestamps may need to have an offset added to
// them. Assume that the input timestamps are sorted in increasing order.
// TODO(oalexan1): May have to add a constraint to only pick
// a timestamp if not further from the bound than a given value.
void pickTimestampsInBounds(std::vector<double> const& timestamps, double left_bound,
                            double right_bound, double offset,
                            std::vector<double>& out_timestamps);

// Must always have NUM_EXIF the last.
enum ExifData { TIMESTAMP = 0, EXPOSURE_TIME, ISO, APERTURE, FOCAL_LENGTH, NUM_EXIF };

// Find the depth measurement. Use nearest neighbor interpolation
// to look into the depth cloud.
bool depthValue(// Inputs
                cv::Mat const& depth_cloud, Eigen::Vector2d const& dist_ip,
                // Output
                Eigen::Vector3d& depth_xyz);
  
// Forward declaration
struct cameraImage;

// Create the image and depth cloud file names
void genImageAndDepthFileNames(  // Inputs
  std::vector<cameraImage> const& cams, std::vector<std::string> const& cam_names,
  std::string const& out_dir,
  // Outputs
  std::vector<std::string>& image_files, std::vector<std::string>& depth_files);

// Convert a string of space-separated numbers to a vector
void strToVec(std::string const& str, std::vector<double> & vec);
  
// Read the images, depth clouds, and their metadata
// Save the properties of images. Use space as separator.
void saveCameraPoses(std::string const& out_dir, std::vector<rig::cameraImage> const& cams,
                    std::vector<Eigen::Affine3d> const& world_to_cam);

// Save the depth clouds and optimized transforms needed to create a mesh with voxblox
// (if depth clouds exist).
void exportToVoxblox(std::vector<std::string> const& cam_names,
                     std::vector<rig::cameraImage> const& cam_images,
                     std::vector<Eigen::Affine3d> const& depth_to_image,
                     std::vector<Eigen::Affine3d> const& world_to_cam,
                     std::string const& out_dir);

void saveTransformedDepthClouds(std::vector<std::string> const& cam_names,
                                std::vector<rig::cameraImage> const& cam_images,
                                std::vector<Eigen::Affine3d> const& depth_to_image,
                                std::vector<Eigen::Affine3d> const& world_to_cam,
                                std::string const& out_dir);
  
// Write the inliers in nvm format. The keypoints are shifted relative to the optical
// center, as written by Theia if shift_keypoints is specified.
// We handle properly the case when a (cid, fid) shows up in many tracks
// (this was a bug).
void writeInliersToNvm
(std::string                                       const& nvm_file,
 bool                                                     shift_keypoints, 
 std::vector<rig::CameraParameters>             const& cam_params,
 std::vector<rig::cameraImage>               const& cams,
 std::vector<Eigen::Affine3d>                      const& world_to_cam,
 std::vector<std::vector<std::pair<float, float>>> const& keypoint_vec,
 std::vector<std::map<int, int>>                   const& pid_to_cid_fid,
 PidCidFid const& pid_cid_fid_inlier,
 std::vector<Eigen::Vector3d>                      const& xyz_vec);
  
}  // namespace rig

#endif  // ASP_RIG_UTILS_H_
