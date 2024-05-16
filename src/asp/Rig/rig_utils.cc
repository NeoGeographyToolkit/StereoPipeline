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

#include <glog/logging.h>

#include <Rig/system_utils.h>
#include <Rig/rig_utils.h>
#include <Rig/camera_image.h>
#include <Rig/transform_utils.h>
#include <Rig/interpolation_utils.h>
#include <Rig/rig_config.h>
#include <Rig/happly.h> // for saving ply files as meshes
#include <Rig/RigCameraParams.h>

#include <boost/filesystem.hpp>

#include <iostream>
#include <fstream>
#include <iomanip>

#include <pcl/io/ply_io.h>
#include <pcl/io/pcd_io.h>

// TODO(oalexan1): This file needs to be broken up

namespace rig {

// A little function to replace separators with space. Note that the backslash
// is a separator, in case, it used as a continuation line.
void replace_separators_with_space(std::string & str) {
  std::string sep = "\\:, \t\r\n";
  for (size_t it = 0; it < sep.size(); it++) 
    std::replace(str.begin(), str.end(), sep[it], ' ');
}
  
// A function to parse a string like
// 'cam1:focal_length,optical_center,distortion cam2:focal_length' and
// extract the intrinsics to float. Separators can be space, comma,
// colon.
void parse_intrinsics_to_float(std::string const& intrinsics_to_float_str,
                               std::vector<std::string> const& cam_names,
                               std::vector<std::set<std::string>>& intrinsics_to_float) {
  // Wipe the output
  intrinsics_to_float.clear();

  std::string input_str = intrinsics_to_float_str; // so we can edit it

  replace_separators_with_space(input_str);

  std::istringstream iss(input_str);
  std::string curr_cam = "";
  std::string val;
  
  // Temporary map of sets for collection. This will ensure variable order
  // of inputs is supported.
  std::map<std::string, std::set<std::string>> local_map;
  while (iss >> val) {
    // See if this is a camera name
    bool have_cam_name = false;
    for (size_t it = 0; it < cam_names.size(); it++) {
      if (val == cam_names[it]) {
        curr_cam = val;
        have_cam_name = true;
        break;
      }
    }

    if (have_cam_name) // recorded the camera name
      continue;
    
    if (val != "focal_length" && val != "optical_center" && val != "distortion")
      LOG(FATAL) << "Unexpected value when parsing intrinsics to float: " << val << "\n";

    if (curr_cam == "") 
      LOG(FATAL) << "Incorrectly set option for floating intrinsics.\n";

    local_map[curr_cam].insert(val);
  }

  // Export this
  intrinsics_to_float.resize(cam_names.size());
  for (size_t it = 0; it < cam_names.size(); it++)
    intrinsics_to_float[it] = local_map[cam_names[it]];
}

// A  function to split a string like 'haz_cam sci_cam' into
// its two constituents and validate against the list of known cameras.
void parse_camera_names(std::vector<std::string> const& cam_names,
                                              std::string const&
                                              depth_to_image_transforms_to_float_str,
                                              std::set<std::string>&
                                              depth_to_image_transforms_to_float) {
  // Wipe the output
  depth_to_image_transforms_to_float.clear();

  std::string input_str = depth_to_image_transforms_to_float_str; // so we can edit it
  replace_separators_with_space(input_str);
  
  std::istringstream iss(input_str);
  std::string curr_cam = "";
  std::string val;
  
  while (iss >> val) {
    bool have_cam_name = false;
    for (size_t it = 0; it < cam_names.size(); it++) {
      if (val == cam_names[it]) {
        have_cam_name = true;
        break;
      }
    }
    
    if (!have_cam_name) 
      LOG(FATAL) << "Error: A specified sensor name is not among the known sensors. "
                 << "Offending camera: " << val << "\n";
    
    depth_to_image_transforms_to_float.insert(val);
  }

  return;
}  
  
// Read a 4x4 pose matrix of doubles from disk
void readPoseMatrix(cv::Mat& pose, std::string const& filename) {
  pose = cv::Mat::zeros(4, 4, CV_64F);
  std::ifstream ifs(filename.c_str(), std::ifstream::in);
  for (int row = 0; row < 4; row++) {
    for (int col = 0; col < 4; col++) {
      double val;
      if (!(ifs >> val)) LOG(FATAL) << "Could not read a 4x4 matrix from: " << filename;
      pose.at<double>(row, col) = val;
    }
  }
}

// Read an affine matrix with double values
bool readAffine(Eigen::Affine3d& T, std::string const& filename) {
  Eigen::MatrixXd M(4, 4);

  std::ifstream ifs(filename.c_str(), std::ifstream::in);
  for (int row = 0; row < 4; row++) {
    for (int col = 0; col < 4; col++) {
      double val;
      if (!(ifs >> val)) return false;

      M(row, col) = val;
    }
  }

  T.linear() = M.block<3, 3>(0, 0);
  T.translation() = M.block<3, 1>(0, 3);

  return true;
}

// Write a matrix with double values
void writeMatrix(Eigen::MatrixXd const& M, std::string const& filename) {
  // std::cout << "Writing: " << filename << std::endl;
  std::ofstream ofs(filename.c_str());
  ofs.precision(17);
  ofs << M << "\n";
  ofs.close();
}

// Save a file with x, y, z rows if point_size is 3, and also a color
// if point_size is 4.
void writeCloud(std::vector<float> const& points, size_t point_size, std::string const& filename) {
  size_t num_points = points.size() / point_size;
  if (point_size * num_points != points.size()) LOG(FATAL) << "Book-keeping failure.";

  // std::cout << "Writing: " << filename << "\n";
  std::ofstream fh(filename.c_str());
  fh.precision(17);
  for (size_t it = 0; it < num_points; it++) {
    for (size_t ch = 0; ch < point_size; ch++) {
      fh << points[point_size * it + ch];
      if (ch + 1 < point_size)
        fh << " ";
      else
        fh << "\n";
    }
  }
  fh.close();
}

// Return the type of an opencv matrix
std::string matType(cv::Mat const& mat) {
  int type = mat.type();
  std::string r;

  uchar depth = type & CV_MAT_DEPTH_MASK;
  uchar chans = 1 + (type >> CV_CN_SHIFT);

  switch (depth) {
    case CV_8U:
      r = "8U";
      break;
    case CV_8S:
      r = "8S";
      break;
    case CV_16U:
      r = "16U";
      break;
    case CV_16S:
      r = "16S";
      break;
    case CV_32S:
      r = "32S";
      break;
    case CV_32F:
      r = "32F";
      break;
    case CV_64F:
      r = "64F";
      break;
    default:
      r = "User";
      break;
  }

  r += "C";
  r += (chans + '0');

  return r;
}

// Implement some heuristic to find the maximum rotation angle that can result
// from applying the given transform. It is assumed that the transform is not
// too different from the identity.
double maxRotationAngle(Eigen::Affine3d const& T) {
  Eigen::Vector3d angles = T.linear().eulerAngles(0, 1, 2);

  // Angles close to +/-pi can result even if the matrix is close to identity
  for (size_t it = 0; it < 3; it++)
    angles[it] = std::min(std::abs(angles[it]), std::abs(M_PI - std::abs(angles[it])));
  double angle_norm = (180.0 / M_PI) * angles.norm();
  return angle_norm;
}

void StampedPoseStorage::addPose(Eigen::Affine3d const& pose, double timestamp) {
  int bin_index = floor(timestamp);
  m_poses[bin_index][timestamp] = pose;
}

bool StampedPoseStorage::interpPose(double input_timestamp, double max_gap, Eigen::Affine3d& out_pose) const {
  bool is_success = false;

  if (m_poses.empty()) return is_success;

  // Look for the nearest pose with timestamp <= input_timestamp.
  double low_timestamp = -1.0;
  Eigen::Affine3d low_pose;
  // Traverse the bins in decreasing order of bin key.
  for (int bin_iter = floor(input_timestamp); bin_iter >= m_poses.begin()->first; bin_iter--) {
    auto bin_ptr = m_poses.find(bin_iter);
    if (bin_ptr == m_poses.end()) continue;  // empty bin

    // Found a bin. Study it in decreasing order of timestamps.
    auto& bin = bin_ptr->second;
    for (auto it = bin.rbegin(); it != bin.rend(); it++) {
      double timestamp = it->first;
      if (timestamp <= input_timestamp) {
        low_timestamp = timestamp;
        low_pose = it->second;
        is_success = true;
        break;
      }
    }
    if (is_success) break;
  }

  if (!is_success) return false;  // Failed

  // Found the lower bound. Now go forward in time. Here the logic is
  // the reverse of the above.
  is_success = false;
  double high_timestamp = -1.0;
  Eigen::Affine3d high_pose;
  for (int bin_iter = floor(input_timestamp); bin_iter <= m_poses.rbegin()->first; bin_iter++) {
    auto bin_ptr = m_poses.find(bin_iter);
    if (bin_ptr == m_poses.end()) continue;  // empty bin

    // Found a bin. Study it in increasing order of timestamps.
    auto& bin = bin_ptr->second;
    for (auto it = bin.begin(); it != bin.end(); it++) {
      double timestamp = it->first;
      if (timestamp >= input_timestamp) {
        high_timestamp = timestamp;
        high_pose = it->second;
        is_success = true;
        break;
      }
    }
    if (is_success) break;
  }

  if (!is_success || high_timestamp - low_timestamp > max_gap) {
    return false;  // Failed
  }

  if (!(low_timestamp <= input_timestamp && input_timestamp <= high_timestamp))
    LOG(FATAL) << "Book-keeping failure in pose interpolation.";

  double alpha = (input_timestamp - low_timestamp) / (high_timestamp - low_timestamp);
  if (high_timestamp == low_timestamp) alpha = 0.0;  // handle division by zero

  out_pose = rig::linearInterp(alpha, low_pose, high_pose);

  return is_success;
}

void StampedPoseStorage::clear() { m_poses.clear(); }

bool StampedPoseStorage::empty() const { return m_poses.empty(); }

// Compute the azimuth and elevation for a (normal) vector
void normalToAzimuthAndElevation(Eigen::Vector3d const& normal,
                                 double& azimuth, double& elevation) {
  if (normal.x() == 0 && normal.y() == 0) {
    azimuth = 0.0;
    if (normal.z() >= 0.0)
      elevation = M_PI / 2.0;
    else
      elevation = -M_PI / 2.0;
  } else {
    azimuth = atan2(normal.y(), normal.x());
    elevation = atan2(normal.z(), Eigen::Vector2d(normal.x(), normal.y()).norm());
  }
}

// Compute a normal vector based on the azimuth and elevation angles
void azimuthAndElevationToNormal(Eigen::Vector3d& normal, double azimuth, double elevation) {
  double ca = cos(azimuth), sa = sin(azimuth);
  double ce = cos(elevation), se = sin(elevation);
  normal = Eigen::Vector3d(ca * ce, sa * ce, se);
}

// Snap the normal to the plane (and the plane itself) to make
// all angles multiple of 45 degrees with the coordinate axes.
void snapPlaneNormal(Eigen::Vector3d& plane_normal) {
  double azimuth, elevation;
  normalToAzimuthAndElevation(plane_normal, azimuth, elevation);

  // Snap to multiple of 45 degrees
  double radian45 = M_PI / 4.0;
  azimuth = radian45 * round(azimuth / radian45);
  elevation = radian45 * round(elevation / radian45);

  azimuthAndElevationToNormal(plane_normal, azimuth, elevation);
}

// Find the best fitting plane to a set of points
void bestFitPlane(const std::vector<Eigen::Vector3d>& points, Eigen::Vector3d& centroid,
                  Eigen::Vector3d& plane_normal) {
  // Copy coordinates to  matrix in Eigen format
  size_t num_points = points.size();
  Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> coord(3, num_points);

  for (size_t i = 0; i < num_points; i++) coord.col(i) = points[i];

  // calculate centroid
  centroid = Eigen::Vector3d(coord.row(0).mean(), coord.row(1).mean(), coord.row(2).mean());

  // subtract centroid
  for (size_t it = 0; it < 3; it++) coord.row(it).array() -= centroid(it);

  // We only need the left-singular matrix here
  // https://math.stackexchange.com/questions/99299/best-fitting-plane-given-a-set-of-points
  auto svd = coord.jacobiSvd(Eigen::ComputeThinU | Eigen::ComputeThinV);

  plane_normal = svd.matrixU().rightCols<1>();
}

// Extract from a string of the form someDir/1234.5678.jpg the number 123.456.
double fileNameToTimestamp(std::string const& file_name) {
  size_t beg = file_name.rfind("/");
  size_t end = file_name.rfind(".");
  if (beg == std::string::npos || end == std::string::npos || beg > end) {
    std::cout << "Could not parse file name: " + file_name;
    exit(1);
  }

  std::string frameStr = file_name.substr(beg + 1, end - beg - 1);
  return atof(frameStr.c_str());
}

// Minor utilities for converting values to a string below

// Convert a string of values separated by spaces to a vector of doubles.
std::vector<double> string_to_vector(std::string const& str) {
  std::istringstream iss(str);
  std::vector<double> vals;
  double val;
  while (iss >> val)
    vals.push_back(val);
  return vals;
}

void readCameraPoses(std::string const& filename,
                     std::map<double, double>& haz_depth_to_image_timestamps,
                     std::map<std::string, std::map<double, Eigen::Affine3d> >&
                     world_to_cam_poses) {
  haz_depth_to_image_timestamps.clear();
  world_to_cam_poses.clear();

  std::ifstream ifs(filename.c_str());
  std::string line;
  while (std::getline(ifs, line)) {
    std::istringstream is(line);

    std::string str;
    if (!(is >> str)) continue;

    if (str == "nav_cam" || str == "sci_cam" || str == "haz_cam") {
      double timestamp;
      if (!(is >> timestamp)) continue;

      Eigen::MatrixXd M(4, 4);
      for (int row = 0; row < 4; row++) {
        for (int col = 0; col < 4; col++) {
          if (!(is >> M(row, col))) LOG(FATAL) << "Failure reading: " << filename;
        }
      }

      world_to_cam_poses[str][timestamp].matrix() = M;

    } else if (str == "haz_depth_to_image") {
      double depth_time, image_time;
      if (!(is >> depth_time >> image_time)) LOG(FATAL) << "Failure reading: " << filename;

      haz_depth_to_image_timestamps[depth_time] = image_time;
    }
  }

  ifs.close();
}

// Gamma correction for x between 0 and 1.
// https://en.wikipedia.org/wiki/SRGB#Specification_of_the_transformation
double gamma(double x) {
  // return pow(x, 1.0/2.6);

  if (x <= 0.0031308) return 12.92 * x;

  return 1.055 * pow(x, 1.0 / 2.4) - 0.055;
}

double inv_gamma(double x) {
  // return pow(x, 2.6);

  if (x <= 0.04045) return x / 12.92;

  return pow((x + 0.055) / (1.055), 2.4);
}

// Apply the inverse gamma transform to images, multiply them by
// max_iso_times_exposure/ISO/exposure_time to adjust for
// lightning differences, then apply the gamma transform back.
void exposureCorrection(double max_iso_times_exposure, double iso, double exposure,
                        cv::Mat const& input_image, cv::Mat& output_image) {
  double scale = max_iso_times_exposure / iso / exposure;

  // Make an image of the same type
  input_image.copyTo(output_image);

  // Apply the inverse gamma correction, multiply by scale,
  // and apply the correction back
#pragma omp parallel for
  for (int row = 0; row < input_image.rows; row++) {
    for (int col = 0; col < input_image.cols; col++) {
      cv::Vec3b b = input_image.at<cv::Vec3b>(row, col);

      cv::Vec3b c;
      for (int color = 0; color < 3; color++) {
        double x = 255.0 * gamma(inv_gamma(static_cast<double>(b[color]) / 255.0) * scale);
        c[color] = std::min(round(x), 255.0);
      }
      output_image.at<cv::Vec3b>(row, col) = c;
    }
  }
}

// Scale an image to correct for lightning variations by taking into
// account that JPEG images have gamma correction applied to them.
// See https://en.wikipedia.org/wiki/Gamma_correction.
void scaleImage(double max_iso_times_exposure, double iso, double exposure,
                cv::Mat const& input_image, cv::Mat& output_image) {
  double scale = pow(max_iso_times_exposure / iso / exposure, 1.0 / 2.2);
  int same_type = -1;
  double offset = 0.0;
  input_image.convertTo(output_image, same_type, scale, offset);
}

// Given two bounds, pick two timestamps within these bounds, the one
// closest to the left bound and the one to the right bound. Take into
// account that the timestamps may need to have an offset added to
// them. Assume that the input timestamps are sorted in increasing order.
// TODO(oalexan1): May have to add a constraint to only pick
// a timestamp if not further from the bound than a given value.
void pickTimestampsInBounds(std::vector<double> const& timestamps,
                            double left_bound, double right_bound, double offset,
                            std::vector<double>& out_timestamps) {
  out_timestamps.clear();

  // Start by simply collecting all timestamps between the given
  // bounds. Much easier to understand than if doing something more
  // fancy.
  std::vector<double> local_timestamps;
  for (size_t it = 0; it < timestamps.size(); it++) {
    double timestamp = timestamps[it];
    if (timestamp + offset >= left_bound && timestamp + offset < right_bound) {
      local_timestamps.push_back(timestamp);
    }
  }

  if (local_timestamps.size() < 1) {
    // Nothing to pick
    return;
  }

  if (local_timestamps.size() == 1) {
    // Only one is present
    out_timestamps.push_back(local_timestamps[0]);
    return;
  }

  // Add the ones at the ends
  out_timestamps.push_back(local_timestamps[0]);
  out_timestamps.push_back(local_timestamps.back());

  return;
}

// Convert a string of space-separated numbers to a vector
void strToVec(std::string const& str, std::vector<double> & vec) {

  vec.clear();
  std::istringstream iss(str);
  double val = 0.0;
  while (iss >> val)
    vec.push_back(val);
}

// Read the images, depth clouds, and their metadata
// Save the properties of images. Use space as separator.
void saveCameraPoses(std::string const& out_dir,
                     std::vector<rig::cameraImage> const& cams,
                     std::vector<Eigen::Affine3d> const& world_to_cam) {
  rig::createDir(out_dir);
  std::string image_list = out_dir + "/cameras.txt";
  std::cout << "Writing: " << image_list << std::endl;

  std::ofstream f;
  f.open(image_list.c_str(), std::ios::binary | std::ios::out);
  if (!f.is_open()) LOG(FATAL) << "Cannot open file for writing: " << image_list << "\n";
  f.precision(17);

  f << "# image_file world_to_camera_transform\n";

  for (size_t it = 0; it < cams.size(); it++) {

    // Convert an affine transform to a 4x4 matrix
    Eigen::MatrixXd T = world_to_cam[it].matrix();

    // Save the rotation and translation of T
    f << cams[it].image_name << " " << rig::affineToStr(world_to_cam[it]) << "\n";
  }

  f.close();
}

// Check if first three coordinates of a vector are 0. That would make it invalid.
bool invalidXyz(cv::Vec3f const& p) { return (p[0] == 0) && (p[1] == 0) && (p[2] == 0); }

// Add a given vertex to the ply file unless already present
void add_vertex(cv::Vec3f const& V, double intensity,
                std::pair<int, int> const& pix,    // NOLINT
                std::map<std::pair<int, int>, size_t>& pix_to_vertex,  // NOLINT
                size_t& vertex_count,                                  // NOLINT
                std::vector<std::array<double, 3>>& vertices,          // NOLINT
                std::vector<std::array<double, 3>>& colors) {          // NOLINT
  // Do not add the invalid zero vertex
  if (invalidXyz(V)) return;

  if (pix_to_vertex.find(pix) != pix_to_vertex.end()) return;  // Vertex already exists

  std::array<double, 3> point = {V[0], V[1], V[2]};
  std::array<double, 3> color = {intensity/255.0, intensity/255.0, intensity/255.0};

  vertices.push_back(point);
  colors.push_back(color);

  // Record the map from the pixel to its location
  pix_to_vertex[pix] = vertex_count;
  vertex_count++;
}

void applyTransformToCloud(cv::Mat const& in_cloud,
                           Eigen::Affine3d const& cam_to_world,
                           cv::Mat & out_cloud) {

  if (in_cloud.channels() != 3) 
    LOG(FATAL) << "Expecting an xyz point cloud.\n";
  
  out_cloud = cv::Mat::zeros(in_cloud.rows, in_cloud.cols, CV_32FC3);
  for (int row = 0; row < in_cloud.rows; row++) {
    for (int col = 0; col < in_cloud.cols; col++) {
      cv::Vec3f xyz = in_cloud.at<cv::Vec3f>(row, col);
      
      if (invalidXyz(xyz)) {
        // Invalid points stay invalid
        out_cloud.at<cv::Vec3f>(row, col) = xyz;
        continue;
      }

      // Transform to world coordinates
      Eigen::Vector3d P(xyz[0], xyz[1], xyz[2]);
      P = cam_to_world * P;
      for (size_t c = 0; c < 3; c++)
        out_cloud.at<cv::Vec3f>(row, col)[c] = P[c];
    }
  }
}
  
// Apply a transform to a point cloud to make it go from camera coordinates to world
// coordinates and save it as a ply file.
void saveTransformedMesh(cv::Mat const& depthMat, cv::Mat const& intensity,
                         Eigen::Affine3d const& depth_to_world,
                         std::string const& plyFileName) {

  // Sanity check
  if (depthMat.cols != intensity.cols || depthMat.rows != intensity.rows)
    LOG(FATAL) << "The depth cloud and its intensity must have same size.\n";
  
  // Apply the transform.
  cv::Mat transMat;
  applyTransformToCloud(depthMat, depth_to_world, transMat);

  size_t vertex_count = 0;
  std::map<std::pair<int, int>, size_t> pix_to_vertex;  // map from pixel to vertex indices
  std::vector<std::array<double, 3>> vertices;
  std::vector<std::array<double, 3>> colors;
  std::vector<std::vector<size_t>> faces;

  for (int row = 0; row < transMat.rows - 1; row++) {
    for (int col = 0; col < transMat.cols - 1; col++) {
      std::pair<int, int> pix_ul = std::make_pair(row, col);
      std::pair<int, int> pix_ur = std::make_pair(row + 1, col);
      std::pair<int, int> pix_ll = std::make_pair(row, col + 1);
      std::pair<int, int> pix_lr = std::make_pair(row + 1, col + 1);
      cv::Vec3f UL = transMat.at<cv::Vec3f>(pix_ul.first, pix_ul.second);
      cv::Vec3f UR = transMat.at<cv::Vec3f>(pix_ur.first, pix_ur.second);
      cv::Vec3f LL = transMat.at<cv::Vec3f>(pix_ll.first, pix_ll.second);
      cv::Vec3f LR = transMat.at<cv::Vec3f>(pix_lr.first, pix_lr.second);

      double inten_UL = intensity.at<uchar>(pix_ul.first, pix_ul.second);
      double inten_UR = intensity.at<uchar>(pix_ur.first, pix_ur.second);
      double inten_LL = intensity.at<uchar>(pix_ll.first, pix_ll.second);
      double inten_LR = intensity.at<uchar>(pix_lr.first, pix_lr.second);

      // Add three vertices of a face
      add_vertex(UL, inten_UL, pix_ul, pix_to_vertex, vertex_count, vertices, colors);
      add_vertex(UR, inten_UR, pix_ur, pix_to_vertex, vertex_count, vertices, colors);
      add_vertex(LL, inten_LL, pix_ll, pix_to_vertex, vertex_count, vertices, colors);

      // Note how we add only valid faces, so all three vertices must be valid (non-zero)
      if (!invalidXyz(UL) && !invalidXyz(UR) && !invalidXyz(LL) &&
          inten_UL >= 0 && inten_UR >= 0 && inten_LL >= 0) {
        std::vector<size_t> face = {pix_to_vertex[pix_ul], pix_to_vertex[pix_ur],
                                    pix_to_vertex[pix_ll]};
        faces.push_back(face);
      }

      // Add the other face, forming a full grid cell
      add_vertex(UR, inten_UR, pix_ur, pix_to_vertex, vertex_count, vertices, colors);
      add_vertex(LR, inten_LR, pix_lr, pix_to_vertex, vertex_count, vertices, colors);
      add_vertex(LL, inten_LL, pix_ll, pix_to_vertex, vertex_count, vertices, colors);
      if (!invalidXyz(UR) && !invalidXyz(LR) && !invalidXyz(LL) &&
          inten_UR >= 0 && inten_LR >= 0 && inten_LL >= 0) {
        std::vector<size_t> face = {pix_to_vertex[pix_ur], pix_to_vertex[pix_lr],
                                    pix_to_vertex[pix_ll]};
        faces.push_back(face);
      }
    }
  }

  // Form and save the ply
  happly::PLYData ply;
  ply.addVertexPositions(vertices);
  ply.addVertexColors(colors);
  ply.addFaceIndices(faces);
  std::cout << "Writing: " << plyFileName << std::endl;
  ply.write(plyFileName, happly::DataFormat::ASCII);
}

// Save the depth clouds and optimized transforms needed to create a mesh with voxblox
// (if depth clouds exist).
void exportToVoxblox(std::vector<std::string> const& cam_names,
                     std::vector<rig::cameraImage> const& cam_images,
                     std::vector<Eigen::Affine3d> const& depth_to_image,
                     std::vector<Eigen::Affine3d> const& world_to_cam,
                     std::string const& out_dir) {

  if (cam_images.size() != world_to_cam.size())
    LOG(FATAL) << "There must be as many camera images as camera poses.\n";
  if (cam_names.size() != depth_to_image.size()) 
    LOG(FATAL) << "Must have as many camera types as depth-to-image transforms.\n";
  if (out_dir.empty())
    LOG(FATAL) << "The output directory is empty.\n";

  std::string voxblox_dir = out_dir + "/voxblox";
  rig::createDir(voxblox_dir);
  std::cout << "Saving voxblox data to: " << voxblox_dir << "\n";
  
  char timestamp_buffer[1000];

  // Must take a pass for each camera type, and then visit
  // all images while ignoring those of a different type. There are very
  // few camera types, so this is not unreasonable.
  // Keep the 2D image structure of the cloud. Replace the invalid
  // entries with Inf.
  double inf = std::numeric_limits<double>::infinity();
  for (size_t cam_type = 0; cam_type < cam_names.size(); cam_type++) {

    std::string voxblox_subdir = voxblox_dir + "/" + cam_names[cam_type];
    rig::createDir(voxblox_subdir);

    int num_saved_clouds = 0;
    std::string index_file = voxblox_subdir + "/index.txt";
    std::cout << "Writing: " << index_file << std::endl;
    std::ofstream ofs(index_file.c_str());

    for (size_t cid = 0; cid < cam_images.size(); cid++) {

      if (cam_images[cid].camera_type != cam_type) 
        continue;

      int depth_cols = cam_images[cid].depth_cloud.cols;
      int depth_rows = cam_images[cid].depth_cloud.rows;
      
      if (depth_cols == 0 || depth_rows == 0)
        continue; // skip empty clouds

      // Sanity check
      if (depth_cols != cam_images[cid].image.cols || 
          depth_rows != cam_images[cid].image.rows)
        LOG(FATAL) << "Found a depth cloud and corresponding image with "
                   << "mismatching dimensions.\n";
        
      // Must use the 10.7f format for the timestamp as everywhere else in the code,
      // as it is in double precision.
      double timestamp = cam_images[cid].timestamp;
      snprintf(timestamp_buffer, sizeof(timestamp_buffer), "%10.7f", timestamp);

      // Sanity check
      if (cam_images[cid].image.channels() != 1) 
        LOG(FATAL) << "Expecting a grayscale input image.\n";
      
      // Form the pcl point cloud. Add the first band of the image as an intensity.
      // TODO(oalexan1): Make this a subroutine
      pcl::PointCloud<pcl::PointNormal> pc;
      pc.width = depth_cols;
      pc.height = depth_rows;
      pc.points.resize(std::int64_t(pc.width) * std::int64_t(pc.height)); // avoid overflow
      int count = 0;
      for (int row = 0; row < depth_rows; row++) {
        for (int col = 0; col < depth_cols; col++) {
          cv::Vec3f xyz = cam_images[cid].depth_cloud.at<cv::Vec3f>(row, col);
          Eigen::Vector3d P;
          if (xyz == cv::Vec3f(0, 0, 0)) {
            P = Eigen::Vector3d(inf, inf, inf);
          } else{
            // Transform from depth cloud coordinates to camera coordinates
            // (later voxblox will transform to world coordinates)
            P = Eigen::Vector3d(xyz[0], xyz[1], xyz[2]);
            P = depth_to_image[cam_images[cid].camera_type] * P;
          }
          pc.points[count].x         = P[0];
          pc.points[count].y         = P[1];
          pc.points[count].z         = P[2];
          pc.points[count].normal_x  = cam_images[cid].image.at<uchar>(row, col); // intensity
          pc.points[count].normal_y  = 1.0; // weight
          pc.points[count].normal_y  = 1.0; // intersection err
          pc.points[count].normal_y  = 0.0; // ensure initialization
          
          count++;
        }
      }

      // Save the transform
      std::string transform_file = voxblox_subdir + "/" + timestamp_buffer + "_cam2world.txt";
      ofs << transform_file << "\n"; // save its name in the index
      rig::writeMatrix(world_to_cam[cid].inverse().matrix(), transform_file);

      // Save the pcd file
      std::string cloud_file = voxblox_subdir + "/" + timestamp_buffer + ".pcd";
      //std::cout << "Writing: " << cloud_file << std::endl;
      ofs << cloud_file << "\n"; // save its name in the index
      pcl::io::savePCDFileBinary(cloud_file, pc); // writing binary is much faster than ascii

      num_saved_clouds++;
    }
    
    if (num_saved_clouds == 0)
      std::cout << "No depth clouds were saved for camera: " << cam_names[cam_type]
                << ". Empty index file: " << index_file << ".\n";
    
  } // end loop over camera types

  return;
}
  
// Save the depth clouds and optimized transforms needed to create a mesh with voxblox
// (if depth clouds exist).
void saveTransformedDepthClouds(std::vector<std::string> const& cam_names,
                                std::vector<rig::cameraImage> const& cam_images,
                                std::vector<Eigen::Affine3d> const& depth_to_image,
                                std::vector<Eigen::Affine3d> const& world_to_cam,
                                std::string const& out_dir) {
  if (cam_images.size() != world_to_cam.size())
    LOG(FATAL) << "There must be as many camera images as camera poses.\n";
  if (cam_names.size() != depth_to_image.size()) 
    LOG(FATAL) << "Must have as many camera types as depth-to-image transforms.\n";
  
  if (out_dir.empty())
    LOG(FATAL) << "The output directory is empty.\n";

  std::string trans_depth_dir = out_dir + "/trans_depth";
  rig::createDir(trans_depth_dir);

  char timestamp_buffer[1000];

  // Must take a pass for each camera type, and then visit
  // all images while ignoring those of a different type. There are very
  // few camera types, so this is not unreasonable.

  for (size_t cam_type = 0; cam_type < cam_names.size(); cam_type++) {

    std::string trans_depth_subdir = trans_depth_dir + "/" + cam_names[cam_type];
    rig::createDir(trans_depth_subdir);

    for (size_t cid = 0; cid < cam_images.size(); cid++) {

      if (cam_images[cid].camera_type != cam_type) 
        continue;

      int depth_cols = cam_images[cid].depth_cloud.cols;
      int depth_rows = cam_images[cid].depth_cloud.rows;
      
      if (depth_cols == 0 || depth_rows == 0)
        continue; // skip empty clouds

      // Sanity check
      if (depth_cols != cam_images[cid].image.cols || 
          depth_rows != cam_images[cid].image.rows)
        LOG(FATAL) << "Found a depth cloud and corresponding image with mismatching dimensions.\n";
        
      // Must use the 10.7f format for the timestamp as everywhere else in the code,
      // as it is in double precision.
      double timestamp = cam_images[cid].timestamp;
      snprintf(timestamp_buffer, sizeof(timestamp_buffer), "%10.7f", timestamp);

      // Sanity check
      if (cam_images[cid].image.channels() != 1) 
        LOG(FATAL) << "Expecting a grayscale input image.\n";

      // Save the pcd file
      std::string cloud_file = trans_depth_subdir + "/" + timestamp_buffer + ".ply";

      // To go from the depth cloud to world coordinates need to first to go from depth
      // to image coordinates, then from image to world. 
      Eigen::Affine3d depth_to_world = (world_to_cam[cid].inverse())
        * depth_to_image[cam_images[cid].camera_type];
      saveTransformedMesh(cam_images[cid].depth_cloud, cam_images[cid].image,
                          depth_to_world, cloud_file);
      
    }
  }
  
}

// Find the depth measurement. Use nearest neighbor interpolation
// to look into the depth cloud.
bool depthValue(// Inputs
                cv::Mat const& depth_cloud, Eigen::Vector2d const& dist_ip,
                // Output
                Eigen::Vector3d& depth_xyz) {
  depth_xyz = Eigen::Vector3d(0, 0, 0);  // initialize

  if (depth_cloud.cols == 0 && depth_cloud.rows == 0)
    return false;  // empty cloud

  int col = round(dist_ip[0]);
  int row = round(dist_ip[1]);

  if (col < 0 || row < 0 || col > depth_cloud.cols || row > depth_cloud.rows)
    LOG(FATAL) << "Out of range in the depth cloud.";

  // After rounding one may hit the bound
  if (col == depth_cloud.cols || row == depth_cloud.rows)
    return false;

  cv::Vec3f cv_depth_xyz = depth_cloud.at<cv::Vec3f>(row, col);

  // Skip invalid measurements
  if (cv_depth_xyz == cv::Vec3f(0, 0, 0))
    return false;

  depth_xyz = Eigen::Vector3d(cv_depth_xyz[0], cv_depth_xyz[1], cv_depth_xyz[2]);

  return true;
}
  
}  // end namespace rig
