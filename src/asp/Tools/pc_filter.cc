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


/// \file pc_filter.cc.

/// Apply some filtering operations to a point cloud.

// TODO(oalexan1): Add ability to remove blobs
// Add median filter based on a window of given size and threshold.
// Make it apply a blur and fill in from neighbors
// See if saving .pcd as binary will speed up voxblox
// Save weight image via --weight-image
// Save the computed weight for each cloud point.
// Replace max_camera_dir_to_camera_ray_angle with
// --max-horizontal-camera-dir-to-camera-ray-angle
// --max-vertical-camera-dir-to-camera-ray-angle

#include <asp/Core/Macros.h>
#include <asp/Core/AspProgramOptions.h>
#include <asp/Core/StereoSettings.h>
#include <asp/Core/EigenUtils.h>
#include <asp/PclIO/PclIO.h>
#include <asp/Core/PointUtils.h>

#include <vw/FileIO/DiskImageUtils.h>
#include <vw/FileIO/MatrixIO.h>
#include <vw/Image/DistanceFunction.h>
#include <vw/Camera/PinholeModel.h>
#include <vw/Cartography/GeoReference.h>
#include <vw/FileIO/FileUtils.h>

#include <boost/filesystem.hpp>
#include <boost/algorithm/string.hpp>

#include <limits>

using namespace vw;
using namespace vw::cartography;
namespace po = boost::program_options;
namespace fs = boost::filesystem;

struct Options: vw::GdalWriteOptions {
  bool transform_to_camera_coordinates;
  std::string input_cloud, input_texture, output_cloud, output_weight, camera_file;
  double max_valid_triangulation_error, max_distance_from_camera,
    max_camera_ray_to_surface_normal_angle, max_camera_dir_to_surface_normal_angle,
    max_camera_dir_to_camera_ray_angle, reliable_surface_resolution,
    distance_from_camera_weight_power, blending_dist, blending_power;

  // The class members will be initialized when parsing happens
  Options() {}
};

void handle_arguments(int argc, char *argv[], Options& opt) {

  po::options_description general_options("General options");
  general_options.add_options()
    ("input-cloud", po::value(&opt.input_cloud)->default_value(""),
     "Input cloud name. A four-band .tif file as produced by stereo triangulation.")
    ("output-cloud", po::value(&opt.output_cloud)->default_value(""),
     "Output cloud name. If having a .tif extension, the same format will be used as the input. Can also save .pcd and .ply files (only vertices are saved, not faces). In those cases the points will be saved with float32 values, so there may be some precision loss. The .pcd file will store in the field for the cloud normal the values image_texture, blending_weight, intersection_error, assuming these are computed.")
    ("input-texture", po::value(&opt.input_texture)->default_value(""),
     "If specified, read the texture from this file. Normally this is the file L.tif from the same run which produced the input point cloud.")
    ("camera", po::value(&opt.camera_file)->default_value(""),
     "The left or right camera used to produce this cloud. Used for some filtering operations.")
    ("max-distance-from-camera", po::value(&opt.max_distance_from_camera)->default_value(0),
     "If positive, remove points further from camera center than this value. Measured in meters.")
    ("max-valid-triangulation-error", po::value(&opt.max_valid_triangulation_error)->default_value(0),
     "If positive, points with triangulation error larger than this will be removed from the cloud. Measured in meters.")
    ("max-camera-ray-to-surface-normal-angle", po::value(&opt.max_camera_ray_to_surface_normal_angle)->default_value(0),
     "If positive, points whose surface normal makes an angle with the ray back to the camera center greater than this will be removed as outliers. Measured in degrees.")
    ("max-camera-dir-to-surface-normal-angle", po::value(&opt.max_camera_dir_to_surface_normal_angle)->default_value(0),
     "If positive, points whose surface normal makes an angle with the camera direction greater than this will be removed as outliers. This eliminates surfaces almost parallel to camera view direction. Measured in degrees.")
      ("max-camera-dir-to-camera-ray-angle", po::value(&opt.max_camera_dir_to_camera_ray_angle)->default_value(0),
       "If positive, and a ray emanating from the camera and ending at the current point makes an angle with the camera direction bigger than this, remove the point as an outlier. In effect, this narrows the camera field of view.")
    ("distance-from-camera-weight-power", po::value(&opt.distance_from_camera_weight_power)->default_value(0),
     "If positive, let the weight of a point be inversely proportional to the distance from the camera center to the point, raised to this power.")
    ("blending-dist", po::value(&opt.blending_dist)->default_value(0.0),
     "If positive and closer to any boundary of valid points than this (measured in point cloud pixels), decrease the weight assigned to the given point proportionally to remaining distance to boundary raised to a power. In effect, points closer to boundary are given less weight. Used in VoxBlox.")
    ("blending-power", po::value(&opt.blending_power)->default_value(1.0),
     "Use this as the power when setting --blending-dist.")
    ("reliable-surface-resolution", po::value(&opt.reliable_surface_resolution)->default_value(0),
     "If positive, let each point's weight be proportional to exp(-curr_surface_resolution/reliable_surface_resolution). This should be set to about half the expected surface resolution, to have the weight of points at lower resolution decrease rather fast. A point's surface resolution is the maximum distance between it and its immediate neighbors.")
    ("transform-to-camera-coordinates",         po::bool_switch(&opt.transform_to_camera_coordinates)->default_value(false),
     "Transform the point cloud to the coordinate system of the camera provided with --camera. For use with VoxBlox.")
    ("output-weight", po::value(&opt.output_weight)->default_value(""),
     "If specified, Save the per-pixel weight to this file. This has the same dimensions as the point cloud and ``L.tif``.(Use the .tif extension.)");

  general_options.add(vw::GdalWriteOptionsDescription(opt));

  po::options_description positional("");
  po::positional_options_description positional_desc;

  std::string usage("--input-cloud input.tif --output-cloud output.tif [other options]");
  bool allow_unregistered = false;
  std::vector<std::string> unregistered;
  po::variables_map vm =
    asp::check_command_line(argc, argv, opt, general_options, general_options,
                             positional, positional_desc, usage,
                            allow_unregistered, unregistered);

  // Validation
  
  if ((opt.transform_to_camera_coordinates            ||
       opt.max_camera_ray_to_surface_normal_angle > 0 ||
       opt.max_camera_dir_to_surface_normal_angle > 0 ||
       opt.max_camera_dir_to_camera_ray_angle > 0)  && opt.camera_file.empty()) 
    vw_throw(ArgumentErr() << "Some of the requested operations require "
              << "a camera model specified via --camera.");

  if (opt.input_cloud.empty() || opt.output_cloud.empty()) 
    vw_throw(ArgumentErr() << "The input and output point clouds must be specified.");

  if (opt.blending_dist > 0 && opt.blending_power <= 0) 
    vw_throw(ArgumentErr() << "When setting --blending-dist, a "
              << "positive value of blending power must be used.");

    // Create the output directory
    vw::create_out_dir(opt.output_cloud);

    // Turn on logging to file
    std::string prog_name = asp::extract_prog_name(argv[0]);
    asp::log_to_file(argc, argv, "", opt.output_cloud);
  
  return;
}

// Apply a rigid transforms to a point cloud in-place. Points at
// origin are considered outliers and are unchanged.
void applyAffineTransform(ImageView<Vector<double, 4>> & pc, vw::Matrix<double> const& T) {

  if (T.rows() != 4 && T.cols() != 4 && T(3, 3) != 1.0)
    vw_throw(ArgumentErr() << "Expecting a 4x4 affine transform.");

  for (int col = 0; col < pc.cols(); col++) {
    for (int row = 0; row < pc.rows(); row++) {
      
      if (subvector(pc(col, row), 0, 3) == Vector3())
        continue; // outlier
      
      // Create homogeneous coordinates
      Vector<double, 4> Q;
      subvector(Q, 0, 3) = subvector(pc(col, row), 0, 3);
      Q[3] = 1; 
        
      // Apply transform
      Q = T * Q;

      // Copy back
      subvector(pc(col, row), 0, 3) = subvector(Q, 0, 3);
    }
  }
}

// Find the surface normal at a given pixel based on the 3x3 neighborhood.
// Return true on success.
bool surfaceNormal(ImageView<Vector<double, 4>> const& point_image,
                   int col, int row, Vector3 & N) {

  // Initialize the output
  N = Vector3();

  // Find the nearby points
  std::vector<Eigen::Vector3d> near_points;
  for (int c = col - 1; c <= col + 1; c++) {
    for (int r = row - 1; r <= row + 1; r++) {
      if (c < 0 || c >= point_image.cols()) 
        continue;
      if (r < 0 || r >= point_image.rows()) 
        continue;
      
      Vector<double, 4> const& Q = point_image(c, r);
      if (Q == Vector<double, 4>()) 
        continue; // outlier

      near_points.push_back(Eigen::Vector3d(Q[0], Q[1], Q[2]));
    }
  }
  
  if (near_points.size() < 5) 
    return false;
  
  Eigen::Vector3d plane_normal, centroid;
  asp::bestFitPlane(near_points, centroid, plane_normal);
  N = Vector3(plane_normal[0], plane_normal[1], plane_normal[2]);

  if (N != N) 
    return false; // NaN
  
  return true;
}

// Estimate surface resolution by looking at four immediate neighbors. 
// Return 0 if at least 2 neighbors are missing or the current point.
void estimate_surface_res(ImageView<Vector<double, 4>> const& point_image,
                          ImageView<float> & surface_res) {

  int offset_x[] = {-1, 0, 0, 1};
  int offset_y[] = {0, -1, 1, 0};
  
  surface_res = ImageView<float>(point_image.cols(), point_image.rows());
  for (int col = 0; col < surface_res.cols(); col++) {
    for (int row = 0; row < surface_res.rows(); row++) {

      surface_res(col, row) = 0.0;

      Vector<double, 4> const& P = point_image(col, row); // alias
      if (P == Vector<double, 4>()) 
        continue; // outlier

      int count = 0;
      double dist = 0.0;
      for (int it = 0; it < 4; it++) {
        int c = col + offset_x[it];
        int r = row + offset_y[it];

        if (c < 0 || c >= point_image.cols()) 
          continue;
        if (r < 0 || r >= point_image.rows()) 
          continue;

        Vector<double, 4> const& Q = point_image(c, r); // alias
        if (Q == Vector<double, 4>()) 
          continue; // outlier

        dist = std::max(dist, norm_2(subvector(P, 0, 3) - subvector(Q, 0, 3)));
        count++;
      }

      if (count < 3)
        continue; // too few neighbors

      surface_res(col, row) = dist;
    }
  }
}

int main(int argc, char *argv[]) {
  Options opt;
  try {
    handle_arguments(argc, argv, opt);

    // For now only 4 channels are supported
    // TODO(oalexan1): Support also 3 channels
    int num_channels = get_num_channels(opt.input_cloud);
    if (num_channels != 4) 
      vw_throw(ArgumentErr() << "The input point cloud must have 4 channels.");

    std::string ext = fs::path(opt.output_cloud).extension().string();
    boost::algorithm::to_lower(ext);
    if (ext != ".tif" && ext != ".pcd" && ext != ".ply") 
      vw_throw(ArgumentErr() << "The output point cloud extension must be .tif, .pcd, or .ply.");
    
    // Read the point cloud fully in memory, and remove any offset
    ImageView<Vector<double, 4>> point_image = asp::read_asp_point_cloud<4>(opt.input_cloud);
    std::cout << "Read point cloud: " << opt.input_cloud << std::endl;

    // Load the texture or set it to 1
    ImageView<float> texture;
    bool has_texture_nodata = false;
    float texture_nodata = -32768.0;
    if (opt.input_texture != "") {
      std::cout << "Read texture file: " << opt.input_texture << std::endl;
      texture = DiskImageView<float>(opt.input_texture);
      if (vw::read_nodata_val(opt.input_texture, texture_nodata)) {
        has_texture_nodata = true;
        std::cout << "Read texture nodata value: " << texture_nodata << std::endl;
      }
    } else {
      texture = ImageView<float>(point_image.cols(), point_image.rows());
      for (int col = 0; col < texture.cols(); col++) {
        for (int row = 0; row < texture.rows(); row++) {
          texture(col, row) = 1.0;
        }
      }
    }

    // Sanity check
    if (point_image.cols() != texture.cols() || point_image.rows() != texture.rows())
    vw_throw(ArgumentErr() << "The input point cloud and texture must have the same dimensions.");

    // Load the camera
    vw::camera::PinholeModel cam;
    vw::Matrix<double> cam2world = vw::math::identity_matrix(4);
    if (opt.camera_file != "") {
      vw_out() << "Reading camera mode: " << opt.camera_file << "\n";
      cam = vw::camera::PinholeModel(opt.camera_file);
      vw::math::submatrix(cam2world, 0, 0, 3, 3) = cam.get_rotation_matrix();
      for (int row = 0; row < 3; row++) 
        cam2world(row, 3) = cam.camera_center()[row];
      vw_out() << "Camera-to-world transform: " << cam2world << "\n";
    }
    vw::Matrix<double> world2cam = inverse(cam2world);

    // Transform the cloud to camera coordinates (if world2cam is read)
    applyAffineTransform(point_image, world2cam);

    ImageView<float> weight(point_image.cols(), point_image.rows());
    ImageView<float> out_texture(point_image.cols(), point_image.rows());
    ImageView<Vector<double, 4>> clean_points(point_image.cols(), point_image.rows());
    for (int col = 0; col < point_image.cols(); col++) {
      for (int row = 0; row < point_image.rows(); row++) {
        weight(col, row) = 0.0;
        out_texture(col, row) = 0.0;
        clean_points(col, row) = Vector<double, 4>();
      }
    }

    ImageView<float> surface_res;
    if (opt.reliable_surface_resolution > 0) 
      estimate_surface_res(point_image, surface_res);

    // Camera direction, in camera's coordinate system
    Vector3 cam_dir(0.0, 0.0, 1.0);

    for (int col = 0; col < point_image.cols(); col++) {
      for (int row = 0; row < point_image.rows(); row++) {
        
        Vector<double, 4> const& P = point_image(col, row); // alias
        if (subvector(P, 0, 3) == Vector3() ||
            (has_texture_nodata && texture(col, row) == texture_nodata) ||
            (opt.max_valid_triangulation_error > 0 && P[3] > opt.max_valid_triangulation_error)) {
          continue; // outlier
        }

        // The first 3 coordinates of P
        Vector3 Q = subvector(P, 0, 3);

        if (opt.max_distance_from_camera > 0 &&
            norm_2(Q) > opt.max_distance_from_camera) {
          continue; // outlier
        }
        
        // All points are given equal weight for now
        double wt = 1.0;
        if (opt.distance_from_camera_weight_power > 0) {
          double dist = norm_2(Q);
          if (dist == 0.0) 
            continue; // outlier
          
          wt = 1.0 / pow(dist, opt.distance_from_camera_weight_power);
        }

        if (opt.max_camera_ray_to_surface_normal_angle > 0 ||
            opt.max_camera_dir_to_surface_normal_angle > 0) {

          // Find the surface normal
          Vector3 N;
          if (!surfaceNormal(point_image, col, row, N))
            continue; // outlier
            
          if (opt.max_camera_ray_to_surface_normal_angle > 0) {
            // Use abs as the normal can point in either direction
            double prod = std::abs(dot_prod(Q, N) / norm_2(Q) / norm_2(N));
            double angle = acos(prod) * (180.0 / M_PI);
            if (std::isnan(angle) || std::isinf(angle) ||
                angle > opt.max_camera_ray_to_surface_normal_angle)
              continue; // something went wrong
            
          } else if (opt.max_camera_dir_to_surface_normal_angle > 0) {
            double prod = std::abs(dot_prod(cam_dir, N) / norm_2(cam_dir) / norm_2(N));
            double angle = acos(prod) * (180.0 / M_PI);
            if (std::isnan(angle) || std::isinf(angle) ||
                angle > opt.max_camera_dir_to_surface_normal_angle)
              continue; // something went wrong
          }
        }
        
        if (opt.max_camera_dir_to_camera_ray_angle > 0) {

          double prod = std::abs(dot_prod(Q, cam_dir)/ norm_2(Q) / norm_2(cam_dir));
          double angle = acos(prod) * (180.0 / M_PI);
          if (std::isnan(angle) || std::isinf(angle))
            continue; // something went wrong
          
          if (angle > opt.max_camera_dir_to_camera_ray_angle) 
            continue; // outlier
        }
        
        // The input texture is usually between 0 and 1.
        // TODO(oalexan1): What is the max color?
        double t = 255.0 * texture(col, row);
        if (t <= 0.0) t = 1.0;  // Ensure a positive value for the color

        // Note how we add back the triangulation error
        clean_points(col, row) = Vector<double, 4>(Q[0], Q[1], Q[2], P[3]);
        out_texture(col, row) = t;
        weight(col, row) = wt;
      }
    }

    if (opt.blending_dist > 0 && opt.blending_power > 0) {
      ImageView<int> mask(clean_points.cols(), clean_points.rows());
      for (int col = 0; col < clean_points.cols(); col++) {
        for (int row = 0; row < clean_points.rows(); row++) {
          mask(col, row) = (subvector(clean_points(col, row), 0, 3) != Vector3());
        }
      }
      ImageView<double> dist;
      vw::bounded_dist(mask, opt.blending_dist, dist);
      
      // Adjust the weight by the normalized distance raised to given power
      for (int col = 0; col < dist.cols(); col++) {
        for (int row = 0; row < dist.rows(); row++) {
          dist(col, row) = pow(dist(col, row) / opt.blending_dist, opt.blending_power);
          weight(col, row) *= dist(col, row);
        }
      }

    }

    if (opt.reliable_surface_resolution > 0) {
      for (int col = 0; col < weight.cols(); col++) {
        for (int row = 0; row < weight.rows(); row++) {
          if (weight(col, row) <= 0) 
            continue;
          
          if (surface_res(col, row) <= 0) {
            weight(col, row) = 0.0;
            continue;
          }

          weight(col, row) *= exp(-surface_res(col, row) / opt.reliable_surface_resolution);
        }
      }
    }
    
    if (!opt.transform_to_camera_coordinates)
      applyAffineTransform(clean_points, cam2world); // convert back to world
    
    // Save as .tif, .pcd, or .pcl
    if (ext == ".tif") {
      vw::cartography::GeoReference georef;
      bool has_georef = vw::cartography::read_georeference(georef, opt.input_cloud);
      bool has_nodata = false;
      double nodata = 0.0;
      vw_out() << "Writing: " << opt.output_cloud << "\n";
      vw::cartography::block_write_gdal_image
        (opt.output_cloud, clean_points, has_georef, georef, has_nodata, nodata, opt,
         TerminalProgressCallback("asp", "\t--> Filter: "));
    } else {
      asp::writeCloud(clean_points, out_texture, weight, opt.output_cloud);
    }

    if (opt.output_weight != "") {
      vw::cartography::GeoReference georef;
      bool has_georef = false;
      bool has_nodata = false;
      double nodata = 0;
      vw_out() << "Writing weights: " << opt.output_weight << "\n";
      vw::cartography::block_write_gdal_image
        (opt.output_weight, weight, has_georef, georef, has_nodata, nodata, opt,
       TerminalProgressCallback("asp", "\t--> Weight per point:"));
    }
    
    return 0;
    
  } ASP_STANDARD_CATCHES;

  return 0;
}
