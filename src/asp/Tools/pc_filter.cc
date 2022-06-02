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

// TODO(oalexan1): Use --max-vertical-angle-from-camera-dir-to-ray
// --max-horizontal-angle-from-camera-dir-to-ray

#include <asp/Core/Macros.h>
#include <asp/Core/Common.h>
#include <asp/Core/StereoSettings.h>
#include <asp/Core/EigenUtils.h>
#include <asp/PclIO/PclIO.h>

#include <vw/FileIO/DiskImageUtils.h>
#include <vw/FileIO/MatrixIO.h>
#include <vw/Image/DistanceFunction.h>

#include <limits>

using namespace vw;
using namespace vw::cartography;
namespace po = boost::program_options;
namespace fs = boost::filesystem;

struct Options: vw::cartography::GdalWriteOptions {
  bool save_nodata_as_infinity;
  std::string input_cloud, input_texture, cam2world_transform, output_cloud;
  double      max_valid_triangulation_error, distance_from_camera_weight_power,
    max_camera_ray_to_surface_normal_angle, max_camera_dir_to_camera_ray_angle,
    blending_dist, blending_power;

  // The value will be initialized when parsing happens
  Options() {}
};

void handle_arguments( int argc, char *argv[], Options& opt ) {

  po::options_description general_options("General options");
  general_options.add_options()
    ("input-cloud", po::value(&opt.input_cloud)->default_value(""),
     "Input cloud name. A four-band .tif file as produced by stereo triangulation.")
    ("output-cloud", po::value(&opt.output_cloud)->default_value(""),
     "Output cloud name. If having a .tif extension, the same format will be used as the input. Can also save .pcd and .ply file; in that case the points will be saved as float32 values, so there may be some precision loss. The .pcd file will store in the field for the cloud normal the values (image_texture, blending_weight, intersection_error), assuming these are computed.")
    ("input-texture", po::value(&opt.input_texture)->default_value(""),
     "If specified, read the texture from this file. Normally it is the file L.tif from the same run which produced the input point cloud.")
    ("cam2world-transform", po::value(&opt.cam2world_transform)->default_value(""),
     "If specified, read transform from camera to world a a 4x4 matrix.")
    ("max-valid-triangulation-error", po::value(&opt.max_valid_triangulation_error)->default_value(0),
     "Outlier removal based on threshold. If positive, points with triangulation error larger than this will be removed from the cloud. Measured in meters.")
    ("max-camera-ray-to-surface-normal-angle", po::value(&opt.max_camera_ray_to_surface_normal_angle)->default_value(0),
     "If positive, surface points whose surface normal makes an angle with the ray back to the camera center greater than this will be removed as an outlier.")
      ("max-camera-dir-to-camera-ray-angle", po::value(&opt.max_camera_dir_to_camera_ray_angle)->default_value(0),
       "If positive, and a ray emanating from the camera and ending at the current point makes an angle with the camera direction bigger than this, remove the point as an outlier. In effect, this narrows the camera field of view.")
    ("distance-from-camera-weight-power", po::value(&opt.distance_from_camera_weight_power)->default_value(0),
     "If positive, let the weight of a 3D point be inversely proportional to the distance from the camera center to the point, raised to this power.")
    ("blending-dist", po::value(&opt.blending_dist)->default_value(0),
     "If positive and closer to any boundary than this (measured in pixels), decrease the weight assigned to the given point proportionally to remaining distance to boundary raised to a power. In effect, points closer to boundary are given less weight.")
    ("blending-power", po::value(&opt.blending_power)->default_value(0),
     "If positive, use this as the power when setting --blending-dist.")
    ("save-nodata-as-infinity",         po::bool_switch(&opt.save_nodata_as_infinity)->default_value(false),
     "If true and saving a .pcd file, set the x, y, z coordinates of an invalid point to infinity rather than to 0.");

  general_options.add(vw::cartography::GdalWriteOptionsDescription(opt));

  po::options_description positional("");
  po::positional_options_description positional_desc;

  std::string usage("[options]");
  bool allow_unregistered = false;
  std::vector<std::string> unregistered;
  po::variables_map vm =
    asp::check_command_line(argc, argv, opt, general_options, general_options,
                             positional, positional_desc, usage,
                             allow_unregistered, unregistered);
}

int main(int argc, char *argv[]) {
  Options opt;
  try {
    handle_arguments(argc, argv, opt);

    // For now only 4 channels are supported
    // TODO(oalexan1): Support also 3 channels
    int num_channels = get_num_channels(opt.input_cloud);
    if (num_channels != 4) 
      vw_throw( ArgumentErr() << "The input point cloud must have 4 channels.");

    // Read the point cloud fully in memory, and remove any offset
    std::string cloud_file = opt.input_cloud;
    ImageViewRef<Vector<double, 4>> point_image = asp::read_asp_point_cloud<4>(cloud_file);
    std::cout << "Read point cloud: " << cloud_file << std::endl;
    
    ImageView<float> texture = DiskImageView<float>(opt.input_texture);
    std::cout << "Read texture file: " << opt.input_texture << std::endl;
    bool has_nodata = false;
    float texture_nodata = -32768.0;
    if (vw::read_nodata_val(opt.input_texture, texture_nodata)) {
      has_nodata = true;
      std::cout << "Read texture nodata value: " << texture_nodata << std::endl;
    }

    if (point_image.cols() != texture.cols() || point_image.rows() != texture.rows())
    vw_throw( ArgumentErr() << "The input point cloud and texture must have the same dimensions.");

    vw::Matrix<double> cam2world;
    vw_out() << "Reading the alignment transform from: " << opt.cam2world_transform << "\n";
    vw::read_matrix_as_txt(opt.cam2world_transform, cam2world);
    std::cout << "Read transform " << cam2world << std::endl;
    vw::Matrix<double> world2cam = inverse(cam2world);

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

    double max_err = opt.max_valid_triangulation_error;
    for (int col = 0; col < point_image.cols(); col++) {
      for (int row = 0; row < point_image.rows(); row++) {
        
        Vector<double, 4> P = point_image(col, row);
        if (subvector(P, 0, 3) == Vector3() || texture(col, row) == texture_nodata ||
            (max_err > 0 && P[3] > max_err)) {
          continue; // outlier
        }

        // Create homogeneous coordinates
        Vector<double, 4> Q;
        subvector(Q, 0, 3) = subvector(P, 0, 3);
        Q[3] = 1; 
        
        // Convert to camera's coordinate system, as expected by voxblox
        Q = world2cam * Q;

        // All points are given equal weight for now
        double wt = 1.0;
        if (opt.distance_from_camera_weight_power > 0) {
          double dist = norm_2(subvector(Q, 0, 3));
          if (dist == 0.0) 
            continue; // outlier
          
          wt = 1.0 / pow(dist, opt.distance_from_camera_weight_power);
        }

        if (opt.max_camera_ray_to_surface_normal_angle > 0) {
          // See the angle between camera ray and surface normal
          std::vector<Eigen::Vector3d> near_points;
          for (int c = col - 1; c <= col + 1; c++) {
            for (int r = row - 1; r <= row + 1; r++) {
              if (c < 0 || c >= point_image.cols()) 
                continue;
              if (r < 0 || r >= point_image.rows()) 
                continue;

              // TODO(oalexan1): Too much code repetition here
              Vector<double, 4> P0 = point_image(c, r);
              if (P0 == Vector<double, 4>()) 
                continue; // outlier
              // Create homogeneous coordinates
              Vector<double, 4> Q0;
              subvector(Q0, 0, 3) = subvector(P0, 0, 3);
              Q0[3] = 1; 
              // Convert to camera's coordinate system, as expected by voxblox
              Q0 = world2cam * Q0;
              
              near_points.push_back(Eigen::Vector3d(Q0[0], Q0[1], Q0[2]));
            }
          }
          
          if (near_points.size() < 5) 
            continue; // outlier
          Eigen::Vector3d plane_normal, centroid;
          asp::bestFitPlane(near_points, centroid, plane_normal);
          
          Vector3 N(plane_normal[0], plane_normal[1], plane_normal[2]);
          // Use abs as the normal can point in either direction
          double prod = std::abs(dot_prod(Q, N)/ norm_2(Q) / norm_2(N));
          
          double angle = acos(prod) * (180.0 / M_PI);
          if (std::isnan(angle) || std::isinf(angle))
            continue; // something went wrong
          
          if (angle > opt.max_camera_ray_to_surface_normal_angle) 
            continue; // outlier
        }

        if (opt.max_camera_dir_to_camera_ray_angle > 0) {
          Vector3 cam_dir(0.0, 0.0, 1.0);

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
      
#if 0
      // TODO(oalexan1): There must be an option to save the weight for inspection
      vw::cartography::GeoReference georef;
      bool has_georef = false;
      bool has_nodata = false;
      double nodata = 0;
      std::string output_image = "tmp.tif";
      vw_out() << "Writing: " << output_image << "\n";
      vw::cartography::block_write_gdal_image
        (output_image, dist, has_georef, georef, has_nodata, nodata, opt,
         TerminalProgressCallback("asp", "\t--> Correlation quality:"));
#endif
      
      for (int col = 0; col < dist.cols(); col++) {
        for (int row = 0; row < dist.rows(); row++) {
          // Normalize to [0, 1]
          dist(col, row) = pow(dist(col, row) / opt.blending_dist, opt.blending_power);
          // Use this to adjust the weight
          weight(col, row) *= dist(col, row);
        }
      }

    }

    asp::writeCloud(clean_points, out_texture, weight, opt.save_nodata_as_infinity,
                    opt.output_cloud);
    
    return 0;
    
  } ASP_STANDARD_CATCHES;

  return 0;
}
