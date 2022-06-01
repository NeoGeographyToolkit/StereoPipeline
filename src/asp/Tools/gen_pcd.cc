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


/// \file gen_pcd.cc.
/// Create a .pcd file and apply a transform to it. This is an undocumented
/// low-level utility which is still not fully developed and hooked
/// up to other functionality.

// TODO(oalexan1): Clean up these header files

// TODO(oalexan1): Use --max-vertical-angle-from-camera-dir-to-ray
// --max-horizontal-angle-from-camera-dir-to-ray

// TODO(oalexan1): Move bestFitPlane to Eigen utils and do not include any Eigen header.
// Eigen, PCL, and VW combined make compilation impossibly long.
// Find the best fitting plane to a set of points.

// Also see if one can take out the pcl header

#include <asp/Core/PointUtils.h>
#include <asp/Core/Macros.h>
#include <asp/Core/Common.h>
#include <asp/Core/StereoSettings.h>
#include <asp/PclIO/PclIO.h>

#include <vw/Core/Stopwatch.h>
#include <vw/Core/StringUtils.h>
#include <vw/Cartography/GeoReferenceUtils.h>
#include <vw/Mosaic/ImageComposite.h>
#include <vw/FileIO/DiskImageUtils.h>
#include <vw/InterestPoint/MatrixIO.h>

#include <Eigen/Core>
#include <Eigen/SVD>

#include <limits>

using namespace vw;
using namespace vw::cartography;
namespace po = boost::program_options;
namespace fs = boost::filesystem;

struct Options : vw::cartography::GdalWriteOptions {
  // Input
  std::vector<std::string> files;
  Vector2     remove_outliers_params;
  double      max_valid_triangulation_error, distance_from_camera_weight_power,
    max_camera_ray_to_surface_normal_angle, max_camera_dir_to_camera_ray_angle,
    blending_dist, blending_power;

  // Defaults that the user doesn't need to see.
  Options() {}
};

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

// Find the Euclidean distance function to boundary of Vector3()
// pixels. Note that pixels at the edges of the image are considered
// invalid. Invalid pixels have a distance of 0. The distance is
// positive for all valid pixels, increasing when moving away from
// invalid pixels, till the magnitude reaches current value where it
// stops growing.

void boundary_dist(ImageViewRef<Vector3> image, double max_dist, ImageView<double> & dist) {
  
  if (max_dist < 0.0) 
    vw_throw(ArgumentErr() << "Expecting positive max_dist.");
  
  int max_dist_int = ceil(max_dist); // an int overestimate
  double max_dist_sq = max_dist * max_dist;
  int cols = image.cols(), rows = image.rows();
  dist.set_size(cols, rows);

  for (int col = 0; col < cols; col++) {
    for (int row = 0; row < rows; row++) {
      // Initialize to the max for valid pixels and not at image edges.
      if (image(col, row) != Vector3() && col > 0 && col < cols - 1 && row > 0 && row < rows - 1)
        dist(col, row) = max_dist;
      else
        dist(col, row) = 0;
    }
  }

  // For each boundary pixel (col, row), which is an invalid pixel
  // with a valid neighbors, adjust the values of dist which are
  // affected by it.
  for (int col = 0; col < cols; col++) {
    for (int row = 0; row < rows; row++) {

      // Look for a boundary pixel
      if (dist(col, row) > 0)
        continue; // valid, but need invalid
      bool is_bd_pix = false;
      for (int c = col - 1; c <= col + 1; c++) {
        for (int r = row - 1; r <= row + 1; r++) {
          if (c < 0 || c >= cols || r < 0 || r >= rows)
            continue;
          if (dist(c, r) > 0) {
            is_bd_pix = true; // has valid neighbor
            break;
          }
        }
        if (is_bd_pix) 
          break;
      }
      if (!is_bd_pix) 
        continue; // does not have valid neighbors

      // Found a boundary pixel. Pixels closer to it than max_dist
      // may have their distance function modified.
      for (int c = col - max_dist_int; c <= col + max_dist_int; c++) {
        for (int r = row - max_dist_int; r <= row + max_dist_int; r++) {
          if (c < 0 || c >= cols || r < 0 || r >= rows)
            continue;

          // Cast to double before multiplying to avoid integer overflow
          double dsq = double(c - col) * double(c - col) + 
            double(r - row) * double(r - row);

          // Too far 
          if (dsq >= max_dist_sq) 
            continue;

          if (dist(c, r) > 0) {
            double d = sqrt(dsq);
            // If this was positive before, it stays positive, as d > 0, since
            // it is the distance between a valid and an invalid pixel
            dist(c, r) = std::min(dist(c, r), d);
          }
        }
      }
        
    }
  }
}

void handle_arguments( int argc, char *argv[], Options& opt ) {


  po::options_description general_options("General options");
  general_options.add_options()
    ("remove-outliers-params",        po::value(&opt.remove_outliers_params)->default_value(Vector2(75.0, 3.0), "pct factor"),
            "Outlier removal based on percentage. Points with triangulation error larger than pct-th percentile times factor and points too far from the cluster of most points will be removed as outliers. [default: pct=75.0, factor=3.0]")
    ("max-valid-triangulation-error", po::value(&opt.max_valid_triangulation_error)->default_value(0),
     "Outlier removal based on threshold. If positive, points with triangulation error larger than this will be removed from the cloud. Measured in meters. This option takes precedence over --remove-outliers-params.")
    ("distance-from-camera-weight-power", po::value(&opt.distance_from_camera_weight_power)->default_value(0),
     "If positive, let the weight of a 3D point be inversely proportional to the distance from the camera center to the point, raised to this power.")

    ("max-camera-ray-to-surface-normal-angle", po::value(&opt.max_camera_ray_to_surface_normal_angle)->default_value(0),
     "If positive, surface points whose surface normal makes an angle with the ray back to the camera center greater than this will be removed as an outlier.")
      ("max-camera-dir-to-camera-ray-angle", po::value(&opt.max_camera_dir_to_camera_ray_angle)->default_value(0),
       "If positive, and a ray emanating from the camera and ending at the current point makes an angle with the camera direction bigger than this, remove the point as an outlier. In effect, this narrows the camera field of view.")
    ("blending-dist", po::value(&opt.blending_dist)->default_value(0),
     "If positive and closer to any boundary than this (measured in pixels), decrease the weight assigned to the given point proportionally to remaining distance to boundary raised to a power. In effect, points closer to boundary are given less weight.")
    ("blending-power", po::value(&opt.blending_power)->default_value(0),
     "If positive, use this as the power when setting --blending-dist.");
  
  po::options_description positional("");
  positional.add_options()
    ("input-files", po::value< std::vector<std::string> >(), "Input files");

  po::positional_options_description positional_desc;
  positional_desc.add("input-files", -1);

  std::string usage("[options] <point-clouds> [ --orthoimage <textures> ]");
  bool allow_unregistered = false;
  std::vector<std::string> unregistered;
  po::variables_map vm =
    asp::check_command_line(argc, argv, opt, general_options, general_options,
                             positional, positional_desc, usage,
                             allow_unregistered, unregistered);

  if (vm.count("input-files") == 0)
    vw_throw( ArgumentErr() << "Missing input point clouds.\n" << usage << general_options );
  opt.files = vm["input-files"].as< std::vector<std::string> >();

}

int main( int argc, char *argv[] ) {
  Options opt;
  try {
    handle_arguments(argc, argv, opt);

    // For now only 4 channels are supported
    // TODO(oalexan1): Support also 3 channels
    int num_channels = get_num_channels(opt.files[0]);
    if (num_channels != 4) 
      vw_throw( ArgumentErr() << "The input point cloud must have 4 channels.");

    // Read the point cloud fully in memory, and remove any offset
    std::string cloud_file = opt.files[0];
    ImageViewRef<Vector<double, 4>> point_image = asp::read_asp_point_cloud<4>(cloud_file);
    std::cout << "Read point cloud: " << cloud_file << std::endl;
    
    std::string texture_file = opt.files[1];
    ImageView<float> texture = DiskImageView<float>(texture_file);
    std::cout << "Read texture file: " << texture_file << std::endl;
    bool has_nodata = false;
    float texture_nodata = -32768.0;
    if (vw::read_nodata_val(texture_file, texture_nodata)) {
      has_nodata = true;
      std::cout << "Read texture nodata value: " << texture_nodata << std::endl;
    }

    if (point_image.cols() != texture.cols() || point_image.rows() != texture.rows())
    vw_throw( ArgumentErr() << "The input point cloud and texture must have the same dimensions.");

    vw::Matrix<double> cam2world;
    std::string trans_file = opt.files[2];
    vw_out() << "Reading the alignment transform from: " << trans_file << "\n";
    vw::read_matrix_as_txt(trans_file, cam2world);
    std::cout << "Read transform " << cam2world << std::endl;
    vw::Matrix<double> world2cam = inverse(cam2world);

    std::string pcd_file = opt.files[3];

    ImageView<float> weight(point_image.cols(), point_image.rows());
    ImageView<float> out_texture(point_image.cols(), point_image.rows());
    ImageView<Vector3> clean_points(point_image.cols(), point_image.rows());
    for (int col = 0; col < point_image.cols(); col++) {
      for (int row = 0; row < point_image.rows(); row++) {
        weight(col, row) = 0.0;
        out_texture(col, row) = 0.0;
        clean_points(col, row) = Vector3();
      }
    }

    double max_err = opt.max_valid_triangulation_error;
    for (int col = 0; col < point_image.cols(); col++) {
      for (int row = 0; row < point_image.rows(); row++) {
        
        Vector<double, 4> P = point_image(col, row);
        if (P == Vector<double, 4>() || texture(col, row) == texture_nodata ||
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
          bestFitPlane(near_points, centroid, plane_normal);
          
          Vector3 N(plane_normal[0], plane_normal[1], plane_normal[2]);
          // Use abs as the normal can point in either direction
          double prod = std::abs(dot_prod(Q, N)/ norm_2(Q) / norm_2(N));
          
          double angle = acos(prod) * (180.0 / M_PI);
          if (std::isnan(angle) || std::isinf(angle))
            continue; // something went wrong
          
          // std::cout << "--angle is " << angle << std::endl;

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

        clean_points(col, row) = Vector3(Q[0], Q[1], Q[2]);
        out_texture(col, row) = t;
        weight(col, row) = wt;
      }
    }

    if (opt.blending_dist > 0 && opt.blending_power > 0) {
      
      ImageView<double> dist;

//       // temporary!
//       for (int col = 0; col < clean_points.cols(); col++) {
//         for (int row = 0; row < clean_points.rows(); row++) {
//           clean_points(col, row) = Vector3(1, 1, 1);
//         }
//       }
      
      boundary_dist(clean_points, opt.blending_dist, dist);
      
#if 1
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

      // TODO(oalexan1): There must be an option to save the weight for inspection
      //exit(0);
    }

    asp::writeCloud(clean_points, out_texture, weight, pcd_file);
    
    return 0;
    
  } ASP_STANDARD_CATCHES;

  return 0;
}
