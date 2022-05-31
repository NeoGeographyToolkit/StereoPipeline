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
#include <asp/Core/PointUtils.h>
#include <asp/Core/Macros.h>
#include <asp/Core/Common.h>
#include <asp/Core/StereoSettings.h>

#include <vw/Core/Stopwatch.h>
#include <vw/Core/StringUtils.h>
#include <vw/Cartography/GeoReferenceUtils.h>
#include <vw/Mosaic/ImageComposite.h>
#include <vw/FileIO/DiskImageUtils.h>
#include <vw/InterestPoint/MatrixIO.h>

#include <pcl/io/pcd_io.h>
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
    max_camera_ray_to_surface_normal_angle, max_camera_dir_to_camera_ray_angle;

  // Defaults that the user doesn't need to see.
  Options() {}
};

// TODO(oalexan1): Move this to utils and do not include any Eigen header.
// Eigen, PCL, and VW combined make compilation impossibly long.
// Find the best fitting plane to a set of points.
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
     "If positive, and a ray emanating from the camera and ending at the current point makes an angle with the camera direction bigger than this, remove the point as an outlier. In effect, this narrows the camera field of view.");

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

    std::cout << "--max valid tri error " << opt.max_valid_triangulation_error << std::endl;
    
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
    
    double inf = std::numeric_limits<double>::infinity();
    pcl::PointCloud<pcl::PointNormal> pci;
    pci.width = point_image.cols();
    pci.height = point_image.rows();
    pci.points.resize(pci.width * pci.height);

    double max_err = opt.max_valid_triangulation_error;
    int count = -1;
    for (int col = 0; col < point_image.cols(); col++) {
      for (int row = 0; row < point_image.rows(); row++) {
        count++;

        // An output point starts by default as an outlier
        // Make it inf, as VoxBlox expects.
        pci.points[count].x         = inf;
        pci.points[count].y         = inf;
        pci.points[count].z         = inf;
        pci.points[count].normal_x  = 0;  // intensity
        pci.points[count].normal_y  = 0;  // weight
        pci.points[count].normal_z  = 0;  // ensure initialization
        pci.points[count].curvature = 0;  // ensure initialization
        
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

        pci.points[count].x         = Q[0];
        pci.points[count].y         = Q[1];
        pci.points[count].z         = Q[2];
        pci.points[count].normal_x  = t;   // intensity
        pci.points[count].normal_y  = wt;  // weight
        pci.points[count].normal_z  = 0;  // ensure initialization to something
        pci.points[count].curvature = 0;  
      }
    }
    
    // Create the output directory
    
    vw::create_out_dir(pcd_file);
    std::cout << "Writing: " << pcd_file << std::endl;
    pcl::io::savePCDFileASCII(pcd_file, pci);
    
    return 0;
    
  } ASP_STANDARD_CATCHES;

  return 0;
}
