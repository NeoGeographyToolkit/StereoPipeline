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

#include <limits>

using namespace vw;
using namespace vw::cartography;
namespace po = boost::program_options;
namespace fs = boost::filesystem;

struct Options : vw::cartography::GdalWriteOptions {
  // Input
  std::vector<std::string> files;
  
  // Defaults that the user doesn't need to see.
  Options() {}
};

void handle_arguments( int argc, char *argv[], Options& opt ) {


  po::options_description general_options("General options");
  //general_options.add_options()

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

    // Read the point cloud fully in memory, and remove any offset
    std::string cloud_file = opt.files[0];
    ImageViewRef<Vector3> point_image = asp::read_asp_point_cloud<3>(cloud_file);
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

    int count = -1;
    for (int col = 0; col < point_image.cols(); col++) {
      for (int row = 0; row < point_image.rows(); row++) {
        count++;

        Vector3 P = point_image(col, row);
        
        if (P == Vector3() || texture(col, row) == texture_nodata) {
          // This is needed for VoxBlox.
          pci.points[count].x = inf;
          pci.points[count].y = inf;
          pci.points[count].z = inf;
          pci.points[count].normal_x  = 0;  // intensity
          pci.points[count].normal_y  = 0;  // weight
          pci.points[count].normal_z  = 0;  // ensure initialization
          pci.points[count].curvature = 0;  // ensure initialization
          continue;
        }

        // Create homogeneous coordinates
        Vector<double, 4> Q;
        subvector(Q, 0, 3) = P;
        Q[3] = 1; 
        
        // Convert to camera's coordinate system, as expected by voxblox
        Q = world2cam * Q;

        pci.points[count].x = Q[0];
        pci.points[count].y = Q[1];
        pci.points[count].z = Q[2];

        // The input texture is usually between 0 and 1.
        // TODO(oalexan1): What is the max color?
        double t = 255.0 * texture(col, row);
        if (t <= 0.0) t = 1.0;  // Ensure a positive value for the color

        // All points are given equal weight for now
        double wt = 1.0;
        
        pci.points[count].normal_x = t;   // intensity
        pci.points[count].normal_y = wt;  // weight
        pci.points[count].normal_z  = 0;  // ensure initialization to something
        pci.points[count].curvature = 0;  
      }
    }
    
    std::cout << "Writing: " << pcd_file << std::endl;
    pcl::io::savePCDFileASCII(pcd_file, pci);
    
    return 0;
    
  } ASP_STANDARD_CATCHES;

  return 0;
}
