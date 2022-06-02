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

// Interface with PCL.

#include <vw/Image/ImageViewRef.h>
#include <vw/FileIO/FileUtils.h>
#include <asp/PclIO/PclIO.h>

#include <boost/filesystem.hpp>
#include <boost/algorithm/string.hpp>

#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>

namespace asp {
  
void writeCloud(vw::ImageViewRef<vw::Vector<double, 4>> cloud,
                vw::ImageViewRef<float> out_texture,
                vw::ImageViewRef<float> weight,
                bool save_nodata_as_infinity,
                std::string const& cloud_file) {
      
  double nodata = 0.0;
  if (save_nodata_as_infinity)
    nodata =  std::numeric_limits<double>::infinity();

  std::string ext = boost::filesystem::extension(cloud_file);
  boost::algorithm::to_lower(ext);
  if (ext != ".pcd" && ext != ".ply") 
    vw::vw_throw(vw::ArgumentErr() << "The input point cloud extension must be .pcd or .ply.");

  // Create the output directory
  vw::create_out_dir(cloud_file);

  // Save the cloud
  std::cout << "Writing: " << cloud_file << std::endl;

  bool write_ply = (ext == ".ply");
  if (!write_ply) {
      
    // Write pcd 
    pcl::PointCloud<pcl::PointNormal> pc;
      
    pc.width = cloud.cols();
    pc.height = cloud.rows();
    pc.points.resize(cloud.cols() * cloud.rows());
      
    int count = -1;
    for (int col = 0; col < cloud.cols(); col++) {
      for (int row = 0; row < cloud.rows(); row++) {
        count++;
          
        // An output point starts by default as an outlier
        // VoxBlox expects infinity for invalid data.
        pc.points[count].x         = nodata;
        pc.points[count].y         = nodata;
        pc.points[count].z         = nodata;
        pc.points[count].normal_x  = 0;  // intensity
        pc.points[count].normal_y  = 0;  // weight
        pc.points[count].normal_z  = 0;  // ensure initialization
        pc.points[count].curvature = 0;  // ensure initialization
          
        if (subvector(cloud(col, row), 0, 3) != vw::Vector3() && weight(col, row) > 0) {
          vw::Vector<double, 4> const& Q = cloud(col, row); // alias
          pc.points[count].x         = Q[0];
          pc.points[count].y         = Q[1];
          pc.points[count].z         = Q[2];
          // As expected by VoxBlox
          pc.points[count].normal_x  = out_texture(col, row);  // intensity
          pc.points[count].normal_y  = weight(col, row); // weight
          pc.points[count].normal_z  = Q[3]; // intersection error
          pc.points[count].curvature = 0;  // ensure initialization
        }
      }
    }

    pcl::io::savePCDFileASCII(cloud_file, pc);

  } else {
        
    // Write ply; remove all the invalid data first
    pcl::PointCloud<pcl::PointXYZI> pc;
    pc.width = cloud.cols();
    pc.height = cloud.rows();
    pc.points.resize(cloud.cols() * cloud.rows());
      
    int count = -1;
    for (int col = 0; col < cloud.cols(); col++) {
      for (int row = 0; row < cloud.rows(); row++) {
        count++;
          
        if (subvector(cloud(col, row), 0, 3) != vw::Vector3() && weight(col, row) > 0) {
          vw::Vector<double, 4> const& Q = cloud(col, row); // alias
          pc.points[count].x         = Q[0];
          pc.points[count].y         = Q[1];
          pc.points[count].z         = Q[2];
          pc.points[count].intensity = out_texture(col, row);
        }
      }
    }
      
    pc.points.resize(count + 1);
    pc.width = count + 1;
    pc.height = 1;
    pcl::io::savePLYFileASCII(cloud_file, pc);
  }
  
  return;
}

} // end namespace asp
