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

#include <pcl/io/pcd_io.h>

namespace asp {
  
  void writeCloud(vw::ImageViewRef<vw::Vector<double, 4>> cloud,
                  vw::ImageViewRef<float> out_texture,
                  vw::ImageViewRef<float> weight,
                  bool save_nodata_as_infinity,
                  std::string const& pcd_file) {
      
    double nodata = 0.0;
    if (save_nodata_as_infinity)
      nodata =  std::numeric_limits<double>::infinity();

    pcl::PointCloud<pcl::PointNormal> pci;

    pci.width = cloud.cols();
    pci.height = cloud.rows();
    pci.points.resize(pci.width * pci.height);

    int count = -1;
    for (int col = 0; col < cloud.cols(); col++) {
      for (int row = 0; row < cloud.rows(); row++) {
        count++;

        // An output point starts by default as an outlier
        // VoxBlox expects infinity for invalid data.
        pci.points[count].x         = nodata;
        pci.points[count].y         = nodata;
        pci.points[count].z         = nodata;
        pci.points[count].normal_x  = 0;  // intensity
        pci.points[count].normal_y  = 0;  // weight
        pci.points[count].normal_z  = 0;  // ensure initialization
        pci.points[count].curvature = 0;  // ensure initialization

        if (subvector(cloud(col, row), 0, 3) != vw::Vector3() && weight(col, row) > 0) {
          vw::Vector<double, 4> Q = cloud(col, row);
          pci.points[count].x         = Q[0];
          pci.points[count].y         = Q[1];
          pci.points[count].z         = Q[2];
          // As expected by VoxBlox
          pci.points[count].normal_x  = out_texture(col, row);  // intensity
          pci.points[count].normal_y  = weight(col, row); // weight
          pci.points[count].normal_z  = Q[3]; // intersection error
          pci.points[count].curvature = 0;  
        }
      }
    }
    
    // Create the output directory
    vw::create_out_dir(pcd_file);

    // Save the pcd
    std::cout << "Writing: " << pcd_file << std::endl;
    pcl::io::savePCDFileASCII(pcd_file, pci);

    return;
  }

}
