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

#ifndef __CORE_EIGEN_UTILS_H__
#define __CORE_EIGEN_UTILS_H__
#include <vw/FileIO/DiskImageView.h>
#include <vw/Cartography/Datum.h>
#include <vw/Cartography/GeoReference.h>

#include <Eigen/Dense>

#include <fstream>
#include <iostream>
#include <sstream>
#include <string>

// A set of routines kept here because they use Eigen, and a set of routine
// auxiliary to the routines using Eigen. 
namespace asp {

class CsvConv; // Forward declaration
  
typedef Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> DoubleMatrix;

// Note: Just changing 3 to 2 below won't be enough to make the code
// work with 2D point clouds. There are some Vector3's all over the place.
const int DIM = 3;

// Return at most m random points out of the input point cloud.
void random_pc_subsample(std::int64_t m, DoubleMatrix& points);
  
// Load a csv file, perhaps sub-sampling it along the way
void load_csv(std::string const& file_name,
              std::int64_t num_points_to_load,
              vw::BBox2 const& lonlat_box,
              bool calc_shift,
              vw::Vector3 & shift,
              vw::cartography::GeoReference const& geo,
              asp::CsvConv const& csv_conv,
              bool & is_lola_rdr_format,
              double & median_longitude, bool verbose,
              DoubleMatrix & data);
  
// Load a DEM, perhaps subsampling it along the way
void load_dem(std::string const& file_name,
              std::int64_t num_points_to_load, vw::BBox2 const& lonlat_box,
              bool calc_shift, vw::Vector3 & shift, bool verbose, 
              DoubleMatrix & data);

// Load an ASP point cloud, perhaps subsampling it along the way  
void load_pc(std::string const& file_name,
             std::int64_t num_points_to_load,
             vw::BBox2 const& lonlat_box,
             bool calc_shift,
             vw::Vector3 & shift,
             vw::cartography::GeoReference const& geo,
             bool verbose, DoubleMatrix & data);

// Find the best-fitting plane to a set of points. It will throw an
// error if called with less than 3 points.
void bestFitPlane(const std::vector<Eigen::Vector3d>& points, Eigen::Vector3d& centroid,
                  Eigen::Vector3d& plane_normal);

  // Compute a rigid transform between n point correspondences.
// There exists another version of this using vw matrices
// in VisionWorkbench called find_3D_transform().  
void computeRigidTransform(const std::vector<Eigen::Vector3d>& src,
                           const std::vector<Eigen::Vector3d>& dst,
                           Eigen::Matrix3d & rot, Eigen::Vector3d & trans);

/// Read a 4x4 rotation + translation + scale transform from disk.
void read_transform(Eigen::MatrixXd & T, std::string const& transFile);

/// Write a 4x4 rotation + translation + scale transform to disk.
void write_transform(Eigen::MatrixXd const& T, std::string const& transFile);

// Apply a rotation + translation transform to a Vector3
vw::Vector3 apply_transform_to_vec(Eigen::MatrixXd const& T, vw::Vector3 const& p);

} //end namespace asp

#endif//__CORE_EIGEN_UTILS_H__
