// __BEGIN_LICENSE__
//  Copyright (c) 2009-2026, United States Government as represented by the
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

/// \file BathyPlaneCalc.h

#ifndef __ASP_CORE_BATHYPLANECALC_H__
#define __ASP_CORE_BATHYPLANECALC_H__

#include <vw/Math/Vector.h>
#include <vw/Math/Matrix.h>
#include <vw/Image/ImageViewRef.h>
#include <vw/Image/PixelMask.h>
#include <vw/Cartography/GeoReference.h>
#include <vw/Geometry/dPoly.h>

#include <Eigen/Dense>

#include <vector>
#include <string>

namespace vw {
  namespace camera {
    class CameraModel;
  }
}

namespace asp {

// Given a set of polygons stored in dPoly, create a single polygon with all those vertices
void formSinglePoly(vw::geometry::dPoly const& inPoly,
                    vw::geometry::dPoly & outPoly);

// Add a polygon made up of just one point to given set of polygons.
// Later that set will be saved as a shapefile made up of points.
void addPointToPoly(vw::geometry::dPoly & poly, vw::Vector2 const& p);

// Add a point to the point_vec, llh_vec, and used_vertices if it is valid
void addPoint(vw::cartography::GeoReference const& dem_georef,
              vw::ImageViewRef<vw::PixelMask<float>> const& interp_dem,
              vw::Vector2 const& lonlat,
              vw::Vector2 const& proj_pt,
              // Append
              std::vector<Eigen::Vector3d> & point_vec,
              std::vector<vw::Vector3> & llh_vec,
              std::vector<vw::Vector2> & used_vertices);

// Estimate the projection and convert llh_vec to projected coordinates
void find_projection(// Inputs
                     vw::cartography::GeoReference const& dem_georef,
                     std::vector<vw::Vector3> const& llh_vec,
                     // Outputs
                     double & proj_lat, double & proj_lon,
                     vw::cartography::GeoReference & stereographic_georef,
                     std::vector<Eigen::Vector3d> & proj_vec);

// Sample the mask boundary (points where the points in the mask have neighbors
// not in the mask), shoot points from there onto the DEM, and return the
// obtained points.
void sampleMaskBd(vw::ImageViewRef<float> mask,
                  float mask_nodata_val,
                  boost::shared_ptr<vw::camera::CameraModel> camera_model,
                  vw::cartography::GeoReference const& shape_georef,
                  vw::cartography::GeoReference const& dem_georef,
                  vw::ImageViewRef<vw::PixelMask<float>> masked_dem,
                  int num_samples,
                  std::vector<Eigen::Vector3d> & ecef_vec,
                  std::vector<vw::Vector3> & llh_vec,
                  std::vector<vw::Vector2> & used_vertices);

// Find the mask boundary (points where the points in the mask have
// neighbors not in the mask), and look up the height in the DEM.
void sampleOrthoMaskBd(std::string const& mask_file,
                       vw::cartography::GeoReference const& mask_georef,
                       vw::cartography::GeoReference const& dem_georef,
                       vw::ImageViewRef<vw::PixelMask<float>> interp_dem,
                       int num_samples,
                       std::vector<Eigen::Vector3d> & point_vec,
                       std::vector<vw::Vector3> & llh_vec,
                       std::vector<vw::Vector2> & used_vertices);

// Compute the 3D locations at the shape corners based on interpolating
// into the DEM and converting either to ECEF or to local projected
// stereographic coordinates.
// If using a curved water surface, compute the stereographic georeference
// with the projection center being the mean lon and lat, and make the 3D locations
// in reference to this projection
void find_points_at_shape_corners(std::vector<vw::geometry::dPoly> const& polyVec,
                                  vw::cartography::GeoReference const& shape_georef,
                                  vw::cartography::GeoReference const& dem_georef,
                                  vw::ImageViewRef<vw::PixelMask<float>> interp_dem,
                                  std::vector<Eigen::Vector3d> & point_vec,
                                  std::vector<vw::Vector3> & llh_vec,
                                  std::vector<vw::Vector2> & used_vertices);

// Read a set of measurements in CSV format, to use later to fit the water surface
void find_points_from_meas_csv(std::string const& water_height_measurements,
                               std::string const& csv_format_str,
                               vw::cartography::GeoReference const& shape_georef,
                               // Outputs
                               std::vector<vw::Vector3> & llh_vec,
                               std::vector<vw::Vector2> & used_vertices);

// Read a set of lon-lat measurements in CSV format, then interpolate into the DEM
// with bilinear interpolation to find the height.
void find_points_from_lon_lat_csv(std::string const& lon_lat_measurements,
                                  std::string const& csv_format_str,
                                  vw::cartography::GeoReference const& shape_georef,
                                  vw::cartography::GeoReference const& dem_georef,
                                  vw::ImageViewRef<vw::PixelMask<float>> interp_dem,
                                  // Outputs
                                  std::vector<Eigen::Vector3d> & point_vec,
                                  std::vector<vw::Vector3> & llh_vec,
                                  std::vector<vw::Vector2> & used_vertices);

// Save the points as a shapefile
void saveShape(std::vector<Eigen::Vector3d> const& point_vec,
               std::string const& mask_boundary_shapefile);

// Calculate a few properties of the plane fitted to the given points and print them out
void calcPlaneProperties(bool use_proj_water_surface,
                         std::vector<Eigen::Vector3d> const& point_vec,
                         std::vector<size_t> const& inlier_indices,
                         vw::cartography::GeoReference & dem_georef,
                         vw::Matrix<double> const& plane);

// Save the bathy plane and the projection parameters if needed
void saveBathyPlane(bool use_proj_water_surface, double proj_lat, double proj_lon,
                    vw::Matrix<double> const& plane, std::string const& plane_file);

vw::ImageViewRef<float> demMinusPlane(vw::ImageViewRef<float> const& dem,
                                      vw::cartography::GeoReference const& dem_georef,
                                      vw::Matrix<double> plane,
                                      double dem_nodata_val,
                                      bool use_proj_water_surface,
                                      vw::cartography::GeoReference const& stereographic_georef);

// Use RANSAC to find the best plane
void calcBathyPlane(bool use_proj_water_surface,
                    int num_ransac_iterations,
                    double inlier_threshold,
                    std::vector<Eigen::Vector3d> const& point_vec,
                    vw::Matrix<double> & plane,
                    std::vector<size_t> & inlier_indices);

} // end namespace asp

#endif // __ASP_CORE_BATHYPLANECALC_H__