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

#include <asp/Core/DemUtils.h>
#include <vw/FileIO/DiskImageView.h>
#include <vw/Cartography/CameraBBox.h>

namespace asp {

// Given an image pixel, trace a ray to the ground and find the intersection.
void queryPixel(std::string const& dem_file, vw::CamPtr camera_model,
                vw::Vector2 const& query_pixel) {

  // The DEM file must be provided
  if (dem_file.empty())
    vw::vw_throw(vw::ArgumentErr()
                 << "The DEM file must be provided to be able to query a pixel.\n");
    
  // Read the georef. It must exist.
  vw::cartography::GeoReference dem_georef;
  bool has_georef = vw::cartography::read_georeference(dem_georef, dem_file);
  if (!has_georef)
    vw::vw_throw(vw::ArgumentErr() << "The DEM file must have a georeference.\n");
  
  // Read the no-data
  double nodata_val = -std::numeric_limits<float>::max(); // note we use a float nodata
  vw::read_nodata_val(dem_file, nodata_val); // ignore the success status

  // Create the masked DEM
  vw::ImageViewRef<vw::PixelMask<float>> masked_dem
    = vw::create_mask(vw::DiskImageView<float>(dem_file), nodata_val);
  
  vw::Vector3 cam_ctr = camera_model->camera_center(query_pixel);
  vw::Vector3 cam_dir = camera_model->pixel_to_vector(query_pixel);
  double height_guess = vw::cartography::demHeightGuess(masked_dem);

  // Intersect the ray going from the given camera pixel with a DEM
  // Use xyz_guess as initial guess and overwrite it with the new value
  bool treat_nodata_as_zero = false;
  bool has_intersection = false;
  double dem_height_error_tol = 1e-3; // 1 mm
  double max_abs_tol = std::min(dem_height_error_tol, 1e-14);
  double max_rel_tol = max_abs_tol;
  int num_max_iter = 100;
  vw::Vector3 xyz_guess(0, 0, 0);
  vw::Vector3 xyz = vw::cartography::camera_pixel_to_dem_xyz
    (cam_ctr, cam_dir, masked_dem,
      dem_georef, treat_nodata_as_zero,
      has_intersection, dem_height_error_tol, 
      max_abs_tol, max_rel_tol, 
      num_max_iter, xyz_guess, height_guess);
  
  double nan = std::numeric_limits<double>::quiet_NaN();
  vw::Vector3 llh(nan, nan, nan);
  vw::Vector2 dem_pix(nan, nan);
  if (has_intersection) {
    llh = dem_georef.datum().cartesian_to_geodetic(xyz);
    dem_pix = dem_georef.lonlat_to_pixel(vw::Vector2(llh[0], llh[1]));
  }
  vw::vw_out() << std::setprecision(17)
                << "DEM intersection point (lon, lat, height): " 
                << llh[0] << ' ' << llh[1] << ' ' << llh[2] << "\n"
                << "DEM intersection pixel (column, row, height): " 
                << dem_pix[0] << ' ' << dem_pix[1] << ' ' << llh[2] << "\n";

}

} //end namespace asp
