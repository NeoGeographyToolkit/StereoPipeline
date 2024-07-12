// __BEGIN_LICENSE__
//  Copyright (c) 2006-2013, United States Government as represented by the
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


#include <string>
#include <vector>

#include <asp/Core/ImageUtils.h>
#include <asp/Core/GCP.h>

#include <vw/FileIO/DiskImageView.h>
#include <vw/Image/Interpolation.h>
#include <vw/Cartography/GeoReference.h>
#include <vw/FileIO/FileUtils.h>

namespace asp {

// Write a GCP file. Can throw exceptions.
// TODO(oalexan1): Factor out the GCP-writing logic. Move it to WV,
// together with the logic for reading GCPs. Support reading
// and writing a georef, not a datum.

// TODO(oalexan1): Add the GUI option --gcp-srs. The default should be WGS84
// with stereographic projection at lon=0, lat=0 unless there exists a DEM,
// when the default should be the long-lat projection for that DEM. --gcp-srs
// can be used with rig_calibrator for local Cartesian coordinates.
void writeGCP(std::vector<std::string> const& image_files,
              std::string const& gcp_file,
              std::string const& dem_file,
              asp::MatchList const& matchlist) {
  
  using namespace vw;
  
  // Must have at lest two images to write a GCP file
  if (image_files.size() < 2)
    vw_throw(ArgumentErr() << "At least two images are needed to write a GCP file.\n");
  
  // Load a georeference to use for the GCPs from the last image
  vw::cartography::GeoReference image_georef;
  const size_t georef_index = image_files.size() - 1;
  const std::string image_georef_file = image_files[georef_index];
  bool has_georef = vw::cartography::read_georeference(image_georef, image_georef_file);
  if (!has_georef)
    vw::vw_throw(vw::ArgumentErr() << "Could not load a valid georeference to use for "
                 << "ground control points in file: " << image_georef_file << ".\n");

  // Init the DEM to use for height interpolation
  vw::ImageViewRef<vw::PixelMask<double>> interp_dem;
  vw::cartography::GeoReference dem_georef;
  asp::create_interp_dem(dem_file, dem_georef, interp_dem);
  BBox2 dem_bbox = bounding_box(interp_dem);

  // A basic sanity check. The orthoimage and DEM datums should be similar.
  double tol = 1.0; // 1 m
  auto const& img_d = image_georef.datum();
  auto const& dem_d = dem_georef.datum();
  // Throw if the datums are too different
  if (img_d.semi_major_axis() - dem_d.semi_major_axis() > tol ||
      img_d.semi_minor_axis() - dem_d.semi_minor_axis() > tol)
    vw::vw_throw(ArgumentErr() << "The orthoimage and DEM datums differ by more than "
             << tol << " meters. This is not supported.\n");
    
  vw_out() << "Writing: " << gcp_file << "\n";
  vw::create_out_dir(gcp_file);
  std::ofstream output_handle(gcp_file.c_str());
  output_handle << std::setprecision(17);

  // It is important to keep track of the datum and projection, because the
  // elevations are relative to it
  // TODO(oalexan1): Put below dem_georef.get_wkt() instead.
  // TODO(oalexan1): Have a single GCP-writing function, and put it in VW.
  // See also the existing one called write_in_gcp_format() in VW.
  output_handle << "# WKT: " << dem_georef.datum().get_wkt() << std::endl;
  // TODO(oalexan1): Write here if the format is lon,lat,height, or easting, northing, height.
  size_t num_pts_skipped = 0, num_pts_used = 0;

  const size_t num_ips = matchlist.getNumPoints();
  for (size_t p = 0; p < num_ips; p++) { // Loop through IPs
    
    // Compute the GDC coordinate of the point
    ip::InterestPoint ip = matchlist.getPoint(georef_index, p);
    Vector2 lonlat    = image_georef.pixel_to_lonlat(Vector2(ip.x, ip.y));
    Vector2 dem_pixel = dem_georef.lonlat_to_pixel(lonlat);
    PixelMask<float> height = interp_dem(dem_pixel[0], dem_pixel[1])[0];
    
    // Bounding box check.
    if ((!dem_bbox.contains(dem_pixel)) || (!is_valid(height))) {
      vw_out() << "Warning: Skipped IP # " << p
               << " because it does not fall on the DEM.\n";
      num_pts_skipped++;
      continue; // Skip locations which do not fall on the DEM
    }
    
    // Write the per-point information
    output_handle << num_pts_used; // The ground control point ID
    bool write_ecef = false;
    // TODO(oalexan1): It can be convenient to export GCP in ECEF, for software
    // which does not know about projections. Could be an option.
    if (!write_ecef) {
      // Write lat, lon, height
      output_handle << ", " << lonlat[1] << ", " << lonlat[0] << ", " << height[0];
    } else {
      // Write x, y, z
      vw::Vector3 P(lonlat[0], lonlat[1], height[0]);
      P = dem_georef.datum().geodetic_to_cartesian(P);
      output_handle << ", " << P[0] << ' ' << P[1] << ' ' << P[2];
    }
    
    // Write sigma values on the same line
    output_handle << ", " << 1 << ", " << 1 << ", " << 1; 
    
    // Write the per-image information
    // The last image is the reference image, so we skip it when saving GCPs
    size_t num_images = image_files.size();
    size_t num_images_to_save = num_images - 1; 
    for (size_t i = 0; i < num_images_to_save; i++) {
      // Add this IP to the current line
      ip::InterestPoint ip = matchlist.getPoint(i, p);
      output_handle << ", " << image_files[i];
      output_handle << ", " << ip.x << ", " << ip.y; // IP location in image
      output_handle << ", " << 1 << ", " << 1; // Sigma values
    } // End loop through IP sets
    output_handle << std::endl; // Finish the line
    num_pts_used++;
  } // End loop through IPs
  
  output_handle.close();

  return;
}

} // namespace asp
