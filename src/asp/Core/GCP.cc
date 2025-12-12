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

#include <asp/Core/ImageUtils.h>
#include <asp/Core/MatchList.h>
#include <asp/Core/GCP.h>

#include <vw/FileIO/DiskImageView.h>
#include <vw/Image/Interpolation.h>
#include <vw/Cartography/GeoReference.h>
#include <vw/FileIO/FileUtils.h>

namespace asp {

// Write GCP to a file
void writeGcp(std::string const& gcpFile,
              vw::cartography::GeoReference const& geo,
              std::vector<Gcp> const& gcp_vec,
              std::vector<std::string> const& image_files) {

  vw::vw_out() << "Writing: " << gcpFile << "\n";
  std::ofstream ofs(gcpFile.c_str());
  ofs.precision(17); // full precision

  // It is important to keep track of the datum and projection, because the
  // elevations are relative to it
  ofs << "# WKT: " << geo.get_wkt() << "\n";
  ofs << "# id lat lon height_above_datum sigma_x sigma_y sigma_z image_name "
      << "pixel_x pixel_y sigma_x sigma_y, etc.\n";

  // Iterate over the gcp_vec
  for (size_t gcp_id = 0; gcp_id < gcp_vec.size(); gcp_id++) {

    auto const& gcp   = gcp_vec[gcp_id]; // alias
    auto const& cp    = gcp.cp; // alias
    auto const& llh   = gcp.llh; // alias
    auto const& sigma = gcp.sigma; // alias
    vw::Vector2 pix_sigma(1, 1);

    // Write the id, lat, lon, height, sigmas
    ofs << gcp_id << " " << llh.y() << " " << llh.x() << " " << llh.z() << " "
        << sigma[0] << " " << sigma[1] << " " << sigma[2] << " ";
        
    for (int im = 0; im < cp.size(); im++) {
      auto const& cm = cp[im]; // measure
      ofs << image_files[cm.image_id()] << " " 
          << cm.position()[0] << " " << cm.position()[1] << " "
          << pix_sigma[0] << " " << pix_sigma[1];
      
      // Have a space between measures, except after the last one
      if (im < int(cp.size()) - 1)
        ofs << " ";
    }
    ofs << "\n";
  }
  
  ofs.close();
}

// Produce and write a GCP file. Can throw exceptions. TODO(oalexan1): Consider
// moving this to WV, together with the logic for reading GCPs. Support reading
// and writing a georef, not a datum. TODO(oalexan1): Add the GUI option
// --gcp-srs. The default should be WGS84 with stereographic projection at
// lon=0, lat=0 unless there exists a DEM, when the default should be the
// long-lat projection for that DEM. --gcp-srs can be used with rig_calibrator
// for local Cartesian coordinates.
void genWriteGcp(std::vector<std::string> const& image_files,
                 std::string const& gcp_file,
                 std::string const& dem_file,
                 asp::MatchList const& matchlist,
                 double xyz_sigma) {
  
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
    
  // Populate the GCPs
  std::vector<asp::Gcp> gcp_vec;
  size_t num_pts_skipped = 0;
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

    asp::Gcp gcp;
    gcp.llh = vw::Vector3(lonlat[0], lonlat[1], height[0]);
    gcp.sigma = vw::Vector3(xyz_sigma, xyz_sigma, xyz_sigma);

    // The last image is the reference image, so we skip it when saving GCPs
    size_t num_images = image_files.size();
    size_t num_images_to_save = num_images - 1; 
    for (size_t i = 0; i < num_images_to_save; i++) {
      // Add this IP to the current line
      ip::InterestPoint ip = matchlist.getPoint(i, p);
      vw::ba::ControlMeasure cm;
      cm.set_image_id(i);
      cm.set_position(vw::Vector2(ip.x, ip.y));
      cm.set_sigma(vw::Vector2(1, 1));
      gcp.cp.add_measure(cm);
    }
    gcp_vec.push_back(gcp);
  }
  
  // Write the GCPs to file
  asp::writeGcp(gcp_file, image_georef, gcp_vec, image_files);

  return;
}

} // namespace asp
