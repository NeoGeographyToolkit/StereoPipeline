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

#include <asp/Core/ImageUtils.h>
#include <vw/FileIO/DiskImageResourceGDAL.h>
#include <vw/Cartography/GeoReference.h>
#include <vw/FileIO/DiskImageUtils.h>
#include <vw/Image/Interpolation.h>

using namespace vw;

namespace asp {

/// Load an input image, georef, and nodata value
void load_image(std::string const& image_file,
                vw::ImageViewRef<double> & image, double & nodata,
                bool & has_georef, vw::cartography::GeoReference & georef) {
  
  // Ensure the output variables are initialized
  nodata = -std::numeric_limits<double>::max();
  has_georef = false;

  image = vw::load_image_as_double(image_file);

  // Read nodata-value from disk
  DiskImageResourceGDAL in_rsrc(image_file);
  bool has_nodata = in_rsrc.has_nodata_read();
  if (has_nodata) {
    nodata = in_rsrc.nodata_read();
    //vw_out() << "Read no-data value for image " << image_file << ": " << nodata << ".\n";
  } else {
    nodata = vw::get_default_nodata(in_rsrc.channel_type());
  }
  
  has_georef = vw::cartography::read_georeference(georef, image_file);
}

/// Create a DEM ready to use for interpolation
void create_interp_dem(std::string const& dem_file,
                       vw::cartography::GeoReference & dem_georef,
                       ImageViewRef<PixelMask<double>> & interp_dem) {
  
  vw_out() << "Loading DEM: " << dem_file << std::endl;

  // Read the no-data
  double nodata_val = -std::numeric_limits<float>::max(); // note we use a float nodata
  if (vw::read_nodata_val(dem_file, nodata_val))
    vw_out() << "Found DEM nodata value: " << nodata_val << std::endl;

  // Create the interpolated DEM. Values out of bounds will be invalid.
  vw::PixelMask<double> invalid_val;
  invalid_val[0] = nodata_val;
  invalid_val.invalidate();
  ImageViewRef<PixelMask<double>> dem
    = create_mask(DiskImageView<double>(dem_file), nodata_val);
  interp_dem = interpolate(dem, BilinearInterpolation(), 
                           vw::ValueEdgeExtension<vw::PixelMask<float>>(invalid_val));

  // Read the georef. It must exist.
  bool is_good = vw::cartography::read_georeference(dem_georef, dem_file);
  if (!is_good) {
    vw_throw(ArgumentErr() << "Error: Cannot read a georeference from DEM: "
             << dem_file << ".\n");
  }
}

/// Take an interest point from a map projected image and convert it
/// to the corresponding IP in the original non-map-projected image.
/// - Return false if the pixel could not be converted.
bool projected_ip_to_raw_ip(vw::ip::InterestPoint &P,
                            vw::ImageViewRef<vw::PixelMask<double>> const& interp_dem,
                            vw::CamPtr camera_model,
                            vw::cartography::GeoReference const& georef,
                            vw::cartography::GeoReference const& dem_georef) {

  // Get IP coordinate in the DEM
  Vector2 pix(P.x, P.y);
  Vector2 ll      = georef.pixel_to_lonlat(pix);
  Vector2 dem_pix = dem_georef.lonlat_to_pixel(ll);
  if (!interp_dem.pixel_in_bounds(dem_pix))
    return false;
  // Load the elevation from the DEM
  PixelMask<double> dem_val = interp_dem(dem_pix[0], dem_pix[1]);
  if (!is_valid(dem_val))
    return false;
  Vector3 llh(ll[0], ll[1], dem_val.child());
  Vector3 xyz = dem_georef.datum().geodetic_to_cartesian(llh);

  // Project into the camera
  Vector2 cam_pix;
  try {
   cam_pix = camera_model->point_to_pixel(xyz);
  } catch(...) {
    return false; // Don't update the point.
  }
  P.x  = cam_pix.x();
  P.y  = cam_pix.y();
  P.ix = P.x;
  P.iy = P.y;
  return true;
}

// Read keywords that describe how the images were map-projected.
void read_mapproj_header(std::string const& map_file,
                         // Outputs
                         std::string & adj_key, std::string & img_file_key,
                         std::string & cam_type_key, std::string & cam_file_key, 
                         std::string & dem_file_key,
                         std::string & adj_prefix,
                         std::string & image_file, std::string & cam_type,
                         std::string & cam_file, std::string & dem_file) {

  boost::shared_ptr<vw::DiskImageResource> rsrc(new vw::DiskImageResourceGDAL(map_file));
  adj_key      = "BUNDLE_ADJUST_PREFIX"; 
  img_file_key = "INPUT_IMAGE_FILE";
  cam_type_key = "CAMERA_MODEL_TYPE",
  cam_file_key = "CAMERA_FILE";
  dem_file_key = "DEM_FILE"; 

  vw::cartography::read_header_string(*rsrc.get(), adj_key,      adj_prefix);
  vw::cartography::read_header_string(*rsrc.get(), img_file_key, image_file);
  vw::cartography::read_header_string(*rsrc.get(), cam_type_key, cam_type);
  vw::cartography::read_header_string(*rsrc.get(), cam_file_key, cam_file);
  vw::cartography::read_header_string(*rsrc.get(), dem_file_key, dem_file);
  
  // This is important. When writing, have to write something, so use NONE,
  // but on reading, if the string is NONE, make it empty.
  if (adj_prefix == "NONE") 
    adj_prefix = "";
} 

} // end namespace asp
