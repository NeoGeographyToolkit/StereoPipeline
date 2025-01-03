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
#include <vw/FileIO/GdalWriteOptions.h>
#include <vw/Cartography/CameraBBox.h>
#include <vw/Cartography/GeoReferenceUtils.h>

namespace fs = boost::filesystem;

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

// Prepare a DEM file that encompasses a given image and with a given height.
void setupDem(vw::ImageViewRef<float> img,
              vw::cartography::GeoReference const& image_georef,
              std::string const& tag, 
              std::string const& out_prefix, 
              double dem_height,
              // Outputs
              std::string & dem_path,
              double & dem_nodata,
              vw::cartography::GeoReference & dem_georef,
              vw::ImageView<float> & dem) {

  dem_path = out_prefix + "-" + tag + "-ortho-dem.tif";
  dem_nodata = -1e+6;
  
  // projection box
  vw::BBox2 img_bbox = vw::bounding_box(img);
  vw::BBox2 img_proj_box = image_georef.pixel_to_point_bbox(img_bbox);
  
  // Number of pixels and grid size of the DEM. The DEM need not be big. Its
  // has constant height anyway. 
  dem_georef = image_georef;
  double num = 100.0;
  double proj_width = img_proj_box.width();
  double proj_height = img_proj_box.height();
  double tr = std::max(proj_width/num, proj_height/num);
  
  // Set up the transform
  vw::Matrix3x3 T = dem_georef.transform();
  if (T(0,0) <= 0.0 || T(1, 1) >= 0.0) 
    vw::vw_throw(vw::ArgumentErr() 
                 << "Nonstandard orthoimage encountered. The "
                 << "pixel size must be positive in x and negative in y.");
  T(0, 0) = tr; 
  T(1, 1) = -tr;
  dem_georef.set_transform(T);

  // The DEM box will be a bit bigger to ensure it fully covers the image
  double s = 5.0;
  vw::BBox2 dem_proj_box = img_proj_box;
  dem_proj_box.min() = tr * (floor(dem_proj_box.min()/tr) - vw::Vector2(s, s));
  dem_proj_box.max() = tr * (ceil(dem_proj_box.max()/tr) + vw::Vector2(s, s));
  
  // This crop will make the upper-left DEM pixel agree with dem_proj_box, and
  // also ensure that the pixel interpretation (PixelAsArea or PixelAsPoint)
  // is respected.
  vw::Vector2 pix1 = dem_georef.point_to_pixel(dem_proj_box.min());
  vw::Vector2 pix2 = dem_georef.point_to_pixel(dem_proj_box.max());
  vw::BBox2 dem_pix_box;
  dem_pix_box.grow(pix1);
  dem_pix_box.grow(pix2);
  dem_georef = vw::cartography::crop(dem_georef, dem_pix_box);

  // Recompute the corner pixels after the crop
  pix1 = dem_georef.point_to_pixel(dem_proj_box.min());
  pix2 = dem_georef.point_to_pixel(dem_proj_box.max());
  dem_pix_box = vw::BBox2();
  dem_pix_box.grow(pix1);
  dem_pix_box.grow(pix2);
  
  // Create the DEM with given corner pixels and set all values to the given
  // height
  dem = vw::ImageView<float>(dem_pix_box.width(), dem_pix_box.height());
  for (int col = 0; col < dem.cols(); col++) {
    for (int row = 0; row < dem.rows(); row++) {
      dem(col, row) = dem_height;
    }
  }

  // Sanity check
  dem_proj_box = dem_georef.pixel_to_point_bbox(vw::bounding_box(dem));
  if (!dem_proj_box.contains(img_proj_box)) 
    vw::vw_throw(vw::ArgumentErr() << "The DEM box does not fully cover the image box.");
}

// Prepare a DEM file that encompasses a given image and with a given height,
// or reuse the one already available if agrees with what is intended.
void setupOrCheckDem(vw::GdalWriteOptions const& options,
                     vw::ImageViewRef<float> img,
                     vw::cartography::GeoReference const& image_georef,
                     std::string const& tag, 
                     std::string const& out_prefix, 
                     double dem_height,
                     // Outputs
                     std::string & dem_path) {

  vw::ImageView<float> dem;
  double dem_nodata = -std::numeric_limits<float>::max(); // will change
  vw::cartography::GeoReference dem_georef;

  // Set it up, but don't write to to disk yet                     
  setupDem(img, image_georef, tag, out_prefix, dem_height,
            dem_path, dem_nodata, dem_georef, dem);

  // See how it agrees with what is on disk
  bool good_already = false;
  if (fs::exists(dem_path)) {
    vw::DiskImageView<float> input_dem(dem_path);
    vw::cartography::GeoReference input_dem_georef;
    bool has_georef = read_georeference(input_dem_georef, dem_path);
    
    bool has_same_values = true;
    for (int col = 0; col < input_dem.cols(); col++) {
      for (int row = 0; row < input_dem.rows(); row++) {
        // Here need to be careful with numerical precision
        if (std::abs(input_dem(col, row) - float(dem_height)) > 1e-6) {
          has_same_values = false;
          break;
        }
      }
      if (!has_same_values) break;
    } 
    
    if (has_georef && has_same_values && 
        dem.cols() == input_dem.cols() && 
        dem.rows() == input_dem.rows() &&
        dem_georef.get_wkt() == input_dem_georef.get_wkt())
      good_already = true;
  }

  if (!good_already) {
    vw::vw_out() << "Writing mapprojection DEM for " << tag << " image: " << dem_path << "\n";
    bool has_georef = true, has_nodata = true;
    vw::TerminalProgressCallback tpc("asp", ": ");
    vw::cartography::block_write_gdal_image(dem_path, dem, has_georef, dem_georef,
                                            has_nodata, dem_nodata, options, tpc);
  } else {
    vw::vw_out() << "Mapprojection DEM for " << tag << " image: " 
      << dem_path << "\n";
  }
}

} //end namespace asp
