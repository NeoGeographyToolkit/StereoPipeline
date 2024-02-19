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

} // end namespace asp
