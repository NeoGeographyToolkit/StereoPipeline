// __BEGIN_LICENSE__
//  Copyright (c) 2006-2025, United States Government as represented by the
//  Administrator of the National Aeronautics and Space Administration. All
//  rights reserved.
//
//  The NASA Vision Workbench is licensed under the Apache License,
//  Version 2.0 (the "License"); you may not use this file except in
//  compliance with the License. You may obtain a copy of the License at
//  http://www.apache.org/licenses/LICENSE-2.0
//
//  Unless required by applicable law or agreed to in writing, software
//  distributed under the License is distributed on an "AS IS" BASIS,
//  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
//  See the License for the specific language governing permissions and
//  limitations under the License.
// __END_LICENSE__

#include <asp/PcAlign/SlopeAspect.h>

#include <omp.h>

namespace vw {
namespace cartography {

// Calculate the DEM slope and aspect in degrees at a given location.
// The code below is adapted from GDAL.
// https://github.com/OSGeo/gdal/blob/46412f0fb7df3f0f71c6c31ff0ae97b7cef6fa61/apps/gdaldem_lib.cpp#L1284C1-L1301C1
//https://github.com/OSGeo/gdal/blob/46412f0fb7df3f0f71c6c31ff0ae97b7cef6fa61/apps/gdaldem_lib.cpp#L1363
// It looks to be an approximation of the Zevenbergen and Thorne method, not the
// real thing. Using same logic for consistency.
inline 
void SlopeAspectZevenbergenThorneAlg(vw::ImageView<vw::PixelMask<float>> const& dem,
                                     vw::cartography::GeoReference const& georef,
                                     int col, int row,
                                     vw::PixelMask<float> & slope,
                                     vw::PixelMask<float> & aspect) {

  // The slope and aspect start as invalid
  slope.invalidate();
  aspect.invalidate();
  
  int num_cols = dem.cols(), num_rows = dem.rows();
  
  // If the pixel is at the border, return invalid
  if (col <= 0 || row <= 0 || col >= num_cols - 1 || row >= num_rows - 1)
    return;
  
  // Form the afWin vector with the pixel values
  std::vector<double> afWin(9);
  int count = 0;
  for (int irow = -1; irow <= 1; irow++) {
    for (int icol = -1; icol <= 1; icol++) {
      
      auto val = dem(col + icol, row + irow);
      
      // If invalid, return invalid right away. Note how we check here only
      // either icol or irow being zero, as we don't use the diagonal values.
      // The original Zevenbergen and Thorne method uses those values,
      // but the GDAL implementation does not.
      if (!is_valid(val) && (icol == 0 || irow == 0))
        return;
        
      afWin[count] = val.child();
      count++;
    }
  }
  
  // To find the local grid size in x, convert pixels to projected coordinates.
  vw::Vector2 left_pt = georef.pixel_to_point(vw::Vector2(col - 1, row));
  vw::Vector2 right_pt = georef.pixel_to_point(vw::Vector2(col + 1, row));
  double grid_x = vw::math::norm_2(right_pt - left_pt) / 2.0;

  // Same for y
  vw::Vector2 top_pt = georef.pixel_to_point(vw::Vector2(col, row - 1));
  vw::Vector2 bottom_pt = georef.pixel_to_point(vw::Vector2(col, row + 1));
  double grid_y = vw::math::norm_2(bottom_pt - top_pt) / 2.0;
  
  // Compute the slope
  double dx = (afWin[3] - afWin[5]) / grid_x;
  double dy = (afWin[7] - afWin[1]) / grid_y;
  double key = dx * dx + dy * dy;
  // The division by 2 is needed because that's how centered differences are found
  slope = atan(sqrt(key)/2.0) * 180.0 / M_PI;
  slope.validate();

  // Compute the aspect   
  // TODO(oalexan1): It is not clear if need to divide by grid_x or grid_y here.
  // The GDAL code does not, but I think it should. These grid sizes are usually 
  // about the same in projected coordinates, so likely it does not matter.
  dx = afWin[5] - afWin[3];
  dy = afWin[7] - afWin[1];  
  if (dx == 0 && dy == 0) {
    aspect.invalidate();
    return;
  }
  double a = atan2(dy, -dx) * 180.0 / M_PI;
  // Using the logic for when psData->bAngleAsAzimuth is true
  if (a > 90.0)
      a = 450.0 - a;
  else
      a = 90.0 - a;  
  if (a >= 360.0)
    a -= 360.0;
    
  aspect = a;
  aspect.validate();  
}

// Calculate the DEM slope and aspect in degrees. Use the same logic as gdaldem. 
void calcSlopeAspect(vw::ImageView<vw::PixelMask<float>> const& dem, 
                     vw::cartography::GeoReference const& georef,
                     // Outputs
                     vw::ImageView<vw::PixelMask<float>> & slope,
                     vw::ImageView<vw::PixelMask<float>> & aspect) {

  // Allocate space for the output slope and aspect
  slope.set_size(dem.cols(), dem.rows());
  aspect.set_size(dem.cols(), dem.rows());
  
  #pragma omp parallel for
  for (int col = 0; col < dem.cols(); col++) {
    for (int row = 0; row < dem.rows(); row++) {
      SlopeAspectZevenbergenThorneAlg(dem, georef, col, row, 
                                      slope(col, row), aspect(col, row)); // outputs
    }
  }

}

}} // end namespace vw::cartography
