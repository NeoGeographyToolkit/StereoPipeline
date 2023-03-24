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

/// \file SfsImageProc.h
/// Image processing routines for SfS

#ifndef __SFS_IMAGE_PROC_H__
#define __SFS_IMAGE_PROC_H__

#include <vw/Image/ImageView.h>
#include <vw/Image/ImageViewRef.h>

namespace vw {
  class GdalWriteOptions;
  namespace cartography {
    class GeoReference;
  }
}

#include <vector>
namespace asp {

// See the .cc file for the documentation.
void adjustBorderlineDataWeights(int cols, int rows,
                                 int blending_dist, double blending_power,
                                 vw::GdalWriteOptions const& opt,
                                 vw::cartography::GeoReference const& geo,
                                 std::set<int> const& skip_images,
                                 std::string const& out_prefix, // for debug data
                                 std::vector<std::string> const& input_images, 
                                 std::vector<std::string> const& input_cameras, 
                                 std::vector<vw::ImageView<double>> & ground_weights);

// See the .cc file for the documentation.
vw::ImageView<double> blendingWeights(vw::ImageViewRef<vw::PixelMask<float>> const& img,
                                      double blending_dist,
                                      double blending_power,
                                      int min_blend_size);
  
// Find the points on a given DEM that are shadowed by other points of
// the DEM.  Start marching from the point on the DEM on a ray towards
// the sun in small increments, until hitting the maximum DEM height.
bool isInShadow(int col, int row, vw::Vector3 const& sunPos,
                vw::ImageView<double> const& dem, double max_dem_height,
                double gridx, double gridy,
                vw::cartography::GeoReference const& geo);

void areInShadow(vw::Vector3 const& sunPos, vw::ImageView<double> const& dem,
                 double gridx, double gridy,
                 vw::cartography::GeoReference const& geo,
                 vw::ImageView<float> & shadow);
  
} // end namespace asp

#endif // __SFS_IMAGE_PROC_H__

