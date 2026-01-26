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

/// \file DemMosaic.h
/// Utility functions for dem_mosaic

#ifndef __ASP_CORE_DEM_MOSAIC_H__
#define __ASP_CORE_DEM_MOSAIC_H__

#include <vw/Image/ImageView.h>
#include <vw/Image/ImageViewRef.h>
#include <vw/Image/PixelTypes.h>
#include <vw/Image/Manipulation.h>
#include <vw/Image/Algorithms.h>
#include <vw/Math/Vector.h>
#include <vw/Math/BBox.h>

// Forward declarations
namespace vw {
  namespace cartography {
    class GeoReference;
  }
}

namespace asp {

void blurWeights(vw::ImageView<double> & weights, double sigma);

void applyExternalWeights(std::string const& weight_file,
                          double min_weight, bool invert_weights,
                          vw::BBox2i const& in_box,
                          vw::ImageView<double>& local_wts);

double computePlateauedWeights(vw::Vector2 const& pix, bool horizontal,
                               std::vector<double> const& centers,
                               std::vector<double> const& widths,
                               double max_weight_val);

// Compute centerline weights using plateaued weight function. Distinguishes
// between interior holes (assigned hole_fill_value) and border pixels (-1).
void centerlineWeightsWithHoles(vw::ImageView<vw::PixelMask<double>> const& img,
                                vw::ImageView<double> & weights,
                                double max_weight_val,
                                double hole_fill_value);

double erfSmoothStep(double x, double M, double L);

// Convert a projected-space bounding box to pixel coordinates with snapping.
// Transforms all 4 corners, rounds near-integer values (within tol) to integers,
// then floors/ceils to ensure integer pixel boundaries.
vw::BBox2 pointToPixelBboxSnapped(vw::cartography::GeoReference const& georef,
                                  vw::BBox2 const& point_bbox, double tol);

// Read a georeference from a file and throw if not found.
vw::cartography::GeoReference readGeorefOrThrow(std::string const& file);

typedef vw::PixelGrayA<double> DoubleGrayA;
DoubleGrayA interpDem(double x, double y,
                      vw::ImageView<DoubleGrayA> const& dem,
                      vw::ImageViewRef<DoubleGrayA> const& interp_dem,
                      double tol,
                      bool propagate_nodata);

// Pixel and weight manipulation utilities
void invalidateByThreshold(double threshold, double nodata_value,
                           vw::ImageView<DoubleGrayA> & dem);

void invalidateNaN(double nodata_value,
                   vw::ImageView<DoubleGrayA> & dem);

void setNoDataByWeight(double out_nodata_value,
                       vw::ImageView<double> & tile,
                       vw::ImageView<double> & weight);

void clampWeights(double bias, vw::ImageView<double> & weight);

void raiseToPower(double exponent, vw::ImageView<double> & weight);

void setWeightsAsAlphaChannel(vw::ImageView<double> const& weights,
                              vw::ImageView<DoubleGrayA>& dem);

// Tile processing utilities
void divideByWeight(vw::ImageView<double> & tile,
                    vw::ImageView<double> const& weights);

void demMosaicDatumCheck(std::vector<vw::cartography::GeoReference> const& georefs,
                         vw::cartography::GeoReference const& out_georef);

void accumWeightedTiles(double out_nodata_value,
                        int save_dem_weight,
                        int dem_index,
                        vw::ImageView<double> const& tile_clip,
                        vw::ImageView<double> const& weight_clip,
                        // Outputs
                        vw::ImageView<double> & tile,
                        vw::ImageView<double> & weights,
                        vw::ImageView<double> & saved_weight);

void computeWeightedAverage(int save_dem_weight,
                            vw::ImageView<double> & tile,
                            vw::ImageView<double> & weights,
                            vw::ImageView<double> & saved_weight);

void saveTileWeight(int dem_iter, vw::BBox2i const& bbox,
                    vw::ImageView<double> const& local_wts,
                    vw::cartography::GeoReference const& georef);

} // end namespace asp

#endif //__ASP_CORE_DEM_MOSAIC_H__
