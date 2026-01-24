// __BEGIN_LICENSE__
//  Copyright (c) 2009-2024, United States Government as represented by the
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

typedef vw::PixelGrayA<double> DoubleGrayA;
DoubleGrayA interpDem(double x, double y, 
                      vw::ImageView<DoubleGrayA> const& dem,
                      vw::ImageViewRef<DoubleGrayA> const& interp_dem,
                      double tol,
                      bool propagate_nodata);

} // end namespace asp

#endif //__ASP_CORE_DEM_MOSAIC_H__
