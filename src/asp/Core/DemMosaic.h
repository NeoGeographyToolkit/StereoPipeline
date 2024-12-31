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

namespace asp {

void blurWeights(vw::ImageView<double> & weights, double sigma);

void applyExternalWeights(std::string const& weight_file,
                          double min_weight, bool invert_weights,
                          vw::BBox2i const& in_box,
                          vw::ImageView<double>& local_wts);

} // end namespace asp

#endif //__ASP_CORE_DEM_MOSAIC_H__
