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

#ifndef __ASP_CORE_OUTLIER_PROCESSING_H__
#define __ASP_CORE_OUTLIER_PROCESSING_H__

// Utilities for handling outliers

#include <vw/Image/ImageViewRef.h>

#include <vector>

namespace asp {

// Estimate a bounding box without outliers. Note that individual percentage factors
// are used in x, y, and z. These are supposed to around 0.75 or so. The outlier factor is 3.0
// or so.
void estimate_inliers_bbox(double pct_factor_x, double pct_factor_y, double pct_factor_z,
                          double outlier_factor,
                          std::vector<double> const& x_vals,
                          std::vector<double> const& y_vals,
                          std::vector<double> const& z_vals,
                          vw::BBox3 & inliers_bbox);
  
// Sample the image and get generous estimates (but without outliers)
// of the maximum triangulation error and of the 3D box containing the
// projected points. These will be tightened later.
double estim_max_tri_error_and_proj_box(vw::ImageViewRef<vw::Vector3> const& proj_points,
                                        vw::ImageViewRef<double> const& error_image,
                                        vw::Vector2 const& remove_outliers_params,
                                        vw::BBox3 & estim_proj_box);
  
} // End namespace asp

#endif//__ASP_CORE_OUTLIER_PROCESSING_H__
