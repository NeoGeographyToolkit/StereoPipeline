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

/// \file NuthFit.h
/// Best fit function for Nuth alignment.

#ifndef __ASP_PC_ALIGN_NUTH_FIT_H__
#define __ASP_PC_ALIGN_NUTH_FIT_H__

#include <vw/Math/Vector.h>

#include <vector>

namespace asp {
 
// Form a Ceres optimization problem. Will fit a curve to bin centers and bin
// medians, while taking into acount the bin count and min_bin_sample_count.
void nuthFit(std::vector<double> const& bin_count, 
             std::vector<double> const& bin_centers, 
             std::vector<double> const& bin_median, 
             int inner_iter,
             int num_threads,
             vw::Vector3 & fit_params);

} // end namespace asp

#endif // __ASP_PC_ALIGN_NUTH_FIT_H__
