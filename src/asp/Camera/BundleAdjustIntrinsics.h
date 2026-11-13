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

/// \file BundleAdjustIntrinsics.h
///
/// Parsing and sanity checks for camera intrinsics options in bundle_adjust.

#ifndef __BUNDLE_ADJUST_INTRINSICS_H__
#define __BUNDLE_ADJUST_INTRINSICS_H__

#include <asp/Core/BaState.h>

#include <string>
#include <vector>

namespace asp {

// When distortion params are shared, their number must agree
void distortion_sanity_check(std::vector<int> const& num_dist_params,
                             IntrinsicOptions const& intrinsics_opts,
                             std::vector<double> const& intrinsics_limits);

/// For each option, the string must include a subset of the entries:
///  "focal_length, optical_center, distortion_params"
/// - Need the extra boolean to handle the case where --intrinsics-to-share
///   is provided as "" in order to share none of them.
void load_intrinsics_options(bool        solve_intrinsics,
                             bool        shared_is_specified,
                             std::string intrinsics_to_float_str, // make a copy
                             std::string intrinsics_to_share_str, // make a copy
                             asp::IntrinsicOptions & intrinsics_options);

// Parse the string of limits and make sure they are all valid pairs.
void parse_intrinsics_limits(std::string const& intrinsics_limits_str,
                             std::vector<double> & intrinsics_limits);

} // end namespace asp

#endif // __BUNDLE_ADJUST_INTRINSICS_H__
