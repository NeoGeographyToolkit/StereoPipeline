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

/// \file PointCloudRead.h
///
/// Pre-compiled wrappers for read_asp_point_cloud<N> to avoid
/// instantiating the 12-type template ladder in every caller.

#ifndef __ASP_CORE_POINT_CLOUD_READ_H__
#define __ASP_CORE_POINT_CLOUD_READ_H__

#include <vw/Image/ImageViewRef.h>
#include <vw/Math/Vector.h>
#include <string>

namespace asp {

// Non-template wrappers for reading ASP point clouds with 3, 4, or 6
// channels. These handle the shift-offset logic and call pre-compiled
// channel readers, avoiding the 12-type template ladder.
vw::ImageViewRef<vw::Vector<double, 3>>
read_asp_point_cloud_3(std::string const& filename);
vw::ImageViewRef<vw::Vector<double, 4>>
read_asp_point_cloud_4(std::string const& filename);
vw::ImageViewRef<vw::Vector<double, 6>>
read_asp_point_cloud_6(std::string const& filename);

} // namespace asp

#endif
