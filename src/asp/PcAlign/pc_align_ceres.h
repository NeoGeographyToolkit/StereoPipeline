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

/// \file pc_align_ceres.h
///

#ifndef __ASP_TOOLS_PC_ALIGN_CERES_H__
#define __ASP_TOOLS_PC_ALIGN_CERES_H__

#include <asp/PcAlign/pc_align_utils.h>

#include <vw/Cartography/Datum.h>
#include <vw/Cartography/GeoReference.h>
#include <vw/Image/ImageViewRef.h>
#include <vw/Image/PixelMask.h>
#include <vw/Math/Quaternion.h>

namespace asp {
 
/// Compute least squares alignment, using Ceres
PointMatcher<RealT>::Matrix
least_squares_alignment(DP const& source_point_cloud, // Should not be modified
			vw::Vector3 const& point_cloud_shift,
			vw::cartography::GeoReference        const& dem_georef,
			vw::ImageViewRef<vw::PixelMask<float> > const& dem_ref,
      std::string const& alignment_method,
      int num_iter, int num_threads); 
}

#endif //__ASP_TOOLS_PC_ALIGN_CERES_H__
