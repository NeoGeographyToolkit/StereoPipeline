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

#include <vw/Cartography/Datum.h>
#include <vw/Cartography/GeoReference.h>
#include <vw/Image/ImageViewRef.h>
#include <vw/Image/PixelMask.h>
#include <vw/Math/Quaternion.h>

#include <pointmatcher/PointMatcher.h>

typedef double RealT; // We will use doubles in libpointmatcher.
typedef PointMatcher<RealT> PM;
typedef PM::DataPoints DP;

namespace asp {
 
/// Compute alignment using least squares
PointMatcher<RealT>::Matrix
least_squares_alignment(DP const& source_point_cloud, // Should not be modified
			vw::Vector3 const& point_cloud_shift,
			vw::cartography::GeoReference        const& dem_georef,
			vw::ImageViewRef<vw::PixelMask<float> > const& dem_ref,
      std::string const& alignment_method,
      int num_iter, int num_threads); 

vw::Vector3 get_cloud_gcc_coord(DP const& point_cloud, 
                                vw::Vector3 const& shift, int index);

template<class F>
void extract_rotation_translation(F * transform, vw::Quat & rotation, 
                                  vw::Vector3 & translation) {
  
  vw::Vector3 axis_angle;
  for (int i = 0; i < 3; i++){
    translation[i] = transform[i];
    axis_angle[i]  = transform[i+3];
  }
  rotation = vw::math::axis_angle_to_quaternion(axis_angle);
}

/// Interpolates the DEM height at the input coordinate.
/// - Returns false if the coordinate falls outside the valid DEM area.
bool interp_dem_height(vw::ImageViewRef<vw::PixelMask<float>> const& dem,
                       vw::cartography::GeoReference          const& georef,
                       vw::Vector3                            const& lonlat,
                       double                                      & dem_height);

/// Consider a 4x4 matrix T which implements a rotation + translation
/// y = A*x + b. Consider a point s in space close to the points
/// x. We want to make that the new origin, so the points x get
/// closer to origin. In the coordinates (x2 = x - s, y2 = y - s) the
/// transform becomes y2 + s = A*(x2 + s) + b, or
/// y2 = A*x2 + b + A*s - s. Encode the obtained transform into another 4x4 matrix T2.
PointMatcher<RealT>::Matrix apply_shift(PointMatcher<RealT>::Matrix const& T,
                                        vw::Vector3 const& shift);


}

#endif //__ASP_TOOLS_PC_ALIGN_CERES_H__
