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

#ifndef __CORE_CARTOGRAPHY_UTILS_H__
#define __CORE_CARTOGRAPHY_UTILS_H__

#include <vw/Cartography/GeoReference.h>
#include <vw/Math/BBox.h>

#include <string>

namespace asp {

// Auto-compute a local projection. It is assumed that the datum is known.
// For Earth, use UTM or polar stereographic. For other datums, use
// local stereographic.
void setAutoProj(double lat, double lon,
                 vw::cartography::GeoReference & output_georef);

// Snap a BBox2 to a grid: floor on min, ceil on max. This ensures the box
// covers at least the original extent with corners at integer multiples of
// the grid spacing. Uses floor/ceil deliberately instead of round(), because
// round() rounds away from zero at 0.5 boundaries, which causes asymmetric
// snapping for positive vs negative coordinates. For example,
// round(-0.5) = -1 but round(0.5) = 1 (a gap of 2 instead of 1). With
// floor/ceil the behavior is consistent regardless of sign.
void snapBBox2ToGrid(vw::BBox2 &bbox, double spacing);

// Same as snapBBox2ToGrid but for BBox3. The z component is also snapped.
void snapBBox3ToGrid(vw::BBox3 &bbox, double spacing);

} //end namespace asp

#endif // __CORE_CARTOGRAPHY_UTILS_H__
