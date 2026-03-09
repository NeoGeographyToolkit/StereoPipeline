// __BEGIN_LICENSE__
//  Copyright (c) 2009-2025, United States Government as represented by the
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

#include <asp/Core/CartographyUtils.h>

#include <vw/Cartography/Utm.h>

#include <cmath>
#include <cstdint>

namespace asp {

// Auto-compute a local projection. It is assumed that the datum is known.
// For Earth, use UTM or polar stereographic. For other datums, use
// local stereographic.
void setAutoProj(double lat, double lon,
                 vw::cartography::GeoReference & output_georef) {

  vw::cartography::Datum datum = output_georef.datum();
  if (datum.name().find("WGS_1984") != std::string::npos) {

    vw::cartography::Datum user_datum = output_georef.datum();
    if (lat > 84)
      output_georef.set_proj4_projection_str("+proj=stere +lat_0=90 +lat_ts=70 +lon_0=-45 +k=1 +x_0=0 +y_0=0 +datum=WGS84 +units=m +no_defs");
    else if (lat < -80)
      output_georef.set_proj4_projection_str("+proj=stere +lat_0=-90 +lat_ts=-70 +lon_0=0 +k=1 +x_0=0 +y_0=0 +datum=WGS84 +units=m +no_defs");
    else
     output_georef.set_UTM(vw::cartography::getUTMZone(lat, lon));

  } else {
    output_georef.set_stereographic(lat, lon, 1, 0, 0);
  }

  return;
}

// Snap a value to the nearest grid multiple, rounding down (floor).
// Uses half-grid rounding to avoid floating-point noise causing floor()
// to round down by an extra grid step when the value is very close to
// a grid multiple. Example: floor(-1580598.8 / 0.3) can give -5266699
// instead of -5266698 because the quotient is -5266698.0000000009.
// Approach: divide by half the spacing, round to nearest integer (robust),
// then convert from half-grid to full-grid units with integer division.
double gridFloor(double val, double spacing) {
  double half = 0.5 * spacing;
  int64_t n = llround(val / half); // half-grid integer, robust
  int64_t g = (n >= 0) ? (n / 2) : ((n - 1) / 2); // floor to full grid
  return g * spacing;
}

// Same as gridFloor but rounding up (ceil).
double gridCeil(double val, double spacing) {
  double half = 0.5 * spacing;
  int64_t n = llround(val / half); // half-grid integer, robust
  int64_t g = (n >= 0) ? ((n + 1) / 2) : (n / 2); // ceil to full grid
  return g * spacing;
}

void snapBBox2ToGrid(vw::BBox2 &bbox, double spacing) {
  for (size_t i = 0; i < bbox.min().size(); i++) {
    bbox.min()[i] = gridFloor(bbox.min()[i], spacing);
    bbox.max()[i] = gridCeil(bbox.max()[i], spacing);
  }
}

void snapBBox3ToGrid(vw::BBox3 &bbox, double spacing) {
  for (size_t i = 0; i < bbox.min().size(); i++) {
    bbox.min()[i] = gridFloor(bbox.min()[i], spacing);
    bbox.max()[i] = gridCeil(bbox.max()[i], spacing);
  }
}

} //end namespace asp
