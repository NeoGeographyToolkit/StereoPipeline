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

#include <asp/Core/CartographyUtils.h>
#include <vw/Cartography/Utm.h>

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

} //end namespace asp
