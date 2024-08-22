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

/// \file StereoSessionGdal.cc
///
#include <vw/Image/ImageMath.h>
#include <vw/Image/Manipulation.h>
#include <vw/Image/MaskViews.h>
#include <vw/Image/Transform.h>
#include <vw/Camera/Extrinsics.h>
#include <vw/Math/Matrix.h>
#include <vw/Cartography/Datum.h>

#include <asp/Core/StereoSettings.h>
#include <asp/Core/AffineEpipolar.h>
#include <asp/Sessions/StereoSessionGdal.h>
#include <asp/Sessions/StereoSessionASTER.h>
#include <asp/Sessions/StereoSessionNadirPinhole.h>
#include <asp/Camera/RPCModel.h>
#include <asp/Camera/RPC_XML.h>

#include <boost/shared_ptr.hpp>
#include <boost/filesystem/operations.hpp>
namespace fs = boost::filesystem;

#include <iostream>
#include <string>
#include <vector>
#include <algorithm>

using namespace vw;

namespace asp {

/// Returns the target datum to use for a given camera model
vw::cartography::Datum StereoSessionCsm::get_datum(const vw::camera::CameraModel* cam,
                                                   bool use_sphere_for_non_earth) const {

  // Peek at the .cub file to get the planet name without reading
  // it as an ISIS camera (which can fail unless the ISISDATA
  // folder exists, and for CSM that is not guaranteed.)
  // The CSM camera .json file itself lacks this information.
  std::string spheroid_name = asp::read_target_name(m_left_image_file);
  std::string datum_name = "D_" + spheroid_name; // may be refined later

  const asp::CsmModel * cast_csm_cam
    = dynamic_cast<const asp::CsmModel*>(vw::camera::unadjusted_model(cam));
  VW_ASSERT(cast_csm_cam != NULL,
            vw::ArgumentErr() << "Could not load a CSM camera.\n");

  // Read the ellipsoid radii
  vw::Vector3 radii = cast_csm_cam->target_radii();
  double radius1 = (radii[0] + radii[1]) / 2; // average the x and y axes (semi-major) 
  double radius2 = radius1;

  // Auto-guess the datum if not available
  vw::cartography::Datum wgs84("WGS84");
  vw::cartography::Datum moon("D_MOON");
  vw::cartography::Datum mars("D_MARS");
  bool is_wgs84 = (std::abs(wgs84.semi_major_axis() - radius1)  < 1e-7 &&
                   std::abs(wgs84.semi_minor_axis() - radii[2]) < 1e-7);
  bool is_moon =  (std::abs(moon.semi_major_axis()  - radius1)  < 1e-7 &&
                   std::abs(moon.semi_minor_axis()  - radii[2]) < 1e-7);
  bool is_mars =  (std::abs(mars.semi_major_axis()  - radius1)  < 1e-7 &&
                   std::abs(mars.semi_minor_axis()  - radii[2]) < 1e-7);
  
  if (boost::to_lower_copy(spheroid_name).find("unknown") != std::string::npos ||
      spheroid_name.empty()) {
    // Unknown datum. Try to fill in the name from above.
    if (is_wgs84)
      return wgs84;
    if (is_moon)
      return moon;
    if (is_mars)
      return mars;
  }
  
  // For Earth always use two radii. The logic below should distinguish Venus.
  bool has_earth_radius = (std::abs(radius1/wgs84.semi_major_axis() - 1.0) < 0.05);
  if (!use_sphere_for_non_earth || has_earth_radius)
    radius2 = radii[2]; // let the semi-minor axis be distinct from the semi-major axis
  
  vw::cartography::Datum datum(datum_name, spheroid_name,
                               "Reference Meridian", radius1, radius2, 0);
  
  return datum;
}
  
} // End namespace asp

