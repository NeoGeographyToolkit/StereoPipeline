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
#include <asp/Core/StereoSettings.h>
#include <asp/Core/AffineEpipolar.h>
#include <asp/Core/FileUtils.h>
#include <asp/Sessions/StereoSessionGdal.h>
#include <asp/Sessions/StereoSessionASTER.h>
#include <asp/Sessions/StereoSessionNadirPinhole.h>
#include <asp/Camera/RPCModel.h>
#include <asp/Camera/RPC_XML.h>

#include <vw/Image/ImageMath.h>
#include <vw/Image/Manipulation.h>
#include <vw/Image/MaskViews.h>
#include <vw/Image/Transform.h>
#include <vw/Camera/Extrinsics.h>
#include <vw/Math/Matrix.h>
#include <vw/Cartography/Datum.h>

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

  const asp::CsmModel * csm_cam
    = dynamic_cast<const asp::CsmModel*>(vw::camera::unadjusted_model(cam));
  VW_ASSERT(csm_cam != NULL,
            vw::ArgumentErr() << "Could not load a CSM camera.\n");

  return csm_cam->get_datum_csm(spheroid_name, use_sphere_for_non_earth);
}

} // End namespace asp

