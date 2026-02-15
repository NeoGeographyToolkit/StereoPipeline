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

/// \file StereoSessionIsis.cc
///

#include <asp/asp_config.h> // defines ASP_HAVE_PKG_ISIS
#if defined(ASP_HAVE_PKG_ISIS) && ASP_HAVE_PKG_ISIS == 1

// Stereo Pipeline
#include <asp/Core/AffineEpipolar.h>
#include <asp/Core/StereoSettings.h>
#include <asp/IsisIO/IsisCameraModel.h>
#include <asp/IsisIO/DiskImageResourceIsis.h>
#include <asp/IsisIO/IsisSpecialPixels.h>
#include <asp/Sessions/StereoSessionIsis.h>
#include <asp/Core/BaseCameraUtils.h>
#include <asp/Core/ImageUtils.h>
#include <asp/Core/ImageNormalization.h>

// Vision Workbench
#include <vw/Core/Settings.h>
#include <vw/Core/Log.h>
#include <vw/Math/Functors.h>
#include <vw/Image/Algorithms.h>
#include <vw/Image/EdgeExtension.h>
#include <vw/Image/ImageViewRef.h>
#include <vw/Image/Manipulation.h>
#include <vw/Image/ImageMath.h>
#include <vw/Image/MaskViews.h>
#include <vw/Image/Statistics.h>
#include <vw/Image/Transform.h>
#include <vw/FileIO/DiskImageView.h>
#include <vw/FileIO/DiskImageResourceGDAL.h>
#include <vw/FileIO/MatrixIO.h>
#include <vw/FileIO/GdalWriteOptions.h>
#include <vw/Camera/CameraModel.h>
#include <vw/Stereo/DisparityMap.h>
#include <vw/FileIO/MatrixIO.h>
#include <vw/Cartography/Datum.h>
#include <vw/FileIO/DiskImageUtils.h>
#include <vw/Image/Filter.h>
#include <vw/Core/Stopwatch.h>

// Boost
#include <boost/filesystem.hpp>
#include <boost/filesystem/operations.hpp>
#include <boost/math/special_functions/next.hpp> // boost::float_next
#include <boost/shared_ptr.hpp>

#include <algorithm>

using namespace vw;
using namespace vw::camera;
using namespace vw::cartography;
namespace fs = boost::filesystem;

namespace asp {

bool StereoSessionIsis::supports_multi_threading () const {
  return false;
}

/// Returns the target datum to use for a given camera model. Note the parameter
/// use_sphere_for_non_earth. During alignment, we'd like to use the most
/// accurate non-spherical datum, hence radii[2]. However, for the purpose of
/// creating a DEM on non-Earth planets people usually just use a spherical
/// datum, which we'll do as well.  Maybe at some point this needs to change.
vw::cartography::Datum StereoSessionIsis::get_datum(const vw::camera::CameraModel* cam,
                                                    bool use_sphere_for_non_earth) const {
  const IsisCameraModel * isis_cam
    = dynamic_cast<const IsisCameraModel*>(vw::camera::unadjusted_model(cam));
  VW_ASSERT(isis_cam != NULL, ArgumentErr() << "StereoSessionISIS: Invalid camera.\n");

  return isis_cam->get_datum_isis(use_sphere_for_non_earth);
}

boost::shared_ptr<vw::camera::CameraModel>
StereoSessionIsis::load_camera_model(std::string const& image_file,
                                     std::string const& camera_file,
                                     std::string const& ba_prefix,
                                     Vector2 pixel_offset) const {

  // If the camera file is empty, then we assume the image file has the camera.
  std::string l_cam = camera_file;
  if (l_cam.empty())
    l_cam = image_file;

  return load_adjusted_model(m_camera_loader.load_isis_camera_model(l_cam),
                            image_file, camera_file, ba_prefix, pixel_offset);
}

}

#endif  // ASP_HAVE_PKG_ISIS
