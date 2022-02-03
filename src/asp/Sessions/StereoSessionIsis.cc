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

/// \file StereoSessionIsis.cc
///


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
#include <vw/FileIO/DiskImageResourceOpenEXR.h>
#include <vw/Camera/CameraModel.h>
#include <vw/Stereo/DisparityMap.h>
#include <vw/InterestPoint/Descriptor.h>
#include <vw/InterestPoint/Detector.h>
#include <vw/InterestPoint/Matcher.h>
#include <vw/InterestPoint/MatrixIO.h>
#include <vw/Cartography/Datum.h>

// Stereo Pipeline
#include <asp/Core/InterestPointMatching.h>
#include <asp/Core/AffineEpipolar.h>
#include <asp/Core/PhotometricOutlier.h>
#include <asp/Camera/CsmModel.h>
#include <asp/IsisIO/IsisCameraModel.h>
#include <asp/IsisIO/DiskImageResourceIsis.h>
#include <asp/IsisIO/Equation.h>
#include <asp/Sessions/StereoSessionIsis.h>

// Boost
#include <boost/filesystem/operations.hpp>
#include <boost/math/special_functions/next.hpp> // boost::float_next
#include <boost/shared_ptr.hpp>

#include <algorithm>

using namespace vw;
using namespace vw::camera;

//#if defined(ASP_HAVE_PKG_ISISIO) && ASP_HAVE_PKG_ISISIO == 1


// #include <asp/IsisIO/IsisCameraModel.h>
// #include <asp/IsisIO/DiskImageResourceIsis.h>
// #include <asp/IsisIO/Equation.h>

// // Boost
// #include <boost/filesystem/operations.hpp>
// #include <boost/math/special_functions/next.hpp> // boost::float_next
// #include <boost/shared_ptr.hpp>
// namespace fs = boost::filesystem;

// #include <algorithm>

// using namespace vw;
// using namespace vw::camera;
// using namespace asp;


// // Allows FileIO to correctly read/write these pixel types
// namespace vw {
//   template<> struct PixelFormatID<Vector3>   { static const PixelFormatEnum value = VW_PIXEL_GENERIC_3_CHANNEL; };

namespace asp {


/// Returns the target datum to use for a given camera model.
/// Note the parameter use_sphere_for_datum.
/// During alignment, we'd like to use the most accurate
/// non-spherical datum, hence radii[2]. However, for the purpose
/// of creating a DEM on non-Earth planets people usually just use
/// a spherrical datum, which we'll do as well.  Maybe at some
/// point this needs to change.
vw::cartography::Datum StereoSessionIsis::get_datum(const vw::camera::CameraModel* cam,
                                                    bool use_sphere_for_datum) const {
  const IsisCameraModel * isis_cam
    = dynamic_cast<const IsisCameraModel*>(vw::camera::unadjusted_model(cam));
  VW_ASSERT(isis_cam != NULL, ArgumentErr() << "StereoSessionISIS: Invalid camera.\n");

  return isis_cam->get_datum(use_sphere_for_datum);
}

// TODO(oalexan1):  Can we share more code with the DG implementation?

boost::shared_ptr<vw::camera::CameraModel>
StereoSessionIsis::load_camera_model
      (std::string const& image_file, std::string const& camera_file, Vector2 pixel_offset) const{

  return load_adjusted_model(m_camera_loader.load_isis_camera_model(camera_file),
                            image_file, camera_file, pixel_offset);
}

// Reverse any pre-alignment that was done to the disparity.
ImageViewRef<PixelMask<Vector2f> >
StereoSessionIsis::pre_pointcloud_hook(std::string const& input_file) {

  std::string dust_result = input_file;
  if (stereo_settings().mask_flatfield) {
    // ****************************************************
    // The following code is for Apollo Metric Camera ONLY!
    // (use at your own risk)
    // ****************************************************
    vw_out() << "\t--> Masking pixels that appear to be dust. "
             << "(NOTE: Use this option with Apollo Metric Camera frames only!)\n";
    photometric_outlier_rejection(this->m_options, this->m_out_prefix, input_file,
                                  dust_result, stereo_settings().corr_kernel[0]);
  }
  return DiskImageView<PixelMask<Vector2f>>(dust_result);
} // End function pre_pointcloud_hook()
  
}

//#endif  // ASP_HAVE_PKG_ISISIO
