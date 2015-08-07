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
#include <asp/Sessions/StereoSessionIsis.h>
#include <asp/IsisIO/IsisCameraModel.h>
#include <asp/IsisIO/DiskImageResourceIsis.h>
#include <asp/IsisIO/Equation.h>

// Boost
#include <boost/filesystem/operations.hpp>
#include <boost/math/special_functions/next.hpp> // boost::float_next
#include <boost/shared_ptr.hpp>
namespace fs = boost::filesystem;

#include <algorithm>

using namespace vw;
using namespace vw::camera;
using namespace asp;

#if defined(ASP_HAVE_PKG_ISISIO) && ASP_HAVE_PKG_ISISIO == 1

// Allows FileIO to correctly read/write these pixel types
namespace vw {
  template<> struct PixelFormatID<Vector3>   { static const PixelFormatEnum value = VW_PIXEL_GENERIC_3_CHANNEL; };
}

#endif  // ASP_HAVE_PKG_ISISIO
