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


/// \file StereoSessionNadirPinhole.cc
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
using namespace asp;

namespace asp {

std::string StereoSessionNadirPinhole::name() const {
  return "nadirpinhole"; 
}

SessionPtr StereoSessionNadirPinhole::construct() { 
  return SessionPtr(new StereoSessionNadirPinhole); 
}

// Must override the StereoSessionPinhole function
bool StereoSessionNadirPinhole::have_datum() const {
  return StereoSession::have_datum();
}

} // End namespace asp
