// __BEGIN_LICENSE__
//  Copyright (c) 2009-2026, United States Government as represented by the
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

/// \file LinescanSpotModel.h
///
/// SPOT 5 CSM camera model loader.

#ifndef __STEREO_CAMERA_LINESCAN_SPOT_MODEL_H__
#define __STEREO_CAMERA_LINESCAN_SPOT_MODEL_H__

#include <boost/shared_ptr.hpp>

#include <string>

namespace asp {

class CsmModel;

/// Load a SPOT5 CSM camera model from an XML file.
/// Populates CSM with vendor extrinsics (positions, velocities, quaternions)
/// and fits intrinsics (focal length, optical center, TRANSVERSE distortion)
/// to the vendor's per-column look angle table.
boost::shared_ptr<CsmModel> load_spot5_csm_camera_model_from_xml(std::string const& path);

} // namespace asp

#endif//__STEREO_CAMERA_LINESCAN_SPOT_MODEL_H__
