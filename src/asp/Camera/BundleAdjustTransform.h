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

/// \file BundleAdjustTransform.h
///
/// Produce updated cameras by applying optimized parameters or a rigid
/// transform, for bundle_adjust.

#ifndef __BUNDLE_ADJUST_TRANSFORM_H__
#define __BUNDLE_ADJUST_TRANSFORM_H__

#include <asp/Core/BaState.h>
#include <asp/Camera/CsmModel.h>

#include <vw/Camera/CameraModel.h>
#include <vw/Math/Matrix.h>
#include <vw/Math/Vector.h>

#include <boost/shared_ptr.hpp>

#include <vector>

namespace vw {
    class PinholeModel;
    class OpticalBarModel;
    class ControlNetwork;
}

namespace asp {

// Given an input pinhole camera and param changes, apply those, returning
// the new camera.
vw::PinholeModel transformedPinholeCamera(int camera_index,
                                          asp::BaState const& ba_state,
                                          vw::PinholeModel const& in_cam);

// Given an input optical bar camera and param changes, apply those, returning
// the new camera.
vw::OpticalBarModel transformedOpticalBarCamera(int camera_index,
                                                asp::BaState const& ba_state,
                                                vw::OpticalBarModel const& in_cam);

// Given an input CSM camera and param changes, apply those, returning
// the new camera.
boost::shared_ptr<asp::CsmModel> transformedCsmCamera(int camera_index,
                                                      asp::BaState const& ba_state,
                                                      asp::CsmModel const& in_cam);

/// Given a transform with origin at the planet center, like output by pc_align,
/// read the adjustments from param storage, apply this transform on top of
/// them, and write the adjustments back to the param storage. Cameras
/// do not change.
void apply_transform_to_params(vw::Matrix4x4 const& M, asp::BaState &ba_state,
                                std::vector<vw::CamPtr>
                                const& cam_ptrs);

// This function takes advantage of the fact that when it is called the cam_ptrs have the same
//  information as is in ba_state!
void apply_transform_to_cameras_pinhole(vw::Matrix4x4 const& M,
                                        asp::BaState & ba_state,
                                        std::vector<vw::CamPtr>
                                        const& cam_ptrs);

// This function takes advantage of the fact that when it is called the cam_ptrs have the same
//  information as is in ba_state!
void apply_transform_to_cameras_optical_bar(vw::Matrix4x4 const& M,
                                            asp::BaState & ba_state,
                                            std::vector<vw::CamPtr>
                                            const& cam_ptrs);

void apply_transform_to_cameras_csm(vw::Matrix4x4 const& M,
                                    asp::BaState & ba_state,
                                    std::vector<vw::CamPtr>
                                    const& cam_ptrs);

/// Apply a scale-rotate-translate transform to pinhole cameras and control points
void apply_rigid_transform(vw::Matrix3x3 const & rotation,
                           vw::Vector3   const & translation,
                           double                scale,
                           std::vector<vw::CamPtr> &camera_models,
                           boost::shared_ptr<vw::ControlNetwork> const& cnet);

} // end namespace asp

#endif // __BUNDLE_ADJUST_TRANSFORM_H__
