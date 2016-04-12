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


/// \file BundleAdjustUtils.h
///

#ifndef __BUNDLE_ADJUST_UTILS_H__
#define __BUNDLE_ADJUST_UTILS_H__

#include <vw/Math/Vector.h>
#include <vw/Math/Quaternion.h>

#include <string>
#include <vector>

#include <boost/smart_ptr/shared_ptr.hpp>

namespace vw {
  namespace camera {
    class CameraModel;
  }
  namespace ba {
    class ControlNetwork;
  }
}

namespace asp{
  /// Read both kinds of adjustments
  void read_adjustments(std::string const& filename,
                        bool        & piecewise_adjustments,
                        vw::Vector2 & adjustment_bounds,
                        std::vector<vw::Vector3> & position_correction,
                        std::vector<vw::Quat>    & pose_correction,
                        std::string & session);

  /// Write piecewise adjustments
  void write_adjustments(std::string const & filename,
                         vw::Vector2 const & adjustment_bounds,
                         std::vector<vw::Vector3> const& position_correction,
                         std::vector<vw::Quat>    const& pose_correction,
                         std::string              const& session);

  /// Write global adjustments
  void write_adjustments(std::string const& filename,
                         vw::Vector3 const& position_correction,
                         vw::Quat    const& pose_correction);

  ///
  void compute_stereo_residuals(std::vector<boost::shared_ptr<vw::camera::CameraModel> >
                                const& camera_models,
                                vw::ba::ControlNetwork const& cnet);
}

#endif // __BUNDLE_ADJUST_UTILS_H__
