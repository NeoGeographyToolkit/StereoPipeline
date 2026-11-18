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

// The GCP-based camera initializations work only for pinhole cameras, without
// a camera position file. Otherwise they must fail, rather than be silently
// skipped.

#include <asp/Camera/BundleAdjustGcp.h>

#include <vw/Core/Exception.h>

#include <test/Helpers.h>

#include <vector>

namespace asp {

TEST(BundleAdjustGcp, GcpInitCameraType) {

  std::vector<asp::BACameraType> non_pinhole = {asp::BaCameraType_OpticalBar,
                                                asp::BaCameraType_CSM,
                                                asp::BaCameraType_Other};
  asp::BACameraType pinhole = asp::BaCameraType_Pinhole;
  bool no_pos_file = false;
  bool pos_file = true;

  // Pinhole accepts each option
  EXPECT_NO_THROW(checkGcpInitCameraType(pinhole, no_pos_file, true, false, false));
  EXPECT_NO_THROW(checkGcpInitCameraType(pinhole, no_pos_file, false, true, false));
  EXPECT_NO_THROW(checkGcpInitCameraType(pinhole, no_pos_file, false, false, true));

  // With a camera position file, the options would have no effect, so must fail
  EXPECT_NO_THROW(checkGcpInitCameraType(pinhole, pos_file, false, false, false));
  EXPECT_THROW(checkGcpInitCameraType(pinhole, pos_file, true, false, false),
               vw::ArgumentErr);
  EXPECT_THROW(checkGcpInitCameraType(pinhole, pos_file, false, true, false),
               vw::ArgumentErr);
  EXPECT_THROW(checkGcpInitCameraType(pinhole, pos_file, false, false, true),
               vw::ArgumentErr);

  for (auto camera_type: non_pinhole) {
    // No GCP-based initialization requested, so no failure
    EXPECT_NO_THROW(checkGcpInitCameraType(camera_type, no_pos_file,
                                           false, false, false));

    // Each option must fail
    EXPECT_THROW(checkGcpInitCameraType(camera_type, no_pos_file, true, false, false),
                 vw::ArgumentErr);
    EXPECT_THROW(checkGcpInitCameraType(camera_type, no_pos_file, false, true, false),
                 vw::ArgumentErr);
    EXPECT_THROW(checkGcpInitCameraType(camera_type, no_pos_file, false, false, true),
                 vw::ArgumentErr);
  }
}

} // end namespace asp
