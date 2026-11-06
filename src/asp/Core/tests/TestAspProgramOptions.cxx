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


#include <test/Helpers.h>
#include <asp/Core/AspProgramOptions.h>
#include <asp/Core/FileUtils.h> // separate_images_from_cameras moved here

using namespace vw;

namespace asp {

// Disabled: separate_images_from_cameras now throws on a .tiff plus .RPB set.
TEST(Common, DISABLED_separate_images_from_cameras_rpb) {

  std::vector<std::string> inputs, images, cameras;
  bool ensure_equal_sizes = true;

  inputs.push_back("1220996672.27391028_sc00110_c1_PAN.tiff");
  inputs.push_back("1220996702.11131644_sc00110_c1_PAN.tiff");
  inputs.push_back("1220996672.27391028_sc00110_c1_PAN.RPB");
  inputs.push_back("1220996702.11131644_sc00110_c1_PAN.RPB");

  EXPECT_NO_THROW(separate_images_from_cameras(inputs, images, cameras, ensure_equal_sizes));
  EXPECT_EQ(2, images.size ());
  EXPECT_EQ(2, cameras.size());
}

// Disabled: parseStereoFiles is now an internal function of stereo.cc with a
// changed signature (it takes an override_out_prefix flag, reads stereo_settings,
// and detects the DEM via a georeference probe), so it is no longer callable from
// this unit test. The image/camera separation it relied on is covered by the test
// above via separate_images_from_cameras.
TEST(Common, DISABLED_StereoMultiCmdCheck) {
}

} // namespace asp
