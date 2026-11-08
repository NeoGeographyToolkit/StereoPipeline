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
#include <asp/Core/FileUtils.h>

#include <fstream>
#include <cstdio>

using namespace vw;

namespace asp {

// Two .tiff images paired with two .RPB cameras must split into two images and
// two cameras. The separation only checks each input's existence and extension,
// so create empty placeholder files and remove them at the end.
TEST(Common, separate_images_from_cameras_rpb) {

  std::vector<std::string> inputs, images, cameras;
  bool ensure_equal_sizes = true;

  inputs.push_back("sep_rpb_test_c1_PAN.tiff");
  inputs.push_back("sep_rpb_test_c2_PAN.tiff");
  inputs.push_back("sep_rpb_test_c1_PAN.RPB");
  inputs.push_back("sep_rpb_test_c2_PAN.RPB");

  for (size_t i = 0; i < inputs.size(); i++) {
    std::ofstream ofs(inputs[i].c_str());
    ofs.close();
  }

  EXPECT_NO_THROW(separate_images_from_cameras(inputs, images, cameras, ensure_equal_sizes));
  EXPECT_EQ(2, images.size ());
  EXPECT_EQ(2, cameras.size());

  for (size_t i = 0; i < inputs.size(); i++)
    std::remove(inputs[i].c_str());
}

} // namespace asp
