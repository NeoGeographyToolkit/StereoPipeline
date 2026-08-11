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
#include <asp/Core/FileUtils.h>

#include <cpl_vsi.h>

using namespace asp;

TEST(FileUtils, isVsiPath) {

  // GDAL virtual file systems
  EXPECT_TRUE(asp::isVsiPath("/vsizip/data.zip/cloud.tif"));
  EXPECT_TRUE(asp::isVsiPath("/vsitar//home/user/data.tar/cloud.tif"));
  EXPECT_TRUE(asp::isVsiPath("/vsicurl/https://example.com/cloud.tif"));
  EXPECT_TRUE(asp::isVsiPath("/vsis3/bucket/cloud.tif"));

  // Ordinary local paths, including ones that merely mention vsi
  EXPECT_FALSE(asp::isVsiPath("cloud.tif"));
  EXPECT_FALSE(asp::isVsiPath("/home/user/cloud.tif"));
  EXPECT_FALSE(asp::isVsiPath("/home/user/vsizip/cloud.tif"));
  EXPECT_FALSE(asp::isVsiPath("vsizip/cloud.tif")); // must be absolute
  EXPECT_FALSE(asp::isVsiPath(""));

  // GDAL matches these prefixes case-sensitively, so this is a local path
  EXPECT_FALSE(asp::isVsiPath("/VSIZIP/data.zip/cloud.tif"));
}

TEST(FileUtils, fileExists) {

  // A local file that exists. TEST_SRCDIR is this file's directory, so the
  // check does not depend on the directory the test is run from.
  EXPECT_TRUE(asp::fileExists(std::string(TEST_SRCDIR) + "/TestFileUtils.cxx"));
  EXPECT_FALSE(asp::fileExists("no_such_file_should_ever_exist.tif"));

  // A virtual path, exercising the GDAL branch in both directions. /vsimem/ is
  // in-memory, so this touches neither the disk nor the network.
  const char* mem_file = "/vsimem/test_file_utils.bin";
  VSIFCloseL(VSIFOpenL(mem_file, "wb"));
  EXPECT_TRUE(asp::fileExists(mem_file));
  VSIUnlink(mem_file);
  EXPECT_FALSE(asp::fileExists(mem_file));

  // A virtual path whose archive does not exist must be false, not an error
  EXPECT_FALSE(asp::fileExists("/vsizip/no_such_archive.zip/cloud.tif"));
  EXPECT_FALSE(asp::fileExists("/vsitar/no_such_archive.tar/cloud.tif"));
}
