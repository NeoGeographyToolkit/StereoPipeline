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


#include <test/Helpers.h>
#include <asp/Core/Common.h>

using namespace vw;
using namespace asp;

TEST( Common, StereoMultiCmdCheck ) {

  std::vector<std::string> files;
  files.push_back("img1.tif");
  files.push_back("img2.tif");
  files.push_back("img1.xml");
  files.push_back("img2.xml");
  files.push_back("run/run");

  std::vector<std::string> image_paths, camera_paths;
  std::string prefix, dem_path;
  parse_multiview_cmd_files(files, image_paths, camera_paths, prefix, dem_path);
  EXPECT_EQ(2, image_paths.size ());
  EXPECT_EQ(2, camera_paths.size());
  EXPECT_EQ("img1.tif", image_paths [0]);
  EXPECT_EQ("img2.tif", image_paths [1]);
  EXPECT_EQ("img1.xml", camera_paths[0]);
  EXPECT_EQ("img2.xml", camera_paths[1]);
  EXPECT_EQ("run/run" , prefix );
  EXPECT_EQ("",         dem_path);

  // ----

  files.clear();
  files.push_back("img1.cub");
  files.push_back("img2.cub");
  files.push_back("run/run");

  parse_multiview_cmd_files(files, image_paths, camera_paths, prefix, dem_path);
  EXPECT_EQ(2, image_paths.size ());
  EXPECT_EQ(2, camera_paths.size());
  EXPECT_EQ("img1.cub", image_paths [0]);
  EXPECT_EQ("img2.cub", image_paths [1]);
  EXPECT_EQ("img1.cub", camera_paths[0]);
  EXPECT_EQ("img2.cub", camera_paths[1]);
  EXPECT_EQ("run/run" , prefix);
  EXPECT_EQ("",         dem_path);

  // ----

  files.clear();
  files.push_back("img1.tif");
  files.push_back("img2.tif");
  files.push_back("img1.cub");
  files.push_back("img2.cub");
  files.push_back("run/run");
  files.push_back("test_dem.tif");

  parse_multiview_cmd_files(files, image_paths, camera_paths, prefix, dem_path);
  EXPECT_EQ(2, image_paths.size ());
  EXPECT_EQ(2, camera_paths.size());
  EXPECT_EQ("img1.tif", image_paths [0]);
  EXPECT_EQ("img2.tif", image_paths [1]);
  EXPECT_EQ("img1.cub", camera_paths[0]);
  EXPECT_EQ("img2.cub", camera_paths[1]);
  EXPECT_EQ("run/run",  prefix);
  EXPECT_EQ("test_dem.tif",  dem_path);

  // ----

  files.clear();
  files.push_back("img1.tif");
  files.push_back("img2.tif");
  files.push_back("img3.tif");
  files.push_back("img4.tif");
  files.push_back("run/run");
  files.push_back("test_dem.tif");

  parse_multiview_cmd_files(files, image_paths, camera_paths, prefix, dem_path);
  EXPECT_EQ(4, image_paths.size());
  EXPECT_EQ(4, camera_paths.size());
  EXPECT_EQ("img1.tif", image_paths [0]);
  EXPECT_EQ("img2.tif", image_paths [1]);
  EXPECT_EQ("img3.tif", image_paths [2]);
  EXPECT_EQ("img4.tif", image_paths [3]);
  EXPECT_EQ("img1.tif", camera_paths[0]);
  EXPECT_EQ("img2.tif", camera_paths[1]);
  EXPECT_EQ("img3.tif", camera_paths[2]);
  EXPECT_EQ("img4.tif", camera_paths[3]);
  EXPECT_EQ("run/run" , prefix);
  EXPECT_EQ("test_dem.tif" , dem_path); 








} // End test StereoMultiCmdCheck
