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

  // Init a georef, the numbers are pretty arbitrary, it just must be valid
  cartography::GeoReference georef;
  georef.set_geographic();
  georef.set_proj4_projection_str("+proj=longlat +a=3396190 +b=3396190 +no_defs ");
  georef.set_well_known_geogcs("D_MARS");
  Matrix3x3 affine;
  affine(0,0) = 0.01; // 100 pix/degree
  affine(1,1) = -0.01; // 100 pix/degree
  affine(2,2) = 1;
  affine(0,2) = 30;   // 30 deg east
  affine(1,2) = -35;  // 35 deg south
  georef.set_transform(affine);

  // For the test below to pass, the files must be present.
  ImageView<float> dem(100, 100);
  double nodata = -1000;
  bool has_nodata = true, has_georef = true;
  TerminalProgressCallback tpc("asp", ": ");
  vw::cartography::GdalWriteOptions opt;

  vw::cartography::block_write_gdal_image("img1.tif", dem, has_georef, georef, has_nodata, nodata, opt, tpc);
  vw::cartography::block_write_gdal_image("img2.tif", dem, has_georef, georef, has_nodata, nodata, opt, tpc);
  vw::cartography::block_write_gdal_image("img3.tif", dem, has_georef, georef, has_nodata, nodata, opt, tpc);
  vw::cartography::block_write_gdal_image("img4.tif", dem, has_georef, georef, has_nodata, nodata, opt, tpc);
  vw::cartography::block_write_gdal_image("img1.cub", dem, has_georef, georef, has_nodata, nodata, opt, tpc);
  vw::cartography::block_write_gdal_image("img2.cub", dem, has_georef, georef, has_nodata, nodata, opt, tpc);
  vw::cartography::block_write_gdal_image("dem.tif", dem, has_georef, georef, has_nodata, nodata, opt, tpc);

  std::ofstream ofs1("img1.xml"); ofs1 << "test" << std::endl; ofs1.close();
  std::ofstream ofs2("img2.xml"); ofs2 << "test" << std::endl; ofs2.close();

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
  EXPECT_EQ(0, camera_paths.size());
  EXPECT_EQ("img1.cub", image_paths [0]);
  EXPECT_EQ("img2.cub", image_paths [1]);
  EXPECT_EQ("run/run" , prefix);
  EXPECT_EQ("",         dem_path);

  // ----
  files.clear();
  files.push_back("img1.tif");
  files.push_back("img2.tif");
  files.push_back("img1.cub");
  files.push_back("img2.cub");
  files.push_back("run/run");
  files.push_back("dem.tif");

  parse_multiview_cmd_files(files, image_paths, camera_paths, prefix, dem_path);
  EXPECT_EQ(2, image_paths.size ());
  EXPECT_EQ(2, camera_paths.size());
  EXPECT_EQ("img1.tif", image_paths [0]);
  EXPECT_EQ("img2.tif", image_paths [1]);
  EXPECT_EQ("img1.cub", camera_paths[0]);
  EXPECT_EQ("img2.cub", camera_paths[1]);
  EXPECT_EQ("run/run",  prefix);
  EXPECT_EQ("dem.tif",  dem_path);

  // ----
  files.clear();
  files.push_back("img1.tif");
  files.push_back("img2.tif");
  files.push_back("img3.tif");
  files.push_back("img4.tif");
  files.push_back("run/run");
  files.push_back("dem.tif");

  parse_multiview_cmd_files(files, image_paths, camera_paths, prefix, dem_path);
  EXPECT_EQ(4, image_paths.size());
  EXPECT_EQ(0, camera_paths.size());
  EXPECT_EQ("img1.tif", image_paths [0]);
  EXPECT_EQ("img2.tif", image_paths [1]);
  EXPECT_EQ("img3.tif", image_paths [2]);
  EXPECT_EQ("img4.tif", image_paths [3]);
  EXPECT_EQ("run/run" , prefix);
  EXPECT_EQ("dem.tif" , dem_path);

} // End test StereoMultiCmdCheck
