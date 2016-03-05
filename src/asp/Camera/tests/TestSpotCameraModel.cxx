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


//#include <asp/Camera/LinescanDGModel.h>
#include <asp/Camera/SPOT_XML.h>
#include <asp/Camera/XMLBase.h>
#include <boost/scoped_ptr.hpp>
#include <test/Helpers.h>

#include <vw/Stereo/StereoModel.h>

#include <vw/Cartography/GeoTransform.h>

using namespace vw;
using namespace asp;
using namespace xercesc;
using namespace vw::test;

TEST(SPOT_camera, XMLReading) {
  XMLPlatformUtils::Initialize();

  SpotXML xml_reader;
  xml_reader.read_xml("spot_example1.xml");

  EXPECT_EQ(300,   xml_reader.image_size[0]); // Cols, a real file is much larger.
  EXPECT_EQ(96168, xml_reader.image_size[1]); // Rows

  const double EPS = 1e-8;
  EXPECT_EQ(4, xml_reader.lonlat_corners.size());
  EXPECT_EQ(4, xml_reader.pixel_corners.size());
  EXPECT_NEAR(-75.519946, xml_reader.lonlat_corners[0].x(), EPS);
  EXPECT_NEAR(-76.636309, xml_reader.lonlat_corners[0].y(), EPS);
  EXPECT_EQ(1, xml_reader.pixel_corners[0].x());
  EXPECT_EQ(1, xml_reader.pixel_corners[0].y());
  EXPECT_NEAR(-71.924271, xml_reader.lonlat_corners[1].x(), EPS);
  EXPECT_NEAR(-77.389028, xml_reader.lonlat_corners[1].y(), EPS);
  EXPECT_EQ(300, xml_reader.pixel_corners[1].x());
  EXPECT_EQ(1,   xml_reader.pixel_corners[1].y());
  EXPECT_NEAR(-88.569672, xml_reader.lonlat_corners[2].x(), EPS);
  EXPECT_NEAR(-80.241112, xml_reader.lonlat_corners[2].y(), EPS);
  EXPECT_EQ(300,   xml_reader.pixel_corners[2].x());
  EXPECT_EQ(96168, xml_reader.pixel_corners[2].y());
  EXPECT_NEAR(-91.818476, xml_reader.lonlat_corners[3].x(), EPS);
  EXPECT_NEAR(-79.298919, xml_reader.lonlat_corners[3].y(), EPS);
  EXPECT_EQ(1,     xml_reader.pixel_corners[3].x());
  EXPECT_EQ(96168, xml_reader.pixel_corners[3].y());

  EXPECT_NEAR(7.5199705115e-04, xml_reader.line_period, EPS);
  EXPECT_EQ("2008-03-04T12:31:39.349737", xml_reader.center_time);
  EXPECT_EQ(48085, xml_reader.center_line);
  EXPECT_EQ(6001,  xml_reader.center_col);

  EXPECT_EQ(10,                  xml_reader.look_angles[9].first);
  EXPECT_NEAR(3.4977741700e-01,  xml_reader.look_angles[9].second.x(), EPS);
  EXPECT_NEAR(-7.1264930800e-02, xml_reader.look_angles[9].second.y(), EPS);

  EXPECT_EQ("2008-03-04T12:31:03.206912",  xml_reader.pose_logs.front().first);
  EXPECT_NEAR(1.1621635280e-03,  xml_reader.pose_logs.front().second.x(), EPS);
  EXPECT_NEAR(-9.5496029568e-04, xml_reader.pose_logs.front().second.y(), EPS);
  EXPECT_NEAR(-1.7549389832e-05, xml_reader.pose_logs.front().second.z(), EPS);

  EXPECT_EQ("2008-03-04T12:32:15.331747",  xml_reader.pose_logs.back().first);
  EXPECT_NEAR(1.2001004714e-03,  xml_reader.pose_logs.back().second.x(), EPS);
  EXPECT_NEAR(-9.5696493468e-04, xml_reader.pose_logs.back().second.y(), EPS);
  EXPECT_NEAR(-2.4331461036e-05, xml_reader.pose_logs.back().second.z(), EPS);


  XMLPlatformUtils::Terminate();
}

