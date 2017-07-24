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

#include <vw/Cartography/Datum.h>
#include <asp/Camera/SPOT_XML.h>
#include <asp/Camera/LinescanSpotModel.h>
#include <boost/scoped_ptr.hpp>
#include <test/Helpers.h>
#include <xercesc/util/PlatformUtils.hpp>
#include <iomanip>

using namespace vw;
using namespace asp;
using namespace xercesc;
using namespace vw::test;

// This test is split to a separate function so that the SpotXML destructor is called
//  before the Xerces platform terminate function is called.
void xml_reading_test() {
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
  EXPECT_EQ(48084, xml_reader.center_line);
  EXPECT_EQ(6000,  xml_reader.center_col);

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

  EXPECT_EQ("2008-03-04T12:28:27.000000",  xml_reader.position_logs.front().first);
  EXPECT_NEAR(1.7495691231e+06,  xml_reader.position_logs.front().second.x(), EPS);
  EXPECT_NEAR(-2.2672541162e+06, xml_reader.position_logs.front().second.y(), EPS);
  EXPECT_NEAR(-6.6183333707e+06, xml_reader.position_logs.front().second.z(), EPS);

  EXPECT_EQ("2008-03-04T12:28:27.000000",  xml_reader.velocity_logs.front().first);
  EXPECT_NEAR(-6.2538959540e+03,  xml_reader.velocity_logs.front().second.x(), EPS);
  EXPECT_NEAR(3.1331303950e+03, xml_reader.velocity_logs.front().second.y(), EPS);
  EXPECT_NEAR(-2.7272519760e+03, xml_reader.velocity_logs.front().second.z(), EPS);
}

TEST(SPOT_camera, XMLReading) {
  XMLPlatformUtils::Initialize();
  xml_reading_test();
  XMLPlatformUtils::Terminate();
}

// This test is disabled because the full model does not work with a cropped down
//  XML file.  The full XML files are very large!

/*
TEST(SPOT_camera, SensorTest) {
  std::cout << "Start second" << std::endl;
  XMLPlatformUtils::Initialize();

  boost::shared_ptr<asp::SPOTCameraModel> cam_ptr = asp::load_spot5_camera_model_from_xml("spot_example1.xml");

  std::cout << "Loading model" << std::endl;
  //boost::shared_ptr<asp::SPOTCameraModel> cam_ptr = 
  //    asp::load_spot5_camera_model_from_xml("METADATA.DIM");
  
  std::cout << std::endl;

  //Vector3 local_vec = cam_ptr->get_local_pixel_vector(Vector2(0, 0));
  //std::cout << "local_vec: " << local_vec << std::endl;

  // Load a list of approximately correct tie points
  // TODO: Lower this number!
  double ERROR_THRESH = 50.0; // Since the ground points came from Google Earth, we can't
                              // expect that they will be super accurate.  Allow them to be
                              // this many pixels off from the recorded location.
  const double PIXEL_SCALE = 12; // Convert from preview to full image coordinates
  std::vector<Vector3> gdc_coords;
  std::vector<Vector2> pixel_coords;
  // These coords are from Google Earth
  //gdc_coords.push_back(Vector3(-83.890520, -78.543415, 213));
  //pixel_coords.push_back(Vector2(203*PIXEL_SCALE, 4870*PIXEL_SCALE));
  //gdc_coords.push_back(Vector3(-83.914755, -78.728363, 336));
  //pixel_coords.push_back(Vector2(333*PIXEL_SCALE, 5086*PIXEL_SCALE));
  //gdc_coords.push_back(Vector3(-83.780710, -79.582781, 1261));
  //pixel_coords.push_back(Vector2(960*PIXEL_SCALE, 6031*PIXEL_SCALE));  
  //gdc_coords.push_back(Vector3(-86.341318, -79.084420, 1383));
  //pixel_coords.push_back(Vector2(340*PIXEL_SCALE, 6191*PIXEL_SCALE));  
  
  // These coords are from someone else's SPOT5 DEM.
  gdc_coords.push_back(Vector3(-83.7819, -78.5474, 477.437));
  pixel_coords.push_back(Vector2(2663.304, 58006.2));
  gdc_coords.push_back(Vector3(-84.4778, -78.456, 1531.65));
  pixel_coords.push_back(Vector2(935.1552, 59211.84));
  gdc_coords.push_back(Vector3(-83.9636, -78.7255, 504.305));
  pixel_coords.push_back(Vector2(3967.08, 61081.2));
  gdc_coords.push_back(Vector3(-83.8405, -78.7615, 417.859));
  pixel_coords.push_back(Vector2(4413.156, 61203.36));
  gdc_coords.push_back(Vector3(-83.6087, -79.3809, 1228.67));
  pixel_coords.push_back(Vector2(9991.296, 68912.88));
  gdc_coords.push_back(Vector3(-84.5157, -78.8572, 1122.1));
  pixel_coords.push_back(Vector2(4543.344, 64987.56));

  // Make sure that vector generation is reversible
  Vector2 testPixel(50, 70);
  Vector3 vec       = cam_ptr->pixel_to_vector(testPixel);
  std::cout << "1";
  Vector3 center    = cam_ptr->camera_center(testPixel);
  std::cout << "2";
  Vector3 testPoint = center + vec*400000.0;
  Vector2 testPixel2 = cam_ptr->point_to_pixel(testPoint);
  std::cout << "3";
  double diff = norm_2(testPixel - testPixel2);
  EXPECT_NEAR(0.0, diff, 0.01);


  // Project all of these coordinates into the camera and see what pixel we get.
  vw::cartography::Datum datum; // Use wgs84 default
  const size_t num_points = gdc_coords.size();
  for (size_t i=0; i<num_points; ++i) {

    Vector3 gcc   = datum.geodetic_to_cartesian(gdc_coords[i]);
    std::cout << "Projecting point: " << gcc << std::endl;
    Vector2 pixel = cam_ptr->point_to_pixel(gcc, 48000);
    std::cout << "Got pixel: " << pixel << std::endl;
  
  
    std::cout << "target pixel: " << pixel_coords[i] << std::endl;    
    Vector3 current_vector = cam_ptr->pixel_to_vector(pixel_coords[i]);
    std::cout << "pixel_to_vector: " << current_vector << std::endl;
    
    Vector3 cam_to_point = gcc - cam_ptr->camera_center(pixel_coords[i]);
    std::cout << "cam_to_point: " << normalize(cam_to_point)  << std::endl;
    
    // The pixels should be somewhat close to the expected location.
    diff = norm_2(pixel - pixel_coords[i]);
    std::cout << "DIFF = " << diff << std::endl;
    EXPECT_NEAR(0.0, diff, ERROR_THRESH);
    
  }

  XMLPlatformUtils::Terminate();
}

*/



