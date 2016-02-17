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
#include <asp/Core/PointUtils.h>

using namespace vw;
using namespace asp;

TEST( PointUtils, CsvConv ) {
  
  // Needed for the Lola tests below.
  vw::cartography::GeoReference geo;
  geo.set_well_known_geogcs("D_MOON");
  
  // Check format parsing
  CsvConv conv;
  conv.parse_csv_format("2:file 5:lon 6:lat 7:height_above_datum", "");
  EXPECT_TRUE(conv.is_configured());
  
  // Check line reading
  bool is_first_line = false;
  bool success = false;
  std::string line = "1, name.jpg, 3, 4, 5, 6, 7, 8, 9";
  CsvConv::CsvRecord vals = conv.parse_csv_line(is_first_line, success, line);
  EXPECT_EQ("name.jpg", conv.file_from_csv(vals));
  Vector3 point = conv.csv_to_geodetic(vals, geo);
  EXPECT_EQ(5, vals.point_data[0]);
  EXPECT_EQ(6, vals.point_data[1]);
  EXPECT_EQ(7, vals.point_data[2]);

  // Check format parsing
  conv.parse_csv_format("1:file 8:y 4:x 2:z", "");
  EXPECT_TRUE(conv.is_configured());
  
  // Check line reading
  line = "name.tif, 2, 3, 4, 5, 6, 7, 8, 9";
  vals = conv.parse_csv_line(is_first_line, success, line);
  EXPECT_EQ("name.tif", conv.file_from_csv(vals));
  point = conv.csv_to_cartesian(vals, geo);
  EXPECT_EQ(4, point[0]); // x
  EXPECT_EQ(8, point[1]); // y
  EXPECT_EQ(2, point[2]); // z
  
  // Check the unsort function
  const double mean_lon = 45; // Should not affect test
  point = conv.cartesian_to_csv(point, geo, mean_lon);
  EXPECT_EQ(2, point[0]); // z
  EXPECT_EQ(4, point[1]); // x
  EXPECT_EQ(8, point[2]); // y
  
  
  // Check format parsing
  conv.parse_csv_format("1:file 12:lat 13:lon 14:height_above_datum", "");
  EXPECT_TRUE(conv.is_configured());   
    
  // Check line reading
  line = "2009_10_20_0778        778    228373.34782  -17.79782     2782345.669     -147696.005     1521956.201       10263.249      0.441      1.973    191.403    -75.22419715    -98.31170774       10263.249 740";
  vals = conv.parse_csv_line(is_first_line, success, line);
  EXPECT_EQ("2009_10_20_0778", conv.file_from_csv(vals));
  point = conv.csv_to_geodetic(vals, geo);
  EXPECT_EQ(-98.31170774,  point[0]); // lon
  EXPECT_EQ(-75.22419715,  point[1]); // lat
  EXPECT_EQ(10263.249,     point[2]); // height
  

  // Check format parsing
  conv.parse_csv_format("1:height_above_datum 3:lon 2:lat", "");
  EXPECT_TRUE(conv.is_configured());   


  // Check line reading
  line = "658.4780	69.3737799999999964,	310.0611559999999827,	"; // Formatting is pretty bad here!
  vals = conv.parse_csv_line(is_first_line, success, line);
  EXPECT_EQ("", conv.file_from_csv(vals));
  point = conv.csv_to_geodetic(vals, geo);
  EXPECT_EQ(310.0611559999999827, point[0]); // lon
  EXPECT_EQ(69.3737799999999964,  point[1]); // lat
  EXPECT_EQ(658.4780,             point[2]); // height


  // Check format parsing
  conv.parse_csv_format("4:lat 2:lon 3:radius_km", "");
  EXPECT_TRUE(conv.is_configured());   

  // There is some loss of precision in the radius to height conversion!
  const double delta = 0.0001;

  // Check line reading
  line = "2009-09-06T05:50:31.41569740	3.7773769	1735.898125	27.4985349	"; // Formatting is pretty bad here
  vals = conv.parse_csv_line(is_first_line, success, line);
  EXPECT_EQ("", conv.file_from_csv(vals));
  point = conv.csv_to_geodetic(vals, geo);
  EXPECT_NEAR(3.77738,    point[0], delta); // lon
  EXPECT_NEAR(27.4985,    point[1], delta); // lat
  EXPECT_NEAR(-1501.8749, point[2], delta); // height



  // Check format parsing
  conv.parse_csv_format("3:lat 2:radius_m 4:lon", "");
  EXPECT_TRUE(conv.is_configured());   

  // Check line reading
  line = "2009-09-06T05:50:31.41569740	1735898.125	27.4985349	3.7773769	"; // Formatting is pretty bad here!
  vals = conv.parse_csv_line(is_first_line, success, line);
  EXPECT_EQ("", conv.file_from_csv(vals));
  point = conv.csv_to_geodetic(vals, geo);
  EXPECT_NEAR(3.77738,    point[0], delta); // lon
  EXPECT_NEAR(27.4985,    point[1], delta); // lat
  EXPECT_NEAR(-1501.8749, point[2], delta); // height
 
  
  
}
