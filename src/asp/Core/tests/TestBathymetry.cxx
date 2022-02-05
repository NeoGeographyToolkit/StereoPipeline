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
#include <asp/Core/Bathymetry.h>

using namespace vw;
using namespace asp;

// Test using Snell's law to see how a ray bends after hitting water
TEST(Bathymetry, BathyStereoModel) {

  double water_refraction_index = 1.333;

  Vector3 camDir(-0.458009742779199258,0.708215158481702134,-0.537269359647531974);
  Vector3 camCtr(1220937.38505603513,-6327090.5660436973,3081502.09552069381);

  std::vector<double> bathy_plane;
  bathy_plane.push_back(0.129446509386046349);
  bathy_plane.push_back(-0.899798011977084089);
  bathy_plane.push_back(0.416661899925893919);
  bathy_plane.push_back(-6374384.66267670784);

  Vector3 waterCtr, waterDir;
  bool ans = asp::snells_law(camCtr, camDir, bathy_plane,  
                             water_refraction_index,  
                             waterCtr, waterDir);
  
  EXPECT_TRUE(ans);
  
  Vector3 expectedWaterDir(-0.377967226380366284,0.770232081899908771,-0.513695742433656122);
  Vector3 expectedWaterCtr(842392.092823269079,-5741750.36671878025,2637448.68844386376);

  EXPECT_VECTOR_NEAR(waterDir, expectedWaterDir, 1e-12);
  EXPECT_VECTOR_NEAR(waterCtr, expectedWaterCtr, 1e-12);

  // Verify that Snell's law holds
  Vector3 plane_normal(bathy_plane[0], bathy_plane[1], bathy_plane[2]);
  double theta1 = acos(dot_prod(plane_normal,  -camDir));  // vectors pointing up in the air
  double theta2 = acos(dot_prod(-plane_normal, waterDir)); // vectors pointing down in the water

  EXPECT_NEAR(sin(theta1), water_refraction_index * sin(theta2), 1e-12);
}

