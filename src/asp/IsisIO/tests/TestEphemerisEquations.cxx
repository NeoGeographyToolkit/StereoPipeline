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

#include <asp/IsisIO/PolyEquation.h>
#include <asp/IsisIO/RPNEquation.h>

using namespace vw;
using namespace asp;

double DELTA = 1e-6;

TEST(EphemerisEquations, polynomial_equal_order) {
  PolyEquation poly(0);
  EXPECT_EQ( poly.size(), 3u );
  EXPECT_EQ( poly.type(), "PolyEquation" );
  poly[0] = 10;
  poly[1] = 1;
  poly[2] = 20;
  EXPECT_EQ( poly[0], 10 );
  EXPECT_EQ( poly[1], 1 );
  EXPECT_EQ( poly[2], 20 );

  Vector3 test = poly(20);
  EXPECT_EQ( 10, test[0] );
  EXPECT_EQ( 1,  test[1] );
  EXPECT_EQ( 20, test[2] );

  poly[1] = -50;
  test = poly(-100);
  EXPECT_EQ( 10,  test[0] );
  EXPECT_EQ( -50, test[1] );
  EXPECT_EQ( 20,  test[2] );

  PolyEquation poly2(1);
  EXPECT_EQ( poly2.size(), 6u );
  poly2[0] = 10; poly2[1] = 0.5;
  poly2[2] = 5; poly2[3] = -0.5;
  poly2[4] = 0; poly2[5] = 6;

  test = poly2(20);
  EXPECT_EQ( 20,  test[0] );
  EXPECT_EQ( -5,  test[1] );
  EXPECT_EQ( 120, test[2] );
}

TEST(EphemerisEquations, polynomial_variable_order) {
  PolyEquation poly(0,2,1);
  EXPECT_EQ( 6u, poly.size() );
  EXPECT_EQ( poly.type(), "PolyEquation" );
  poly[0] = 11;
  poly[1] = -5; poly[2] = 0.6; poly[3] = .1;
  poly[4] = -4; poly[5] = 2.5;
  EXPECT_EQ( -5, poly[1] );

  Vector3 test = poly(-1.5);
  EXPECT_NEAR( 11, test[0], DELTA );
  EXPECT_NEAR( -5.675, test[1], DELTA );
  EXPECT_NEAR( -7.75, test[2], DELTA );
}

TEST(EphemerisEquations, polynomial_defined_const) {
  Vector<double> x_coeff(1);
  x_coeff[0] = 11;
  Vector<double> y_coeff(3);
  y_coeff[0] = -5;
  y_coeff[1] = 0.6;
  y_coeff[2] = .1;
  Vector<double> z_coeff(2);
  z_coeff[0] = -4;
  z_coeff[1] = 2.5;
  PolyEquation poly(x_coeff,
                    y_coeff,
                    z_coeff);
  EXPECT_EQ( 6u, poly.size() );
  EXPECT_EQ( poly.type(), "PolyEquation" );
  EXPECT_EQ( -5, poly[1] );

  Vector3 test = poly(-1.5);
  EXPECT_NEAR( 11, test[0], DELTA );
  EXPECT_NEAR( -5.675, test[1], DELTA );
  EXPECT_NEAR( -7.75, test[2], DELTA );
}

TEST(EphemerisEquations, reversepolish) {
  std::string x_eq("3 t t * * 1 +");
  std::string y_eq("t sin 4 * t +");
  std::string z_eq("t t 2 * * 5 t / - abs");
  RPNEquation rpn( x_eq, y_eq, z_eq );
  EXPECT_EQ( 5u, rpn.size() );
  EXPECT_EQ( "RPNEquation", rpn.type() );

  Vector3 test = rpn(-1.5);
  EXPECT_NEAR( 7.75, test[0], DELTA );
  EXPECT_NEAR( -5.48997994641622, test[1], DELTA );
  EXPECT_NEAR( 7.8333333333, test[2], DELTA );

  rpn[0] = 22;
  rpn[2] = 9;
  rpn[3] = 8;
  test = rpn(-1.5);
  EXPECT_NEAR( 50.5, test[0], DELTA );
  EXPECT_NEAR( -10.4774548794365, test[1], DELTA );
  EXPECT_NEAR( 21.333333333, test[2], DELTA );

  rpn.set_time_offset( -20 );
  EXPECT_EQ( -20, rpn.get_time_offset() );
  test = rpn(-1.5); // like eval with 18.5
  EXPECT_NEAR( 7530.5, test[0], DELTA );
  EXPECT_NEAR( 15.4176744337735, test[1], DELTA );
  EXPECT_NEAR( 2737.72972972973, test[2], DELTA );
}
