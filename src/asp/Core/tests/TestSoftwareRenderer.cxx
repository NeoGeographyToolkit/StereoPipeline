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
#include <vw/Image/ImageView.h>
#include <vw/Image/Manipulation.h>
#include <asp/Core/SoftwareRenderer.h>

#include <vector>

#include <boost/assign/list_of.hpp>
#include <boost/assign/std/vector.hpp>

using namespace vw;
using namespace boost::assign;

class SoftwareRenderTest : public ::testing::Test {
protected:
  SoftwareRenderTest() : render_buffer( 128, 128 ), renderer( 128, 128, &render_buffer(0,0) ), vertices(6), color(3) {
  }

  virtual void SetUp() {
    renderer.Ortho2D( 0, 1, 0, 1 );
    renderer.SetVertexPointer( 2, &vertices[0] );
    renderer.SetColorPointer( 1, &color[0] );
    renderer.Clear(0.0);
  }

  ImageView<float> render_buffer;
  stereo::SoftwareRenderer renderer;
  std::vector<float> vertices, color;
};

TEST_F( SoftwareRenderTest, InsideTriangle ) {
  EXPECT_THROW( renderer.Ortho2D( 0, 0, 0, 0 ), LogicErr );
  renderer.Ortho2D( 0, 128, 0, 128 ); // These bounds are exclusive

  // Verify that clear works
  renderer.Clear( 1.0 );
  for ( size_t i = 0; i < 128; i++ ) {
    for ( size_t j = 0; j < 128; j++ ) {
      EXPECT_EQ( 1.0, render_buffer(i,j) );
    }
  }
  renderer.Clear( 0.0 );

  // Draw a Triangle in the top left
  // Vertices should be draw counter clockwise.
  color = list_of(1.)(1.)(1.);
  vertices = list_of(0.)(0.)(0.)(2.)(2.)(0.0);
  renderer.DrawPolygon(0,3);
  EXPECT_EQ( 1.0, render_buffer(0,0) );
  EXPECT_EQ( 0.0, render_buffer(0,1) );
  EXPECT_EQ( 0.0, render_buffer(1,1) );
  EXPECT_EQ( 0.0, render_buffer(1,0) );

  // Draw a Triangle in the bottom right
  vertices = list_of(127)(127)(127)(129)(129)(127);
  renderer.DrawPolygon(0,3);
  EXPECT_EQ( 1.0, render_buffer(127,127) );
  EXPECT_EQ( 0.0, render_buffer(126,127) );
  EXPECT_EQ( 0.0, render_buffer(127,126) );
  EXPECT_EQ( 0.0, render_buffer(126,126) );

  // Draw a Triangle centered
  color = list_of(1.0)(0.3)(0.6);
  vertices = list_of(64)(43)(43)(86)(86)(86);
  renderer.DrawPolygon(0,3);
  EXPECT_NEAR( 1.0, render_buffer( 63, 43 ), 0.02 );
  EXPECT_NEAR( 0.3, render_buffer( 43, 85 ), 0.02 );
  EXPECT_NEAR( 0.6, render_buffer( 84, 85 ), 0.02 );
  EXPECT_NEAR( 0.63, render_buffer(63, 63 ), 0.1 );
}

TEST_F( SoftwareRenderTest, VertexOrderInvariance ) {
  color = list_of(1.0)(1.0)(1.0);
  vertices = list_of(0.5)(1./3.)(1./3.)(2./3.)(2./3.)(2./3.);
  renderer.DrawPolygon(0,3);
  ImageView<float> ground_truth = copy(render_buffer);
  ASSERT_NE( &ground_truth(0,0), &render_buffer(0,0) );

  // Now rotate
  vertices = list_of(1./3.)(2./3.)(2./3.)(2./3.)(0.5)(1./3.);
  renderer.Clear(0.0);
  renderer.DrawPolygon(0,3);
  EXPECT_SEQ_EQ( ground_truth, render_buffer );

  // Now rotate
  vertices = list_of(2./3.)(2./3.)(0.5)(1./3.)(1./3.)(2./3.);
  renderer.Clear(0.0);
  renderer.DrawPolygon(0,3);
  EXPECT_SEQ_EQ( ground_truth, render_buffer );

  // Now convert to clockwise ... which invokes new code
  vertices = list_of(0.5)(1./3.)(2./3.)(2./3.)(1./3.)(2./3.);
  renderer.Clear(0.0);
  renderer.DrawPolygon(0,3);
  EXPECT_SEQ_EQ( ground_truth, render_buffer );

  // Rotate
  vertices = list_of(2./3.)(2./3.)(1./3.)(2./3.)(0.5)(1./3.);
  renderer.Clear(0.0);
  renderer.DrawPolygon(0,3);
  EXPECT_SEQ_EQ( ground_truth, render_buffer );

  // Rotate
  vertices = list_of(1./3.)(2./3.)(0.5)(1./3.)(2./3.)(2./3.);
  renderer.Clear(0.0);
  renderer.DrawPolygon(0,3);
  EXPECT_SEQ_EQ( ground_truth, render_buffer );
}

TEST_F( SoftwareRenderTest, ClippedOnOneSideTriangle ) {
  color.clear();
  color += 1.,0.5,1.;

  // Draw a triangle just off frame on the left
  vertices.clear();
  vertices += 0.0,0.3,-0.3,0.6,0.3,0.6;
  renderer.Clear(0.0);
  renderer.DrawPolygon(0,3);
  EXPECT_GT( render_buffer(0, 64 ), 0.0 );
  EXPECT_EQ( render_buffer(64, 0 ), 0.0 );
  EXPECT_EQ( render_buffer(127, 64), 0.0 );
  EXPECT_EQ( render_buffer(64, 127), 0.0 );

  // Draw a triangle just off frame on the right
  vertices.clear();
  vertices += 1.0,0.3,0.6,0.6,1.3,0.6;
  renderer.Clear(0.0);
  renderer.DrawPolygon(0,3);
  EXPECT_GT( render_buffer(127, 64), 0.0 );
  EXPECT_EQ( render_buffer(0, 64 ), 0.0 );
  EXPECT_EQ( render_buffer(64, 0 ), 0.0 );
  EXPECT_EQ( render_buffer(64, 127), 0.0 );

  // Draw a triangle just above frame
  vertices.clear();
  vertices += 0.5,-0.3,0.3,0.3,0.7,0.3;
  renderer.Clear(0.0);
  renderer.DrawPolygon(0,3);
  EXPECT_GT( render_buffer(64, 0), 0.0 );
  EXPECT_EQ( render_buffer(127, 64), 0.0 );
  EXPECT_EQ( render_buffer(0, 64 ), 0.0 );
  EXPECT_EQ( render_buffer(64, 127), 0.0 );

  // Draw a triangle just below frame
  vertices.clear();
  vertices += 0.5,0.7,0.3,1.3,0.7,1.3;
  renderer.Clear(0.0);
  renderer.DrawPolygon(0,3);
  EXPECT_GT( render_buffer(64, 127), 0.0 );
  EXPECT_EQ( render_buffer(64, 0), 0.0 );
  EXPECT_EQ( render_buffer(127, 64), 0.0 );
  EXPECT_EQ( render_buffer(0, 64 ), 0.0 );
}

TEST_F( SoftwareRenderTest, ExtremelyLargeTriangle ) {
  color.clear();
  color += 1.0,1.0,1.0;

  // Draw a triangle that covers the entire image
  vertices.clear();
  vertices += 0.,-1.1,0,2.1,1.6,0.5;
  renderer.Clear(0.0);
  renderer.DrawPolygon(0,3);
  for ( size_t i = 0; i < 128; i++ ) {
    for ( size_t j = 0; j < 128; j++ ) {
      EXPECT_EQ( 1.0, render_buffer(i,j) ) << i << "," << j;
    }
  }
}
