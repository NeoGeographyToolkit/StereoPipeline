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


#include <asp/Sessions/StereoSessionSpot.h>
#include <asp/Camera/XMLBase.h>
#include <asp/Camera/SPOT_XML.h>
#include <boost/scoped_ptr.hpp>
#include <test/Helpers.h>

#include <vw/Stereo/StereoModel.h>

#include <vw/Cartography/GeoTransform.h>

using namespace vw;
using namespace asp;
using namespace xercesc;
using namespace vw::test;

//TODO: Run tests!


/*
TEST(StereoSessionSpot, CreateCamera) {
  StereoSessionDG session;

  boost::shared_ptr<camera::CameraModel> cam1( session.camera_model("", "dg_example1.xml") ),
    cam2( session.camera_model("", "dg_example2.xml") );
  ASSERT_TRUE( cam1.get() != 0 );
  ASSERT_TRUE( cam2.get() != 0 );

  Vector2 m1(13864, 5351), m2(15045, 5183);
  m1 *= 2; m2 *= 2;

  stereo::StereoModel sm(cam1.get(), cam2.get());
  double error;
  Vector3 pos = sm(m1,m2,error);
  EXPECT_LT( error, 5.0 ); // Triangulation should be better than 5 meters.
  EXPECT_LT( norm_2(pos), norm_2(cam1->camera_center(m1)) ); // Point should be below camera.

  // Verify that projection back into the camera hits the right
  // spot. It will slightly be wrong due to the above triangulation error.
#if __GNUC__
#if __x86_64__ || __ppc64__
  EXPECT_VECTOR_NEAR( m1, cam1->point_to_pixel( pos ), 4 );
  EXPECT_VECTOR_NEAR( m2, cam2->point_to_pixel( pos ), 4 );
#else
  EXPECT_VECTOR_NEAR( m1, cam1->point_to_pixel( pos ), 9 );
  EXPECT_VECTOR_NEAR( m2, cam2->point_to_pixel( pos ), 25 );
#endif
#endif

  // A more accurate test is just to project out and back into the same camera
  for ( size_t i = 0; i < 30000; i += 500 ) {
    for ( size_t j = 0; j < 24000; j += 500 ) {
      EXPECT_VECTOR_NEAR( Vector2(i,j),
                          cam1->point_to_pixel( cam1->camera_center(Vector2(i,j)) +
                                                2e4 * cam1->pixel_to_vector( Vector2(i,j) ) ),
                          1e-1 / *pixels* /);
    }
  }

  // Create a camera that only has a single TLC entry. It will throw
  // an error in the event that first line time and TLC entry lookup
  // don't agree. That should be good enough for a test.
  EXPECT_NO_THROW( boost::shared_ptr<camera::CameraModel> cam3( session.camera_model("", "dg_example3.xml") ) );
}


TEST(StereoSessionDG, ProjectRPC) {
  XMLPlatformUtils::Initialize();

  // Read the RPC
  RPCXML xml;
  xml.read_from_file( "dg_example1.xml" );
  boost::scoped_ptr<RPCModel> rpc_model( new RPCModel(*xml.rpc_ptr()) );

  // Read the Digital Globe camera model
  StereoSessionDG session;
  boost::shared_ptr<camera::CameraModel> dg_model( session.camera_model("", "dg_example1.xml") );

  // Verify the measurement between the two cameras
  cartography::Datum datum("WGS84");
  // This should be 0,0
  Vector3 xyz = datum.geodetic_to_cartesian( Vector3(-105.42399,39.833107,2595.9) );
  EXPECT_VECTOR_NEAR( rpc_model->point_to_pixel( xyz ),
                      dg_model->point_to_pixel( xyz ), 25 );
  // This should be 35170, 0
  xyz = datum.geodetic_to_cartesian( Vector3(-105.35823,39.842179,2441.3) );
  EXPECT_VECTOR_NEAR( rpc_model->point_to_pixel( xyz ),
                      dg_model->point_to_pixel( xyz ), 25 );
  // This should be 35170, 23708
  xyz = datum.geodetic_to_cartesian( Vector3(-105.35795,39.803541,2612.6) );
  EXPECT_VECTOR_NEAR( rpc_model->point_to_pixel( xyz ),
                      dg_model->point_to_pixel( xyz ), 25 );
  // This should be 0, 23708
  xyz = datum.geodetic_to_cartesian( Vector3(-105.42418,39.793464,2801.8) );
  EXPECT_VECTOR_NEAR( rpc_model->point_to_pixel( xyz ),
                      dg_model->point_to_pixel( xyz ), 25 );
}

*/
