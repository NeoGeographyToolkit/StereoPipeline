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


#include <asp/Camera/LinescanDGModel.h>
#include <asp/Camera/RPC_XML.h>
#include <asp/Camera/XMLBase.h>
#include <asp/Camera/RPCModel.h>
#include <boost/scoped_ptr.hpp>
#include <test/Helpers.h>

#include <vw/Stereo/StereoModel.h>

#include <vw/Cartography/GeoTransform.h>

using namespace vw;
using namespace asp;
using namespace xercesc;
using namespace vw::test;

TEST(StereoSessionDG, XMLReading) {
  XMLPlatformUtils::Initialize();

  GeometricXML geo;
  AttitudeXML  att;
  EphemerisXML eph;
  ImageXML     img;
  RPCXML       rpc;

  EXPECT_FALSE( geo.is_good() );
  EXPECT_FALSE( att.is_good() );
  EXPECT_FALSE( eph.is_good() );
  EXPECT_FALSE( img.is_good() );
  EXPECT_FALSE( rpc.is_good() );

  read_xml( "dg_example1.xml", geo, att, eph, img, rpc );

  EXPECT_TRUE( geo.is_good() );
  EXPECT_TRUE( att.is_good() );
  EXPECT_TRUE( eph.is_good() );
  EXPECT_TRUE( img.is_good() );
  EXPECT_TRUE( rpc.is_good() );

  // Checking GEO
  EXPECT_NEAR( 7949.165, geo.principal_distance, 1e-6 );
  EXPECT_EQ( 0, geo.optical_polyorder );
  EXPECT_VECTOR_NEAR( Vector3(), geo.perspective_center, 1e-6 );
  EXPECT_NEAR( 0, geo.camera_attitude.x(), 1e-6 );
  EXPECT_NEAR( 1, geo.camera_attitude.w(), 1e-6 );
  EXPECT_VECTOR_NEAR( Vector2(.05372,140.71193), geo.detector_origin, 1e-6 );
  EXPECT_NEAR( 0, geo.detector_rotation, 1e-6 );
  EXPECT_NEAR( .008, geo.detector_pixel_pitch, 1e-6 );

  // Checking ATT
  EXPECT_FALSE( att.start_time.empty() );
  EXPECT_NEAR( .02, att.time_interval, 1e-6 );
  EXPECT_EQ( 840, att.quat_vec.size() );
  EXPECT_EQ( 840, att.covariance_vec.size() );
  for ( size_t i = 0; i < 840; i++ ) {
    EXPECT_NE( 0, att.quat_vec[i].w() );
    EXPECT_NE( 0, att.covariance_vec[i][5] );
  }
  EXPECT_VECTOR_NEAR( Vector3(3.72e-12, 3.51e-12, 1.12e-13),
                      subvector(att.covariance_vec[0],0,3), 1e-20 );

  // Checking EPH
  EXPECT_FALSE( eph.start_time.empty() );
  EXPECT_NEAR( .02, eph.time_interval, 1e-6 );
  EXPECT_EQ( 840, eph.position_vec.size() );
  EXPECT_EQ( 840, eph.velocity_vec.size() );
  EXPECT_EQ( 840, eph.covariance_vec.size() );
  for ( size_t i = 0; i < 840; i++ ) {
    EXPECT_NE( 0, eph.position_vec[i].x() ) << i;
    EXPECT_NE( 0, eph.velocity_vec[i].x() );
    EXPECT_NE( 0, eph.covariance_vec[i][3] );
  }
  EXPECT_VECTOR_NEAR( Vector3(-1.150529111070304e+06,
                              -4.900037170821411e+06,
                              4.673402253879593e+06),
                      eph.position_vec[0], 1e-6 );

  // Checking IMG
  EXPECT_FALSE( img.tlc_start_time.empty() );
  EXPECT_FALSE( img.first_line_start_time.empty() );
  EXPECT_EQ( 2, img.tlc_vec.size() );
  EXPECT_NEAR( 0, img.tlc_vec[0].first, 1e-8 );
  EXPECT_NEAR( 0, img.tlc_vec[0].second, 1e-8 );
  EXPECT_NEAR( 23708, img.tlc_vec[1].first, 1e-8 );
  EXPECT_NEAR( 1.975667, img.tlc_vec[1].second, 1e-8 );
  EXPECT_EQ( 23708, img.image_size.y() );
  EXPECT_EQ( 35170, img.image_size.x() );

  XMLPlatformUtils::Terminate();
}

TEST(DGCameraModel, CreateCamera) {

  xercesc::XMLPlatformUtils::Initialize();
  
  typedef boost::shared_ptr<vw::camera::CameraModel> CameraModelPtr;
  CameraModelPtr cam1, cam2, cam3;
  cam1 = CameraModelPtr(load_dg_camera_model_from_xml("dg_example1.xml"));
  cam2 = CameraModelPtr(load_dg_camera_model_from_xml("dg_example2.xml"));
  
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
                          1e-1 /*pixels*/);
    }
  }

  // Create a camera that only has a single TLC entry. It will throw
  // an error in the event that first line time and TLC entry lookup
  // don't agree. That should be good enough for a test.
  EXPECT_NO_THROW( cam3=load_dg_camera_model_from_xml("dg_example3.xml"); );

  XMLPlatformUtils::Terminate();
}

