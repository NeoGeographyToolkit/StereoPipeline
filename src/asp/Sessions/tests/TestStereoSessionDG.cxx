// __BEGIN_LICENSE__
//  Copyright (c) 2009-2012, United States Government as represented by the
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


#include <asp/Sessions/DG/StereoSessionDG.h>
#include <asp/Sessions/DG/XML.h>
#include <asp/Sessions/RPC/RPCModel.h>
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
  AttitudeXML att;
  EphemerisXML eph;
  ImageXML img;
  RPCXML rpc;

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

TEST(StereoSessionDG, CreateCamera) {
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
  // spot. It will slightly be wrong due to the above triangulation
  // error.
  EXPECT_VECTOR_NEAR( m1, cam1->point_to_pixel( pos ), 4 );
  EXPECT_VECTOR_NEAR( m2, cam2->point_to_pixel( pos ), 4 );

  // A more accurate test is just to project out and back into the
  // same camera
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
  EXPECT_NO_THROW( boost::shared_ptr<camera::CameraModel> cam3( session.camera_model("", "dg_example3.xml") ) );
}

TEST(StereoSessionDG, ReadRPC) {
  XMLPlatformUtils::Initialize();

  RPCXML xml;

  EXPECT_FALSE( xml.is_good() );

  xml.read_from_file( "dg_example1.xml" );

  EXPECT_TRUE( xml.is_good() );

  EXPECT_VECTOR_NEAR( Vector2(17564,11856), xml.rpc_ptr()->xy_offset(), 1e-6 );
  EXPECT_VECTOR_NEAR( Vector2(17927,12384), xml.rpc_ptr()->xy_scale(), 1e-6 );
  EXPECT_VECTOR_NEAR( Vector3(-105.2903,39.7454,2281), xml.rpc_ptr()->lonlatheight_offset(), 1e-6 );
  EXPECT_VECTOR_NEAR( Vector3(.1345,.1003,637), xml.rpc_ptr()->lonlatheight_scale(), 1e-6 );
  EXPECT_NEAR( 4.683662e-3, xml.rpc_ptr()->line_num_coeff()[0], 1e-6 );
  EXPECT_NEAR( 4.395393e-6, xml.rpc_ptr()->line_num_coeff()[19], 1e-6 );
  EXPECT_NEAR( 1, xml.rpc_ptr()->line_den_coeff()[0], 1e-6 );
  EXPECT_NEAR( -3.71156e-8, xml.rpc_ptr()->line_den_coeff()[19], 1e-6 );
  EXPECT_NEAR( -7.306375e-3, xml.rpc_ptr()->sample_num_coeff()[0], 1e-6 );
  EXPECT_NEAR( -1.585929e-6, xml.rpc_ptr()->sample_num_coeff()[19], 1e-6 );
  EXPECT_NEAR( 1, xml.rpc_ptr()->sample_den_coeff()[0], 1e-6 );
  EXPECT_NEAR( -1.211995e-7, xml.rpc_ptr()->sample_den_coeff()[19], 1e-6 );
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

TEST(StereoSessionDG, MapProjectedInput) {
  // Generate GeoReference projection that matches something that the
  // RPC models in dg_example1 and dg_example4 see.
  cartography::GeoReference lowres_georef; // WGS84
  lowres_georef.set_equirectangular( 37.745, -103.29, 37 );
  Matrix<double> tx = math::identity_matrix<3>();
  tx(0,0) = tx(1,1) = 500;
  tx(0,2) = tx(1,2) = -1000;
  lowres_georef.set_transform( tx );

  // Make low res images. We have to overcrop, otherwise the bicubic
  // interpolation of the DEM will have edge effects.
  cartography::GeoReference lowres_overcrop_georef = lowres_georef;
  tx(0,2) = tx(1,2) = -2000;
  lowres_overcrop_georef.set_transform( tx );
  ImageView<float> lowres_overcrop_image(8,8);
  fill( lowres_overcrop_image, 2287 ); // Average height of this area
  UnlinkName lowres_dem_name( "lowres_dem.tif" );
  write_georeferenced_image( lowres_dem_name, lowres_overcrop_image, lowres_overcrop_georef );

  // Make hires images
  cartography::GeoReference hires_georef = lowres_georef;
  tx(0,0) = tx(1,1) = 100;
  tx(0,2) = tx(1,2) = -1000;
  hires_georef.set_transform( tx );
  ImageView<float> hires_image(20,20);
  fill( hires_image, 1 );
  ImageView<float> lowres_image(4,4);
  fill( lowres_image, 2 );
  UnlinkName left_image_name("faked_left.tif"), right_image_name("faked_right.tif");
  write_georeferenced_image( left_image_name, lowres_image, lowres_georef );
  write_georeferenced_image( right_image_name, hires_image, hires_georef );

  // Create session
  BaseOptions opt;
  StereoSessionDG session;
  session.initialize( opt, left_image_name, right_image_name,
                      "dg_example1.xml", "dg_example4.xml",
                      "debug/debug", lowres_dem_name, "", "", "" );
  RPCXML left_xml, right_xml;
  left_xml.read_from_file( "dg_example1.xml" );
  right_xml.read_from_file( "dg_example4.xml" );
  StereoSessionDG::left_tx_type left_tx = session.tx_left();
  StereoSessionDG::right_tx_type right_tx = session.tx_right();
  cartography::Datum datum;

  // Verify that LX matches what we got by hand.
  for ( size_t j = 0; j < 4; j++ ) {
    for ( size_t i = 0; i < 4; i++ ) {
      Vector3 lonlatheight;
      subvector(lonlatheight,0,2) = lowres_georef.pixel_to_lonlat( Vector2(i,j) );
      lonlatheight.z() = 2287;
      Vector3 xyz = datum.geodetic_to_cartesian( lonlatheight );
      EXPECT_VECTOR_NEAR( left_xml.rpc_ptr()->point_to_pixel( xyz ),
                          left_tx.reverse( Vector2(i,j) ), 1e-1 );
    }
  }

  // Verify that RX matches what we got by hand
  for ( size_t j = 0; j < 20; j++ ) {
    for ( size_t i = 0; i < 20; i++ ) {
      Vector3 lonlatheight;
      subvector(lonlatheight,0,2) = hires_georef.pixel_to_lonlat( Vector2(i,j) );
      lonlatheight.z() = 2287;
      Vector3 xyz = datum.geodetic_to_cartesian( lonlatheight );
      EXPECT_VECTOR_NEAR( right_xml.rpc_ptr()->point_to_pixel( xyz ),
                          right_tx.reverse( Vector2(i,j) ), 1e-1 );
    }
  }
}
