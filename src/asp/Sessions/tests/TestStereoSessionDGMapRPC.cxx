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


#include <asp/Sessions/StereoSessionDGMapRPC.h>
#include <asp/Camera/RPC_XML.h>
#include <asp/Camera/RPCModel.h>
#include <boost/scoped_ptr.hpp>
#include <test/Helpers.h>

#include <vw/Stereo/StereoModel.h>

using namespace vw;
using namespace asp;
using namespace vw::test;

TEST(StereoSessionDGMapRPC, TransformCycle) {
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
  double dem_height = 2287; // Average height of this area
  fill( lowres_overcrop_image, dem_height );
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
  vw::cartography::GdalWriteOptions opt;
  StereoSessionDGMapRPC session;
  session.initialize( opt, left_image_name, right_image_name,
                      "dg_example1.xml", "dg_example4.xml",
                      "debug/debug", lowres_dem_name);
  RPCXML left_xml, right_xml;
  left_xml.read_from_file( "dg_example1.xml" );
  right_xml.read_from_file( "dg_example4.xml" );
  StereoSessionDGMapRPC::tx_type left_tx = session.tx_left();
  StereoSessionDGMapRPC::tx_type right_tx = session.tx_right();
  cartography::Datum datum;

  // Verify that LX matches what we got by hand.
  for ( size_t j = 0; j < 4; j++ ) {
    for ( size_t i = 0; i < 4; i++ ) {
      Vector3 lonlatheight;
      subvector(lonlatheight,0,2) = lowres_georef.pixel_to_lonlat( Vector2(i,j) );
      lonlatheight.z() = dem_height;
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
      lonlatheight.z() = dem_height;
      Vector3 xyz = datum.geodetic_to_cartesian( lonlatheight );
      EXPECT_VECTOR_NEAR( right_xml.rpc_ptr()->point_to_pixel( xyz ),
                          right_tx.reverse( Vector2(i,j) ), 1e-1 );
    }
  }
}
