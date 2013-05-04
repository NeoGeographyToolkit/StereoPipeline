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


#include <asp/Sessions/RPC/StereoSessionRPC.h>
#include <asp/Sessions/RPC/RPCModel.h>
#include <asp/Sessions/RPC/RPCStereoModel.h>
#include <asp/Sessions/DG/XML.h>
#include <test/Helpers.h>

using namespace vw;
using namespace asp;
using namespace xercesc;

TEST( StereoSessionRPC, InstantiateTest ) {
  XMLPlatformUtils::Initialize();

  // Create an RPC Model
  RPCXML xml;
  xml.read_from_file( "dg_example1.xml" );
  RPCModel model( *xml.rpc_ptr() );

  // Verify some of the values
  EXPECT_NEAR( 4.683662e-3, model.line_num_coeff()[0], 1e-6 );
  EXPECT_NEAR( 1, model.line_den_coeff()[0], 1e-6 );
  EXPECT_NEAR( -7.306375e-3, model.sample_num_coeff()[0], 1e-6 );
  EXPECT_NEAR( 1, model.sample_den_coeff()[0], 1e-6 );
  EXPECT_VECTOR_NEAR( Vector2(17564,11856), model.xy_offset(), 1e-3 );
  EXPECT_VECTOR_NEAR( Vector2(17927,12384), model.xy_scale(), 1e-3 );

  // Verify the Jacobian numerically
  Vector3 location(-105.29,39.745,2281);
  double tol = 1e-4;
  Matrix<double, 2, 3> Je = model.geodetic_to_pixel_Jacobian(location);
  Matrix<double, 2, 3> Jn = model.geodetic_to_pixel_numerical_Jacobian(location, tol);
  double relErr = max(abs(Je-Jn))/max(abs(Je));
  EXPECT_LT(relErr, 1e-5);

  // Verify that if we go from pixel to lonlat and back, then we
  // arrive at the starting point.
  Vector2 pix = 0.0;
  double h = 10.0; 
  Vector2 lonlat  = model.image_to_ground(pix, h);
  Vector2 pix_out = model.geodetic_to_pixel(Vector3(lonlat[0], lonlat[1], h));
  EXPECT_LT( norm_2(pix - pix_out), 1.0e-9 );
  
  // Verify that nothing segfaults or has a run time error.
  EXPECT_NO_THROW( model.calculate_terms( location ) );
  EXPECT_NO_THROW( model.terms_Jacobian3( location ) );
  EXPECT_NO_THROW( model.normalization_Jacobian( location ) );
  EXPECT_NO_THROW( model.geodetic_to_pixel_Jacobian( location ) );
  EXPECT_NO_THROW( model.geodetic_to_pixel( location ) );
  EXPECT_NO_THROW( model.pixel_to_vector( Vector2() ) );
  EXPECT_NO_THROW( model.camera_center( Vector2() ) );

  XMLPlatformUtils::Terminate();
}

TEST( StereoSessionRPC, CheckStereo ) {

  XMLPlatformUtils::Initialize();

  RPCXML xml1;
  xml1.read_from_file( "dg_example1.xml" );
  RPCModel model1( *xml1.rpc_ptr() );

  RPCXML xml2;
  xml2.read_from_file( "dg_example4.xml" );
  RPCModel model2( *xml2.rpc_ptr() );
  std::cout << std::endl;
  
  RPCStereoModel RPC_stereo(&model1, &model2);
  double error;
  Vector3 p = RPC_stereo(Vector2(), Vector2(), error);
  
  EXPECT_NEAR( error, 54682.96251543280232, 1e-3 );
}
