// __BEGIN_LICENSE__
//  Copyright (c) 2006-2013, United States Government as represented by the
//  Administrator of the National Aeronautics and Space Administration. All
//  rights reserved.
//
//  The NASA Vision Workbench is licensed under the Apache License,
//  Version 2.0 (the "License"); you may not use this file except in
//  compliance with the License. You may obtain a copy of the License at
//  http://www.apache.org/licenses/LICENSE-2.0
//
//  Unless required by applicable law or agreed to in writing, software
//  distributed under the License is distributed on an "AS IS" BASIS,
//  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
//  See the License for the specific language governing permissions and
//  limitations under the License.
// __END_LICENSE__


// TestRPCStereoModel.h


// This also contains the RPCModel tests so they should be seperated out some time.

#include <vw/Camera/CameraModel.h>
#include <vw/Stereo/StereoModel.h>
#include <test/Helpers.h>
#include <asp/Camera/XMLBase.h>
#include <asp/Camera/RPC_XML.h>
#include <asp/Camera/LinescanDGModel.h>
#include <asp/Camera/RPCModel.h>
#include <asp/Camera/RPCStereoModel.h>
#include <asp/Core/StereoSettings.h>
#include <xercesc/util/PlatformUtils.hpp>


using namespace vw;
using namespace vw::stereo;
using namespace vw::camera;
using namespace vw::math;
using namespace asp;




/// Load an RPC camera file
/// - TODO: Remove this once we have a nice RPC load function outside the Loader class.
boost::shared_ptr<vw::camera::CameraModel> load_rpc_camera_model(std::string const& path)
{
  // Try the default loading method
  RPCModel* rpc_model = NULL;
  try {
    RPCXML rpc_xml; // This is for reading XML files
    rpc_xml.read_from_file(path);
    rpc_model = new RPCModel(*rpc_xml.rpc_ptr()); // Copy the value
  } catch (...) {}

  // We don't catch an error here because the user will need to
  // know of a failure at this point.
  return boost::shared_ptr<asp::RPCModel>(rpc_model);
}

void test_stereo_models(const std::string &path1, const std::string &path2) {

  // Load the camera models
  boost::shared_ptr<vw::camera::CameraModel> camPtr1, camPtr2;

  // Object to handle camera model loading
  camPtr1 = load_rpc_camera_model(path1);
  camPtr2 = load_rpc_camera_model(path2);

  // Set up the stereo models
  vw::stereo::StereoModel plainStereoModel(camPtr1.get(), camPtr2.get());
  asp::RPCStereoModel     rpcStereoModel  (camPtr1.get(), camPtr2.get());

  // Generate a bunch of pixel pairs
  const int NUM_PIXELS = 10;
  Vector2 pixelBase(10000, 10000);
  std::vector<Vector2> pixels1(NUM_PIXELS), pixels2(NUM_PIXELS);
  for (int i=0; i<NUM_PIXELS; ++i) {
    pixels1[i] = pixelBase + Vector2(i, 0);
    pixels2[i] = pixelBase;
  }

  // Try out a bunch of pixel pairs
  double errorPlain, errorRpc;
  for (int i=0; i<NUM_PIXELS; ++i) {
    // Use both stereo models to find the ray intersection
    Vector3 xyzPlain = plainStereoModel(pixels1[i], pixels2[i], errorPlain);
    Vector3 xyzRpc   = rpcStereoModel  (pixels1[i], pixels2[i], errorRpc  );

    // The results should be identical
    EXPECT_VECTOR_NEAR(xyzPlain, xyzRpc, 1e-4);
    //EXPECT_LT(abs(errorPlain - errorRpc), 1e-4);
  }
}


TEST(RPCXML, ReadRPC) {
  xercesc::XMLPlatformUtils::Initialize();

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
  
  xercesc::XMLPlatformUtils::Terminate();
}



TEST( RPCStereoModel, mvpMatchTest ) {
  xercesc::XMLPlatformUtils::Initialize();
  test_stereo_models("wv_mvp_1.xml", "wv_mvp_2.xml");
  xercesc::XMLPlatformUtils::Terminate();
}

TEST( RPCStereoModel, greenlandMatchTest ) {
  xercesc::XMLPlatformUtils::Initialize();
  test_stereo_models("wv_test1.xml", "wv_test2.xml");
  xercesc::XMLPlatformUtils::Terminate();
}



TEST( StereoSessionRPC, InstantiateTest ) {
  xercesc::XMLPlatformUtils::Initialize();

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
  Vector2 pix(0, 0);
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

  xercesc::XMLPlatformUtils::Terminate();
}

TEST( StereoSessionRPC, CheckStereo ) {

  xercesc::XMLPlatformUtils::Initialize();

  RPCXML xml1, xml2;
  xml1.read_from_file( "dg_example1.xml" );
  xml2.read_from_file( "dg_example4.xml" );
  RPCModel model1( *xml1.rpc_ptr() );
  RPCModel model2( *xml2.rpc_ptr() );
  std::cout << std::endl;

  RPCStereoModel RPC_stereo(&model1, &model2);
  double error;
  RPC_stereo(Vector2(), Vector2(), error);

  //EXPECT_NEAR( error, 54682.96251543280232, 1e-3 );
  EXPECT_NEAR( error, 54705.95473285091, 1e-3 );


  xercesc::XMLPlatformUtils::Terminate();
}



/// Make sure that the AdjustedCameraModel class handles cropping with RPC models
TEST( StereoSessionRPC, CheckRpcCrop ) {

  xercesc::XMLPlatformUtils::Initialize();

  // Load the camera model as DG and RPC
  boost::shared_ptr<vw::camera::CameraModel> rpcModel = load_rpc_camera_model("dg_example1.xml");

  stereo_settings().disable_correct_velocity_aberration = true;
  boost::shared_ptr<vw::camera::CameraModel> dgModel  = load_dg_camera_model_from_xml("dg_example1.xml");

  // Verify that the RPC and DG models are similar
  Vector2 testPix(100, 200);
  Vector3 rpcVector = rpcModel->pixel_to_vector(testPix);
  Vector3 dgVector  = dgModel->pixel_to_vector(testPix);
  EXPECT_LT( norm_2(rpcVector - dgVector), 1.0e-4 );

  // Now try the same thing with a cropped image
  Vector2 pixel_offset(40, 80);
  Vector3 position_correction;
  Quaternion<double> pose_correction = Quat(math::identity_matrix<3>());

  boost::shared_ptr<camera::CameraModel> croppedRpc(
    new vw::camera::AdjustedCameraModel(rpcModel, position_correction, pose_correction, pixel_offset));
  boost::shared_ptr<camera::CameraModel> croppedDg(
    new vw::camera::AdjustedCameraModel(dgModel, position_correction, pose_correction, pixel_offset));

  rpcVector = croppedRpc->pixel_to_vector(testPix);
  dgVector  = croppedDg->pixel_to_vector(testPix);
  EXPECT_LT( norm_2(rpcVector - dgVector), 1.0e-4 );

  xercesc::XMLPlatformUtils::Terminate();
}
