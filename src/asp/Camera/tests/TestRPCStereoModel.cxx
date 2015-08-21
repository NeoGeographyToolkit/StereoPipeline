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

#include <vw/Camera/CameraModel.h>
#include <vw/Stereo/StereoModel.h>
#include <test/Helpers.h>
#include <asp/Camera/DG_XML.h>
#include <asp/Camera/RPCModel.h>
#include <asp/Camera/RPCStereoModel.h>

using namespace vw;
using namespace vw::stereo;
using namespace vw::camera;
using namespace vw::math;
using namespace asp;

// TODO: Move this!
boost::shared_ptr<vw::camera::CameraModel> load_rpc_camera_model(std::string const& path)
{
  // Try the default loading method
  RPCModel* rpc_model = NULL;
  try {
    RPCXML rpc_xml; // This is for reading XML files
    rpc_xml.read_from_file(path);
    rpc_model = new RPCModel(*rpc_xml.rpc_ptr()); // Copy the value
  } catch (...) {}
  if (!rpc_model) // The default loading method failed, try the backup method.
  {
    rpc_model = new RPCModel(path); // This is for reading .tif files
  }

  // We don't catch an error here because the user will need to
  // know of a failure at this point.
  return boost::shared_ptr<vw::camera::CameraModel>(rpc_model);
}




void test_stereo_models(const std::string &path1, const std::string &path2) {

  // Load the camera models
  boost::shared_ptr<vw::camera::CameraModel> camPtr1, camPtr2;
  //camPtr1 = load_rpc_camera_model("wv_test1.xml");
  //camPtr2 = load_rpc_camera_model("wv_test2.xml");

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




