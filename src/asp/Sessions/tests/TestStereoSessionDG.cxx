// __BEGIN_LICENSE__
//  Copyright (c) 2009-2026, United States Government as represented by the
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


#include <asp/Sessions/StereoSessionGdal.h>
#include <asp/Camera/XMLBase.h>
#include <asp/Camera/RPC_XML.h>
#include <asp/Camera/RPCModel.h>
#include <boost/shared_ptr.hpp>
#include <test/Helpers.h>

#include <vw/Stereo/StereoModel.h>

#include <vw/Cartography/GeoTransform.h>

using namespace vw;
using namespace xercesc;
using namespace vw::test;

namespace asp {


// Compare the RPC camera model against the Digital Globe (linescan) model.
TEST(StereoSessionDG, ProjectRPC) {
  XMLPlatformUtils::Initialize();

  // Read the RPC
  RPCXML xml;
  xml.read_from_file("dg_example1.xml");
  boost::shared_ptr<RPCModel> rpc_model(new RPCModel(*xml.rpc_ptr()));

  // Read the Digital Globe camera model
  StereoSessionDG session;
  boost::shared_ptr<vw::CameraModel> dg_model(session.camera_model("", "dg_example1.xml"));

  // Verify the measurement between the two cameras
  vw::Datum datum("WGS84");
  // This should be 0,0
  Vector3 xyz = datum.geodetic_to_cartesian(Vector3(-105.42399,39.833107,2595.9));
  EXPECT_VECTOR_NEAR(rpc_model->point_to_pixel(xyz),
                     dg_model->point_to_pixel(xyz), 25);
  // This should be 35170, 0
  xyz = datum.geodetic_to_cartesian(Vector3(-105.35823,39.842179,2441.3));
  EXPECT_VECTOR_NEAR(rpc_model->point_to_pixel(xyz),
                     dg_model->point_to_pixel(xyz), 25);
  // This should be 35170, 23708
  xyz = datum.geodetic_to_cartesian(Vector3(-105.35795,39.803541,2612.6));
  EXPECT_VECTOR_NEAR(rpc_model->point_to_pixel(xyz),
                     dg_model->point_to_pixel(xyz), 25);
  // This should be 0, 23708
  xyz = datum.geodetic_to_cartesian(Vector3(-105.42418,39.793464,2801.8));
  EXPECT_VECTOR_NEAR(rpc_model->point_to_pixel(xyz),
                     dg_model->point_to_pixel(xyz), 25);
}

} // namespace asp
