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
#include <asp/Sessions/RPC/StereoSessionRPC.h>
#include <asp/Sessions/ISIS/StereoSessionIsis.h>
#include <asp/Sessions/Pinhole/StereoSessionPinhole.h>
#include <asp/Sessions/NadirPinhole/StereoSessionNadirPinhole.h>

#include <test/Helpers.h>

using namespace vw;
using namespace asp;

// This is to make sure the developers provide types for their
// sessions. The test is to see if this even compiles.
template <typename T>
class InstantiationTest : public ::testing::Test {
public:
  typedef T SessionT;

  InstantiationTest() {}

  virtual void SetUp() {}

  SessionT session;
};

typedef ::testing::Types<StereoSessionDG, StereoSessionRPC, StereoSessionIsis, StereoSessionPinhole, StereoSessionNadirPinhole> SessionTypes;
TYPED_TEST_CASE( InstantiationTest, SessionTypes );

TYPED_TEST( InstantiationTest, Typedefs ) {
  // Verify object for left transform
  typename TestFixture::SessionT::left_tx_type left_tx =
    this->session.tx_left();

  // Verify object for right transform
  typename TestFixture::SessionT::right_tx_type right_tx =
    this->session.tx_right();

  // Verify object for stereo model
  typename TestFixture::SessionT::stereo_model_type stereo_model();
}
