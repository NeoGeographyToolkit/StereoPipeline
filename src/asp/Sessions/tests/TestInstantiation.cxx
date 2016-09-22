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


#include <asp/Sessions/StereoSessionGdal.h>
#include <asp/Sessions/StereoSessionDGMapRPC.h>
#include <asp/Sessions/StereoSessionRPC.h>
#include <asp/Sessions/StereoSessionIsis.h>
#include <asp/Sessions/StereoSessionPinhole.h>
#include <asp/Sessions/StereoSessionNadirPinhole.h>

#include <test/Helpers.h>

using namespace vw;
using namespace asp;

// This is to make sure the developers provide types for their
// sessions. The test is to see if this even compiles. Not if it
// runs. (It doesn't)
template <typename T>
class InstantiationTest : public ::testing::Test {
public:
  typedef T SessionT;

  InstantiationTest() {}

  virtual void SetUp() {}

  SessionT session;
};

typedef ::testing::Types<StereoSessionDG, StereoSessionDGMapRPC, StereoSessionRPC,
#if defined(ASP_HAVE_PKG_ISISIO) && ASP_HAVE_PKG_ISISIO == 1
                         StereoSessionIsis,
#endif
                         StereoSessionPinhole, StereoSessionNadirPinhole> SessionTypes;
TYPED_TEST_CASE( InstantiationTest, SessionTypes );

TYPED_TEST( InstantiationTest, Typedefs ) {
  try {
    // Verify object for left transform
    typename TestFixture::SessionT::tx_type left_tx =
      this->session.tx_left();
    // Verify object for right transform
    typename TestFixture::SessionT::tx_type right_tx =
      this->session.tx_right();
    // Verify object for stereo model
    typename TestFixture::SessionT::stereo_model_type stereo_model();
  } catch ( const vw::Exception& e ) {}
}

TEST( Instantiation, Names ) {
  std::vector<StereoSession*> sessions;
  vw::cartography::GdalWriteOptions opt;
  sessions.push_back( StereoSessionDG::construct() );
  sessions.push_back( StereoSessionDGMapRPC::construct() );
  sessions.push_back( StereoSessionRPC::construct() );
#if defined(ASP_HAVE_PKG_ISISIO) && ASP_HAVE_PKG_ISISIO == 1
  sessions.push_back( StereoSessionIsis::construct() );
#endif
  sessions.push_back( StereoSessionPinhole::construct() );
  sessions.push_back( StereoSessionNadirPinhole::construct() );

  for ( size_t i = 0; i < sessions.size(); i++ ) {
    EXPECT_TRUE( sessions[i] );
  }

  std::set<std::string> names;
  for ( size_t i = 0; i < sessions.size(); i++ ) {
    EXPECT_EQ( names.end(), names.find( sessions[i]->name() ) );
    names.insert( sessions[i]->name() );
  }
  EXPECT_EQ( names.size(), sessions.size() );

  for ( size_t i = 0; i < sessions.size(); i++ ) {
    delete sessions[i];
  }
}
