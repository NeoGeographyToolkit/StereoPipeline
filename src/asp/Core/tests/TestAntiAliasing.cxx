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


#include <test/Helpers.h>
#include <asp/Core/AntiAliasing.h>

using namespace vw;
using namespace asp;

TEST( AntiAliasing, VerifySameOperation ) {

  // This just verifies that Weighted Summation View produces the same
  // result as the simplier to write WeightedAAFilter. This is what I
  // used to verify that the new code worked as it was kinda complex.

  typedef PixelMask<float> PT;
  ImageView<PT > input( 4, 4 );
  input( 0,0 ) = PT(1);
  input( 2,0 ) = PT(2);
  input( 3,0 ) = PT(-1);
  input( 0,1 ) = PT(0);
  input( 1,1 ) = PT(1);
  input( 2,1 ) = PT(2);
  input( 0,2 ) = input( 1,2 ) = input( 2,2 ) = input( 3,2 ) = PT(2);
  input( 0,3 ) = input( 1,3 ) = input( 2,3 ) = input( 3,3 ) = PT(1);

  ImageView< PT > output_org =
    per_pixel_accessor_filter( input, WeightedAAFilter( 3 ) );

  for ( size_t y = 0; y < 4; y++ ) {
    for ( size_t x = 0; x < 4; x++ ) {
      ASSERT_TRUE( is_valid( output_org(x,y) ) );
    }
  }

  ImageView< PT > output_new =
    weighted_summation( input, 3 );

  for ( size_t y = 0; y < 4; y++ ) {
    for ( size_t x = 0; x < 4; x++ ) {
      EXPECT_TRUE( is_valid( output_new(x,y) ) ) << x << "," << y;
      EXPECT_EQ( output_new(x,y), output_org(x,y) );
    }
  }
}
