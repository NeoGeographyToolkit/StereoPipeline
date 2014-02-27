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

#include <vw/Image/ImageViewRef.h>
#include <asp/Core/ErodeView.h>

using namespace vw;

TEST(ErodeView, basic_test) {
  ImageView<PixelMask<uint8> > test(3,3);
  EXPECT_EQ( PixelMask<uint8>(), test(0,0) );
  EXPECT_FALSE( is_valid(test(0,0)) );
  test(1,1) = PixelMask<uint8>(100);
  EXPECT_TRUE( is_valid(test(1,1)) );

  BlobIndexThreaded bindex( test, 100 );
  EXPECT_EQ( 1u, bindex.num_blobs() );

  ImageViewRef<PixelMask<uint8> > erode_ref =
    ErodeView<ImageView<PixelMask<uint8> > >( test, bindex );
  EXPECT_FALSE( is_valid(erode_ref(1,1)) );
  EXPECT_FALSE( is_valid(erode_ref(0,0)) );
  ImageView<PixelMask<uint8> > eroded = erode_ref;
  EXPECT_FALSE( is_valid(eroded(0,0) ) );
  EXPECT_FALSE( is_valid(eroded(1,1) ) );
  EXPECT_EQ( 0, eroded(1,1).child() );
}

TEST(ErodeView, larger_test) {
  ImageView<PixelMask<uint8> > test(5,5);

 /* Set up this image:
   X  X  .  X  .
   .  .  X  X  .
   X  .  .  .  X
   X  X  .  X  X
   X  .  .  X  X
 */

  test(0,0) = PixelMask<uint8>(100);
  test(0,1) = PixelMask<uint8>(100);
  test(0,3) = PixelMask<uint8>(100);
  test(1,2) = PixelMask<uint8>(100);
  test(1,3) = PixelMask<uint8>(100);
  test(2,0) = PixelMask<uint8>(100);
  test(2,4) = PixelMask<uint8>(100);
  test(3,0) = PixelMask<uint8>(100);
  test(3,1) = PixelMask<uint8>(100);
  test(3,3) = PixelMask<uint8>(100);
  test(3,4) = PixelMask<uint8>(100);
  test(4,0) = PixelMask<uint8>(100);
  test(4,3) = PixelMask<uint8>(100);
  test(4,4) = PixelMask<uint8>(100);

  /* Set up this erosion mask:
     .  .  .  .  .
     .  .  X  .  .
     .  .  .  .  .
     .  X  .  X  X
     .  .  .  .  .
   */
  ImageView<PixelMask<uint8> > blobs(5,5);
  blobs(1,2) = PixelMask<uint8>(100);
  blobs(3,1) = PixelMask<uint8>(100);
  blobs(3,3) = PixelMask<uint8>(100);
  blobs(3,4) = PixelMask<uint8>(100);

  // Process blobs
  BlobIndexThreaded bindex( blobs, 100, 100 );
  EXPECT_EQ( 3u, bindex.num_blobs() );

  // Set up blobs as erosion mask on image
  ImageViewRef<PixelMask<uint8> > erode_ref =
    ErodeView<ImageView<PixelMask<uint8> > >( test, bindex );

  EXPECT_FALSE( is_valid(erode_ref(1,0)) );
  EXPECT_FALSE( is_valid(erode_ref(1,2)) );
  EXPECT_TRUE ( is_valid(erode_ref(4,4) ) );
  ImageView<PixelMask<uint8> > eroded = erode_ref;
  EXPECT_FALSE( is_valid(eroded(3,2) ) );
  EXPECT_FALSE( is_valid(eroded(3,4) ) );
  EXPECT_TRUE ( is_valid(eroded(4,3) ) );

}


