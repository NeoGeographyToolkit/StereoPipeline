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
#include <asp/Core/BlobIndexThreaded.h>
#include <vw/Math/BBox.h>
#include <vw/Math/Vector.h>
#include <vw/Image/Manipulation.h>
#include <vw/Image/MaskViews.h>
#include <vw/Image/PerPixelViews.h>
#include <vw/Image/PixelMask.h>
#include <vw/Image/PixelMath.h>
#include <vw/FileIO/DiskImageView.h>

#include <boost/assign/std/vector.hpp>
#include <boost/assign/list_of.hpp>

using namespace vw;
using namespace boost::assign;

TEST(BlobIndexThreaded, TestImage1) {
  DiskImageView<PixelGray<uint8> > input("ThreadTest1.tif");
  EXPECT_EQ( 20, input.cols() );
  BlobIndexThreaded bindex( create_mask(input,255), 1000, 10 );
  EXPECT_EQ( 2u, bindex.num_blobs() );
}

TEST(BlobIndexThreaded, TestImage2) {
  DiskImageView<PixelGray<uint8> > input("ThreadTest2.tif");
  EXPECT_EQ( 10, input.cols() );
  BlobIndexThreaded bindex( create_mask(input,255), 1000, 5 );
  EXPECT_EQ( 1u, bindex.num_blobs() );
}

TEST(BlobIndexThreaded, TestImage3) {
  DiskImageView<PixelGray<uint8> > input("ThreadTest3.tif");
  EXPECT_EQ( 10, input.cols() );
  BlobIndexThreaded bindex( create_mask(input,255), 1000, 5 );
  EXPECT_EQ( 2u, bindex.num_blobs() );
}

TEST(BlobIndexThreaded, BlobCompressedIntersect) {
  std::vector<std::list<int32> > starts, ends;
  starts += list_of(0), list_of(0), list_of(0), list_of(0), list_of(0);
  ends += list_of(5), list_of(1), list_of(1), list_of(1), list_of(1);
  blob::BlobCompressed test_blob( Vector2i(5,5), starts, ends );

  test_blob.print();

  EXPECT_FALSE( test_blob.intersects( BBox2i(6,6,2,2) ) );
  EXPECT_FALSE( test_blob.intersects( BBox2i(4,4,1,1) ) );
  EXPECT_FALSE( test_blob.intersects( BBox2i(6,4,3,1) ) );
  EXPECT_FALSE( test_blob.intersects( BBox2i(3,5,2,8) ) );
  EXPECT_TRUE( test_blob.intersects( BBox2i(6,2,4,10) ) );
  EXPECT_TRUE( test_blob.intersects( BBox2i(3,4,6,2) ) );
  EXPECT_TRUE( test_blob.intersects( BBox2i(4,7,2,2) ) );
}
